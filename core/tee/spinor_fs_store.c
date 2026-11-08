// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * SPI-NOR FS storage backend: a log of hash-tree items in a SPI-NOR region.
 *
 * The region is split into 4 KiB sectors of three kinds:
 * - log sectors start with a struct sfs_log_hdr and hold records appended
 *   in order: hash-tree headers and nodes with their payload, references
 *   to data blocks, and file removal or truncation markers;
 * - data sectors hold exactly one 4 KiB hash-tree data block, written
 *   before the record that references it;
 * - free sectors are erased before use.
 *
 * Every record carries a key (file, type, index, version), a sequence
 * number and a CRC. An index in secure RAM maps each key to its newest
 * record and is rebuilt at mount by replaying the log sectors in sequence
 * order. A record cut by power loss fails its CRC and is ignored, so the
 * previous version of that item stays visible. Atomic commits come from
 * fs_htree itself: it writes the inactive version of every item, then a new
 * header, so the store needs no commit record of its own.
 *
 * Space is reclaimed by compacting the oldest log sector: its live records
 * are appended again and the sector is erased. Compacting the oldest sector
 * first means removal and truncation markers in it can be dropped: every
 * record they cancel is in the same sector or already gone. A sector full
 * of live records is still moved, so the next one can be compacted.
 * Superseded data sectors become free at once.
 *
 * Callers serialize all accesses (the file system instance mutex).
 */

#include <assert.h>
#include <drivers/spi_nor.h>
#include <inttypes.h>
#include <kernel/dt.h>
#include <libfdt.h>
#include <malloc.h>
#include <stddef.h>
#include <string.h>
#include <tee/fs_dirfile.h>
#include <tee/fs_htree.h>
#include <tee/tee_fs_backend.h>
#include <tee/tee_fs_rpc.h>
#include <trace.h>
#include <util.h>

#define SFS_UNIT		4096
#define SFS_LOG_MAGIC		0x4c464e53	/* "SNFL" */
#define SFS_LOG_VERSION		1
#define SFS_REC_MAGIC		0x5243
#define SFS_DIRF_FILE		0x7fffffff
#define SFS_MIN_SECTORS		8
/* Sectors kept free so compaction can always make progress */
#define SFS_RESERVE		2
/*
 * Sectors further kept from file data so that dirf.db can still be updated,
 * and files removed, when the store is full
 */
#define SFS_DIRF_RESERVE	2

/* Name of the flash partition holding the store */
#define SPINOR_FS_PARTITION	"optee-fs"

enum sfs_rec_type {
	SFS_REC_HEAD = TEE_FS_HTREE_TYPE_HEAD,
	SFS_REC_NODE = TEE_FS_HTREE_TYPE_NODE,
	SFS_REC_BLOCK = TEE_FS_HTREE_TYPE_BLOCK,
	SFS_REC_DEL,
	SFS_REC_TRUNC,
};

enum sfs_sec_state {
	SFS_SEC_FREE,		/* Content unknown, erase before use */
	SFS_SEC_ERASED,
	SFS_SEC_LOG,
	SFS_SEC_DATA,
};

struct sfs_log_hdr {
	uint32_t magic;
	uint32_t version;
	uint32_t seq;
	uint32_t crc;
};

/*
 * @file, @type, @idx, @vers: key of the item
 * @len: payload bytes following the header (headers and nodes)
 * @sector: data sector of a block, or truncation index for SFS_REC_TRUNC
 * @data_crc: CRC of the data block
 * @crc: CRC of the header up to this field and of the payload
 */
struct sfs_rec_hdr {
	uint16_t magic;
	uint8_t type;
	uint8_t vers;
	uint32_t file;
	uint32_t idx;
	uint32_t seq;
	uint16_t len;
	uint16_t sector;
	uint32_t data_crc;
	uint32_t crc;
};

#define SFS_MAX_PAYLOAD \
	(sizeof(struct tee_fs_htree_image) > \
	 sizeof(struct tee_fs_htree_node_image) ? \
	 sizeof(struct tee_fs_htree_image) : \
	 sizeof(struct tee_fs_htree_node_image))

static_assert(sizeof(struct sfs_rec_hdr) == 28);
static_assert(SFS_MAX_PAYLOAD <= 256);

/*
 * @loc: byte offset in the region of the payload for headers and nodes,
 *	data sector for blocks
 * @rec_sector: log sector holding the record
 */
struct sfs_item {
	uint32_t file;
	uint32_t idx;
	uint8_t type;
	uint8_t vers;
	uint16_t len;
	uint32_t loc;
	uint32_t seq;
	uint32_t rec_sector;
	uint32_t data_crc;
};

struct sfs {
	struct spi_nor *nor;
	uint64_t base;
	uint32_t nsec;
	bool mounted;
	uint8_t *state;
	uint32_t *log_seq;
	uint32_t head;
	uint32_t head_off;
	bool head_valid;
	uint32_t next_seq;
	uint32_t next_log_seq;
	uint32_t cursor;
	struct sfs_item *items;
	size_t nitems;
	size_t max_items;
	uint8_t *iobuf;		/* One hash-tree item for the htree callbacks */
	uint8_t *recbuf;	/* Header and payload of one record */
};

static uint32_t crc32_update(uint32_t crc, const void *buf, size_t len)
{
	const uint8_t *p = buf;
	size_t n = 0;
	int k = 0;

	crc = ~crc;
	for (n = 0; n < len; n++) {
		crc ^= p[n];
		for (k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xedb88320 & (0 - (crc & 1)));
	}

	return ~crc;
}

static uint32_t rec_crc(const struct sfs_rec_hdr *h, const void *payload)
{
	uint32_t crc = crc32_update(0, h, offsetof(struct sfs_rec_hdr, crc));

	return crc32_update(crc, payload, h->len);
}

static size_t item_size(uint8_t type)
{
	switch (type) {
	case SFS_REC_HEAD:
		return sizeof(struct tee_fs_htree_image);
	case SFS_REC_NODE:
		return sizeof(struct tee_fs_htree_node_image);
	default:
		return SFS_UNIT;
	}
}

static uint64_t sec_addr(struct sfs *s, uint32_t sec)
{
	return s->base + (uint64_t)sec * SFS_UNIT;
}

static struct sfs_item *find_item(struct sfs *s, uint32_t file, uint8_t type,
				  uint32_t idx, uint8_t vers)
{
	size_t n = 0;

	for (n = 0; n < s->nitems; n++) {
		struct sfs_item *it = s->items + n;

		if (it->file == file && it->type == type && it->idx == idx &&
		    it->vers == vers)
			return it;
	}

	return NULL;
}

static bool file_exists(struct sfs *s, uint32_t file)
{
	size_t n = 0;

	for (n = 0; n < s->nitems; n++)
		if (s->items[n].file == file)
			return true;

	return false;
}

static void drop_item(struct sfs *s, size_t n)
{
	struct sfs_item *it = s->items + n;

	if (it->type == SFS_REC_BLOCK && it->loc < s->nsec &&
	    s->state[it->loc] == SFS_SEC_DATA)
		s->state[it->loc] = SFS_SEC_FREE;

	s->items[n] = s->items[--s->nitems];
}

/* Drop items of @file older than @seq: all of them, or blocks >= @from */
static void drop_file(struct sfs *s, uint32_t file, uint32_t seq,
		      bool blocks_only, uint32_t from)
{
	size_t n = 0;

	while (n < s->nitems) {
		struct sfs_item *it = s->items + n;

		if (it->file == file && it->seq < seq &&
		    (!blocks_only ||
		     (it->type == SFS_REC_BLOCK && it->idx >= from)))
			drop_item(s, n);
		else
			n++;
	}
}

static TEE_Result set_item(struct sfs *s, const struct sfs_item *new)
{
	struct sfs_item *it = find_item(s, new->file, new->type, new->idx,
					new->vers);

	if (it) {
		if (it->seq > new->seq)
			return TEE_SUCCESS;
		if (it->type == SFS_REC_BLOCK && it->loc != new->loc &&
		    s->state[it->loc] == SFS_SEC_DATA)
			s->state[it->loc] = SFS_SEC_FREE;
		*it = *new;
		return TEE_SUCCESS;
	}

	if (s->nitems == s->max_items) {
		size_t max = s->max_items + 32;
		struct sfs_item *p = realloc(s->items, max * sizeof(*p));

		if (!p)
			return TEE_ERROR_OUT_OF_MEMORY;
		s->items = p;
		s->max_items = max;
	}

	s->items[s->nitems++] = *new;

	return TEE_SUCCESS;
}

static TEE_Result apply_rec(struct sfs *s, const struct sfs_rec_hdr *h,
			    uint32_t rec_sector, uint32_t payload_offs)
{
	struct sfs_item it = {
		.file = h->file, .idx = h->idx, .type = h->type,
		.vers = h->vers, .len = h->len, .seq = h->seq,
		.rec_sector = rec_sector, .data_crc = h->data_crc,
	};

	switch (h->type) {
	case SFS_REC_HEAD:
	case SFS_REC_NODE:
		it.loc = payload_offs;
		return set_item(s, &it);
	case SFS_REC_BLOCK:
		/*
		 * An older, superseded record may point at a sector reused
		 * since; only live blocks are checked, after replay.
		 */
		if (h->sector >= s->nsec)
			return TEE_ERROR_CORRUPT_OBJECT;
		it.loc = h->sector;
		return set_item(s, &it);
	case SFS_REC_DEL:
		drop_file(s, h->file, h->seq, false, 0);
		return TEE_SUCCESS;
	case SFS_REC_TRUNC:
		drop_file(s, h->file, h->seq, true, h->idx);
		return TEE_SUCCESS;
	default:
		return TEE_ERROR_CORRUPT_OBJECT;
	}
}

static bool all_ff(const uint8_t *p, size_t len)
{
	size_t n = 0;

	for (n = 0; n < len; n++)
		if (p[n] != 0xff)
			return false;

	return true;
}

/* Replay one log sector, return the offset where appending could resume */
static TEE_Result replay_sector(struct sfs *s, uint32_t sec, uint32_t *end)
{
	uint32_t off = sizeof(struct sfs_log_hdr);
	struct sfs_rec_hdr h = { };
	uint8_t *payload = s->recbuf + sizeof(h);
	TEE_Result res = TEE_ERROR_GENERIC;

	while (off + sizeof(h) <= SFS_UNIT) {
		res = spi_nor_read(s->nor, sec_addr(s, sec) + off, &h,
				   sizeof(h));
		if (res)
			return res;

		if (h.magic != SFS_REC_MAGIC) {
			/* Clean end of log, or a torn header: stop here */
			if (all_ff((uint8_t *)&h, sizeof(h)))
				*end = off;
			else
				*end = SFS_UNIT;
			return TEE_SUCCESS;
		}

		if (h.len > SFS_MAX_PAYLOAD ||
		    off + sizeof(h) + h.len > SFS_UNIT) {
			*end = SFS_UNIT;
			return TEE_SUCCESS;
		}

		res = spi_nor_read(s->nor, sec_addr(s, sec) + off + sizeof(h),
				   payload, h.len);
		if (res)
			return res;

		/* The last sequence number is never written: it would wrap */
		if (rec_crc(&h, payload) != h.crc || h.seq == UINT32_MAX) {
			DMSG("SPI-NOR FS: torn record in sector %"PRIu32, sec);
			*end = SFS_UNIT;
			return TEE_SUCCESS;
		}

		res = apply_rec(s, &h, sec, sec * SFS_UNIT + off + sizeof(h));
		if (res)
			return res;

		if (h.seq >= s->next_seq)
			s->next_seq = h.seq + 1;
		off += ROUNDUP(sizeof(h) + h.len, 4);
	}

	*end = SFS_UNIT;

	return TEE_SUCCESS;
}

/*
 * Two live blocks on one data sector: only the newer can be right, the
 * older one comes from a record that a lost marker should have cancelled.
 */
static void drop_stale_blocks(struct sfs *s)
{
	size_t n = 0;
	size_t m = 0;

	for (n = 0; n < s->nitems; n++) {
		struct sfs_item *a = s->items + n;

		if (a->type != SFS_REC_BLOCK)
			continue;

		m = n + 1;
		while (m < s->nitems) {
			struct sfs_item *b = s->items + m;

			if (b->type != SFS_REC_BLOCK || b->loc != a->loc) {
				m++;
				continue;
			}

			EMSG("SPI-NOR FS: shared sector %"PRIu32, a->loc);
			if (b->seq > a->seq)
				*a = *b;
			s->items[m] = s->items[--s->nitems];
		}
	}
}

static TEE_Result scan(struct sfs *s)
{
	struct sfs_log_hdr lh = { };
	uint32_t *order = NULL;
	TEE_Result res = TEE_SUCCESS;
	uint32_t nlog = 0;
	uint32_t end = 0;
	uint32_t n = 0;
	uint32_t m = 0;

	order = calloc(s->nsec, sizeof(*order));
	if (!order)
		return TEE_ERROR_OUT_OF_MEMORY;

	for (n = 0; n < s->nsec; n++) {
		res = spi_nor_read(s->nor, sec_addr(s, n), &lh, sizeof(lh));
		if (res)
			goto out;

		s->state[n] = SFS_SEC_FREE;
		if (lh.magic != SFS_LOG_MAGIC ||
		    lh.version != SFS_LOG_VERSION || lh.seq == UINT32_MAX ||
		    crc32_update(0, &lh, offsetof(struct sfs_log_hdr, crc)) !=
		    lh.crc)
			continue;

		s->state[n] = SFS_SEC_LOG;
		s->log_seq[n] = lh.seq;
		if (lh.seq >= s->next_log_seq)
			s->next_log_seq = lh.seq + 1;

		/* Insert sorted by log sequence */
		for (m = nlog; m && s->log_seq[order[m - 1]] > lh.seq; m--)
			order[m] = order[m - 1];
		order[m] = n;
		nlog++;
	}

	for (n = 0; n < nlog; n++) {
		res = replay_sector(s, order[n], &end);
		if (res)
			goto out;
	}

	/* Append to the newest log sector if it ended cleanly */
	if (nlog && end < SFS_UNIT) {
		s->head = order[nlog - 1];
		s->head_off = end;
		s->head_valid = true;
	}

	drop_stale_blocks(s);

	n = 0;
	while (n < s->nitems) {
		struct sfs_item *it = s->items + n;

		if (it->type == SFS_REC_BLOCK &&
		    s->state[it->loc] != SFS_SEC_FREE) {
			EMSG("SPI-NOR FS: block %"PRIu32" of file %"PRIu32
			     " points at a used sector", it->idx, it->file);
			s->items[n] = s->items[--s->nitems];
			continue;
		}
		if (it->type == SFS_REC_BLOCK)
			s->state[it->loc] = SFS_SEC_DATA;
		n++;
	}
out:
	free(order);

	return res;
}

static TEE_Result sfs_mount(struct sfs *s)
{
	TEE_Result res = TEE_ERROR_GENERIC;

	if (s->mounted)
		return TEE_SUCCESS;

	s->state = calloc(s->nsec, sizeof(*s->state));
	s->log_seq = calloc(s->nsec, sizeof(*s->log_seq));
	s->iobuf = malloc(SFS_UNIT);
	s->recbuf = malloc(sizeof(struct sfs_rec_hdr) + SFS_MAX_PAYLOAD);
	if (!s->state || !s->log_seq || !s->iobuf || !s->recbuf) {
		res = TEE_ERROR_OUT_OF_MEMORY;
		goto err;
	}

	s->nitems = 0;
	s->head_valid = false;
	s->next_seq = 1;
	s->next_log_seq = 1;
	s->cursor = 0;

	res = scan(s);
	if (res)
		goto err;

	s->mounted = true;
	DMSG("SPI-NOR FS: %"PRIu32" sectors, %zu items", s->nsec, s->nitems);

	return TEE_SUCCESS;
err:
	EMSG("SPI-NOR FS: mount failed %#"PRIx32, res);
	free(s->state);
	free(s->log_seq);
	free(s->iobuf);
	free(s->recbuf);
	free(s->items);
	s->state = NULL;
	s->log_seq = NULL;
	s->iobuf = NULL;
	s->recbuf = NULL;
	s->items = NULL;
	s->max_items = 0;
	s->nitems = 0;

	return res;
}

static void __maybe_unused sfs_unmount(struct sfs *s)
{
	free(s->state);
	free(s->log_seq);
	free(s->iobuf);
	free(s->recbuf);
	free(s->items);
	memset(s, 0, sizeof(*s));
}

static uint32_t count_free(struct sfs *s)
{
	uint32_t n = 0;
	uint32_t free_secs = 0;

	for (n = 0; n < s->nsec; n++)
		if (s->state[n] == SFS_SEC_FREE ||
		    s->state[n] == SFS_SEC_ERASED)
			free_secs++;

	return free_secs;
}

/* Free sectors an allocation must leave for writes to @file */
static uint32_t keep_for(uint32_t file)
{
	if (file == SFS_DIRF_FILE)
		return SFS_RESERVE;

	return SFS_RESERVE + SFS_DIRF_RESERVE;
}

static TEE_Result compact(struct sfs *s);

/*
 * Allocate an erased sector, leaving at least @keep free ones. Compaction
 * passes a @keep of 0: it uses the reserve.
 */
static TEE_Result alloc_sector(struct sfs *s, uint32_t keep, uint32_t *sec)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t tries = 0;
	uint32_t n = 0;

	/* Each pass compacts one log sector; give up if none helps */
	while (keep && count_free(s) <= keep) {
		if (tries++ >= s->nsec)
			return TEE_ERROR_STORAGE_NO_SPACE;
		res = compact(s);
		if (res)
			return res;
	}

	for (n = 0; n < s->nsec; n++) {
		uint32_t c = (s->cursor + n) % s->nsec;

		if (s->state[c] != SFS_SEC_FREE &&
		    s->state[c] != SFS_SEC_ERASED)
			continue;

		if (s->state[c] == SFS_SEC_FREE) {
			res = spi_nor_erase(s->nor, sec_addr(s, c), SFS_UNIT);
			if (res)
				return res;
			s->state[c] = SFS_SEC_ERASED;
		}

		/* Round-robin allocation spreads erase cycles */
		s->cursor = (c + 1) % s->nsec;
		*sec = c;

		return TEE_SUCCESS;
	}

	return TEE_ERROR_STORAGE_NO_SPACE;
}

static TEE_Result new_log_sector(struct sfs *s, uint32_t keep)
{
	struct sfs_log_hdr lh = {
		.magic = SFS_LOG_MAGIC,
		.version = SFS_LOG_VERSION,
	};
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t sec = 0;

	if (s->next_log_seq == UINT32_MAX)
		return TEE_ERROR_STORAGE_NO_SPACE;

	res = alloc_sector(s, keep, &sec);
	if (res)
		return res;

	lh.seq = s->next_log_seq++;
	lh.crc = crc32_update(0, &lh, offsetof(struct sfs_log_hdr, crc));

	res = spi_nor_write(s->nor, sec_addr(s, sec), &lh, sizeof(lh));
	if (res) {
		s->state[sec] = SFS_SEC_FREE;
		return res;
	}

	s->state[sec] = SFS_SEC_LOG;
	s->log_seq[sec] = lh.seq;
	s->head = sec;
	s->head_off = sizeof(lh);
	s->head_valid = true;

	return TEE_SUCCESS;
}

/* Append a record, fill in its sequence number and location */
static TEE_Result append(struct sfs *s, struct sfs_rec_hdr *h,
			 const void *payload, uint32_t keep,
			 uint32_t *payload_offs)
{
	size_t size = sizeof(*h) + h->len;
	TEE_Result res = TEE_ERROR_GENERIC;

	if (s->next_seq == UINT32_MAX)
		return TEE_ERROR_STORAGE_NO_SPACE;

	if (!s->head_valid ||
	    s->head_off + ROUNDUP(size, 4) > SFS_UNIT) {
		res = new_log_sector(s, keep);
		if (res)
			return res;
	}

	h->magic = SFS_REC_MAGIC;
	h->seq = s->next_seq++;
	h->crc = rec_crc(h, payload);

	memcpy(s->recbuf, h, sizeof(*h));
	memcpy(s->recbuf + sizeof(*h), payload, h->len);

	res = spi_nor_write(s->nor, sec_addr(s, s->head) + s->head_off,
			    s->recbuf, size);
	if (res) {
		/* The sector may hold a partial record: never append to it */
		s->head_valid = false;
		return res;
	}

	if (payload_offs)
		*payload_offs = s->head * SFS_UNIT + s->head_off + sizeof(*h);
	s->head_off += ROUNDUP(size, 4);

	return TEE_SUCCESS;
}

/* Room for records in a log sector, less what may stay unused at its end */
#define SFS_LOG_ROOM	(SFS_UNIT - sizeof(struct sfs_log_hdr) - \
			 ROUNDUP(sizeof(struct sfs_rec_hdr) + \
				 SFS_MAX_PAYLOAD, 4))

/*
 * Compacting the log sectors one after the other packs their live records:
 * a sector can only be freed if those fit in one log sector less.
 */
static bool compaction_frees(struct sfs *s)
{
	uint64_t live = 0;
	uint32_t nlog = 0;
	size_t n = 0;

	for (n = 0; n < s->nsec; n++)
		if (s->state[n] == SFS_SEC_LOG)
			nlog++;

	for (n = 0; n < s->nitems; n++)
		live += ROUNDUP(sizeof(struct sfs_rec_hdr) + s->items[n].len,
				4);

	return nlog > 1 && live <= (uint64_t)(nlog - 1) * SFS_LOG_ROOM;
}

/*
 * Move the live records of the oldest log sector to the head and erase it.
 * A sector holding only live records frees nothing itself, but moving it
 * lets the next oldest sector be compacted.
 */
static TEE_Result compact(struct sfs *s)
{
	uint8_t payload[SFS_MAX_PAYLOAD] = { };
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t victim = UINT32_MAX;
	uint32_t invalid = 0;
	uint32_t n = 0;

	if (!compaction_frees(s))
		return TEE_ERROR_STORAGE_NO_SPACE;

	for (n = 0; n < s->nsec; n++) {
		if (s->state[n] != SFS_SEC_LOG ||
		    (s->head_valid && n == s->head))
			continue;
		if (victim == UINT32_MAX || s->log_seq[n] < s->log_seq[victim])
			victim = n;
	}
	if (victim == UINT32_MAX)
		return TEE_ERROR_STORAGE_NO_SPACE;

	for (n = 0; n < s->nitems; n++) {
		struct sfs_item *it = s->items + n;
		struct sfs_rec_hdr h = {
			.type = it->type, .vers = it->vers, .file = it->file,
			.idx = it->idx, .len = it->len,
			.data_crc = it->data_crc,
		};
		uint32_t offs = 0;

		if (it->rec_sector != victim)
			continue;

		if (it->type == SFS_REC_BLOCK) {
			h.sector = it->loc;
		} else {
			res = spi_nor_read(s->nor, s->base + it->loc, payload,
					   it->len);
			if (res)
				return res;
		}

		res = append(s, &h, payload, 0, &offs);
		if (res)
			return res;

		it->seq = h.seq;
		it->rec_sector = s->head;
		if (it->type != SFS_REC_BLOCK)
			it->loc = offs;
	}

	/*
	 * Clear the header first: an erase cut by power loss can leave old
	 * records without the markers that cancelled them, they must not be
	 * replayed. Programming bits to 0 needs no erase.
	 */
	res = spi_nor_write(s->nor, sec_addr(s, victim), &invalid,
			    sizeof(invalid));
	if (res)
		return res;
	s->state[victim] = SFS_SEC_FREE;

	res = spi_nor_erase(s->nor, sec_addr(s, victim), SFS_UNIT);
	if (res)
		return res;
	s->state[victim] = SFS_SEC_ERASED;

	return TEE_SUCCESS;
}

static TEE_Result sfs_write_item(struct sfs *s, uint32_t file, uint8_t type,
				 uint32_t idx, uint8_t vers, const void *data)
{
	struct sfs_rec_hdr h = {
		.type = type, .vers = vers, .file = file, .idx = idx,
	};
	struct sfs_item it = {
		.file = file, .idx = idx, .type = type, .vers = vers,
	};
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t sec = 0;

	if (type == SFS_REC_BLOCK) {
		res = alloc_sector(s, keep_for(file), &sec);
		if (res)
			return res;

		res = spi_nor_write(s->nor, sec_addr(s, sec), data, SFS_UNIT);
		if (res) {
			s->state[sec] = SFS_SEC_FREE;
			return res;
		}
		s->state[sec] = SFS_SEC_DATA;
		h.sector = sec;
		h.data_crc = crc32_update(0, data, SFS_UNIT);

		res = append(s, &h, NULL, keep_for(file), NULL);
		if (res) {
			s->state[sec] = SFS_SEC_FREE;
			return res;
		}
		it.loc = sec;
	} else {
		h.len = item_size(type);
		res = append(s, &h, data, keep_for(file), &it.loc);
		if (res)
			return res;
	}

	it.len = h.len;
	it.seq = h.seq;
	it.rec_sector = s->head;
	it.data_crc = h.data_crc;

	return set_item(s, &it);
}

static TEE_Result sfs_read_item(struct sfs *s, uint32_t file, uint8_t type,
				uint32_t idx, uint8_t vers, void *buf,
				size_t *len)
{
	struct sfs_item *it = find_item(s, file, type, idx, vers);
	TEE_Result res = TEE_ERROR_GENERIC;

	if (!it) {
		/*
		 * Like a sparse REE FS file: a header or node never written
		 * reads as zeros, a missing data block as a short read.
		 */
		if (type == SFS_REC_BLOCK) {
			*len = 0;
		} else {
			*len = item_size(type);
			memset(buf, 0, *len);
		}
		return TEE_SUCCESS;
	}

	if (type == SFS_REC_BLOCK) {
		res = spi_nor_read(s->nor, sec_addr(s, it->loc), buf, SFS_UNIT);
		if (res)
			return res;
		if (crc32_update(0, buf, SFS_UNIT) != it->data_crc)
			return TEE_ERROR_CORRUPT_OBJECT;
		*len = SFS_UNIT;
	} else {
		res = spi_nor_read(s->nor, s->base + it->loc, buf, it->len);
		if (res)
			return res;
		*len = it->len;
	}

	return TEE_SUCCESS;
}

static TEE_Result sfs_marker(struct sfs *s, uint8_t type, uint32_t file,
			     uint32_t idx)
{
	struct sfs_rec_hdr h = { .type = type, .file = file, .idx = idx };
	TEE_Result res = TEE_ERROR_GENERIC;

	/* Markers free space: they may use the dirf.db reserve */
	res = append(s, &h, NULL, SFS_RESERVE, NULL);
	if (res)
		return res;

	drop_file(s, file, h.seq, type == SFS_REC_TRUNC, idx);

	return TEE_SUCCESS;
}

static TEE_Result sfs_remove_file(struct sfs *s, uint32_t file)
{
	if (!file_exists(s, file))
		return TEE_SUCCESS;

	return sfs_marker(s, SFS_REC_DEL, file, 0);
}

static TEE_Result sfs_truncate_file(struct sfs *s, uint32_t file,
				    uint32_t num_blocks)
{
	size_t n = 0;

	for (n = 0; n < s->nitems; n++)
		if (s->items[n].file == file &&
		    s->items[n].type == SFS_REC_BLOCK &&
		    s->items[n].idx >= num_blocks)
			return sfs_marker(s, SFS_REC_TRUNC, file, num_blocks);

	return TEE_SUCCESS;
}

static TEE_Result sfs_init(struct sfs *s, struct spi_nor *nor, uint64_t offs,
			   uint64_t size)
{
	struct spi_nor_info info = { };

	s->nor = nor;
	spi_nor_get_info(s->nor, &info);
	if (!size)
		size = info.size > offs ? info.size - offs : 0;

	if (SFS_UNIT % info.erase_size || offs % SFS_UNIT ||
	    size % SFS_UNIT || offs > info.size || size > info.size - offs ||
	    size / SFS_UNIT < SFS_MIN_SECTORS ||
	    size / SFS_UNIT > UINT16_MAX) {
		EMSG("SPI-NOR FS: bad region %#"PRIx64" + %#"PRIx64, offs,
		     size);
		return TEE_ERROR_STORAGE_NOT_AVAILABLE;
	}

	s->base = offs;
	s->nsec = size / SFS_UNIT;

	return sfs_mount(s);
}

/* Enabled DT node with label @label, or a negative value */
static int dt_find(const void *fdt, const char *label)
{
	int node = 0;

	if (!fdt)
		return -1;

	node = fdt_node_offset_by_prop_value(fdt, -1, "label", label,
					     strlen(label) + 1);
	if (node >= 0 && fdt_get_status(fdt, node) == DT_STATUS_DISABLED)
		return -1;

	return node;
}

static bool dt_declares(const void *fdt, const char *label)
{
	return dt_find(fdt, label) >= 0;
}

/*
 * Find the region of the store. The flash partition named @label is used
 * if there is one. If the secure DT @fdt declares it but no probed flash
 * has it, the flash failed: the store is unavailable, it never moves to
 * another region. Without such a partition, the region comes from the
 * build configuration (a size of 0 meaning to the end of the flash).
 */
static TEE_Result find_region(const void *fdt, const char *label,
			      struct spi_nor **nor, uint64_t *offs,
			      uint64_t *size)
{
	struct spi_nor_partition *part = spi_nor_find_partition(label);

	if (part) {
		*nor = part->nor;
		*offs = part->offs;
		*size = part->size;
		return TEE_SUCCESS;
	}

	if (dt_declares(fdt, label)) {
		EMSG("SPI-NOR FS: no flash with partition \"%s\"", label);
		return TEE_ERROR_STORAGE_NOT_AVAILABLE;
	}

	if (spi_nor_get(CFG_SPI_NOR_FS_DEVICE, nor))
		return TEE_ERROR_STORAGE_NOT_AVAILABLE;
	*offs = CFG_SPI_NOR_FS_OFFSET;
	*size = CFG_SPI_NOR_FS_SIZE;

	return TEE_SUCCESS;
}

/* The store behind TEE_STORAGE_PRIVATE_SPI_NOR */
static struct sfs fs_store;

static TEE_Result fs_store_ready(void)
{
	struct spi_nor *nor = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	uint64_t offs = 0;
	uint64_t size = 0;

	if (fs_store.mounted)
		return TEE_SUCCESS;

	res = find_region(get_embedded_dt(), SPINOR_FS_PARTITION, &nor, &offs,
			  &size);
	if (res)
		return res;

	/*
	 * A region from the build configuration is declared as a partition
	 * too, so that no other partition, such as one the normal world may
	 * access, can overlap it.
	 */
	if (!spi_nor_find_partition(SPINOR_FS_PARTITION)) {
		struct spi_nor_info info = { };

		spi_nor_get_info(nor, &info);
		if (!size && info.size > offs)
			size = info.size - offs;
		if (spi_nor_add_partition(nor, SPINOR_FS_PARTITION, offs, size,
					  NULL)) {
			EMSG("SPI-NOR FS: region %#"PRIx64" + %#"PRIx64
			     " overlaps a partition or is invalid", offs, size);
			return TEE_ERROR_STORAGE_NOT_AVAILABLE;
		}
	}

	return sfs_init(&fs_store, nor, offs, size);
}

TEE_Result spinor_fs_store_get_flash(struct spi_nor **nor)
{
	TEE_Result res = fs_store_ready();

	if (!res)
		*nor = fs_store.nor;

	return res;
}

const char *spinor_fs_store_dt_anchor(void)
{
	const void *fdt = get_embedded_dt();
	int node = dt_find(fdt, SPINOR_FS_PARTITION);

	if (node < 0)
		return NULL;

	return fdt_stringlist_get(fdt, node, "optee,rollback-anchor", 0, NULL);
}

static uint32_t dfh_to_file(const struct tee_fs_dirfile_fileh *dfh)
{
	if (!dfh)
		return SFS_DIRF_FILE;

	assert(dfh->file_number < SFS_DIRF_FILE);

	return dfh->file_number;
}

/*
 * Hash-tree item callbacks. The two-step interface is kept from the RPC
 * transport: *_init() hands out a secure buffer, *_final() completes the
 * transfer. The key of a pending write is kept in @op.
 */
static TEE_Result be_read_init(void *aux, struct tee_fs_rpc_operation *op,
			       enum tee_fs_htree_type type, size_t idx,
			       uint8_t vers, void **data)
{
	uint32_t file = *(int *)aux;
	TEE_Result res = TEE_ERROR_GENERIC;
	size_t len = 0;

	res = sfs_read_item(&fs_store, file, type, idx, vers, fs_store.iobuf,
			    &len);
	if (res)
		return res;

	memset(op, 0, sizeof(*op));
	op->params[0].u.value.a = len;
	*data = fs_store.iobuf;

	return TEE_SUCCESS;
}

static TEE_Result be_read_final(struct tee_fs_rpc_operation *op,
				size_t *bytes)
{
	*bytes = op->params[0].u.value.a;

	return TEE_SUCCESS;
}

static TEE_Result be_write_init(void *aux, struct tee_fs_rpc_operation *op,
				enum tee_fs_htree_type type, size_t idx,
				uint8_t vers, void **data)
{
	memset(op, 0, sizeof(*op));
	op->params[0].u.value.a = *(int *)aux;
	op->params[0].u.value.b = idx;
	op->params[0].u.value.c = (type << 8) | vers;
	*data = fs_store.iobuf;

	return TEE_SUCCESS;
}

static TEE_Result be_write_final(struct tee_fs_rpc_operation *op)
{
	uint64_t c = op->params[0].u.value.c;

	return sfs_write_item(&fs_store, op->params[0].u.value.a, c >> 8,
			      op->params[0].u.value.b, c & 0xff,
			      fs_store.iobuf);
}

static const struct tee_fs_htree_storage be_htree_storage = {
	.block_size = SFS_UNIT,
	.rpc_read_init = be_read_init,
	.rpc_read_final = be_read_final,
	.rpc_write_init = be_write_init,
	.rpc_write_final = be_write_final,
};

static TEE_Result be_open(const struct tee_fs_dirfile_fileh *dfh, int *fd)
{
	uint32_t file = dfh_to_file(dfh);
	TEE_Result res = fs_store_ready();

	if (res)
		return res;
	if (!file_exists(&fs_store, file))
		return TEE_ERROR_ITEM_NOT_FOUND;

	*fd = file;

	return TEE_SUCCESS;
}

static TEE_Result be_create(const struct tee_fs_dirfile_fileh *dfh, int *fd)
{
	uint32_t file = dfh_to_file(dfh);
	TEE_Result res = fs_store_ready();

	if (!res)
		res = sfs_remove_file(&fs_store, file);
	if (!res)
		*fd = file;

	return res;
}

static TEE_Result be_close(int fd __unused)
{
	return TEE_SUCCESS;
}

static TEE_Result be_truncate(int fd, size_t num_blocks)
{
	if (num_blocks > UINT32_MAX)
		return TEE_ERROR_BAD_PARAMETERS;

	return sfs_truncate_file(&fs_store, fd, num_blocks);
}

static TEE_Result be_remove(const struct tee_fs_dirfile_fileh *dfh)
{
	TEE_Result res = fs_store_ready();

	if (res)
		return res;

	return sfs_remove_file(&fs_store, dfh_to_file(dfh));
}

const struct tee_fs_backend_ops spinor_fs_backend = {
	.htree_storage = &be_htree_storage,
	.open = be_open,
	.create = be_create,
	.close = be_close,
	.truncate = be_truncate,
	.remove = be_remove,
};

#ifdef CFG_SPI_NOR_EMBEDDED_TEST
void spinor_fs_store_remount(void)
{
	sfs_unmount(&fs_store);
}

/*
 * Store self test on the part of the simulated flash after the SPI-NOR FS
 * region, run with the core self tests. It formats, writes, remounts,
 * injects a torn record and fills that test region.
 */
static unsigned int st_failures;

static void st_check(bool cond, const char *what, int line)
{
	if (!cond) {
		EMSG("SPI-NOR FS store self test failed at line %d: %s", line,
		     what);
		st_failures++;
	}
}

#define ST_CHECK(cond)	st_check((cond), #cond, __LINE__)

static void st_fill(uint8_t *buf, size_t len, uint8_t seed)
{
	size_t n = 0;

	for (n = 0; n < len; n++)
		buf[n] = seed + n * 7;
}

static bool st_item_is(struct sfs *s, uint32_t file, uint8_t type,
		       uint32_t idx, uint8_t seed)
{
	uint8_t *buf = malloc(SFS_UNIT);
	uint8_t *ref = malloc(SFS_UNIT);
	size_t len = 0;
	bool ok = false;

	if (buf && ref &&
	    !sfs_read_item(s, file, type, idx, 0, buf, &len) &&
	    len == item_size(type)) {
		st_fill(ref, len, seed);
		ok = !memcmp(buf, ref, len);
	}

	free(buf);
	free(ref);

	return ok;
}

static TEE_Result st_remount(struct sfs *s, struct spi_nor *nor,
			     uint64_t offs, uint64_t size)
{
	sfs_unmount(s);

	return sfs_init(s, nor, offs, size);
}

/*
 * Region lookup: partition first, no fallback when the DT declares it. The
 * test partition is at @offs_part, in the test region.
 */
static void st_find_region(struct spi_nor *nor, uint64_t offs_part)
{
	const char *label = "st-sfs";
	struct spi_nor_partition *part = NULL;
	struct spi_nor *found = NULL;
	uint64_t offs = 0;
	uint64_t size = 0;
	/* libfdt needs an 8-byte aligned tree */
	uint64_t fdt_buf[32] = { };
	void *fdt = fdt_buf;
	bool ok = false;

	ok = !fdt_create(fdt, sizeof(fdt_buf)) && !fdt_finish_reservemap(fdt) &&
	     !fdt_begin_node(fdt, "") && !fdt_begin_node(fdt, "p@0") &&
	     !fdt_property_string(fdt, "label", label) &&
	     !fdt_end_node(fdt) && !fdt_end_node(fdt) && !fdt_finish(fdt);
	ST_CHECK(ok);
	if (!ok)
		return;

	/* Declared but not probed: unavailable */
	ST_CHECK(find_region(fdt, label, &found, &offs, &size) ==
		 TEE_ERROR_STORAGE_NOT_AVAILABLE);

	/* Not declared: the build-time region */
	ST_CHECK(!find_region(NULL, label, &found, &offs, &size));
	ST_CHECK(found && offs == CFG_SPI_NOR_FS_OFFSET &&
		 size == CFG_SPI_NOR_FS_SIZE);

	/* A probed partition wins */
	ST_CHECK(!spi_nor_add_partition(nor, label, offs_part, SFS_UNIT,
					&part));
	ST_CHECK(!find_region(fdt, label, &found, &offs, &size));
	ST_CHECK(found == nor && offs == offs_part && size == SFS_UNIT);
	if (part)
		spi_nor_del_partition(part);
}

TEE_Result spinor_fs_store_self_test(void)
{
	struct spi_nor_info info = { };
	struct spi_nor *nor = NULL;
	struct sfs s = { };
	uint8_t *data = NULL;
	uint64_t offs = 0;
	uint64_t size = 0;
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t n = 0;

	/* The test erases flash: only on the simulated one */
	if (spi_nor_get(CFG_SPI_NOR_FS_DEVICE, &nor) ||
	    nor != spi_nor_sim_get())
		return TEE_SUCCESS;
	spi_nor_get_info(nor, &info);

	offs = CFG_SPI_NOR_FS_OFFSET +
	       (CFG_SPI_NOR_FS_SIZE ? CFG_SPI_NOR_FS_SIZE :
				      info.size - CFG_SPI_NOR_FS_OFFSET);
	size = info.size > offs ? info.size - offs : 0;
	size = ROUNDDOWN(size, SFS_UNIT);
	if (size / SFS_UNIT < 16) {
		IMSG("SPI-NOR FS store self test skipped: no spare region");
		return TEE_SUCCESS;
	}

	data = malloc(SFS_UNIT);
	if (!data)
		return TEE_ERROR_OUT_OF_MEMORY;

	st_failures = 0;

	st_find_region(nor, offs);

	/* Format: an erased region mounts empty */
	ST_CHECK(!spi_nor_erase(nor, offs, size));
	ST_CHECK(!sfs_init(&s, nor, offs, size));
	ST_CHECK(!s.nitems);

	/* Write all item types, overwrite one, then remount */
	st_fill(data, SFS_UNIT, 1);
	ST_CHECK(!sfs_write_item(&s, 1, SFS_REC_HEAD, 0, 0, data));
	st_fill(data, SFS_UNIT, 2);
	ST_CHECK(!sfs_write_item(&s, 1, SFS_REC_NODE, 0, 0, data));
	st_fill(data, SFS_UNIT, 3);
	ST_CHECK(!sfs_write_item(&s, 1, SFS_REC_BLOCK, 0, 0, data));
	st_fill(data, SFS_UNIT, 4);
	ST_CHECK(!sfs_write_item(&s, 1, SFS_REC_BLOCK, 0, 0, data));
	st_fill(data, SFS_UNIT, 5);
	ST_CHECK(!sfs_write_item(&s, 2, SFS_REC_BLOCK, 7, 0, data));
	ST_CHECK(!st_remount(&s, nor, offs, size));
	ST_CHECK(st_item_is(&s, 1, SFS_REC_HEAD, 0, 1));
	ST_CHECK(st_item_is(&s, 1, SFS_REC_NODE, 0, 2));
	ST_CHECK(st_item_is(&s, 1, SFS_REC_BLOCK, 0, 4));
	ST_CHECK(st_item_is(&s, 2, SFS_REC_BLOCK, 7, 5));

	/* Truncation and removal survive a remount */
	ST_CHECK(!sfs_truncate_file(&s, 2, 7));
	ST_CHECK(!sfs_remove_file(&s, 1));
	ST_CHECK(!st_remount(&s, nor, offs, size));
	ST_CHECK(!file_exists(&s, 1));
	ST_CHECK(!find_item(&s, 2, SFS_REC_BLOCK, 7, 0));

	/* A torn record is ignored and appending resumes in a new sector */
	st_fill(data, SFS_UNIT, 6);
	ST_CHECK(!sfs_write_item(&s, 3, SFS_REC_NODE, 1, 0, data));
	if (s.head_valid) {
		struct sfs_rec_hdr h = {
			.magic = SFS_REC_MAGIC, .type = SFS_REC_NODE,
			.file = 3, .idx = 1, .seq = UINT32_MAX - 1,
			.len = item_size(SFS_REC_NODE), .crc = 0x12345678,
		};

		ST_CHECK(!spi_nor_write(nor, sec_addr(&s, s.head) +
					s.head_off, &h, sizeof(h)));
	}
	ST_CHECK(!st_remount(&s, nor, offs, size));
	ST_CHECK(st_item_is(&s, 3, SFS_REC_NODE, 1, 6));
	ST_CHECK(!s.head_valid);
	st_fill(data, SFS_UNIT, 7);
	ST_CHECK(!sfs_write_item(&s, 3, SFS_REC_NODE, 2, 0, data));
	ST_CHECK(!st_remount(&s, nor, offs, size));
	ST_CHECK(st_item_is(&s, 3, SFS_REC_NODE, 2, 7));

	/* Rewriting one item many times wraps the log through compaction */
	for (n = 0; n < 4 * s.nsec * (SFS_UNIT / 128); n++) {
		st_fill(data, SFS_UNIT, n);
		res = sfs_write_item(&s, 4, SFS_REC_NODE, 0, 0, data);
		if (res)
			break;
	}
	ST_CHECK(!res);
	ST_CHECK(!st_remount(&s, nor, offs, size));
	ST_CHECK(st_item_is(&s, 4, SFS_REC_NODE, 0, (uint8_t)(n - 1)));

	/*
	 * Items written once end up packed in full log sectors; rewriting
	 * another item must still compact past them.
	 */
	st_fill(data, SFS_UNIT, 8);
	for (n = 0; n < 100; n++)
		ST_CHECK(!sfs_write_item(&s, 7, SFS_REC_NODE, n, 0, data));
	for (n = 0; n < 4 * s.nsec * (SFS_UNIT / 128); n++) {
		res = sfs_write_item(&s, 4, SFS_REC_NODE, 0, 0, data);
		if (res)
			break;
	}
	ST_CHECK(!res);
	ST_CHECK(!st_remount(&s, nor, offs, size));
	ST_CHECK(st_item_is(&s, 7, SFS_REC_NODE, 99, 8));
	ST_CHECK(!sfs_remove_file(&s, 7));

	/* Of two blocks on one sector, the newer wins at mount */
	st_fill(data, SFS_UNIT, 9);
	ST_CHECK(!sfs_write_item(&s, 8, SFS_REC_BLOCK, 0, 0, data));
	if (find_item(&s, 8, SFS_REC_BLOCK, 0, 0)) {
		struct sfs_rec_hdr h = {
			.type = SFS_REC_BLOCK, .file = 9,
			.sector = find_item(&s, 8, SFS_REC_BLOCK, 0, 0)->loc,
			.data_crc = crc32_update(0, data, SFS_UNIT),
		};

		ST_CHECK(!append(&s, &h, NULL, SFS_RESERVE, NULL));
	}
	ST_CHECK(!st_remount(&s, nor, offs, size));
	ST_CHECK(!find_item(&s, 8, SFS_REC_BLOCK, 0, 0));
	ST_CHECK(st_item_is(&s, 9, SFS_REC_BLOCK, 0, 9));
	ST_CHECK(!sfs_remove_file(&s, 9));

	/*
	 * Filling with data blocks ends with NO_SPACE, dirf.db can still be
	 * updated, and removal frees
	 */
	for (n = 0; n < s.nsec; n++) {
		res = sfs_write_item(&s, 5, SFS_REC_BLOCK, n, 0, data);
		if (res)
			break;
	}
	ST_CHECK(res == TEE_ERROR_STORAGE_NO_SPACE);
	ST_CHECK(!sfs_write_item(&s, SFS_DIRF_FILE, SFS_REC_BLOCK, 0, 0, data));
	ST_CHECK(!sfs_write_item(&s, SFS_DIRF_FILE, SFS_REC_HEAD, 0, 0, data));
	ST_CHECK(!sfs_remove_file(&s, 5));
	ST_CHECK(!sfs_write_item(&s, 6, SFS_REC_BLOCK, 0, 0, data));

	sfs_unmount(&s);
	free(data);

	return st_failures ? TEE_ERROR_GENERIC : TEE_SUCCESS;
}
#endif
