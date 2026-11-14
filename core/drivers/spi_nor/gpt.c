// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * GUID Partition Table (UEFI specification, section 5.3) on a SPI-NOR flash.
 * The whole flash is the disk: the primary header is in logical block 1,
 * the backup header in the last block, and blocks of 512 or 4096 bytes are
 * recognized. Each used entry becomes a partition labelled with the entry
 * name. The protective MBR is not checked and the attributes are ignored:
 * GPT has no notion of secure or normal-world access.
 *
 * The table is read from flash, so all of it is checked before any partition
 * is added: both CRC32, the header fields against the flash size and every
 * entry against the usable range.
 */

#include <drivers/spi_nor.h>
#include <inttypes.h>
#include <io.h>
#include <malloc.h>
#include <string.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "spi_nor_priv.h"

#define GPT_SIGNATURE		0x5452415020494645ULL	/* "EFI PART" */
#define GPT_REVISION_MAJOR	1
#define GPT_HDR_MIN_SIZE	92
#define GPT_ENTRY_MIN_SIZE	128
#define GPT_ENTRIES_MAX_SIZE	(64 * 1024)
#define GPT_NAME_UNITS		36

/* Header field offsets */
#define GPT_HDR_REVISION	8
#define GPT_HDR_SIZE		12
#define GPT_HDR_CRC		16
#define GPT_HDR_MY_LBA		24
#define GPT_HDR_FIRST_USABLE	40
#define GPT_HDR_LAST_USABLE	48
#define GPT_HDR_ENTRIES_LBA	72
#define GPT_HDR_NUM_ENTRIES	80
#define GPT_HDR_ENTRY_SIZE	84
#define GPT_HDR_ENTRIES_CRC	88

/* Entry field offsets */
#define GPT_ENT_TYPE		0
#define GPT_ENT_FIRST_LBA	32
#define GPT_ENT_LAST_LBA	40
#define GPT_ENT_NAME		56

static const size_t gpt_block_sizes[] = { 512, 4096 };

struct gpt_part {
	char label[SPI_NOR_PART_LABEL_LEN];
	uint64_t offs;
	uint64_t size;
};

struct gpt_table {
	struct gpt_part *parts;
	size_t count;
	size_t bsize;
	uint64_t lba;
	uint32_t num_entries;
};

struct gpt_hdr {
	uint64_t first_usable;
	uint64_t last_usable;
	uint64_t entries_lba;
	uint32_t num_entries;
	uint32_t entry_size;
	uint32_t entries_crc;
};

/* CRC32 as in the UEFI specification: reflected polynomial 0xedb88320 */
static uint32_t gpt_crc32(uint32_t crc, const uint8_t *buf, size_t len)
{
	size_t n = 0;
	int bit = 0;

	crc = ~crc;
	for (n = 0; n < len; n++) {
		crc ^= buf[n];
		for (bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
	}

	return ~crc;
}

uint32_t spi_nor_gpt_crc32(const void *buf, size_t len)
{
	return gpt_crc32(0, buf, len);
}

static TEE_Result read_header(spi_nor_gpt_read_func read, void *ctx,
			      size_t bsize, uint64_t nblocks, uint64_t lba,
			      struct gpt_hdr *hdr)
{
	uint64_t entries_blocks = 0;
	uint64_t entries_size = 0;
	TEE_Result res = TEE_ERROR_GENERIC;
	uint8_t *buf = NULL;
	uint32_t hdr_size = 0;
	uint32_t crc = 0;

	buf = malloc(bsize);
	if (!buf)
		return TEE_ERROR_OUT_OF_MEMORY;

	res = read(ctx, lba * bsize, buf, bsize);
	if (res)
		goto out;

	if (get_le64(buf) != GPT_SIGNATURE) {
		res = TEE_ERROR_ITEM_NOT_FOUND;
		goto out;
	}

	res = TEE_ERROR_CORRUPT_OBJECT;
	hdr_size = get_le32(buf + GPT_HDR_SIZE);
	if (hdr_size < GPT_HDR_MIN_SIZE || hdr_size > bsize)
		goto out;
	crc = get_le32(buf + GPT_HDR_CRC);
	put_le32(buf + GPT_HDR_CRC, 0);
	if (spi_nor_gpt_crc32(buf, hdr_size) != crc)
		goto out;

	if (get_le32(buf + GPT_HDR_REVISION) >> 16 != GPT_REVISION_MAJOR ||
	    get_le64(buf + GPT_HDR_MY_LBA) != lba)
		goto out;

	hdr->first_usable = get_le64(buf + GPT_HDR_FIRST_USABLE);
	hdr->last_usable = get_le64(buf + GPT_HDR_LAST_USABLE);
	hdr->entries_lba = get_le64(buf + GPT_HDR_ENTRIES_LBA);
	hdr->num_entries = get_le32(buf + GPT_HDR_NUM_ENTRIES);
	hdr->entry_size = get_le32(buf + GPT_HDR_ENTRY_SIZE);
	hdr->entries_crc = get_le32(buf + GPT_HDR_ENTRIES_CRC);

	/* Entries are 128 << n bytes, here at most one block */
	if (hdr->entry_size < GPT_ENTRY_MIN_SIZE ||
	    !IS_POWER_OF_TWO(hdr->entry_size) || hdr->entry_size > bsize)
		goto out;
	entries_size = (uint64_t)hdr->num_entries * hdr->entry_size;
	if (entries_size > GPT_ENTRIES_MAX_SIZE)
		goto out;
	entries_blocks = ROUNDUP_DIV(entries_size, bsize);

	/* Usable range, header and entry array are disjoint and in the disk */
	if (hdr->first_usable > hdr->last_usable ||
	    hdr->last_usable >= nblocks || entries_blocks > nblocks ||
	    (lba >= hdr->first_usable && lba <= hdr->last_usable) ||
	    hdr->entries_lba > nblocks - entries_blocks ||
	    (lba >= hdr->entries_lba &&
	     lba < hdr->entries_lba + entries_blocks) ||
	    (hdr->entries_lba + entries_blocks > hdr->first_usable &&
	     hdr->entries_lba <= hdr->last_usable))
		goto out;

	res = TEE_SUCCESS;
out:
	free(buf);
	return res;
}

/* ASCII copy of the UTF-16LE entry name, false if it has no such form */
static bool get_label(const uint8_t *name, char *label)
{
	unsigned int unit = 0;
	size_t n = 0;

	for (n = 0; n < GPT_NAME_UNITS; n++) {
		unit = name[2 * n] | (name[2 * n + 1] << 8);
		if (!unit)
			break;
		if (unit < 0x20 || unit > 0x7e ||
		    n == SPI_NOR_PART_LABEL_LEN - 1)
			return false;
		label[n] = unit;
	}
	label[n] = '\0';

	return n > 0;
}

static bool is_zero(const uint8_t *buf, size_t len)
{
	size_t n = 0;

	for (n = 0; n < len; n++)
		if (buf[n])
			return false;

	return true;
}

static TEE_Result add_entry(struct gpt_table *tbl, const struct gpt_hdr *hdr,
			    size_t bsize, const uint8_t *ent, uint32_t index)
{
	uint64_t first = get_le64(ent + GPT_ENT_FIRST_LBA);
	uint64_t last = get_le64(ent + GPT_ENT_LAST_LBA);
	struct gpt_part *parts = NULL;
	struct gpt_part part = { };

	if (first < hdr->first_usable || last > hdr->last_usable ||
	    first > last) {
		EMSG("SPI-NOR: GPT entry %"PRIu32" out of the usable range",
		     index);
		return TEE_ERROR_CORRUPT_OBJECT;
	}

	if (!get_label(ent + GPT_ENT_NAME, part.label)) {
		IMSG("SPI-NOR: GPT entry %"PRIu32" has no ASCII name, skipped",
		     index);
		return TEE_SUCCESS;
	}
	part.offs = first * bsize;
	part.size = (last - first + 1) * bsize;

	parts = realloc(tbl->parts, (tbl->count + 1) * sizeof(*parts));
	if (!parts)
		return TEE_ERROR_OUT_OF_MEMORY;
	parts[tbl->count++] = part;
	tbl->parts = parts;

	return TEE_SUCCESS;
}

/* Read the entry array a block at a time: entries never span two blocks */
static TEE_Result read_entries(spi_nor_gpt_read_func read, void *ctx,
			       size_t bsize, const struct gpt_hdr *hdr,
			       struct gpt_table *tbl)
{
	size_t total = (size_t)hdr->num_entries * hdr->entry_size;
	TEE_Result res = TEE_SUCCESS;
	uint8_t *buf = NULL;
	uint32_t index = 0;
	uint32_t crc = 0;
	size_t offs = 0;
	size_t len = 0;
	size_t n = 0;

	buf = malloc(bsize);
	if (!buf)
		return TEE_ERROR_OUT_OF_MEMORY;

	for (offs = 0; offs < total && !res; offs += len) {
		len = MIN(bsize, total - offs);
		res = read(ctx, hdr->entries_lba * bsize + offs, buf, len);
		if (res)
			break;
		crc = gpt_crc32(crc, buf, len);
		for (n = 0; n < len && !res; n += hdr->entry_size, index++)
			if (!is_zero(buf + n + GPT_ENT_TYPE, 16))
				res = add_entry(tbl, hdr, bsize, buf + n,
						index);
	}

	if (!res && crc != hdr->entries_crc)
		res = TEE_ERROR_CORRUPT_OBJECT;

	free(buf);
	return res;
}

static TEE_Result read_table(spi_nor_gpt_read_func read, void *ctx,
			     uint64_t size, size_t bsize, bool backup,
			     struct gpt_table *tbl)
{
	uint64_t nblocks = size / bsize;
	struct gpt_hdr hdr = { };
	TEE_Result res = TEE_ERROR_GENERIC;

	if (nblocks < 3)
		return TEE_ERROR_ITEM_NOT_FOUND;

	tbl->bsize = bsize;
	tbl->lba = backup ? nblocks - 1 : 1;
	res = read_header(read, ctx, bsize, nblocks, tbl->lba, &hdr);
	if (!res) {
		tbl->num_entries = hdr.num_entries;
		res = read_entries(read, ctx, bsize, &hdr, tbl);
	}

	if (res) {
		free(tbl->parts);
		tbl->parts = NULL;
		tbl->count = 0;
	}

	return res;
}

static TEE_Result find_table(spi_nor_gpt_read_func read, void *ctx,
			     uint64_t size, struct gpt_table *tbl)
{
	TEE_Result ret = TEE_ERROR_ITEM_NOT_FOUND;
	TEE_Result res = TEE_ERROR_GENERIC;
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(gpt_block_sizes); n++) {
		res = read_table(read, ctx, size, gpt_block_sizes[n], false,
				 tbl);
		if (!res)
			return TEE_SUCCESS;
		if (res != TEE_ERROR_ITEM_NOT_FOUND)
			ret = res;

		/* A bad primary header or entry array: the backup copy */
		res = read_table(read, ctx, size, gpt_block_sizes[n], true,
				 tbl);
		if (!res) {
			if (ret != TEE_ERROR_ITEM_NOT_FOUND)
				IMSG("SPI-NOR: bad primary GPT, using backup");
			return TEE_SUCCESS;
		}
		if (res != TEE_ERROR_ITEM_NOT_FOUND)
			ret = res;
	}

	return ret;
}

TEE_Result spi_nor_gpt_add(struct spi_nor *nor, spi_nor_gpt_read_func read,
			   void *ctx, uint64_t size)
{
	struct spi_nor_partition **added = NULL;
	struct gpt_table tbl = { };
	TEE_Result res = TEE_ERROR_GENERIC;
	size_t n = 0;

	res = find_table(read, ctx, size, &tbl);
	if (res)
		return res;

	added = calloc(tbl.count, sizeof(*added));
	if (tbl.count && !added) {
		res = TEE_ERROR_OUT_OF_MEMORY;
		goto out;
	}

	for (n = 0; n < tbl.count; n++) {
		res = spi_nor_add_partition(nor, tbl.parts[n].label,
					    tbl.parts[n].offs,
					    tbl.parts[n].size, added + n);
		if (res) {
			/* All of the table or nothing */
			while (n--)
				spi_nor_del_partition(added[n]);
			goto out;
		}
	}

	IMSG("SPI-NOR: %s GPT, %zu-byte blocks, %zu of %"PRIu32" entries",
	     tbl.lba == 1 ? "primary" : "backup", tbl.bsize, tbl.count,
	     tbl.num_entries);
out:
	free(added);
	free(tbl.parts);
	return res;
}

static TEE_Result nor_read(void *ctx, uint64_t offs, void *buf, size_t len)
{
	return spi_nor_read(ctx, offs, buf, len);
}

TEE_Result spi_nor_gpt_add_partitions(struct spi_nor *nor)
{
	struct spi_nor_info info = { };

	spi_nor_get_info(nor, &info);

	return spi_nor_gpt_add(nor, nor_read, nor, info.size);
}
