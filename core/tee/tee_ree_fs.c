// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2015, Linaro Limited
 */

#include <assert.h>
#include <config.h>
#include <drivers/spi_nor_rpmc.h>
#include <inttypes.h>
#include <kernel/mutex.h>
#include <kernel/nv_counter.h>
#include <kernel/panic.h>
#include <kernel/thread.h>
#include <kernel/user_access.h>
#include <mempool.h>
#include <mm/core_memprot.h>
#include <mm/tee_pager.h>
#include <optee_rpc_cmd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <tee/fs_dirfile.h>
#include <tee/fs_htree.h>
#include <tee/tee_fs.h>
#include <tee/tee_fs_backend.h>
#include <tee/tee_fs_rpc.h>
#include <tee/tee_pobj.h>
#include <trace.h>
#include <utee_defines.h>
#include <util.h>

#define BLOCK_SHIFT	12

#define BLOCK_SIZE	(1 << BLOCK_SHIFT)

struct fs_instance;

struct tee_fs_fd {
	struct fs_instance *fs;
	struct tee_fs_htree *ht;
	int fd;
	struct tee_fs_dirfile_fileh dfh;
	const TEE_UUID *uuid;
};

struct tee_fs_dir {
	struct fs_instance *fs;
	struct tee_fs_dirfile_dirh *dirh;
	int idx;
	struct tee_fs_dirent d;
	const TEE_UUID *uuid;
};

/*
 * struct fs_instance - One hash-tree file system on one storage backend
 *
 * @be: Storage backend holding the files
 * @dirf_ops: Directory file operations bound to this instance
 * @open_dirh: Open the directory file and check it against the rollback
 *	anchor
 * @commit_dirh_writes: Commit the directory file and update the anchor
 * @close_dirh: Close the directory file
 * @mutex: Serializes all operations of the instance
 * @dirh: Cached directory file handle, see get_dirh()
 * @dirh_refcount: Number of users of @dirh
 */
struct fs_instance {
	const struct tee_fs_backend_ops *be;
	const struct tee_fs_dirfile_operations *dirf_ops;
	TEE_Result (*open_dirh)(struct fs_instance *fs,
				struct tee_fs_dirfile_dirh **dirh);
	TEE_Result (*commit_dirh_writes)(struct fs_instance *fs,
					 struct tee_fs_dirfile_dirh *dirh);
	void (*close_dirh)(struct fs_instance *fs,
			   struct tee_fs_dirfile_dirh **dirh);
	struct mutex mutex; /* Serializes all operations of the instance */
	struct tee_fs_dirfile_dirh *dirh;
	size_t dirh_refcount;
};

static size_t pos_to_block_num(size_t position)
{
	return position >> BLOCK_SHIFT;
}

static void *get_tmp_block(void)
{
	return mempool_alloc(mempool_default, BLOCK_SIZE);
}

static void put_tmp_block(void *tmp_block)
{
	mempool_free(mempool_default, tmp_block);
}

static TEE_Result out_of_place_write(struct tee_fs_fd *fdp, size_t pos,
				     const void *buf_core,
				     const void *buf_user, size_t len)
{
	TEE_Result res;
	size_t start_block_num = pos_to_block_num(pos);
	size_t end_block_num = pos_to_block_num(pos + len - 1);
	size_t remain_bytes = len;
	uint8_t *data_core_ptr = (uint8_t *)buf_core;
	uint8_t *data_user_ptr = (uint8_t *)buf_user;
	uint8_t *block;
	struct tee_fs_htree_meta *meta = tee_fs_htree_get_meta(fdp->ht);

	assert(meta);

	/*
	 * It doesn't make sense to call this function if nothing is to be
	 * written. This also guards against end_block_num getting an
	 * unexpected value when pos == 0 and len == 0.
	 */
	if (!len)
		return TEE_ERROR_BAD_PARAMETERS;

	block = get_tmp_block();
	if (!block)
		return TEE_ERROR_OUT_OF_MEMORY;

	while (start_block_num <= end_block_num) {
		size_t offset = pos % BLOCK_SIZE;
		size_t size_to_write = MIN(remain_bytes, (size_t)BLOCK_SIZE);

		if (size_to_write + offset > BLOCK_SIZE)
			size_to_write = BLOCK_SIZE - offset;

		if (start_block_num * BLOCK_SIZE <
		    ROUNDUP(meta->length, BLOCK_SIZE)) {
			res = tee_fs_htree_read_block(&fdp->ht,
						      start_block_num, block);
			if (res != TEE_SUCCESS)
				goto exit;
		} else {
			memset(block, 0, BLOCK_SIZE);
		}

		if (data_core_ptr) {
			memcpy(block + offset, data_core_ptr, size_to_write);
		} else if (data_user_ptr) {
			res = copy_from_user(block + offset, data_user_ptr,
					     size_to_write);
			if (res)
				goto exit;
		} else {
			memset(block + offset, 0, size_to_write);
		}

		res = tee_fs_htree_write_block(&fdp->ht, start_block_num,
					       block);
		if (res != TEE_SUCCESS)
			goto exit;

		if (data_core_ptr)
			data_core_ptr += size_to_write;
		if (data_user_ptr)
			data_user_ptr += size_to_write;
		remain_bytes -= size_to_write;
		start_block_num++;
		pos += size_to_write;
	}

	if (pos > meta->length) {
		meta->length = pos;
		tee_fs_htree_meta_set_dirty(fdp->ht);
	}

exit:
	if (block)
		put_tmp_block(block);
	return res;
}

#ifdef CFG_REE_FS
static TEE_Result get_offs_size(enum tee_fs_htree_type type, size_t idx,
				uint8_t vers, tee_fs_off_t *offs, size_t *size)
{
	const size_t node_size = sizeof(struct tee_fs_htree_node_image);
	const size_t block_nodes = BLOCK_SIZE / (node_size * 2);
	const uint64_t idx64 = idx;
	uint64_t pbn = 0;
	uint64_t bidx = 0;

	assert(vers == 0 || vers == 1);

	/*
	 * File layout
	 * [demo with input:
	 * BLOCK_SIZE = 4096,
	 * node_size = 66,
	 * block_nodes = 4096/(66*2) = 31 ]
	 *
	 * phys block 0:
	 * tee_fs_htree_image vers 0 @ offs = 0
	 * tee_fs_htree_image vers 1 @ offs = sizeof(tee_fs_htree_image)
	 *
	 * phys block 1:
	 * tee_fs_htree_node_image 0  vers 0 @ offs = 0
	 * tee_fs_htree_node_image 0  vers 1 @ offs = node_size
	 * tee_fs_htree_node_image 1  vers 0 @ offs = node_size * 2
	 * tee_fs_htree_node_image 1  vers 1 @ offs = node_size * 3
	 * ...
	 * tee_fs_htree_node_image 30 vers 0 @ offs = node_size * 60
	 * tee_fs_htree_node_image 30 vers 1 @ offs = node_size * 61
	 *
	 * phys block 2:
	 * data block 0 vers 0
	 *
	 * phys block 3:
	 * data block 0 vers 1
	 *
	 * ...
	 * phys block 62:
	 * data block 30 vers 0
	 *
	 * phys block 63:
	 * data block 30 vers 1
	 *
	 * phys block 64:
	 * tee_fs_htree_node_image 31  vers 0 @ offs = 0
	 * tee_fs_htree_node_image 31  vers 1 @ offs = node_size
	 * tee_fs_htree_node_image 32  vers 0 @ offs = node_size * 2
	 * tee_fs_htree_node_image 32  vers 1 @ offs = node_size * 3
	 * ...
	 * tee_fs_htree_node_image 61 vers 0 @ offs = node_size * 60
	 * tee_fs_htree_node_image 61 vers 1 @ offs = node_size * 61
	 *
	 * phys block 65:
	 * data block 31 vers 0
	 *
	 * phys block 66:
	 * data block 31 vers 1
	 * ...
	 */

	switch (type) {
	case TEE_FS_HTREE_TYPE_HEAD:
		*offs = sizeof(struct tee_fs_htree_image) * vers;
		*size = sizeof(struct tee_fs_htree_image);
		return TEE_SUCCESS;
	case TEE_FS_HTREE_TYPE_NODE:
		pbn = 1 + (idx64 / block_nodes) * block_nodes * 2;
		*offs = pbn * BLOCK_SIZE +
			2 * node_size * (idx64 % block_nodes) +
			node_size * vers;
		*size = node_size;
		return TEE_SUCCESS;
	case TEE_FS_HTREE_TYPE_BLOCK:
		bidx = 2 * idx64 + vers;
		pbn = 2 + bidx + bidx / (block_nodes * 2 - 1);
		*offs = pbn * BLOCK_SIZE;
		*size = BLOCK_SIZE;
		return TEE_SUCCESS;
	default:
		return TEE_ERROR_GENERIC;
	}
}

static TEE_Result ree_fs_rpc_read_init(void *aux,
				       struct tee_fs_rpc_operation *op,
				       enum tee_fs_htree_type type, size_t idx,
				       uint8_t vers, void **data)
{
	int fd = *(int *)aux;
	TEE_Result res;
	tee_fs_off_t offs = 0;
	size_t size;

	res = get_offs_size(type, idx, vers, &offs, &size);
	if (res != TEE_SUCCESS)
		return res;

	return tee_fs_rpc_read_init(op, OPTEE_RPC_CMD_FS, fd, offs, size,
				    data);
}

static TEE_Result ree_fs_rpc_write_init(void *aux,
					struct tee_fs_rpc_operation *op,
					enum tee_fs_htree_type type, size_t idx,
					uint8_t vers, void **data)
{
	int fd = *(int *)aux;
	TEE_Result res;
	tee_fs_off_t offs = 0;
	size_t size;

	res = get_offs_size(type, idx, vers, &offs, &size);
	if (res != TEE_SUCCESS)
		return res;

	return tee_fs_rpc_write_init(op, OPTEE_RPC_CMD_FS, fd, offs, size,
				     data);
}

static const struct tee_fs_htree_storage ree_fs_storage_ops = {
	.block_size = BLOCK_SIZE,
	.rpc_read_init = ree_fs_rpc_read_init,
	.rpc_read_final = tee_fs_rpc_read_final,
	.rpc_write_init = ree_fs_rpc_write_init,
	.rpc_write_final = tee_fs_rpc_write_final,
};

static TEE_Result ree_fs_rpc_open(const struct tee_fs_dirfile_fileh *dfh,
				  int *fd)
{
	return tee_fs_rpc_open_dfh(OPTEE_RPC_CMD_FS, dfh, fd);
}

static TEE_Result ree_fs_rpc_create(const struct tee_fs_dirfile_fileh *dfh,
				    int *fd)
{
	return tee_fs_rpc_create_dfh(OPTEE_RPC_CMD_FS, dfh, fd);
}

static TEE_Result ree_fs_rpc_close(int fd)
{
	return tee_fs_rpc_close(OPTEE_RPC_CMD_FS, fd);
}

static TEE_Result ree_fs_rpc_truncate(int fd, size_t num_blocks)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	tee_fs_off_t offs = 0;
	size_t sz = 0;

	res = get_offs_size(TEE_FS_HTREE_TYPE_BLOCK, num_blocks, 1, &offs,
			    &sz);
	if (res != TEE_SUCCESS)
		return res;

	return tee_fs_rpc_truncate(OPTEE_RPC_CMD_FS, fd,
				   offs + (tee_fs_off_t)sz);
}

static TEE_Result ree_fs_rpc_remove(const struct tee_fs_dirfile_fileh *dfh)
{
	return tee_fs_rpc_remove_dfh(OPTEE_RPC_CMD_FS, dfh);
}

static const struct tee_fs_backend_ops ree_fs_rpc_backend = {
	.htree_storage = &ree_fs_storage_ops,
	.open = ree_fs_rpc_open,
	.create = ree_fs_rpc_create,
	.close = ree_fs_rpc_close,
	.truncate = ree_fs_rpc_truncate,
	.remove = ree_fs_rpc_remove,
};
#endif /*CFG_REE_FS*/

static TEE_Result ree_fs_ftruncate_internal(struct tee_fs_fd *fdp,
					    tee_fs_off_t new_file_len)
{
	TEE_Result res;
	struct tee_fs_htree_meta *meta = NULL;

	assert(fdp);
	meta = tee_fs_htree_get_meta(fdp->ht);
	if (!meta)
		return TEE_ERROR_CORRUPT_OBJECT;

	if ((size_t)new_file_len > meta->length) {
		size_t ext_len = new_file_len - meta->length;

		res = out_of_place_write(fdp, meta->length, NULL, NULL,
					 ext_len);
		if (res != TEE_SUCCESS)
			return res;
	} else {
		/*
		 * The blocks past the new end stay in storage until the
		 * caller has committed: until then the old tree uses them.
		 */
		res = tee_fs_htree_truncate(&fdp->ht,
					    new_file_len / BLOCK_SIZE);
		if (res != TEE_SUCCESS)
			return res;

		meta->length = new_file_len;
		tee_fs_htree_meta_set_dirty(fdp->ht);
	}

	return TEE_SUCCESS;
}

static TEE_Result ree_fs_read_primitive(struct tee_file_handle *fh, size_t pos,
					void *buf_core, void *buf_user,
					size_t *len)
{
	TEE_Result res;
	size_t start_block_num = 0;
	size_t end_block_num = 0;
	size_t remain_bytes;
	uint8_t *data_core_ptr = buf_core;
	uint8_t *data_user_ptr = buf_user;
	uint8_t *block = NULL;
	struct tee_fs_fd *fdp = (struct tee_fs_fd *)fh;
	struct tee_fs_htree_meta *meta = NULL;

	/* One of buf_core and buf_user must be NULL */
	assert(!buf_core || !buf_user);

	assert(fdp);
	meta = tee_fs_htree_get_meta(fdp->ht);
	if (!meta)
		return TEE_ERROR_CORRUPT_OBJECT;

	remain_bytes = *len;
	if ((pos + remain_bytes) < remain_bytes || pos > meta->length)
		remain_bytes = 0;
	else if (pos + remain_bytes > meta->length)
		remain_bytes = meta->length - pos;

	*len = remain_bytes;

	if (!remain_bytes) {
		res = TEE_SUCCESS;
		goto exit;
	}

	start_block_num = pos_to_block_num(pos);
	end_block_num = pos_to_block_num(pos + remain_bytes - 1);

	block = get_tmp_block();
	if (!block) {
		res = TEE_ERROR_OUT_OF_MEMORY;
		goto exit;
	}

	while (start_block_num <= end_block_num) {
		size_t offset = pos % BLOCK_SIZE;
		size_t size_to_read = MIN(remain_bytes, (size_t)BLOCK_SIZE);

		if (size_to_read + offset > BLOCK_SIZE)
			size_to_read = BLOCK_SIZE - offset;

		res = tee_fs_htree_read_block(&fdp->ht, start_block_num, block);
		if (res != TEE_SUCCESS)
			goto exit;

		if (data_core_ptr) {
			memcpy(data_core_ptr, block + offset, size_to_read);
			data_core_ptr += size_to_read;
		} else if (data_user_ptr) {
			res = copy_to_user(data_user_ptr, block + offset,
					   size_to_read);
			if (res)
				goto exit;
			data_user_ptr += size_to_read;
		}

		remain_bytes -= size_to_read;
		pos += size_to_read;

		start_block_num++;
	}
	res = TEE_SUCCESS;
exit:
	if (block)
		put_tmp_block(block);
	return res;
}

static TEE_Result ree_fs_read(struct tee_file_handle *fh, size_t pos,
			      void *buf_core, void *buf_user, size_t *len)
{
	struct fs_instance *fs = ((struct tee_fs_fd *)fh)->fs;
	TEE_Result res;

	mutex_lock(&fs->mutex);
	res = ree_fs_read_primitive(fh, pos, buf_core, buf_user, len);
	mutex_unlock(&fs->mutex);

	return res;
}

static TEE_Result ree_fs_write_primitive(struct tee_file_handle *fh, size_t pos,
					 const void *buf_core,
					 const void *buf_user, size_t len)
{
	TEE_Result res;
	struct tee_fs_fd *fdp = (struct tee_fs_fd *)fh;
	struct tee_fs_htree_meta *meta = NULL;
	size_t file_size;

	/* One of buf_core and buf_user must be NULL */
	assert(!buf_core || !buf_user);

	if (!len)
		return TEE_SUCCESS;

	assert(fdp);
	meta = tee_fs_htree_get_meta(fdp->ht);
	if (!meta)
		return TEE_ERROR_CORRUPT_OBJECT;

	file_size = meta->length;

	if ((pos + len) < len)
		return TEE_ERROR_BAD_PARAMETERS;

	if (file_size < pos) {
		res = ree_fs_ftruncate_internal(fdp, pos);
		if (res != TEE_SUCCESS)
			return res;
	}

	return out_of_place_write(fdp, pos, buf_core, buf_user, len);
}

static TEE_Result ree_fs_open_primitive(struct fs_instance *fs, bool create,
					uint8_t *hash, uint32_t min_counter,
					const TEE_UUID *uuid,
					struct tee_fs_dirfile_fileh *dfh,
					struct tee_file_handle **fh)
{
	TEE_Result res;
	struct tee_fs_fd *fdp;

	fdp = calloc(1, sizeof(struct tee_fs_fd));
	if (!fdp)
		return TEE_ERROR_OUT_OF_MEMORY;
	fdp->fs = fs;
	fdp->fd = -1;
	fdp->uuid = uuid;

	if (create)
		res = fs->be->create(dfh, &fdp->fd);
	else
		res = fs->be->open(dfh, &fdp->fd);

	if (res != TEE_SUCCESS)
		goto out;

	res = tee_fs_htree_open(create, hash, min_counter, uuid,
				fs->be->htree_storage, &fdp->fd, &fdp->ht);
out:
	if (res == TEE_SUCCESS) {
		if (dfh)
			fdp->dfh = *dfh;
		else
			fdp->dfh.idx = -1;
		*fh = (struct tee_file_handle *)fdp;
	} else {
		if (res == TEE_ERROR_SECURITY)
			EMSG("Secure storage corruption detected");
		if (fdp->fd != -1)
			fs->be->close(fdp->fd);
		/*
		 * Remove the file if hash is NULL and min_counter is 0,
		 * as it is not yet rollback-protected, but only when it is
		 * found corrupt. A failed request says nothing about it.
		 */
		if (create || (!hash && !min_counter &&
			       (res == TEE_ERROR_CORRUPT_OBJECT ||
				res == TEE_ERROR_SECURITY))) {
			DMSG("Remove corrupt file");
			fs->be->remove(dfh);
		}
		free(fdp);
	}

	return res;
}

static void ree_fs_close_primitive(struct tee_file_handle *fh)
{
	struct tee_fs_fd *fdp = (struct tee_fs_fd *)fh;

	if (fdp) {
		tee_fs_htree_close(&fdp->ht);
		fdp->fs->be->close(fdp->fd);
		free(fdp);
	}
}

static TEE_Result ree_dirf_commit_writes(struct tee_file_handle *fh,
					 uint8_t *hash, uint32_t *counter)
{
	TEE_Result res;
	struct tee_fs_fd *fdp = (struct tee_fs_fd *)fh;

	res = tee_fs_htree_sync_to_storage(&fdp->ht, fdp->dfh.hash, counter);

	if (!res && hash)
		memcpy(hash, fdp->dfh.hash, sizeof(fdp->dfh.hash));

	return res;
}

static TEE_Result dirf_read(struct tee_file_handle *fh, size_t pos, void *buf,
			    size_t *len)
{
	return ree_fs_read_primitive(fh, pos, buf, NULL, len);
}

static TEE_Result dirf_write(struct tee_file_handle *fh, size_t pos,
			     const void *buf, size_t len)
{
	return ree_fs_write_primitive(fh, pos, buf, NULL, len);
}

#ifdef CFG_REE_FS
static struct fs_instance ree_fs_inst;

static TEE_Result ree_dirf_open(bool create, uint8_t *hash,
				uint32_t min_counter, const TEE_UUID *uuid,
				struct tee_fs_dirfile_fileh *dfh,
				struct tee_file_handle **fh)
{
	return ree_fs_open_primitive(&ree_fs_inst, create, hash, min_counter,
				     uuid, dfh, fh);
}

static const struct tee_fs_dirfile_operations ree_dirf_ops = {
	.open = ree_dirf_open,
	.close = ree_fs_close_primitive,
	.read = dirf_read,
	.write = dirf_write,
	.commit_writes = ree_dirf_commit_writes,
};

#ifdef CFG_REE_FS_INTEGRITY_RPMB
static struct tee_file_handle *ree_fs_rpmb_fh;

static TEE_Result open_dirh(struct fs_instance *fs,
			    struct tee_fs_dirfile_dirh **dirh)
{
	TEE_Result res;
	uint8_t hash[TEE_FS_HTREE_HASH_SIZE];
	uint8_t *hashp = NULL;
	const char fname[] = "dirfile.db.hash";

	res = tee_rpmb_fs_raw_open(fname, false, &ree_fs_rpmb_fh);
	if (!res) {
		size_t l = sizeof(hash);

		res = rpmb_fs_ops.read(ree_fs_rpmb_fh, 0, hash, NULL, &l);
		if (res)
			return res;
		if (l == sizeof(hash))
			hashp = hash;
	} else if (res == TEE_ERROR_ITEM_NOT_FOUND) {
		res = tee_rpmb_fs_raw_open(fname, true, &ree_fs_rpmb_fh);
	}
	if (res)
		return res;

	if (hashp) {
		res = tee_fs_dirfile_open(false, hashp, 0, fs->dirf_ops, dirh);
		if (res != TEE_ERROR_ITEM_NOT_FOUND)
			goto out;

		if (!IS_ENABLED(CFG_REE_FS_ALLOW_RESET)) {
			EMSG("dirf.db file not found");
			res = TEE_ERROR_SECURITY;
			goto out;
		}

		/*
		 * Drop the anchor before creating the new dirf.db.
		 * An interruption from here on leaves the anchor empty, which
		 * is handled as uninitialized below and comes back to this
		 * path on the next boot.
		 */
		IMSG("dirf.db not found, resetting secure storage");
		res = rpmb_fs_ops.truncate(ree_fs_rpmb_fh, 0);
		if (res) {
			EMSG("Can't clear hash: %#"PRIx32, res);
			res = TEE_ERROR_SECURITY;
			goto out;
		}
	}

	/*
	 * Without an anchor no dirf.db can be authenticated, so secure
	 * storage is uninitialized.
	 * Create an empty dirf.db, overwriting anything left behind, and
	 * anchor it before use so that this is the only state which can
	 * be opened later.
	 */
	IMSG("Create dirf.db");
	res = tee_fs_dirfile_open(true, hash, 0, fs->dirf_ops, dirh);
	if (res)
		goto out;

	res = rpmb_fs_ops.write(ree_fs_rpmb_fh, 0, hash, NULL, sizeof(hash));
	if (res) {
		EMSG("Can't anchor dirf.db: %#"PRIx32, res);
		tee_fs_dirfile_close(*dirh);
		*dirh = NULL;
		res = TEE_ERROR_SECURITY;
	}

out:
	if (res)
		rpmb_fs_ops.close(&ree_fs_rpmb_fh);

	return res;
}

static TEE_Result commit_dirh_writes(struct fs_instance *fs __unused,
				     struct tee_fs_dirfile_dirh *dirh)
{
	TEE_Result res;
	uint8_t hash[TEE_FS_HTREE_HASH_SIZE];

	res = tee_fs_dirfile_commit_writes(dirh, hash, NULL);
	if (res)
		return res;
	return rpmb_fs_ops.write(ree_fs_rpmb_fh, 0, hash, NULL, sizeof(hash));
}

static void close_dirh(struct fs_instance *fs __unused,
		       struct tee_fs_dirfile_dirh **dirh)
{
	tee_fs_dirfile_close(*dirh);
	*dirh = NULL;
	rpmb_fs_ops.close(&ree_fs_rpmb_fh);
}

#else /*!CFG_REE_FS_INTEGRITY_RPMB*/
static TEE_Result open_dirh(struct fs_instance *fs,
			    struct tee_fs_dirfile_dirh **dirh)
{
	TEE_Result res = TEE_SUCCESS;
	uint32_t min_counter = 0;

	res = nv_counter_get_ree_fs(&min_counter);
	if (res) {
		static bool once;

		if (res != TEE_ERROR_NOT_IMPLEMENTED ||
		    !IS_ENABLED(CFG_INSECURE))
			return res;

		if (!once) {
			IMSG("WARNING (insecure configuration): Failed to get monotonic counter for REE FS, using 0");
			once = true;
		}
		min_counter = 0;
	}
	res = tee_fs_dirfile_open(false, NULL, min_counter, fs->dirf_ops,
				  dirh);
	if (res == TEE_ERROR_ITEM_NOT_FOUND) {
		if (min_counter) {
			if (!IS_ENABLED(CFG_REE_FS_ALLOW_RESET)) {
				EMSG("dirf.db file not found");
				return TEE_ERROR_SECURITY;
			}
			DMSG("dirf.db not found, initializing with a non-zero monotonic counter");
		}
		return tee_fs_dirfile_open(true, NULL, min_counter,
					   fs->dirf_ops, dirh);
	}

	return res;
}

static TEE_Result commit_dirh_writes(struct fs_instance *fs __unused,
				     struct tee_fs_dirfile_dirh *dirh)
{
	TEE_Result res = TEE_SUCCESS;
	uint32_t counter = 0;

	res = tee_fs_dirfile_commit_writes(dirh, NULL, &counter);
	if (res)
		return res;
	res = nv_counter_incr_ree_fs_to(counter);
	if (res == TEE_ERROR_NOT_IMPLEMENTED && IS_ENABLED(CFG_INSECURE)) {
		static bool once;

		if (!once) {
			IMSG("WARNING (insecure configuration): Failed to commit dirh counter %"PRIu32, counter);
			once = true;
		}
		return TEE_SUCCESS;
	}
	return res;
}

static void close_dirh(struct fs_instance *fs __unused,
		       struct tee_fs_dirfile_dirh **dirh)
{
	tee_fs_dirfile_close(*dirh);
	*dirh = NULL;
}
#endif /*!CFG_REE_FS_INTEGRITY_RPMB*/

static struct fs_instance ree_fs_inst = {
	.be = &ree_fs_rpc_backend,
	.dirf_ops = &ree_dirf_ops,
	.open_dirh = open_dirh,
	.commit_dirh_writes = commit_dirh_writes,
	.close_dirh = close_dirh,
	.mutex = MUTEX_INITIALIZER,
};
#endif /*CFG_REE_FS*/

/*
 * fs->dirh is caching the dirfile handle to avoid frequent opening and
 * closing of that handle. When fs->dirh_refcount reaches 0, fs->dirh will
 * be freed. However, fs->dirh_refcount > 0 is not a guarantee that fs->dirh
 * will not be freed, it may very well be freed earlier in an error path.
 * get_dirh() must be used to get the fs->dirh pointer each time it's
 * needed if fs->mutex has been unlocked in between.
 */
static TEE_Result get_dirh(struct fs_instance *fs,
			   struct tee_fs_dirfile_dirh **dirh)
{
	if (!fs->dirh) {
		TEE_Result res = fs->open_dirh(fs, &fs->dirh);

		if (res) {
			*dirh = NULL;
			return res;
		}
	}
	fs->dirh_refcount++;
	assert(fs->dirh);
	assert(fs->dirh_refcount);
	*dirh = fs->dirh;
	return TEE_SUCCESS;
}

static void put_dirh_primitive(struct fs_instance *fs, bool close)
{
	assert(fs->dirh_refcount);

	/*
	 * During the execution of one of the ree_fs_ops fs->dirh is
	 * guareteed to be a valid pointer. But when the fop has returned
	 * another thread may get an error or something causing that fop
	 * to do a put with close=1.
	 *
	 * For all fops but ree_fs_close() and ree_fs_closedir_rpc() there's a
	 * call to get_dirh() to get a new dirh which will open it again if it
	 * was closed before. But in the ree_fs_close() and
	 * ree_fs_closedir_rpc() cases there's no call to get_dirh() only to
	 * this function, put_dirh_primitive(), and in this case
	 * fs->dirh may actually be NULL.
	 */
	fs->dirh_refcount--;
	if (fs->dirh && (!fs->dirh_refcount || close))
		fs->close_dirh(fs, &fs->dirh);
}

static void put_dirh(struct fs_instance *fs,
		     struct tee_fs_dirfile_dirh *dirh, bool close)
{
	if (dirh) {
		assert(dirh == fs->dirh);
		put_dirh_primitive(fs, close);
	}
}

static TEE_Result fs_open(struct fs_instance *fs, struct tee_pobj *po,
			  size_t *size, struct tee_file_handle **fh)
{
	TEE_Result res;
	struct tee_fs_dirfile_dirh *dirh = NULL;
	struct tee_fs_dirfile_fileh dfh;

	mutex_lock(&fs->mutex);

	res = get_dirh(fs, &dirh);
	if (res != TEE_SUCCESS)
		goto out;

	res = tee_fs_dirfile_find(dirh, &po->uuid, po->obj_id, po->obj_id_len,
				  &dfh);
	if (res != TEE_SUCCESS)
		goto out;

	res = ree_fs_open_primitive(fs, false, dfh.hash, 0, &po->uuid, &dfh,
				    fh);
	if (res == TEE_ERROR_ITEM_NOT_FOUND) {
		/*
		 * If the object isn't found someone has tampered with it,
		 * treat it as corrupt.
		 */
		res = TEE_ERROR_CORRUPT_OBJECT;
	} else if (!res && size) {
		struct tee_fs_fd *fdp = (struct tee_fs_fd *)*fh;

		assert(fdp->ht);
		*size = tee_fs_htree_get_meta(fdp->ht)->length;
	}

out:
	if (res)
		put_dirh(fs, dirh, true);
	mutex_unlock(&fs->mutex);

	return res;
}

static TEE_Result set_name(struct tee_fs_dirfile_dirh *dirh,
			   struct tee_fs_fd *fdp, struct tee_pobj *po,
			   bool overwrite)
{
	struct fs_instance *fs = fdp->fs;
	TEE_Result res;
	bool have_old_dfh = false;
	struct tee_fs_dirfile_fileh old_dfh = { .idx = -1 };

	res = tee_fs_dirfile_find(dirh, &po->uuid, po->obj_id, po->obj_id_len,
				  &old_dfh);
	if (!overwrite && !res)
		return TEE_ERROR_ACCESS_CONFLICT;

	if (!res)
		have_old_dfh = true;

	/*
	 * If old_dfh wasn't found, the idx will be -1 and
	 * tee_fs_dirfile_rename() will allocate a new index.
	 */
	fdp->dfh.idx = old_dfh.idx;
	old_dfh.idx = -1;
	res = tee_fs_dirfile_rename(dirh, &po->uuid, &fdp->dfh,
				    po->obj_id, po->obj_id_len);
	if (res)
		return res;

	res = fs->commit_dirh_writes(fs, dirh);
	if (res)
		return res;

	if (have_old_dfh)
		fs->be->remove(&old_dfh);

	return TEE_SUCCESS;
}

static void ree_fs_close(struct tee_file_handle **fh)
{
	if (*fh) {
		struct fs_instance *fs = ((struct tee_fs_fd *)*fh)->fs;

		mutex_lock(&fs->mutex);
		put_dirh_primitive(fs, false);
		ree_fs_close_primitive(*fh);
		*fh = NULL;
		mutex_unlock(&fs->mutex);

	}
}

static TEE_Result fs_create(struct fs_instance *fs, struct tee_pobj *po,
			    bool overwrite, const void *head, size_t head_size,
			    const void *attr, size_t attr_size,
			    const void *data_core, const void *data_user,
			    size_t data_size, struct tee_file_handle **fh)
{
	struct tee_fs_fd *fdp;
	struct tee_fs_dirfile_dirh *dirh = NULL;
	struct tee_fs_dirfile_fileh dfh;
	TEE_Result res;
	size_t pos = 0;

	/* One of data_core and data_user must be NULL */
	assert(!data_core || !data_user);

	*fh = NULL;
	mutex_lock(&fs->mutex);

	res = get_dirh(fs, &dirh);
	if (res)
		goto out;

	res = tee_fs_dirfile_get_tmp(dirh, &dfh);
	if (res)
		goto out;

	res = ree_fs_open_primitive(fs, true, dfh.hash, 0, &po->uuid, &dfh, fh);
	if (res)
		goto out;

	if (head && head_size) {
		res = ree_fs_write_primitive(*fh, pos, head, NULL, head_size);
		if (res)
			goto out;
		pos += head_size;
	}

	if (attr && attr_size) {
		res = ree_fs_write_primitive(*fh, pos, attr, NULL, attr_size);
		if (res)
			goto out;
		pos += attr_size;
	}

	if ((data_core || data_user) && data_size) {
		res = ree_fs_write_primitive(*fh, pos, data_core, data_user,
					     data_size);
		if (res)
			goto out;
	}

	fdp = (struct tee_fs_fd *)*fh;
	res = tee_fs_htree_sync_to_storage(&fdp->ht, fdp->dfh.hash, NULL);
	if (res)
		goto out;

	res = set_name(dirh, fdp, po, overwrite);
out:
	if (res) {
		put_dirh(fs, dirh, true);
		if (*fh) {
			ree_fs_close_primitive(*fh);
			*fh = NULL;
			fs->be->remove(&dfh);
		}
	}
	mutex_unlock(&fs->mutex);

	return res;
}

static TEE_Result ree_fs_write(struct tee_file_handle *fh, size_t pos,
			       const void *buf_core, const void *buf_user,
			       size_t len)
{
	TEE_Result res;
	struct tee_fs_dirfile_dirh *dirh = NULL;
	struct tee_fs_fd *fdp = (struct tee_fs_fd *)fh;
	struct fs_instance *fs = fdp->fs;

	/* One of buf_core and buf_user must be NULL */
	assert(!buf_core || !buf_user);

	mutex_lock(&fs->mutex);

	res = get_dirh(fs, &dirh);
	if (res)
		goto out;

	res = ree_fs_write_primitive(fh, pos, buf_core, buf_user, len);
	if (res)
		goto out;

	res = tee_fs_htree_sync_to_storage(&fdp->ht, fdp->dfh.hash, NULL);
	if (res)
		goto out;

	res = tee_fs_dirfile_update_hash(dirh, &fdp->dfh);
	if (res)
		goto out;
	res = fs->commit_dirh_writes(fs, dirh);
out:
	put_dirh(fs, dirh, res);
	mutex_unlock(&fs->mutex);

	return res;
}

static TEE_Result fs_rename(struct fs_instance *fs, struct tee_pobj *old,
			    struct tee_pobj *new, bool overwrite)
{
	TEE_Result res;
	struct tee_fs_dirfile_dirh *dirh = NULL;
	struct tee_fs_dirfile_fileh dfh;
	struct tee_fs_dirfile_fileh remove_dfh = { .idx = -1 };

	if (!new)
		return TEE_ERROR_BAD_PARAMETERS;

	mutex_lock(&fs->mutex);
	res = get_dirh(fs, &dirh);
	if (res)
		goto out;

	res = tee_fs_dirfile_find(dirh, &new->uuid, new->obj_id,
				  new->obj_id_len, &remove_dfh);
	if (!res && !overwrite) {
		res = TEE_ERROR_ACCESS_CONFLICT;
		goto out;
	}

	res = tee_fs_dirfile_find(dirh, &old->uuid, old->obj_id,
				  old->obj_id_len, &dfh);
	if (res)
		goto out;

	res = tee_fs_dirfile_rename(dirh, &new->uuid, &dfh, new->obj_id,
				    new->obj_id_len);
	if (res)
		goto out;

	if (remove_dfh.idx != -1) {
		res = tee_fs_dirfile_remove(dirh, &remove_dfh);
		if (res)
			goto out;
	}

	res = fs->commit_dirh_writes(fs, dirh);
	if (res)
		goto out;

	if (remove_dfh.idx != -1)
		fs->be->remove(&remove_dfh);

out:
	put_dirh(fs, dirh, res);
	mutex_unlock(&fs->mutex);

	return res;

}

static TEE_Result fs_remove(struct fs_instance *fs, struct tee_pobj *po)
{
	TEE_Result res;
	struct tee_fs_dirfile_dirh *dirh = NULL;
	struct tee_fs_dirfile_fileh dfh;

	mutex_lock(&fs->mutex);
	res = get_dirh(fs, &dirh);
	if (res)
		goto out;

	res = tee_fs_dirfile_find(dirh, &po->uuid, po->obj_id, po->obj_id_len,
				  &dfh);
	if (res)
		goto out;

	res = tee_fs_dirfile_remove(dirh, &dfh);
	if (res)
		goto out;

	res = fs->commit_dirh_writes(fs, dirh);
	if (res)
		goto out;

	fs->be->remove(&dfh);

	assert(tee_fs_dirfile_find(dirh, &po->uuid, po->obj_id, po->obj_id_len,
				   &dfh));
out:
	put_dirh(fs, dirh, res);
	mutex_unlock(&fs->mutex);

	return res;
}

static TEE_Result ree_fs_truncate(struct tee_file_handle *fh, size_t len)
{
	TEE_Result res;
	struct tee_fs_dirfile_dirh *dirh = NULL;
	struct tee_fs_fd *fdp = (struct tee_fs_fd *)fh;
	struct fs_instance *fs = fdp->fs;
	struct tee_fs_htree_meta *meta = NULL;
	bool shrink = false;

	mutex_lock(&fs->mutex);

	res = get_dirh(fs, &dirh);
	if (res)
		goto out;

	meta = tee_fs_htree_get_meta(fdp->ht);
	shrink = meta && len < meta->length;

	res = ree_fs_ftruncate_internal(fdp, len);
	if (res)
		goto out;

	res = tee_fs_htree_sync_to_storage(&fdp->ht, fdp->dfh.hash, NULL);
	if (res)
		goto out;

	res = tee_fs_dirfile_update_hash(dirh, &fdp->dfh);
	if (res)
		goto out;
	res = fs->commit_dirh_writes(fs, dirh);
	if (res || !shrink)
		goto out;

	/*
	 * Drop the blocks past the new end only now that no committed tree
	 * uses them. If this fails they are just unused space.
	 */
	if (fs->be->truncate(fdp->fd, ROUNDUP_DIV(len, BLOCK_SIZE)))
		DMSG("Truncation of the stored file failed");
out:
	put_dirh(fs, dirh, res);
	mutex_unlock(&fs->mutex);

	return res;
}

static TEE_Result fs_opendir(struct fs_instance *fs, const TEE_UUID *uuid,
			     struct tee_fs_dir **dir)
{
	TEE_Result res = TEE_SUCCESS;
	struct tee_fs_dirfile_dirh *dirh = NULL;
	struct tee_fs_dir *d = calloc(1, sizeof(*d));

	if (!d)
		return TEE_ERROR_OUT_OF_MEMORY;

	d->fs = fs;
	d->uuid = uuid;

	mutex_lock(&fs->mutex);

	res = get_dirh(fs, &dirh);
	if (res)
		goto out;

	/* See that there's at least one file */
	d->idx = -1;
	d->d.oidlen = sizeof(d->d.oid);
	res = tee_fs_dirfile_get_next(dirh, d->uuid, &d->idx, d->d.oid,
				      &d->d.oidlen);
	d->idx = -1;

out:
	if (!res) {
		*dir = d;
	} else {
		if (d)
			put_dirh(fs, dirh, false);
		free(d);
	}
	mutex_unlock(&fs->mutex);

	return res;
}

static void ree_fs_closedir_rpc(struct tee_fs_dir *d)
{
	if (d) {
		struct fs_instance *fs = d->fs;

		mutex_lock(&fs->mutex);

		put_dirh_primitive(fs, false);
		free(d);

		mutex_unlock(&fs->mutex);
	}
}

static TEE_Result ree_fs_readdir_rpc(struct tee_fs_dir *d,
				     struct tee_fs_dirent **ent)
{
	struct fs_instance *fs = d->fs;
	struct tee_fs_dirfile_dirh *dirh = NULL;
	TEE_Result res = TEE_SUCCESS;
	bool close = false;

	mutex_lock(&fs->mutex);

	res = get_dirh(fs, &dirh);
	if (res)
		goto out;

	d->d.oidlen = sizeof(d->d.oid);
	res = tee_fs_dirfile_get_next(dirh, d->uuid, &d->idx, d->d.oid,
				      &d->d.oidlen);
	if (res == TEE_SUCCESS)
		*ent = &d->d;

	close = (res != TEE_SUCCESS && res != TEE_ERROR_ITEM_NOT_FOUND);

	put_dirh(fs, dirh, close);
out:
	mutex_unlock(&fs->mutex);

	return res;
}

#ifdef CFG_REE_FS
static TEE_Result ree_fs_open(struct tee_pobj *po, size_t *size,
			      struct tee_file_handle **fh)
{
	return fs_open(&ree_fs_inst, po, size, fh);
}

static TEE_Result ree_fs_create(struct tee_pobj *po, bool overwrite,
				const void *head, size_t head_size,
				const void *attr, size_t attr_size,
				const void *data_core, const void *data_user,
				size_t data_size, struct tee_file_handle **fh)
{
	return fs_create(&ree_fs_inst, po, overwrite, head, head_size, attr,
			 attr_size, data_core, data_user, data_size, fh);
}

static TEE_Result ree_fs_rename(struct tee_pobj *old, struct tee_pobj *new,
				bool overwrite)
{
	return fs_rename(&ree_fs_inst, old, new, overwrite);
}

static TEE_Result ree_fs_remove(struct tee_pobj *po)
{
	return fs_remove(&ree_fs_inst, po);
}

static TEE_Result ree_fs_opendir_rpc(const TEE_UUID *uuid,
				     struct tee_fs_dir **dir)
{
	return fs_opendir(&ree_fs_inst, uuid, dir);
}

const struct tee_file_operations ree_fs_ops = {
	.open = ree_fs_open,
	.create = ree_fs_create,
	.close = ree_fs_close,
	.read = ree_fs_read,
	.write = ree_fs_write,
	.truncate = ree_fs_truncate,
	.rename = ree_fs_rename,
	.remove = ree_fs_remove,
	.opendir = ree_fs_opendir_rpc,
	.closedir = ree_fs_closedir_rpc,
	.readdir = ree_fs_readdir_rpc,
};
#endif /*CFG_REE_FS*/

#ifdef CFG_SPI_NOR_FS
/*
 * SPI-NOR FS: the same file system on a region of a secure-only SPI-NOR
 * flash (core/tee/spinor_fs_store.c), with no normal-world involvement.
 */
static struct fs_instance spinor_fs_inst;

static TEE_Result spinor_dirf_open(bool create, uint8_t *hash,
				   uint32_t min_counter, const TEE_UUID *uuid,
				   struct tee_fs_dirfile_fileh *dfh,
				   struct tee_file_handle **fh)
{
	return ree_fs_open_primitive(&spinor_fs_inst, create, hash,
				     min_counter, uuid, dfh, fh);
}

static const struct tee_fs_dirfile_operations spinor_dirf_ops = {
	.open = spinor_dirf_open,
	.close = ree_fs_close_primitive,
	.read = dirf_read,
	.write = dirf_write,
	.commit_writes = ree_dirf_commit_writes,
};

/*
 * Rollback anchor of SPI-NOR FS
 *
 * The flash is only reachable from the secure world, so the normal world
 * cannot roll it back, but a physical attacker can reprogram it with an
 * older image. The anchor is a counter outside the flash that can only
 * increase; dirf.db must carry a counter at least as high to mount, and
 * each commit raises the anchor to the new dirf.db counter, as for REE FS
 * with nv_counter_*_ree_fs().
 *
 * The anchor is an RPMC counter of the flash if it has one that is
 * provisioned, else the platform counter behind nv_counter_*_spinor_fs().
 * Without either, only an insecure configuration mounts. The policy comes
 * from CFG_SPI_NOR_FS_ANCHOR or the "optee,rollback-anchor" property of
 * the partition in the secure DT: "auto", "rpmc" or "nv-counter".
 */
enum spinor_anchor {
	SPINOR_ANCHOR_UNSET,
	SPINOR_ANCHOR_NONE,
	SPINOR_ANCHOR_RPMC,
	SPINOR_ANCHOR_NV_COUNTER,
};

enum spinor_anchor_policy {
	SPINOR_POLICY_AUTO,
	SPINOR_POLICY_RPMC,
	SPINOR_POLICY_NV_COUNTER,
};

/* CFG_SPI_NOR_FS_ANCHOR */
#if defined(_CFG_SPI_NOR_FS_ANCHOR_RPMC)
#define SPINOR_BUILD_POLICY	SPINOR_POLICY_RPMC
#elif defined(_CFG_SPI_NOR_FS_ANCHOR_NV_COUNTER)
#define SPINOR_BUILD_POLICY	SPINOR_POLICY_NV_COUNTER
#else
#define SPINOR_BUILD_POLICY	SPINOR_POLICY_AUTO
#endif

static enum spinor_anchor spinor_anchor;
static struct spi_nor *spinor_anchor_nor;

static TEE_Result spinor_anchor_policy(enum spinor_anchor_policy *policy)
{
	const char *dt = spinor_fs_store_dt_anchor();

	if (!dt) {
		*policy = SPINOR_BUILD_POLICY;
	} else if (!strcmp(dt, "auto")) {
		*policy = SPINOR_POLICY_AUTO;
	} else if (!strcmp(dt, "rpmc")) {
		*policy = SPINOR_POLICY_RPMC;
	} else if (!strcmp(dt, "nv-counter")) {
		*policy = SPINOR_POLICY_NV_COUNTER;
	} else {
		EMSG("SPI-NOR FS: bad optee,rollback-anchor \"%s\"", dt);
		return TEE_ERROR_BAD_FORMAT;
	}

	return TEE_SUCCESS;
}

/* Use the RPMC counter if the flash has it and it is provisioned */
static TEE_Result spinor_try_rpmc(void)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	struct spi_nor *nor = NULL;
	uint32_t val = 0;

	res = spinor_fs_store_get_flash(&nor);
	if (res)
		return res;

	res = spi_nor_rpmc_counter_get(nor, CFG_SPI_NOR_FS_RPMC_COUNTER, &val);
	if (res)
		return res;

	spinor_anchor_nor = nor;
	spinor_anchor = SPINOR_ANCHOR_RPMC;
	DMSG("SPI-NOR FS: RPMC counter %d anchors the file system",
	     CFG_SPI_NOR_FS_RPMC_COUNTER);

	return TEE_SUCCESS;
}

static TEE_Result spinor_select_anchor(void)
{
	enum spinor_anchor_policy policy = SPINOR_POLICY_AUTO;
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t val = 0;

	if (spinor_anchor != SPINOR_ANCHOR_UNSET)
		return TEE_SUCCESS;

	res = spinor_anchor_policy(&policy);
	if (res)
		return res;

	if (policy != SPINOR_POLICY_NV_COUNTER) {
		res = spinor_try_rpmc();
		if (!res)
			return TEE_SUCCESS;
		/* Never trade a counter that misbehaves for a weaker anchor */
		if (policy == SPINOR_POLICY_RPMC ||
		    (res != TEE_ERROR_NOT_SUPPORTED &&
		     res != TEE_ERROR_BAD_STATE)) {
			EMSG("SPI-NOR FS: no RPMC anchor: %#"PRIx32, res);
			return TEE_ERROR_STORAGE_NOT_AVAILABLE;
		}
	}

	res = nv_counter_get_spinor_fs(&val);
	if (!res) {
		spinor_anchor = SPINOR_ANCHOR_NV_COUNTER;
		DMSG("SPI-NOR FS: platform counter anchors the file system");
		return TEE_SUCCESS;
	}
	if (res != TEE_ERROR_NOT_IMPLEMENTED || policy != SPINOR_POLICY_AUTO ||
	    !IS_ENABLED(CFG_INSECURE)) {
		EMSG("SPI-NOR FS: no rollback anchor: %#"PRIx32, res);
		return TEE_ERROR_STORAGE_NOT_AVAILABLE;
	}

	IMSG("WARNING (insecure configuration): SPI-NOR FS has no rollback anchor");
	spinor_anchor = SPINOR_ANCHOR_NONE;

	return TEE_SUCCESS;
}

static TEE_Result spinor_anchor_get(uint32_t *value)
{
	switch (spinor_anchor) {
	case SPINOR_ANCHOR_RPMC:
		return spi_nor_rpmc_counter_get(spinor_anchor_nor,
						CFG_SPI_NOR_FS_RPMC_COUNTER,
						value);
	case SPINOR_ANCHOR_NV_COUNTER:
		return nv_counter_get_spinor_fs(value);
	default:
		*value = 0;
		return TEE_SUCCESS;
	}
}

static TEE_Result spinor_anchor_incr_to(uint32_t value)
{
	switch (spinor_anchor) {
	case SPINOR_ANCHOR_RPMC:
		return spi_nor_rpmc_counter_incr_to(spinor_anchor_nor,
						    CFG_SPI_NOR_FS_RPMC_COUNTER,
						    value);
	case SPINOR_ANCHOR_NV_COUNTER:
		return nv_counter_incr_spinor_fs_to(value);
	default:
		return TEE_SUCCESS;
	}
}

static TEE_Result spinor_open_dirh(struct fs_instance *fs,
				   struct tee_fs_dirfile_dirh **dirh)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t min_counter = 0;

	res = spinor_select_anchor();
	if (!res)
		res = spinor_anchor_get(&min_counter);
	if (res)
		return res;

	res = tee_fs_dirfile_open(false, NULL, min_counter, fs->dirf_ops,
				  dirh);
	if (res == TEE_ERROR_SECURITY)
		EMSG("SPI-NOR FS: dirf.db is older than the rollback anchor");
	if (res != TEE_ERROR_ITEM_NOT_FOUND)
		return res;

	if (min_counter) {
		if (!IS_ENABLED(CFG_SPI_NOR_FS_ALLOW_RESET)) {
			EMSG("SPI-NOR FS: dirf.db not found");
			return TEE_ERROR_SECURITY;
		}
		IMSG("SPI-NOR FS: dirf.db not found, starting over at counter %"PRIu32,
		     min_counter);
	}

	return tee_fs_dirfile_open(true, NULL, min_counter, fs->dirf_ops, dirh);
}

static TEE_Result spinor_commit_dirh_writes(struct fs_instance *fs __unused,
					    struct tee_fs_dirfile_dirh *dirh)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t counter = 0;

	res = tee_fs_dirfile_commit_writes(dirh, NULL, &counter);
	if (res)
		return res;

	return spinor_anchor_incr_to(counter);
}

static void spinor_close_dirh(struct fs_instance *fs __unused,
			      struct tee_fs_dirfile_dirh **dirh)
{
	tee_fs_dirfile_close(*dirh);
	*dirh = NULL;
}

static struct fs_instance spinor_fs_inst = {
	.be = &spinor_fs_backend,
	.dirf_ops = &spinor_dirf_ops,
	.open_dirh = spinor_open_dirh,
	.commit_dirh_writes = spinor_commit_dirh_writes,
	.close_dirh = spinor_close_dirh,
	.mutex = MUTEX_INITIALIZER,
};

static TEE_Result spinor_fs_open(struct tee_pobj *po, size_t *size,
				 struct tee_file_handle **fh)
{
	return fs_open(&spinor_fs_inst, po, size, fh);
}

static TEE_Result spinor_fs_create(struct tee_pobj *po, bool overwrite,
				   const void *head, size_t head_size,
				   const void *attr, size_t attr_size,
				   const void *data_core,
				   const void *data_user, size_t data_size,
				   struct tee_file_handle **fh)
{
	return fs_create(&spinor_fs_inst, po, overwrite, head, head_size, attr,
			 attr_size, data_core, data_user, data_size, fh);
}

static TEE_Result spinor_fs_rename(struct tee_pobj *old, struct tee_pobj *new,
				   bool overwrite)
{
	return fs_rename(&spinor_fs_inst, old, new, overwrite);
}

static TEE_Result spinor_fs_remove(struct tee_pobj *po)
{
	return fs_remove(&spinor_fs_inst, po);
}

static TEE_Result spinor_fs_opendir(const TEE_UUID *uuid,
				    struct tee_fs_dir **dir)
{
	return fs_opendir(&spinor_fs_inst, uuid, dir);
}

const struct tee_file_operations spinor_fs_ops = {
	.open = spinor_fs_open,
	.create = spinor_fs_create,
	.close = ree_fs_close,
	.read = ree_fs_read,
	.write = ree_fs_write,
	.truncate = ree_fs_truncate,
	.rename = spinor_fs_rename,
	.remove = spinor_fs_remove,
	.opendir = spinor_fs_opendir,
	.closedir = ree_fs_closedir_rpc,
	.readdir = ree_fs_readdir_rpc,
};

#ifdef CFG_SPI_NOR_EMBEDDED_TEST
static TEE_Result st_write(struct tee_pobj *po, bool create, uint8_t val)
{
	struct tee_file_handle *fh = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	size_t size = 0;

	if (create) {
		res = spinor_fs_ops.create(po, true, NULL, 0, NULL, 0, &val,
					   NULL, 1, &fh);
	} else {
		res = spinor_fs_ops.open(po, &size, &fh);
		if (!res)
			res = spinor_fs_ops.write(fh, 0, &val, NULL, 1);
	}
	spinor_fs_ops.close(&fh);

	return res;
}

static TEE_Result st_read(struct tee_pobj *po, uint8_t *val)
{
	struct tee_file_handle *fh = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	size_t size = 0;
	size_t len = 1;

	res = spinor_fs_ops.open(po, &size, &fh);
	if (res)
		return res;
	res = spinor_fs_ops.read(fh, 0, val, NULL, &len);
	spinor_fs_ops.close(&fh);

	return res;
}

/*
 * Put back the flash image from before the last commit, as an attacker
 * with an external programmer would, and check that it does not mount
 * when there is a rollback anchor. Then restore the current image.
 */
TEE_Result spinor_fs_rollback_self_test(void)
{
	char obj_id[] = "spinor-fs-rollback-test";
	struct tee_pobj po = {
		.uuid = { 0x5d2c2d40, 0x3a57, 0x4ab7,
			  { 0x9d, 0x1b, 0x31, 0x5f, 0x7b, 0x26, 0x0b, 0x3c } },
		.obj_id = obj_id,
		.obj_id_len = sizeof(obj_id),
		.fops = &spinor_fs_ops,
	};
	TEE_Result expect = TEE_ERROR_SECURITY;
	TEE_Result res = TEE_ERROR_GENERIC;
	struct spi_nor *nor = NULL;
	uint8_t val = 0;

	/* The test swaps flash images: only on the simulated flash */
	if (spinor_fs_store_get_flash(&nor) || nor != spi_nor_sim_get()) {
		IMSG("SPI-NOR FS rollback self test skipped: no storage");
		return TEE_SUCCESS;
	}

	/* The store is remounted below: no other user may hold it */
	mutex_lock(&spinor_fs_inst.mutex);
	res = spinor_fs_inst.dirh_refcount ? TEE_ERROR_BUSY : TEE_SUCCESS;
	mutex_unlock(&spinor_fs_inst.mutex);
	if (res) {
		IMSG("SPI-NOR FS rollback self test skipped: storage in use");
		return TEE_SUCCESS;
	}

	res = st_write(&po, true, 1);
	if (res)
		goto err;

	spi_nor_sim_snapshot();
	res = st_write(&po, false, 2);
	if (res)
		goto err;

	/* The old image */
	res = spi_nor_sim_swap_snapshot();
	if (res)
		goto err;
	spinor_fs_store_remount();
	res = st_read(&po, &val);
	if (spinor_anchor == SPINOR_ANCHOR_NONE) {
		/* Without an anchor the old image is accepted: the gap */
		expect = TEE_SUCCESS;
	}
	if (res != expect) {
		EMSG("Replayed image: %#"PRIx32", expected %#"PRIx32, res,
		     expect);
		res = TEE_ERROR_GENERIC;
	} else {
		res = TEE_SUCCESS;
	}

	/* The current image */
	if (spi_nor_sim_swap_snapshot())
		panic();
	spinor_fs_store_remount();
	if (res)
		goto err;

	res = st_read(&po, &val);
	if (!res && val != 2)
		res = TEE_ERROR_GENERIC;
	if (res)
		goto err;

	return spinor_fs_ops.remove(&po);
err:
	EMSG("SPI-NOR FS rollback self test failed: %#"PRIx32, res);
	return TEE_ERROR_GENERIC;
}
#endif /*CFG_SPI_NOR_EMBEDDED_TEST*/
#endif /*CFG_SPI_NOR_FS*/
