/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __TEE_TEE_FS_BACKEND_H
#define __TEE_TEE_FS_BACKEND_H

#include <stddef.h>
#include <tee_api_types.h>
#include <tee/fs_dirfile.h>
#include <tee/fs_htree.h>

/**
 * struct tee_fs_backend_ops - Storage transport of the hash-tree file system
 *
 * The hash-tree file system (core/tee/tee_ree_fs.c) keeps each object in a
 * file made of hash-tree items, and a directory file (dirf.db) mapping
 * objects to file numbers. A backend stores those files. REE FS uses a
 * backend that forwards the requests to tee-supplicant through RPC.
 *
 * Files are identified by their directory file handle, NULL meaning the
 * directory file itself. An open file is identified by an int chosen by the
 * backend.
 *
 * @htree_storage: Hash-tree item read and write for an open file. The
 *	@aux argument of these callbacks points to the int returned by @open
 *	or @create.
 * @open: Open an existing file
 * @create: Create a file, replacing any existing one
 * @close: Close an open file
 * @truncate: Truncate an open file so it keeps hash-tree data blocks
 *	0 to @num_blocks - 1 and the nodes covering them
 * @remove: Remove a file that is not open
 */
struct tee_fs_backend_ops {
	const struct tee_fs_htree_storage *htree_storage;
	TEE_Result (*open)(const struct tee_fs_dirfile_fileh *dfh, int *fd);
	TEE_Result (*create)(const struct tee_fs_dirfile_fileh *dfh, int *fd);
	TEE_Result (*close)(int fd);
	TEE_Result (*truncate)(int fd, size_t num_blocks);
	TEE_Result (*remove)(const struct tee_fs_dirfile_fileh *dfh);
};

#endif /* __TEE_TEE_FS_BACKEND_H */
