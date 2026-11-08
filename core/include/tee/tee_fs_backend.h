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

#ifdef CFG_SPI_NOR_FS
struct spi_nor;

/* SPI-NOR flash item store, core/tee/spinor_fs_store.c */
extern const struct tee_fs_backend_ops spinor_fs_backend;

/* spinor_fs_store_get_flash() - Mount the store and get its flash */
TEE_Result spinor_fs_store_get_flash(struct spi_nor **nor);

/*
 * spinor_fs_store_dt_anchor() - Value of the "optee,rollback-anchor"
 *	property of the store partition in the secure DT, or NULL
 */
const char *spinor_fs_store_dt_anchor(void);
#endif

#if defined(CFG_SPI_NOR_FS) && defined(CFG_SPI_NOR_EMBEDDED_TEST)
/* Store format, remount, torn-record and fill tests on the simulator */
TEE_Result spinor_fs_store_self_test(void);

/* Drop the store index so the next access mounts the flash again */
void spinor_fs_store_remount(void);

/* Replay of an old flash image against the rollback anchor */
TEE_Result spinor_fs_rollback_self_test(void);
#else
static inline TEE_Result spinor_fs_store_self_test(void)
{
	return TEE_SUCCESS;
}

static inline TEE_Result spinor_fs_rollback_self_test(void)
{
	return TEE_SUCCESS;
}
#endif

#endif /* __TEE_TEE_FS_BACKEND_H */
