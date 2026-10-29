/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __DRIVERS_SPI_NOR_H
#define __DRIVERS_SPI_NOR_H

#include <drivers/spi_mem.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <tee_api_types.h>

/*
 * SPI-NOR flash core
 *
 * The core speaks JEDEC SPI-NOR commands over the spi-mem interface. It
 * discovers the flash geometry from the JEDEC SFDP tables, falling back to
 * a small JEDEC ID table for parts without SFDP. Devices are created with
 * spi_nor_probe() on a struct spi_mem.
 *
 * All operations are polled and serialized by a per-device mutex, so they
 * must be called from thread context. Program and erase do not erase or
 * merge implicitly: a program can only clear bits and an erase sets a whole
 * erase unit to 0xff, as on the flash itself.
 */

#define SPI_NOR_MAX_ID_LEN		6
#define SPI_NOR_MAX_ERASE_TYPES		4

/**
 * struct spi_nor_info - SPI-NOR flash geometry
 *
 * @jedec_id: Manufacturer and device ID bytes from READ ID (0x9f)
 * @size: Flash size in bytes
 * @page_size: Program page size in bytes, usually 256
 * @erase_size: Smallest erase unit in bytes, usually 4096
 * @has_sfdp: True if the geometry comes from SFDP, false if from the ID table
 */
struct spi_nor_info {
	uint8_t jedec_id[SPI_NOR_MAX_ID_LEN];
	uint64_t size;
	size_t page_size;
	size_t erase_size;
	bool has_sfdp;
};

struct spi_nor;

#ifdef CFG_DRIVERS_SPI_NOR
/**
 * spi_nor_probe() - Identify a SPI-NOR flash and create its device
 *
 * @mem: SPI memory device the flash is connected to
 * @out_nor: Output device upon success
 *
 * The device is added to the list returned by spi_nor_get().
 *
 * Return TEE_SUCCESS on success
 * Return TEE_ERROR_NOT_SUPPORTED if the flash cannot be identified
 * Return another TEE_Result compliant code on error
 */
TEE_Result spi_nor_probe(struct spi_mem *mem, struct spi_nor **out_nor);

/**
 * spi_nor_get() - Get a probed SPI-NOR device
 *
 * @index: Index in probe order, starting at 0
 * @out_nor: Output device upon success
 *
 * Return TEE_SUCCESS or TEE_ERROR_ITEM_NOT_FOUND
 */
TEE_Result spi_nor_get(unsigned int index, struct spi_nor **out_nor);

/**
 * spi_nor_get_info() - Get the flash geometry
 *
 * @nor: SPI-NOR device
 * @info: Output geometry
 */
void spi_nor_get_info(struct spi_nor *nor, struct spi_nor_info *info);

/**
 * spi_nor_read() - Read from the flash
 *
 * @nor: SPI-NOR device
 * @offs: Byte offset in the flash
 * @buf: Destination buffer
 * @len: Number of bytes to read
 *
 * Return a TEE_Result compliant code
 */
TEE_Result spi_nor_read(struct spi_nor *nor, uint64_t offs, void *buf,
			size_t len);

/**
 * spi_nor_write() - Program the flash and verify the result
 *
 * @nor: SPI-NOR device
 * @offs: Byte offset in the flash, any alignment
 * @buf: Data to program
 * @len: Number of bytes
 *
 * The range must have been erased; programming can only clear bits.
 *
 * Return TEE_SUCCESS on success
 * Return TEE_ERROR_STORAGE_NOT_AVAILABLE if the flash times out or the data
 *	read back differs from @buf
 * Return another TEE_Result compliant code on error
 */
TEE_Result spi_nor_write(struct spi_nor *nor, uint64_t offs, const void *buf,
			 size_t len);

/**
 * spi_nor_erase() - Erase whole erase units
 *
 * @nor: SPI-NOR device
 * @offs: Byte offset, aligned to the smallest erase unit
 * @len: Number of bytes, a multiple of the smallest erase unit
 *
 * Return TEE_SUCCESS on success
 * Return TEE_ERROR_BAD_PARAMETERS on misalignment or out-of-range
 * Return TEE_ERROR_STORAGE_NOT_AVAILABLE if the flash times out; the
 *	erased range is not read back
 */
TEE_Result spi_nor_erase(struct spi_nor *nor, uint64_t offs, size_t len);
#else
static inline TEE_Result spi_nor_probe(struct spi_mem *mem __unused,
				       struct spi_nor **out_nor __unused)
{
	return TEE_ERROR_NOT_SUPPORTED;
}

static inline TEE_Result spi_nor_get(unsigned int index __unused,
				     struct spi_nor **out_nor __unused)
{
	return TEE_ERROR_ITEM_NOT_FOUND;
}
#endif /* CFG_DRIVERS_SPI_NOR */

#ifdef CFG_SPI_NOR_SIM
/*
 * spi_nor_sim_get() - The simulated flash, or NULL before it is probed.
 *	Tests that erase a flash must only use this one.
 */
struct spi_nor *spi_nor_sim_get(void);
#endif

#ifdef CFG_SPI_NOR_EMBEDDED_TEST
/*
 * spi_nor_self_test() - SFDP parser and NOR semantics tests on the simulated
 *	flash; erases and programs it
 */
TEE_Result spi_nor_self_test(void);
#else
static inline TEE_Result spi_nor_self_test(void)
{
	return TEE_SUCCESS;
}
#endif

#endif /* __DRIVERS_SPI_NOR_H */
