/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __PTA_SPI_NOR_H
#define __PTA_SPI_NOR_H

#include <stdint.h>

/*
 * Normal-world access to the SPI-NOR partitions OP-TEE exposes, for a
 * kernel driver such as an MTD driver. Only sessions from the REE kernel
 * are accepted. The PTA is enumerated on the TEE bus without
 * tee-supplicant.
 */
#define PTA_SPI_NOR_UUID { 0x80a406d3, 0x2f6e, 0x4202, \
		{ 0xb1, 0x44, 0x15, 0x94, 0x1c, 0x6d, 0x75, 0x90 } }

/* Largest read or write per command */
#define PTA_SPI_NOR_MAX_XFER		4096
/* Largest erase per command */
#define PTA_SPI_NOR_MAX_ERASE		(64 * 1024)

#define PTA_SPI_NOR_ACCESS_RO		1
#define PTA_SPI_NOR_ACCESS_RW		2

#define PTA_SPI_NOR_LABEL_LEN		32

/*
 * struct pta_spi_nor_part - An exposed partition
 * @id: Partition ID used by the other commands
 * @access: PTA_SPI_NOR_ACCESS_RO or PTA_SPI_NOR_ACCESS_RW
 * @size: Size in bytes
 * @erase_size: Erase unit in bytes
 * @page_size: Program page size in bytes
 * @label: Partition name, NUL terminated
 */
struct pta_spi_nor_part {
	uint32_t id;
	uint32_t access;
	uint64_t size;
	uint32_t erase_size;
	uint32_t page_size;
	char label[PTA_SPI_NOR_LABEL_LEN];
};

/*
 * PTA_SPI_NOR_CMD_GET_INFO - List the exposed partitions
 *
 * [out]     memref[0]: Array of struct pta_spi_nor_part; on
 *			TEE_ERROR_SHORT_BUFFER, the size needed
 *
 * Return TEE_SUCCESS or TEE_ERROR_SHORT_BUFFER
 */
#define PTA_SPI_NOR_CMD_GET_INFO	0

/*
 * PTA_SPI_NOR_CMD_READ - Read from a partition
 *
 * [in]      value[0].a: Partition ID
 * [in]      value[0].b: Offset in the partition
 * [out]     memref[1]: Data, at most PTA_SPI_NOR_MAX_XFER bytes
 *
 * Return TEE_SUCCESS on success
 * Return TEE_ERROR_ITEM_NOT_FOUND for an unknown partition
 * Return TEE_ERROR_BAD_PARAMETERS for a range outside the partition
 */
#define PTA_SPI_NOR_CMD_READ		1

/*
 * PTA_SPI_NOR_CMD_WRITE - Program a read-write partition; the range must
 * be erased
 *
 * [in]      value[0].a: Partition ID
 * [in]      value[0].b: Offset in the partition
 * [in]      memref[1]: Data, at most PTA_SPI_NOR_MAX_XFER bytes
 *
 * Return codes as for PTA_SPI_NOR_CMD_READ, and TEE_ERROR_ACCESS_DENIED on
 * a read-only partition
 */
#define PTA_SPI_NOR_CMD_WRITE		2

/*
 * PTA_SPI_NOR_CMD_ERASE - Erase in a read-write partition
 *
 * [in]      value[0].a: Partition ID
 * [in]      value[0].b: Offset in the partition, aligned to the erase unit
 * [in]      value[1].a: Length, a multiple of the erase unit, at most
 *			PTA_SPI_NOR_MAX_ERASE bytes
 *
 * Return codes as for PTA_SPI_NOR_CMD_WRITE
 */
#define PTA_SPI_NOR_CMD_ERASE		3

#endif /* __PTA_SPI_NOR_H */
