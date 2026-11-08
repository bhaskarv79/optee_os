/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __DRIVERS_SPI_NOR_NS_H
#define __DRIVERS_SPI_NOR_NS_H

#include <drivers/spi_nor.h>
#include <stddef.h>
#include <stdint.h>
#include <tee_api_types.h>

/*
 * Non-secure access to SPI-NOR partitions
 *
 * The flash belongs to the secure world. This layer lets normal-world
 * requests, received by a front-end such as a pseudo TA, read, program and
 * erase the partitions whose "optee,ns-access" is "ro" or "rw", and
 * nothing else. Exposed partitions are numbered from 0 in partition order.
 *
 * Every request is checked against the partition bounds and access, and
 * its data goes through a secure buffer, so the flash never sees a
 * normal-world buffer and the data cannot change between check and use.
 * Requests are bounded so that none holds the flash for long.
 */

/* Largest read or program per request */
#define SPI_NOR_NS_MAX_XFER		4096
/* Largest erase per request */
#define SPI_NOR_NS_MAX_ERASE		(64 * 1024)

/**
 * struct spi_nor_ns_info - An exposed partition
 *
 * @label: Partition name
 * @access: SPI_NOR_NS_ACCESS_RO or SPI_NOR_NS_ACCESS_RW
 * @size: Size in bytes
 * @erase_size: Erase unit in bytes
 * @page_size: Program page size in bytes
 */
struct spi_nor_ns_info {
	char label[SPI_NOR_PART_LABEL_LEN];
	enum spi_nor_ns_access access;
	uint64_t size;
	size_t erase_size;
	size_t page_size;
};

/* spi_nor_ns_count() - Number of exposed partitions */
unsigned int spi_nor_ns_count(void);

/**
 * spi_nor_ns_get_info() - Describe an exposed partition
 *
 * @id: Exposed partition number
 * @info: Output description
 *
 * Return TEE_SUCCESS or TEE_ERROR_ITEM_NOT_FOUND
 */
TEE_Result spi_nor_ns_get_info(unsigned int id, struct spi_nor_ns_info *info);

/**
 * spi_nor_ns_read() - Read from an exposed partition
 *
 * @id: Exposed partition number
 * @offs: Offset in the partition
 * @buf: Destination, may be normal-world shared memory
 * @len: At most SPI_NOR_NS_MAX_XFER bytes
 *
 * Return TEE_SUCCESS on success
 * Return TEE_ERROR_ITEM_NOT_FOUND for an unknown partition
 * Return TEE_ERROR_BAD_PARAMETERS if the range is outside the partition or
 *	too long
 * Return another TEE_Result compliant code on error
 */
TEE_Result spi_nor_ns_read(unsigned int id, uint64_t offs, void *buf,
			   size_t len);

/**
 * spi_nor_ns_write() - Program an exposed read-write partition
 *
 * @id: Exposed partition number
 * @offs: Offset in the partition
 * @buf: Data, may be normal-world shared memory
 * @len: At most SPI_NOR_NS_MAX_XFER bytes
 *
 * Return codes as for spi_nor_ns_read(), and TEE_ERROR_ACCESS_DENIED if
 * the partition is read-only
 */
TEE_Result spi_nor_ns_write(unsigned int id, uint64_t offs, const void *buf,
			    size_t len);

/**
 * spi_nor_ns_erase() - Erase in an exposed read-write partition
 *
 * @id: Exposed partition number
 * @offs: Offset in the partition, aligned to the erase unit
 * @len: Multiple of the erase unit, at most SPI_NOR_NS_MAX_ERASE bytes
 *
 * Return codes as for spi_nor_ns_write()
 */
TEE_Result spi_nor_ns_erase(unsigned int id, uint64_t offs, size_t len);

#endif /* __DRIVERS_SPI_NOR_NS_H */
