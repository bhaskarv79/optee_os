/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __SPI_NOR_PRIV_H
#define __SPI_NOR_PRIV_H

#include <drivers/spi_nor.h>
#include <kernel/mutex.h>
#include <stdint.h>
#include <sys/queue.h>
#include <tee_api_types.h>

/* JEDEC SPI-NOR opcodes used by the core */
#define SPI_NOR_OP_WREN		0x06	/* Write enable */
#define SPI_NOR_OP_RDSR		0x05	/* Read status register */
#define SPI_NOR_OP_RDID		0x9f	/* Read JEDEC ID */
#define SPI_NOR_OP_RDSFDP	0x5a	/* Read SFDP */
#define SPI_NOR_OP_READ		0x03	/* Read data, no dummy cycles */
#define SPI_NOR_OP_PP		0x02	/* Page program */
#define SPI_NOR_OP_EN4B		0xb7	/* Enter 4-byte address mode */

#define SPI_NOR_SR_WIP		BIT(0)	/* Write in progress */
#define SPI_NOR_SR_WEL		BIT(1)	/* Write enable latch */

/* Flashes above this size need 4-byte addresses */
#define SPI_NOR_3B_ADDR_MAX	(16 * 1024 * 1024)

/**
 * struct spi_nor_erase_type - One erase command
 *
 * @size: Erase unit in bytes, a power of two; 0 if the slot is unused
 * @opcode: Erase opcode
 */
struct spi_nor_erase_type {
	uint32_t size;
	uint8_t opcode;
};

/**
 * struct spi_nor_rpmc_params - RPMC parameters from SFDP (JESD216)
 *
 * @num_counters: Number of monotonic counters, 0 if the flash has no RPMC
 * @op1: Opcode of the RPMC commands, commonly 0x9b
 * @op2: Opcode reading the RPMC extended status and data, commonly 0x96
 * @poll_op2: Poll the busy bit of the extended status, not of the status
 *	register
 */
struct spi_nor_rpmc_params {
	uint8_t num_counters;
	uint8_t op1;
	uint8_t op2;
	bool poll_op2;
};

/**
 * struct spi_nor_geometry - What probing learns about the flash
 *
 * @size: Flash size in bytes
 * @page_size: Program page size in bytes
 * @erase: Erase types, sorted by increasing size, unused slots at the end
 * @addr4_capable: The flash supports 4-byte addressing
 * @rpmc: RPMC parameters, if any
 */
struct spi_nor_geometry {
	uint64_t size;
	uint32_t page_size;
	struct spi_nor_erase_type erase[SPI_NOR_MAX_ERASE_TYPES];
	bool addr4_capable;
	struct spi_nor_rpmc_params rpmc;
};

/**
 * struct spi_nor - SPI-NOR device
 *
 * @mem: SPI memory device the flash is connected to
 * @info: Geometry reported to users
 * @geo: Geometry found at probe, including erase opcodes
 * @addr_nbytes: Address bytes sent with read, program and erase, 3 or 4
 * @busy: An operation timed out; the flash may still be busy and ignore
 *	commands until it is done
 * @lock: Serializes the command sequences of read, program and erase
 * @link: Entry in the list of probed devices
 */
struct spi_nor {
	struct spi_mem *mem;
	struct spi_nor_info info;
	struct spi_nor_geometry geo;
	uint8_t addr_nbytes;
	bool busy;
	struct mutex lock; /* Serializes command sequences on this device */
	STAILQ_ENTRY(spi_nor) link;
};

/*
 * spi_nor_wait_ready() - Poll the status register until the flash is not
 *	busy; called with nor->lock held
 *
 * Return TEE_ERROR_STORAGE_NOT_AVAILABLE on timeout
 */
TEE_Result spi_nor_wait_ready(struct spi_nor *nor, uint32_t timeout_us);

/*
 * spi_nor_recover() - If an operation timed out, wait until the flash is
 *	done before sending it a new command; called with nor->lock held
 */
TEE_Result spi_nor_recover(struct spi_nor *nor);

/*
 * sfdp_read_func - Read @len bytes of the SFDP space at @addr
 */
typedef TEE_Result (*sfdp_read_func)(void *ctx, uint32_t addr, void *buf,
				     size_t len);

/*
 * spi_nor_sfdp_parse() - Parse the SFDP header, the Basic Flash Parameter
 *	Table and, if present, the RPMC Parameter Table (JESD216)
 *
 * Return TEE_SUCCESS and fill @geo on success
 * Return TEE_ERROR_ITEM_NOT_FOUND if there is no valid SFDP signature
 * Return TEE_ERROR_BAD_FORMAT if the tables are inconsistent
 */
TEE_Result spi_nor_sfdp_parse(sfdp_read_func read, void *ctx,
			      struct spi_nor_geometry *geo);

/*
 * spi_nor_ids_lookup() - Geometry for a flash without usable SFDP
 *
 * Return TEE_SUCCESS and fill @geo if @id is in the table,
 *	TEE_ERROR_ITEM_NOT_FOUND otherwise
 */
TEE_Result spi_nor_ids_lookup(const uint8_t id[SPI_NOR_MAX_ID_LEN],
			      struct spi_nor_geometry *geo);

/*
 * spi_nor_dt_add_partitions() - Add the partitions of the "partitions"
 *	subnode of flash node @node; none are kept if one is invalid
 */
TEE_Result spi_nor_dt_add_partitions(struct spi_nor *nor, const void *fdt,
				     int node);

#endif /* __SPI_NOR_PRIV_H */
