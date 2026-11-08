/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __DRIVERS_SPI_NOR_RPMC_H
#define __DRIVERS_SPI_NOR_RPMC_H

#include <drivers/spi_nor.h>
#include <stdint.h>
#include <tee_api_types.h>

/*
 * Replay Protected Monotonic Counters (RPMC) of a SPI-NOR flash
 *
 * An RPMC flash has a few 32-bit counters that can only increase. Each
 * counter has a root key, written once. A session HMAC key is derived in
 * the flash as HMAC-SHA256(root key, key data). Commands are signed with
 * HMAC-SHA256 and counter reads carry a caller-chosen tag and a signature,
 * so the SPI bus does not need to be trusted. The counters protect
 * themselves only, not the flash array.
 *
 * Callers own the keys; these functions keep no key material.
 */

#define SPI_NOR_RPMC_KEY_SIZE		32

#ifdef CFG_SPI_NOR_RPMC
/**
 * spi_nor_rpmc_counters() - Number of RPMC counters of a flash
 *
 * @nor: SPI-NOR device
 *
 * Return the number of counters, 0 if the flash has no RPMC
 */
unsigned int spi_nor_rpmc_counters(struct spi_nor *nor);

/**
 * spi_nor_rpmc_write_root_key() - Provision the root key of a counter
 *
 * @nor: SPI-NOR device
 * @ctr: Counter index
 * @root_key: Root key
 *
 * The root key can only be written once per counter.
 *
 * Return TEE_SUCCESS on success
 * Return TEE_ERROR_BAD_STATE if the flash rejects the command, for instance
 *	because the root key is already written
 * Return another TEE_Result compliant code on error
 */
TEE_Result
spi_nor_rpmc_write_root_key(struct spi_nor *nor, unsigned int ctr,
			    const uint8_t root_key[SPI_NOR_RPMC_KEY_SIZE]);

/**
 * spi_nor_rpmc_update_hmac_key() - Start a session on a counter
 *
 * @nor: SPI-NOR device
 * @ctr: Counter index
 * @root_key: Root key of the counter
 * @key_data: Fresh value the session key is derived from
 * @hmac_key: Output session key, HMAC-SHA256(@root_key, @key_data)
 *
 * Return TEE_SUCCESS on success
 * Return TEE_ERROR_ITEM_NOT_FOUND if the counter has no root key
 * Return TEE_ERROR_BAD_STATE if the flash rejects the command, for instance
 *	because the root key differs
 * Return another TEE_Result compliant code on error
 */
TEE_Result
spi_nor_rpmc_update_hmac_key(struct spi_nor *nor, unsigned int ctr,
			     const uint8_t root_key[SPI_NOR_RPMC_KEY_SIZE],
			     uint32_t key_data,
			     uint8_t hmac_key[SPI_NOR_RPMC_KEY_SIZE]);
#else
static inline unsigned int spi_nor_rpmc_counters(struct spi_nor *nor __unused)
{
	return 0;
}
#endif /* CFG_SPI_NOR_RPMC */

#endif /* __DRIVERS_SPI_NOR_RPMC_H */
