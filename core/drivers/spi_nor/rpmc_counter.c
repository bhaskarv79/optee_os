// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * RPMC counters as monotonic counters of OP-TEE.
 *
 * The root key of a counter is derived from the HUK, the JEDEC ID of the
 * flash and the counter index, so it is bound to the device and never
 * stored. The first use of a counter after boot starts a session with a
 * fresh random key data, so session keys and signed replies of an earlier
 * boot are useless. Root keys are only written by builds with
 * CFG_RPMC_WRITE_ROOT_KEY, meant for provisioning.
 */

#include <config.h>
#include <crypto/crypto.h>
#include <drivers/spi_nor.h>
#include <drivers/spi_nor_rpmc.h>
#include <inttypes.h>
#include <kernel/huk_subkey.h>
#include <kernel/mutex.h>
#include <kernel/nv_counter.h>
#include <string.h>
#include <string_ext.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#define MAX_SESSIONS	4

struct rpmc_session {
	struct spi_nor *nor;
	unsigned int ctr;
	uint8_t hmac_key[SPI_NOR_RPMC_KEY_SIZE];
};

static struct rpmc_session sessions[MAX_SESSIONS];
static struct mutex sessions_mu = MUTEX_INITIALIZER;

static TEE_Result derive_root_key(struct spi_nor *nor, unsigned int ctr,
				  uint8_t key[SPI_NOR_RPMC_KEY_SIZE])
{
	uint8_t data[SPI_NOR_MAX_ID_LEN + 1] = { };
	struct spi_nor_info info = { };

	spi_nor_get_info(nor, &info);
	memcpy(data, info.jedec_id, SPI_NOR_MAX_ID_LEN);
	data[SPI_NOR_MAX_ID_LEN] = ctr;

	return huk_subkey_derive(HUK_SUBKEY_RPMC, data, sizeof(data), key,
				 SPI_NOR_RPMC_KEY_SIZE);
}

static TEE_Result open_session(struct rpmc_session *s)
{
	uint8_t root[SPI_NOR_RPMC_KEY_SIZE] = { };
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t key_data = 0;

	res = derive_root_key(s->nor, s->ctr, root);
	if (!res)
		res = crypto_rng_read(&key_data, sizeof(key_data));
	if (res)
		goto out;

	res = spi_nor_rpmc_update_hmac_key(s->nor, s->ctr, root, key_data,
					   s->hmac_key);
	/*
	 * The root key goes in clear on the bus: write it only when the
	 * flash says the counter has none, never on another error such as
	 * a signature mismatch.
	 */
	if (res == TEE_ERROR_ITEM_NOT_FOUND &&
	    IS_ENABLED(CFG_RPMC_WRITE_ROOT_KEY)) {
		IMSG("RPMC: writing the root key of counter %u", s->ctr);
		res = spi_nor_rpmc_write_root_key(s->nor, s->ctr, root);
		if (!res)
			res = spi_nor_rpmc_update_hmac_key(s->nor, s->ctr,
							   root, key_data,
							   s->hmac_key);
	}
	if (res == TEE_ERROR_ITEM_NOT_FOUND || res == TEE_ERROR_BAD_STATE) {
		EMSG("RPMC: counter %u not provisioned for this device",
		     s->ctr);
		res = TEE_ERROR_BAD_STATE;
	}
out:
	memzero_explicit(root, sizeof(root));

	return res;
}

/*
 * Read the counter of session @s. The flash loses its HMAC key when it is
 * reset or power cycled: if it rejects the request, start a new session
 * and try once more. Called with sessions_mu held.
 */
static TEE_Result read_counter(struct rpmc_session *s, uint32_t *value)
{
	TEE_Result res = spi_nor_rpmc_read(s->nor, s->ctr, s->hmac_key, value);

	if (res == TEE_ERROR_BAD_STATE && !open_session(s))
		res = spi_nor_rpmc_read(s->nor, s->ctr, s->hmac_key, value);

	return res;
}

/* Called with sessions_mu held */
static TEE_Result get_session(struct spi_nor *nor, unsigned int ctr,
			      struct rpmc_session **out)
{
	struct rpmc_session *free_slot = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	unsigned int n = 0;

	if (ctr >= spi_nor_rpmc_counters(nor))
		return TEE_ERROR_NOT_SUPPORTED;

	for (n = 0; n < ARRAY_SIZE(sessions); n++) {
		if (sessions[n].nor == nor && sessions[n].ctr == ctr) {
			*out = sessions + n;
			return TEE_SUCCESS;
		}
		if (!sessions[n].nor && !free_slot)
			free_slot = sessions + n;
	}

	if (!free_slot)
		return TEE_ERROR_OUT_OF_MEMORY;

	free_slot->nor = nor;
	free_slot->ctr = ctr;
	res = open_session(free_slot);
	if (res) {
		memzero_explicit(free_slot, sizeof(*free_slot));
		return res;
	}

	*out = free_slot;

	return TEE_SUCCESS;
}

TEE_Result spi_nor_rpmc_counter_get(struct spi_nor *nor, unsigned int ctr,
				    uint32_t *value)
{
	struct rpmc_session *s = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;

	mutex_lock(&sessions_mu);
	res = get_session(nor, ctr, &s);
	if (!res)
		res = read_counter(s, value);
	mutex_unlock(&sessions_mu);

	return res;
}

TEE_Result spi_nor_rpmc_counter_incr_to(struct spi_nor *nor, unsigned int ctr,
					uint32_t value)
{
	struct rpmc_session *s = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t cur = 0;

	mutex_lock(&sessions_mu);
	res = get_session(nor, ctr, &s);
	if (!res)
		res = read_counter(s, &cur);
	while (!res && cur < value) {
		res = spi_nor_rpmc_increment(nor, ctr, s->hmac_key, cur);
		if (res == TEE_ERROR_BAD_STATE) {
			res = read_counter(s, &cur);
			if (!res && cur < value)
				res = spi_nor_rpmc_increment(nor, ctr,
							     s->hmac_key, cur);
		}
		cur++;
	}
	mutex_unlock(&sessions_mu);

	return res;
}

#ifdef CFG_REE_FS_INTEGRITY_RPMC
/* REE FS monotonic counter on the first flash with enough RPMC counters */
static TEE_Result ree_fs_flash(struct spi_nor **nor)
{
	unsigned int n = 0;

	while (!spi_nor_get(n++, nor))
		if (spi_nor_rpmc_counters(*nor) > CFG_REE_FS_RPMC_COUNTER)
			return TEE_SUCCESS;

	EMSG("RPMC: no flash with counter %d for REE FS",
	     CFG_REE_FS_RPMC_COUNTER);

	return TEE_ERROR_STORAGE_NOT_AVAILABLE;
}

TEE_Result nv_counter_get_ree_fs(uint32_t *value)
{
	struct spi_nor *nor = NULL;
	TEE_Result res = ree_fs_flash(&nor);

	if (res)
		return res;

	return spi_nor_rpmc_counter_get(nor, CFG_REE_FS_RPMC_COUNTER, value);
}

TEE_Result nv_counter_incr_ree_fs_to(uint32_t value)
{
	struct spi_nor *nor = NULL;
	TEE_Result res = ree_fs_flash(&nor);

	if (res)
		return res;

	return spi_nor_rpmc_counter_incr_to(nor, CFG_REE_FS_RPMC_COUNTER,
					    value);
}
#endif /*CFG_REE_FS_INTEGRITY_RPMC*/
