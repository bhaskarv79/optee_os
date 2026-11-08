// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * RPMC commands, JESD216 RPMC Parameter Table and the Serial Flash
 * Hardening (RPMC) command set.
 *
 * A command is sent with opcode OP1 and a packet starting with the same
 * four bytes: OP1, command type, counter address, a reserved byte. The
 * flash then reports busy; when it is done, opcode OP2 followed by one
 * dummy byte reads the extended status, and after a counter request the
 * tag, the counter and the signature of the reply.
 *
 * The signatures are HMAC-SHA256 over the packet bytes before the
 * signature: with the root key for Write Root Key (truncated to its last
 * 28 bytes), with the session key otherwise, including Update HMAC Key,
 * which is signed with the new session key HMAC-SHA256(root key, key
 * data). A reply is signed with the session key over the tag and the
 * counter.
 */

#include <crypto/crypto.h>
#include <drivers/spi_mem.h>
#include <drivers/spi_nor.h>
#include <drivers/spi_nor_rpmc.h>
#include <inttypes.h>
#include <io.h>
#include <kernel/delay.h>
#include <kernel/mutex.h>
#include <string.h>
#include <string_ext.h>
#include <tee_api_defines.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "spi_nor_priv.h"

#define RPMC_CMD_WRITE_ROOT_KEY		0x00
#define RPMC_CMD_UPDATE_HMAC_KEY	0x01
#define RPMC_CMD_INCREMENT		0x02
#define RPMC_CMD_REQUEST		0x03

#define RPMC_STATUS_BUSY		BIT(0)
/* Update HMAC Key: the counter has no root key */
#define RPMC_STATUS_NO_ROOT_KEY		BIT(1)
#define RPMC_STATUS_OK			BIT(7)

#define RPMC_HDR_SIZE			4
#define RPMC_SIG_SIZE			32
#define RPMC_TRUNC_SIG_SIZE		28
#define RPMC_TAG_SIZE			12
#define RPMC_CTR_SIZE			4

/* Status, tag, counter and signature read with OP2 */
#define RPMC_REPLY_SIZE	(1 + RPMC_TAG_SIZE + RPMC_CTR_SIZE + RPMC_SIG_SIZE)

#define RPMC_TIMEOUT_US			(5 * 1000 * 1000)

static TEE_Result hmac(const uint8_t key[SPI_NOR_RPMC_KEY_SIZE],
		       const uint8_t *msg, size_t len,
		       uint8_t out[RPMC_SIG_SIZE])
{
	TEE_Result res = TEE_ERROR_GENERIC;
	void *ctx = NULL;

	res = crypto_mac_alloc_ctx(&ctx, TEE_ALG_HMAC_SHA256);
	if (res)
		return res;

	res = crypto_mac_init(ctx, key, SPI_NOR_RPMC_KEY_SIZE);
	if (!res)
		res = crypto_mac_update(ctx, msg, len);
	if (!res)
		res = crypto_mac_final(ctx, out, RPMC_SIG_SIZE);

	crypto_mac_free_ctx(ctx);

	return res;
}

static TEE_Result op2_read(struct spi_nor *nor, uint8_t *buf, size_t len)
{
	struct spi_mem_op op = {
		.cmd = { .nbytes = 1, .buswidth = 1,
			 .opcode = nor->geo.rpmc.op2 },
		.dummy = { .nbytes = 1, .buswidth = 1 },
		.data = { .buswidth = 1, .dir = SPI_MEM_DATA_IN, .nbytes = len,
			  .buf.in = buf },
	};

	return spi_mem_exec_op(nor->mem, &op);
}

static TEE_Result wait_done(struct spi_nor *nor, uint8_t *status)
{
	uint64_t tmo = timeout_init_us(RPMC_TIMEOUT_US);
	TEE_Result res = TEE_ERROR_GENERIC;
	bool last = false;

	if (!nor->geo.rpmc.poll_op2) {
		res = spi_nor_wait_ready(nor, RPMC_TIMEOUT_US);
		if (res)
			return res;
		return op2_read(nor, status, 1);
	}

	while (true) {
		last = timeout_elapsed(tmo);

		res = op2_read(nor, status, 1);
		if (res)
			return res;
		if (!(*status & RPMC_STATUS_BUSY))
			return TEE_SUCCESS;

		if (last) {
			EMSG("RPMC: timeout");
			nor->busy = true;
			return TEE_ERROR_STORAGE_NOT_AVAILABLE;
		}
	}
}

/*
 * Send the packet @pkt (OP1 first) and wait for completion. Called with
 * nor->lock held.
 */
static TEE_Result op1_exec(struct spi_nor *nor, const uint8_t *pkt,
			   size_t len)
{
	struct spi_mem_op op = {
		.cmd = { .nbytes = 1, .buswidth = 1, .opcode = pkt[0] },
		.data = { .buswidth = 1, .dir = SPI_MEM_DATA_OUT,
			  .nbytes = len - 1, .buf.out = pkt + 1 },
	};
	TEE_Result res = TEE_ERROR_GENERIC;
	uint8_t status = 0;

	res = spi_nor_recover(nor);
	if (!res)
		res = spi_mem_exec_op(nor->mem, &op);
	if (!res)
		res = wait_done(nor, &status);
	if (res)
		return res;

	if (status != RPMC_STATUS_OK) {
		DMSG("RPMC: command %#"PRIx8" counter %"PRIu8" status %#"PRIx8,
		     pkt[1], pkt[2], status);
		if (pkt[1] == RPMC_CMD_UPDATE_HMAC_KEY &&
		    status == RPMC_STATUS_NO_ROOT_KEY)
			return TEE_ERROR_ITEM_NOT_FOUND;
		return TEE_ERROR_BAD_STATE;
	}

	return TEE_SUCCESS;
}

static TEE_Result check_ctr(struct spi_nor *nor, unsigned int ctr)
{
	if (!nor->geo.rpmc.num_counters)
		return TEE_ERROR_NOT_SUPPORTED;
	if (ctr >= nor->geo.rpmc.num_counters)
		return TEE_ERROR_BAD_PARAMETERS;

	return TEE_SUCCESS;
}

static void set_hdr(struct spi_nor *nor, uint8_t *pkt, uint8_t cmd,
		    unsigned int ctr)
{
	pkt[0] = nor->geo.rpmc.op1;
	pkt[1] = cmd;
	pkt[2] = ctr;
	pkt[3] = 0;
}

unsigned int spi_nor_rpmc_counters(struct spi_nor *nor)
{
	return nor->geo.rpmc.num_counters;
}

TEE_Result
spi_nor_rpmc_write_root_key(struct spi_nor *nor, unsigned int ctr,
			    const uint8_t root_key[SPI_NOR_RPMC_KEY_SIZE])
{
	uint8_t pkt[RPMC_HDR_SIZE + SPI_NOR_RPMC_KEY_SIZE +
		    RPMC_TRUNC_SIG_SIZE] = { };
	uint8_t sig[RPMC_SIG_SIZE] = { };
	TEE_Result res = TEE_ERROR_GENERIC;

	res = check_ctr(nor, ctr);
	if (res)
		return res;

	set_hdr(nor, pkt, RPMC_CMD_WRITE_ROOT_KEY, ctr);
	res = hmac(root_key, pkt, RPMC_HDR_SIZE, sig);
	if (res)
		goto out;

	memcpy(pkt + RPMC_HDR_SIZE, root_key, SPI_NOR_RPMC_KEY_SIZE);
	memcpy(pkt + RPMC_HDR_SIZE + SPI_NOR_RPMC_KEY_SIZE,
	       sig + RPMC_SIG_SIZE - RPMC_TRUNC_SIG_SIZE, RPMC_TRUNC_SIG_SIZE);

	mutex_lock(&nor->lock);
	res = op1_exec(nor, pkt, sizeof(pkt));
	mutex_unlock(&nor->lock);
out:
	memzero_explicit(pkt, sizeof(pkt));

	return res;
}

TEE_Result
spi_nor_rpmc_update_hmac_key(struct spi_nor *nor, unsigned int ctr,
			     const uint8_t root_key[SPI_NOR_RPMC_KEY_SIZE],
			     uint32_t key_data,
			     uint8_t hmac_key[SPI_NOR_RPMC_KEY_SIZE])
{
	uint8_t pkt[RPMC_HDR_SIZE + 4 + RPMC_SIG_SIZE] = { };
	TEE_Result res = TEE_ERROR_GENERIC;

	res = check_ctr(nor, ctr);
	if (res)
		return res;

	set_hdr(nor, pkt, RPMC_CMD_UPDATE_HMAC_KEY, ctr);
	put_be32(pkt + RPMC_HDR_SIZE, key_data);
	res = hmac(root_key, pkt + RPMC_HDR_SIZE, 4, hmac_key);
	if (!res)
		res = hmac(hmac_key, pkt, RPMC_HDR_SIZE + 4,
			   pkt + RPMC_HDR_SIZE + 4);
	if (res) {
		memzero_explicit(hmac_key, SPI_NOR_RPMC_KEY_SIZE);
		return res;
	}

	mutex_lock(&nor->lock);
	res = op1_exec(nor, pkt, sizeof(pkt));
	mutex_unlock(&nor->lock);

	if (res)
		memzero_explicit(hmac_key, SPI_NOR_RPMC_KEY_SIZE);

	return res;
}

/* Request the counter and read the signed reply. Called with nor->lock */
static TEE_Result read_locked(struct spi_nor *nor, unsigned int ctr,
			      const uint8_t hmac_key[SPI_NOR_RPMC_KEY_SIZE],
			      uint32_t *value)
{
	uint8_t pkt[RPMC_HDR_SIZE + RPMC_TAG_SIZE + RPMC_SIG_SIZE] = { };
	uint8_t reply[RPMC_REPLY_SIZE] = { };
	uint8_t sig[RPMC_SIG_SIZE] = { };
	uint8_t *tag = pkt + RPMC_HDR_SIZE;
	TEE_Result res = TEE_ERROR_GENERIC;

	set_hdr(nor, pkt, RPMC_CMD_REQUEST, ctr);
	res = crypto_rng_read(tag, RPMC_TAG_SIZE);
	if (!res)
		res = hmac(hmac_key, pkt, RPMC_HDR_SIZE + RPMC_TAG_SIZE,
			   pkt + RPMC_HDR_SIZE + RPMC_TAG_SIZE);
	if (!res)
		res = op1_exec(nor, pkt, sizeof(pkt));
	if (!res)
		res = op2_read(nor, reply, sizeof(reply));
	if (res)
		return res;

	res = hmac(hmac_key, reply + 1, RPMC_TAG_SIZE + RPMC_CTR_SIZE, sig);
	if (res)
		return res;

	if (reply[0] != RPMC_STATUS_OK ||
	    consttime_memcmp(reply + 1, tag, RPMC_TAG_SIZE) ||
	    consttime_memcmp(reply + 1 + RPMC_TAG_SIZE + RPMC_CTR_SIZE, sig,
			     RPMC_SIG_SIZE)) {
		EMSG("RPMC: bad reply for counter %u", ctr);
		return TEE_ERROR_SECURITY;
	}

	*value = get_be32(reply + 1 + RPMC_TAG_SIZE);

	return TEE_SUCCESS;
}

TEE_Result spi_nor_rpmc_read(struct spi_nor *nor, unsigned int ctr,
			     const uint8_t hmac_key[SPI_NOR_RPMC_KEY_SIZE],
			     uint32_t *value)
{
	TEE_Result res = TEE_ERROR_GENERIC;

	res = check_ctr(nor, ctr);
	if (res)
		return res;

	mutex_lock(&nor->lock);
	res = read_locked(nor, ctr, hmac_key, value);
	mutex_unlock(&nor->lock);

	return res;
}

TEE_Result spi_nor_rpmc_increment(struct spi_nor *nor, unsigned int ctr,
				  const uint8_t hmac_key[SPI_NOR_RPMC_KEY_SIZE],
				  uint32_t cur)
{
	uint8_t pkt[RPMC_HDR_SIZE + RPMC_CTR_SIZE + RPMC_SIG_SIZE] = { };
	TEE_Result res = TEE_ERROR_GENERIC;
	uint32_t val = 0;

	res = check_ctr(nor, ctr);
	if (res)
		return res;
	if (cur == UINT32_MAX)
		return TEE_ERROR_OVERFLOW;

	set_hdr(nor, pkt, RPMC_CMD_INCREMENT, ctr);
	put_be32(pkt + RPMC_HDR_SIZE, cur);
	res = hmac(hmac_key, pkt, RPMC_HDR_SIZE + RPMC_CTR_SIZE,
		   pkt + RPMC_HDR_SIZE + RPMC_CTR_SIZE);
	if (res)
		return res;

	mutex_lock(&nor->lock);
	res = op1_exec(nor, pkt, sizeof(pkt));
	if (!res)
		res = read_locked(nor, ctr, hmac_key, &val);
	mutex_unlock(&nor->lock);
	if (res)
		return res;

	if (val != cur + 1) {
		EMSG("RPMC: counter %u is %"PRIu32", expected %"PRIu32, ctr,
		     val, cur + 1);
		return TEE_ERROR_SECURITY;
	}

	return TEE_SUCCESS;
}
