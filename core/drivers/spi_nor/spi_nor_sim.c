// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * SPI-NOR flash simulator, for testing only (CFG_SPI_NOR_SIM).
 *
 * A spi-mem controller that interprets SPI-NOR commands against an array
 * in secure RAM, with NOR semantics: a page program only clears bits and
 * wraps inside its page, an erase sets a whole unit to 0xff, both need a
 * prior write enable and report busy for a few status reads or 100 us,
 * whichever ends first, so a caller preempted for longer does not see a
 * flash that is still busy long after a real one would be done. It publishes
 * a JESD216B SFDP table so the core probes it like a real flash.
 *
 * With CFG_SPI_NOR_RPMC it also has four RPMC counters, kept in secure RAM
 * like the array, so they start unprovisioned at every boot.
 */

#include <assert.h>
#include <config.h>
#include <crypto/crypto.h>
#include <drivers/spi_mem.h>
#include <drivers/spi_nor.h>
#include <initcall.h>
#include <io.h>
#include <kernel/delay.h>
#include <string.h>
#include <tee_api_defines.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "spi_nor_priv.h"

#define SIM_SIZE		CFG_SPI_NOR_SIM_SIZE
#define SIM_PAGE_SIZE		256
#define SIM_SECTOR_SIZE		0x1000
#define SIM_BLOCK_SIZE		0x10000
#define SIM_OP_SE		0x20	/* 4 KiB sector erase */
#define SIM_OP_BE		0xd8	/* 64 KiB block erase */
#define SIM_BUSY_POLLS		2
#define SIM_BUSY_US		100

static_assert(IS_POWER_OF_TWO(SIM_SIZE) && SIM_SIZE >= SIM_BLOCK_SIZE &&
	      SIM_SIZE <= SPI_NOR_3B_ADDR_MAX);

/* Not a real manufacturer: keeps the simulator out of the ID table */
static const uint8_t sim_jedec_id[SPI_NOR_MAX_ID_LEN] = {
	0x00, 0x5a, 0x1e, 0x00, 0x00, 0x00
};

#define SIM_SFDP_BFPT_OFFS	0x20
#define SIM_SFDP_BFPT_DWORDS	16
#define SIM_SFDP_RPMC_OFFS	(SIM_SFDP_BFPT_OFFS + SIM_SFDP_BFPT_DWORDS * 4)
#define SIM_SFDP_RPMC_DWORDS	2

#define SIM_RPMC_OP1		0x9b
#define SIM_RPMC_OP2		0x96
#define SIM_RPMC_COUNTERS	4
#define SIM_RPMC_KEY_SIZE	32
#define SIM_RPMC_TAG_SIZE	12

/* RPMC extended status */
#define SIM_RPMC_BUSY		BIT(0)
#define SIM_RPMC_ERR_KEY	BIT(1)	/* Root key overwrite or missing */
#define SIM_RPMC_ERR_SIG	BIT(2)	/* Bad signature, counter or size */
#define SIM_RPMC_ERR_NO_HMAC	BIT(3)	/* No session key */
#define SIM_RPMC_ERR_CTR	BIT(4)	/* Counter data mismatch */
#define SIM_RPMC_OK		BIT(7)

struct sim_rpmc_ctr {
	bool root_set;
	bool hmac_set;
	uint8_t root[SIM_RPMC_KEY_SIZE];
	uint8_t hmac[SIM_RPMC_KEY_SIZE];
	uint32_t value;
};

/* Status, tag, counter, signature */
#define SIM_RPMC_REPLY_SIZE	(1 + SIM_RPMC_TAG_SIZE + 4 + 32)

struct sim_rpmc {
	struct sim_rpmc_ctr ctr[SIM_RPMC_COUNTERS];
	uint8_t reply[SIM_RPMC_REPLY_SIZE];
	uint8_t prev_reply[SIM_RPMC_REPLY_SIZE];
	unsigned int busy;
	enum spi_nor_sim_rpmc_fault fault;
};

struct spi_nor_sim {
	struct spi_mem_ctrl ctrl;
	struct spi_mem mem;
	uint8_t sfdp[SIM_SFDP_RPMC_OFFS + SIM_SFDP_RPMC_DWORDS * 4];
	struct sim_rpmc rpmc;
	bool wel;
	unsigned int busy;
	uint64_t busy_end;
	uint8_t array[SIM_SIZE];
};

static struct spi_nor_sim sim;

/*
 * Copy-on-write snapshot: while recording, the first program or erase of
 * a sector saves its content first.
 */
#define SIM_SNAP_SECTORS	16

static struct {
	bool recording;
	bool overflow;
	unsigned int count;
	uint32_t addr[SIM_SNAP_SECTORS];
	uint8_t data[SIM_SNAP_SECTORS][SIM_SECTOR_SIZE];
} snap;

static void sim_snap_save(uint64_t addr, size_t len)
{
	uint64_t sec = ROUNDDOWN(addr, SIM_SECTOR_SIZE);
	unsigned int n = 0;

	if (!snap.recording)
		return;

	for (; sec < addr + len; sec += SIM_SECTOR_SIZE) {
		for (n = 0; n < snap.count; n++)
			if (snap.addr[n] == sec)
				break;
		if (n < snap.count)
			continue;
		if (snap.count == SIM_SNAP_SECTORS) {
			snap.overflow = true;
			return;
		}
		snap.addr[snap.count] = sec;
		memcpy(snap.data[snap.count], sim.array + sec,
		       SIM_SECTOR_SIZE);
		snap.count++;
	}
}

static void build_sfdp(uint8_t *sfdp)
{
	uint8_t *bfpt = sfdp + SIM_SFDP_BFPT_OFFS;

	/* SFDP header: signature, revision 1.6, one parameter header */
	put_le32(sfdp, 0x50444653);
	sfdp[4] = 6;
	sfdp[5] = 1;
	sfdp[6] = IS_ENABLED(CFG_SPI_NOR_RPMC) ? 1 : 0;
	sfdp[7] = 0xff;

	/* BFPT parameter header: ID 0xff00, revision 1.6, 16 dwords */
	sfdp[8] = 0x00;
	sfdp[9] = 6;
	sfdp[10] = 1;
	sfdp[11] = SIM_SFDP_BFPT_DWORDS;
	sfdp[12] = SIM_SFDP_BFPT_OFFS;
	sfdp[13] = 0;
	sfdp[14] = 0;
	sfdp[15] = 0xff;

	/* Dword 1: 4 KiB erase with SIM_OP_SE, 3-byte addresses only */
	put_le32(bfpt, 0x1 | (SIM_OP_SE << 8));
	/* Dword 2: density in bits minus one */
	put_le32(bfpt + 4, SIM_SIZE * 8 - 1);
	/* Dword 8: erase types 1 and 2 as size exponent and opcode */
	put_le32(bfpt + 7 * 4, 12 | (SIM_OP_SE << 8) | (16 << 16) |
			       (SIM_OP_BE << 24));
	/* Dword 11: page size exponent in bits 7:4 */
	put_le32(bfpt + 10 * 4, 8 << 4);

	if (!IS_ENABLED(CFG_SPI_NOR_RPMC))
		return;

	/* RPMC parameter header: ID 0xff03, revision 1.0, 2 dwords */
	sfdp[16] = 0x03;
	sfdp[17] = 0;
	sfdp[18] = 1;
	sfdp[19] = SIM_SFDP_RPMC_DWORDS;
	sfdp[20] = SIM_SFDP_RPMC_OFFS;
	sfdp[21] = 0;
	sfdp[22] = 0;
	sfdp[23] = 0xff;

	/*
	 * Dword 1: flash hardening, 32-bit counters, busy polled with OP2,
	 * bit 3 reserved (1), the counter count minus one, OP1 and OP2
	 */
	put_le32(sfdp + SIM_SFDP_RPMC_OFFS,
		 BIT(3) | ((SIM_RPMC_COUNTERS - 1) << 4) |
		 (SIM_RPMC_OP1 << 8) | (SIM_RPMC_OP2 << 16));
}

static bool sim_hmac(const uint8_t *key, const uint8_t *msg, size_t len,
		     uint8_t out[32])
{
	void *ctx = NULL;
	bool ok = false;

	if (crypto_mac_alloc_ctx(&ctx, TEE_ALG_HMAC_SHA256))
		return false;

	ok = !crypto_mac_init(ctx, key, SIM_RPMC_KEY_SIZE) &&
	     !crypto_mac_update(ctx, msg, len) &&
	     !crypto_mac_final(ctx, out, 32);
	crypto_mac_free_ctx(ctx);

	return ok;
}

/* True if the 32-byte signature at @sig matches HMAC(@key, @msg) */
static bool sim_sig_ok(const uint8_t *key, const uint8_t *msg, size_t len,
		       const uint8_t *sig, size_t sig_len)
{
	uint8_t mac[32] = { };

	return sim_hmac(key, msg, len, mac) &&
	       !memcmp(mac + sizeof(mac) - sig_len, sig, sig_len);
}

/* Interpret an RPMC packet; @pkt starts with OP1 */
static uint8_t sim_rpmc_cmd(const uint8_t *pkt, size_t len)
{
	uint8_t key[SIM_RPMC_KEY_SIZE] = { };
	struct sim_rpmc_ctr *c = NULL;
	uint8_t *r = sim.rpmc.reply;

	if (len < 4 || pkt[2] >= SIM_RPMC_COUNTERS)
		return SIM_RPMC_ERR_SIG;
	c = sim.rpmc.ctr + pkt[2];

	switch (pkt[1]) {
	case 0x00:	/* Write root key */
		if (len != 4 + 32 + 28)
			return SIM_RPMC_ERR_SIG;
		if (c->root_set)
			return SIM_RPMC_ERR_KEY;
		if (!sim_sig_ok(pkt + 4, pkt, 4, pkt + 36, 28))
			return SIM_RPMC_ERR_SIG;
		memcpy(c->root, pkt + 4, SIM_RPMC_KEY_SIZE);
		c->root_set = true;
		return SIM_RPMC_OK;
	case 0x01:	/* Update HMAC key */
		if (len != 4 + 4 + 32)
			return SIM_RPMC_ERR_SIG;
		if (!c->root_set)
			return SIM_RPMC_ERR_KEY;
		/* Signed with the new session key */
		if (!sim_hmac(c->root, pkt + 4, 4, key) ||
		    !sim_sig_ok(key, pkt, 8, pkt + 8, 32))
			return SIM_RPMC_ERR_SIG;
		memcpy(c->hmac, key, sizeof(c->hmac));
		c->hmac_set = true;
		return SIM_RPMC_OK;
	case 0x02:	/* Increment counter */
		if (len != 4 + 4 + 32)
			return SIM_RPMC_ERR_SIG;
		if (!c->hmac_set)
			return SIM_RPMC_ERR_NO_HMAC;
		if (!sim_sig_ok(c->hmac, pkt, 8, pkt + 8, 32))
			return SIM_RPMC_ERR_SIG;
		if (get_be32(pkt + 4) != c->value)
			return SIM_RPMC_ERR_CTR;
		c->value++;
		return SIM_RPMC_OK;
	case 0x03:	/* Request counter */
		if (len != 4 + SIM_RPMC_TAG_SIZE + 32)
			return SIM_RPMC_ERR_SIG;
		if (!c->hmac_set)
			return SIM_RPMC_ERR_NO_HMAC;
		if (!sim_sig_ok(c->hmac, pkt, 4 + SIM_RPMC_TAG_SIZE,
				pkt + 4 + SIM_RPMC_TAG_SIZE, 32))
			return SIM_RPMC_ERR_SIG;
		memcpy(sim.rpmc.prev_reply, r, sizeof(sim.rpmc.prev_reply));
		memcpy(r + 1, pkt + 4, SIM_RPMC_TAG_SIZE);
		put_be32(r + 1 + SIM_RPMC_TAG_SIZE, c->value);
		if (!sim_hmac(c->hmac, r + 1, SIM_RPMC_TAG_SIZE + 4,
			      r + 1 + SIM_RPMC_TAG_SIZE + 4))
			return SIM_RPMC_ERR_SIG;
		return SIM_RPMC_OK;
	default:
		return SIM_RPMC_ERR_SIG;
	}
}

static TEE_Result sim_rpmc_op1(const struct spi_mem_op *op)
{
	uint8_t pkt[4 + 32 + 32] = { SIM_RPMC_OP1 };
	uint8_t status = 0;

	if (op->data.dir != SPI_MEM_DATA_OUT ||
	    op->data.nbytes > sizeof(pkt) - 1)
		return TEE_ERROR_BAD_PARAMETERS;

	memcpy(pkt + 1, op->data.buf.out, op->data.nbytes);
	status = sim_rpmc_cmd(pkt, op->data.nbytes + 1);

	if (sim.rpmc.fault == SPI_NOR_SIM_RPMC_BAD_SIG)
		sim.rpmc.reply[SIM_RPMC_REPLY_SIZE - 1] ^= 1;
	else if (sim.rpmc.fault == SPI_NOR_SIM_RPMC_REPLAY)
		memcpy(sim.rpmc.reply, sim.rpmc.prev_reply,
		       sizeof(sim.rpmc.reply));
	sim.rpmc.fault = SPI_NOR_SIM_RPMC_NO_FAULT;

	sim.rpmc.reply[0] = status;
	sim.rpmc.busy = SIM_BUSY_POLLS;

	return TEE_SUCCESS;
}

static TEE_Result sim_rpmc_op2(const struct spi_mem_op *op)
{
	size_t n = MIN(op->data.nbytes, sizeof(sim.rpmc.reply));

	if (op->data.dir != SPI_MEM_DATA_IN)
		return TEE_ERROR_BAD_PARAMETERS;

	memcpy(op->data.buf.in, sim.rpmc.reply, n);
	if (sim.rpmc.busy) {
		/* While busy only the status is meaningful */
		((uint8_t *)op->data.buf.in)[0] = SIM_RPMC_BUSY;
		sim.rpmc.busy--;
	}

	return TEE_SUCCESS;
}

void spi_nor_sim_snapshot(void)
{
	snap.recording = true;
	snap.overflow = false;
	snap.count = 0;
}

TEE_Result spi_nor_sim_swap_snapshot(void)
{
	uint8_t *cur = NULL;
	uint8_t tmp = 0;
	unsigned int n = 0;
	size_t i = 0;

	snap.recording = false;
	if (snap.overflow)
		return TEE_ERROR_OUT_OF_MEMORY;

	for (n = 0; n < snap.count; n++) {
		cur = sim.array + snap.addr[n];
		for (i = 0; i < SIM_SECTOR_SIZE; i++) {
			tmp = cur[i];
			cur[i] = snap.data[n][i];
			snap.data[n][i] = tmp;
		}
	}

	return TEE_SUCCESS;
}

void spi_nor_sim_rpmc_fault(enum spi_nor_sim_rpmc_fault fault)
{
	sim.rpmc.fault = fault;
}

static TEE_Result sim_read_bytes(const uint8_t *src, size_t src_size,
				 const struct spi_mem_op *op)
{
	if (op->data.dir != SPI_MEM_DATA_IN || op->addr.val > src_size ||
	    op->data.nbytes > src_size - op->addr.val)
		return TEE_ERROR_BAD_PARAMETERS;

	memcpy(op->data.buf.in, src + op->addr.val, op->data.nbytes);

	return TEE_SUCCESS;
}

static void sim_set_busy(void)
{
	sim.busy = SIM_BUSY_POLLS;
	sim.busy_end = timeout_init_us(SIM_BUSY_US);
}

static TEE_Result sim_program(const struct spi_mem_op *op)
{
	uint64_t page = ROUNDDOWN(op->addr.val, SIM_PAGE_SIZE);
	const uint8_t *data = op->data.buf.out;
	size_t col = op->addr.val - page;
	size_t n = 0;

	if (!sim.wel || op->data.dir != SPI_MEM_DATA_OUT ||
	    op->addr.val >= SIM_SIZE)
		return TEE_ERROR_BAD_STATE;

	sim_snap_save(page, SIM_PAGE_SIZE);

	/* Program clears bits only and wraps inside the page */
	for (n = 0; n < op->data.nbytes; n++)
		sim.array[page + (col + n) % SIM_PAGE_SIZE] &= data[n];

	sim.wel = false;
	sim_set_busy();

	return TEE_SUCCESS;
}

static TEE_Result sim_erase(const struct spi_mem_op *op, size_t unit)
{
	if (!sim.wel || op->addr.val >= SIM_SIZE)
		return TEE_ERROR_BAD_STATE;

	sim_snap_save(ROUNDDOWN(op->addr.val, unit), unit);
	memset(sim.array + ROUNDDOWN(op->addr.val, unit), 0xff, unit);
	sim.wel = false;
	sim_set_busy();

	return TEE_SUCCESS;
}

static TEE_Result sim_exec_op(struct spi_mem *mem __unused,
			      const struct spi_mem_op *op)
{
	uint8_t sr = 0;

	if (sim.busy && timeout_elapsed(sim.busy_end))
		sim.busy = 0;

	/* While busy, a real flash ignores everything but RDSR */
	if (sim.busy && op->cmd.opcode != SPI_NOR_OP_RDSR)
		return TEE_ERROR_BUSY;

	switch (op->cmd.opcode) {
	case SPI_NOR_OP_RDID:
		return sim_read_bytes(sim_jedec_id, sizeof(sim_jedec_id), op);
	case SPI_NOR_OP_RDSFDP:
		return sim_read_bytes(sim.sfdp, sizeof(sim.sfdp), op);
	case SPI_NOR_OP_RDSR:
		if (sim.busy) {
			sr |= SPI_NOR_SR_WIP;
			sim.busy--;
		}
		if (sim.wel)
			sr |= SPI_NOR_SR_WEL;
		memset(op->data.buf.in, sr, op->data.nbytes);
		return TEE_SUCCESS;
	case SPI_NOR_OP_WREN:
		sim.wel = true;
		return TEE_SUCCESS;
	case SPI_NOR_OP_READ:
		return sim_read_bytes(sim.array, sizeof(sim.array), op);
	case SPI_NOR_OP_PP:
		return sim_program(op);
	case SIM_OP_SE:
		return sim_erase(op, SIM_SECTOR_SIZE);
	case SIM_OP_BE:
		return sim_erase(op, SIM_BLOCK_SIZE);
	case SIM_RPMC_OP1:
		if (!IS_ENABLED(CFG_SPI_NOR_RPMC))
			return TEE_ERROR_NOT_SUPPORTED;
		return sim_rpmc_op1(op);
	case SIM_RPMC_OP2:
		if (!IS_ENABLED(CFG_SPI_NOR_RPMC))
			return TEE_ERROR_NOT_SUPPORTED;
		return sim_rpmc_op2(op);
	default:
		return TEE_ERROR_NOT_SUPPORTED;
	}
}

static const struct spi_mem_ctrl_ops sim_ops = {
	.exec_op = sim_exec_op,
};

static struct spi_nor *sim_nor;

struct spi_nor *spi_nor_sim_get(void)
{
	return sim_nor;
}

static TEE_Result spi_nor_sim_init(void)
{
	struct spi_nor_partition *part = NULL;
	struct spi_nor *nor = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;

	build_sfdp(sim.sfdp);
	memset(sim.array, 0xff, sizeof(sim.array));

	sim.ctrl.ops = &sim_ops;
	sim.mem.ctrl = &sim.ctrl;
	sim.mem.rx_buswidth = 1;
	sim.mem.tx_buswidth = 1;

	res = spi_nor_probe(&sim.mem, &nor);
	if (res)
		return res;
	sim_nor = nor;
	if (!CFG_SPI_NOR_SIM_NS_SIZE)
		return TEE_SUCCESS;

	/* A partition the normal world may use, as a DT would declare */
	res = spi_nor_add_partition(nor, "sim-ns", CFG_SPI_NOR_SIM_NS_OFFSET,
				    CFG_SPI_NOR_SIM_NS_SIZE, &part);
	if (!res)
		part->ns_access = SPI_NOR_NS_ACCESS_RW;

	return res;
}

driver_init(spi_nor_sim_init);
