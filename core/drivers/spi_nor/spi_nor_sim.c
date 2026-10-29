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
 */

#include <assert.h>
#include <drivers/spi_mem.h>
#include <drivers/spi_nor.h>
#include <initcall.h>
#include <io.h>
#include <kernel/delay.h>
#include <string.h>
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

#define SIM_SFDP_BFPT_OFFS	0x10
#define SIM_SFDP_BFPT_DWORDS	16

struct spi_nor_sim {
	struct spi_mem_ctrl ctrl;
	struct spi_mem mem;
	uint8_t sfdp[SIM_SFDP_BFPT_OFFS + SIM_SFDP_BFPT_DWORDS * 4];
	bool wel;
	unsigned int busy;
	uint64_t busy_end;
	uint8_t array[SIM_SIZE];
};

static struct spi_nor_sim sim;

static void build_sfdp(uint8_t *sfdp)
{
	uint8_t *bfpt = sfdp + SIM_SFDP_BFPT_OFFS;

	/* SFDP header: signature, revision 1.6, one parameter header */
	put_le32(sfdp, 0x50444653);
	sfdp[4] = 6;
	sfdp[5] = 1;
	sfdp[6] = 0;
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
	struct spi_nor *nor = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;

	build_sfdp(sim.sfdp);
	memset(sim.array, 0xff, sizeof(sim.array));

	sim.ctrl.ops = &sim_ops;
	sim.mem.ctrl = &sim.ctrl;
	sim.mem.rx_buswidth = 1;
	sim.mem.tx_buswidth = 1;

	res = spi_nor_probe(&sim.mem, &nor);
	if (!res)
		sim_nor = nor;

	return res;
}

driver_init(spi_nor_sim_init);
