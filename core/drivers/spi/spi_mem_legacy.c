// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <assert.h>
#include <drivers/spi_mem.h>
#include <malloc.h>
#include <spi.h>
#include <string.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

/*
 * Bound the bounce buffers allocated per operation. Memory device drivers
 * split larger transfers, see spi_mem_max_xfer_size().
 */
#define SPI_MEM_LEGACY_MAX_XFER		4096

/* Value clocked out on MOSI during dummy cycles and data reads */
#define SPI_MEM_LEGACY_FILL_BYTE	0xff

struct spi_mem_legacy {
	struct spi_mem_ctrl ctrl;
	struct spi_mem mem;
	struct spi_chip *chip;
};

static struct spi_chip *mem_to_chip(struct spi_mem *mem)
{
	return container_of(mem->ctrl, struct spi_mem_legacy, ctrl)->chip;
}

static size_t put_be(uint8_t *buf, uint64_t val, size_t nbytes)
{
	size_t n = 0;

	for (n = 0; n < nbytes; n++)
		buf[n] = val >> (8 * (nbytes - 1 - n));

	return nbytes;
}

static TEE_Result legacy_exec_op(struct spi_mem *mem,
				 const struct spi_mem_op *op)
{
	struct spi_chip *chip = mem_to_chip(mem);
	TEE_Result res = TEE_ERROR_GENERIC;
	enum spi_result sres = SPI_ERR_GENERIC;
	size_t hdr_len = 0;
	size_t total = 0;
	uint8_t *wbuf = NULL;
	uint8_t *rbuf = NULL;
	size_t pos = 0;

	hdr_len = op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes;
	total = hdr_len + op->data.nbytes;

	wbuf = malloc(total);
	/*
	 * Always receive: some controllers (pl022) leave received bytes in
	 * their FIFO when no read buffer is given.
	 */
	rbuf = malloc(total);
	if (!wbuf || !rbuf) {
		res = TEE_ERROR_OUT_OF_MEMORY;
		goto out;
	}

	pos += put_be(wbuf + pos, op->cmd.opcode, op->cmd.nbytes);
	pos += put_be(wbuf + pos, op->addr.val, op->addr.nbytes);
	memset(wbuf + pos, SPI_MEM_LEGACY_FILL_BYTE, total - pos);
	if (op->data.dir == SPI_MEM_DATA_OUT)
		memcpy(wbuf + hdr_len, op->data.buf.out, op->data.nbytes);

	if (chip->ops->start)
		chip->ops->start(chip);
	sres = chip->ops->txrx8(chip, wbuf, rbuf, total);
	if (chip->ops->end)
		chip->ops->end(chip);

	if (sres != SPI_OK) {
		DMSG("SPI transfer failed: %d", sres);
		res = TEE_ERROR_COMMUNICATION;
		goto out;
	}

	if (op->data.dir == SPI_MEM_DATA_IN)
		memcpy(op->data.buf.in, rbuf + hdr_len, op->data.nbytes);

	res = TEE_SUCCESS;
out:
	free(wbuf);
	free(rbuf);

	return res;
}

static size_t legacy_max_xfer_size(struct spi_mem *mem __unused)
{
	return SPI_MEM_LEGACY_MAX_XFER;
}

static const struct spi_mem_ctrl_ops legacy_ops = {
	.exec_op = legacy_exec_op,
	.max_xfer_size = legacy_max_xfer_size,
};

TEE_Result spi_mem_legacy_get_dev(struct spi_chip *chip,
				  struct spi_mem **out_mem)
{
	struct spi_mem_legacy *lg = NULL;

	if (!chip || !chip->ops || !chip->ops->txrx8 || !out_mem)
		return TEE_ERROR_BAD_PARAMETERS;

	lg = calloc(1, sizeof(*lg));
	if (!lg)
		return TEE_ERROR_OUT_OF_MEMORY;

	lg->chip = chip;
	lg->ctrl.ops = &legacy_ops;
	lg->mem.ctrl = &lg->ctrl;
	lg->mem.rx_buswidth = 1;
	lg->mem.tx_buswidth = 1;

	*out_mem = &lg->mem;

	return TEE_SUCCESS;
}

void spi_mem_legacy_free_dev(struct spi_mem *mem)
{
	if (mem)
		free(container_of(mem->ctrl, struct spi_mem_legacy, ctrl));
}
