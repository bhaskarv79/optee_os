// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <drivers/spi_mem.h>
#include <spi.h>
#include <string.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "misc.h"

/*
 * Loopback legacy controller: records the bytes sent in one transfer and
 * returns a known pattern so spi-mem framing can be checked byte by byte.
 */
#define FAKE_BUF_SIZE	64

struct fake_spi {
	struct spi_chip chip;
	uint8_t sent[FAKE_BUF_SIZE];
	size_t sent_len;
	unsigned int starts;
	unsigned int ends;
};

static uint8_t fake_rx_byte(size_t i)
{
	return 0xa0 + i;
}

static void fake_start(struct spi_chip *chip)
{
	container_of(chip, struct fake_spi, chip)->starts++;
}

static void fake_end(struct spi_chip *chip)
{
	container_of(chip, struct fake_spi, chip)->ends++;
}

static enum spi_result fake_txrx8(struct spi_chip *chip, uint8_t *wdat,
				  uint8_t *rdat, size_t num_pkts)
{
	struct fake_spi *fs = container_of(chip, struct fake_spi, chip);
	size_t i = 0;

	if (!wdat || !rdat || num_pkts > sizeof(fs->sent))
		return SPI_ERR_PKTCNT;

	memcpy(fs->sent, wdat, num_pkts);
	fs->sent_len = num_pkts;
	for (i = 0; i < num_pkts; i++)
		rdat[i] = fake_rx_byte(i);

	return SPI_OK;
}

static const struct spi_ops fake_ops = {
	.start = fake_start,
	.txrx8 = fake_txrx8,
	.end = fake_end,
};

static TEE_Result test_read(struct spi_mem *mem, struct fake_spi *fs)
{
	static const uint8_t hdr[] = { 0x0b, 0x12, 0x34, 0x56, 0xff };
	uint8_t data[4] = { };
	struct spi_mem_op op = {
		.cmd = { .nbytes = 1, .buswidth = 1, .opcode = 0x0b },
		.addr = { .nbytes = 3, .buswidth = 1, .val = 0x123456 },
		.dummy = { .nbytes = 1, .buswidth = 1 },
		.data = { .buswidth = 1, .dir = SPI_MEM_DATA_IN,
			  .nbytes = sizeof(data), .buf.in = data },
	};
	size_t i = 0;

	if (spi_mem_exec_op(mem, &op))
		return TEE_ERROR_GENERIC;

	if (fs->sent_len != sizeof(hdr) + sizeof(data) ||
	    memcmp(fs->sent, hdr, sizeof(hdr)) ||
	    fs->starts != 1 || fs->ends != 1)
		return TEE_ERROR_GENERIC;

	/* Clocked-out bytes during the data phase are the fill byte */
	for (i = 0; i < sizeof(data); i++)
		if (fs->sent[sizeof(hdr) + i] != 0xff ||
		    data[i] != fake_rx_byte(sizeof(hdr) + i))
			return TEE_ERROR_GENERIC;

	return TEE_SUCCESS;
}

static TEE_Result test_write(struct spi_mem *mem, struct fake_spi *fs)
{
	static const uint8_t hdr[] = { 0x12, 0x01, 0x02, 0x03, 0x04 };
	static const uint8_t data[] = { 'a', 'b', 'c', 'd' };
	struct spi_mem_op op = {
		.cmd = { .nbytes = 1, .buswidth = 1, .opcode = 0x12 },
		.addr = { .nbytes = 4, .buswidth = 1, .val = 0x01020304 },
		.data = { .buswidth = 1, .dir = SPI_MEM_DATA_OUT,
			  .nbytes = sizeof(data), .buf.out = data },
	};

	if (spi_mem_exec_op(mem, &op))
		return TEE_ERROR_GENERIC;

	if (fs->sent_len != sizeof(hdr) + sizeof(data) ||
	    memcmp(fs->sent, hdr, sizeof(hdr)) ||
	    memcmp(fs->sent + sizeof(hdr), data, sizeof(data)))
		return TEE_ERROR_GENERIC;

	return TEE_SUCCESS;
}

static TEE_Result test_rejected_ops(struct spi_mem *mem)
{
	uint8_t data[4] = { };
	struct spi_mem_op op = { };

	/* No opcode byte */
	op = (struct spi_mem_op){ .cmd = { .nbytes = 0, .buswidth = 1 } };
	if (spi_mem_exec_op(mem, &op) != TEE_ERROR_BAD_PARAMETERS)
		return TEE_ERROR_GENERIC;

	/* Data phase without a buffer */
	op = (struct spi_mem_op){
		.cmd = { .nbytes = 1, .buswidth = 1, .opcode = 0x03 },
		.data = { .buswidth = 1, .dir = SPI_MEM_DATA_IN, .nbytes = 4 },
	};
	if (spi_mem_exec_op(mem, &op) != TEE_ERROR_BAD_PARAMETERS)
		return TEE_ERROR_GENERIC;

	/* Quad data on a single-line device */
	op = (struct spi_mem_op){
		.cmd = { .nbytes = 1, .buswidth = 1, .opcode = 0x6b },
		.data = { .buswidth = 4, .dir = SPI_MEM_DATA_IN,
			  .nbytes = sizeof(data), .buf.in = data },
	};
	if (spi_mem_supports_op(mem, &op) ||
	    spi_mem_exec_op(mem, &op) != TEE_ERROR_NOT_SUPPORTED)
		return TEE_ERROR_GENERIC;

	/* Larger than the controller can transfer at once */
	op = (struct spi_mem_op){
		.cmd = { .nbytes = 1, .buswidth = 1, .opcode = 0x03 },
		.data = { .buswidth = 1, .dir = SPI_MEM_DATA_IN,
			  .nbytes = spi_mem_max_xfer_size(mem) + 1,
			  .buf.in = data },
	};
	if (spi_mem_exec_op(mem, &op) != TEE_ERROR_NOT_SUPPORTED)
		return TEE_ERROR_GENERIC;

	return TEE_SUCCESS;
}

TEE_Result self_test_spi_mem(void)
{
	struct fake_spi fs = { .chip = { .ops = &fake_ops } };
	struct spi_mem *mem = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;

	res = spi_mem_legacy_get_dev(&fs.chip, &mem);
	if (res)
		return res;

	res = test_read(mem, &fs);
	if (!res)
		res = test_write(mem, &fs);
	if (!res)
		res = test_rejected_ops(mem);

	if (res)
		EMSG("spi-mem self test failed");

	spi_mem_legacy_free_dev(mem);

	return res;
}
