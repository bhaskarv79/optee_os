// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <assert.h>
#include <drivers/spi_mem.h>
#include <drivers/spi_nor.h>
#include <inttypes.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <kernel/mutex.h>
#include <malloc.h>
#include <string.h>
#include <sys/queue.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "spi_nor_priv.h"

/*
 * Worst-case operation times. Datasheets give typical page programs under
 * 1 ms and 64 KiB erases under 1 s; these leave a wide margin.
 */
#define SPI_NOR_PROGRAM_TIMEOUT_US	(10 * 1000)
#define SPI_NOR_ERASE_TIMEOUT_US	(5 * 1000 * 1000)
#define SPI_NOR_REG_TIMEOUT_US		(1000)

static STAILQ_HEAD(, spi_nor) spi_nor_list =
	STAILQ_HEAD_INITIALIZER(spi_nor_list);

static TEE_Result exec(struct spi_nor *nor, uint8_t opcode, bool has_addr,
		       uint64_t addr, uint8_t dummy_nbytes,
		       enum spi_mem_data_dir dir, void *buf, size_t len)
{
	struct spi_mem_op op = {
		.cmd = { .nbytes = 1, .buswidth = 1, .opcode = opcode },
		.dummy = { .nbytes = dummy_nbytes, .buswidth = 1 },
		.data = { .buswidth = 1, .dir = dir, .nbytes = len },
	};

	if (has_addr) {
		op.addr.nbytes = nor->addr_nbytes;
		op.addr.buswidth = 1;
		op.addr.val = addr;
	}

	if (dir == SPI_MEM_DATA_IN)
		op.data.buf.in = buf;
	else if (dir == SPI_MEM_DATA_OUT)
		op.data.buf.out = buf;

	return spi_mem_exec_op(nor->mem, &op);
}

static TEE_Result cmd(struct spi_nor *nor, uint8_t opcode)
{
	return exec(nor, opcode, false, 0, 0, SPI_MEM_NO_DATA, NULL, 0);
}

TEE_Result spi_nor_wait_ready(struct spi_nor *nor, uint32_t timeout_us)
{
	uint64_t tmo = timeout_init_us(timeout_us);
	TEE_Result res = TEE_ERROR_GENERIC;
	uint8_t sr = 0;
	bool last = false;

	while (true) {
		last = timeout_elapsed(tmo);

		res = exec(nor, SPI_NOR_OP_RDSR, false, 0, 0,
			   SPI_MEM_DATA_IN, &sr, sizeof(sr));
		if (res)
			return res;

		if (!(sr & SPI_NOR_SR_WIP)) {
			nor->busy = false;
			return TEE_SUCCESS;
		}

		if (last) {
			EMSG("SPI-NOR: timeout, status %#"PRIx8, sr);
			nor->busy = true;
			return TEE_ERROR_STORAGE_NOT_AVAILABLE;
		}
	}
}

TEE_Result spi_nor_recover(struct spi_nor *nor)
{
	if (!nor->busy)
		return TEE_SUCCESS;

	return spi_nor_wait_ready(nor, SPI_NOR_ERASE_TIMEOUT_US);
}

static TEE_Result read_locked(struct spi_nor *nor, uint64_t offs, void *buf,
			      size_t len)
{
	size_t max = spi_mem_max_xfer_size(nor->mem);
	uint8_t *p = buf;
	size_t n = 0;
	TEE_Result res = TEE_ERROR_GENERIC;

	while (len) {
		n = MIN(len, max);
		res = exec(nor, SPI_NOR_OP_READ, true, offs, 0,
			   SPI_MEM_DATA_IN, p, n);
		if (res)
			return res;

		offs += n;
		p += n;
		len -= n;
	}

	return TEE_SUCCESS;
}

static bool range_is_valid(struct spi_nor *nor, uint64_t offs, size_t len)
{
	return offs <= nor->geo.size && len <= nor->geo.size - offs;
}

TEE_Result spi_nor_read(struct spi_nor *nor, uint64_t offs, void *buf,
			size_t len)
{
	TEE_Result res = TEE_ERROR_GENERIC;

	if (!range_is_valid(nor, offs, len) || (len && !buf))
		return TEE_ERROR_BAD_PARAMETERS;

	mutex_lock(&nor->lock);
	res = spi_nor_recover(nor);
	if (!res)
		res = read_locked(nor, offs, buf, len);
	mutex_unlock(&nor->lock);

	return res;
}

static TEE_Result program_chunk(struct spi_nor *nor, uint64_t offs,
				const uint8_t *data, size_t len,
				uint8_t *verify)
{
	TEE_Result res = TEE_ERROR_GENERIC;

	res = cmd(nor, SPI_NOR_OP_WREN);
	if (res)
		return res;

	res = exec(nor, SPI_NOR_OP_PP, true, offs, 0, SPI_MEM_DATA_OUT,
		   (void *)data, len);
	if (res) {
		/* The flash may have started programming anyway */
		nor->busy = true;
		return res;
	}

	res = spi_nor_wait_ready(nor, SPI_NOR_PROGRAM_TIMEOUT_US);
	if (res)
		return res;

	res = read_locked(nor, offs, verify, len);
	if (res)
		return res;

	if (memcmp(verify, data, len)) {
		EMSG("SPI-NOR: program verify failed at %#"PRIx64, offs);
		return TEE_ERROR_STORAGE_NOT_AVAILABLE;
	}

	return TEE_SUCCESS;
}

TEE_Result spi_nor_write(struct spi_nor *nor, uint64_t offs, const void *buf,
			 size_t len)
{
	size_t chunk_max = MIN((size_t)nor->geo.page_size,
			       spi_mem_max_xfer_size(nor->mem));
	TEE_Result res = TEE_SUCCESS;
	const uint8_t *p = buf;
	uint8_t *verify = NULL;
	size_t page_rem = 0;
	size_t n = 0;

	if (!range_is_valid(nor, offs, len) || (len && !buf))
		return TEE_ERROR_BAD_PARAMETERS;
	if (!len)
		return TEE_SUCCESS;

	verify = malloc(chunk_max);
	if (!verify)
		return TEE_ERROR_OUT_OF_MEMORY;

	mutex_lock(&nor->lock);
	res = spi_nor_recover(nor);
	while (!res && len) {
		/* A page program wraps inside its page: never cross one */
		page_rem = nor->geo.page_size - (offs % nor->geo.page_size);
		n = MIN(len, MIN(page_rem, chunk_max));

		res = program_chunk(nor, offs, p, n, verify);
		if (res)
			break;

		offs += n;
		p += n;
		len -= n;
	}
	mutex_unlock(&nor->lock);

	free(verify);

	return res;
}

/* Largest erase type aligned at @offs that fits in @len */
static const struct spi_nor_erase_type *pick_erase(struct spi_nor *nor,
						   uint64_t offs, size_t len)
{
	const struct spi_nor_erase_type *best = NULL;
	unsigned int n = 0;

	for (n = 0; n < SPI_NOR_MAX_ERASE_TYPES; n++) {
		const struct spi_nor_erase_type *et = nor->geo.erase + n;

		if (!et->size)
			break;
		if (!(offs % et->size) && len >= et->size)
			best = et;
	}

	return best;
}

TEE_Result spi_nor_erase(struct spi_nor *nor, uint64_t offs, size_t len)
{
	uint32_t unit = nor->geo.erase[0].size;
	const struct spi_nor_erase_type *et = NULL;
	TEE_Result res = TEE_SUCCESS;

	if (!range_is_valid(nor, offs, len) || offs % unit || len % unit)
		return TEE_ERROR_BAD_PARAMETERS;

	mutex_lock(&nor->lock);
	res = spi_nor_recover(nor);
	while (!res && len) {
		et = pick_erase(nor, offs, len);
		assert(et);

		res = cmd(nor, SPI_NOR_OP_WREN);
		if (!res) {
			res = exec(nor, et->opcode, true, offs, 0,
				   SPI_MEM_NO_DATA, NULL, 0);
			/* The flash may have started erasing anyway */
			if (res)
				nor->busy = true;
		}
		if (!res)
			res = spi_nor_wait_ready(nor, SPI_NOR_ERASE_TIMEOUT_US);
		if (res)
			break;

		offs += et->size;
		len -= et->size;
	}
	mutex_unlock(&nor->lock);

	return res;
}

void spi_nor_get_info(struct spi_nor *nor, struct spi_nor_info *info)
{
	*info = nor->info;
}

TEE_Result spi_nor_get(unsigned int index, struct spi_nor **out_nor)
{
	struct spi_nor *nor = NULL;

	STAILQ_FOREACH(nor, &spi_nor_list, link) {
		if (!index--) {
			*out_nor = nor;
			return TEE_SUCCESS;
		}
	}

	return TEE_ERROR_ITEM_NOT_FOUND;
}

static TEE_Result sfdp_read(void *ctx, uint32_t addr, void *buf, size_t len)
{
	struct spi_nor *nor = ctx;
	/* SFDP is always read with 3-byte addresses and 8 dummy clocks */
	struct spi_mem_op op = {
		.cmd = { .nbytes = 1, .buswidth = 1,
			 .opcode = SPI_NOR_OP_RDSFDP },
		.addr = { .nbytes = 3, .buswidth = 1, .val = addr },
		.dummy = { .nbytes = 1, .buswidth = 1 },
		.data = { .buswidth = 1, .dir = SPI_MEM_DATA_IN, .nbytes = len,
			  .buf.in = buf },
	};

	return spi_mem_exec_op(nor->mem, &op);
}

static bool id_is_valid(const uint8_t *id)
{
	/* No device answers all zeros or all ones on a floating bus */
	return !((id[0] == 0x00 && id[1] == 0x00 && id[2] == 0x00) ||
		 (id[0] == 0xff && id[1] == 0xff && id[2] == 0xff));
}

static TEE_Result identify(struct spi_nor *nor)
{
	TEE_Result res = TEE_ERROR_GENERIC;

	res = exec(nor, SPI_NOR_OP_RDID, false, 0, 0, SPI_MEM_DATA_IN,
		   nor->info.jedec_id, sizeof(nor->info.jedec_id));
	if (res)
		return res;

	if (!id_is_valid(nor->info.jedec_id))
		return TEE_ERROR_NOT_SUPPORTED;

	res = spi_nor_sfdp_parse(sfdp_read, nor, &nor->geo);
	if (!res) {
		nor->info.has_sfdp = true;
		return TEE_SUCCESS;
	}
	DMSG("SPI-NOR: no usable SFDP (%#"PRIx32"), trying ID table", res);

	res = spi_nor_ids_lookup(nor->info.jedec_id, &nor->geo);
	if (res) {
		EMSG("SPI-NOR: unknown flash %02x %02x %02x",
		     nor->info.jedec_id[0], nor->info.jedec_id[1],
		     nor->info.jedec_id[2]);
		return TEE_ERROR_NOT_SUPPORTED;
	}

	return TEE_SUCCESS;
}

static TEE_Result set_addr_mode(struct spi_nor *nor)
{
	TEE_Result res = TEE_ERROR_GENERIC;

	nor->addr_nbytes = 3;
	if (nor->geo.size <= SPI_NOR_3B_ADDR_MAX)
		return TEE_SUCCESS;

	if (!nor->geo.addr4_capable) {
		EMSG("SPI-NOR: %"PRIu64" bytes needs 4-byte addresses",
		     nor->geo.size);
		return TEE_ERROR_NOT_SUPPORTED;
	}

	res = cmd(nor, SPI_NOR_OP_EN4B);
	if (res)
		return res;

	nor->addr_nbytes = 4;

	return TEE_SUCCESS;
}

TEE_Result spi_nor_probe(struct spi_mem *mem, struct spi_nor **out_nor)
{
	struct spi_nor *nor = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;

	nor = calloc(1, sizeof(*nor));
	if (!nor)
		return TEE_ERROR_OUT_OF_MEMORY;

	nor->mem = mem;
	nor->addr_nbytes = 3;
	mutex_init(&nor->lock);

	res = identify(nor);
	if (!res)
		res = set_addr_mode(nor);
	if (!res)
		res = spi_nor_wait_ready(nor, SPI_NOR_REG_TIMEOUT_US);
	if (res) {
		free(nor);
		return res;
	}

	nor->info.size = nor->geo.size;
	nor->info.page_size = nor->geo.page_size;
	nor->info.erase_size = nor->geo.erase[0].size;

	STAILQ_INSERT_TAIL(&spi_nor_list, nor, link);

	IMSG("SPI-NOR: %02x %02x %02x, %"PRIu64" KiB, %s",
	     nor->info.jedec_id[0], nor->info.jedec_id[1],
	     nor->info.jedec_id[2], nor->info.size / 1024,
	     nor->info.has_sfdp ? "SFDP" : "ID table");

	*out_nor = nor;

	return TEE_SUCCESS;
}

#ifdef CFG_DT
static TEE_Result spi_nor_dt_probe(struct spi_mem *mem,
				   const void *fdt __unused,
				   int node __unused,
				   const void *compat_data __unused)
{
	struct spi_nor *nor = NULL;

	return spi_nor_probe(mem, &nor);
}

static const struct dt_device_match spi_nor_match_table[] = {
	{ .compatible = "jedec,spi-nor" },
	{ }
};

DEFINE_SPI_MEM_DEV_DRIVER(spi_nor, spi_nor_match_table, spi_nor_dt_probe);
#endif
