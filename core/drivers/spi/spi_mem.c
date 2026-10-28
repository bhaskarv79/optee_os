// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <assert.h>
#include <drivers/spi_mem.h>
#include <stdint.h>
#include <tee_api_types.h>

static bool buswidth_is_valid(uint8_t buswidth)
{
	return buswidth == 1 || buswidth == 2 || buswidth == 4 ||
	       buswidth == 8;
}

static bool op_is_valid(const struct spi_mem_op *op)
{
	if (op->cmd.nbytes < 1 || op->cmd.nbytes > 2 ||
	    !buswidth_is_valid(op->cmd.buswidth))
		return false;

	if (op->addr.nbytes > 8 ||
	    (op->addr.nbytes && !buswidth_is_valid(op->addr.buswidth)))
		return false;

	if (op->dummy.nbytes && !buswidth_is_valid(op->dummy.buswidth))
		return false;

	switch (op->data.dir) {
	case SPI_MEM_NO_DATA:
		return !op->data.nbytes;
	case SPI_MEM_DATA_IN:
		return op->data.nbytes && op->data.buf.in &&
		       buswidth_is_valid(op->data.buswidth);
	case SPI_MEM_DATA_OUT:
		return op->data.nbytes && op->data.buf.out &&
		       buswidth_is_valid(op->data.buswidth);
	default:
		return false;
	}
}

/* Check every phase against the lines the device is wired with */
static bool op_fits_device(struct spi_mem *mem, const struct spi_mem_op *op)
{
	if (op->cmd.buswidth > mem->tx_buswidth)
		return false;

	if (op->addr.nbytes && op->addr.buswidth > mem->tx_buswidth)
		return false;

	if (op->dummy.nbytes && op->dummy.buswidth > mem->tx_buswidth)
		return false;

	if (op->data.dir == SPI_MEM_DATA_IN &&
	    op->data.buswidth > mem->rx_buswidth)
		return false;

	if (op->data.dir == SPI_MEM_DATA_OUT &&
	    op->data.buswidth > mem->tx_buswidth)
		return false;

	return true;
}

size_t spi_mem_max_xfer_size(struct spi_mem *mem)
{
	assert(mem && mem->ctrl && mem->ctrl->ops);

	if (!mem->ctrl->ops->max_xfer_size)
		return SIZE_MAX;

	return mem->ctrl->ops->max_xfer_size(mem);
}

static bool op_is_supported(struct spi_mem *mem, const struct spi_mem_op *op)
{
	if (!op_fits_device(mem, op))
		return false;

	if (op->data.nbytes > spi_mem_max_xfer_size(mem))
		return false;

	if (mem->ctrl->ops->supports_op)
		return mem->ctrl->ops->supports_op(mem, op);

	return true;
}

bool spi_mem_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	assert(mem && mem->ctrl && mem->ctrl->ops && op);

	return op_is_valid(op) && op_is_supported(mem, op);
}

TEE_Result spi_mem_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	assert(mem && mem->ctrl && mem->ctrl->ops &&
	       mem->ctrl->ops->exec_op && op);

	if (!op_is_valid(op))
		return TEE_ERROR_BAD_PARAMETERS;

	if (!op_is_supported(mem, op))
		return TEE_ERROR_NOT_SUPPORTED;

	return mem->ctrl->ops->exec_op(mem, op);
}
