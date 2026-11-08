// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <assert.h>
#include <drivers/spi_nor_ns.h>
#include <kernel/pseudo_ta.h>
#include <kernel/tee_ta_manager.h>
#include <kernel/ts_manager.h>
#include <pta_spi_nor.h>
#include <string.h>
#include <tee_api_types.h>
#include <util.h>

#define PTA_NAME "spi_nor.pta"

static_assert(PTA_SPI_NOR_MAX_XFER == SPI_NOR_NS_MAX_XFER);
static_assert(PTA_SPI_NOR_MAX_ERASE == SPI_NOR_NS_MAX_ERASE);
static_assert(PTA_SPI_NOR_LABEL_LEN == SPI_NOR_PART_LABEL_LEN);

static TEE_Result get_info(uint32_t ptypes, TEE_Param params[TEE_NUM_PARAMS])
{
	struct pta_spi_nor_part *out = params[0].memref.buffer;
	struct spi_nor_ns_info info = { };
	unsigned int count = spi_nor_ns_count();
	size_t need = count * sizeof(*out);
	unsigned int n = 0;

	if (ptypes != TEE_PARAM_TYPES(TEE_PARAM_TYPE_MEMREF_OUTPUT,
				      TEE_PARAM_TYPE_NONE,
				      TEE_PARAM_TYPE_NONE,
				      TEE_PARAM_TYPE_NONE))
		return TEE_ERROR_BAD_PARAMETERS;

	if (params[0].memref.size < need) {
		params[0].memref.size = need;
		return TEE_ERROR_SHORT_BUFFER;
	}
	if (count && !out)
		return TEE_ERROR_BAD_PARAMETERS;

	for (n = 0; n < count; n++) {
		struct pta_spi_nor_part part = { .id = n };

		if (spi_nor_ns_get_info(n, &info))
			return TEE_ERROR_GENERIC;

		if (info.access == SPI_NOR_NS_ACCESS_RW)
			part.access = PTA_SPI_NOR_ACCESS_RW;
		else
			part.access = PTA_SPI_NOR_ACCESS_RO;
		part.size = info.size;
		part.erase_size = info.erase_size;
		part.page_size = info.page_size;
		memcpy(part.label, info.label, sizeof(part.label));
		part.label[sizeof(part.label) - 1] = '\0';

		memcpy(out + n, &part, sizeof(part));
	}
	params[0].memref.size = need;

	return TEE_SUCCESS;
}

static TEE_Result read_write(bool write, uint32_t ptypes,
			     TEE_Param params[TEE_NUM_PARAMS])
{
	uint32_t data_type = TEE_PARAM_TYPE_MEMREF_OUTPUT;
	void *buf = params[1].memref.buffer;
	size_t len = params[1].memref.size;

	if (write)
		data_type = TEE_PARAM_TYPE_MEMREF_INPUT;

	if (ptypes != TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT, data_type,
				      TEE_PARAM_TYPE_NONE,
				      TEE_PARAM_TYPE_NONE) || !buf)
		return TEE_ERROR_BAD_PARAMETERS;

	if (write)
		return spi_nor_ns_write(params[0].value.a, params[0].value.b,
					buf, len);

	return spi_nor_ns_read(params[0].value.a, params[0].value.b, buf, len);
}

static TEE_Result erase(uint32_t ptypes, TEE_Param params[TEE_NUM_PARAMS])
{
	if (ptypes != TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
				      TEE_PARAM_TYPE_VALUE_INPUT,
				      TEE_PARAM_TYPE_NONE,
				      TEE_PARAM_TYPE_NONE))
		return TEE_ERROR_BAD_PARAMETERS;

	return spi_nor_ns_erase(params[0].value.a, params[0].value.b,
				params[1].value.a);
}

static TEE_Result open_session(uint32_t ptypes __unused,
			       TEE_Param params[TEE_NUM_PARAMS] __unused,
			       void **session __unused)
{
	struct ts_session *ts = ts_get_current_session();
	struct tee_ta_session *ta_session = to_ta_session(ts);

	/* Only the REE kernel drives the flash; user space uses its driver */
	if (ta_session->clnt_id.login != TEE_LOGIN_REE_KERNEL)
		return TEE_ERROR_ACCESS_DENIED;

	return TEE_SUCCESS;
}

static TEE_Result invoke_command(void *session __unused, uint32_t cmd,
				 uint32_t ptypes,
				 TEE_Param params[TEE_NUM_PARAMS])
{
	switch (cmd) {
	case PTA_SPI_NOR_CMD_GET_INFO:
		return get_info(ptypes, params);
	case PTA_SPI_NOR_CMD_READ:
		return read_write(false, ptypes, params);
	case PTA_SPI_NOR_CMD_WRITE:
		return read_write(true, ptypes, params);
	case PTA_SPI_NOR_CMD_ERASE:
		return erase(ptypes, params);
	default:
		return TEE_ERROR_NOT_IMPLEMENTED;
	}
}

pseudo_ta_register(.uuid = PTA_SPI_NOR_UUID, .name = PTA_NAME,
		   .flags = PTA_DEFAULT_FLAGS | TA_FLAG_CONCURRENT |
			    TA_FLAG_DEVICE_ENUM,
		   .open_session_entry_point = open_session,
		   .invoke_command_entry_point = invoke_command);
