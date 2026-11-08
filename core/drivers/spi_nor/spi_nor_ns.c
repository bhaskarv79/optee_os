// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <drivers/spi_nor.h>
#include <drivers/spi_nor_ns.h>
#include <malloc.h>
#include <string.h>
#include <tee_api_types.h>
#include <util.h>

static struct spi_nor_partition *find_exposed(unsigned int id)
{
	struct spi_nor_partition *part = NULL;

	while ((part = spi_nor_next_partition(part)))
		if (part->ns_access != SPI_NOR_NS_ACCESS_NONE && !id--)
			return part;

	return NULL;
}

unsigned int spi_nor_ns_count(void)
{
	struct spi_nor_partition *part = NULL;
	unsigned int n = 0;

	while ((part = spi_nor_next_partition(part)))
		if (part->ns_access != SPI_NOR_NS_ACCESS_NONE)
			n++;

	return n;
}

TEE_Result spi_nor_ns_get_info(unsigned int id, struct spi_nor_ns_info *info)
{
	struct spi_nor_partition *part = find_exposed(id);
	struct spi_nor_info nor_info = { };

	if (!part)
		return TEE_ERROR_ITEM_NOT_FOUND;

	spi_nor_get_info(part->nor, &nor_info);

	memset(info, 0, sizeof(*info));
	memcpy(info->label, part->label, sizeof(info->label));
	info->access = part->ns_access;
	info->size = part->size;
	info->erase_size = nor_info.erase_size;
	info->page_size = nor_info.page_size;

	return TEE_SUCCESS;
}

/* Find partition @id and check that it allows @offs + @len and @access */
static TEE_Result check(unsigned int id, uint64_t offs, size_t len,
			size_t max_len, enum spi_nor_ns_access access,
			struct spi_nor_partition **out_part)
{
	struct spi_nor_partition *part = find_exposed(id);

	if (!part)
		return TEE_ERROR_ITEM_NOT_FOUND;
	if (!len || len > max_len || offs > part->size ||
	    len > part->size - offs)
		return TEE_ERROR_BAD_PARAMETERS;
	if (access == SPI_NOR_NS_ACCESS_RW &&
	    part->ns_access != SPI_NOR_NS_ACCESS_RW)
		return TEE_ERROR_ACCESS_DENIED;

	*out_part = part;

	return TEE_SUCCESS;
}

TEE_Result spi_nor_ns_read(unsigned int id, uint64_t offs, void *buf,
			   size_t len)
{
	struct spi_nor_partition *part = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	void *bounce = NULL;

	res = check(id, offs, len, SPI_NOR_NS_MAX_XFER, SPI_NOR_NS_ACCESS_RO,
		    &part);
	if (res)
		return res;

	bounce = malloc(len);
	if (!bounce)
		return TEE_ERROR_OUT_OF_MEMORY;

	res = spi_nor_read(part->nor, part->offs + offs, bounce, len);
	if (!res)
		memcpy(buf, bounce, len);
	free(bounce);

	return res;
}

TEE_Result spi_nor_ns_write(unsigned int id, uint64_t offs, const void *buf,
			    size_t len)
{
	struct spi_nor_partition *part = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	void *bounce = NULL;

	res = check(id, offs, len, SPI_NOR_NS_MAX_XFER, SPI_NOR_NS_ACCESS_RW,
		    &part);
	if (res)
		return res;

	bounce = malloc(len);
	if (!bounce)
		return TEE_ERROR_OUT_OF_MEMORY;

	memcpy(bounce, buf, len);
	res = spi_nor_write(part->nor, part->offs + offs, bounce, len);
	free(bounce);

	return res;
}

TEE_Result spi_nor_ns_erase(unsigned int id, uint64_t offs, size_t len)
{
	struct spi_nor_partition *part = NULL;
	struct spi_nor_info info = { };
	TEE_Result res = TEE_ERROR_GENERIC;

	res = check(id, offs, len, SPI_NOR_NS_MAX_ERASE, SPI_NOR_NS_ACCESS_RW,
		    &part);
	if (res)
		return res;

	spi_nor_get_info(part->nor, &info);
	if (offs % info.erase_size || len % info.erase_size)
		return TEE_ERROR_BAD_PARAMETERS;

	return spi_nor_erase(part->nor, part->offs + offs, len);
}
