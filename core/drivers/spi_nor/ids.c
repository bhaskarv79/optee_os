// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <string.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "spi_nor_priv.h"

/*
 * Flashes whose geometry cannot be read from SFDP. Only add a part here
 * when its SFDP tables are missing or wrong; SFDP is used first.
 */
#define SZ_4K	0x1000
#define SZ_64K	0x10000

struct spi_nor_id {
	const char *name;
	uint8_t id[3];
	uint64_t size;
	bool addr4_capable;
};

static const struct spi_nor_id spi_nor_ids[] = {
	{ .name = "w25q128", .id = { 0xef, 0x40, 0x18 },
	  .size = 16 * 1024 * 1024 },
	{ .name = "w25q256", .id = { 0xef, 0x40, 0x19 },
	  .size = 32 * 1024 * 1024, .addr4_capable = true },
	{ .name = "mx25l12835f", .id = { 0xc2, 0x20, 0x18 },
	  .size = 16 * 1024 * 1024 },
};

TEE_Result spi_nor_ids_lookup(const uint8_t id[SPI_NOR_MAX_ID_LEN],
			      struct spi_nor_geometry *geo)
{
	const struct spi_nor_id *e = NULL;
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(spi_nor_ids); n++) {
		e = spi_nor_ids + n;
		if (memcmp(e->id, id, sizeof(e->id)))
			continue;

		memset(geo, 0, sizeof(*geo));
		geo->size = e->size;
		geo->page_size = 256;
		geo->erase[0] = (struct spi_nor_erase_type){
			.size = SZ_4K, .opcode = 0x20 };
		geo->erase[1] = (struct spi_nor_erase_type){
			.size = SZ_64K, .opcode = 0xd8 };
		geo->addr4_capable = e->addr4_capable;
		DMSG("SPI-NOR: %s from ID table", e->name);

		return TEE_SUCCESS;
	}

	return TEE_ERROR_ITEM_NOT_FOUND;
}
