// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <drivers/spi_nor.h>
#include <inttypes.h>
#include <kernel/dt.h>
#include <libfdt.h>
#include <malloc.h>
#include <string.h>
#include <sys/queue.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "spi_nor_priv.h"

struct part_entry {
	struct spi_nor_partition part;
	STAILQ_ENTRY(part_entry) link;
};

static STAILQ_HEAD(, part_entry) part_list =
	STAILQ_HEAD_INITIALIZER(part_list);

static bool overlaps(const struct spi_nor_partition *p, struct spi_nor *nor,
		     uint64_t offs, uint64_t size)
{
	return p->nor == nor && offs < p->offs + p->size &&
	       p->offs < offs + size;
}

TEE_Result spi_nor_add_partition(struct spi_nor *nor, const char *label,
				 uint64_t offs, uint64_t size,
				 struct spi_nor_partition **out_part)
{
	size_t label_len = strnlen(label, SPI_NOR_PART_LABEL_LEN);
	uint64_t unit = nor->info.erase_size;
	struct part_entry *e = NULL;

	if (!label_len || label_len == SPI_NOR_PART_LABEL_LEN || !size ||
	    offs % unit || size % unit || offs > nor->info.size ||
	    size > nor->info.size - offs) {
		EMSG("SPI-NOR: bad partition \"%.*s\" %#"PRIx64" + %#"PRIx64,
		     SPI_NOR_PART_LABEL_LEN, label, offs, size);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	STAILQ_FOREACH(e, &part_list, link) {
		if (!strcmp(e->part.label, label) ||
		    overlaps(&e->part, nor, offs, size)) {
			EMSG("SPI-NOR: partition \"%s\" clashes with \"%s\"",
			     label, e->part.label);
			return TEE_ERROR_BAD_PARAMETERS;
		}
	}

	e = calloc(1, sizeof(*e));
	if (!e)
		return TEE_ERROR_OUT_OF_MEMORY;

	e->part.nor = nor;
	memcpy(e->part.label, label, label_len);
	e->part.offs = offs;
	e->part.size = size;
	STAILQ_INSERT_TAIL(&part_list, e, link);

	DMSG("SPI-NOR: partition \"%s\" %#"PRIx64" + %#"PRIx64, label, offs,
	     size);

	if (out_part)
		*out_part = &e->part;

	return TEE_SUCCESS;
}

struct spi_nor_partition *spi_nor_find_partition(const char *label)
{
	struct part_entry *e = NULL;

	STAILQ_FOREACH(e, &part_list, link)
		if (!strcmp(e->part.label, label))
			return &e->part;

	return NULL;
}

void spi_nor_del_partition(struct spi_nor_partition *part)
{
	struct part_entry *e = container_of(part, struct part_entry, part);

	STAILQ_REMOVE(&part_list, e, part_entry, link);
	free(e);
}

#ifdef CFG_DT
static void del_partitions(struct spi_nor *nor)
{
	struct part_entry *next = NULL;
	struct part_entry *e = NULL;

	STAILQ_FOREACH_SAFE(e, &part_list, link, next)
		if (e->part.nor == nor)
			spi_nor_del_partition(&e->part);
}

static TEE_Result read_cells(const fdt32_t *cell, int ncells, uint64_t *val)
{
	if (ncells == 1)
		*val = fdt32_to_cpu(cell[0]);
	else if (ncells == 2)
		*val = ((uint64_t)fdt32_to_cpu(cell[0]) << 32) |
		       fdt32_to_cpu(cell[1]);
	else
		return TEE_ERROR_BAD_FORMAT;

	return TEE_SUCCESS;
}

static TEE_Result add_dt_partition(struct spi_nor *nor, const void *fdt,
				   int node, int acells, int scells)
{
	const fdt32_t *reg = NULL;
	const char *label = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	uint64_t offs = 0;
	uint64_t size = 0;
	int len = 0;

	reg = fdt_getprop(fdt, node, "reg", &len);
	if (!reg || len != (acells + scells) * (int)sizeof(*reg))
		return TEE_ERROR_BAD_FORMAT;

	res = read_cells(reg, acells, &offs);
	if (!res)
		res = read_cells(reg + acells, scells, &size);
	if (res)
		return res;

	label = fdt_stringlist_get(fdt, node, "label", 0, NULL);
	if (!label)
		label = fdt_get_name(fdt, node, NULL);

	return spi_nor_add_partition(nor, label, offs, size, NULL);
}

TEE_Result spi_nor_dt_add_partitions(struct spi_nor *nor, const void *fdt,
				     int node)
{
	TEE_Result res = TEE_SUCCESS;
	int scells = 0;
	int acells = 0;
	int parts = 0;
	int child = 0;

	parts = fdt_subnode_offset(fdt, node, "partitions");
	if (parts < 0)
		return TEE_SUCCESS;

	acells = fdt_address_cells(fdt, parts);
	scells = fdt_size_cells(fdt, parts);

	fdt_for_each_subnode(child, fdt, parts) {
		if (fdt_get_status(fdt, child) == DT_STATUS_DISABLED)
			continue;

		res = add_dt_partition(nor, fdt, child, acells, scells);
		if (res) {
			EMSG("SPI-NOR: bad partition node %s",
			     fdt_get_name(fdt, child, NULL));
			del_partitions(nor);
			return res;
		}
	}

	return TEE_SUCCESS;
}
#endif /*CFG_DT*/
