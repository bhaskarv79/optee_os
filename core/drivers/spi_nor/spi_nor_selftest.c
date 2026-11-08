// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * SPI-NOR self tests, run with the core self tests (xtest 1001) when
 * CFG_SPI_NOR_EMBEDDED_TEST is enabled. They erase and program the simulated
 * flash.
 */

#include <drivers/spi_nor.h>
#include <io.h>
#include <libfdt.h>
#include <malloc.h>
#include <string.h>
#include <tee_api_types.h>
#include <trace.h>
#include <util.h>

#include "spi_nor_priv.h"

static unsigned int failures;

static void check(bool cond, const char *what, int line)
{
	if (!cond) {
		EMSG("SPI-NOR self test failed at line %d: %s", line, what);
		failures++;
	}
}

#define CHECK(cond)	check((cond), #cond, __LINE__)

/* Header, two parameter headers, BFPT at 0x20, RPMC table at 0x60 */
#define IMG_BFPT	0x20
#define IMG_RPMC	0x60

struct sfdp_image {
	uint8_t data[IMG_RPMC + 2 * 4];
};

static TEE_Result image_read(void *ctx, uint32_t addr, void *buf, size_t len)
{
	struct sfdp_image *img = ctx;

	if (addr > sizeof(img->data) || len > sizeof(img->data) - addr)
		return TEE_ERROR_BAD_PARAMETERS;

	memcpy(buf, img->data + addr, len);

	return TEE_SUCCESS;
}

/* 8 MiB, 4 KiB and 64 KiB erase, 256-byte pages, 3 or 4-byte addresses */
static void make_image(struct sfdp_image *img, uint8_t bfpt_dwords)
{
	uint8_t *bfpt = img->data + IMG_BFPT;

	memset(img, 0, sizeof(*img));
	put_le32(img->data, 0x50444653);
	img->data[5] = 1;
	img->data[7] = 0xff;
	img->data[11] = bfpt_dwords;
	img->data[12] = IMG_BFPT;
	img->data[15] = 0xff;

	put_le32(bfpt, 0x1 | (0x20 << 8) | (0x1 << 17));
	put_le32(bfpt + 4, 8 * 1024 * 1024 * 8 - 1);
	put_le32(bfpt + 7 * 4, 16 | (0xd8 << 8) | (12 << 16) | (0x20 << 24));
	put_le32(bfpt + 10 * 4, 8 << 4);
}

/* Add an RPMC Parameter Table with dword 1 @dw1 */
static void add_rpmc(struct sfdp_image *img, uint32_t dw1)
{
	img->data[6] = 1;
	img->data[16] = 0x03;
	img->data[18] = 1;
	img->data[19] = 2;
	img->data[20] = IMG_RPMC;
	img->data[23] = 0xff;
	put_le32(img->data + IMG_RPMC, dw1);
}

static void test_sfdp(void)
{
	struct spi_nor_geometry geo = { };
	struct sfdp_image img = { };

	make_image(&img, 16);
	CHECK(!spi_nor_sfdp_parse(image_read, &img, &geo));
	CHECK(geo.size == 8 * 1024 * 1024);
	CHECK(geo.page_size == 256);
	CHECK(geo.addr4_capable);
	/* Erase types come out sorted by size whatever the table order */
	CHECK(geo.erase[0].size == 0x1000 && geo.erase[0].opcode == 0x20);
	CHECK(geo.erase[1].size == 0x10000 && geo.erase[1].opcode == 0xd8);
	CHECK(!geo.erase[2].size);

	/* JESD216 9-dword table: no page size field, default 256 */
	make_image(&img, 9);
	CHECK(!spi_nor_sfdp_parse(image_read, &img, &geo));
	CHECK(geo.page_size == 256);

	make_image(&img, 16);
	img.data[0] = 'X';
	CHECK(spi_nor_sfdp_parse(image_read, &img, &geo) ==
	      TEE_ERROR_ITEM_NOT_FOUND);

	make_image(&img, 8);
	CHECK(spi_nor_sfdp_parse(image_read, &img, &geo) ==
	      TEE_ERROR_BAD_FORMAT);

	/* No erase type and no 4 KiB erase in dword 1 */
	make_image(&img, 16);
	put_le32(img.data + IMG_BFPT, 0);
	put_le32(img.data + IMG_BFPT + 7 * 4, 0);
	CHECK(spi_nor_sfdp_parse(image_read, &img, &geo) ==
	      TEE_ERROR_BAD_FORMAT);

	/* No RPMC table */
	make_image(&img, 16);
	CHECK(!spi_nor_sfdp_parse(image_read, &img, &geo));
	CHECK(!geo.rpmc.num_counters);

	/* Four counters, OP1 0x9b, OP2 0x96, busy polled with OP2 */
	add_rpmc(&img, (3 << 4) | (0x9b << 8) | (0x96 << 16));
	CHECK(!spi_nor_sfdp_parse(image_read, &img, &geo));
	CHECK(geo.rpmc.num_counters == 4);
	CHECK(geo.rpmc.op1 == 0x9b && geo.rpmc.op2 == 0x96);
	CHECK(geo.rpmc.poll_op2);

	/* QEMU w25r128jv model: counter-size bit and reserved bits set */
	add_rpmc(&img, 0xf0969b3a);
	CHECK(!spi_nor_sfdp_parse(image_read, &img, &geo));
	CHECK(geo.rpmc.num_counters == 4 && geo.rpmc.poll_op2);
	CHECK(geo.rpmc.op1 == 0x9b && geo.rpmc.op2 == 0x96);

	/* No flash hardening: RPMC is not used, the flash still is */
	add_rpmc(&img, 1 | (3 << 4) | (0x9b << 8) | (0x96 << 16));
	CHECK(!spi_nor_sfdp_parse(image_read, &img, &geo));
	CHECK(!geo.rpmc.num_counters);
}

static bool all_bytes(const uint8_t *buf, size_t len, uint8_t val)
{
	size_t n = 0;

	for (n = 0; n < len; n++)
		if (buf[n] != val)
			return false;

	return true;
}

static void test_nor(struct spi_nor *nor)
{
	struct spi_nor_info info = { };
	uint8_t data[64] = { };
	uint8_t buf[64] = { };
	uint8_t byte = 0;
	size_t n = 0;

	spi_nor_get_info(nor, &info);
	CHECK(info.has_sfdp);
	CHECK(info.size == CFG_SPI_NOR_SIM_SIZE);
	CHECK(info.page_size == 256 && info.erase_size == 0x1000);

	/* 64 KiB erase on a 64 KiB boundary, then 4 KiB erase */
	CHECK(!spi_nor_erase(nor, 0, 0x10000 + 0x1000));
	CHECK(!spi_nor_read(nor, 0x10000 + 0x1000 - sizeof(buf), buf,
			    sizeof(buf)));
	CHECK(all_bytes(buf, sizeof(buf), 0xff));

	/* A write across a page boundary is split and verified */
	for (n = 0; n < sizeof(data); n++)
		data[n] = n;
	CHECK(!spi_nor_write(nor, 256 - 20, data, sizeof(data)));
	CHECK(!spi_nor_read(nor, 256 - 20, buf, sizeof(buf)));
	CHECK(!memcmp(buf, data, sizeof(data)));

	/* Programming only clears bits: 0xf0 then 0x0f leaves 0x00 */
	byte = 0xf0;
	CHECK(!spi_nor_write(nor, 0x800, &byte, 1));
	byte = 0x0f;
	CHECK(spi_nor_write(nor, 0x800, &byte, 1) ==
	      TEE_ERROR_STORAGE_NOT_AVAILABLE);
	CHECK(!spi_nor_read(nor, 0x800, &byte, 1));
	CHECK(byte == 0x00);

	/* Erase restores 0xff */
	CHECK(!spi_nor_erase(nor, 0, 0x1000));
	CHECK(!spi_nor_read(nor, 0x800, &byte, 1));
	CHECK(byte == 0xff);

	/* Misaligned and out-of-range requests are rejected */
	CHECK(spi_nor_erase(nor, 0x100, 0x1000) == TEE_ERROR_BAD_PARAMETERS);
	CHECK(spi_nor_erase(nor, 0, 0x100) == TEE_ERROR_BAD_PARAMETERS);
	CHECK(spi_nor_erase(nor, info.size, 0x1000) ==
	      TEE_ERROR_BAD_PARAMETERS);
	CHECK(!spi_nor_read(nor, info.size - 1, &byte, 1));
	CHECK(spi_nor_read(nor, info.size, &byte, 1) ==
	      TEE_ERROR_BAD_PARAMETERS);
}

static void test_partitions(struct spi_nor *nor)
{
	struct spi_nor_partition *a = NULL;
	struct spi_nor_partition *b = NULL;

	CHECK(!spi_nor_add_partition(nor, "st-a", 0, 0x2000, &a));
	CHECK(spi_nor_find_partition("st-a") == a);
	CHECK(a && a->nor == nor && a->offs == 0 && a->size == 0x2000);

	/* Overlaps, duplicate labels and bad ranges are rejected */
	CHECK(spi_nor_add_partition(nor, "st-b", 0x1000, 0x2000, &b) ==
	      TEE_ERROR_BAD_PARAMETERS);
	CHECK(spi_nor_add_partition(nor, "st-a", 0x4000, 0x1000, &b) ==
	      TEE_ERROR_BAD_PARAMETERS);
	CHECK(spi_nor_add_partition(nor, "st-b", 0x2100, 0x1000, &b) ==
	      TEE_ERROR_BAD_PARAMETERS);
	CHECK(spi_nor_add_partition(nor, "st-b", 0x2000, 0, &b) ==
	      TEE_ERROR_BAD_PARAMETERS);
	CHECK(spi_nor_add_partition(nor, "", 0x2000, 0x1000, &b) ==
	      TEE_ERROR_BAD_PARAMETERS);
	CHECK(!spi_nor_find_partition("st-b"));

	/* Adjacent is fine */
	CHECK(!spi_nor_add_partition(nor, "st-b", 0x2000, 0x1000, &b));
	CHECK(spi_nor_find_partition("st-b") == b);

	if (a)
		spi_nor_del_partition(a);
	if (b)
		spi_nor_del_partition(b);
	CHECK(!spi_nor_find_partition("st-a"));
	CHECK(!spi_nor_find_partition("st-b"));
}

#ifdef CFG_DT
static void *make_dt(bool overlap)
{
	const size_t sz = 1024;
	void *fdt = malloc(sz);
	bool ok = false;

	if (!fdt)
		return NULL;

	ok = !fdt_create(fdt, sz) && !fdt_finish_reservemap(fdt) &&
	     !fdt_begin_node(fdt, "") && !fdt_begin_node(fdt, "flash@0") &&
	     !fdt_begin_node(fdt, "partitions") &&
	     !fdt_property_u32(fdt, "#address-cells", 1) &&
	     !fdt_property_u32(fdt, "#size-cells", 1);
	if (ok) {
		/* Labelled partition */
		fdt32_t reg_a[] = { cpu_to_fdt32(0x4000),
				    cpu_to_fdt32(0x2000) };
		/* No label: the node name is used */
		fdt32_t reg_b[] = { cpu_to_fdt32(0x6000),
				    cpu_to_fdt32(0x1000) };
		/* Disabled, so ignored even though it overlaps */
		fdt32_t reg_c[] = { cpu_to_fdt32(0x4000),
				    cpu_to_fdt32(0x1000) };

		if (overlap)
			reg_b[0] = cpu_to_fdt32(0x5000);

		ok = !fdt_begin_node(fdt, "partition@4000") &&
		     !fdt_property_string(fdt, "label", "st-dt-a") &&
		     !fdt_property(fdt, "reg", reg_a, sizeof(reg_a)) &&
		     !fdt_end_node(fdt) &&
		     !fdt_begin_node(fdt, "st-dt-b") &&
		     !fdt_property(fdt, "reg", reg_b, sizeof(reg_b)) &&
		     !fdt_end_node(fdt) &&
		     !fdt_begin_node(fdt, "st-dt-c") &&
		     !fdt_property_string(fdt, "status", "disabled") &&
		     !fdt_property(fdt, "reg", reg_c, sizeof(reg_c)) &&
		     !fdt_end_node(fdt) &&
		     !fdt_end_node(fdt) && !fdt_end_node(fdt) &&
		     !fdt_end_node(fdt) && !fdt_finish(fdt);
	}
	if (!ok) {
		free(fdt);
		return NULL;
	}

	return fdt;
}

static void test_dt_partitions(struct spi_nor *nor)
{
	struct spi_nor_partition *a = NULL;
	struct spi_nor_partition *b = NULL;
	void *fdt = NULL;
	int node = 0;

	fdt = make_dt(false);
	CHECK(fdt);
	if (!fdt)
		return;
	node = fdt_path_offset(fdt, "/flash@0");
	CHECK(!spi_nor_dt_add_partitions(nor, fdt, node));
	a = spi_nor_find_partition("st-dt-a");
	b = spi_nor_find_partition("st-dt-b");
	CHECK(a && a->offs == 0x4000 && a->size == 0x2000);
	CHECK(b && b->offs == 0x6000 && b->size == 0x1000);
	CHECK(!spi_nor_find_partition("st-dt-c"));
	if (a)
		spi_nor_del_partition(a);
	if (b)
		spi_nor_del_partition(b);
	free(fdt);

	/* One bad partition: none is kept */
	fdt = make_dt(true);
	CHECK(fdt);
	if (!fdt)
		return;
	node = fdt_path_offset(fdt, "/flash@0");
	CHECK(spi_nor_dt_add_partitions(nor, fdt, node) ==
	      TEE_ERROR_BAD_PARAMETERS);
	CHECK(!spi_nor_find_partition("st-dt-a"));
	CHECK(!spi_nor_find_partition("st-dt-b"));
	free(fdt);
}
#else
static void test_dt_partitions(struct spi_nor *nor __unused)
{
}
#endif

TEE_Result spi_nor_self_test(void)
{
	struct spi_nor *nor = spi_nor_sim_get();

	failures = 0;
	test_sfdp();

	/* Never a real flash: the tests erase it */
	if (!nor) {
		EMSG("SPI-NOR self test: no simulated flash");
		failures++;
	} else {
		test_nor(nor);
		test_partitions(nor);
		test_dt_partitions(nor);
	}

	return failures ? TEE_ERROR_GENERIC : TEE_SUCCESS;
}
