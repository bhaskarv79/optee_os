// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * Secure SPI-NOR flash of the QEMU virt machine (-machine virt,spi=secure):
 * a PL022 SPI controller with a 32 MiB JESD260 RPMC capable flash on chip
 * select 0, driven by line 0 of a dedicated PL061. OP-TEE drives the PL022
 * through the legacy spi-mem adapter and declares the flash partitions here,
 * as a secure DT would, then reports what each partition holds.
 */

#include <drivers/pl022_spi.h>
#include <drivers/pl061_gpio.h>
#include <drivers/spi_mem.h>
#include <drivers/spi_nor.h>
#include <initcall.h>
#include <inttypes.h>
#include <io.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <stdio.h>
#include <trace.h>
#include <util.h>

#define QEMU_SPI_BASE		0x090c0000
#define QEMU_SPI_GPIO_BASE	0x090d0000
#define QEMU_SPI_CS_PIN		0
#define QEMU_SPI_CLK_HZ		24000000
#define QEMU_SPI_SPEED_HZ	12000000

/* Magic numbers of the images the content report recognizes */
#define UIMAGE_MAGIC		0x27051956	/* U-Boot legacy image, BE */
#define UIMAGE_NAME_OFFS	32
#define UIMAGE_NAME_LEN		32
#define FDT_MAGIC		0xd00dfeed	/* DTB or FIT image, BE */
#define ZIMAGE_MAGIC_OFFS	0x24
#define ZIMAGE_MAGIC		0x016f2818	/* ARM Linux zImage, LE */
#define SPINOR_FS_MAGIC		0x4c464e53	/* SPI-NOR FS log sector, LE */

register_phys_mem_pgdir(MEM_AREA_IO_SEC, QEMU_SPI_BASE, PL022_REG_SIZE);
register_phys_mem_pgdir(MEM_AREA_IO_SEC, QEMU_SPI_GPIO_BASE, PL061_REG_SIZE);

static struct pl061_data gpio;
static struct pl022_data spi;

struct qemu_part {
	const char *label;
	uint64_t offs;
	uint64_t size;
	enum spi_nor_ns_access ns_access;
};

static const struct qemu_part qemu_parts[] = {
	{ "boot", CFG_QEMU_SPI_NOR_BOOT_OFFSET, CFG_QEMU_SPI_NOR_BOOT_SIZE,
	  SPI_NOR_NS_ACCESS_RO },
	{ "kernel", CFG_QEMU_SPI_NOR_KERNEL_OFFSET,
	  CFG_QEMU_SPI_NOR_KERNEL_SIZE, SPI_NOR_NS_ACCESS_RO },
	{ "ns-data", CFG_QEMU_SPI_NOR_NS_OFFSET, CFG_QEMU_SPI_NOR_NS_SIZE,
	  SPI_NOR_NS_ACCESS_RW },
	{ "optee-fs", CFG_QEMU_SPI_NOR_FS_OFFSET, CFG_QEMU_SPI_NOR_FS_SIZE,
	  SPI_NOR_NS_ACCESS_NONE },
};

static TEE_Result add_partitions(struct spi_nor *nor)
{
	struct spi_nor_partition *part = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;
	size_t n = 0;

	for (n = 0; n < ARRAY_SIZE(qemu_parts); n++) {
		if (!qemu_parts[n].size)
			continue;

		res = spi_nor_add_partition(nor, qemu_parts[n].label,
					    qemu_parts[n].offs,
					    qemu_parts[n].size, &part);
		if (res) {
			EMSG("QEMU SPI-NOR: bad partition \"%s\"",
			     qemu_parts[n].label);
			return res;
		}
		part->ns_access = qemu_parts[n].ns_access;
	}

	return TEE_SUCCESS;
}

static const char *ns_access_str(enum spi_nor_ns_access ns_access)
{
	switch (ns_access) {
	case SPI_NOR_NS_ACCESS_RO:
		return "ns:ro";
	case SPI_NOR_NS_ACCESS_RW:
		return "ns:rw";
	default:
		return "secure";
	}
}

/* Name the content of a partition from its first bytes */
static void describe(const uint8_t *hdr, size_t len, char *buf, size_t size)
{
	size_t n = 0;

	while (n < len && hdr[n] == 0xff)
		n++;

	if (n == len)
		snprintf(buf, size, "erased");
	else if (get_be32(hdr) == UIMAGE_MAGIC)
		snprintf(buf, size, "uImage \"%.*s\"", UIMAGE_NAME_LEN,
			 (const char *)hdr + UIMAGE_NAME_OFFS);
	else if (get_be32(hdr) == FDT_MAGIC)
		snprintf(buf, size, "FDT/FIT, %"PRIu32" bytes",
			 get_be32(hdr + 4));
	else if (get_le32(hdr + ZIMAGE_MAGIC_OFFS) == ZIMAGE_MAGIC)
		snprintf(buf, size, "Linux zImage");
	else if (get_le32(hdr) == SPINOR_FS_MAGIC)
		snprintf(buf, size, "SPI-NOR FS");
	else
		snprintf(buf, size, "data");
}

static const char *manufacturer(uint8_t id)
{
	switch (id) {
	case 0x01:
		return "Infineon";
	case 0x20:
		return "Micron";
	case 0xc2:
		return "Macronix";
	case 0xc8:
		return "GigaDevice";
	case 0xef:
		return "Winbond";
	default:
		return "unknown";
	}
}

/* List the flash, its partitions and what they hold */
static void report(struct spi_nor *nor)
{
	uint8_t hdr[UIMAGE_NAME_OFFS + UIMAGE_NAME_LEN] = { };
	struct spi_nor_partition *part = NULL;
	struct spi_nor_info info = { };
	char what[UIMAGE_NAME_LEN + 16] = { };

	spi_nor_get_info(nor, &info);
	IMSG("QEMU SPI-NOR: %s %02x %02x %02x, %"PRIu64" MiB, partitions:",
	     manufacturer(info.jedec_id[0]), info.jedec_id[0],
	     info.jedec_id[1], info.jedec_id[2], info.size / SIZE_1M);

	while ((part = spi_nor_next_partition(part))) {
		if (part->nor != nor)
			continue;

		if (spi_nor_read(nor, part->offs, hdr, sizeof(hdr)))
			snprintf(what, sizeof(what), "read error");
		else
			describe(hdr, sizeof(hdr), what, sizeof(what));

		IMSG("  %-9s 0x%08"PRIx64" %6"PRIu64" KiB  %-6s  %s",
		     part->label, part->offs, part->size / 1024,
		     ns_access_str(part->ns_access), what);
	}
}

static TEE_Result qemu_spi_nor_init(void)
{
	struct spi_mem *mem = NULL;
	struct spi_nor *nor = NULL;
	TEE_Result res = TEE_ERROR_GENERIC;

	pl061_init(&gpio);
	pl061_register(core_mmu_get_va(QEMU_SPI_GPIO_BASE, MEM_AREA_IO_SEC,
				       PL061_REG_SIZE), 0);

	spi.base = core_mmu_get_va(QEMU_SPI_BASE, MEM_AREA_IO_SEC,
				   PL022_REG_SIZE);
	spi.cs_control = PL022_CS_CTRL_AUTO_GPIO;
	spi.cs_data.gpio_data.chip = &gpio.chip;
	spi.cs_data.gpio_data.pin_num = QEMU_SPI_CS_PIN;
	spi.clk_hz = QEMU_SPI_CLK_HZ;
	spi.speed_hz = QEMU_SPI_SPEED_HZ;
	spi.mode = SPI_MODE0;
	spi.data_size_bits = 8;
	pl022_init(&spi);
	spi.chip.ops->configure(&spi.chip);

	res = spi_mem_legacy_get_dev(&spi.chip, &mem);
	if (res)
		return res;

	res = spi_nor_probe(mem, &nor);
	if (res) {
		EMSG("QEMU SPI-NOR: no flash, is QEMU run with spi=secure?");
		spi_mem_legacy_free_dev(mem);
		return res;
	}

	res = add_partitions(nor);
	if (!res)
		report(nor);

	return res;
}

driver_init(qemu_spi_nor_init);
