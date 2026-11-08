// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * Secure SPI-NOR flash of the QEMU virt machine (-machine virt,spi=secure):
 * a PL022 SPI controller with a JESD260 RPMC capable flash on chip select
 * 0, driven by line 0 of a dedicated PL061. OP-TEE drives the PL022 through
 * the legacy spi-mem adapter and declares the flash partitions here, as a
 * secure DT would.
 */

#include <drivers/pl022_spi.h>
#include <drivers/pl061_gpio.h>
#include <drivers/spi_mem.h>
#include <drivers/spi_nor.h>
#include <initcall.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <trace.h>

#define QEMU_SPI_BASE		0x090c0000
#define QEMU_SPI_GPIO_BASE	0x090d0000
#define QEMU_SPI_CS_PIN		0
#define QEMU_SPI_CLK_HZ		24000000
#define QEMU_SPI_SPEED_HZ	12000000

register_phys_mem_pgdir(MEM_AREA_IO_SEC, QEMU_SPI_BASE, PL022_REG_SIZE);
register_phys_mem_pgdir(MEM_AREA_IO_SEC, QEMU_SPI_GPIO_BASE, PL061_REG_SIZE);

static struct pl061_data gpio;
static struct pl022_data spi;

static TEE_Result add_partitions(struct spi_nor *nor)
{
	return spi_nor_add_partition(nor, "optee-fs",
				     CFG_QEMU_SPI_NOR_FS_OFFSET,
				     CFG_QEMU_SPI_NOR_FS_SIZE, NULL);
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

	return add_partitions(nor);
}

driver_init(qemu_spi_nor_init);
