global-incdirs-y += .
srcs-y += main.c
srcs-$(CFG_QEMU_SPI_NOR) += spi_nor_qemu.c
ifeq ($(PLATFORM_FLAVOR_juno),y)
srcs-$(CFG_ARM32_core) += juno_core_pos_a32.S
srcs-$(CFG_ARM64_core) += juno_core_pos_a64.S
endif
