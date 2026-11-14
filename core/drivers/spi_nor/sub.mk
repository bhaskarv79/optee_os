srcs-y += core.c
srcs-$(CFG_SPI_NOR_GPT) += gpt.c
srcs-y += ids.c
srcs-y += partition.c
srcs-y += sfdp.c
srcs-$(CFG_SPI_NOR_SIM) += spi_nor_sim.c
srcs-$(CFG_SPI_NOR_EMBEDDED_TEST) += spi_nor_selftest.c
