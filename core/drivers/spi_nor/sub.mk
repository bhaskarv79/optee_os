srcs-y += core.c
srcs-y += ids.c
srcs-y += partition.c
srcs-$(CFG_SPI_NOR_RPMC) += rpmc.c
srcs-$(CFG_SPI_NOR_RPMC) += rpmc_counter.c
srcs-y += sfdp.c
srcs-$(CFG_SPI_NOR_SIM) += spi_nor_sim.c
srcs-$(CFG_SPI_NOR_EMBEDDED_TEST) += spi_nor_selftest.c
