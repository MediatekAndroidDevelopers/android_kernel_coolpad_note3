#
# Makefile for MMC/SD card drivers
#

obj-$(CONFIG_MMC_BLOCK)		+= mmc_block.o
mmc_block-objs			:= block.o queue.o
obj-$(CONFIG_MMC_TEST)		+= mmc_test.o

obj-$(CONFIG_SDIO_UART)		+= sdio_uart.o

obj-$(CONFIG_MMC_FFU)		+= ffu.o

#yulong add for eMMC yl-params && yl_panic partitions 2015.12.17
obj-y				+= yl_params.o yl_panic.o
