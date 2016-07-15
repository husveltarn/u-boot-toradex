/*
 * Copyright (c) 2012-2015 Toradex, Inc.
 * Copyright (c) 2016 Host Mobiltiy AB
 *
 * Configuration settings for the Toradex Colibri T30 on a MX-4 T30 carrier
 * board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <linux/sizes.h>

#include "tegra30-common.h"

#define CONFIG_ARCH_MISC_INIT

#define MX4_PRODUCT_TYPE "t30"

/* High-level configuration options */
#define V_PROMPT			"MX-4 T30 # "
#define CONFIG_CUSTOM_BOARDINFO		/* not from device-tree model node */
#undef CONFIG_DISPLAY_BOARDINFO
#define CONFIG_DISPLAY_BOARDINFO_LATE

/* Board-specific serial config */
#define CONFIG_SERIAL_MULTI
#define CONFIG_TEGRA_ENABLE_UARTA
#define CONFIG_SYS_NS16550_COM1		NV_PA_APB_UARTA_BASE

#define CONFIG_MACH_TYPE		MACH_TYPE_COLIBRI_T30

#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SERIAL_TAG

/* Make the HW version stuff available in U-Boot env */
#define CONFIG_VERSION_VARIABLE		/* ver environment variable */
#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_CMD_ASKENV

/* SPI */
#define CONFIG_TEGRA20_SLINK
#define CONFIG_CMD_SPI

/* I2C */
#define CONFIG_SYS_I2C_TEGRA
#define CONFIG_CMD_I2C

/* SD/MMC support */
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_TEGRA_MMC
#define CONFIG_CMD_MMC

/* Environment in eMMC, before config block at the end of 1st "boot sector" */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(-CONFIG_ENV_SIZE + \
					 CONFIG_TRDX_CFG_BLOCK_OFFSET)
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_SYS_MMC_ENV_PART		1

/* USB host support */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_TEGRA
#define CONFIG_USB_MAX_CONTROLLER_COUNT	3
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_USB

/* USB networking support */
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX

/* General networking support */
#define CONFIG_CMD_NET
#define CONFIG_CMD_DHCP
#define CONFIG_IP_DEFRAG
#define CONFIG_TFTP_BLOCKSIZE		16384
#define CONFIG_TFTP_TSIZE

/* Miscellaneous commands */
#define CONFIG_CMD_SETEXPR
#define CONFIG_FAT_WRITE

/* Miscellaneous commands */
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT		\
	"\nMX4 - booting... stop with ENTER\n"
#define CONFIG_AUTOBOOT_DELAY_STR	"\r"
#define CONFIG_AUTOBOOT_DELAY_STR2	"\n"
#define CONFIG_RESET_TO_RETRY
#define CONFIG_BOOT_RETRY_TIME	30

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY	1
#define CONFIG_ZERO_BOOTDELAY_CHECK
#undef CONFIG_IPADDR
#define CONFIG_IPADDR		192.168.10.2
#define CONFIG_NETMASK		255.255.255.0
#undef CONFIG_SERVERIP
#define CONFIG_SERVERIP		192.168.10.1

#define DFU_ALT_EMMC_INFO	"colibri_t30.img raw 0x0 0x500 mmcpart 1; " \
				"boot part 0 1 mmcpart 0; " \
				"rootfs part 0 2 mmcpart 0; " \
				"uImage fat 0 1 mmcpart 0; " \
				"tegra30-mx4-t30.dtb fat 0 1 mmcpart 0"

#define EMMC_BOOTCMD \
	"emmcargs=ip=off root=/dev/mmcblk0p4 rw,noatime rootfstype=ext3 " \
		"rootwait\0" \
	"emmcboot=mx4_pic restart; run setup; " \
		"setenv bootargs ${defargs} ${emmcargs} " \
		"${setupargs} ${vidargs}; echo Booting from internal eMMC " \
		"chip...; run emmcdtbload; load mmc 0:1 ${kernel_addr_r} " \
		"${boot_file} && bootm ${kernel_addr_r} - ${dtbparam}\0" \
	"emmcdtbload=setenv dtbparam; load mmc 0:1 ${fdt_addr_r} " \
		"${soc}-mx4-${fdt_board}.dtb && " \
		"setenv dtbparam ${fdt_addr_r}\0"

#define PROBE_USB_FOR_HMUPDATE \
	"if run is_firmware_update || mx4_pic is_extr; " \
	"then usb start && fatls usb 0:1 && " \
	"mx4_pic set_state 2 && fatload usb 0:1 ${loadaddr} ${updatefilename}; fi "

#define PROBE_EMMC_FOR_HMUPDATE \
	"if ${firmware_update} -eq true; then " \
	"mx4_pic set_state 2; " \
	"echo \"Loading /boot/${updatefilename}...\"; " \
	"load mmc 0:4 ${loadaddr} /boot/${updatefilename}; "\
	"fi "

#define IS_FIRMWARE_UPDATE \
	"if ${firmware_update} -eq true; then true; fi"

#define CONFIG_BOOTCOMMAND \
    "if run probe_usb || run probe_emmc; then " \
	    "if source ${loadaddr}; then " \
	    	"exit; " \
	    "else " \
	    	"bootm ${loadaddr}; " \
	    "fi; " \
    "fi; " \
    "run emmcboot;"

#define BOARD_EXTRA_ENV_SETTINGS \
    "firmware_update=false\0" \
	"probe_usb=" PROBE_USB_FOR_HMUPDATE "\0" \
	"probe_emmc=" PROBE_EMMC_FOR_HMUPDATE "\0" \
	"is_firmware_update=" IS_FIRMWARE_UPDATE "\0" \
	"updatefilename=t30_hmupdate.img\0" \
	"boot_file=uImage\0" \
	"console=ttyS0\0" \
	"defargs=core_edp_mv=1300 usb_high_speed=1 quiet isolcpus=1\0" \
	"dfu_alt_info=" DFU_ALT_EMMC_INFO "\0" \
	EMMC_BOOTCMD \
	"fdt_board=t30\0" \
	"setethupdate=if env exists ethaddr; then; else setenv ethaddr " \
		"00:14:2d:00:00:00; fi; usb start && tftpboot " \
		"${kernel_addr_r} flash_eth.img\0" \
	"setup=setenv setupargs asix_mac=${ethaddr} asix_mac2=${ethaddr2} " \
		"consoleblank=0 no_console_suspend=1 console=tty1 " \
		"console=${console},${baudrate}n8 debug_uartport=lsport,0 " \
		"vmalloc=128M mem=1012M@2048M fbmem=12M@3060M\0" \
	"setupdate=run setethupdate;" \
		" source ${kernel_addr_r}\0" \
	"vidargs=video=tegrafb0:640x480-16@60\0"

/* Increase console I/O buffer size */
#undef CONFIG_SYS_CBSIZE
#define CONFIG_SYS_CBSIZE		1024

/* Increase arguments buffer size */
#undef CONFIG_SYS_BARGSIZE
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

/* Increase print buffer size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

/* Increase maximum number of arguments */
#undef CONFIG_SYS_MAXARGS
#define CONFIG_SYS_MAXARGS		32

#define CONFIG_CMD_TIME
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST

#define CONFIG_OF_SYSTEM_SETUP

#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_SYS_BOOT_RAMDISK_HIGH

#include "tegra-common-post.h"

#endif /* __CONFIG_H */
