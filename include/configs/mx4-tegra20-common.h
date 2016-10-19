/*
 * Copyright (c) 2016 Host Mobility AB
 *
 * Configuration settings for the Toradex Colibri T20 SOM
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _MX4_COMMON_H_
#define _MX4_COMMON_H_

#define CONFIG_NO_DISTRO_BOOTCMD
#include "tegra20-common.h"

#undef CONFIG_SYS_DCACHE_OFF		/* breaks L4T kernel boot */
#define CONFIG_ARCH_MISC_INIT

#define CONFIG_TEGRA_EARLY_TPS6586X

#define CONFIG_CUSTOM_BOARDINFO		/* not from device-tree model node */
#undef CONFIG_DISPLAY_BOARDINFO
#define CONFIG_DISPLAY_BOARDINFO_LATE

/* Board-specific serial config */
#define CONFIG_SERIAL_MULTI
#define CONFIG_TEGRA_ENABLE_UARTA
#define CONFIG_TEGRA_UARTA_SDIO1
#define CONFIG_SYS_NS16550_COM1		NV_PA_APB_UARTA_BASE

#define CONFIG_MACH_TYPE		MACH_TYPE_COLIBRI_T20

#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SERIAL_TAG

/* Make the HW version stuff available in U-Boot env */
#define CONFIG_VERSION_VARIABLE		/* ver environment variable */
#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_CMD_ASKENV

/* I2C */
#define CONFIG_SYS_I2C_TEGRA
#define CONFIG_CMD_I2C

 /* SPI */
 #define CONFIG_TEGRA20_SLINK
 #define CONFIG_CMD_SPI

/* SD/MMC support */
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_TEGRA_MMC
#define CONFIG_CMD_MMC

/* USB host support */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_TEGRA
#define CONFIG_USB_ULPI
#define CONFIG_USB_ULPI_VIEWPORT
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
#define CONFIG_TFTP_BLOCKSIZE		1536
#define CONFIG_TFTP_TSIZE

/* NAND support */
#define CONFIG_CMD_NAND
#define CONFIG_TEGRA_NAND
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* Dynamic MTD partition support */
#define CONFIG_CMD_MTDPARTS	/* Enable 'mtdparts' command line support */
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE	/* needed for mtdparts commands */

#undef CONFIG_CMD_IMLS

/* Environment in NAND, 64K is a bit excessive but erase block is 512K anyway */
#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_OFFSET		(SZ_2M)
#undef CONFIG_ENV_SIZE	/* undef size from tegra20-common.h */
#define CONFIG_ENV_SIZE			(SZ_64K)
#define CONFIG_ENV_RANGE		(SZ_1M)

#define CONFIG_RBTREE
#define CONFIG_LZO
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_UBI
#define CONFIG_MTD_UBI_FASTMAP
#define CONFIG_CMD_UBIFS	/* increases size by almost 60 KB */

/* Debug commands */
#define CONFIG_CMD_BDI
#define CONFIG_CMD_CACHE

/* Miscellaneous commands */
#define CONFIG_CMD_SETEXPR
#define CONFIG_FAT_WRITE

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

#define CONFIG_OF_LIBFDT
#define CONFIG_OF_SYSTEM_SETUP

#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_SYS_BOOT_RAMDISK_HIGH

#define HM_UPDATE_FILE_NAME	"hmupdate.img"

/* Note! HUSH scripting needs to be enabled for bootcommand/autoscripting */
#define UBI_BOOTCMD \
	"ubiboot=run setup; setenv bootargs ${defargs} ${hmargs} " \
	"${ubiargs} ${mtdparts} ${setupargs} ${vidargs}; " \
	"mx4_pic restart; " \
	"nboot ${kernel_addr_r} 0 ${kernel_addr_nand} && bootm ${kernel_addr_r}"

#define PROBE_USB_FOR_HMUPDATE \
	"if run is_firmware_update || mx4_pic is_extr; then " \
		"usb start; " \
		"fatls usb 0:1; " \
		"mx4_pic set_state 2; " \
		"fatload usb 0:1 ${loadaddr} ${updatefilename}; " \
	"fi; "

#define PROBE_UBI_FOR_HMUPDATE \
	"if ${firmware_update} -eq true; then " \
		"ubi part ubi; " \
		"ubifsmount ubi0:rootfs; " \
		"ubifsload ${loadaddr} /boot/${updatefilename}; " \
		"mx4_pic set_state 2; " \
		"ubifsumount; " \
	"fi; "

#define PROBE_USB_FOR_RAMDISK \
	"if mx4_pic is_extr; then " \
		"usb start; " \
		"fatload usb 0:1 ${loadaddr} ${kernelfilename}; " \
		"fatload usb 0:1 ${ramdisk_loadaddr} ${ramdiskfilename}; " \
		"run ramboot; " \
	"fi; "

#define IS_FIRMWARE_UPDATE \
	"if ${firmware_update} -eq true; then true; fi"

#define CONFIG_BOOTCOMMAND \
	"if run probe_usb || run probe_ubi; then " \
		"if source ${loadaddr}; then " \
			"exit; " \
		"else " \
			"bootm ${loadaddr}; " \
		"fi; " \
	"fi; " \
	"run probe_ramdisk; run ubiboot;"

#define CONFIG_COMMON_EXTRA_ENV_SETTINGS \
	"firmware_update=false\0" \
	"probe_usb=" PROBE_USB_FOR_HMUPDATE "\0" \
	"probe_ubi=" PROBE_UBI_FOR_HMUPDATE "\0" \
	"probe_ramdisk=" PROBE_USB_FOR_RAMDISK "\0" \
	"is_firmware_update=" IS_FIRMWARE_UPDATE "\0" \
	"ramargs=root=/dev/ram0 rw\0" \
	"ramdiskfilename=uRamdisk\0" \
	"ramdisk_loadaddr=1000000\0" \
	"ramargs=root=/dev/ram0 rw\0" \
	"updatefilename=" HM_UPDATE_FILE_NAME "\0" \
	"kernelfilename=uImage\0" \
	"defargs=vmalloc=128M usb_high_speed=1 quiet\0" \
	"console=ttyS0\0" \
	"setup=setenv setupargs asix_mac=${ethaddr} g_android.dev_addr=${rndis_dev_mac}" \
		"consoleblank=0 no_console_suspend=1 console=tty1 " \
		"console=${console},${baudrate}n8 debug_uartport=lsport,0 " \
		"${memargs}\0" \
	"ubiargs=ubi.mtd=ubi root=ubi0:rootfs rootfstype=ubifs "	\
		"ubi.fm_autoconvert=1\0" \
	"mtdparts=" MTDPARTS_DEFAULT "\0" \
	UBI_BOOTCMD "\0" \

/* Miscellaneous configurable options */
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT		\
	"\nMX4 - booting... stop with ENTER\n"
#define CONFIG_AUTOBOOT_DELAY_STR	"\r"
#define CONFIG_AUTOBOOT_DELAY_STR2	"\n"
#define CONFIG_RESET_TO_RETRY
#define CONFIG_BOOT_RETRY_TIME	30

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY	0
#define CONFIG_ZERO_BOOTDELAY_CHECK
#undef CONFIG_IPADDR
#define CONFIG_IPADDR		192.168.10.2
#define CONFIG_NETMASK		255.255.255.0
#undef CONFIG_SERVERIP
#define CONFIG_SERVERIP		192.168.10.1

#endif