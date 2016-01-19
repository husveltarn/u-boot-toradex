/*
 * Copyright (C) 2014, Toradex AG
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * Helpers for i.MX OTP fusing during module production
*/

#include <common.h>
#include <fuse.h>

static int mfgr_fuse(void)
{
	unsigned val, val6;

	fuse_sense(0, 5, &val);
	printf("Fuse 0, 5: %8x\n", val);
	fuse_sense(0, 6, &val6);
	printf("Fuse 0, 6: %8x\n", val6);
	fuse_sense(4, 3, &val);
	printf("Fuse 4, 3: %8x\n", val);
	fuse_sense(4, 2, &val);
	printf("Fuse 4, 2: %8x\n", val);
	if(val6 & 0x10)
	{
		puts("BT_FUSE_SEL already fused, will do nothing\n");
		return CMD_RET_FAILURE;
	}
	/* boot cfg */
	fuse_prog(0, 5, 0x00005062);
	/* BT_FUSE_SEL */
	fuse_prog(0, 6, 0x00000010);
	return CMD_RET_SUCCESS;
}

int do_mfgr_fuse(cmd_tbl_t *cmdtp, int flag, int argc,
		char * const argv[])
{
	int ret;
	puts("Fusing...\n");
	ret = mfgr_fuse();
	if (ret == CMD_RET_SUCCESS)
		puts("done.\n");
	else
		puts("failed.\n");
	return ret;
}

U_BOOT_CMD(
	mfgr_fuse, 1, 0, do_mfgr_fuse,
	"OTP fusing during module production\n",
	""
);
