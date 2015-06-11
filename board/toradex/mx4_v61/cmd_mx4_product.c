/*
* Copyright (c) 2014, Host Mobility AB.  All rights reserved.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <common.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_TEGRA2
static const char* str_ram_size(void)
{
	if (gd->ram_size == 0x10000000) {
		/* 256 MB */
		return "256";
	} else {
		/* 512 MB */
		return "512";
	}
	return "NONE";
}

static int validate_tegra_ram_size(const char *target)
{
	if (strcmp(target, "NONE") == 0)
		return 0;

	if (gd->ram_size == 0x10000000) {
		/* 256 MB */
		if (strcmp(target, "512") == 0)
			return 1;
	} else {
		/* 512 MB */
		if (strcmp(target, "256") == 0)
			return 1;
	}

	return 0;
}
#endif /* CONFIG_TEGRA2 */

static int do_mx4_product(cmd_tbl_t *cmdtp, int flag, int argc,
		       char * const argv[])
{
	if (argc < 2) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 0;
	}

#ifdef CONFIG_TEGRA2
	if (argc == 3) {
		if (validate_tegra_ram_size(argv[2])) {
			printf("ram size missmatch - expected: %s, target: %s\n", argv[2],
				str_ram_size());
			return 1;
		}
	}
#endif /* CONFIG_TEGRA2 */

	if (strcmp(argv[1], MX4_PRODUCT_TYPE) != 0) {
		printf("product missmatch - expected: %s, target: %s\n",
			argv[1], MX4_PRODUCT_TYPE);
		return 1;
	}

	return 0;
}

U_BOOT_CMD(
	mx4_product, 3, 0, do_mx4_product,
	"mx4_prouct <target system type> <target system variant>",
	"Match MX-4 product type to expected MX-4 type"
);
