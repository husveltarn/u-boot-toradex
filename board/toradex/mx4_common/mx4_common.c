/*
 * Copyright (c) 2011-2015 Host Mobility AB.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <errno.h>
#include <spi.h>

#define MX4_MAX_SPI_BYTES 8

int ping_mx4_pic(void)
{
	struct spi_slave *slave;

	unsigned int bus = 0, cs = 0, mode = SPI_MODE_1;
	int	bitlen = 56;
	int rcode = 0;
	char *empty = 0;

	uchar dout[] = { 0x02, 0x95, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uchar din[MX4_MAX_SPI_BYTES];

#ifdef CONFIG_DM_SPI
	char name[30], *str;
	struct udevice *dev;

	snprintf(name, sizeof(name), "generic_%d:%d", bus, cs);
	str = strdup(name);
	rcode = spi_get_bus_and_cs(bus, cs, 1000000, mode, "spi_generic_drv",
				 str, &dev, &slave);
	if (rcode)
		return rcode;
#else
	slave = spi_setup_slave(bus, cs, 1000000, mode);
	if (!slave) {
		printf("Invalid device %d:%d\n", bus, cs);
		return -EINVAL;
	}
#endif

	spi_claim_bus(slave);
	if(spi_xfer(slave, bitlen, dout, din, SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
		printf("Error during SPI transaction\n");
		rcode = 1;
	}
	spi_release_bus(slave);

	udelay(1000*10);

	bitlen = 24;
	/* Clock out response so that we are synced */
	spi_claim_bus(slave);
	if(spi_xfer(slave, bitlen, empty, din, SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
		printf("Error during SPI transaction\n");
		rcode = 1;
	}

	spi_release_bus(slave);
	spi_free_slave(slave);

	return rcode;
}