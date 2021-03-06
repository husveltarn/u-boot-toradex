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
#include <errno.h>
#include <spi.h>

#define REG_BIT(bit)                    (1UL << (bit))

#define MX4_PRC_POWER_ON_RESET              REG_BIT(0)
#define MX4_PRC_BROWN_OUT_RESET             REG_BIT(1)
#define MX4_PRC_IDLE                        REG_BIT(2)
#define MX4_PRC_SLEEP                       REG_BIT(3)
#define MX4_PRC_WDTO                        REG_BIT(4)
#define MX4_PRC_SWR                         REG_BIT(5)
#define MX4_PRC_MCLR                        REG_BIT(6)
#define MX4_PRC_CONFIG_MISMATCH             REG_BIT(7)
#define MX4_PRC_DEEP_SLEEP                  REG_BIT(8)
#define MX4_PRC_ILLEGAL_OPCODE_RESET        REG_BIT(9)
#define MX4_PRC_TRAP_CONFLICT_RESET         REG_BIT(10)
/* Below bits are fabricated and set by software on trap interrupts */
#define MX4_PRC_TRAP_DEFAULT				REG_BIT(11)
#define MX4_PRC_TRAP_OSC					REG_BIT(12)
#define MX4_PRC_TRAP_ADDRESS				REG_BIT(13)
#define MX4_PRC_TRAP_STACK					REG_BIT(14)
#define MX4_PRC_TRAP_MATH					REG_BIT(15)
#define MX4_PRC_PROTOCOL_WATCHDOG			REG_BIT(16)
#define MX4_PRC_CLOCK_SWITCH_FAILED			REG_BIT(17)
#define MX4_PRC_DEEP_SLEEP_EXIT				REG_BIT(18)

#define MX4_MAX_SPI_BYTES 8

#define MX4_CMD_READ				0x01
#define MX4_CMD_WRITE				0x02

#define MX4_CMD_SW_RESET			0x01
#define MX4_CMD_PING				0x95
#define MX4_CMD_RESET_CAUSE		    0x83

enum {
	MX4_SYS_FMS_BOOTING = 0,
    MX4_SYS_FMS_RUNNING,
    MX4_SYS_FMS_UPDATING,
    MX4_SYS_FMS_ERROR,
    MX4_SYS_FMS_RECOVERY_MODE,
};

enum {
	MX4_PROT_TYPE,
	MX4_PROT_CMD,
	MX4_PROT_DATA_OFFSET,
	MX4_PROT_CRC_OFFSET = 6,
};

/* 	I am aware of that there is a possibility that there might be multiple bits
	set, at least on the hardware bits. But since we print the raw hex value as
	well we can ignore it in this simple helper converter and just print the
	first bit we see.
*/
static const char* mx4_pic_str_reset_cause(int reset_cause)
{
	switch(reset_cause) {
		case MX4_PRC_POWER_ON_RESET:
			return "Power on reset";
		break;
		case MX4_PRC_BROWN_OUT_RESET:
			return "Brown out reset";
		break;
		case MX4_PRC_IDLE:
			return "Idle reset";
		break;
		case MX4_PRC_SLEEP:
			return "Sleep reset";
		break;
		case MX4_PRC_WDTO:
			return "Watchdog reset";
		break;
		case MX4_PRC_SWR:
			return "Software reset";
		break;
		case MX4_PRC_MCLR:
			return "MCLR reset";
		break;
		case MX4_PRC_CONFIG_MISMATCH:
			return "Config missmatch reset";
		break;
		case MX4_PRC_DEEP_SLEEP:
			return "Deep sleep reset";
		break;
		case MX4_PRC_ILLEGAL_OPCODE_RESET:
			return "Illegal opcode reset";
		break;
		case MX4_PRC_TRAP_CONFLICT_RESET:
			return "Trap conflict reset";
		break;
		case MX4_PRC_TRAP_MATH:
			return "Trap Math reset";
		break;
		case MX4_PRC_TRAP_STACK:
			return "Trap Stack reset";
		break;
		case MX4_PRC_TRAP_ADDRESS:
			return "Trap Address reset";
		break;
		case MX4_PRC_TRAP_OSC:
			return "Trap OSC reset";
		break;
		case MX4_PRC_TRAP_DEFAULT:
			return "Trap Default reset";
		break;
		case MX4_PRC_PROTOCOL_WATCHDOG:
			return "Protocol Watchdog reset";
		break;
		case MX4_PRC_CLOCK_SWITCH_FAILED:
			return "Clock Switch Failed";
		break;
		case MX4_PRC_DEEP_SLEEP_EXIT:
			return "Deep Sleep Exit";
		break;
		default:
			return "Unknown reset cause";
		break;
	}
}

static const char* mx4_pic_str_system_state(uint32_t state)
{
	switch(state) {
		case MX4_SYS_FMS_ERROR:
			return "Error";
		break;
		case MX4_SYS_FMS_BOOTING:
			return "Booting";
		break;
		case MX4_SYS_FMS_RUNNING:
			return "Running";
		break;
		case MX4_SYS_FMS_UPDATING:
			return "Updating";
		break;
		case MX4_SYS_FMS_RECOVERY_MODE:
			return "Recovery mode";
		break;
		default:
			return "Unknown";
		break;
	}
}

static uchar make8(uint32_t var, uchar offset)
{
	return (uchar) ((var >> (offset * 8)) & 0xff);
}

static uchar status_ok(const uchar* buffer)
{
	if (!buffer)
		return 0;

	return (buffer[2] == 0x01);
}

static void mx4_print_buffer(const uchar *buf, int length)
{
	int j;

	if (!buf)
		return;

	for (j = 0; j < length; j++) {
		printf("%02X", buf[j]);
	}
	printf("\n");
}

static uchar mx4_checksum_accumulate(const void* buffer, unsigned length)
{
	uchar result = 0;
	const uchar* buff = (const uchar*) buffer;
	unsigned i;

	if (!buffer)
		return 0;

	for (i = 0; i < length; ++i, ++buff) {	result += *buff; }

	return (0 - result);
}

static uchar mx4_calculate_checksum(const void* buffer, unsigned length)
{
	uchar result = 0;
	const uchar* buff = (const uchar*) buffer;
	unsigned i;

	if (!buffer)
		return 0;

	for (i = 0; i < length; ++i, ++buff) {	result += *buff; }

	return result;
}

static uchar mx4_checksum_fail (const void* buffer, unsigned length,
	uchar checksum)
{
	if (!buffer)
		return 0;

	return checksum + mx4_calculate_checksum (buffer, length);
}

/* data must always be a valid pointer */
static int mx4_spi_write(uchar type, uchar cmd, uint32_t *data)
{
	struct spi_slave *slave;

	uchar dout[MX4_MAX_SPI_BYTES] = { 0 };
	uchar din[MX4_MAX_SPI_BYTES] = { 0 };
	unsigned int bus = 0, cs = 0, mode = SPI_MODE_1, dout_bitlen = 0;
	unsigned int din_bitlen = 0;
	int rcode = 0;

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

	if (!data)
		return 1;

	dout[MX4_PROT_TYPE] = type;
	dout[MX4_PROT_CMD] = cmd;

	if (type == MX4_CMD_WRITE) {
		dout_bitlen = 56;
		din_bitlen = 24;

		dout[MX4_PROT_DATA_OFFSET] = make8(*data, 0);
		dout[MX4_PROT_DATA_OFFSET + 1] = make8(*data, 1);
		dout[MX4_PROT_DATA_OFFSET + 2] = make8(*data, 2);
		dout[MX4_PROT_DATA_OFFSET + 3] = make8(*data, 3);

		dout[MX4_PROT_CRC_OFFSET] = mx4_checksum_accumulate(dout + MX4_PROT_DATA_OFFSET, 4);
	} else if(type == MX4_CMD_READ) {
		dout_bitlen = 16;
		din_bitlen = 64;
	}

	printf(" >> ");
	mx4_print_buffer(dout, dout_bitlen/8);

	spi_claim_bus(slave);
	if (spi_xfer(slave, dout_bitlen, dout, din, SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
		printf("Error during SPI transaction\n");
		rcode = 1;
	}
	spi_release_bus(slave);

	udelay(1000*10);

	/* Clock out response so that we are synced */
	spi_claim_bus(slave);
	if (spi_xfer(slave, din_bitlen, NULL, din, SPI_XFER_BEGIN | SPI_XFER_END) != 0) {
		printf("Error during SPI transaction\n");
		rcode = 1;
	}

	printf(" << ");
	mx4_print_buffer(din, din_bitlen/8);

	/* 0x01 Software reset command does not have a response */
	if (!status_ok(din) && dout[1] != 0x01) {
		printf("Command response not OK: 0x%x\n", din[2]);
		rcode = 1;
	}

	if(type == MX4_CMD_READ) {
		*data |= (din[6] << 24);
		*data |= (din[5] << 16);
		*data |= (din[4] << 8);
		*data |= (din[3] << 0);

		if (mx4_checksum_fail(din + 2, 5, din[7])) {
			printf("CRC error\n");
			return 1;
		}
	}

	spi_release_bus(slave);
#ifndef CONFIG_DM_SPI
	spi_free_slave(slave);
#endif

	return rcode;
}

static int mx4_pic_ping(void)
{
	uint32_t data = 0;
	return mx4_spi_write(MX4_CMD_WRITE, MX4_CMD_PING, &data);
}

static int mx4_pic_is_hw_reset(void)
{
	uint32_t reset_cause = 0;

	if (mx4_spi_write(MX4_CMD_READ, MX4_CMD_RESET_CAUSE, &reset_cause))
		return 0;

	printf("PIC reset cause: %s\n", mx4_pic_str_reset_cause(reset_cause));

	/* External reset button */
	if (reset_cause & MX4_PRC_MCLR) {
		printf("Hardware reset detected.\n");
		return 0;
	}

	return 1;
}

static int mx4_pic_software_restart(void)
{
	uint32_t data = 0;
	uint8_t retry_cnt = 10;
	int ret = 0;

	/* SW reset command does not generate response */
	mx4_spi_write(MX4_CMD_WRITE, MX4_CMD_SW_RESET, &data);

	/* Wait for PIC to restart
	*  Restart time according to documentation is around 65 ms
	*/
	udelay(1000*100);

	while ((ret = mx4_pic_ping()) && retry_cnt) {
		udelay(1000*100);
		retry_cnt--;
	}

	return ret;
}

static int mx4_pic_set_system_state(uint32_t state)
{
	mx4_spi_write(MX4_CMD_WRITE, MX4_CMD_PING, &state);

	/* Always return success, Pic might not have application and we do not
	want to signal fail on led blink behaviour. */
	return 0;
}

static int do_mx4_pic(cmd_tbl_t *cmdtp, int flag, int argc,
		       char * const argv[])
{
	if (argc < 2) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	if (strncmp(argv[1], "ping", 4) == 0) {
		return mx4_pic_ping();
	}

	if (strncmp(argv[1], "is_extr", 7) == 0) {
		return mx4_pic_is_hw_reset();
	}

	if (strncmp(argv[1], "restart", 7) == 0) {
		return mx4_pic_software_restart();
	}

	if (strncmp(argv[1], "set_state", 9) == 0) {
		if (argc < 3) {
			printf("Usage:\n%s\n", cmdtp->usage);
			return 1;
		}

		uint32_t state = simple_strtoul(argv[2], NULL, 10);
		printf("System state: %s\n", mx4_pic_str_system_state(state));

		return mx4_pic_set_system_state(state);
	}

	printf("Usage:\n%s\n", cmdtp->usage);
	return 1;
}

U_BOOT_CMD(
	mx4_pic, 4, 0, do_mx4_pic,
	"mx4_pic <ping, is_extr, restart, set_state> <state>\n",
	"mx4_pic ping - Ping co-cpu to let it know we are alive\n"
	"mx4_pic is_extr - Is system reset by external reset button\n"
	"mx4_pic restart - Do a software reset on pic\n"
	"mx4_pic set_state <n> - Do a software reset on pic"
);
