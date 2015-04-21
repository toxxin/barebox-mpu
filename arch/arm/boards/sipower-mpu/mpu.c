/*
 * (C) 2009 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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
 *
 */

#include <common.h>
#include <init.h>
#include <driver.h>
#include <gpio.h>
#include <environment.h>
#include <mach/imx25-regs.h>
#include <asm/armlinux.h>
#include <asm/sections.h>
#include <asm/barebox-arm.h>
#include <io.h>
#include <partition.h>
#include <generated/mach-types.h>
#include <mach/imx-nand.h>
#include <fec.h>
#include <nand.h>
#include <spi/spi.h>
#include <spi/eeprom.h>
#include <mach/imx-flash-header.h>
#include <mach/iomux-mx25.h>
#include <mach/generic.h>
#include <mach/iim.h>
#include <linux/err.h>
#include <mach/devices-imx25.h>
#include <asm/barebox-arm-head.h>
#include <mach/esdctl.h>

static struct fec_platform_data fec_info = {
	.xcv_type	= PHY_INTERFACE_MODE_RMII,
	.phy_addr	= 1,
};

struct imx_nand_platform_data nand_info = {
	.width	= 1,
	.hw_ecc	= 1,
};

static struct spi_eeprom at25_data = {
	.name = "at25x0",
	.page_size = 16,
	.flags = EE_ADDR1 | EE_READONLY,
	.size = 256,
};

static int spi_0_cs[] = {
	IMX_GPIO_NR(1, 16),
	IMX_GPIO_NR(1, 17)
};
static int spi_2_cs[] = {IMX_GPIO_NR(1, 31)};

static struct spi_imx_master spi_0_data = {
	.chipselect = spi_0_cs,
	.num_chipselect = ARRAY_SIZE(spi_0_cs),
};

static struct spi_imx_master spi_2_data = {
	.chipselect = spi_2_cs,
	.num_chipselect = ARRAY_SIZE(spi_2_cs),
};

static const struct spi_board_info mx25_spi_board_info[] = {
	{
		.name = "m25p80",
		.max_speed_hz = 66000000,
		.bus_num = 0,
		.chip_select = 1,
	},
	{
		.name = "at25x",
		.max_speed_hz = 10000000,
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &at25_data,
	}
};

static int imx25_mpu_fec_init(void)
{
	/* 
	 * This part was copied from 3dstack board file and 
	 * keeps here to support fec
	 */ 
	return 0;
}
late_initcall(imx25_mpu_fec_init);

static int imx25_mem_init(void)
{
#if defined CONFIG_SIPOWER_MX25_MPU_SDRAM_16MB_SDRAM
    arm_add_mem_device("ram0", MX25_CSD0_BASE_ADDR, SZ_16M);
#elif defined CONFIG_SIPOWER_MX25_MPU_SDRAM_32MB_SDRAM
    arm_add_mem_device("ram0", MX25_CSD0_BASE_ADDR, SZ_32M);
#else
#error "Unsupported SDRAM type"
#endif
    add_mem_device("sram0", 0x78000000, 128 * 1024, IORESOURCE_MEM_WRITEABLE);

    return 0;
}
mem_initcall(imx25_mem_init);

static int imx25_sdram_fixup(void)
{
    imx_esdctl_disable();    

    return 0;
}
postcore_initcall(imx25_sdram_fixup); 


static int imx25_mpu_devices_init(void)
{
	imx25_iim_register_fec_ethaddr();
	imx25_add_fec(&fec_info);

	if (readl(MX25_CCM_BASE_ADDR + MX25_CCM_RCSR) & (1 << 14))
		nand_info.width = 2;

	gpio_direction_output(IMX_GPIO_NR(1, 17), 1);
	gpio_direction_output(IMX_GPIO_NR(1, 31), 1);
	spi_register_board_info(mx25_spi_board_info,
			ARRAY_SIZE(mx25_spi_board_info));
	imx25_add_spi0(&spi_0_data);
	imx25_add_spi2(&spi_2_data);

	devfs_add_partition("m25p0", 0x00000, 0x20000, DEVFS_PARTITION_FIXED, "barebox");
	devfs_add_partition("m25p0", 0x20000, 0x800000, DEVFS_PARTITION_FIXED, "firmware");

	armlinux_set_bootparams((void *)0x80000100);
	armlinux_set_architecture(MACH_TYPE_MX25_3DS);
	//armlinux_set_architecture(MACH_TYPE_MX25_MPU);
	armlinux_set_serial(imx_uid());

	return 0;
}

device_initcall(imx25_mpu_devices_init);

static iomux_v3_cfg_t imx25_pads[] = {
	MX25_PAD_FEC_MDC__FEC_MDC,
	MX25_PAD_FEC_MDIO__FEC_MDIO,
	MX25_PAD_FEC_RDATA0__FEC_RDATA0,
	MX25_PAD_FEC_RDATA1__FEC_RDATA1,
	MX25_PAD_FEC_RX_DV__FEC_RX_DV,
	MX25_PAD_FEC_TDATA0__FEC_TDATA0,
	MX25_PAD_FEC_TDATA1__FEC_TDATA1,
	MX25_PAD_FEC_TX_CLK__FEC_TX_CLK,
	MX25_PAD_FEC_TX_EN__FEC_TX_EN,
	MX25_PAD_A18__FEC_COL,
	MX25_PAD_A19__FEC_RX_ER,
	MX25_PAD_POWER_FAIL__POWER_FAIL,
	MX25_PAD_A17__GPIO_2_3,
	MX25_PAD_D12__GPIO_4_8,
	/* UART1 */
	MX25_PAD_UART1_RXD__UART1_RXD,
	MX25_PAD_UART1_TXD__UART1_TXD,
	MX25_PAD_UART1_RTS__UART1_RTS,
	MX25_PAD_UART1_CTS__UART1_CTS,
	/* spi1 */
	MX25_PAD_CSPI1_MOSI__CSPI1_MOSI,
	MX25_PAD_CSPI1_MISO__CSPI1_MISO,
	MX25_PAD_CSPI1_SCLK__CSPI1_SCLK,
	MX25_PAD_CSPI1_RDY__CSPI1_RDY,
	/* do not use hardware cs support, treat as gpio */
	MX25_PAD_CSPI1_SS0__GPIO_1_16,
	MX25_PAD_CSPI1_SS1__GPIO_1_17,
};

static int imx25_console_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(imx25_pads, ARRAY_SIZE(imx25_pads));

	writel(0x03010101, 0x53f80024);

	barebox_set_model("Sipower i.MX25 MPU");
	barebox_set_hostname("mx25-mpu");

	imx25_add_uart0();
	return 0;
}

console_initcall(imx25_console_init);

static int imx25_core_setup(void)
{
	writel(0x01010103, MX25_CCM_BASE_ADDR + MX25_CCM_PCDR2);
	return 0;

}
core_initcall(imx25_core_setup);
