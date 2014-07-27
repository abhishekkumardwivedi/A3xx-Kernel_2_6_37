/*
 * linux/arch/arm/mach-omap2/board-a31x.c
 *
 * (C) Copyright 2012 - 2014 Micronet Ltd <http://www.micronet.co.il>
 * Vladimir Zatulovsky, vladimirz@micronet.co.il
 * A31x board
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/automotiveio.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>
#include <linux/usb/android_composite.h>
#ifdef CONFIG_TOUCHSCREEN_ADS7846
#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#endif
#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <linux/i2c.h>
#include <linux/mfd/twl4030-codec.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>

#ifdef CONFIG_WL12XX_PLATFORM_DATA
#include <linux/wl12xx.h>
#include <linux/regulator/fixed.h>
#endif

#ifdef CONFIG_TI_ST
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#endif

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "board-flash.h"

#ifdef CONFIG_SND_SOC_WL1271BT
#include "control.h"
#endif

#define NAND_BLOCK_SIZE		SZ_128K

static char *identlcd = 0;
module_param(identlcd, charp, S_IRUGO);
MODULE_PARM_DESC(identlcd, "Ident LCD");

void *boot_fbuff;//

int get_ident_lcd(void)
{
	if(NULL == identlcd)
		return -1;
	return *identlcd;
}
EXPORT_SYMBOL(get_ident_lcd);
#if 0
//TODO
#define GPIO_TX_EN	162
int a31x_txter_enable(int set_on)
{
	if(set_on)
		gpio_set_value(GPIO_TX_EN, 1);
	else
		gpio_set_value(GPIO_TX_EN, 0);
}
static void a31x_txter_init(void)
{
	int console = //check console enabled
	if(gpio_request(GPIO_TX_EN, "a31x_uart") < 0)
	{
		printk("gpio_request %d failed\n", GPIO_TX_EN);
		return;
	}

	gpio_direction_output(GPIO_TX_EN, 1);
	if(console)//check console enabled
		a31x_enable_uart(GPIO_TX_EN, 1);
	else
		a31x_enable_uart(GPIO_TX_EN);
}
#endif
extern void omap_pm_sys_offmode_select(int);
extern void omap_pm_sys_clkreq_pol(int);
extern void omap_pm_sys_offmode_pol(int);
extern void omap_pm_auto_off(int);
extern void omap_pm_auto_ret(int);
static void __init
omap3_a31x_pm_init(void)
{
    /* Use sys_offmode signal */
    omap_pm_sys_offmode_select(1);

    /* sys_clkreq - active high */
    omap_pm_sys_clkreq_pol(1);

    /* sys_offmode - active low */
    omap_pm_sys_offmode_pol(0);

    /* Automatically send OFF command */
    omap_pm_auto_off(1);

    /* Automatically send RET command */
    omap_pm_auto_ret(1);

    /*initialize sleep relational register for PM components*/

    omap_pm_sys_offmode_pol(0);
    omap_pm_sys_clkreq_pol(1);

    omap_pm_sys_offmode_select(1);
    omap_pm_auto_ret(1);
    omap_pm_auto_off(1);
}

#ifdef CONFIG_TOUCHSCREEN_ADS7846
#define PEN_nIRQ_GPIO 103

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(PEN_nIRQ_GPIO);
}

static struct ads7846_platform_data ads7846_config = {
	.x_max			    = 0x0fff,
	.y_max			    = 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 1,
	.debounce_tol		= 85,
	.debounce_rep		= 3, //L4B from 1 to 2. //30 was good.
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,//4,
	.wakeup				= 0,
	.model 				= 7845,
};

static struct spi_board_info a31x_spi_board_info[] = {
	[0] = {
			.modalias			= "ads7846",
			.bus_num			= 1,
			.chip_select		= 0,
			.max_speed_hz		= 125000,
			.controller_data	= &ads7846_mcspi_config,
			.irq			    = OMAP_GPIO_IRQ(PEN_nIRQ_GPIO),
			.platform_data		= &ads7846_config,
		  },
};

static void ads7846_pendown_init(void)
{
	printk("%s ads7846 pen down initialize\n", __func__);
	if (gpio_request(PEN_nIRQ_GPIO, "ads7846 pen down") < 0)
		printk(KERN_ERR "%s failure to request pen down gpio\n", __func__);

	gpio_direction_input(PEN_nIRQ_GPIO);
	gpio_set_debounce(PEN_nIRQ_GPIO, 0xa);
}

#else
static inline void __init
ads7846_pendown_init(void) { return; }
#endif

/*
 * PWMA register offsets (TWL4030_MODULE_PWMA)
 */
#define TWL_PWMAON	 		0x00
#define TWL_PWMAOFF	 		0x01
#define TWL4030_LED_LEDEN	0x0
u8 buttonlightlevel;

static int a31x_set_keylight_level(const char *inval, const struct kernel_param *kp)
{
	unsigned long val;
	char *end;
	int ret;
	static int fisrt_time = 1;
	u8 reg;

	if(fisrt_time)
	{
		twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &reg, TWL4030_REG_VIBRA_CTL);
		twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,(reg & ~TWL4030_VIBRA_EN), TWL4030_REG_VIBRA_CTL);

		fisrt_time = 0;
	}

	val = simple_strtoul(inval, &end, 0);

	if(val > 255)
		return ~EINVAL;

	ret = end + 1 - inval;
	if (ret == 0)
		ret = -EINVAL;

	twl_i2c_read_u8(TWL4030_MODULE_LED, &reg, TWL4030_LED_LEDEN);

    if(val){ //turn on
		reg |= 0x22;
		val = (0x7f >> 1);
    } else { //turn off
		//reg &= ~0x22;
		val = 0x7f;
    }

	twl_i2c_write_u8(TWL4030_MODULE_PWMB, 0,   TWL_PWMAON);
	twl_i2c_write_u8(TWL4030_MODULE_PWMB, val, TWL_PWMAOFF);
	twl_i2c_write_u8(TWL4030_MODULE_LED,  reg, TWL4030_LED_LEDEN);

	buttonlightlevel = (int)val;

	return ret;
}

int a31x_get_keylight_level(char *buffer, const struct kernel_param *kp)
{
	int result;

	result = sprintf(buffer,"%d", buttonlightlevel);

	return result;
}

static struct kernel_param_ops lightbutton_ops_str = {
	.set = a31x_set_keylight_level,
	.get = a31x_get_keylight_level,
};

module_param_cb(buttons_light, &lightbutton_ops_str, NULL, 0664);

static struct platform_device lightsensor_device = {
	.name          = "temt6200",
	.id            = -1,
	.dev            = {
		.platform_data = NULL,
	},
};

static int a31x_set_bl_intensity(struct omap_dss_device *dssdev, int level)
{
	unsigned char c;
	static int fisrt_time = 1;

	if (level > dssdev->max_backlight_level)
		level = dssdev->max_backlight_level;

	if (level < 9)
		level = 9;

	c = ((125 * level) / 100) + 1;
	if(fisrt_time)
	{
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x02, TWL_PWMAOFF); // off
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMAON);  // on
		fisrt_time = 0;
	}

	twl_i2c_write_u8(TWL4030_MODULE_PWMA, c, TWL_PWMAOFF);

	return 0;
}

char previous_bkl_duty_cycle = 126;

static int a31x_enable_lcd(struct omap_dss_device *dssdev)
{
	u8 en;
	// Configure the duty cycle of LEDA
	printk("%s\n", __func__);
	twl_i2c_write_u8(TWL4030_MODULE_PWMA, previous_bkl_duty_cycle, TWL_PWMAOFF); // off
	twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMAON);  // on
	twl_i2c_read_u8(TWL4030_MODULE_LED, &en, TWL4030_LED_LEDEN);
	twl_i2c_write_u8(TWL4030_MODULE_LED, en | 0x11, TWL4030_LED_LEDEN);

	return 0;
}

static void a31x_disable_lcd(struct omap_dss_device *dssdev)
{
	u8 en;

	printk("%s\n", __func__);
	twl_i2c_read_u8(TWL4030_MODULE_PWMA, &previous_bkl_duty_cycle, TWL_PWMAOFF);
	if(previous_bkl_duty_cycle < 10)
		previous_bkl_duty_cycle = 10;

	twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x00, TWL_PWMAOFF); // off
	twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x7F, TWL_PWMAON); // on

	twl_i2c_read_u8(TWL4030_MODULE_LED, &en, TWL4030_LED_LEDEN);
	twl_i2c_write_u8(TWL4030_MODULE_LED, en & (~0x11), TWL4030_LED_LEDEN);
}

static struct omap_dss_device a31x_lcd_device = {
	.name				= "lcd",
	.driver_name		= "a31x_panel",
	.type				= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.max_backlight_level= 100,
	.platform_enable	= a31x_enable_lcd,
	.platform_disable	= a31x_disable_lcd,
	.set_backlight		= a31x_set_bl_intensity,
};

static struct mtd_partition a31x_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
    {
		.name       = "xloader",
		.offset     = 0,
		.size       = 4 * NAND_BLOCK_SIZE,
		.mask_flags = MTD_WRITEABLE  /* force read-only */
	},
	{
		.name		= "ubootone",
		.offset		=  4 * NAND_BLOCK_SIZE,	/* Offset = 0x80000 */
		.size		= 16 * NAND_BLOCK_SIZE, 	/* Offset = 0x180000 */
	},
	{
		.name		= "kernelone",
		.offset		= 20 * NAND_BLOCK_SIZE,	/* Offset = 0x280000 */
		.size		= 40 * NAND_BLOCK_SIZE, /* Offset = 0x780000 */
	},
	{
		.name		= "uboottwo",
		.offset		= 60 * NAND_BLOCK_SIZE,	/* Offset = 0x780000 */
		.size		= 8 * NAND_BLOCK_SIZE, 	/* Offset = 0x880000 */
	},
	{
		.name		= "kerneltwo",
		.offset		= 68 * NAND_BLOCK_SIZE,	/* Offset = 0x880000 */
		.size		= 40 * NAND_BLOCK_SIZE,	/* Offset = 0xD80000 */
	},
	{
		.name		= "rootfilesystem",
		.offset		=  108 * NAND_BLOCK_SIZE,  /* Offset = 0xD80000 */
		.size		= 1280 * NAND_BLOCK_SIZE, /* Offset = 0xAD7FFFF */
	},
	{
		.name		= "uservolume",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0xAD80000 */
		.size		= MTDPART_SIZ_FULL,
//		.mask_flags	= MTD_NO_ERASE,
	},
};

/* DSS
 * OMAP3 LCD Panel control signals
 */

static int a31x_enable_dvi(struct omap_dss_device *dssdev)
{
	int pwr_pin 	= (dssdev->reset_gpio >> 16) & 0xFFFF;
	int buf_en_pin 	= dssdev->reset_gpio & 0xFFFF;

	printk("%s\n", __func__);

	if(gpio_is_valid(pwr_pin))
		gpio_set_value(pwr_pin, 1);
	if(gpio_is_valid(buf_en_pin))
		gpio_set_value(buf_en_pin, 0);

	a31x_enable_lcd(dssdev);

	return 0;
}

static void a31x_disable_dvi(struct omap_dss_device *dssdev)
{
	int pwr_pin 	= (dssdev->reset_gpio >> 16) & 0xFFFF;
	int buf_en_pin 	= dssdev->reset_gpio & 0xFFFF;

	printk("%s\n", __func__);

	a31x_disable_lcd(dssdev);

	if(gpio_is_valid(buf_en_pin))
		gpio_set_value(buf_en_pin, 1);
	if(gpio_is_valid(pwr_pin))
		gpio_set_value(pwr_pin, 0);
}

static struct omap_dss_device a31x_dvi_device = {
	.type 				= OMAP_DISPLAY_TYPE_DPI,
	.name 				= "dvi",
	.driver_name 		= "generic_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio 		= (164 << 16) | 104, // power and LCD buffer enable pins, cannot be used with generic code
	.platform_enable 	= a31x_enable_dvi,
	.platform_disable 	= a31x_disable_dvi,
};

static struct omap_dss_device a31x_tv_device = {
	.name 			= "tv",
	.driver_name 	= "venc",
	.type 			= OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type 	= OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *a31x_dss_devices[] = {
	&a31x_lcd_device,
	&a31x_dvi_device,
	&a31x_tv_device,
};

static struct omap_dss_board_info a31x_dss_data = {
	.num_devices 	= ARRAY_SIZE(a31x_dss_devices),
	.devices 		= a31x_dss_devices,
	.default_device = &a31x_dvi_device,
};

static struct platform_device a31x_dss_device = {
	.name          	= "omapdss",
	.id            	= -1,
	.dev            = {
		.platform_data = &a31x_dss_data,
	},
};

static struct regulator_consumer_supply a31x_vdac_supply =	REGULATOR_SUPPLY("vdda_dac", "omapdss");
static struct regulator_consumer_supply a31x_vdvi_supply =	REGULATOR_SUPPLY("vdds_dsi", "omapdss");

static void __init
a31x_display_init(void)
{
	int r;

	int pwr_pin 	= (a31x_dvi_device.reset_gpio >> 16) & 0xFFFF;
	int buf_en_pin 	= a31x_dvi_device.reset_gpio & 0xFFFF;

	printk("%s\n", __func__);

	r = gpio_request(buf_en_pin, "A31x LCD buffer enable");
	if(r < 0)
	{
		printk("%s: failure to request LCD buffer enable pin %d\n", __func__, buf_en_pin);
		return;
	}
	//gpio_direction_output(buf_en_pin, 1);

	r = gpio_request(pwr_pin, "A31x LCD panel power");
	if(r < 0)
	{
		printk("%s: failure to request LCD power pin %d\n", __func__, pwr_pin);
		return;
	}
	//gpio_direction_output(pwr_pin, 0);
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= 109,
	},
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	{
	.name           = "wl1271",
	.mmc            = 2,
	.caps           = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
	.gpio_wp        = -EINVAL,
	.gpio_cd        = -EINVAL,
	.transceiver	= true,
//	.ocr_mask	= 0x00100000,	/* 3.3V */
	.nonremovable   = true,
	},
#endif
	{}	/* Terminator */
};

static struct regulator_consumer_supply a31x_vmmc1_supply = {
	.supply			= "vmmc",
};

/* TI-ST for WL12XX BT */
#ifdef CONFIG_TI_ST
#define WG7311_BT_EN 137

int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}

int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	printk("%s\n", __func__);

	/* Enable BT module via rising BT_EN pin*/
	// done by kim
	//gpio_set_value_cansleep(kim_data->nshutdown, 1);

	return 0;
}

int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	printk("%s\n", __func__);

	/* Disable BT module via falling BT_EN pin*/
	// done by kim
	//gpio_set_value_cansleep(kim_data->nshutdown, 0);

	return 0;
}

struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = WG7311_BT_EN,
	.dev_name 		= "/dev/ttyO1",
	.flow_cntrl 	= 1,
	.baud_rate 		= 3000000,
	.suspend 		= plat_kim_suspend,
	.resume 		= plat_kim_resume,
	.chip_enable 	= plat_kim_chip_enable,
	.chip_disable 	= plat_kim_chip_disable,
};

static struct platform_device wl12xx_device = {
	.name			= "kim",
	.id				= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static inline void __init
a31x_init_btwilink(void)
{
	printk("%s\n", __func__);

	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
}
#endif

#ifdef CONFIG_WL12XX_PLATFORM_DATA
#define WG7311_WL_EN      138
#define WG7311_WLAN_nIRQ  139

static struct regulator_consumer_supply a31x_vmmc2_supply =	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1");

/* VMMC2 for driving the WL12xx module */
static struct regulator_init_data a31x_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &a31x_vmmc2_supply,
};

static struct fixed_voltage_config a31x_vwlan = {
	.supply_name            = "vwl1271",
	.microvolts             = 1800000, /* 1.80V */
	.gpio                   = WG7311_WL_EN,
	.startup_delay          = 70000, /* 70ms */
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &a31x_vmmc2,
};

static struct platform_device a31x_wlan_regulator = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data  = &a31x_vwlan,
	},
};

struct wl12xx_platform_data a31x_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(WG7311_WLAN_nIRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4 MHz */
};

#endif

static struct regulator_consumer_supply a31x_vaux3_supply = {
	.supply         = "cam_1v8",
};

static struct regulator_consumer_supply a31x_vaux4_supply = {
	.supply         = "cam_2v8",
};

static int a31x_twl_gpio_setup(struct device *dev, unsigned gpio, unsigned ngpio)
{
	printk("%s\n", __func__);

	mmc[0].gpio_cd = gpio + 0; //Card detection
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	a31x_vmmc1_supply.dev = mmc[0].dev;

	return 0;
}

static struct twl4030_gpio_platform_data a31x_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= 0,
	/*.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13) | BIT(15) | BIT(16) | BIT(17),*/
	.setup		= a31x_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data a31x_vmmc1 = {
	.constraints = {
		.min_uV				= 1850000,
		.max_uV				= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies		= &a31x_vmmc1_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data a31x_vdac = {
	.constraints = {
		.min_uV				= 1800000,
		.max_uV				= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies		= &a31x_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data a31x_vpll2 = {
	.constraints = {
		.name				= "VDVI",
		.min_uV				= 1800000,
		.max_uV				= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies		= &a31x_vdvi_supply,
};

/* VAUX3 for CAM_1V8 */
static struct regulator_init_data a31x_vaux3 = {
	.constraints = {
		.min_uV             = 1800000,
		.max_uV             = 1800000,
		.apply_uV           = true,
		.valid_modes_mask   = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask     = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &a31x_vaux3_supply,
};

 /* VAUX4 for CAM_2V8 */
static struct regulator_init_data a31x_vaux4 = {
	.constraints = {
		.min_uV             = 1800000,
		.max_uV             = 1800000,
		.apply_uV           = true,
		.valid_modes_mask   = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
		.valid_ops_mask     = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &a31x_vaux4_supply,
};

static struct twl4030_usb_data a31x_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

#if 0
// TODO: stolen in panther board, must be checked and corrected before using
// Vladimir
/**
 ** Macro to configure resources
 **/
#define TWL4030_RESCONFIG(res,grp,typ1,typ2,state)  \
{                       		\
    .resource   	= res,      \
    .devgroup   	= grp,      \
    .type       	= typ1,     \
    .type2      	= typ2,     \
    .remap_sleep    = state     \
}

static struct twl4030_resconfig  __initdata board_twl4030_rconfig[] = {
    TWL4030_RESCONFIG(RES_VPLL1, 		DEV_GRP_P1,  3, 1, RES_STATE_OFF),    	/* ? */
    TWL4030_RESCONFIG(RES_VINTANA1, 	DEV_GRP_ALL, 1, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_VINTANA2, 	DEV_GRP_ALL, 0, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_VINTDIG, 		DEV_GRP_ALL, 1, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_VIO, 			DEV_GRP_ALL, 2, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_VDD1, 		DEV_GRP_P1,  4, 1, RES_STATE_OFF),      /* ? */
    TWL4030_RESCONFIG(RES_VDD2, 		DEV_GRP_P1,  3, 1, RES_STATE_OFF),      /* ? */
    TWL4030_RESCONFIG(RES_REGEN, 		DEV_GRP_ALL, 2, 1, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_NRES_PWRON, 	DEV_GRP_ALL, 0, 1, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_CLKEN, 		DEV_GRP_ALL, 3, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_SYSEN, 		DEV_GRP_ALL, 6, 1, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_HFCLKOUT, 	DEV_GRP_P3,  0, 2, RES_STATE_SLEEP), 	/* ? */
    TWL4030_RESCONFIG(0, 0, 0, 0, 0),
};

/**
 ** Optimized 'Active to Sleep' sequence
 **/
static struct twl4030_ins a31x_sleep_seq[] __initdata = {
    { MSG_SINGULAR (DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_SLEEP), 20},
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL,  RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_SLEEP), 2 },
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL,  RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_SLEEP), 2 },
};

static struct twl4030_script a31x_sleep_script __initdata = {
    .script = a31x_sleep_seq,
    .size   = ARRAY_SIZE(a31x_sleep_seq),
    .flags  = TWL4030_SLEEP_SCRIPT,
};

/**
 ** Optimized 'Sleep to Active (P12)' sequence
 **/
static struct twl4030_ins a31x_wake_p12_seq[] __initdata = {
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_ACTIVE), 2 }
};

static struct twl4030_script a31x_wake_p12_script __initdata = {
    .script = a31x_wake_p12_seq,
    .size   = ARRAY_SIZE(a31x_wake_p12_seq),
    .flags  = TWL4030_WAKEUP12_SCRIPT,
};

/**
 ** Optimized 'Sleep to Active' (P3) sequence
 **/
static struct twl4030_ins a31x_wake_p3_seq[] __initdata = {
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_ACTIVE), 2 }
};

static struct twl4030_script a31x_wake_p3_script __initdata = {
    .script = a31x_wake_p3_seq,
    .size   = ARRAY_SIZE(a31x_wake_p3_seq),
    .flags  = TWL4030_WAKEUP3_SCRIPT,
};

/**
 ** Optimized warm reset sequence (for less power surge)
 **/
static struct twl4030_ins a31x_wrst_seq[] __initdata = {
    { MSG_SINGULAR (DEV_GRP_NULL, RES_RESET, 	RES_STATE_OFF),  2 },
    { MSG_SINGULAR (DEV_GRP_NULL, RES_MAIN_REF, RES_STATE_WRST), 2 },
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, 	RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_WRST), 2},
    { MSG_SINGULAR (DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 2 },
    { MSG_SINGULAR (DEV_GRP_NULL, RES_VPLL1,    RES_STATE_WRST), 2 },
    { MSG_SINGULAR (DEV_GRP_NULL, RES_VDD2, 	RES_STATE_WRST), 7 },
    { MSG_SINGULAR (DEV_GRP_NULL, RES_VDD1, 	RES_STATE_WRST), 0x25 },
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, 	RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_WRST), 2 },
    { MSG_SINGULAR (DEV_GRP_NULL, RES_RESET, 	RES_STATE_ACTIVE), 2 },
};

static struct twl4030_script a31x_wrst_script __initdata = {
    .script = a31x_wrst_seq,
    .size   = ARRAY_SIZE(a31x_wrst_seq),
    .flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script __initdata *board_twl4030_scripts[] = {
    &a31x_wake_p12_script,
    &a31x_wake_p3_script,
    &a31x_sleep_script,
    &a31x_wrst_script
};

static struct twl4030_power_data __initdata a31x_script_data = {
    .scripts        	= board_twl4030_scripts,
    .num            	= ARRAY_SIZE(board_twl4030_scripts),
    .resource_config    = board_twl4030_rconfig,
};
#endif

static struct twl4030_codec_audio_data a31x_audio_data = {
	.audio_mclk 		= 26000000,
	.digimic_delay 		= 1,
	.ramp_delay_value 	= 1,
	.offset_cncl_path 	= 1,
	.check_defaults 	= false,
	.reset_registers 	= true,
	.cradle_aud_gpio 	= -1, 	// doesn't exist.
	.ext_speaker_gpio 	= 176, 	// shutdown external speaker amplifier
	.int_speaker_gpio 	= 175,	// shutdown internal speaker amplifier
	.ext_mic_gpio 		= 161,	// switch microphones
};

static struct twl4030_codec_data a31x_codec_data = {
	.audio_mclk 	= 26000000,
	.audio 			= &a31x_audio_data,
};

static struct twl4030_madc_platform_data tps65930_madc_pdata = {
	.ch_sel_gpio 	= {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	.irq_line		= -1,
};

static __u32 board_keymap[] = {
	KEY(0, 0, 0),
	KEY(0, 1, 0),
	KEY(0, 2, 0),
	KEY(0, 3, 0),
	KEY(0, 4, 0),
	KEY(0, 5, 0),
	KEY(0, 6, 0),
	KEY(0, 7, 0),
	KEY(0, 8, KEY_DOWN),

	KEY(1, 0, 0),
	KEY(1, 1, 0),
	KEY(1, 2, 0),
	KEY(1, 3, 0),
	KEY(1, 4, 0),
	KEY(1, 5, 0),
	KEY(1, 6, 0),
	KEY(1, 7, 0),
	KEY(1, 8, 0),

	KEY(2, 0, 0),
	KEY(2, 1, 0),
	KEY(2, 2, 0),
	KEY(2, 3, 0),
	KEY(2, 4, 0),
	KEY(2, 5, 0),
	KEY(2, 6, 0),
	KEY(2, 7, 0),
	KEY(2, 8, KEY_RIGHT),

	KEY(3, 0, 0),
	KEY(3, 1, 0),
	KEY(3, 2, 0),
	KEY(3, 3, 0),
	KEY(3, 4, 0),
	KEY(3, 5, 0),
	KEY(3, 6, 0),
	KEY(3, 7, 0),
	KEY(3, 8, KEY_LEFT),

	KEY(4, 0, 0),
	KEY(4, 1, 0),
	KEY(4, 2, 0),
	KEY(4, 3, 0),
	KEY(4, 4, 0),
	KEY(4, 5, 0),
	KEY(4, 6, 0),
	KEY(4, 7, 0),
	KEY(4, 8, KEY_SPACE),

	KEY(5, 0, 0),
	KEY(5, 1, 0),
	KEY(5, 2, 0),
	KEY(5, 3, 0),
	KEY(5, 4, 0),
	KEY(5, 5, 0),
	KEY(5, 6, 0),
	KEY(5, 7, 0),
	KEY(5, 8, KEY_BACK),

	KEY(6, 0, 0),
	KEY(6, 1, 0),
	KEY(6, 2, 0),
	KEY(6, 3, 0),
	KEY(6, 4, 0),
	KEY(6, 5, 0),
	KEY(6, 6, 0),
	KEY(6, 7, 0),
	KEY(6, 8, KEY_UP),

	KEY(7, 0, 0),
	KEY(7, 1, 0),
	KEY(7, 2, 0),
	KEY(7, 3, 0),
	KEY(7, 4, 0),
	KEY(7, 5, 0),
	KEY(7, 6, 0),
	KEY(7, 7, 0),
	KEY(7, 8, KEY_ENTER),
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size	= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data a31x_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 8,
	.cols		= 8,
	.rep		= 1,
};

// The following table shows the relationship of devices and regulators.
//
//	[LDO NAME]	[CONNECTED DEVICE]
//	VPLL2			DSI
//	VMMC1			SD
//	VMMC2			WLAN
//	VAUX1			None
//	VAUX2			USB PHY
//	VAUX3			None
//	VAUX4			None
// ========== Listed below are the regulators which used inside AP module ==========
//	VDAC			Video Buffer, DAC(S-Video & CVBS)
//	VPLL1			DPLL, DLL
//	VMIC1			Unknown
//	VMIC2			Unknown
//	VSIM			GPIO_126, GPIO127, GPIO_129

static struct twl4030_platform_data a31x_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &a31x_kp_data,
	.usb		= &a31x_usb_data,
	.gpio		= &a31x_gpio_data,
	.madc		= &tps65930_madc_pdata,
	.codec		= &a31x_codec_data,
	.vmmc1		= &a31x_vmmc1,
	.vdac		= &a31x_vdac,
	.vpll2		= &a31x_vpll2,
	.vaux3		= &a31x_vaux3,
	.vaux4		= &a31x_vaux4,
};

static struct i2c_board_info __initdata a31x_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = /*OMAP_GPIO_IRQ (0)*/INT_34XX_SYS_NIRQ,
		.platform_data = &a31x_twldata,
	},
};

#ifdef CONFIG_SND_SOC_WL1271BT
/* WL1271 Audio */
static struct platform_device wl1271bt_audio_device = {
	.name		= "wl1271bt",
	.id			= -1,
};

static struct platform_device wl1271bt_codec_device = {
	.name		= "wl1271bt-dummy-codec",
	.id			= -1,
};

static void wl1271bt_clk_setup(void)
{
	//u16 reg;
	//u32 val;

	// TBD: Should we start the oscillator here ???
	//
	// Enable CODEC oscillator ---- Vladimir: the CODEC oscillator should enabled together with BT voice mixing
	//	gpio_direction_output(41, 0);
	/*
	 * Set DEVCONF0 register to connect
	 * MCBSP1_CLKR -> MCBSP1_CLKX & MCBSP1_FSR -> MCBSP1_FSX
	 */
	//reg = OMAP2_CONTROL_DEVCONF0;
	//val = omap_ctrl_readl(reg);
	//val = val | 0x18;
	//omap_ctrl_writel(val, reg);
}
#endif

#if defined (CONFIG_POWER_LOST_DETECT)
struct power_lost_detect_info {
	int pl_det;
	int rst_det;
	int wake_en;
	int pwron_clk_en;
	int pl_act_lvl;
	int rst_act_lvl;
	int clk_en_pol;
};

static struct power_lost_detect_info power_lost_detect_platform_data = {
	.pl_det 		= 126,
	.rst_det 		= 167,
	.wake_en 		= 28,
	.pwron_clk_en 	= 186,
	.pl_act_lvl 	= 1,
	.rst_act_lvl 	= 1,
	.clk_en_pol 	= 1,
};

static struct platform_device power_lost_detect_device = {
	.name          = "power-lost-detect",
	.id            = -1,
	.dev           = {
		.platform_data = &power_lost_detect_platform_data,
	},
};
#endif

#if !defined (CONFIG_BATTERY_NULL)

static struct platform_device ignition_device = {
	.name	= "Ignition",
	.id     = -1,
};
#endif

static struct automotiveio_platform_data automotiveio_info = {
	.boardversion			= 846,
	.modemtype              = 0,
	.gpio_in_1 				= 102,
	.gpio_in_2 				= -1,
	.gpio_in_active_low_1 	=  0,
	.gpio_in_active_low_2 	= -1,
	.gpio_cradle 			= -1,
	.gpio_wall 				= -1,
	.gpio_out_1 			= -1,
	.gpio_out_2 			= -1,
	.gpio_out 				= 94,
	.gpio_audswitch 		= -1, // TODO:
	.gpio_modem_on_off 		= 41,
	.gpio_modem_uncon_shutdown = 40,
	.gpio_modem_usb_en 		= 61,
	.gpio_modem_usb_en_pol  = 1, //straight polarity
	.gpio_modem_power_ctl   = 110,
};

static struct platform_device automotiveio_device = {
	.name          = "a31x AutomotiveIO",
	.id            = -1,
	.dev           = {
		.platform_data = &automotiveio_info,
	},
};

#if defined (CONFIG_BATTERY_NULL)
#include <linux/ignition.h>

static struct ignition_platform_data ignition_info = {
	.ig_in = 102,
	.active_level = 1,
};
static struct platform_device ignition_device = {
	.name	= "Ignition",
	.id     = -1,
	.dev           = {
		.platform_data = &ignition_info,
	},
};

static struct platform_device battery_device = {
	.name          = "virtual-battery",
	.id            = -1,
	.dev           = {
		.platform_data = &ignition_info,
	},
};
#endif

#if 0
#define CAP_TOUCH_nIRQ
#define CAP_TOUCH_RESET_0
#define CAP_TOUCH_RESET_n

static struct ct36x_platform_data ct360_info = {

	.rst = CAP_TOUCH_RESET_0,
	.ss  = CAP_TOUCH_nIRQ,
};

static struct i2c_board_info __initdata a31x_i2c_ct360[] = {
	{
		I2C_BOARD_INFO("ct360_ts", 0x01),
		.flags = 0,
		.irq   = CAP_TOUCH_nIRQ,
		.platform_data = &ct360_info,
	},
};
#endif

static int __init
a31x_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, a31x_i2c_boardinfo, ARRAY_SIZE(a31x_i2c_boardinfo));

	/* for future use */
	omap_register_i2c_bus(2, 400, NULL, 0);

#if 0
	if(capacitive touch defined)
	{
		((struct ct36x_platform_data *)(a31x_i2c_ct360[0].platform_data))->rst = CAP_TOUCH_RESET;
		omap_register_i2c_bus(3, 100, a31x_i2c_ct360, ARRAY_SIZE(a31x_i2c_ct360));
	}
#endif

	return 0;
}

static void __init
a31x_init_irq(void)
{
	omap2_init_common_infrastructure();

// Vladimir
// TODO: the SDRAM already initialized by x-loader
// 	Must be removed

	omap2_init_common_devices(mt46h32m32lf6_sdrc_params, mt46h32m32lf6_sdrc_params);
	omap_init_irq();
// Vladimir
// 	GPMC is already initialized twice by x-loader (bus timing) and u-boot (error correction)
//	Are we need the third initialization?
	gpmc_init();
#ifdef CONFIG_OMAP_32K_TIMER
		omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct platform_device *a31x_devices[] __initdata = {
#if defined (CONFIG_POWER_LOST_DETECT)
	&power_lost_detect_device,
#endif
	&a31x_dss_device,
#ifdef CONFIG_SND_SOC_WL1271BT
	&wl1271bt_audio_device,
	&wl1271bt_codec_device,
#endif
	&lightsensor_device,
	&automotiveio_device,
	&ignition_device,
#if defined (CONFIG_BATTERY_NULL)
	&battery_device,
#endif
};

static void __init
a31x_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	/* find out the chip-select on which NAND exists */
	while(cs < GPMC_CS_NUM) {
		u32 ret = 0;

		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);
		if((ret & 0xC00) == 0x800) {
			printk("%s: NAND is found on CS%d\n", __func__, cs);
			nandcs = cs;
			break;
		}
		cs++;
	}

	if(nandcs < GPMC_CS_NUM){
		board_nand_init(a31x_nand_partitions, ARRAY_SIZE(a31x_nand_partitions),	nandcs, NAND_BUSWIDTH_16);
	} else {
		printk("%s: NAND critical error - not found\n", __func__);
	}

	return;
}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = 65,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#if defined (CONFIG_OMAP_MUX)
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode				= MUSB_OTG,
	.power				= 100,
};

static void _board_revision(void)
{
	// Vladimir
	// TODO: board revision vis a2d
	//
}

#if defined (CONFIG_TI_ST) || defined (CONFIG_WL12XX_PLATFORM_DATA)
/*
 * Enable wireless module
 *
 */
static inline void __init
a31x_wl1271_en(void)
{
	printk("%s: enable WG7311\n", __func__);

	// power up sequence
	//
	if(gpio_request(WG7311_WL_EN, "WiFi_EN") < 0)
		printk("%s, failure to enable WiFi\n", __func__);
	else
	{
		gpio_direction_output(WG7311_WL_EN, 1);
		gpio_free(WG7311_WL_EN);
	}

#if 0
	// done by kim driver
	if(gpio_request(WG7311_BT_EN, "BT_EN") < 0)
		printk("%s, failure to enable BT\n", __func__);
	else
	{
		gpio_direction_output(WG7311_BT_EN, 1);
		gpio_free(WG7311_BT_EN);
	}
#endif
}
#endif

///////////////////////
/// AUDIO - TWL4030 ///
///////////////////////
static void __init
a31x_audio_init(void){
	printk("%s: Select internal speaker and microphone\n", __func__);

	// Shutdown external amplifier
	if(gpio_request(176, "ext_spkr_amp") < 0)
		printk("%s: failure to handle external amplifier\n", __func__);
	else
	{
		gpio_direction_output(176, 0);
		//gpio_free(176);
	}
	// Switch to internal microphone
	if(gpio_request(161, "ext_mic") < 0)
		printk("%s: failure to handle external microphones\n", __func__);
	else
	{
		gpio_direction_output(161, 0);
		//gpio_free(161);
	}

	// Enable internal speaker amplifier
	if(gpio_request(175, "int_spkr_amp") < 0)
		printk("%s: failure to handle internal amplifier\n", __func__);
	else
	{
		gpio_direction_output(175, 0);
		//gpio_free(175);
	}
}

/**
 *  * Board specific initialization of PM components
 *   */
static void __init
a31x_init(void)
{
	void *vaddr = phys_to_virt(0x81E00000);

	_board_revision();


#if defined (CONFIG_OMAP_MUX)
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
#endif

    a31x_i2c_init();
	a31x_audio_init();

	platform_add_devices(a31x_devices, ARRAY_SIZE(a31x_devices));

	omap_serial_init();

	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);


#ifdef CONFIG_TI_ST
	a31x_wl1271_en();
#endif

	a31x_flash_init();
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	ads7846_pendown_init();
	spi_register_board_info(a31x_spi_board_info, ARRAY_SIZE(a31x_spi_board_info));
#endif

	// errata 318e, advisory 1.91: pins cke0/cke1 connected to VSS in CBP package,
	// must be unconnected from memory/high impedance
	//
	/* Ensure SDRC pins are mux'd for self-refresh */
	//omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	//omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	if(gpio_request(27, "SW 3.3 V") < 0)
		printk("%s: failure to enable peripheral SW 3.3 V\n", __func__);
	else
		gpio_direction_output(27, 1);

	if(gpio_request(98, "DEV_ON") < 0)
		printk("%s: failure to enable device, urgent shutdown\n", __func__);
	else
		gpio_direction_output(98, 0);

	boot_fbuff = kzalloc(800 * 480 * 3, GFP_KERNEL);
	memcpy(boot_fbuff, vaddr, 800 * 480 * 3);

	a31x_display_init();

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&a31x_wlan_data))
		pr_err("error setting wl12xx data\n");

	platform_device_register(&a31x_wlan_regulator);
#endif

#ifdef CONFIG_TI_ST
	a31x_init_btwilink();
#endif

#ifdef CONFIG_SND_SOC_WL1271BT
	wl1271bt_clk_setup();
#endif

	printk("%s: prepare HSUSB Host\n", __func__);

	if(gpio_request(111, "HSUSB PHY 1.8 V") < 0)
		printk("%s: failure to power up HSUSB PHY 1.8V\n", __func__);
	else
		gpio_direction_output(111,0);


	if(gpio_request(65, "PHY - nRESETB") < 0)
		printk("%s: failure to reset HSUSB PHY\n", __func__);
	else
		gpio_direction_output(65, 1);

	if(gpio_request(34, "HUB - nRESET") < 0)
		printk("%s: failure to reset USB HUB\n", __func__);
	else
		gpio_direction_output(34, 1);

	//	TODO: UNCOMMENT TO INIT BOARD PM COMPONENTS.
	//
	//omap3_a31x_pm_init();
}


MACHINE_START(A317, "A317 Board")\
	.boot_params	= 0x80000100,
	.map_io			= omap3_map_io,
	.reserve		= omap_reserve,
	.init_irq		= a31x_init_irq,
	.init_machine	= a31x_init,
	.timer			= &omap_timer,
MACHINE_END
