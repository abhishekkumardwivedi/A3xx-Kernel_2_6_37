/*
 * linux/arch/arm/mach-omap2/board-micronet-ce300p1.c
 *
 * Copyright (C) 2012 Micronet
 *
 * Modified from mach-omap2/board-omap3beagle.c
 *
 * Initial code:
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
#include <linux/bq24172.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <plat/mcspi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>
#include <linux/usb/android_composite.h>
#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <linux/i2c.h>
#include <linux/i2c/pca9575.h>
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

#include <linux/smsc911x.h>
//#include <linux/i2c/ct360.h>
#include <linux/goodix_touch.h>
#include <linux/ignition.h>

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "board-flash.h"

#ifdef CONFIG_SND_SOC_WL1271BT
#include "control.h"
#endif

#define NAND_BLOCK_SIZE		SZ_128K

#define A300_ETHR_START	0x15000000 //0x2c000000
#define A300_ETHR_SIZE	1024
#define A300_ETHR_ID_REV	0x50
#define A300_ETHR_GPIO_IRQ	95
#define A300_SMSC911X_CS	5

static unsigned int s_board_revision = 5;

static char *identlcd = 0;
module_param(identlcd, charp, S_IRUGO);
MODULE_PARM_DESC(identlcd, "Ident LCD");

void *boot_fbuff;//

static struct mtd_partition a300_nand_partitions[] = {
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

/* DSS */
/*
 * OMAP3 LCD Panel control signals
 */

/*
 * PWMA register offsets (TWL4030_MODULE_PWMA)
 */
#define TWL_PWMAON	 0x00
#define TWL_PWMAOFF	 0x01

#define TWL4030_LED_LEDEN	0x0

u8 buttonlightlevel;

int a300_set_keypad_light_level(const char *inval, const struct kernel_param *kp)
{
	unsigned long val;
	char *end;
	int ret;
	static int fisrt_time = 1;
	u8 reg;

	if (fisrt_time)
	{
		twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE,
			&reg, TWL4030_REG_VIBRA_CTL);
		twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,
			 (reg & ~TWL4030_VIBRA_EN), TWL4030_REG_VIBRA_CTL);

		fisrt_time = 0;
	}

	val = simple_strtoul(inval, &end, 0);

	if (val > 255) {
		ret = -EINVAL;
		goto bail;
	}

	ret = end + 1 - inval;
	if (ret == 0)
		ret = -EINVAL;

	twl_i2c_read_u8(TWL4030_MODULE_LED, &reg, TWL4030_LED_LEDEN);

    if (val == 0) { //turn off
		reg &= ~0x22;
		val = 0x1;
    } else { //turn on
		reg |= 0x22;
		val = 0x7f;
    }

	twl_i2c_write_u8(TWL4030_MODULE_PWMB, val, TWL_PWMAON);  // on
	twl_i2c_write_u8(TWL4030_MODULE_PWMB, 0x7f, TWL_PWMAOFF); // off//0x7F, TWL_PWMAOFF); // off
	twl_i2c_write_u8(TWL4030_MODULE_LED, reg, TWL4030_LED_LEDEN);

	buttonlightlevel = (int)val;

bail:
	return ret;
}

int a300_get_keypad_light_level(char *buffer, const struct kernel_param *kp)
{
	int result;

	result = sprintf(buffer,"%d",buttonlightlevel);

	return result;
}

static struct kernel_param_ops lightbutton_ops_str = {
	.set = a300_set_keypad_light_level,
	.get = a300_get_keypad_light_level,
};

module_param_cb(buttons_light, &lightbutton_ops_str, NULL, 0664);

// In our A300 board, the duty cycle values are inverted so 0x7e at the PWMAOFF means 0x01
static int a300_set_bl_intensity(struct omap_dss_device *dssdev, int level)
{
	unsigned char c;
	static int fisrt_time = 1;

	if (level > dssdev->max_backlight_level)
		level = dssdev->max_backlight_level;

	if (level < 9)
		level = 9;

	c = ((125 * level) / 100) + 1;
//	printk(">>>>>>>>>>>>>>>>>>>>>>>> a300_set_bl_intensity() level=%d c=%d\n", level, c);
	if (fisrt_time)
	{
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x02, TWL_PWMAOFF); // off
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMAON);  // on
		fisrt_time = 0;
	}

	twl_i2c_write_u8(TWL4030_MODULE_PWMA, c, TWL_PWMAOFF);

	return 0;
}

char previous_bkl_duty_cycle = 126;

static int a300_enable_lcd(struct omap_dss_device *dssdev)
{
	u8 en;
	// Configure the duty cycle of LEDA
	printk(">>>>>>>>>>>>>>>>>>>>>>>> a300_enable_lcd()\n");
	twl_i2c_write_u8(TWL4030_MODULE_PWMA, previous_bkl_duty_cycle, TWL_PWMAOFF); // off
	twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMAON);  // on
	twl_i2c_read_u8(TWL4030_MODULE_LED, &en, TWL4030_LED_LEDEN);
	twl_i2c_write_u8(TWL4030_MODULE_LED, en | 0x11, TWL4030_LED_LEDEN);

	return 0;
}

static void a300_disable_lcd(struct omap_dss_device *dssdev)
{
	u8 en;

	printk(">>>>>>>>>>>>>>>>>>>>>>>> a300_disable_lcd()\n");
	twl_i2c_read_u8(TWL4030_MODULE_PWMA, &previous_bkl_duty_cycle, TWL_PWMAOFF);
	if(previous_bkl_duty_cycle < 10)
		previous_bkl_duty_cycle = 10;

	twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x00, TWL_PWMAOFF); // off
	twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x7F, TWL_PWMAON); // on

	twl_i2c_read_u8(TWL4030_MODULE_LED, &en, TWL4030_LED_LEDEN);
	twl_i2c_write_u8(TWL4030_MODULE_LED, en & (~0x11), TWL4030_LED_LEDEN);
}

static struct omap_dss_device a300_lcd_device = {
	.name			= "lcd",
	.driver_name		= "a300_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.max_backlight_level	= 100,
	.platform_enable	= a300_enable_lcd,
	.platform_disable	= a300_disable_lcd,
	.set_backlight	= a300_set_bl_intensity,
};

static int a300_enable_dvi(struct omap_dss_device *dssdev)
{
	printk(">>>>>>>>>>>>>>>>>>>>>>>> a300_enable_dvi\n");

	if (gpio_is_valid(dssdev->reset_gpio))
	{
		gpio_set_value(29, 1);

		if(16 > s_board_revision)
		{
			if(identlcd && ('A' == *identlcd || 'C' == *identlcd))
				gpio_set_value(161, 1);
		}
		gpio_set_value(dssdev->reset_gpio, 0);
		a300_enable_lcd(dssdev);
	}
	else
		printk("invalid control pin %d\n", dssdev->reset_gpio);

	return 0;
}

static void a300_disable_dvi(struct omap_dss_device *dssdev)
{
	printk(">>>>>>>>>>>>>>>>>>>>>>>> a300_disable_dvi\n");
	if (gpio_is_valid(dssdev->reset_gpio))
	{
		a300_disable_lcd(dssdev);
		gpio_set_value(dssdev->reset_gpio, 1);
		gpio_set_value(29, 0);
		if(16 > s_board_revision)
		{
			if(identlcd && ('A' == *identlcd || 'C' == *identlcd))
				gpio_set_value(161, 0);
		}
	}
	else
		printk("invalid control pin %d\n", dssdev->reset_gpio);
}

static struct omap_dss_device a300_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio = 104,
	.platform_enable = a300_enable_dvi,
	.platform_disable = a300_disable_dvi,
};

static struct omap_dss_device a300_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *a300_dss_devices[] = {
	&a300_lcd_device,
	&a300_dvi_device,
	&a300_tv_device,
};

static struct omap_dss_board_info a300_dss_data = {
	.num_devices = ARRAY_SIZE(a300_dss_devices),
	.devices = a300_dss_devices,
	.default_device = &a300_dvi_device,
};

static struct platform_device a300_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &a300_dss_data,
	},
};

static struct platform_device lightsensor_device = {
	.name          = "tept4400",
	.id            = -1,
	.dev            = {
		.platform_data = NULL,
	},
};

static struct ignition_platform_data ignition_info = {
		.ig_in = 102,
		.active_level = 0,
};

#if defined (CONFIG_BATTERY_NULL)

static struct platform_device battery_device = {
	.name          = "virtual-battery",
	.id            = -1,
	.dev           = {
		.platform_data = &ignition_info,
	},
};
#endif

static struct automotiveio_platform_data automotiveio_info = {
	.gpio_in_1 = 102, //307i and 307
	.gpio_in_2 = 182, //307i only
	.gpio_in_active_low_1 = 1, //307i and 307
	.gpio_in_active_low_2 = 0, //307i only
	.gpio_cradle = 211, //307i only
	.gpio_wall = 212,   //307i only
	.gpio_out_1 = 218,  //307i only
	.gpio_out_2 = 219,  //307i only
	.gpio_out = 94,     //307 only
	.gpio_audswitch = 220,  //307i only
	.gpio_modem_on_off = 214, //307i only
	.gpio_modem_uncon_shutdown = 215, //307i only
	.gpio_modem_usb_en = 210,//307i only revision B only
	.gpio_modem_usb_en_pol = 0,//reverse polarity
	.gpio_modem_power_ctl   = 225,// 307i only	
};

static struct platform_device automotiveio_device = {
	.name          = "A300 AutomotiveIO",
	.id            = -1,
	.dev           = {
		.platform_data = &automotiveio_info,
	},
};

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
	.wake_en 		= 10,
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

static struct regulator_consumer_supply a300_vdac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss");

static struct regulator_consumer_supply a300_vdvi_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");
static struct platform_device ignition_device = {
	.name          = "Ignition",
	.id            = -1,
	.dev           = {
		.platform_data = &ignition_info,
	},
};

/* 
replaced by _lcd_config function 
static void __init a300_display_init(void)
{
	int r;

	printk(">>>>>>>>>>>>>>>>>>>>>>>> a300_display_init\n");

	// power on display buffer
	r = gpio_request(29, "Display buffer power");
	if(r < 0)
	{
		printk(KERN_ERR "Unable to power on display buffer\n");
		return;
	}
	gpio_direction_output(29, 0);

	if(16 > s_board_revision)
	{
		r = gpio_request(161, "disp");
		if(r < 0)
		{
			printk(KERN_ERR "Unable to power on disp\n");
			return;
		}
		gpio_direction_output(161, 0);
	}
	// Enable display buffer direction DISPC->LCD
	r = gpio_request(a300_dvi_device.reset_gpio, "DVI reset");
	if(r < 0)
	{
		printk(KERN_ERR "impossible to enable display buffer\n");
		return;
	}

	gpio_direction_output(a300_dvi_device.reset_gpio, 1);
}
*/
///////////////////////
/// AUDIO - TWL4030 ///
///////////////////////
static void __init a300_audio_init(void){

	int ret = 0;
	int gpio = OMAP_MAX_GPIO_LINES;
//	unsigned char reg_value = 0;

	printk("AUDIO INIT (a300), Entered\n");

	// shutdown the BT codec oscilator
	ret = gpio_request(41, "btc_osc");
	if (ret < 0)
		printk("a300_audio_init: impossible to shutdown the BT codec oscilator\n");
	gpio_direction_output(41, 0);

	// Shutdown external amplifier
	ret = gpio_request(59, "ext_amp");
	if (ret < 0)
		printk("TWL ERROR, couldn't get gpio 59\n");

	gpio_direction_output(59, 0);

	// Shutdown external microphone
	if( 15 < s_board_revision ){
		ret = gpio_request(177, "ext_mic");
		if (ret < 0)
			printk("twl ERROR, couldn't get gpio 177\n");

		gpio_direction_output(177, 0);
	}
	else
	{
		ret = gpio_request(180, "ext_mic");
		if (ret < 0)
			printk("twl ERROR, couldn't get gpio 180\n");

		gpio_direction_output(180, 0);
	}

	

	// Turn internal speaker amplifier on (1)
	ret = gpio_request(gpio + 15, "int_spkr");
	if (ret < 0)
		printk("twl ERROR, couldn't get gpio int_spkr\n");

	gpio_direction_output(gpio + 15, 0);
	mdelay(100);
	//gpio_set_value(gpio + 15, 1);
//	gpio_free(gpio + 15);
}
#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
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

static struct regulator_consumer_supply a300_vmmc1_supply = {
	.supply			= "vmmc",
};

#ifdef CONFIG_TI_ST
/* TI-ST for WL1271 BT */

#define A300_WL1271_BT_EN_GPIO ( 137 )

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
	printk(KERN_INFO"%s\n", __func__);

	/* Enable BT module via rising BT_EN pin and prevent HCI operations during at least 150 ms*/

	return 0;
}

int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	printk(KERN_INFO"%s\n", __func__);

	/* Disable BT module via falling BT_EN pin and prevent HCI operations during at least 150 ms*/

	return 0;
}

struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = A300_WL1271_BT_EN_GPIO,
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};

static struct platform_device wl127x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static inline void __init a300_init_btwilink(void)
{
	pr_info("a300: bt init\n");

	platform_device_register(&wl127x_device);
	platform_device_register(&btwilink_device);
}
#endif
#ifdef CONFIG_WL12XX_PLATFORM_DATA

#define A300_WLAN_PMEA300      (138)
#define A300_WLAN_IRQ_GPIO     (139)

static struct regulator_consumer_supply a300_vmmc2_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1");

/* VMMC2 for driving the WL12xx module */
static struct regulator_init_data a300_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &a300_vmmc2_supply,
};

static struct fixed_voltage_config a300_vwlan = {
	.supply_name            = "vwl1271",
	.microvolts             = 1800000, /* 1.80V */
	.gpio                   = A300_WLAN_PMEA300,
	.startup_delay          = 70000, /* 70ms */
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &a300_vmmc2,
};

static struct platform_device a300_wlan_regulator = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data  = &a300_vwlan,
	},
};

struct wl12xx_platform_data a300_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(A300_WLAN_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4 MHz */
};

#endif

static struct regulator_consumer_supply a300_vaux3_supply = {
	.supply         = "cam_1v8",
};

static struct regulator_consumer_supply a300_vaux4_supply = {
	.supply         = "cam_2v8",
};

static struct gpio_led gpio_leds[];

// Vladimir
// TODO: The main goal of the function is a preparing of TPS65930 CC GPIOs for using. Pay attention the CE300 board has six(6)
//  TPS65930 GPIO pins that are multiplexed with other components, such as MMC card detect etc.
// 	All manipulation with display buffer should be moved to display controller handler
// 	All manipulation with audio and speakers/microphones/amplifiers should be moved to audio controller handler
//
static int a300_twl_gpio_setup(struct device *dev, unsigned gpio, unsigned ngpio)
{
//	int ret;
/*
	ret = gpio_request(104, "disp_buf");
	if (ret < 0)
		printk("twl ERROR, couldn't get gpio 104\n");

	gpio_direction_output(104, 0);
*/
	printk("a300_twl_gpio_setup: setup the board GPIOs\n");

	a300_audio_init();

	mmc[0].gpio_cd = gpio + 0; //Card detection
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	a300_vmmc1_supply.dev = mmc[0].dev;

	return 0;
}

static struct twl4030_gpio_platform_data a300_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= 0, //true,
	/*.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),*/
	.setup		= a300_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data a300_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &a300_vmmc1_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data a300_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &a300_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data a300_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &a300_vdvi_supply,
};

/* VAUX3 for CAM_1V8 */
static struct regulator_init_data a300_vaux3 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &a300_vaux3_supply,
};

 /* VAUX4 for CAM_2V8 */
static struct regulator_init_data a300_vaux4 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &a300_vaux4_supply,
};

static struct twl4030_usb_data a300_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data a300_audio_data = {
	.audio_mclk = 26000000,
	.digimic_delay = 1,
	.ramp_delay_value = 1,
	.offset_cncl_path = 1,
	.check_defaults = false,
	.reset_registers = true,
	.cradle_aud_gpio = 220, // cradle audio switch gpio. accessed via pca9575. M307i only.
	.ext_speaker_gpio = 59, // external speaker amplifier
	.int_speaker_gpio = 207,// internal speaker amplifier (192 + 15 = 207)
	.ext_mic_gpio = 180,	// external microphone
};

static struct twl4030_codec_data a300_codec_data = {
	.audio_mclk = 26000000,
	.audio = &a300_audio_data,
	.audpwron_gpio = OMAP_MAX_GPIO_LINES +15,
};

static uint32_t board_keymap[] = {
	KEY(0, 0, 0),
	KEY(0, 1, KEY_F3), 	//F3
	KEY(0, 2, KEY_F5),//F5
	KEY(0, 3, KEY_F9), 	//F9
	KEY(0, 4, KEY_DOWN),//DOWN
	KEY(0, 5, 0),
	KEY(0, 6, 0),
	KEY(0, 7, 0),

	KEY(1, 0, KEY_RIGHT),//RIGHT
	KEY(1, 1, KEY_LEFT), //LEFT
	KEY(1, 2, KEY_F6),	//F6
	KEY(1, 3, KEY_F10), //F10
	KEY(1, 4, KEY_F8),	//F8
	KEY(1, 5, 0),
	KEY(1, 6, 0),
	KEY(1, 7, 0),

	KEY(2, 0, KEY_BACK),// //BACK or CANCEL
	KEY(2, 1, KEY_SPACE), //Accept button
	KEY(2, 2, KEY_F7), //F7
	KEY(2, 3, KEY_F11),//F11
	KEY(2, 4, KEY_F12),//F12
	KEY(2, 5, 0),
	KEY(2, 6, 0),
	KEY(2, 7, 0),

	KEY(3, 0, 0),
	KEY(3, 1, 0),
	KEY(3, 2, 0),
	KEY(3, 3, 0),
	KEY(3, 4, 0),
	KEY(3, 5, 0),
	KEY(3, 6, KEY_F2), //F2
	KEY(3, 7, 0),

	KEY(4, 0, 0),
	KEY(4, 1, 0),
	KEY(4, 2, 0),
	KEY(4, 3, 0),
	KEY(4, 4, 0),
	KEY(4, 5, 0),
	KEY(4, 6, KEY_F1), //F1
	KEY(4, 7, 0),

	KEY(5, 0, 0),
	KEY(5, 1, 0),
	KEY(5, 2, 0),
	KEY(5, 3, 0),
	KEY(5, 4, 0),
	KEY(5, 5, 0),
	KEY(5, 6, KEY_F4), //F4
	KEY(5, 7, 0),
#if defined (CONFIG_MACH_A300)
	KEY(6, 0, KEY_UP),		//UP
	KEY(6, 1, KEY_ENTER),	//PUSH button
	KEY(6, 2, 0),
	KEY(6, 3, 0),
	KEY(6, 4, 0),
	KEY(6, 5, 0),
	KEY(6, 6, 0),
	KEY(6, 7, 0),
#endif
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size	= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data a300_kp_data = {
	.keymap_data	= &board_map_data,
#if defined (CONFIG_MACH_A300)
	.rows		= 7,
#else
	.rows		= 6,
#endif
	.cols		= 6,
	.rep		= 1,
};

static struct twl4030_madc_platform_data tps65930_madc_pdata = {
	.ch_sel_gpio = {OMAP_MAX_GPIO_LINES + 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	.irq_line	= -1,
};

static struct twl4030_platform_data a300_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &a300_kp_data,
	.usb		= &a300_usb_data,
	.gpio		= &a300_gpio_data,
	.madc		= &tps65930_madc_pdata,
	.codec		= &a300_codec_data,
	.vmmc1		= &a300_vmmc1,
	.vdac		= &a300_vdac,
	.vpll2		= &a300_vpll2,
	.vaux3		= &a300_vaux3,
	.vaux4		= &a300_vaux4,
};

static struct i2c_board_info __initdata a300_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = /*OMAP_GPIO_IRQ (0)*/INT_34XX_SYS_NIRQ,
		.platform_data = &a300_twldata,
	},
};

static struct pca9575_platform_data pca9575_data = {
	.irq_base	= OMAP_GPMC_IRQ_END,
	.gpio_base  = TWL4030_GPIO_MAX + OMAP_MAX_GPIO_LINES,
	.imask      = 0xFFF1,
};

struct goodix_i2c_platform_data goodix_data1 = {
	.irq_int = 103,
	.power_gpio = 174,
};
struct goodix_i2c_platform_data goodix_data2 = {
	.irq_int = 103,
	.power_gpio = 223,
};

#if defined (CONFIG_CHARGER_BQ24172)
static struct bq24172_platform_data bq24172_info = {
	.charge_status_in = 94,
	.charge_status_lvl = 0,
	.pwr_too_low_in = 161,
	.pwr_too_low_lvl = 0,
	.pwr_too_high_in = 126,
	.pwr_too_high_lvl = 0,
	.v5_good_in = TWL4030_GPIO_MAX + OMAP_MAX_GPIO_LINES + 3,
	.v5_good_lvl = 1,
};

static struct platform_device charger_device = {
	.name          = "bq24172-charger",
	.id            = -1,
	.dev           = {
		.platform_data = &bq24172_info,
	},
};
#endif

#if defined (CONFIG_BATTERY_BQ20Z75)
struct battery_platform_data {
	int transport_type;
	int reserved;
};
static struct battery_platform_data battery_data = {
	// for future use
	.transport_type = 0,
};
#endif

static struct i2c_board_info __initdata a300_i2c_2[] = {
#if defined (CONFIG_BATTERY_BQ20Z75)
	{
		I2C_BOARD_INFO("bq20z75", 0x0B), // Smart Battery reserved address Data Specification v 1.1 December 1998
		.platform_data = &battery_data,
	},
#endif
	{
		I2C_BOARD_INFO("pca9575", 0x20),
		.flags = I2C_CLIENT_WAKE,
		.irq = 55,//GPIO number that is connected to PCA9575
		.platform_data = &pca9575_data,
	},
};
#define A300_BOARD_TS_GPIO 103
#define M307_BOARD_TS_RESET 94

static struct i2c_board_info __initdata a300_i2c_3[] = {
	{
		I2C_BOARD_INFO("Goodix-TS", 0x55),
		.irq = 103, // Interupt line to module
		.platform_data = &goodix_data1,
	},
}; 

static int __init
a300_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, a300_i2c_boardinfo, ARRAY_SIZE(a300_i2c_boardinfo));

	if( 15 < s_board_revision ) {
		if (gpio_request(55, "PCA9575 Interrput") < 0) printk(KERN_ERR "can't get PCA9575 Interrput GPIO\n");
		gpio_direction_input(55);
		gpio_set_debounce(55, 0xa); 
	
		// SMBus maximum rate 100 kHz
		omap_register_i2c_bus(2, 100, a300_i2c_2, ARRAY_SIZE(a300_i2c_2));
	}
	else
	{/* Bus 2 is used for Camera/Sensor interface */
		omap_register_i2c_bus(2, 400, NULL, 0);
	}

	/* Bus 3 is attached to the capacitive touchscreen in 100kHz */
	if('D' == *identlcd) { //Capacitive touchscreen
		if( 15 < s_board_revision ) {

			a300_i2c_3[0].platform_data = &goodix_data2;
			omap_register_i2c_bus(3, 100, a300_i2c_3, ARRAY_SIZE(a300_i2c_3));
		}
		else
			omap_register_i2c_bus(3, 100, a300_i2c_3, ARRAY_SIZE(a300_i2c_3));
	}

	return 0;
}

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 7,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

#define A300_BOARD_TS_GPIO 103

static void touchscreen_pendown_gpio_init(void)
{
	printk(">>>>>>>>>>>>>>>>>>>>>>>> touchscreen_pendown_gpio_init\n");
	if (gpio_request(A300_BOARD_TS_GPIO, "Touchscreen pendown") < 0)
		printk(KERN_ERR "can't get Touchscreen pen down GPIO\n");

	gpio_direction_input(A300_BOARD_TS_GPIO);
	gpio_set_debounce(A300_BOARD_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(A300_BOARD_TS_GPIO);
}

static struct ads7846_platform_data ads7846_config = {
	.x_max			    = 0x0fff,
	.y_max			    = 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 1,
	.debounce_tol		= 20,
	.debounce_rep		= 3, //L4B from 1 to 2. //30 was good.
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,//4,
	//.penirq_recheck_delay_usecs = 600,
	//.settle_delay_usecs	= 30,
	//.vref_mv		= 2500,
	.wakeup			= false,
	.model = 7845,
//	.swap_xy = true,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info a300_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 125000,//290000,//120000, // Vladimir recommends to reduce spi clk rate to 2900. didn't work.
		.controller_data	= &ads7846_mcspi_config,
		.irq			    = OMAP_GPIO_IRQ(A300_BOARD_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

static void __init a300_init_irq(void)
{
	omap2_init_common_infrastructure();

// Vladimir
// TODO: the A300 boards has no micron SDRAM, in any case the SDRAM already initialized by x-loader
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

#ifdef CONFIG_SND_SOC_WL1271BT
/* WL1271 Audio */
static struct platform_device wl1271bt_audio_device = {
	.name		= "wl1271bt",
	.id		= -1,
};

static struct platform_device wl1271bt_codec_device = {
	.name		= "wl1271bt-dummy-codec",
	.id		= -1,
};

static void wl1271bt_clk_setup(void)
{
	u16 reg;
	u32 val;

	// TBD: Should we start the oscillator here ???
	//
	// Enable CODEC oscillator ---- Vladimir: the CODEC oscillator should enabled together with BT voice mixing
	//	gpio_direction_output(41, 0);
	/*
	 * Set DEVCONF0 register to connect
	 * MCBSP1_CLKR -> MCBSP1_CLKX & MCBSP1_FSR -> MCBSP1_FSX
	 */
	reg = OMAP2_CONTROL_DEVCONF0;
	val = omap_ctrl_readl(reg);
	val = val | 0x18;
	omap_ctrl_writel(val, reg);
}
#endif

static struct platform_device *a300_devices[] __initdata = {
#if defined (CONFIG_CHARGER_BQ24172)
	&charger_device,
#endif
#if defined (CONFIG_POWER_LOST_DETECT)
	&power_lost_detect_device,
#endif
	&a300_dss_device,
	//&usb_mass_storage_device,
#ifdef CONFIG_SND_SOC_WL1271BT
	&wl1271bt_audio_device,
	&wl1271bt_codec_device,
#endif
	&keys_gpio,
	&lightsensor_device,
	&automotiveio_device,
	&ignition_device,
#if defined (CONFIG_BATTERY_NULL)
	&battery_device,
#endif
};

static void __init
a300_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		board_nand_init(a300_nand_partitions,
			ARRAY_SIZE(a300_nand_partitions),
			nandcs, NAND_BUSWIDTH_16);
	}
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

static struct resource a300_smsc911x_resources[] = {
	[0] =	{
		.start	= A300_ETHR_START,
		.end	= (A300_ETHR_START + A300_ETHR_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
	[1] =	{
		.start	= OMAP_GPIO_IRQ(A300_ETHR_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(A300_ETHR_GPIO_IRQ),
		.flags	= (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface  = PHY_INTERFACE_MODE_MII,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags          = (SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS),
};

static struct platform_device a300_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(a300_smsc911x_resources),
	.resource	= &a300_smsc911x_resources[0],
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init
a300_init_smsc911x(void)
{
	int eth_cs, eth_rst;
	struct clk *l3ck;
	unsigned int rate;

	//adidardi
	//gpio_direction_output(98, 0);

	eth_rst = 61;
	eth_cs = A300_SMSC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpio_request(eth_rst, "SMSC911x gpio") < 0) {
		pr_err(KERN_ERR "Failed to request GPIO7 for smsc911x\n");
		return;
	}

	gpio_direction_output(eth_rst, 0);
	/* reset pulse to ethernet controller*/
	//usleep_range(150, 220);
	//gpio_set_value(eth_rst, 0);
	usleep_range(150, 220);
	gpio_set_value(eth_rst, 1);
	usleep_range(1, 2);


	if (gpio_request(A300_ETHR_GPIO_IRQ, "SMSC911x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smsc911x IRQ\n",
			A300_ETHR_GPIO_IRQ);
		return;
	}

	gpio_direction_input(A300_ETHR_GPIO_IRQ);
	platform_device_register(&a300_smsc911x_device);
}
static void _board_revision(void)
{
	gpio_request(99, "rev1");
	gpio_request(100, "rev2");
	gpio_request(105, "rev3");
	gpio_request(106, "rev4");
	gpio_request(163, "rev5");
	gpio_direction_input(99);
	gpio_direction_input(100);
	gpio_direction_input(105);
	gpio_direction_input(106);
	gpio_direction_input(163);

	s_board_revision = 0;

	s_board_revision = gpio_get_value(99)  << 0 | gpio_get_value(100) << 1 |
					 gpio_get_value(105) << 2 | gpio_get_value(106) << 3 | gpio_get_value(163) << 4;

	automotiveio_info.boardversion = s_board_revision;
	if ( 16 > s_board_revision ) {//307 HW 830 board
		automotiveio_info.gpio_in_2 			= -1;
		automotiveio_info.gpio_in_active_low_2 	= -1;
		automotiveio_info.gpio_cradle			= -1;
		automotiveio_info.gpio_wall 			= -1;
		automotiveio_info.gpio_out_1 			= -1;
		automotiveio_info.gpio_out_2 			= -1;
		automotiveio_info.gpio_audswitch		= -1;
		automotiveio_info.gpio_modem_on_off		= -1;
		automotiveio_info.gpio_modem_uncon_shutdown	= -1;
		automotiveio_info.gpio_modem_usb_en		= -1;
		automotiveio_info.gpio_modem_power_ctl	= -1;
	}
	if (s_board_revision > 15) {//307i HW 846 board
	    gpio_request(107, "MODEM 3G");
	    gpio_direction_input(107);
		automotiveio_info.modemtype = gpio_get_value(107);
		if (s_board_revision == 16) {//revision A - no USB buffer on board
			automotiveio_info.gpio_modem_usb_en	= -1;
		}
	}

	printk(">>>>>>>>>>>>>>>A300 board file - BOARD REVISION = %d MODEM 3G = %d\n", s_board_revision, automotiveio_info.modemtype);
}
int get_ident_lcd(void)
{
	if(NULL == identlcd)
		return -1;
	return *identlcd;
}
EXPORT_SYMBOL(get_ident_lcd);

static void _lcd_config(void)
{
	printk("LCD config for\'%c\'\n", (identlcd ? *identlcd : 0));
	gpio_request(157, "ident1");	// identLCD1
	gpio_request(158, "ident2");	// identLCD2
	gpio_request(159, "ident3");	// identLCD3

	gpio_direction_input(157);
	gpio_direction_input(158);
	gpio_direction_input(159);

	if(16 > s_board_revision)
	{
		gpio_request(101, "ident4");	// identLCD
		gpio_direction_input(101);

		gpio_request(161, "disp");
		//gpio_direction_output(161, 0);
		gpio_direction_output(161, 1);
	}
}
#ifdef CONFIG_TI_ST
/*
 * Initialize audio related GPIOs
 *
 * TODO: Remove this function after same gpios init
 *       is done properly at u-boot  (ce300p1.c).
 */
static inline void __init
a300_init_wl1271_wifi_bt_gpios(void)
{
	printk("INIT WIFI and BT\n\n");

	if (gpio_request(138, "WiFi RST") < 0) {
		pr_err(KERN_ERR "Impossible to reset WiFi\n");
		return;
	}
	gpio_direction_output(138, 0);

	mdelay(100);

	// real enabling depends of last configuration selected by application
	gpio_set_value(138, 1);

	mdelay(100);

	gpio_free(138);
}
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

///**
// * Board specific initialization of PM components
// *
// * TODO: Adjust to A300 board !!
// */
//static void __init omap3_a300_pm_init(void)
//{
//	/* Use sys_offmode signal */
//	omap_pm_sys_offmode_select(1);
//
//	/* sys_clkreq - active high */
//	omap_pm_sys_clkreq_pol(1);
//
//	/* sys_offmode - active low */
//	omap_pm_sys_offmode_pol(0);
//
//	/* Automatically send OFF command */
//	omap_pm_auto_off(1);
//
//	/* Automatically send RET command */
//	omap_pm_auto_ret(1);
//}
static void __init
a300_init(void)
{
	int ret;

	_board_revision();

	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);

	if( 15 < s_board_revision ){
		a300_audio_data.ext_mic_gpio = 177;
		automotiveio_info.gpio_out = -1;
	}
	else{
		a300_audio_data.ext_mic_gpio = 180;
		automotiveio_info.gpio_out = 94;
	}

#if defined (CONFIG_CHARGER_BQ24172)
        if(16 < s_board_revision)
	{
                bq24172_info.charge_status_lvl = 1;
	}
        else if(16 > s_board_revision)
	{
		bq24172_info.charge_status_in = -1;
		bq24172_info.pwr_too_low_in = -1;
		bq24172_info.pwr_too_high_in = -1;
		bq24172_info.v5_good_in = -1;
	}

#endif

	a300_i2c_init();

#if defined (CONFIG_MACH_A300)
	{
		void *vaddr = phys_to_virt(0x81E00000);//(0x84000000);//(0x80088000);
		boot_fbuff = kzalloc(800 * 480 * 3, GFP_KERNEL);
		memcpy(boot_fbuff, vaddr, 800 * 480 * 3);//d1000000 size=4194304//
	}
#endif
	_lcd_config();
//	a300_display_init();

	platform_add_devices(a300_devices, ARRAY_SIZE(a300_devices));

	if('D' != *identlcd) { //resistive touchscreen
		spi_register_board_info(a300_spi_board_info, ARRAY_SIZE(a300_spi_board_info));
	}

	omap_serial_init();

	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);

	if('D' != *identlcd)                  //resistive touchscreen	
		touchscreen_pendown_gpio_init();
	
	a300_init_smsc911x();

#ifdef CONFIG_TI_ST
	a300_init_wl1271_wifi_bt_gpios();
#endif

	a300_flash_init();

	// errata 318e, advisory 1.91: pins cke0/cke1 connected to VSS in CBP package,
	// must be unconnected from memory/high impedance
	//
	/* Ensure SDRC pins are mux'd for self-refresh */
	//omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	//omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

//	omap3_a300_pm_init(); <=== TODO: UNCOMMENT TO INIT BOARD PM COMPONENTS.

#if defined (CONFIG_POWER_LOST_DETECT)
	if(16 > s_board_revision)
	{
		ret = gpio_request(178, "Super CAP feature");
		if (ret >= 0)
		{
			gpio_direction_input(178);
			if(gpio_get_value(178))
			{
				printk("Super CAP enabled\n");
				ret = gpio_request(55, "Super CAP charge");
				if (ret >= 0)
				{
					gpio_direction_output(55,1);
					printk("Super CAP charging\n");
				}
			}
			else
			{
				printk("Super CAP disapled\n");
				power_lost_detect_platform_data.pl_det = -1;
			}
		}
	}
	else
	{
		power_lost_detect_platform_data.pl_det = -1;
		power_lost_detect_platform_data.pl_act_lvl = 0;
		power_lost_detect_platform_data.rst_act_lvl = 1;
	}
#endif

	ret = gpio_request(27, "SW 3.3 V");
	if (ret >= 0)
	{
		printk("GPIO_27 -> SW 3.3 V configured ok\n");
		gpio_direction_output(27,0);
	}

	ret = gpio_request(98, "DEV_ON");
	if (ret >= 0)
	{
		printk("GPIO_98 -> DEV_ON configured ok\n");
		gpio_direction_output(98,0);
	}

	ret = gpio_request(111, "HSUSB PHY 1.8 V");
	if (ret >= 0)
	{
		printk("GPIO_111 -> HSUSB PHY 1.8V configured ok\n");
		gpio_direction_output(111,0);
	}


	ret = gpio_request(65, "PHY - nRESETB");
	if (ret >= 0)
	{
		printk("GPIO_65 -> PHY - nRESETB configured ok\n");
		gpio_direction_output(65,1);
	}

 	ret = gpio_request(54, "HUB - nRESET");
	if (ret >= 0)
	{
		printk("GPIO_54 -> HUB - nRESET configured ok\n");
		gpio_direction_output(54,0);
	}

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&a300_wlan_data))
		pr_err("error setting wl12xx data\n");

	platform_device_register(&a300_wlan_regulator);
#endif

#ifdef CONFIG_TI_ST
	a300_init_btwilink();
#endif

#ifdef CONFIG_SND_SOC_WL1271BT
	wl1271bt_clk_setup();
#endif
}

MACHINE_START(A300, "A300 Board")\
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= a300_init_irq,
	.init_machine	= a300_init,
	.timer		= &omap_timer,
MACHINE_END
