/*
 * Generic panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <plat/display.h>

struct temp_data {
	struct backlight_device *bl;
};

extern unsigned int get_ident_lcd(void);
#if 1
static struct omap_video_timings a31x_panel_timings = {
	/* 7 inch LCD */
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock= 39272,
	.hfp		= 40,
	.hsw		= 48,
	.hbp		= 40,
	.vfp		= 13,
	.vsw		= 3,
	.vbp		= 29,
};
#else
// EKing LCD
static struct omap_video_timings a31x_panel_timings = {
	/* 7 inch LCD */
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock= 33300, //39272,
	.hfp		= 208,
	.hsw		= 2,
	.hbp		= 46,
	.vfp		= 20,
	.vsw		= 2,
	.vbp		= 23,
};
#endif

static struct omap_video_timings a31x_panel_C_timings = {
	/* 4 inch LCD */
	.x_res		= 480,
	.y_res		= 272,
	.pixel_clock= KHZ2PICOS(9009),
	.hsw		= 41,
	.hfp		= 2,
	.hbp		= 2,
	.vsw		= 10,
	.vfp		= 2,
	.vbp		= 2,
};


static int a31x_panel_bl_update_status(struct backlight_device *bl)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	int level;

	if (!dssdev->set_backlight)
		return -EINVAL;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	return dssdev->set_backlight(dssdev, level);
}

static int a31x_panel_bl_get_brightness(struct backlight_device *bl)
{
	if(bl->props.fb_blank == FB_BLANK_UNBLANK && bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}

static const struct backlight_ops a31x_panel_bl_ops = {
	.get_brightness = a31x_panel_bl_get_brightness,
	.update_status  = a31x_panel_bl_update_status,
};

static int a31x_panel_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	struct temp_data *sd;
	int r;
	u32 ident_lcd = get_ident_lcd();

	//dssdev->panel.config |= OMAP_DSS_LCD_TFT;
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_ONOFF;

	if('C' == ident_lcd || 'A' == ident_lcd)
		dssdev->panel.timings = a31x_panel_C_timings;
	else
		dssdev->panel.timings = a31x_panel_timings;

	printk("a31x_panel_probe %s, %s\n", (dssdev->name)?dssdev->name:"unknown device",
			(dssdev->driver_name)?dssdev->driver_name:"unknown driver");

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if(!sd)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, sd);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = dssdev->max_backlight_level;

	bl = backlight_device_register("a31x-panel", &dssdev->dev, dssdev, &a31x_panel_bl_ops, &props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		kfree(sd);
		return r;
	}
	sd->bl = bl;

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = dssdev->max_backlight_level;
	r = a31x_panel_bl_update_status(bl);
	if (r < 0)
		dev_err(&dssdev->dev, "failed to set lcd brightness\n");

	return 0;
}

static void a31x_panel_remove(struct omap_dss_device *dssdev)
{
	struct temp_data *sd = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bl = sd->bl;

	bl->props.power = FB_BLANK_POWERDOWN;
	a31x_panel_bl_update_status(bl);
	backlight_device_unregister(bl);

	kfree(sd);
}

static int a31x_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	printk("%s: %d\n", __func__, dssdev->state);
	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	printk("%s: failure %d\n", __func__, dssdev->state);
	return r;
}

static void a31x_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);
	printk("%s: %d\n", __func__, dssdev->state);
}

static int a31x_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = a31x_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	printk("%s: %d\n", __func__, dssdev->state);
	return 0;
}

static void a31x_panel_disable(struct omap_dss_device *dssdev)
{
	a31x_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	printk("%s: %d\n", __func__, dssdev->state);
}

static int a31x_panel_suspend(struct omap_dss_device *dssdev)
{
	a31x_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	printk("%s: %d\n", __func__, dssdev->state);
	return 0;
}

static int a31x_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = a31x_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	printk("%s: %d\n", __func__, dssdev->state);
	return 0;
}

//#define DISPC_BASE			0x48050400
//#define DISPC_TIMING_H		0x0064
//#define DISPC_TIMING_V		0x0068

static void a31x_panel_set_timings(struct omap_dss_device *dssdev, struct omap_video_timings *timings)
{
//	int val;
//	void __iomem *dispc_base;

	dpi_set_timings(dssdev, timings);
#if 0
	dispc_base = ioremap(DISPC_BASE, SZ_1K);
	if(dispc_base)
	{
		val = __raw_readl(DISPC_BASE + DISPC_TIMING_H);
		printk("%s: HBP(%d), HFP(%d), HSW(%d)\n", __func__, (val >> 20) + 1, ((val >> 8) & 0x0FFF) + 1, (val & 0xFF) + 1);

		val = __raw_readl(DISPC_BASE + DISPC_TIMING_V);
		printk("%s: VBP(%d), VFP(%d), VSW(%d)\n", __func__, val >> 20, (val >> 8) & 0x0FFF, (val & 0xFF) + 1);

		iounmap(dispc_base);
	}
#endif
}

static void a31x_panel_get_timings(struct omap_dss_device *dssdev, struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int a31x_panel_check_timings(struct omap_dss_device *dssdev, struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}

static struct omap_dss_driver temp_driver = {
	.probe		= a31x_panel_probe,
	.remove		= a31x_panel_remove,

	.enable		= a31x_panel_enable,
	.disable	= a31x_panel_disable,
	.suspend	= a31x_panel_suspend,
	.resume		= a31x_panel_resume,

	.set_timings	= a31x_panel_set_timings,
	.get_timings	= a31x_panel_get_timings,
	.check_timings	= a31x_panel_check_timings,

	.driver         = {
		.name   = "a31x_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init
a31x_panel_drv_init(void)
{
	return omap_dss_register_driver(&temp_driver);
}

static void __exit
a31x_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&temp_driver);
}

module_init(a31x_panel_drv_init);
module_exit(a31x_panel_drv_exit);
MODULE_LICENSE("GPL");
