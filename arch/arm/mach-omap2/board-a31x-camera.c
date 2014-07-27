/*
 * linux/arch/arm/mach-omap2/board-a31x-camera.c
 *
 * (C) Copyright 2014 Micronet Ltd <http://www.micronet.co.il>
 * Vladimir Zatulovsky, vladimirz@micronet.co.il
 * A31x board
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>

#include <media/mt9v113.h>

#include <../drivers/media/video/isp/isp.h>

#include "devices.h"

#define CAM_USE_XCLKA			0

static struct regulator *a31x_cam_1v8;
static struct regulator *a31x_cam_2v8;

#ifdef CONFIG_VIDEO_MT9V113
static int a31x_mt9v113_s_power(struct v4l2_subdev *subdev, int on)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (!a31x_cam_1v8 || !a31x_cam_2v8) {
		dev_err(isp->dev, "No regulator available\n");
		return -ENODEV;
	}
	if (on) {
		/* Check Voltage-Levels */
		if (regulator_get_voltage(a31x_cam_1v8) != 1800000)
			regulator_set_voltage(a31x_cam_1v8, 1800000, 1800000);
		if (regulator_get_voltage(a31x_cam_2v8) != 1800000)
			regulator_set_voltage(a31x_cam_2v8, 1800000, 1800000);
		/*
		 * Power Up Sequence
		 */
		/* Turn on VDD */
		regulator_enable(a31x_cam_1v8);
		mdelay(1);
		regulator_enable(a31x_cam_2v8);

		mdelay(50);
		/* Enable EXTCLK */
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 24000000, CAM_USE_XCLKA);
		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 70) / 24000000) = aprox 3 us.
		 */
		udelay(3);
	} else {
		/*
		 * Power Down Sequence
		 */
		if (regulator_is_enabled(a31x_cam_1v8))
			regulator_disable(a31x_cam_1v8);
		if (regulator_is_enabled(a31x_cam_2v8))
			regulator_disable(a31x_cam_2v8);

		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 0, CAM_USE_XCLKA);
	}

	return 0;
}

static int a31x_mt9v113_configure_interface(struct v4l2_subdev *subdev, u32 pixclk)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (isp->platform_cb.set_pixel_clock)
		isp->platform_cb.set_pixel_clock(isp, pixclk);

	return 0;
}

static struct mt9v113_platform_data a31x_mt9v113_platform_data = {
	.s_power				= a31x_mt9v113_s_power,
	.configure_interface	= a31x_mt9v113_configure_interface,
};

#define CAMERA_I2C_BUS_NUM		2

static struct i2c_board_info a31x_mt9v113_i2c_devices[] = {
	{
		I2C_BOARD_INFO(MT9V113_MODULE_NAME, MT9V113_I2C_ADDR),
		.platform_data = &a31x_mt9v113_platform_data,
	},
};

static struct isp_subdev_i2c_board_info a31x_mt9v113_primary_subdevs[] = {
	{
		.board_info 	= &a31x_mt9v113_i2c_devices[0],
		.i2c_adapter_id = CAMERA_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};
#endif

static struct isp_v4l2_subdevs_group a31x_camera_subdevs[] = {
#ifdef CONFIG_VIDEO_MT9V113
	{
		.subdevs = a31x_mt9v113_primary_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = {
			.parallel = {
				.data_lane_shift	= 2,
				.clk_pol			= 0,
				.hdpol				= 0,
				.vdpol				= 0,
				.fldmode			= 0,
				.bridge				= 3,
			},
		},
	},
#endif
	{ NULL, 0 },
};

static struct isp_platform_data a31x_isp_platform_data = {
	.subdevs = a31x_camera_subdevs,
};

static int __init a31x_cam_init(void)
{
	/*
	 * Regulator supply required for camera interface
	 */

	return -ENODEV;

	a31x_cam_1v8 = regulator_get(NULL, "cam_1v8");
	if (IS_ERR(a31x_cam_1v8)) {
		printk(KERN_ERR "cam_1v8 regulator missing\n");
		return PTR_ERR(a31x_cam_1v8);
	}
	a31x_cam_2v8 = regulator_get(NULL, "cam_2v8");
	if (IS_ERR(a31x_cam_2v8)) {
		printk(KERN_ERR "cam_2v8 regulator missing\n");
		regulator_put(a31x_cam_1v8);
		return PTR_ERR(a31x_cam_2v8);
	}
	omap3_init_camera(&a31x_isp_platform_data);

	return 0;
}

static void __exit a31x_cam_exit(void)
{
	if (regulator_is_enabled(a31x_cam_1v8))
		regulator_disable(a31x_cam_1v8);
	regulator_put(a31x_cam_1v8);
	if (regulator_is_enabled(a31x_cam_2v8))
		regulator_disable(a31x_cam_2v8);
	regulator_put(a31x_cam_2v8);
}

module_init(a31x_cam_init);
module_exit(a31x_cam_exit);

MODULE_AUTHOR("Micronet Ltd");
MODULE_DESCRIPTION("a31x Camera Module");
MODULE_LICENSE("GPL");
