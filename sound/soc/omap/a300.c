/*
 * a300.c  --  SoC audio for A300
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"

static int a300_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt;
	int ret;

	CE300_SND_SOC_TRACE("XXXXXXXXXXXXXXXXXXXX a300_hw_params() params_channels(params)=%d \n", params_channels(params));
	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 4: /* Four channel TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	default:
		return -EINVAL;
	}

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops a300_ops = {
	.hw_params = a300_hw_params,
};

#ifdef CONFIG_SND_SOC_WL1271BT
static int a300_wl1271bt_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set cpu DAI configuration for WL1271 Bluetooth codec */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_DSP_B |
				SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "Can't set cpu DAI configuration for "\
				"WL1271 Bluetooth codec\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops a300_wl1271bt_pcm_ops = {
	.hw_params = a300_wl1271bt_pcm_hw_params,
};
#endif
/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link a300_dai[] = {
{
	.name = "TWL4030",
	.stream_name = "TWL4030",
	.cpu_dai_name = "omap-mcbsp-dai.1",
	.platform_name = "omap-pcm-audio",
	.codec_dai_name = "twl4030-hifi",
	.codec_name = "twl4030-codec",
	.ops = &a300_ops,
},
#ifdef CONFIG_SND_SOC_WL1271BT
{
	.name		= "WL1271BT",
	.stream_name	= "WL1271BT",
	.cpu_dai_name	= "omap-mcbsp-dai.0",
	.codec_dai_name	= "wl1271bt",
	.platform_name	= "omap-pcm-audio",
	.codec_name	= "wl1271bt-dummy-codec",
	.ops		= &a300_wl1271bt_pcm_ops,
},
#endif
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_a300 = {
	.name = "a300",
	.owner = THIS_MODULE,
	.dai_link = a300_dai,
	.num_links = ARRAY_SIZE(a300_dai),
};

static struct platform_device *a300_snd_device;

static int __init a300_soc_init(void)
{
	int ret;

	if (!(machine_is_a300()))
		return -ENODEV;

	pr_info("A300 SoC init\n");

	a300_snd_device = platform_device_alloc("soc-audio", -1);
	if (!a300_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(a300_snd_device, &snd_soc_a300);

	ret = platform_device_add(a300_snd_device);
	if(!ret)
	{
		return 0;
	}

	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(a300_snd_device);

	return ret;
}

static void __exit a300_soc_exit(void)
{
	platform_device_unregister(a300_snd_device);
}

module_init(a300_soc_init);
module_exit(a300_soc_exit);

MODULE_AUTHOR("Steve Sakoman <steve@sakoman.com>");
MODULE_DESCRIPTION("ALSA SoC A300");
MODULE_LICENSE("GPL");
