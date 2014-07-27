/*
 * tps65950.c - Based on sound/soc/codecs/twl4030.c
 *
 * ALSA SoC TPS65950 codec driver
 *
 *
 * Original Author:      Steve Sakoman, <steve@sakoman.com> (twl4030.c)
 * (C) Copyright 2012 - 2014 Micronet Ltd <http://www.micronet.co.il>
 * Vladimir Zatulovsky, vladimirz@micronet.co.il
 * A31x board
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

/* Register descriptions are here */
#include <linux/gpio.h>
#include <linux/mfd/twl4030-codec.h>

/* Shadow register used by the audio driver */
#define TWL4030_REG_SW_SHADOW		0x4A
#define TWL4030_CACHEREGNUM			(TWL4030_REG_SW_SHADOW + 1)

/* TWL4030_REG_SW_SHADOW (0x4A) Fields */
#define TWL4030_HFL_EN			0x01
#define TWL4030_HFR_EN			0x02

#define TWL4030_REG_DUMMY_FOR_SPEAKER 0x31
#define TWL4030_REG_DUMMY_FOR_MIC     0x32

/*
 * twl4030 register cache & default register settings
 */
static const u8 twl4030_reg[TWL4030_CACHEREGNUM] = {
	0x00, /* this register not used		*/
	0x00, /* REG_CODEC_MODE		(0x1)	*/
	0xC0, /* REG_OPTION			(0x2)	*/
	0x00, /* REG_UNKNOWN		(0x3)	*/
	0x01, /* REG_MICBIAS_CTL	(0x4)	*/
	0x11, /* REG_ANAMICL		(0x5)	*/
	0x00, /* REG_ANAMICR		(0x6)	*/
	0x08, /* REG_AVADC_CTL		(0x7)	*/
	0x00, /* REG_ADCMICSEL		(0x8)	*/
	0x00, /* REG_DIGMIXING		(0x9)	*/
	0x18, /* REG_ATXL1PGA		(0xA)	*/
	0x0f, /* REG_ATXR1PGA		(0xB)	*/
	0x00, /* REG_AVTXL2PGA		(0xC)	*/
	0x00, /* REG_AVTXR2PGA		(0xD)	*/
	0x01, /* REG_AUDIO_IF		(0xE)	*/
	0x01, /* REG_VOICE_IF		(0xF)	*/
	0x00, /* REG_ARXR1PGA		(0x10)	*/
	0x2C, /* REG_ARXL1PGA		(0x11)	*/
	0x00, /* REG_ARXR2PGA		(0x12)	*/
	0x00, /* REG_ARXL2PGA		(0x13)	*/
	0x25, /* REG_VRXPGA			(0x14)	*/
	0x00, /* REG_VSTPGA			(0x15)	*/
	0x00, /* REG_VRX2ARXPGA		(0x16)	*/
	0x02, /* REG_AVDAC_CTL		(0x17)	*/
	0x00, /* REG_ARX2VTXPGA		(0x18)	*/
	0x03, /* REG_ARXL1_APGA_CTL	(0x19)	*/
	0x00, /* REG_ARXR1_APGA_CTL	(0x1A)	*/
	0x00, /* REG_ARXL2_APGA_CTL	(0x1B)	*/
	0x00, /* REG_ARXR2_APGA_CTL	(0x1C)	*/
	0x00, /* REG_ATX2ARXPGA		(0x1D)	*/
	0x60, /* REG_BT_IF			(0x1E)	*/
	0xF0, /* REG_BTPGA			(0x1F)	*/
	0x00, /* REG_BTSTPGA		(0x20)	*/
	0x00, /* REG_EAR_CTL		(0x21)	*/
	0x00, /* REG_HS_SEL			(0x22)	*/
	0x00, /* REG_HS_GAIN_SET	(0x23)	*/
	0x46, /* REG_HS_POPN_SET	(0x24)	*/
	0x9E, /* REG_PREDL_CTL		(0x25)	*/
	0x00, /* REG_PREDR_CTL		(0x26)	*/
	0x00, /* REG_PRECKL_CTL		(0x27)	*/
	0x00, /* REG_PRECKR_CTL		(0x28)	*/
	0x00, /* REG_HFL_CTL		(0x29)	*/
	0x00, /* REG_HFR_CTL		(0x2A)	*/
	0x0D, /* REG_ALC_CTL		(0x2B)	*/
	0x00, /* REG_ALC_SET1		(0x2C)	*/
	0x64, /* REG_ALC_SET2		(0x2D)	*/
	0x01, /* REG_BOOST_CTL		(0x2E)	*/
	0x01, /* REG_SOFTVOL_CTL	(0x2F)	*/
	0x13, /* REG_DTMF_FREQSEL	(0x30)	*/
	0x00, /* REG_DTMF_TONEXT1H	(0x31)	*/
	0x00, /* REG_DTMF_TONEXT1L	(0x32)	*/
	0x00, /* REG_DTMF_TONEXT2H	(0x33)	*/
	0x00, /* REG_DTMF_TONEXT2L	(0x34)	*/
	0x79, /* REG_DTMF_TONOFF	(0x35)	*/
	0x11, /* REG_DTMF_WANONOFF	(0x36)	*/
	0x00, /* REG_I2S_RX_SCRAMBLE_H	(0x37)	*/
	0x00, /* REG_I2S_RX_SCRAMBLE_M	(0x38)	*/
	0x00, /* REG_I2S_RX_SCRAMBLE_L	(0x39)	*/
	0x06, /* REG_APLL_CTL		(0x3A)	*/
	0x00, /* REG_DTMF_CTL		(0x3B)	*/
	0x44, /* REG_DTMF_PGA_CTL2	(0x3C)	*/
	0x69, /* REG_DTMF_PGA_CTL1	(0x3D)	*/
	0x02, /* REG_MISC_SET_1		(0x3E)	*/
	0x00, /* REG_PCMBTMUX		(0x3F)	*/
	0x00, /* not used			(0x40)	*/
	0x00, /* not used			(0x41)	*/
	0x00, /* not used			(0x42)	*/
	0x0C, /* REG_RX_PATH_SEL	(0x43)	*/
	0x32, /* REG_VDL_APGA_CTL	(0x44)	*/
	0x00, /* REG_VIBRA_CTL		(0x45)	*/
	0x00, /* REG_VIBRA_SET		(0x46)	*/
	0x00, /* REG_VIBRA_PWM_SET	(0x47)	*/
	0x04, /* REG_ANAMIC_GAIN	(0x48)	*/
	0x08, /* REG_MISC_SET_2		(0x49)	*/
	0x00, /* REG_SW_SHADOW		(0x4A)	- Shadow, non HW register */
};

//#define DEBUG_TPS65950 1
#ifdef DEBUG_TPS65950
typedef struct reg_dbg {
	u8 reg_val;
	const char * reg_name;
	u8 reg_id;
} reg_debug;

static  reg_debug twl4030_reg_debug[TWL4030_CACHEREGNUM] = {
	{0x00, "this register not used", 	0x00},
	{0x00, "REG_CODEC_MODE",			0x1},
	{0x00, "REG_OPTION",				0x2},
	{0x00, "REG_UNKNOWN",				0x3},
	{0x00, "REG_MICBIAS_CTL",			0x4},
	{0x00, "REG_ANAMICL",				0x5},
	{0x00, "REG_ANAMICR",				0x6},
	{0x00, "REG_AVADC_CTL",				0x7},
	{0x00, "REG_ADCMICSEL",				0x8},
	{0x00, "REG_DIGMIXING",				0x9},
	{0x00, "REG_ATXL1PGA",				0xA},
	{0x00, "REG_ATXR1PGA",				0xB},
	{0x00, "REG_AVTXL2PGA",				0xC},
	{0x00, "REG_AVTXR2PGA",				0xD},
	{0x00, "REG_AUDIO_IF",				0xE},
	{0x00, "REG_VOICE_IF",				0xF},
	{0x00, "REG_ARXR1PGA",				0x10},
	{0x00, "REG_ARXL1PGA",				0x11},
	{0x00, "REG_ARXR2PGA",				0x12},
	{0x00, "REG_ARXL2PGA",				0x13},
	{0x00, "REG_VRXPGA",				0x14},
	{0x00, "REG_VSTPGA",				0x15},
	{0x00, "REG_VRX2ARXPGA",			0x16},
	{0x00, "REG_AVDAC_CTL",				0x17},
	{0x00, "REG_ARX2VTXPGA",			0x18},
	{0x00, "REG_ARXL1_APGA_CTL",		0x19},
	{0x00, "REG_ARXR1_APGA_CTL",		0x1A},
	{0x00, "REG_ARXL2_APGA_CTL",		0x1B},
	{0x00, "REG_ARXR2_APGA_CTL",		0x1C},
	{0x00, "REG_ATX2ARXPGA",			0x1D},
	{0x00, "REG_BT_IF",					0x1E},
	{0x00, "REG_BTPGA",					0x1F},
	{0x00, "REG_BTSTPGA",				0x20},
	{0x00, "REG_EAR_CTL",				0x21},
	{0x00, "REG_HS_SEL",				0x22},
	{0x00, "REG_HS_GAIN_SET",			0x23},
	{0x00, "REG_HS_POPN_SET",			0x24},
	{0x00, "REG_PREDL_CTL",				0x25},
	{0x00, "REG_PREDR_CTL",				0x26},
	{0x00, "REG_PRECKL_CTL",			0x27},
	{0x00, "REG_PRECKR_CTL",			0x28},
	{0x00, "REG_HFL_CTL",				0x29},
	{0x00, "REG_HFR_CTL",				0x2A},
	{0x00, "REG_ALC_CTL",				0x2B},
	{0x00, "REG_ALC_SET1",				0x2C},
	{0x00, "REG_ALC_SET2",				0x2D},
	{0x00, "REG_BOOST_CTL",				0x2E},

	{0x00, "REG_SOFTVOL_CTL",			0x2F},
	{0x00, "REG_DTMF_FREQSEL",			0x30},
	{0x00, "REG_DTMF_TONEXT1H",			0x31},
	{0x00, "REG_DTMF_TONEXT1L",			0x32},
	{0x00, "REG_DTMF_TONEXT2H",			0x33},
	{0x00, "REG_DTMF_TONEXT2L",			0x34},
	{0x00, "REG_DTMF_TONOFF",			0x35},
	{0x00, "REG_DTMF_WANONOFF",			0x36},
	{0x00, "REG_I2S_RX_SCRAMBLE_H",		0x37},
	{0x00, "REG_I2S_RX_SCRAMBLE_M",		0x38},
	{0x00, "REG_I2S_RX_SCRAMBLE_L",		0x39},
	{0x00, "REG_APLL_CTL",				0x3A},
	{0x00, "REG_DTMF_CTL",				0x3B},
	{0x00, "REG_DTMF_PGA_CTL2",			0x3C},

	{0x00, "REG_DTMF_PGA_CTL1",			0x3D},
	{0x00, "REG_MISC_SET_1",			0x3E},
	{0x00, "REG_PCMBTMUX",				0x3F},
	{0x00, "not used1",					0x40},
	{0x00, "not used2",					0x41},
	{0x00, "not used3",					0x42},
	{0x00, "REG_RX_PATH_SEL",			0x43},
	{0x00, "REG_VDL_APGA_CTL",			0x44},
	{0x00, "REG_VIBRA_CTL",				0x45},
	{0x00, "REG_VIBRA_SET",				0x46},
	{0x00, "REG_VIBRA_PWM_SET",			0x47},
	{0x00, "REG_ANAMIC_GAIN",			0x48},
	{0x00, "REG_MISC_SET_2",			0x49},
	{0x00, "REG_SW_SHADOW",				0x4A}
};
#endif

/* codec private data */
struct twl4030_priv {
	struct snd_soc_codec codec;

	unsigned int codec_powered;

	/* reference counts of AIF/APLL users */
	unsigned int apll_enabled;

	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;

	unsigned int configured;
	unsigned int rate;
	unsigned int sample_bits;
	unsigned int channels;

	unsigned int sysclk;

	/* Output (with associated amp) states */
	u8 hsl_enabled, hsr_enabled;
	u8 earpiece_enabled;
	u8 predrivel_enabled, predriver_enabled;
	u8 carkitl_enabled, carkitr_enabled;

	/* Delay needed after enabling the digimic interface */
	unsigned int digimic_delay;

	/* Needed for HW GPIO input/output routing */
	unsigned int ext_speaker_gpio; // external speaker amplifier
	unsigned int int_speaker_gpio; // internal speaker amplifier
	unsigned int ext_mic_gpio;     // external microphone
	unsigned int cradle_aud_gpio;  // cradle audio switch
	u8 ext_mic_enabled;
	u8 ext_spkr_enabled;
};

/*
 * read twl4030 register cache
 */
static inline unsigned int twl4030_read_reg_cache(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL4030_CACHEREGNUM)
		return -EIO;

	return cache[reg];
}

/*
 * write twl4030 register cache
 */
static inline void twl4030_write_reg_cache(struct snd_soc_codec *codec, u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if(reg >= TWL4030_CACHEREGNUM)
		return;

	cache[reg] = value;
}

#ifdef DEBUG_TPS65950
static void dump_reg_vals(struct snd_soc_codec *codec){
	u8 reg, val;
	printk("%s:\tCache\tHW-Value\tName\n", __func__);

	for(reg = 0; reg < TWL4030_CACHEREGNUM; reg++){
        twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &val, reg);
		printk("%s: \t%2X\t%2X\t%s\n", __func__, reg, twl4030_read_reg_cache(codec, reg), val, twl4030_reg_debug[reg].reg_name);
	}
}
#endif

static void set_codec_output_gain(struct snd_soc_codec *codec, int external){
//	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

#ifdef DEBUG_TPS65930
	dump_reg_vals(codec);
#endif

	if(!(external)){

	} else {

	}

}

static void set_codec_output_routing(struct snd_soc_codec *codec, int external){

	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	switch(external){
		case 0: // Internal Audio
		case 4: // Default state set when codec is enabled
			if(twl4030->codec_powered)
			{
				mdelay(25);
				if(twl4030->int_speaker_gpio != -1)
					gpio_direction_output(twl4030->int_speaker_gpio, 1);
				if(twl4030->ext_speaker_gpio != -1)
					gpio_direction_output(twl4030->ext_speaker_gpio, 0);
			}
			twl4030->ext_spkr_enabled = 0;
			break;
		case 1:
			if(twl4030->codec_powered)
			{
				mdelay(25);
				if(twl4030->int_speaker_gpio != -1)
					gpio_direction_output(twl4030->int_speaker_gpio, 0);
				if(twl4030->ext_speaker_gpio != -1)
					gpio_direction_output(twl4030->ext_speaker_gpio, 1);
			}

			twl4030->ext_spkr_enabled = 1;
			break;
		case 2:
			//Cradle audio switch option
			if(twl4030->cradle_aud_gpio != -1)
				gpio_direction_output(twl4030->cradle_aud_gpio, 0);
			break;
		case 3: //Cradle audio switch option
			if(twl4030->cradle_aud_gpio != -1)
				gpio_direction_output(twl4030->cradle_aud_gpio, 1);
			break;
		default:
			break;
	}
}

static void set_codec_input_routing(struct snd_soc_codec *codec, int external){
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

#ifdef DEBUG_TPS65950
	dump_reg_vals(codec);
#endif

	if(twl4030->ext_mic_gpio != -1){
		if(external)
			gpio_direction_output(twl4030->ext_mic_gpio, 1);
		else
			gpio_direction_output(twl4030->ext_mic_gpio, 0);

		twl4030->ext_mic_enabled = external;

		mdelay(25);
	}
}

/*
 * write to the twl4030 register space
 */
static int twl4030_write(struct snd_soc_codec *codec, unsigned int reg, unsigned int value)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	int write_to_reg = 0;

	twl4030_write_reg_cache(codec, reg, value);

	if(likely(reg < TWL4030_REG_SW_SHADOW)){
		/* Decide if the given register can be written */
		switch(reg){
			case TWL4030_REG_EAR_CTL:
				if(twl4030->earpiece_enabled)
					write_to_reg = 1;
				break;
			case TWL4030_REG_PREDL_CTL:
				if(twl4030->predrivel_enabled)
					write_to_reg = 1;
				break;
			case TWL4030_REG_PREDR_CTL:
				if(twl4030->predriver_enabled)
					write_to_reg = 1;
				break;
			case TWL4030_REG_PRECKL_CTL:
				if(twl4030->carkitl_enabled)
					write_to_reg = 1;
				break;
			case TWL4030_REG_PRECKR_CTL:
				if(twl4030->carkitr_enabled)
					write_to_reg = 1;
				break;
			case TWL4030_REG_HS_GAIN_SET:
				if(twl4030->hsl_enabled || twl4030->hsr_enabled)
					write_to_reg = 1;
				break;
			case TWL4030_REG_DUMMY_FOR_SPEAKER:
				set_codec_output_gain(codec, value);
				set_codec_output_routing(codec, value);
				write_to_reg = 1;
				break;
			case TWL4030_REG_DUMMY_FOR_MIC:
				set_codec_input_routing(codec, value);
				break;
			default:
				/* All other register can be written */
				write_to_reg = 1;
				break;
		}

		if(write_to_reg)
			return twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE, value, reg);
	}

	return 0;
}

static void twl4030_codec_enable(struct snd_soc_codec *codec, int enable)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	int 	mode;

	if(enable == twl4030->codec_powered)
		return;

	if(enable)
		mode = twl4030_codec_enable_resource(TWL4030_CODEC_RES_POWER);
	else
		mode = twl4030_codec_disable_resource(TWL4030_CODEC_RES_POWER);

	if(mode < 0)
		return;

	twl4030_write_reg_cache(codec, TWL4030_REG_CODEC_MODE, mode);
	twl4030->codec_powered = enable;

	/* REVISIT: this delay is present in TI sample drivers */
	/* but there seems to be no TRM requirement for it     */
	udelay(10);
}

static inline void twl4030_check_defaults(struct snd_soc_codec *codec)
{
	int i, difference = 0;
	u8 val;

	printk("%s: Check audio default configuration\n", __func__);

	for(i = 1; i <= TWL4030_REG_MISC_SET_2; i++){
		twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &val, i);
		if(val != twl4030_reg[i]){
			difference++;
#ifdef DEBUG_TPS65950
			printk("%s: %s 0x%02x NEQ 0x%02x\n", __func__, twl4030_reg_debug[i].reg_name, val, twl4030_reg[i]);
#endif
		}
	}

	printk("%s: Found %d non matching registers. %s\n", __func__, difference, difference ? "Not OK" : "OK");
}

static inline void twl4030_reset_registers(struct snd_soc_codec *codec)
{
	int i;

	/* set all audio section registers to reasonable defaults */
	for(i = TWL4030_REG_OPTION; i <= TWL4030_REG_MISC_SET_2; i++)
		if(i != TWL4030_REG_APLL_CTL)
			twl4030_write(codec, i, twl4030_reg[i]);

}

static void twl4030_init_chip(struct snd_soc_codec *codec)
{
	struct twl4030_codec_audio_data *pdata 	 = dev_get_platdata(codec->dev);
	struct twl4030_priv 			*twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 reg, byte;
	int i = 0;

	/* Check defaults, if instructed before anything else */
	if(pdata && pdata->check_defaults)
		twl4030_check_defaults(codec);

	/* Reset registers, if no setup data or if instructed to do so */
	if(!pdata || (pdata && pdata->reset_registers))
		twl4030_reset_registers(codec);

	/* Refresh APLL_CTL register from HW */
	twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &byte, TWL4030_REG_APLL_CTL);
	twl4030_write_reg_cache(codec, TWL4030_REG_APLL_CTL, byte);

	/* anti-pop when changing analog gain */
	reg = twl4030_read_reg_cache(codec, TWL4030_REG_MISC_SET_1);
	twl4030_write(codec, TWL4030_REG_MISC_SET_1, reg | TWL4030_SMOOTH_ANAVOL_EN);

//	twl4030_write(codec, TWL4030_REG_OPTION, TWL4030_ATXL1_EN | TWL4030_ATXR1_EN | TWL4030_ARXL2_EN | TWL4030_ARXR2_EN);
	twl4030_write(codec, TWL4030_REG_OPTION, TWL4030_ATXL1_EN | TWL4030_ARXL2_EN | TWL4030_ARXR2_EN);

	/* REG_ARXR2_APGA_CTL reset according to the TRM: 0dB, DA_EN */
//	twl4030_write(codec, TWL4030_REG_ARXR2_APGA_CTL, 0x32);

	/* Machine dependent setup */
	if(!pdata)
		return;

	reg = twl4030_read_reg_cache(codec, TWL4030_REG_HS_POPN_SET);
	reg &= ~TWL4030_RAMP_DELAY;
	reg |= (pdata->ramp_delay_value << 2);
	twl4030_write_reg_cache(codec, TWL4030_REG_HS_POPN_SET, reg);

	/* initiate offset cancellation */
	twl4030_codec_enable(codec, 1);

	reg = twl4030_read_reg_cache(codec, TWL4030_REG_ANAMICL);
	reg &= ~TWL4030_OFFSET_CNCL_SEL;
	reg |= pdata->offset_cncl_path;
	twl4030_write(codec, TWL4030_REG_ANAMICL, reg | TWL4030_CNCL_OFFSET_START);

	/* wait for offset cancellation to complete */
	do {
		/* this takes a little while, so don't slam i2c */
		udelay(2000);
		twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &byte, TWL4030_REG_ANAMICL);
	} while ((i++ < 100) && ((byte & TWL4030_CNCL_OFFSET_START) == TWL4030_CNCL_OFFSET_START));

	/* Make sure that the reg_cache has the same value as the HW */
	twl4030_write_reg_cache(codec, TWL4030_REG_ANAMICL, byte);

	twl4030_codec_enable(codec, 0);
}

static void twl4030_apll_enable(struct snd_soc_codec *codec, int enable)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	int status = -1;

	if (enable) {
		twl4030->apll_enabled++;
		if (twl4030->apll_enabled == 1)
			status = twl4030_codec_enable_resource(TWL4030_CODEC_RES_APLL);
	} else {
		twl4030->apll_enabled--;
		if (!twl4030->apll_enabled)
			status = twl4030_codec_disable_resource(TWL4030_CODEC_RES_APLL);
	}

	if (status >= 0)
		twl4030_write_reg_cache(codec, TWL4030_REG_APLL_CTL, status);
}

/* PreDrive Left */
static const struct snd_kcontrol_new twl4030_dapm_predrivel_controls[] = {
//	SOC_DAPM_SINGLE("Voice", 	TWL4030_REG_PREDL_CTL, 0, 1, 0), // TODO: use with modem PCM
	SOC_DAPM_SINGLE("Switch", 	TWL4030_REG_PREDL_CTL, 1, 1, 0),
};

/* Left analog microphone selection */
static const struct snd_kcontrol_new twl4030_dapm_analoglmic_controls[] = {
	SOC_DAPM_SINGLE("Main Mic Capture Switch", 	TWL4030_REG_ANAMICL, 0, 1, 0),
	SOC_DAPM_SINGLE("AUXL Capture Switch", 		TWL4030_REG_ANAMICL, 2, 1, 0),
};

/* TX1 L/R Analog/Digital microphone selection */
static const char *twl4030_micpathtx1_texts[] = {"Analog", "Digimic0"};

static const struct soc_enum twl4030_micpathtx1_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_ADCMICSEL, 0, ARRAY_SIZE(twl4030_micpathtx1_texts), twl4030_micpathtx1_texts);

static const struct snd_kcontrol_new twl4030_dapm_micpathtx1_control =
SOC_DAPM_ENUM("Route", twl4030_micpathtx1_enum);

/* Analog bypass for AudioR1 */
static const struct snd_kcontrol_new twl4030_dapm_abypassr1_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_ARXR1_APGA_CTL, 2, 1, 0);

/* Analog bypass for AudioL1 */
static const struct snd_kcontrol_new twl4030_dapm_abypassl1_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_ARXL1_APGA_CTL, 2, 1, 0);

/* Analog bypass for AudioR2 */
static const struct snd_kcontrol_new twl4030_dapm_abypassr2_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_ARXR2_APGA_CTL, 2, 1, 0);

/* Analog bypass for AudioL2 */
static const struct snd_kcontrol_new twl4030_dapm_abypassl2_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_ARXL2_APGA_CTL, 2, 1, 0);

/* Analog bypass for Voice */
static const struct snd_kcontrol_new twl4030_dapm_abypassv_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_VDL_APGA_CTL, 2, 1, 0);

/* Digital bypass gain, mute instead of -30dB */
static const unsigned int twl4030_dapm_dbypass_tlv[] = {
	TLV_DB_RANGE_HEAD(3),
	0, 1, TLV_DB_SCALE_ITEM(-3000, 600, 1),
	2, 3, TLV_DB_SCALE_ITEM(-2400, 0, 0),
	4, 7, TLV_DB_SCALE_ITEM(-1800, 600, 0),
};

/* Digital bypass left (TX1L -> RX2L) */
static const struct snd_kcontrol_new twl4030_dapm_dbypassl_control =
	SOC_DAPM_SINGLE_TLV("Volume",
			TWL4030_REG_ATX2ARXPGA, 3, 7, 0,
			twl4030_dapm_dbypass_tlv);

/* Digital bypass right (TX1R -> RX2R) */
static const struct snd_kcontrol_new twl4030_dapm_dbypassr_control =
	SOC_DAPM_SINGLE_TLV("Volume",
			TWL4030_REG_ATX2ARXPGA, 0, 7, 0,
			twl4030_dapm_dbypass_tlv);

/*
 * Voice Sidetone GAIN volume control:
 * from -51 to -10 dB in 1 dB steps (mute instead of -51 dB)
 */
static DECLARE_TLV_DB_SCALE(twl4030_dapm_dbypassv_tlv, -5100, 100, 1);

/* Digital bypass voice: sidetone (VUL -> VDL)*/
static const struct snd_kcontrol_new twl4030_dapm_dbypassv_control =
	SOC_DAPM_SINGLE_TLV("Volume",
			TWL4030_REG_VSTPGA, 0, 0x29, 0,
			twl4030_dapm_dbypassv_tlv);

/* External/Internal speaker selection */
static const char *twl4030_speakerroute_texts[] =
		{"Internal", "External", "Aux & Internal off", "Aux & Internal On", "default"};

static const struct soc_enum twl4030_speakerroute_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_DUMMY_FOR_SPEAKER, 0, ARRAY_SIZE(twl4030_speakerroute_texts), twl4030_speakerroute_texts);

static const struct snd_kcontrol_new twl4030_dapm_speakerroute_control =
SOC_DAPM_ENUM("Route", twl4030_speakerroute_enum);

static const char *twl4030_microute_texts[] = {"Internal", "External"};

static const struct soc_enum twl4030_microute_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_DUMMY_FOR_MIC, 0,
			ARRAY_SIZE(twl4030_microute_texts),
			twl4030_microute_texts);

static const struct snd_kcontrol_new twl4030_dapm_microute_control =
SOC_DAPM_ENUM("Route", twl4030_microute_enum);

/*
 * Output PGA builder:
 * Handle the muting and unmuting of the given output (turning off the
 * amplifier associated with the output pin)
 * On mute bypass the reg_cache and write 0 to the register
 * On unmute: restore the register content from the reg_cache
 * Outputs handled in this way:  Earpiece, PreDrivL/R, CarkitL/R
 */
#define TWL4030_OUTPUT_PGA(pin_name, reg, mask)				\
static int pin_name##pga_event(struct snd_soc_dapm_widget *w,		\
		struct snd_kcontrol *kcontrol, int event)		\
{									\
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(w->codec); \
									\
	switch (event) {						\
	case SND_SOC_DAPM_POST_PMU:					\
		twl4030->pin_name##_enabled = 1;			\
		twl4030_write(w->codec, reg,				\
			twl4030_read_reg_cache(w->codec, reg));		\
		break;							\
	case SND_SOC_DAPM_POST_PMD:					\
		twl4030->pin_name##_enabled = 0;			\
		twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,		\
					0, reg);			\
		break;							\
	}								\
	return 0;							\
}

TWL4030_OUTPUT_PGA(predrivel, TWL4030_REG_PREDL_CTL, TWL4030_PREDL_GAIN);

static int apll_event(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		twl4030_apll_enable(w->codec, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		twl4030_apll_enable(w->codec, 0);
		break;
	}
	return 0;
}

static int aif_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	u8 audio_if;

	audio_if = twl4030_read_reg_cache(w->codec, TWL4030_REG_AUDIO_IF);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enable AIF */
		/* enable the PLL before we use it to clock the DAI */
		twl4030_apll_enable(w->codec, 1);

		twl4030_write(w->codec, TWL4030_REG_AUDIO_IF,
						audio_if | TWL4030_AIF_EN);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* disable the DAI before we stop it's source PLL */
		twl4030_write(w->codec, TWL4030_REG_AUDIO_IF,
						audio_if &  ~TWL4030_AIF_EN);
		twl4030_apll_enable(w->codec, 0);
		break;
	}
	return 0;
}

static int digimic_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(w->codec);

	if (twl4030->digimic_delay)
		mdelay(twl4030->digimic_delay);
	return 0;
}

/*
 * Some of the gain controls in TWL (mostly those which are associated with
 * the outputs) are implemented in an interesting way:
 * 0x0 : Power down (mute)
 * 0x1 : 6dB
 * 0x2 : 0 dB
 * 0x3 : -6 dB
 * Inverting not going to help with these.
 * Custom volsw and volsw_2r get/put functions to handle these gain bits.
 */
#define SOC_DOUBLE_TLV_TWL4030(xname, xreg, shift_left, shift_right, xmax,\
			       xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw_twl4030, \
	.put = snd_soc_put_volsw_twl4030, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .shift = shift_left, .rshift = shift_right,\
		 .max = xmax, .invert = xinvert} }
#define SOC_DOUBLE_R_TLV_TWL4030(xname, reg_left, reg_right, xshift, xmax,\
				 xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r, \
	.get = snd_soc_get_volsw_r2_twl4030,\
	.put = snd_soc_put_volsw_r2_twl4030, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		 .rshift = xshift, .max = xmax, .invert = xinvert} }
#define SOC_SINGLE_TLV_TWL4030(xname, xreg, xshift, xmax, xinvert, tlv_array) \
	SOC_DOUBLE_TLV_TWL4030(xname, xreg, xshift, xshift, xmax, \
			       xinvert, tlv_array)

static int snd_soc_get_volsw_twl4030(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	if (ucontrol->value.integer.value[0])
		ucontrol->value.integer.value[0] =
			max + 1 - ucontrol->value.integer.value[0];

	if (shift != rshift) {
		ucontrol->value.integer.value[1] =
			(snd_soc_read(codec, reg) >> rshift) & mask;
		if (ucontrol->value.integer.value[1])
			ucontrol->value.integer.value[1] =
				max + 1 - ucontrol->value.integer.value[1];
	}

	return 0;
}

static int snd_soc_put_volsw_twl4030(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	unsigned short val, val2, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);

	val_mask = mask << shift;
	if (val)
		val = max + 1 - val;
	val = val << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		val_mask |= mask << rshift;
		if (val2)
			val2 = max + 1 - val2;
		val |= val2 << rshift;
	}

	return snd_soc_update_bits(codec, reg, val_mask, val);
}

static int snd_soc_get_volsw_r2_twl4030(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	int mask = (1<<fls(max))-1;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	ucontrol->value.integer.value[1] =
		(snd_soc_read(codec, reg2) >> shift) & mask;

	if (ucontrol->value.integer.value[0])
		ucontrol->value.integer.value[0] =
			max + 1 - ucontrol->value.integer.value[0];
	if (ucontrol->value.integer.value[1])
		ucontrol->value.integer.value[1] =
			max + 1 - ucontrol->value.integer.value[1];

	return 0;
}

static int snd_soc_put_volsw_r2_twl4030(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	int err;
	unsigned short val, val2, val_mask;

	val_mask = mask << shift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	if (val)
		val = max + 1 - val;
	if (val2)
		val2 = max + 1 - val2;

	val = val << shift;
	val2 = val2 << shift;

	err = snd_soc_update_bits(codec, reg, val_mask, val);
	if (err < 0)
		return err;

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/* Codec operation modes */
static const char *twl4030_op_modes_texts[] = {
	"Option 2 (voice/audio)", "Option 1 (audio)"
};

static const struct soc_enum twl4030_op_modes_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_CODEC_MODE, 0,
			ARRAY_SIZE(twl4030_op_modes_texts),
			twl4030_op_modes_texts);

static int snd_soc_put_twl4030_opmode_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned short val;
	unsigned short mask, bitmask;

	if (twl4030->configured) {
		printk(KERN_ERR "twl4030 operation mode cannot be "
			"changed on-the-fly\n");
		return -EBUSY;
	}

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	val = ucontrol->value.enumerated.item[0] << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

	return snd_soc_update_bits(codec, e->reg, mask, val);
}

/*
 * FGAIN volume control:
 * from -62 to 0 dB in 1 dB steps (mute instead of -63 dB)
 */
static DECLARE_TLV_DB_SCALE(digital_fine_tlv, -6300, 100, 1);

/*
 * CGAIN volume control:
 * 0 dB to 12 dB in 6 dB steps
 * value 2 and 3 means 12 dB
 */
static DECLARE_TLV_DB_SCALE(digital_coarse_tlv, 0, 600, 0);

/*
 * Analog play back gain
 * -24 dB to 12 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(analog_tlv, -2400, 200, 0);

/*
 * Gain controls tied to outputs
 * -6 dB to 6 dB in 6 dB steps (mute instead of -12)
 */
static DECLARE_TLV_DB_SCALE(output_tvl, -1200, 600, 1);

/*
 * Capture gain after the ADCs
 * from 0 dB to 31 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(digital_capture_tlv, 0, 100, 0);

/*
 * Gain control for input amplifiers
 * 0 dB to 30 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(input_gain_tlv, 0, 600, 0);

static const struct snd_kcontrol_new twl4030_snd_controls[] = {
	/* Common play back gain controls */
	SOC_SINGLE_TLV("Speaker Digital Fine Volume", 	TWL4030_REG_ARXL1PGA, 		0, 0x2C, 0, digital_fine_tlv),
	SOC_SINGLE_TLV("Speaker Digital Coarse Volume", TWL4030_REG_ARXL1PGA, 		6, 0x00, 0, digital_coarse_tlv),
	SOC_SINGLE_TLV("Speaker Analog Volume",			TWL4030_REG_ARXL1_APGA_CTL, 3, 0x00, 1, analog_tlv),
	SOC_SINGLE	  ("Speaker Analog Switch", 		TWL4030_REG_ARXL1_APGA_CTL, 3, 0x00, 0),
	/* Separate output gain controls */
	SOC_SINGLE_TLV_TWL4030("Speaker Master Volume",	TWL4030_REG_PREDL_CTL, 4, 3, 0, output_tvl),

	/* Common capture gain controls */
	SOC_DOUBLE_R_TLV("Microphone Digital Volume",	TWL4030_REG_ATXL1PGA, TWL4030_REG_ATXR1PGA,
													0, 0x18, 0, digital_capture_tlv),
	SOC_DOUBLE_TLV("Microphone Analog Volume", 	  	TWL4030_REG_ANAMIC_GAIN, 0, 3, 4, 0, input_gain_tlv),
};

static const struct snd_soc_dapm_widget twl4030_dapm_widgets[] = {
	/* Left channel inputs */
	SND_SOC_DAPM_INPUT("MAINMIC"),
	SND_SOC_DAPM_INPUT("AUXL"),
	/* Right channel inputs */
	SND_SOC_DAPM_INPUT("AUXR"),
	/* Digital microphones (Stereo) */
	SND_SOC_DAPM_INPUT("DIGIMIC0"),
	SND_SOC_DAPM_INPUT("DIGIMIC1"),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("PREDRIVEL"),

	/* AIF and APLL clocks for running DAIs (including loopback) */
	SND_SOC_DAPM_OUTPUT("Virtual HiFi OUT"),
	SND_SOC_DAPM_INPUT("Virtual HiFi IN"),
	SND_SOC_DAPM_OUTPUT("Virtual Voice OUT"),

	/* DACs */
	SND_SOC_DAPM_DAC("DAC Right1", "Right Front HiFi Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Left1", "Left Front HiFi Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Right2", "Right Rear HiFi Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Left2", "Left Rear HiFi Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Voice", "Voice Playback",
			SND_SOC_NOPM, 0, 0),

	/* Analog bypasses */
	SND_SOC_DAPM_SWITCH("Right1 Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassr1_control),
	SND_SOC_DAPM_SWITCH("Left1 Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassl1_control),
	SND_SOC_DAPM_SWITCH("Right2 Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassr2_control),
	SND_SOC_DAPM_SWITCH("Left2 Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassl2_control),
	SND_SOC_DAPM_SWITCH("Voice Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassv_control),

	/* Master analog loopback switch */
	SND_SOC_DAPM_SUPPLY("FM Loop Enable", TWL4030_REG_MISC_SET_1, 5, 0,
			    NULL, 0),

	/* Digital bypasses */
	SND_SOC_DAPM_SWITCH("Left Digital Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_dbypassl_control),
	SND_SOC_DAPM_SWITCH("Right Digital Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_dbypassr_control),
	SND_SOC_DAPM_SWITCH("Voice Digital Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_dbypassv_control),

	/* Digital mixers, power control for the physical DACs */
	SND_SOC_DAPM_MIXER("Digital R1 Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Digital L1 Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Digital R2 Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Digital L2 Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 3, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Digital Voice Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 4, 0, NULL, 0),

	/* Analog mixers, power control for the physical PGAs */
	SND_SOC_DAPM_MIXER("Analog R1 Playback Mixer",
			TWL4030_REG_ARXR1_APGA_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Analog L1 Playback Mixer",
			TWL4030_REG_ARXL1_APGA_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Analog R2 Playback Mixer",
			TWL4030_REG_ARXR2_APGA_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Analog L2 Playback Mixer",
			TWL4030_REG_ARXL2_APGA_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Analog Voice Playback Mixer",
			TWL4030_REG_VDL_APGA_CTL, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("APLL Enable", SND_SOC_NOPM, 0, 0, apll_event,
			    SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("AIF Enable", SND_SOC_NOPM, 0, 0, aif_event,
			    SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMD),

	/* Output MIXER controls */
	/* PreDrivL/R */
	SND_SOC_DAPM_MIXER("Speaker Mixer", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_predrivel_controls[0],
			ARRAY_SIZE(twl4030_dapm_predrivel_controls)),
	SND_SOC_DAPM_PGA_E("PredriveL PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, predrivelpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),

	/* Output MUX controls */

	/* Introducing four virtual ADC, since TWL4030 have four channel for
	   capture */
	SND_SOC_DAPM_ADC("ADC Virtual Left1", "Left Front Capture",
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC Virtual Right1", "Right Front Capture",
		SND_SOC_NOPM, 0, 0),

	/* Analog/Digital mic path selection.
	   TX1 Left/Right: either analog Left/Right or Digimic0
	   TX2 Left/Right: either analog Left/Right or Digimic1 */
	SND_SOC_DAPM_MUX("Microphone Capture Route", SND_SOC_NOPM, 0, 0, &twl4030_dapm_micpathtx1_control),

	/* Analog input mixers for the capture amplifiers */
	SND_SOC_DAPM_MIXER("Analog Left",
		TWL4030_REG_ANAMICL, 4, 0,
		&twl4030_dapm_analoglmic_controls[0],
		ARRAY_SIZE(twl4030_dapm_analoglmic_controls)),
//	SND_SOC_DAPM_MIXER("Analog Right",
//		TWL4030_REG_ANAMICR, 4, 0,
//		&twl4030_dapm_analogrmic_controls[0],
//		ARRAY_SIZE(twl4030_dapm_analogrmic_controls)),

	SND_SOC_DAPM_PGA("ADC Physical Left",
		TWL4030_REG_AVADC_CTL, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC Physical Right",
		TWL4030_REG_AVADC_CTL, 1, 0, NULL, 0),

	SND_SOC_DAPM_PGA_E("Digimic0 Enable",
		TWL4030_REG_ADCMICSEL, 1, 0, NULL, 0,
		digimic_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_E("Digimic1 Enable",
		TWL4030_REG_ADCMICSEL, 3, 0, NULL, 0,
		digimic_event, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_SUPPLY("micbias1 select", TWL4030_REG_MICBIAS_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("micbias2 select", TWL4030_REG_MICBIAS_CTL, 6, 0, NULL, 0),
	SND_SOC_DAPM_MICBIAS("Mic Bias 1", TWL4030_REG_MICBIAS_CTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Mic Bias 2", TWL4030_REG_MICBIAS_CTL, 1, 0),
	SND_SOC_DAPM_MUX("Microphone Route", SND_SOC_NOPM, 0, 0, &twl4030_dapm_microute_control),
	SND_SOC_DAPM_MUX("Speaker Route", SND_SOC_NOPM, 0, 0, &twl4030_dapm_speakerroute_control),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"Digital L1 Playback Mixer", NULL, "DAC Left1"},
	{"Digital R1 Playback Mixer", NULL, "DAC Right1"},
	{"Digital L2 Playback Mixer", NULL, "DAC Left2"},
	{"Digital R2 Playback Mixer", NULL, "DAC Right2"},
	{"Digital Voice Playback Mixer", NULL, "DAC Voice"},

	/* Supply for the digital part (APLL) */
	{"Digital Voice Playback Mixer", NULL, "APLL Enable"},

	{"DAC Left1", NULL, "AIF Enable"},
	{"DAC Right1", NULL, "AIF Enable"},
	{"DAC Left2", NULL, "AIF Enable"},
	{"DAC Right1", NULL, "AIF Enable"},

	{"Digital R2 Playback Mixer", NULL, "AIF Enable"},
	{"Digital L2 Playback Mixer", NULL, "AIF Enable"},

	{"Analog L1 Playback Mixer", NULL, "Digital L1 Playback Mixer"},
	{"Analog R1 Playback Mixer", NULL, "Digital R1 Playback Mixer"},
	{"Analog L2 Playback Mixer", NULL, "Digital L2 Playback Mixer"},
	{"Analog R2 Playback Mixer", NULL, "Digital R2 Playback Mixer"},
	{"Analog Voice Playback Mixer", NULL, "Digital Voice Playback Mixer"},

	/* Internal playback routings */
	/* PreDrivL */
	{"Speaker Mixer", "Switch", "Analog L1 Playback Mixer"},
	{"PredriveL PGA", NULL, "Speaker Mixer"},

	/* outputs */
	/* Must be always connected (for AIF and APLL) */
	{"Virtual HiFi OUT", NULL, "DAC Left1"},
	{"Virtual HiFi OUT", NULL, "DAC Right1"},
	{"Virtual HiFi OUT", NULL, "DAC Left2"},
	{"Virtual HiFi OUT", NULL, "DAC Right2"},
	/* Must be always connected (for APLL) */
	{"Virtual Voice OUT", NULL, "Digital Voice Playback Mixer"},
	/* Physical outputs */
	{"PREDRIVEL", NULL, "PredriveL PGA"},

	/* Capture path */
	/* Must be always connected (for AIF and APLL) */
	{"ADC Virtual Left1", NULL, "Virtual HiFi IN"},
	{"ADC Virtual Right1", NULL, "Virtual HiFi IN"},
	/* Physical inputs */
	{"Analog Left", "Main Mic Capture Switch", "MAINMIC"},

	{"ADC Physical Left", NULL, "Analog Left"},
	{"ADC Physical Right", NULL, "Analog Right"},

	{"Digimic0 Enable", NULL, "DIGIMIC0"},
	{"Digimic1 Enable", NULL, "DIGIMIC1"},

	{"DIGIMIC0", NULL, "micbias1 select"},
	{"DIGIMIC1", NULL, "micbias2 select"},

	/* TX1 Left capture path */
	{"Microphone Capture Route", "Analog", "ADC Physical Left"},
	{"Microphone Capture Route", "Digimic0", "Digimic0 Enable"},
	/* TX1 Right capture path */
	{"Microphone Capture Route", "Analog", "ADC Physical Right"},
	{"Microphone Capture Route", "Digimic0", "Digimic0 Enable"},

	{"ADC Virtual Left1", NULL, "Microphone Capture Route"},
	{"ADC Virtual Right1", NULL, "Microphone Capture Route"},

	{"ADC Virtual Left1", NULL, "AIF Enable"},
	{"ADC Virtual Right1", NULL, "AIF Enable"},

	/* Analog bypass routes */
	{"Analog L1 Playback Mixer", NULL, "Left1 Analog Loopback"},
	{"Analog R2 Playback Mixer", NULL, "Right2 Analog Loopback"},
	{"Analog L2 Playback Mixer", NULL, "Left2 Analog Loopback"},
	{"Analog Voice Playback Mixer", NULL, "Voice Analog Loopback"},

	/* Digital bypass routes */
	{"Digital R2 Playback Mixer", NULL, "Right Digital Loopback"},
	{"Digital L2 Playback Mixer", NULL, "Left Digital Loopback"},
	{"Digital Voice Playback Mixer", NULL, "Voice Digital Loopback"},

	{"Speaker Route", NULL, "PredriveL PGA"},
};

static int twl4030_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, twl4030_dapm_widgets,
				 ARRAY_SIZE(twl4030_dapm_widgets));

	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	return 0;
}

static int twl4030_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->bias_level == SND_SOC_BIAS_OFF)
			twl4030_codec_enable(codec, 1);
		break;
	case SND_SOC_BIAS_OFF:
		twl4030_codec_enable(codec, 0);
		break;
	}
	codec->bias_level = level;

	return 0;
}

static void twl4030_constraints(struct twl4030_priv *twl4030,
				struct snd_pcm_substream *mst_substream)
{
	struct snd_pcm_substream *slv_substream;

	/* Pick the stream, which need to be constrained */
	if (mst_substream == twl4030->master_substream)
		slv_substream = twl4030->slave_substream;
	else if (mst_substream == twl4030->slave_substream)
		slv_substream = twl4030->master_substream;
	else /* This should not happen.. */
		return;

	/* Set the constraints according to the already configured stream */
	snd_pcm_hw_constraint_minmax(slv_substream->runtime,
				SNDRV_PCM_HW_PARAM_RATE,
				twl4030->rate,
				twl4030->rate);

	snd_pcm_hw_constraint_minmax(slv_substream->runtime,
				SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
				twl4030->sample_bits,
				twl4030->sample_bits);

	snd_pcm_hw_constraint_minmax(slv_substream->runtime,
				SNDRV_PCM_HW_PARAM_CHANNELS,
				twl4030->channels,
				twl4030->channels);
}

/* In case of 4 channel mode, the RX1 L/R for playback and the TX2 L/R for
 * capture has to be enabled/disabled. */
static void twl4030_tdm_enable(struct snd_soc_codec *codec, int direction,
				int enable)
{
	u8 reg, mask;

	reg = twl4030_read_reg_cache(codec, TWL4030_REG_OPTION);

	if (direction == SNDRV_PCM_STREAM_PLAYBACK)
		mask = TWL4030_ARXL1_VRX_EN | TWL4030_ARXR1_EN;
	else
		mask = TWL4030_ATXL2_VTXL_EN | TWL4030_ATXR2_VTXR_EN;

	if (enable)
		reg |= mask;
	else
		reg &= ~mask;

	twl4030_write(codec, TWL4030_REG_OPTION, reg);
}

static int twl4030_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	if (twl4030->master_substream) {
		twl4030->slave_substream = substream;
		/* The DAI has one configuration for playback and capture, so
		 * if the DAI has been already configured then constrain this
		 * substream to match it. */
		if (twl4030->configured)
			twl4030_constraints(twl4030, twl4030->master_substream);
	} else {
		if (!(twl4030_read_reg_cache(codec, TWL4030_REG_CODEC_MODE) &
			TWL4030_OPTION_1)) {
			/* In option2 4 channel is not supported, set the
			 * constraint for the first stream for channels, the
			 * second stream will 'inherit' this cosntraint */
			snd_pcm_hw_constraint_minmax(substream->runtime,
						SNDRV_PCM_HW_PARAM_CHANNELS,
						2, 2);
		}
		twl4030->master_substream = substream;
	}

	return 0;
}

static void twl4030_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	if (twl4030->master_substream == substream)
		twl4030->master_substream = twl4030->slave_substream;

	twl4030->slave_substream = NULL;

	/* If all streams are closed, or the remaining stream has not yet
	 * been configured than set the DAI as not configured. */
	if (!twl4030->master_substream)
		twl4030->configured = 0;
	 else if (!twl4030->master_substream->runtime->channels)
		twl4030->configured = 0;

	 /* If the closing substream had 4 channel, do the necessary cleanup */
	if (substream->runtime->channels == 4)
		twl4030_tdm_enable(codec, substream->stream, 0);
}

static int twl4030_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 mode, old_mode, format, old_format;

	 /* If the substream has 4 channel, do the necessary setup */
	if (params_channels(params) == 4) {
		format = twl4030_read_reg_cache(codec, TWL4030_REG_AUDIO_IF);
		mode = twl4030_read_reg_cache(codec, TWL4030_REG_CODEC_MODE);

		/* Safety check: are we in the correct operating mode and
		 * the interface is in TDM mode? */
		if ((mode & TWL4030_OPTION_1) &&
		    ((format & TWL4030_AIF_FORMAT) == TWL4030_AIF_FORMAT_TDM))
			twl4030_tdm_enable(codec, substream->stream, 1);
		else
			return -EINVAL;
	}

	if (twl4030->configured)
		/* Ignoring hw_params for already configured DAI */
		return 0;

	/* bit rate */
	old_mode = twl4030_read_reg_cache(codec,
			TWL4030_REG_CODEC_MODE) & ~TWL4030_CODECPDZ;
	mode = old_mode & ~TWL4030_APLL_RATE;

	switch (params_rate(params)) {
	case 8000:
		mode |= TWL4030_APLL_RATE_8000;
		break;
	case 11025:
		mode |= TWL4030_APLL_RATE_11025;
		break;
	case 12000:
		mode |= TWL4030_APLL_RATE_12000;
		break;
	case 16000:
		mode |= TWL4030_APLL_RATE_16000;
		break;
	case 22050:
		mode |= TWL4030_APLL_RATE_22050;
		break;
	case 24000:
		mode |= TWL4030_APLL_RATE_24000;
		break;
	case 32000:
		mode |= TWL4030_APLL_RATE_32000;
		break;
	case 44100:
		mode |= TWL4030_APLL_RATE_44100;
		break;
	case 48000:
		mode |= TWL4030_APLL_RATE_48000;
		break;
	case 96000:
		mode |= TWL4030_APLL_RATE_96000;
		break;
	default:
		printk(KERN_ERR "TWL4030 hw params: unknown rate %d\n",
			params_rate(params));
		return -EINVAL;
	}

	/* sample size */
	old_format = twl4030_read_reg_cache(codec, TWL4030_REG_AUDIO_IF);
	format = old_format;
	format &= ~TWL4030_DATA_WIDTH;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		format |= TWL4030_DATA_WIDTH_16S_16W;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		format |= TWL4030_DATA_WIDTH_32S_24W;
		break;
	default:
		printk(KERN_ERR "TWL4030 hw params: unknown format %d\n",
			params_format(params));
		return -EINVAL;
	}

	if (format != old_format || mode != old_mode) {
		if (twl4030->codec_powered) {
			/*
			 * If the codec is powered, than we need to toggle the
			 * codec power.
			 */
			twl4030_codec_enable(codec, 0);
			twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
			twl4030_write(codec, TWL4030_REG_AUDIO_IF, format);
			twl4030_codec_enable(codec, 1);
		} else {
			twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
			twl4030_write(codec, TWL4030_REG_AUDIO_IF, format);
		}
	}

	/* Store the important parameters for the DAI configuration and set
	 * the DAI as configured */
	twl4030->configured = 1;
	twl4030->rate = params_rate(params);
	twl4030->sample_bits = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_SAMPLE_BITS)->min;
	twl4030->channels = params_channels(params);

	/* If both playback and capture streams are open, and one of them
	 * is setting the hw parameters right now (since we are here), set
	 * constraints to the other stream to match the current one. */
	if (twl4030->slave_substream)
		twl4030_constraints(twl4030, substream);

	return 0;
}

static int twl4030_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	switch (freq) {
	case 19200000:
	case 26000000:
	case 38400000:
		break;
	default:
		dev_err(codec->dev, "Unsupported APLL mclk: %u\n", freq);
		return -EINVAL;
	}

	if ((freq / 1000) != twl4030->sysclk) {
		dev_err(codec->dev,
			"Mismatch in APLL mclk: %u (configured: %u)\n",
			freq, twl4030->sysclk * 1000);
		return -EINVAL;
	}

	return 0;
}

static int twl4030_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 old_format, format;

	/* get format */
	old_format = twl4030_read_reg_cache(codec, TWL4030_REG_AUDIO_IF);
	format = old_format;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		format &= ~(TWL4030_AIF_SLAVE_EN);
		format &= ~(TWL4030_CLK256FS_EN);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		format |= TWL4030_AIF_SLAVE_EN;
		format |= TWL4030_CLK256FS_EN;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	format &= ~TWL4030_AIF_FORMAT;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format |= TWL4030_AIF_FORMAT_CODEC;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		format |= TWL4030_AIF_FORMAT_TDM;
		break;
	default:
		return -EINVAL;
	}

	if (format != old_format) {
		if (twl4030->codec_powered) {
			/*
			 * If the codec is powered, than we need to toggle the
			 * codec power.
			 */
			twl4030_codec_enable(codec, 0);
			twl4030_write(codec, TWL4030_REG_AUDIO_IF, format);
			twl4030_codec_enable(codec, 1);
		} else {
			twl4030_write(codec, TWL4030_REG_AUDIO_IF, format);
		}
	}

	return 0;
}

static int twl4030_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg = twl4030_read_reg_cache(codec, TWL4030_REG_AUDIO_IF);

	if (tristate)
		reg |= TWL4030_AIF_TRI_EN;
	else
		reg &= ~TWL4030_AIF_TRI_EN;

	return twl4030_write(codec, TWL4030_REG_AUDIO_IF, reg);
}

static int twl4030_set_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	// Output
	if(twl4030->ext_speaker_gpio != -1)
		gpio_direction_output(twl4030->ext_speaker_gpio, (!mute) && (twl4030->ext_spkr_enabled));
	if(twl4030->int_speaker_gpio != -1)
		gpio_direction_output(twl4030->int_speaker_gpio, (!mute) && (!twl4030->ext_spkr_enabled));

	return 0;
}

/* In case of voice mode, the RX1 L(VRX) for downlink and the TX2 L/R
 * (VTXL, VTXR) for uplink has to be enabled/disabled. */
static void twl4030_voice_enable(struct snd_soc_codec *codec, int direction, int enable)
{
	u8 reg, mask;

	reg = twl4030_read_reg_cache(codec, TWL4030_REG_OPTION);

	if (direction == SNDRV_PCM_STREAM_PLAYBACK)
		mask = TWL4030_ARXL1_VRX_EN;
	else
		mask = TWL4030_ATXL2_VTXL_EN | TWL4030_ATXR2_VTXR_EN;

	if (enable)
		reg |= mask;
	else
		reg &= ~mask;

	twl4030_write(codec, TWL4030_REG_OPTION, reg);
}

static int twl4030_voice_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 mode;

	/* If the system master clock is not 26MHz, the voice PCM interface is
	 * not avilable.
	 */
	if (twl4030->sysclk != 26000) {
		dev_err(codec->dev, "The board is configured for %u Hz, while"
			"the Voice interface needs 26MHz APLL mclk\n",
			twl4030->sysclk * 1000);
		return -EINVAL;
	}

	/* If the codec mode is not option2, the voice PCM interface is not
	 * avilable.
	 */
	mode = twl4030_read_reg_cache(codec, TWL4030_REG_CODEC_MODE)
		& TWL4030_OPT_MODE;

	if (mode != TWL4030_OPTION_2) {
		printk(KERN_ERR "TWL4030 voice startup: "
			"the codec mode is not option2\n");
		return -EINVAL;
	}

	return 0;
}

static void twl4030_voice_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	/* Enable voice digital filters */
	twl4030_voice_enable(codec, substream->stream, 0);
}

static int twl4030_voice_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 old_mode, mode;

	/* Enable voice digital filters */
	twl4030_voice_enable(codec, substream->stream, 1);

	/* bit rate */
	old_mode = twl4030_read_reg_cache(codec, TWL4030_REG_CODEC_MODE)
		& ~(TWL4030_CODECPDZ);
	mode = old_mode;

	switch (params_rate(params)) {
	case 8000:
		mode &= ~(TWL4030_SEL_16K);
		break;
	case 16000:
		mode |= TWL4030_SEL_16K;
		break;
	default:
		printk(KERN_ERR "TWL4030 voice hw params: unknown rate %d\n",
			params_rate(params));
		return -EINVAL;
	}

	if (mode != old_mode) {
		if (twl4030->codec_powered) {
			/*
			 * If the codec is powered, than we need to toggle the
			 * codec power.
			 */
			twl4030_codec_enable(codec, 0);
			twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
			twl4030_codec_enable(codec, 1);
		} else {
			twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
		}
	}

	return 0;
}

static int twl4030_voice_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	if (freq != 26000000) {
		dev_err(codec->dev, "Unsupported APLL mclk: %u, the Voice"
			"interface needs 26MHz APLL mclk\n", freq);
		return -EINVAL;
	}
	if ((freq / 1000) != twl4030->sysclk) {
		dev_err(codec->dev,
			"Mismatch in APLL mclk: %u (configured: %u)\n",
			freq, twl4030->sysclk * 1000);
		return -EINVAL;
	}
	return 0;
}

static int twl4030_voice_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 old_format, format;

	/* get format */
	old_format = twl4030_read_reg_cache(codec, TWL4030_REG_VOICE_IF);
	format = old_format;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		format &= ~(TWL4030_VIF_SLAVE_EN);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		format |= TWL4030_VIF_SLAVE_EN;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_NF:
		format &= ~(TWL4030_VIF_FORMAT);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		format |= TWL4030_VIF_FORMAT;
		break;
	default:
		return -EINVAL;
	}

	if (format != old_format) {
		if (twl4030->codec_powered) {
			/*
			 * If the codec is powered, than we need to toggle the
			 * codec power.
			 */
			twl4030_codec_enable(codec, 0);
			twl4030_write(codec, TWL4030_REG_VOICE_IF, format);
			twl4030_codec_enable(codec, 1);
		} else {
			twl4030_write(codec, TWL4030_REG_VOICE_IF, format);
		}
	}

	return 0;
}

static int twl4030_voice_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg = twl4030_read_reg_cache(codec, TWL4030_REG_VOICE_IF);

	if (tristate)
		reg |= TWL4030_VIF_TRI_EN;
	else
		reg &= ~TWL4030_VIF_TRI_EN;

	return twl4030_write(codec, TWL4030_REG_VOICE_IF, reg);
}

#define TWL4030_RATES	 (SNDRV_PCM_RATE_8000_48000)
#define TWL4030_FORMATS	 (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FORMAT_S24_LE)

static struct snd_soc_dai_ops twl4030_dai_hifi_ops = {
	.startup		= twl4030_startup,
	.shutdown		= twl4030_shutdown,
	.hw_params		= twl4030_hw_params,
	.set_sysclk		= twl4030_set_dai_sysclk,
	.set_fmt		= twl4030_set_dai_fmt,
	.set_tristate	= twl4030_set_tristate,
	.digital_mute 	= twl4030_set_digital_mute,
};

static struct snd_soc_dai_ops twl4030_dai_voice_ops = {
	.startup		= twl4030_voice_startup,
	.shutdown		= twl4030_voice_shutdown,
	.hw_params		= twl4030_voice_hw_params,
	.set_sysclk		= twl4030_voice_set_dai_sysclk,
	.set_fmt		= twl4030_voice_set_dai_fmt,
	.set_tristate	= twl4030_voice_set_tristate,
};

static struct snd_soc_dai_driver twl4030_dai[] = {
{
	.name = "twl4030-hifi",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 2,
		.channels_max = 4,
		.rates = TWL4030_RATES | SNDRV_PCM_RATE_96000,
		.formats = TWL4030_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 4,
		.rates = TWL4030_RATES,
		.formats = TWL4030_FORMATS,},
	.ops = &twl4030_dai_hifi_ops,
},
{
	.name = "twl4030-voice",
	.playback = {
		.stream_name = "Voice Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &twl4030_dai_voice_ops,
},
};

static int twl4030_soc_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	twl4030_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int twl4030_soc_resume(struct snd_soc_codec *codec)
{
	twl4030_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static int twl4030_soc_probe(struct snd_soc_codec *codec)
{
	struct twl4030_priv 			*twl4030;
	struct twl4030_codec_audio_data *pdata = dev_get_platdata(codec->dev);

	printk("%s: \n", __func__);

	twl4030 = kzalloc(sizeof(struct twl4030_priv), GFP_KERNEL);
	if(!twl4030){
		printk("Can not allocate memroy\n");
		return -ENOMEM;
	}

	printk("%s: platform specific\n", __func__);
	twl4030->digimic_delay 		= pdata->digimic_delay;
	twl4030->ext_speaker_gpio 	= pdata->ext_speaker_gpio; 	// external speaker amplifier
	twl4030->int_speaker_gpio 	= pdata->int_speaker_gpio; 	// internal speaker amplifier
	twl4030->ext_mic_gpio 		= pdata->ext_mic_gpio;		// external microphone
	twl4030->cradle_aud_gpio 	= pdata->cradle_aud_gpio;   // M307i cradle audio switch

	snd_soc_codec_set_drvdata(codec, twl4030);
	/* Set the defaults, and power up the codec */
	twl4030->sysclk = twl4030_codec_get_mclk() / 1000;
	codec->idle_bias_off = 1;

	twl4030_init_chip(codec);

	twl4030->ext_spkr_enabled 	= 0;
	twl4030->ext_mic_enabled 	= 0;

	printk("twl4030_soc_probe: add sound controls\n");
	snd_soc_add_controls(codec, twl4030_snd_controls, ARRAY_SIZE(twl4030_snd_controls));
	twl4030_add_widgets(codec);

	return 0;
}

static int twl4030_soc_remove(struct snd_soc_codec *codec)
{
	/* Reset registers to their chip default before leaving */
	printk("%s: \n", __func__);
	twl4030_reset_registers(codec);
	twl4030_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_twl4030 = {
	.probe 			= twl4030_soc_probe,
	.remove 		= twl4030_soc_remove,
	.suspend 		= twl4030_soc_suspend,
	.resume 		= twl4030_soc_resume,
	.read 			= twl4030_read_reg_cache,
	.write 			= twl4030_write,
	.set_bias_level = twl4030_set_bias_level,
	.reg_cache_size = sizeof(twl4030_reg),
	.reg_word_size 	= sizeof(u8),
	.reg_cache_default = twl4030_reg,
};

static int __devinit twl4030_codec_probe(struct platform_device *pdev)
{
	struct twl4030_codec_audio_data *pdata = pdev->dev.platform_data;

	if(!pdata){
		dev_err(&pdev->dev, "platform_data is missing\n");
		return -EINVAL;
	}

	printk("%s: \n", __func__);

	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_twl4030, twl4030_dai, ARRAY_SIZE(twl4030_dai));
}

static int __devexit
twl4030_codec_remove(struct platform_device *pdev)
{
	struct twl4030_priv *twl4030 = dev_get_drvdata(&pdev->dev);

	printk("%s: \n", __func__);
	snd_soc_unregister_codec(&pdev->dev);
	kfree(twl4030);
	return 0;
}

MODULE_ALIAS("platform:twl4030-codec");

static struct platform_driver twl4030_codec_driver = {
	.probe		= twl4030_codec_probe,
	.remove		= __devexit_p(twl4030_codec_remove),
	.driver		= {
		.name	= "twl4030-codec",
		.owner	= THIS_MODULE,
	},
};

static int __init
twl4030_modinit(void)
{
	return platform_driver_register(&twl4030_codec_driver);
}
module_init(twl4030_modinit);

static void __exit
twl4030_exit(void)
{
	platform_driver_unregister(&twl4030_codec_driver);
}
module_exit(twl4030_exit);

MODULE_DESCRIPTION("ASoC TPS65950 for A31x boards codec driver");
MODULE_AUTHOR("Vladimir Zatulovsky");
MODULE_LICENSE("GPL");

