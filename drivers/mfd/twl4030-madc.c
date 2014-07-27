/*
 *
 * TWL4030 MADC module driver-This driver monitors the real time
 * conversion of analog signals like battery temperature,
 * battery type, battery level etc.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * J Keerthy <j-keerthy@ti.com>
 *
 * Based on twl4030-madc.c
 * Copyright (C) 2008 Nokia Corporation
 * Mikko Ylinen <mikko.k.ylinen@nokia.com>
 *
 * Amit Kucheria <amit.kucheria@canonical.com>
 *
 * Copyright (C) 2013 micronet ltd. - http://www.micronet.co.il/
 * Vladimir Zatulovsky <vladimirz@micronet.co.il>
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

#include <generated/autoconf.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/mutex.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/types.h>
#include <linux/gfp.h>
#include <linux/err.h>

/*
 * struct twl4030_madc_data - a container for madc info
 * @dev - pointer to device structure for madc
 * @lock - mutex protecting this data structure
 * @requests - Array of request struct corresponding to SW1, SW2 and RT
 * @imr - Interrupt mask register of MADC
 * @isr - Interrupt status register of MADC
 */
struct twl4030_madc_data {
	struct device *dev;
	struct mutex lock;	/* mutex protecting this data structure */
	struct mutex in_lock;	/* mutex protecting this data structure */
	struct twl4030_madc_request requests[TWL4030_MADC_NUM_METHODS];
	int imr;
	int isr;
};

static struct twl4030_madc_data *twl4030_madc;

struct twl4030_prescale_divider_ratios {
	s16 numerator;
	s16 denominator;
};

static const struct twl4030_prescale_divider_ratios
twl4030_divider_ratios[16] = {
	{1, 1},		/* CHANNEL 0 No Prescaler */
	{1, 1},		/* CHANNEL 1 No Prescaler */
	{6, 10},	/* CHANNEL 2 Prescaler is 1.5V/2.5V == .6*/
	{6, 10},	/* CHANNEL 3 */
	{6, 10},	/* CHANNEL 4 */
	{6, 10},	/* CHANNEL 5 */
	{6, 10},	/* CHANNEL 6 */
	{6, 10},	/* CHANNEL 7 */
	{3, 14},	/* CHANNEL 8 */
	{1, 3},		/* CHANNEL 9 */
	{1, 1},		/* CHANNEL 10 No Prescaler */
	{15, 100},	/* CHANNEL 11 */
	{1, 4},		/* CHANNEL 12 */
	{1, 1},		/* CHANNEL 13 Reserved channels */
	{1, 1},		/* CHANNEL 14 Reseved channels */
	{5, 11},	/* CHANNEL 15 */
};

#if !defined (TPS65930)
#if !defined (CONFIG_MACH_A317)
/*
 * Conversion table from -3 to 55 degree Celcius
 */
static int therm_tbl[] =
{
	30800,	29500,	28300,	27100,
	26000,	24900,	23900,	22900,	22000,	21100,	20300,	19400,	18700,	17900,
	17200,	16500,	15900,	15300,	14700,	14100,	13600,	13100,	12600,	12100,
	11600,	11200,	10800,	10400,	10000,	9630,	9280,	8950,	8620,	8310,
	8020,	7730,	7460,	7200,	6950,	6710,	6470,	6250,	6040,	5830,
	5640,	5450,	5260,	5090,	4920,	4760,	4600,	4450,	4310,	4170,
	4040,	3910,	3790,	3670,	3550
};
#endif
#endif
/*
 * Structure containing the registers
 * of different conversion methods supported by MADC.
 * Hardware or RT real time conversion request initiated by external host
 * processor for RT Signal conversions.
 * External host processors can also request for non RT conversions
 * SW1 and SW2 software conversions also called asynchronous or GPC request.
 */
static const struct twl4030_madc_conversion_method twl4030_conversion_methods[] = {
	[TWL4030_MADC_RT] = {
			     .sel = TWL4030_MADC_RTSELECT_LSB,
			     .avg = TWL4030_MADC_RTAVERAGE_LSB,
			     .rbase = TWL4030_MADC_RTCH0_LSB,
			     },
	[TWL4030_MADC_SW1] = {
			      .sel = TWL4030_MADC_SW1SELECT_LSB,
			      .avg = TWL4030_MADC_SW1AVERAGE_LSB,
			      .rbase = TWL4030_MADC_GPCH0_LSB,
			      .ctrl = TWL4030_MADC_CTRL_SW1,
			      },
	[TWL4030_MADC_SW2] = {
			      .sel = TWL4030_MADC_SW2SELECT_LSB,
			      .avg = TWL4030_MADC_SW2AVERAGE_LSB,
			      .rbase = TWL4030_MADC_GPCH0_LSB,
			      .ctrl = TWL4030_MADC_CTRL_SW2,
			      },
};


#if !defined (TPS65930)
/*
 * Function to enable or disable bias current for
 * main battery type reading or temperature sensing
 * @madc - pointer to twl4030_madc_data struct
 * @chan - can be one of the two values
 * TWL4030_BCI_ITHEN - Enables bias current for main battery type reading
 * TWL4030_BCI_TYPEN - Enables bias current for main battery temperature
 * sensing
 * @on - enable or disable chan.
 */
static int twl4030_madc_set_current_generator(struct twl4030_madc_data *madc, int chan, int on)
{
	int ret;
	u8 regval;

	ret = twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &regval, TWL4030_BCI_BCICTL1);
	if(ret){
		dev_err(madc->dev, "unable to read BCICTL1 reg 0x%X", TWL4030_BCI_BCICTL1);
		return ret;
	}
	if(on)
		regval |= (chan ? TWL4030_BCI_ITHEN : TWL4030_BCI_TYPEN) | TWL4030_BCI_MESBAT;
	else
		regval &= (chan ? ~TWL4030_BCI_ITHEN : ~TWL4030_BCI_TYPEN) & ~TWL4030_BCI_MESBAT;

	ret = twl_i2c_write_u8(TWL_MODULE_MAIN_CHARGE, regval, TWL4030_BCI_BCICTL1);
	if (ret) {
		dev_err(madc->dev, "unable to write BCICTL1 reg 0x%X\n", TWL4030_BCI_BCICTL1);
		return ret;
	}

	return 0;
}
#endif

/*
 * Function that sets MADC software power on bit to enable MADC
 * @madc - pointer to twl4030_madc_data struct
 * @on - Enable or disable MADC software powen on bit.
 * returns error if i2c read/write fails else 0
 */
static int twl4030_madc_set_power(int on)
{
	u8 regval;
	int ret, chan;

#if defined (CONFIG_MACH_A317)
	chan = 1;;
#else
	chan = 1;;
#endif

	if(on)
		regval = TWL4030_MADC_MADCON;
	else
		regval = 0;

	ret = twl_i2c_write_u8(TWL4030_MODULE_MADC, regval, TWL4030_MADC_CTRL1);
	mdelay(10);
	if (ret) {
		printk("%s: unable to write MADC ctrl1 register 0x%X\n", __func__, TWL4030_MADC_CTRL1);
		return ret;
	}

#if !defined (TPS65930)
	ret = twl4030_madc_set_current_generator(twl4030_madc, chan, on);
	if (ret < 0)
	{
		printk("%s: unable to set current generation\n", __func__);
		return ret;
	}
#endif

	return 0;
}

/*
 * Function to read a particular channel value.
 * @madc - pointer to struct twl4030_madc_data
 * @reg - lsb of ADC Channel
 * If the i2c read fails it returns an error else returns 0.
 */
static int twl4030_madc_channel_raw_read(struct twl4030_madc_data *madc, u8 reg)
{
	u8 msb, lsb;
	int ret;
	/*
	 * For each ADC channel, we have MSB and LSB register pair. MSB address
	 * is always LSB address+1. reg parameter is the address of LSB register
	 */
	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, &msb, reg + 1);
	if(ret)
	{
		dev_err(madc->dev, "unable to read MSB register 0x%X\n", reg + 1);
		return ret;
	}
	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, &lsb, reg);
	if(ret)
	{
		dev_err(madc->dev, "unable to read LSB register 0x%X\n", reg);
		return ret;
	}

	return (int)(((msb << 8) | lsb) >> 6);
}

#if !defined (TPS65930)

#if defined (CONFIG_MACH_A317)
/*
 * Return battery temperature
 * Or < 0 on failure.
 */
static int twl4030input_impedance(int volts)
{
	u8 val;
	int curr, res;

	u8 msb, lsb;

	twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &lsb, 6);
	twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &msb, 7);

	val = ((msb & 3) << 8) | lsb;
	printk("%s: TBAT %d %d or %d\n", __func__, val, msb, lsb);

	twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &lsb, 0x0a);
	twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &msb, 0x0b);

	val = ((msb & 3) << 8) | lsb;
	printk("%s: BCIAVC %d %d or %d\n", __func__, val, msb, lsb);

	twl_i2c_read_u8(TWL4030_MODULE_MADC, &lsb, 0x57);
	twl_i2c_read_u8(TWL4030_MODULE_MADC, &msb, 0x58);

	val = (lsb >> 6) | (msb << 2);
	printk("%s: BCICH0 %d %d or %d\n", __func__, val, msb, lsb);

	/* Getting and calculating the supply current in u-amperes */
	if(twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &val, TWL4030_BCI_BCICTL2) < 0)
		return -1;

	curr = ((val & TWL4030_BCI_ITHSENS) + 1) * 10;

	/* Getting and calculating the thermistor resistance in OHMs */
	res = 1000 * volts / curr;
	printk("%s: %d mV %d mA %d ohm\n", __func__, volts, curr, res);

	return res;
}
#else
/*
 * Return battery temperature
 * Or < 0 on failure.
 */
static int twl4030battery_temperature(int raw_volt)
{
	u8 val;
	int temp, curr, volt, res, ret;

	volt = (raw_volt * TEMP_STEP_SIZE) / TEMP_PSR_R;
	printk("%s: Volts %d(%d)\n", __func__, raw_volt, volt);
	/* Getting and calculating the supply current in micro ampers */
	ret = twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &val, TWL4030_BCI_BCICTL2);
	if (ret < 0)
		return ret;
	curr = ((val & TWL4030_BCI_ITHSENS) + 1) * 10;
	printk("%s: Current %d\n", __func__, curr);
	/* Getting and calculating the thermistor resistance in ohms */
	res = volt * 1000 / curr;
	printk("%s: Resistance %d\n", __func__, res);
	/* calculating temperature */
	for (temp = 58; temp >= 0; temp--) {
		int actual = therm_tbl[temp];

		if ((actual - res) >= 0)
			break;
	}

	return temp + 1;
}
#endif

static int twl4030battery_current(int raw_volt)
{
	int ret;
	u8 val;

	ret = twl_i2c_read_u8(TWL_MODULE_MAIN_CHARGE, &val, TWL4030_BCI_BCICTL1);
	if (ret)
		return ret;
	if (val & TWL4030_BCI_CGAIN) /* slope of 0.44 mV/mA */
		return (raw_volt * CURR_STEP_SIZE) / CURR_PSR_R1;
	else /* slope of 0.88 mV/mA */
		return (raw_volt * CURR_STEP_SIZE) / CURR_PSR_R2;
}
#endif

/*
 * Function to read channel values
 * @madc - pointer to twl4030_madc_data struct
 * @reg_base - Base address of the first channel
 * @Channels - 16 bit bitmap. If the bit is set, channel value is read
 * @buf - The channel values are stored here. if read fails error
 * value is stored
 * Returns the number of successfully read channels.
 */
static int twl4030_madc_read_channels(struct twl4030_madc_data *madc, unsigned char reg_base, unsigned long channels, int *buf)
{
	int count = 0, count_req = 0, i;
	unsigned char reg;

	for_each_set_bit(i, &channels, TWL4030_MADC_MAX_CHANNELS)
	{
		reg = reg_base + 2 * i;
		buf[i] = twl4030_madc_channel_raw_read(madc, reg);
		if(buf[i] < 0)
		{
			dev_err(madc->dev, "Unable to read register 0x%X\n", reg);
			count_req++;
			continue;
		}
		switch(i)
		{
#if !defined (TPS65930)
			case 10:
			{
				buf[i] = twl4030battery_current(buf[i]);
				if (buf[i] < 0)
				{
					dev_err(madc->dev, "err reading current\n");
					count_req++;
				}
				else
				{
					count++;
					buf[i] = buf[i] - 750;
				}
				break;
			}
			case 1:
			{
#if defined (CONFIG_MACH_A317)
				buf[TWL4030_MADC_MAX_CHANNELS + i] = twl4030input_impedance((3*1000*buf[i]*twl4030_divider_ratios[i].denominator)/
						(2 * 1023 * twl4030_divider_ratios[i].numerator));
#else
				buf[i] = twl4030battery_temperature(buf[i]);
#endif
				if(buf[i] < 0)
				{
					dev_err(madc->dev, "err reading temperature\n");
					count_req++;
				}
				else
				{
#if !defined (CONFIG_MACH_A317)
					buf[i] -= 3;
#endif
					count++;
				}
				break;
			}
#endif
			default:
			{
				count++;
				/* Analog Input (V) = conv_result * step_size / R
				 * conv_result = decimal value of 10-bit conversion result
				 * step size = 1.5 / (2 ^ 10 -1)
				 * R = Prescaler ratio for input channels.
				 * Result given in mV hence multiplied by 1000.
				 */
				buf[TWL4030_MADC_MAX_CHANNELS + i] = (buf[i] * 3 * 1000 * twl4030_divider_ratios[i].denominator) /
						 	 	 	 	 	 	 	 (2 * 1023 * twl4030_divider_ratios[i].numerator);
				break;
			}
		}
	}
	if(count_req)
		dev_err(madc->dev, "%d channel conversion failed\n", count_req);

	return count;
}

/*
 * Enables irq.
 * @madc - pointer to twl4030_madc_data struct
 * @id - irq number to be enabled
 * can take one of TWL4030_MADC_RT, TWL4030_MADC_SW1, TWL4030_MADC_SW2
 * corresponding to RT, SW1, SW2 conversion requests.
 * If the i2c read fails it returns an error else returns 0.
 */
static int twl4030_madc_enable_irq(struct twl4030_madc_data *madc, u8 id)
{
	u8 val;
	int ret;

	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, &val, madc->imr);
	if(ret)
	{
		dev_err(madc->dev, "unable to read imr register 0x%X\n", madc->imr);
		return ret;
	}
	val &= ~(1 << id);
	ret = twl_i2c_write_u8(TWL4030_MODULE_MADC, val, madc->imr);
	if(ret)
	{
		dev_err(madc->dev, "unable to write imr register 0x%X\n", madc->imr);
		return ret;

	}

	return 0;
}

/*
 * Disables irq.
 * @madc - pointer to twl4030_madc_data struct
 * @id - irq number to be disabled
 * can take one of TWL4030_MADC_RT, TWL4030_MADC_SW1, TWL4030_MADC_SW2
 * corresponding to RT, SW1, SW2 conversion requests.
 * Returns error if i2c read/write fails.
 */
static int twl4030_madc_disable_irq(struct twl4030_madc_data *madc, u8 id)
{
	u8 val;
	int ret;

	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, &val, madc->imr);
	if(ret)
	{
		dev_err(madc->dev, "unable to read imr register 0x%X\n", madc->imr);
		return ret;
	}
	val |= (1 << id);
	ret = twl_i2c_write_u8(TWL4030_MODULE_MADC, val, madc->imr);
	if(ret)
	{
		dev_err(madc->dev, "unable to write imr register 0x%X\n", madc->imr);
		return ret;
	}

	return 0;
}

static irqreturn_t twl4030_madc_threaded_irq_handler(int irq, void *_madc)
{
	struct twl4030_madc_data *madc = _madc;
	const struct twl4030_madc_conversion_method *method;
	u8 isr_val, imr_val;
	int i, len, ret;
	struct twl4030_madc_request *r;

	mutex_lock(&madc->lock);
	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, &isr_val, madc->isr);
	if (ret) {
		dev_err(madc->dev, "unable to read isr register 0x%X\n", madc->isr);
		goto err_i2c;
	}
	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, &imr_val, madc->imr);
	if (ret) {
		dev_err(madc->dev, "unable to read imr register 0x%X\n", madc->imr);
		goto err_i2c;
	}
	isr_val &= ~imr_val;
	for (i = 0; i < TWL4030_MADC_NUM_METHODS; i++) {
		if (!(isr_val & (1 << i)))
			continue;
		ret = twl4030_madc_disable_irq(madc, i);
		if (ret < 0)
			dev_dbg(madc->dev, "Disable interrupt failed%d\n", i);
		madc->requests[i].result_pending = 1;
	}
	for (i = 0; i < TWL4030_MADC_NUM_METHODS; i++) {
		r = &madc->requests[i];
		/* No pending results for this method, move to next one */
		if (!r->result_pending)
			continue;
		method = &twl4030_conversion_methods[r->method];
		/* Read results */
		len = twl4030_madc_read_channels(madc, method->rbase, r->channels, r->rbuf);
		/* Return results to caller */
		if (r->func_cb != NULL) {
			r->func_cb(len, r->channels, r->rbuf);
			r->func_cb = NULL;
		}
		/* Free request */
		r->result_pending = 0;
		r->active = 0;
	}
	mutex_unlock(&madc->lock);

	return IRQ_HANDLED;

err_i2c:
	/*
	 * In case of error check whichever request is active
	 * and service the same.
	 */
	for (i = 0; i < TWL4030_MADC_NUM_METHODS; i++) {
		r = &madc->requests[i];
		if (r->active == 0)
			continue;
		method = &twl4030_conversion_methods[r->method];
		/* Read results */
		len = twl4030_madc_read_channels(madc, method->rbase, r->channels, r->rbuf);
		/* Return results to caller */
		if (r->func_cb != NULL) {
			r->func_cb(len, r->channels, r->rbuf);
			r->func_cb = NULL;
		}
		/* Free request */
		r->result_pending = 0;
		r->active = 0;
	}
	mutex_unlock(&madc->lock);

	return IRQ_HANDLED;
}

static int twl4030_madc_set_irq(struct twl4030_madc_data *madc, struct twl4030_madc_request *req)
{
	struct twl4030_madc_request *p;
	int ret;

	p = &madc->requests[req->method];
	memcpy(p, req, sizeof(*req));
	ret = twl4030_madc_enable_irq(madc, req->method);
	if (ret < 0) {
		dev_err(madc->dev, "enable irq failed!!\n");
		return ret;
	}

	return 0;
}

/*
 * Function which enables the madc conversion
 * by writing to the control register.
 * @madc - pointer to twl4030_madc_data struct
 * @conv_method - can be TWL4030_MADC_RT, TWL4030_MADC_SW2, TWL4030_MADC_SW1
 * corresponding to RT SW1 or SW2 conversion methods.
 * Returns 0 if succeeds else a negative error value
 */
static int twl4030_madc_start_conversion(struct twl4030_madc_data *madc, int conv_method)
{
	const struct twl4030_madc_conversion_method *method;
	int ret = 0;
	method = &twl4030_conversion_methods[conv_method];
	switch (conv_method) {
	case TWL4030_MADC_SW1:
	case TWL4030_MADC_SW2:
		ret = twl_i2c_write_u8(TWL4030_MODULE_MADC, TWL4030_MADC_SW_START, method->ctrl);
		udelay(50);
		if (ret) {
			dev_err(madc->dev, "unable to write ctrl register 0x%X\n", method->ctrl);
			return ret;
		}
		break;
	default:
		break;
	}

	return 0;
}

/*
 * Function that waits for conversion to be ready
 * @madc - pointer to twl4030_madc_data struct
 * @timeout_ms - timeout value in milliseconds
 * @status_reg - ctrl register
 * returns 0 if succeeds else a negative error value
 */
static int twl4030_madc_wait_conversion_ready(struct twl4030_madc_data *madc, int m, unsigned int timeout_ms, u8 status_reg)
{
	unsigned long timeout;
	const struct twl4030_madc_conversion_method *method;
	int ret;
	unsigned ch_msb, ch_lsb;
	u8 reg;

	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, &reg, status_reg);
		if (ret) {
			dev_err(madc->dev, "unable to read status register 0x%X\n", status_reg);
			return ret;
		}
		if(!(reg & TWL4030_MADC_BUSY) && (reg & TWL4030_MADC_EOC_SW))
			return 0;
		usleep_range(500, 2000);
	} while (!time_after(jiffies, timeout));


	method = &twl4030_conversion_methods[m];
	/* Select channels to be converted */
	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, (unsigned char *)&ch_msb, method->sel + 1);
	ret = twl_i2c_read_u8(TWL4030_MODULE_MADC, (unsigned char *)&ch_lsb, method->sel);

	dev_err(madc->dev, "conversion timeout (%X %2X%2X)!\n", reg, ch_msb, ch_lsb);

	return -EAGAIN;
}

/*
 * An exported function which can be called from other kernel drivers.
 * @req twl4030_madc_request structure
 * req->rbuf will be filled with read values of channels based on the
 * channel index. If a particular channel reading fails there will
 * be a negative error value in the corresponding array element.
 * returns 0 if succeeds else error value
 */
int twl4030_madc_conversion(struct twl4030_madc_request *req)
{
	const struct twl4030_madc_conversion_method *method;
	u8 ch_msb, ch_lsb;
	int ret;

	if (!req || !twl4030_madc)
		return -EINVAL;

	mutex_lock(&twl4030_madc->lock);
	twl4030_madc_set_power(1);
	if(req->method < TWL4030_MADC_RT || req->method > TWL4030_MADC_SW2) {
		ret = -EINVAL;
		goto out;
	}
	/* Do we have a conversion request ongoing */
	if (twl4030_madc->requests[req->method].active) {
		ret = -EBUSY;
		goto out;
	}
	ch_msb = (req->channels >> 8) & 0xff;
	ch_lsb = req->channels & 0xff;
	method = &twl4030_conversion_methods[req->method];
	/* Select channels to be converted */
	ret = twl_i2c_write_u8(TWL4030_MODULE_MADC, ch_msb, method->sel + 1);
	if (ret) {
		dev_err(twl4030_madc->dev, "unable to write sel register 0x%X\n", method->sel + 1);
		goto out;
	}
	ret = twl_i2c_write_u8(TWL4030_MODULE_MADC, ch_lsb, method->sel);
	if (ret) {
		dev_err(twl4030_madc->dev, "unable to write sel register 0x%X\n", method->sel + 1);
		goto out;
	}
	/* Select averaging for all channels if do_avg is set */
	if (req->do_avg) {
		ret = twl_i2c_write_u8(TWL4030_MODULE_MADC, ch_msb, method->avg + 1);
		if (ret) {
			dev_err(twl4030_madc->dev, "unable to write avg register 0x%X\n", method->avg + 1);
			goto out;
		}
		ret = twl_i2c_write_u8(TWL4030_MODULE_MADC, ch_lsb, method->avg);
		if (ret) {
			dev_err(twl4030_madc->dev, "unable to write sel reg 0x%X\n", method->sel + 1);
			goto out;
		}
	}
	if (req->type == TWL4030_MADC_IRQ_ONESHOT && req->func_cb != NULL) {
		ret = twl4030_madc_set_irq(twl4030_madc, req);
		if (ret < 0)
			goto out;
		ret = twl4030_madc_start_conversion(twl4030_madc, req->method);
		if (ret < 0)
			goto out;
		twl4030_madc->requests[req->method].active = 1;
		ret = 0;
		goto out;
	}
	/* With RT method we should not be here anymore */
	if (req->method == TWL4030_MADC_RT) {
		ret = -EINVAL;
		goto out;
	}
	ret = twl4030_madc_start_conversion(twl4030_madc, req->method);
	if (ret < 0)
		goto out;
	twl4030_madc->requests[req->method].active = 1;
	/* Wait until conversion is ready (ctrl register returns EOC) */
	ret = twl4030_madc_wait_conversion_ready(twl4030_madc, req->method, 8, method->ctrl);
	if (ret) {
		twl4030_madc->requests[req->method].active = 0;
		goto out;
	}
	ret = twl4030_madc_read_channels(twl4030_madc, method->rbase, req->channels, req->rbuf);
	twl4030_madc->requests[req->method].active = 0;

out:
	twl4030_madc_set_power(0);
	mutex_unlock(&twl4030_madc->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(twl4030_madc_conversion);

/*
 * Return channel value
 * Or < 0 on failure.
 */
int twl4030_get_madc_conversion(int channel_no)
{
	struct twl4030_madc_request req;
	int temp = 0;
	int ret;

	req.channels = (1 << channel_no);
	req.method = TWL4030_MADC_SW1;
	req.active = 0;
	req.func_cb = NULL;
	if(channel_no == 2)
		req.do_avg = 1;
	ret = twl4030_madc_conversion(&req);
	if(ret < 0)
		return ret;
	if(req.rbuf[channel_no] > 0)
		temp = req.rbuf[channel_no] | (req.rbuf[TWL4030_MADC_MAX_CHANNELS + channel_no] << 16);

//	printk("%s: %d, %d\n", __func__, channel_no, temp);

	return temp;
}
EXPORT_SYMBOL_GPL(twl4030_get_madc_conversion);

/*
 * Return channel value
 * Or < 0 on failure.
 */
int thermistor_conversion(void)
{
	int ret;
 
	mutex_lock(&twl4030_madc->in_lock);
	if(((struct twl4030_madc_platform_data *)(twl4030_madc->dev->platform_data))->ch_sel_gpio[0] != -1)
		gpio_set_value_cansleep(((struct twl4030_madc_platform_data *)(twl4030_madc->dev->platform_data))->ch_sel_gpio[0], 1);
	ret = twl4030_get_madc_conversion(0);
	mutex_unlock(&twl4030_madc->in_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(thermistor_conversion);

/*
 * Return channel value
 * Or < 0 on failure.
 */
int brd_cfg_conversion(void)
{
	int ret;

	mutex_lock(&twl4030_madc->in_lock);
	ret = twl4030_get_madc_conversion(1);
	mutex_unlock(&twl4030_madc->in_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(brd_cfg_conversion);

/*
 * Return channel value
 * Or < 0 on failure.
 */
int analog_in_conversion(void)
{
	int ret;
	unsigned mV;

	mutex_lock(&twl4030_madc->in_lock);
	if(((struct twl4030_madc_platform_data *)(twl4030_madc->dev->platform_data))->ch_sel_gpio[0] != -1)
		gpio_set_value_cansleep(((struct twl4030_madc_platform_data *)(twl4030_madc->dev->platform_data))->ch_sel_gpio[0], 0);
	ret = twl4030_get_madc_conversion(0);
	mutex_unlock(&twl4030_madc->in_lock);
	if(ret < 0)
		return ret;

	mV = 22*(ret >> 16);
	ret &= 0xFFFF;
	ret |= mV<<16;

	//printk("%s: %d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(analog_in_conversion);

/*
 * Return channel value
 * Or < 0 on failure.
 */
int ls_conversion(void)
{
	int ret, i;
	unsigned mV, raw;

	mutex_lock(&twl4030_madc->in_lock);
	ret = twl4030_get_madc_conversion(2);
	mV = ret >> 16;
	raw = ret & 0xFFFF;
#if 0
	for(i = 0; i < 10; i++)
	{
		ret = twl4030_get_madc_conversion(2);
		mV += ret >> 16;
		raw += ret & 0xFFFF;
		mV >>= 1;
		raw >>= 1;
	}
#endif
	mutex_unlock(&twl4030_madc->in_lock);
	if(ret < 0)
		return ret;

	// Vladimir
	// In spite of A317 platform limits ADC input by 1.8V, but reality value is relative
	// and current multiplier stays unchanged for clearness of code
#if defined (CONFIG_MACH_A317)
	if(mV > 1000)
		mV = (5*mV)>>1;
	else if(mV > 800)
		mV = 18*mV/10;
	else if(mV > 500)
		mV = (3*mV)>>1;
	else if(mV > 300)
		mV = (6*mV)/5;
#else
	mV = (3*mV)>>1;
#endif
	ret = raw;
	ret |= mV<<16;

	//printk("%s: %d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(ls_conversion);

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

/*
 * sysfs hook function
 */
static ssize_t madc_conversion(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *attr;
	char name[48];
	unsigned int val;

	attr = to_sensor_dev_attr(devattr);
	//printk("%s: %d\n", __func__, attr->index);

	if(0 == attr->index)
	{
#if defined (CONFIG_MACH_A317)
		val = brd_cfg_conversion();
		//printk("%s: %d, %d, %x\n", __func__, attr->index, val, val);
		sprintf(name, "brd_cfg connected to adcin1");
#else
		val = thermistor_conversion();
		sprintf(name, "thermistor connected to adcin0");
#endif
	}
	else if(1 == attr->index)
	{
		val = analog_in_conversion();
		sprintf(name, "analog input connected to adcin0");
	}
	else
	{
		val = ls_conversion();
		sprintf(name, "light sensor connected to adcin2");
	}

#if defined (CONFIG_MACH_A317)
	if(attr->index)
	{
		if(val >= 0)
			val = sprintf(buf, "%s (raw %d --> %d mV)\n", name, val & 0xFFFF, val >> 16);
	}
	else
	{
		int imp, raw;

		raw = val & 0xFFFF;
		imp = val>>16;
		if(raw >= 0 && imp >= 0)
			val = sprintf(buf, "%s (raw %d --> %d ohm)\n", name, raw, imp);
	}

	return val;
#else
	if(val < 0)
		return val;

	return sprintf(buf, "%s (raw %d --> %d mV)\n", name, val & 0xFFFF, val >> 16);
#endif
}

static ssize_t madc_mv(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	int val;

#if defined (CONFIG_MACH_A317)
	if(0 == attr->index)
		val = brd_cfg_conversion();
#else
	if(0 == attr->index)
		val = thermistor_conversion();
#endif
	else if(1 == attr->index)
		val = analog_in_conversion();
	else
		val = ls_conversion();

	if(val < 0)
		return val;

	return sprintf(buf, "%d\n", val >> 16);
}


/* sysfs nodes to read individual channels from user side */
#if defined (CONFIG_MACH_A317)
static SENSOR_DEVICE_ATTR(adcin1_brd_cfg,	 	S_IRUGO, madc_conversion, NULL, 0);
#else
static SENSOR_DEVICE_ATTR(adcin0_thermistor,   	S_IRUGO, madc_conversion, NULL, 0);
#endif
static SENSOR_DEVICE_ATTR(adcin0_analog_in,    	S_IRUGO, madc_conversion, NULL, 1);
static SENSOR_DEVICE_ATTR(adcin0_analog_in_mv, 	S_IRUGO, madc_mv,         NULL, 1);
static SENSOR_DEVICE_ATTR(adcin2_ls_in,  	   	S_IRUGO, madc_conversion, NULL, 2);

static struct attribute *twl4030_madc_attributes[] = {
#if defined (CONFIG_MACH_A317)
	&sensor_dev_attr_adcin1_brd_cfg.dev_attr.attr,
#else
	&sensor_dev_attr_adcin0_thermistor.dev_attr.attr,
#endif
	&sensor_dev_attr_adcin0_analog_in.dev_attr.attr,
	&sensor_dev_attr_adcin0_analog_in_mv.dev_attr.attr,	
	&sensor_dev_attr_adcin2_ls_in.dev_attr.attr,
	NULL
};

static const struct attribute_group twl4030_madc_group = {
	.attrs = twl4030_madc_attributes,
};

/*
 * Initialize MADC and request for threaded irq
 */
static int twl4030_madc_probe(struct platform_device *pdev)
{
	struct twl4030_madc_data *madc;
	struct twl4030_madc_platform_data *pdata = pdev->dev.platform_data;
	struct device *hwmon;
	int ret, i;
	char name[32];
	u8 regval;

	if(!pdata)
	{
		dev_err(&pdev->dev, "platform_data not available\n");
		return -EINVAL;
	}

	madc = kzalloc(sizeof(*madc), GFP_KERNEL);
	if(!madc)
		return -ENOMEM;

	madc->dev = &pdev->dev;

	/*
	 * 2 interrupt signals are available for primary interrupt handler during RT/SW1/2 conversation.
	 * The external start conversation pin may be connected to initiate RT conversation
	 * Hence two separate ISR and IMR registers.
	 */
	if(pdata->irq_line == 2)
	{
		madc->imr = TWL4030_MADC_IMR2;
		madc->isr = TWL4030_MADC_ISR2;
	}
	else
	{
		madc->imr = TWL4030_MADC_IMR1;
		madc->isr = TWL4030_MADC_ISR1;
	}

	for(i = 0; i < sizeof(pdata->ch_sel_gpio)/sizeof(pdata->ch_sel_gpio[0]); i++)
	{
		if(pdata->ch_sel_gpio[i] != -1)
		{
			sprintf(name, "adcin%d switch", i);
			gpio_request(pdata->ch_sel_gpio[i], name);
			gpio_direction_output(pdata->ch_sel_gpio[i], i);
		}
	}

	twl4030_madc = madc;

	ret = twl4030_madc_set_power(1);
	if (ret < 0)
		goto err_power;

	/* Check that MADC clock is on */
	ret = twl_i2c_read_u8(TWL4030_MODULE_INTBR, &regval, TWL4030_REG_GPBR1);
	if(ret)
	{
		dev_err(&pdev->dev, "unable to read reg GPBR1 0x%X\n", TWL4030_REG_GPBR1);
		goto err_i2c;
	}

	// select MADC clock - 1 MHz
	regval |= (TWL4030_GPBR1_MADC_HFCLK_EN | TWL4030_GPBR1_MADC_DFLT_CLK_EN);
	dev_info(&pdev->dev, "1 MHz clock enabling\n");
	ret = twl_i2c_write_u8(TWL4030_MODULE_INTBR, regval, TWL4030_REG_GPBR1);
	if(ret)
	{
		dev_err(&pdev->dev, "unable to write reg GPBR1 0x%X\n", TWL4030_REG_GPBR1);
		goto err_i2c;
	}

	platform_set_drvdata(pdev, madc);
	mutex_init(&madc->lock);
	mutex_init(&madc->in_lock);
	ret = request_threaded_irq(platform_get_irq(pdev, 0), NULL,
							   twl4030_madc_threaded_irq_handler, IRQF_TRIGGER_RISING, "twl4030_madc", madc);
	if(ret)
	{
		dev_dbg(&pdev->dev, "could not request irq\n");
		goto err_irq;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &twl4030_madc_group);
	if(ret)
	{
		printk("%s: failed register fs group.\n", __func__);
		goto err_irq;
	}

	hwmon = hwmon_device_register(&pdev->dev);
	if(IS_ERR(hwmon))
	{
		printk("%s: failed.\n", __func__);
		ret = PTR_ERR(hwmon);
		goto err_reg;
	}

	twl4030_madc_set_power(0);
	return 0;
err_reg:
	sysfs_remove_group(&pdev->dev.kobj, &twl4030_madc_group);
err_irq:
	platform_set_drvdata(pdev, NULL);
err_i2c:
	twl4030_madc_set_power(0);
err_power:
	kfree(madc);

	return ret;
}

static int twl4030_madc_remove(struct platform_device *pdev)
{
	int i;
	struct twl4030_madc_data *madc = platform_get_drvdata(pdev);
	struct twl4030_madc_platform_data *pdata = pdev->dev.platform_data;

	hwmon_device_unregister(&pdev->dev);
	sysfs_remove_group(&pdev->dev.kobj, &twl4030_madc_group);

	free_irq(platform_get_irq(pdev, 0), madc);
	platform_set_drvdata(pdev, 0);
	twl4030_madc_set_power(0);

	for(i = 0; i < sizeof(pdata->ch_sel_gpio)/sizeof(pdata->ch_sel_gpio[0]); i++)
	{
		if(pdata->ch_sel_gpio[i] != -1)
		{
			gpio_free(pdata->ch_sel_gpio[i]);
		}
	}
	kfree(madc);

	return 0;
}

static struct platform_driver twl4030_madc_driver = {
	.probe = twl4030_madc_probe,
	.remove = twl4030_madc_remove,
	.driver = {
		   .name = "twl4030_madc",
		   .owner = THIS_MODULE,
		   },
};

static int __init
twl4030_madc_init(void)
{
	printk("%s\n", __func__);
	return platform_driver_register(&twl4030_madc_driver);
}

module_init(twl4030_madc_init);

static void __exit
twl4030_madc_destroy(void)
{
	printk("%s\n", __func__);
	platform_driver_unregister(&twl4030_madc_driver);
}

module_exit(twl4030_madc_destroy);

MODULE_DESCRIPTION("TWL4030 ADC driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("J Keerthy");
MODULE_ALIAS("platform:twl4030_madc");
 
