/*
 * OMAP37XX system control module driver file
 *
 * Copyright (C) 2013 Micronet IL - http://www.micronet.co.il/
 * Author: R Meyerstein <ranm@micronet.co.il>
 *
 * (C) Copyright 2012 - 2014 Micronet Ltd <http://www.micronet.co.il>
 * Vladimir Zatulovsky, vladimirz@micronet.co.il
 * A31x boards adaptation
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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <plat/omap_device.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <plat/scm.h>
#include <linux/mfd/omap3_scm.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/cpu.h>

#if defined(CONFIG_CPU_THERMAL)
#include <linux/thermal.h>
#include <linux/omap_thermal.h>
#include <linux/platform_data/omap3_thermal_data.h>
#endif
/* Offsets from the base of temperature sensor registers */

#define OMAP3703_TEMP_SENSOR_CTRL_OFFSET	0x2B4

#define OMAP3703_MAX_FREQ           1000000
#define OMAP3703_MAX_FREQ_800MHZ    800000
#define OMAP3703_MIN_FREQ           600000
#define OMAP3703_MIN_TEMP           -40000
#define OMAP3703_MAX_TEMP	        123000
#define OMAP3703_HYST_VAL		    5000
#define OMAP3703_ADC_START_VALUE	0
#define OMAP3703_ADC_END_VALUE		127
#define OMAP3703_T_HOT		        99000	/* 110 deg C - MJT for now for 1GHz support*/
#define OMAP3703_T_COLD             90000	/* 95 deg C - LTT for now for 1GHz support*/
#define OMAP3703_FORCE_UPDATE       1

#define TEMPSENSOR_CONVERSION_VALUE	1501

#define show_one_par(file_name, object)			\
static ssize_t show_##file_name				\
(struct scm *scm_ptr, char *buf)		\
{							\
	return sprintf(buf, "%u\n", scm_ptr->object);	\
}

struct temperature_attr {
	struct attribute attr;
	ssize_t (*show)(struct scm *, char *);
	ssize_t (*store)(struct scm *, const char *, size_t count);
};

#define temperature_sensor_attr_ro_perm(_name, _perm)	\
static struct temperature_attr _name =			\
__ATTR(_name, _perm, show_##_name, NULL)


show_one_par(current_bgapts_temp, tbgp);
show_one_par(current_compan_temp, tcom);

temperature_sensor_attr_ro_perm(current_bgapts_temp, 0400);
temperature_sensor_attr_ro_perm(current_compan_temp, 0400);

#define to_scm(k) container_of(k, struct scm, kobj)
#define to_temperature_attr(a) container_of(a, struct temperature_attr, attr)

static struct attribute *default_attrs[] = {
	&current_bgapts_temp.attr,
	&current_compan_temp.attr,
	NULL
};

static ssize_t show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct scm *scm_ptr = to_scm(kobj);
	struct temperature_attr *tattr = to_temperature_attr(attr);
	ssize_t ret = -EINVAL;

	if (tattr->show)
		ret = tattr->show(scm_ptr, buf);
	else
		ret = -EIO;

	return ret;
}

static ssize_t store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	struct scm *scm_ptr = to_scm(kobj);
	struct temperature_attr *tattr = to_temperature_attr(attr);
	ssize_t ret = -EINVAL;

	if (tattr->store)
		ret = tattr->store(scm_ptr, buf, count);
	else
		ret = -EIO;

	return ret;
}

static const struct sysfs_ops sysfs_ops = {
	.show	= show,
	.store	= store,
};

static void omap3703tempsensor_sysfs_release(struct kobject *kobj)
{
	struct scm *scm_ptr = to_scm(kobj);
	printk("last reference is dropped\n");
	complete(&scm_ptr->kobj_unregister);
}

static struct kobj_type ktype_scm = {
	.sysfs_ops	= &sysfs_ops,
	.default_attrs	= default_attrs,
	.release	= omap3703tempsensor_sysfs_release,
};

extern u32 omap2_get_dpll_rate(struct clk *clk);

/* Thresholds and limits for OMAP3703 MPU temperature sensor */
struct omap3703_temp_sensor_data omap3703_mpu_temp_sensor_data = {
	.t_hot = 0,
	.t_cold = 0,
	.min_freq = OMAP3703_MIN_FREQ,
	.max_freq = OMAP3703_MAX_FREQ,
	.max_temp = OMAP3703_MAX_TEMP,
	.min_temp = OMAP3703_MIN_TEMP,
	.hyst_val = OMAP3703_HYST_VAL,
	.adc_start_val = OMAP3703_ADC_START_VALUE,
	.adc_end_val = OMAP3703_ADC_END_VALUE,
	.update_int1 = 1000,
	.update_int2 = 2000,
};


/*
 * Temperature values in degree celsius
 * ADC code values from 0 to 127
 */
int omap3703_adc_to_temp[OMAP3703_ADC_END_VALUE - OMAP3703_ADC_START_VALUE + 1]
	= {-40000 ,-40000 ,-40000 ,-40000 ,-40000 ,-40000 ,-40000 ,-40000 ,-40000 ,-40000 ,-40000 ,-40000 ,-40000 , //0	    12
       -39000 ,-37000 ,-35000 ,-33000 ,-31000 ,-29000 ,-27000 ,-25000 ,-23000 ,-21000 ,-19000 ,-18000 ,-16000 , //13	25
	   -14000 ,-13000 ,-11000 ,-9000  ,-7000  ,-6000  ,-4000  ,-2000  ,-1000  ,1000   ,3000   ,4000   ,6000   , //26	38
		7000  ,9000   ,11000  ,13000  ,14000  ,16000  ,18000  ,20000  ,22000  ,24000  ,26000  ,28000  ,29000  , //39	51
		31000 ,33000  ,34000  ,36000  ,38000  ,39000  ,41000  ,43000  ,44000  ,46000  ,48000  ,49000  ,51000  , //52	64
		53000 ,54000  ,56000  ,58000  ,59000  ,61000  ,63000  ,65000  ,67000  ,69000  ,71000  ,73000  ,74000  , //65	77
		76000 ,78000  ,79000  ,81000  ,83000  ,84000  ,86000  ,88000  ,89000  ,91000  ,93000  ,94000  ,96000  , //78	90
		98000 ,99000  ,101000 ,103000 ,105000 ,106000 ,108000 ,110000 ,112000 ,114000 ,116000 ,118000 ,120000 , //91	103
		121000,123000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 , //104	116
		125000,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000 ,125000                   //117	127
};

#if !defined (CONFIG_MACH_A317)
int omap3703_cc_therm_voltage[] =      { 1186, 1128, 1064, 997, 927, 856, 785, 716, 648, 584, 524, 468, 416, 370, 328, 291, 258, 228, 202, 179, 159, 141, 126, 112, 100,  46,   0 };
int omap3703_cc_therm_temperatures[] = {  -10,   -5,    0,   5,  10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  60,  65,  70,  75,  80,  85,  90,  95, 100, 105, 110, 115, 120 };
#endif

struct omap_thermal_zone *omap_zones[3];

/* temp/frequency table for 1GHz chip */
static struct omap3_thermal_data omap3_1000mhz_bandgap_data = {
	.trigger_levels[0] = 95000,
	.trigger_levels[1] = 100000,
	.trigger_levels[2] = 110000,
	/* 1000Mhz */
	.freq_tab[0] = {
		.freq_clip_max = 1000 * 1000,
		.polling_interval = 1000,
		},
	/* 800MHz */
	.freq_tab[1] = {
		.freq_clip_max = 800 * 1000,
		.polling_interval = 300,
		},
	/* 600MHz */
	.freq_tab[2] = {
		.freq_clip_max = 600 * 1000,
		.polling_interval = 300,
		},
	/* Shutdown */
	.freq_tab[3] = {
		.freq_clip_max = 0,
		},
	.freq_tab_count = 3,
};

/* temp/frequency table for 800MHz chip */
static struct omap3_thermal_data omap3_800mhz_bandgap_data = {
	.trigger_levels[0] = 110000,
	.trigger_levels[1] = 115000,
	/* 800Mhz */
	.freq_tab[0] = {
		.freq_clip_max = 800 * 1000,
		.polling_interval = 1000,
		},
	/* 600MHz */
	.freq_tab[1] = {
		.freq_clip_max = 600 * 1000,
		.polling_interval = 300,
		},
	/* Shutdown */
	.freq_tab[3] = {
		.freq_clip_max = 0,
		},
	.freq_tab_count = 2,
};

static int omap_read_temp(void *private_data)
{
	struct scm *scm_ptr = private_data;
	int temp;

	mutex_lock(&scm_ptr->scm_mutex);
	temp = omap3703_scm_read_temp(scm_ptr, 0);
	scm_ptr->tbgp = temp;
	mutex_unlock(&scm_ptr->scm_mutex);

	return temp;
}

int adc_to_temp_conversion(struct scm *scm_ptr, int id, int adc_val)
{
#ifdef DEBUGCDS
	printk("adc_to_temp_conversion adc_val=[%d] id[%d] scm_ptr->conv_table[adc_val]=[%d] scm_ptr->ts_data[0]->adc_start_val=[%d]\n", adc_val, id, scm_ptr->conv_table[adc_val], scm_ptr->ts_data[0]->adc_start_val);
#endif
	return scm_ptr->conv_table[adc_val];
}

static int omap3703_companionchip_thermistor_read(struct scm *scm_ptr, int *result)
{
#if !defined (CONFIG_MACH_A317)
	int i, temp, mv = 0;

	temp = thermistor_conversion();
	mv = temp >> 16;
	mv &= 0xFFFF;

	while (mv < omap3703_cc_therm_voltage[i]) {
		i++;
	}

	*result = omap3703_cc_therm_temperatures[i];
#else
	if(scm_ptr && gpio_get_value_cansleep(scm_ptr->overheat_irq_in) == scm_ptr->overheat_irq_pol)
	{
		printk("%s: over heat\n", __func__);
		*result = 120;	// over heat
	}
	else
	{
		//printk("%s: normal\n", __func__);
		*result = 0;	// normal
	}
#endif
#ifdef DEBUGCDS
	printk("Companion Chip Thermistor Temperature [%d]\n", *result);
#endif

	return 0;
}

static int omap3703_companionchip_handle(struct scm *scm_ptr, int temp)
{
#ifdef DEBUGCDS
	printk("omap3703_companionchip_handle temp[%d] omap_zones[0]->ghz_state[%d]\n",temp,omap_zones[0]->ghz_state);
#endif
	if (omap_zones[0]->ghz_state != ONE_GHZ_NOT_ALLOWED) {
		if (temp >= 110) {
			if (omap_zones[0]->ghz_state == ONE_GHZ_ALLOWED_UNLIMITED) {
				if (omap3_opp_disable_1GHz() == 0) {
					omap_zones[0]->ghz_state = ONE_GHZ_ALLOWED_LIMITED;
					printk("Succesfully limited to 800MHz \n");
					return OMAP3703_FORCE_UPDATE;
				} else
					return -EINVAL;
			}

		}
		if (temp <= 105) {
			if (omap_zones[0]->ghz_state == ONE_GHZ_ALLOWED_LIMITED) {
				if (omap3_opp_enable_1GHz() == 0) {
					omap_zones[0]->ghz_state = ONE_GHZ_ALLOWED_UNLIMITED;
					printk("Succesfully limited to 1GHz \n");
					return OMAP3703_FORCE_UPDATE;
				} else
					return -EINVAL;
			}
		}
	}

	return 0;
}

int omap3703_scm_read_temp(struct scm *scm_ptr, int id)
{
	int temp, ret;

	//Check if initialized
	if (!omap_zones[0]) {
		return 0;
	}

	//Read Companion Chip Thermistor A2D
	omap3703_companionchip_thermistor_read(scm_ptr, &temp);

	scm_ptr->tcom = temp;

	ret = omap3703_companionchip_handle(scm_ptr, temp);
	if(ret == OMAP3703_FORCE_UPDATE) {
		//force frequency to minimum
		printk("%s: [%d] force frequency to update \n", __func__, scm_ptr->conv_table[OMAP3703_ADC_END_VALUE]);
		return scm_ptr->conv_table[OMAP3703_ADC_END_VALUE];
	}

	//Read internal OMAP temperature sensor
	temp = __raw_readl(scm_ptr->base) & 0xff;

	return adc_to_temp_conversion(scm_ptr, id, temp);
}
EXPORT_SYMBOL(omap3703_scm_read_temp);

/**
 * enable_continuous_mode() - One time enbaling of continuous conversion mode
 * @scm_ptr - pointer to scm instance
 */
static void enable_continuous_mode(struct scm *scm_ptr)
{
	__raw_writel(0, scm_ptr->base);
	__raw_writel(0x600, scm_ptr->base);
}

int omap3703_tshut_init(struct scm *scm_ptr)
{
	int status, gpio_nr = 127;

	/* Request for gpio_127 line */
	status = gpio_request(gpio_nr, "tshut");
	if (status < 0) {
		pr_err("Could not request for TSHUT GPIO:%i\n", 86);
		return status;
	}
	status = gpio_direction_input(gpio_nr);
	if (status) {
		pr_err("Cannot set input TSHUT GPIO %d\n", gpio_nr);
		return status;
	}

	return 0;
}

static int omap3703_cpu_cooling_init(struct scm *scm_ptr)
{
	scm_ptr->cpu_therm = kzalloc(sizeof(struct thermal_sensor_conf) *
								1, GFP_KERNEL);
								 //scm_ptr->cnt, GFP_KERNEL);
	if (!scm_ptr->cpu_therm) {
		pr_err("Unable to allocate memory for ts data\n");

		return -ENOMEM;
	}

	if (scm_ptr->rev == 1) {//This is A300 with companion chip TPS65920 that doesn't support 1GHz 
		strncpy(scm_ptr->cpu_therm[0].name, "omap_ondie_sensor", SENSOR_NAME_LEN);
		scm_ptr->cpu_therm[0].private_data = scm_ptr;
		scm_ptr->cpu_therm[0].read_temperature = omap_read_temp;
		scm_ptr->cpu_therm[0].sensor_data = &omap3_800mhz_bandgap_data;
		omap_zones[0] = omap3_register_thermal(scm_ptr->cpu_therm);
		omap_zones[0]->ghz_state = ONE_GHZ_NOT_ALLOWED;
	} else if (scm_ptr->rev == 2) {//This is a revision with TPS65950 A3
		strncpy(scm_ptr->cpu_therm[0].name, "omap_ondie_sensor", SENSOR_NAME_LEN);
		scm_ptr->cpu_therm[0].private_data = scm_ptr;
		scm_ptr->cpu_therm[0].read_temperature = omap_read_temp;
		if(omap3_has_mfreq1ghz())
			scm_ptr->cpu_therm[0].sensor_data = &omap3_1000mhz_bandgap_data;
		else
			scm_ptr->cpu_therm[0].sensor_data = &omap3_800mhz_bandgap_data;
		omap_zones[0] = omap3_register_thermal(scm_ptr->cpu_therm);
		if (omap3_has_mfreq1ghz()) 
			omap_zones[0]->ghz_state = ONE_GHZ_ALLOWED_UNLIMITED;
		else
			omap_zones[0]->ghz_state = ONE_GHZ_NOT_ALLOWED;
	}

	printk("%s: MPU 1 GHz %s\n", __func__, (omap_zones[0]->ghz_state == ONE_GHZ_ALLOWED_UNLIMITED)?"allowed unlimited":
										   (omap_zones[0]->ghz_state == ONE_GHZ_ALLOWED_LIMITED)?"allowed":
										   "not allowed");

	return 0;
}

int omap3703_temp_sensor_init(struct scm *scm_ptr)
{
	struct omap3703_temp_sensor_data *ts_ptr;
	int rate, clk_rate, ret = 0;
	const char * str = "dpll1_ck";
	struct clk * mpuclk; 
	struct sys_device *sys_dev;

	scm_ptr->base = ioremap(0x48002524, 4);

	scm_ptr->ts_data = kzalloc(sizeof(ts_ptr), GFP_KERNEL);
	if (!scm_ptr->ts_data) {
		pr_err("Unable to allocate memory for ts data\n");
		ret = -ENOMEM;
		goto free_ts_data;
	}

	scm_ptr->conv_table = omap3703_adc_to_temp;
	scm_ptr->ts_data[0] = &omap3703_mpu_temp_sensor_data;

	if(omap3_has_mfreq1ghz() == 0)
	{
		printk("%s: MPU doesn't support 1 GHz clock\n", __func__);
		scm_ptr->ts_data[0]->max_freq = OMAP3703_MAX_FREQ_800MHZ;
	}

	//Extract MPU Rate from Boot parameter indicating maximal rate for this HW
	mpuclk = clk_get(NULL, str);
	rate = omap2_get_dpll_rate(mpuclk);
	
	if(rate == 1000000000) {
		scm_ptr->rev = 2;
	}
	else
	{
		scm_ptr->rev = 1;
	}

	clk_rate = clk_round_rate(scm_ptr->div_clk, scm_ptr->ts_data[0]->max_freq);
	printk("%s: MPU max clock (%d, %d, %d)\n", __func__, scm_ptr->ts_data[0]->max_freq, rate, clk_rate);

	ret = omap3703_cpu_cooling_init(scm_ptr);
	if (ret)
		goto free_ts_data;

	enable_continuous_mode(scm_ptr);

	/* prepare interface data */
	sys_dev = get_cpu_sysdev(0);//only one cpu supported
	ret = kobject_init_and_add(&scm_ptr->kobj, &ktype_scm, &sys_dev->kobj, "scm");

	return ret;

free_ts_data:
	kfree(scm_ptr->ts_data);
	return ret;
}

void omap3703_temp_sensor_deinit(struct scm *scm_ptr)
{
	kfree(scm_ptr->ts_data);
}

