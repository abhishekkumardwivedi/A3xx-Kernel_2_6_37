/*
 * OMAP system control module header file
 *
 * Copyright (C) 2013 Micronet IL - http://www.micronet.co.il/
 * Author: R Meyerstein <ranm@micronet.co.il>
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

/*
 * The register offsets and but fields might change across
 * OMAP versions hence populating them in this structure.
 */


/*
 * The thresholds and limits for temperature sensors.
 */
struct omap3703_temp_sensor_data {
	u32	t_hot;
	u32	t_cold;
	u32	min_freq;
	u32	max_freq;
	int	max_temp;
	int	min_temp;
	int	hyst_val;
	u32	adc_start_val;
	u32	adc_end_val;
	u32	update_int1;
	u32	update_int2;
};

/* forward declaration */
struct scm;

/**
 * struct temp_sensor_hwmon - temperature sensor hwmon device structure
 * @scm_ptr: pointer to system control module structure
 * @pdev: platform device pointer for hwmon device
 * @name: Name of the hwmon temp sensor device
 */
struct temp_sensor_hwmon {
	struct scm		*scm_ptr;
	struct platform_device	*pdev;
	const char		*name;
};


/**
 * struct system control module - scm device structure
 * @dev: device pointer
 * @ts_data: Pointer to struct with thresholds, limits of temperature sensor
 * @therm_fw: Pointer to list of thermal devices
 * @fclock: pointer to functional clock of temperature sensor
 * @div_clk: pointer to parent clock of temperature sensor fclk
 * @tsh_ptr: pointer to temperature sesnor hwmon struct
 * @name: pointer to list of temperature sensor instance names is scmi
 * @conv_table: Pointer to adc to temperature conversion table
 * @scm_mutex: Mutex for sysfs, irq and PM
 * @irq: MPU Irq number for thermal alert
 * @base: Base of the temp I/O
 * @clk_rate: Holds current clock rate
 * @cnt: count of temperature sensor device in scm
 * @rev: Revision of Temperature sensor
 * @Acc: Accuracy of the temperature
 */
struct scm {
	struct device			*dev;
	struct omap3703_temp_sensor_data **ts_data;
#if defined(CONFIG_CPU_THERMAL)
	struct thermal_sensor_conf	*cpu_therm;
#endif
	struct clk		*fclock;
	struct clk		*div_clk;
	struct temp_sensor_hwmon *tsh_ptr;
	const char		**name;
	int			*conv_table;
	struct mutex		scm_mutex; /* Mutex for sysfs, irq and PM */
	unsigned int		irq;
	void __iomem		*base;
	u32			clk_rate;
	u32			cnt;
	int			rev;
	bool        accurate;
	int         tbgp;
    int         tcom;
	struct kobject		kobj;
	struct completion	kobj_unregister;
	int overheat_irq_in;
	int overheat_irq_pol;
	int overheat_irq;
	struct delayed_work overhet_work;
};  


u32 omap3703scm_readl(struct scm *scm_ptr, u32 reg);
void omap3703_scm_writel(struct scm *scm_ptr, u32 val, u32 reg);
int omap3703_temp_sensor_init(struct scm *scm_ptr);
void omap3703_temp_sensor_deinit(struct scm *scm_ptr);
int omap3703_tshut_init(struct scm *scm_ptr);
int omap3703_scm_read_temp(struct scm *scm_ptr, int id);
int adc_to_temp_conversion(struct scm *scm_ptr, int id, int val);
#ifdef CONFIG_CPU_THERMAL
extern struct omap_thermal_zone *omap_zones[3];
#endif

