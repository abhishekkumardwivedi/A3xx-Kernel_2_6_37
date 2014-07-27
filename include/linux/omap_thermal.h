#ifndef	OMAP_THERMAL_H
#define	OMAP_THERMAL_H

#define SENSOR_NAME_LEN	20
#define MAX_COOLING_DEVICE 5

enum omap3_one_ghz_state {
	ONE_GHZ_NOT_ALLOWED = 0,
	ONE_GHZ_ALLOWED_UNLIMITED,
	ONE_GHZ_ALLOWED_LIMITED
};

struct thermal_sensor_conf {
	char	name[SENSOR_NAME_LEN];
	int	(*read_temperature)(void *data);
	void	*private_data;
	void	*sensor_data;
};

struct omap_thermal_zone {
	unsigned int idle_interval;
	unsigned int active_interval;
	struct thermal_zone_device *therm_dev;
	struct thermal_cooling_device *cool_dev[MAX_COOLING_DEVICE];
	unsigned int cool_dev_size;
	enum thermal_device_mode mode;
	struct thermal_sensor_conf *sensor_conf;
	struct omap3_thermal_data *sensor_data;
	enum omap3_one_ghz_state ghz_state;
};

/**
 * omap3_register_thermal: Register to the omap thermal 
 * interface. 
 * @sensor_conf:   Structure containing temperature sensor information
 *
 * returns zero on success, else negative errno.
 */
struct omap_thermal_zone *omap3_register_thermal(
	struct thermal_sensor_conf *sensor_conf);

/**
 * omap3_unregister_thermal: Un-register from the omap thermal 
 * interface. 
 *
 * return not applicable.
 */
void omap3_unregister_thermal(struct omap_thermal_zone *th_zone);

void omap3_report_trigger(struct omap_thermal_zone *th_zone);

#endif
