#ifndef	OMAP3_THERMAL_DATA_H
#define	OMAP3_THERMAL_DATA_H

#include <linux/cpu_cooling.h>

#define TWL_GPIO_2 194

struct omap3_thermal_data {
	int threshold;
	int trigger_levels[6];
	struct freq_clip_table freq_tab[6];
	unsigned int freq_tab_count;
};

#endif

