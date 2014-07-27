/* platform data for the PCA9575 16-bit I/O expander driver */
#ifndef PCA_9575_H__
#define PCA_9575_H__

#include <linux/i2c.h>

struct pca9575_platform_data {

	unsigned    irq_base;

	/* number of the interrupt GPIO provided to OMAP*/
	unsigned	gpio_base;

	/* interrupt mask */
	uint16_t    imask;

	/* initial polarity inversion setting */
	uint16_t	invert;


	void		*context;	/* param to setup/teardown */


	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
};

#endif
