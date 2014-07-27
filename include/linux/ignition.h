/*  linux/include/linux/ignition.h
 *
 *  This file contains the structure definitions for ignition events.
 *
 */
#ifndef _IGNITION_H
#define _IGNITION_H

#include <linux/notifier.h>

#define AIN_LOW			0
#define AIN_HIGH		1

/* Clock event notification values */
enum Ignition_event_nofitiers {
	IGNITION_EVT_NOTIFY_IGN_OFF,
	IGNITION_EVT_NOTIFY_IGN_ON,
};
typedef struct ignition_platform_data {
	int ig_in;
	int active_level;
};

extern int ignition_get_state(void);

extern int ignitionevents_register_notifier(struct notifier_block *nb);

extern void ignitionevents_notify(unsigned long reason, void *arg);

#endif //_IGNITION_H
