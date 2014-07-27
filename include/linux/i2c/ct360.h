#ifndef CT360_H
#define CT360_H

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

// ****************************************************************************
// Defines
// ****************************************************************************
#define CT36X_TS_CORE_DEBUG				0
#define CT36X_TS_EVENT_DEBUG			0
#define CT36X_TS_CHIP_DEBUG			    0//1

/* max touch points supported */
#define CT36X_TS_POINT_NUM			5

#define CT36X_TS_ABS_X_MAX			1024//800
#define CT36X_TS_ABS_Y_MAX			600//768//480

#define CT360_I2C_ADDRESS           1

/* data structure of point event */
/* Old Touch Points Protocol
---------+-+-+-+-+-+-+-+-+
Byte0|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Finger ID|Statu|
---------+-+-+-+-+-+-+-+-+
Byte1|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |X High         |
---------+-+-+-+-+-+-+-+-+
Byte2|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Y High         |
---------+-+-+-+-+-+-+-+-+
Byte3|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |X Low  |X High |
---------+-+-+-+-+-+-+-+-+
*/
/* New Touch Points Protocol
---------+-+-+-+-+-+-+-+-+
Byte0|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |X High         |
---------+-+-+-+-+-+-+-+-+
Byte1|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Y High         |
---------+-+-+-+-+-+-+-+-+
Byte2|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |X Low  |X High |
---------+-+-+-+-+-+-+-+-+
Byte3|Bit|7|6|5|4|3|2|1|0|
---------+-+-+-+-+-+-+-+-+
         |Finger ID|Statu|
---------+-+-+-+-+-+-+-+-+
*/

struct ct36x_finger_info {
	unsigned char	xhi;			// X coordinate Hi
	unsigned char	yhi;			// Y coordinate Hi
	unsigned char	ylo : 4;		// Y coordinate Lo
	unsigned char	xlo : 4;		// X coordinate Lo
	unsigned char	status : 4;		// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 4;			// ID information, from 1 to CFG_MAX_POINT_NUM
};


enum enum_ct36x_state {
	CT36X_STATE_UNKNOWN,
	CT36X_STATE_INIT,
	CT36X_STATE_NORMAL,
	CT36X_STATE_SLEEP,
	CT36X_STATE_UPDATE,
};

enum enum_ct36x_ts_cmds {
	CT36X_TS_CHIP_ID,
	CT36X_TS_CHIP_RESET,
	CT36X_TS_FW_VER,
	CT36X_TS_FW_CHKSUM,
	CT36X_TS_FW_UPDATE,
	CT36X_TS_BIN_VER,
	CT36X_TS_BIN_CHKSUM,
};

// Platform data
struct ct36x_platform_data {
	int 				rst;
	int 				ss;
};

union ct36x_i2c_data {
	struct ct36x_finger_info	pts[CT36X_TS_POINT_NUM];
	unsigned char			buf[CT36X_TS_POINT_NUM * sizeof(struct ct36x_finger_info)];
};

struct ct36x_ts_info {
	/* Chip ID */
	int				chip_id;

	/* platform data */
	struct ct36x_platform_data	*pdata;

	/* Communication settings */
	int				i2c_bus;
	unsigned short			i2c_address;
	struct i2c_client		*client;

	/* input devices */
	struct input_dev		*input;

	/* hw config */
	int 				irq;
	int 				rst;
	int 				ss;
	int				state;

	// Early suspend
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend		early_suspend;
#endif

	// Proc Control
	struct proc_dir_entry		*proc_entry;

	// Work thread settings
	struct work_struct		event_work;
	struct workqueue_struct 	*workqueue;
	
	/* touch event data & status */
	union ct36x_i2c_data		data;
	int				press;
	int				release;
};

//////////////////////////////////////////////

int ct36x_chip_get_binchksum(unsigned char *buf);
int ct36x_chip_get_fwchksum(struct i2c_client *client, unsigned char *buf);

void ct36x_chip_go_sleep(struct i2c_client *client, unsigned char *buf);
int ct36x_chip_go_bootloader(struct i2c_client *client, unsigned char *buf);

void ct36x_chip_set_adapter_on(struct i2c_client *client, unsigned char *buf);
void ct36x_chip_set_adapter_off(struct i2c_client *client, unsigned char *buf);

int ct36x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
void ct36x_ts_shutdown(struct i2c_client *client);
int ct36x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
int ct36x_ts_resume(struct i2c_client *client);
int __devexit ct36x_ts_remove(struct i2c_client *client);

int __init ct36x_ts_init(void);
void __exit ct36x_ts_exit(void);

#endif
