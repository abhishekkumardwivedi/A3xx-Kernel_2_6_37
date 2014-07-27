/*
 * drivers/input/touchscreen/ct36x_ts.c
 *
 * Copyright (C) 2013 Micronet, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <plat/board.h>
#include <linux/i2c/ct360.h>

//#define CT36X_TS_CHIP_DEBUG 1

#define CT36X_CHIP_FLASH_SECTOR_NUM	8
#define CT36X_CHIP_FLASH_SECTOR_SIZE	2048
#define CT36X_CHIP_FLASH_SOURCE_SIZE	8

#define GPIO_LOW  0
#define GPIO_HIGH 1

#define ct360_TS_NAME "ct360_ts"

static struct i2c_device_id ct36x_ts_id[] = {
	{ ct360_TS_NAME, 0 },
	{ }
};

// ****************************************************************************
// Globel or static variables
// ****************************************************************************
static struct ct36x_ts_info	ct36x_ts;

static unsigned char binary_data[] = {
#include "jx02yk02_13x17_CT360_V03_20130130.dat"
};

// ****************************************************************************
// Function declaration
// ****************************************************************************
int ct36x_cmd_list_ind[] = {
	CT36X_TS_CHIP_ID,
	CT36X_TS_CHIP_RESET,
	CT36X_TS_FW_VER,
	CT36X_TS_FW_CHKSUM,
	CT36X_TS_FW_UPDATE,
	CT36X_TS_BIN_VER,
	CT36X_TS_BIN_CHKSUM,
};

void ct36x_platform_get_cfg(struct ct36x_ts_info *ct36x_ts)
{
	if (ct36x_ts->state == CT36X_STATE_INIT) {
		/* I2C config */
		ct36x_ts->i2c_address = CT360_I2C_ADDRESS;

		/* GPIO config */
		ct36x_ts->rst = ct36x_ts->pdata->rst; //must be M307/M307I dependent  
		ct36x_ts->ss = ct36x_ts->pdata->ss;   //103; 

		/* IRQ config*/
		ct36x_ts->irq = gpio_to_irq(ct36x_ts->ss);
	}
}

void ct36x_ts_reg_read(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	struct i2c_msg msgs;

	int ret;

	msgs.addr = addr;
	msgs.flags = 0x01;  // 0x00: write 0x01:read
	msgs.len = len;
	msgs.buf = buf;

	ret = i2c_transfer(client->adapter, &msgs, 1);

	//if (CT36X_TS_CHIP_DEBUG)
		//printk("%s: i2c_transfer addr[0x%x] return state[%d]\n",__FUNCTION__, msgs.addr ,ret);
}

void ct36x_ts_reg_write(struct i2c_client *client, unsigned short addr, char *buf, int len)
{
	struct i2c_msg msgs;
	int ret;

	msgs.addr = addr;
	msgs.flags = 0x00;  // 0x00: write 0x01:read
	msgs.len = len;
	msgs.buf = buf;

	ret = i2c_transfer(client->adapter, &msgs, 1);

	//printk("%s: i2c_transfer addr[0x%x] return state[%d]\n",__FUNCTION__, msgs.addr ,ret);
}
static void ct36x_chip_set_idle(struct i2c_client *client, unsigned char *buf)
{
	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0x00;
	buf[1] = 0xA5;
	ct36x_ts_reg_write(client, 0x7F, buf, 2);
	mdelay(10);
}

static void ct36x_chip_rst_offset(struct i2c_client *client, unsigned char *buf)
{
	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0x00;
	ct36x_ts_reg_write(client, 0x7F, buf, 1);
	mdelay(10);
}

static int ct36x_chip_set_code(unsigned int flash_addr, unsigned char *buf)
{
	unsigned char cod_chksum;

	//if ( CT36X_TS_CHIP_DEBUG )
	//	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// Flash address
	// data length
	buf[2] = (char)(flash_addr >> 8);
	buf[3] = (char)(flash_addr & 0xFF);
	buf[4] = 0x08;

	// Fill firmware source data
	//if ( (sec == 1 && cod == 4) || (sec == 1 && cod == 5) ) {
	//if ( flash_addr == (CT36X_CHIP_FLASH_SECTOR_SIZE + 32) ||
	//flash_addr == (CT36X_CHIP_FLASH_SECTOR_SIZE + 40) ) {
	if (flash_addr == (160) || flash_addr == (168)) {
		buf[6] = ~binary_data[flash_addr + 0];
		buf[7] = ~binary_data[flash_addr + 1];
		buf[8] = ~binary_data[flash_addr + 2];
		buf[9] = ~binary_data[flash_addr + 3];
		buf[10] = ~binary_data[flash_addr + 4];
		buf[11] = ~binary_data[flash_addr + 5];
		buf[12] = ~binary_data[flash_addr + 6];
		buf[13] = ~binary_data[flash_addr + 7];
	} else {
		buf[6] = binary_data[flash_addr + 0];
		buf[7] = binary_data[flash_addr + 1];
		buf[8] = binary_data[flash_addr + 2];
		buf[9] = binary_data[flash_addr + 3];
		buf[10] = binary_data[flash_addr + 4];
		buf[11] = binary_data[flash_addr + 5];
		buf[12] = binary_data[flash_addr + 6];
		buf[13] = binary_data[flash_addr + 7];
	}

	/* Calculate a checksum by Host controller. 
	** Checksum =  ~(FLASH_ADRH+FLASH_ADRL+LENGTH+
	** Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
	** Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
	*/
	cod_chksum = ~(buf[2] + buf[3] + buf[4] +
		buf[6] + buf[7] + buf[8] + buf[9] +
		buf[10] + buf[11] + buf[12] + buf[13]) + 1;
	buf[5] = cod_chksum;

	return cod_chksum;
}

int ct36x_chip_get_binchksum(unsigned char *buf)
{
	//int ret = -1;
	int sec, cod;
	unsigned char cod_chksum;
	unsigned int fin_chksum = 0;
	unsigned int flash_addr;

	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// 8 sectors, 2048 bytes per sectors
	for (sec = 0; sec < CT36X_CHIP_FLASH_SECTOR_NUM; sec++) {
		flash_addr = sec * CT36X_CHIP_FLASH_SECTOR_SIZE;
		// 256 segments, 8 bytes per segment
		for (cod = 0; cod < (CT36X_CHIP_FLASH_SECTOR_SIZE / CT36X_CHIP_FLASH_SOURCE_SIZE); cod++) {
			// Fill binary data
			cod_chksum = ct36x_chip_set_code(flash_addr, buf);
			fin_chksum += cod_chksum;

			// Increase flash address 8bytes for each write command
			flash_addr += CT36X_CHIP_FLASH_SOURCE_SIZE;
		}
	}

	return (unsigned short)fin_chksum;
}

void ct36x_chip_set_adapter_off(struct i2c_client *client, unsigned char *buf)
{
	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0xFF;
	buf[1] = 0x0F;
	buf[2] = 0xFF;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(3);

	buf[0] = 0x00;
	buf[1] = 0xE2;
	ct36x_ts_reg_write(client, client->addr, buf, 2);
	mdelay(3);
}

void ct36x_chip_set_adapter_on(struct i2c_client *client, unsigned char *buf)
{
	if ( CT36X_TS_CHIP_DEBUG )
	printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0xFF;
	buf[1] = 0x0F;
	buf[2] = 0xFF;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(3);

	buf[0] = 0x00;
	buf[1] = 0xE3;
	ct36x_ts_reg_write(client, client->addr, buf, 2);
	mdelay(3);
}

int ct36x_chip_get_fwchksum(struct i2c_client *client, unsigned char *buf)
{
	int fwchksum = 0x00;

	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0xFF;
	buf[1] = 0x0F;
	buf[2] = 0xFF;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(20);

	buf[0] = 0x00;
	buf[1] = 0xE1;
	ct36x_ts_reg_write(client, client->addr, buf, 2);
	mdelay(500);

	buf[0] = 0xFF;
	buf[1] = 0x0A;
	buf[2] = 0x0D;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(20);

	ct36x_chip_rst_offset(client, buf);

	ct36x_ts_reg_read(client, client->addr, buf, 3);
	mdelay(20);

	fwchksum = ((buf[0] << 8) | buf[1]);

	return fwchksum;
}

void ct36x_chip_go_sleep(struct i2c_client *client, unsigned char *buf)
{
	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	buf[0] = 0xFF;
	buf[1] = 0x0F;
	buf[2] = 0x2B;
	ct36x_ts_reg_write(client, client->addr, buf, 3);
	mdelay(3);

	buf[0] = 0x00;
	buf[1] = 0x00;
	ct36x_ts_reg_write(client, client->addr, buf, 2);
	mdelay(3);

	//mdelay(50);
}


int ct36x_platform_get_resource(struct ct36x_ts_info *ts)
{
	gpio_direction_input(ts->ss);
	gpio_set_debounce(ts->ss, 0xa);
	return 0;
}

void ct36x_platform_put_resource(struct ct36x_ts_info *ts)
{
	gpio_free(ts->rst);
	gpio_free(ts->ss);
}

void ct36x_platform_hw_reset(struct ct36x_ts_info *ts)
{
	int resetpin = ts->rst;
	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	gpio_direction_output(resetpin, GPIO_LOW);
	gpio_set_value(resetpin, GPIO_LOW);
	mdelay(80);
	gpio_set_value(resetpin, GPIO_LOW);
	mdelay(10);
	gpio_direction_input(resetpin);
	mdelay(40);
}

/******************************************************************************
* Private functions
*/
static int ct36x_chip_get_busstatus(struct i2c_client *client, unsigned char *buf)
{
	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ct36x_ts_reg_read(client, 0x7F, buf, 1);
	mdelay(10);

	return buf[0];
}

static int ct36x_chip_erase_flash(struct i2c_client *client, unsigned char *buf)
{
	int ret = -1;
	int sec;

	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// Erase 16k flash
	for (sec = 0; sec < CT36X_CHIP_FLASH_SECTOR_NUM; sec++) {
		buf[0] = 0x00;
		buf[1] = 0x33;
		buf[2] = sec * 8;
		ct36x_ts_reg_write(client, 0x7F, buf, 3);
		mdelay(100);

		// Reset I2C offset address
		ct36x_chip_rst_offset(client, buf);

		// Read I2C bus status
		ret = ct36x_chip_get_busstatus(client, buf);
		if (ret != 0xAA) {
			return -1;
		}
	}

	return 0;
}

/*
** Prepare code segment
*/
static int ct36x_chip_write_firmware(struct i2c_client *client, unsigned char *buf)
{
	//int ret = -1;
	int sec, cod;
	unsigned char cod_chksum;
	unsigned int fin_chksum;
	unsigned int flash_addr;

	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// Code checksum, final checksum
	cod_chksum = 0x00; fin_chksum = 0x00;

	// Flash write command
	buf[0] = 0x00;
	buf[1] = 0x55;

	// 8 sectors, 2048 bytes per sectors
	for (sec = 0; sec < CT36X_CHIP_FLASH_SECTOR_NUM; sec++) {
		flash_addr = sec * CT36X_CHIP_FLASH_SECTOR_SIZE;
		// 256 segments, 8 bytes per segment
		for (cod = 0; cod < (CT36X_CHIP_FLASH_SECTOR_SIZE / CT36X_CHIP_FLASH_SOURCE_SIZE); cod++) {
			// Fill binary data
			cod_chksum = ct36x_chip_set_code(flash_addr, buf);
			fin_chksum += cod_chksum;

			// Write firmware source data
			ct36x_ts_reg_write(client, 0x7F, buf, 14);

			//
			mdelay(1);

			// Increase flash address 8bytes for each write command
			flash_addr += CT36X_CHIP_FLASH_SOURCE_SIZE;
		}
		//
		mdelay(20);
	}

	return 0;
}

int ct36x_chip_go_bootloader(struct i2c_client *client, unsigned char *buf)
{
	int ret = -1;

	if (CT36X_TS_CHIP_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// Init bootloader
	ct36x_chip_set_idle(client, buf);

	// Reset I2C offset address
	ct36x_chip_rst_offset(client, buf);

	// Get I2C bus status
	ret = ct36x_chip_get_busstatus(client, buf);
	if (ret != 0xAA) {
		printk("I2C bus status: 0x%x.\n", ret);
		return -1;
	}

	// Erase flash
	ret = ct36x_chip_erase_flash(client, buf);
	if (ret) {
		printk("Erase flash failed.\n");
		return -1;
	}

	// Write source data
	ct36x_chip_write_firmware(client, buf);

	return 0;
}

// ****************************************************************************
// Function declaration
// ****************************************************************************
char ct36x_cmd_list_cmd[] = { 'i', 'r', 'v', 'c', 'u', 'b', 'k', 0, };

static int ct36x_ts_cmd(char *cmdlist, const char cmd)
{
	int i = 0;

	// search cmd
	while (cmdlist[i]) {
		if (cmd == cmdlist[i])
			return ct36x_cmd_list_ind[i];
		i++;
	}

	return -1;
}

static int ct36x_ts_open(struct inode *inode, struct file *file)
{
	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	return 0;
}

static int ct36x_ts_close(struct inode *inode, struct file *file)
{
	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	return 0;
}

static ssize_t ct36x_ts_write(struct file *file, const char __user *buffer, size_t count, loff_t *offset)
{
	int cmd = 0;
	int rslt = 0;

	if (CT36X_TS_CORE_DEBUG) {
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
	}

	/* search cmd */
	cmd = ct36x_ts_cmd(ct36x_cmd_list_cmd, buffer[0]);

	/* execute cmd */
	if (ct36x_ts.state == CT36X_STATE_NORMAL)
		switch (cmd) {
		case CT36X_TS_CHIP_ID:
			break;

		case CT36X_TS_CHIP_RESET:
			printk("%s(): CT36X_TS_CHIP_RESET\n", __FUNCTION__);
			ct36x_platform_hw_reset(&ct36x_ts);
			break;

		case CT36X_TS_FW_VER:
			break;

		case CT36X_TS_FW_CHKSUM:
			printk("%s(): CT36X_TS_FW_CHKSUM\n", __FUNCTION__);
			rslt = ct36x_chip_get_fwchksum(ct36x_ts.client, ct36x_ts.data.buf);
			printk("%s(): Fw checksum: 0x%x\n", __FUNCTION__, rslt);
			break;

		case CT36X_TS_FW_UPDATE:
			printk("%s(): CT36X_TS_FW_UPDATE\n", __FUNCTION__);
			ct36x_chip_go_bootloader(ct36x_ts.client, ct36x_ts.data.buf);
			break;

		case CT36X_TS_BIN_VER:
			break;

		case CT36X_TS_BIN_CHKSUM:
			printk("%s(): CT36X_TS_BIN_CHKSUM\n", __FUNCTION__);
			rslt = ct36x_chip_get_binchksum(ct36x_ts.data.buf);
			printk("%s(): bin checksum: 0x%x\n", __FUNCTION__, rslt);
			break;

		default:
			printk("%s(): No such command (0x%x). \n", __FUNCTION__, buffer[0]);
			break;
		}


	return count;
}

static ssize_t ct36x_ts_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	//9int err = -1;

	if (CT36X_TS_CORE_DEBUG) {
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);
		printk("%s(): count=0x%x \n", __FUNCTION__, count);
	}

	if (ct36x_ts.state == CT36X_STATE_NORMAL) {
		//ct36x_ts_reg_read(ct36x_ts->client, buf[0], buf+1, buf[]);
	}

	return count;
}

static struct file_operations ct36x_ts_fops = {
	.owner = THIS_MODULE,
	.open = ct36x_ts_open,
	.release = ct36x_ts_close,
	.write = ct36x_ts_write,
	.read = ct36x_ts_read,
};

static irqreturn_t ct36x_ts_irq(int irq, void *dev)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)dev;

	// touch device is ready??
	if (ts->state == CT36X_STATE_NORMAL) {
		// Disable ts interrupt
		disable_irq_nosync(ts->irq);

		queue_work(ts->workqueue, &ts->event_work);
	}

	return IRQ_HANDLED;
}

static void ct36x_ts_workfunc(struct work_struct *work)
{
	int iter;
	int sync;
	int x, y;
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = container_of(work, struct ct36x_ts_info, event_work);

	/* read touch points */
	ct36x_ts_reg_read(ts->client, ts->i2c_address, (char *)ts->data.pts, sizeof(struct ct36x_finger_info) * CT36X_TS_POINT_NUM);

	/* report points */
	sync = 0; ts->press = 0;
	for (iter = 0; iter < CT36X_TS_POINT_NUM; iter++) {
		if (ts->data.pts[iter].xhi != 0xFF && ts->data.pts[iter].yhi != 0xFF &&
			(ts->data.pts[iter].status == 1 || ts->data.pts[iter].status == 2)) {
#ifdef CONFIG_TOUCHSCREEN_CT36X_MISC_XY_SWAP
			x = (ts->data.pts[iter].yhi << 4) | (ts->data.pts[iter].ylo & 0xF);
			y = (ts->data.pts[iter].xhi << 4) | (ts->data.pts[iter].xlo & 0xF);
#else
			x = (ts->data.pts[iter].xhi << 4) | (ts->data.pts[iter].xlo & 0xF);
			y = (ts->data.pts[iter].yhi << 4) | (ts->data.pts[iter].ylo & 0xF);
#endif
#ifdef CONFIG_TOUCHSCREEN_CT36X_MISC_X_REVERSE
			x = CT36X_TS_ABS_X_MAX - x;
#endif
#ifdef CONFIG_TOUCHSCREEN_CT36X_MISC_Y_REVERSE
			y = CT36X_TS_ABS_Y_MAX - y;
#endif

			if (CT36X_TS_EVENT_DEBUG) {
				printk("ID:       %d\n", ts->data.pts[iter].id);
				printk("status:   %d\n", ts->data.pts[iter].status);
				printk("X Lo:     %d\n", ts->data.pts[iter].xlo);
				printk("Y Lo:     %d\n", ts->data.pts[iter].ylo);
				printk("X Hi:     %d\n", ts->data.pts[iter].xhi);
				printk("Y Hi:     %d\n", ts->data.pts[iter].yhi);
				printk("X:        %d\n", x);
				printk("Y:        %d\n", y);
			}

			input_mt_slot(ts->input, ts->data.pts[iter].id - 1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
			input_report_abs(ts->input, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 30);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 128);

			sync = 1;
			ts->press |= 0x01 << (ts->data.pts[iter].id - 1);
		}
	}

	ts->release &= ts->release ^ ts->press;
	for (iter = 0; iter < CT36X_TS_POINT_NUM; iter++) {
		if (ts->release & (0x01 << iter)) {
			input_mt_slot(ts->input, iter);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
			sync = 1;
		}
	}
	ts->release = ts->press;

	if (sync)
		input_sync(ts->input);

	// Enable ts interrupt
	enable_irq(ts->irq);

}

static void ct36x_ts_adapter(int state)
{
	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	if (ct36x_ts.state == CT36X_STATE_NORMAL) {
		if (state)
			ct36x_chip_set_adapter_on(ct36x_ts.client, ct36x_ts.data.buf);
		else
			ct36x_chip_set_adapter_off(ct36x_ts.client, ct36x_ts.data.buf);
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ct36x_early_suspend(struct early_suspend *handler)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = container_of(handler, struct ct36x_ts_info, early_suspend);

	ct36x_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void ct36x_early_resume(struct early_suspend *handler)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = container_of(handler, struct ct36x_ts_info, early_suspend);

	ct36x_ts_resume(ts->client);
}
#endif

int ct36x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = -1;
	int binchksum, fwchksum;
	int updcnt;
	struct ct36x_ts_info *ts;
	struct device *dev;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ct36x_ts.state = CT36X_STATE_INIT;

	dev = &client->dev;

	ct36x_ts.pdata = dev->platform_data;

	/* this code segment is platform dependent */
	if (ct36x_ts.pdata) {
		/* for platform data driver */
		ct36x_ts.client = client;

		ct36x_platform_get_cfg(&ct36x_ts);
		i2c_set_clientdata(client, &ct36x_ts);
		printk("%s: i2c_set_clientdata called \n", __FUNCTION__);
	}

	/* this code segment is platform dependent */
	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	/* Create Proc Entry File */
	ts->proc_entry = create_proc_entry(ct360_TS_NAME, 0666/*S_IFREG | S_IRUGO | S_IWUSR*/, NULL);
	if (ts->proc_entry == NULL) {
		dev_err(dev, "Failed creating proc dir entry file.\n");
	} else {
		ts->proc_entry->proc_fops = &ct36x_ts_fops;
	}

	/* register early suspend */
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ct36x_early_suspend;
	ts->early_suspend.resume = ct36x_early_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	/* Check I2C Functionality */
	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!err) {
		dev_err(dev, "Check I2C Functionality Failed.\n");
		goto ERR_I2C_CHK;
	}

	/* Request platform resources (gpio/interrupt pins) */
	err = ct36x_platform_get_resource(ts);
	if (err) {
		dev_err(dev, "Unable to request platform resource for device %s.\n", ct360_TS_NAME);
		goto ERR_PLAT_RSC;
	}

	if (gpio_request(ct36x_ts.rst, "ct360_init_gpio") < 0) {
		printk("ct360_init_gpio not available.................\n");
	}
	/* Hardware reset */
	ct36x_platform_hw_reset(ts);

	// Get binary Checksum
	binchksum = ct36x_chip_get_binchksum(ts->data.buf);
	if (CT36X_TS_CORE_DEBUG)
		printk("Bin checksum: 0x%x\n", binchksum);

	// Get firmware Checksum
	fwchksum = ct36x_chip_get_fwchksum(client, ts->data.buf);
	if (CT36X_TS_CORE_DEBUG)
		printk("Fw checksum: 0x%x\n", fwchksum);

	updcnt = 5;
	while (binchksum != fwchksum && updcnt--) {
		/* Update Firmware */
		ct36x_chip_go_bootloader(client, ts->data.buf);

		/* Hardware reset */
		ct36x_platform_hw_reset(ts);

		// Get firmware Checksum
		fwchksum = ct36x_chip_get_fwchksum(client, ts->data.buf);
		if (CT36X_TS_CORE_DEBUG)
			printk("Fw checksum: 0x%x\n", fwchksum);
	}

	printk("VTL CT360 - FW update %s. 0x%x, 0x%x\n", binchksum != fwchksum ? "Failed" : "Success", binchksum, fwchksum);

	/* Hardware reset */
	ct36x_platform_hw_reset(ts);

	/* allocate input device */
	ts->input = input_allocate_device();
	if (!ts->input) {
		dev_err(dev, "Unable to allocate input device for device %s.\n", ct360_TS_NAME);
		err = -ENOMEM;
		goto ERR_INPUT_ALLOC;
	}

	/* config input device */
	__set_bit(EV_SYN, ts->input->evbit);
	__set_bit(EV_KEY, ts->input->evbit);
	__set_bit(EV_ABS, ts->input->evbit);

	// For android 4.x only
	__set_bit(INPUT_PROP_DIRECT, ts->input->propbit);

	// For android 4.x only
	input_mt_init_slots(ts->input, CT36X_TS_POINT_NUM);

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, CT36X_TS_ABS_X_MAX, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, CT36X_TS_ABS_Y_MAX, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	ts->input->name = ct360_TS_NAME;
	ts->input->id.bustype =	BUS_I2C;

	/* register input device */
	err = input_register_device(ts->input);
	if (err) {
		dev_err(dev, "Unable to register input device for device %s.\n", ct360_TS_NAME);
		goto ERR_INPUT_REGIS;
	}

	/* Create work queue */
	INIT_WORK(&ts->event_work, ct36x_ts_workfunc);
	ts->workqueue = create_singlethread_workqueue(dev_name(&client->dev));

	if (CT36X_TS_CORE_DEBUG)
		printk("Workqueue pointer[0x%x]\n",(int)ts->workqueue);

	/* Init irq */
	/* this code segment is platform dependent */
	// irq request for generic platform (enabled when unsure)
	ts->irq = gpio_to_irq(client->irq);

	if (CT36X_TS_CORE_DEBUG)
		printk("Requesing irq[%d] gpiotoirq[%d]\n", client->irq, ts->irq );

	err = request_irq(ts->irq, ct36x_ts_irq, IRQF_TRIGGER_FALLING, ct360_TS_NAME, ts);

	if (err) {
		dev_err(dev, "Unable to request irq for device %s.\n", ct360_TS_NAME);
		goto ERR_IRQ_REQ;
	}

	/* Set device is ready */
	ts->state = CT36X_STATE_NORMAL;

	/* power denoisy*/
	ct36x_ts_adapter(1);

	return 0;

ERR_IRQ_REQ:
	destroy_workqueue(ts->workqueue);
ERR_INPUT_REGIS:
	input_free_device(ts->input);
ERR_INPUT_ALLOC:
ERR_PLAT_RSC:
	ct36x_platform_put_resource(ts);
ERR_I2C_CHK:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	remove_proc_entry(ct360_TS_NAME, NULL);
	return err;
}

void ct36x_ts_shutdown(struct i2c_client *client)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	ct36x_chip_go_sleep(client, ts->data.buf);
}

int ct36x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	if (ts->state == CT36X_STATE_NORMAL) {
		ts->state = CT36X_STATE_SLEEP;
		disable_irq(ts->irq);
		//cancel_work_sync(&ts->event_work);
		ct36x_chip_go_sleep(client, ts->data.buf);
	}

	return 0;
}

int ct36x_ts_resume(struct i2c_client *client)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	if (ts->state == CT36X_STATE_SLEEP) {
		/* Hardware reset */
		ct36x_platform_hw_reset(ts);
		enable_irq(ts->irq);
		ts->state = CT36X_STATE_NORMAL;
	}

	return 0;
}

int __devexit ct36x_ts_remove(struct i2c_client *client)
{
	struct ct36x_ts_info *ts;

	if (CT36X_TS_CORE_DEBUG)
		printk(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	/* Driver clean up */
	disable_irq(ts->irq);
	cancel_work_sync(&ts->event_work);
	destroy_workqueue(ts->workqueue);
	input_free_device(ts->input);
	free_irq(ts->irq, ts);
	ct36x_platform_put_resource(ts);
	i2c_unregister_device(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	remove_proc_entry(ct360_TS_NAME, NULL);

	return 0;
}

static struct i2c_driver ct360_ts_driver = {
	.probe      = ct36x_ts_probe,
	.remove     = ct36x_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = ct36x_ts_suspend,
	.resume     = ct36x_ts_resume,
#endif
	.id_table   = ct36x_ts_id,
	.driver = {
		.name   = ct360_TS_NAME,
	},
};

int __init ct36x_ts_init(void)
{
	int err = -1;

	printk("VTL ct36x TouchScreen driver, <micronet.co.il>.\n");

	ct36x_ts.state = CT36X_STATE_UNKNOWN;

	ct36x_platform_get_cfg(&ct36x_ts);

	err = i2c_add_driver(&ct360_ts_driver);
	if (err)
		goto ERR_INIT;

	return 0;

ERR_INIT:
	return err;
}

void __exit ct36x_ts_exit(void)
{
	i2c_del_driver(&ct360_ts_driver);
}

static const struct i2c_device_id ct360_ts_id[] = {
	{ ct360_TS_NAME, 0 },
	{ }
};

module_init(ct36x_ts_init);
module_exit(ct36x_ts_exit);

MODULE_DESCRIPTION("ct360 Touchscreen Driver");
MODULE_LICENSE("GPL");
