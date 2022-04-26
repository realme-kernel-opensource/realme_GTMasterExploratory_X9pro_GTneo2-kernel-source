/** Copyright (C), 2004-2017, OPPO Mobile Comm Corp., Ltd.
** OPLUS_FEATURE_SENSOR_DRIVER
** File: - sensor_feedback.c
** Description: Source file for sensor feedback.
** Version: 1.0
** Date : 2020/09/12
** Author: tangjh@PSW.BSP.Sensor
**
** --------------------------- Revision History: ---------------------
* <version> <date>      <author>                    <desc>
* Revision 1.0      2020/09/12       tangjh@PSW.BSP.Sensor      Created, sensor feedback
*******************************************************************/

#ifndef __SENSOR_FEEDBACK_H__
#define __SENSOR_FEEDBACK_H__

#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#ifdef CONFIG_ARM
#include <linux/sched.h>
#else
#include <linux/wait.h>
#endif
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/param.h>


#define THREAD_WAKEUP  0
#define THREAD_SLEEP   1



enum sensor_fb_event_id {
	/*1~100*/
	PS_INIT_FAIL_COUNT_ID = 1,
	PS_OPEN_COUNT_ID,
	PS_OPEN_TIME_ID,
	PS_POWER_RESUME_ID,
	PS_WAKE_UP_COUNT_ID,
	PS_I2C_ERR_COUNT_ID,
	PS_ALLOC_FAIL_COUNT_ID,
	PS_ESD_REST_COUNT_ID,
	PS_NO_INTERRUPT_COUNT_ID,
	PS_FIRST_REPORT_DELAY_COUNT_ID,
	PS_POCKET_REPORT_FAR_COUNT_ID,
	PS_ORIGIN_DATA_TO_ZERO_COUNT_ID,

	/*100~200*/
	ALS_INIT_FAIL_COUNT_ID = 100,
	ALS_OPEN_COUNT_ID,
	ALS_OPEN_TIME_ID,
	ALS_POWER_RESUME_ID,
	ALS_WAKE_UP_COUNT_ID,
	ALS_I2C_ERR_COUNT_ID,
	ALS_ALLOC_FAIL_COUNT_ID,
	ALS_ESD_REST_COUNT_ID,
	ALS_NO_INTERRUPT_COUNT_ID,
	ALS_FIRST_REPORT_DELAY_COUNT_ID,

	/*200~300*/
	ACCEL_INIT_FAIL_COUNT_ID = 200,
	ACCEL_OPEN_COUNT_ID,
};


struct fd_data {
	int data_x;
	int data_y;
	int data_z;
};

#define EVNET_DATA_LEN 3
struct fb_event {
	unsigned short event_id;
	unsigned int count;
	union {
		int buff[EVNET_DATA_LEN];
		struct fd_data data;
	};
};


#define EVNET_NUM_MAX 109
struct fb_event_smem {
	struct fb_event event[EVNET_NUM_MAX];
};


struct sensor_fb_conf {
	uint16_t event_id;
	char *fb_field;
};


struct sensor_fb_cxt {
	/*struct miscdevice sensor_fb_dev;*/
	struct platform_device *sensor_fb_dev;
	spinlock_t   rw_lock;
	wait_queue_head_t wq;
	struct task_struct *report_task; /*kernel thread*/
	uint16_t adsp_event_counts;
	struct fb_event_smem fb_smem;
	uint16_t node_type;
	unsigned long wakeup_flag;
};
#endif /*__SENSOR_FEEDBACK_H__*/

