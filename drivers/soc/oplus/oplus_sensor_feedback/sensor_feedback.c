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
* Revision 1.0      2020/09/12       tangjh@PSW.BSP.Sensor      Created, sensor feadback
*******************************************************************/

#define pr_fmt(fmt) "<sensor_feedback>" fmt

#include <linux/init.h>
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/soc/qcom/smem.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "sensor_feedback.h"
#include <soc/oplus/system/kernel_fb.h>


static struct sensor_fb_cxt *g_sensor_fb_cxt = NULL;

/*fb_field :maxlen 19*/
struct sensor_fb_conf g_fb_conf[] = {
	{PS_INIT_FAIL_COUNT_ID, "ps_init_fail"},
	{PS_OPEN_COUNT_ID, "ps_open"},
	{PS_OPEN_TIME_ID, "ps_open_time"},
	{PS_POWER_RESUME_ID, "ps_power_resume"},
	{PS_WAKE_UP_COUNT_ID, "ps_wake_up"},
	{PS_I2C_ERR_COUNT_ID, "ps_i2c_err"},
	{PS_ALLOC_FAIL_COUNT_ID, "ps_alloc_fail"},
	{PS_ESD_REST_COUNT_ID, "ps_esd_reset"},
	{PS_NO_INTERRUPT_COUNT_ID, "ps_no_interrupt"},
	{PS_FIRST_REPORT_DELAY_COUNT_ID, "ps_rpt_delay"}, /*ps_first_reprot_delay*/
	{PS_POCKET_REPORT_FAR_COUNT_ID, "ps_rpt_far"}, /*ps_pocket_report_far*/
	{PS_ORIGIN_DATA_TO_ZERO_COUNT_ID, "ps_to_zero"},/*PS_ORIGIN_DATA_TO_ZERO_COUNT*/
	{ALS_INIT_FAIL_COUNT_ID, "als_init_fail"},
	{ALS_OPEN_COUNT_ID, "als_open"},
	{ALS_OPEN_TIME_ID, "als_open_time"},
	{ALS_POWER_RESUME_ID, "als_power_resume"},
	{ALS_WAKE_UP_COUNT_ID, "als_wake_up"},
	{ALS_I2C_ERR_COUNT_ID, "als_i2c_err"},
	{ALS_ALLOC_FAIL_COUNT_ID, "als_alloc_fail"},
	{ALS_ESD_REST_COUNT_ID, "als_esd_reset"},
	{ALS_NO_INTERRUPT_COUNT_ID, "als_no_interrupt"},
	{ALS_FIRST_REPORT_DELAY_COUNT_ID, "als_rpt_delay"}
};


static ssize_t adsp_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	uint16_t adsp_event_counts = 0;
	spin_lock(&sensor_fb_cxt->rw_lock);
	adsp_event_counts = sensor_fb_cxt->adsp_event_counts;
	spin_unlock(&sensor_fb_cxt->rw_lock);
	pr_info(" adsp_value = %d\n", adsp_event_counts);

	return snprintf(buf, PAGE_SIZE, "%d\n", adsp_event_counts);
}

static ssize_t adsp_notify_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	uint16_t adsp_event_counts = 0;
	uint16_t node_type = 0;
	int err = 0;

	err = sscanf(buf, "%hu %hu", &node_type, &adsp_event_counts);

	if (err < 0) {
		pr_err("adsp_notify_store error: err = %d\n", err);
		return err;
	}

	spin_lock(&sensor_fb_cxt->rw_lock);
	sensor_fb_cxt->adsp_event_counts = adsp_event_counts;
	sensor_fb_cxt->node_type = node_type;
	spin_unlock(&sensor_fb_cxt->rw_lock);
	pr_info("adsp_notify_store adsp_value = %d, node_type=%d\n", adsp_event_counts,
		node_type);

	set_bit(THREAD_WAKEUP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
	/*wake_up_interruptible(&sensor_fb_cxt->wq);*/
	wake_up(&sensor_fb_cxt->wq);

	return count;
}


static ssize_t hal_info_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t hal_info_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	pr_info("hal_info_store count = %d\n", count);
	return count;
}

static ssize_t test_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_info("test_id_show\n");
	return 0;
}

static ssize_t test_id_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	uint16_t adsp_event_counts = 0;
	uint16_t node_type = 0;
	uint16_t event_id = 0;
	uint16_t event_data = 0;
	int err = 0;

	err = sscanf(buf, "%hu %hu %hu %hu", &node_type, &adsp_event_counts, &event_id,
			&event_data);

	if (err < 0) {
		pr_err("test_id_store error: err = %d\n", err);
		return err;
	}

	spin_lock(&sensor_fb_cxt->rw_lock);
	sensor_fb_cxt->adsp_event_counts = adsp_event_counts;
	sensor_fb_cxt->node_type = node_type;
	spin_unlock(&sensor_fb_cxt->rw_lock);

	sensor_fb_cxt->fb_smem.event[0].event_id = event_id;
	sensor_fb_cxt->fb_smem.event[0].count = event_data;

	pr_info("test_id_store adsp_value = %d, node_type=%d \n", adsp_event_counts,
		node_type);
	pr_info("test_id_store event_id = %d, event_data=%d \n", event_id, event_data);


	set_bit(THREAD_WAKEUP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
	/*wake_up_interruptible(&sensor_fb_cxt->wq);*/
	wake_up(&sensor_fb_cxt->wq);

	return count;
}


DEVICE_ATTR(adsp_notify, 0644, adsp_notify_show, adsp_notify_store);
DEVICE_ATTR(hal_info, 0644, hal_info_show, hal_info_store);
DEVICE_ATTR(test_id, 0644, test_id_show, test_id_store);



static struct attribute *sensor_feedback_attributes[] = {
	&dev_attr_adsp_notify.attr,
	&dev_attr_hal_info.attr,
	&dev_attr_test_id.attr,
	NULL
};



static struct attribute_group sensor_feedback_attribute_group = {
	.attrs = sensor_feedback_attributes
};

#define SMEM_SENSOR_FEEDBACK (128)
static int read_data_from_share_mem(struct sensor_fb_cxt *sensor_fb_cxt)
{
	int ret = 0;
	size_t smem_size = 0;
	void *smem_addr = NULL;
	struct fb_event_smem *fb_event = NULL;
	smem_addr = qcom_smem_get(QCOM_SMEM_HOST_ANY,
			SMEM_SENSOR_FEEDBACK,
			&smem_size);

	if (IS_ERR(smem_addr)) {
		pr_err("unable to acquire smem SMEM_SENSOR_FEEDBACK entry\n");
		return -1;
	}

	fb_event = (struct fb_event_smem *)smem_addr;

	if (fb_event == ERR_PTR(-EPROBE_DEFER)) {
		fb_event = NULL;
		return -2;
	}

	memcpy((void *)&sensor_fb_cxt->fb_smem, (void *)fb_event, smem_size);
	return ret;
}


static int find_event_id(int16_t event_id)
{
	int len = sizeof(g_fb_conf) / sizeof(g_fb_conf[0]);
	int ret = -1;
	int index = 0;

	for (index = 0; index < len; index++) {
		if (g_fb_conf[index].event_id == event_id) {
			ret = index;
		}
	}

	return ret;
}
/*
static unsigned int BKDRHash(char *str, unsigned int len)
{
    unsigned int seed = 131;
        // 31 131 1313 13131 131313 etc..
    unsigned int hash = 0;
    unsigned int i    = 0;

    if (str == NULL) {
        return 0;
    }

    for(i = 0; i < len; str++, i++) {
        hash = (hash * seed) + (*str);
    }

    return hash;
}*/


static int parse_shr_info(struct sensor_fb_cxt *sensor_fb_cxt)
{
	int ret = 0;
	int count = 0;
	uint16_t event_id = 0;
	int index = 0;
	unsigned char payload[1024] = {0x00};
	int fb_len = 0;
	unsigned char detail_buff[256] = {0x00};

	for (count = 0; count < sensor_fb_cxt->adsp_event_counts; count ++) {
		event_id = sensor_fb_cxt->fb_smem.event[count].event_id;
		pr_info("event_id =%d, count =%d\n", event_id, count);

		index = find_event_id(event_id);

		if (index == -1) {
			pr_info("event_id =%d, count =%d\n", event_id, count);
			continue;
		}

		memset(payload, 0, sizeof(payload));
		memset(detail_buff, 0, sizeof(detail_buff));
		snprintf(detail_buff, sizeof(detail_buff), "%d %d %d",
			sensor_fb_cxt->fb_smem.event[count].buff[0],
			sensor_fb_cxt->fb_smem.event[count].buff[1],
			sensor_fb_cxt->fb_smem.event[count].buff[2]);
		fb_len += scnprintf(payload, sizeof(payload),
				"NULL$$EventID@@%d$$EventData@@%d$$PackageName@@%s$$detailDatas@@%s",
				event_id,
				sensor_fb_cxt->fb_smem.event[count].count,
				g_fb_conf[index].fb_field,
				detail_buff);
		pr_info("payload =%s\n", payload);
		oplus_kevent_fb(FB_SENSOR, g_fb_conf[index].fb_field, payload);
	}

	return ret;
}


static int sensor_report_thread(void *arg)
{
	int ret = 0;
	struct sensor_fb_cxt *sensor_fb_cxt = (struct sensor_fb_cxt *)arg;
	pr_info("sensor_feedback: sensor_report_thread step1!\n");

	while (!kthread_should_stop()) {
		pr_info("sensor_feedback: sensor_report_thread step2!\n");
		wait_event_interruptible(sensor_fb_cxt->wq, test_bit(THREAD_WAKEUP,
				(unsigned long *)&sensor_fb_cxt->wakeup_flag));

		clear_bit(THREAD_WAKEUP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
		set_bit(THREAD_SLEEP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);
		pr_info("sensor_feedback: sensor_report_thread step3!\n");

		if (sensor_fb_cxt->node_type == 0) {
			ret = read_data_from_share_mem(sensor_fb_cxt);

		} else {
			pr_info("sensor_feedback test from node \n");
		}

		ret = parse_shr_info(sensor_fb_cxt);
	}

	memset((void *)&sensor_fb_cxt->fb_smem, 0, sizeof(struct fb_event_smem));
	pr_info("sensor_feedback ret =%s\n", ret);
	return ret;
}


static int sensor_feedback_probe(struct platform_device *pdev)
{
	int err = 0;
	struct sensor_fb_cxt *sensor_fb_cxt = NULL;

	sensor_fb_cxt = kzalloc(sizeof(struct sensor_fb_cxt), GFP_KERNEL);

	if (sensor_fb_cxt == NULL) {
		pr_err("kzalloc g_sensor_fb_cxt failed\n");
		err = -ENOMEM;
		goto alloc_sensor_fb_failed;
	}

	g_sensor_fb_cxt = sensor_fb_cxt;


	spin_lock_init(&sensor_fb_cxt->rw_lock);
	init_waitqueue_head(&sensor_fb_cxt->wq);

	sensor_fb_cxt->sensor_fb_dev = pdev;
	err = sysfs_create_group(&sensor_fb_cxt->sensor_fb_dev->dev.kobj,
			&sensor_feedback_attribute_group);

	if (err < 0) {
		pr_err("unable to create sensor_feedback_attribute_group file err=%d\n", err);
		goto sysfs_create_failed;
	}

	kobject_uevent(&sensor_fb_cxt->sensor_fb_dev->dev.kobj, KOBJ_ADD);

	init_waitqueue_head(&sensor_fb_cxt->wq);

	set_bit(THREAD_SLEEP, (unsigned long *)&sensor_fb_cxt->wakeup_flag);

	sensor_fb_cxt->report_task = kthread_create(sensor_report_thread,
			(void *)sensor_fb_cxt,
			"sensor_feedback_task");

	if (IS_ERR(sensor_fb_cxt->report_task)) {
		err = PTR_ERR(sensor_fb_cxt->report_task);
		goto create_task_failed;
	}

	platform_set_drvdata(pdev, sensor_fb_cxt);
	wake_up_process(sensor_fb_cxt->report_task);

	pr_info("sensor_feedback_init success\n");
	return 0;
create_task_failed:
	sysfs_remove_group(&sensor_fb_cxt->sensor_fb_dev->dev.kobj,
		&sensor_feedback_attribute_group);
sysfs_create_failed:
	kfree(sensor_fb_cxt);
	g_sensor_fb_cxt = NULL;
alloc_sensor_fb_failed:
	return err;
}


static int sensor_feedback_remove(struct platform_device *pdev)
{
	struct sensor_fb_cxt *sensor_fb_cxt = g_sensor_fb_cxt;
	sysfs_remove_group(&sensor_fb_cxt->sensor_fb_dev->dev.kobj,
		&sensor_feedback_attribute_group);
	kfree(sensor_fb_cxt);
	g_sensor_fb_cxt = NULL;
	return 0;
}

static const struct of_device_id of_drv_match[] = {
	{ .compatible = "oplus,sensor-feedback"},
	{},
};
MODULE_DEVICE_TABLE(of, of_drv_match);

static struct platform_driver _driver = {
	.probe      = sensor_feedback_probe,
	.remove     = sensor_feedback_remove,
	.driver     = {
		.name       = "sensor_feedback",
		.of_match_table = of_drv_match,
	},
};

static int __init sensor_feedback_init(void)
{
	pr_info("oppo_devinfo_init call\n");

	platform_driver_register(&_driver);
	return 0;
}

core_initcall(sensor_feedback_init);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("JiangHua.Tang");

