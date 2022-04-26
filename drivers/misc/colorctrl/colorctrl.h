/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef H_COLORCTRL
#define H_COLORCTRL

#include <linux/printk.h>
#include <linux/thermal.h>

#define COLOR_INFO(fmt, args...) \
    pr_info("COLOR-CTRL: %s:" fmt "\n", __func__, ##args)

#define GPIO_HIGH (1)
#define GPIO_LOW  (0)
#define PAGESIZE  512
#define UV_PER_MV 1000
#define MAX_PARAMETER 50
#define NAME_TAG_SIZE 50
#define MAX_CTRL_TYPE 8

enum color_ctrl_write_type {
    LIGHT_BLUE_FTM,
    BLUE_FTM,
    TRANSPARENT_FTM,
    BLUE_NORMAL,
    TRANSPARENT_NORMAL,
    RESET,
    RESET_FTM,
    STOP_RECHARGE,
    OPEN_RECHARGE,
};

typedef enum color_ctrl_type {
    LIGHT_BLUE,
    BLUE,
    TRANSPARENT,
    OPEN_CIRCUIT,
    SHORT_CIRCUIT,
    UNKNOWN = 0xFF,
} color_status;

typedef enum color_ctrl_temp_type {
    LOW_TEMP,
    ROOM_TEMP,
    HIGH_TEMP,
    ABNORMAL_TEMP,
} temp_status;

struct color_ctrl_hw_resource {
    unsigned int        sleep_en_gpio;
    unsigned int        si_in_1_gpio;
    unsigned int        si_in_2_gpio;
    unsigned int        vm_enable_gpio;
    struct iio_channel  *vm_v_chan;
    struct regulator    *vm;        /*driver power*/
};

struct color_ctrl_control_para {
    unsigned int        low_temp_charge_time;
    unsigned int        low_temp_charge_vol;
    unsigned int        normal_temp_charge_time;
    unsigned int        normal_temp_charge_vol;
    unsigned int        high_temp_charge_time;
    unsigned int        high_temp_charge_vol;
    unsigned int        low_temp_recharge_time_1;
    unsigned int        low_temp_recharge_vol_1;
    unsigned int        normal_temp_recharge_time_1;
    unsigned int        normal_temp_recharge_vol_1;
    unsigned int        high_temp_recharge_time_1;
    unsigned int        high_temp_recharge_vol_1;
    unsigned int        low_temp_recharge_time_2;
    unsigned int        low_temp_recharge_vol_2;
    unsigned int        normal_temp_recharge_time_2;
    unsigned int        normal_temp_recharge_vol_2;
    unsigned int        high_temp_recharge_time_2;
    unsigned int        high_temp_recharge_vol_2;
    unsigned int        recharge_vol_thd_1;
    unsigned int        recharge_vol_thd_2;
    unsigned int        recharge_vol_thd_3;
};

struct color_ctrl_device {
    struct platform_device          *pdev;
    struct device                   *dev;
    struct mutex                    rw_lock;
    struct proc_dir_entry           *prEntry_cr;
    struct color_ctrl_hw_resource   *hw_res;
    struct thermal_zone_device      *thermal_zone_device;
    struct hrtimer                  hrtimer;
    struct work_struct              recharge_work;
    struct workqueue_struct         *recharge_wq;
    color_status                    color_status;
    temp_status                     temp_status;
    int                             platform_support_project[10];
    int                             project_num;
    unsigned int                    intermediate_wait_time;
    bool                            need_recharge;
    unsigned int                    recharge_time;
    unsigned int                    low_temp_low_thd;
    unsigned int                    low_temp_high_thd;
    unsigned int                    normal_temp_low_thd;
    unsigned int                    normal_temp_high_thd;
    unsigned int                    high_temp_low_thd;
    unsigned int                    high_temp_high_thd;
    unsigned int                    open_circuit_thd;
    unsigned int                    blue_short_circuit_thd;
    unsigned int                    transparent_short_circuit_thd;
    struct color_ctrl_control_para  *blue_control_para;
    struct color_ctrl_control_para  *transparent_control_para;
};

#endif
