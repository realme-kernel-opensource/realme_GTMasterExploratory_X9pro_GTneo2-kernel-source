// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>
#include <soc/oplus/system/oplus_project.h>
#include "colorctrl.h"

#define DRIVER_NAME "color-ctrl"

static void colorctrl_blue_recharge_operation(struct color_ctrl_device *cd);
static void colorctrl_transparent_recharge_operation(struct color_ctrl_device *cd);

static const struct of_device_id colorctrl_match_table[] = {
    {.compatible = "oplus,color-ctrl"},
    {}
};

static void colorctrl_msleep(int time)
{
    COLOR_INFO("wait for time : %d(ms).", time);
    msleep(time);
}

static int colorctrl_power_control(struct color_ctrl_hw_resource *hw_res, int vol, bool on)
{
    int ret = 0;

    if (on) {
        if (!IS_ERR_OR_NULL(hw_res->vm)) {
            COLOR_INFO("enable the vm power to voltage : %d(mV).", vol);
            if (vol) {
                ret = regulator_set_voltage(hw_res->vm, vol * UV_PER_MV, vol * UV_PER_MV);
                if (ret) {
                    COLOR_INFO("Regulator vm set voltage failed, ret = %d", ret);
                    return ret;
                }
            }
            ret = regulator_enable(hw_res->vm);
            if (ret) {
                COLOR_INFO("Regulator vm enable failed, ret = %d", ret);
                return ret;
            }
            if (gpio_is_valid(hw_res->vm_enable_gpio)) {
                COLOR_INFO("enable vm_enable_gpio.");
                ret = gpio_direction_output(hw_res->vm_enable_gpio, GPIO_HIGH);
                if(ret) {
                    COLOR_INFO("enable vm_enable_gpio fail.");
                    return ret;
                }
            }
        }
    } else {
        if (!IS_ERR_OR_NULL(hw_res->vm)) {
            if (gpio_is_valid(hw_res->vm_enable_gpio)) {
                COLOR_INFO("disable vm_enable_gpio.");
                ret = gpio_direction_output(hw_res->vm_enable_gpio, GPIO_LOW);
                if(ret) {
                    COLOR_INFO("disable vm_enable_gpio fail.");
                    return ret;
                }
            }
            COLOR_INFO("disable the vm power.");
            ret = regulator_disable(hw_res->vm);
            if (ret) {
                COLOR_INFO("Regulator vm disable failed, ret = %d", ret);
                return ret;
            }
        }
    }

    return ret;
}

static void colorctrl_recharge_work(struct work_struct *work)
{
    struct color_ctrl_device *cd = container_of(work, struct color_ctrl_device, recharge_work);

    if (!cd) {
        COLOR_INFO("no dev find, return.");
        return;
    }
    COLOR_INFO("is call.");

    if (!cd->need_recharge) {
        COLOR_INFO("not need to do the recharge work due to force close.");
        return;
    }

    mutex_lock(&cd->rw_lock);

    if (cd->color_status == BLUE) {
        colorctrl_blue_recharge_operation(cd);
    } else if (cd->color_status == TRANSPARENT) {
        colorctrl_transparent_recharge_operation(cd);
    } else {
        COLOR_INFO("not need to do the recharge work, color_status : %d.", cd->color_status);
    }

    mutex_unlock(&cd->rw_lock);
}

static void colorctrl_reset_hrtimer(struct color_ctrl_device *cd)
{
    if (!cd) {
        COLOR_INFO("no dev find, return.");
        return;
    }

    COLOR_INFO("reset hrtimer expires time : %d(s).", cd->recharge_time);
    hrtimer_cancel(&cd->hrtimer);
    hrtimer_start(&cd->hrtimer, ktime_set(cd->recharge_time, 0), HRTIMER_MODE_REL);
}

static enum hrtimer_restart colorctrl_hrtimer_handler(struct hrtimer *timer)
{
    struct color_ctrl_device *cd = container_of(timer, struct color_ctrl_device, hrtimer);

    if (!cd) {
        COLOR_INFO("no dev find, return.");
        return HRTIMER_NORESTART;
    }

    COLOR_INFO("is call.");

    queue_work(cd->recharge_wq, &cd->recharge_work);
    //hrtimer_forward(timer, timer->base->get_time(), ktime_set(cd->recharge_time, 0));

    return HRTIMER_NORESTART;
}

static void colorctrl_update_temperature_status(struct color_ctrl_device *cd)
{
    int ret = 0, temp = 0;

    if (!cd || !cd->thermal_zone_device) {
        COLOR_INFO("no dev or resources find, return.");
        return;
    }

    ret = thermal_zone_get_temp(cd->thermal_zone_device, &temp);
    if (ret) {
        COLOR_INFO("fail to get shell_back temperature: %d", ret);
    } else {
        COLOR_INFO("current shell back temperature is : %d", temp);
    }

    if (temp <= cd->low_temp_high_thd && temp > cd->low_temp_low_thd) {
        COLOR_INFO("it's low temperature now");
        cd->temp_status = LOW_TEMP;
    } else if (temp <= cd->normal_temp_high_thd && temp > cd->normal_temp_low_thd) {
        COLOR_INFO("it's normal temperature now");
        cd->temp_status = ROOM_TEMP;
    } else if (temp < cd->high_temp_high_thd && temp > cd->high_temp_low_thd) {
        COLOR_INFO("it's high temperature now");
        cd->temp_status = HIGH_TEMP;
    } else {
        COLOR_INFO("it's abnormal temperature now");
        cd->temp_status = ABNORMAL_TEMP;
    }
}

static void colorctrl_blue_recharge_operation(struct color_ctrl_device *cd)
{
    struct color_ctrl_hw_resource *hw_res = NULL;
    int ret = 0, vm_volt = 0, i = 0;
    int charge_time = 0, recharge_time_1 = 0, recharge_time_2 = 0;
    int charge_volt = 0, recharge_volt_1 = 0, recharge_volt_2 = 0;
    struct color_ctrl_control_para *para = NULL;

    if (!cd || !cd->hw_res || !cd->blue_control_para) {
        COLOR_INFO("no dev or resources find, return.");
        return;
    }

    colorctrl_update_temperature_status(cd);
    if (cd->temp_status == ABNORMAL_TEMP) {
        COLOR_INFO("abnormal temperature occur, can not do recharge operation");
        return;
    }

    hw_res = cd->hw_res;
    para = cd->blue_control_para;

    switch (cd->temp_status) {
    case LOW_TEMP : {
        charge_volt = para->low_temp_charge_vol;
        recharge_volt_1 = para->low_temp_recharge_vol_1;
        recharge_volt_2 = para->low_temp_recharge_vol_2;
        charge_time = para->low_temp_charge_time;
        recharge_time_1 = para->low_temp_recharge_time_1;
        recharge_time_2 = para->low_temp_recharge_time_2;
        break;
    }
    case ROOM_TEMP : {
        charge_volt = para->normal_temp_charge_vol;
        recharge_volt_1 = para->normal_temp_recharge_vol_1;
        recharge_volt_2 = para->normal_temp_recharge_vol_2;
        charge_time = para->normal_temp_charge_time;
        recharge_time_1 = para->normal_temp_recharge_time_1;
        recharge_time_2 = para->normal_temp_recharge_time_2;
        break;
    }
    case HIGH_TEMP : {
        charge_volt = para->high_temp_charge_vol;
        recharge_volt_1 = para->high_temp_recharge_vol_1;
        recharge_volt_2 = para->high_temp_recharge_vol_2;
        charge_time = para->high_temp_charge_time;
        recharge_time_1 = para->high_temp_recharge_time_1;
        recharge_time_2 = para->high_temp_recharge_time_2;
        break;
    }
    case ABNORMAL_TEMP : {
        COLOR_INFO("abnormal temperature occur, do not change color.");
        ret = -1;
        goto OUT;
        break;
    }
    default :
        break;
    }

    ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_HIGH);
    ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
    ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
    if (ret) {
        COLOR_INFO("config gpio status failed.");
        goto OUT;
    }
    msleep(2);
    ret = iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
    if (ret < 0) {
        COLOR_INFO("iio_read_channel_processed get error ret = %d", ret);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
        goto OUT;
    }
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    vm_volt = vm_volt / UV_PER_MV;
    COLOR_INFO("current volt : %d(mV).", vm_volt);

    for (i = 0; i < 3; i++) {
        if (vm_volt < para->recharge_vol_thd_1) {
            COLOR_INFO("volt is too low, try to do normal charging.");
            ret = colorctrl_power_control(cd->hw_res, charge_volt, true);
            if (ret) {
                COLOR_INFO("enable power failed.");
                goto OUT;
            }
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(charge_time);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_1 && vm_volt < para->recharge_vol_thd_2) {
            COLOR_INFO("volt is too low, try to do recharging(1).");
            colorctrl_power_control(cd->hw_res, recharge_volt_1, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(recharge_time_1);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_2 && vm_volt < para->recharge_vol_thd_3) {
            COLOR_INFO("volt is too low, try to do recharging(2).");
            colorctrl_power_control(cd->hw_res, recharge_volt_2, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(recharge_time_2);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_3) {
            break;
        }

        msleep(2000);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
        msleep(2);
        iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
        vm_volt = vm_volt / UV_PER_MV;
        COLOR_INFO("volt : %d(mV), rety %d times.", vm_volt, i + 1);
    }

    if (vm_volt < cd->open_circuit_thd) {
        cd->color_status = OPEN_CIRCUIT;
        COLOR_INFO("open circuit fault detected.");
        ret = -1;
    } else if (vm_volt >= cd->open_circuit_thd && vm_volt < cd->blue_short_circuit_thd) {
        cd->color_status = SHORT_CIRCUIT;
        COLOR_INFO("short circuit fault detected.");
        ret = -1;
    } else {
        cd->color_status = BLUE;
        colorctrl_reset_hrtimer(cd);
    }

OUT:
    gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
    gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    COLOR_INFO("%s recharge color to blue %s.", ret < 0 ? "failed" : "success");

    return;
}

static void colorctrl_blue_operation(struct color_ctrl_device *cd, bool is_ftm)
{
    struct color_ctrl_hw_resource *hw_res = NULL;
    int ret = 0, vm_volt = 0, i = 0;
    int charge_time = 0, recharge_time_1 = 0, recharge_time_2 = 0;
    int charge_volt = 0, recharge_volt_1 = 0, recharge_volt_2 = 0;
    struct color_ctrl_control_para *para = NULL;

    if (!cd || !cd->hw_res || !cd->blue_control_para) {
        COLOR_INFO("no dev or resources find, return.");
        return;
    }

    if (cd->color_status == BLUE) {
        COLOR_INFO("device is already in blue status.");
        return;
    }

    colorctrl_update_temperature_status(cd);

    hw_res = cd->hw_res;
    para = cd->blue_control_para;

    switch (cd->temp_status) {
    case LOW_TEMP : {
        charge_volt = para->low_temp_charge_vol;
        recharge_volt_1 = para->low_temp_recharge_vol_1;
        recharge_volt_2 = para->low_temp_recharge_vol_2;
        charge_time = para->low_temp_charge_time;
        recharge_time_1 = para->low_temp_recharge_time_1;
        recharge_time_2 = para->low_temp_recharge_time_2;
        break;
    }
    case ROOM_TEMP : {
        charge_volt = para->normal_temp_charge_vol;
        recharge_volt_1 = para->normal_temp_recharge_vol_1;
        recharge_volt_2 = para->normal_temp_recharge_vol_2;
        charge_time = para->normal_temp_charge_time;
        recharge_time_1 = para->normal_temp_recharge_time_1;
        recharge_time_2 = para->normal_temp_recharge_time_2;
        break;
    }
    case HIGH_TEMP : {
        charge_volt = para->high_temp_charge_vol;
        recharge_volt_1 = para->high_temp_recharge_vol_1;
        recharge_volt_2 = para->high_temp_recharge_vol_2;
        charge_time = para->high_temp_charge_time;
        recharge_time_1 = para->high_temp_recharge_time_1;
        recharge_time_2 = para->high_temp_recharge_time_2;
        break;
    }
    case ABNORMAL_TEMP : {
        COLOR_INFO("abnormal temperature occur, do not change color.");
        ret = -1;
        goto OUT;
        break;
    }
    default :
        break;
    }

    ret = colorctrl_power_control(cd->hw_res, charge_volt, true);
    if (ret) {
        COLOR_INFO("enable power failed.");
        goto OUT;
    }

    ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_HIGH);
    ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
    ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
    colorctrl_msleep(charge_time);
    ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    if (ret) {
        COLOR_INFO("config gpio status failed.");
        colorctrl_power_control(cd->hw_res, 0, false);
        goto OUT;
    }
    colorctrl_power_control(cd->hw_res, 0, false);

    if (is_ftm) {
        colorctrl_reset_hrtimer(cd);
        cd->color_status = BLUE;
        COLOR_INFO("ftm mode operation, no need to do voltage detection.");
        goto OUT;
    }

    msleep(2000);
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
    msleep(2);
    ret = iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
    if (ret < 0) {
        COLOR_INFO("iio_read_channel_processed get error ret = %d", ret);
        goto OUT;
    }
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    vm_volt = vm_volt / UV_PER_MV;
    COLOR_INFO("current volt : %d(mV).", vm_volt);

    for (i = 0; i < 3; i++) {
        if (vm_volt < para->recharge_vol_thd_1) {
            COLOR_INFO("volt is too low, try to do normal charging.");
            colorctrl_power_control(cd->hw_res, charge_volt, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(charge_time);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_1 && vm_volt < para->recharge_vol_thd_2) {
            COLOR_INFO("volt is too low, try to do recharging(1).");
            colorctrl_power_control(cd->hw_res, recharge_volt_1, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(recharge_time_1);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_2 && vm_volt < para->recharge_vol_thd_3) {
            COLOR_INFO("volt is too low, try to do recharging(2).");
            colorctrl_power_control(cd->hw_res, recharge_volt_2, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(recharge_time_2);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_3) {
            break;
        }

        msleep(2000);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
        msleep(2);
        iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
        vm_volt = vm_volt / UV_PER_MV;
        COLOR_INFO("volt : %d(mV), rety %d times.", vm_volt, i + 1);
    }

    if (vm_volt < cd->open_circuit_thd) {
        cd->color_status = OPEN_CIRCUIT;
        COLOR_INFO("open circuit fault detected.");
        ret = -1;
    } else if (vm_volt >= cd->open_circuit_thd && vm_volt < cd->blue_short_circuit_thd) {
        cd->color_status = SHORT_CIRCUIT;
        COLOR_INFO("short circuit fault detected.");
        ret = -1;
    } else {
        cd->color_status = BLUE;
        colorctrl_reset_hrtimer(cd);
    }

OUT:
    gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
    gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    COLOR_INFO("%s change color to blue %s.", ret < 0 ? "failed" : "success");

    return;
}

static void colorctrl_transparent_recharge_operation(struct color_ctrl_device *cd)
{
    struct color_ctrl_hw_resource *hw_res = NULL;
    int ret = 0, vm_volt = 0, i = 0;
    int charge_time = 0, recharge_time_1 = 0, recharge_time_2 = 0;
    int charge_volt = 0, recharge_volt_1 = 0, recharge_volt_2 = 0;
    struct color_ctrl_control_para *para = NULL;

    if (!cd || !cd->hw_res || !cd->transparent_control_para) {
        COLOR_INFO("no dev or resources find, return.");
        return;
    }

    colorctrl_update_temperature_status(cd);
    if (cd->temp_status == ABNORMAL_TEMP) {
        COLOR_INFO("abnormal temperature occur, can not do recharge operation");
        return;
    }

    hw_res = cd->hw_res;
    para = cd->transparent_control_para;

    switch (cd->temp_status) {
    case LOW_TEMP : {
        charge_volt = para->low_temp_charge_vol;
        recharge_volt_1 = para->low_temp_recharge_vol_1;
        recharge_volt_2 = para->low_temp_recharge_vol_2;
        charge_time = para->low_temp_charge_time;
        recharge_time_1 = para->low_temp_recharge_time_1;
        recharge_time_2 = para->low_temp_recharge_time_2;
        break;
    }
    case ROOM_TEMP : {
        charge_volt = para->normal_temp_charge_vol;
        recharge_volt_1 = para->normal_temp_recharge_vol_1;
        recharge_volt_2 = para->normal_temp_recharge_vol_2;
        charge_time = para->normal_temp_charge_time;
        recharge_time_1 = para->normal_temp_recharge_time_1;
        recharge_time_2 = para->normal_temp_recharge_time_2;
        break;
    }
    case HIGH_TEMP : {
        charge_volt = para->high_temp_charge_vol;
        recharge_volt_1 = para->high_temp_recharge_vol_1;
        recharge_volt_2 = para->high_temp_recharge_vol_2;
        charge_time = para->high_temp_charge_time;
        recharge_time_1 = para->high_temp_recharge_time_1;
        recharge_time_2 = para->high_temp_recharge_time_2;
        break;
    }
    case ABNORMAL_TEMP : {
        COLOR_INFO("abnormal temperature occur, do not change color.");
        ret = -1;
        goto OUT;
        break;
    }
    default :
        break;
    }

    ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
    ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_HIGH);
    ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
    if (ret) {
        COLOR_INFO("config gpio status failed.");
        goto OUT;
    }
    msleep(2);
    ret = iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
    if (ret < 0) {
        COLOR_INFO("iio_read_channel_processed get error ret = %d", ret);
        goto OUT;
    }
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    vm_volt = vm_volt / UV_PER_MV;
    COLOR_INFO("current volt : %d(mV).", vm_volt);

    for (i = 0; i < 3; i++) {
        if (vm_volt < para->recharge_vol_thd_1) {
            COLOR_INFO("volt is too low, try to do normal charging.");
            ret = colorctrl_power_control(cd->hw_res, charge_volt, true);
            if (ret) {
                COLOR_INFO("enable power failed.");
                goto OUT;
            }
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(charge_time);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_1 && vm_volt < para->recharge_vol_thd_2) {
            COLOR_INFO("volt is too low, try to do recharging(1).");
            colorctrl_power_control(cd->hw_res, recharge_volt_1, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(recharge_time_1);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_2 && vm_volt < para->recharge_vol_thd_3) {
            COLOR_INFO("volt is too low, try to do recharging(2).");
            colorctrl_power_control(cd->hw_res, recharge_volt_2, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(recharge_time_2);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_3) {
            break;
        }

        msleep(2000);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
        msleep(2);
        iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
        vm_volt = vm_volt / UV_PER_MV;
        COLOR_INFO("volt : %d(mV), rety %d times.", vm_volt, i + 1);
    }

    if (vm_volt < cd->open_circuit_thd) {
        cd->color_status = OPEN_CIRCUIT;
        COLOR_INFO("open circuit fault detected.");
        ret = -1;
    } else if (vm_volt >= cd->open_circuit_thd && vm_volt < cd->transparent_short_circuit_thd) {
        cd->color_status = SHORT_CIRCUIT;
        COLOR_INFO("short circuit fault detected.");
        ret = -1;
    } else {
        cd->color_status = TRANSPARENT;
        colorctrl_reset_hrtimer(cd);
    }

OUT:
    gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
    gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    COLOR_INFO("%s recharge color to transparent %s.", ret < 0 ? "failed" : "success");

    return;
}

static void colorctrl_transparent_operation(struct color_ctrl_device *cd, bool is_ftm)
{
    struct color_ctrl_hw_resource *hw_res = NULL;
    int ret = 0, vm_volt = 0, i = 0;
    int charge_time = 0, recharge_time_1 = 0, recharge_time_2 = 0;
    int charge_volt = 0, recharge_volt_1 = 0, recharge_volt_2 = 0;
    struct color_ctrl_control_para *para = NULL;

    if (!cd || !cd->hw_res || !cd->transparent_control_para) {
        COLOR_INFO("no dev or resources find, return.");
        return;
    }

    if (cd->color_status == TRANSPARENT) {
        COLOR_INFO("device is already in transparent status.");
        return;
    }

    colorctrl_update_temperature_status(cd);

    hw_res = cd->hw_res;
    para = cd->transparent_control_para;

    switch (cd->temp_status) {
    case LOW_TEMP : {
        charge_volt = para->low_temp_charge_vol;
        recharge_volt_1 = para->low_temp_recharge_vol_1;
        recharge_volt_2 = para->low_temp_recharge_vol_2;
        charge_time = para->low_temp_charge_time;
        recharge_time_1 = para->low_temp_recharge_time_1;
        recharge_time_2 = para->low_temp_recharge_time_2;
        break;
    }
    case ROOM_TEMP : {
        charge_volt = para->normal_temp_charge_vol;
        recharge_volt_1 = para->normal_temp_recharge_vol_1;
        recharge_volt_2 = para->normal_temp_recharge_vol_2;
        charge_time = para->normal_temp_charge_time;
        recharge_time_1 = para->normal_temp_recharge_time_1;
        recharge_time_2 = para->normal_temp_recharge_time_2;
        break;
    }
    case HIGH_TEMP : {
        charge_volt = para->high_temp_charge_vol;
        recharge_volt_1 = para->high_temp_recharge_vol_1;
        recharge_volt_2 = para->high_temp_recharge_vol_2;
        charge_time = para->high_temp_charge_time;
        recharge_time_1 = para->high_temp_recharge_time_1;
        recharge_time_2 = para->high_temp_recharge_time_2;
        break;
    }
    case ABNORMAL_TEMP : {
        COLOR_INFO("abnormal temperature occur, do not change color.");
        ret = -1;
        goto OUT;
        break;
    }
    default :
        break;
    }

    ret = colorctrl_power_control(cd->hw_res, charge_volt, true);
    if (ret) {
        COLOR_INFO("enable power failed.");
        goto OUT;
    }

    ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
    ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_HIGH);
    ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
    colorctrl_msleep(charge_time);
    ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    if (ret) {
        COLOR_INFO("Config gpio status failed.");
        colorctrl_power_control(cd->hw_res, 0, false);
        goto OUT;
    }
    colorctrl_power_control(cd->hw_res, 0, false);

    if (is_ftm) {
        cd->color_status = TRANSPARENT;
        colorctrl_reset_hrtimer(cd);
        COLOR_INFO("ftm mode operation, no need to do voltage detection.");
        goto OUT;
    }

    msleep(2000);
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
    msleep(2);
    ret = iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
    if (ret < 0) {
        COLOR_INFO("iio_read_channel_processed get error ret = %d", ret);
        goto OUT;
    }
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    vm_volt = vm_volt / UV_PER_MV;
    COLOR_INFO("current volt : %d(mV).", vm_volt);

    for (i = 0; i < 3; i++) {
        if (vm_volt < para->recharge_vol_thd_1) {
            COLOR_INFO("volt is too low, try to do normal charging.");
            colorctrl_power_control(cd->hw_res, charge_volt, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(charge_time);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_1 && vm_volt < para->recharge_vol_thd_2) {
            COLOR_INFO("volt is too low, try to do recharging(1).");
            colorctrl_power_control(cd->hw_res, recharge_volt_1, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(recharge_time_1);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_2 && vm_volt < para->recharge_vol_thd_3) {
            COLOR_INFO("volt is too low, try to do recharging(2).");
            colorctrl_power_control(cd->hw_res, recharge_volt_2, true);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
            colorctrl_msleep(recharge_time_2);
            gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            colorctrl_power_control(cd->hw_res, 0, false);
        } else if (vm_volt >= para->recharge_vol_thd_3) {
            break;
        }

        msleep(2000);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
        msleep(2);
        iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
        gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
        vm_volt = vm_volt / UV_PER_MV;
        COLOR_INFO("volt : %d(mV), rety %d times.", vm_volt, i + 1);
    }

    if (vm_volt < cd->open_circuit_thd) {
        cd->color_status = OPEN_CIRCUIT;
        COLOR_INFO("open circuit fault detected.");
        ret = -1;
    } else if (vm_volt >= cd->open_circuit_thd && vm_volt < cd->transparent_short_circuit_thd) {
        cd->color_status = SHORT_CIRCUIT;
        COLOR_INFO("short circuit fault detected.");
        ret = -1;
    } else {
        cd->color_status = TRANSPARENT;
        colorctrl_reset_hrtimer(cd);
    }

OUT:
    gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
    gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    COLOR_INFO("%s change color to transparent %s.", ret < 0 ? "failed" : "success");

    return;
}

static void colorctrl_light_blue_operation(struct color_ctrl_device *cd)
{
    struct color_ctrl_hw_resource *hw_res = NULL;
    int ret = 0;

    if (!cd || !cd->hw_res) {
        COLOR_INFO("No dev or resources find, return.");
        return;
    }

    if (cd->color_status == LIGHT_BLUE) {
        COLOR_INFO("device is already in light blue status.");
        return;
    }

    hw_res = cd->hw_res;

    ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_HIGH);
    ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_HIGH);
    ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
    colorctrl_msleep(cd->intermediate_wait_time);
    ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
    ret |= gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
    ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
    if (ret) {
        COLOR_INFO("Config gpio status failed.");
    } else {
        cd->color_status = LIGHT_BLUE;
    }
    COLOR_INFO("change color to light blue %s.", ret < 0 ? "failed" : "success");

    return;
}

static ssize_t proc_colorctrl_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    char buf[8] = {0};
    int temp = 0;
    struct color_ctrl_device *cd = PDE_DATA(file_inode(file));

    if (!cd || count > 2) {
        return count;
    }

    if (copy_from_user(buf, buffer, count)) {
        COLOR_INFO("read proc input error.");
        return count;
    }
    sscanf(buf, "%d", &temp);
    COLOR_INFO("write value: %d.", temp);

    mutex_lock(&cd->rw_lock);

    if (temp > MAX_CTRL_TYPE) {
        COLOR_INFO("not support change color type.");
        mutex_unlock(&cd->rw_lock);
        return count;
    }

    if (cd->color_status == OPEN_CIRCUIT || cd->color_status == SHORT_CIRCUIT) {
        COLOR_INFO("device is in bad status, can not do any operation, color_status : %d", cd->color_status);
        mutex_unlock(&cd->rw_lock);
        return count;
    }

    switch (temp) {
    case LIGHT_BLUE_FTM : {
        colorctrl_light_blue_operation(cd);
        break;
    }
    case BLUE_FTM : {
        colorctrl_blue_operation(cd, true);
        break;
    }
    case TRANSPARENT_FTM : {
        colorctrl_transparent_operation(cd, true);
        break;
    }
    case BLUE_NORMAL: {
        if (cd->color_status !=  UNKNOWN) {
            colorctrl_blue_operation(cd, false);
        } else {
            colorctrl_blue_recharge_operation(cd);
        }
        break;
    }
    case TRANSPARENT_NORMAL : {
        if (cd->color_status != UNKNOWN) {
            colorctrl_transparent_operation(cd, false);
        } else {
            colorctrl_transparent_recharge_operation(cd);
        }
        break;
    }
    case RESET : {
        COLOR_INFO("reset color to default status.");
        colorctrl_light_blue_operation(cd);
        colorctrl_transparent_operation(cd, false);
        break;
    }
    case RESET_FTM : {
        COLOR_INFO("ftm mode reset color to default status.");
        if (cd->color_status == UNKNOWN || cd->color_status == BLUE) {
            colorctrl_transparent_operation(cd, true);
            colorctrl_light_blue_operation(cd);
        } else if (cd->color_status == TRANSPARENT) {
            colorctrl_light_blue_operation(cd);
        } else {
            COLOR_INFO("current color status is : %d, not need to do any operation.", cd->color_status);
        }
        break;
    }
    case STOP_RECHARGE : {
        cd->need_recharge = false;
        COLOR_INFO("stop recharge work.");
        break;
    }
    case OPEN_RECHARGE : {
        cd->need_recharge = true;
        COLOR_INFO("open recharge work.");
        break;
    }
    default :
        COLOR_INFO("not support color status.");
        break;
    }

    mutex_unlock(&cd->rw_lock);

    return count;
}

static ssize_t proc_colorctrl_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    char page[PAGESIZE] = {0};
    struct color_ctrl_device *cd = PDE_DATA(file_inode(file));

    if (!cd || *ppos != 0) {
        return 0;
    }

    mutex_lock(&cd->rw_lock);
    snprintf(page, PAGESIZE - 1, "%d\n", cd->color_status);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    mutex_unlock(&cd->rw_lock);
    COLOR_INFO("read value: %d.", cd->color_status);

    return ret;
}

static const struct file_operations proc_colorctrl_ops = {
    .read  = proc_colorctrl_read,
    .write = proc_colorctrl_write,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t proc_temperature_control_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    char page[PAGESIZE] = {0};
    int temp = 0;
    struct color_ctrl_device *cd = PDE_DATA(file_inode(file));

    if (!cd || *ppos != 0) {
        return 0;
    }

    mutex_lock(&cd->rw_lock);

    if (cd->thermal_zone_device) {
        ret = thermal_zone_get_temp(cd->thermal_zone_device, &temp);
        if (ret) {
            COLOR_INFO("fail to get shell_back temperature: %d", ret);
        } else {
            COLOR_INFO("current shell back temperature is : %d", temp);
            snprintf(page, PAGESIZE - 1, "%d\n", temp);
        }
    } else {
        COLOR_INFO("temperature read is not support.", temp);
        snprintf(page, PAGESIZE - 1, "temperature read is not support\n");
    }

    mutex_unlock(&cd->rw_lock);

    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));

    return ret;
}

static const struct file_operations proc_temperature_control_ops = {
    .read  = proc_temperature_control_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static ssize_t colorctrl_voltage_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    int vm_volt = 0;
    char page[PAGESIZE] = {0};
    struct color_ctrl_device *cd = PDE_DATA(file_inode(file));
    struct color_ctrl_hw_resource *hw_res = NULL;

    if (!cd || *ppos != 0) {
        return 0;
    }

    mutex_lock(&cd->rw_lock);

    hw_res = cd->hw_res;

    if (!hw_res->vm_v_chan) {
        COLOR_INFO("voltage read is not support.", ret);
        snprintf(page, PAGESIZE - 1, "voltage read is not support.\n");
        goto OUT;
    }

    switch (cd->color_status) {
    case LIGHT_BLUE : {
        ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
        ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_HIGH);
        ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
        if (ret) {
            COLOR_INFO("Config gpio status failed.");
            goto OUT;
        }
        break;
    }
    case BLUE : {
        ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_HIGH);
        ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
        ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
        if (ret) {
            COLOR_INFO("Config gpio status failed.");
            goto OUT;
        }
        break;
    }
    case TRANSPARENT : {
        ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
        ret |= gpio_direction_output(hw_res->si_in_2_gpio, GPIO_HIGH);
        ret |= gpio_direction_output(hw_res->sleep_en_gpio, GPIO_HIGH);
        if (ret) {
            COLOR_INFO("Config gpio status failed.");
            goto OUT;
        }
        break;
    }
    default :
        COLOR_INFO("not support voltage read type, current color status : %d.", cd->color_status);
        goto OUT;
        break;
    }

    msleep(2);

    ret = iio_read_channel_processed(hw_res->vm_v_chan, &vm_volt);
    if (ret < 0) {
        COLOR_INFO("iio_read_channel_processed get error ret = %d", ret);
        goto OUT;
    }

    gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);

    vm_volt = vm_volt / UV_PER_MV;
    COLOR_INFO("vm_volt: %d", vm_volt);
    snprintf(page, PAGESIZE - 1, "%d\n", vm_volt);

OUT:
    mutex_unlock(&cd->rw_lock);
    return simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
}

static const struct file_operations proc_adc_voltage_ops = {
    .read  = colorctrl_voltage_read,
    .open  = simple_open,
    .owner = THIS_MODULE,
};

static int colorctrl_str_to_int(char *in, int start_pos, int end_pos)
{
    int i = 0, value = 0;

    if (start_pos > end_pos) {
        return -1;
    }

    for (i = start_pos; i <= end_pos; i++) {
        value = (value * 10) + (in[i] - '0');
    }

    COLOR_INFO("return %d.", value);
    return value;
}

//parse string according to name:value1,value2,value3...
static int colorctrl_str_parse(char *in, char *name, unsigned int max_len, unsigned int *array, unsigned int array_max)
{
    int i = 0, in_cnt = 0, name_index = 0;
    int start_pos = 0, value_cnt = 0;

    if (!array || !in) {
        COLOR_INFO("array or in is null.");
        return -1;
    }

    in_cnt = strlen(in);

    //parse name
    for (i = 0; i < in_cnt; i++) {
        if (':' == in[i]) {     //split name and parameter by ":" symbol
            if (i > max_len) {
                COLOR_INFO("string %s name too long.\n", in);
                return -1;
            }
            name_index = i;
            memcpy(name, in, name_index);   //copy to name buffer
            COLOR_INFO("set name %s.", name);
        }
    }

    //parse parameter and put it into split_value array
    start_pos = name_index + 1;
    for (i = name_index + 1; i <= in_cnt; i++) {
        if (in[i] < '0' || in[i] > '9') {
            if ((' ' == in[i]) || (0 == in[i]) || ('\n' == in[i]) || (',' == in[i])) {
                if (value_cnt <= array_max) {
                    array[value_cnt++] = colorctrl_str_to_int(in, start_pos, i - 1);
                    start_pos = i + 1;
                } else {
                    COLOR_INFO("too many parameter(%s).", in);
                    return -1;
                }
            } else {
                COLOR_INFO("incorrect char 0x%02x in %s.", in[i], in);
                return -1;
            }
        }
    }

    value_cnt = value_cnt - 1;
    COLOR_INFO("input para count is %d.", value_cnt);

    return value_cnt;
}

static int colorctrl_main_parameter_read(struct seq_file *s, void *v)
{
    struct color_ctrl_device *cd = s->private;
    struct color_ctrl_control_para *blue_para = NULL;
    struct color_ctrl_control_para *transparent_para = NULL;

    if (!cd || !cd->blue_control_para || !cd->transparent_control_para) {
        return 0;
    }
    COLOR_INFO(" call.");

    blue_para = cd->blue_control_para;
    transparent_para = cd->transparent_control_para;

    mutex_lock(&cd->rw_lock);
    seq_printf(s, "recharge time : %d(s)\n", cd->recharge_time);
    seq_printf(s, "intermediate state wait time : %d(ms)\n", cd->intermediate_wait_time);
    seq_printf(s, "low tempetature threshold : [%d, %d]\n", cd->low_temp_low_thd, cd->low_temp_high_thd);
    seq_printf(s, "normal tempetature threshold : [%d, %d]\n", cd->normal_temp_low_thd, cd->normal_temp_high_thd);
    seq_printf(s, "high tempetature threshold : [%d, %d]\n", cd->high_temp_low_thd, cd->high_temp_high_thd);
    seq_printf(s, "open circuit voltage threshold : %d(mV)\n", cd->open_circuit_thd);
    seq_printf(s, "blue short circuit voltage threshold : %d(mV)\n", cd->blue_short_circuit_thd);
    seq_printf(s, "transparent short circuit voltage threshold : %d(mV)\n", cd->transparent_short_circuit_thd);
    seq_printf(s, "blue charge control parameter : low_temp:[%d, %d], normal_temp:[%d, %d], high_temp:[%d, %d]\n",
        blue_para->low_temp_charge_vol, blue_para->low_temp_charge_time, blue_para->normal_temp_charge_vol,
        blue_para->normal_temp_charge_time, blue_para->high_temp_charge_vol, blue_para->high_temp_charge_time);
    seq_printf(s, "transparent charge control parameter : low_temp:[%d, %d], normal_temp:[%d, %d], high_temp:[%d, %d]\n",
        transparent_para->low_temp_charge_vol, transparent_para->low_temp_charge_time, transparent_para->normal_temp_charge_vol,
        transparent_para->normal_temp_charge_time, transparent_para->high_temp_charge_vol, transparent_para->high_temp_charge_time);
    seq_printf(s, "blue recharge control parameter(1) : low_temp:[%d, %d], normal_temp:[%d, %d], high_temp:[%d, %d]\n",
        blue_para->low_temp_recharge_vol_1, blue_para->low_temp_recharge_time_1, blue_para->normal_temp_recharge_vol_1,
        blue_para->normal_temp_recharge_time_1, blue_para->high_temp_recharge_vol_1, blue_para->high_temp_recharge_time_1);
    seq_printf(s, "blue recharge control parameter(2) : low_temp:[%d, %d], normal_temp:[%d, %d], high_temp:[%d, %d]\n",
        blue_para->low_temp_recharge_vol_2, blue_para->low_temp_recharge_time_2, blue_para->normal_temp_recharge_vol_2,
        blue_para->normal_temp_recharge_time_2, blue_para->high_temp_recharge_vol_2, blue_para->high_temp_recharge_time_2);
    seq_printf(s, "transparent recharge control parameter(1) : low_temp:[%d, %d], normal_temp:[%d, %d], high_temp:[%d, %d]\n",
        transparent_para->low_temp_recharge_vol_1, transparent_para->low_temp_recharge_time_1, transparent_para->normal_temp_recharge_vol_1,
        transparent_para->normal_temp_recharge_time_1, transparent_para->high_temp_recharge_vol_1, transparent_para->high_temp_recharge_time_1);
    seq_printf(s, "transparent recharge control parameter(2) : low_temp:[%d, %d], normal_temp:[%d, %d], high_temp:[%d, %d]\n",
        transparent_para->low_temp_recharge_vol_2, transparent_para->low_temp_recharge_time_2, transparent_para->normal_temp_recharge_vol_2,
        transparent_para->normal_temp_recharge_time_2, transparent_para->high_temp_recharge_vol_2, transparent_para->high_temp_recharge_time_2);
    seq_printf(s, "blue recharge voltage thd : [%d(mV), %d(mV), %d(mV)]\n", blue_para->recharge_vol_thd_1, blue_para->recharge_vol_thd_2,
        blue_para->recharge_vol_thd_3);
    seq_printf(s, "transparent recharge voltage thd : [%d(mV), %d(mV), %d(mV)]\n", transparent_para->recharge_vol_thd_1, transparent_para->recharge_vol_thd_2,
        transparent_para->recharge_vol_thd_3);
    mutex_unlock(&cd->rw_lock);

    return 0;
}

static int main_parameter_open(struct inode *inode, struct file *file)
{
    return single_open(file, colorctrl_main_parameter_read, PDE_DATA(inode));
}

static ssize_t proc_colorctrl_main_parameter_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    char buf[PAGESIZE] = {0};
    int value_cnt = 0;
    char name[NAME_TAG_SIZE] = {0};
    unsigned int split_value[MAX_PARAMETER] = {0};
    struct color_ctrl_device *cd = PDE_DATA(file_inode(file));
    struct color_ctrl_control_para *blue_para = NULL;
    struct color_ctrl_control_para *transparent_para = NULL;

    if (!cd || !cd->blue_control_para || !cd->transparent_control_para || count >= PAGESIZE) {
        return count;
    }

    if (copy_from_user(buf, buffer, count)) {
        COLOR_INFO("read proc input error.");
        return count;
    }

    blue_para = cd->blue_control_para;
    transparent_para = cd->transparent_control_para;

    value_cnt = colorctrl_str_parse(buf, name, NAME_TAG_SIZE, split_value, MAX_PARAMETER);
    if (value_cnt < 0) {
        COLOR_INFO("str parse failed.");
        return count;
    }

    mutex_lock(&cd->rw_lock);

    if (strstr(name, "recharge_time") && (value_cnt == 1)) {
        cd->recharge_time = split_value[0];
        COLOR_INFO("%s is change to %d.", name, split_value[0]);
    } else if (strstr(name, "intermediate_wait_time") && (value_cnt == 1)) {
        cd->intermediate_wait_time = split_value[0];
        COLOR_INFO("%s is change to %d.", name, split_value[0]);
    } else if (strstr(name, "temperature_thd") && (value_cnt == 6)) {
        cd->low_temp_low_thd = split_value[0];
        cd->low_temp_high_thd = split_value[1];
        cd->normal_temp_low_thd = split_value[2];
        cd->normal_temp_high_thd = split_value[3];
        cd->high_temp_low_thd = split_value[4];
        cd->high_temp_high_thd = split_value[5];
        COLOR_INFO("%s is change : low temp [%d, %d], normal temp [%d, %d], high temp [%d, %d].", name, split_value[0], split_value[1], 
            split_value[2], split_value[3], split_value[4], split_value[5]);
    } else if (strstr(name, "abnormal_circuit_thd") && (value_cnt == 3)) {
        cd->open_circuit_thd = split_value[0];
        cd->blue_short_circuit_thd = split_value[1];
        cd->transparent_short_circuit_thd = split_value[2];
        COLOR_INFO("%s is change : [%d, %d, %d].", name, split_value[0], split_value[1], split_value[2]);
    } else if (strstr(name, "blue_charge_para") && (value_cnt == 6)) {
        blue_para->low_temp_charge_vol = split_value[0];
        blue_para->low_temp_charge_time = split_value[1];
        blue_para->normal_temp_charge_vol = split_value[2];
        blue_para->normal_temp_charge_time = split_value[3];
        blue_para->high_temp_charge_vol = split_value[4];
        blue_para->high_temp_charge_time = split_value[5];
        COLOR_INFO("%s is change : low temp [%d, %d], normal temp [%d, %d], high temp [%d, %d].", name, split_value[0], split_value[1], 
            split_value[2], split_value[3], split_value[4], split_value[5]);
    } else if (strstr(name, "blue_recharge_para_1") && (value_cnt == 6)) {
        blue_para->low_temp_recharge_vol_1 = split_value[0];
        blue_para->low_temp_recharge_time_1 = split_value[1];
        blue_para->normal_temp_recharge_vol_1 = split_value[2];
        blue_para->normal_temp_recharge_time_1 = split_value[3];
        blue_para->high_temp_recharge_vol_1 = split_value[4];
        blue_para->high_temp_recharge_time_1 = split_value[5];
        COLOR_INFO("%s is change : low temp [%d, %d], normal temp [%d, %d], high temp [%d, %d].", name, split_value[0], split_value[1], 
            split_value[2], split_value[3], split_value[4], split_value[5]);
    } else if (strstr(name, "blue_recharge_para_2") && (value_cnt == 6)) {
        blue_para->low_temp_recharge_vol_2 = split_value[0];
        blue_para->low_temp_recharge_time_2 = split_value[1];
        blue_para->normal_temp_recharge_vol_2 = split_value[2];
        blue_para->normal_temp_recharge_time_2 = split_value[3];
        blue_para->high_temp_recharge_vol_2 = split_value[4];
        blue_para->high_temp_recharge_time_2 = split_value[5];
        COLOR_INFO("%s is change : low temp [%d, %d], normal temp [%d, %d], high temp [%d, %d].", name, split_value[0], split_value[1], 
            split_value[2], split_value[3], split_value[4], split_value[5]);
    } else if (strstr(name, "transparent_charge_para") && (value_cnt == 6)) {
        transparent_para->low_temp_charge_vol = split_value[0];
        transparent_para->low_temp_charge_time = split_value[1];
        transparent_para->normal_temp_charge_vol = split_value[2];
        transparent_para->normal_temp_charge_time = split_value[3];
        transparent_para->high_temp_charge_vol = split_value[4];
        transparent_para->high_temp_charge_time = split_value[5];
        COLOR_INFO("%s is change : low temp [%d, %d], normal temp [%d, %d], high temp [%d, %d].", name, split_value[0], split_value[1], 
            split_value[2], split_value[3], split_value[4], split_value[5]);
    } else if (strstr(name, "transparent_recharge_para_1") && (value_cnt == 6)) {
        transparent_para->low_temp_recharge_vol_1 = split_value[0];
        transparent_para->low_temp_recharge_time_1 = split_value[1];
        transparent_para->normal_temp_recharge_vol_1 = split_value[2];
        transparent_para->normal_temp_recharge_time_1 = split_value[3];
        transparent_para->high_temp_recharge_vol_1 = split_value[4];
        transparent_para->high_temp_recharge_time_1 = split_value[5];
        COLOR_INFO("%s is change : low temp [%d, %d], normal temp [%d, %d], high temp [%d, %d].", name, split_value[0], split_value[1], 
            split_value[2], split_value[3], split_value[4], split_value[5]);
    } else if (strstr(name, "transparent_recharge_para_2") && (value_cnt == 6)) {
        transparent_para->low_temp_recharge_vol_2 = split_value[0];
        transparent_para->low_temp_recharge_time_2 = split_value[1];
        transparent_para->normal_temp_recharge_vol_2 = split_value[2];
        transparent_para->normal_temp_recharge_time_2 = split_value[3];
        transparent_para->high_temp_recharge_vol_2 = split_value[4];
        transparent_para->high_temp_recharge_time_2 = split_value[5];
        COLOR_INFO("%s is change : low temp [%d, %d], normal temp [%d, %d], high temp [%d, %d].", name, split_value[0], split_value[1], 
            split_value[2], split_value[3], split_value[4], split_value[5]);
    } else if (strstr(name, "blue_recharge_voltage_thd") && (value_cnt == 3)) {
        blue_para->recharge_vol_thd_1 = split_value[0];
        blue_para->recharge_vol_thd_2 = split_value[1];
        blue_para->recharge_vol_thd_3 = split_value[2];
        COLOR_INFO("%s is change : [%d, %d, %d].", name, split_value[0], split_value[1], split_value[2]);
    } else if (strstr(name, "transparent_recharge_voltage_thd") && (value_cnt == 3)) {
        transparent_para->recharge_vol_thd_1 = split_value[0];
        transparent_para->recharge_vol_thd_2 = split_value[1];
        transparent_para->recharge_vol_thd_3 = split_value[2];
        COLOR_INFO("%s is change : [%d, %d, %d].", name, split_value[0], split_value[1], split_value[2]);
    } else {
        COLOR_INFO("%s is not support or input value count is wrong.", name);
    }

    mutex_unlock(&cd->rw_lock);

    return count;
}

static const struct file_operations proc_main_parameter_ops = {
    .owner = THIS_MODULE,
    .open  = main_parameter_open,
    .read  = seq_read,
    .write = proc_colorctrl_main_parameter_write,
    .release = single_release,
};

static int colorctrl_init_proc(struct color_ctrl_device *cd)
{
    int ret = 0;
    struct proc_dir_entry *prEntry_cr = NULL;
    struct proc_dir_entry *prEntry_tmp = NULL;

    COLOR_INFO("entry");

    //proc files-step1:/proc/colorctrl
    prEntry_cr = proc_mkdir("colorctrl", NULL);
    if (prEntry_cr == NULL) {
        ret = -ENOMEM;
        COLOR_INFO("Couldn't create color ctrl proc entry");
    }

    //proc files-step2:/proc/touchpanel/color_ctrl (color control interface)
    prEntry_tmp = proc_create_data("color_ctrl", 0666, prEntry_cr, &proc_colorctrl_ops, cd);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        COLOR_INFO("Couldn't create color_ctrl proc entry");
    }

    //proc files-step2:/proc/touchpanel/temperature (color control temperature interface)
    prEntry_tmp = proc_create_data("temperature", 0666, prEntry_cr, &proc_temperature_control_ops, cd);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        COLOR_INFO("Couldn't create temperature proc entry");
    }

    //proc files-step2:/proc/touchpanel/voltage (color control voltage interface)
    prEntry_tmp = proc_create_data("voltage", 0666, prEntry_cr, &proc_adc_voltage_ops, cd);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        COLOR_INFO("Couldn't create voltage proc entry");
    }

    // show main_register interface
    prEntry_tmp = proc_create_data("main_parameter", 0666, prEntry_cr, &proc_main_parameter_ops, cd);
    if (prEntry_tmp == NULL) {
        ret = -ENOMEM;
        COLOR_INFO("Couldn't create main parameter proc entry");
    }

    cd->prEntry_cr = prEntry_cr;

    return ret;
}

static int colorctrl_parse_dt(struct device *dev, struct color_ctrl_device *cd)
{
    int ret = 0, i = 0;
    int prj_id = 0;
    int temp_array[10] = {0};
    struct device_node *np = dev->of_node;
    struct color_ctrl_hw_resource *hw_res = cd->hw_res;
    struct color_ctrl_control_para *blue_para = NULL;
    struct color_ctrl_control_para *transparent_para = NULL;

    if(!np || !hw_res) {
        COLOR_INFO("Don't has device of_node.");
        return -1;
    }

    blue_para = devm_kzalloc(dev, sizeof(struct color_ctrl_control_para), GFP_KERNEL);
    if(!blue_para) {
        COLOR_INFO("Malloc memory for color control blue para fail.");
        return -ENOMEM;
    }

    transparent_para = devm_kzalloc(dev, sizeof(struct color_ctrl_control_para), GFP_KERNEL);
    if(!transparent_para) {
        COLOR_INFO("Malloc memory for color control transparent para fail.");
        return -ENOMEM;
    }

    cd->blue_control_para = blue_para;
    cd->transparent_control_para = transparent_para;

    cd->project_num  = of_property_count_u32_elems(np, "platform_support_project");
    if (cd->project_num <= 0) {
        COLOR_INFO("project not specified, need to config the support project.");
        return -1;
    } else {
        ret = of_property_read_u32_array(np, "platform_support_project", cd->platform_support_project, cd->project_num);
        if (ret) {
            COLOR_INFO("platform_support_project not specified.");
            return -1;
        }
        prj_id = get_project();
        for (i = 0; i < cd->project_num; i++) {
            if (prj_id == cd->platform_support_project[i]) {
                COLOR_INFO("driver match the project.");
                break;
            }
        }
        if (i == cd->project_num) {
            COLOR_INFO("driver does not match the project.");
            return -1;
        }
    }

    hw_res->sleep_en_gpio = of_get_named_gpio(np, "gpio-sleep_en", 0);
    if ((!gpio_is_valid(hw_res->sleep_en_gpio))) {
        COLOR_INFO("parse gpio-sleep_en fail.");
        return -1;
    }

    hw_res->si_in_1_gpio = of_get_named_gpio(np, "gpio-si_in_1", 0);
    if ((!gpio_is_valid(hw_res->si_in_1_gpio))) {
        COLOR_INFO("parse gpio-si_in_1 fail.");
        return -1;
    }

    hw_res->si_in_2_gpio = of_get_named_gpio(np, "gpio-si_in_2", 0);
    if ((!gpio_is_valid(hw_res->si_in_2_gpio))) {
        COLOR_INFO("parse gpio-si_in_2 fail.");
        return -1;
    }

    hw_res->vm_enable_gpio = of_get_named_gpio(np, "enable_vm_gpio", 0);
    if (!gpio_is_valid(hw_res->vm_enable_gpio)) {
        COLOR_INFO("enable_vm_gpio is not specified.");
    }

    hw_res->vm = devm_regulator_get(dev, "vm");
    if (IS_ERR_OR_NULL(hw_res->vm)) {
        COLOR_INFO("Regulator vm get failed.");
        return -1;
    }

    hw_res->vm_v_chan = devm_iio_channel_get(dev, "colorctrl_voltage_adc");
    if (IS_ERR(hw_res->vm_v_chan)) {
        ret = PTR_ERR(hw_res->vm_v_chan);
        hw_res->vm_v_chan = NULL;
        COLOR_INFO("vm_v_chan get error or voltage read is not support, ret = %d\n", ret);
    } else {
        COLOR_INFO("hw_res->vm_v_chan get success.\n");
    }

    COLOR_INFO("Parse dt ok, get gpios:[sleep_en_gpio:%d si_in_1_gpio:%d si_in_2_gpio:%d]",
               hw_res->sleep_en_gpio, hw_res->si_in_1_gpio, hw_res->si_in_2_gpio);

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,recharge_time", temp_array, 1);
    if (ret) {
        cd->recharge_time = 43200;
        COLOR_INFO("recharge time using default.");
    } else {
        cd->recharge_time = temp_array[0];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,intermediate_wait_time", temp_array, 1);
    if (ret) {
        cd->intermediate_wait_time = 20000;
        COLOR_INFO("intermediate state wait time using default.");
    } else {
        cd->intermediate_wait_time = temp_array[0];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,temperature_thd", temp_array, 6);
    if (ret) {
        cd->low_temp_low_thd = 5000;
        cd->low_temp_high_thd = 10000;
        cd->normal_temp_low_thd = 10000;
        cd->normal_temp_high_thd = 50000;
        cd->high_temp_low_thd = 50000;
        cd->high_temp_high_thd = 65000;
        COLOR_INFO("temperature threshold using default.");
    } else {
        cd->low_temp_low_thd = temp_array[0];
        cd->low_temp_high_thd = temp_array[1];
        cd->normal_temp_low_thd = temp_array[2];
        cd->normal_temp_high_thd = temp_array[3];
        cd->high_temp_low_thd = temp_array[4];
        cd->high_temp_high_thd = temp_array[5];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,abnormal_voltage_thd", temp_array, 3);
    if (ret) {
        cd->open_circuit_thd = 100;
        cd->blue_short_circuit_thd = 300;
        cd->transparent_short_circuit_thd = 300;
        COLOR_INFO("abnormal circuit voltage using default.");
    } else {
        cd->open_circuit_thd = temp_array[0];
        cd->blue_short_circuit_thd = temp_array[1];
        cd->transparent_short_circuit_thd = temp_array[2];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,blue_charge_para", temp_array, 6);
    if (ret) {
        blue_para->low_temp_charge_vol = 1000;
        blue_para->low_temp_charge_time= 15000;
        blue_para->normal_temp_charge_vol = 1000;
        blue_para->normal_temp_charge_time= 8000;
        blue_para->high_temp_charge_vol = 800;
        blue_para->high_temp_charge_time = 8000;
        COLOR_INFO("blue charge para using default.");
    } else {
        blue_para->low_temp_charge_vol = temp_array[0];
        blue_para->low_temp_charge_time= temp_array[1];
        blue_para->normal_temp_charge_vol = temp_array[2];
        blue_para->normal_temp_charge_time= temp_array[3];
        blue_para->high_temp_charge_vol = temp_array[4];
        blue_para->high_temp_charge_time = temp_array[5];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,transparent_charge_para", temp_array, 6);
    if (ret) {
        transparent_para->low_temp_charge_vol = 1000;
        transparent_para->low_temp_charge_time= 15000;
        transparent_para->normal_temp_charge_vol = 1000;
        transparent_para->normal_temp_charge_time= 8000;
        transparent_para->high_temp_charge_vol = 600;
        transparent_para->high_temp_charge_time = 8000;
        COLOR_INFO("transparent charge para using default.");
    } else {
        transparent_para->low_temp_charge_vol = temp_array[0];
        transparent_para->low_temp_charge_time= temp_array[1];
        transparent_para->normal_temp_charge_vol = temp_array[2];
        transparent_para->normal_temp_charge_time= temp_array[3];
        transparent_para->high_temp_charge_vol = temp_array[4];
        transparent_para->high_temp_charge_time = temp_array[5];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,blue_recharge_para_1", temp_array, 6);
    if (ret) {
        blue_para->low_temp_recharge_vol_1 = 800;
        blue_para->low_temp_recharge_time_1 = 5000;
        blue_para->normal_temp_recharge_vol_1 = 800;
        blue_para->normal_temp_recharge_time_1 = 5000;
        blue_para->high_temp_recharge_vol_1 = 800;
        blue_para->high_temp_recharge_time_1 = 5000;
        COLOR_INFO("blue recharge para(1) using default.");
    } else {
        blue_para->low_temp_recharge_vol_1 = temp_array[0];
        blue_para->low_temp_recharge_time_1 = temp_array[1];
        blue_para->normal_temp_recharge_vol_1 = temp_array[2];
        blue_para->normal_temp_recharge_time_1 = temp_array[3];
        blue_para->high_temp_recharge_vol_1 = temp_array[4];
        blue_para->high_temp_recharge_time_1 = temp_array[5];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,blue_recharge_para_2", temp_array, 6);
    if (ret) {
        blue_para->low_temp_recharge_vol_2 = 800;
        blue_para->low_temp_recharge_time_2 = 5000;
        blue_para->normal_temp_recharge_vol_2 = 800;
        blue_para->normal_temp_recharge_time_2 = 5000;
        blue_para->high_temp_recharge_vol_2 = 800;
        blue_para->high_temp_recharge_time_2 = 5000;
        COLOR_INFO("blue recharge para(2) using default.");
    } else {
        blue_para->low_temp_recharge_vol_2 = temp_array[0];
        blue_para->low_temp_recharge_time_2 = temp_array[1];
        blue_para->normal_temp_recharge_vol_2 = temp_array[2];
        blue_para->normal_temp_recharge_time_2 = temp_array[3];
        blue_para->high_temp_recharge_vol_2 = temp_array[4];
        blue_para->high_temp_recharge_time_2 = temp_array[5];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,transparent_recharge_para_1", temp_array, 6);
    if (ret) {
        transparent_para->low_temp_recharge_vol_1 = 600;
        transparent_para->low_temp_recharge_time_1 = 5000;
        transparent_para->normal_temp_recharge_vol_1 = 600;
        transparent_para->normal_temp_recharge_time_1 = 5000;
        transparent_para->high_temp_recharge_vol_1 = 600;
        transparent_para->high_temp_recharge_time_1 = 5000;
        COLOR_INFO("transparent recharge para(1) using default.");
    } else {
        transparent_para->low_temp_recharge_vol_1 = temp_array[0];
        transparent_para->low_temp_recharge_time_1 = temp_array[1];
        transparent_para->normal_temp_recharge_vol_1 = temp_array[2];
        transparent_para->normal_temp_recharge_time_1 = temp_array[3];
        transparent_para->high_temp_recharge_vol_1 = temp_array[4];
        transparent_para->high_temp_recharge_time_1 = temp_array[5];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,transparent_recharge_para_2", temp_array, 6);
    if (ret) {
        transparent_para->low_temp_recharge_vol_2 = 600;
        transparent_para->low_temp_recharge_time_2 = 5000;
        transparent_para->normal_temp_recharge_vol_2 = 600;
        transparent_para->normal_temp_recharge_time_2 = 5000;
        transparent_para->high_temp_recharge_vol_2 = 600;
        transparent_para->high_temp_recharge_time_2 = 5000;
        COLOR_INFO("transparent recharge para(2) using default.");
    } else {
        transparent_para->low_temp_recharge_vol_2 = temp_array[0];
        transparent_para->low_temp_recharge_time_2 = temp_array[1];
        transparent_para->normal_temp_recharge_vol_2 = temp_array[2];
        transparent_para->normal_temp_recharge_time_2 = temp_array[3];
        transparent_para->high_temp_recharge_vol_2 = temp_array[4];
        transparent_para->high_temp_recharge_time_2 = temp_array[5];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,blue_recharge_vol_thd", temp_array, 3);
    if (ret) {
        blue_para->recharge_vol_thd_1 = 100;
        blue_para->recharge_vol_thd_2 = 300;
        blue_para->recharge_vol_thd_3 = 300;
        COLOR_INFO("blue recharge voltage threshold using default.");
    } else {
        blue_para->recharge_vol_thd_1 = temp_array[0];
        blue_para->recharge_vol_thd_2 = temp_array[1];
        blue_para->recharge_vol_thd_3 = temp_array[2];
    }

    ret = of_property_read_u32_array(dev->of_node, "colorctrl,transparent_recharge_vol_thd", temp_array, 3);
    if (ret) {
        transparent_para->recharge_vol_thd_1 = 100;
        transparent_para->recharge_vol_thd_2 = 300;
        transparent_para->recharge_vol_thd_3 = 300;
        COLOR_INFO("transparent recharge voltage threshold using default.");
    } else {
        transparent_para->recharge_vol_thd_1 = temp_array[0];
        transparent_para->recharge_vol_thd_2 = temp_array[1];
        transparent_para->recharge_vol_thd_3 = temp_array[2];
    }

    return 0;
}

static int colorctrl_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct color_ctrl_hw_resource *hw_res = NULL;
    struct color_ctrl_device *cd = NULL;

    COLOR_INFO("start to probe color ctr driver.");

    /*malloc memory for hardware resource */
    if(pdev->dev.of_node) {
        hw_res = devm_kzalloc(&pdev->dev, sizeof(struct color_ctrl_hw_resource), GFP_KERNEL);
        if(!hw_res) {
            ret = -ENOMEM;
            COLOR_INFO("Malloc memory for hardware resoure fail.");
            goto PROBE_ERR;
        }
    } else {
        hw_res = pdev->dev.platform_data;
    }

    /*malloc memory for color ctrl device*/
    cd = devm_kzalloc(&pdev->dev, sizeof(struct color_ctrl_device), GFP_KERNEL);
    if(!cd) {
        COLOR_INFO("Malloc memory for color ctr device fail.");
        ret = -ENOMEM;
        goto PROBE_ERR;
    }

    cd->hw_res = hw_res;

    ret = colorctrl_parse_dt(&pdev->dev, cd);
    if (ret) {
        COLOR_INFO("parse dts fail.");
        goto PROBE_ERR;
    }

    /*Request and config these gpios*/
    if (gpio_is_valid(hw_res->sleep_en_gpio)) {
        ret = devm_gpio_request(&pdev->dev, hw_res->sleep_en_gpio, "sleep_en_gpio");
        if(ret) {
            COLOR_INFO("request sleep_en_gpio fail.");
            goto PROBE_ERR;
        } else {
            /*Enable the sleep en gpio.*/
            ret = gpio_direction_output(hw_res->sleep_en_gpio, GPIO_LOW);
            if(ret) {
                COLOR_INFO("Config sleep_en_gpio gpio output direction fail.");
                goto PROBE_ERR;
            }
        }
    } else {
        hw_res->sleep_en_gpio = -EINVAL;
        COLOR_INFO("sleep_en_gpio gpio is invalid.");
        goto PROBE_ERR;
    }

    if (gpio_is_valid(hw_res->si_in_1_gpio)) {
        ret = devm_gpio_request(&pdev->dev, hw_res->si_in_1_gpio, "si_in_1_gpio");
        if(ret) {
            COLOR_INFO("request si_in_1_gpio fail.");
            goto PROBE_ERR;
        } else {
            ret = gpio_direction_output(hw_res->si_in_1_gpio, GPIO_LOW);
            if(ret) {
                COLOR_INFO("Config si_in_1_gpio gpio output direction fail.");
                goto PROBE_ERR;
            }
        }
    } else {
        hw_res->si_in_1_gpio = -EINVAL;
        COLOR_INFO("si_in_1_gpio gpio is invalid.");
        goto PROBE_ERR;
    }

    if (gpio_is_valid(hw_res->si_in_2_gpio)) {
        ret = devm_gpio_request(&pdev->dev, hw_res->si_in_2_gpio, "si_in_2_gpio");
        if(ret) {
            COLOR_INFO("request si_in_2_gpio fail.");
            goto PROBE_ERR;
        } else {
            ret = gpio_direction_output(hw_res->si_in_2_gpio, GPIO_LOW);
            if(ret) {
                COLOR_INFO("Config si_in_2_gpio gpio output direction fail.");
                goto PROBE_ERR;
            }
        }
    } else {
        hw_res->si_in_2_gpio = -EINVAL;
        COLOR_INFO("si_in_2_gpio gpio is invalid.");
        goto PROBE_ERR;
    }

    if (gpio_is_valid(hw_res->vm_enable_gpio)) {
        ret = devm_gpio_request(&pdev->dev, hw_res->vm_enable_gpio, "vm_enable_gpio");
        if(ret) {
            COLOR_INFO("request vm_enable_gpio fail.");
            goto PROBE_ERR;
        } else {
            ret = gpio_direction_output(hw_res->vm_enable_gpio, GPIO_LOW);
            if(ret) {
                COLOR_INFO("Config vm_enable_gpio gpio output direction fail.");
                goto PROBE_ERR;
            }
        }
    }

    /*Request thermal device*/
    cd->thermal_zone_device = thermal_zone_get_zone_by_name("shell_back");
    if (IS_ERR(cd->thermal_zone_device)) {
        ret = PTR_ERR(cd->thermal_zone_device);
        cd->thermal_zone_device = NULL;
        COLOR_INFO("fail to get shell_back thermal_zone_device: %d", ret);
    } else {
        COLOR_INFO("success get shell_back thermal_zone_device.");
    }

    /*setup color control device hrtimer*/
    hrtimer_init(&cd->hrtimer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
    cd->hrtimer.function = colorctrl_hrtimer_handler;

    cd->recharge_wq = create_singlethread_workqueue("recharge_wq");
    if (!cd->recharge_wq) {
        ret = -ENOMEM;
        goto PROBE_ERR;
    }

    INIT_WORK(&cd->recharge_work, colorctrl_recharge_work);

    cd->pdev = pdev;
    cd->dev = &pdev->dev;
    cd->color_status = UNKNOWN;
    cd->temp_status = ROOM_TEMP;
    cd->need_recharge = true;
    mutex_init(&cd->rw_lock);
    platform_set_drvdata(pdev, cd);

    ret = colorctrl_init_proc(cd);
    if (ret) {
        COLOR_INFO("creat color ctrl proc error.");
        goto PROBE_ERR;
    }

    COLOR_INFO("color ctrl device probe : normal end.");
    return ret;

PROBE_ERR:
    COLOR_INFO("color ctrl device probe error.");
    return ret;
}

static int colorctrl_remove(struct platform_device *dev)
{
    struct color_ctrl_device *cd = platform_get_drvdata(dev);

    COLOR_INFO("start remove the color ctrl platform dev.");

    if (cd) {
        proc_remove(cd->prEntry_cr);
        cd->prEntry_cr = NULL;
    }

    return 0;
}

static struct platform_driver colorctrl_driver = {
    .probe = colorctrl_probe,
    .remove = colorctrl_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name  = DRIVER_NAME,
        .of_match_table = colorctrl_match_table,
    },
};

static int __init colorctrl_driver_init(void)
{
    return platform_driver_register(&colorctrl_driver);
}

static void __exit colorctrl_driver_exit(void)
{
    return platform_driver_unregister(&colorctrl_driver);
}

late_initcall(colorctrl_driver_init);
module_exit(colorctrl_driver_exit);

//module_platform_driver(colorctrl_driver);

MODULE_DESCRIPTION("Color Ctrl Driver Module");
MODULE_AUTHOR("Zengpeng.Chen");
MODULE_LICENSE("GPL v2");
