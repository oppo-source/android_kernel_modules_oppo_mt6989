// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * leds-aw21024.c   aw21024 led module
 *
 * Copyright (c) 2020 Shanghai Awinic Technology Co., Ltd. All Rights Reserved
 *
 *  Author: Awinic
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/leds.h>
#include <soc/oplus/system/oplus_project.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <soc/oplus/boot/boot_mode.h>
/* copy mtk_boot_common.h */
#define KERNEL_POWER_OFF_CHARGING_BOOT 8
#define LOW_POWER_OFF_CHARGING_BOOT 9
#include "leds_aw21024.h"
//#include "leds_aw21024_reg.h"
#define DLED_NUM 2
#define DLED_RGB_NUM 3

static struct aw21024 *aw21024_g_chip[DLED_NUM][DLED_RGB_NUM] = {{NULL,NULL,NULL}, {NULL,NULL,NULL}};
struct aw21024 *aw21024_glo;
static int max_led = 0;

//static int aw21024_hw_enable(struct aw21024 *aw21024, bool flag);
//static int aw21024_led_init(struct aw21024 *aw21024);
static int aw21024_led_change_mode(struct aw21024 *led, enum AW2023_LED_MODE mode);
//static int l_type_change_mode(enum lights_types ltype, int val);

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW21024_I2C_NAME "aw21024_led"
#define AW21024_DRIVER_VERSION "V1.2.0"
#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1
#define AW_START_TO_BREATH 200

#define R_ISNK_ON_MASK					0x04
#define G_ISNK_ON_MASK					0x02
#define B_ISNK_ON_MASK					0x01

#define LED_SUPPORT_TYPE					"support"
// #define BLINK_USE_AW21024
/******************************************************
 *
 * led effect
 *
 ******************************************************/
/*#define AW21024_CFG_NAME_MAX		64*/

/*struct aw21024_cfg aw21024_cfg_array[] = {
	{aw21024_cfg_led_off, sizeof(aw21024_cfg_led_off)},
	{aw21024_all_leds_on, sizeof(aw21024_all_leds_on)},
	{aw21024_red_leds_on, sizeof(aw21024_red_leds_on)},
	{aw21024_green_leds_on, sizeof(aw21024_green_leds_on)},
	{aw21024_blue_leds_on, sizeof(aw21024_blue_leds_on)},
	{aw21024_breath_leds_on, sizeof(aw21024_breath_leds_on)}
};*/

/******************************************************
 *
 * aw21024 i2c write/read
 *
 ******************************************************/

static int
aw21024_i2c_write(struct aw21024 *aw21024, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw21024->i2c, reg_addr, reg_data);
		if (ret < 0)
			pr_err("%s: i2c_write cnt=%d error=%d reg_addr=0x%x reg_data=0x%x\n", __func__, cnt, ret, reg_addr, reg_data);
		else
			break;

		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}
	if (ret < 0)
	{
		dump_stack();
	}
	return ret;
}

static int
aw21024_i2c_read(struct aw21024 *aw21024, unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	if (aw21024 == NULL) {
		pr_err("%s: aw21024_i2c_read aw21024 is NULL \n", __func__);
		return 0;
	}
	
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw21024->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw21024_i2c_write_bits(struct aw21024 *aw21024,
				unsigned char reg_addr, unsigned int mask,
				unsigned char reg_data)
{
	unsigned char reg_val = 0;

	aw21024_i2c_read(aw21024, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	aw21024_i2c_write(aw21024, reg_addr, reg_val);

	return 0;
}

/*****************************************************
 *
 * aw21024 led effect cfg
 *
 *****************************************************/
/*static void aw21024_update_cfg_array(struct aw21024 *aw21024,
				     unsigned char *p_cfg_data,
				     unsigned int cfg_size)
{
	unsigned int i = 0;

	for (i = 0; i < cfg_size; i += 2)
		aw21024_i2c_write(aw21024, p_cfg_data[i], p_cfg_data[i + 1]);
}

static int aw21024_cfg_update_array(struct aw21024 *aw21024)
{
	aw21024_update_cfg_array(aw21024, aw21024_cfg_array[aw21024->effect].p,
				 aw21024_cfg_array[aw21024->effect].count);
	return 0;
}*/

/*****************************************************
 *
 * aw21024 led init
 *
 *****************************************************/
static int aw21024_current_conversion(struct aw21024 *aw21024, unsigned int current_data)
{
	if (aw21024->cdev.max_brightness == 255) {
		aw21024->conversion_led_current = current_data;
	} else if (aw21024->cdev.max_brightness == 1023) {
		if ((current_data >= 1) && (current_data <= 4)) {
			aw21024->conversion_led_current = 1;
		} else {
			aw21024->conversion_led_current = (current_data * 255)
						/ aw21024->cdev.max_brightness;
		}
	} else {
		if ((current_data >= 1) && (current_data <= 8)) {
			aw21024->conversion_led_current = 1;
		} else {
			aw21024->conversion_led_current = (current_data * 255)
						/ aw21024->cdev.max_brightness;
		}
	}
	return 0;
}

/*static int aw21024_brightness_conversion(struct aw21024 *aw21024, unsigned int brightness_data)
{
	if (aw21024->cdev.max_brightness == 255) {
		aw21024->cdev.brightness = brightness_data;
	} else if (aw21024->cdev.max_brightness == 1023) {
		if ((brightness_data >= 1) && (brightness_data <= 4)) {
			aw21024->cdev.brightness = 1;
		} else {
			aw21024->cdev.brightness = (brightness_data * 255)
						/ aw21024->cdev.max_brightness;
		}
	} else {
		if ((brightness_data >= 1) && (brightness_data <= 8)) {
			aw21024->cdev.brightness = 1;
		} else {
			aw21024->cdev.brightness = (brightness_data * 255)
						/ aw21024->cdev.max_brightness;
		}
	}
	return 0;
}*/

static int aw21024_chip_enable(struct aw21024 *aw21024, bool flag)
{
	if (flag) {
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CHIP_EN_CLOSE_MASK,
				       AW21024_BIT_CHIP_EN);
	} else {
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CHIP_EN_CLOSE_MASK,
				       AW21024_BIT_CHIP_CLOSE);
	}
	return 0;
}

static int aw21024_pwm_set(struct aw21024 *aw21024, unsigned int mode)
{
	switch (mode) {
	case AW21024_CLKPRQ_16MH:
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CLKPRQ_MASK,
				       AW21024_BIT_CLKPRQ_16MH);
		break;
	case AW21024_CLKPRQ_8MH:
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CLKPRQ_MASK,
				       AW21024_BIT_CLKPRQ_8MH);
		break;
	case AW21024_CLKPRQ_1MH:
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CLKPRQ_MASK,
				       AW21024_BIT_CLKPRQ_1MH);
		break;
	case AW21024_CLKPRQ_512KH:
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CLKPRQ_MASK,
				       AW21024_BIT_CLKPRQ_512KH);
		break;
	case AW21024_CLKPRQ_256KH:
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CLKPRQ_MASK,
				       AW21024_BIT_CLKPRQ_256KH);
		break;
	case AW21024_CLKPRQ_125KH:
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CLKPRQ_MASK,
				       AW21024_BIT_CLKPRQ_125KH);
		break;
	case AW21024_CLKPRQ_62KH:
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CLKPRQ_MASK,
				       AW21024_BIT_CLKPRQ_62KH);
		break;
	case AW21024_CLKPRQ_31KH:
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_CLKPRQ_MASK,
				       AW21024_BIT_CLKPRQ_31KH);
		break;
	default:
		break;
	}
	return 0;
}

static int aw21024_apse_set(struct aw21024 *aw21024, bool mode)
{
	if (mode) {
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_APSE_MASK,
				       AW21024_BIT_APSE_ENABLE);
	} else {
		aw21024_i2c_write_bits(aw21024,
				       AW21024_REG_GCR,
				       AW21024_BIT_APSE_MASK,
				       AW21024_BIT_APSE_DISABLE);
	}

	return 0;
}

static int aw21024_br_update(struct aw21024 *aw21024)
{
	aw21024_i2c_write(aw21024, AW21024_REG_UPDATE, 0x00);
	return 0;
}

static int aw21024_led_init(struct aw21024 *aw21024)
{
	aw21024_chip_enable(aw21024, true);
	usleep_range(200, 300);
	aw21024_pwm_set(aw21024, aw21024->clk_pwm);
	aw21024_apse_set(aw21024, aw21024->apse_mode);
	pr_info("before aw21024_current_conversion aw21024->dts_led_current is %d\n",
						aw21024->dts_led_current);
	aw21024_current_conversion(aw21024, aw21024->dts_led_current);
	pr_info("after aw21024_current_conversion aw21024->conversion_led_current is %d\n",
					aw21024->conversion_led_current);
	aw21024_i2c_write(aw21024, AW21024_REG_GCCR,
				aw21024->conversion_led_current);
	aw21024_i2c_write(aw21024, AW21024_REG_WBG, 0x55);

	return 0;
}

/*****************************************************
 *
 * aw21024 led hw reset
 *
 *****************************************************/
static int aw21024_hw_reset(struct aw21024 *aw21024)
{
	dev_err(aw21024->dev, "%s:  start\n", __func__);
	if (aw21024 && gpio_is_valid(aw21024->reset_gpio) && gpio_is_valid(aw21024->vbled_enable_gpio)) {
		gpio_set_value_cansleep(aw21024->reset_gpio, 0);
		usleep_range(2000, 4000);
		gpio_set_value_cansleep(aw21024->reset_gpio, 1);
		usleep_range(2000, 4000);
		gpio_set_value_cansleep(aw21024->vbled_enable_gpio, 1);
		aw21024_i2c_write(aw21024, AW21024_REG_RESET, 0x00);
		usleep_range(2000, 4000);
		dev_err(aw21024->dev, "%s:  finish\n", __func__);
	} else {
		dev_err(aw21024->dev, "%s:  failed\n", __func__);
	}
	aw21024_led_init(aw21024);
	aw21024->led_enable = true;
	return 0;
}

static int aw21024_hw_off(struct aw21024 *aw21024)
{
	dev_err(aw21024->dev, "%s:  start\n", __func__);
	if (aw21024 && gpio_is_valid(aw21024->reset_gpio) && gpio_is_valid(aw21024->vbled_enable_gpio)) {
		gpio_set_value_cansleep(aw21024->reset_gpio, 0);
		usleep_range(200, 400);
		gpio_set_value_cansleep(aw21024->vbled_enable_gpio, 0);
		usleep_range(200, 400);
		dev_err(aw21024->dev, "%s:  finish\n", __func__);
	} else {
		dev_err(aw21024->dev, "%s:  failed\n", __func__);
	}
	aw21024->led_enable = false;
	return 0;
}

static void aw21024_update(struct aw21024 *aw21024)
{
	aw21024_i2c_write(aw21024, AW21024_REG_UPDATE, AW21024_UPDATE_BR_SL);
}

void aw21024_global_set(struct aw21024 *aw21024)
{
	if (aw21024->light_sensor_state) {
		aw21024_i2c_write(aw21024, AW21024_REG_GCCR, aw21024->glo_current_max);
	}
	else {
		aw21024_i2c_write(aw21024, AW21024_REG_GCCR, aw21024->glo_current_min);
	}
}

void aw21024_current_set(struct aw21024 *aw21024)
{
	if (aw21024->light_sensor_state) {
		aw21024_i2c_write(aw21024, AW21024_REG_GCCR, aw21024->glo_current_min);
	}
	else {
		aw21024_i2c_write(aw21024, AW21024_REG_GCCR, aw21024->glo_current_max);
	}
}
/*********************************************************
 *
 * light effect
 *
 ********************************************************/
 /*初始化四颗灯，每科灯需要的结构体数据*/
void aw21024_rgb_multi_breath_init(const AW_MULTI_BREATH_DATA_STRUCT *data, effect_select_t effect)
{
	unsigned char i;

	aw21024_interface[effect].getBrightnessfunc = aw21024_get_breath_brightness_algo_func(BREATH_ALGO_GAMMA_CORRECTION);//leds_aw21024.h定义AW_COLORFUL_INTERFACE_STRUCT aw21024_interface；aw_breath_algorithm.h中定义了该函数getBrightnessfunc
	algo_data[effect].cur_frame = 0;//aw_lamp_interface.h中定义了algo_data
	algo_data[effect].total_frames = 20;
	for (i = 0; i < 3; i++) {
		algo_data[effect].data_start[i] = 0;
		algo_data[effect].data_end[i] = 0;
	}
	aw21024_interface[effect].p_algo_data = &algo_data[effect];
	AW_ERR("MTC_LOG: enter %s, RGB_NUM is %d\n", __func__, RGB_NUM);

	for (i = 0; i < RGB_NUM; i++) { //RGB_NUM is 4，表示4颗RGB灯
		colorful_cur_frame[effect][i] = 0;//leds_aw21024.h中定义了
		colorful_total_frames[effect][i] = 20;//leds_aw21024.h中定义了
		colorful_cur_color_index[effect][i] = 0;//leds_aw21024.h中定义了
		colorful_cur_phase[effect][i] = 0;//leds_aw21024.h中定义了
		colorful_phase_nums[effect][i] = 5;
		breath_cur_phase[effect][i] = 0;
		breath_phase_nums[effect][i] = 6;
		aw21024_algo_data[effect][i].cur_frame = 0;//leds_aw21024.h中定义ALGO_DATA_STRUCT aw21024_algo_data[RGB_NUM]，每一颗灯有有一个该变量
		aw21024_algo_data[effect][i].total_frames = (data[i].time[0] + data[i].frame_factor - 1) / data[i].frame_factor + 1;
		aw21024_algo_data[effect][i].data_start[0] = data[i].fadel[0].r;
		aw21024_algo_data[effect][i].data_end[0] = data[i].fadeh[0].r;
		aw21024_algo_data[effect][i].data_start[1] = data[i].fadel[0].g;
		aw21024_algo_data[effect][i].data_end[1] = data[i].fadeh[0].g;
		aw21024_algo_data[effect][i].data_start[2] = data[i].fadel[0].b;
		aw21024_algo_data[effect][i].data_end[2] = data[i].fadeh[0].b;
		source_color[effect][i].r = 0x00;
		source_color[effect][i].g = 0x00;
		source_color[effect][i].b = 0x00;
		destination_color[effect][i].r = 0x00;
		destination_color[effect][i].g = 0x00;
		destination_color[effect][i].b = 0x00;
		loop_end[effect][i] = 0;
		breath_cur_loop[effect][i]=0;
	}
}

void aw21024_update_frame_idx( AW_MULTI_BREATH_DATA_STRUCT *data, effect_select_t effect)
{
	unsigned char i;
	int update_frame_idx = 0;
    int index,dest_index;
    
	//AW_ERR("MTC_LOG: enter %s\n", __func__);
	for (i = 0; i < RGB_NUM; i++) {
		update_frame_idx = 1;
		if (loop_end[effect][i] == 1) //leds_aw21024.h中定义
			continue;

		aw21024_algo_data[effect][i].cur_frame++;
		if (aw21024_algo_data[effect][i].cur_frame >= aw21024_algo_data[effect][i].total_frames) {
			aw21024_algo_data[effect][i].cur_frame = 0;
			breath_cur_phase[effect][i]++;//leds_aw21024.h中定义了该变量
			if(data[i].color_nums == 5){
				if(breath_cur_phase[effect][i] == 6){
					if(0 == data[i].repeat_nums){
							breath_cur_phase[effect][i] = 0;
							breath_cur_loop[effect][i] = 0;
						}
						else if(breath_cur_loop[effect][i] < data[i].repeat_nums - 1){
							breath_cur_phase[effect][i] = 0;
							breath_cur_loop[effect][i]++;
						}
				}
			}else{
				if(breath_cur_phase[effect][i] == 5){
					if(0 == data[i].repeat_nums){
						breath_cur_phase[effect][i] = 1;
						breath_cur_loop[effect][i] = 0;//leds_aw21024.h中定义了该变量
					}
					else if(breath_cur_loop[effect][i] < data[i].repeat_nums - 1){
						breath_cur_phase[effect][i] = 1;
						breath_cur_loop[effect][i]++;
					}
				}
			}
			if (breath_cur_phase[effect][i] >= breath_phase_nums[effect][i]) {
				breath_cur_phase[effect][i] = 0;
				update_frame_idx = 0;
			}

			if (update_frame_idx) {
				aw21024_algo_data[effect][i].total_frames =
					(data[i].time[breath_cur_phase[effect][i]])/data[i].frame_factor + 1;
				if(aw21024_algo_data[effect][i].total_frames == 1){
					continue;
				}
				if (breath_cur_phase[effect][i] == 1) {
					aw21024_algo_data[effect][i].data_start[0] = data[i].fadel[0].r;
					aw21024_algo_data[effect][i].data_end[0] = data[i].fadeh[0].r;
					aw21024_algo_data[effect][i].data_start[1] = data[i].fadel[0].g;
					aw21024_algo_data[effect][i].data_end[1] = data[i].fadeh[0].g;
					aw21024_algo_data[effect][i].data_start[2] = data[i].fadel[0].b;
					aw21024_algo_data[effect][i].data_end[2] = data[i].fadeh[0].b;
				} else if (breath_cur_phase[effect][i] == 2) {
					aw21024_algo_data[effect][i].data_start[0] = data[i].fadeh[0].r;
					aw21024_algo_data[effect][i].data_end[0] = data[i].fadeh[0].r;
					aw21024_algo_data[effect][i].data_start[1] = data[i].fadeh[0].g;
					aw21024_algo_data[effect][i].data_end[1] = data[i].fadeh[0].g;
					aw21024_algo_data[effect][i].data_start[2] = data[i].fadeh[0].b;
					aw21024_algo_data[effect][i].data_end[2] = data[i].fadeh[0].b;
				} else if (breath_cur_phase[effect][i] == 3) {
					aw21024_algo_data[effect][i].data_start[0] = data[i].fadeh[0].r;
					aw21024_algo_data[effect][i].data_end[0] = data[i].fadel[0].r;
					aw21024_algo_data[effect][i].data_start[1] = data[i].fadeh[0].g;
					aw21024_algo_data[effect][i].data_end[1] = data[i].fadel[0].g;
					aw21024_algo_data[effect][i].data_start[2] = data[i].fadeh[0].b;
					aw21024_algo_data[effect][i].data_end[2] = data[i].fadel[0].b;
				} else {
					aw21024_algo_data[effect][i].data_start[0] = data[i].fadel[0].r;
					aw21024_algo_data[effect][i].data_end[0] = data[i].fadel[0].r;
					aw21024_algo_data[effect][i].data_start[1] = data[i].fadel[0].g;
					aw21024_algo_data[effect][i].data_end[1] = data[i].fadel[0].g;
					aw21024_algo_data[effect][i].data_start[2] = data[i].fadel[0].b;
					aw21024_algo_data[effect][i].data_end[2] = data[i].fadel[0].b;
				}
				/* breath_cur_phase[i]++; */
			} else {
				aw21024_algo_data[effect][i].cur_frame = 0;
				aw21024_algo_data[effect][i].total_frames = 1;
				aw21024_algo_data[effect][i].data_start[0] = 0;
				aw21024_algo_data[effect][i].data_end[0] = 0;
				aw21024_algo_data[effect][i].data_start[1] = 0;
				aw21024_algo_data[effect][i].data_end[1] = 0;
				aw21024_algo_data[effect][i].data_start[2] = 0;
				aw21024_algo_data[effect][i].data_end[2] = 0;
				loop_end[effect][i] = 1;
			}
		}

		if(data[i].color_nums == 1){
			source_color[effect][i].r = destination_color[effect][i].r;
			source_color[effect][i].g = destination_color[effect][i].g;
			source_color[effect][i].b = destination_color[effect][i].b;
			destination_color[effect][i].r = data[i].rgb_color_list[0].r;
			destination_color[effect][i].g = data[i].rgb_color_list[0].g;
			destination_color[effect][i].b = data[i].rgb_color_list[0].b;		
			
		}else if(data[i].color_nums == 9){
			/*source_color[i].r = data[i].rgb_color_list[breath_cur_loop[i]].r;
			source_color[i].g = data[i].rgb_color_list[breath_cur_loop[i]].g;
			source_color[i].b = data[i].rgb_color_list[breath_cur_loop[i]].b;
			destination_color[i].r = data[i].rgb_color_list[breath_cur_loop[i]+1].r;
			destination_color[i].g = data[i].rgb_color_list[breath_cur_loop[i]+1].g;
			destination_color[i].b = data[i].rgb_color_list[breath_cur_loop[i]+1].b;*/
			index = (i+breath_cur_loop[effect][i])% (RGB_NUM);
			dest_index = (index+1)%RGB_NUM;
			source_color[effect][i].r = data[i].rgb_color_list[index].r;
			source_color[effect][i].g = data[i].rgb_color_list[index].g;
			source_color[effect][i].b = data[i].rgb_color_list[index].b;
			destination_color[effect][i].r = data[i].rgb_color_list[dest_index].r;
			destination_color[effect][i].g = data[i].rgb_color_list[dest_index].g;
			destination_color[effect][i].b = data[i].rgb_color_list[dest_index].b;
		}else{
			source_color[effect][i].r = destination_color[effect][i].r;
			source_color[effect][i].g = destination_color[effect][i].g;
			source_color[effect][i].b = destination_color[effect][i].b;
			if(breath_cur_loop[effect][i] < data[i].color_nums)
			{
				destination_color[effect][i].r = data[i].rgb_color_list[breath_cur_loop[effect][i]].r;
				destination_color[effect][i].g = data[i].rgb_color_list[breath_cur_loop[effect][i]].g;
				destination_color[effect][i].b = data[i].rgb_color_list[breath_cur_loop[effect][i]].b;
			}else
			{
				destination_color[effect][i].r = data[i].rgb_color_list[breath_cur_loop[effect][i]%data[i].color_nums].r;
				destination_color[effect][i].g = data[i].rgb_color_list[breath_cur_loop[effect][i]%data[i].color_nums].g;
				destination_color[effect][i].b = data[i].rgb_color_list[breath_cur_loop[effect][i]%data[i].color_nums].b;
			}
			
		}

//		AW_ERR(" rgb = %d, ufi = %d, breath_cur_phase[%d] = %d, breath_cur_loop = %d, \n",i,update_frame_idx, i, breath_cur_phase[i], breath_cur_loop[i]);
//		AW_ERR(" rgb = %d, cur_frame = %d, total_frames = %d, data_start = %d, data_end= %d \n",i,aw21024_algo_data[i].cur_frame, aw21024_algo_data[i].total_frames, aw21024_algo_data[i].data_start, aw21024_algo_data[i].data_end);
		
	}
}

void aw21024_frame_display(effect_select_t effect)
{
	unsigned char i = 0;
	unsigned char brightness[3] = {0};
	//AW_ERR("MTC_LOG: enter %s\n", __func__);

	for (i = 0; i < RGB_NUM; i++) {
		aw21024_interface[effect].p_color_1 = &source_color[effect][i];////aw_lamp_interface.h中定义aw21024_interface,leds_aw21024_reg.h中定义soruce_color
		aw21024_interface[effect].p_color_2 = &destination_color[effect][i];//leds_aw21024.h中定义
		aw21024_interface[effect].cur_frame = aw21024_algo_data[effect][i].cur_frame;
		aw21024_interface[effect].total_frames = aw21024_algo_data[effect][i].total_frames;
		if(aw21024_interface[effect].total_frames > 1){
			aw21024_set_colorful_rgb_data(i, dim_data[effect], &aw21024_interface[effect]);//aw_lamp_interface.h中定义
			if (breath_cur_phase[effect][i] == 0)
			{
				brightness[0] = 0;
				brightness[1] = 0;
				brightness[2] = 0;
			}
			else {
				brightness[0] = aw21024_interface[effect].getBrightnessfunc(&aw21024_algo_data[effect][i],0);//返回一个start_index，貌似是初始亮度
				brightness[1] = aw21024_interface[effect].getBrightnessfunc(&aw21024_algo_data[effect][i],1);//返回一个start_index，貌似是初始亮度
				brightness[2] = aw21024_interface[effect].getBrightnessfunc(&aw21024_algo_data[effect][i],2);//返回一个start_index，貌似是初始亮度
			}
			aw21024_set_rgb_brightness(i, fade_data[effect], brightness);//aw_lamp_interface.h中定义
			//AW_ERR("MTC_LOG:aw21024_frame_display, rgb index is %d, dim_data clor is 0x%x 0x%x 0x%x, brightness is 0x%x 0x%x 0x%x\n", i, dim_data[effect][i*3], dim_data[effect][i*3 + 1], dim_data[effect][i*3 + 2], brightness[0], brightness[1], brightness[2]);
		}
	}
}

void aw21024_update_effect(struct aw21024 *aw21024,effect_select_t effect)
{
	unsigned char i = 0;

	//AW_ERR("MTC_LOG:pepare to aw21024_update\n");
	for (i = 0; i < RGB_NUM; i++) {
		/*aw21024_col_data[j * 3 + 0] = dim_data[i * 3 + 0];
		aw21024_br_data[j * 3 + 0] = fade_data[i * 3 + 0];
		aw21024_col_data[j * 3 + 1] = dim_data[i * 3 + 1];
		aw21024_br_data[j * 3 + 1] = fade_data[i * 3 + 1];
		aw21024_col_data[j * 3 + 2] = dim_data[i * 3 + 2];
		aw21024_br_data[j * 3 + 2] = fade_data[i * 3 + 2];*/
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 0], dim_data[effect][i * 3 + 0]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 1], fade_data[effect][i * 3 + 0]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 2], dim_data[effect][i * 3 + 1]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 3], fade_data[effect][i * 3 + 1]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 4], dim_data[effect][i * 3 + 2]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 5], fade_data[effect][i * 3 + 2]);
		//AW_ERR("MTC_LOG:aw21024_update_effect, rgb index is %d, dim_data clor is 0x%x 0x%x 0x%x, brightness is 0x%x 0x%x 0x%x\n", i, dim_data[effect][i*3], dim_data[effect][i*3 + 1], dim_data[effect][i*3 + 2], fade_data[effect][i * 3 + 0], fade_data[effect][i * 3 + 1], fade_data[effect][i * 3 + 2]);
	}
	/*
	for (i = 0,j=0; i < RGB_NUM; i+=2,j++)  {
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 1], aw21024_br_data[j * 3 + 0]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 3], aw21024_br_data[j * 3 + 1]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 5], aw21024_br_data[j * 3 + 2]);
	}

	for (i = 0,j=0; i < RGB_NUM; i+=2,j++) {
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 0], aw21024_col_data[j * 3 + 0]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 2], aw21024_col_data[j * 3 + 1]);
		aw21024_i2c_write(aw21024, aw21024_reg_map[i * 6 + 4], aw21024_col_data[j * 3 + 2]);
	}*/

	/*for (i = 1,j=0; i < RGB_NUM; i+=2,j++) {*/
/*		aw21024_col_data[j * 3 + 0] = dim_data[i * 3 + 0];
		aw21024_br_data[j * 3 + 0] = fade_data[i * 3 + 0];
		aw21024_col_data[j * 3 + 1] = dim_data[i * 3 + 1];
		aw21024_br_data[j * 3 + 1] = fade_data[i * 3 + 1];
		aw21024_col_data[j * 3 + 2] = dim_data[i * 3 + 2];
		aw21024_br_data[j * 3 + 2] = fade_data[i * 3 + 2
*/
		/*aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 0], dim_data[effect][i * 3 + 0]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 1], fade_data[effect][i * 3 + 0]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 2], dim_data[effect][i * 3 + 1]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 3], fade_data[effect][i * 3 + 1]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 4], dim_data[effect][i * 3 + 2]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 5], fade_data[effect][i * 3 + 2]);
	}*/
/*
	for (i = 1,j=0; i < RGB_NUM; i+=2,j++)  {
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 1], aw21024_br_data[j * 3 + 0]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 3], aw21024_br_data[j * 3 + 1]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 5], aw21024_br_data[j * 3 + 2]);
	}

	for (i = 1,j=0; i < RGB_NUM; i+=2,j++) {
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 0], aw21024_col_data[j * 3 + 0]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 2], aw21024_col_data[j * 3 + 1]);
		aw21024_i2c_write(aw21024_g_chip[1][0], aw21024_reg_map[i * 6 + 4], aw21024_col_data[j * 3 + 2]);
	}
*/
	aw21024_update(aw21024);
	//aw21024_update(aw21024_g_chip[1][0]);
}

int aw21024_start_next_effect( effect_select_t effect,struct aw21024 *aw21024){
		int i=0;
		unsigned char size = aw21024_cfg_array[effect].count/RGB_NUM;
		//AW_ERR("MTC_LOG: enter %s\n", __func__);
		for(i=0;i<RGB_NUM;i++){
			if(loop_end[effect][i]==0){
				break;
			}else{
				effect_stop_val[effect]++;
			}
		}
		if( effect_stop_val[effect] >= RGB_NUM && num[effect] < size){
			num[effect]++;
			if(num[effect] >= size){
				num[effect] = 0;
				return -1;
			}
			effect_data[effect] += RGB_NUM;
			aw21024_rgb_multi_breath_init(effect_data[effect],effect);
			aw21024_frame_display(effect);
			aw21024_update_effect(aw21024,effect);
		}
		effect_stop_val[effect] = 0;
		return 0;
}

void print_effectdata(effect_select_t effect)
{
	int i =0,j =0;
	#define BUF_SIZE 200
	char buf[BUF_SIZE];
	int len=0;
	AW_ERR("DebugLog: effectindex = %d  \n", effect);
	for(i=0;i<aw21024_cfg_array[effect].count;i++)
	{
		len=0;
		memset(buf,0,sizeof(buf));
		len += snprintf(buf + len, BUF_SIZE - len,"%d,%d,%d,", effect, i,aw21024_cfg_array[effect].p[i].frame_factor);
		for(j=0;j<6;j++)
		{
			len += snprintf(buf + len, BUF_SIZE - len, "%d,", aw21024_cfg_array[effect].p[i].time[j]);
		}
		len += snprintf(buf + len, BUF_SIZE - len, "%d,", aw21024_cfg_array[effect].p[i].repeat_nums);
		len += snprintf(buf + len, BUF_SIZE - len, "%d,", aw21024_cfg_array[effect].p[i].color_nums);
		for(j=0;j<aw21024_cfg_array[effect].p[i].color_nums;j++)
		{
			len += snprintf(buf + len, BUF_SIZE - len, "%d,", aw21024_cfg_array[effect].p[i].rgb_color_list[j].r);
			len += snprintf(buf + len, BUF_SIZE - len, "%d,", aw21024_cfg_array[effect].p[i].rgb_color_list[j].g);
			len += snprintf(buf + len, BUF_SIZE - len, "%d,", aw21024_cfg_array[effect].p[i].rgb_color_list[j].b);
		}
		len += snprintf(buf + len, BUF_SIZE - len, "%d,%d,%d,", aw21024_cfg_array[effect].p[i].fadeh[0].r, aw21024_cfg_array[effect].p[i].fadeh[0].g, aw21024_cfg_array[effect].p[i].fadeh[0].b);
		len += snprintf(buf + len, BUF_SIZE - len, "%d,%d,%d,", aw21024_cfg_array[effect].p[i].fadel[0].r, aw21024_cfg_array[effect].p[i].fadel[0].g, aw21024_cfg_array[effect].p[i].fadel[0].b);
		buf[--len] = 0x00;
		AW_ERR("DebugLog: %s  \n", buf);
	}
//	aw21024_cfg_array[effect].p[dataindex].time[i]
}

int check_effect_state(effect_select_t effect)
{
	switch(effect) {
		case INCALL_EFFECT:
			if(effect_state.state[AW21024_LED_INCALL_MODE] == 1){
				return 0;
			}
			if(effect_state.state[AW21024_LED_INCALL_MODE] == 0){
				return -1;
			}
			break;
		case POWERON_EFFECT:
			if(effect_state.state[AW21024_LED_POWERON_MODE] == 1){
				return 0;
			}
			if(effect_state.state[AW21024_LED_POWERON_MODE] == 0){
				return -1;
			}
			while(effect_state.state[AW21024_LED_POWERON_MODE] == 2)
			{
				usleep_range(100000, 120000);
				AW_ERR(" AW21024_LED_POWERON_MODE pause\n" );
				if(effect_state.state[AW21024_LED_CHARGE_MODE] == 1)
				{
					//TBD enable hw
					;
				}
			}
			break;
		case CHARGE_EFFECT:
			if(effect_state.state[AW21024_LED_CHARGE_MODE] == 1){
				return 0;
			}
			if(effect_state.state[AW21024_LED_CHARGE_MODE] == 0){
				return -1;
			}
			while(effect_state.state[AW21024_LED_CHARGE_MODE] == 2)
			{
				usleep_range(100000, 120000);
				AW_ERR(" AW21024_LED_CHARGE_MODE pause \n" );
				if(effect_state.state[AW21024_LED_CHARGE_MODE] == 1)
				{
					//TBD enable hw
					;
				}
			}
			return 0;
			break;
		case GAME_ENTER_EFFECT:
			if(effect_state.state[AW21024_LED_GAMEMODE] == 1){
				return 0;
			}
			if(effect_state.state[AW21024_LED_GAMEMODE] == 0){
				return -1;
			}
			while(effect_state.state[AW21024_LED_GAMEMODE] == 2)
			{
				usleep_range(100000, 120000);
				AW_ERR(" AW21024_LED_GAMEMODE pause\n" );
				if(effect_state.state[AW21024_LED_GAMEMODE] == 1)
				{
					//TBD enable hw
					;
				}
			}
			break;
		case NOTIFY_EFFECT:
			if(effect_state.state[AW21024_LED_NOTIFY_MODE] == 1){
				return 0;
			}
			if(effect_state.state[AW21024_LED_NOTIFY_MODE] == 0){
				return -1;
			}
			while(effect_state.state[AW21024_LED_NOTIFY_MODE] == 2)
			{
				usleep_range(100000, 120000);
				AW_ERR(" AW21024_LED_NOTIFY_MODE pause \n" );
				if(effect_state.state[AW21024_LED_NOTIFY_MODE] == 1)
				{
					//TBD enable hw
					;
				}
			}
			break;
		default:
			break;
	}
	return -1;
}

void run_alwayson_effect(struct aw21024 *aw21024){

	//struct aw21024 *aw21024_id1 = aw21024_g_chip[1][0];
	struct aw21024 *led = aw21024_g_chip[0][0];
	int i =0;
	//AW_ERR(" effect_state.data[AW21024_LED_NEW_ALWAYSON] = %d, last_run_effect = %d", effect_state.data[AW21024_LED_NEW_ALWAYSON], last_run_effect);
	if( (effect_state.data[AW21024_LED_NEW_ALWAYSON] == 1) || (last_run_effect != AW21024_LED_NEW_ALWAYSON)) {
		effect_state.data[AW21024_LED_NEW_ALWAYSON] = 0;
		for (i=0; i<3; i++)
		{
			AW_ERR(" new_always_on_color = %d,%d,%d,%d,%d,%d,%d,%d,%d",new_always_on_color[i][0],new_always_on_color[i][1],new_always_on_color[i][2],new_always_on_color[i][3],
			new_always_on_color[i][4],new_always_on_color[i][5],new_always_on_color[i][6],new_always_on_color[i][7],new_always_on_color[i][8]);
			AW_ERR(" frame entry \n");
		}

		//brightness_sbmd_setup(BR_RESOLUTION_8BIT , false);
		for(i=0;i<3;i++){
			aw21024_i2c_write(led, AW21024_REG_BR0 + i, new_always_on_color[i][8]);
			aw21024_i2c_write(led, AW21024_REG_BR3 + i, new_always_on_color[i][8]);
			aw21024_i2c_write(led, AW21024_REG_BR6 + i, new_always_on_color[i][8]);
			aw21024_i2c_write(led, AW21024_REG_BR9 + i, new_always_on_color[i][8]);
			aw21024_i2c_write(led, AW21024_REG_BR12 + i, new_always_on_color[i][8]);
			aw21024_i2c_write(led, AW21024_REG_BR15 + i, new_always_on_color[i][8]);
			aw21024_i2c_write(led, AW21024_REG_BR18 + i, new_always_on_color[i][8]);
			aw21024_i2c_write(led, AW21024_REG_BR21 + i, new_always_on_color[i][8]);

			/*aw21024_i2c_write(aw21024_id1, AW21024_REG_BR00L + 2*i, new_always_on_color[i][8]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_BR03L + 2*i, new_always_on_color[i][8]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_BR06L + 2*i, new_always_on_color[i][8]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_BR09L + 2*i, new_always_on_color[i][8]);*/

			aw21024_i2c_write(led, AW21024_REG_COL0 + i, new_always_on_color[i][0]);
			aw21024_i2c_write(led, AW21024_REG_COL3 + i, new_always_on_color[i][1]);
			aw21024_i2c_write(led, AW21024_REG_COL6 + i, new_always_on_color[i][2]);
			aw21024_i2c_write(led, AW21024_REG_COL9 + i, new_always_on_color[i][3]);
			aw21024_i2c_write(led, AW21024_REG_COL12 + i, new_always_on_color[i][4]);
			aw21024_i2c_write(led, AW21024_REG_COL15 + i, new_always_on_color[i][5]);
			aw21024_i2c_write(led, AW21024_REG_COL18 + i, new_always_on_color[i][6]);
			aw21024_i2c_write(led, AW21024_REG_COL21 + i, new_always_on_color[i][7]);

			/*aw21024_i2c_write(aw21024_id1, AW21024_REG_COL0 + i, new_always_on_color[i][4]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_COL3 + i, new_always_on_color[i][5]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_COL6 + i, new_always_on_color[i][6]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_COL9 + i, new_always_on_color[i][7]);*/
		}

		aw21024_update(led);
		//aw21024_update(aw21024_id1);
	}
}

void run_music_effect(struct aw21024 *aw21024){

	//struct aw21024 *aw21024_id1 = aw21024_g_chip[1][0];
	struct aw21024 *led = aw21024_g_chip[0][0];
	int i =0,brightness[3];
	//AW_ERR(" effect_state.data[AW21024_LED_MUSICMODE] = %d, last_run_effect = %d", effect_state.data[AW21024_LED_MUSICMODE], last_run_effect);
	if((effect_state.data[AW21024_LED_MUSICMODE] == 1) || (last_run_effect != AW21024_LED_MUSICMODE)) {
		effect_state.data[AW21024_LED_MUSICMODE] = 0;
		for (i=0; i<3; i++)
		{
			AW_ERR(" music_color = %d,%d,%d,%d,%d,%d,%d,%d,%d",music_color[i][0],music_color[i][1],music_color[i][2],music_color[i][3],
			music_color[i][4],music_color[i][5],music_color[i][6],music_color[i][7],music_color[i][8]);
			if (music_color[i][8] > 16) {
				brightness[i] = (music_color[i][8] * music_color[i][8]) / 16;
			}
			else {
				brightness[i] = music_color[i][8];
			}
		}
		AW_ERR(" frame entry \n");
		//brightness_sbmd_setup(BR_RESOLUTION_9_AND_3_BIT , false);
		for(i=0;i<3;i++){
			//aw21024_i2c_write(led, AW21024_REG_BR00H + 2 * i, 0xff & (brightness[i] >> 8));
			aw21024_i2c_write(led, AW21024_REG_BR0 +  i, 0xff & brightness[i]);
			//aw21024_i2c_write(led, AW21024_REG_BR03H +  i, 0xff & (brightness[i] >> 8));
			aw21024_i2c_write(led, AW21024_REG_BR3 +  i, 0xff & brightness[i]);
			//aw21024_i2c_write(led, AW21024_REG_BR06H +  i, 0xff & (brightness[i] >> 8));
			aw21024_i2c_write(led, AW21024_REG_BR6 +  i, 0xff & brightness[i]);
			//aw21024_i2c_write(led, AW21024_REG_BR09H +  i, 0xff & (brightness[i] >> 8));
			aw21024_i2c_write(led, AW21024_REG_BR9 +  i, 0xff & brightness[i]);
			//aw21024_i2c_write(led, AW21024_REG_BR00H +  i, 0xff & (brightness[i] >> 8));
			aw21024_i2c_write(led, AW21024_REG_BR12 +  i, 0xff & brightness[i]);
			//aw21024_i2c_write(led, AW21024_REG_BR03H +  i, 0xff & (brightness[i] >> 8));
			aw21024_i2c_write(led, AW21024_REG_BR15 +  i, 0xff & brightness[i]);
			//aw21024_i2c_write(led, AW21024_REG_BR06H +  i, 0xff & (brightness[i] >> 8));
			aw21024_i2c_write(led, AW21024_REG_BR18 +  i, 0xff & brightness[i]);
			//aw21024_i2c_write(led, AW21024_REG_BR09H +  i, 0xff & (brightness[i] >> 8));
			aw21024_i2c_write(led, AW21024_REG_BR21 +  i, 0xff & brightness[i]);
			aw21024_i2c_write(led, AW21024_REG_PATCFG, 0x01);

				/*aw21024_i2c_write(aw21024_id1, AW21024_REG_BR00H + 2 * i, 0xff & (brightness[i] >> 8));
				aw21024_i2c_write(aw21024_id1, AW21024_REG_BR00L + 2 * i, 0xff & brightness[i]);
				aw21024_i2c_write(aw21024_id1, AW21024_REG_BR03H + 2 * i, 0xff & (brightness[i] >> 8));
				aw21024_i2c_write(aw21024_id1, AW21024_REG_BR03L + 2 * i, 0xff & brightness[i]);
				aw21024_i2c_write(aw21024_id1, AW21024_REG_BR06H + 2 * i, 0xff & (brightness[i] >> 8));
				aw21024_i2c_write(aw21024_id1, AW21024_REG_BR06L + 2 * i, 0xff & brightness[i]);
				aw21024_i2c_write(aw21024_id1, AW21024_REG_BR09H + 2 * i, 0xff & (brightness[i] >> 8));
				aw21024_i2c_write(aw21024_id1, AW21024_REG_BR09L + 2 * i, 0xff & brightness[i]);
				aw21024_i2c_write(aw21024_id1, AW21024_REG_PATCFG, 0x01);*/

			aw21024_i2c_write(led, AW21024_REG_COL0 + i, music_color[i][0]);
			aw21024_i2c_write(led, AW21024_REG_COL3 + i, music_color[i][1]);
			aw21024_i2c_write(led, AW21024_REG_COL6 + i, music_color[i][2]);
			aw21024_i2c_write(led, AW21024_REG_COL9 + i, music_color[i][3]);
			aw21024_i2c_write(led, AW21024_REG_COL12 + i, music_color[i][4]);
			aw21024_i2c_write(led, AW21024_REG_COL15 + i, music_color[i][5]);
			aw21024_i2c_write(led, AW21024_REG_COL18 + i, music_color[i][6]);
			aw21024_i2c_write(led, AW21024_REG_COL21 + i, music_color[i][7]);
			aw21024_i2c_write(led, AW21024_REG_GCFG1, 0x10);
			aw21024_i2c_write(led, AW21024_REG_GCFG0, 0x00);
			/*aw21024_i2c_write(aw21024_id1, AW21024_REG_COL0 + i, music_color[i][4]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_COL3 + i, music_color[i][5]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_COL6 + i, music_color[i][6]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_COL9 + i, music_color[i][7]);
			aw21024_i2c_write(aw21024_id1, AW21024_REG_GCFG, 0x40);*/
		}
		aw21024_update(led);
		//aw21024_update(aw21024_id1);
	}
}

void run_notify_effect_autonomous(struct aw21024 *aw21024){

	//struct aw21024 *aw21024_id1 = aw21024_g_chip[1][0];
	struct aw21024 *led = aw21024_g_chip[0][0];
	int i=0;

	/*int notify_color[3];
	notify_color[0] = rgb_notify_list[0].r;
	notify_color[1] = rgb_notify_list[0].g;
	notify_color[2] = rgb_notify_list[0].b;*/
	//AW_ERR(" effect_state.data[AW21024_LED_MUSICMODE] = %d, last_run_effect = %d", effect_state.data[AW21024_LED_MUSICMODE], last_run_effect);
	if((effect_state.data[AW21024_LED_NOTIFY_MODE] == 1) || (last_run_effect != AW21024_LED_NOTIFY_MODE)) {
		effect_state.data[AW21024_LED_NOTIFY_MODE] = 0;
		AW_ERR(" entry \n");
		//brightness_sbmd_setup(BR_RESOLUTION_8BIT , true);

/*		aw21024_i2c_write(led, AW21024_REG_GSLR, rgb_notify_list[0].r);
		aw21024_i2c_write(led, AW21024_REG_GSLG, rgb_notify_list[0].g);
		aw21024_i2c_write(led, AW21024_REG_GSLB, rgb_notify_list[0].b);
*/
        /*for(i=0;i<3;i++) {
            aw21024_i2c_write(led, AW21024_REG_COL0 + i, notify_color[i]);
			aw21024_i2c_write(led, AW21024_REG_COL3 + i, notify_color[i]);
			aw21024_i2c_write(led, AW21024_REG_COL6 + i, notify_color[i]);
			aw21024_i2c_write(led, AW21024_REG_COL9 + i, notify_color[i]);
			aw21024_i2c_write(led, AW21024_REG_COL12 + i, notify_color[i]);
			aw21024_i2c_write(led, AW21024_REG_COL15 + i, notify_color[i]);
			aw21024_i2c_write(led, AW21024_REG_COL18 + i, notify_color[i]);
			aw21024_i2c_write(led, AW21024_REG_COL21 + i, notify_color[i]);
        }*/
		/*only four RGBs is set to blink in notify mode*/
		for(i=0;i<4;i++) {
			aw21024_i2c_write(led, AW21024_REG_COL0 + i * 3, rgb_notify_list[i].r);
			aw21024_i2c_write(led, AW21024_REG_COL0 + i * 3 + 1, rgb_notify_list[i].g);
			aw21024_i2c_write(led, AW21024_REG_COL0 + i * 3 + 2, rgb_notify_list[i].b);
		}

		aw21024_i2c_write(led, AW21024_REG_FADEH, br_notify_fadeh[0].r | br_notify_fadeh[0].g | br_notify_fadeh[0].b);
		aw21024_i2c_write(led, AW21024_REG_FADEL, 0x00);

		aw21024_i2c_write(led, AW21024_REG_GCFG0, 0xFF);
		aw21024_i2c_write(led, AW21024_REG_GCFG1, 0x10);
		aw21024_i2c_write(led, AW21024_REG_PATCFG, 0x03);
		aw21024_i2c_write(led, AW21024_REG_PATT0 , notify_breath_cycle[0]);
		aw21024_i2c_write(led, AW21024_REG_PATT1 , notify_breath_cycle[1]);

		aw21024_update(led);

		/*if (led->pdata->led->led_groups_num == 8 && aw21024_id1 != NULL) {
			for(i=0;i<12;i++)
			{
				aw21024_i2c_write(aw21024_id1, AW21024_REG_COL0+i, 0x00);
			}
			aw21024_update(aw21024_id1);
		}*/

		aw21024_i2c_write(led, AW21024_REG_PATGO, 0x01);

/*
		if (led->pdata->led->led_groups_num == 8 && aw21024_id1 != NULL) {
			aw21024_i2c_write(aw21024_id1, AW21024_REG_PATGO, 0x01);
		}
*/

	}
}

void initiate_effect(struct aw21024 *aw21024, effect_select_t effect) {
	print_effectdata(effect);
	effect_data[effect] = aw21024_cfg_array[effect].p;
	effect_stop_val[effect] = 0;
	num[effect] = 0;
	aw21024_rgb_multi_breath_init(effect_data[effect],effect);
	aw21024_frame_display(effect);
	aw21024_update_effect(aw21024,effect);
}

int update_next_frame(struct aw21024 *aw21024, effect_select_t effect){
		//AW_ERR("MTC_LOG: while %s\n", __func__);
		int ret =0;
		aw21024_update_frame_idx(effect_data[effect], effect);

		aw21024_frame_display(effect);

		aw21024_update_effect(aw21024, effect);

		ret = aw21024_start_next_effect(effect,aw21024);

		if(ret != 0){
			AW_ERR("exit2 \n");
			return 1;
		}
		return 0;
}

void run_poweron_effect(struct aw21024 *aw21024){
	int ret=0;
	//brightness_sbmd_setup(BR_RESOLUTION_8BIT , false);
	//effect_state.state  = 0 stopped, 1 initial, 2 running, 3 pause,
	if (effect_state.state[AW21024_LED_POWERON_MODE] == 1){
		AW_ERR(" AW21024_LED_POWERON_MODE enter \n" );
		initiate_effect(aw21024, POWERON_EFFECT);
		if(effect_state.state[AW21024_LED_POWERON_MODE] != 0) // TBD protect by lock for syncronization
			effect_state.state[AW21024_LED_POWERON_MODE] = 2;
	}
	if(effect_state.state[AW21024_LED_POWERON_MODE] == 2){
		ret = update_next_frame(aw21024, POWERON_EFFECT);
		if(ret == 1) // current effect cycle is completed
		{
			effect_state.state[AW21024_LED_POWERON_MODE] = 0;
		}
	}
}

void run_incall_effect(struct aw21024 *aw21024){
	int ret=0;
	//brightness_sbmd_setup(BR_RESOLUTION_8BIT, false);
	//effect_state.state  = 0 stopped, 1 initial, 2 running, 3 pause,
	if (effect_state.state[AW21024_LED_INCALL_MODE] == 1){
		AW_ERR(" AW21024_LED_INCALL_MODE enter \n" );
		initiate_effect(aw21024, INCALL_EFFECT);
		if(effect_state.state[AW21024_LED_INCALL_MODE] != 0) // TBD protect by lock for syncronization
			effect_state.state[AW21024_LED_INCALL_MODE] = 2;
	}
	if(effect_state.state[AW21024_LED_INCALL_MODE] == 2){
		ret = update_next_frame(aw21024, INCALL_EFFECT);
		if(ret == 1) // current effect cycle is completed
		{
			if(effect_state.state[AW21024_LED_INCALL_MODE] != 0) // if effect is not stopped then restart
				effect_state.state[AW21024_LED_INCALL_MODE] = 1;
		}
	}
}

void run_game_effect(struct aw21024 *aw21024){
	int ret=0;
	//brightness_sbmd_setup(BR_RESOLUTION_8BIT, false);
	//effect_state.state  = 0 stopped, 1 initial, 2 running, 3 pause,
	if (effect_state.state[AW21024_LED_GAMEMODE] == 1){
		AW_ERR(" AW21024_LED_GAMEMODE enter \n" );
		initiate_effect(aw21024, GAME_ENTER_EFFECT);
		if(effect_state.state[AW21024_LED_GAMEMODE] != 0) // TBD protect by lock for syncronization
			effect_state.state[AW21024_LED_GAMEMODE] = 2;
	}
	if(effect_state.state[AW21024_LED_GAMEMODE] == 2){
		ret = update_next_frame(aw21024, GAME_ENTER_EFFECT);
		if(ret == 1) // current effect cycle is completed
		{
			effect_state.state[AW21024_LED_GAMEMODE] = 0;
		}
	}
}

void run_notify_effect(struct aw21024 *aw21024){
	int ret=0;
	//brightness_sbmd_setup(BR_RESOLUTION_8BIT , false);
	//effect_state.state  = 0 stopped, 1 initial, 2 running, 3 pause,
	if (effect_state.state[AW21024_LED_NOTIFY_MODE] == 1){
		AW_ERR(" AW21024_LED_NOTIFY_MODE enter \n" );
		initiate_effect(aw21024, NOTIFY_EFFECT);
		if(effect_state.state[AW21024_LED_NOTIFY_MODE] != 0) // TBD protect by lock for syncronization
			effect_state.state[AW21024_LED_NOTIFY_MODE] = 2;
	}
	if(effect_state.state[AW21024_LED_NOTIFY_MODE] == 2){
		ret = update_next_frame(aw21024, NOTIFY_EFFECT);
		if(ret == 1) // current effect cycle is completed
		{
			if(effect_state.state[AW21024_LED_NOTIFY_MODE] != 0) // if effect is not stopped then restart
				effect_state.state[AW21024_LED_NOTIFY_MODE] = 1;
		}
	}
}

void run_charger_effect(struct aw21024 *aw21024){
	int ret=0;
	//brightness_sbmd_setup(BR_RESOLUTION_8BIT, false);
	//effect_state.state  = 0 disable, 1 init, 2 running, 3 pause, 4 second stage init (for charger),5 second stage running, 6 100% recover  stage (for charger)
	if (effect_state.state[AW21024_LED_CHARGE_MODE] == 1){
		AW_ERR(" AW21024_LED_CHARGE_MODE enter \n" );
		initiate_effect(aw21024, CHARGE_EFFECT);
		if(effect_state.state[AW21024_LED_CHARGE_MODE] != 0) // TBD protect by lock for syncronization
			effect_state.state[AW21024_LED_CHARGE_MODE] = 2;
	}
	if(effect_state.state[AW21024_LED_CHARGE_MODE] == 2){
		ret = update_next_frame(aw21024, CHARGE_EFFECT);
		if(ret == 1) // current effect cycle is completed
		{
			if(effect_state.state[AW21024_LED_CHARGE_MODE] != 0)
			{
				effect_state.state[AW21024_LED_CHARGE_MODE] = 4;
				blevel = -1;
			}
		}
	}
	if(effect_state.state[AW21024_LED_CHARGE_MODE] == 4){
		AW_ERR(" AW21024_LED_CHARGE_MODE stage 2 \n" );
		charge_ongoing_frame = 0;
		initiate_effect(aw21024, CHARGE_STAGE2_EFFECT);
		if(effect_state.state[AW21024_LED_CHARGE_MODE] != 0) // TBD protect by lock for syncronization
			effect_state.state[AW21024_LED_CHARGE_MODE] = 5;
	}
	if(effect_state.state[AW21024_LED_CHARGE_MODE] == 5){
		if(charge_ongoing_frame < charge_frame_map[effect_state.data[AW21024_LED_CHARGE_MODE]] ) {
			charge_ongoing_frame++;
			ret = update_next_frame(aw21024, CHARGE_STAGE2_EFFECT);
			if(ret == 1) // current effect cycle is completed
			{
				effect_state.state[AW21024_LED_CHARGE_MODE] = 0;
				charge_ongoing_frame =0;
			}
		}
		else if(last_run_effect != AW21024_LED_CHARGE_MODE) {
			aw21024_update_effect(aw21024, CHARGE_STAGE2_EFFECT);
		}
	}

}

void reset_led(void){

	struct aw21024 *led = aw21024_g_chip[0][0];
/*
	struct aw21024 *aw21024_id1 = aw21024_g_chip[1][0];
	aw21024_i2c_write(led, AW21024_REG_FADEH, 0x00);
	aw21024_i2c_write(led, AW21024_REG_FADEL, 0xff);
	aw21024_i2c_write(led, AW21024_REG_PATCFG, 0x00);
	aw21024_i2c_write(led, AW21024_REG_PATGO, 0x00);
	if (led->pdata->led->led_groups_num == 8 && aw21024_id1 != NULL) {
		aw21024_i2c_write(aw21024_id1, AW21024_REG_FADEH, 0x00);
		aw21024_i2c_write(aw21024_id1, AW21024_REG_FADEL, 0xff);
		aw21024_i2c_write(aw21024_id1, AW21024_REG_PATCFG, 0x00);
		aw21024_i2c_write(aw21024_id1, AW21024_REG_PATGO, 0x00);
	}
*/
	AW_LOG("enter");
	//led->pdata->led->power_change_state |= (1 << 0);
	mutex_lock(&led->pdata->led->lock);
	if (aw21024_hw_off(led->pdata->led)) {
		AW_LOG("aw21024 hw disable failed");
	}
	usleep_range(10000,10500);

	if (aw21024_hw_reset(led->pdata->led)) {
		AW_LOG("aw21024 hw enable failed");
	}
	mutex_unlock(&led->pdata->led->lock);
	//led->pdata->led->power_change_state &= ~(1 << 0);
}

void run_effect(struct aw21024 *aw21024)
{
	while (1)
	{
		if(effect_state.state[AW21024_LED_POWERON_MODE] != 0) {
			run_poweron_effect(aw21024);
			last_run_effect = AW21024_LED_POWERON_MODE;
			usleep_range(20000,20500);
			continue;
		}
		else if (effect_state.state[AW21024_LED_NEW_ALWAYSON] != 0){
			run_alwayson_effect(aw21024);
			last_run_effect = AW21024_LED_NEW_ALWAYSON;
			usleep_range(20000,20500);
			continue;
		}
		else if (effect_state.state[AW21024_LED_INCALL_MODE] != 0){
			if(last_run_effect == AW21024_LED_NOTIFY_MODE ){
				reset_led();
			}
			run_incall_effect(aw21024);
			last_run_effect = AW21024_LED_INCALL_MODE;
			usleep_range(6500,7000);
			continue;
		}
		else if (effect_state.state[AW21024_LED_MUSICMODE] != 0){
			run_music_effect(aw21024);
			last_run_effect = AW21024_LED_MUSICMODE;
			usleep_range(3000,5000);
			continue;
		}
		else if(effect_state.state[AW21024_LED_GAMEMODE] != 0) {
			run_game_effect(aw21024);
			last_run_effect = AW21024_LED_GAMEMODE;
			usleep_range(4500,5000);
			continue;
		}
		else if(effect_state.state[AW21024_LED_NOTIFY_MODE] != 0) {
			run_notify_effect_autonomous(aw21024);
			last_run_effect = AW21024_LED_NOTIFY_MODE;
			usleep_range(20000,20500);
			continue;
		}
		else if(effect_state.state[AW21024_LED_CHARGE_MODE] != 0) {
			if(last_run_effect == AW21024_LED_NOTIFY_MODE ){
				reset_led();
			}
			run_charger_effect(aw21024);
			last_run_effect = AW21024_LED_CHARGE_MODE;
			usleep_range(20000,20500);
			continue;
		}
		break; // no effect running
	}
}

/*****************************************************
 *
 * aw21024 led brightness
 *
 *****************************************************/
 static void aw21024_brightness(struct aw21024 *led)
{
	int i = 0;
	int led_brightness = 0;
	AW_LOG("id = %d brightness = %d\n", led->id, led->cdev.brightness);

	switch(led->id) {
	case AW21024_LED_RED:
		if (led->cdev.brightness > 0)
			led->pdata->led->rgb_isnk_on |= R_ISNK_ON_MASK;
		else
			led->pdata->led->rgb_isnk_on &= ~R_ISNK_ON_MASK;
		break;
	case AW21024_LED_GREEN:
		if (led->cdev.brightness > 0)
			led->pdata->led->rgb_isnk_on |= G_ISNK_ON_MASK;
		else
			led->pdata->led->rgb_isnk_on &= ~G_ISNK_ON_MASK;
		break;
	case AW21024_LED_BLUE:
		if (led->cdev.brightness > 0)
			led->pdata->led->rgb_isnk_on |= B_ISNK_ON_MASK;
		else
			led->pdata->led->rgb_isnk_on &= ~B_ISNK_ON_MASK;
		break;
	}
	AW_LOG("rgb_isnk_on = 0x%x\n", led->pdata->led->rgb_isnk_on);
	if (led->cdev.brightness > 0) {
		mutex_lock(&led->pdata->led->lock);
		if (!led->pdata->led->led_enable && led->pdata->led->rgb_isnk_on) {
			if (aw21024_hw_reset(led->pdata->led)) {
				AW_LOG("aw21024 hw enable failed");
			}
		}
		mutex_unlock(&led->pdata->led->lock);
	} else {
		mutex_lock(&led->pdata->led->lock);
		if (led->pdata->led->led_enable && !(led->pdata->led->rgb_isnk_on)) {
			if (aw21024_hw_off(led->pdata->led)) {
				AW_LOG("aw21024 hw disable failed");
			}
		}
		mutex_unlock(&led->pdata->led->lock);
	}

	if (led->cdev.brightness == 0) {
		usleep_range(40000, 40500);
	}

	if (!led->pdata->led->led_enable) {
		AW_LOG("aw21024 hw is disabled");
		return;
	}

	switch(led->pdata->led_mode) {
		case AW21024_LED_INCALL_MODE:
		case AW21024_LED_NOTIFY_MODE:
		case AW21024_LED_CHARGE_MODE:
		case AW21024_LED_POWERON_MODE:
		case AW21024_LED_GAMEMODE:
		case AW21024_LED_NEW_ALWAYSON:
		case AW21024_LED_MUSICMODE:
			AW_ERR(" initial effect mode %d enter \n",led->pdata->led_mode );
			/*light effect*/
			run_effect(led);
			AW_ERR(" effect mode %d exit \n", led->pdata->led_mode );
			break;
		default:
			break;
	}
	//aw21024_i2c_write(led, AW21024_REG_COL0 + led->id, led->cdev.brightness);
	//aw21024_i2c_write(led, AW21024_REG_COL3 + led->id, led->cdev.brightness);
	//aw21024_i2c_write(led, AW21024_REG_GCFG, 0x47);

	if (led->id == 0) {
		led_brightness = (led->cdev.brightness / 3);
		AW_LOG("id = %d set brightness = %d\n", led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL0 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL3 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL6 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL9 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL12 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL15 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL18 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL21 + led->id, led_brightness);
	} else if (led->id == 1) {
		led_brightness = (led->cdev.brightness / 3);
		AW_LOG("id = %d set brightness = %d\n", led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL0 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL3 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL6 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL9 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL12 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL15 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL18 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL21 + led->id, led_brightness);
	} else if (led->id == 2) {
		led_brightness = (led->cdev.brightness / 3);
		AW_LOG("id = %d set brightness = %d\n", led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL0 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL3 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL6 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL9 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL12 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL15 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL18 + led->id, led_brightness);
		aw21024_i2c_write(led, AW21024_REG_COL21 + led->id, led_brightness);
	}
	aw21024_i2c_write(led, AW21024_REG_GCFG0, 0xff);
	aw21024_i2c_write(led, AW21024_REG_GCFG1, 0x10);



	/*aw21024 led cc mode set up*/
	if (led->pdata->led_mode == AW21024_LED_CCMODE) {
		aw21024_i2c_write(led, AW21024_REG_FADEH, 0x00);
		aw21024_i2c_write(led, AW21024_REG_FADEL, 0xff);
		aw21024_i2c_write(led, AW21024_REG_PATCFG, 0x00);
	}

	/*aw21024 led breath set up*/
	if (led->pdata->led_mode == AW21024_LED_BREATHMODE) {
		aw21024_i2c_write(led, AW21024_REG_PATT0 ,
				(led->pdata->rise_time_ms << 4 | led->pdata->hold_time_ms));
		aw21024_i2c_write(led, AW21024_REG_PATT1 ,
				(led->pdata->fall_time_ms << 4 | led->pdata->off_time_ms));

		aw21024_i2c_write(led, AW21024_REG_FADEH, 0xff);
		aw21024_i2c_write(led, AW21024_REG_FADEL, 0x00);
		aw21024_i2c_write(led, AW21024_REG_PATCFG, 0x03);
		for (i = 0; i < max_led; i++)
			cancel_delayed_work(&(&aw21024_glo[i])->breath_work);

		schedule_delayed_work(&led->breath_work, msecs_to_jiffies(AW_START_TO_BREATH));
	}

	/*aw21024 led blink time*/
	if (led->pdata->led_mode == AW21024_LED_BLINKMODE) {
		#ifdef BLINK_USE_AW21024
			aw21024_i2c_write(led, AW21024_REG_PATT0 , 0x04);
			aw21024_i2c_write(led, AW21024_REG_PATT1 , 0x04);

			aw21024_i2c_write(led, AW21024_REG_FADEH, 0xff);
			aw21024_i2c_write(led, AW21024_REG_FADEL, 0x00);

			aw21024_i2c_write(led, AW21024_REG_PATCFG, 0x03);
			//if(led->id == 2)
				//aw21024_i2c_write(led, AW21024_REG_PATGO, 0x01);
			for (i = 0; i < max_led; i++)
			cancel_delayed_work(&(&aw21024_glo[i])->breath_work);

		schedule_delayed_work(&led->breath_work, msecs_to_jiffies(AW_START_TO_BREATH));
		#else
			aw21024_i2c_write(led, AW21024_REG_FADEH, 0x00);
			aw21024_i2c_write(led, AW21024_REG_FADEL, 0xff);
			aw21024_i2c_write(led, AW21024_REG_PATCFG, 0x00);
		#endif
	}

	/* update */
	aw21024_update(led);

	AW_LOG("%s:  brightness[%d]=%x led_brightness=%x led_mode[%d]=%x \n",__func__,led->id,led->cdev.brightness,led_brightness,led->id,led->pdata->led_mode);
}

static void aw21024_breath_func(struct work_struct *work)
{
	struct aw21024 *led = container_of(work, struct aw21024, breath_work.work);
	aw21024_i2c_write(led, AW21024_REG_PATGO, 0x01);
}
static void aw21024_brightness_work(struct work_struct *work)
{
	struct aw21024 *aw21024 = container_of(work, struct aw21024, brightness_work);
	aw21024_brightness(aw21024);
	/*if (aw21024->cdev.brightness > 0) {
		pr_info("aw21024->cdev.brightness high 0\n");
		if (aw21024->cdev.brightness > aw21024->cdev.max_brightness)
			aw21024->cdev.brightness = aw21024->cdev.max_brightness;

		pr_info("before aw21024_brightness_conversion aw21024->cdev.brightness is %d\n",
						aw21024->cdev.brightness);
		aw21024_brightness_conversion(aw21024,
						aw21024->cdev.brightness);
		pr_info("after aw21024_brightness_conversion aw21024->cdev.brightness is %d\n",
						aw21024->cdev.brightness);
		aw21024_i2c_write(aw21024, AW21024_REG_GCCR,
						aw21024->cdev.brightness);
	} else {
		pr_info("aw21024->cdev.brightness low 0\n");
		aw21024_i2c_write(aw21024, AW21024_REG_GCCR, 0);
	}*/
}

static void aw21024_set_brightness(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct aw21024 *aw21024 = container_of(cdev, struct aw21024, cdev);

	aw21024->cdev.brightness = brightness;
	if(aw21024->cdev.trigger != NULL)
	{
		if(strcmp(aw21024->cdev.trigger->name,"timer") == 0)
		{
			aw21024_led_change_mode(aw21024, AW21024_LED_BLINKMODE);
			AW_LOG("%s[%d]:  trigger = %s\n",__func__,aw21024->id,aw21024->cdev.trigger->name);
		}
	}

	schedule_work(&aw21024->brightness_work);
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw21024_read_chip_id(struct aw21024 *aw21024)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg_val = 0;
	dev_err(aw21024->dev, "%s:  start\n", __func__);
	/* hardware reset */
	aw21024_hw_reset(aw21024);
	usleep_range(200, 400);

	while (cnt++ < AW_READ_CHIPID_RETRIES) {
		ret = aw21024_i2c_read(aw21024, AW21024_REG_RESET, &reg_val);
		if (ret < 0) {
			dev_err(aw21024->dev,
				"%s: failed to read AW21024_CHIP_ID : %d\n",
				__func__, ret);
		} else {
			if (reg_val == AW21024_CHIP_ID) {
				pr_info("AW21024_CHIP_ID is : 0x%x\n", reg_val);
				return 0;
			}
		}
		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}
	pr_info("This chipid is not AW21024 and reg_data is : 0x%x\n", reg_val);

	return -EINVAL;
}

static int aw21024_read_chip_version(struct aw21024 *aw21024)
{
	int ret = -1;
	unsigned char reg_val = 0;

	ret = aw21024_i2c_read(aw21024, AW21024_REG_VER, &reg_val);
	if (ret < 0)
		dev_err(aw21024->dev, "%s: failed to read VERSION : %d\n", __func__, ret);
	else
		pr_info("THE CHIP_VERSION: 0x%x\n", reg_val);

	return ret;
}

/******************************************************
 *
 * sys group attribute
 *
 ******************************************************/

static ssize_t
aw21024_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);

	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2)
		aw21024_i2c_write(aw21024, databuf[0], databuf[1]);
	return count;
}

static ssize_t aw21024_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW21024_REG_MAX; i++) {
		if (!(aw21024_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw21024_i2c_read(aw21024, i, &reg_val);
		len +=
		    snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x\n",
			     i, reg_val);
	}
	return len;
}

static ssize_t
aw21024_hwen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 1)
		aw21024_hw_reset(aw21024);
	else
		aw21024_hw_off(aw21024);

	return count;
}

static ssize_t
aw21024_hwen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);
	ssize_t len = 0;

	len +=
	snprintf(buf + len, PAGE_SIZE - len, "hwen=%d\n", gpio_get_value(aw21024->reset_gpio));

	return len;
}

static ssize_t
aw21024_rgbcolor_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int databuf[2] = { 0, 0 };
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);


	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		/* GEn 0-7=0 */
		aw21024_i2c_write(aw21024, AW21024_REG_GCFG0, 0x00);
		/* GEn disable */
		aw21024_i2c_write(aw21024, AW21024_REG_GCFG1, 0x10);
		/* RGBMD=1 3=1 */
		aw21024_i2c_write(aw21024, AW21024_REG_GCR2, 0x01);
		/* brightness default */
		aw21024_i2c_write(aw21024, AW21024_REG_GCCR, 0x25);
		/* before trim */
		/*aw21024_i2c_write(aw21024, AW21024_REG_BR0 + databuf[0] * 2, 0xff);*/
		/* after trim */
		aw21024_i2c_write(aw21024, AW21024_REG_BR0 + databuf[0], 0xff);
		aw21024->rgbcolor = (databuf[1] & 0x00ff0000) >> 16;
		aw21024_i2c_write(aw21024, AW21024_REG_COL0 + databuf[0] * 3, aw21024->rgbcolor);
		aw21024->rgbcolor = (databuf[1] & 0x0000ff00) >> 8;
		aw21024_i2c_write(aw21024, AW21024_REG_COL0 + databuf[0] * 3 + 1,
				  aw21024->rgbcolor);
		aw21024->rgbcolor = (databuf[1] & 0x000000ff);
		aw21024_i2c_write(aw21024, AW21024_REG_COL0 + databuf[0] * 3 + 2,
				  aw21024->rgbcolor);
		aw21024_br_update(aw21024);
	}
	return len;
}

/*static ssize_t
aw21024_effect_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val < (sizeof(aw21024_cfg_array) / sizeof(struct aw21024_cfg))) {
		pr_info("%s: enter effect!!!\n", __func__);
		aw21024->effect = val;
		aw21024_cfg_update_array(aw21024);
	}

	return len;
}

static ssize_t
aw21024_effect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);

	for (i = 0; i < sizeof(aw21024_cfg_array) / sizeof(struct aw21024_cfg); i++) {
		len +=
		snprintf(buf + len, PAGE_SIZE - len, "cfg[%x] = %ps\n", i, aw21024_cfg_array[i].p);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "current cfg = %ps\n",
		aw21024_cfg_array[aw21024->effect].p);

	return len;
}*/
void store_effect_data(char *tmp_buf)
{
	unsigned long data = 0;
	char *delim = ",";
	char *str = NULL;
	int i = 0;
	int effectindex=0, dataindex=0;
	int ret =0;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	effectindex = data;
	AW_LOG("effectindex = %d \n",effectindex);
	if(effectindex > MAX_EFFECT)
	{
		return;
	}

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	dataindex=data;
	AW_LOG("dataindex = %d \n",dataindex);
	if(dataindex >= aw21024_cfg_array[effectindex].count)
	{
		return;
	}
	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	if(data>=1)
	{
		aw21024_cfg_array[effectindex].p[dataindex].frame_factor = data;
	}
	for (i = 0; i < 6; i++) {
		str = strsep(&tmp_buf, delim);
		if (str == NULL) {
			break;
		}
		ret = kstrtoul(str, 10, &data);
		if (ret) {
			return;
		}
		aw21024_cfg_array[effectindex].p[dataindex].time[i] = data;
		AW_LOG("time[%d] = %lu \n",i, data);
	}
	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	aw21024_cfg_array[effectindex].p[dataindex].repeat_nums = data;
	AW_LOG("repeat_nums = %lu \n", data);

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	aw21024_cfg_array[effectindex].p[dataindex].color_nums = data;
	AW_LOG("color_nums = %lu \n", data);
	for (i=0;i<aw21024_cfg_array[effectindex].p[dataindex].color_nums;i++){
		str = strsep(&tmp_buf, delim);
		if (str == NULL) {
			return;
		}
		ret = kstrtoul(str, 10, &data);
		if (ret) {
			return;
		}
		aw21024_cfg_array[effectindex].p[dataindex].rgb_color_list[i].r = data;
		AW_LOG("rgb_color_list[%d].r = %lu \n",i,data);
		str = strsep(&tmp_buf, delim);
		if (str == NULL) {
			return;
		}
		ret = kstrtoul(str, 10, &data);
		if (ret) {
			return;
		}
		aw21024_cfg_array[effectindex].p[dataindex].rgb_color_list[i].g = data;
		AW_LOG("rgb_color_list[%d].g = %lu \n", i,data);
		str = strsep(&tmp_buf, delim);
		if (str == NULL) {
			return;
		}
		ret = kstrtoul(str, 10, &data);
		if (ret) {
			return;
		}
		aw21024_cfg_array[effectindex].p[dataindex].rgb_color_list[i].b = data;
		AW_LOG("rgb_color_list[%d].b = %lu \n",i, data);
	}

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	aw21024_cfg_array[effectindex].p[dataindex].fadeh[0].r = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	aw21024_cfg_array[effectindex].p[dataindex].fadeh[0].g = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	aw21024_cfg_array[effectindex].p[dataindex].fadeh[0].b = data;
	AW_LOG("fadeh.r = %d, fadeh.g =%d, fadeh.b = %d \n", aw21024_cfg_array[effectindex].p[dataindex].fadeh[0].r, aw21024_cfg_array[effectindex].p[dataindex].fadeh[0].g, aw21024_cfg_array[effectindex].p[dataindex].fadeh[0].b);

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	} 
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	aw21024_cfg_array[effectindex].p[dataindex].fadel[0].r = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	} 
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	aw21024_cfg_array[effectindex].p[dataindex].fadel[0].g = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		return;
	} 
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		return;
	}
	aw21024_cfg_array[effectindex].p[dataindex].fadel[0].b = data;
	AW_LOG("fadel.r = %d, fadel.g = %d, fadel.b = %d \n", aw21024_cfg_array[effectindex].p[dataindex].fadel[0].r, aw21024_cfg_array[effectindex].p[dataindex].fadel[0].g, aw21024_cfg_array[effectindex].p[dataindex].fadel[0].b);

}

void store_effect_color_and_brightness(char *tmp_buf)
{
	unsigned long data = 0;
	char *delim = ",";
	char *str = NULL;
	int i = 0,ret=0;
	int gli=0;
	int effect_index=0;
	unsigned int br_rgb = 0;
	unsigned int rgb = 0,prev_rgb = 0;

	int *led_order = aw21024_g_chip[0][0]->pdata->led->led_allocation_order;
	/*
	store datatype,effect_index , br_rgb(in hexadecimal), rgb(in hexadecimal),...
	0,0,br_rgb,rgb    // incall
	0,1,br_rgb,rgb    // poweron
	0,2,br_rgb,rgb,rgb,rgb,rgb,rgb,rgb,rgb,rgb  // music mode
	0,3,br_rgb,rgb,rgb,rgb,rgb,rgb,rgb,rgb,rgb  // game mode
	0,4,br_rgb,rgb    // notify
	0,5,br_rgb,rgb,rgb,rgb,rgb,rgb,rgb,rgb,rgb // always on mode
	*/

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		AW_ERR("error in parsing 1.1");
		return;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		AW_ERR("error in parsing 1.2");
		return;
	}
	effect_index = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		AW_ERR("error in parsing 1.1");
		return;
	}
	ret = kstrtoul(str, 16, &data);
	if (ret) {
		AW_ERR("error in parsing 1.2");
		return;
	}
	br_rgb = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		AW_ERR("error in parsing 1.1");
		return;
	}
	ret = kstrtoul(str, 16, &data);
	if (ret) {
		AW_ERR("error in parsing 1.2");
		return;
	}
	rgb = data;

	switch(effect_index) {
		case INCALL_EFFECT:
			rgb_incall_list[led_order[0]].r = (rgb >> 16) & 0xff; rgb_incall_list[led_order[0]].g = (rgb >> 8) & 0xff; rgb_incall_list[led_order[0]].b = rgb & 0xff;
			br_incall_fadeh[0].r = (br_rgb >> 16) & 0xff; br_incall_fadeh[0].g = (br_rgb >> 8) & 0xff; br_incall_fadeh[0].b = br_rgb & 0xff;
			for(i=1;i<8;i++)
			{
				str = strsep(&tmp_buf, delim);
				if (str == NULL) {
					AW_ERR("error in parsing 1.1");
					return;
				}
				ret = kstrtoul(str, 16, &data);
				if (ret) {
					AW_ERR("error in parsing 1.2");
					return;
				}
				rgb = data;
				rgb_incall_list[led_order[i]].r = (rgb >> 16) & 0xff;
				rgb_incall_list[led_order[i]].g = (rgb >>8) & 0xff;
				rgb_incall_list[led_order[i]].b = rgb & 0xff;
			}
			break;
		case POWERON_EFFECT:
			rgb_poweron_list[led_order[0]].r = (rgb >> 16) & 0xff; rgb_poweron_list[led_order[0]].g = (rgb >> 8) & 0xff; rgb_poweron_list[led_order[0]].b = rgb & 0xff;
			br_poweron_fadeh[0].r = (br_rgb >> 16) & 0xff; br_poweron_fadeh[0].g = (br_rgb >> 8) & 0xff; br_poweron_fadeh[0].b = br_rgb & 0xff;
			for(i=1;i<8;i++)
			{
				str = strsep(&tmp_buf, delim);
				if (str == NULL) {
					AW_ERR("error in parsing 1.1");
					return;
				}
				ret = kstrtoul(str, 16, &data);
				if (ret) {
					AW_ERR("error in parsing 1.2");
					return;
				}
				rgb = data;
				rgb_poweron_list[led_order[i]].r = (rgb >> 16) & 0xff;
				rgb_poweron_list[led_order[i]].g = (rgb >>8) & 0xff;
				rgb_poweron_list[led_order[i]].b = rgb & 0xff;
			}
			break;
		case CHARGE_EFFECT:
			rgb_green_list[0].r = (rgb >> 16) & 0xff; rgb_green_list[0].g = (rgb >> 8) & 0xff; rgb_green_list[0].b = rgb & 0xff;
			br_green_fadeh[0].r = (br_rgb >> 16) & 0xff; br_green_fadeh[0].g = (br_rgb >> 8) & 0xff; br_green_fadeh[0].b = br_rgb & 0xff;
			break;
		case GAME_ENTER_EFFECT:
			br_game_fadeh[0].r = (br_rgb >> 16) & 0xff; br_game_fadeh[0].g = (br_rgb >> 8) & 0xff; br_game_fadeh[0].b = br_rgb & 0xff;
			rgb_multi_game_racing[0].r = (rgb >> 16) & 0xff; rgb_multi_game_racing[0].g = (rgb >> 8) & 0xff; rgb_multi_game_racing[0].b = rgb & 0xff;
			rgb_multi_game_lap[0].r = (rgb >> 16) & 0xff; rgb_multi_game_lap[0].g = (rgb >> 8) & 0xff; rgb_multi_game_lap[0].b = rgb & 0xff;
			gli = 1;
			prev_rgb = rgb;
			for(i=1;i<8;i++)
			{
				str = strsep(&tmp_buf, delim);
				if (str == NULL) {
					AW_ERR("error in parsing 1.1");
					return;
				}
				ret = kstrtoul(str, 16, &data);
				if (ret) {
					AW_ERR("error in parsing 1.2");
					return;
				}
				rgb = data;
				rgb_multi_game_racing[i].r = (rgb >> 16) & 0xff; rgb_multi_game_racing[i].g = (rgb >> 8) & 0xff; rgb_multi_game_racing[i].b = rgb & 0xff;
				if ((rgb != prev_rgb) && (gli < sizeof(rgb_multi_game_lap)/sizeof(AW_COLOR_STRUCT) ))
				{
					rgb_multi_game_lap[gli].r = (rgb >> 16) & 0xff; rgb_multi_game_lap[gli].g = (rgb >> 8) & 0xff; rgb_multi_game_lap[gli].b = rgb & 0xff;
					gli++;
					prev_rgb = rgb;
				}
			}
			for(i=gli;i< sizeof(rgb_multi_game_lap)/sizeof(AW_COLOR_STRUCT); i++)
			{
				rgb_multi_game_lap[i].r = rgb_multi_game_lap[i%gli].r; 
				rgb_multi_game_lap[i].g = rgb_multi_game_lap[i%gli].g;
				rgb_multi_game_lap[i].b = rgb_multi_game_lap[i%gli].b;
			}
			break;
		case NOTIFY_EFFECT:
			effect_state.data[AW21024_LED_NOTIFY_MODE] = 1;
			rgb_notify_list[led_order[0]].r = (rgb >> 16) & 0xff; rgb_notify_list[led_order[0]].g = (rgb >> 8) & 0xff; rgb_notify_list[led_order[0]].b = rgb & 0xff;
			br_notify_fadeh[0].r = (br_rgb >> 16) & 0xff; br_notify_fadeh[0].g = (br_rgb >> 8) & 0xff; br_notify_fadeh[0].b = br_rgb & 0xff;
			for(i=1;i<8;i++)
			{
				str = strsep(&tmp_buf, delim);
				if (str == NULL) {
					AW_ERR("error in parsing 1.1");
					return;
				}
				ret = kstrtoul(str, 16, &data);
				if (ret) {
					AW_ERR("error in parsing 1.2");
					return;
				}
				rgb = data;
				rgb_notify_list[led_order[i]].r = (rgb >> 16) & 0xff;
				rgb_notify_list[led_order[i]].g = (rgb >>8) & 0xff;
				rgb_notify_list[led_order[i]].b = rgb & 0xff;
			}

			break;
		case NEWALWAYSON_EFFECT:
			effect_state.data[AW21024_LED_NEW_ALWAYSON] = 1;
			new_always_on_color[0][8] = (br_rgb >> 16) & 0xff; new_always_on_color[1][8]= (br_rgb >> 8) & 0xff; new_always_on_color[2][8] = br_rgb & 0xff;
			new_always_on_color[0][led_order[0]] = (rgb >> 16) & 0xff; 
			new_always_on_color[1][led_order[0]] = (rgb >>8) & 0xff; 
			new_always_on_color[2][led_order[0]] = rgb & 0xff; 
			for(i=1;i<8;i++)
			{
				str = strsep(&tmp_buf, delim);
				if (str == NULL) {
					AW_ERR("error in parsing 1.1");
					return;
				}
				ret = kstrtoul(str, 16, &data);
				if (ret) {
					AW_ERR("error in parsing 1.2");
					return;
				}
				rgb = data;
				if(led_order[i] <8) {
					new_always_on_color[0][led_order[i]] = (rgb >> 16) & 0xff; 
					new_always_on_color[1][led_order[i]] = (rgb >>8) & 0xff; 
					new_always_on_color[2][led_order[i]] = rgb & 0xff;
				}
				else {
					AW_ERR("led_order[i] %d is out of index",led_order[i]);
				}
			}
			break;
		case MUSIC_EFFECT:
			effect_state.data[AW21024_LED_MUSICMODE] = 1;
			music_color[0][8] = (br_rgb >> 16) & 0xff; music_color[1][8]= (br_rgb >> 8) & 0xff; music_color[2][8] = br_rgb & 0xff;
			music_color[0][led_order[0]] = (rgb >> 16) & 0xff; 
			music_color[1][led_order[0]] = (rgb >>8) & 0xff; 
			music_color[2][led_order[0]] = rgb & 0xff; 
			for(i=1;i<8;i++)
			{
				str = strsep(&tmp_buf, delim);
				if (str == NULL) {
					AW_ERR("error in parsing 1.1");
					return;
				}
				ret = kstrtoul(str, 16, &data);
				if (ret) {
					AW_ERR("error in parsing 1.2");
					return;
				}
				rgb = data;
				if(led_order[i] <8) {
					music_color[0][led_order[i]] = (rgb >> 16) & 0xff; 
					music_color[1][led_order[i]] = (rgb >>8) & 0xff; 
					music_color[2][led_order[i]] = rgb & 0xff;
				}
				else {
					AW_ERR("led_order[i] %d is out of index",led_order[i]);
				}
			}
			break;
		default:
			break;
	}	
}

static ssize_t aw21024_effectdata_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	char *delim = ",";
	char *str = NULL;
	char *p_buf, *tmp_buf;

	ssize_t ret = -EINVAL;

	AW_LOG("effectdata raw %s\n", buf);

	p_buf = kstrdup(buf, GFP_KERNEL);
	tmp_buf = p_buf;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		goto errout;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		goto errout;
	}

	if(data == 0){
		store_effect_color_and_brightness(tmp_buf);
	}
	else {
		store_effect_data(tmp_buf);
	}

errout:
	kfree(p_buf);
	return cnt;
}

static ssize_t aw21024_l_type_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i=0;
	int len =0;
	
	for(i=0;i<AW21024_LED_MAXMODE; i++)
	{
		len += snprintf(buf, PAGE_SIZE - len, " %d = %d, ",i, effect_state.state[i]);
	}
	len += snprintf(buf, PAGE_SIZE - len, " %d(max:%d)",i, AW21024_LED_MAXMODE -1 );
	return len;
}

static ssize_t aw21024_l_type_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	char *delim = ",";
	char *str = NULL;
	char *p_buf, *tmp_buf;
	//int i = 0;
	int lmode=0,val=0;

	ssize_t ret = -EINVAL;

	AW_LOG("l_type raw %s\n", buf);

	p_buf = kstrdup(buf, GFP_KERNEL);
	tmp_buf = p_buf;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		goto errout;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		goto errout;
	}
	lmode = data;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		goto errout;
	}
	ret = kstrtoul(str, 10, &data);
	if (ret) {
		goto errout;
	}
	val = data;

	if(lmode >=0 &&  lmode<AW21024_LED_MAXMODE)
	{
		str = strsep(&tmp_buf, delim);
		if (str == NULL) {
			goto errout;
		}
		ret = kstrtoul(str, 10, &data);
		if (ret) {
			goto errout;
		}

		if( lmode == AW21024_LED_CHARGE_MODE ){
			if( (data >=0) && (data <= 4)) {
				effect_state.data[AW21024_LED_CHARGE_MODE] = data;
			}
			if( val == 0)
				effect_state.state[lmode] = val;

			if(effect_state.state[lmode] != 0) {
				AW_ERR("skip charger state setting state = %d\n", effect_state.state[lmode] );
				goto errout;
			}
		}
		effect_state.state[lmode] = val;
	}

errout:
	kfree(p_buf);
	return cnt;
}

static ssize_t aw21024_light_sensor_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);
	u32 value;
	aw21024 = aw21024->pdata->led;

	if (sscanf(buf, "%d", &value) != 1) {
		AW_ERR("invaild arguments\n");
		return cnt;
	}
	aw21024->light_sensor_state = value;
	aw21024_global_set(aw21024);
	/*if ((aw21024->led_groups_num == 8) && (aw21024_g_chip[1][0] != NULL)) {
		aw21024_g_chip[1][0]->light_sensor_state = value;
		aw21024_global_set(aw21024_g_chip[1][0]);
	}*/

	AW_ERR("light_sensor value %d\n", value);
	return cnt;
}

static ssize_t aw21024_light_sensor_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);
	ssize_t len = 0;
	aw21024 = aw21024->pdata->led;
	len += snprintf(buf + len, PAGE_SIZE - len, "light_sensor: %d\n", aw21024->light_sensor_state);
	return len;
}


static ssize_t aw21024_global_current_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);

	unsigned long data = 0;
	char *delim = ",";
	char *str = NULL;
	char *p_buf, *tmp_buf;
	ssize_t ret = -EINVAL;

	aw21024 = aw21024->pdata->led;

	p_buf = kstrdup(buf, GFP_KERNEL);
	tmp_buf = p_buf;

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		goto errout;
	}
	ret = kstrtoul(str, 16, &data);
	if (ret) {
		goto errout;
	}
	if(data < 0x100) {
		aw21024->glo_current_min = data;
	}
	else {
		aw21024->glo_current_min = 0xff;
	}

	str = strsep(&tmp_buf, delim);
	if (str == NULL) {
		goto errout;
	}
	ret = kstrtoul(str, 16, &data);
	if (ret) {
		goto errout;
	}
	if(data < 0x100) {
		aw21024->glo_current_max = data;
	}
	else {
		aw21024->glo_current_max = 0xff;
	}

	aw21024_global_set(aw21024);
	/*if ((aw21024->led_groups_num == 8) && (aw21024_g_chip[1][0] != NULL)) {
		aw21024_g_chip[1][0]->glo_current_min = aw21024->glo_current_min;
		aw21024_g_chip[1][0]->glo_current_max = aw21024->glo_current_max;
		aw21024_global_set(aw21024_g_chip[1][0]);
	}*/

	AW_ERR("gccr values min = %x max = %x\n", aw21024->glo_current_min, aw21024->glo_current_max);

errout:
	kfree(p_buf);
	return cnt;
}

static ssize_t aw21024_global_current_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *aw21024 = container_of(led_cdev, struct aw21024, cdev);
	ssize_t len = 0;
	aw21024 = aw21024->pdata->led;
	len += snprintf(buf + len, PAGE_SIZE - len, "gccr values min = %x max = %x\n", aw21024->glo_current_min, aw21024->glo_current_max);
	return len;
}

static ssize_t aw21024_led_ton_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->hold_time_ms);
}

static ssize_t aw21024_led_ton_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->hold_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);
	AW_LOG("[%d]: hold_time_ms=%d (max:15)\n",led->id,led->pdata->hold_time_ms);

	return cnt;
}

static ssize_t aw21024_led_tr1_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->rise_time_ms);
}

static ssize_t aw21024_led_tr1_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->rise_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);

	AW_LOG("[%d]: rise_time_ms=%d (max:15)\n",led->id,led->pdata->rise_time_ms);

	return cnt;
}

static ssize_t aw21024_led_tf1_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->fall_time_ms);

}

static ssize_t aw21024_led_tf1_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->fall_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);

	AW_LOG("[%d]: fall_time_ms=%d (max:15)\n",led->id,led->pdata->fall_time_ms);

	return cnt;
}

static ssize_t aw21024_led_toff_attr_show (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	return snprintf(buf, PAGE_SIZE, "%d (max:15)\n",led->pdata->off_time_ms);
}

static ssize_t aw21024_led_toff_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t cnt)
{
	unsigned long data = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	mutex_lock(&led->pdata->led->lock);
	led->pdata->off_time_ms = data;
	mutex_unlock(&led->pdata->led->lock);
	AW_LOG("[%d]: off_time_ms=%d (max:15)\n",led->id,led->pdata->off_time_ms);

	return cnt;
}

static ssize_t aw21024_led_support_attr_show (struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int prj_id = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw21024 *led = container_of(led_cdev, struct aw21024, cdev);

	prj_id = get_project();

//	return snprintf(buf, PAGE_SIZE, "%s-%d-white\n", LED_SUPPORT_TYPE, (int)led->cdev.max_brightness);
	return snprintf(buf, PAGE_SIZE, "%s-%d\n",LED_SUPPORT_TYPE,led->cdev.max_brightness);
}

static DEVICE_ATTR(support, 0664, aw21024_led_support_attr_show, NULL);
static DEVICE_ATTR(reg, 0664, aw21024_reg_show, aw21024_reg_store);
static DEVICE_ATTR(hwen, 0664, aw21024_hwen_show, aw21024_hwen_store);
static DEVICE_ATTR(rgbcolor, 0664, NULL, aw21024_rgbcolor_store);
//static DEVICE_ATTR(effect, 0664, aw21024_effect_show, aw21024_effect_store);
static DEVICE_ATTR(effectdata, 0664, NULL, aw21024_effectdata_attr_store);
static DEVICE_ATTR(l_type, 0664, aw21024_l_type_attr_show, aw21024_l_type_attr_store);
static DEVICE_ATTR(light_sensor, 0664, aw21024_light_sensor_attr_show, aw21024_light_sensor_attr_store);
static DEVICE_ATTR(global_current, 0664, aw21024_global_current_attr_show, aw21024_global_current_attr_store);
static DEVICE_ATTR(ton, 0664, aw21024_led_ton_attr_show, aw21024_led_ton_attr_store);
static DEVICE_ATTR(toff, 0664, aw21024_led_toff_attr_show, aw21024_led_toff_attr_store);
static DEVICE_ATTR(tr1, 0664, aw21024_led_tr1_attr_show, aw21024_led_tr1_attr_store);
static DEVICE_ATTR(tf1, 0664, aw21024_led_tf1_attr_show, aw21024_led_tf1_attr_store);

static struct attribute *aw21024_attributes[] = {
	&dev_attr_support.attr,
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_rgbcolor.attr,
	//&dev_attr_effect.attr,
	&dev_attr_effectdata.attr,
	&dev_attr_l_type.attr,
	&dev_attr_light_sensor.attr,
	&dev_attr_global_current.attr,
	&dev_attr_ton.attr,
	&dev_attr_toff.attr,
	&dev_attr_tr1.attr,
	&dev_attr_tf1.attr,
	NULL
};

static struct attribute_group aw21024_attribute_group = {
	.attrs = aw21024_attributes
};

static int aw21024_led_err_handle(struct aw21024 *led_array,
		int parsed_leds)
{
	int i=0;
	/*
	* If probe fails, cannot free resource of all LEDs, only free
	* resources of LEDs which have allocated these resource really.
	*/
	for (i = 0; i < parsed_leds; i++) {
		sysfs_remove_group(&led_array[i].cdev.dev->kobj,
				&aw21024_attribute_group);
		led_classdev_unregister(&led_array[i].cdev);
		cancel_work_sync(&led_array[i].brightness_work);
		devm_kfree(&led_array->i2c->dev, led_array[i].pdata);
		led_array[i].pdata = NULL;
	}
	return i;
}
/******************************************************
 *
 * dts gpio
 *
 ******************************************************/
static int aw21024_parse_led_cdev(struct aw21024 *aw21024,
		struct device_node *node)
{
	struct aw21024 *led;
	struct aw21024_platform_data *pdata;
	int rc = -1, parsed_leds = 0;
	struct device_node *temp;

	AW_LOG("enter\n");
	aw21024_glo = aw21024;
	for_each_child_of_node(node, temp) {
		led = &aw21024[parsed_leds];
		aw21024_g_chip[0][parsed_leds] = led;
		led->i2c = aw21024->i2c;

		pdata = devm_kzalloc(&led->i2c->dev,
				sizeof(struct aw21024_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&led->i2c->dev,
				"Failed to allocate memory\n");
			goto free_err;
		}
		pdata->led = aw21024;
		led->pdata = pdata;

		rc = of_property_read_string(temp, "aw21024,name",
			&led->cdev.name);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading led name, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw21024,id",
			&led->id);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading id, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw21024,imax",
			&led->pdata->imax);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading imax, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_string(temp, "aw21024,led_default_trigger",
									&led->pdata->led_default_trigger);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure led_default_trigger, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw21024,brightness",
			&led->cdev.brightness);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading brightness, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw21024,max-brightness",
			&led->cdev.max_brightness);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading max-brightness, rc = %d\n",
				rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw21024,rise-time-ms",
			&led->pdata->rise_time_ms);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading rise-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw21024,hold-time-ms",
			&led->pdata->hold_time_ms);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading hold-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw21024,fall-time-ms",
			&led->pdata->fall_time_ms);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading fall-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		rc = of_property_read_u32(temp, "aw21024,off-time-ms",
			&led->pdata->off_time_ms);
		if (rc < 0) {
			dev_err(&led->i2c->dev,
				"Failure reading off-time-ms, rc = %d\n", rc);
			goto free_pdata;
		}

		INIT_WORK(&led->brightness_work, aw21024_brightness_work);
		INIT_DELAYED_WORK(&led->breath_work, aw21024_breath_func);

		led->cdev.brightness_set = aw21024_set_brightness;
//		led->cdev.default_trigger = pdata->led_default_trigger;
		rc = led_classdev_register(&led->i2c->dev, &led->cdev);
		if (rc) {
			dev_err(&led->i2c->dev,
				"unable to register led %d,rc=%d\n",
				led->id, rc);
			goto free_pdata;
		}

		rc = sysfs_create_group(&led->cdev.dev->kobj,
				&aw21024_attribute_group);
		if (rc) {
			dev_err(&led->i2c->dev, "led sysfs rc: %d\n", rc);
			goto free_class;
		}
		parsed_leds++;
	}
	max_led = parsed_leds;

	return 0;

free_class:
	aw21024_led_err_handle(aw21024, parsed_leds);
	led_classdev_unregister(&aw21024[parsed_leds].cdev);
	cancel_work_sync(&aw21024[parsed_leds].brightness_work);
	devm_kfree(&led->i2c->dev, aw21024[parsed_leds].pdata);
	aw21024[parsed_leds].pdata = NULL;
	return rc;

free_pdata:
	aw21024_led_err_handle(aw21024, parsed_leds);
	devm_kfree(&led->i2c->dev, aw21024[parsed_leds].pdata);
	return rc;

free_err:
	aw21024_led_err_handle(aw21024, parsed_leds);
	return rc;
}

static int aw21024_led_change_mode(struct aw21024 *led,
		enum AW2023_LED_MODE mode)
{
	int ret=0;
	switch(mode) {
		case AW21024_LED_CCMODE:
			led->pdata->led_mode = AW21024_LED_CCMODE;
			break;
		case AW21024_LED_NEW_ALWAYSON:
			led->pdata->led_mode = AW21024_LED_NEW_ALWAYSON;
			break;
		case AW21024_LED_BLINKMODE:
			led->pdata->hold_time_ms = 4;
			led->pdata->off_time_ms =  4;
			led->pdata->rise_time_ms = 0;
			led->pdata->fall_time_ms = 0;
			led->pdata->led_mode = AW21024_LED_BLINKMODE;
			break;
		case AW21024_LED_BREATHMODE:
			led->pdata->hold_time_ms = 0;
			led->pdata->off_time_ms =  0;
			led->pdata->rise_time_ms = 6;
			led->pdata->fall_time_ms = 6;
			led->pdata->led_mode = AW21024_LED_BREATHMODE;
			break;
		case AW21024_LED_INDIVIDUAL_CTL_BREATH:
			led->pdata->hold_time_ms = 0;
			led->pdata->off_time_ms =  0;
			led->pdata->rise_time_ms = 6;
			led->pdata->fall_time_ms = 6;
			led->pdata->led_mode = AW21024_LED_INDIVIDUAL_CTL_BREATH;
			break;
		case AW21024_LED_MUSICMODE:
			led->pdata->led_mode = AW21024_LED_MUSICMODE;
			break;
		case AW21024_LED_INCALL_MODE:
			led->pdata->led_mode = AW21024_LED_INCALL_MODE;
			break;
		case AW21024_LED_NOTIFY_MODE:
			led->pdata->led_mode = AW21024_LED_NOTIFY_MODE;
			break;
		case AW21024_LED_CHARGE_MODE:
			led->pdata->led_mode = AW21024_LED_CHARGE_MODE;
			break;
		case AW21024_LED_POWERON_MODE:
			led->pdata->led_mode = AW21024_LED_POWERON_MODE;
			break;
		case AW21024_LED_GAMEMODE:
			led->pdata->led_mode = AW21024_LED_GAMEMODE;
			break;
		default:
			led->pdata->led_mode = AW21024_LED_NONE;
			break;
	}
	return ret;
}
/*
static struct attribute *aw21024_led_cc_mode_attrs[] = {
};

static struct attribute *aw21024_led_blink_mode_attrs[] = {
};

static struct attribute *aw21024_led_breath_mode_attrs[] = {
};

ATTRIBUTE_GROUPS(aw21024_led_cc_mode);
ATTRIBUTE_GROUPS(aw21024_led_blink_mode);
ATTRIBUTE_GROUPS(aw21024_led_breath_mode);
*/
static int	aw21024_led_cc_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_CCMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}

	return ret;
}

static void  aw21024_led_cc_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: deactivate",led->id);
}

static int aw21024_led_new_always_on_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_NEW_ALWAYSON);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}

	return ret;
}

static void  aw21024_led_new_always_on_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: new_always_on deactivate",led->id);
}

static int	aw21024_led_blink_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("%s[%d]: activate",__func__,led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_BLINKMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}

	return ret;
}

static void aw21024_led_blink_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: deactivate",led->id);
}

static int aw21024_led_breath_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_BREATHMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw21024_led_breath_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: deactivate",led->id);
}

static int aw21024_led_individual_ctl_breath_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_INDIVIDUAL_CTL_BREATH);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw21024_led_individual_ctl_breath_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: individual_ctl_breath deactivate",led->id);
}

static int aw21024_led_music_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_MUSICMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw21024_led_music_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: music deactivate",led->id);
}

static int aw21024_led_incall_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_INCALL_MODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw21024_led_incall_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: %s\n", led->id, __func__);
}

static int aw21024_led_notify_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_NOTIFY_MODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw21024_led_notify_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: %s\n", led->id, __func__);
}

static int aw21024_led_charge_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_CHARGE_MODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw21024_led_charge_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: %s\n", led->id, __func__);
}

static int aw21024_led_poweron_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_POWERON_MODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw21024_led_poweron_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: %s\n", led->id, __func__);
}

static int aw21024_led_game_activate(struct led_classdev *cdev)
{
	int ret = 0;
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);

	AW_LOG("[%d]: activate",led->id);

	ret = aw21024_led_change_mode(led, AW21024_LED_GAMEMODE);
	if (ret < 0) {
		dev_err(led->cdev.dev, "%s: aw21024_led_change_mode fail\n", __func__);
		return ret;
	}
	return ret;
}

static void aw21024_led_game_deactivate(struct led_classdev *cdev)
{
	struct aw21024 *led = container_of(cdev, struct aw21024, cdev);
	aw21024_led_change_mode(led, AW21024_LED_NONE);
	AW_LOG("[%d]: %s\n", led->id, __func__);
}

static struct led_trigger aw21024_led_trigger[LEDMODE_MAX_NUM] = {
	{
		.name = "cc_mode",
		.activate = aw21024_led_cc_activate,
		.deactivate = aw21024_led_cc_deactivate,
//		.groups = aw21024_led_cc_mode_groups,
	},
	{
		.name = "new_always_on_mode",
		.activate = aw21024_led_new_always_on_activate,
		.deactivate = aw21024_led_new_always_on_deactivate,
//		.groups = aw21024_led_new_always_on_mode_groups,
	},
	{
		.name = "blink_mode",
		.activate = aw21024_led_blink_activate,
		.deactivate = aw21024_led_blink_deactivate,
//		.groups = aw21024_led_blink_mode_groups,
	},
	{
		.name = "breath_mode",
		.activate = aw21024_led_breath_activate,
		.deactivate = aw21024_led_breath_deactivate,
//		.groups = aw21024_led_breath_mode_groups,
	},
	{
		.name = "individual_ctl_breath",
		.activate = aw21024_led_individual_ctl_breath_activate,
		.deactivate = aw21024_led_individual_ctl_breath_deactivate,
//		.groups = aw21024_led_individual_ctl_breath_mode_groups,
	},
	{
		.name = "music_mode",
		.activate = aw21024_led_music_activate,
		.deactivate = aw21024_led_music_deactivate,
//		.groups = aw21024_led_music_mode_groups,
	},
	{
		.name = "incall_mode",
		.activate = aw21024_led_incall_activate,
		.deactivate = aw21024_led_incall_deactivate,
//		.groups = aw21024_led_music_mode_groups,
	},
	{
		.name = "notify_mode",
		.activate = aw21024_led_notify_activate,
		.deactivate = aw21024_led_notify_deactivate,
//		.groups = aw21024_led_music_mode_groups,
	},
	{
		.name = "charge_mode",
		.activate = aw21024_led_charge_activate,
		.deactivate = aw21024_led_charge_deactivate,
//		.groups = aw21024_led_music_mode_groups,
	},
	{
		.name = "poweron_mode",
		.activate = aw21024_led_poweron_activate,
		.deactivate = aw21024_led_poweron_deactivate,
//		.groups = aw21024_led_music_mode_groups,
	},
	{
		.name = "game_mode",
		.activate = aw21024_led_game_activate,
		.deactivate = aw21024_led_game_deactivate,
//		.groups = aw21024_led_music_mode_groups,
	},
};

static int aw21024_parse_dts(struct device *dev, struct aw21024 *aw21024, struct device_node *np)
{
	int ret = 0;
	int led_order[8], i = 0;
	aw21024->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);

	aw21024->vbled_enable_gpio = of_get_named_gpio(np, "vbled-enable-gpio", 0);
	if (aw21024->vbled_enable_gpio < 0) {
		aw21024->vbled_enable_gpio = -1;
		dev_err(dev, "%s no vbled enable gpio provided, HW enable unsupported\n", __func__);
	}
	ret = of_property_read_u32(np, "led_groups_num",
			&aw21024->led_groups_num);
	if (ret < 0) {
		AW_ERR("led_groups_num is not set, set led_groups_num = 8\n");
		aw21024->led_groups_num = 8;
	}

	ret = of_property_read_u32_array(np, "led_allocation_order", led_order, aw21024->led_groups_num);

	if (ret) {
		for (i = 0; i < aw21024->led_groups_num; i++) {
			aw21024->led_allocation_order[i] = i;
		}
		AW_ERR("led_allocation_order is not set, set led_allocation_order = 1,2,3,4,5,6,7,8\n");
	} else {
		for (i = 0; i < aw21024->led_groups_num; i++) {
			aw21024->led_allocation_order[i] = led_order[i];
		}
		AW_ERR("led_allocation_order is %d,%d,%d,%d,%d,%d,%d,%d\n", aw21024->led_allocation_order[0] + 1, aw21024->led_allocation_order[1] + 1,
			aw21024->led_allocation_order[2] + 1, aw21024->led_allocation_order[3] + 1,aw21024->led_allocation_order[4] + 1,aw21024->led_allocation_order[5] + 1,
			aw21024->led_allocation_order[6] + 1,aw21024->led_allocation_order[7] + 1);
	}

	ret = of_property_read_u32(np, "vbled_volt", &aw21024->vbled_volt);
	if (ret < 0) {
		dev_err(dev, "%s vbled_volt is not set\n", __func__);
		aw21024->vbled_volt = 0;
	}

	ret = of_property_read_u32(np, "clk_pwm", &aw21024->clk_pwm);
	if (ret) {
		aw21024->clk_pwm = 1;
		dev_err(dev, "%s dts clk_pwm not found\n", __func__);
	} else {
		dev_info(dev, "%s read dts clk_pwm = %d\n", __func__, aw21024->clk_pwm);
	}
	aw21024->apse_mode = of_property_read_bool(np, "apse_mode");
	if (aw21024->apse_mode)
		dev_info(dev, "%s driver use apse_mode\n", __func__);
	else
		dev_info(dev, "%s driver use general_mode\n", __func__);

	ret = of_property_read_u32(np, "led_current", &aw21024->dts_led_current);
	if (ret) {
		aw21024->dts_led_current = 125;
		dev_err(dev, "%s dts led_current not found\n", __func__);
	} else {
		dev_info(dev, "%s read dts led_current = %d\n", __func__, aw21024->dts_led_current);
	}
	ret = of_property_read_u32(np, "brightness", &aw21024->cdev.brightness);
	if (ret) {
		aw21024->cdev.brightness = 125;
		dev_err(dev, "%s dts brightness not found\n", __func__);
	} else {
		dev_info(dev, "%s read dts brightness = %d\n", __func__, aw21024->cdev.brightness);
	}
	ret = of_property_read_u32(np, "max_brightness", &aw21024->cdev.max_brightness);
	if (ret) {
		aw21024->cdev.max_brightness = 255;
		dev_err(dev, "%s dts max_brightness not found\n", __func__);
	} else {
		dev_info(dev, "%s read dts max_brightness = %d\n", __func__,
			aw21024->cdev.max_brightness);
	}

	ret = of_property_read_u32(np, "global_current_max",
			&aw21024->glo_current_max);
	if (ret < 0) {
		AW_ERR("global current max resolution unsupported\n");
		//return ret;
	}

	ret = of_property_read_u32(np, "global_current_min",
			&aw21024->glo_current_min);
	if (ret < 0) {
		AW_ERR("global current min resolution unsupported\n");
		//return ret;
	}

	return 0;
}

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw21024_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw21024 *aw21024;
	struct device_node *np = i2c->dev.of_node;
	int ret, num_leds = 0, i = 0;
	dev_err(&i2c->dev, "aw21024: start to probe\n");
	
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT ||
		get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT) {
		AW_LOG("boot_mode is power_off_charging skip probe");
		return 0;
	}

	num_leds = of_get_child_count(np);
	dev_err(&i2c->dev, "aw21024: the num of leds is %d\n", num_leds);

	if (!num_leds)
		return -EINVAL;
	
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -ENODEV;
	}

	aw21024 = devm_kzalloc(&i2c->dev, (sizeof(struct aw21024) * num_leds), GFP_KERNEL);
	if (aw21024 == NULL)
		return -ENOMEM;

	aw21024->dev = &i2c->dev;
	aw21024->i2c = i2c;
	aw21024->num_leds = num_leds;
	i2c_set_clientdata(i2c, aw21024);
	dev_set_drvdata(&i2c->dev, aw21024);
	aw21024->rgb_isnk_on = 0;
	mutex_init(&aw21024->lock);

	/* get dts info */
	if (np) {
		ret = aw21024_parse_dts(&i2c->dev, aw21024, np);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
			goto err_parse_dts;
		}
	} else {
		aw21024->reset_gpio = -1;
	}

	if (gpio_is_valid(aw21024->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw21024->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw21024_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
			goto err_gpio_request;
		}
	}

	if (gpio_is_valid(aw21024->vbled_enable_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw21024->vbled_enable_gpio,
				GPIOF_OUT_INIT_LOW, "aw21024_vbled_en");
		if (ret) {
			dev_err(&i2c->dev, "%s: vbled enable gpio request failed\n", __func__);
			goto err_gpio_request;
		}
	}

	/* vdd 3.3v*/
	aw21024->vbled = regulator_get(aw21024->dev, "vbled");

	if (!aw21024->vbled_volt || IS_ERR_OR_NULL(aw21024->vbled)) {
		dev_err(&i2c->dev, "%s: Regulator vdd3v3 get failed\n", __func__);
	} else {
		dev_err(&i2c->dev, "%s: Regulator vbled volt set %u \n", __func__, aw21024->vbled_volt);
		if (aw21024->vbled_volt) {
			ret = regulator_set_voltage(aw21024->vbled, aw21024->vbled_volt,
							aw21024->vbled_volt);
		} else {
			ret = regulator_set_voltage(aw21024->vbled, 3300000, 3300000);
		}
		if (ret) {
			dev_err(&i2c->dev, "%s: Regulator set_vtg failed vdd rc = %d\n", __func__, ret);
		}
	}

	/* aw21024 chip id */
	ret = aw21024_read_chip_id(aw21024);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw21024_read_chip_id failed ret=%d\n", __func__, ret);
		goto err_id;
	}

	ret = aw21024_read_chip_version(aw21024);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw21024_read_chip_version failed ret=%d\n", __func__, ret);
		goto err_id;
	}

	aw21024_parse_led_cdev(aw21024, np);
	if (ret < 0) {
		AW_ERR("error creating led class dev\n");
		goto err_sysfs;
	}

	/* aw21024 led trigger register */
	for (i = 0; i < LEDMODE_MAX_NUM; i++) {
	    ret = led_trigger_register(&aw21024_led_trigger[i]);
		if (ret < 0) {
			dev_err(&i2c->dev, "register %d trigger fail\n", i);
			goto fail_led_trigger;
		}
	}

	aw21024_led_init(aw21024);
	aw21024->light_sensor_state = 1;
	effect_state.cur_effect = -1;
	effect_state.prev_effect = -1;
	for(i=0;i<AW21024_LED_MAXMODE;i++)
	{
		effect_state.state[i]=0;
		effect_state.data[i]=0;
	}
	aw21024_hw_off(aw21024);
	
	
	/*aw21024->cdev.name = "aw21024_led";
	INIT_WORK(&aw21024->brightness_work, aw21024_brightness_work);
	aw21024->cdev.brightness_set = aw21024_set_brightness;
	ret = led_classdev_register(aw21024->dev, &aw21024->cdev);
	if (ret) {
		dev_err(aw21024->dev, "unable to register led ret=%d\n", ret);
		goto err_class;
	}
	ret = sysfs_create_group(&aw21024->cdev.dev->kobj, &aw21024_attribute_group);
	if (ret) {
		dev_err(aw21024->dev, "led sysfs ret: %d\n", ret);
		goto err_sysfs;
	}*/
	pr_info("%s: probe successful!!!\n", __func__);

	return 0;
fail_led_trigger:
	while (--i >= 0)
		led_trigger_register(&aw21024_led_trigger[i]);
	aw21024_led_err_handle(aw21024, num_leds);
	
err_sysfs:
	led_classdev_unregister(&aw21024->cdev);
//err_class:
err_id:
	gpio_free(aw21024->vbled_enable_gpio);
	gpio_free(aw21024->reset_gpio);
err_gpio_request:
err_parse_dts:
	devm_kfree(&i2c->dev, aw21024);
	aw21024 = NULL;
	return ret;
}

static void aw21024_i2c_remove(struct i2c_client *i2c)
{
	struct aw21024 *aw21024 = i2c_get_clientdata(i2c);

	sysfs_remove_group(&aw21024->cdev.dev->kobj, &aw21024_attribute_group);
	led_classdev_unregister(&aw21024->cdev);

	if (gpio_is_valid(aw21024->reset_gpio))
		gpio_free(aw21024->reset_gpio);
	if (gpio_is_valid(aw21024->vbled_enable_gpio))
		gpio_free(aw21024->vbled_enable_gpio);
	mutex_destroy(&aw21024->pdata->led->lock);
	devm_kfree(&i2c->dev, aw21024);
	aw21024 = NULL;

	//return 0;
}

static const struct i2c_device_id aw21024_i2c_id[] = {
	{AW21024_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw21024_i2c_id);

static const struct of_device_id aw21024_dt_match[] = {
	{.compatible = "awinic,aw21024_led"},
	{},
};

static struct i2c_driver aw21024_i2c_driver = {
	.driver = {
		   .name = AW21024_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw21024_dt_match),
		   },
	.probe = aw21024_i2c_probe,
	.remove = aw21024_i2c_remove,
	.id_table = aw21024_i2c_id,
};

static int __init aw21024_i2c_init(void)
{
	int ret = 0;

	pr_info("aw21024 driver version %s\n", AW21024_DRIVER_VERSION);
	ret = i2c_add_driver(&aw21024_i2c_driver);
	if (ret) {
		pr_err("fail to add aw21024 device into i2c\n");
		return ret;
	}
	return 0;
}

module_init(aw21024_i2c_init);

static void __exit aw21024_i2c_exit(void)
{
	i2c_del_driver(&aw21024_i2c_driver);
}

module_exit(aw21024_i2c_exit);

MODULE_DESCRIPTION("AW21024 LED Driver");
MODULE_LICENSE("GPL v2");
