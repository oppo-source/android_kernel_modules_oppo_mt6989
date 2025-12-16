/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _KTZ8868_SW_H_
#define _KTZ8868_SW_H_

struct i2c_client *lcd_bl_i2c_client;
int ktz8868_read_byte(unsigned char i2c_client_addr, unsigned char *i2c_client_buf);
int ktz8868_write_byte(unsigned char i2c_client_addr, unsigned char value);
int ktz8868_brightness_enable(bool enable);
int ktz8868_set_brightness(unsigned int bl_lvl);
int ktz8868_set_lcd_bias_by_gpio(bool enable);
int ktz8868_set_lcd_bias_by_reg(bool enable);
int ktz8868_hw_en(bool enable);
#endif
