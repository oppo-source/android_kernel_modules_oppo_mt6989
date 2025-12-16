// SPDX-License-Identifier: GPL-2.0
/*
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "ktz8868.h"

#define BL_I2C_ADDRESS                  0x11
#define KTZ8868_HW_GPIO_NAME            "pm-enable-gpios"
#define KTZ8868_BAIS_ENP_GPIO_NAME      "bias-enp-en-gpios"
#define KTZ8868_BAIS_ENN_GPIO_NAME      "bias-enn-en-gpios"
#define KTZ8868_IC_BL_LEVEL_MAX         (2047)
#define LCD_BL_I2C_ID_NAME              "lcd_bl"

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static struct i2c_client *g_i2c_client = NULL;
static int ktz8868_hw_en_gpio_num      = -1;
static int ktz8868_bais_enp_gpio_num   = -1;
static int ktz8868_bais_enn_gpio_num   = -1;
static DEFINE_MUTEX(read_lock);

extern unsigned long esd_flag;
/*****************************************************************************
 * Extern Area
 *****************************************************************************/

int ktz8868_write_byte(unsigned char i2c_client_addr, unsigned char value)
{
	int ret = 0;
	unsigned char write_data[2] = {0};

	if (NULL == g_i2c_client) {
		pr_err("[KTZ8868]%s: i2c_client is null! line=%d\n", __func__, __LINE__);
		return -EINVAL;
	}


	write_data[0] = i2c_client_addr;
	write_data[1] = value;
	ret = i2c_master_send(g_i2c_client, write_data, 2);
	if (ret < 0)
		pr_err("[KTZ8868]%s: i2c write data fail! line=%d\n", __func__, __LINE__);

	return ret;
}
EXPORT_SYMBOL(ktz8868_write_byte);

int ktz8868_read_byte(unsigned char i2c_client_addr, unsigned char *i2c_client_buf)
{
	int res = 0;

	if (NULL == g_i2c_client) {
		pr_err("[KTZ8868]%s: i2c_client is null!! line=%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	mutex_lock(&read_lock);

	res = i2c_master_send(g_i2c_client, &i2c_client_addr, 0x1);
	if (res <= 0) {
		mutex_unlock(&read_lock);
		pr_err("[KTZ8868]%s: i2c_master_send failed! res=%d, line=%d\n", __func__, res, __LINE__);
		return res;
	}

	res = i2c_master_recv(g_i2c_client, i2c_client_buf, 0x1);
	if (res <= 0) {
		mutex_unlock(&read_lock);
		pr_err("[KTZ8868]%s: i2c_master_recv failed! res=%d, line=%d\n", __func__, res, __LINE__);
		return res;
	}

	mutex_unlock(&read_lock);

	return res;
}
EXPORT_SYMBOL(ktz8868_read_byte);

int ktz8868_brightness_enable(bool enable)
{
	static bool ktz8868_enable_flag = false;

	if (esd_flag) {
		ktz8868_enable_flag = false;
		pr_info("[KTZ8868][esd]%s: flag=false\n", __func__);
	}

	if (enable) {
		if (!ktz8868_enable_flag) {
			ktz8868_write_byte(0x02, 0x53);
			ktz8868_write_byte(0x03, 0xCD);
			ktz8868_write_byte(0x11, 0x76);
			ktz8868_write_byte(0x15, 0xF8);
			ktz8868_enable_flag = true;
		}
	} else {
		ktz8868_write_byte(0x01, 0x00);
		ktz8868_enable_flag = false;
	}
	pr_info("[KTZ8868]%s: enable=%d flag:%d\n", __func__, enable, ktz8868_enable_flag);

	return 0;
}
EXPORT_SYMBOL(ktz8868_brightness_enable);

int ktz8868_set_brightness(unsigned int bl_lvl)
{
	static bool ktz8868_set_bl_flag = false;

	if (esd_flag) {
		ktz8868_set_bl_flag = false;
		pr_info("[KTZ8868][esd]%s: flag=false\n", __func__);
	}

	if (bl_lvl > 0) {
		if(ktz8868_set_bl_flag == false) {
			ktz8868_write_byte(0x08, 0xFF);
		}
		ktz8868_write_byte(0x04, bl_lvl & 0x07);
		ktz8868_write_byte(0x05, (bl_lvl >> 3) & 0xFF);
		if (ktz8868_set_bl_flag == false) {
			usleep_range(15000, 15100);
			ktz8868_write_byte(0x01, 0x01);
			ktz8868_set_bl_flag = true;
			pr_info("[KTZ8868]%s: flag=%d\n", __func__, ktz8868_set_bl_flag);
		}
	} else if (bl_lvl == 0) {
		/* Current ramp 256ms  */
		ktz8868_write_byte(0x03, 0xCD);
		ktz8868_write_byte(0x04, 0x00);
		ktz8868_write_byte(0x05, 0x00);

		if(ktz8868_set_bl_flag == true) {
			ktz8868_set_bl_flag = false;
			pr_info("[KTZ8868]%s: flag=%d\n", __func__, ktz8868_set_bl_flag);
		}
	}
	pr_info("[KTZ8868]%s: bl_lvl=%d\n", __func__, bl_lvl);

	return 0;
}
EXPORT_SYMBOL(ktz8868_set_brightness);

int ktz8868_set_lcd_bias_by_gpio(bool enable)
{
	int  rc = 0;;

	pr_info("[KTZ8868]%s: ++\n", __func__);

	if (enable) {
		usleep_range(1000, 1100);
		/* enable bl bais enp */
		if (gpio_is_valid(ktz8868_bais_enp_gpio_num)) {
			rc = gpio_direction_output(ktz8868_bais_enp_gpio_num, true);
			if (rc < 0) {
				pr_err("[KTZ8868]%s: unable to set bl_bais_enp to high rc=%d\n", __func__, rc);
				gpio_free(ktz8868_bais_enp_gpio_num);
			}
		}
		usleep_range(8000, 8100);
		/* enable bl bais enn */
		if (gpio_is_valid(ktz8868_bais_enn_gpio_num)) {
			rc = gpio_direction_output(ktz8868_bais_enn_gpio_num, true);
			if (rc < 0) {
				pr_err("[KTZ8868]%s: unable to set bl_bais_enn to high rc=%d\n", __func__, rc);
				gpio_free(ktz8868_bais_enn_gpio_num);
			}
		}
		usleep_range(10000, 10100);
	} else {
		usleep_range(2000, 2100);
		/* disable bl bais enn */
		if (gpio_is_valid(ktz8868_bais_enn_gpio_num)) {
			rc = gpio_direction_output(ktz8868_bais_enn_gpio_num, false);
			if (rc < 0) {
				pr_err("[KTZ8868]%s: unable to set bl_bais_enn to low rc=%d\n", __func__, rc);
				gpio_free(ktz8868_bais_enn_gpio_num);
			}
		}
		usleep_range(8000, 8100);
		/* disable bl bais enp */
		if (gpio_is_valid(ktz8868_bais_enp_gpio_num)) {
			rc = gpio_direction_output(ktz8868_bais_enp_gpio_num, false);
			if (rc < 0) {
				pr_err("[KTZ8868]%s: unable to  set bl_bais_enp to low rc=%d\n", __func__, rc);
				gpio_free(ktz8868_bais_enp_gpio_num);
			}
		}
		usleep_range(10000, 10100);
	}
	pr_info("[KTZ8868]%s: enable=%d --\n", __func__, enable);

	return 0;
}
EXPORT_SYMBOL(ktz8868_set_lcd_bias_by_gpio);

int ktz8868_set_lcd_bias_by_reg(bool enable)
{
	if (enable) {
		pr_info("[KTZ8868] enable lcd_enable_bias by reg\n");
		/* only config i2c0*/
		ktz8868_write_byte(0x0C, 0x32);/* LCD_BOOST_CFG */
		ktz8868_write_byte(0x0D, 0x28);/* OUTP_CFG，OUTP = 6.0V */
		ktz8868_write_byte(0x0E, 0x28);/* OUTN_CFG，OUTN = -6.0V */
		ktz8868_write_byte(0x09, 0x9E);/* enable OUTP */
	} else {
		pr_info("[KTZ8868] disable lcd_enable_bias by reg\n");
		ktz8868_write_byte(0x09, 0x9C);/* Disable OUTN */
		usleep_range(5000, 5100);
		ktz8868_write_byte(0x09, 0x98);/* Disable OUTP */
	}
	return 0;
}
EXPORT_SYMBOL(ktz8868_set_lcd_bias_by_reg);

int ktz8868_hw_en(bool enable)
{
	int ret = 0;
	u8 value  = enable ? 1 : 0;

	pr_info("[KTZ8868]%s: ++\n", __func__);

	if(gpio_is_valid(ktz8868_hw_en_gpio_num)) {
		ret = gpio_direction_output(ktz8868_hw_en_gpio_num, value);
		if(ret){
			pr_err("[KTZ8868]failed to set %s gpio %d, ret = %d\n", KTZ8868_HW_GPIO_NAME, value, ret);
			return ret;
		} else {
			pr_err("[KTZ8868]%s:set KTZ8868_HW_EN enable=%d succ\n", __func__, enable);
		}
		if (value) {
			usleep_range(1000, 1100);
			ktz8868_write_byte(0x0C, 0x28);/* LCD_BOOST_CFG */
			ktz8868_write_byte(0x0D, 0x1E);/* OUTP_CFG，OUTP = 6.0V */
			ktz8868_write_byte(0x0E, 0x1E);/* OUTN_CFG，OUTN = -6.0V */
			ktz8868_write_byte(0x09, 0x99);/* enable OUTP */
		}
	} else {
		pr_err("[KTZ8868]get KTZ8868_HW_EN gpio(%d) is not vaild\n", ktz8868_hw_en_gpio_num);
	}

	pr_info("[KTZ8868]%s: --\n", __func__);

	return 0;
}
EXPORT_SYMBOL(ktz8868_hw_en);

static int ktz8868_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_info("[KTZ8868]%s: ++\n", __func__);

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[KTZ8868]%s: No I2C_FUNC_I2C support line=%d\n", __func__, __LINE__);
		return -EIO;
	}

	if(NULL == client) {
		pr_err("[KTZ8868]%s: client is NULL! line=%d\n", __func__, __LINE__);
		return -EIO;
	}
	g_i2c_client = client;

	if (client->dev.of_node) {
		ktz8868_hw_en_gpio_num = of_get_named_gpio(client->dev.of_node, KTZ8868_HW_GPIO_NAME, 0);
		if (ktz8868_hw_en_gpio_num < 0) {
			pr_err("[KTZ8868]%s: failed to get %s\n", __func__, KTZ8868_HW_GPIO_NAME);
		} else {
			pr_info("[KTZ8868]%s:get %s num=%d SUCC!\n", __func__, KTZ8868_HW_GPIO_NAME, ktz8868_hw_en_gpio_num);
		}

		ktz8868_bais_enp_gpio_num = of_get_named_gpio(client->dev.of_node, KTZ8868_BAIS_ENP_GPIO_NAME, 0);
		if (ktz8868_bais_enp_gpio_num < 0) {
			pr_err("[KTZ8868]%s: failed to get %s\n", __func__, KTZ8868_BAIS_ENP_GPIO_NAME);
		} else {
			pr_info("[KTZ8868]%s:get %s num=%d SUCC!\n", __func__, KTZ8868_BAIS_ENP_GPIO_NAME, ktz8868_bais_enp_gpio_num);
		}
		ktz8868_bais_enn_gpio_num = of_get_named_gpio(client->dev.of_node, KTZ8868_BAIS_ENN_GPIO_NAME, 0);
		if (ktz8868_bais_enn_gpio_num < 0) {
			pr_err("[KTZ8868]%s: failed to get %s\n", KTZ8868_BAIS_ENN_GPIO_NAME, __func__);
		} else {
			pr_info("[KTZ8868]%s:get %s num=%d SUCC!\n", __func__, KTZ8868_BAIS_ENN_GPIO_NAME, ktz8868_bais_enn_gpio_num);
		}
	}
	pr_info("[KTZ8868]%s: --\n", __func__);

	return 0;
}

static void ktz8868_i2c_remove(struct i2c_client *client)
{
	i2c_unregister_device(client);
	g_i2c_client = NULL;
	pr_info("[KTZ8868]%s: dev_name:%s unregister\n", __func__, dev_name(&client->dev));
}

/************************************************************
Attention:
Althouh i2c_bus do not use .id_table to match, but it must be defined,
otherwise the probe function will not be executed!
************************************************************/
static const struct i2c_device_id ktz8868_i2c_id_table[] = {
	{LCD_BL_I2C_ID_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ktz8868_i2c_id_table);

static const struct of_device_id ktz8868_i2c_of_match[] = {
	{ .compatible = "ktz,ktz8868", },
	{},
};
MODULE_DEVICE_TABLE(of, ktz8868_i2c_of_match);

static struct i2c_driver ktz8868_i2c_driver = {
	.probe = ktz8868_i2c_probe,
	.remove = ktz8868_i2c_remove,
	.id_table = ktz8868_i2c_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.name = LCD_BL_I2C_ID_NAME,
		.of_match_table = ktz8868_i2c_of_match,
    },
};

int __init ktz8868_init(void)
{
	pr_info("[KTZ8868]%s: ++\n", __func__);

	g_i2c_client = NULL;
	if (i2c_add_driver(&ktz8868_i2c_driver)) {
		pr_err("[KTZ8868]%s: Failed to register ktz8868_i2c_driver! line=%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	pr_info("[KTZ8868]%s: --\n", __func__);

	return 0;
}

void __exit ktz8868_exit(void)
{
	i2c_del_driver(&ktz8868_i2c_driver);
	g_i2c_client = NULL;
}

module_init(ktz8868_init);
module_exit(ktz8868_exit);

MODULE_AUTHOR("zhangzepu <zhangzepu@vanyol.com>");
MODULE_DESCRIPTION("Mediatek LCD BL I2C Driver");
MODULE_LICENSE("GPL");



