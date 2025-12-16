/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "pw_iris_api.h"
#include "pw_iris_lightup.h"
#include "pw_iris_gpio.h"
#include "pw_iris_log.h"


#define IRIS_GPIO_HIGH 1
#define IRIS_GPIO_LOW  0
#define POR_CLOCK 180	/* 0.1 Mhz */

static int gpio_pulse_delay = 16 * 16 * 4 * 10 / POR_CLOCK;
static int gpio_cmd_delay = 10;

int iris_enable_pinctrl(void *dev, void *cfg)
{
	int rc = 0;
	struct platform_device *pdev = dev;
	struct iris_cfg *pcfg = cfg;

	pcfg->pinctrl.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pcfg->pinctrl.pinctrl)) {
		rc = PTR_ERR(pcfg->pinctrl.pinctrl);
		IRIS_LOGE("%s(), failed to get pinctrl, return: %d",
				__func__, rc);
		return -EINVAL;
	}

	pcfg->pinctrl.active = pinctrl_lookup_state(pcfg->pinctrl.pinctrl,
			"iris_active");
	if (IS_ERR_OR_NULL(pcfg->pinctrl.active)) {
		rc = PTR_ERR(pcfg->pinctrl.active);
		IRIS_LOGE("%s(), failed to get pinctrl active state, return: %d",
				__func__, rc);
		return -EINVAL;
	}

	pcfg->pinctrl.suspend = pinctrl_lookup_state(pcfg->pinctrl.pinctrl,
			"iris_suspend");
	if (IS_ERR_OR_NULL(pcfg->pinctrl.suspend)) {
		rc = PTR_ERR(pcfg->pinctrl.suspend);
		IRIS_LOGE("%s(), failed to get pinctrl suspend state, retrun: %d",
				__func__, rc);
		return -EINVAL;
	}

	return 0;
}

void iris_reset_sys_domain(void)
{
	iris_send_one_wired_cmd(IRIS_POWER_DOWN_SYS);
	usleep_range(3500, 3501);
	iris_send_one_wired_cmd(IRIS_POWER_UP_SYS);
	usleep_range(3500, 3501);
}

int iris_set_pinctrl_state(bool enable)
{
	int rc = 0;
	struct pinctrl_state *state;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (IS_ERR_OR_NULL(pcfg->pinctrl.pinctrl)) {
		IRIS_LOGE("%s(), failed to set pinctrl", __func__);
		return -EINVAL;
	}
	IRIS_LOGI("%s(), set state: %s", __func__,
			enable?"active":"suspend");
	if (enable)
		state = pcfg->pinctrl.active;
	else
		state = pcfg->pinctrl.suspend;

	rc = pinctrl_select_state(pcfg->pinctrl.pinctrl, state);
	if (rc)
		IRIS_LOGE("%s(), failed to set pin state %d, return: %d",
				__func__, enable, rc);

	return rc;
}

/* init one wired command GPIO */

int _iris_init_one_wired(struct device *dev)
{
	struct gpio_desc *one_wired_gpio = NULL;
	struct gpio_desc *one_wired_status_gpio = NULL;

	one_wired_gpio = devm_gpiod_get(dev, "iris-wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(one_wired_gpio)) {
		IRIS_LOGE("%s(%d), one wired GPIO not configured",
				__func__, __LINE__);
		return -EINVAL;
	}
	gpiod_set_value(one_wired_gpio, IRIS_GPIO_LOW);
	devm_gpiod_put(dev, one_wired_gpio);

	one_wired_status_gpio = devm_gpiod_get(dev, "iris-abyp-ready", GPIOD_IN);
	if (IS_ERR(one_wired_status_gpio)) {
		IRIS_LOGE("%s(%d), ABYP status GPIO not configured.",
				__func__, __LINE__);
		return -EINVAL;
	}
	devm_gpiod_put(dev, one_wired_status_gpio);
	return 0;
}

int iris_init_one_wired(void)
{
	struct mipi_dsi_device *dsi = iris_get_cfg()->dsi_dev;
	struct platform_device *pdev = iris_get_cfg()->pdev;

	if (dsi)
		_iris_init_one_wired(&dsi->dev);
	else if (pdev)
		_iris_init_one_wired(&pdev->dev);

	return 0;
}

/* send one wired commands via GPIO */
void iris_send_one_wired_cmd(IRIS_ONE_WIRE_TYPE type)
{
	int cnt = 0;
	u32 start_end_delay = 0;
	u32 pulse_delay = 0;
	unsigned long flags;
	struct iris_cfg *pcfg = iris_get_cfg();
	const int pulse_count[IRIS_ONE_WIRE_CMD_CNT] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	};
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;
	struct platform_device *pdev = pcfg->pdev;
	struct device *dev = dsi ? &dsi->dev : (pdev ? &pdev->dev : NULL);
	struct gpio_desc *one_wired_gpio =
		dev ? devm_gpiod_get(dev, "iris-wakeup", GPIOD_OUT_LOW) : NULL;

	if (IS_ERR(one_wired_gpio)) {
		IRIS_LOGE("%s(%d), one wired GPIO not configured",
				__func__, __LINE__);
		return;
	}

	start_end_delay = 16 * 16 * 16 * 10 / POR_CLOCK;  /*us*/
	pulse_delay = gpio_pulse_delay;  /*us*/

	IRIS_LOGI("%s(), type: %d, pulse delay: %d, gpio cmd delay: %d",
			__func__, type, pulse_delay, gpio_cmd_delay);

	spin_lock_irqsave(&pcfg->iris_1w_lock, flags);
	for (cnt = 0; cnt < pulse_count[type]; cnt++) {
		gpiod_set_value(one_wired_gpio, IRIS_GPIO_HIGH);
		udelay(pulse_delay);
		gpiod_set_value(one_wired_gpio, IRIS_GPIO_LOW);
		udelay(pulse_delay);
	}

	udelay(gpio_cmd_delay);
	spin_unlock_irqrestore(&pcfg->iris_1w_lock, flags);

	udelay(start_end_delay);

	devm_gpiod_put(dev, one_wired_gpio);
}

int iris_check_abyp_ready(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;
	struct platform_device *pdev = pcfg->pdev;
	struct device *dev = dsi ? &dsi->dev : (pdev ? &pdev->dev : NULL);
	struct gpio_desc *iris_ready_gpio;
	u32 value = 0;

	iris_ready_gpio =
		dev ? devm_gpiod_get(dev, "iris-abyp-ready", GPIOD_IN) : NULL;

	if (IS_ERR(iris_ready_gpio)) {
		IRIS_LOGE("%s(%d), iris ready GPIO not configured",
				__func__, __LINE__);
		return 1;
	}
	value = gpiod_get_value(iris_ready_gpio);

	devm_gpiod_put(dev, iris_ready_gpio);
	return value;
}

int iris_parse_gpio(void *dev, void *cfg)
{
#if 0
	struct platform_device *pdev = dev;
	struct device_node *of_node = pdev->dev.of_node;
	struct iris_cfg *pcfg = cfg;

	IRIS_LOGI("%s(), enter", __func__);

	pcfg->iris_wakeup_gpio = of_get_named_gpio(of_node,
			"iris-wakeup-gpio", 0);
	IRIS_LOGI("%s(), wakeup gpio %d", __func__,
			pcfg->iris_wakeup_gpio);
	if (!gpio_is_valid(pcfg->iris_wakeup_gpio));

		IRIS_LOGW("%s(), wake up gpio is not specified", __func__)
	pcfg->iris_abyp_ready_gpio = of_get_named_gpio(of_node,
			"iris-abyp-ready-gpio", 0);
	IRIS_LOGI("%s(), abyp ready status gpio %d", __func__,
			pcfg->iris_abyp_ready_gpio);
	if (!gpio_is_valid(pcfg->iris_abyp_ready_gpio))
		IRIS_LOGW("%s(), abyp ready gpio is not specified", __func__);

	pcfg->iris_reset_gpio = of_get_named_gpio(of_node,
			"iris-reset-gpio", 0);
	IRIS_LOGI("%s(), iris reset gpio %d", __func__,
			pcfg->iris_reset_gpio);
	if (!gpio_is_valid(pcfg->iris_reset_gpio))
		IRIS_LOGW("%s(), iris reset gpio is not specified", __func__);

	pcfg->iris_vdd_gpio = of_get_named_gpio(of_node,
			"iris-vdd-gpio", 0);
	IRIS_LOGI("%s(), iris vdd gpio %d", __func__,
			pcfg->iris_vdd_gpio);
	if (!gpio_is_valid(pcfg->iris_vdd_gpio))
		IRIS_LOGW("%s(), iris vdd gpio not specified", __func__);

	pcfg->iris_osd_gpio = pcfg->iris_abyp_ready_gpio;
	if (!gpio_is_valid(pcfg->iris_osd_gpio))
		IRIS_LOGW("%s(), osd gpio %d not specified",
				__func__, pcfg->iris_osd_gpio);

	IRIS_LOGI("%s(), exit", __func__);
#endif
	return 0;
}

void iris_request_gpio(void)
{
#if 0
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(), enter", __func__);

	if (gpio_is_valid(pcfg->iris_vdd_gpio)) {
		rc = gpio_request(pcfg->iris_vdd_gpio, "iris_vdd");
		if (rc)
			IRIS_LOGW("%s(), failed to load vdd, return: %d", __func__, rc);
	}

	if (gpio_is_valid(pcfg->iris_wakeup_gpio)) {
		rc = gpio_request(pcfg->iris_wakeup_gpio, "iris_wake_up");
		if (rc)
			IRIS_LOGW("%s(), failed to load wake up, return: %d", __func__, rc);
	}

	if (gpio_is_valid(pcfg->iris_abyp_ready_gpio)) {
		rc = gpio_request(pcfg->iris_abyp_ready_gpio, "iris_abyp_ready");
		if (rc)
			IRIS_LOGW("%s(), failed to load abyp ready, return: %d", __func__, rc);
	}

	if (gpio_is_valid(pcfg->iris_reset_gpio)) {
		rc = gpio_request(pcfg->iris_reset_gpio, "iris_reset");
		if (rc)
			IRIS_LOGW("%s(), failed to load reset, return: %d", __func__, rc);
	}
	IRIS_LOGI("%s(), exit", __func__);
#endif
}

void iris_release_gpio(void *cfg)
{
#if 0
	struct iris_cfg *pcfg = cfg;

	IRIS_LOGI("%s(), enter", __func__);

	if (gpio_is_valid(pcfg->iris_wakeup_gpio))
		gpio_free(pcfg->iris_wakeup_gpio);
	if (gpio_is_valid(pcfg->iris_abyp_ready_gpio))
		gpio_free(pcfg->iris_abyp_ready_gpio);
	if (gpio_is_valid(pcfg->iris_reset_gpio))
		gpio_free(pcfg->iris_reset_gpio);
	if (gpio_is_valid(pcfg->iris_vdd_gpio))
		gpio_free(pcfg->iris_vdd_gpio);

	IRIS_LOGI("%s(), exit", __func__);
#endif
}

bool iris_vdd_valid(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;
	struct platform_device *pdev = pcfg->pdev;
	struct device *dev = dsi ? &dsi->dev : (pdev ? &pdev->dev : NULL);
	struct gpio_desc *iris_vdd_gpio;

	iris_vdd_gpio =
		dev ? devm_gpiod_get(dev, "iris-vdd", GPIOD_OUT_HIGH) : NULL;

	if (IS_ERR(iris_vdd_gpio))
		return false;

	devm_gpiod_put(dev, iris_vdd_gpio);
	return true;
}

void iris_enable_vdd(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;
	struct platform_device *pdev = pcfg->pdev;
	struct device *dev = dsi ? &dsi->dev : (pdev ? &pdev->dev : NULL);
	struct gpio_desc *iris_vdd_gpio = NULL;

	IRIS_LOGI("%s(), vdd enable", __func__);

	iris_vdd_gpio =
		dev ? devm_gpiod_get(dev, "iris-vdd", GPIOD_OUT_HIGH) : NULL;
	if (IS_ERR(iris_vdd_gpio)) {
		IRIS_LOGE("%s(), unable to set iris vdd gpio", __func__);
		return;
	}
	gpiod_set_value(iris_vdd_gpio, IRIS_GPIO_HIGH);
	devm_gpiod_put(dev, iris_vdd_gpio);
}

void iris_disable_vdd(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;
	struct platform_device *pdev = pcfg->pdev;
	struct device *dev = dsi ? &dsi->dev : (pdev ? &pdev->dev : NULL);
	struct gpio_desc *iris_vdd_gpio = NULL;

	IRIS_LOGI("%s(), vdd disable", __func__);

	iris_vdd_gpio =
		dev ? devm_gpiod_get(dev, "iris-vdd", GPIOD_OUT_LOW) : NULL;
	if (IS_ERR(iris_vdd_gpio)) {
		IRIS_LOGE("%s(), unable to set iris vdd gpio", __func__);
		return;
	}
	gpiod_set_value(iris_vdd_gpio, IRIS_GPIO_LOW);
	devm_gpiod_put(dev, iris_vdd_gpio);
}



static void _iris_reset_chip(struct device *dev)
{
	struct gpio_desc *iris_reset_gpio =
		devm_gpiod_get(dev, "iris-reset", GPIOD_OUT_LOW);

	if (IS_ERR(iris_reset_gpio)) {
		IRIS_LOGE("%s(), unable to set iris reset gpio", __func__);
		return;
	}

	IRIS_LOGI("%s(), reset start", __func__);
	gpiod_set_value(iris_reset_gpio, IRIS_GPIO_LOW);
	usleep_range(1000, 1001);
	gpiod_set_value(iris_reset_gpio, IRIS_GPIO_HIGH);
	usleep_range(2000, 2001);

	devm_gpiod_put(dev, iris_reset_gpio);
	IRIS_LOGI("%s(), reset end", __func__);
}

void iris_reset_chip(void)
{
	struct mipi_dsi_device *dsi = iris_get_cfg()->dsi_dev;
	struct platform_device *pdev = iris_get_cfg()->pdev;

	if (dsi)
		_iris_reset_chip(&dsi->dev);
	else if (pdev)
		_iris_reset_chip(&pdev->dev);
}

void iris_reset(void *dev)
{
	//struct device *_dev = dev;
#if 0
	if (!iris_is_chip_supported())
		return;
#endif
	IRIS_LOGI("%s()", __func__);

	if (!dev)
	{
		iris_init_one_wired();
		iris_reset_chip();
	} else {
		_iris_init_one_wired(dev);
		_iris_reset_chip(dev);
	}

	return;
}
EXPORT_SYMBOL(iris_reset);

void _iris_reset_chip_off(struct device *dev)
{
	struct gpio_desc *iris_reset_gpio =
		devm_gpiod_get(dev, "iris-reset", GPIOD_OUT_HIGH);

	if (IS_ERR(iris_reset_gpio)) {
		IRIS_LOGE("%s(), unable to set iris reset gpio", __func__);
		return;
	}
	gpiod_set_value(iris_reset_gpio, 0);

	devm_gpiod_put(dev, iris_reset_gpio);
	IRIS_LOGI("%s(), reset end", __func__);
}


void iris_reset_off(void *dev)
{
	if (!dev) {
		struct mipi_dsi_device *dsi = iris_get_cfg()->dsi_dev;
		struct platform_device *pdev = iris_get_cfg()->pdev;

		if (dsi)
			_iris_reset_chip_off(&dsi->dev);
		else if (pdev)
			_iris_reset_chip_off(&pdev->dev);
	} else
		_iris_reset_chip_off(dev);
}
EXPORT_SYMBOL(iris_reset_off);

int iris_dbg_gpio_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("%s(), create debug dir for iris failed, error %ld",
					__func__, PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("pulse_delay", 0644, pcfg->dbg_root,
			(u32 *)&gpio_pulse_delay);
	debugfs_create_u32("cmd_delay", 0644, pcfg->dbg_root,
			(u32 *)&gpio_cmd_delay);

	return 0;
}
