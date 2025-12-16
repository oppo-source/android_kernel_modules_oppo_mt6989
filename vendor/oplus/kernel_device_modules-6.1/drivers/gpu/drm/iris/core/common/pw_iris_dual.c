// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include "pw_iris_api.h"
#include "pw_iris_log.h"
#include "pw_iris_dual.h"
#include "pw_iris_memc.h"

void iris_inc_osd_irq_cnt_impl(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	atomic_inc(&pcfg->osd_irq_cnt);
	IRIS_LOGD("osd_irq: %d", atomic_read(&pcfg->osd_irq_cnt));
}

void iris_register_osd_irq_impl(void *disp)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg || !pcfg->iris_memc_ops.iris_register_osd_irq) {
		IRIS_LOGE("%s(), Invalid params", __func__);
		return;
	}
	pcfg->iris_memc_ops.iris_register_osd_irq(disp);
}

int iris_osd_auto_refresh_enable(u32 val)
{
	int osd_gpio = -1;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGV("%s(%d), value: %d", __func__, __LINE__, val);

	if (pcfg == NULL) {
		IRIS_LOGE("%s(), no secondary display.", __func__);
		return -EINVAL;
	}

	osd_gpio = pcfg->iris_osd_gpio;
	if (!gpio_is_valid(osd_gpio)) {
		IRIS_LOGE("%s(), invalid GPIO %d", __func__, osd_gpio);
		return -EINVAL;
	}

	if (iris_disable_mipi1_autorefresh_get()) {
		if (pcfg->iris_osd_autorefresh_enabled)
			disable_irq(gpio_to_irq(osd_gpio));
		pcfg->iris_osd_autorefresh_enabled = false;
		IRIS_LOGI("%s(), mipi1 autofresh force disable.", __func__);
		return 0;
	}

	if (val == 1) {
		IRIS_LOGD("%s(), enable osd auto refresh", __func__);
		enable_irq(gpio_to_irq(osd_gpio));
		pcfg->iris_osd_autorefresh_enabled = true;
	} else if (val == 2) {
		IRIS_LOGD("%s(), refresh frame from upper", __func__);
		enable_irq(gpio_to_irq(osd_gpio));
		pcfg->iris_osd_autorefresh_enabled = false;
	} else {
		IRIS_LOGD("%s(), disable osd auto refresh", __func__);
		disable_irq(gpio_to_irq(osd_gpio));
		pcfg->iris_osd_autorefresh_enabled = false;
		iris_osd_irq_cnt_init();
	}

	return 0;
}

int iris_osd_overflow_status_get(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (atomic_read(&pcfg->osd_irq_cnt) >= 1)
		pcfg->iris_osd_overflow_st = false;
	else
		pcfg->iris_osd_overflow_st = true;

	return pcfg->iris_osd_overflow_st;
}

void iris_osd_irq_cnt_init(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	atomic_set(&pcfg->osd_irq_cnt, 0);
	pcfg->iris_osd_overflow_st = false;
}

bool iris_is_display1_autorefresh_enabled_impl(bool is_secondary)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg || !is_secondary)
		return false;

	IRIS_LOGV("%s(), auto refresh: %s", __func__,
			pcfg->iris_cur_osd_autorefresh ? "true" : "false");
	if (!pcfg->iris_cur_osd_autorefresh) {
		pcfg->iris_osd_autorefresh_enabled = false;
		return false;
	}

	iris_osd_irq_cnt_init();

	return true;
}