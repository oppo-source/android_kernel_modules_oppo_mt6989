/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#ifndef _PW_IRIS_DUAL_COMMON_
#define _PW_IRIS_DUAL_COMMON_

int iris_osd_auto_refresh_enable(u32 val);
int iris_osd_overflow_status_get(void);
void iris_osd_irq_cnt_init(void);
bool iris_is_display1_autorefresh_enabled_impl(bool is_secondary);
void iris_inc_osd_irq_cnt_impl(void);
void iris_register_osd_irq_impl(void *disp);
u32 iris_disable_mipi1_autorefresh_get(void);

#endif // _PW_IRIS_DUAL_COMMON_