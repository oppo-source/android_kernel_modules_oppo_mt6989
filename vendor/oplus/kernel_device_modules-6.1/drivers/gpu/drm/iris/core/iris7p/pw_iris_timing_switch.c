// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#include "pw_iris_def.h"
#include "pw_iris_timing_switch_def.h"


void iris_init_timing_switch_i7p(void)
{
	IRIS_LOGI("%s()", __func__);
}

void iris_send_dynamic_seq_i7p(void)
{
	IRIS_LOGI("%s()", __func__);
}

void iris_timing_switch_setup_i7p(struct iris_timing_switch_ops *timing_switch_ops)
{
	timing_switch_ops->iris_init_timing_switch_cb = iris_init_timing_switch_i7p;
	timing_switch_ops->iris_send_dynamic_seq = iris_send_dynamic_seq_i7p;
	timing_switch_ops->iris_pre_switch = NULL;
	timing_switch_ops->iris_pre_switch_proc = NULL;
	timing_switch_ops->iris_post_switch_proc = NULL;
	timing_switch_ops->iris_set_tm_sw_dbg_param = NULL;
	timing_switch_ops->iris_restore_capen = NULL;
}
