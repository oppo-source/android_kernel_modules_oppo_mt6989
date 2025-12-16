// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#ifndef _PW_IRIS_MEMC_COMMON_
#define _PW_IRIS_MEMC_COMMON_

enum mcu_state_op {
	MCU_START,
	MCU_INT_DISABLE,
	MCU_STOP,
};

void iris_pt_sr_set(int enable, int processWidth, int processHeight);
int iris_configure_memc(u32 type, u32 value);
int iris_configure_ex_memc(u32 type, u32 count, u32 *values);
int iris_configure_get_memc(u32 type, u32 count, u32 *values);
void iris_init_memc(void);
void iris_lightoff_memc(void);
void iris_enable_memc(bool is_secondary);
void iris_sr_update(void);
void iris_frc_setting_init(void);
int iris_dbgfs_memc_init(void);
void iris_parse_memc_param(void);
void iris_frc_timing_setting_update(void);
void iris_pt_sr_reset(void);
void iris_mcu_state_set(u32 mode);
void iris_mcu_ctrl_set(u32 ctrl);
void iris_memc_vfr_video_update_monitor(struct iris_cfg *pcfg, bool is_secondary);
int iris_low_latency_mode_get(void);
bool iris_health_care(void);
int iris_demo_wnd_conf(u32 count, u32 *values);
void iris_demo_wnd_set_i5(void);
void iris_demo_wnd_set_i7(void);
void iris_demo_wnd_set_i7p(void);
void iris_demo_wnd_set_i8(void);
void iris_memc_func_init_i5(struct iris_memc_func *memc_func);
void iris_memc_func_init_i7(struct iris_memc_func *memc_func);
void iris_memc_func_init_i7p(struct iris_memc_func *memc_func);
void iris_memc_func_init_i8(struct iris_memc_func *memc_func);
#endif
