// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _DSI_IRIS_MEMC_COMMON_
#define _DSI_IRIS_MEMC_COMMON_
int iris_get_main_panel_timing_info(struct iris_panel_timing_info *timing_info);
int iris_get_main_panel_curr_mode_dsc_en(bool *dsc_en);
int iris_get_aux_panel_timing_info(struct iris_panel_timing_info *timing_info);
int iris_get_aux_panel_curr_mode_dsc_en(bool *dsc_en);
int iris_get_aux_panel_curr_mode_dsc_size(uint32_t *slice_width, uint32_t *slice_height);
unsigned long long iris_set_idle_check_interval(unsigned int crtc_id, unsigned long long new_interval);
//memc dual
void iris_set_idlemgr(unsigned int crtc_id, unsigned int enable, bool need_lock);
#endif
