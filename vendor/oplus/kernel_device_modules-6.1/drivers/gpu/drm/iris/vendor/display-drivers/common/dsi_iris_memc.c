// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/types.h>
#include "mtk_dsi.h"
#include "mtk_drm_drv.h"
#include "mtk_drm_crtc.h"

#include "dsi_iris_api.h"
#include "dsi_iris_lightup.h"
#include "pw_iris_lp.h"
#include "pw_iris_log.h"
#include "dsi_iris_memc.h"

int iris_get_main_panel_timing_info(struct iris_panel_timing_info *timing_info)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg || !timing_info) {
		IRIS_LOGE("%s(), cannot get timing info of curr mode!", __func__);
		return -EINVAL;
	}

	timing_info->flag = timing_info->flag; //not change
	timing_info->width = pcfg->timing.h_active;
	timing_info->height = pcfg->timing.v_active;
	timing_info->fps = pcfg->timing.refresh_rate;
	timing_info->dsc = pcfg->timing.dsc_enabled;
	timing_info->h_back_porch = pcfg->timing.h_back_porch;
	timing_info->h_sync_width = pcfg->timing.h_sync_width;
	timing_info->h_front_porch = pcfg->timing.h_front_porch;
	timing_info->v_back_porch = pcfg->timing.v_back_porch;
	timing_info->v_sync_width = pcfg->timing.v_sync_width;
	timing_info->v_front_porch = pcfg->timing.v_front_porch;

	IRIS_LOGD("%s(), flag: %d, width: %d, height: %d, fps: %d, dsc_en: %d,"
		"h_back_porch: %d, h_sync_width: %d, h_front_porch: %d, v_back_porch: %d,"
		"v_sync_width: %d, v_front_porch: %d", __func__, timing_info->flag, timing_info->width,
		timing_info->height, timing_info->fps, timing_info->dsc, timing_info->h_back_porch,
		timing_info->h_sync_width, timing_info->h_front_porch, timing_info->v_back_porch,
		timing_info->v_sync_width, timing_info->v_front_porch);

	return 0;
}

int iris_get_main_panel_curr_mode_dsc_en(bool *dsc_en)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	if (pcfg_ven && pcfg_ven->panel_ext && pcfg_ven->panel_ext->params)
		*dsc_en = pcfg_ven->panel_ext->params->dsc_params.enable;
	else
		*dsc_en = false;

	IRIS_LOGD("%s(), dsc_en: %d", __func__, *dsc_en);

	return 0;
}

int iris_get_aux_panel_timing_info(struct iris_panel_timing_info *timing_info)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct drm_display_mode *mode = NULL;
	struct mtk_drm_private *private = NULL;
	struct drm_crtc *crtc = NULL;
	struct mtk_crtc_state *mtk_state = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct iris_mode_info timing;

	if (!pcfg_ven || !pcfg_ven->drm || !pcfg_ven->drm->dev_private || !timing_info) {
		IRIS_LOGE("%s:%d, Invalid parms!",__func__, __LINE__);
		return -EINVAL;
	}

	private = pcfg_ven->drm->dev_private;
	crtc = private->crtc[3];
	if (!crtc) {
		IRIS_LOGE("%s:%d, Invalid parms",__func__, __LINE__);
		return -EINVAL;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc) {
		IRIS_LOGE("%s:%d, Invalid parms!",__func__, __LINE__);
		return -EINVAL;
	}

	mtk_state = to_mtk_crtc_state(mtk_crtc->base.state);
	if (!mtk_state) {
		IRIS_LOGE("%s:%d, Invalid parms!",__func__, __LINE__);
		return -EINVAL;
	}

	mode =  mtk_drm_crtc_avail_disp_mode(crtc, mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);
	if (mode == NULL) {
		IRIS_LOGE("%s:%d, get display mode failed!",__func__, __LINE__);
		return -EINVAL;
	}

	iris_sync_aux_timing(&timing, mode);
	timing_info->flag = timing_info->flag; //not change
	timing_info->width = timing.h_active;
	timing_info->height = timing.v_active;
	timing_info->fps = timing.refresh_rate;
	timing_info->dsc = timing.dsc_enabled;
	timing_info->h_back_porch = timing.h_back_porch;
	timing_info->h_sync_width = timing.h_sync_width;
	timing_info->h_front_porch = timing.h_front_porch;
	timing_info->v_back_porch = timing.v_back_porch;
	timing_info->v_sync_width = timing.v_sync_width;
	timing_info->v_front_porch = timing.v_front_porch;

	IRIS_LOGD("%s(), flag: %d, width: %d, height: %d, fps: %d, dsc_en: %d,"
		"h_back_porch: %d, h_sync_width: %d, h_front_porch: %d, v_back_porch: %d,"
		"v_sync_width: %d, v_front_porch: %d", __func__, timing_info->flag, timing_info->width,
		timing_info->height, timing_info->fps, timing_info->dsc, timing_info->h_back_porch,
		timing_info->h_sync_width, timing_info->h_front_porch, timing_info->v_back_porch,
		timing_info->v_sync_width, timing_info->v_front_porch);

	return 0;
}

int iris_get_aux_panel_curr_mode_dsc_en(bool *dsc_en)
{
	struct mtk_panel_params *ext_panel_params = NULL;

	ext_panel_params = iris_get_ext_panel_params();
	if (ext_panel_params)
		*dsc_en =  ext_panel_params->dsc_params.enable;
	else
		*dsc_en = false;
	IRIS_LOGD("%s(), dsc_en: %d", __func__, *dsc_en);

	return 0;
}

int iris_get_aux_panel_curr_mode_dsc_size(uint32_t *slice_width, uint32_t *slice_height)
{
	struct mtk_panel_params *ext_panel_params = NULL;

	ext_panel_params = iris_get_ext_panel_params();
	if (ext_panel_params) {
		*slice_width = ext_panel_params->dsc_params.slice_width;
		*slice_height = ext_panel_params->dsc_params.slice_height;
		IRIS_LOGD("%s(), slice_width: %d, slice_height:%d",
			__func__, *slice_width, *slice_height);
	} else
		return -EINVAL;
	return 0;
}
