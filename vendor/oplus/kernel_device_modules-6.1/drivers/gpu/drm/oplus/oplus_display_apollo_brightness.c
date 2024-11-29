/***************************************************************
** Copyright (C),  2023,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_high_frequency_pwm.c
** Description : oplus high frequency PWM
** Version : 1.0
** Date : 2023/05/23
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Ping      2023/05/23        1.0           Build this moudle
******************************************************************/
#include <linux/delay.h>
#include <linux/ktime.h>
#include "mtk_panel_ext.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_drv.h"
#include "mtk_debug.h"
#include "mtk_drm_trace.h"
#include "oplus_display_trace.h"
#include "oplus_display_apollo_brightness.h"
#include "oplus_display_high_frequency_pwm.h"
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

extern int ffl_backlight_backup;
extern struct oplus_demura_setting_table demura_setting;
extern void oplus_panel_backlight_demura_dbv_switch(struct drm_crtc *crtc, struct cmdq_pkt *cmdq_handle, unsigned int level);
extern unsigned int oplus_bl_demura_dbv_switched;
extern void apollo_set_brightness_for_show(unsigned int level);
extern u16 mtk_get_gpr(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle);
extern int oplus_display_panel_set_pwm_bl(struct drm_crtc *crtc, struct cmdq_pkt *cmdq_handle, unsigned int level, bool is_sync);

static void oplus_bl_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

void oplus_display_apollo_init_para(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!crtc) {
		DDPPR_ERR("oplus_display_apollo_crtc_init_para init fail\n");
		return;
	}

	mtk_crtc->oplus_apollo_br = kzalloc(sizeof(struct oplus_apollo_brightness),
			GFP_KERNEL);
	DDPMSG("%s: oplus_apollo_brightness need allocate memory\n", __func__);

	if (!mtk_crtc->oplus_apollo_br) {
		DDPPR_ERR("%s: oplus_apollo_brightness allocate memory fail\n", __func__);
		return;
	}

	mtk_crtc->oplus_apollo_br->oplus_power_on = true;
	mtk_crtc->oplus_apollo_br->oplus_refresh_rate_switching = false;
	mtk_crtc->oplus_apollo_br->oplus_te_tag_ns = 0;
	mtk_crtc->oplus_apollo_br->oplus_te_diff_ns = 0;
	mtk_crtc->oplus_apollo_br->cur_vsync = 0;
	mtk_crtc->oplus_apollo_br->limit_superior_ns = 0;
	mtk_crtc->oplus_apollo_br->limit_inferior_ns = 0;
	mtk_crtc->oplus_apollo_br->transfer_time_us = 0;
	mtk_crtc->oplus_apollo_br->pending_vsync = 0;
	mtk_crtc->oplus_apollo_br->pending_limit_superior_ns = 0;
	mtk_crtc->oplus_apollo_br->pending_limit_inferior_ns = 0;
	mtk_crtc->oplus_apollo_br->pending_transfer_time_us = 0;
}

int oplus_display_backlight_property_update(struct drm_crtc *crtc, int prop_id, unsigned int prop_val)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!crtc) {
		DDPPR_ERR("backlight_property_update - find crtc fail\n");
		return 0;
	}

	if (!mtk_crtc->oplus_apollo_br) {
		mtk_crtc->oplus_apollo_br = kzalloc(sizeof(struct oplus_apollo_brightness),
			GFP_KERNEL);
		DDPMSG("%s: need allocate memory\n", __func__);
	}
	if (!mtk_crtc->oplus_apollo_br) {
		DDPPR_ERR("%s: allocate memory fail\n", __func__);
		return 0;
	}

	switch (prop_id) {
	case CRTC_PROP_HW_BRIGHTNESS:
		mtk_crtc->oplus_apollo_br->oplus_backlight_updated = true;
		mtk_crtc->oplus_apollo_br->oplus_pending_backlight = prop_val;
		apollo_set_brightness_for_show(prop_val);
		break;
	case CRTC_PROP_BRIGHTNESS_NEED_SYNC:
		mtk_crtc->oplus_apollo_br->oplus_backlight_need_sync = !!prop_val;
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(oplus_display_backlight_property_update);

void oplus_sync_panel_brightness(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!crtc) {
		DDPPR_ERR("sync_panel_brightness - find crtc fail\n");
		return;
	}

	if (!mtk_crtc->oplus_apollo_br) {
		mtk_crtc->oplus_apollo_br = kzalloc(sizeof(struct oplus_apollo_brightness),
			GFP_KERNEL);
		DDPMSG("%s: oplus_apollo_brightness need allocate memory\n", __func__);
	}

	if (!mtk_crtc->oplus_apollo_br) {
		DDPPR_ERR("%s: oplus_apollo_brightness allocate memory fail\n", __func__);
		return;
	}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		oplus_ofp_lhbm_backlight_update(crtc);
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params && mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps != 0) {
		mtk_crtc->oplus_apollo_br->pending_vsync = 1000000000 / mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
		mtk_crtc->oplus_apollo_br->pending_limit_superior_ns = mtk_crtc->panel_ext->params->dyn_fps.apollo_limit_superior_us * 1000;
		mtk_crtc->oplus_apollo_br->pending_limit_inferior_ns = mtk_crtc->panel_ext->params->dyn_fps.apollo_limit_inferior_us * 1000;
		mtk_crtc->oplus_apollo_br->pending_transfer_time_us = mtk_crtc->panel_ext->params->dyn_fps.apollo_transfer_time_us;
	} else {
		DDPPR_ERR("%s fps get failed! \n", __func__);
	}

	if (!mtk_crtc->oplus_apollo_br->oplus_backlight_updated) {
		return;
	}

	/* oplus_sync_panel_brightness must after mode switch! */
	if (mtk_crtc->oplus_apollo_br->oplus_refresh_rate_switching) {
		if (atomic_read(&mtk_crtc->singal_for_mode_switch)) {
			wait_event_interruptible(mtk_crtc->mode_switch_end_wq,
				(atomic_read(&mtk_crtc->singal_for_mode_switch) == 0));
		}
	}

	oplus_disp_trace_begin("sync_panel_brightness level(%d) sync(%d)", mtk_crtc->oplus_apollo_br->oplus_pending_backlight,
							mtk_crtc->oplus_apollo_br->oplus_backlight_need_sync);
	mtk_drm_setbacklight_without_lock(crtc, mtk_crtc->oplus_apollo_br->oplus_pending_backlight, 0, 0x1 << SET_BACKLIGHT_LEVEL);
	oplus_disp_trace_end("sync_panel_brightness level(%d) sync(%d)", mtk_crtc->oplus_apollo_br->oplus_pending_backlight,
							mtk_crtc->oplus_apollo_br->oplus_backlight_need_sync);
}
EXPORT_SYMBOL(oplus_sync_panel_brightness);

void oplus_update_apollo_para(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!crtc) {
		DDPPR_ERR("sync_panel_brightness - find crtc fail\n");
		return;
	}

	if (!mtk_crtc->oplus_apollo_br) {
		mtk_crtc->oplus_apollo_br = kzalloc(sizeof(struct oplus_apollo_brightness),
			GFP_KERNEL);
		DDPMSG("%s: oplus_apollo_brightness need allocate memory\n", __func__);
	}

	if (!mtk_crtc->oplus_apollo_br) {
		DDPPR_ERR("%s: oplus_apollo_brightness allocate memory fail\n", __func__);
		return;
	}

	mtk_crtc->oplus_apollo_br->oplus_refresh_rate_switching = false;
	mtk_crtc->oplus_apollo_br->cur_vsync = mtk_crtc->oplus_apollo_br->pending_vsync;
	mtk_crtc->oplus_apollo_br->limit_superior_ns = mtk_crtc->oplus_apollo_br->pending_limit_superior_ns;
	mtk_crtc->oplus_apollo_br->limit_inferior_ns = mtk_crtc->oplus_apollo_br->pending_limit_inferior_ns;
	mtk_crtc->oplus_apollo_br->transfer_time_us = mtk_crtc->oplus_apollo_br->pending_transfer_time_us;
}
EXPORT_SYMBOL(oplus_update_apollo_para);

void oplus_printf_backlight_log(struct drm_crtc *crtc, unsigned int bl_lvl) {
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct mtk_panel_params *params = mtk_crtc->panel_ext->params;
	struct timespec64 now;
	struct tm broken_time;
	static time64_t time_last = 0;
	struct backlight_log *bl_log;
	int i = 0;
	int len = 0;
	char backlight_log_buf[1024];

	if (!comp) {
		DDPPR_ERR("failed to get comp\n");
		return;
	}

	ktime_get_real_ts64(&now);
	time64_to_tm(now.tv_sec, 0, &broken_time);
	if (now.tv_sec - time_last >= 60) {
		pr_info("<%s> dsi_display_set_backlight time:%02d:%02d:%02d.%03ld,bl_lvl:%d\n",
				params->vendor, broken_time.tm_hour, broken_time.tm_min,
				broken_time.tm_sec, now.tv_nsec / 1000000, bl_lvl);
		time_last = now.tv_sec;
	}

	if (comp->id == DDP_COMPONENT_DSI1) {
		bl_log = &oplus_bl_log[DISPLAY_SECONDARY];
	} else {
		bl_log = &oplus_bl_log[DISPLAY_PRIMARY];
	}


	bl_log->backlight[bl_log->bl_count] = bl_lvl;
	bl_log->past_times[bl_log->bl_count] = now;
	bl_log->bl_count++;
	if (bl_log->bl_count >= BACKLIGHT_CACHE_MAX) {
		bl_log->bl_count = 0;
		memset(backlight_log_buf, 0, sizeof(backlight_log_buf));
		for (i = 0; i < BACKLIGHT_CACHE_MAX; i++) {
			time64_to_tm(bl_log->past_times[i].tv_sec, 0, &broken_time);
			len += snprintf(backlight_log_buf + len, sizeof(backlight_log_buf) - len,
				"%02d:%02d:%02d.%03ld:%d,", broken_time.tm_hour, broken_time.tm_min,
				broken_time.tm_sec, bl_log->past_times[i].tv_nsec / 1000000, bl_log->backlight[i]);
		}
		pr_info("<%s> len:%d dsi_display_set_backlight %s\n", params->vendor, len, backlight_log_buf);
	}
}

int  mtk_drm_setbacklight_without_lock(struct drm_crtc *crtc, unsigned int level,
	unsigned int panel_ext_param, unsigned int cfg_flag)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct mtk_ddp_comp *oddmr_comp;
	struct mtk_cmdq_cb_data *cb_data;
	struct mtk_bl_ext_config bl_ext_config;
	static unsigned int bl_cnt;
	int delay_us, time_gap_ns;
	bool is_frame_mode;
	struct cmdq_client *client;
	int index = drm_crtc_index(crtc);
	int ret = 0;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	bool adfr_fps_need_sync = false;

	if (mtk_crtc->oplus_apollo_br->oplus_power_on == false) {
		level = 0;
	}
	CRTC_MMP_EVENT_START(index, backlight, (unsigned long)crtc,
			level);

	if (!(mtk_crtc->enabled)) {
		DDPINFO("Sleep State set backlight stop --crtc not ebable\n");
		CRTC_MMP_EVENT_END(index, backlight, 0, 0);

		return -EINVAL;
	}

	if (!comp) {
		DDPINFO("%s no output comp\n", __func__);
		CRTC_MMP_EVENT_END(index, backlight, 0, 1);

		return -EINVAL;
	}

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPPR_ERR("cb data creation failed\n");
		CRTC_MMP_EVENT_END(index, backlight, 0, 2);
		return -EINVAL;
	}

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);
	if (is_frame_mode) {
		if (atomic_read(&mtk_crtc->singal_for_mode_switch)) {
			DDPINFO("%s:Wait event from mode_switch\n", __func__);
			wait_event_interruptible(mtk_crtc->mode_switch_end_wq,
				(atomic_read(&mtk_crtc->singal_for_mode_switch) == 0));
		}
	}

	/* setbacklight would use CLIENT_DSI_CFG in  DSI VDO mode only */
	client = (is_frame_mode) ? mtk_crtc->gce_obj.client[CLIENT_CFG] :
				mtk_crtc->gce_obj.client[CLIENT_DSI_CFG];
	mtk_crtc_pkt_create(&cmdq_handle, crtc, client);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, 0);

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}
	ffl_backlight_backup = level;
	drm_trace_tag_value("sync_backlight", level);

	if (mtk_crtc->oplus_apollo_br->cur_vsync == 0) {
		mtk_crtc->oplus_apollo_br->cur_vsync = mtk_crtc->oplus_apollo_br->pending_vsync;
		mtk_crtc->oplus_apollo_br->limit_superior_ns = mtk_crtc->oplus_apollo_br->pending_limit_superior_ns;
		mtk_crtc->oplus_apollo_br->limit_inferior_ns = mtk_crtc->oplus_apollo_br->pending_limit_inferior_ns;
		mtk_crtc->oplus_apollo_br->transfer_time_us = mtk_crtc->oplus_apollo_br->pending_transfer_time_us;
	}

	if (strcmp(mtk_crtc->panel_ext->params->vendor, "A0004") == 0) {
		if (mtk_crtc->oplus_apollo_br->cur_vsync == APOLLO_CUR_VSYNC_60) {
			adfr_fps_need_sync = true;
		}
	}

	oplus_bl_demura_dbv_switched = 0;
	if (demura_setting.oplus_bl_demura_dbv_support) {
		if (oplus_ofp_is_supported()) {
			if (oplus_ofp_get_hbm_state()) {
				pr_info("ofp_backlight_filter(1), hbm has sent demura, ignore current backlight with demura\n");
			} else {
				oplus_panel_backlight_demura_dbv_switch(crtc, cmdq_handle, level);
				if (oplus_bl_demura_dbv_switched) {
					delay_us = mtk_crtc->oplus_apollo_br->cur_vsync / 2 / 1000;
					oplus_disp_trace_begin("DSI_SET_DEMURA_BL cmdq sleep %d us", delay_us);
					cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), mtk_get_gpr(comp, cmdq_handle));
					oplus_disp_trace_end("DSI_SET_DEMURA_BL cmdq sleep %d us", delay_us);
				}
			}
		} else {
			oplus_panel_backlight_demura_dbv_switch(crtc, cmdq_handle, level);
			if (oplus_bl_demura_dbv_switched) {
				delay_us = mtk_crtc->oplus_apollo_br->cur_vsync / 2 / 1000;
				oplus_disp_trace_begin("DSI_SET_DEMURA_BL cmdq sleep %d us", delay_us);
				cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), mtk_get_gpr(comp, cmdq_handle));
				oplus_disp_trace_end("DSI_SET_DEMURA_BL cmdq sleep %d us", delay_us);
			}
		}
	}

	if ((mtk_crtc->oplus_apollo_br->oplus_backlight_need_sync || adfr_fps_need_sync)
	&& mtk_crtc->oplus_apollo_br->cur_vsync != 0
	&& oplus_bl_demura_dbv_switched == 0) {
		/* backlight sync start */
		oplus_disp_trace_begin("cur_vsync(%d) pending_vsync(%d)", mtk_crtc->oplus_apollo_br->cur_vsync, mtk_crtc->oplus_apollo_br->pending_vsync);
		if (mtk_crtc->oplus_apollo_br->oplus_refresh_rate_switching) {
			usleep_range(mtk_crtc->oplus_apollo_br->cur_vsync/1000, mtk_crtc->oplus_apollo_br->cur_vsync/1000 + 100);
		}

		time_gap_ns = ktime_get() > mtk_crtc->oplus_apollo_br->oplus_te_tag_ns ? ktime_get() - mtk_crtc->oplus_apollo_br->oplus_te_tag_ns : 0;
		if (time_gap_ns >= 0 && time_gap_ns <= mtk_crtc->oplus_apollo_br->cur_vsync) {
			if (time_gap_ns < mtk_crtc->oplus_apollo_br->limit_superior_ns) {
				if ((mtk_crtc->oplus_apollo_br->oplus_te_tag_ns > mutex_sof_ns
						&& mtk_crtc->oplus_apollo_br->oplus_te_tag_ns - mutex_sof_ns < mtk_crtc->oplus_apollo_br->cur_vsync / 2)
					|| (mutex_sof_ns > mtk_crtc->oplus_apollo_br->oplus_te_tag_ns
						&& mutex_sof_ns - mtk_crtc->oplus_apollo_br->oplus_te_tag_ns < mtk_crtc->oplus_apollo_br->cur_vsync / 2)) {
					delay_us = mtk_crtc->panel_ext->params->dyn_fps.apollo_limit_superior_us - mtk_crtc->oplus_apollo_br->transfer_time_us;
				} else {
					delay_us = (mtk_crtc->oplus_apollo_br->limit_superior_ns - time_gap_ns) / 1000;
				}
				if (delay_us > 0) {
					oplus_disp_trace_begin("cmdq sleep %d us-0", delay_us);
					cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), mtk_get_gpr(comp, cmdq_handle));
					oplus_disp_trace_end("cmdq sleep %d us-0", delay_us);
				}
			} else if (time_gap_ns > mtk_crtc->oplus_apollo_br->limit_inferior_ns) {
				delay_us = (mtk_crtc->oplus_apollo_br->cur_vsync - time_gap_ns) / 1000 + mtk_crtc->panel_ext->params->dyn_fps.apollo_limit_superior_us;
				if (delay_us > 0) {
					oplus_disp_trace_begin("cmdq sleep %d us-1", delay_us);
					cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), mtk_get_gpr(comp, cmdq_handle));
					oplus_disp_trace_end("cmdq sleep %d us-1", delay_us);
				}
			}
		} else {
			pr_err("%s backlight sync failed ! time_gap_ns(%d) is large than cur_vsync(%d)\n", __func__, time_gap_ns, mtk_crtc->oplus_apollo_br->cur_vsync);
		}
		oplus_disp_trace_end("cur_vsync(%d) pending_vsync(%d)", mtk_crtc->oplus_apollo_br->cur_vsync, mtk_crtc->oplus_apollo_br->pending_vsync);
		/* backlight sync end */
	}

	oddmr_comp = priv->ddp_comp[DDP_COMPONENT_ODDMR0];
	mtk_ddp_comp_io_cmd(oddmr_comp, cmdq_handle, ODDMR_BL_CHG, &level);

	oplus_printf_backlight_log(crtc, level);
	if ((cfg_flag & (0x1 << SET_BACKLIGHT_LEVEL)) && !(cfg_flag & (0x1 << SET_ELVSS_PN))) {
		DDPINFO("%s cfg_flag = %d, level=%d\n", __func__, cfg_flag, level);
		oplus_display_panel_set_pwm_bl(crtc, cmdq_handle, level, true);
	} else {
		/* set backlight and elvss */
		bl_ext_config.cfg_flag = cfg_flag;
		bl_ext_config.backlight_level = level;
		bl_ext_config.elvss_pn = panel_ext_param;
		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL_ELVSS, &bl_ext_config);
	}
	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	CRTC_MMP_MARK(index, backlight, bl_cnt, 0);
	drm_trace_tag_mark("backlight");
	bl_cnt++;

	cb_data->crtc = crtc;
	cb_data->cmdq_handle = cmdq_handle;

	if (cmdq_pkt_flush_threaded(cmdq_handle, oplus_bl_cmdq_cb, cb_data) < 0) {
		DDPPR_ERR("failed to flush oplus_bl_cmdq_cb\n");
		ret = -EINVAL;
	}

	CRTC_MMP_EVENT_END(index, backlight, (unsigned long)crtc,
			level);

	return ret;
}

