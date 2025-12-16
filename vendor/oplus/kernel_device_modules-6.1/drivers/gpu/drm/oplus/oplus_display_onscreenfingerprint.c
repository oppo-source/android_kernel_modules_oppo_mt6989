/***************************************************************
** Copyright (C), 2022, OPLUS Mobile Comm Corp., Ltd
** File : oplus_display_onscreenfingerprint.c
** Description : oplus_display_onscreenfingerprint implement
** Version : 1.0
** Date : 2021/12/10
** Author : Display
***************************************************************/

#include "oplus_display_onscreenfingerprint.h"
#include "oplus_display_trace.h"
#include "mtk_drm_drv.h"
#include "mtk_drm_helper.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_mmp.h"
#include "mtk_disp_notify.h"
#include "mtk_log.h"
#include "mtk_dump.h"
#include "oplus_display_common.h"
#include "oplus_display_high_frequency_pwm.h"
#include "oplus_display_apollo_brightness.h"
#include "oplus_adfr.h"
#include <soc/oplus/touchpanel_event_notify.h>
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "oplus_display_temp_compensation.h"
#include <soc/oplus/system/oplus_project.h>
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */
#include "mtk_dsi.h"

/* -------------------- macro -------------------- */
/* fp type bit setting */
#define OPLUS_OFP_FP_TYPE_LCD_CAPACITIVE					(BIT(0))
#define OPLUS_OFP_FP_TYPE_OLED_CAPACITIVE					(BIT(1))
#define OPLUS_OFP_FP_TYPE_OPTICAL_OLD_SOLUTION				(BIT(2))
#define OPLUS_OFP_FP_TYPE_OPTICAL_NEW_SOLUTION				(BIT(3))
#define OPLUS_OFP_FP_TYPE_LOCAL_HBM							(BIT(4))
#define OPLUS_OFP_FP_TYPE_BRIGHTNESS_ADAPTATION				(BIT(5))
#define OPLUS_OFP_FP_TYPE_ULTRASONIC						(BIT(6))
#define OPLUS_OFP_FP_TYPE_ULTRA_LOW_POWER_AOD				(BIT(7))
#define OPLUS_OFP_FP_TYPE_VIDEO_MODE_AOD_FOD				(BIT(9))
#define OPLUS_OFP_FP_TYPE_VIDEO_MODE_30HZ_AOD                           (BIT(11))
/* get fp type config */
#define OPLUS_OFP_GET_LCD_CAPACITIVE_CONFIG(fp_type)		((fp_type) & OPLUS_OFP_FP_TYPE_LCD_CAPACITIVE)
#define OPLUS_OFP_GET_OLED_CAPACITIVE_CONFIG(fp_type)		((fp_type) & OPLUS_OFP_FP_TYPE_OLED_CAPACITIVE)
#define OPLUS_OFP_GET_OPTICAL_OLD_SOLUTION_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_OPTICAL_OLD_SOLUTION)
#define OPLUS_OFP_GET_OPTICAL_NEW_SOLUTION_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_OPTICAL_NEW_SOLUTION)
#define OPLUS_OFP_GET_LOCAL_HBM_CONFIG(fp_type)				((fp_type) & OPLUS_OFP_FP_TYPE_LOCAL_HBM)
#define OPLUS_OFP_GET_BRIGHTNESS_ADAPTATION_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_BRIGHTNESS_ADAPTATION)
#define OPLUS_OFP_GET_ULTRASONIC_CONFIG(fp_type)			((fp_type) & OPLUS_OFP_FP_TYPE_ULTRASONIC)
#define OPLUS_OFP_GET_ULTRA_LOW_POWER_AOD_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_ULTRA_LOW_POWER_AOD)
#define OPLUS_OFP_GET_VIDEO_MODE_AOD_FOD_CONFIG(fp_type)	((fp_type) & OPLUS_OFP_FP_TYPE_VIDEO_MODE_AOD_FOD)
#define OPLUS_OFP_GET_VIDEO_MODE_30HZ_AOD_CONFIG(fp_type)       ((fp_type) & OPLUS_OFP_FP_TYPE_VIDEO_MODE_30HZ_AOD)
/* dbv level */
#define OPLUS_OFP_900NIT_DBV_LEVEL 							(0x0DBB)

/* notifier event */
#define DRM_PANEL_EVENT_HBM_STATE							1
#define MTK_ONSCREENFINGERPRINT_EVENT						20
#define LCM_BRIGHTNESS_TYPE 2
/* -------------------- parameters -------------------- */
/* log level config */
int oplus_ofp_log_level = OPLUS_OFP_LOG_LEVEL_INFO;
EXPORT_SYMBOL(oplus_ofp_log_level);
/* ofp global structure */
static struct oplus_ofp_params g_oplus_ofp_params = {0};

/* ofp and aod off need swtich pwm pulse */
bool ofp_aod_off_swtich_pulse = false;
EXPORT_SYMBOL(ofp_aod_off_swtich_pulse);

/* -------------------- extern -------------------- */
/* extern params */
struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};
extern unsigned int m_db;
extern unsigned int last_backlight;
extern unsigned int oplus_display_brightness;
extern struct oplus_demura_setting_table demura_setting;

/* extern functions */
extern void lcdinfo_notify(unsigned long val, void *v);
extern int mtk_ddic_vdo_aod_ctrl(unsigned char aod_en, bool need_lock);
extern bool oplus_apollo_unsupported(void);

/* -------------------- oplus_ofp_params -------------------- */
static struct oplus_ofp_params *oplus_ofp_get_params(void)
{
	return &g_oplus_ofp_params;
}

int oplus_ofp_fp_type_compatible_mode_config(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (p_oplus_ofp_params->fp_type_compatible_mode) {
		if (m_db <= 6) {
			/* these panels do not support lhbm */
			p_oplus_ofp_params->fp_type = 0x08;
			OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);
		}
	}

	OFP_DEBUG("end\n");

	return 0;
}

bool oplus_ofp_video_mode_aod_fod_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_VIDEO_MODE_AOD_FOD_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_video_mode_30hz_aod_is_enabled(void)
{
        struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

        if (!p_oplus_ofp_params) {
                OFP_ERR("Invalid params\n");
                return false;
        }

        return (bool)(OPLUS_OFP_GET_VIDEO_MODE_30HZ_AOD_CONFIG(p_oplus_ofp_params->fp_type));
}
/* get fp_type value from panel dtsi */
int oplus_ofp_init(void *device)
{
	int rc = 0;
	int length = 0;
	unsigned int i = 0;
	unsigned int value = 0;
	struct device *dev = device;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!dev || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(dev->of_node, "oplus,ofp-fp-type", &value);
	if (rc) {
		OFP_INFO("failed to read oplus,ofp-fp-type, rc=%d\n", rc);
		/* set default value to BIT(0) */
		p_oplus_ofp_params->fp_type = BIT(0);
	} else {
		p_oplus_ofp_params->fp_type = value;
	}

	OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);

	p_oplus_ofp_params->fp_local_hbm_calibration = of_property_read_bool(dev->of_node, "oplus,ofp-fp-local-hbm-calibration");
	OFP_INFO("fp_local_hbm_calibration:%d\n", p_oplus_ofp_params->fp_local_hbm_calibration);

	/* indicates whether fp type compatible mode is set or not */
	p_oplus_ofp_params->fp_type_compatible_mode = of_property_read_bool(dev->of_node, "oplus,ofp-fp-type-compatible-mode");
	OFP_INFO("fp_type_compatible_mode:%d\n", p_oplus_ofp_params->fp_type_compatible_mode);

	/* indicates whether lhbm on brightness needs to be read and updated or not */
	p_oplus_ofp_params->need_to_update_lhbm_brightness = of_property_read_bool(dev->of_node, "oplus,ofp-need-to-update-lhbm-brightness");
	OFP_INFO("need_to_update_lhbm_brightness:%d\n", p_oplus_ofp_params->need_to_update_lhbm_brightness);

	if (oplus_ofp_is_supported()) {
		/* read by the framework for compatibility with different aod modes */
		rc = of_property_read_u32(dev->of_node, "oplus,ofp-longrui-aod-config", &value);
		if (rc) {
			OFP_INFO("failed to read oplus,ofp-longrui-aod-config, rc=%d\n", rc);
			/* set default value to 0 */
			p_oplus_ofp_params->longrui_aod_config = 0;
		} else {
			p_oplus_ofp_params->longrui_aod_config = value;
		}
		OFP_INFO("longrui_aod_config:0x%x\n", p_oplus_ofp_params->longrui_aod_config);

		/* add for uiready notifier call chain */
		hrtimer_init(&p_oplus_ofp_params->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		p_oplus_ofp_params->timer.function = oplus_ofp_notify_uiready_timer_handler;

		/* add for touchpanel event notifier */
		p_oplus_ofp_params->touchpanel_event_notifier.notifier_call = oplus_ofp_touchpanel_event_notifier_call;
		rc = touchpanel_event_register_notifier(&p_oplus_ofp_params->touchpanel_event_notifier);
		if (rc < 0) {
			OFP_ERR("failed to register touchpanel event notifier: %d\n", rc);
		}

		/* add workqueue to send aod off cmd */
		p_oplus_ofp_params->aod_off_set_wq = create_singlethread_workqueue("aod_off_set");
		p_oplus_ofp_params->uiready_event_wq = create_singlethread_workqueue("uiready_event");
		INIT_WORK(&p_oplus_ofp_params->aod_off_set_work, oplus_ofp_aod_off_set_work_handler);
		INIT_WORK(&p_oplus_ofp_params->uiready_event_work, oplus_ofp_uiready_event_work_handler);

		if (oplus_ofp_local_hbm_is_enabled()) {
			/* indicates whether lhbm pressed icon gamma needs to be read and updated or not */
			p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma = of_property_read_bool(dev->of_node, "oplus,ofp-need-to-update-lhbm-pressed-icon-gamma");
			OFP_INFO("need_to_update_lhbm_pressed_icon_gamma:%d\n", p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma);

			/* indicates which line the dbv alpha register is in the cmd set */
			rc = of_property_read_u32(dev->of_node, "oplus,ofp-lhbm-dbv-alpha-cmd-index", &value);
			if (rc) {
				OFP_ERR("failed to read oplus,ofp-lhbm-dbv-alpha-cmd-index, rc=%d\n", rc);
			} else {
				p_oplus_ofp_params->lhbm_dbv_alpha_cmd_index = value;
			}
			OFP_INFO("oplus_ofp_lhbm_dbv_alpha_cmd_index:%u\n", p_oplus_ofp_params->lhbm_dbv_alpha_cmd_index);

			/* indicates where the dbv alpha register is in the line instruction */
			rc = of_property_read_u32(dev->of_node, "oplus,ofp-lhbm-dbv-alpha-register-offset", &value);
			if (rc) {
				OFP_ERR("failed to read oplus,ofp-lhbm-dbv-alpha-register-offset, rc=%d\n", rc);
			} else {
				p_oplus_ofp_params->lhbm_dbv_alpha_register_offset = value;
			}
			OFP_INFO("oplus_ofp_lhbm_dbv_alpha_register_offset:%u\n", p_oplus_ofp_params->lhbm_dbv_alpha_register_offset);

			/* the lhbm dbv alpha value which represents the alpha register setting corresponding to the backlight level */
			length = of_property_count_elems_of_size(dev->of_node, "oplus,ofp-lhbm-dbv-alpha-value", sizeof(unsigned int));
			if (length < 0) {
				OFP_ERR("failed to get the count of oplus,ofp-lhbm-dbv-alpha-value\n");
				rc = -EINVAL;
			} else if (!p_oplus_ofp_params->lhbm_dbv_alpha_value) {
				p_oplus_ofp_params->lhbm_dbv_alpha_value = kzalloc(length * sizeof(unsigned int), GFP_KERNEL);
				if (!p_oplus_ofp_params->lhbm_dbv_alpha_value) {
					OFP_ERR("failed to kzalloc lhbm_dbv_alpha_value\n");
					rc = -ENOMEM;
				} else {
					rc = of_property_read_u32_array(dev->of_node, "oplus,ofp-lhbm-dbv-alpha-value", p_oplus_ofp_params->lhbm_dbv_alpha_value, length);
					if (rc) {
						OFP_ERR("failed to read oplus,ofp-lhbm-dbv-alpha-value\n");
						rc = -EINVAL;
						kfree(p_oplus_ofp_params->lhbm_dbv_alpha_value);
						p_oplus_ofp_params->lhbm_dbv_alpha_value = NULL;
					} else {
						OFP_INFO("property:oplus,ofp-lhbm-dbv-alpha-value,length:%u\n", length);
						for (i = 0; i < length; i++) {
							OFP_DEBUG("lhbm_dbv_alpha_value[%u]=%u\n", i, p_oplus_ofp_params->lhbm_dbv_alpha_value[i]);
						}
						p_oplus_ofp_params->lhbm_dbv_alpha_value_count = length;
						OFP_INFO("oplus_ofp_lhbm_dbv_alpha_value_count:%u\n", p_oplus_ofp_params->lhbm_dbv_alpha_value_count);
					}
				}
			}
		}
	}

	OFP_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_ofp_init);

bool oplus_ofp_is_supported(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (oplus_ofp_video_mode_aod_fod_is_enabled()) {
		return true;
	}
	/* global config, support oled panel */
	return (bool)(!OPLUS_OFP_GET_LCD_CAPACITIVE_CONFIG(p_oplus_ofp_params->fp_type));
}
EXPORT_SYMBOL(oplus_ofp_is_supported);

bool oplus_ofp_oled_capacitive_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, oled capacitive is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_OLED_CAPACITIVE_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_optical_new_solution_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, optical new solution is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_OPTICAL_NEW_SOLUTION_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_local_hbm_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, local hbm is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_LOCAL_HBM_CONFIG(p_oplus_ofp_params->fp_type));
}
EXPORT_SYMBOL(oplus_ofp_local_hbm_is_enabled);

/*
static bool oplus_ofp_brightness_adaptation_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, brightness adaptation is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_BRIGHTNESS_ADAPTATION_CONFIG(p_oplus_ofp_params->fp_type));
}
*/
bool oplus_ofp_ultrasonic_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, ultrasonic is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_ULTRASONIC_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_ultra_low_power_aod_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, ultra low power aod is also not supported\n");
		return false;
	}

	return (bool)(OPLUS_OFP_GET_ULTRA_LOW_POWER_AOD_CONFIG(p_oplus_ofp_params->fp_type));
}

bool oplus_ofp_a_mirror_to_the_end_aod_mode_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, a mirror to the end aod mode is also not supported\n");
		return false;
	}

	return (bool)((p_oplus_ofp_params->longrui_aod_config & OPLUS_OFP_A_MIRROR_TO_THE_END_AOD_CONFIG)
					&& (p_oplus_ofp_params->longrui_aod_mode & OPLUS_OFP_A_MIRROR_TO_THE_END_AOD_MODE)
						&& (p_oplus_ofp_params->longrui_aod_mode & OPLUS_OFP_AOD_ON)
							&& !((oplus_ofp_optical_new_solution_is_enabled() || oplus_ofp_video_mode_aod_fod_is_enabled())
								&& p_oplus_ofp_params->dimlayer_hbm) && !oplus_ofp_local_hbm_is_enabled());
}

bool oplus_ofp_full_screen_aod_mode_is_enabled(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not support, full screen aod mode is also not supported\n");
		return false;
	}

	return (bool)((p_oplus_ofp_params->longrui_aod_config & OPLUS_OFP_FULL_SCREEN_AOD_CONFIG)
					&& (p_oplus_ofp_params->longrui_aod_mode & OPLUS_OFP_FULL_SCREEN_AOD_MODE));
}

/* aod unlocking value update */
int oplus_ofp_aod_unlocking_update(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->fp_press || p_oplus_ofp_params->doze_active != 0) {
		/* press icon layer is ready in aod mode */
		p_oplus_ofp_params->aod_unlocking = true;
		OFP_INFO("oplus_ofp_aod_unlocking: %d\n", p_oplus_ofp_params->aod_unlocking);
		oplus_disp_trace_c("%d|oplus_ofp_aod_unlocking|%d", g_commit_pid, p_oplus_ofp_params->aod_unlocking);
	}

	return 0;
}

bool oplus_ofp_get_aod_unlocking(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->aod_unlocking;
}
EXPORT_SYMBOL(oplus_ofp_get_aod_unlocking);

int oplus_ofp_get_hbm_enable(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->hbm_enable;
}
EXPORT_SYMBOL(oplus_ofp_get_hbm_enable);

bool oplus_ofp_get_aod_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->aod_state;
}
EXPORT_SYMBOL(oplus_ofp_get_aod_state);

int oplus_ofp_set_aod_state(bool aod_state)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	p_oplus_ofp_params->aod_state = aod_state;
	OFP_INFO("oplus_ofp_aod_state: %d\n", p_oplus_ofp_params->aod_state);
	oplus_disp_trace_c("%d|oplus_ofp_aod_state|%d", g_commit_pid, p_oplus_ofp_params->aod_state);

	return 0;
}

bool oplus_ofp_get_fake_aod_mode(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	oplus_disp_trace_begin("oplus_ofp_get_fake_aod_mode");
	OFP_DEBUG("oplus_ofp_fake_aod_mode:%u\n", p_oplus_ofp_params->fake_aod_mode);
	oplus_disp_trace_end("oplus_ofp_get_fake_aod_mode");

	OFP_DEBUG("end\n");

	return p_oplus_ofp_params->fake_aod_mode;
}
EXPORT_SYMBOL(oplus_ofp_get_fake_aod_mode);

int oplus_ofp_get_hbm_state(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return 0;
	}

	return p_oplus_ofp_params->hbm_state;
}
EXPORT_SYMBOL(oplus_ofp_get_hbm_state);

int oplus_ofp_set_hbm_state(bool hbm_state)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	p_oplus_ofp_params->hbm_state = hbm_state;
	OFP_INFO("oplus_ofp_hbm_state: %d\n", hbm_state);
	oplus_disp_trace_c("%d|oplus_ofp_hbm_state|%d", g_commit_pid, hbm_state);

	oplus_ofp_send_hbm_state_event(hbm_state);

	return 0;
}

/* update doze_active and hbm_enable property value */
int oplus_ofp_property_update(int prop_id, unsigned int prop_val)
{
	static unsigned int last_disp_mode_idx = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	switch (prop_id) {
	case CRTC_PROP_DOZE_ACTIVE:
		if (prop_val != p_oplus_ofp_params->doze_active) {
			OFP_INFO("DOZE_ACTIVE:%d\n", prop_val);
		}
		p_oplus_ofp_params->doze_active = prop_val;
		oplus_disp_trace_c("%d|oplus_ofp_doze_active|%d", g_commit_pid, p_oplus_ofp_params->doze_active);
		break;

	case CRTC_PROP_HBM_ENABLE:
		if (prop_val != p_oplus_ofp_params->hbm_enable) {
			OFP_INFO("HBM_ENABLE:%u,dim:%u,fingerpress:%u,icon:%u,aod:%u\n", prop_val, (prop_val & OPLUS_OFP_PROPERTY_DIM_LAYER),
				(prop_val & OPLUS_OFP_PROPERTY_FINGERPRESS_LAYER), (prop_val & OPLUS_OFP_PROPERTY_ICON_LAYER),
					(prop_val & OPLUS_OFP_PROPERTY_AOD_LAYER));
		}
		p_oplus_ofp_params->hbm_enable = prop_val;
		oplus_disp_trace_c("%d|oplus_ofp_hbm_enable|%d", g_commit_pid, p_oplus_ofp_params->hbm_enable);
		break;
	case CRTC_PROP_DISP_MODE_IDX:
		if (prop_val != last_disp_mode_idx) {
			p_oplus_ofp_params->timing_switching = true;
			OFP_DEBUG("DISP_MODE_IDX:%u\n", prop_val);
		} else {
			p_oplus_ofp_params->timing_switching = false;
		}
		OFP_DEBUG("p_oplus_ofp_params->timing_switching:%d\n", p_oplus_ofp_params->timing_switching);
		oplus_disp_trace_c("%d|oplus_ofp_timing_switching|%d", g_commit_pid, p_oplus_ofp_params->timing_switching);
		last_disp_mode_idx = prop_val;
		break;

	default:
		break;
	}

	return 0;
}


/* -------------------- fod -------------------- */
/*
 due to the fact that the brightness of lhbm pressed icon may change with the backlight,
 it is necessary to readjust the lhbm pressed icon gamma to meet the requirements of fingerprint unlocking
*/
int oplus_ofp_lhbm_pressed_icon_gamma_update(void *drm_crtc, void *LCM_setting_table)
{
	static bool calibrated = false;
	unsigned char registerswitchpage[4] = {0xFF, 0x08, 0x38, 0x2F};
	unsigned char register99[2] = {0x99, 0xB2};
	unsigned char rx_buf_0[5] = {0};
	unsigned char rx_buf_1[5] = {0};
	static unsigned char extrapolated_value[5][6] = {0};
	int rc = 0;
	int rgb_data_0[3] = {0};
	int rgb_data_1[3] = {0};
	int rgb_data_2[3] = {0};
	unsigned int i = 0;
	unsigned int j = 0;
	static unsigned int failure_count = 0;
	struct drm_crtc *crtc = drm_crtc;
	struct LCM_setting_table *table = LCM_setting_table;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, no need to update lhbm pressed icon gamma\n");
		return 0;
	}

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid p_oplus_ofp_params param\n");
		return -EINVAL;
	}

	if (!p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma) {
		OFP_DEBUG("need_to_update_lhbm_pressed_icon_gamma is not config, no need to update lhbm pressed icon gamma\n");
		return 0;
	}

	if (p_oplus_ofp_params->failed_to_read_lhbm_pressed_icon_gamma) {
		OFP_DEBUG("lhbm pressed icon gamma reading is failed, no need to update lhbm pressed icon gamma\n");
		return 0;
	}

	if (!calibrated && (failure_count < 100) && !LCM_setting_table) {
		if (!crtc) {
			OFP_ERR("Invalid crtc param\n");
			rc = -EINVAL;
			goto error;
		}

		mtk_crtc = to_mtk_crtc(crtc);
		if (!mtk_crtc) {
			OFP_ERR("Invalid mtk_crtc param\n");
			rc = -EINVAL;
			goto error;
		}

		if (!mtk_crtc->enabled) {
			OFP_ERR("mtk crtc is not enabled\n");
			rc = -EFAULT;
			goto error;
		}

		if (!mtk_drm_lcm_is_connect(mtk_crtc)) {
			OFP_ERR("lcm is not connect\n");
			rc = -EFAULT;
			goto error;
		}

		/* read lhbm pressed icon gamma */
		oplus_ddic_dsi_send_cmd(4, registerswitchpage);
		oplus_ddic_dsi_send_cmd(2, register99);
		oplus_mtk_read_ddic_v2(0x96, 5, rx_buf_0);
		oplus_mtk_read_ddic_v2(0x97, 5, rx_buf_1);
		register99[1] = 0x00;
		oplus_ddic_dsi_send_cmd(2, register99);
		for (i = 0; i < 5; i++) {
			OFP_INFO("rx_buf_0[%u] = 0x%02X, rx_buf_1[%u] = 0x%02X\n", i, rx_buf_0[i], i, rx_buf_1[i]);
		}
		if (!rx_buf_0[0] && !rx_buf_0[1] && !rx_buf_0[2] && !rx_buf_0[3] && !rx_buf_0[4]
				&& !rx_buf_1[0] && !rx_buf_1[1] && !rx_buf_1[2] && !rx_buf_1[3] && !rx_buf_1[4]) {
			OFP_ERR("the rx_buf_0 and rx_buf_1 are 0\n");
			p_oplus_ofp_params->failed_to_read_lhbm_pressed_icon_gamma = true;
			OFP_INFO("failed_to_read_lhbm_pressed_icon_gamma:%d\n", p_oplus_ofp_params->failed_to_read_lhbm_pressed_icon_gamma);
			rc = -EINVAL;
			goto error;
		}
		rgb_data_0[0] = ((rx_buf_0[0] & 0x0F) << 8) | rx_buf_0[2];
		rgb_data_0[1] = ((rx_buf_0[1] & 0xF0) << 4) | rx_buf_0[3];
		rgb_data_0[2] = ((rx_buf_0[1] & 0x0F) << 8) | rx_buf_0[4];
		rgb_data_1[0] = ((rx_buf_1[0] & 0x0F) << 8) | rx_buf_1[2];
		rgb_data_1[1] = ((rx_buf_1[1] & 0xF0) << 4) | rx_buf_1[3];
		rgb_data_1[2] = ((rx_buf_1[1] & 0x0F) << 8) | rx_buf_1[4];
		if (((rgb_data_0[0] < 0x881) || (rgb_data_0[0] > 0xCD7)) || ((rgb_data_0[1] < 0x7DF) || (rgb_data_0[1] > 0xBCF))
				|| ((rgb_data_0[2] < 0x9C0) || (rgb_data_0[2] > 0xE4F)) || ((rgb_data_1[0] < 0x881) || (rgb_data_1[0] > 0xCD7))
				|| ((rgb_data_1[1] < 0x7DF) || (rgb_data_1[1] > 0xBCF)) || ((rgb_data_1[2] < 0x9C0) || (rgb_data_1[2] > 0xE4F))) {
			OFP_ERR("the rx_buf_0 or rx_buf_1 are incorrect\n");
			p_oplus_ofp_params->failed_to_read_lhbm_pressed_icon_gamma = true;
			OFP_INFO("failed_to_read_lhbm_pressed_icon_gamma:%d\n", p_oplus_ofp_params->failed_to_read_lhbm_pressed_icon_gamma);
			rc = -EINVAL;
			goto error;
		}

		/* readjust the corresponding gamma value through extrapolation */
		rgb_data_2[0] = (173 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (173 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (173 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[0][0] = 0x93;
		extrapolated_value[0][1] = rgb_data_2[0] >> 8;
		extrapolated_value[0][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[0][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[0][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[0][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[0][1] && !extrapolated_value[0][2] && !extrapolated_value[0][3]
				&& !extrapolated_value[0][4] && !extrapolated_value[0][5]) {
			OFP_ERR("the extrapolated_value[0] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		rgb_data_2[0] = (205 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (205 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (205 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[1][0] = 0x94;
		extrapolated_value[1][1] = rgb_data_2[0] >> 8;
		extrapolated_value[1][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[1][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[1][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[1][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[1][1] && !extrapolated_value[1][2] && !extrapolated_value[1][3]
				&& !extrapolated_value[1][4] && !extrapolated_value[1][5]) {
			OFP_ERR("the extrapolated_value[1] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		rgb_data_2[0] = (226 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (226 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (226 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[2][0] = 0x95;
		extrapolated_value[2][1] = rgb_data_2[0] >> 8;
		extrapolated_value[2][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[2][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[2][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[2][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[2][1] && !extrapolated_value[2][2] && !extrapolated_value[2][3]
				&& !extrapolated_value[2][4] && !extrapolated_value[2][5]) {
			OFP_ERR("the extrapolated_value[2] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		rgb_data_2[0] = (247 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (247 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (247 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[3][0] = 0x96;
		extrapolated_value[3][1] = rgb_data_2[0] >> 8;
		extrapolated_value[3][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[3][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[3][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[3][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[3][1] && !extrapolated_value[3][2] && !extrapolated_value[3][3]
				&& !extrapolated_value[3][4] && !extrapolated_value[3][5]) {
			OFP_ERR("the extrapolated_value[3] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		rgb_data_2[0] = (255 * 110 - 24700) * (rgb_data_1[0] - rgb_data_0[0]) / 800 + rgb_data_0[0];
		rgb_data_2[1] = (255 * 110 - 24700) * (rgb_data_1[1] - rgb_data_0[1]) / 800 + rgb_data_0[1];
		rgb_data_2[2] = (255 * 110 - 24700) * (rgb_data_1[2] - rgb_data_0[2]) / 800 + rgb_data_0[2];
		extrapolated_value[4][0] = 0x97;
		extrapolated_value[4][1] = rgb_data_2[0] >> 8;
		extrapolated_value[4][2] = ((rgb_data_2[1] & 0xF00) >> 4) | ((rgb_data_2[2] & 0xF00) >> 8);
		extrapolated_value[4][3] = rgb_data_2[0] & 0xFF;
		extrapolated_value[4][4] = rgb_data_2[1] & 0xFF;
		extrapolated_value[4][5] = rgb_data_2[2] & 0xFF;
		if (!extrapolated_value[4][1] && !extrapolated_value[4][2] && !extrapolated_value[4][3]
				&& !extrapolated_value[4][4] && !extrapolated_value[4][5]) {
			OFP_ERR("the extrapolated_value[4] are incorrect\n");
			rc = -EINVAL;
			goto error;
		}

		for (i = 0; i < 5; i++) {
			for (j = 0; j < 6; j++) {
				OFP_INFO("extrapolated_value[%u][%u] = 0x%02X\n", i, j, extrapolated_value[i][j]);
			}
		}

		calibrated = true;
		OFP_INFO("update lhbm pressed icon gamma successfully\n");
	}

	if (calibrated && !drm_crtc) {
		if (!table) {
			OFP_ERR("Invalid table param\n");
			rc = -EINVAL;
		} else {
			/* update pressed icon gamma value */
			for (i = 0; i < 5; i++) {
				memcpy(table[2+i].para_list, extrapolated_value[i], 6);
				OFP_INFO("lhbm_pressed_icon_gamma_cmd[%u]:0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n", 2+i, table[2+i].para_list[0], table[2+i].para_list[1],
							table[2+i].para_list[2], table[2+i].para_list[3], table[2+i].para_list[4], table[2+i].para_list[5]);
			}
			OFP_INFO("update lhbm pressed icon gamma cmds value\n");
			rc = 1;
		}
	}

error:
	/* if gamma read fails more than 100 times, no further operation will be performed */
	if (!calibrated && (failure_count < 100)) {
		failure_count++;
		OFP_ERR("failure_count:%u\n", failure_count);
	}

	OFP_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_ofp_lhbm_pressed_icon_gamma_update);

/*
 when the backlight is in the outdoor hbm state, the brightness of the lhbm pressed icon is too high, which causes the unlocking failure.
 therefore, the backlight brightness must be limited to the normal state in the fingerprint unlocking scenario
*/
int oplus_ofp_lhbm_backlight_update(void *drm_crtc)
{
	int rc = 0;
	static int current_backlight = 0;
	static unsigned int last_dimlayer_hbm = 0;
	struct drm_crtc *crtc = drm_crtc;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, no need to update backlight after dimlayer_hbm on/off\n");
		return 0;
	}

	if (!crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	if(!p_oplus_ofp_params->need_to_update_lhbm_brightness) {
		if (!p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma) {
			OFP_DEBUG("need_to_update_lhbm_pressed_icon_gamma is not set, no need to update backlight after dimlayer_hbm on/off\n");
			return 0;
		}
	}

	if (drm_crtc_index(crtc)) {
		OFP_DEBUG("not in dsi mode, should not update backlight after dimlayer_hbm on/off\n");
		return 0;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc) {
		OFP_ERR("Invalid mtk_crtc param\n");
		return -EINVAL;
	}

	if (!mtk_crtc->oplus_apollo_br) {
		OFP_ERR("Invalid oplus_apollo_br param\n");
		return -EINVAL;
	}

	if (mtk_crtc->oplus_apollo_br->oplus_backlight_updated) {
		current_backlight = mtk_crtc->oplus_apollo_br->oplus_pending_backlight;
		OFP_DEBUG("current_backlight:%d\n", current_backlight);
	}

	if (!last_dimlayer_hbm && p_oplus_ofp_params->dimlayer_hbm) {
		if ((mtk_crtc->oplus_apollo_br->oplus_pending_backlight > OPLUS_OFP_900NIT_DBV_LEVEL)
			&& !p_oplus_ofp_params->doze_active) {
			OFP_INFO("set backlight level to OPLUS_OFP_900NIT_DBV_LEVEL after dimlayer_hbm on\n");
			mtk_crtc->oplus_apollo_br->oplus_backlight_updated = true;
			mtk_crtc->oplus_apollo_br->oplus_pending_backlight = OPLUS_OFP_900NIT_DBV_LEVEL;
		}
	} else if (last_dimlayer_hbm && !p_oplus_ofp_params->dimlayer_hbm) {
		if ((current_backlight > OPLUS_OFP_900NIT_DBV_LEVEL
			&& !oplus_ofp_get_aod_state())) {
			OFP_INFO("recovery backlight level to %u after dimlayer_hbm off\n", current_backlight);
			mtk_crtc->oplus_apollo_br->oplus_backlight_updated = true;
			mtk_crtc->oplus_apollo_br->oplus_pending_backlight = current_backlight;
		}
	} else if (p_oplus_ofp_params->dimlayer_hbm && (mtk_crtc->oplus_apollo_br->oplus_pending_backlight > OPLUS_OFP_900NIT_DBV_LEVEL)) {
		mtk_crtc->oplus_apollo_br->oplus_pending_backlight = OPLUS_OFP_900NIT_DBV_LEVEL;
		OFP_INFO("dimlayer_hbm is on and backlight lvl is greater than OPLUS_OFP_900NIT_DBV_LEVEL, set backlight to OPLUS_OFP_900NIT_DBV_LEVEL\n");
	}

	last_dimlayer_hbm = p_oplus_ofp_params->dimlayer_hbm;

	OFP_DEBUG("end\n");

	return rc;
}

/*
 in the manual lhbm dbv alpha solution, adjust the corresponding lhbm dbv alpha registers when backlight changing 
 after entering the local hbm mode
*/
int oplus_ofp_lhbm_dbv_alpha_update(void *LCM_setting_table, unsigned int lcm_cmd_count, bool forced_updating, unsigned int bl_level)
{
	unsigned char *tx_buf = NULL;
	int rc = 0;
	struct LCM_setting_table *table = LCM_setting_table;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, no need to update lhbm dbv alpha\n");
		return 0;
	}

	if (!table ||!lcm_cmd_count || !p_oplus_ofp_params) {
		OFP_ERR("Invalid table params\n");
		return -EINVAL;
	}

	if (!p_oplus_ofp_params->lhbm_dbv_alpha_value) {
		OFP_DEBUG("lhbm_dbv_alpha_value is not config, no need to update lhbm dbv alpha\n");
		return 0;
	}

	if (!oplus_ofp_get_hbm_state() && !forced_updating) {
		OFP_DEBUG("should not update lhbm dbv alpha if hbm state is false\n");
		return 0;
	}

	if (lcm_cmd_count < (p_oplus_ofp_params->lhbm_dbv_alpha_cmd_index + 1)) {
		OFP_ERR("wrong lcm cmd count\n");
		rc = -EINVAL;
		goto end;
	} else if (table[p_oplus_ofp_params->lhbm_dbv_alpha_cmd_index].count < (p_oplus_ofp_params->lhbm_dbv_alpha_register_offset + 2)) {
		OFP_ERR("wrong tx_len\n");
		rc = -EINVAL;
		goto end;
	}

	tx_buf = (unsigned char *)table[p_oplus_ofp_params->lhbm_dbv_alpha_cmd_index].para_list;
	if (!tx_buf) {
		OFP_ERR("Invalid tx_buf param\n");
		rc = -EINVAL;
		goto end;
	}

	if (bl_level >= p_oplus_ofp_params->lhbm_dbv_alpha_value_count) {
		tx_buf[p_oplus_ofp_params->lhbm_dbv_alpha_register_offset] =
			p_oplus_ofp_params->lhbm_dbv_alpha_value[p_oplus_ofp_params->lhbm_dbv_alpha_value_count - 1] >> 8;
		tx_buf[p_oplus_ofp_params->lhbm_dbv_alpha_register_offset + 1] =
			p_oplus_ofp_params->lhbm_dbv_alpha_value[p_oplus_ofp_params->lhbm_dbv_alpha_value_count - 1] & 0x0FF;
	} else {
		tx_buf[p_oplus_ofp_params->lhbm_dbv_alpha_register_offset] = p_oplus_ofp_params->lhbm_dbv_alpha_value[bl_level] >> 8;
		tx_buf[p_oplus_ofp_params->lhbm_dbv_alpha_register_offset + 1] = p_oplus_ofp_params->lhbm_dbv_alpha_value[bl_level] & 0x0FF;
	}

	OFP_INFO("bl_level:%u, lhbm dbv alpha:0x%02X 0x%02X\n", bl_level, tx_buf[p_oplus_ofp_params->lhbm_dbv_alpha_register_offset],
																tx_buf[p_oplus_ofp_params->lhbm_dbv_alpha_register_offset + 1]);
	rc = 1;

end:
	OFP_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_ofp_lhbm_dbv_alpha_update);

int oplus_ofp_send_hbm_state_event(unsigned int hbm_state)
{
	OFP_DEBUG("start\n");

	if (oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is enabled, no need to send hbm state event\n");
		return 0;
	}

	lcdinfo_notify(DRM_PANEL_EVENT_HBM_STATE, &hbm_state);
	OFP_INFO("DRM_PANEL_EVENT_HBM_STATE:%u\n", hbm_state);

	OFP_DEBUG("end\n");

	return 0;
}

static int oplus_ofp_set_panel_hbm_witch_demura(struct mtk_drm_crtc *mtk_crtc, struct cmdq_pkt *cmdq_handle, bool hbm_en)
{
	unsigned int level = 0;
	int bl_demura_mode = OPLUS_DEMURA_DBV_MODE_MAX;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (!(comp && comp->funcs && comp->funcs->io_cmd)) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}
	if (cmdq_handle == NULL) {
		cmdq_handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	}
	oplus_disp_trace_begin("DSI_SET_DEMURA_BL");

	if (hbm_en)
		level = 0xF01;
	else
		level = oplus_display_brightness;

	if ((level > 1) && (level < demura_setting.demura_switch_dvb1)) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE0;
	} else if ((level >= demura_setting.demura_switch_dvb1) && (level < demura_setting.demura_switch_dvb2)) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE1;
	} else if ((level >= demura_setting.demura_switch_dvb2) && (level < demura_setting.demura_switch_dvb3)) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE2;
	} else if ((level >= demura_setting.demura_switch_dvb3) && (level < demura_setting.demura_switch_dvb4)) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE3;
	} else if (level >= demura_setting.demura_switch_dvb4) {
		bl_demura_mode = OPLUS_DEMURA_DBV_MODE4;
	}

	comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_DEMURA_BL, &bl_demura_mode);
	OFP_DEBUG("bl_demura_mode =%d\n", bl_demura_mode);
	oplus_disp_trace_end("DSI_SET_DEMURA_BL");

	return 0;
}
/* wait te and delay while using cmdq */
static int oplus_ofp_cmdq_pkt_wait(struct mtk_drm_crtc *mtk_crtc, struct cmdq_pkt *cmdq_handle, int te_count, int delay_us, bool hbm_en, bool before_hbm)
{
	int wait_te_count = te_count;
	struct mtk_ddp_comp *comp = NULL;
	int prj_id = 0;

	if ((wait_te_count <= 0) && (delay_us <= 0)) {
		return 0;
	}

	if (!mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!comp || !comp->funcs || !comp->funcs->io_cmd) {
		OFP_ERR("Invalid comp params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_cmdq_pkt_wait");

	if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base)) {
		if (cmdq_handle == NULL) {
			cmdq_handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
		}

		OFP_INFO("wait %d te and delay %dus\n", wait_te_count, delay_us);

		if (wait_te_count > 0) {
			if (mtk_crtc_is_event_loop_active(mtk_crtc)) {
				cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_TE]);
			} else {
				cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			}

			while (wait_te_count > 0) {
				OFP_DEBUG("start to wait EVENT_TE, remain %d te count\n", wait_te_count);
				if (mtk_crtc_is_event_loop_active(mtk_crtc)) {
					cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_TE]);
				} else {
					cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
				}
				OFP_DEBUG("complete the EVENT_TE waiting\n");
				prj_id = get_project();
				if (prj_id == 24021 || prj_id == 24261) {
					OFP_INFO("don't need demura compensation\n");
				} else if (demura_setting.oplus_bl_demura_dbv_support && before_hbm && wait_te_count == 1) {
					oplus_ofp_set_panel_hbm_witch_demura(mtk_crtc, cmdq_handle, hbm_en);
				}
				wait_te_count--;
			}
		}

		if (delay_us > 0) {
			OFP_DEBUG("start to sleep %d us", delay_us);
			cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), CMDQ_GPR_R14);
		}
	} else {
		while (wait_te_count) {
			mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);
			wait_te_count--;
			comp->funcs->io_cmd(comp, NULL, DSI_HBM_WAIT, NULL);
		}

		if (delay_us > 0) {
			OFP_DEBUG("start to sleep %d us", delay_us);
			cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(delay_us), mtk_get_gpr(comp, cmdq_handle));
		}
	}

	oplus_disp_trace_end("oplus_ofp_cmdq_pkt_wait");

	return 0;
}

static int oplus_ofp_hbm_wait_handle(struct drm_crtc *crtc, struct cmdq_pkt *cmdq_handle, bool before_hbm, bool hbm_en)
{
	unsigned int refresh_rate = 0;
	unsigned int us_per_frame = 0;
	unsigned int te_count = 0;
	unsigned int delay_us = 0;
	int ret = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();
	struct mtk_panel_params *panel_ext = mtk_drm_get_lcm_ext_params(crtc);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle2;

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!panel_ext | !mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if(oplus_ofp_video_mode_aod_fod_is_enabled()) {
		OFP_DEBUG("video mode No need to wait\n");
		return 0;
	}

	if (p_oplus_ofp_params->aod_unlocking && hbm_en && !p_oplus_ofp_params->aod_off_hbm_on_delay
			&& !panel_ext->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
		OFP_DEBUG("no need to delay in aod unlocking\n");
		return 0;
	}

	oplus_disp_trace_begin("oplus_ofp_hbm_wait_handle");

	refresh_rate = panel_ext->dyn_fps.vact_timing_fps;
	us_per_frame = 1000000/refresh_rate;

	if (before_hbm) {
		if (hbm_en) {
			if (p_oplus_ofp_params->aod_unlocking == true) {
				if (panel_ext->oplus_ofp_aod_off_insert_black > 0) {
					bool need_aod_off_hbm_on_delay = (bool)p_oplus_ofp_params->aod_off_hbm_on_delay;

					if (panel_ext->oplus_ofp_aod_off_black_frame_total_time > 0) {
						if (ktime_sub(ktime_get(), p_oplus_ofp_params->aod_off_cmd_timestamp) >= ms_to_ktime(panel_ext->oplus_ofp_aod_off_black_frame_total_time)) {
							/* as te_rdy irq will be disabled in idle mode, so the aod_off_hbm_on_delay
							may not be accurate. then add oplus_ofp_aod_off_black_frame_total_time
							to check whether the timestamp of hbm cmd is in the black frames or not */
							OFP_DEBUG("no need to do some delay because it's not in the black frames\n");
							need_aod_off_hbm_on_delay = false;
						}
					}

					if (need_aod_off_hbm_on_delay) {
						/* do some frame delay to keep apart aod off cmd and hbm on cmd */
						te_count = p_oplus_ofp_params->aod_off_hbm_on_delay;
						if (refresh_rate == 120) {
							delay_us = (us_per_frame >> 1) + 900;
						} else {
							delay_us = (us_per_frame >> 1) + (us_per_frame >> 2);
						}
						p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
						OFP_INFO("wait %d te and %dus to keep apart aod off cmd and hbm on cmd\n", te_count, delay_us);
					} else if (panel_ext->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
						if (p_oplus_ofp_params->timing_switching) {
							/*
							 when timing switch cmds and hbm on cmds are sent at the same frame,
							 ddic has not changed to the latest frame rate, but delay has been calculated
							 according to the latest frame rate which would cause flicker issue,
							 so add one te wait to fix it
							*/
							te_count = 2;
						} else {
							/* wait 2 te ,then send hbm on cmds in the second half of the frame */
							te_count = 2;
						}
						if (refresh_rate == 120) {
							delay_us = (us_per_frame >> 1) + 900;
						} else {
							delay_us = (us_per_frame >> 1) + (us_per_frame >> 2);
						}
					}
				} else if (panel_ext->oplus_ofp_need_to_sync_data_in_aod_unlocking) {
					/* wait 1 te ,then send hbm on cmds in the second half of the frame */
					te_count = 1;
					if (refresh_rate == 120) {
						delay_us = (us_per_frame >> 1) + 900;
					} else {
						delay_us = (us_per_frame >> 1) + (us_per_frame >> 2);
					}
				}
			} else {
				/* backlight will affect hbm on/off time in some panel, need to keep apart the 51 cmd for stable hbm time */
				if (panel_ext->oplus_ofp_need_keep_apart_backlight) {
					/* flush the blocking frame , otherwise the dim Layer would be delay to next frame so that flicker happen */
					oplus_disp_trace_begin("cmdq_handle2");
					cmdq_handle2 = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
					cmdq_pkt_wait_no_clear(cmdq_handle2, mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
					/* delay one frame */
					cmdq_pkt_sleep(cmdq_handle2, CMDQ_US_TO_TICK(us_per_frame), CMDQ_GPR_R14);
					cmdq_pkt_flush(cmdq_handle2);
					cmdq_pkt_destroy(cmdq_handle2);
					oplus_disp_trace_end("cmdq_handle2");

					/* send hbm on cmd in next frame */
					te_count = 2;
				} else {
					te_count = 1;
				}

				if (refresh_rate == 120) {
					delay_us = (us_per_frame >> 1) + 900;
				} else {
					delay_us = (us_per_frame >> 1) + (us_per_frame >> 2);
				}
			}
		} else {
			if (panel_ext->oplus_ofp_pre_hbm_off_delay) {
				te_count = 1;
				/* the delay time bfore hbm off */
				delay_us = panel_ext->oplus_ofp_pre_hbm_off_delay * 1000;

				/* flush the blocking frame , otherwise the dim Layer would be delay to next frame so that flicker happen */
				oplus_disp_trace_begin("cmdq_handle2");
				cmdq_handle2 = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
				cmdq_pkt_wait_no_clear(cmdq_handle2, mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
				/* delay some time to wait for data */
				cmdq_pkt_sleep(cmdq_handle2, CMDQ_US_TO_TICK(delay_us), CMDQ_GPR_R14);
				cmdq_pkt_flush(cmdq_handle2);
				cmdq_pkt_destroy(cmdq_handle2);
				oplus_disp_trace_end("cmdq_handle2");
			}
		}
	} else {
		if (hbm_en) {
			if (p_oplus_ofp_params->aod_unlocking == true) {
				OFP_DEBUG("no need to delay after hbm on cmds have been sent in aod unlocking\n");
			} else if (panel_ext->oplus_ofp_hbm_on_delay) {
				te_count = 0;
				/* the time when hbm on need */
				delay_us = panel_ext->oplus_ofp_hbm_on_delay * 1000;
			}
		} else {
			if (panel_ext->oplus_ofp_hbm_off_delay) {
				te_count = 0;
				/* the time when hbm off need */
				delay_us = panel_ext->oplus_ofp_hbm_off_delay * 1000;
			}
		}
	}

	OFP_INFO("before_hbm = %d, te_count = %d, delay_us = %d\n", before_hbm, te_count, delay_us);
	ret = oplus_ofp_cmdq_pkt_wait(mtk_crtc, cmdq_handle, te_count, delay_us, hbm_en, before_hbm);
	if (ret) {
		OFP_ERR("oplus_ofp_cmdq_pkt_wait failed\n");
	}

	oplus_disp_trace_end("oplus_ofp_hbm_wait_handle");

	return ret;
}

static int oplus_ofp_set_panel_hbm(struct drm_crtc *crtc, bool hbm_en)
{
	bool doze_en = false;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct cmdq_pkt *cmdq_handle;

	if (!p_oplus_ofp_params || !crtc || !mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (oplus_ofp_video_mode_30hz_aod_is_enabled() && oplus_ofp_get_aod_state()) {
		OFP_INFO("should not set panel hbm if panel is in video mode aod state\n");
		return 0;
	}

	if (oplus_ofp_get_hbm_state() == hbm_en) {
		OFP_DEBUG("already in hbm state %d\n", hbm_en);
		return 0;
	}

	if (!(comp && comp->funcs && comp->funcs->io_cmd)) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!(mtk_crtc->enabled)) {
		OFP_ERR("skip, slept\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_set_panel_hbm");

	oplus_disp_trace_begin("mtk_drm_send_lcm_cmd_prepare");
	OFP_INFO("prepare to send hbm cmd\n");
	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);
	oplus_disp_trace_end("mtk_drm_send_lcm_cmd_prepare");

	if (oplus_ofp_get_aod_state()) {
		OFP_INFO("send aod off cmd before hbm on because panel is still in aod mode\n");
		oplus_ofp_aod_off_status_handle(mtk_crtc);

		oplus_disp_trace_begin("DSI_SET_DOZE");
		comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_DOZE, &doze_en);
		OFP_INFO("DSI_SET_DOZE %d\n", doze_en);
		oplus_disp_trace_end("DSI_SET_DOZE");
	}

	/* delay before hbm cmd */
	oplus_ofp_hbm_wait_handle(crtc, cmdq_handle, true, hbm_en);

	if (oplus_ofp_local_hbm_is_enabled()) {
		/* send lhbm pressed icon cmd */
		oplus_disp_trace_begin("OPLUS_OFP_SET_LHBM_PRESSED_ICON");
		comp->funcs->io_cmd(comp, cmdq_handle, OPLUS_OFP_SET_LHBM_PRESSED_ICON, &hbm_en);
		OFP_INFO("OPLUS_OFP_SET_LHBM_PRESSED_ICON %d\n", hbm_en);
		oplus_disp_trace_end("OPLUS_OFP_SET_LHBM_PRESSED_ICON");
	} else {
		/* send hbm cmd */
		oplus_disp_trace_begin("DSI_HBM_SET");
		comp->funcs->io_cmd(comp, cmdq_handle, DSI_HBM_SET, &hbm_en);
		OFP_INFO("DSI_HBM_SET %d\n", hbm_en);
		oplus_disp_trace_end("DSI_HBM_SET");
	}

	/* delay after hbm cmd */
	oplus_ofp_hbm_wait_handle(crtc, cmdq_handle, false, hbm_en);

	oplus_disp_trace_begin("mtk_drm_send_lcm_cmd_flush");
	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, true);
	oplus_ofp_set_hbm_state(hbm_en);
	oplus_disp_trace_end("mtk_drm_send_lcm_cmd_flush");

	if (hbm_en == false && p_oplus_ofp_params->aod_unlocking == true) {
		p_oplus_ofp_params->aod_unlocking = false;
		OFP_INFO("oplus_ofp_aod_unlocking: %d\n", p_oplus_ofp_params->aod_unlocking);
		oplus_disp_trace_c("%d|oplus_ofp_aod_unlocking|%d", g_commit_pid, p_oplus_ofp_params->aod_unlocking);
	}

	OFP_INFO("hbm cmd is flushed\n");
	oplus_disp_trace_end("oplus_ofp_set_panel_hbm");

	return 0;
}

int oplus_ofp_hbm_handle(void *drm_crtc, void *mtk_crtc_state, void *cmdq_pkt)
{
	int hbm_en = 0;
	unsigned int refresh_rate = 0;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct drm_crtc *crtc = drm_crtc;
	struct mtk_crtc_state *state = mtk_crtc_state;
	struct cmdq_pkt *cmdq_handle = cmdq_pkt;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!crtc || !state || !cmdq_handle || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->hbm_mode) {
		OFP_DEBUG("already in hbm mode %u\n", p_oplus_ofp_params->hbm_mode);
		return -EFAULT;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EFAULT;
	}
	refresh_rate = drm_mode_vrefresh(&mtk_crtc->base.state->adjusted_mode);

	oplus_disp_trace_begin("oplus_ofp_hbm_handle");

	hbm_en = state->prop_val[CRTC_PROP_HBM_ENABLE] & OPLUS_OFP_PROPERTY_DIMLAYER_AND_PRESSICON;
	OFP_DEBUG("FOD hbm_en = %d\n", hbm_en);

	if (mtk_crtc_is_frame_trigger_mode(crtc) || oplus_ofp_video_mode_aod_fod_is_enabled()) {
		if ((!state->prop_val[CRTC_PROP_DOZE_ACTIVE] && hbm_en > 0 && oplus_display_brightness != 0)
			|| (state->prop_val[CRTC_PROP_DOZE_ACTIVE] && hbm_en > 1 && oplus_display_brightness != 0)) {
			OFP_DEBUG("set hbm on\n");
			if (oplus_ofp_video_mode_30hz_aod_is_enabled()) {
				if (refresh_rate == 120) {
					oplus_ofp_set_panel_hbm(crtc, true);
				} else {
					OFP_INFO("refresh not 120fps\n");
				}

			} else {
				oplus_ofp_set_panel_hbm(crtc, true);
			}

			if (!oplus_ofp_local_hbm_is_enabled()) {
				/*bypass pq when enter hbm */
				mtk_atomic_hbm_bypass_pq(crtc, cmdq_handle, 1);
				OFP_DEBUG("bypass pq in hbm mode\n");
			}
		} else if (hbm_en == 0 || p_oplus_ofp_params->fp_press == false
			|| oplus_display_brightness == 0) {
			OFP_DEBUG("set hbm off\n");
			oplus_ofp_set_panel_hbm(crtc, false);

			if (!oplus_ofp_local_hbm_is_enabled()) {
				/*no need to bypass pq when exit hbm */
				mtk_atomic_hbm_bypass_pq(crtc, cmdq_handle, 0);
				OFP_DEBUG("no need to bypass pq in normal mode\n");
			}
		}
	}

	oplus_disp_trace_end("oplus_ofp_hbm_handle");

	return 0;
}

/* update pressed icon status */
int oplus_ofp_pressed_icon_status_update(int irq_type)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();
	static int last_hbm_enable = 0;

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_pressed_icon_status_update");
	if (irq_type == OPLUS_OFP_FRAME_DONE) {
		if (((last_hbm_enable & 0x2) == 0) && ((p_oplus_ofp_params->hbm_enable & 0x2) > 0)) {
			/* pressed icon has been flush to DDIC ram */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_ON_FRAME_DONE;
			OFP_INFO("pressed icon status: OPLUS_OFP_PRESSED_ICON_ON_FRAME_DONE\n");
		} else if (((last_hbm_enable & 0x2) > 0) && ((p_oplus_ofp_params->hbm_enable & 0x2) == 0)) {
			/* pressed icon has not been flush to DDIC ram */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_OFF_FRAME_DONE;
			OFP_INFO("pressed icon status: OPLUS_OFP_PRESSED_ICON_OFF_FRAME_DONE\n");
		}
		last_hbm_enable = p_oplus_ofp_params->hbm_enable;
	} else if (irq_type == OPLUS_OFP_TE_RDY) {
		if (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON_FRAME_DONE) {
			/* pressed icon has been displayed in panel */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_ON;
			OFP_INFO("pressed icon status: OPLUS_OFP_PRESSED_ICON_ON\n");
		} else if (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_OFF_FRAME_DONE) {
			/* pressed icon has not been displayed in panel */
			p_oplus_ofp_params->pressed_icon_status = OPLUS_OFP_PRESSED_ICON_OFF;
			OFP_INFO("pressed icon status: OPLUS_OFP_PRESSED_ICON_OFF\n");
		}
	}
	oplus_disp_trace_c("%d|oplus_ofp_pressed_icon_status|%d", g_commit_pid, p_oplus_ofp_params->pressed_icon_status);
	oplus_disp_trace_end("oplus_ofp_pressed_icon_status_update");

	return 0;
}

/* timer */
enum hrtimer_restart oplus_ofp_notify_uiready_timer_handler(struct hrtimer *timer)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_notify_uiready_timer_handler");
	/* notifer fingerprint that pressed icon ui is ready */
	OFP_INFO("send uiready: %d\n", p_oplus_ofp_params->notifier_chain_value);
	queue_work(p_oplus_ofp_params->uiready_event_wq, &p_oplus_ofp_params->uiready_event_work);
	oplus_disp_trace_c("%d|oplus_ofp_notifier_chain_value|%d", g_commit_pid, p_oplus_ofp_params->notifier_chain_value);
	oplus_disp_trace_end("oplus_ofp_notify_uiready_timer_handler");

	return HRTIMER_NORESTART;
}

void oplus_ofp_uiready_event_work_handler(struct work_struct *work_item)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return;
	}

	oplus_disp_trace_begin("oplus_ofp_uiready_event_work_handler");

	/* hbm is taking effect, so send uiready immediately */
	OFP_INFO("send uiready:%u\n", p_oplus_ofp_params->notifier_chain_value);
	mtk_disp_notifier_call_chain(MTK_ONSCREENFINGERPRINT_EVENT, &p_oplus_ofp_params->notifier_chain_value);
	oplus_disp_trace_c("%d|oplus_ofp_notifier_chain_value|%d", g_commit_pid, p_oplus_ofp_params->notifier_chain_value);

	oplus_disp_trace_end("oplus_ofp_uiready_event_work_handler");

	OFP_DEBUG("end\n");

	return;
}

/* notify uiready */
int oplus_ofp_notify_uiready(void *mtk_drm_crtc)
{
	static int last_notifier_chain_value = 0;
	static int last_hbm_state = false;
	static ktime_t hbm_cmd_timestamp = 0;
	ktime_t delta_time = 0;
	unsigned int refresh_rate = 0;
	unsigned int delay_ms = 0;
	struct mtk_drm_crtc *mtk_crtc = mtk_drm_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!mtk_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->aod_unlocking == true) {
		if (last_hbm_state == false && (oplus_ofp_get_hbm_state() == true)) {
			/* hbm cmd is sent to ddic */
			hbm_cmd_timestamp = ktime_get();
			OFP_INFO("hbm_cmd_timestamp:%lld\n", ktime_to_ms(hbm_cmd_timestamp));
		}

		if ((p_oplus_ofp_params->fp_press == true) && (p_oplus_ofp_params->hbm_enable & 0x2) > 0
			&& (oplus_ofp_get_hbm_state() == true) && (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON)) {
			/* pressed icon has been displayed in panel but hbm may not take effect */
			p_oplus_ofp_params->notifier_chain_value = 1;
		} else if (p_oplus_ofp_params->fp_press == false && (p_oplus_ofp_params->hbm_enable & 0x2) == 0) {
			/*  finger is not pressed down */
			p_oplus_ofp_params->notifier_chain_value = 0;
		}
	} else {
		if ((p_oplus_ofp_params->fp_press == true) && (p_oplus_ofp_params->hbm_enable & 0x2) > 0
			&& (oplus_ofp_get_hbm_state() == true) && (p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_ON)) {
			/* pressed icon has been displayed in panel and hbm is always in effect */
			p_oplus_ofp_params->notifier_chain_value = 1;
		} else if (p_oplus_ofp_params->fp_press == false && (p_oplus_ofp_params->hbm_enable & 0x2) == 0
					&& p_oplus_ofp_params->pressed_icon_status == OPLUS_OFP_PRESSED_ICON_OFF) {
			/* hbm is on but pressed icon has not been displayed */
			p_oplus_ofp_params->notifier_chain_value = 0;
		}
	}
	last_hbm_state = oplus_ofp_get_hbm_state();

	if (last_notifier_chain_value != p_oplus_ofp_params->notifier_chain_value) {
		oplus_disp_trace_begin("oplus_ofp_notify_uiready");
		if (p_oplus_ofp_params->aod_unlocking == true && p_oplus_ofp_params->notifier_chain_value == 1) {
			/* check whether hbm is taking effect or not */
			if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
				refresh_rate = mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
				delay_ms = mtk_crtc->panel_ext->params->oplus_ofp_hbm_on_delay + 1000/refresh_rate;
			} else {
				delay_ms = 17;
			}

			delta_time = ktime_sub(ktime_get(), hbm_cmd_timestamp);
			if (ktime_sub(ms_to_ktime(delay_ms), delta_time) > 0) {
				/* hbm is not taking effect, set a timer to wait and then send uiready */
				hrtimer_start(&p_oplus_ofp_params->timer, ktime_sub(ms_to_ktime(delay_ms), delta_time), HRTIMER_MODE_REL);
				OFP_INFO("delay %lld to notify\n", ktime_to_us(ktime_sub(ms_to_ktime(delay_ms), delta_time)));
			} else {
				OFP_INFO("queue uiready event work\n");
				queue_work(p_oplus_ofp_params->uiready_event_wq, &p_oplus_ofp_params->uiready_event_work);
			}
		} else {
			OFP_INFO("queue uiready event work\n");
			queue_work(p_oplus_ofp_params->uiready_event_wq, &p_oplus_ofp_params->uiready_event_work);
		}
		last_notifier_chain_value = p_oplus_ofp_params->notifier_chain_value;
		oplus_disp_trace_end("oplus_ofp_notify_uiready");
	}

	return 0;
}

/*
 due to the fact that the brightness of lhbm pressed icon may change with the backlight,
 it is necessary to readjust the lhbm pressed icon grayscale to meet the requirements of fingerprint unlocking
*/
int oplus_ofp_lhbm_pressed_icon_grayscale_update(void *para_list, unsigned int bl_level)
{
	bool pwm_is_changing = false;
	bool need_to_update_grayscale = false;
	static bool last_pwm_state = false;
	unsigned char *tx_buf = para_list;
	unsigned char tx_buf_0[7] = {0xC2, 0xFE, 0xFE, 0xDD, 0x61, 0x00, 0x62};
	unsigned char tx_buf_1[7] = {0xC2, 0xF9, 0xF9, 0xDA, 0x61, 0x00, 0x62};
	unsigned char tx_buf_2[7] = {0xC2, 0xF5, 0xF5, 0xD6, 0x61, 0x00, 0x62};
	unsigned char tx_buf_3[7] = {0xC2, 0xEF, 0xEF, 0xDA, 0x61, 0x00, 0x62};
	unsigned char tx_buf_4[7] = {0xC2, 0xE9, 0xE9, 0xD4, 0x61, 0x00, 0x62};
	unsigned char tx_buf_5[7] = {0xC2, 0xFA, 0xFA, 0xD9, 0x61, 0x00, 0x62};
	unsigned char tx_buf_6[7] = {0xC2, 0xF4, 0xF4, 0xD2, 0x61, 0x00, 0x62};
	unsigned char tx_buf_7[7] = {0xC2, 0xF0, 0xF0, 0xD6, 0x61, 0x00, 0x62};
	unsigned char tx_buf_8[7] = {0xC2, 0xEA, 0xEA, 0xD2, 0x61, 0x00, 0x62};
	unsigned char tx_buf_9[7] = {0xC2, 0xE5, 0xE5, 0xCE, 0x61, 0x00, 0x62};
	int rc = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, no need to update lhbm pressed icon grayscale\n");
		return 0;
	}

	if (!tx_buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma) {
		OFP_DEBUG("need_to_update_lhbm_pressed_icon_gamma is not config, no need to update lhbm pressed icon grayscale\n");
		return 0;
	}

	if (p_oplus_ofp_params->failed_to_read_lhbm_pressed_icon_gamma) {
		OFP_DEBUG("lhbm pressed icon gamma reading is failed, no need to update lhbm pressed icon grayscale\n");
		return 0;
	}

	pwm_is_changing = (last_pwm_state != oplus_panel_pwm_onepulse_is_enabled());

	if (!oplus_panel_pwm_onepulse_is_enabled()) {
		if (((last_backlight == 0x0000) || (last_backlight > 0x0700) || pwm_is_changing)
				&& ((bl_level > 0x0000) && (bl_level <= 0x0700))) {
			memcpy(tx_buf, tx_buf_0, 7);
			need_to_update_grayscale = true;
		} else if (((last_backlight <= 0x0700) || (last_backlight > 0x08F0) || pwm_is_changing)
						&& ((bl_level > 0x0700) && (bl_level <= 0x08F0))) {
			memcpy(tx_buf, tx_buf_1, 7);
			need_to_update_grayscale = true;
		} else if (((last_backlight <= 0x08F0) || (last_backlight > 0x0A88) || pwm_is_changing)
						&& ((bl_level > 0x08F0) && (bl_level <= 0x0A88))) {
			memcpy(tx_buf, tx_buf_2, 7);
			need_to_update_grayscale = true;
		} else if (((last_backlight <= 0x0A88) || (last_backlight > 0x0C00) || pwm_is_changing)
						&& ((bl_level > 0x0A88) && (bl_level <= 0x0C00))) {
			memcpy(tx_buf, tx_buf_3, 7);
			need_to_update_grayscale = true;
		} else if (((last_backlight <= 0x0C00) || (last_backlight > 0x0DBB) || pwm_is_changing)
						&& ((bl_level > 0x0C00) && (bl_level <= 0x0DBB))) {
			memcpy(tx_buf, tx_buf_4, 7);
			need_to_update_grayscale = true;
		}
	} else {
		if (((last_backlight == 0x0000) || (last_backlight > 0x0700) || pwm_is_changing)
				&& ((bl_level > 0x0000) && (bl_level <= 0x0700))) {
			memcpy(tx_buf, tx_buf_5, 7);
			need_to_update_grayscale = true;
		} else if (((last_backlight <= 0x0700) || (last_backlight > 0x08F0) || pwm_is_changing)
						&& ((bl_level > 0x0700) && (bl_level <= 0x08F0))) {
			memcpy(tx_buf, tx_buf_6, 7);
			need_to_update_grayscale = true;
		} else if (((last_backlight <= 0x08F0) || (last_backlight > 0x0A88) || pwm_is_changing)
						&& ((bl_level > 0x08F0) && (bl_level <= 0x0A88))) {
			memcpy(tx_buf, tx_buf_7, 7);
			need_to_update_grayscale = true;
		} else if (((last_backlight <= 0x0A88) || (last_backlight > 0x0C00) || pwm_is_changing)
						&& ((bl_level > 0x0A88) && (bl_level <= 0x0C00))) {
			memcpy(tx_buf, tx_buf_8, 7);
			need_to_update_grayscale = true;
		} else if (((last_backlight <= 0x0C00) || (last_backlight > 0x0DBB) || pwm_is_changing)
						&& ((bl_level > 0x0C00) && (bl_level <= 0x0DBB))) {
			memcpy(tx_buf, tx_buf_9, 7);
			need_to_update_grayscale = true;
		}
	}

	if (need_to_update_grayscale) {
		OFP_INFO("lhbm pressed icon grayscale:0x%02X, 0x%02X, 0x%02X\n", tx_buf[1],  tx_buf[2],  tx_buf[3]);
		rc = 1;
	}

	last_pwm_state = oplus_panel_pwm_onepulse_is_enabled();

	OFP_DEBUG("end\n");

	return rc;
}
EXPORT_SYMBOL(oplus_ofp_lhbm_pressed_icon_grayscale_update);

/* need filter backlight in hbm state and aod unlocking process */
bool oplus_ofp_backlight_filter(void *drm_crtc, void *cmdq_pkt, unsigned int bl_level)
{
	bool hbm_en = false;
	uint64_t hbm_enable = false;
	bool need_filter_backlight = false;
	struct mtk_crtc_state *crtc_state;
	struct drm_crtc *crtc = drm_crtc;
	struct cmdq_pkt *cmdq_handle = cmdq_pkt;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct mtk_ddp_comp *comp = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid crtc param\n");
		return false;
	}

	crtc_state = to_mtk_crtc_state(crtc->state);
	if (!crtc_state) {
		OFP_ERR("Invalid mtk_crtc or crtc_state param\n");
		return false;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc) {
		OFP_ERR("Invalid mtk_crtc param\n");
		return false;
	}

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!(comp && comp->funcs && comp->funcs->io_cmd)) {
		OFP_ERR("Invalid comp params\n");
		return false;
	}

	if (!mtk_crtc->enabled) {
		OFP_ERR("mtk crtc is not enabled\n");
		return false;
	}

	oplus_disp_trace_begin("oplus_ofp_backlight_filter");

	hbm_enable = crtc_state->prop_val[CRTC_PROP_HBM_ENABLE];
	if (oplus_ofp_get_hbm_state()) {
		if (bl_level == 0) {
			if (oplus_ofp_local_hbm_is_enabled()) {
				/* send lhbm pressed icon cmd */
				oplus_disp_trace_begin("OPLUS_OFP_SET_LHBM_PRESSED_ICON");
				comp->funcs->io_cmd(comp, cmdq_handle, OPLUS_OFP_SET_LHBM_PRESSED_ICON, &hbm_en);
				OFP_INFO("OPLUS_OFP_SET_LHBM_PRESSED_ICON %d\n", hbm_en);
				oplus_disp_trace_end("OPLUS_OFP_SET_LHBM_PRESSED_ICON");
			}
			oplus_ofp_set_hbm_state(false);
			OFP_DEBUG("backlight is 0, set hbm state to false\n");
			if (p_oplus_ofp_params->aod_unlocking == true) {
				p_oplus_ofp_params->aod_unlocking = false;
				OFP_INFO("oplus_ofp_aod_unlocking: %d\n", p_oplus_ofp_params->aod_unlocking);
				oplus_disp_trace_c("%d|oplus_ofp_aod_unlocking|%d", g_commit_pid, p_oplus_ofp_params->aod_unlocking);
			}
			need_filter_backlight = false;
		} else {
			if (!oplus_ofp_local_hbm_is_enabled() || p_oplus_ofp_params->hbm_mode) {
				OFP_INFO("hbm state is true, filter backlight %u setting\n", bl_level);
				need_filter_backlight = true;
			} else if (p_oplus_ofp_params->need_to_update_lhbm_pressed_icon_gamma && (bl_level > OPLUS_OFP_900NIT_DBV_LEVEL)) {
				OFP_INFO("hbm state is true and backlight lvl is greater than OPLUS_OFP_900NIT_DBV_LEVEL, filter backlight %u setting\n", bl_level);
				need_filter_backlight = true;
			}
		}
	} else if (p_oplus_ofp_params->aod_unlocking && p_oplus_ofp_params->fp_press && bl_level
				&& !oplus_ofp_full_screen_aod_mode_is_enabled()
				&& (!oplus_ofp_video_mode_30hz_aod_is_enabled() && oplus_ofp_get_hbm_state())) {
		OFP_INFO("aod unlocking is true, filter backlight setting\n");
		need_filter_backlight = true;
	} else if (!p_oplus_ofp_params->aod_unlocking && !p_oplus_ofp_params->doze_active
				&& (hbm_enable & OPLUS_OFP_PROPERTY_DIM_LAYER) && bl_level) {
		/* backlight will affect hbm on time in some panel, need to separate the 51 cmd for stable hbm on time */
		OFP_INFO("dim layer exist, filter backlight %u setting in advance\n", bl_level);
		need_filter_backlight = true;
	} else if (oplus_ofp_get_aod_state() && !oplus_ofp_video_mode_30hz_aod_is_enabled()) {
		OFP_INFO("aod state is true, filter backlight %u setting\n", bl_level);
		need_filter_backlight = true;
	} else if (!oplus_ofp_get_aod_state() && (hbm_enable & OPLUS_OFP_PROPERTY_AOD_LAYER) && bl_level
				&& !((p_oplus_ofp_params->longrui_aod_config & OPLUS_OFP_A_MIRROR_TO_THE_END_AOD_CONFIG)
					&& (p_oplus_ofp_params->longrui_aod_mode & OPLUS_OFP_A_MIRROR_TO_THE_END_AOD_MODE))
				&& !oplus_ofp_full_screen_aod_mode_is_enabled()) {
		OFP_INFO("aod layer exist, filter backlight %u setting\n", bl_level);
		need_filter_backlight = true;
	} else if (p_oplus_ofp_params->dimlayer_hbm || hbm_enable) {
		OFP_INFO("backlight lvl:%u\n", bl_level);
	}

	oplus_disp_trace_end("oplus_ofp_backlight_filter");
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &bl_level);
	OFP_DEBUG("end\n");

	return need_filter_backlight;
}
EXPORT_SYMBOL(oplus_ofp_backlight_filter);

int oplus_ofp_lhbm_setbacklight(void *dsi)
{
	struct mtk_dsi *mtk_dsi = dsi;
	bool lhbm_pressed_icon_on;

	if (!mtk_dsi) {
		return -EINVAL;
	}
	if((oplus_ofp_is_supported()) && (oplus_ofp_get_hbm_state())) {
		lhbm_pressed_icon_on = true;
		mtk_ddp_comp_io_cmd(&mtk_dsi->ddp_comp, NULL, OPLUS_OFP_SET_LHBM_PRESSED_ICON, &lhbm_pressed_icon_on);
	}
	return 1;
}
EXPORT_SYMBOL(oplus_ofp_lhbm_setbacklight);

int oplus_ofp_aod_off_backlight_recovery(void *drm_crtc, void *mtk_crtc_state, void *cmdq_pkt)
{
	int rc = 0;
	char *panel_name;
	uint64_t hbm_enable = 0;
	static bool last_aod_layer_status = false;
	bool new_aod_layer_status = false;
	struct drm_crtc *crtc = drm_crtc;
	struct mtk_crtc_state *state = mtk_crtc_state;
	struct cmdq_pkt *cmdq_handle = cmdq_pkt;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct mtk_ddp_comp *comp = NULL;
	struct mtk_panel_ext *ext = NULL;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if(oplus_ofp_video_mode_30hz_aod_is_enabled() && !oplus_apollo_unsupported())
		return 0;

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc) {
		OFP_ERR("Invalid mtk_crtc param\n");
		return false;
	}

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return false;
	}

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!(comp && comp->funcs && comp->funcs->io_cmd)) {
		OFP_ERR("Invalid comp params\n");
		return false;
	}

	ext = mtk_crtc->panel_ext;
	if(ext == NULL) {
		DDPINFO("%s %d mtk_crtc->panel_ext is NULL\n", __func__, __LINE__);
		return false;
	}

	hbm_enable = state->prop_val[CRTC_PROP_HBM_ENABLE];
	new_aod_layer_status = hbm_enable & OPLUS_OFP_PROPERTY_AOD_LAYER;
	OFP_DEBUG("Aod layer property %llu\n", hbm_enable);

	if (last_aod_layer_status && !new_aod_layer_status) {
		mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);
		OFP_INFO("recovery backlight level = %d after aod layer disappear, vact_timing_fps:%d\n", oplus_display_brightness, ext->params->dyn_fps.vact_timing_fps);
		mtk_ddp_comp_io_cmd(comp, NULL, GET_PANEL_NAME, &panel_name);
		if (oplus_panel_pwm_onepulse_is_enabled() && oplus_display_brightness > 1
			&& (!strcmp(panel_name, "panel_ac094_p_3_a0004_dsi_cmd"))) {
			oplus_disp_trace_begin("mtk_drm_send_aod_off_bl_recovery");
			ofp_aod_off_swtich_pulse = true;
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_PULSE_BL, &oplus_display_brightness);
			oplus_disp_trace_end("mtk_drm_send_aod_off_bl_recovery");
		} else {
			oplus_disp_trace_begin("mtk_drm_send_aod_off_bl_recovery");
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &oplus_display_brightness);
			oplus_disp_trace_end("mtk_drm_send_aod_off_bl_recovery");
		}
		mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);
	}

	last_aod_layer_status = new_aod_layer_status;
	return rc;
}

/* -------------------- aod -------------------- */
/* aod off status handle */
int oplus_ofp_aod_off_status_handle(void *mtk_drm_crtc)
{
	struct mtk_drm_crtc *mtk_crtc = mtk_drm_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!mtk_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_aod_off_status_handle");
	/* doze disable status handle */
	OFP_INFO("aod off status handle\n");

/* #ifdef OPLUS_ADFR */
	/* switch to tp vsync */
	if (oplus_adfr_is_support() && (oplus_adfr_get_vsync_mode() == OPLUS_EXTERNAL_TE_TP_VSYNC)) {
		oplus_adfr_aod_fod_vsync_switch(mtk_crtc, false);
	}
/* #endif */ /* OPLUS_ADFR */

	if (oplus_ofp_is_supported()) {
		/* update aod unlocking value */
		oplus_ofp_aod_unlocking_update();
	}

	oplus_ofp_set_aod_state(false);

	/* aod off cmd is sent to ddic */
	p_oplus_ofp_params->aod_off_cmd_timestamp = ktime_get();
	OFP_DEBUG("aod_off_cmd_timestamp:%lld\n", ktime_to_ms(p_oplus_ofp_params->aod_off_cmd_timestamp));

	oplus_disp_trace_end("oplus_ofp_aod_off_status_handle");

	return 0;
}

/* aod status handle */
int oplus_ofp_doze_status_handle(bool doze_enable, void *drm_crtc, void *mtk_panel_ext, void *drm_panel, void *mtk_dsi, void *dcs_write_gce_pack)
{
	struct drm_crtc *crtc = drm_crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!crtc || !mtk_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_doze_status_handle");

	if (doze_enable) {
		OFP_INFO("doze status handle before doze enable\n");
		/* before doze enable */
		if (oplus_ofp_is_supported()) {
			/* hbm mode -> normal mode -> aod mode */
			if (oplus_ofp_get_hbm_state() == true) {
				if ((mtk_crtc_is_frame_trigger_mode(crtc) && !oplus_ofp_full_screen_aod_mode_is_enabled())
						|| (oplus_ofp_video_mode_aod_fod_is_enabled() && !oplus_ofp_full_screen_aod_mode_is_enabled())) {
					struct mtk_panel_ext *ext = mtk_panel_ext;

					oplus_disp_trace_begin("oplus_ofp_hbm_off_before_doze_enable");
					if (oplus_ofp_local_hbm_is_enabled()) {
						if (ext && ext->funcs && ext->funcs->oplus_ofp_set_lhbm_pressed_icon) {
							OFP_INFO("lhbm pressed icon off before doze enable\n");
							ext->funcs->oplus_ofp_set_lhbm_pressed_icon(drm_panel, mtk_dsi, dcs_write_gce_pack, NULL, false);
						} else if (ext && ext->funcs && ext->funcs->oplus_ofp_set_lhbm_pressed_icon_single) {
							OFP_INFO("lhbm pressed icon single off before doze enable\n");
							ext->funcs->oplus_ofp_set_lhbm_pressed_icon_single(drm_panel, mtk_dsi, dcs_write_gce_pack, NULL, false);
						}
					} else {
						if (ext && ext->funcs && ext->funcs->oplus_hbm_set_cmdq) {
							OFP_INFO("hbm off before doze enable\n");
							ext->funcs->oplus_hbm_set_cmdq(drm_panel, mtk_dsi, dcs_write_gce_pack, NULL, false);
						}
					}
					oplus_ofp_set_hbm_state(false);
					oplus_disp_trace_end("oplus_ofp_hbm_off_before_doze_enable");
				}
			}

			/* reset aod unlocking flag when fingerprint unlocking failed */
			if (p_oplus_ofp_params->aod_unlocking == true) {
				p_oplus_ofp_params->aod_unlocking = false;
				OFP_INFO("oplus_ofp_aod_unlocking: %d\n", p_oplus_ofp_params->aod_unlocking);
				oplus_disp_trace_c("%d|oplus_ofp_aod_unlocking|%d", g_commit_pid, p_oplus_ofp_params->aod_unlocking);
			}
		}
/* #ifdef OPLUS_ADFR */
		/* switch to te */
		if (oplus_adfr_is_support() && (oplus_adfr_get_vsync_mode() == OPLUS_EXTERNAL_TE_TP_VSYNC)
				&& !oplus_ofp_full_screen_aod_mode_is_enabled()) {
			oplus_adfr_aod_fod_vsync_switch(mtk_crtc, true);
		}
/* #endif */ /* OPLUS_ADFR */

		if (!oplus_ofp_full_screen_aod_mode_is_enabled()) {
			oplus_ofp_set_aod_state(true);
		}
	} else {
		oplus_ofp_aod_off_status_handle(mtk_crtc);
	}

	oplus_disp_trace_end("oplus_ofp_doze_status_handle");

	return 0;
}

int oplus_ofp_set_aod_light_mode_after_doze_enable(void *mtk_panel_ext, void *mtk_dsi, void *dcs_write_gce)
{
	struct mtk_panel_ext *ext = mtk_panel_ext;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!ext || !mtk_dsi || !dcs_write_gce || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->aod_light_mode) {
		if (ext && ext->funcs && ext->funcs->set_aod_light_mode) {
			ext->funcs->set_aod_light_mode(mtk_dsi, dcs_write_gce, NULL, p_oplus_ofp_params->aod_light_mode);
			OFP_INFO("set_aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
		}
	}

	OFP_DEBUG("end\n");

	return 0;
}

/* aod off cmd cmdq set */
int oplus_ofp_aod_off_set_cmdq(struct drm_crtc *crtc)
{
	bool is_frame_mode;
	int i, j;
	bool doze_en = false;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int crtc_id = drm_crtc_index(&mtk_crtc->base);
	struct mtk_ddp_comp *output_comp, *comp;
	struct cmdq_pkt *cmdq_handle;
	struct cmdq_client *client;
	struct mtk_crtc_state *crtc_state;

	if (!crtc || !mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_aod_off_set_cmdq");

	crtc_state = to_mtk_crtc_state(crtc->state);
	if (!crtc_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
		OFP_INFO("not in doze mode\n");
	}

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!output_comp)) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		oplus_disp_trace_end("oplus_ofp_aod_off_set_cmdq");
		return -ENODEV;
	}

	client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	if (!mtk_crtc->enabled) {
		mtk_drm_crtc_wk_lock(crtc, 1, __func__, __LINE__);
		/* power on mtcmos */
		oplus_disp_trace_begin("power_on_mtcmos");
		mtk_drm_top_clk_prepare_enable(crtc->dev);

		/*APSRC control*/
		mtk_crtc_v_idle_apsrc_control(crtc, NULL, false, false, crtc_id, true);

		cmdq_mbox_enable(client->chan);
		if (mtk_crtc_with_event_loop(crtc) &&
				(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_event_loop(crtc);
		if (mtk_crtc_with_trigger_loop(crtc))
			mtk_crtc_start_trig_loop(crtc);

		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_ENABLE, NULL);

		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_dump_analysis(comp);

		OFP_INFO("power on mtcmos\n");
		oplus_disp_trace_end("power_on_mtcmos");
	}

	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);
	/* send LCM CMD */
	oplus_disp_trace_begin("prepare_to_send_aod_off_cmd");
	OFP_INFO("prepare to send aod off cmd\n");
	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

	if (is_frame_mode || mtk_crtc->gce_obj.client[CLIENT_DSI_CFG] == NULL)
		cmdq_handle =
			cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	else
		cmdq_handle =
			cmdq_pkt_create(
			mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -EINVAL;
	}

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, 0);

	/* Record Vblank start timestamp */
	mtk_vblank_config_rec_start(mtk_crtc, cmdq_handle, SET_BL);
	oplus_disp_trace_end("prepare_to_send_aod_off_cmd");

	oplus_disp_trace_begin("DSI_SET_DOZE");
	oplus_ofp_set_aod_state(false);
	oplus_ofp_aod_unlocking_update();
	if (output_comp->funcs && output_comp->funcs->io_cmd)
		output_comp->funcs->io_cmd(output_comp,
			cmdq_handle, DSI_SET_DOZE, &doze_en);
	OFP_INFO("DSI_SET_DOZE %d\n", doze_en);
	oplus_disp_trace_end("DSI_SET_DOZE");

	oplus_disp_trace_begin("flush_aod_off_cmd");
	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	/* Record Vblank end timestamp and calculate duration */
	mtk_vblank_config_rec_end_cal(mtk_crtc, cmdq_handle, SET_BL);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
	OFP_INFO("flush aod off cmd\n");
	oplus_disp_trace_end("flush_aod_off_cmd");

	oplus_ofp_aod_off_status_handle(mtk_crtc);

	if (!mtk_crtc->enabled) {
		oplus_disp_trace_begin("power_off_mtcmos");
		if (mtk_crtc_with_trigger_loop(crtc))
			mtk_crtc_stop_trig_loop(crtc);

		if (mtk_crtc_with_event_loop(crtc) &&
				(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_stop_event_loop(crtc);
		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_DISABLE, NULL);

		cmdq_mbox_disable(client->chan);
		DDPFENCE("%s:%d power_state = false\n", __func__, __LINE__);
		mtk_drm_top_clk_disable_unprepare(crtc->dev);
		mtk_drm_crtc_wk_lock(crtc, 0, __func__, __LINE__);
		OFP_INFO("power off mtcmos\n");
		oplus_disp_trace_end("power_off_mtcmos");
	}

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	oplus_disp_trace_end("oplus_ofp_aod_off_set_cmdq");

	return 0;
}

int oplus_ofp_crtc_aod_off_set(void)
{
	int ret = 0;
	struct drm_crtc *crtc;
	struct drm_device *drm_dev = get_drm_device();

	OFP_DEBUG("start\n");

	if (IS_ERR_OR_NULL(drm_dev)) {
		OFP_ERR("invalid drm dev\n");
		return -EINVAL;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(drm_dev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (IS_ERR_OR_NULL(crtc)) {
		OFP_ERR("find crtc fail\n");
		return -EINVAL;
	}

	ret = oplus_ofp_aod_off_set_cmdq(crtc);

	OFP_DEBUG("end\n");

	return ret;
}

int oplus_ofp_video_mode_aod_handle(void *drm_crtc, void *mtk_panel_ext, void *drm_panel, void *mtk_dsi, void *dcs_write_gce, void *cmdq_handle)
{
	unsigned int refresh_rate = 0;
	static unsigned int last_refresh_rate = 0;
	struct drm_crtc *crtc = drm_crtc;
	struct mtk_panel_ext *ext = mtk_panel_ext;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *handle = cmdq_handle;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!oplus_ofp_video_mode_aod_fod_is_enabled()) {
		OFP_ERR("video mode aod and fod are not enabled, no need to handle video mode aod\n");
		return 0;
	}

	if (!crtc || !mtk_crtc || !handle || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_video_mode_aod_handle");

	refresh_rate = drm_mode_vrefresh(&mtk_crtc->base.state->adjusted_mode);

	/* due to aod sequence requirements, the aod of video mode is bound to 30hz timing */
	OFP_INFO("video mode aod state:%d, refresh_rate:%d\n", oplus_ofp_get_aod_state(), refresh_rate);
	if (!oplus_ofp_get_aod_state() && (refresh_rate == 30)) {
		oplus_ofp_set_aod_state(true);

		/* aod on */
		if (ext && ext->funcs && ext->funcs->doze_enable) {
			OFP_INFO("debug for doze_enable\n");
			ext->funcs->doze_enable(drm_panel, mtk_dsi, dcs_write_gce, handle);
		}

		if (p_oplus_ofp_params->aod_light_mode) {
			if (ext && ext->funcs && ext->funcs->set_aod_light_mode) {
				ext->funcs->set_aod_light_mode(mtk_dsi, dcs_write_gce, handle, p_oplus_ofp_params->aod_light_mode);
				OFP_INFO("set_aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
			}
		}
	} else if (oplus_ofp_get_aod_state() && (refresh_rate != 30)) {
		if (ext && ext->funcs && ext->funcs->doze_disable) {
			OFP_INFO("debug for doze_disable\n");
			ext->funcs->doze_disable(drm_panel, mtk_dsi, dcs_write_gce, handle);
			oplus_ofp_set_aod_state(false);
		}
	}
	last_refresh_rate = refresh_rate;
	oplus_disp_trace_end("oplus_ofp_video_mode_aod_handle");

	OFP_DEBUG("end\n");

	return 0;
}

void oplus_ofp_aod_off_set_work_handler(struct work_struct *work_item)
{
	int ret = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();
	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
	}

	oplus_disp_trace_begin("oplus_ofp_aod_off_set_work_handler");
	OFP_INFO("send aod off cmd to speed up aod unlocking\n");
	if(oplus_ofp_video_mode_30hz_aod_is_enabled()) {
		oplus_ofp_aod_unlocking_update();
		ret = mtk_ddic_vdo_aod_ctrl(0, true);
		p_oplus_ofp_params->aod_off_cmd_timestamp = ktime_get();
		OFP_DEBUG("aod_off_cmd_timestamp:%lld\n", ktime_to_ms(p_oplus_ofp_params->aod_off_cmd_timestamp));
	} else
		ret = oplus_ofp_crtc_aod_off_set();
	if (ret) {
		OFP_ERR("failed to send aod off cmd\n");
	}
	oplus_disp_trace_end("oplus_ofp_aod_off_set_work_handler");

	return;
}

int oplus_ofp_aod_off_set(void)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_DEBUG("start\n");

	if (oplus_ofp_get_hbm_state()) {
		OFP_DEBUG("ignore aod off setting in hbm state\n");
		return 0;
	}

	oplus_disp_trace_begin("oplus_ofp_aod_off_set");
	if (oplus_ofp_get_aod_state() && p_oplus_ofp_params->doze_active != 0) {
		OFP_INFO("queue aod off set work\n");
		queue_work(p_oplus_ofp_params->aod_off_set_wq, &p_oplus_ofp_params->aod_off_set_work);
		if(!oplus_ofp_video_mode_30hz_aod_is_enabled())
			oplus_ofp_set_aod_state(false);
	}
	oplus_disp_trace_end("oplus_ofp_aod_off_set");

	OFP_DEBUG("end\n");

	return 0;
}

/*
 touchpanel notify touchdown event when fingerprint is pressed,
 then display driver send aod off cmd immediately and vsync change to 120hz,
 so that press icon layer can sent down faster
*/
int oplus_ofp_touchpanel_event_notifier_call(struct notifier_block *nb, unsigned long action, void *data)
{
	struct touchpanel_event *tp_event = (struct touchpanel_event *)data;

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("no need to send aod off cmd in doze mode to speed up fingerprint unlocking\n");
		return NOTIFY_OK;
	}

	oplus_disp_trace_begin("oplus_ofp_touchpanel_event_notifier_call");
	if (tp_event) {
		if (action == EVENT_ACTION_FOR_FINGPRINT) {
			OFP_DEBUG("EVENT_ACTION_FOR_FINGPRINT\n");

			if (tp_event->touch_state == 1) {
				OFP_INFO("tp touchdown\n");
				/* send aod off cmd in doze mode to speed up fingerprint unlocking */
				oplus_ofp_aod_off_set();
			}
		}
	}
	oplus_disp_trace_end("oplus_ofp_touchpanel_event_notifier_call");

	return NOTIFY_OK;
}

/*
 as there have some black frames are inserted in aod off cmd flow which will affect hbm on cmd execution time,
 so check how many black frames have taken effect first,
 then calculate delay time to keep apart aod off cmd and hbm on cmd to make sure ui ready is accurate
*/
int oplus_ofp_aod_off_hbm_on_delay_check(void *mtk_drm_crtc)
{
	static bool last_aod_unlocking = false;
	static unsigned int te_rdy_irq_count = 0;
	unsigned int aod_off_insert_black = 0;
	struct mtk_drm_crtc *mtk_crtc = mtk_drm_crtc;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!oplus_ofp_is_supported() || oplus_ofp_video_mode_aod_fod_is_enabled()) {
		OFP_DEBUG("no need to check aod off hbm on delay\n");
		return 0;
	}

	oplus_disp_trace_begin("oplus_ofp_aod_off_hbm_on_delay_check");

	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
			&& mtk_crtc->panel_ext->params->oplus_ofp_aod_off_insert_black > 0) {
		if (p_oplus_ofp_params->aod_unlocking == true) {
			if (last_aod_unlocking == false) {
				te_rdy_irq_count = 1;
			} else if (te_rdy_irq_count != 0 && te_rdy_irq_count < 10) {
				te_rdy_irq_count++;
			} else {
				/* 10 irq is enough */
				te_rdy_irq_count = 10;
			}
		} else {
			te_rdy_irq_count = 0;
			p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
		}

		aod_off_insert_black = mtk_crtc->panel_ext->params->oplus_ofp_aod_off_insert_black;
		if (te_rdy_irq_count < aod_off_insert_black) {
			p_oplus_ofp_params->aod_off_hbm_on_delay = aod_off_insert_black - te_rdy_irq_count;
			OFP_DEBUG("aod_off_insert_black=%d,te_rdy_irq_count=%d,aod_off_hbm_on_delay=%d\n",
				aod_off_insert_black, te_rdy_irq_count, p_oplus_ofp_params->aod_off_hbm_on_delay);
		} else {
			p_oplus_ofp_params->aod_off_hbm_on_delay = 0;
		}

		last_aod_unlocking = p_oplus_ofp_params->aod_unlocking;
	}

	oplus_disp_trace_end("oplus_ofp_aod_off_hbm_on_delay_check");

	OFP_DEBUG("end\n");

	return 0;
}

/* -------------------- node -------------------- */
/* fp_type */
int oplus_ofp_set_fp_type(void *buf)
{
	unsigned int *fp_type = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_set_fp_type");

	p_oplus_ofp_params->fp_type = *fp_type;
	OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);
	oplus_disp_trace_c("%d|oplus_ofp_fp_type|%d", g_commit_pid, p_oplus_ofp_params->fp_type);

	oplus_disp_trace_end("oplus_ofp_set_fp_type");

	OFP_DEBUG("end\n");

	return 0;
}
EXPORT_SYMBOL(oplus_ofp_set_fp_type);

int oplus_ofp_get_fp_type(void *buf)
{
	unsigned int *fp_type = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_get_fp_type");

	*fp_type = p_oplus_ofp_params->fp_type;
	OFP_DEBUG("fp_type:0x%x\n", *fp_type);

	oplus_disp_trace_end("oplus_ofp_get_fp_type");

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_fp_type_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int fp_type = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	if (kstrtouint(buf, 10, &fp_type)) {
		OFP_ERR("kstrtouint error!\n");
		return count;
	}

	oplus_disp_trace_begin("oplus_ofp_set_fp_type_attr");

	p_oplus_ofp_params->fp_type = fp_type;
	OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);
	oplus_disp_trace_c("%d|oplus_ofp_fp_type|%d", g_commit_pid, p_oplus_ofp_params->fp_type);

	oplus_disp_trace_end("oplus_ofp_set_fp_type_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_fp_type_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_get_fp_type_attr");

	OFP_INFO("fp_type:0x%x\n", p_oplus_ofp_params->fp_type);

	oplus_disp_trace_end("oplus_ofp_get_fp_type_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%d\n", p_oplus_ofp_params->fp_type);
}

/* fod part */
/* dimlayer_hbm */
int oplus_ofp_set_dimlayer_hbm(void *buf)
{
	unsigned int *dimlayer_hbm = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!oplus_ofp_is_supported() || oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to set dimlayer hbm\n");
		return 0;
	}

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	p_oplus_ofp_params->dimlayer_hbm = *dimlayer_hbm;
	OFP_INFO("dimlayer_hbm:%u\n", p_oplus_ofp_params->dimlayer_hbm);

	OFP_DEBUG("end\n");

	return 0;
}

int oplus_ofp_get_dimlayer_hbm(void *buf)
{
	unsigned int *dimlayer_hbm = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	*dimlayer_hbm = p_oplus_ofp_params->dimlayer_hbm;
	OFP_INFO("dimlayer_hbm:%u\n", *dimlayer_hbm);

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_dimlayer_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int dimlayer_hbm = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!oplus_ofp_is_supported() || oplus_ofp_oled_capacitive_is_enabled() || oplus_ofp_ultrasonic_is_enabled()) {
		OFP_DEBUG("no need to set dimlayer hbm\n");
		return count;
	}

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%u", &dimlayer_hbm);

	p_oplus_ofp_params->dimlayer_hbm = dimlayer_hbm;
	OFP_INFO("dimlayer_hbm:%u\n", p_oplus_ofp_params->dimlayer_hbm);

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_dimlayer_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("dimlayer_hbm:%u\n", p_oplus_ofp_params->dimlayer_hbm);

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->dimlayer_hbm);
}

/* notify fp press for hidl */
int oplus_ofp_notify_fp_press(void *buf)
{
	unsigned int *fp_press = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params || !fp_press) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if ((*fp_press) == 1) {
		/* finger is pressed down and pressed icon layer is ready */
		p_oplus_ofp_params->fp_press = true;
	} else {
		p_oplus_ofp_params->fp_press = false;
	}
	OFP_INFO("receive uiready %d\n", p_oplus_ofp_params->fp_press);
	oplus_disp_trace_c("%d|oplus_ofp_fp_press|%d", g_commit_pid, p_oplus_ofp_params->fp_press);


	if (oplus_ofp_is_supported() && p_oplus_ofp_params->fp_press) {
		/* send aod off cmd in doze mode to speed up fingerprint unlocking */
		OFP_DEBUG("fp press is true\n");
		oplus_ofp_aod_off_set();
	}

	return 0;
}

/* notify fp press for sysfs */
ssize_t oplus_ofp_notify_fp_press_attr(struct kobject *obj, struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	unsigned int fp_press = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	if (kstrtouint(buf, 0, &fp_press)) {
		OFP_ERR("kstrtouint error!\n");
		return count;
	}

	if (fp_press == 1) {
		/* finger is pressed down and pressed icon layer is ready */
		p_oplus_ofp_params->fp_press = true;
	} else {
		p_oplus_ofp_params->fp_press = false;
	}
	OFP_INFO("receive uiready %d\n", p_oplus_ofp_params->fp_press);
	oplus_disp_trace_c("%d|oplus_ofp_fp_press|%d", g_commit_pid, p_oplus_ofp_params->fp_press);


	if (oplus_ofp_is_supported() && p_oplus_ofp_params->fp_press && !oplus_ofp_video_mode_aod_fod_is_enabled()) {
		/* send aod off cmd in doze mode to speed up fingerprint unlocking */
		OFP_DEBUG("fp press is true\n");
		oplus_ofp_aod_off_set();
	}

	return count;
}

int oplus_ofp_set_ultra_low_power_aod_after_doze_enable(struct drm_panel *panel, void *mtk_panel_ext,
		void *mtk_dsi, void *dcs_write_gce)
{
	struct mtk_panel_ext *ext = mtk_panel_ext;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!panel || !ext || !mtk_dsi || !dcs_write_gce || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (p_oplus_ofp_params->ultra_low_power_aod_mode) {
		if (ext && ext->funcs && ext->funcs->set_ultra_low_power_aod) {
			ext->funcs->set_ultra_low_power_aod(panel, mtk_dsi, dcs_write_gce, NULL, p_oplus_ofp_params->ultra_low_power_aod_mode);
			OFP_INFO("set_ultra_low_power_aod:%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);
		}
	}

	OFP_DEBUG("end\n");

	return 0;
}

int oplus_ofp_drm_set_ultra_low_power_aod(struct drm_crtc *crtc, unsigned int ultra_low_power_aod)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp, *comp;
	unsigned int crtc_id = drm_crtc_index(&mtk_crtc->base);
	struct cmdq_pkt *cmdq_handle;
	bool is_frame_mode;
	struct cmdq_client *client;
	int i, j;
	struct mtk_crtc_state *crtc_state;

	if (!crtc || !mtk_crtc) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_drm_set_ultra_low_power_aod");

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	crtc_state = to_mtk_crtc_state(crtc->state);
	if ((!crtc_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) || (!oplus_ofp_get_aod_state())) {
		OFP_INFO("not in doze mode\n");
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		oplus_disp_trace_end("oplus_ofp_drm_set_ultra_low_power_aod");
		return 0;
	}

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!output_comp)) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		oplus_disp_trace_end("oplus_ofp_drm_set_ultra_low_power_aod");
		return -ENODEV;
	}

	client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	if (!mtk_crtc->enabled) {
		mtk_drm_crtc_wk_lock(crtc, 1, __func__, __LINE__);
		/* power on mtcmos */
		oplus_disp_trace_begin("power_on_mtcmos");
		mtk_drm_top_clk_prepare_enable(crtc->dev);

		/*APSRC control*/
		mtk_crtc_v_idle_apsrc_control(crtc, NULL, false, false, crtc_id, true);

		cmdq_mbox_enable(client->chan);
		if (mtk_crtc_with_event_loop(crtc) &&
				(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_event_loop(crtc);

		if (mtk_crtc_with_trigger_loop(crtc))
			mtk_crtc_start_trig_loop(crtc);

		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_ENABLE, NULL);

		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_dump_analysis(comp);

		OFP_INFO("power on mtcmos\n");
		oplus_disp_trace_end("power_on_mtcmos");
	}

	/* send LCM CMD */
	oplus_disp_trace_begin("prepare_to_send_ultra_low_power_aod_cmd");
	OFP_INFO("prepare to send ultra low power aod cmd\n");
	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

	if (is_frame_mode || mtk_crtc->gce_obj.client[CLIENT_DSI_CFG] == NULL)
		cmdq_handle =
			cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	else
		cmdq_handle =
			cmdq_pkt_create(
			mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -EINVAL;
	}

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, 0);

	/* Record Vblank start timestamp */
	mtk_vblank_config_rec_start(mtk_crtc, cmdq_handle, SET_BL);
	oplus_disp_trace_end("prepare_to_send_ultra_low_power_aod_cmd");

	/* set low power aod */
	oplus_disp_trace_begin("DSI_CMD_ULTRA_LOW_POWER_AOD");
	if (output_comp->funcs && output_comp->funcs->io_cmd)
		output_comp->funcs->io_cmd(output_comp,
			cmdq_handle, DSI_CMD_ULTRA_LOW_POWER_AOD, &ultra_low_power_aod);
	OFP_INFO("ultra low power aod %d\n", ultra_low_power_aod);
	oplus_disp_trace_end("DSI_CMD_ULTRA_LOW_POWER_AOD");

	oplus_disp_trace_begin("flush_ultra_low_power_aod_cmd");
	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	/* Record Vblank end timestamp and calculate duration */
	mtk_vblank_config_rec_end_cal(mtk_crtc, cmdq_handle, SET_BL);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
	OFP_INFO("flush ultra low power aod cmd\n");
	oplus_disp_trace_end("flush_ultra_low_power_aod_cmd");

	if (!mtk_crtc->enabled) {
		oplus_disp_trace_begin("power_off_mtcmos");
		if (mtk_crtc_with_trigger_loop(crtc))
			mtk_crtc_stop_trig_loop(crtc);

		if (mtk_crtc_with_event_loop(crtc) &&
				(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_stop_event_loop(crtc);

		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_DISABLE, NULL);

		cmdq_mbox_disable(client->chan);
		DDPFENCE("%s:%d power_state = false\n", __func__, __LINE__);
		mtk_drm_top_clk_disable_unprepare(crtc->dev);
		mtk_drm_crtc_wk_lock(crtc, 0, __func__, __LINE__);
		OFP_INFO("power off mtcmos\n");
		oplus_disp_trace_end("power_off_mtcmos");
	}

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	oplus_disp_trace_end("oplus_ofp_drm_set_ultra_low_power_aod");

	return 0;
}

int oplus_ofp_drm_set_hbm(struct drm_crtc *crtc, unsigned int hbm_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!crtc || !mtk_crtc || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	if (!(mtk_crtc->enabled)) {
		OFP_ERR("should not set hbm if mtk crtc is not enabled\n");
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -EFAULT;
	} else if (mtk_crtc->ddp_mode == DDP_NO_USE) {
		OFP_ERR("skip ddp_mode: NO_USE\n");
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -EFAULT;
	}

	oplus_disp_trace_begin("mtk_drm_send_lcm_cmd_prepare");
	OFP_INFO("prepare to send hbm cmd\n");
	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);
	oplus_disp_trace_end("mtk_drm_send_lcm_cmd_prepare");

	if (oplus_ofp_local_hbm_is_enabled() && (p_oplus_ofp_params->fp_local_hbm_calibration)) {
	/* send lhbm pressed icon cmd */
		oplus_disp_trace_begin("OPLUS_OFP_SET_LHBM_PRESSED_ICON");
		comp->funcs->io_cmd(comp, cmdq_handle, OPLUS_OFP_SET_LHBM_PRESSED_ICON, &hbm_mode);
		OFP_INFO("OPLUS_OFP_SET_LHBM_PRESSED_ICON %d\n", hbm_mode);
		oplus_disp_trace_end("OPLUS_OFP_SET_LHBM_PRESSED_ICON");
	} else {
	/* set hbm */
		oplus_disp_trace_begin("LCM_HBM");
		comp->funcs->io_cmd(comp, cmdq_handle, LCM_HBM, &hbm_mode);
		OFP_INFO("LCM_HBM\n");
		oplus_disp_trace_end("LCM_HBM");
	}

	oplus_disp_trace_begin("mtk_drm_send_lcm_cmd_flush");
	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);
	OFP_INFO("mtk_drm_send_lcm_cmd_flush end\n");
	oplus_ofp_set_hbm_state(hbm_mode);
	oplus_disp_trace_end("mtk_drm_send_lcm_cmd_flush");

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	 return 0;
}

int oplus_ofp_get_hbm(void *buf)
{
	unsigned int *hbm_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	(*hbm_mode) = p_oplus_ofp_params->hbm_mode;
	OFP_INFO("hbm_mode = %d\n", *hbm_mode);

	return 0;
}

int oplus_ofp_set_hbm(void *buf)
{
	unsigned int *hbm_mode = buf;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!ddev || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		OFP_ERR("find crtc fail\n");
		return -EINVAL;
	}

	OFP_INFO("%d to %d\n", p_oplus_ofp_params->hbm_mode, *hbm_mode);
	oplus_ofp_drm_set_hbm(crtc, *hbm_mode);
	p_oplus_ofp_params->hbm_mode = (*hbm_mode);
	oplus_disp_trace_c("%d|oplus_ofp_hbm_mode|%d", g_commit_pid, p_oplus_ofp_params->hbm_mode);

	return 0;
}

ssize_t oplus_ofp_get_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("hbm_mode = %d\n", p_oplus_ofp_params->hbm_mode);

	return sprintf(buf, "%d\n", p_oplus_ofp_params->hbm_mode);
}

ssize_t oplus_ofp_set_hbm_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int hbm_mode = 0;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	if (!ddev || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	if (kstrtouint(buf, 10, &hbm_mode)) {
		OFP_ERR("kstrtouint error!\n");
		return count;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		OFP_ERR("find crtc fail\n");
		return -count;
	}

	OFP_INFO("%d to %d\n", p_oplus_ofp_params->hbm_mode, hbm_mode);
	oplus_ofp_drm_set_hbm(crtc, hbm_mode);
	p_oplus_ofp_params->hbm_mode = hbm_mode;
	oplus_disp_trace_c("%d|oplus_ofp_hbm_mode|%d", g_commit_pid, p_oplus_ofp_params->hbm_mode);

	return count;
}

/* aod part */
int oplus_ofp_set_aod_light_mode(void *buf)
{
	int rc = 0;
	unsigned int *aod_light_mode = buf;
	static unsigned int last_aod_light_mode = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	last_aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	p_oplus_ofp_params->aod_light_mode = (*aod_light_mode);
	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
	oplus_disp_trace_c("%d|oplus_ofp_aod_light_mode|%d", g_commit_pid, p_oplus_ofp_params->aod_light_mode);

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("aod is not supported\n");
		return 0;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_ERR("not in aod mode, should not set aod_light_mode\n");
		return 0;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore aod light mode setting in hbm state\n");
		return 0;
	}

	if (last_aod_light_mode != p_oplus_ofp_params->aod_light_mode) {
		mtkfb_set_aod_backlight_level(p_oplus_ofp_params->aod_light_mode);
	}

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_get_aod_light_mode(void *buf)
{
	unsigned int *aod_light_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	*aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	OFP_INFO("aod_light_mode:%u\n", *aod_light_mode);

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int aod_light_mode = 0;
	static unsigned int last_aod_light_mode = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%u", &aod_light_mode);

	last_aod_light_mode = p_oplus_ofp_params->aod_light_mode;
	p_oplus_ofp_params->aod_light_mode = aod_light_mode;
	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);
	oplus_disp_trace_c("%d|oplus_ofp_aod_light_mode|%d", g_commit_pid, p_oplus_ofp_params->aod_light_mode);

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("aod is not supported\n");
		return count;
	} else if (!oplus_ofp_get_aod_state()) {
		OFP_ERR("not in aod mode, should not set aod_light_mode\n");
		return count;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore aod light mode setting in hbm state\n");
		return count;
	}

	if (last_aod_light_mode != p_oplus_ofp_params->aod_light_mode) {
		mtkfb_set_aod_backlight_level(p_oplus_ofp_params->aod_light_mode);
	}

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_aod_light_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("aod_light_mode:%u\n", p_oplus_ofp_params->aod_light_mode);

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->aod_light_mode);
}

/* ultra_low_power_aod_mode */
int oplus_ofp_set_ultra_low_power_aod_mode(void *buf)
{
	int rc = 0;
	struct drm_crtc *crtc;
	struct mtk_crtc_state *crtc_state;
	struct drm_device *ddev = get_drm_device();
	unsigned int *ultra_low_power_aod_mode = buf;
	static unsigned int last_ultra_low_power_aod_mode = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !ddev || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		OFP_ERR("find crtc fail\n");
		return -EINVAL;
	}

	last_ultra_low_power_aod_mode = p_oplus_ofp_params->ultra_low_power_aod_mode;
	p_oplus_ofp_params->ultra_low_power_aod_mode = (*ultra_low_power_aod_mode);
	OFP_INFO("ultra_low_power_aod_mode:%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);
	oplus_disp_trace_c("%d|oplus_ofp_set_ultra_low_power_aod_mode|%d", g_commit_pid, p_oplus_ofp_params->ultra_low_power_aod_mode);

	crtc_state = to_mtk_crtc_state(crtc->state);
	if (!oplus_ofp_is_supported()) {
		OFP_INFO("aod is not supported\n");
		return 0;
	} else if (!oplus_ofp_get_aod_state() || !crtc_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
		OFP_INFO("not in aod mode, not set ultra_low_power_aod_mode\n");
		return 0;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore aod light mode setting in hbm state\n");
		return 0;
	}

	if (last_ultra_low_power_aod_mode != p_oplus_ofp_params->ultra_low_power_aod_mode) {
		rc = oplus_ofp_drm_set_ultra_low_power_aod(crtc, p_oplus_ofp_params->ultra_low_power_aod_mode);
		if (rc) {
			OFP_ERR("Failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_ON cmds, rc=%d\n", rc);
		}
	}

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_get_ultra_low_power_aod_mode(void *buf)
{
	unsigned int *ultra_low_power_aod_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	*ultra_low_power_aod_mode = p_oplus_ofp_params->ultra_low_power_aod_mode;
	OFP_INFO("ultra_low_power_aod_mode:%u\n", *ultra_low_power_aod_mode);

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_ultra_low_power_aod_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	struct drm_crtc *crtc;
	struct mtk_crtc_state *crtc_state;
	struct drm_device *ddev = get_drm_device();
	unsigned int ultra_low_power_aod_mode = 0;
	static unsigned int last_ultra_low_power_aod_mode = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !ddev || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		OFP_ERR("find crtc fail\n");
		return -EINVAL;
	}

	sscanf(buf, "%u", &ultra_low_power_aod_mode);

	last_ultra_low_power_aod_mode = p_oplus_ofp_params->ultra_low_power_aod_mode;
	p_oplus_ofp_params->ultra_low_power_aod_mode = ultra_low_power_aod_mode;
	OFP_INFO("ultra_low_power_aod_mode:%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);
	oplus_disp_trace_c("%d|oplus_ofp_ultra_low_power_aod_mode|%d", g_commit_pid, p_oplus_ofp_params->ultra_low_power_aod_mode);

	crtc_state = to_mtk_crtc_state(crtc->state);
	if (!oplus_ofp_is_supported()) {
		OFP_INFO("aod is not supported\n");
		return 0;
	} else if (!oplus_ofp_get_aod_state() || !crtc_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
		OFP_INFO("not in aod mode, not set ultra_low_power_aod_mode\n");
		return 0;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore aod light mode setting in hbm state\n");
		return 0;
	}

	if (last_ultra_low_power_aod_mode != p_oplus_ofp_params->ultra_low_power_aod_mode) {
		rc = oplus_ofp_drm_set_ultra_low_power_aod(crtc, p_oplus_ofp_params->ultra_low_power_aod_mode);
		if (rc) {
			OFP_ERR("Failed to send DSI_CMD_ULTRA_LOW_POWER_AOD_ON cmds, rc=%d\n", rc);
		}
	}

	OFP_DEBUG("end\n");
	return count;
}

ssize_t oplus_ofp_get_ultra_low_power_aod_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("ultra_low_power_aod_mode:%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->ultra_low_power_aod_mode);
}

int oplus_ofp_set_fake_aod(void *buf)
{
	unsigned int *fake_aod_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_set_fake_aod");

	p_oplus_ofp_params->fake_aod_mode = *fake_aod_mode;
	OFP_INFO("fake_aod_mode:%u\n", p_oplus_ofp_params->fake_aod_mode);
	oplus_disp_trace_c("%d|oplus_ofp_fake_aod_mode|%d", g_commit_pid, p_oplus_ofp_params->fake_aod_mode);

	oplus_disp_trace_end("oplus_ofp_set_fake_aod");

	OFP_DEBUG("end\n");

	return 0;
}

int oplus_ofp_get_fake_aod(void *buf)
{
	unsigned int *fake_aod_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_get_fake_aod");

	*fake_aod_mode = p_oplus_ofp_params->fake_aod_mode;
	OFP_INFO("fake_aod_mode:%u\n", *fake_aod_mode);

	oplus_disp_trace_end("oplus_ofp_get_fake_aod");

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_fake_aod_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int fake_aod_mode = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	oplus_disp_trace_begin("oplus_ofp_set_fake_aod_attr");

	sscanf(buf, "%u", &fake_aod_mode);

	p_oplus_ofp_params->fake_aod_mode = fake_aod_mode;
	OFP_INFO("fake_aod_mode:%u\n", p_oplus_ofp_params->fake_aod_mode);
	oplus_disp_trace_c("%d|oplus_ofp_fake_aod_mode|%d", g_commit_pid, p_oplus_ofp_params->fake_aod_mode);

	oplus_disp_trace_end("oplus_ofp_set_fake_aod_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_fake_aod_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_get_fake_aod_attr");

	OFP_INFO("fake_aod_mode:%u\n", p_oplus_ofp_params->fake_aod_mode);

	oplus_disp_trace_end("oplus_ofp_get_fake_aod_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->fake_aod_mode);
}

int oplus_ofp_set_longrui_aod_mode(void *buf)
{
	int rc = 0;
	unsigned int *longrui_aod_mode = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}
	p_oplus_ofp_params->longrui_aod_mode = (*longrui_aod_mode);
	OFP_INFO("longrui_aod_mode:0x%x\n", p_oplus_ofp_params->longrui_aod_mode);
	oplus_disp_trace_c("%d|oplus_ofp_longrui_aod_mode|%d", g_commit_pid, p_oplus_ofp_params->longrui_aod_mode);

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not supported\n");
		return 0;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore longrui aod mode setting in hbm state\n");
		return 0;
	}

	oplus_disp_trace_begin("oplus_ofp_set_longrui_aod_mode");

	if ((p_oplus_ofp_params->longrui_aod_config & OPLUS_OFP_A_MIRROR_TO_THE_END_AOD_CONFIG)
			&& (p_oplus_ofp_params->longrui_aod_mode & OPLUS_OFP_A_MIRROR_TO_THE_END_AOD_MODE)
			&& !(p_oplus_ofp_params->longrui_aod_mode & OPLUS_OFP_AOD_ON)) {
		if (oplus_ofp_get_aod_state() && !p_oplus_ofp_params->doze_active) {
			rc = oplus_ofp_crtc_aod_off_set();
			if (rc) {
				OFP_ERR("failed to set aod off, rc=%d\n", rc);
			}
		}
	}

	oplus_disp_trace_end("oplus_ofp_set_longrui_aod_mode");

	OFP_DEBUG("end\n");

	return rc;
}

int oplus_ofp_get_longrui_aod_config(void *buf)
{
	unsigned int *longrui_aod_config = buf;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_get_longrui_aod_config");

	*longrui_aod_config = p_oplus_ofp_params->longrui_aod_config;
	OFP_INFO("longrui_aod_config:0x%x\n", *longrui_aod_config);

	oplus_disp_trace_end("oplus_ofp_get_longrui_aod_config");

	OFP_DEBUG("end\n");

	return 0;
}

ssize_t oplus_ofp_set_longrui_aod_mode_attr(struct kobject *obj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	unsigned int longrui_aod_mode = 0;
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return count;
	}

	sscanf(buf, "%u", &longrui_aod_mode);
	p_oplus_ofp_params->longrui_aod_mode = longrui_aod_mode;
	OFP_INFO("longrui_aod_mode:0x%x\n", p_oplus_ofp_params->longrui_aod_mode);
	oplus_disp_trace_c("%d|oplus_ofp_longrui_aod_mode|%d", g_commit_pid, p_oplus_ofp_params->longrui_aod_mode);

	if (!oplus_ofp_is_supported()) {
		OFP_DEBUG("ofp is not supported\n");
		return count;
	} else if (oplus_ofp_get_hbm_state()) {
		OFP_INFO("ignore longrui aod mode setting in hbm state\n");
		return count;
	}

	oplus_disp_trace_begin("oplus_ofp_set_longrui_aod_mode_attr");

	if ((p_oplus_ofp_params->longrui_aod_config & OPLUS_OFP_A_MIRROR_TO_THE_END_AOD_CONFIG)
			&& (p_oplus_ofp_params->longrui_aod_mode & OPLUS_OFP_A_MIRROR_TO_THE_END_AOD_MODE)
			&& !(p_oplus_ofp_params->longrui_aod_mode & OPLUS_OFP_AOD_ON)) {
		if (oplus_ofp_get_aod_state() && !p_oplus_ofp_params->doze_active) {
			rc = oplus_ofp_crtc_aod_off_set();
			if (rc) {
				OFP_ERR("failed to set aod off, rc=%d\n", rc);
			}
		}
	}

	oplus_disp_trace_end("oplus_ofp_set_longrui_aod_mode_attr");

	OFP_DEBUG("end\n");

	return count;
}

ssize_t oplus_ofp_get_longrui_aod_config_attr(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	struct oplus_ofp_params *p_oplus_ofp_params = oplus_ofp_get_params();

	OFP_DEBUG("start\n");

	if (!buf || !p_oplus_ofp_params) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	oplus_disp_trace_begin("oplus_ofp_get_longrui_aod_config_attr");

	OFP_INFO("longrui_aod_config:0x%x\n", p_oplus_ofp_params->longrui_aod_config);

	oplus_disp_trace_end("oplus_ofp_get_longrui_aod_config_attr");

	OFP_DEBUG("end\n");

	return sprintf(buf, "%u\n", p_oplus_ofp_params->longrui_aod_config);
}

#define REGFLAG_CMD				0xFFFA
#define OPLUS_DSI_CMD_PRINT_BUF_SIZE 2048

bool g_gamma_regs_read_done = false;
EXPORT_SYMBOL(g_gamma_regs_read_done);

bool g_elvss_regs_read_done = false;
EXPORT_SYMBOL(g_elvss_regs_read_done);

char regs1[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
EXPORT_SYMBOL(regs1);
char regs2[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
EXPORT_SYMBOL(regs2);
char regs3[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
EXPORT_SYMBOL(regs3);
static int oplus_panel_ac178_gamma_compensation(void *dsi)
{
	u32 retry_count = 0;
	u32 index = 0;
	u32 cnt = 0;
	char print_buf[OPLUS_DSI_CMD_PRINT_BUF_SIZE] = {0};
	char regs1_last[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
	char regs2_last[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
	char regs3_last[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
	const char reg_base[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
	char switch_page1[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02};
	char switch_page2[2] = {0xBF, 0x08};
	char switch_page3[2] = {0x6F, 0x0C};

	while (!g_gamma_regs_read_done && retry_count < AC178_GAMMA_COMPENSATION_READ_RETRY_MAX) {
		pr_info("read gamma compensation regs, retry_count=%d\n", retry_count);
		memset(regs1, 0, AC178_GAMMA_COMPENSATION_READ_LENGTH);
		memset(regs2, 0, AC178_GAMMA_COMPENSATION_READ_LENGTH);
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);

		oplus_ddic_dsi_send_cmd(6, switch_page1);
		oplus_ddic_dsi_send_cmd(2, switch_page2);
		oplus_ddic_dsi_send_cmd(2, switch_page3);

		oplus_mtk_read_ddic_v2(AC178_GAMMA_COMPENSATION_READ_REG1, AC178_GAMMA_COMPENSATION_READ_LENGTH, regs1);
		cnt = 0;
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);
		for (index = 0; index < AC178_GAMMA_COMPENSATION_READ_LENGTH; index++) {
			cnt += snprintf(print_buf + cnt, 4, "%02X ", regs1[index]);
		}
		pr_info("read regs1 len=%d, buf=[%s]\n", AC178_GAMMA_COMPENSATION_READ_LENGTH, print_buf);

		oplus_ddic_dsi_send_cmd(6, switch_page1);
		oplus_ddic_dsi_send_cmd(2, switch_page2);
		oplus_ddic_dsi_send_cmd(2, switch_page3);

		oplus_mtk_read_ddic_v2(AC178_GAMMA_COMPENSATION_READ_REG2, AC178_GAMMA_COMPENSATION_READ_LENGTH, regs2);
		cnt = 0;
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);
		for (index = 0; index < AC178_GAMMA_COMPENSATION_READ_LENGTH; index++) {
			cnt += snprintf(print_buf + cnt, 4, "%02X ", regs2[index]);
		}
		pr_info("read regs2 len=%d, buf=[%s]\n", AC178_GAMMA_COMPENSATION_READ_LENGTH, print_buf);

		oplus_ddic_dsi_send_cmd(6, switch_page1);
		oplus_ddic_dsi_send_cmd(2, switch_page2);
		oplus_ddic_dsi_send_cmd(2, switch_page3);

		oplus_mtk_read_ddic_v2(AC178_GAMMA_COMPENSATION_READ_REG3, AC178_GAMMA_COMPENSATION_READ_LENGTH, regs3);
		cnt = 0;
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);
		for (index = 0; index < AC178_GAMMA_COMPENSATION_READ_LENGTH; index++) {
			cnt += snprintf(print_buf + cnt, 4, "%02X ", regs3[index]);
		}
		pr_info("read regs3 len=%d, buf=[%s]\n", AC178_GAMMA_COMPENSATION_READ_LENGTH, print_buf);
		if (!strncmp(regs1, reg_base, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
				!strncmp(regs2, reg_base, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
				!strncmp(regs3, reg_base, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
				strncmp(regs1, regs1_last, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
				strncmp(regs2, regs2_last, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
				strncmp(regs3, regs3_last, AC178_GAMMA_COMPENSATION_READ_LENGTH)) {
			pr_info("gamma compensation regs is invalid, retry\n");
			memcpy(regs1_last, regs1, AC178_GAMMA_COMPENSATION_READ_LENGTH);
			memcpy(regs2_last, regs2, AC178_GAMMA_COMPENSATION_READ_LENGTH);
			memcpy(regs3_last, regs3, AC178_GAMMA_COMPENSATION_READ_LENGTH);
			retry_count++;
			continue;
		}
		g_gamma_regs_read_done = true;

		break;
	}

	if (!g_gamma_regs_read_done) {
		return -EFAULT;
	}

	return 0;
}

static int oplus_panel_ae174_gamma_compensation(void *dsi)
{
	u32 retry_count = 0;
	u32 index = 0;
	u32 cnt = 0;
	char print_buf[OPLUS_DSI_CMD_PRINT_BUF_SIZE] = {0};
	char regs1_last[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
	char regs2_last[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
	char regs3_last[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
	const char reg_base[AC178_GAMMA_COMPENSATION_READ_LENGTH] = {0};
	char switch_page1[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02};
	char switch_page2[2] = {0xBF, 0x0B};
	char switch_page3[2] = {0x6F, 0x0C};

	while (!g_gamma_regs_read_done && retry_count < AC178_GAMMA_COMPENSATION_READ_RETRY_MAX) {
		pr_info("read gamma compensation regs, retry_count=%d\n", retry_count);
		memset(regs1, 0, AC178_GAMMA_COMPENSATION_READ_LENGTH);
		memset(regs2, 0, AC178_GAMMA_COMPENSATION_READ_LENGTH);
		memset(regs3, 0, AC178_GAMMA_COMPENSATION_READ_LENGTH);
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);

		oplus_ddic_dsi_send_cmd(6, switch_page1);
		oplus_ddic_dsi_send_cmd(2, switch_page2);
		oplus_ddic_dsi_send_cmd(2, switch_page3);

		oplus_mtk_read_ddic_v2(AC178_GAMMA_COMPENSATION_READ_REG1, AC178_GAMMA_COMPENSATION_READ_LENGTH, regs1);
		cnt = 0;
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);
		for (index = 0; index < AC178_GAMMA_COMPENSATION_READ_LENGTH; index++) {
			cnt += snprintf(print_buf + cnt, 4, "%02X ", regs1[index]);
		}
		pr_info("read regs1 len=%d, buf=[%s]\n", AC178_GAMMA_COMPENSATION_READ_LENGTH, print_buf);

		oplus_ddic_dsi_send_cmd(2, switch_page3);

		oplus_mtk_read_ddic_v2(AC178_GAMMA_COMPENSATION_READ_REG2, AC178_GAMMA_COMPENSATION_READ_LENGTH, regs2);
		cnt = 0;
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);
		for (index = 0; index < AC178_GAMMA_COMPENSATION_READ_LENGTH; index++) {
			cnt += snprintf(print_buf + cnt, 4, "%02X ", regs2[index]);
		}
		pr_info("read regs2 len=%d, buf=[%s]\n", AC178_GAMMA_COMPENSATION_READ_LENGTH, print_buf);

		oplus_ddic_dsi_send_cmd(2, switch_page3);

		oplus_mtk_read_ddic_v2(AC178_GAMMA_COMPENSATION_READ_REG3, AC178_GAMMA_COMPENSATION_READ_LENGTH, regs3);
		cnt = 0;
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);
		for (index = 0; index < AC178_GAMMA_COMPENSATION_READ_LENGTH; index++) {
			cnt += snprintf(print_buf + cnt, 4, "%02X ", regs3[index]);
		}
		pr_info("read regs3 len=%d, buf=[%s]\n", AC178_GAMMA_COMPENSATION_READ_LENGTH, print_buf);
		if (!strncmp(regs1, reg_base, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
			!strncmp(regs2, reg_base, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
			!strncmp(regs3, reg_base, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
			strncmp(regs1, regs1_last, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
			strncmp(regs2, regs2_last, AC178_GAMMA_COMPENSATION_READ_LENGTH) ||
			strncmp(regs3, regs3_last, AC178_GAMMA_COMPENSATION_READ_LENGTH)) {
			pr_info("gamma compensation regs is invalid, retry\n");
			memcpy(regs1_last, regs1, AC178_GAMMA_COMPENSATION_READ_LENGTH);
			memcpy(regs2_last, regs2, AC178_GAMMA_COMPENSATION_READ_LENGTH);
			memcpy(regs3_last, regs3, AC178_GAMMA_COMPENSATION_READ_LENGTH);
			retry_count++;
			continue;
		}
		g_gamma_regs_read_done = true;

		break;
	}

	if (!g_gamma_regs_read_done) {
		return -EFAULT;
	}

	return 0;
}

char regs4[AE174_ELVSS_READ_LENGTH] = {0};
EXPORT_SYMBOL(regs4);
static int oplus_panel_ae174_elvss_read(void *dsi)
{
	u32 retry_count = 0;
	u32 index = 0;
	u32 cnt = 0;
	char print_buf[OPLUS_DSI_CMD_PRINT_BUF_SIZE] = {0};
	char regs_last[AE174_ELVSS_READ_LENGTH] = {0};
	const char reg_base[AE174_ELVSS_READ_LENGTH] = {0};
	char switch_page1[6] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
	char switch_page2[2] = {0x6F, 0x08};

	while (!g_elvss_regs_read_done && retry_count < AE174_ELVSS_READ_RETRY_MAX) {
		pr_info("read elvss regs4, retry_count=%d\n", retry_count);
		memset(regs4, 0, AE174_ELVSS_READ_LENGTH);
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);

		oplus_ddic_dsi_send_cmd(6, switch_page1);
		oplus_ddic_dsi_send_cmd(2, switch_page2);

		oplus_mtk_read_ddic_v2(AE174_ELVSS_READ_REG, AE174_ELVSS_READ_LENGTH, regs4);
		cnt = 0;
		memset(print_buf, 0, OPLUS_DSI_CMD_PRINT_BUF_SIZE);
		for (index = 0; index < AE174_ELVSS_READ_LENGTH; index++) {
			cnt += snprintf(print_buf + cnt, 4, "%02X ", regs4[index]);
		}
		pr_info("read elvss regs4 len=%d, buf=[%s]\n", AE174_ELVSS_READ_LENGTH, print_buf);

		if (!strncmp(regs4, reg_base, AE174_ELVSS_READ_LENGTH)) {
			pr_info("elvss regs is invalid, retry\n");
			memcpy(regs_last, regs4, AE174_ELVSS_READ_LENGTH);
			retry_count++;
			continue;
		}
		g_elvss_regs_read_done = true;

		break;
	}

	if (!g_elvss_regs_read_done) {
		return -EFAULT;
	}

	return 0;
}

int oplus_panel_ext_init(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_ddp_comp *comp;
	struct mtk_dsi *dsi;
	char *panel_name;
	int rc = 0;

	if (!crtc) {
		pr_err("find crtc fail\n");
		return -EFAULT;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (mtk_crtc == NULL || crtc->state == NULL) {
		pr_err("mtk_crtc or crtc->state is NULL\n");
		return -EFAULT;
	}
	if (!(mtk_crtc->enabled)) {
		pr_err("crtc%d disable skip\n", drm_crtc_index(&mtk_crtc->base));
		return -EFAULT;
	} else if (mtk_crtc->ddp_mode == DDP_NO_USE) {
		pr_err("skip ddp_mode: NO_USE\n");
		return -EFAULT;
	}
	if (!mtk_drm_lcm_is_connect(mtk_crtc)) {
		pr_err("lcm is not connect\n");
		return -EFAULT;
	}

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!comp)) {
		pr_err("invalid output comp\n");
		return -EFAULT;
	}
	dsi = container_of(comp, struct mtk_dsi, ddp_comp);
	if (!dsi) {
		pr_err("dsi is NULL\n");
		return -EFAULT;
	}

	mtk_ddp_comp_io_cmd(comp, NULL, GET_PANEL_NAME, &panel_name);

	if((!strcmp("ac222_p_7_a0014_dsi_cmd_panel", panel_name)
		|| !strcmp("ac230_p_7_a0014_dsi_cmd_t1", panel_name)
		|| !strcmp("ac264_p_7_a0014_dsi_cmd", panel_name)
		|| !strcmp("panel_ae016_p_7_a0014_dsi_cmd", panel_name)
		|| !strcmp("aa600_p_7_a0025_vdo_panel", panel_name)
		|| !strcmp("aa597_p_7_a0025_dsi_vdo", panel_name))
		&& !g_gamma_regs_read_done) {
		rc |= oplus_panel_ac178_gamma_compensation(dsi);
	}
	if((!strcmp("ae174_p_1_a0037_cmd_panel", panel_name))
	   && !g_gamma_regs_read_done) {
		rc |= oplus_panel_ae174_gamma_compensation(dsi);
	}
	if((!strcmp("ae174_p_1_a0037_cmd_panel", panel_name))
	   && !g_elvss_regs_read_done) {
		rc |= oplus_panel_ae174_elvss_read(dsi);
	}
	return rc;
}

MODULE_AUTHOR("Liuhe Zhong");
MODULE_DESCRIPTION("OPPO ofp device");
MODULE_LICENSE("GPL v2");
