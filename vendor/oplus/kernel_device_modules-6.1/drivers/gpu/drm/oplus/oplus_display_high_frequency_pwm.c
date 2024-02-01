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
#include "oplus_display_high_frequency_pwm.h"
#include <linux/thermal.h>
#include <linux/delay.h>
#include <../drm/drm_device.h>
#include <../drm/drm_crtc.h>
#include "mtk_panel_ext.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_drv.h"
#include "mtk_debug.h"
#include "mtk_dsi.h"
#include "oplus_display_trace.h"
#include <soc/oplus/system/oplus_project.h>
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
#include "oplus_display_temp_compensation.h"
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

/* -------------------- macro ---------------------------------- */
/* config bit setting */
#define REGFLAG_CMD									0xFFFA
#define DRM_PANEL_EVENT_PWM_TURBO  0x14

/* -------------------- parameters ----------------------------- */
static DEFINE_MUTEX(g_pwm_turbo_lock);
static DEFINE_MUTEX(g_pwm_turbo_onepulse_lock);

/* log level config */
unsigned int oplus_pwm_turbo_log = 0;
EXPORT_SYMBOL(oplus_pwm_turbo_log);

extern long long oplus_last_te_time;
extern unsigned int oplus_display_brightness;
unsigned int last_backlight = 0;
EXPORT_SYMBOL(last_backlight);
bool pulse_flg = false;
EXPORT_SYMBOL(pulse_flg);
unsigned int pwm_backlight_record;

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* -------------------- extern ---------------------------------- */
/* extern params */
extern unsigned int lcm_id1;
extern unsigned int lcm_id2;
extern unsigned int oplus_display_brightness;

/* extern functions */
extern struct drm_device *get_drm_device(void);
extern void lcdinfo_notify(unsigned long val, void *v);
extern void oplus_ddic_dsi_send_cmd(unsigned int cmd_num, char val[20]);
extern void oplus_mtk_read_ddic_v2(u8 ddic_reg, int ret_num, char ret_val[10]);
extern void mtk_crtc_cmdq_timeout_cb(struct cmdq_cb_data data);
extern void mtk_drm_send_lcm_cmd_prepare_wait_for_vsync(struct drm_crtc *crtc, struct cmdq_pkt **cmdq_handle);
extern void mtk_drm_send_lcm_cmd_prepare(struct drm_crtc *crtc, struct cmdq_pkt **cmdq_handle);

/*  oplus hpwm functions  */
static struct oplus_pwm_turbo_params g_oplus_pwm_turbo_params = {0};

struct oplus_pwm_turbo_params *oplus_pwm_turbo_get_params(void)
{
	return &g_oplus_pwm_turbo_params;
}
EXPORT_SYMBOL(oplus_pwm_turbo_get_params);

inline bool oplus_panel_pwm_turbo_is_enabled(void)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return false;
	}

	return (bool)(pwm_params->pwm_turbo_support && pwm_params->pwm_turbo_enabled);
}
EXPORT_SYMBOL(oplus_panel_pwm_turbo_is_enabled);

inline bool oplus_panel_pwm_turbo_switch_state(void)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return false;
	}

	return (bool)(pwm_params->pwm_turbo_support   &&
			pwm_params->oplus_pwm_switch_state);
}
EXPORT_SYMBOL(oplus_panel_pwm_turbo_switch_state);

bool pwm_turbo_support(void)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return false;
	}

	return pwm_params->pwm_turbo_support;
}
EXPORT_SYMBOL(pwm_turbo_support);

inline bool get_pwm_turbo_states(void)
{
	bool states = false;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return false;
	}

	states = pwm_params->pwm_turbo_enabled;
	return states;
}
EXPORT_SYMBOL(get_pwm_turbo_states);

int get_pwm_turbo_plus_bl(void)
{
	int bl = 0;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return 0;
	}

	bl = pwm_params->pwm_bl_threshold;
	return bl;
}
EXPORT_SYMBOL(get_pwm_turbo_plus_bl);

inline void set_pwm_turbo_switch_state(enum PWM_SWITCH_STATE state)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return;
	}

	pwm_params->oplus_pwm_switch_state = state;
	pr_info("pwm_turbo lcdinfo_notify 0x14, oplus_pwm_switch_state:%d\n", state);
	lcdinfo_notify(DRM_PANEL_EVENT_PWM_TURBO, &pwm_params->oplus_pwm_switch_state);
}
EXPORT_SYMBOL(set_pwm_turbo_switch_state);

int get_pwm_turbo_fps_mode(void)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return 0;
	}

	return pwm_params->pwm_fps_mode;
}
EXPORT_SYMBOL(get_pwm_turbo_fps_mode);

void set_pwm_turbo_power_on(bool en)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return;
	}

	pwm_params->pwm_power_on = en;
}
EXPORT_SYMBOL(set_pwm_turbo_power_on);

int oplus_pwm_turbo_probe(struct device *dev)
{
	int rc = 0;
	u32 config = 0;
	struct oplus_pwm_turbo_params *pwm_params;

	if (IS_ERR_OR_NULL(dev)) {
		PWM_TURBO_ERR("Invalid params\n");
		return -EINVAL;
	}

	memset((void *)(&g_oplus_pwm_turbo_params), 0, sizeof(struct oplus_pwm_turbo_params));
	pwm_params = oplus_pwm_turbo_get_params();

	if (pwm_params == NULL) {
		DDPPR_ERR("%s: pwm_params NULL fail\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(dev->of_node, "oplus,pwm-turbo-support", &config);
	if (rc == 0) {
		pwm_params->config = config;
		pr_info("pwm_turbo config = %d, oplus,pwm-turbo-support = %d\n", config, pwm_params->config);
	} else {
		pwm_params->config = 0;
		pr_info("pwm_turbo oplus,pwm-turbo-support = %d\n", pwm_params->config);
	}

	if(pwm_params->config == OPLUS_PWM_TRUBO_CLOSE) {
		pr_info("pwm_turbo panel not support\n");
		pwm_params->pwm_turbo_support = false;
		return 0;
	} else {
		pwm_params->pwm_turbo_support = true;
		pr_info("pwm_turbo pwm_turbo_support: %d\n", pwm_params->pwm_turbo_support);
	}

	if (pwm_turbo_support()) {
		if (pwm_params->config == OPLUS_PWM_TRUBO_GLOBAL_OPEN_NO_SWITCH) {
			/* global pwm default switch is true */
			pwm_params->pwm_turbo_enabled = true;
			pr_info("pwm_turbo pwm_turbo_enabled: %d\n", pwm_params->pwm_turbo_enabled);
		}

		rc = of_property_read_u32(dev->of_node, "oplus,pwm-switch-backlight-threshold", &config);
		if (rc == 0) {
			pwm_params->pwm_bl_threshold = config;
			pr_info("pwm_turbo pwm_params plus dbv= %d\n", pwm_params->pwm_bl_threshold);
		} else {
			pwm_params->pwm_bl_threshold = 0;
			pr_info("pwm_turbo pwm_params dbv = %d\n", pwm_params->pwm_bl_threshold);
		}

		rc = of_property_read_u32(dev->of_node, "oplus,pwm-turbo-wait-te", &config);
		if (rc == 0) {
			pwm_params->pwm_wait_te = config;
			pr_info("pwm_turbo pwm_wait_te = %d\n",  pwm_params->pwm_wait_te);
		} else {
			pwm_params->pwm_wait_te = 0;
			pr_info("pwm_turbo pwm_wait_te config = %d\n", pwm_params->pwm_wait_te);
		}

		rc = of_property_read_u32(dev->of_node, "oplus,pwm-onepulse-support", &config);
		if (rc == 0) {
			pwm_params->pwm_onepulse_support = config;
			pr_info("pwm_turbo pwm_onepulse_support = %d\n",  pwm_params->pwm_onepulse_support);
		} else {
			pwm_params->pwm_onepulse_support = false;
			pr_info("pwm_turbo pwm_onepulse_support config = %d\n", pwm_params->pwm_onepulse_support);
		}

		rc = of_property_read_u32(dev->of_node, "oplus,pwm-onepulse-enabled", &config);
		if (rc == 0) {
			pwm_params->pwm_onepulse_enabled = config;
			pr_info("pwm_turbo pwm_onepulse_enable = %d\n",  pwm_params->pwm_onepulse_enabled);
		} else {
			pwm_params->pwm_onepulse_enabled = false;
			pr_info("pwm_turbo pwm_onepulse_support config = %d\n", pwm_params->pwm_onepulse_enabled);
		}

		pwm_params->pwm_power_on = true;
		pwm_params->pwm_fps_mode = 120;
		pwm_params->pwm_switch_support = false;
		pwm_params->oplus_pwm_switch_state = PWM_SWITCH_DC_STATE;
		pwm_params->oplus_pwm_threshold = 0;
	}

	pr_info("pwm_turbo oplus_pwm_turbo_probe successful\n");
	return 0;
}

EXPORT_SYMBOL(oplus_pwm_turbo_probe);

void oplus_display_panel_wait_te(int cnt)
{
	struct drm_device *ddev = get_drm_device();
	struct drm_crtc *crtc = NULL;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct cmdq_pkt *cmdq_handle = NULL;
	int count = cnt;

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		pr_info("%s: pwm_turbo find crtc fail, %d\n", __func__, __LINE__);
		return;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->enabled) {
		pr_info("%s: pwm_turbo find mtk_crtc fail, %d\n", __func__, __LINE__);
		return;
	}

	mtk_drm_send_lcm_cmd_prepare_wait_for_vsync(crtc, &cmdq_handle);

	while (count > 0) {
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		pr_info("%s: pwm_turbo cmdq_pkt_clear_event %d\n", __func__, __LINE__);
		if (mtk_drm_lcm_is_connect(mtk_crtc)) {
			pr_info("%s: pwm_turbo wait 1 te %d\n", __func__, __LINE__);
			cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		}
		count--;
	}
	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);
}
EXPORT_SYMBOL(oplus_display_panel_wait_te);

int oplus_display_panel_set_pwm_turbo_switch_onepulse(struct drm_crtc *crtc, unsigned int en)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	struct mtk_panel_ext *ext = mtk_crtc->panel_ext;
	struct cmdq_pkt *cmdq_handle;
	struct mtk_crtc_state *state =
	    to_mtk_crtc_state(mtk_crtc->base.state);
	unsigned int src_mode =
	    state->prop_val[CRTC_PROP_DISP_MODE_IDX];
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	int plus_bl = get_pwm_turbo_plus_bl();

	if (ext == NULL) {
		PWM_TURBO_ERR("%s %d mtk_crtc->panel_ext is NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	pr_info("%s, en=%d, src_mode=%d, ext->params->dyn_fps.vact_timing_fps=%d\n",
		__func__, en, src_mode, ext->params->dyn_fps.vact_timing_fps);
	mutex_lock(&g_pwm_turbo_lock);

	if (!mtk_crtc->enabled)
		goto done;

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!comp) {
		PWM_TURBO_ERR("request output fail\n");
		mutex_unlock(&g_pwm_turbo_lock);
		return -EINVAL;
	}

	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);

		if (oplus_display_brightness > plus_bl) {
			if (en) {
				pulse_flg = true;
				pwm_params->pwm_pul_cmd_id = PWM_SWITCH_3TO1;
				set_pwm_turbo_switch_state(PWM_SWITCH_ONEPULSE_STATE);
			} else {
				pulse_flg = true;
				pwm_params->pwm_pul_cmd_id = PWM_SWITCH_1TO3;
				set_pwm_turbo_switch_state(PWM_SWITCH_DC_STATE);
			}
			if (comp->funcs && comp->funcs->io_cmd) {
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_PULSE_BL, &oplus_display_brightness);
			}
			if (comp->funcs && comp->funcs->io_cmd) {
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_PULSE_BL, &oplus_display_brightness);
			}
		}

	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);

done:
	mutex_unlock(&g_pwm_turbo_lock);

	return 0;
}

int oplus_display_panel_set_pwm_turbo_switch(struct drm_crtc *crtc, unsigned int en)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	struct mtk_panel_ext *ext = mtk_crtc->panel_ext;
	struct cmdq_pkt *cmdq_handle;
	struct mtk_crtc_state *state =
	    to_mtk_crtc_state(mtk_crtc->base.state);
	unsigned int src_mode =
	    state->prop_val[CRTC_PROP_DISP_MODE_IDX];
	unsigned int fps = 0;
	unsigned int i = 0;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	if (ext == NULL) {
		DDPINFO("%s %d mtk_crtc->panel_ext is NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (pwm_params->config == OPLUS_PWM_TRUBO_GLOBAL_OPEN_NO_SWITCH) {
		PWM_TURBO_ERR("pwm_turbo no swtich\n");
		return 0;
	}

	if (pwm_params->pwm_turbo_enabled == en) {
        PWM_TURBO_ERR("pwm_turbo_enabled no changer\n");
		return 0;
	}

	pr_info("%s, en=%d, src_mode=%d, ext->params->dyn_fps.vact_timing_fps=%d\n",
		__func__, en, src_mode, ext->params->dyn_fps.vact_timing_fps);
	mutex_lock(&g_pwm_turbo_lock);

	if (!mtk_crtc->enabled)
		goto done;

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!comp) {
		PWM_TURBO_ERR("request output fail\n");
		mutex_unlock(&g_pwm_turbo_lock);
		return -EINVAL;
	}

	/* because mode = 2 (90fps), not support hpwm */
	if (ext->params->dyn_fps.vact_timing_fps == 90) {
		pr_info("%s, if 90fps goto done\n", __func__);
		goto done;
	}

	mtk_drm_send_lcm_cmd_prepare_wait_for_vsync(crtc, &cmdq_handle);

	if (pwm_params->pwm_wait_te > 0) {
		/** wait one TE **/
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		if (mtk_drm_lcm_is_connect(mtk_crtc))
			cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
	}
	/* because only mode = 3 (60fps), we need changed to 120fps */
	pr_info("pwm_turbo %s, src_mode=%d\n", __func__, src_mode);
	if (ext->params->dyn_fps.vact_timing_fps == 60) {
		fps = 120;
		if (!en)
			pwm_params->pwm_fps_mode = !en;
		pr_info("pwm_turbo %s, src_mode=%d,hpwm_fps_mode=%d\n", __func__, src_mode, pwm_params->pwm_fps_mode);
		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_FPS, &fps);
		if (pwm_params->pwm_wait_te > 0) {
			for (i = 0; i < pwm_params->pwm_wait_te; i++) {
				/** wait one TE **/
				cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
				if (mtk_drm_lcm_is_connect(mtk_crtc))
					cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			}
		}
		pwm_params->pwm_fps_mode = en;
	}

	if (comp && comp->funcs && comp->funcs->io_cmd)
		comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM, &en);

	if (pwm_params->pwm_wait_te > 0) {
		/** wait one TE **/
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		if (mtk_drm_lcm_is_connect(mtk_crtc))
			cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
	}

	if (comp && comp->funcs && comp->funcs->io_cmd)
		comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_ELVSS, &en);

	/* because only mode = 3 (60fps), we need recovery 60fps */
	if (ext->params->dyn_fps.vact_timing_fps == 60) {
		fps = 60;
		if (pwm_params->pwm_wait_te > 0) {
			/** wait one TE **/
			cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			if (mtk_drm_lcm_is_connect(mtk_crtc))
				cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		}
		pr_info("pwm_turbo %s, 60fps src_mode=%d, pwm_fps_mode=%d\n", __func__, src_mode, pwm_params->pwm_fps_mode);
		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_FPS, &fps);
	}

	/* because only mode = 0 (120fps), we need set fps close pwm */
	if (ext->params->dyn_fps.vact_timing_fps == 120) {
		fps = 120;
		pwm_params->pwm_fps_mode = en;
		pr_info("pwm_turbo %s, 120fps src_mode=%d, pwm_fps_mode=%d\n", __func__, src_mode, pwm_params->pwm_fps_mode);
		if (pwm_params->pwm_wait_te > 0) {
			/** wait one TE **/
			cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
			if (mtk_drm_lcm_is_connect(mtk_crtc))
				cmdq_pkt_wfe(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
		}
		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_FPS, &fps);
	}
	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);

done:
	mutex_unlock(&g_pwm_turbo_lock);

	return 0;
}

/* -------------------- oplus hidl nodes --------------------------------------- */
int oplus_display_panel_set_pwm_status(void *data)
{
	int rc = 0;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *pwm_status = data;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	if (!data) {
		pr_err("%s: set pwm status data is null\n", __func__);
		return -EINVAL;
	}

	printk(KERN_INFO "oplus high pwm mode = %d\n", *pwm_status);

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}

	if (pwm_params->config == OPLUS_PWM_TRUBO_GLOBAL_OPEN_NO_SWITCH) {
		/* global pwm default pwm_turbo_enabled is ture */
		pwm_params->pwm_turbo_enabled = true;
	} else {
		rc = oplus_display_panel_set_pwm_turbo_switch(crtc, *pwm_status);
		pwm_params->pwm_turbo_enabled = (long)*pwm_status;
	}

	return rc;
}

int oplus_display_panel_get_pwm_status(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *pwm_status = buf;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();


	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	pr_info("%s: %s\n", __func__, mtk_crtc->panel_ext->params->vendor);

	mutex_lock(&g_pwm_turbo_lock);
	if (!strcmp(mtk_crtc->panel_ext->params->vendor, "Tianma_NT37705")) {
		*pwm_status = pwm_params->pwm_turbo_enabled;
		pr_info("%s: high pwm mode = %d\n", __func__, pwm_params->pwm_turbo_enabled);
	} else {
		*pwm_status = 0;
	}
	mutex_unlock(&g_pwm_turbo_lock);

	return 0;
}

int oplus_display_panel_get_pwm_status_for_90hz(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *pwm_status = buf;

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	if (!strcmp(mtk_crtc->panel_ext->params->vendor, "22823_Tianma_NT37705")
		|| !strcmp(mtk_crtc->panel_ext->params->vendor, "Tianma_NT37705")) {
		*pwm_status = 0;
	} else {
		*pwm_status = 10;
	}

	return 0;
}

/* -------------------- oplus api nodes ----------------------------------------------- */
ssize_t  oplus_display_get_high_pwm(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
		struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

        printk(KERN_INFO "high pwm mode = %d\n", pwm_params->pwm_turbo_enabled);

        return sprintf(buf, "%d\n", pwm_params->pwm_turbo_enabled);
}

ssize_t  oplus_display_set_high_pwm(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct drm_crtc *crtc;
	unsigned int temp_save = 0;
	int ret = 0;
	struct drm_device *ddev = get_drm_device();
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	ret = kstrtouint(buf, 10, &temp_save);
	printk(KERN_INFO "pwm_turbo mode = %d\n", temp_save);

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return 0;
	}
	if (pwm_params->config == OPLUS_PWM_TRUBO_GLOBAL_OPEN_NO_SWITCH) {
		/* global pwm default hpwm_mode is ture */
		pwm_params->pwm_turbo_enabled = true;
	} else {
		oplus_display_panel_set_pwm_turbo_switch(crtc, temp_save);
		pwm_params->pwm_turbo_enabled = temp_save;
	}

	return count;
}

static void hpwm_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;
	oplus_disp_trace_begin("hpwm_cmdq_cb");
	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
	oplus_disp_trace_end("hpwm_cmdq_cb");
}

int oplus_display_panel_set_pwm_bl_temp(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct mtk_cmdq_cb_data *cb_data;
	struct cmdq_pkt *cmdq_handle;
	struct cmdq_client *client;
	bool is_frame_mode;

	if (!(comp && comp->funcs && comp->funcs->io_cmd))
		return -EINVAL;

	if (!(mtk_crtc->enabled)) {
		DDPINFO("%s: skip, slept\n", __func__);
		return -EINVAL;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPPR_ERR("hpwm_temp cb data creation failed\n");
		return -EINVAL;
	}

	DDPINFO("%s: start\n", __func__);

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

	/* send temp cmd would use VM CMD in  DSI VDO mode only */
	client = (is_frame_mode) ? mtk_crtc->gce_obj.client[CLIENT_CFG] :
				mtk_crtc->gce_obj.client[CLIENT_DSI_CFG];
	#ifndef OPLUS_FEATURE_DISPLAY
	cmdq_handle =
		cmdq_pkt_create(client);
	#else
	mtk_crtc_pkt_create(&cmdq_handle, crtc, client);
	#endif

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	/** wait one TE **/
	cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);
	if (mtk_drm_lcm_is_connect(mtk_crtc))
		cmdq_pkt_wait_no_clear(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_TE]);

	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}
	if(mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps == 90) {
		usleep_range(3000, 3100);
	} else {
		usleep_range(1100, 1200);
	}
	DDPPR_ERR("%s:%d 90nit send temp cmd\n", __func__, __LINE__);
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	if (oplus_temp_compensation_is_supported()) {
		oplus_temp_compensation_io_cmd_set(comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING);
	}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	cb_data->crtc = crtc;
	cb_data->cmdq_handle = cmdq_handle;

	if (cmdq_pkt_flush_threaded(cmdq_handle, hpwm_cmdq_cb, cb_data) < 0) {
		DDPPR_ERR("failed to flush hpwm_cmdq_cb\n");
		return -EINVAL;
	}

	DDPINFO("%s: end\n", __func__);

	return 0;
}

int oplus_display_panel_set_pwm_bl(struct drm_crtc *crtc, struct cmdq_pkt *cmdq_handle, unsigned int level, bool is_sync)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();

	oplus_display_brightness = level;
	pwm_params->oplus_pwm_threshold = PWM_NORMAL_BL;

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	if (oplus_ofp_is_supported()) {
		if (oplus_ofp_backlight_filter(crtc, cmdq_handle, level)) {
			goto end;
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	if (oplus_panel_pwm_turbo_is_enabled()) {
		if (comp->funcs && comp->funcs->io_cmd) {
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_HPWM_PULSE_BL, &level);
		}
	} else {
		if (comp->funcs && comp->funcs->io_cmd) {
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &level);
		}
	}
end:
#ifdef OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
	if (oplus_temp_compensation_is_supported()) {
		if (!((pwm_params->oplus_pwm_threshold != PWM_NORMAL_BL) && (is_sync == true))) {
			oplus_temp_compensation_io_cmd_set(comp, cmdq_handle, OPLUS_TEMP_COMPENSATION_BACKLIGHT_SETTING);
		}
	}
#endif /* OPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION */

	return 0;
}

inline bool oplus_panel_pwm_onepulse_is_enabled(void)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return false;
	}

	return (bool)(pwm_params->pwm_onepulse_support &&
		pwm_params->pwm_onepulse_enabled);
}
EXPORT_SYMBOL(oplus_panel_pwm_onepulse_is_enabled);


inline bool oplus_panel_pwm_onepulse_switch_state(void)
{
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return false;
	}

	return (bool)(pwm_params->pwm_onepulse_support &&
		pwm_params->pwm_onepulse_enabled && pwm_params->oplus_pwm_switch_state);
}
EXPORT_SYMBOL(oplus_panel_pwm_onepulse_switch_state);

int oplus_display_panel_set_pwm_pulse(void *data)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int enabled = *((unsigned int*)data);
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return -EINVAL;
	}

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		PWM_TURBO_ERR("find crtc fail\n");
		return -EINVAL;
	}

	if (!pwm_params->pwm_onepulse_support) {
		pr_info("pwm_turbo falied to set pwm onepulse status, not support\n");
		return -EINVAL;
	}

	pr_info("set pwm onepulse status: %d\n", enabled);
	if ((bool)enabled == pwm_params->pwm_onepulse_enabled) {
		pr_info("skip setting duplicate pwm onepulse status: %d\n", enabled);
		return -EINVAL;
	}

	pwm_params->pwm_onepulse_enabled = (bool)enabled;
	pwm_params->oplus_pwm_switch_state_changed = true;
	oplus_display_panel_set_pwm_turbo_switch_onepulse(crtc, enabled);

	return 0;
}

int oplus_display_panel_get_pwm_pulse(void *data)
{
	bool *enabled = (bool*)data;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!pwm_params->pwm_onepulse_support) {
		pr_info("pwm_turbo falied to set pwm onepulse status, not support\n");
		return -EINVAL;
	}

	mutex_lock(&g_pwm_turbo_lock);
	*enabled = pwm_params->pwm_onepulse_enabled;
	mutex_unlock(&g_pwm_turbo_lock);
	pr_info("get pwm onepulse status: %d\n", *enabled);

	return 0;
}

ssize_t oplus_get_pwm_pulse_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	unsigned int enabled = false;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!pwm_params->pwm_onepulse_support) {
		pr_info("pwm_turbo falied to set pwm onepulse status, not support\n");
		return -EINVAL;
	}

	mutex_lock(&g_pwm_turbo_lock);
	enabled = pwm_params->pwm_onepulse_enabled;
	mutex_unlock(&g_pwm_turbo_lock);
	pr_info("get pwm onepulse status: %d\n", enabled);

	return sysfs_emit(buf, "%d\n", enabled);
}

ssize_t oplus_set_pwm_pulse_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int enabled = 0;
	int rc = 0;
	struct oplus_pwm_turbo_params *pwm_params = oplus_pwm_turbo_get_params();
	if (IS_ERR_OR_NULL(pwm_params)) {
		PWM_TURBO_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (!pwm_params->pwm_onepulse_support) {
		pr_info("pwm_turbo falied to set pwm onepulse status, not support\n");
		return -EINVAL;
	}

	rc = kstrtou32(buf, 10, &enabled);
	if (rc) {
		pr_info("%s cannot be converted to u32", buf);
		return count;
	}

	pr_info("set pwm onepulse status: %d\n", enabled);
	mutex_lock(&g_pwm_turbo_onepulse_lock);
	oplus_display_panel_set_pwm_pulse(&enabled);
	mutex_unlock(&g_pwm_turbo_onepulse_lock);
	return count;
}
