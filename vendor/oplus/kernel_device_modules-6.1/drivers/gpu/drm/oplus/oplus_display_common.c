/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_dc.c
** Description : oplus dc feature
** Version : 1.0
** Date : 2020/07/1
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  JianBin.Zhang   2020/07/01        1.0           Build this moudle
**  Xiaolei.Gao     2021/08/14        1.1           Build this moudle
***************************************************************/
#include <oplus_display_common.h>
#include "oplus_display_panel.h"
#include "oplus_display_trace.h"
#include "mtk_drm_mmp.h"

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#define PANEL_SERIAL_NUM_REG 0xA1
#define PANEL_REG_READ_LEN   10
#define BOE_PANEL_SERIAL_NUM_REG 0xA3
#define PANEL_SERIAL_NUM_REG_TIANMA 0xD6
#define SILKY_MAX_NORMAL_BRIGHTNESS 8191

bool g_is_mode_switch_pack = false;
EXPORT_SYMBOL(g_is_mode_switch_pack);
int g_last_mode_idx = 0;
EXPORT_SYMBOL(g_last_mode_idx);
/* pcp: panel_cmdq_pkg */
wait_queue_head_t oplus_pcp_lock_clear_wq;
EXPORT_SYMBOL(oplus_pcp_lock_clear_wq);
struct task_struct *oplus_pcp_task;
EXPORT_SYMBOL(oplus_pcp_task);
struct mutex oplus_pcp_lock;
EXPORT_SYMBOL(oplus_pcp_lock);
atomic_t oplus_pcp_handle_lock;
EXPORT_SYMBOL(oplus_pcp_handle_lock);
atomic_t oplus_pcp_num;
EXPORT_SYMBOL(oplus_pcp_num);
int oplus_dc_enable_real = 0;
EXPORT_SYMBOL(oplus_dc_enable_real);
int oplus_dc_enable = 0;
EXPORT_SYMBOL(oplus_dc_enable);
int oplus_dc_recovery = 0;
EXPORT_SYMBOL(oplus_dc_recovery);
int exit_dc_flag = 0;
EXPORT_SYMBOL(exit_dc_flag);
int ffl_backlight_backup = 0;
EXPORT_SYMBOL(ffl_backlight_backup);

extern unsigned int cabc_mode;
extern unsigned int cabc_true_mode;
extern unsigned int cabc_sun_flag;
extern unsigned int cabc_back_flag;
extern bool panel_is_dunhuang(void);
extern void disp_aal_set_dre_en(struct mtk_ddp_comp *comp, int enable);
extern unsigned int silence_mode;
extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern uint64_t serial_number;
extern unsigned int esd_mode;
extern unsigned int seed_mode;
extern unsigned int m_da;
extern unsigned int m_db;
extern unsigned int m_dc;
extern bool g_dp_support;
extern bool pq_trigger;
extern unsigned int get_project(void);
extern char lcm_version[32];
extern char lcm_manufacture[32];
extern bool oplus_hbm_max_en;
extern unsigned long long oplus_last_te_time;
DEFINE_MUTEX(oplus_seed_lock);
DEFINE_MUTEX(g_oplus_hbm_max_lock);
DEFINE_MUTEX(g_oplus_hbm_max_switch_lock);

extern struct drm_device *get_drm_device(void);
extern int mtk_drm_setbacklight(struct drm_crtc *crtc, unsigned int level, unsigned int panel_ext_param, unsigned int cfg_flag, unsigned int lock);

extern int panel_serial_number_read(struct drm_crtc *crtc, char cmd, int num);
extern int oplus_mtk_drm_setcabc(struct drm_crtc *crtc, unsigned int hbm_mode);
extern int oplus_mtk_drm_setseed(struct drm_crtc *crtc, unsigned int seed_mode);
extern void mtk_drm_send_lcm_cmd_prepare_wait_for_vsync(struct drm_crtc *crtc, struct cmdq_pkt **cmdq_handle);
extern void mtk_crtc_wait_frame_done(struct mtk_drm_crtc *mtk_crtc,
			      struct cmdq_pkt *cmdq_handle,
			      enum CRTC_DDP_PATH ddp_path,
			      int clear_event);
extern u16 mtk_get_gpr(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle);

enum {
	CABC_LEVEL_0,
	CABC_LEVEL_1,
	CABC_LEVEL_2 = 3,
	CABC_EXIT_SPECIAL = 8,
	CABC_ENTER_SPECIAL = 9,
};

unsigned int oplus_display_panel_max_brightness = 4095;
EXPORT_SYMBOL(oplus_display_panel_max_brightness);

int oplus_display_set_brightness(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *set_brightness = buf;
	unsigned int oplus_set_brightness = (*set_brightness);

	printk("%s %d\n", __func__, oplus_set_brightness);

	if (oplus_set_brightness > OPLUS_MAX_BRIGHTNESS || oplus_set_brightness < OPLUS_MIN_BRIGHTNESS) {
		printk(KERN_ERR "%s, brightness:%d out of scope\n", __func__, oplus_set_brightness);
		return -1;
	}

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mtk_drm_setbacklight(crtc, oplus_set_brightness, 0, 0x1 << SET_BACKLIGHT_LEVEL, 1);

	return 0;
}

int oplus_display_get_brightness(void *buf)
{
	unsigned int *brightness = buf;

	(*brightness) = oplus_display_brightness;

	return 0;
}

int oplus_display_panel_get_max_brightness(void *buf)
{
	unsigned int *brightness = buf;

	(*brightness) = oplus_max_normal_brightness;

	return 0;
}

int oplus_display_panel_get_max_dbv(void *data)
{
	unsigned int *dbv = (unsigned int*)data;
	*dbv = oplus_display_panel_max_brightness;
	pr_info("%s get max dbv: %d\n", __func__, oplus_display_panel_max_brightness);
	return 0;
}

int oplus_display_panel_dbv_probe(struct device *dev)
{
	u32 config = 0;
	int rc = 0;

	if (IS_ERR_OR_NULL(dev)) {
		pr_err("%s Invalid params\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(dev->of_node, "oplus,panel-max-brightness", &config);
	if (rc == 0) {
		oplus_display_panel_max_brightness = config;
	} else {
		oplus_display_panel_max_brightness = 4095;
	}
	pr_info("%s config=%d, oplus_display_panel_max_brightness=%d\n",
		__func__, config, oplus_display_panel_max_brightness);
	return 0;
}
EXPORT_SYMBOL(oplus_display_panel_dbv_probe);

int oplus_display_panel_get_panel_bpp(void *buf)
{
	unsigned int *panel_bpp = buf;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();

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

	(*panel_bpp) = mtk_crtc->panel_ext->params->panel_bpp;
	printk("%s panel_bpp : %d\n", __func__, *panel_bpp);

	return 0;
}

int oplus_display_panel_get_serial_number(void *buf)
{
	struct panel_serial_number *p_snumber = buf;
	int ret = 0;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
		typeof(*crtc), head);
	if (!crtc) {
		DDPPR_ERR("get_panel_serial_number find crtc fail\n");
		return 0;
	}

	if (oplus_display_brightness == 0) {
		pr_info("backlight is 0, skip get serial number!\n");
		return 0;
	}

	if (serial_number == 0) {
		panel_serial_number_read(crtc, PANEL_SERIAL_NUM_REG, PANEL_REG_READ_LEN);
		pr_info("%s read, serial_number: 0x%llx, da=0x%x, db=0x%x, dc=0x%x\n", __func__, serial_number, m_da, m_db, m_dc);
	}

	printk("%s read serial number 0x%llx\n", __func__, serial_number);
	ret = scnprintf(p_snumber->serial_number, sizeof(p_snumber->serial_number)+1, "Get panel serial number: %llx\n", serial_number);
	return ret;
}

int oplus_display_panel_get_cabc(void *buf)
{
	unsigned int *c_mode = buf;

	printk("%s CABC_mode=%d\n", __func__, cabc_true_mode);
	*c_mode = cabc_true_mode;

	return 0;
}

int oplus_display_panel_set_cabc(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_ddp_comp *comp;
	uint32_t *cabc_mode_temp = buf;

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

	comp = mtk_ddp_comp_sel_in_cur_crtc_path(mtk_crtc, MTK_DISP_AAL, 0);
	if (!comp) {
		DDPPR_ERR("%s, comp is null!\n", __func__);
		return 0;
	}

	cabc_mode = *cabc_mode_temp;
	cabc_true_mode = cabc_mode;
	printk("%s,cabc mode is %d, cabc_back_flag is %d, oplus_display_global_dre = %d\n", __func__,
		cabc_mode, cabc_back_flag, mtk_crtc->panel_ext->params->oplus_display_global_dre);
	if (cabc_mode < 4) {
		cabc_back_flag = cabc_mode;
	}

	if (cabc_mode == CABC_ENTER_SPECIAL) {
		cabc_sun_flag = 1;
		cabc_true_mode = 0;
	} else if (cabc_mode == CABC_EXIT_SPECIAL) {
		cabc_sun_flag = 0;
		cabc_true_mode = cabc_back_flag;
	} else if (cabc_sun_flag == 1) {
		if (cabc_back_flag == CABC_LEVEL_0 || mtk_crtc->panel_ext->params->oplus_display_global_dre) {
			disp_aal_set_dre_en(comp, 1);
			printk("%s sun enable dre\n", __func__);
		} else {
			disp_aal_set_dre_en(comp, 0);
			printk("%s sun disable dre\n", __func__);
		}
		return 0;
	}

	printk("%s,cabc mode is %d\n", __func__, cabc_true_mode);

	if ((cabc_true_mode == CABC_LEVEL_0 && cabc_back_flag == CABC_LEVEL_0)|| mtk_crtc->panel_ext->params->oplus_display_global_dre) {
		disp_aal_set_dre_en(comp, 1);
		printk("%s enable dre\n", __func__);
	} else {
		disp_aal_set_dre_en(comp, 0);
		printk("%s disable dre\n", __func__);
	}
	if (panel_is_dunhuang()) {
		oplus_mtk_drm_setcabc(crtc, cabc_true_mode);
	}
	if (cabc_true_mode != cabc_back_flag) {
		cabc_true_mode = cabc_back_flag;
	}
	return 0;
}



int oplus_display_panel_get_closebl_flag(void *buf)
{
	unsigned int *closebl_flag = buf;

	printk("%s silence_mode=%d\n", __func__, silence_mode);
	(*closebl_flag) = silence_mode;

	return 0;
}

int oplus_display_panel_set_closebl_flag(void *buf)
{
	unsigned int *closebl_flag = buf;

	msleep(1000);
	silence_mode = (*closebl_flag);
	printk("%s silence_mode=%d\n", __func__, silence_mode);

	return 0;
}

int oplus_display_panel_get_esd(void *buf)
{
	unsigned int *p_esd = buf;

	printk("%s esd=%d\n", __func__, esd_mode);
	(*p_esd) = esd_mode;

	return 0;
}

int oplus_display_panel_set_esd(void *buf)
{
	unsigned int *p_esd = buf;

	esd_mode = (*p_esd);
	printk("%s,esd mode is %d\n", __func__, esd_mode);

	return 0;
}

int oplus_display_panel_get_vendor(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct panel_info *p_info = buf;
	struct drm_device *ddev = get_drm_device();

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail,p_info=%p\n", p_info);
		return -1;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc || !mtk_crtc->panel_ext || !mtk_crtc->panel_ext->params) {
		pr_err("falied to get lcd proc info\n");
		return -EINVAL;
	}

	pr_info("get lcd proc info: vendor[%s] lcm_version[%s] lcm_manufacture[%s]\n", mtk_crtc->panel_ext->params->vendor,
			lcm_version, lcm_manufacture);
	if (!strcmp(lcm_version, "")) {
		memcpy(p_info->version, mtk_crtc->panel_ext->params->vendor,
               sizeof(mtk_crtc->panel_ext->params->vendor) >= 31?31:(sizeof(mtk_crtc->panel_ext->params->vendor)+1));
	} else {
		memcpy(p_info->version, lcm_version,
            sizeof(lcm_version) >= 31?31:(sizeof(lcm_version)+1));
	}

	if (!strcmp(lcm_manufacture, "")) {
		memcpy(p_info->manufacture, mtk_crtc->panel_ext->params->manufacture,
               sizeof(mtk_crtc->panel_ext->params->manufacture) >= 31?31:(sizeof(mtk_crtc->panel_ext->params->manufacture)+1));
	} else {
		memcpy(p_info->manufacture, lcm_manufacture,
               sizeof(lcm_manufacture) >= 31?31:(sizeof(lcm_manufacture)+1));
	}

	return 0;
}

int oplus_display_get_softiris_color_status(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	struct softiris_color *iris_color_status = buf;

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

	iris_color_status->color_vivid_status = mtk_crtc->panel_ext->params->color_vivid_status;
	iris_color_status->color_srgb_status = mtk_crtc->panel_ext->params->color_srgb_status;
	iris_color_status->color_softiris_status = mtk_crtc->panel_ext->params->color_softiris_status;
	iris_color_status->color_dual_panel_status = mtk_crtc->panel_ext->params->color_dual_panel_status;
	iris_color_status->color_dual_brightness_status = mtk_crtc->panel_ext->params->color_dual_brightness_status;
	iris_color_status->color_oplus_calibrate_status = mtk_crtc->panel_ext->params->color_oplus_calibrate_status;
	iris_color_status->color_samsung_status = mtk_crtc->panel_ext->params->color_samsung_status;
	iris_color_status->color_loading_status = mtk_crtc->panel_ext->params->color_loading_status;
	iris_color_status->color_2nit_status = mtk_crtc->panel_ext->params->color_2nit_status;
	iris_color_status->color_nature_profession_status = mtk_crtc->panel_ext->params->color_nature_profession_status;
	pr_err("oplus_color_vivid_status: %s", iris_color_status->color_vivid_status ? "true" : "false");
	pr_err("oplus_color_srgb_status: %s", iris_color_status->color_srgb_status ? "true" : "false");
	pr_err("oplus_color_softiris_status: %s", iris_color_status->color_softiris_status ? "true" : "false");
	pr_err("color_dual_panel_status: %s", iris_color_status->color_dual_panel_status ? "true" : "false");
	pr_err("color_dual_brightness_status: %s", iris_color_status->color_dual_brightness_status ? "true" : "false");
	pr_err("color_samsung_status: %s", iris_color_status->color_samsung_status ? "true" : "false");
	pr_err("color_loading_status: %s", iris_color_status->color_loading_status ? "true" : "false");
	pr_err("color_2nit_status: %s", iris_color_status->color_2nit_status ? "true" : "false");
	pr_err("color_nature_profession_status: %s", iris_color_status->color_nature_profession_status ? "true" : "false");
	return 0;
}

int oplus_mtk_drm_setseed(struct drm_crtc *crtc, unsigned int seed_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_crtc_state *crtc_state;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);

	mutex_lock(&mtk_crtc->lock);

	crtc_state = to_mtk_crtc_state(crtc->state);
	if (!mtk_crtc->enabled || crtc_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
		DDPINFO("%s:%d, crtc is not reusmed!\n", __func__, __LINE__);
		mutex_unlock(&mtk_crtc->lock);
		return -EINVAL;
	}

	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);

	/* set hbm */
	if (comp && comp->funcs && comp->funcs->io_cmd)
		comp->funcs->io_cmd(comp, cmdq_handle, LCM_SEED, &seed_mode);

	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);
	mutex_unlock(&mtk_crtc->lock);

	return 0;
}

int oplus_display_panel_get_seed(void *buf)
{
	unsigned int *seed = buf;

	mutex_lock(&oplus_seed_lock);
	printk("%s seed_mode=%d\n", __func__, seed_mode);
	(*seed) = seed_mode;
	mutex_unlock(&oplus_seed_lock);

	return 0;
}

int oplus_display_panel_set_seed(void *buf)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *seed_mode_tmp = buf;

	printk("%s, %d to be %d\n", __func__, seed_mode, *seed_mode_tmp);

	/* this debug cmd only for crtc0 */
	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return -1;
	}
	mutex_lock(&oplus_seed_lock);
	oplus_mtk_drm_setseed(crtc, *seed_mode_tmp);
	seed_mode = (*seed_mode_tmp);
	mutex_unlock(&oplus_seed_lock);

	return 0;
}

int oplus_display_panel_get_id(void *buf)
{
	struct panel_id *panel_rid = buf;

	pr_err("%s: 0xDA= 0x%x, 0xDB=0x%x, 0xDC=0x%x\n", __func__, m_da, m_db, m_dc);

	panel_rid->DA = (uint32_t)m_da;
	panel_rid->DB = (uint32_t)m_db;
	panel_rid->DC = (uint32_t)m_dc;

	return 0;
}

int oplus_display_get_dp_support(void *buf)
{
	uint32_t *dp_support = buf;

	pr_info("%s: dp_support = %s\n", __func__, g_dp_support ? "true" : "false");

	*dp_support = g_dp_support;

	return 0;
}
int oplus_display_panel_get_pq_trigger(void *buf)
{
        unsigned int *pq_trigger_flag = buf;

        printk("%s pq_trigger=%d\n", __func__, pq_trigger);
        (*pq_trigger_flag) = pq_trigger;

        return 0;
}

int oplus_display_panel_set_pq_trigger(void *buf)
{
        unsigned int *pq_trigger_flag = buf;

        pq_trigger = (*pq_trigger_flag);
        printk("%s pq_trigger=%d\n", __func__, pq_trigger);

        return 0;
}

int g_need_read_reg = 0;
void oplus_te_check(struct mtk_drm_crtc *mtk_crtc, unsigned long long te_time_diff)
{
	struct drm_crtc *crtc = NULL;
	static int refresh_rate;
	static int vsync_period;
	static int last_refresh_rate = 0;
	static int count = 0;
	static int entry_count = 0;
	static int need_te_check = 0;

	if (!mtk_crtc) {
		DDPPR_ERR("oplus_te_check mtk_crtc is null");
		return;
	}

	crtc = &mtk_crtc->base;

	if (oplus_display_brightness < 1) {
		return;
	}

	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
		&& mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps != 0)
		refresh_rate = mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;

	if (refresh_rate != last_refresh_rate &&  crtc->state->mode.hskew != 2) {
		/* pr_err("need_te_check, refresh_rate = %d, last_refresh_rate = %d\n", refresh_rate, last_refresh_rate); */
		need_te_check = 1;
		count = 0;
		vsync_period = 1000000 / refresh_rate;
	}
	last_refresh_rate = refresh_rate;

	if (need_te_check == 0) {
		return;
	}

	/* pr_err("refresh_rate = %d, vsync_period = %d, te_time_diff = %lu\n", refresh_rate, vsync_period, te_time_diff); */
	if (abs(te_time_diff / 1000 - vsync_period) > vsync_period / 5) { /* The error is more than 20% */
		count++;
		if (count > 10) { /* TE error for 10 consecutive frames */
			DDPPR_ERR("oplus_te_check failed, refresh_rate=%d, te_time_diff=%llu\n", refresh_rate, te_time_diff);
			g_need_read_reg = 1;
			need_te_check = 0;
			count = 0;
		}
	} else {
		entry_count++;
		if (entry_count > 100) { /* 100 consecutive frames is no problem, clear the count */
			entry_count = 0;
			count = 0;
			need_te_check = 0;
		}
	}
}

void oplus_mtk_read_ddic_v2(u8 ddic_reg, int ret_num, char ret_val[10])
{
		unsigned int j = 0;
		unsigned int ret_dlen = 0;
		int ret;
		struct mtk_ddic_dsi_msg *cmd_msg =
				vmalloc(sizeof(struct mtk_ddic_dsi_msg));
		u8 tx[10] = {0};
		DDPMSG("%s read val %d\n", __func__, ret_num);

		if (!cmd_msg) {
			DDPPR_ERR("cmd msg is NULL\n");
			return;
		}
		memset(cmd_msg, 0, sizeof(struct mtk_ddic_dsi_msg));

		cmd_msg->channel = 0;
		cmd_msg->tx_cmd_num = 1;
		cmd_msg->type[0] = 0x06;
		tx[0] = ddic_reg;
		cmd_msg->tx_buf[0] = tx;
		cmd_msg->tx_len[0] = 1;

		cmd_msg->rx_cmd_num = 1;
		cmd_msg->rx_buf[0] = vmalloc(20 * sizeof(unsigned char));
		memset(cmd_msg->rx_buf[0], 0, 20);
		cmd_msg->rx_len[0] = ret_num;

		ret = mtk_ddic_dsi_read_cmd(cmd_msg);

		if (ret != 0) {
			DDPPR_ERR("%s error\n", __func__);
			goto  done;
		}

		ret_dlen = cmd_msg->rx_len[0];
		DDPMSG("read lcm addr:0x%x--dlen:%d\n",
			*(char *)(cmd_msg->tx_buf[0]), ret_dlen);

		for (j = 0; j < ret_dlen; j++)
			ret_val[j] = *(char *)(cmd_msg->rx_buf[0] + j);

done:
		vfree(cmd_msg->rx_buf[0]);
		vfree(cmd_msg);

		DDPMSG("%s end -\n", __func__);
}

EXPORT_SYMBOL(oplus_mtk_read_ddic_v2);

void oplus_ddic_dsi_send_cmd(unsigned int cmd_num,
	char val[20])
{
	unsigned int i = 0, j = 0;
	int ret;
	struct mtk_ddic_dsi_msg *cmd_msg =
		vmalloc(sizeof(struct mtk_ddic_dsi_msg));
	u8 tx[10] = {0};

	DDPMSG("%s cmd_num:%d\n", __func__, cmd_num);

	if (!cmd_num || cmd_num > 10)
		goto  done;

	memset(cmd_msg, 0, sizeof(struct mtk_ddic_dsi_msg));

	switch (cmd_num) {
	case 1:
		cmd_msg->type[0] = 0x05;
		break;
	case 2:
		cmd_msg->type[0] = 0x15;
		break;
	default:
		cmd_msg->type[0] = 0x39;
		break;
	}

	cmd_msg->channel = 0;
	cmd_msg->flags |= MIPI_DSI_MSG_USE_LPM;
	cmd_msg->tx_cmd_num = 1;
	for (i = 0; i < cmd_num; i++) {
		tx[i] = val[i];
		DDPMSG("val[%d]:%d\n", i, val[i]);
	}
	cmd_msg->tx_buf[0] = tx;
	cmd_msg->tx_len[0] = cmd_num;

	DDPMSG("send lcm tx_cmd_num:%d\n", (int)cmd_msg->tx_cmd_num);
	for (i = 0; i < (int)cmd_msg->tx_cmd_num; i++) {
		DDPMSG("send lcm tx_len[%d]=%d\n",
			i, (int)cmd_msg->tx_len[i]);
		for (j = 0; j < (int)cmd_msg->tx_len[i]; j++) {
			DDPMSG(
				"send lcm type[%d]=0x%x, tx_buf[%d]--byte:%d,val:0x%x\n",
				i, cmd_msg->type[i], i, j,
				*(char *)(cmd_msg->tx_buf[i] + j));
		}
	}

	ret = mtk_ddic_dsi_send_cmd(cmd_msg, true);
	if (ret != 0) {
		DDPPR_ERR("mtk_ddic_dsi_send_cmd error\n");
		goto  done;
	}
done:
	vfree(cmd_msg);

	DDPMSG("%s end -\n", __func__);
}

EXPORT_SYMBOL(oplus_ddic_dsi_send_cmd);

void oplus_mtk_read_ddic_v3(u8 ddic_reg, int ret_num, char ret_val[20])
{
		unsigned int j = 0;
		unsigned int ret_dlen = 0;
		int ret;
		struct mtk_ddic_dsi_msg *cmd_msg =
						vmalloc(sizeof(struct mtk_ddic_dsi_msg));
		u8 tx[20] = {0};
		DDPMSG("%s read val %d\n", __func__, ret_num);

		if (!cmd_msg) {
				DDPPR_ERR("cmd msg is NULL\n");
				return;
		}
		memset(cmd_msg, 0, sizeof(struct mtk_ddic_dsi_msg));

		cmd_msg->channel = 0;
		cmd_msg->tx_cmd_num = 1;
		cmd_msg->type[0] = 0x06;
		tx[0] = ddic_reg;
		cmd_msg->tx_buf[0] = tx;
		cmd_msg->tx_len[0] = 1;

		cmd_msg->rx_cmd_num = 1;
		cmd_msg->rx_buf[0] = vmalloc(20 * sizeof(unsigned char));
		memset(cmd_msg->rx_buf[0], 0, 20);
		cmd_msg->rx_len[0] = 20;

		ret = mtk_ddic_dsi_read_cmd(cmd_msg);

		if (ret != 0) {
				DDPPR_ERR("%s error\n", __func__);
				goto  done;
		}

		ret_dlen = cmd_msg->rx_len[0];
		DDPMSG("read lcm addr:0x%x--dlen:%d\n",
				*(char *)(cmd_msg->tx_buf[0]), ret_dlen);

		for (j = 0; j < ret_dlen; j++)
				ret_val[j] = *(char *)(cmd_msg->rx_buf[0] + j);

done:
		vfree(cmd_msg->rx_buf[0]);
		vfree(cmd_msg);

		DDPMSG("%s end -\n", __func__);
}
int oplus_display_panel_get_panel_type(void *buf)
{
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int *panel_type = buf;

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

	*panel_type = mtk_crtc->panel_ext->params->panel_type;
	pr_info("%s: panel_type = %d\n", __func__, *panel_type);

	return 0;
}
EXPORT_SYMBOL(oplus_mtk_read_ddic_v3);

int oplus_display_panel_set_hbm_max_switch(struct drm_crtc *crtc, unsigned int en)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	struct cmdq_pkt *cmdq_handle;


	pr_info("%s, en=%d\n", __func__, en);
	mutex_lock(&g_oplus_hbm_max_switch_lock);

	if (!mtk_crtc->enabled) {
		pr_err("%s crtc not enabled\n", __func__);
		goto done;
	}

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!comp) {
		pr_err("%s, request output fail\n", __func__);
		mutex_unlock(&g_oplus_hbm_max_switch_lock);
		return -EINVAL;
	}

	mtk_drm_send_lcm_cmd_prepare_wait_for_vsync(crtc, &cmdq_handle);
	mtk_ddp_comp_io_cmd(comp, cmdq_handle, DSI_SET_HBM_MAX, &en);
	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);

done:
	mutex_unlock(&g_oplus_hbm_max_switch_lock);

	return 0;
}


int oplus_display_panel_set_hbm_max(void *data)
{
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	unsigned int enabled = *((unsigned int*)data);

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		pr_err("find crtc fail\n");
		return -EINVAL;
	}

	pr_info("%s set hbm_max status: %d\n", __func__, enabled);
	if ((bool)enabled == oplus_hbm_max_en) {
		pr_info("%s skip setting duplicate hbm_apl status: %d\n", __func__, enabled);
		return -EINVAL;
	}

	oplus_hbm_max_en = (bool)enabled;
	oplus_display_panel_set_hbm_max_switch(crtc, enabled);

	return 0;
}

int oplus_display_panel_get_hbm_max(void *data)
{
	bool *enabled = (bool*)data;

	mutex_lock(&g_oplus_hbm_max_lock);
	*enabled = oplus_hbm_max_en;
	mutex_unlock(&g_oplus_hbm_max_lock);
	pr_info("%s get hbm_apl status: %d\n", __func__, *enabled);

	return 0;
}

ssize_t oplus_get_hbm_max_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf)
{
	unsigned int enabled = false;

	mutex_lock(&g_oplus_hbm_max_lock);
	enabled = oplus_hbm_max_en;
	mutex_unlock(&g_oplus_hbm_max_lock);
	pr_info("%s get hbm_max status: %d\n", __func__, enabled);

	return sysfs_emit(buf, "%d\n", enabled);
}

ssize_t oplus_set_hbm_max_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int enabled = 0;
	int rc = 0;

	rc = kstrtou32(buf, 10, &enabled);
	if (rc) {
		pr_info("%s cannot be converted to u32", buf);
		return count;
	}

	pr_info("%s set hbm_max status: %d\n", __func__, enabled);
	mutex_lock(&g_oplus_hbm_max_lock);
	oplus_display_panel_set_hbm_max(&enabled);
	mutex_unlock(&g_oplus_hbm_max_lock);
	return count;
}

void oplus_pcp_handle(bool cmd_is_pcp, void *handle)
{
	int last_fps;
	int cur_fps;
	unsigned int mode_idx;
	/* frame_time,diff_time,merged_time(us);cur_time(ns) */
	unsigned int frame_time = 0;
	unsigned int diff_time = 0;
	unsigned int merged_time = 0;
	unsigned long long cur_time = 0;
	struct drm_crtc *crtc;
	struct drm_device *ddev = get_drm_device();
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_crtc_state *state;
	struct drm_display_mode *last_mode;
	struct drm_display_mode *cur_mode;

	crtc = list_first_entry(&(ddev)->mode_config.crtc_list,
				typeof(*crtc), head);
	if (!crtc) {
		printk(KERN_ERR "find crtc fail\n");
		return;
	}

	mtk_crtc = to_mtk_crtc(crtc);
	state = to_mtk_crtc_state(mtk_crtc->base.state);
	mode_idx = state->prop_val[CRTC_PROP_DISP_MODE_IDX];
	last_mode = &(mtk_crtc->avail_modes[g_last_mode_idx]);
	cur_mode = &(mtk_crtc->avail_modes[mode_idx]);
	last_fps = drm_mode_vrefresh(last_mode);
	cur_fps = drm_mode_vrefresh(cur_mode);
	frame_time = 1000000 / cur_fps;

	if (cmd_is_pcp) {
		oplus_disp_trace_begin("M_LOCK_PCP");
		atomic_inc(&oplus_pcp_num);
		pr_info("%s oplus_pcp_lock, oplus_pcp_num = %d\n", __func__, atomic_read(&oplus_pcp_num));
		mutex_lock(&oplus_pcp_lock);
		if (handle) {
			cur_time = ktime_get();
			diff_time = (cur_time - oplus_last_te_time) / 1000;
			if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
				&& mtk_crtc->panel_ext->params->merge_trig_offset != 0) {
				merged_time = frame_time - (mtk_crtc->panel_ext->params->merge_trig_offset / 26) - 100;
			}
			pr_info("%s handle is'nt null, diff_time = %d, merged_time = %d\n", __func__, diff_time, merged_time);
			if (diff_time > merged_time) {
				usleep_range(1200, 1300);
			}
		} else {
			if (g_is_mode_switch_pack) {
				cur_time = ktime_get();
				diff_time = (cur_time - oplus_last_te_time) / 1000;
				pr_info("%s handle is null, last_fps = %d, hskew = %d, diff_time = %d\n",
					__func__, last_fps, last_mode->hskew, diff_time);
				if (last_fps == 60) {
					if (diff_time < 10500) {
						usleep_range(10500 - diff_time, 10600 - diff_time);
					}
				} else if (last_fps == 120 && last_mode->hskew != 2) {
					if (diff_time > 1300 && diff_time < 3300) {
						usleep_range(3300 - diff_time, 3400 - diff_time);
					} else if (diff_time > 9300 && diff_time < 11600) {
						usleep_range(11600 - diff_time, 11700 - diff_time);
					}
				} else if (last_fps == 90) {
					if (diff_time > 6500 && diff_time < 8500) {
						usleep_range(8500 - diff_time, 8600 - diff_time);
					}
				}
			} else if (cur_fps == 60) {
				cur_time = ktime_get();
				diff_time = (cur_time - oplus_last_te_time) / 1000;
				if (diff_time < 10500) {
					usleep_range(10500 - diff_time, 10600 - diff_time);
				}
				pr_info("%s handle is null, diff_time = %d\n", __func__, diff_time);
			} else {
				pr_info("%s handle is null\n", __func__);
			}
		}
	} else {
		DDPINFO("%s isn't panel cmdq package\n", __func__);
	}
}
EXPORT_SYMBOL(oplus_pcp_handle);

int oplus_pcp_lock_clear(void)
{
	wake_up_interruptible(&oplus_pcp_lock_clear_wq);
	return 0;
}
EXPORT_SYMBOL(oplus_pcp_lock_clear);

int oplus_pcp_lock_wait_clear_thread(void *data)
{
	while(1) {
		wait_event_interruptible(
			oplus_pcp_lock_clear_wq
			, atomic_read(&oplus_pcp_handle_lock));

		usleep_range(10400, 10500);

		DDPINFO("%s atommic ++ %d\n", __func__, atomic_read(&oplus_pcp_handle_lock));
		atomic_dec(&oplus_pcp_handle_lock);
		DDPINFO("%s atommic -- %d\n", __func__, atomic_read(&oplus_pcp_handle_lock));
		mutex_unlock(&oplus_pcp_lock);
		atomic_dec(&oplus_pcp_num);
		oplus_disp_trace_end("M_LOCK_PCP");
		pr_info("%s oplus_pcp_unlock\n", __func__);
	}
}
EXPORT_SYMBOL(oplus_pcp_lock_wait_clear_thread);

int oplus_mtk_drm_setbacklight(struct drm_crtc *crtc, unsigned int level)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);

	int index = drm_crtc_index(crtc);

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

	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);

	/* set backlight */
	if (comp->funcs && comp->funcs->io_cmd)
		comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &level);

	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);

	CRTC_MMP_EVENT_END(index, backlight, (unsigned long)crtc,
			level);

	return 0;
}
EXPORT_SYMBOL(oplus_mtk_drm_setbacklight);

int oplus_mtk_dc_backlight_exit(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	int index = drm_crtc_index(crtc);

	CRTC_MMP_EVENT_START(index, backlight, (unsigned long)crtc,
			0);
	if (!(mtk_crtc->enabled)) {
		DDPINFO("Sleep State set backlight stop --crtc not ebable\n");
		mutex_unlock(&mtk_crtc->lock);
		CRTC_MMP_EVENT_END(index, backlight, 0, 0);
		return -EINVAL;
	}
	if (!comp) {
		DDPINFO("%s no output comp\n", __func__);
		mutex_unlock(&mtk_crtc->lock);
		CRTC_MMP_EVENT_END(index, backlight, 0, 1);

		return -EINVAL;
	}

	mtk_drm_send_lcm_cmd_prepare(crtc, &cmdq_handle);

	/* exit dc */
	if (comp->funcs && comp->funcs->io_cmd)
		comp->funcs->io_cmd(comp, cmdq_handle, DC_POST_EXIT, NULL);

	mtk_drm_send_lcm_cmd_flush(crtc, &cmdq_handle, 0);

	CRTC_MMP_EVENT_END(index, backlight, (unsigned long)crtc,
			0);

	return 0;
}
EXPORT_SYMBOL(oplus_mtk_dc_backlight_exit);
