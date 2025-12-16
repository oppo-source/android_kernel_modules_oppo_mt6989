#ifndef _IRIS_MTK_API_
#define _IRIS_MTK_API_

#include <linux/types.h>

struct cmdq_pkt;
struct drm_panel;
struct mtk_panel_ext;
struct mtk_ddp_comp;
struct drm_crtc;
struct mipi_dsi_msg;
struct msm_iris_operate_value;
struct mipi_dsi_msg;
struct drm_connector;
struct mtk_drm_crtc;
struct mtk_ddic_dsi_msg;

struct iris_mtk_dsi_op {
	void (*transfer)(struct mtk_ddp_comp *comp, struct cmdq_pkt *handle,
	void *data, unsigned int len, int state);
	int (*transfer_rdcmd)(struct mtk_ddp_comp *comp, void *handle,
		struct mipi_dsi_msg *msg, unsigned int slot_index);
	int (*obtain_rdvalue)(struct mtk_ddp_comp *comp, struct mipi_dsi_msg *msg,
		unsigned int slot_index);
	void (*vdo2cmd_cb)(struct cmdq_pkt *handle,
		struct mtk_ddp_comp *comp,
		const void *data, size_t len, u32 flag);
	};

int iris_drm_operate_conf(void *argp);
//int iris_operate_tool(struct msm_iris_operate_value *argp);

void iris_set_mtk_dsi_op(struct iris_mtk_dsi_op *op);
void iris_dsi_pre_cmd(struct mtk_ddp_comp *comp, struct drm_crtc *crtc);
void iris_dsi_pos_cmd(struct mtk_ddp_comp *comp, struct drm_crtc *crtc);

void iris_prepare(void);
u8 iris_get_cmd_type(u8 cmd, u32 count);
void iris_init(struct drm_panel *panel, struct mtk_panel_ext *panel_ext, struct drm_connector *conn);
void iris_deinit(struct drm_connector *conn);
int iris_drm_kickoff(bool is_secondary);
int iris_prepare_for_kickoff(struct mtk_ddp_comp *comp);
//bool iris_is_dual_supported(void);
int iris_switch_mtk(void *handle, void *dev, int id);
int iris_switch_fps_with_hfp(void *handle, void *dev, int id);
int iris_switch_fps_with_vfp(void *handle, void *dev, int id);
//void iris_ddp_mutex_lock(void);
//void iris_ddp_mutex_unlock(void);
int iris_parse_param(void *dev);
//bool iris_main_dsi(struct mtk_ddp_comp *comp);
//bool iris_aux_dsi(struct mtk_ddp_comp *comp);
//bool iris_virtual_connector(struct drm_connector *c);
//bool iris_is_crtc_enabled(void);
int iris_mtk_send_panel_cmd(struct mipi_dsi_msg *msg);
//void iris_set_idlemgr(unsigned int crtc_id, unsigned int enable, bool need_lock);
//unsigned long long iris_set_idle_check_interval(unsigned int crtc_id, unsigned long long new_interval);
//int iris_mtk_disp_set_metadata(uint32_t value);
int iris_send_ddic_cmd(struct cmdq_pkt *handle,
               struct mtk_drm_crtc *crtc,
               struct mtk_ddic_dsi_msg *cmd_msg);

int iris_read_status(int i, unsigned char cmd);
u32 iris_get_panel_esd_state(int i);
void iris_set_esd_check_ongoing(bool status);
bool iris_get_esd_check_ongoing(void);
void iris_set_esd_check_num(u32 n);
int iris_vdo_mode_send_panel_cmd(struct cmdq_pkt *handle, u8 *data, int len);
#endif
