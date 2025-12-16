#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <drm/drm_vblank.h>

#include "mtk_drm_mmp.h"
#include "mtk_drm_drv.h"
#include "mtk_drm_crtc.h"
#include "drm_internal.h"
#include "mtk_dsi.h"
#include "mtk_drm_helper.h"

#include "dsi_iris_api.h"
#include "pw_iris_log.h"
#include "pw_iris_lp.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_lightup_ocp.h"
#include "dsi_iris_mtk_api.h"

#if 0
void iris_ddp_mutex_lock(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct drm_crtc *crtc1 = pcfg_ven->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc1);

	//IRIS_LOGD("%s:%d wait vblank+\n", __func__, __LINE__);
	//drm_wait_one_vblank(pcfg_ven->drm, 0);
	//IRIS_LOGD("%s:%d wait vblank-\n", __func__, __LINE__);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	mtk_drm_set_idlemgr(crtc1, 0, 0);
}

void iris_ddp_mutex_unlock(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct drm_crtc *crtc1 = pcfg_ven->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc1);

	mtk_drm_set_idlemgr(crtc1, 1, 0);
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
}
#endif

void iris_send_cmdq_cmds_pre(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	iris_dsi_pre_cmd(pcfg_ven->mtk_comp, pcfg_ven->crtc);
	DDPINFO("%s:%d, default pre-command\n", __func__, __LINE__);
}

void iris_send_cmdq_cmds_post(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	iris_dsi_pos_cmd(pcfg_ven->mtk_comp, pcfg_ven->crtc);
	DDPINFO("%s:%d, default post-command\n", __func__, __LINE__);
}


bool iris_main_dsi(struct mtk_ddp_comp *comp)
{
	if (comp && (comp->id == DDP_COMPONENT_DSI0))
		return true;

	return false;
}
bool iris_aux_dsi(struct mtk_ddp_comp *comp)
{
	if (comp && (comp->id == DDP_COMPONENT_DSI1))
		return true;

	return false;
}

bool iris_virtual_connector(struct drm_connector *c)
{
	struct mtk_dsi *dsi;
	if (c == NULL) {
		IRIS_LOGE("drm_connector is NULL");
		return false;
	}
	dsi = container_of(c, struct mtk_dsi, conn);
	if (dsi == NULL) {
		IRIS_LOGE("dsi is NULL");
		return false;
	}
	return (dsi->ddp_comp.id != DDP_COMPONENT_DSI0);
}
#if 0
bool iris_is_crtc_enabled(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	if (pcfg_ven && pcfg_ven->crtc && to_mtk_crtc(pcfg_ven->crtc))
		return to_mtk_crtc(pcfg_ven->crtc)->enabled;
	else
		return false;
}
#endif
int iris_mtk_send_panel_cmd(struct mipi_dsi_msg *msg)
{
	int rc = 0;
	struct iris_cmd_set cmdset;
	struct iris_cmd_desc desc[1];

	memset(&cmdset, 0x00, sizeof(cmdset));
	IRIS_LOGI("%s(), %d\n", __func__, __LINE__);

	cmdset.cmds = desc;
	cmdset.cmds[0].msg.channel = msg->channel;
	cmdset.cmds[0].msg.flags = msg->flags;
	cmdset.cmds[0].msg.type = msg->type;
	cmdset.cmds[0].msg.tx_len = msg->tx_len;
	cmdset.cmds[0].msg.tx_buf = msg->tx_buf;
	cmdset.cmds[0].msg.rx_len = msg->rx_len;
	cmdset.cmds[0].msg.rx_buf = msg->rx_buf;
	cmdset.count = 1;
	if (msg->flags & MIPI_DSI_MSG_USE_LPM)
		cmdset.state = IRIS_CMD_SET_STATE_LP;
	else
		cmdset.state = IRIS_CMD_SET_STATE_HS;

	rc = iris_pt_send_panel_cmd(&cmdset);

	return rc;
}

void iris_set_idlemgr(unsigned int crtc_id, unsigned int enable, bool need_lock)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct mtk_drm_private *private;
	struct drm_crtc *crtc;

	if (IS_ERR_OR_NULL(pcfg_ven->drm)) {
		IRIS_LOGE("%s:%d, drm_dev is NULL\n",
			__func__, __LINE__);
		return;
	}
	if (IS_ERR_OR_NULL(pcfg_ven->drm->dev_private)) {
		IRIS_LOGE("%s:%d, drm_dev->dev_private is NULL\n",
			__func__, __LINE__);
		return;
	}
	private = pcfg_ven->drm->dev_private;
	crtc = private->crtc[crtc_id];
	if (crtc == NULL) {
		IRIS_LOGE("%s:%d, crtc: %d is NULL\n",
			__func__, __LINE__, crtc_id);
		return;
	}
	mtk_drm_set_idlemgr(crtc, enable, need_lock);
	IRIS_LOGI("%s, crtc: %d, enable: %d, need_lock: %d",
		__func__, crtc_id, enable, need_lock);
}

unsigned long long iris_set_idle_check_interval(unsigned int crtc_id, unsigned long long new_interval)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct mtk_drm_private *private;
	struct drm_crtc *crtc;
	unsigned long long old_interval = 0;

	if (IS_ERR_OR_NULL(pcfg_ven->drm)) {
		IRIS_LOGE("%s:%d, drm_dev is NULL\n",
			__func__, __LINE__);
		return 0;
	}
	if (IS_ERR_OR_NULL(pcfg_ven->drm->dev_private)) {
		IRIS_LOGE("%s:%d, drm_dev->dev_private is NULL\n",
			__func__, __LINE__);
		return 0;
	}
	private = pcfg_ven->drm->dev_private;
	crtc = private->crtc[crtc_id];
	if (crtc == NULL) {
		IRIS_LOGE("%s:%d, crtc: %d is NULL\n",
			__func__, __LINE__, crtc_id);
		return 0;
	}
	old_interval = mtk_drm_set_idle_check_interval(crtc, new_interval);
	IRIS_LOGI("%s, crtc: %d, old_interval: %llu, new_interval: %llu",
		__func__, crtc_id, old_interval, new_interval);
	return old_interval;
}
