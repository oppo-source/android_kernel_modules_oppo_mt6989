
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <video/mipi_display.h>
#include <drm/drm_modes.h>

#include "mtk_drm_mmp.h"
#include "drm_internal.h"

#include "pw_iris_def.h"
#include "pw_iris_ioctl.h"
#include "pw_iris_pq.h"
#include "dsi_iris_api.h"
#include "dsi_iris_mtk_api.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_cmpt.h"

int iris_dsi_get_mode(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	return mtk_dsi_is_cmd_mode(pcfg_ven->mtk_comp)
		? IRIS_CMD_MODE: IRIS_VIDEO_MODE;
}

bool iris_is_curmode_cmd_mode(void)
{
	return iris_dsi_get_mode() == IRIS_CMD_MODE;
}

bool iris_is_curmode_vid_mode(void)
{
	return iris_dsi_get_mode() == IRIS_VIDEO_MODE;
}

int iris_switch_cmd_type(int type)
{
	return type;
}

int iris_drm_operate_conf(void *argp)
{
	if (argp)
		return iris_operate_conf(argp);

	return -EINVAL;
}

int iris_drm_kickoff(bool is_secondary)
{
	return iris_kickoff(is_secondary);
}
