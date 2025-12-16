/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2019 MediaTek Inc. */

#ifndef __ADAPTOR_HW_H__
#define __ADAPTOR_HW_H__

#if defined(OPLUS_FEATURE_CAMERA_COMMON) && defined(CONFIG_OPLUS_CAM_EVENT_REPORT_MODULE)
#include "oplus/inc/oplus_cam_olc_exception.h"
#endif /* OPLUS_FEATURE_CAMERA_COMMON */

int adaptor_hw_power_on(struct adaptor_ctx *ctx);
int adaptor_hw_power_off(struct adaptor_ctx *ctx);
int adaptor_hw_init(struct adaptor_ctx *ctx);
int adaptor_hw_sensor_reset(struct adaptor_ctx *ctx);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
int adaptor_hw_vsvoter_parse(struct adaptor_ctx *ctx);
void adaptor_hw_vsvoter_set(struct adaptor_ctx *ctx);
void adaptor_hw_vsvoter_clr(struct adaptor_ctx *ctx);
#endif /* OPLUS_FEATURE_CAMERA_COMMON */
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#define VLPCFG_RSV_REQ_STA (0x1C000500)
#define VLPCFG_RSV_REQ_SET (0x1C000504)
#define VLPCFG_RSV_REQ_CLR (0x1C000508)

#define SPM_SW_RSV_3       (0x1C00161C)
#endif /* OPLUS_FEATURE_CAMERA_COMMON */
#ifdef OPLUS_FEATURE_CAMERA_COMMON
/* pmic vs voter */
 /* PMIC Voter offset */
#define VS_VOTER_EN_LO 0x0
#define VS_VOTER_EN_LO_SET 0x1
#define VS_VOTER_EN_LO_CLR 0x2
struct hw_vsvoter {
	struct device *dev;
	struct regmap *vsv;
	u32 vsv_reg;
	u32 vsv_mask;
	u32 vsv_vers;
};
#endif /* OPLUS_FEATURE_CAMERA_COMMON */
#endif
