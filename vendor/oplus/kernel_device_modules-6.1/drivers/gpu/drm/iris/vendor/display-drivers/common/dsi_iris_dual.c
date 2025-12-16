// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <drm/display/drm_dsc.h>
#include "mtk_panel_ext.h"
#include "dsi_iris_lightup.h"
#include "dsi_iris_dual.h"
#include "pw_iris_api.h"
#include "pw_iris_log.h"

static irqreturn_t iris_osd_irq_handler_i7(int irq, void *data)
{
	iris_inc_osd_irq_cnt();
	return IRQ_HANDLED;
}

void iris_register_osd_irq_ext_i7(void *disp)
{
	int rc = 0;
	int osd_gpio = -1;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;

	if (!iris_is_dual_supported())
		return;

	if (dsi) {
		struct device *dev = &dsi->dev;
		if (dev) {
			struct device_node *np = dev->of_node;
			struct property *prop = of_find_property(np, "iris-abyp-ready-gpios", NULL);
			if (prop && prop->length) {
				pcfg->iris_osd_gpio = of_get_named_gpio_flags(np,
					"iris-abyp-ready-gpios", 0, NULL);
				IRIS_LOGI("osd status gpio is = %d", pcfg->iris_osd_gpio);
			} else {
				pcfg->iris_osd_gpio = -1;
				IRIS_LOGI("cannot find osd status gpio");
				return;
			}
		}
	}

	osd_gpio = pcfg->iris_osd_gpio;
	if (!gpio_is_valid(osd_gpio)) {
		IRIS_LOGE("%s(%d), osd status gpio not specified",
				__func__, __LINE__);
		return;
	}

	gpio_direction_input(osd_gpio);
	IRIS_LOGI("%s, irq: %d", __func__, gpio_to_irq(osd_gpio));

	// cat /proc/interrupts | grep OSD
	rc = devm_request_irq(&dsi->dev, gpio_to_irq(osd_gpio), iris_osd_irq_handler_i7,
			IRQF_TRIGGER_RISING, "IRIS_OSD_GPIO", NULL);
	if (rc) {
		IRIS_LOGE("%s(), IRIS OSD load irq failed", __func__);
		return;
	}
	disable_irq(gpio_to_irq(osd_gpio));
}

static irqreturn_t iris_osd_irq_handler_i8(int irq, void *data)
{
	iris_inc_osd_irq_cnt();
	return IRQ_HANDLED;
}

void iris_register_osd_irq_ext_i8(void *disp)
{
	int rc = 0;
	int osd_gpio = -1;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct mipi_dsi_device *dsi = pcfg->dsi_dev;

	if (!iris_is_dual_supported())
		return;

	if (dsi) {
		struct device *dev = &dsi->dev;

		if (dev) {
			struct device_node *np = dev->of_node;
			struct property *prop = of_find_property(np, "iris-abyp-ready-gpios", NULL);

			if (prop && prop->length) {
				pcfg->iris_osd_gpio = of_get_named_gpio_flags(np,
					"iris-abyp-ready-gpios", 0, NULL);
				IRIS_LOGI("osd status gpio is = %d", pcfg->iris_osd_gpio);
			} else {
				pcfg->iris_osd_gpio = -1;
				IRIS_LOGI("cannot find osd status gpio");
				return;
			}
		}
	}

	osd_gpio = pcfg->iris_osd_gpio;
	if (!gpio_is_valid(osd_gpio)) {
		IRIS_LOGE("%s(%d), osd status gpio not specified",
				__func__, __LINE__);
		return;
	}

	gpio_direction_input(osd_gpio);
	IRIS_LOGI("%s, irq: %d", __func__, gpio_to_irq(osd_gpio));

	// cat /proc/interrupts | grep OSD
	rc = devm_request_irq(&dsi->dev, gpio_to_irq(osd_gpio), iris_osd_irq_handler_i8,
			IRQF_TRIGGER_RISING, "IRIS_OSD_GPIO", NULL);
	if (rc) {
		IRIS_LOGE("%s(), IRIS OSD load irq failed", __func__);
		return;
	}
	disable_irq(gpio_to_irq(osd_gpio));
}

int iris_create_pps_buf_cmd(char *buf, int pps_id, u32 len, bool is_secondary)
{
	struct mtk_panel_dsc_params *dsc = NULL;
	char *bp = buf;
	char data;
	u32 i, bpp;
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	static unsigned int mtk_rc_buf_thresh_default[][DSC_NUM_BUF_RANGES - 1] = {
		{0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b, 0x7d, 0x7e},
		{0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b, 0x7d, 0x7e},
		};
	static unsigned int mtk_range_min_qp_default[][DSC_NUM_BUF_RANGES] = {
		{0, 4, 5, 5, 7, 7, 7, 7, 7, 7, 9, 9, 9, 13, 16},
		{0, 0, 1, 1, 3, 3, 3, 3, 3, 3, 5, 5, 5, 7, 13},
		};
	static unsigned int mtk_range_max_qp_default[][DSC_NUM_BUF_RANGES] = {
		{8, 8, 9, 10, 11, 11, 11, 12, 13, 14, 14, 15, 15, 16, 17},
		{4, 4, 5, 6, 7, 7, 7, 8, 9, 10, 11, 12, 13, 13, 15},
		};
	static int mtk_range_bpg_ofs_default[][DSC_NUM_BUF_RANGES] = {
		{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12},
		{2, 0, 0, -2, -4, -6, -8, -8, -8, -10, -10, -12, -12, -12, -12},
		};

	if (len < IRIS_DSC_PPS_SIZE || !buf || !pcfg_ven)
		return -EINVAL;

	if (is_secondary) {
		if (!pcfg_ven->mtk_panel_ext_2nd || !pcfg_ven->mtk_panel_ext_2nd->params) {
			IRIS_LOGE("%s(), aux invalid params!", __func__);
			return -EINVAL;
		}
		dsc = &pcfg_ven->mtk_panel_ext_2nd->params->dsc_params;
	} else {
		if (!pcfg_ven->panel_ext || !pcfg_ven->panel_ext->params) {
			IRIS_LOGE("%s(), invalid params!", __func__);
			return -EINVAL;
		}
		dsc = &pcfg_ven->panel_ext->params->dsc_params;
	}

	memset(buf, 0, len);
	/* pps0 */
	*bp++ = dsc->ver;
	*bp++ = (pps_id & 0xff);		/* pps1 */
	bp++;					/* pps2, reserved */

	data = dsc->dsc_line_buf_depth & 0x0f;
	data |= ((dsc->bit_per_channel & 0xf) << DSC_PPS_BPC_SHIFT);
	*bp++ = data;				/* pps3 */

	bpp = dsc->bit_per_pixel;
	data = (bpp >> DSC_PPS_MSB_SHIFT);
	data &= 0x03;				/* upper two bits */
	data |= ((dsc->bp_enable & 0x1) << 5);
	data |= ((dsc->rct_on & 0x1) << 4);
	#if 0 //TODO
	data |= ((dsc->simple_422 & 0x1) << 3);
	data |= ((dsc->vbr_enable & 0x1) << 2);
	#endif
	*bp++ = data;				/* pps4 */

	*bp++ = (bpp & DSC_PPS_LSB_MASK);	/* pps5 */

	*bp++ = ((dsc->pic_height >> 8) & 0xff); /* pps6 */
	*bp++ = (dsc->pic_height & 0x0ff);	/* pps7 */
	*bp++ = ((dsc->pic_width >> 8) & 0xff);	/* pps8 */
	*bp++ = (dsc->pic_width & 0x0ff);	/* pps9 */

	*bp++ = ((dsc->slice_height >> 8) & 0xff);/* pps10 */
	*bp++ = (dsc->slice_height & 0x0ff);	/* pps11 */
	*bp++ = ((dsc->slice_width >> 8) & 0xff); /* pps12 */
	*bp++ = (dsc->slice_width & 0x0ff);	/* pps13 */

	*bp++ = ((dsc->chunk_size >> 8) & 0xff);/* pps14 */
	*bp++ = (dsc->chunk_size & 0x0ff);	/* pps15 */

	*bp++ = (dsc->xmit_delay >> 8) & 0x3; /* pps16 */
	*bp++ = (dsc->xmit_delay & 0xff);/* pps17 */

	*bp++ = ((dsc->dec_delay >> 8) & 0xff); /* pps18 */
	*bp++ = (dsc->dec_delay & 0xff);/* pps19 */

	bp++;				/* pps20, reserved */

	*bp++ = (dsc->scale_value & 0x3f); /* pps21 */

	*bp++ = ((dsc->increment_interval >> 8) & 0xff); /* pps22 */
	*bp++ = (dsc->increment_interval & 0xff); /* pps23 */

	*bp++ = ((dsc->decrement_interval >> 8) & 0xf); /* pps24 */
	*bp++ = (dsc->decrement_interval & 0x0ff);/* pps25 */

	bp++;					/* pps26, reserved */

	*bp++ = (dsc->line_bpg_offset & 0x1f);/* pps27 */

	*bp++ = ((dsc->nfl_bpg_offset >> 8) & 0xff);/* pps28 */
	*bp++ = (dsc->nfl_bpg_offset & 0x0ff);	/* pps29 */
	*bp++ = ((dsc->slice_bpg_offset >> 8) & 0xff);/* pps30 */
	*bp++ = (dsc->slice_bpg_offset & 0x0ff);/* pps31 */

	*bp++ = ((dsc->initial_offset >> 8) & 0xff);/* pps32 */
	*bp++ = (dsc->initial_offset & 0x0ff);	/* pps33 */

	*bp++ = ((dsc->final_offset >> 8) & 0xff);/* pps34 */
	*bp++ = (dsc->final_offset & 0x0ff);	/* pps35 */

	*bp++ = (dsc->flatness_minqp & 0x1f);	/* pps36 */
	*bp++ = (dsc->flatness_maxqp & 0x1f);	/* pps37 */

	*bp++ = ((dsc->rc_model_size >> 8) & 0xff);/* pps38 */
	*bp++ = (dsc->rc_model_size & 0x0ff);	/* pps39 */

	*bp++ = (dsc->rc_edge_factor & 0x0f);	/* pps40 */

	*bp++ = (dsc->rc_quant_incr_limit0 & 0x1f);	/* pps41 */
	*bp++ = (dsc->rc_quant_incr_limit1 & 0x1f);	/* pps42 */

	data = ((dsc->rc_tgt_offset_hi & 0xf) << 4);
	data |= (dsc->rc_tgt_offset_lo & 0x0f);
	*bp++ = data;				/* pps43 */

	if (dsc->ext_pps_cfg.enable && dsc->ext_pps_cfg.rc_buf_thresh) {
		for (i = 0; i < DSC_NUM_BUF_RANGES - 1; i++)
			*bp++ = ((dsc->ext_pps_cfg.rc_buf_thresh[i] >> 6) & 0xff); /* pps44 - pps57 */
	} else {
		int index = (dsc->bit_per_channel == 10) ? 0 : 1;

		for (i = 0; i < DSC_NUM_BUF_RANGES - 1; i++)
			*bp++ = (mtk_rc_buf_thresh_default[index][i] & 0xff); /* pps44 - pps57 */
	}

	if (dsc->ext_pps_cfg.enable && dsc->ext_pps_cfg.range_min_qp
			&& dsc->ext_pps_cfg.range_max_qp && dsc->ext_pps_cfg.range_bpg_ofs) {
		for (i = 0; i < DSC_NUM_BUF_RANGES; i++) {
			/* pps58 - pps87 */
			data = (dsc->ext_pps_cfg.range_min_qp[i] & 0x1f);
			data <<= 3;
			data |= ((dsc->ext_pps_cfg.range_max_qp[i] >> 2) & 0x07);
			*bp++ = data;
			data = (dsc->ext_pps_cfg.range_max_qp[i] & 0x03);
			data <<= 6;
			data |= (dsc->ext_pps_cfg.range_bpg_ofs[i] & 0x3f);
			*bp++ = data;
		}
	} else {
		/* pps58 - pps87 */
		int index = (dsc->bit_per_channel == 10) ? 0 : 1;

		for (i = 0; i < DSC_NUM_BUF_RANGES; i++) {
			data = (mtk_range_min_qp_default[index][i] & 0x1f);
			data <<= 3;
			data |= ((mtk_range_max_qp_default[index][i] >> 2) & 0x07);
			*bp++ = data;
			data = (mtk_range_max_qp_default[index][i] & 0x03);
			data <<= 6;
			data |= (mtk_range_bpg_ofs_default[index][i] & 0x3f);
			*bp++ = data;
		}
	}
#if 0 //TODO
	if ((dsc->ver & 0x0f) == 0x2) {
		if (dsc->native_422)
			data = BIT(0);
		else if (dsc->native_420)
			data = BIT(1);
		*bp++ = data;				/* pps88 */
		*bp++ = dsc->second_line_bpg_offset;	/* pps89 */

		*bp++ = ((dsc->nsl_bpg_offset >> 8) & 0xff);/* pps90 */
		*bp++ = (dsc->nsl_bpg_offset & 0x0ff);	/* pps91 */

		*bp++ = ((dsc->second_line_offset_adj >> 8) & 0xff); /* pps92*/
		*bp++ = (dsc->second_line_offset_adj & 0x0ff);	/* pps93 */

		/* rest bytes are reserved and set to 0 */
	}
#endif
	return 0;
}

bool iris_aux_panel_initialized(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();
	struct iris_cfg *pcfg = iris_get_cfg();

	if (unlikely(!pcfg || !pcfg_ven || !pcfg_ven->mtk_panel_ext_2nd)) {
		IRIS_LOGE("%s(), No secondary panel configured!", __func__);
		return false;
	}

	return pcfg->ap_mipi1_power_st;
}

bool iris_aux_panel_existed(void)
{
	struct iris_vendor_cfg *pcfg_ven = iris_get_vendor_cfg();

	if (unlikely(!pcfg_ven || !pcfg_ven->mtk_panel_ext_2nd)) {
		IRIS_LOGE("%s(), No secondary panel configured!", __func__);
		return false;
	}

	return true;
}
