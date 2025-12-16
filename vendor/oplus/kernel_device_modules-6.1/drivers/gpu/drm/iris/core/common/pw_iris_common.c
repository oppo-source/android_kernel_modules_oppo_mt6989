// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2023, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2023.
 */

#include <linux/of_device.h>

#define CREATE_TRACE_POINTS
#include "pw_iris_api.h"
#include "pw_iris_log.h"
#include "pw_iris_def.h"

static bool iris_chip_enable;
static bool soft_iris_enable;
static bool iris_dual_enable;
static bool iris_graphic_memc_enable;
static struct iris_chip_caps iris_chip_capabilities;


static struct device_node *iris_find_pxlw_node(void)
{
	char *config = "pxlw,iris";
	struct device_node *pxlw_node = NULL;
	struct device_node *chosen_node = NULL;

	IRIS_LOGI("%s(), search pxlw,iris node", __func__);
	chosen_node = of_find_node_by_path("/chosen");
	if (!chosen_node) {
		IRIS_LOGE("chosen node is null");
		return NULL;
	}

	pxlw_node = of_find_node_by_name(chosen_node, config);
	if (!pxlw_node)
		IRIS_LOGE("%s(), failed to find %s node", __func__, config);

	return pxlw_node;
}

void iris_query_capability(void)
{
	bool chip_enable = false;
	u32 chip_caps_element[IRIS_CAPS_ELEMENT_MAX] = {0};
	int chip_caps_element_num = 0;
	int rc = 0;
	bool support_dual_memc = false;
	bool support_graphic_memc = false;
	union iris_chip_basic_caps caps;
	int i = 0;
	int dtsi_idx = 0;
	struct device_node *pxlw_iris_node = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	caps.val = 0;
	iris_chip_enable = false;
	soft_iris_enable = false;
	memset(&iris_chip_capabilities, 0, sizeof(struct iris_chip_caps));

	if (!pcfg->pdev) {
		IRIS_LOGW("%s(), pdev is null, will search pxlw node!", __func__);
		pxlw_iris_node = iris_find_pxlw_node();
		chip_caps_element_num  = of_property_count_u32_elems(pxlw_iris_node, "pxlw,iris-capability");
	} else {
		chip_caps_element_num  = of_property_count_u32_elems(pcfg->pdev->dev.of_node, "pxlw,iris-capability");
	}
	if ((chip_caps_element_num < IRIS_CAPS_ELEMENT_MIN) || (chip_caps_element_num > IRIS_CAPS_ELEMENT_MAX)) {
		IRIS_LOGE("%s(), illegal iris capability, element num is %d",
			__func__, chip_caps_element_num);
		return;
	} else {
		IRIS_LOGI("%s(), legal iris chip capability, element num is %d",
			__func__, chip_caps_element_num);
	}

	if (!pcfg->pdev) {
		rc = of_property_read_u32_array(pxlw_iris_node, "pxlw,iris-capability",
				chip_caps_element, chip_caps_element_num);
	} else {
		rc = of_property_read_u32_array(pcfg->pdev->dev.of_node, "pxlw,iris-capability",
				chip_caps_element, chip_caps_element_num);
	}
	if (rc) {
		IRIS_LOGE("%s(), failed to parse pxlw,iris-capability, rc = %d\n", __func__, rc);
		return;
	}

	chip_enable          = chip_caps_element[0];
	caps.version_is_new  = chip_caps_element[1];
	caps.version_number  = chip_caps_element[2];
	caps.feature_enabled = chip_caps_element[3];
	caps.asic_type       = chip_caps_element[4];

	iris_chip_enable = chip_enable;
	if (caps.feature_enabled & (1 << (SUPPORT_SOFT_IRIS - 16)))
		soft_iris_enable = true;
	IRIS_LOGI("%s(), iris chip enable: %s, soft iris enable: %s",
		__func__,
		iris_chip_enable ? "true" : "false",
		soft_iris_enable ? "true" : "false");

	support_dual_memc = caps.feature_enabled & (1 << (SUPPORT_DUAL_MEMC - 16));

	if (iris_chip_enable && support_dual_memc)
		iris_dual_enable = true;

	iris_chip_capabilities.caps = caps.val;
	IRIS_LOGI("%s(), %s%d, platform:%s, version:%d, feature: %#x",
		__func__,
		caps.version_is_new ?  "iris" : "legacy(<7)", caps.version_number,
		caps.asic_type ? "ASIC" : "FPGA", caps.asic_type - 1,
		caps.feature_enabled);
	if (caps.feature_enabled & (1 << (SUPPORT_MORE - 16))) {
		IRIS_LOGI("%s(), SUPPORT_MORE is enabled in dtsi.", __func__);
		for (i = 0; i < IRIS_FEATURES_MAX; i++) {
			dtsi_idx = i + IRIS_CAPS_ELEMENT_MIN;
			if ((dtsi_idx < chip_caps_element_num)) {
				iris_chip_capabilities.features[i] = chip_caps_element[dtsi_idx];
				IRIS_LOGI("%s(), from dtsi extend feature %d : 0x%08x", __func__, i, iris_chip_capabilities.features[i]);
			} else {
				iris_chip_capabilities.features[i] = 0;
				IRIS_LOGD("%s(), from dtsi extend feature %d is 0", __func__, i);
			}
		}

		support_graphic_memc = iris_chip_capabilities.features[0] & (1 << SUPPORT_GMEMC);
		if (iris_chip_enable && support_graphic_memc)
			iris_graphic_memc_enable = true;
	} else {
		IRIS_LOGI("%s(), SUPPORT_MORE is disabled in dtsi.", __func__);
	}
	iris_memc_func_init();
	IRIS_LOGI("%s(), from dtsi basic feature : 0x%08x", __func__, iris_chip_capabilities.caps);
	return;
}

bool iris_is_chip_supported(void)
{
	return iris_chip_enable;
}
EXPORT_SYMBOL(iris_is_chip_supported);

bool iris_is_softiris_supported(void)
{
	return soft_iris_enable;
}

struct iris_chip_caps iris_get_chip_caps(void)
{
	return iris_chip_capabilities;
}

bool iris_is_dual_supported(void)
{
	return iris_chip_enable && iris_dual_enable;
}

bool iris_is_graphic_memc_supported(void)
{
	return iris_chip_enable && iris_graphic_memc_enable;
}

enum iris_chip_type iris_get_chip_type(void)
{
	struct iris_chip_caps chip_capabilities;
	enum iris_chip_type rc = CHIP_UNKNOWN;

	chip_capabilities = iris_get_chip_caps();
	switch ((chip_capabilities.caps >> 8) & 0xFF) {
	case 0x5:
		rc = CHIP_IRIS5;
		break;
	case 0x6:
		rc = CHIP_IRIS6;
		break;
	case 0x7:
		if (((chip_capabilities.caps >> 30) & 0x3) > 1)
			rc = CHIP_IRIS7P;
		else
			rc = CHIP_IRIS7;
		break;
	case 0x8:
		rc = CHIP_IRIS8;
		break;
	default:
		rc = CHIP_UNKNOWN;
		IRIS_LOGE("%s(), chip type is unknown. chip_basic_capabilities: 0x%x", __func__, chip_capabilities.caps);
		break;
	}

	return rc;
}

bool iris_need_short_read_workaround(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if ((pcfg->panel_name != NULL) &&
			!strcmp(pcfg->panel_name, "HX8379A fwvga video mode dsi panel v2 27Mhz 2lanes"))
		return true;
	else
		return false;
}

int iris_need_update_pps_one_time(void)
{
	int rc = 0;
	static int flag = 1;

	if (flag == 1 && iris_get_chip_type() == CHIP_IRIS7P) {
		rc = 1;
		flag = 0;
		IRIS_LOGI("Iris force update pps for one time!");
	}
	return rc;
}
