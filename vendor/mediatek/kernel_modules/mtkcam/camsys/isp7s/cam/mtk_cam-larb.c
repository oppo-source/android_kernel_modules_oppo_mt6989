// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Tiffany Lin <tiffany.lin@mediatek.com>
 *
 */

#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <soc/mediatek/smi.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/component.h>

#include "mtk_cam.h"
#include "mtk_cam-larb.h"

struct mtk_cam_larb_device {
	unsigned int	larb_id;
	struct device	*dev;
};

int mtk_cam_larb_id(struct device *dev)
{
	struct mtk_cam_larb_device *larb_dev = dev_get_drvdata(dev);

	return larb_dev->larb_id;
}

static int mtk_cam_pm_component_bind(struct device *dev, struct device *master,
				  void *data)
{
	struct mtk_cam_larb_device *larb_dev = dev_get_drvdata(dev);
	struct mtk_cam_device *cam_dev = data;

	dev_info(dev, "%s: id=%d\n", __func__, larb_dev->larb_id);
	return mtk_cam_set_dev_larb(cam_dev->dev, dev);
}

static void mtk_cam_pm_component_unbind(struct device *dev, struct device *master,
				     void *data)
{
}

static const struct component_ops mtk_cam_pm_component_ops = {
	.bind = mtk_cam_pm_component_bind,
	.unbind = mtk_cam_pm_component_unbind,
};

static int mtk_cam_larb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_cam_larb_device   *larb_dev;
	int ret;

	larb_dev = devm_kzalloc(dev, sizeof(*dev), GFP_KERNEL);
	if (!larb_dev)
		return -ENOMEM;
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34)))
		dev_info(dev, "%s: No suitable DMA available\n", __func__);

	if (!dev->dma_parms) {
		dev->dma_parms =
			devm_kzalloc(dev, sizeof(*dev->dma_parms), GFP_KERNEL);
		if (!dev->dma_parms)
			return -ENOMEM;
	}

	if (dev->dma_parms) {
		ret = dma_set_max_seg_size(dev, UINT_MAX);
		if (ret)
			dev_info(dev, "Failed to set DMA segment size\n");
	}

	ret = of_property_read_u32(dev->of_node, "mediatek,larb-id",
				   &larb_dev->larb_id);
	if (ret != 0)
		return ret;

	larb_dev->dev = dev;
	dev_set_drvdata(dev, larb_dev);

	pm_runtime_enable(dev);

	return component_add(dev, &mtk_cam_pm_component_ops);
}

static int mtk_cam_larb_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_disable(dev);

	component_del(dev, &mtk_cam_pm_component_ops);
	return 0;
}

static const struct of_device_id mtk_cam_larb_match[] = {
	{.compatible = "mediatek,camisp-larb",},
	{},
};

//MODULE_DEVICE_TABLE(of, mtk_cam_larb_match);

struct platform_driver mtk_cam_larb_driver = {
	.probe  = mtk_cam_larb_probe,
	.remove = mtk_cam_larb_remove,
	.driver = {
		.name   = "mtk-cam-larb",
		.of_match_table = of_match_ptr(mtk_cam_larb_match),
	},
};
//module_platform_driver(mtk_cam_larb_driver);

//MODULE_LICENSE("GPL v2");
//MODULE_DESCRIPTION("Mediatek video camera isp larb driver");

