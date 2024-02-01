/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018 MediaTek Inc.
 *
 * Author: Frederic Chen <frederic.chen@mediatek.com>
 *
 */

#ifndef _MTK_ME_V4L2_VNODE_H_
#define _MTK_ME_V4L2_VNODE_H_

/*#include <linux/platform_device.h>
 * #include <linux/module.h>
 * #include <linux/of_device.h>
 * #include <linux/pm_runtime.h>
 * #include <linux/remoteproc.h>
 * #include <linux/remoteproc/mtk_scp.h>
 */
#include <linux/videodev2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-event.h>
#include "mtk_imgsys-dev.h"
#include "mtk_imgsys-hw.h"

//#include "mtk_imgsys_v4l2.h"
#include "mtk_header_desc.h"

#define defaultdesc 0


static const struct mtk_imgsys_dev_format in_fmts[] = {
	{
		.format = V4L2_META_FMT_MTISP_DESC,
		.num_planes = 1,
#if defaultdesc
		.depth	 = { 8 },
		.row_depth = { 8 },
		.num_cplanes = 1,
#endif
		.buffer_size = sizeof(struct header_desc),
	},
	{
		.format = V4L2_META_FMT_MTISP_DESC_NORM,
		.num_planes = 1,
		.buffer_size = sizeof(struct header_desc_norm),
	},
};

static const struct mtk_imgsys_dev_format out_fmts[] = {
	{
		.format = V4L2_META_FMT_MTISP_DESC,
		.num_planes = 1,
#if defaultdesc
		.depth = { 8 },
		.row_depth = { 8 },
		.num_cplanes = 1,
#endif
		.buffer_size = sizeof(struct header_desc),
	},
	{
		.format = V4L2_META_FMT_MTISP_DESC_NORM,
		.num_planes = 1,
		.buffer_size = sizeof(struct header_desc_norm),
	},

};

static const struct v4l2_frmsizeenum in_frmsizeenum = {
	.type = V4L2_FRMSIZE_TYPE_CONTINUOUS,
	.stepwise.max_width = MTK_DIP_CAPTURE_MAX_WIDTH,
	.stepwise.min_width = MTK_DIP_CAPTURE_MIN_WIDTH,
	.stepwise.max_height = MTK_DIP_CAPTURE_MAX_HEIGHT,
	.stepwise.min_height = MTK_DIP_CAPTURE_MIN_HEIGHT,
	.stepwise.step_height = 1,
	.stepwise.step_width = 1,
};

static const struct v4l2_frmsizeenum out_frmsizeenum = {
	.type = V4L2_FRMSIZE_TYPE_CONTINUOUS,
	.stepwise.max_width = MTK_DIP_OUTPUT_MAX_WIDTH,
	.stepwise.min_width = MTK_DIP_OUTPUT_MIN_WIDTH,
	.stepwise.max_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
	.stepwise.min_height = MTK_DIP_OUTPUT_MIN_HEIGHT,
	.stepwise.step_height = 1,
	.stepwise.step_width = 1,
};

static const struct mtk_imgsys_video_device_desc me_setting[] = {
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL0IMG0_OUT,
		.name = "L0I0I Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME Input",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL0IMG1_OUT,
		.name = "L0I1I Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME Input",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL1IMG0_OUT,
		.name = "L1I0I Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME L1 Input ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL1IMG1_OUT,
		.name = "L1I1I Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME L1 Input ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEIMGSTATS_OUT,
		.name = "STATI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME IMGSTATS Input ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL0RMV_OUT,
		.name = "L0RMVI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME L0RMV Input ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL1RMV_OUT,
		.name = "L1RMVI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME L1RMV Input ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL0FMB_OUT,
		.name = "L0FMBI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME L0FMB Input ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL1FMB_OUT,
		.name = "L1FMBI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME L1FMB Input ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEMIL_OUT,
		.name = "MEMILI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = in_fmts,
		.num_fmts = ARRAY_SIZE(in_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &in_frmsizeenum,
		.description = "ME MIL Input ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEMMGMIL_CAPTURE,
		.name = "MILO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME MMGMIL Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL0WMV_CAPTURE,
		.name = "L0WMVO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME L0WMV Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL1WMV_CAPTURE,
		.name = "L1WMVO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME L1WMV Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MECONF_CAPTURE,
		.name = "CONFO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME CONF Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEWMAP_CAPTURE,
		.name = "WMAPO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME WMAP Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEFMV_CAPTURE,
		.name = "FMVO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME OTHERS Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEFST_CAPTURE,
		.name = "FSTO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME FST Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MELMI_CAPTURE,
		.name = "LMIO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME LMI Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL0WFMB_CAPTURE,
		.name = "L0FMBO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME L0FMB Output ",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_MEL1WFMB_CAPTURE,
		.name = "L1FMBO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = out_fmts,
		.num_fmts = ARRAY_SIZE(out_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_DIP_OUTPUT_MAX_WIDTH,
		.default_height = MTK_DIP_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &out_frmsizeenum,
		.description = "ME L1FMB Output ",
	},
};

#endif // _MTK_ME_V4L2_VNODE_H_
