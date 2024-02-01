/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 *
 * Author: Shih-Fang chuang <shih-fang.chuang@mediatek.com>
 *
 */

#ifndef _MTK_TRAW_V4L2_VNODE_H_
#define _MTK_TRAW_V4L2_VNODE_H_

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

#define MTK_TRAW_OUTPUT_MIN_WIDTH		2U
#define MTK_TRAW_OUTPUT_MIN_HEIGHT		2U
#define MTK_TRAW_OUTPUT_MAX_WIDTH		5376U
#define MTK_TRAW_OUTPUT_MAX_HEIGHT		4032U
#define MTK_TRAW_CAPTURE_MIN_WIDTH		2U
#define MTK_TRAW_CAPTURE_MIN_HEIGHT		2U
#define MTK_TRAW_CAPTURE_MAX_WIDTH		5376U
#define MTK_TRAW_CAPTURE_MAX_HEIGHT		4032U

static const struct mtk_imgsys_dev_format traw_imgi_fmts[] = {
	{
		.format = V4L2_PIX_FMT_VYUY,
		.depth	 = { 16 },
		.row_depth = { 16 },
		.num_planes = 1,
		.num_cplanes = 1,
	},
	{
		.format = V4L2_PIX_FMT_YUYV,
		.depth	 = { 16 },
		.row_depth = { 16 },
		.num_planes = 1,
		.num_cplanes = 1,
	},
	{
		.format = V4L2_PIX_FMT_YVYU,
		.depth	 = { 16 },
		.row_depth = { 16 },
		.num_planes = 1,
		.num_cplanes = 1,
	},
	{
		.format = V4L2_PIX_FMT_NV12,
		.depth = { 12 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 2,
	},
#ifdef TRAW_FMT
	{
		.format = V4L2_PIX_FMT_MTISP_SBGGR8,
		.depth = { 8 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGBRG8,
		.depth = { 8 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGRBG8,
		.depth = { 8 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SRGGB8,
		.depth = { 8 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
#endif
	{
		.format = V4L2_PIX_FMT_MTISP_SBGGR8F,
		.depth = { 12 },
		.row_depth = { 8},
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGBRG8F,
		.depth = { 12 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGRBG8F,
		.depth = { 12 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SRGGB8F,
		.depth = { 12 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SBGGR10,
		.depth = { 10 },
		.row_depth = { 10 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGBRG10,
		.depth = { 10 },
		.row_depth = { 10 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGRBG10,
		.depth = { 10 },
		.row_depth = { 10 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SRGGB10,
		.depth = { 10 },
		.row_depth = { 10 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SBGGR10F,
		.depth = { 15 },
		.row_depth = { 10 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGBRG10F,
		.depth = { 15 },
		.row_depth = { 10 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGRBG10F,
		.depth = { 15 },
		.row_depth = { 10 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SRGGB10F,
		.depth = { 15 },
		.row_depth = { 10 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SBGGR12,
		.depth = { 12 },
		.row_depth = { 12 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGBRG12,
		.depth = { 12 },
		.row_depth = { 12 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGRBG12,
		.depth = { 12 },
		.row_depth = { 12 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SRGGB12,
		.depth = { 12 },
		.row_depth = { 12 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SBGGR12F,
		.depth = { 18 },
		.row_depth = { 12 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGBRG12F,
		.depth = { 18 },
		.row_depth = { 12 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGRBG12F,
		.depth = { 18 },
		.row_depth = { 12 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SRGGB12F,
		.depth = { 18 },
		.row_depth = { 12 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SBGGR14F,
		.depth = { 21 },
		.row_depth = { 14 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGBRG14F,
		.depth = { 21 },
		.row_depth = { 14 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SGRBG14F,
		.depth = { 21 },
		.row_depth = { 14 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format = V4L2_PIX_FMT_MTISP_SRGGB14F,
		.depth = { 21 },
		.row_depth = { 14 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 8,
	},
	{
		.format	= V4L2_PIX_FMT_YUV420M,
		.depth		= { 8, 2, 2 },
		.row_depth	= { 8, 4, 4 },
		.num_planes	= 3,
		.num_cplanes = 1,
	},
	{
		.format	= V4L2_PIX_FMT_YVU420M,
		.depth		= { 8, 2, 2 },
		.row_depth	= { 8, 4, 4 },
		.num_planes	= 3,
		.num_cplanes = 1,
	},
	{
		.format	= V4L2_PIX_FMT_NV12M,
		.depth		= { 8, 4 },
		.row_depth	= { 8, 8 },
		.num_planes	= 2,
		.num_cplanes = 1,
	},
	{	// Must have for SMVR/Multis-cale for every video_device nodes
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
		.format = V4L2_META_FMT_MTISP_SD,
		.num_planes = 1,
#if defaultdesc
		.depth = { 8 },
		.row_depth = { 8 },
		.num_cplanes = 1,
#endif
		.buffer_size = sizeof(struct singlenode_desc),
	},
	{
		.format = V4L2_META_FMT_MTISP_DESC_NORM,
		.num_planes = 1,
		.buffer_size = sizeof(struct header_desc_norm),
	},

};

static const struct mtk_imgsys_dev_format traw_metai_fmts[] = {
#ifdef TRAW_META
	{
		.format = V4L2_PIX_FMT_MTISP_SBGGR8,
		.depth = { 8 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 1,
	},
#endif
	{	// Must have for SMVR/Multis-cale for every video_device nodes
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

static const struct mtk_imgsys_dev_format traw_yuvo_fmts[] = {
	{
		.format = V4L2_PIX_FMT_VYUY,
		.depth = { 16 },
		.row_depth = { 16 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_YUYV,
		.depth = { 16 },
		.row_depth = { 16 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_YVYU,
		.depth = { 16 },
		.row_depth = { 16 },
		.num_planes = 1,
		.num_cplanes = 1,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_YVU420,
		.depth = { 12 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 3,
		.pass_1_align = 4,
	},
	{
		.format = V4L2_PIX_FMT_NV12,
		.depth = { 12 },
		.row_depth = { 8 },
		.num_planes = 1,
		.num_cplanes = 2,
		.pass_1_align = 4,
	},
	{	// Must have for SMVR/Multis-cale for every video_device nodes
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

static const struct mtk_imgsys_dev_format traw_pdc_fmts[] = {
	/* Y8 bit */
	{
		.format	= V4L2_PIX_FMT_GREY,
		.depth		= { 8 },
		.row_depth	= { 8 },
		.num_planes	= 1,
		.num_cplanes = 1,
	},
	/* Must have for SMVR/Multis-cale for every video_device nodes */
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


static const struct v4l2_frmsizeenum traw_in_frmsizeenum = {
	.type = V4L2_FRMSIZE_TYPE_CONTINUOUS,
	.stepwise.max_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
	.stepwise.min_width = MTK_TRAW_CAPTURE_MIN_WIDTH,
	.stepwise.max_height = MTK_TRAW_CAPTURE_MAX_HEIGHT,
	.stepwise.min_height = MTK_TRAW_CAPTURE_MIN_HEIGHT,
	.stepwise.step_height = 1,
	.stepwise.step_width = 1,
};

static const struct v4l2_frmsizeenum traw_out_frmsizeenum = {
	.type = V4L2_FRMSIZE_TYPE_CONTINUOUS,
	.stepwise.max_width = MTK_TRAW_OUTPUT_MAX_WIDTH,
	.stepwise.min_width = MTK_TRAW_OUTPUT_MIN_WIDTH,
	.stepwise.max_height = MTK_TRAW_OUTPUT_MAX_HEIGHT,
	.stepwise.min_height = MTK_TRAW_OUTPUT_MIN_HEIGHT,
	.stepwise.step_height = 1,
	.stepwise.step_width = 1,
};

static const struct mtk_imgsys_video_device_desc traw_setting[] = {
	/* Input Video Node */
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TIMGI_OUT,
		.name = "TIMGI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_imgi_fmts,
		.num_fmts = ARRAY_SIZE(traw_imgi_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_TRAW_OUTPUT_MAX_WIDTH,
		.default_height = MTK_TRAW_OUTPUT_MAX_HEIGHT,
		.dma_port = 0,
		.frmsizeenum = &traw_in_frmsizeenum,
		.description = "Imgi image source",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TUFDI_OUT,
		.name = "TUFDI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_imgi_fmts,
		.num_fmts = ARRAY_SIZE(traw_imgi_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_TRAW_OUTPUT_MAX_WIDTH,
		.default_height = MTK_TRAW_OUTPUT_MAX_HEIGHT,
		.dma_port = 1,
		.frmsizeenum = &traw_in_frmsizeenum,
		.description = "Ufdi image source",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_METAI_OUT,
		.name = "METAI Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_metai_fmts,
		.num_fmts = ARRAY_SIZE(traw_metai_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_HEIGHT,
		.dma_port = 2,
		.frmsizeenum = &traw_in_frmsizeenum,
		.description = "METAI image source",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_PDC_OUT,
		.name = "PDC Input",
		.cap = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_pdc_fmts,
		.num_fmts = ARRAY_SIZE(traw_pdc_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_HEIGHT,
		.dma_port = 3,
		.frmsizeenum = &traw_in_frmsizeenum,
		.description = "PDC image source",
	},
	/* Output Video Node */
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TYUVO_CAPTURE,
		.name = "TYUVO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_yuvo_fmts,
		.num_fmts = ARRAY_SIZE(traw_yuvo_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 0,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "YUVO output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TYUV2O_CAPTURE,
		.name = "TYUV2O Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_yuvo_fmts,
		.num_fmts = ARRAY_SIZE(traw_yuvo_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 1,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "YUV2O output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TYUV3O_CAPTURE,
		.name = "TYUV3O Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_yuvo_fmts,
		.num_fmts = ARRAY_SIZE(traw_yuvo_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 2,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "YUV3O output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TYUV4O_CAPTURE,
		.name = "TYUV4O Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_yuvo_fmts,
		.num_fmts = ARRAY_SIZE(traw_yuvo_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 3,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "YUV4O output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TYUV5O_CAPTURE,
		.name = "TYUV5O Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_yuvo_fmts,
		.num_fmts = ARRAY_SIZE(traw_yuvo_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 4,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "YUV5O output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_FEO_CAPTURE,
		.name = "FEO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_metai_fmts,
		.num_fmts = ARRAY_SIZE(traw_metai_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 5,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "FEO output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TIMGO_CAPTURE,
		.name = "TIMGO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_imgi_fmts,
		.num_fmts = ARRAY_SIZE(traw_imgi_fmts),
		.default_fmt_idx = 4,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 6,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "TIMGO output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_IMGSTATO_CAPTURE,
		.name = "IMGSTATO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_metai_fmts,
		.num_fmts = ARRAY_SIZE(traw_metai_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 7,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "IMGSTATO output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_TMEO_CAPTURE,
		.name = "TMEO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_pdc_fmts,
		.num_fmts = ARRAY_SIZE(traw_pdc_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 8,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "TMEO output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_XTMEO_CAPTURE,
		.name = "XTMEO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_pdc_fmts,
		.num_fmts = ARRAY_SIZE(traw_pdc_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 9,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "XTMEO output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_XTFDO_CAPTURE,
		.name = "XTFDO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_yuvo_fmts,
		.num_fmts = ARRAY_SIZE(traw_yuvo_fmts),
		.default_fmt_idx = 1,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 10,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "XTFDO output",
	},
	{
		.id = MTK_IMGSYS_VIDEO_NODE_ID_XTADLDBGO_CAPTURE,
		.name = "XTDBGO Output",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.smem_alloc = 0,
		.flags = MEDIA_LNK_FL_DYNAMIC,
		.fmts = traw_imgi_fmts,
		.num_fmts = ARRAY_SIZE(traw_imgi_fmts),
		.default_fmt_idx = 2,
		.default_width = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.default_height = MTK_TRAW_CAPTURE_MAX_WIDTH,
		.dma_port = 11,
		.frmsizeenum = &traw_out_frmsizeenum,
		.description = "XTDBGO output",
	},

};

#endif // _MTK_TRAW_V4L2_VNODE_H_
