/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef _PW_IRIS_DEF_H_
#define _PW_IRIS_DEF_H_
#include <linux/types.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <drm/drm_mipi_dsi.h>

// Use Iris Analog bypass mode to light up panel
// Note: input timing should be same with output timing
//#define IRIS_ABYP_LIGHTUP
//#define IRIS_MIPI_TEST
#ifndef MIPI_DSI_MSG_LASTCOMMAND
#define MIPI_DSI_MSG_LASTCOMMAND  BIT(3)
#endif

#define IRIS_CMD_SIZE SZ_512K
#define ESD_CHK_EXIT (0xEDF)
#define ESD_CHK_NUM 3 // the same as ESD_CHECK_NUM in mtk_panel_ext.h
#define IRIS_FIRMWARE_NAME_I8	"iris8.fw"
#define IRIS_CCF1_FIRMWARE_NAME_I8 "iris8_ccf1.fw"
#define IRIS_CCF2_FIRMWARE_NAME_I8 "iris8_ccf2.fw"
#define IRIS_CCF3_FIRMWARE_NAME_I8 "iris8_ccf3.fw"
#define IRIS_CCF1_CALIBRATED_FIRMWARE_NAME_I8 "iris8_ccf1b.fw"
#define IRIS_CCF2_CALIBRATED_FIRMWARE_NAME_I8 "iris8_ccf2b.fw"
#define IRIS_CCF3_CALIBRATED_FIRMWARE_NAME_I8 "iris8_ccf3b.fw"

#define IRIS_FIRMWARE_NAME_I7P	"iris7p.fw"
#define IRIS_CCF1_FIRMWARE_NAME_I7P "iris7p_ccf1.fw"
#define IRIS_CCF2_FIRMWARE_NAME_I7P "iris7p_ccf2.fw"
#define IRIS_CCF3_FIRMWARE_NAME_I7P "iris7p_ccf3.fw"
#define IRIS_CCF1_CALIBRATED_FIRMWARE_NAME_I7P "iris7p_ccf1b.fw"
#define IRIS_CCF2_CALIBRATED_FIRMWARE_NAME_I7P "iris7p_ccf2b.fw"
#define IRIS_CCF3_CALIBRATED_FIRMWARE_NAME_I7P "iris7p_ccf3b.fw"

#define IRIS_FIRMWARE_NAME_I7	"iris7.fw"
#define IRIS_CCF1_FIRMWARE_NAME_I7 "iris7_ccf1.fw"
#define IRIS_CCF2_FIRMWARE_NAME_I7 "iris7_ccf2.fw"
#define IRIS_CCF3_FIRMWARE_NAME_I7 "iris7_ccf3.fw"
#define IRIS_CCF4_FIRMWARE_NAME_I7 "iris7_ccf4.fw"
#define IRIS_CCF1_CALIBRATED_FIRMWARE_NAME_I7 "iris7_ccf1b.fw"
#define IRIS_CCF2_CALIBRATED_FIRMWARE_NAME_I7 "iris7_ccf2b.fw"
#define IRIS_CCF3_CALIBRATED_FIRMWARE_NAME_I7 "iris7_ccf3b.fw"
#define IRIS_CCF4_CALIBRATED_FIRMWARE_NAME_I7 "iris7_ccf4b.fw"

#define IRIS_FIRMWARE_NAME_I5	"iris5.fw"
#define IRIS_CCF1_FIRMWARE_NAME_I5 "iris5_ccf1.fw"
#define IRIS_CCF2_FIRMWARE_NAME_I5 "iris5_ccf2.fw"
#define IRIS_CCF1_CALIBRATED_FIRMWARE_NAME_I5 "iris5_ccf1b.fw"
#define IRIS_CCF2_CALIBRATED_FIRMWARE_NAME_I5 "iris5_ccf2b.fw"

#define IRIS3_CHIP_VERSION	0x6933
#define IRIS5_CHIP_VERSION	0x6935
#define IRIS7_CHIP_VERSION      0x4777

#define IRIS_CHIP_VER_0   0
#define IRIS_CHIP_VER_1   1

#define DIRECT_BUS_HEADER_SIZE 8
#define IRIS_DBG_TOP_DIR "iris"

// bit mask
#define	BIT_MSK(bit)			((uint32_t)1 << (bit))

// set 1 bit to 1
#define BIT_SET(val, offset)	((val) | BIT_MSK(offset))

// set 1 bit to 0
#define BIT_CLR(val, offset)	((val) & (~BIT_MSK(offset)))

// bits mask
#define	BITS_MSK(bits)			(BIT_MSK(bits) - 1)

// left shift bits mask for offset bits
#define	BITS_SHFT(bits, offset)		(BITS_MSK(bits) << (offset))

// clear bits which from offeset for val
#define	BITS_CLR(val, bits, offset)	((val) & ~(BITS_SHFT(bits, offset)))

// get bits value which from offset for val
#define	BITS_GET(val, bits, offset)	\
	(((val) & BITS_SHFT(bits, offset)) >> (offset))

// set bits value which from offset by bitval
#define	BITS_SET(val, bits, offset, bitsval)	\
	(BITS_CLR(val, bits, offset) | (((bitsval) & BITS_MSK(bits)) << (offset)))

#define LUT_LEN 256
#define DPP_3DLUT_GROUP 3 // table 0,3,6 should store at the same address in iris
#define SCALER1D_LUT_NUMBER_I5 9
#define SCALER1D_LUT_NUMBER 64
#define SDR2HDR_LUT_BLOCK_SIZE (128*4)
#define SDR2HDR_LUT2_BLOCK_NUMBER (6)
#define SDR2HDR_LUTUVY_BLOCK_NUMBER (12)
#define SDR2HDR_LUT2_ADDRESS 0x3000
#define SDR2HDR_LUTUVY_ADDRESS 0x6000
#define SDR2HDR_LUT_BLOCK_ADDRESS_INC 0x400
#define SDR2HDR_LUT2_BLOCK_CNT (6)  //for ambient light lut
#define SDR2HDR_LUTUVY_BLOCK_CNT (12)  // for maxcll lut

#define PANEL_BL_MAX_RATIO 10000
#define IRIS_MODE_RFB                   0x0
#define IRIS_MODE_FRC_PREPARE           0x1
#define IRIS_MODE_FRC_PREPARE_DONE      0x2
#define IRIS_MODE_FRC                   0x3
#define IRIS_MODE_FRC_CANCEL            0x4
#define IRIS_MODE_FRC_PREPARE_RFB       0x5
#define IRIS_MODE_FRC_PREPARE_TIMEOUT   0x6
#define IRIS_MODE_RFB2FRC               0x7
#define IRIS_MODE_RFB_PREPARE           0x8
#define IRIS_MODE_RFB_PREPARE_DONE      0x9
#define IRIS_MODE_RFB_PREPARE_TIMEOUT   0xa
#define IRIS_MODE_FRC2RFB               0xb
#define IRIS_MODE_PT_PREPARE            0xc
#define IRIS_MODE_PT_PREPARE_DONE       0xd
#define IRIS_MODE_PT_PREPARE_TIMEOUT    0xe
#define IRIS_MODE_RFB2PT                0xf
#define IRIS_MODE_PT2RFB                0x10
#define IRIS_MODE_PT                    0x11
#define IRIS_MODE_KICKOFF60_ENABLE      0x12
#define IRIS_MODE_KICKOFF60_DISABLE     0x13
#define IRIS_MODE_PT2BYPASS             0x14
#define IRIS_MODE_BYPASS                0x15
#define IRIS_MODE_BYPASS2PT             0x16
#define IRIS_MODE_PTLOW_PREPARE         0x17
#define IRIS_MODE_DSI_SWITCH_2PT        0x18    // dsi mode switch during RFB->PT
#define IRIS_MODE_DSI_SWITCH_2RFB       0x19    // dsi mode switch during PT->RFB
#define IRIS_MODE_FRC_POST              0x1a    // for set parameters after FRC
#define IRIS_MODE_RFB_PREPARE_DELAY     0x1b    // for set parameters before RFB
#define IRIS_MODE_RFB_POST              0x1c    // for set parameters after RFB
#define IRIS_MODE_PT_POST               0x1d
#define IRIS_MODE_INITING               0xff
#define IRIS_MODE_OFF                   0xf0
#define IRIS_MODE_HDR_EN                0x20

#define IRIS_EMV_MIN			0x40
#define IRIS_EMV_ON_PREPARE		0x40
#define IRIS_EMV_ON_SWAP		0x41
#define IRIS_EMV_ON_CONFIGURE		0x42
#define IRIS_EMV_ON_FRC			0x43
#define IRIS_EMV_OFF_PREPARE		0x44
#define IRIS_EMV_CLOSE_THE_SET		0x45
#define IRIS_EMV_OFF_CONFIGURE		0x46
#define IRIS_EMV_OFF_PT		0x47
#define IRIS_EMV_OFF_FINAL		0x48
#define IRIS_EMV_CLOSE_PIPE_0		0x49
#define IRIS_EMV_OPEN_PIPE_0		0x4a
#define IRIS_EMV_CLOSE_PIPE_1		0x4b
#define IRIS_EMV_OPEN_PIPE_1		0x4c
#define IRIS_EMV_OFF_PRECONFIG		0x4d
#define IRIS_EMV_DUMP			0x4e
#define IRIS_EMV_ON_FRC_FINAL		0x4f
#define IRIS_EMV_HEALTH_DOWN		0x50
#define IRIS_EMV_HEALTH_UP		0x51
#define IRIS_EMV_ON_PRECONFIG		0x52
#define IRIS_EMV_HELPER_A		0x53
#define IRIS_EMV_ON_FRC_POST		0x54
#define IRIS_EMV_MAX			0x5f
#define IRIS_CAPS_ELEMENT_MIN		5
#define IRIS_CAPS_ELEMENT_MAX		20
#define IRIS_FEATURES_MAX		(IRIS_CAPS_ELEMENT_MAX - IRIS_CAPS_ELEMENT_MIN)

#define dsi_cmdset_to_iris_cmdset(pdst, psrc)  \
do {   \
	(pdst)->state = (psrc)->state;  \
	(pdst)->count = (psrc)->count;   \
	(pdst)->cmds = (struct iris_cmd_desc *)((psrc)->cmds);     \
} while (0)

#define dsi_mode_to_iris_mode(dst, src)  \
do {	\
	(dst)->h_active = (src)->h_active;	\
	(dst)->v_active = (src)->v_active;	\
	(dst)->refresh_rate = (src)->refresh_rate;	\
	(dst)->clk_rate_hz = (src)->clk_rate_hz;	\
	(dst)->mdp_transfer_time_us = (src)->mdp_transfer_time_us;	\
	(dst)->dsc_enabled = (src)->dsc_enabled;	\
} while (0)

enum IRIS_PLATFORM {
	IRIS_FPGA = 0,
	IRIS_ASIC,
};

enum DBC_LEVEL {
	DBC_INIT = 0,
	DBC_OFF,
	DBC_LOW,
	DBC_MIDDLE,
	DBC_HIGH,
	CABC_DLV_OFF = 0xF1,
	CABC_DLV_LOW,
	CABC_DLV_MIDDLE,
	CABC_DLV_HIGH,
};

enum SDR2HDR_LEVEL {
	SDR2HDR_LEVEL0 = 0,
	SDR2HDR_LEVEL1,
	SDR2HDR_LEVEL2,
	SDR2HDR_LEVEL3,
	SDR2HDR_LEVEL4,
	SDR2HDR_LEVEL5,
	SDR2HDR_LEVEL_CNT
};

enum SDR2HDR_TABLE_TYPE {
	SDR2HDR_LUT0 = 0,
	SDR2HDR_LUT1,
	SDR2HDR_LUT2,
	SDR2HDR_LUT3,
	SDR2HDR_UVY0,
	SDR2HDR_UVY1,
	SDR2HDR_UVY2,
	SDR2HDR_INV_UV0,
	SDR2HDR_INV_UV1,
};

enum FRC_PHASE_TYPE {
	FRC_PHASE_V1_60TE = 0,
	FRC_PHASE_V1_90TE,
	FRC_PHASE_V1_120TE,
	FRC_PHASE_TYPE_CNT
};

enum {
	IRIS_CONT_SPLASH_LK = 1,
	IRIS_CONT_SPLASH_KERNEL,
	IRIS_CONT_SPLASH_NONE,
	IRIS_CONT_SPLASH_BYPASS_PRELOAD,
};

enum {
	IRIS_DTSI_PIP_IDX_START = 0,
	IRIS_DTSI0_PIP_IDX = IRIS_DTSI_PIP_IDX_START,
	IRIS_DTSI1_PIP_IDX,
	IRIS_DTSI2_PIP_IDX,
	IRIS_DTSI3_PIP_IDX,
	IRIS_DTSI4_PIP_IDX,
	IRIS_DTSI5_PIP_IDX,
	IRIS_DTSI6_PIP_IDX,
	IRIS_DTSI7_PIP_IDX,
	IRIS_DTSI_PIP_IDX_CNT,
	IRIS_LUT_PIP_IDX = IRIS_DTSI_PIP_IDX_CNT,
	IRIS_PIP_IDX_CNT,

	IRIS_DTSI_NONE = 0xFF,
};

enum {
	IRIS_IP_START = 0x00,
	IRIS_IP_SYS = IRIS_IP_START,
	IRIS_IP_RX = 0x01,
	IRIS_IP_TX = 0x02,
	IRIS_IP_PWIL = 0x03,
	IRIS_IP_DPORT = 0x04,
	IRIS_IP_DTG = 0x05,
	IRIS_IP_PWM = 0x06,
	IRIS_IP_DSC_DEN = 0x07,
	IRIS_IP_DSC_ENC = 0x08,
	IRIS_IP_SDR2HDR = 0x09,
	IRIS_IP_CM = 0x0a,
	IRIS_IP_SDR2HDR_2 = IRIS_IP_CM,
	IRIS_IP_SCALER1D = 0x0b,
	IRIS_IP_IOINC1D = IRIS_IP_SCALER1D,
	IRIS_IP_PEAKING = 0x0c,
	IRIS_IP_LCE = 0x0d,
	IRIS_IP_DPP = 0x0e,
	IRIS_IP_DBC = 0x0f,
	IRIS_IP_EXT = 0x10,
	IRIS_IP_DMA = 0x11,
	IRIS_IP_AI = 0x12,

	IRIS_IP_RX_2 = 0x021,
	IRIS_IP_SRAM = 0x022,
	IRIS_IP_PWIL_2 = 0x023,
	IRIS_IP_DSC_ENC_2 = 0x24,
	IRIS_IP_DSC_DEN_2 = 0x25,
	IRIS_IP_DSC_DEN_3 = 0x26,
	IRIS_IP_PBSEL_2 = 0x27,
	IRIS_IP_PBSEL_3 = 0x28,
	IRIS_IP_PBSEL_4 = 0x29,
	IRIS_IP_OSD_COMP = 0x2a,
	IRIS_IP_OSD_DECOMP = 0x2b,
	IRIS_IP_OSD_BW = 0x2c,
	IRIS_IP_PSR_MIF = 0x2d,
	IRIS_IP_BLEND = 0x2e,
	IRIS_IP_SCALER1D_2 = 0x2f,
	IRIS_IP_IOINC1D_2 = IRIS_IP_SCALER1D_2,
	IRIS_IP_FRC_MIF = 0x30,
	IRIS_IP_FRC_DS = 0x31,
	IRIS_IP_GMD = 0x32,
	IRIS_IP_FBD = 0x33,
	IRIS_IP_CADDET = 0x34,
	IRIS_IP_MVC = 0x35,
	IRIS_IP_FI = 0x36,
	IRIS_IP_DSC_DEC_AUX = 0x37,
	IRIS_IP_DSC_ENC_TNR = 0x38,
	IRIS_IP_SR = 0x39,
	IRIS_IP_END,
	IRIS_IP_CNT = IRIS_IP_END
};

enum LUT_TYPE {
	LUT_IP_START = 128, /*0x80*/
	DBC_LUT = LUT_IP_START,
	CM_LUT,
	DPP_3DLUT = CM_LUT, //0x81
	SDR2HDR_LUT,
	SCALER1D_LUT,
	IOINC1D_LUT = SCALER1D_LUT,  //0x83
	AMBINET_HDR_GAIN, /*HDR case*/
	AMBINET_SDR2HDR_LUT, /*SDR2HDR case;*/  //0x85
	GAMMA_LUT,
	FRC_PHASE_LUT,
	APP_CODE_LUT,
	DPP_DITHER_LUT,
	SCALER1D_PP_LUT,
	IOINC1D_PP_LUT = SCALER1D_PP_LUT,  //0x8a
	DTG_PHASE_LUT,
	APP_VERSION_LUT,
	DPP_DEMURA_LUT,
	IOINC1D_LUT_SHARP,  //0x8e
	IOINC1D_PP_LUT_SHARP,
	IOINC1D_LUT_9TAP,  //0x90
	IOINC1D_LUT_SHARP_9TAP,
	DPP_PRE_LUT, //0x92
	BLENDING_LUT,
	SR_LUT,       //0x94
	MISC_INFO_LUT, //0x95
	DPP_DLV_LUT,
	SR_IOINC1D_LUT, //0x97
	LUT_IP_END
};

enum FIRMWARE_STATUS {
	FIRMWARE_LOAD_FAIL,
	FIRMWARE_LOAD_SUCCESS,
	FIRMWARE_IN_USING,
};

enum result {
	IRIS_FAILED = -1,
	IRIS_SUCCESS = 0,
};

enum PANEL_TYPE {
	PANEL_LCD_SRGB = 0,
	PANEL_LCD_P3,
	PANEL_OLED,
};

enum LUT_MODE {
	INTERPOLATION_MODE = 0,
	SINGLE_MODE,
};

enum SCALER_IP_TYPE {
	SCALER_INPUT = 0,
	SCALER_PP,
	SCALER_INPUT_SHARP,
	SCALER_PP_SHARP,
	SCALER_INPUT_9TAP,
	SCALER_INPUT_SHARP_9TAP,
};

enum IRIS_MEMC_MODE {
	MEMC_DISABLE = 0,
	MEMC_SINGLE_VIDEO_ENABLE,
	MEMC_DUAL_VIDEO_ENABLE,
	MEMC_SINGLE_GAME_ENABLE,
	MEMC_DUAL_EXTMV_ENABLE,
	MEMC_DUAL_GAME_ENABLE,
	MEMC_SINGLE_EXTMV_ENABLE,
};

enum {
	IRIS_DEBUG_MCU_ENABLE,
	IRIS_DEBUG_FRC_VAR_DISPLAY,
	IRIS_DEBUG_FRC_PT_SWITCH,
	IRIS_DEBUG_TRACE_EN,
	IRIS_DEBUG_CHECK_PWIL,
	IRIS_DEBUG_ESD_CTRL,
};

struct iris_pq_setting {
	u64 peaking:4;
	u64 cm6axis:2;
	u64 cmcolortempmode:3;
	u64 cmcolorgamut:12;
	u64 lcemode:2;
	u64 lcelevel:3;
	u64 graphicdet:1;
	u64 alenable:1;
	u64 dbc:2;
	u64 demomode:3;
	u64 sdr2hdr:4;
	u64 readingmode:4;
	u64 reserved:23;
};

enum IRIS_PERF_KT {
	kt_perf_start = 0,
	kt_frcen_set,
	kt_metaset_init,
	kt_iris2nd_up_start,
	kt_iris2nd_holdon,
	kt_iris2nd_open,
	kt_iris2nd_close,
	kt_iris2nd_up_ready,
	kt_iris2nd_down,
	kt_ap2nd_down,
	kt_ap2nd_up,
	kt_dual_open,
	kt_flush_osd,
	kt_flush_video,
	kt_flush_none,
	kt_dualon_start,
	kt_dualon_swap,
	kt_dualon_ready,
	kt_autorefresh_on,
	kt_autorefresh_off,
	kt_memcinfo_set,
	kt_frcdsc_start,
	kt_frcdsc_changed,
	kt_frc_prep_start,
	kt_frc_prep_end,
	kt_pt_frcon_start,
	kt_pt_frcon_end,
	kt_pt_frc_ready,
	kt_pt_prep_start,
	kt_pt_prep_end,
	kt_frc_pt_start,
	kt_frc_pt_end,
	kt_pt_ready,
	kt_frcsw_timeout,
	kt_off_prep,
	kt_close_set,
	kt_dual_close,
	kt_metaset_final,
	KT_PERF_MAX,
};

struct extmv_clockinout {
	ktime_t kt[KT_PERF_MAX];
	bool valid;
};

struct extmv_frc_meta {
	u32 mode;
	u32 gamePixelFormat;
	u32 gameWidthSrc;  /*H-resolution*/
	u32 gameHeightSrc; /*V-resolution*/
	u32 gameLeftSrc;   /*start position X*/
	u32 gameTopSrc;    /*start position Y*/
	u32 mvd0PixelFormat;
	u32 mvd0Width;  /*H-resolution*/
	u32 mvd0Height; /*V-resolution*/
	u32 mvd0Left;   /*start position X*/
	u32 mvd0Top;    /*start position Y*/
	u32 mvd1PixelFormat;
	u32 mvd1Width;  /*H-resolution*/
	u32 mvd1Height; /*V-resolution*/
	u32 mvd1Left;   /*start position X*/
	u32 mvd1Top;    /*start position Y*/
	u32 valid;
	u32 bmvSize;
	u32 orientation;
	u32 gmvdPixelFormat;
	u32 gmvdWidthSrc;  /*H-resolution*/
	u32 gmvdHeightSrc; /*V-resolution*/
	u32 gmvdLeft;   /*start position X*/
	u32 gmvdTop;    /*start position Y*/
	u32 containerWidth;
	u32 containerHeight;
	u32 gameWidth;
	u32 gameHeight;
	u32 gameLeft;
	u32 gameTop;
	u32 overflow;
	u32 gmvdWidthSrcLast;
	u32 gmvdHeightSrcLast;
	u32 gameWidthSrcLast;
	u32 gameHeightSrcLast;
};

struct iris_panel_timing_info {
	u32 flag;  //1--power on, 0---off
	u32 width;
	u32 height;
	u32 fps;
	u32 dsc;

	u32 h_back_porch;
	u32 h_sync_width;
	u32 h_front_porch;
	u32 v_back_porch;
	u32 v_sync_width;
	u32 v_front_porch;
	u32 dsi_xfer_ms;
};

struct iris_memc_info {
	u8 bit_mask;
	u8 memc_mode;
	u8 memc_level;
	u32 memc_app;
	u8 video_fps;
	u8 panel_fps;
	u8 vfr_en;
	u8 tnr_en;
	u8 low_latency_mode;
	u8 n2m_mode;
	u8 native_frc_en;
	u8 osd_window_en;
	u8 emv_game_mode;
	u16 capt_left;
	u16 capt_top;
	u16 capt_hres;
	u16 capt_vres;
	u32 osd_window[4];
	u32 mv_hres;
	u32 mv_vres;
	u32 latencyValue[64];
	u32 OSDProtection[64];
	u32 mBorderWidth;
};

enum low_latency_mode {
	NO_LT_MODE,
	NORMAL_LT,
	LT_MODE,
	ULTRA_LT_MODE,
	NORMAL_LT_DB,
	LT_MODE_DB,
	ULL_TB,
	LT_INVALID,
};

struct iris_switch_dump {
	bool trigger;
	u32 sw_pwr_ctrl;
	u32 rx_frame_cnt0;
	u32 rx_frame_cnt1;
	u32 rx_video_meta;
	u32 pwil_cur_meta0;
	u32 pwil_status;
	u32 pwil_disp_ctrl0;
	u32 pwil_int;
	u32 dport_int;
	u32 fi_debugbus[3];
	u32 ip_hangup_status_0;
	u32 ip_hangup_status_1;
	u32 alt_ctrl2;
	u32 rx1_frame_cnt;
	u32 rx1_video_meta;
	u32 fmif_emv_abnormal_st;
	u32 blending_int;
	u32 osd_status;
	u32 input_decoder_main_int;
	u32 input_dsc_decoder_aux;
	u32 osd_comp_int;
	u32 osd_decomp_int;
	u32 srcnn_int;
	u32 input_meta_ctrl;
	u32 video_ctrl5;
	u32 video_ctrl10_emv;
};

struct quality_setting {
	struct iris_pq_setting pq_setting;
	u32 cctvalue;
	u32 colortempvalue;
	u32 luxvalue;
	u32 maxcll;
	u32 source_switch;
	u32 al_bl_ratio;
	u32 system_brightness;
	u32 min_colortempvalue;
	u32 max_colortempvalue;
	u32 dspp_dirty;
	u32 cmftc;
	u32 colortempplusvalue;
	u32 lce_gain_reset;
	u32 cmlut_type;
	u32 cmlut_step;
	u32 cmlut_max_step;
	u32 cm_sdr2hdr_enable;
	u32 cm_para_level;
	u32 sdr2hdr_lce;
	u32 sdr2hdr_de;
	u32 sdr2hdr_tf_coef;
	u32 sdr2hdr_ftc;
	u32 sdr2hdr_csc;
	u32 sdr2hdr_de_ftc;
	u32 sdr2hdr_scurve;
	u32 sdr2hdr_graphic_det;
	u32 sdr2hdr_ai_tm;
	u32 sdr2hdr_ai_lce;
	u32 sdr2hdr_ai_de;
	u32 sdr2hdr_ai_graphic;
	u32 ai_ambient;
	u32 ai_backlight;
	u32 ai_auto_en;
	u32 pwil_dport_disable;
	u32 dpp_demura_en;
	u32 scurvelevel;
	u32 blending_mode;
};

struct iris_setting_info {
	struct quality_setting quality_cur;
	struct quality_setting quality_def;
};
struct ocp_header {
	u32 header;
	u32 address;
};

struct iris_update_ipopt {
	uint8_t ip;
	uint8_t opt_old;
	uint8_t opt_new;
	uint8_t chain;
};

struct iris_update_regval {
	uint8_t ip;
	uint8_t opt_id;
	uint16_t reserved;
	uint32_t mask;
	//uint32_t addr;
	uint32_t value;
};

struct iris_lp_ctrl {
	bool dynamic_power;
	bool ulps_lp;
	bool dbp_mode;	  // 0: pt mode; 1: dbp mode
	bool flfp_enable;	// first line first pixel enable or not
	bool te_swap; 	// indicated using te_swap and mask 3 TE for every 4 TEs from panel.
	u8 abyp_lp;
	bool force_exit_ulps_during_switching;
	// bit [0]: iris esd check [1]: panel esd check    [2]: recovery enable
	//     [3]: print more     [4]: force trigger once [5]: regdump disable
	int esd_ctrl;
	uint32_t esd_cnt_iris;
	uint32_t esd_cnt_panel;
	int esd_trigger;
};

struct iris_abyp_ctrl {
	bool abyp_failed;
	bool lightup_sys_powerdown; // power down sys directly in light up
	bool preloaded;
	uint8_t abypass_mode;
	uint16_t pending_mode;	// pending_mode is accessed by SDEEncoder and HWBinder
	struct mutex abypass_mutex;
	int default_abyp_mode;
	struct completion bypass_switch;
	bool wait_cmdq;
	unsigned int cmdq_num;
};

/**
 * Various Ops needed for different iris chip.
 */
struct iris_abyp_ops {
	bool (*abyp_enter)(int mode);

	bool (*abyp_exit)(int mode);

	void (*mipi_reset)(void);

	int (*abyp_select)(int dest_mode);

	void (*pre_config_for_timing)(void);

	void (*configure_abyp)(int value);
};

struct iris_frc_setting {
	u8 mv_buf_num;
	u8 vid_buf_num;
	u8 rgme_buf_num;
	u8 layer_c_en;
	u16 disp_hres;
	u16 disp_vres;
	u16 refresh_rate;
	bool disp_dsc;
	u16 input_vtotal;
	u16 disp_htotal;
	u16 disp_vtotal;
	u32 mv_hres;
	u32 mv_vres;
	u16 hres_2nd;
	u16 vres_2nd;
	u16 refresh_rate_2nd;
	bool dsc_2nd;
	u32 emv_hres;
	u32 emv_vres;
	u32 emv_baseaddr;
	u32 emv_hstride;
	u32 emv_offset;
	u32 hmv_depth_offset;
	u32 hmv_depth_line_bits;
	u32 hmv_mv_offset;
	u32 hmv_mv_line_bits;
	u32 sei_baseaddr;
	u32 sei_hstride;
	u32 sei_offset;
	u32 pmemc_write_index;
	u32 pmemc_pb_frame_index[2];
	u32 init_video_baseaddr;
	u32 init_dual_video_baseaddr;
	u32 init_emv_video_baseaddr;
	u32 init_pmemc_video_baseaddr;
	u32 init_emv_emv_baseaddr;
	u32 init_pmemc_emv_baseaddr;
	u32 init_single_mv_hres;
	u32 init_single_mv_vres;
	u32 init_dual_mv_hres;
	u32 init_dual_mv_vres;
	u32 init_ull_mv_hres;
	u32 init_ull_mv_vres;
	u32 mv_coef;
	u8 pps_table_sel;
	u32 dsc_enc_ctrl0;
	u32 dsc_enc_ctrl1;
	u32 video_baseaddr;
	u32 mv_baseaddr;
	u8 sr_sel;
	u8 frc_vfr_disp;
	u8 frc_dynen;
	u8 force_repeat;
	u8 sr_en;
	u32 iris_osd0_tl;
	u32 iris_osd1_tl;
	u32 iris_osd2_tl;
	u32 iris_osd3_tl;
	u32 iris_osd4_tl;
	u32 iris_osd0_br;
	u32 iris_osd1_br;
	u32 iris_osd2_br;
	u32 iris_osd3_br;
	u32 iris_osd4_br;
	u32 iris_osd_window_ctrl;
	u32 iris_osd_win_dynCompensate;
	bool emv_prepared;
	u8 memc_level;
	u8 in_fps;
	u8 out_fps;
	u8 memc_osd;
	u8 v2_lut_index;
	u8 v2_phaselux_idx_max;
	u32 v2_period_phasenum;
	u8 in_fps_configured;
	u32 frcc_pref_ctrl;
	uint32_t short_video;
};

struct iris_mspwil_setting {
	u8 memc_mode;
	u8 memc_lt_mode;
	u8 memc_n2m_mode;
	u8 tnr_mode;
	u8 input_scale_level;
	u8 pp_scale_level;
	u8 dsc_para_indx;
	u32 mv_hres;
	u32 mv_vres;
	u32 panel_te;
	u32 disp_hres;
	u32 disp_vres;
};

struct iris_mspwil_parameter {
	int frc_var_disp;	// -1: mean no update
	int frc_pt_switch_on;	// -1: mean no update
	int cmd_disp_on;	// -1: mean no update
	int ratio_update;
	int out_fps_ratio;
	int in_fps_ratio;
	int mvc_01phase_update;
	int mvc_01phase;
	int gmd_fmd_en;
};

enum pwil_mode {
	PT_MODE,
	RFB_MODE,
	FRC_MODE,
};

enum iris_config_type {
	IRIS_PEAKING = 0,
	IRIS_MEMC_LEVEL = 5,
	USER_DEMO_WND = 17,
	IRIS_MEMC_ACTIVE = 18,
	IRIS_CHIP_VERSION = 33,      // 0x0 : IRIS2, 0x1 : IRIS2-plus, 0x2 : IRIS3-lite
	IRIS_LUX_VALUE = 34,
	IRIS_CCT_VALUE = 35,
	IRIS_READING_MODE = 36,
	IRIS_CM_6AXES = 37,
	IRIS_CM_FTC_ENABLE = 38,
	IRIS_CM_COLOR_TEMP_MODE = 39,
	IRIS_CM_COLOR_GAMUT = 40,
	IRIS_LCE_MODE = 41,
	IRIS_LCE_LEVEL = 42,
	IRIS_GRAPHIC_DET_ENABLE = 43,
	IRIS_AL_ENABLE = 44,			//AL means ambient light
	IRIS_DBC_LEVEL = 45,
	IRIS_DEMO_MODE = 46,
	IRIS_SDR2HDR = 47,
	IRIS_COLOR_TEMP_VALUE = 48,
	IRIS_HDR_MAXCLL = 49,
	IRIS_CM_COLOR_GAMUT_PRE = 51,
	IRIS_DBC_LCE_POWER = 52,
	IRIS_DBC_LCE_DATA_PATH = 53,
	//IRIS_PP_DATA_PATH = 53,
	IRIS_DYNAMIC_POWER_CTRL = 54,
	IRIS_DMA_LOAD = 55,
	IRIS_DBP_MODE = IRIS_DMA_LOAD,
	IRIS_ANALOG_BYPASS_MODE = 56,
	IRIS_PANEL_TYPE = 57,
	IRIS_DPP_ONLY = 59,
	IRIS_HDR_PANEL_NITES_SET = 60,
	IRIS_PEAKING_IDLE_CLK_ENABLE = 61,
	IRIS_CM_MAGENTA_GAIN = 62,
	IRIS_CM_RED_GAIN = 63,
	IRIS_CM_YELLOW_GAIN = 64,
	IRIS_CM_GREEN_GAIN = 65,
	IRIS_CM_BLUE_GAIN = 66,
	IRIS_CM_CYAN_GAIN = 67,
	IRIS_BLC_PWM_ENABLE = 68,
	IRIS_DBC_LED_GAIN = 69,
	IRIS_SCALER_FILTER_LEVEL = 70,
	IRIS_CCF1_UPDATE = 71,
	IRIS_CCF2_UPDATE = 72,
	IRIS_FW_UPDATE = 73,
	IRIS_HUE_SAT_ADJ = 74,
	IRIS_SCALER_PP_FILTER_LEVEL = 76,
	IRIS_CSC_MATRIX = 75,
	IRIS_LOOP_BACK_MODE = 77,
	IRIS_SR_FILTER_LEVEL = 78,
	IRIS_CONTRAST_DIMMING = 80,
	IRIS_S_CURVE = 81,
	IRIS_DC_DIMMING = 87,
	IRIS_CLEAR_TRIGGER = 88,
	IRIS_BRIGHTNESS_CHIP = 82,
	IRIS_HDR_PREPARE = 90,
	IRIS_HDR_COMPLETE = 91,
	IRIS_MCF_DATA = 92,
	IRIS_BLENDING_CSR_CTRL = 93,
	IRIS_SET_DPP_APL_ABS = 94,
	IRIS_SET_DPP_APL_RES = 95,
	IRIS_GET_DPP_MCU_RES = 96,
	IRIS_ENABLE_DPP_APL = 97,
	IRIS_GET_DPP_APL_RES = 98,
	IRIS_LCE_GAIN = IRIS_GET_DPP_APL_RES,
	IRIS_PANEL_NITS = 99,
	IRIS_DUMP_APL_PER_FRAME = 100,

	IRIS_DBG_TARGET_REGADDR_VALUE_GET = 103,
	IRIS_DBG_TARGET_REG_DUMP = 104,
	IRIS_DBG_TARGET_REGADDR_VALUE_SET = 105,
	IRIS_DBG_KERNEL_LOG_LEVEL = 106,
	IRIS_DBG_SEND_PACKAGE = 107,
	IRIS_DBG_TIMING_SWITCH_LEVEL = 110,
	IRIS_DBG_TARGET_REGADDR_VALUE_SET2 = 112,
	IRIS_DEBUG_CAP = 113,
	IRIS_MIPI_RX_VALIDATE = 114,
	IRIS_CLEAR_FRC_MIF_INT = 116,
	IRIS_GET_FRC_MIF_INTRAW = 117,
	IRIS_GET_MEMC_REG_STATUS = 118,
	IRIS_MODE_SET = 120,
	IRIS_VIDEO_FRAME_RATE_SET = 121,
	IRIS_OUT_FRAME_RATE_SET = 122,	// debug only
	IRIS_OSD_ENABLE = 123,
	IRIS_OSD_AUTOREFRESH = 124,
	IRIS_OSD_OVERFLOW_ST = 125,
	// [23-16]: pwil mode, [15-8]: tx mode, [7-0]: rx mode
	IRIS_WORK_MODE = 126,
	IRIS_FRC_LOW_LATENCY = 127,
	IRIS_PANEL_TE = 128,
	IRIS_AP_TE = 129,
	IRIS_N2M_ENABLE = 130,
	IRIS_WAIT_VSYNC = 132,
	IRIS_MIPI2RX_PWRST = 133,
	IRIS_DUAL2SINGLE_ST = 134,
	IRIS_MEMC_OSD = 135,
	IRIS_MEMC_OSD_PROTECT = 136,
	IRIS_LCE_DEMO_WINDOW = 137,
	IRIS_CM_PARA_SET = 138,

	IRIS_DPP_DEMO_WINDOW = 139,
	IRIS_FINGER_DISPLAY = 140,
	IRIS_DPP_FADE_INOUT = 141,
	IRIS_DPP_PATH_MUX = 142,
	IRIS_GAMMA_MODE = 143,

	IRIS_PARAM_VALID = 144,
	IRIS_SDR2HDR_AI_ENALE = 145,
	IRIS_SDR2HDR_AI_INPUT_AMBIENTLIGHT = 146,
	IRIS_SDR2HDR_AI_INPUT_BACKLIGHT = 148,
	IRIS_SDR2HDR_LCE = 155,
	IRIS_SDR2HDR_DE = 156,
	IRIS_SDR2HDR_TF_COEF = 157,
	IRIS_SDR2HDR_FTC = 158,
	IRIS_HDR10PLUS = 161,
	IRIS_DPP_3DLUT_GAIN = 163,
	IRIS_DPP_3DLUT_CONTROL = 164,
	IRIS_SET_DSI_MODE_INFO = 167,
	IRIS_OSD_LAYER_EMPTY = 168,
	IRIS_SET_METADATA = 169,
	IRIS_SET_METADATA_LOCK = 170,
	IRIS_GET_METADATA = 171,
	IRIS_DUAL_CH_CTRL = 172,
	IRIS_MEMC_CTRL = 173,
	IRIS_MEMC_INFO_SET = 174,
	IRIS_DEBUG_SET = 175,
	IRIS_DEBUG_GET = 176,
	IRIS_KERNEL_STATUS_GET = 177,
	IRIS_SET_MVD_META = 178,
	IRIS_SDR2HDR_CSC_SWITCH = 179,
	IRIS_TNR_MODE = 180,
	IRIS_VFR_MODE = 181,
	IRIS_SDR2HDR_DE_FTC = 182,
	IRIS_SDR2HDR_SCURVE = 183,
	IRIS_SDR2HDR_GRAPHIC_DET = 184,
	IRIS_SDR2HDR_AI_TM = 185,
	IRIS_SDR2HDR_AI_LCE = 186,
	IRIS_SDR2HDR_AI_DE = 187,
	IRIS_SDR2HDR_AI_GRAPHIC = 188,
	IRIS_PWIL_DPORT_DISABLE = 189,
	IRIS_HDR_DATA_PATH = 190,
	IRIS_DPP_CSC_SET = 191,
	IRIS_PT_SR_SET = 192,
	IRIS_DEMURA_LUT_SET = 193,
	IRIS_DEMURA_ENABLE = 194,
	IRIS_DEMURA_XY_LUT_SET = 195,
	IRIS_FRC_PQ_LEVEL = 196,
	IRIS_SDR2HDR_UPDATE = 197,
	IRIS_SCL_CONFIG = 198,
	IRIS_SCL_MODEL  = 199,
	IRIS_MEMC_DSC_CONFIG = 200,
	IRIS_PT_SR_LUT_SET = 201,
	IRIS_PMEMC_SET = 202,
	IRIS_BLENDING_MODE_SET = 203,
	IRIS_TIMING_INFO = 204,
	IRIS_SPECIAL_MODE_SWITCH = 205,
	IRIS_GET_CMDQ_EMPTY = 206,
	IRIS_CM_RATIO_SET = 210,
	IRIS_BRIGHTNESS_V2_INFO = 211,
	IRIS_CONFIG_TYPE_MAX
	// Enumeration range is 0 ~ 255 and 768 ~ 1023.
};

enum SDR2HDR_CASE {
	SDR2HDR_Bypass = 0,
	HDR10In_ICtCp = 1,
	HLG_HDR = HDR10In_ICtCp,
	HDR10In_YCbCr = 2,
	HDR10 = HDR10In_YCbCr,
	ICtCpIn_YCbCr = 3,
	SDR2HDR_1 = ICtCpIn_YCbCr,
	SDR709_2_709 = 4,
	SDR2HDR_2 = SDR709_2_709,
	SDR709_2_p3 = 5,
	SDR2HDR_3 = SDR709_2_p3,
	SDR709_2_2020 = 6,
	SDR2HDR_4 = SDR709_2_2020,
	SDR2HDR_5,
	SDR2HDR_6,
	SDR2HDR_7,
	SDR2HDR_8,
	SDR2HDR_9,
	SDR2HDR_10,
	SDR2HDR_11,
	SDR2HDR_12,
	SDR2HDR_13,
};

enum HDR_POWER {
	HDR_POWER_OFF = 0,
	HDR_POWER_ON,
};

enum VC_ST {
	VC_PT = 0,
	VC_FRC,
	VC_ST_MAX
};

enum SDR2HDR_LUT_GAMMA_INDEX {
	SDR2HDR_LUT_GAMMA_120 = 0,
	SDR2HDR_LUT_GAMMA_106 = 1,
	SDR2HDR_LUT_GAMMA_102 = 2,
	SDR2HDR_LUT_GAMMA_100 = 3,
	SDR2HDR_LUT_GAMMA_MAX
};

enum iris_legacy_enabled {
	IRIS2_VER = 0,
	IRIS2PLUS_VER,
	IRIS3LITE_VER,
	IRIS5_VER,
	IRIS6_VER,
	IRISSOFT_VER,
	IRIS5DUAL_VER,
	UNKNOWN_VER
};

enum iris_feature_enable_bit {
	SUPPORT_HDR10 = 16,
	SUPPORT_SDR2HDR,
	SUPPORT_MEMC,
	SUPPORT_DUAL_MEMC,
	SUPPORT_EMV,
	SUPPORT_SR,
	SUPPORT_SOFT_IRIS,
	SUPPORT_EDR,
	SUPPORT_MEMC_SR,
	SUPPORT_HESR,
	SUPPORT_TAA,
	SUPPORT_FXAA,
	SUPPORT_HMV,
	SUPPORT_MORE = 29,
};

enum iris_feature_group2_enable_bit {
	SUPPORT_PMEMC = 0,
	SUPPORT_GMEMC,
	SUPPORT_MAX = 18,
};

union iris_chip_basic_caps {
	u32 val;
	struct {
		u32 legacy_enabled : 7;
		u32 version_is_new : 1;
		u32 version_number : 8;
		u32 feature_enabled: 14;
		u32 asic_type      : 2;
	};
};

struct iris_chip_caps {
	u32 caps;
	u32 features[IRIS_FEATURES_MAX];
};

enum iris_abypass_status {
	PASS_THROUGH_MODE = 0,
	ANALOG_BYPASS_MODE,
	ABP_SLEEP_MODE,
	ABP2PT_SWITCHING,
};

enum iris_abyp_status {
	IRIS_PT_MODE = 0,
	IRIS_ABYP_MODE,
	IRIS_PT_TO_ABYP_MODE,
	IRIS_ABYP_TO_PT_MODE,
	MAX_MODE = 255,
};

struct msmfb_iris_tm_points_info {
	void *lut_lutx_payload;
	void *lut_luty_payload;
	void *lut_luttm_payload;
	void *lut_lutcsc_payload;
	void *lut_lutratio_payload;
};

struct msmfb_iris_demura_info {
	void *lut_swpayload;
};

struct msmfb_iris_demura_xy {
	void *lut_xypayload;
};

struct iris_vc_ctrl {
	bool vc_enable;
	uint8_t vc_arr[VC_ST_MAX];
	uint8_t to_iris_vc_id;
	uint8_t to_panel_hs_vc_id;
	uint8_t to_panel_lp_vc_id;
};

enum AI_CASE {
	AI_AMBIENT_BACKLIGHT_DISABLE = 0,
	AI_AMBIENT_ENABLE,
	AI_BACKLIGHT_ENABLE,
	AI_AMBIENT_BACKLIGHT_ENABLE,
};

enum MSG_FLAG {
	READ_FLAG,
	LAST_FLAG,
	BATCH_FLAG,
};

enum dts_ctx_id {
	DTS_CTX_FROM_IMG = 1,
	DTS_CTX_FROM_FW,
};

struct iris_dts_ops {
	enum dts_ctx_id id;

	const void *(*get_property)(const struct device_node *np, const char *name,
		int *lenp);
	bool (*read_bool)(const struct device_node *np,
		const char *propname);
	int (*read_u8)(const struct device_node *np,
		const char *propname, u8 *out_value);
	int (*count_u8_elems)(const struct device_node *np,
		const char *propname);
	int (*read_u8_array)(const struct device_node *np,
		const char *propname, u8 *out_values, size_t sz);
	int (*read_u32)(const struct device_node *np,
		const char *propname, u32 *out_value);
	int (*read_u32_array)(const struct device_node *np,
		const char *propname, u32 *out_values, size_t sz);
	int (*count_u32_elems)(const struct device_node *np,
			const char *propname);
};

enum iris_chip_type {
	CHIP_UNKNOWN = 0,
	CHIP_IRIS5,
	CHIP_IRIS6,
	CHIP_IRIS7,
	CHIP_IRIS7P,
	CHIP_IRIS8,
	CHIP_MAX,
};

struct iris_ver_spec_info {
	enum iris_chip_type version;
};

enum pw_iris_mode {
	IRIS_VIDEO_MODE = 0,
	IRIS_CMD_MODE,
	IRIS_MODE_MAX
};

enum iris_cmd_set_state {
	IRIS_CMD_SET_STATE_LP = 0,
	IRIS_CMD_SET_STATE_HS,
	IRIS_CMD_SET_STATE_MAX
};

enum iris_dsi_display_type {
	IRIS_DSI_PRIMARY = 0,
	IRIS_DSI_SECONDARY,
	IRIS_MAX_DISPLAY,
};

struct iris_cmd_desc {
	struct mipi_dsi_msg msg;
	bool last_command;
	u32  post_wait_ms;
	u32 ctrl;
	u32 ctrl_flags;
	ktime_t ts;
};

struct iris_cmd_set {
	uint32_t state;
	uint32_t count;
	struct iris_cmd_desc *cmds;
};

struct iris_mode_info {
	u32 h_active;
	u32 h_back_porch;
	u32 h_sync_width;
	u32 h_front_porch;

	u32 v_active;
	u32 v_back_porch;
	u32 v_sync_width;
	u32 v_front_porch;

	u32 refresh_rate;
	u64 clk_rate_hz;
	u32 mdp_transfer_time_us;
	bool dsc_enabled;
};

#define IRIS_CHIP_CNT   2
#define IRIS_SYSFS_TOP_DIR   "iris"
#define CHIP_VERSION_IS_I7   1
#define CHIP_VERSION_IS_I7P   2

//#define IRIS_EXT_CLK // use for external gpio clk

/* iris ip option, it will create according to opt_id.
 *  link_state will be create according to the last cmds
 */
struct iris_ip_opt {
	uint8_t opt_id; /*option identifier*/
	uint32_t cmd_cnt; /*option length*/
	uint8_t link_state; /*high speed or low power*/
	struct iris_cmd_desc *cmd; /*the first cmd of desc*/
};

/*ip search index*/
struct iris_ip_index {
	int32_t opt_cnt; /*ip option number*/
	struct iris_ip_opt *opt; /*option array*/
};

struct iris_pq_ipopt_val {
	int32_t opt_cnt;
	uint8_t ip;
	uint8_t *popt;
};

struct iris_pq_init_val {
	int32_t ip_cnt;
	struct iris_pq_ipopt_val *val;
};

/*used to control iris_ctrl opt sequence*/
struct iris_ctrl_opt {
	uint8_t ip;
	uint8_t opt_id;
	uint8_t chain;
};

struct iris_ctrl_seq {
	int32_t cnt;
	struct iris_ctrl_opt *ctrl_opt;
};

//will pack all the commands here
struct iris_out_cmds {
	/* will be used before cmds sent out */
	struct iris_cmd_desc *iris_cmds_buf;
	u32 cmds_index;
};

struct iris_pq_update_cmd {
	struct iris_update_ipopt *update_ipopt_array;
	u32 array_index;
};

struct iris_i2c_cfg {
	uint8_t *buf;
	uint32_t buf_index;
};

struct iris_i2c_msg {
	uint8_t *buf;
	uint32_t len;
};

enum PATH_TYPE {
	PATH_I2C = 0,
	PATH_DSI,
	PATH_I2C_S,
};

#define I2C_MSG_MAX_LEN (65535)
#define IRIS_I2C_BUF_LEN (256*1024)  //256k bytes

typedef int (*iris_i2c_read_cb)(u32 reg_addr, u32 *reg_val);
typedef int (*iris_i2c_write_cb)(u32 reg_addr, u32 reg_val);
typedef int (*iris_i2c_burst_write_cb)(u32 start_addr, u32 *lut_buffer, u16 reg_num);
typedef int (*iris_i2c_conver_ocp_write_cb)(u32 start_addr, u32 *lut_buffer, u32 reg_num, bool is_burst);

typedef void (*iris_acquire_panel_lock_cb)(void);
typedef void (*iris_release_panel_lock_cb)(void);
typedef int (*iris_dsi_send_cmds_cb)(struct iris_cmd_desc *cmds, u32 count,
	enum iris_cmd_set_state state, u8 vc_id);
typedef u32 (*get_panel2_power_refcount_cb)(void);
typedef int (*iris_obtain_cur_timing_info_cb)(struct iris_mode_info *);
typedef void (*iris_set_esd_status_cb)(bool enable);
typedef int (*iris_debug_display_info_get_cb)(char *kbuf, int size);
typedef int (*iris_wait_vsync_cb)(void);
typedef void (*iris_send_pwil_cmd_cb)(struct iris_cmd_set *, u32 addr,  u32);
typedef void (*iris_vdo_mode_send_cmd_with_handle_cb)(void *handle,
		void *data, int len, u32 flag, int type);

typedef void (*iris_change_header_cb)(void  *pcmd_comp);
struct iris_lightup_ops {
	iris_acquire_panel_lock_cb acquire_panel_lock;
	iris_release_panel_lock_cb release_panel_lock;
	iris_dsi_send_cmds_cb  transfer;
	iris_obtain_cur_timing_info_cb  obtain_cur_timing_info;
	get_panel2_power_refcount_cb get_panel2_power_refcount;
	iris_debug_display_info_get_cb get_display_info;
	iris_wait_vsync_cb wait_vsync;
	iris_send_pwil_cmd_cb send_pwil_cmd;
	iris_change_header_cb change_header;
	iris_vdo_mode_send_cmd_with_handle_cb vdo_mode_send_cmd_with_handle;
};

typedef int (*iris_configure_get_i7_selected_cb)(u32 display, u32 type, u32 count, u32 *values);
struct iris_ioctl_ops {
	iris_configure_get_i7_selected_cb get_selected_configure;
};

typedef int (*iris_get_panel_mode_cb)(void);
typedef int (*iris_memc_get_main_panel_timing_info_cb)(struct iris_panel_timing_info *timing_info);
typedef int (*iris_memc_get_main_panel_dsc_en_info_cb)(bool *dsc_en);
typedef int (*iris_memc_get_aux_panel_timing_info_cb)(struct iris_panel_timing_info *timing_info);
typedef int (*iris_memc_get_aux_panel_dsc_en_info_cb)(bool *dsc_en);
typedef int (*iris_memc_get_aux_panel_dsc_size_info_cb)(uint32_t *slice_width, uint32_t *slice_height);
typedef int (*iris_memc_try_panel_lock_cb)(void);
typedef void (*iris_dual_register_osd_irq_cb)(void *disp);
typedef int (*iris_dual_create_pps_buf_cmd_cb)(char *buf, int pps_id, u32 len, bool is_secondary);
typedef bool (*iris_dual_main_panel_initialized_cb)(void);
typedef bool (*iris_dual_aux_panel_initialized_cb)(void);
typedef bool (*iris_dual_main_panel_existed_cb)(void);
typedef bool (*iris_dual_aux_panel_existed_cb)(void);
typedef void (*iris_set_idlemgr_cb)(unsigned int crtc_id, unsigned int enable, bool need_lock);
typedef unsigned long long (*iris_set_idle_check_interval_cb)(unsigned int crtc_id, unsigned long long new_interval);

struct iris_memc_ops_cb {
	iris_memc_get_main_panel_timing_info_cb iris_memc_get_main_panel_timing_info;
	iris_memc_get_main_panel_dsc_en_info_cb iris_memc_get_main_panel_dsc_en_info;
	iris_memc_get_aux_panel_timing_info_cb iris_memc_get_aux_panel_timing_info;
	iris_memc_get_aux_panel_dsc_en_info_cb iris_memc_get_aux_panel_dsc_en_info;
	iris_memc_get_aux_panel_dsc_size_info_cb iris_memc_get_aux_panel_dsc_size_info;
	iris_memc_try_panel_lock_cb iris_memc_try_panel_lock;
	iris_dual_register_osd_irq_cb iris_register_osd_irq;
	iris_dual_create_pps_buf_cmd_cb iris_memc_create_pps_buf_cmd;
	iris_dual_main_panel_initialized_cb iris_memc_main_panel_initialized;
	iris_dual_aux_panel_initialized_cb iris_memc_aux_panel_initialized;
	iris_dual_main_panel_initialized_cb iris_memc_main_panel_existed;
	iris_dual_aux_panel_initialized_cb iris_memc_aux_panel_existed;
	iris_set_idlemgr_cb iris_set_idlemgr;
	iris_set_idle_check_interval_cb iris_set_idle_check_interval;
};
typedef void (*iris_wait_pre_frame_done_cb)(void);
typedef void (*iris_lightup_cb)(void);
typedef void (*iris_clk_set_cb)(bool enable, bool is_secondary);

typedef bool (*iris_is_read_cmd_cb)(struct iris_cmd_desc *pdesc);
typedef bool (*iris_is_last_cmd_cb)(const struct mipi_dsi_msg  *pmsg);
typedef bool (*iris_is_curmode_cmd_mode_cb)(void);
typedef bool (*iris_is_curmode_vid_mode_cb)(void);
typedef void (*iris_set_msg_flags_cb)(struct iris_cmd_desc *pdesc, int type);
typedef int (*iris_switch_cmd_type_cb)(int type);
typedef void (*iris_set_msg_ctrl_cb)(struct iris_cmd_desc *pdesc);
typedef int (*iris_wait_cmdq_done_cb)(void);
typedef bool (*iris_is_cmdq_empty_cb)(void);

typedef void (*iris_cmd_desc_para_fill_cb)(struct iris_cmd_desc *dsi_cmd);
struct iris_platform_ops {
	iris_cmd_desc_para_fill_cb fill_desc_para;
};

struct iris_cfg;

struct iris_memc_func {
	void (*register_osd_irq)(void *disp);
	void (*update_panel_ap_te)(void *handle, u32 new_te);
	void (*exit_abyp_update_panel_ap_te)(void *handle, u32 new_te);
	void (*inc_osd_irq_cnt)(void);
	bool (*is_display1_autorefresh_enabled)(bool is_secondary);
	u32 (*disable_mipi1_autorefresh)(void);
	void (*pt_sr_set)(int enable, int processWidth, int processHeight);
	int (*configure_memc)(u32 type, u32 value);
	int (*configure_ex_memc)(u32 type, u32 count, u32 *values);
	int (*configure_get_memc)(u32 type, u32 count, u32 *values);
	void (*init_memc)(void);
	void (*lightoff_memc)(void);
	void (*enable_memc)(bool is_secondary);
	void (*sr_update)(void);
	void (*frc_setting_init)(void);
	int (*dbgfs_memc_init)(void);
	void (*parse_memc_param)(void);
	void (*frc_timing_setting_update)(void);
	void (*pt_sr_reset)(void);
	void (*mcu_state_set)(u32 mode);
	void (*mcu_ctrl_set)(u32 ctrl);
	void (*memc_vfr_video_update_monitor)(struct iris_cfg *pcfg, bool is_secondary);
	int (*low_latency_mode_get)(void);
	bool (*health_care)(void);
	void (*dsi_rx_mode_switch)(uint8_t rx_mode);
	bool (*not_allow_off_primary)(void);
	bool (*not_allow_off_secondary)(void);
	int (*blending_enable)(bool enable);
	int (*get_drm_property)(int id);
	void (*iris_demo_wnd_set)(void);
};

struct iris_memc_helper_ops {
	void (*iris_dsc_up_format)(bool up, uint32_t *payload, uint32_t pps_sel);
	void (*iris_ioinc_pp_init)(void);
	bool (*iris_ioinc_pp_proc)(bool enable, uint32_t ioinc_pos,
		int32_t in_h, int32_t in_v, int32_t out_h, int32_t out_v,
		uint32_t path_sel, uint32_t sel_mode, bool *blend_changed);
	void (*iris_ioinc_pp_disable)(void);
	void (*iris_ioinc_pp_filter)(uint32_t count, uint32_t *values);
};

struct iris_timing_switch_ops {
	void (*iris_init_timing_switch_cb)(void);
	void (*iris_send_dynamic_seq)(void);
	void (*iris_pre_switch)(uint32_t refresh_rate, bool clock_changed);
	void (*iris_pre_switch_proc)(void);
	void (*iris_post_switch_proc)(void);
	void (*iris_set_tm_sw_dbg_param)(uint32_t type, uint32_t value);
	void (*iris_restore_capen)(void);
};

typedef int (*iris_debug_display_mode_get_cb)(char *kbuf, int size, bool debug);
typedef int (*iris_debug_pq_info_get_cb)(char *kbuf, int size, bool debug);
typedef int (*iris_send_lut_i5_cb)(u8 lut_type, u8 lut_table_index, u32 lut_abtable_index);
typedef int (*iris_send_lut_cb)(u8 lut_type, u8 lut_table_index);
typedef int (*iris_configure_cb)(u32 display, u32 type, u32 value);
typedef int (*iris_configure_ex_cb)(u32 display, u32 type, u32 count, u32 *values);
typedef int (*iris_configure_get_cb)(u32 display, u32 type, u32 count, u32 *values);
typedef int (*iris_dbgfs_adb_type_init_cb)(void *display);
typedef void (*iris_ocp_write_mult_vals_i5_cb)(u32 size, u32 *pvalues);
typedef int32_t (*iris_parse_frc_setting_cb)(struct device_node *np, struct iris_cfg *pcfg);
typedef void (*iris_mult_addr_pad_cb)(uint8_t **p, uint32_t *poff, uint32_t left_len);
typedef void (*iris_clean_frc_status_cb)(struct iris_cfg *pcfg);
typedef u8 (*iris_get_dbc_lut_index_cb)(void);
typedef int (*iris_loop_back_validate_cb)(void);
typedef int (*iris_mipi_rx0_validate_cb)(void);
typedef void (*iris_set_loopback_flag_cb)(uint32_t val);
typedef uint32_t (*iris_get_loopback_flag_cb)(void);
typedef void (*iris_lp_preinit_i5_cb)(void);
typedef void (*iris_lp_init_i5_cb)(void);
typedef void (*iris_dynamic_power_set_i5_cb)(bool enable, bool chain);
typedef bool (*iris_disable_ulps_cb)(uint8_t path);
typedef void (*iris_enable_ulps_cb)(uint8_t path, bool is_ulps_enable);
typedef int (*iris_dphy_itf_check_cb)(bool aux_channel);
typedef int (*iris_pmu_bsram_set_i5_cb)(bool on);
typedef void (*iris_dma_gen_ctrl_cb)(int channels, int source, bool chain);
typedef void (*iris_global_var_init_cb)(void);
typedef void (*iris_pwil_update_i7p_cb)(void);
typedef bool (*iris_abypass_switch_proc_cb)(int mode, bool pending, bool first);
typedef int (*iris_esd_check_cb)(void);
typedef void (*iris_qsync_set_i5_cb)(bool enable);
typedef int (*iris_change_dpp_lutrgb_type_addr_i7_cb)(void);
typedef int (*iris_parse_lut_cmds_cb)(uint32_t flag);
typedef int (*iris_get_hdr_enable_cb)(void);
typedef void (*iris_quality_setting_off_cb)(void);
typedef void (*iris_end_dpp_cb)(bool bcommit);
typedef void (*iris_pq_parameter_init_cb)(void);
typedef void (*iris_init_tm_points_lut_cb)(void);
typedef void (*iris_init_lut_buf_cb)(void);
typedef int (*iris_cm_ratio_set_i5_cb)(uint8_t skip_last);
typedef void (*iris_cm_ratio_set_cb)(void);
typedef void (*iris_cm_color_gamut_set_cb)(u32 level, bool bcommit);
typedef void (*iris_dpp_gamma_set_i5_cb)(void);
typedef void (*iris_dpp_precsc_enable_cb)(u32 enable, bool bcommit);
typedef void (*iris_lux_set_cb)(u32 level, bool update);
typedef void (*iris_al_enable_cb)(bool enable);
typedef void (*iris_pwil_dport_disable_cb)(bool enable, u32 value);
typedef void (*iris_hdr_ai_input_bl_cb)(u32 bl_value, bool update);
typedef void (*iris_dom_set_cb)(int mode);
typedef void (*iris_csc2_para_set_cb)(uint32_t *values);
typedef void (*iris_pwil_dpp_en_cb)(bool dpp_en);
typedef void (*iris_dpp_en_cb)(bool dpp_en);
typedef int (*iris_EDR_backlight_ctrl_i7_cb)(u32 hdr_nit, u32 ratio_panel);
typedef void (*iris_frc_prepare_i5_cb)(struct iris_cfg *pcfg);
#if 0
typedef void (*iris_init_timing_switch_cb)(void);
typedef void (*iris_send_dynamic_seq_cb)(void);
typedef void (*iris_pre_switch_proc_cb)(void);
typedef void (*iris_post_switch_proc_cb)(void);
typedef void (*iris_dsc_up_format_i8_cb)(bool up, uint32_t *payload, uint32_t pps_sel);
typedef void (*iris_ioinc_pp_init_i8_cb)(void);
typedef void (*iris_ioinc_pp_disable_i8_cb)(void);
typedef void (*iris_ioinc_pp_filter_i8_cb)(uint32_t count, uint32_t *values);
#else
typedef void (*iris_memc_func_init_cb)(struct iris_memc_func *memc_func);
typedef void (*iris_memc_helper_setup_cb)(struct iris_memc_helper_ops *memc_helper_ops);
typedef void (*iris_timing_switch_setup_cb)(struct iris_timing_switch_ops *timing_switch_ops);
#endif
typedef int (*iris_convert_dsi_to_i2c_cb)(u8 *payload);
struct pw_chip_func_register_ops {
	iris_memc_func_init_cb iris_memc_func_init_;
	iris_memc_helper_setup_cb iris_memc_helper_setup_;
	iris_timing_switch_setup_cb iris_timing_switch_setup_;
	iris_debug_display_mode_get_cb iris_debug_display_mode_get_;
	iris_debug_pq_info_get_cb iris_debug_pq_info_get_;
	iris_send_lut_i5_cb iris_send_lut_i5_;
	iris_send_lut_cb iris_send_lut_;
	iris_configure_cb iris_configure_;
	iris_configure_ex_cb iris_configure_ex_;
	iris_configure_get_cb iris_configure_get_;
	iris_dbgfs_adb_type_init_cb iris_dbgfs_adb_type_init_;
	iris_ocp_write_mult_vals_i5_cb iris_ocp_write_mult_vals_i5_;
	iris_parse_frc_setting_cb iris_parse_frc_setting_;
	iris_clean_frc_status_cb iris_clean_frc_status_;
	iris_mult_addr_pad_cb iris_mult_addr_pad_;
	iris_get_dbc_lut_index_cb iris_get_dbc_lut_index_;
	iris_loop_back_validate_cb iris_loop_back_validate_;
	iris_mipi_rx0_validate_cb iris_mipi_rx0_validate_;
	iris_set_loopback_flag_cb iris_set_loopback_flag_;
	iris_get_loopback_flag_cb iris_get_loopback_flag_;
	iris_lp_preinit_i5_cb iris_lp_preinit_i5_;
	iris_lp_init_i5_cb iris_lp_init_i5_;
	iris_dynamic_power_set_i5_cb iris_dynamic_power_set_i5_;
	iris_disable_ulps_cb iris_disable_ulps_;
	iris_enable_ulps_cb iris_enable_ulps_;
	iris_dphy_itf_check_cb iris_dphy_itf_check_;
	iris_pmu_bsram_set_i5_cb iris_pmu_bsram_set_i5_;
	iris_dma_gen_ctrl_cb iris_dma_gen_ctrl_;
	iris_global_var_init_cb iris_global_var_init_;
	iris_pwil_update_i7p_cb iris_pwil_update_i7p_;
	iris_abypass_switch_proc_cb iris_abypass_switch_proc_;
	iris_esd_check_cb iris_esd_check_;
	iris_qsync_set_i5_cb iris_qsync_set_i5_;
	iris_change_dpp_lutrgb_type_addr_i7_cb iris_change_dpp_lutrgb_type_addr_i7_;
	iris_parse_lut_cmds_cb iris_parse_lut_cmds_;
	iris_get_hdr_enable_cb iris_get_hdr_enable_;
	iris_quality_setting_off_cb iris_quality_setting_off_;
	iris_end_dpp_cb iris_end_dpp_;
	iris_pq_parameter_init_cb iris_pq_parameter_init_;
	iris_init_tm_points_lut_cb iris_init_tm_points_lut_;
	iris_init_lut_buf_cb iris_init_lut_buf_;
	iris_cm_ratio_set_i5_cb iris_cm_ratio_set_i5_;
	iris_cm_ratio_set_cb iris_cm_ratio_set_;
	iris_cm_color_gamut_set_cb iris_cm_color_gamut_set_;
	iris_dpp_gamma_set_i5_cb iris_dpp_gamma_set_i5_;
	iris_dpp_precsc_enable_cb iris_dpp_precsc_enable_;
	iris_lux_set_cb iris_lux_set_;
	iris_al_enable_cb iris_al_enable_;
	iris_pwil_dport_disable_cb iris_pwil_dport_disable_;
	iris_hdr_ai_input_bl_cb iris_hdr_ai_input_bl_;
	iris_dom_set_cb iris_dom_set_;
	iris_csc2_para_set_cb iris_csc2_para_set_;
	iris_pwil_dpp_en_cb iris_pwil_dpp_en_;
	iris_dpp_en_cb iris_dpp_en_;
	iris_EDR_backlight_ctrl_i7_cb iris_EDR_backlight_ctrl_i7_;
	iris_frc_prepare_i5_cb iris_frc_prepare_i5_;
	iris_convert_dsi_to_i2c_cb iris_convert_dsi_to_i2c;
	#if 0
	iris_init_timing_switch_cb iris_init_timing_switch_;
	iris_send_dynamic_seq_cb iris_send_dynamic_seq_;
	iris_pre_switch_proc_cb iris_pre_switch_proc_;
	iris_post_switch_proc_cb iris_post_switch_proc_;
	iris_dsc_up_format_i8_cb iris_dsc_up_format_i8_;
	iris_ioinc_pp_init_i8_cb iris_ioinc_pp_init_i8_;
	iris_ioinc_pp_disable_i8_cb iris_ioinc_pp_disable_i8_;
	iris_ioinc_pp_filter_i8_cb iris_ioinc_pp_filter_i8_;
	#endif
};

enum IRIS_PARAM_VALID {
	PARAM_NONE = 0,
	PARAM_EMPTY,
	PARAM_PARSED,
	PARAM_PREPARED,
	PARAM_LIGHTUP,
};

/* memc demo window info */
enum IRIS_DEMO_WINDOW {
	DEMO_WINDOW_NONE = 0,
	DEMO_WINDOW_LEFT,
	DEMO_WINDOW_RIGHT,
	DEMO_WINDOW_DOWN,
	DEMO_WINDOW_UP,
	DEMO_WINDOW_FULL_OEM, /* OEM only distingiush imemc/ememc */
	DEMO_WINDOW_FULL, /* PW distingiush memc */
	DEMO_WINDOW_MAX,
};

/* 0: TOP 1: LEFT 2:BOTTOM 3: RIGHT */
enum DEMO_FROM {
	DEMO_TOP = 0,
	DEMO_LEFT,
	DEMO_BOTTOM,
	DEMO_RIGHT,
};

/* 0xf: off 0: inited 1: moving 2: still */
enum DEMO_STATE {
	DEMO_OFF = 0xf,
	DEMO_INITED = 0,
	DEMO_MOVE = 1,
	DEMO_STILL = 2,
};

enum {
	I2C_MULT_ADDR_VAL_PAIR,
	I2C_MULT_CONT_ADDR_VAL,
};

struct backlight_v2_data {
	int level;
	int delay; // This may block kickoff, so generally it is set to 0.
	int wait_vsync_count;
	ktime_t rd_ptr_ktime;
	spinlock_t bl_spinlock;
};

/* iris lightup configure commands */
struct iris_cfg {
	// struct dsi_display *display;
	// struct dsi_panel *panel;
	// move to struct of iris_vendor_cfg

	struct platform_device *pdev;
	struct {
		struct pinctrl *pinctrl;
		struct pinctrl_state *active;
		struct pinctrl_state *suspend;
	} pinctrl;
	int iris_reset_gpio;
	int iris_wakeup_gpio;
	int iris_abyp_ready_gpio;
	int iris_osd_gpio;
	int iris_vdd_gpio;

	/* hardware version and initialization status */
	uint8_t chip_id;
	uint32_t chip_ver;
	uint32_t chip_value[2];
	uint8_t valid; /* 0: none, 1: empty, 2: parse ok, 3: minimum light up, 4. full light up */
	//TODO
	bool iris_initialized;
	uint32_t platform_type; /* 0: FPGA, 1~: ASIC */
	uint32_t cmd_param_from_fw;
	bool mcu_code_downloaded;
	bool switch_bl_endian;
	bool iris_i2c_switch;
	bool iris_i2c_preload;
	struct work_struct iris_i2c_preload_work;

	/* static configuration */
	uint8_t panel_type;
	uint8_t lut_mode;
	uint32_t split_pkt_size;
	uint32_t min_color_temp;
	uint32_t max_color_temp;
	uint8_t rx_mode; /* 0: DSI_VIDEO_MODE, 1: DSI_CMD_MODE */
	uint8_t tx_mode;
	uint8_t read_path; /* 0: DSI, 1: I2C */
	const char *panel_name;

	/* current state */
	struct iris_lp_ctrl lp_ctrl;
	struct iris_abyp_ctrl abyp_ctrl;
	uint16_t panel_nits;
	uint32_t panel_dimming_brightness;
	uint8_t panel_hbm[2];
	struct iris_frc_setting frc_setting;
	int pwil_mode;
	struct iris_vc_ctrl vc_ctrl;

	uint32_t panel_te;
	uint32_t default_panel_te;
	uint32_t ap_te;
	uint32_t switch_mode;
	uint8_t power_mode;
	bool n2m_enable;
	int dport_output_mode;
	bool dynamic_vfr;
	u8 n2m_ratio;
	u32 dtg_ctrl_pt;
	uint32_t sw_n2m;

	/* secondary display related */
	// struct dsi_display *display2; // secondary display
	// struct dsi_panel *panel2;     // secondary panel
	// move to struct of iris_vendor_cfg

	// retain autorefresh in prepare_commit
	bool iris_cur_osd_autorefresh;
	bool iris_osd_autorefresh_enabled;
	bool iris_osd_irq_enabled;
	bool osd_enable;
	bool osd_on;
	bool osd_switch_on_pending;
	ktime_t ktime_osd_irq_clear;
	ktime_t ktime_kickoff_main;
	ktime_t ktime_kickoff_main_last;
	ktime_t ktime_kickoff_aux;
	ktime_t ktime_kickoff_aux_last;
	u32 osd_noof_max;
	u32 osd_repeat_max;
	atomic_t osd_irq_cnt;
	atomic_t video_update_wo_osd;
	bool dual_setting;
	uint32_t dual_test;
	bool osd_layer_empty;


	char display_mode_name[32];
	uint32_t app_version;
	uint32_t app_version1;
	uint8_t app_date[4];
	uint8_t abyp_prev_mode;
	struct clk *ext_clk;

	int32_t panel_pending;
	int32_t panel_delay;
	int32_t panel_level;

	bool aod;
	bool fod;
	bool fod_pending;
	atomic_t fod_cnt;

	/* configuration commands, parsed from dt, dynamically modified
	 * panel->panel_lock must be locked before access and for DSI command send
	 */
	uint32_t lut_cmds_cnt;
	uint32_t dtsi_cmds_cnt;
	uint32_t ip_opt_cnt;
	struct iris_ip_index ip_index_arr[IRIS_PIP_IDX_CNT][IRIS_IP_CNT];
	struct iris_ctrl_seq ctrl_seq[IRIS_CHIP_CNT];
	struct iris_ctrl_seq ctrl_seq_cs[IRIS_CHIP_CNT];
	struct iris_pq_init_val pq_init_val;
	struct iris_out_cmds iris_cmds;
	struct iris_pq_update_cmd pq_update_cmd;

	/* one wire gpio lock */
	spinlock_t iris_1w_lock;
	struct dentry *dbg_root;
	struct kobject *iris_kobj;
	struct work_struct lut_update_work;
	struct work_struct vfr_update_work;
	struct completion frame_ready_completion;

	/* hook for i2c extension */
	struct mutex gs_mutex;
	struct mutex ioctl_mutex;
	struct mutex i2c_read_mutex;
	iris_i2c_read_cb iris_i2c_read;
	iris_i2c_write_cb iris_i2c_write;
	iris_i2c_burst_write_cb iris_i2c_burst_write;
	iris_i2c_conver_ocp_write_cb iris_i2c_conver_ocp_write;
	struct iris_i2c_cfg iris_i2c_cfg;
	struct iris_i2c_cfg iris_pure_i2c_cfg;

	uint32_t metadata;

	/* iris status */
	bool iris_mipi1_power_st;
	bool ap_mipi1_power_st;
	bool iris_pwil_blend_st;
	bool iris_mipi1_power_on_pending;
	bool iris_osd_overflow_st;
	bool iris_frc_vfr_st;
	u32 iris_pwil_mode_state;
	bool dual_enabled;
	bool frc_enabled;
	bool frc_setting_ready;
	bool proFPGA_detected;

	struct iris_switch_dump switch_dump;
	struct iris_mspwil_setting chip_mspwil_setting;

	/* memc info */
	struct iris_memc_info memc_info;
	int osd_label;
	int frc_label;
	int frc_demo_window;
	bool frc_trilateral;
	u32 frc_app_info;
	bool frc_low_latency;

	/* emv info */
	struct extmv_frc_meta emv_info;

	/* emv perf info */
	struct extmv_clockinout emv_perf;

	/* pt_sr info*/
	bool pt_sr_enable;
	bool pt_sr_enable_restore;
	int pt_sr_hsize;
	int pt_sr_vsize;
	int pt_sr_guided_level;
	int pt_sr_dejaggy_level;
	int pt_sr_peaking_level;
	int pt_sr_DLTI_level;
	/* frc_pq info*/
	int frc_pq_guided_level;
	int frc_pq_dejaggy_level;
	int frc_pq_peaking_level;
	int frc_pq_DLTI_level;
	/* frcgame_pq info*/
	int frcgame_pq_guided_level;
	int frcgame_pq_dejaggy_level;
	int frcgame_pq_peaking_level;
	int frcgame_pq_DLTI_level;
	/*dsi send mode select*/
	uint8_t dsi_trans_mode[2];
	uint8_t *dsi_trans_buf;
	/*lightup pqupdate*/
	uint32_t dsi_trans_len[3][2];
	uint32_t ovs_delay;
	uint32_t ovs_delay_frc;
	uint32_t vsw_vbp_delay;
	bool dtg_eco_enabled;
	uint32_t ocp_read_by_i2c;
	int aux_width_in_using;
	int aux_height_in_using;
	/* calibration golden fw name */
	const char *ccf1_name;
	const char *ccf2_name;
	const char *ccf3_name;
#ifdef IRIS_EXT_CLK
	bool clk_enable_flag;
#endif
	/******TODO******/
	struct iris_mode_info timing;
	/*********************/
	atomic_t iris_esd_flag;
	uint32_t status_reg_addr;
	uint32_t id_sys_enter_abyp;
	uint32_t id_sys_exit_abyp;
	uint32_t ulps_mask_value;
	uint32_t id_piad_blend_info;
	uint32_t te_swap_mask_value;
	uint32_t id_tx_te_flow_ctrl;
	uint32_t id_tx_bypass_ctrl;
	uint32_t id_sys_mpg;
	uint32_t id_sys_dpg;
	uint32_t id_sys_ulps;
	uint32_t id_sys_abyp_ctrl;
	uint32_t id_sys_dma_ctrl;
	uint32_t id_sys_dma_gen_ctrl;
	uint32_t id_sys_te_swap;
	uint32_t id_sys_te_bypass;
	uint32_t id_sys_pmu_ctrl;
	uint32_t pq_pwr;
	uint32_t frc_pwr;
	uint32_t bsram_pwr;
	uint32_t id_rx_dphy;
	uint32_t id_rx_init;
	uint32_t iris_rd_packet_data;
	uint32_t iris_tx_intstat_raw;
	uint32_t iris_tx_intclr;
	uint32_t iris_mipi_tx_header_addr;
	uint32_t iris_mipi_tx_payload_addr;
	uint32_t iris_mipi_tx_header_addr_i3;
	uint32_t iris_mipi_tx_payload_addr_i3;
	uint32_t iris_dtg_addr;
	uint32_t dtg_ctrl;
	uint32_t dtg_update;
	uint32_t ovs_dly;
	uint32_t frc_dsc_init_delay;
	enum iris_chip_type iris_chip_type;
	u32 qsync_mode;
	struct device *dev;
	struct mipi_dsi_device *dsi_dev;
	bool memc_chain;
	bool memc_chain_en;
	struct iris_lightup_ops lightup_ops;
	u32 bl_max_level;
	iris_get_panel_mode_cb get_panel_mode;
	struct iris_memc_ops_cb iris_memc_ops;
	iris_wait_pre_frame_done_cb wait_pre_framedone;
	iris_wait_cmdq_done_cb wait_cmdq_done;
	iris_is_cmdq_empty_cb cmdq_empty;
	iris_lightup_cb iris_core_lightup;
	iris_clk_set_cb iris_clk_set;
	iris_set_esd_status_cb set_esd_status;
	iris_is_read_cmd_cb iris_is_read_cmd;
	iris_is_last_cmd_cb iris_is_last_cmd;
	iris_is_curmode_cmd_mode_cb iris_is_curmode_cmd_mode;
	iris_is_curmode_vid_mode_cb iris_is_curmode_vid_mode;
	iris_set_msg_flags_cb iris_set_msg_flags;
	iris_switch_cmd_type_cb iris_switch_cmd_type;
	iris_set_msg_ctrl_cb iris_set_msg_ctrl;
	struct pw_chip_func_register_ops pw_chip_func_ops;
	struct iris_ioctl_ops ioctl_ops;
	struct iris_platform_ops platform_ops;
	unsigned long long crtc0_old_interval;
	bool iris_mipi1_power_on_pending_en;

	/* demo wnd start */
	bool demo_enable;
	uint32_t demo_duration;
	uint32_t demo_from; /* 0: TOP 1: LEFT 2:BOTTOM 3: RIGHT */
	uint32_t demo_argb;
	uint32_t demo_start_x;
	uint32_t demo_start_y;
	uint32_t demo_end_x;
	uint32_t demo_end_y;
	uint32_t demo_step;
	uint32_t demo_interval;
	uint32_t demo_offset;
	uint32_t demo_cur_start_x;
	uint32_t demo_cur_start_y;
	uint32_t demo_cur_end_x;
	uint32_t demo_cur_end_y;
	uint32_t demo_state; /* 0xf: off 0: inited 1: moving 2: still */
	uint32_t demo_border;
	uint32_t demo_color_en;
	uint32_t demo_color;
	struct workqueue_struct *demo_wnd_wq;
	struct delayed_work demo_wnd_wk;
	/* demo wnd end */

	u32 iris_pq_disable;
	uint32_t default_dma_gen_ctrl;
	uint32_t default_dma_gen_ctrl_2;

	bool mcu_running;
	bool pwil_abnormal;
	bool prev_mode_rfb;
	bool debug_frc_switch;

	uint32_t ocp_write_by_i2c;
	bool force_i2c_type;

	/*iris abypass function*/
	bool iris_abyp_ops_init;
	struct iris_abyp_ops abyp_ops;

	struct backlight_v2_data backlight_v2;
};

#endif // _DSI_IRIS_DEF_H_
