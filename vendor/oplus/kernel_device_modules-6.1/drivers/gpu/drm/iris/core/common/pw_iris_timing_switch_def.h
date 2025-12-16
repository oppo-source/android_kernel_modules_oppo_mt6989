/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */
#ifndef __PW_IRIS_TIMING_SWITCH_DEF__
#define __PW_IRIS_TIMING_SWITCH_DEF__
#include "pw_iris_log.h"

enum {
	LOG_NORMAL_LEVEL = 0,
	LOG_DEBUG_LEVEL,
	LOG_VERBOSE_LEVEL,
	LOG_VERY_VERBOSE_LEVEL,
	LOG_LEVEL_COUNT
};

enum {
	DBG_DUMP_CMDLIST = 0,
	DBG_SEND_DTG_ONLY,
	DBG_DPORT_SKIP_FRAME,
	DBG_PRE_CAP_SKIP_FRAME,
	DBG_POST_CAP_SKIP_FRAME,
	DBG_PRE_DELAY_MS,
	DBG_POST_DELAY_MS,
	DBG_DISABLE_ALL = 0x0F,
};

inline uint32_t iris_get_tm_sw_loglevel(void);
#define LOG_NORMAL_INFO	(IRIS_IF_LOGI())
#define LOG_DEBUG_INFO		\
	((iris_get_tm_sw_loglevel() >= LOG_DEBUG_LEVEL) || IRIS_IF_LOGD())
#define LOG_VERBOSE_INFO	\
	((iris_get_tm_sw_loglevel() >= LOG_VERBOSE_LEVEL) || IRIS_IF_LOGV())
#define LOG_VERY_VERBOSE_INFO	\
	((iris_get_tm_sw_loglevel() >= LOG_VERY_VERBOSE_LEVEL) || IRIS_IF_LOGVV())

/* Internal interface for I5 */
//void iris_init_timing_switch_i5(void);
//void iris_send_dynamic_seq_i7(void);

/* Internal interface for I7 */
void iris_init_timing_switch_i7(void);
void iris_send_dynamic_seq_i7(void);


/* Internal interface for I7P */
void iris_init_timing_switch_i7p(void);
void iris_send_dynamic_seq_i7p(void);


/* Internal interface for I8 */
void iris_init_timing_switch_i8(void);
void iris_send_dynamic_seq_i8(void);
void iris_pre_switch_i8(uint32_t refresh_rate, bool clock_changed);
void iris_pre_switch_proc_i8(void);
void iris_post_switch_proc_i8(void);
void iris_set_tm_sw_dbg_param_i8(uint32_t type, uint32_t value);
void iris_restore_capen_i8(void);

#endif //__DSI_IRIS_TIMING_SWITCH_DEF__
