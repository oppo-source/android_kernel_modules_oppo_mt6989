/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _GAME_H_
#define _GAME_H_

extern void (*game_set_heaviest_pid_fp)(int heaviest_pid);
extern void (*game_engine_cooler_set_last_sleep_duration_fp)(int cur_pid);
extern void (*game_set_fps_fp)(int pid, int target_fps);
extern int fpsgo_other2fstb_get_fps(int pid, unsigned long long bufID,
	int *qfps_arr, int *qfps_num, int max_qfps_num,
	int *tfps_arr, int *tfps_num, int max_tfps_num);
#endif //_GAME_H_
