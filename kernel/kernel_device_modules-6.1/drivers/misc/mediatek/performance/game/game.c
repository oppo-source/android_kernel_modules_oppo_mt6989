// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/module.h>
#include <trace/hooks/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include "game.h"

static int yield_pid = -1;
static int yield_duration = 2000;
static int lr_frame_time_buffer = 300000;
static int smallest_yield_time;
static int last_sleep_duration;
static int render_pid = -1;
static int targetFps = -1;
#define GAME_VERSION_MODULE "1.0"
#define MAX_RENDER_TID 10

enum TASK_STATE {
	IDLE_STATE = 0,
};

module_param(yield_duration, int ,0644);
module_param(lr_frame_time_buffer, int ,0644);
module_param(smallest_yield_time, int, 0644);

struct render_info_fps {
	int cur_fps;
	int target_fps;
};

static int game_get_tgid(int pid)
{
	struct task_struct *tsk;
	int tgid = 0;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (tsk)
		get_task_struct(tsk);
	rcu_read_unlock();

	if (!tsk)
		return 0;

	tgid = tsk->tgid;
	put_task_struct(tsk);

	return tgid;
}

void game_set_heaviest_pid(int heaviest_pid)
{
	yield_pid = heaviest_pid;
}

void game_set_fps(int pid, int target_fps)
{
	if (pid == render_pid)
		targetFps = target_fps / 1000;
}

void game_engine_cooler_set_last_sleep_duration(int cur_pid)
{
	if (yield_pid != -1 && game_get_tgid(cur_pid) == game_get_tgid(yield_pid)) {
		last_sleep_duration = 0;
		render_pid = cur_pid;
	}
}

void game_register_func(void *funcPtr, void *data)
{
	register_trace_android_rvh_before_do_sched_yield(funcPtr, data);
}
EXPORT_SYMBOL(game_register_func);

int engine_cooler_get_last_sleep_duration(void)
{
	return last_sleep_duration;
}
EXPORT_SYMBOL(engine_cooler_get_last_sleep_duration);

void engine_cooler_set_last_sleep_duration(int duration)
{
	last_sleep_duration = duration;
}
EXPORT_SYMBOL(engine_cooler_set_last_sleep_duration);

int engine_cooler_get_yield_monitor_pid(void)
{
	return yield_pid;
}
EXPORT_SYMBOL(engine_cooler_get_yield_monitor_pid);


int engine_cooler_get_firtst_sleep_duration(void)
{
	return yield_duration * 1000;
}
EXPORT_SYMBOL(engine_cooler_get_firtst_sleep_duration);

int engine_cooler_get_target_fps(void)
{

	if (targetFps != -1) {
		return targetFps;
		pr_debug("targetFPS is %d\n", targetFps);
	}


	if (render_pid != -1) {
		int ret = 0;
		int *local_qfps_arr = NULL;
		int *local_tfps_arr = NULL;
		int local_qfps_arr_num = 0;
		int local_tfps_arr_num = 0;

		local_qfps_arr = kcalloc(1, sizeof(int), GFP_KERNEL);
		if (!local_qfps_arr)
			goto out;

		local_tfps_arr = kcalloc(1, sizeof(int), GFP_KERNEL);
		if (!local_tfps_arr)
			goto out;

		ret = fpsgo_other2fstb_get_fps(render_pid, 0,
		local_qfps_arr, &local_qfps_arr_num, 1,
		local_tfps_arr, &local_tfps_arr_num, 1);

		if (ret == 0 && !local_tfps_arr[0])
			targetFps = local_tfps_arr[0];
out:
		kfree(local_qfps_arr);
		kfree(local_tfps_arr);
	}
	return targetFps;
}
EXPORT_SYMBOL(engine_cooler_get_target_fps);

int engine_cooler_get_lr_frame_time_buffer(void)
{
	return lr_frame_time_buffer;
}
EXPORT_SYMBOL(engine_cooler_get_lr_frame_time_buffer);

int engine_cooler_get_smallest_yield_time(void)
{
	return smallest_yield_time;
}
EXPORT_SYMBOL(engine_cooler_get_smallest_yield_time);

u64 get_now_time(void)
{
	return sched_clock();
}
EXPORT_SYMBOL(get_now_time);

void call_usleep_range_state(int min, int max, int state)
{
	if (state == IDLE_STATE)
		usleep_range_state(min, max, TASK_IDLE);
}
EXPORT_SYMBOL(call_usleep_range_state);

static void __exit game_exit(void)
{

}

static int __init game_init(void)
{
	game_set_heaviest_pid_fp = game_set_heaviest_pid;
	game_engine_cooler_set_last_sleep_duration_fp = game_engine_cooler_set_last_sleep_duration;
	game_set_fps_fp = game_set_fps;
	return 0;
}

module_init(game_init);
module_exit(game_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek GAME");
MODULE_AUTHOR("MediaTek Inc.");
MODULE_VERSION(GAME_VERSION_MODULE);
