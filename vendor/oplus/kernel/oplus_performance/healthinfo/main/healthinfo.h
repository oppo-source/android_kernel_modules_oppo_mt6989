/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */

#ifndef _HEALTHINFO_H_
#define _HEALTHINFO_H_

#include <linux/latencytop.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/cpuidle.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cgroup.h>
#include <linux/healthinfo/fg.h>

#define ohm_err(fmt, ...) \
        printk(KERN_ERR "[OHM_ERR][%s]"fmt, __func__, ##__VA_ARGS__)
#define ohm_debug(fmt, ...) \
        printk(KERN_INFO "[OHM_INFO][%s]"fmt, __func__, ##__VA_ARGS__)
#define ohm_debug_deferred(fmt, ...) \
		printk_deferred(KERN_INFO "[OHM_INFO][%s]"fmt, __func__, ##__VA_ARGS__)
#define ohm_err_deferred(fmt, ...) \
        printk_deferred(KERN_ERR "[OHM_ERR][%s]"fmt, __func__, ##__VA_ARGS__)

#if IS_ENABLED(CONFIG_CGROUP_SCHED)
#define SA_CGROUP_SYS_BACKGROUND	(1)
#endif
#define SA_CGROUP_FOREGROUND		(2)
#define SA_CGROUP_BACKGROUND		(3)
#if IS_ENABLED(CONFIG_CGROUP_SCHED)
#define SA_CGROUP_TOP_APP			(4)
#define SA_CGROUP_UX				(9)
#endif

#define OHM_FLASH_TYPE_EMC 1
#define OHM_FLASH_TYPE_UFS 2
#define SA_CGROUP_FOREGROUND        (2)
#define SA_CGROUP_BACKGROUND        (3)
#define OHM_SCHED_TYPE_MAX 12

enum {
	TYPE_TOPAPP,
	TYPE_UNTOPAPP,
	TYPE_ALL,
	TYPE_DEFAULT,
};
enum {
	IN_THREAD_TODO_LIST,
	IN_PROC_TODO_LIST,
	IN_ALL,
	IN_DEFAULT,
};

struct oplus_binder_proc_status{
	bool ctrl;
	int warning;
	int warning_cg_bg;
	int warning_cg_fg;
	int trans2proc[TYPE_DEFAULT][IN_DEFAULT];
	char comm[TASK_COMM_LEN];
};

enum {
        /* SCHED_STATS 0 -11 */
        OHM_SCHED_IOWAIT = 0,
        OHM_SCHED_SCHEDLATENCY,
        OHM_SCHED_FSYNC,
        OHM_SCHED_EMMCIO,
        OHM_SCHED_DSTATE,
        OHM_SCHED_TOTAL,
        /* OTHER_TYPE 12 - */
        OHM_CPU_LOAD_CUR = OHM_SCHED_TYPE_MAX,
        OHM_MEM_MON,
        OHM_IOPANIC_MON,
        OHM_SVM_MON,
        OHM_RLIMIT_MON,
        OHM_ION_MON,
		OHM_MEM_VMA_ALLOC_ERR,
        OHM_BLK_MON,
        OHM_TYPE_TOTAL
};

#ifdef CONFIG_OPLUS_BINDER_STRATEGY
struct ob_struct{
	struct binder_proc *ob_proc;
	struct list_head ob_list;
	pid_t pid;
	u64 ob_check_ts;
	bool init;
};

struct binder_proc_status{
	u64 warning;
	u64 warning_cg_bg;
	u64 async_mem_over_high;
	u64 async_mem_over_low;
	u64 sync_mem_over_high;
	u64 sync_mem_over_low;
};
#endif

struct sched_stat_common {
        u64 max_ms;
        u64 high_cnt;
        u64 low_cnt;
        u64 total_ms;
        u64 total_cnt;
};

struct long_wait_record {
	u32 pid;
	u32 priv;
	u64 timestamp;
	u32 ms;
};

#define LWR_SHIFT	3
#define LWR_MASK	((1ULL << LWR_SHIFT) - 1)
#define LWR_SIZE	(1ULL << LWR_SHIFT)

struct sched_stat_para {
        bool ctrl;
        bool logon;
        bool trig;
        int low_thresh_ms;
        int high_thresh_ms;
        u64 delta_ms;
	spinlock_t lock;
        struct sched_stat_common all;
        struct sched_stat_common fg;
        struct sched_stat_common ux;
	struct sched_stat_common top;
	struct sched_stat_common bg;
	struct sched_stat_common sysbg;
	atomic_t lwr_index;
	struct long_wait_record last_n_lwr[LWR_SIZE];
};
extern struct sched_stat_para sched_para[OHM_SCHED_TOTAL];

struct alloc_wait_para {
	u64 total_alloc_wait_max_order;
	u64 fg_alloc_wait_max_order;
	u64 ux_alloc_wait_max_order;
	struct sched_stat_common total_alloc_wait;
	struct sched_stat_common fg_alloc_wait;
	struct sched_stat_common ux_alloc_wait;
	atomic_t lwr_index;
	struct long_wait_record last_n_lwr[LWR_SIZE];
};

struct ion_wait_para {
	struct sched_stat_common ux_ion_wait;
	struct sched_stat_common fg_ion_wait;
	struct sched_stat_common total_ion_wait;
	atomic_t lwr_index;
	struct long_wait_record last_n_lwr[LWR_SIZE];
};

struct blk_wait_para {
	int wait_h_ms;
	int wait_l_ms;
	struct sched_stat_common wait_stat;
	atomic_t lwr_index;
	struct long_wait_record last_n_lwr[LWR_SIZE];
};

extern void ohm_schedstats_record(int sched_type, struct task_struct *task, u64 delta_ms);
extern int ohm_get_cur_cpuload(bool ctrl);
extern void ohm_action_trig_with_msg(int type, char *msg);
extern struct oplus_binder_proc_status system_server_proc;
#ifdef CONFIG_OPLUS_BINDER_STRATEGY
extern struct binder_proc_status system_server_proc_status;
#endif
#endif /* _HEALTHINFO_H_*/

