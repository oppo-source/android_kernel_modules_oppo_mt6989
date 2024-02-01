// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_MTK_TINYSYS_SSPM_SUPPORT)
#include <sspm_reservedmem.h>
#endif

#include <mtk_swpm_common_sysfs.h>
#include <mtk_swpm_sysfs.h>
#include <swpm_module.h>
#include <swpm_module_ext.h>
#include <swpm_v6985.h>
#include <swpm_v6985_ext.h>

#define SWPM_INTERNAL_TEST (0)
#define DEFAULT_UPDATE_MS (10000)
#define RETRY_UPDATE_MS (1000)
#define CORE_SRAM (share_idx_ref_ext->core_idx_ext)
#define DDR_SRAM (share_idx_ref_ext->mem_idx_ext)
#define SUSPEND_SRAM (share_idx_ref_ext->suspend)

#define OPP_FREQ_TO_DDR(x) \
	((x == 1066 || x == 1333 || x == 4266) ? (x * 2 + 1) : (x * 2))

static struct timer_list swpm_sp_timer;
static DEFINE_SPINLOCK(swpm_sp_spinlock);

/* share sram for extension index */
static struct share_index_ext *share_idx_ref_ext;
static struct share_ctrl_ext *share_idx_ctrl_ext;

/* share sram for static data */
static struct subsys_swpm_data *subsys_swpm_data_ptr;
static struct mem_swpm_data *mem_swpm_data_ptr;
static struct core_swpm_data *core_swpm_data_ptr;

static unsigned int update_interval_ms = DEFAULT_UPDATE_MS;

/* core voltage time distribution */
static struct vol_duration core_vol_duration[NR_CORE_VOLT];
/* core ip stat with time distribution */
static struct ip_stats core_ip_stats[NR_CORE_IP];

/* ddr freq in active time distribution */
static struct ddr_act_times ddr_act_duration[NR_DDR_FREQ];
/* ddr freq in active time distribution */
static struct ddr_sr_pd_times ddr_sr_pd_duration;
/* ddr ip stat with bw/freq distribution */
static struct ddr_ip_bc_stats ddr_ip_stats[NR_DDR_BC_IP];

struct suspend_time suspend_time;
static uint64_t total_suspend_us;

/* core ip (mmsys, venc, vdec, scp */
static char core_ip_str[NR_CORE_IP][MAX_IP_NAME_LENGTH] = {
	"MDP", "DISP", "VENC", "VDEC", "SCP",
};
/* ddr bw ip (total r/total w/cpu/gpu/mm/md) */
static char ddr_bc_ip_str[NR_DDR_BC_IP][MAX_IP_NAME_LENGTH] = {
	"TOTAL_R", "TOTAL_W", "CPU", "GPU", "MM", "OTHERS",
};

/* static unsigned int retry_cnt = 0; */

/* critical section function */
static void swpm_sp_internal_update(void)
{
	int i, j;
	struct core_ip_pwr_sta *core_ip_sta_ptr;
	struct mem_ip_bc *ddr_ip_bc_ptr;
	unsigned int word_L, word_H;

	if (share_idx_ref_ext && share_idx_ctrl_ext) {

		if (share_idx_ctrl_ext->clear_flag)
			return;

		if (share_idx_ctrl_ext->write_lock) {
			update_interval_ms = RETRY_UPDATE_MS;
			/* retry_cnt++; */
			return;
		}
		share_idx_ctrl_ext->read_lock = 1;

		for (i = 0; i < NR_CORE_VOLT; i++) {
			core_vol_duration[i].duration +=
				CORE_SRAM.acc_time[i] / 1000;
		}
		for (i = 0; i < NR_DDR_FREQ; i++) {
			ddr_act_duration[i].active_time +=
			DDR_SRAM.acc_time[i] / 1000;
		}
		ddr_sr_pd_duration.sr_time +=
			DDR_SRAM.acc_sr_time / 1000;
		ddr_sr_pd_duration.pd_time +=
			DDR_SRAM.acc_pd_time / 1000;
		for (i = 0; i < NR_CORE_IP; i++) {
			if (!core_ip_stats[i].times)
				continue;
			core_ip_sta_ptr = &(CORE_SRAM.pwr_state[i]);
			core_ip_stats[i].times->active_time +=
			core_ip_sta_ptr->state[PMSR_ACTIVE] / 1000;
			core_ip_stats[i].times->idle_time +=
			core_ip_sta_ptr->state[PMSR_IDLE] / 1000;
			core_ip_stats[i].times->off_time +=
			core_ip_sta_ptr->state[PMSR_OFF] / 1000;
		}
		for (i = 0; i < NR_DDR_BC_IP; i++) {
			if (!ddr_ip_stats[i].bc_stats)
				continue;
			ddr_ip_bc_ptr = &(DDR_SRAM.data[i]);
			for (j = 0; j < NR_DDR_FREQ; j++) {
				word_H = ddr_ip_bc_ptr->word_cnt_H[j];
				word_L = ddr_ip_bc_ptr->word_cnt_L[j];
				ddr_ip_stats[i].bc_stats[j].value +=
				(((uint64_t) word_H << 32) | word_L) * 8;
			}
		}

		swpm_v6985_sub_ext_update();

		suspend_time.time_H = SUSPEND_SRAM.time_H;
		suspend_time.time_L = SUSPEND_SRAM.time_L;
		total_suspend_us +=
			((uint64_t)suspend_time.time_H << 32) |
			suspend_time.time_L;

		share_idx_ctrl_ext->clear_flag = 1;
		share_idx_ctrl_ext->read_lock = 0;
	}
}

static void swpm_sp_routine(struct timer_list *t)
{
	unsigned long flags;

	spin_lock_irqsave(&swpm_sp_spinlock, flags);
	swpm_sp_internal_update();
	spin_unlock_irqrestore(&swpm_sp_spinlock, flags);

/*	if (update_interval_ms == DEFAULT_UPDATE_MS)
 *		pr_notice("swpm_sp_routine regular update(%d), total_suspend(%llu)\n",
 *				  retry_cnt, total_suspend_us);
 *	else
 *		pr_notice("swpm_sp_routine retry update(%d), total_suspend(%llu)\n",
 *				  retry_cnt, total_suspend_us);
 */

	mod_timer(t, jiffies + msecs_to_jiffies(update_interval_ms));
	update_interval_ms = DEFAULT_UPDATE_MS;
}

static int swpm_sp_dispatcher(unsigned int type,
			       unsigned int val)
{
	switch (type) {
	case SYNC_DATA:
		/* do update */
		swpm_sp_routine(&swpm_sp_timer);
		break;
	case SET_INTERVAL:
		/* set update interval */
		break;
	}

	return 0;
}

static int32_t swpm_ddr_act_times(int32_t freq_num,
			      struct ddr_act_times *ddr_times)
{
	unsigned long flags;

	if (ddr_times && freq_num == NR_DDR_FREQ) {
		spin_lock_irqsave(&swpm_sp_spinlock, flags);
		memcpy(ddr_times, ddr_act_duration,
		       sizeof(struct ddr_act_times) * NR_DDR_FREQ);
		spin_unlock_irqrestore(&swpm_sp_spinlock, flags);
	}
	return 0;
}
static int32_t swpm_ddr_sr_pd_times(struct ddr_sr_pd_times *ddr_times)
{
	unsigned long flags;

	if (ddr_times) {
		spin_lock_irqsave(&swpm_sp_spinlock, flags);
		memcpy(ddr_times, &ddr_sr_pd_duration,
		       sizeof(struct ddr_sr_pd_times));
		spin_unlock_irqrestore(&swpm_sp_spinlock, flags);
	}
	return 0;
}
static int32_t swpm_ddr_freq_data_ip_stats(int32_t data_ip_num,
					   int32_t freq_num,
					   void *stats)
{
	unsigned long flags;
	int i;
	struct ddr_ip_bc_stats *p = stats;

	if (p && data_ip_num == NR_DDR_BC_IP && freq_num == NR_DDR_FREQ) {
		spin_lock_irqsave(&swpm_sp_spinlock, flags);
		for (i = 0; i < NR_DDR_BC_IP && p[i].bc_stats; i++) {
			strncpy(p[i].ip_name,
				ddr_ip_stats[i].ip_name,
				MAX_IP_NAME_LENGTH - 1);
			memcpy(p[i].bc_stats,
			       ddr_ip_stats[i].bc_stats,
			       sizeof(struct ddr_bc_stats) * NR_DDR_FREQ);
		}
		spin_unlock_irqrestore(&swpm_sp_spinlock, flags);
	}
	return 0;
}
static int32_t swpm_vcore_ip_vol_stats(int32_t ip_num,
				       int32_t vol_num,
				       void *stats)
{
	unsigned long flags;
	int i;
	struct ip_stats *p = stats;

	if (p && ip_num == NR_CORE_IP && vol_num == NR_CORE_VOLT) {
		spin_lock_irqsave(&swpm_sp_spinlock, flags);
		for (i = 0; i < NR_CORE_IP && p[i].times; i++) {
			strncpy(p[i].ip_name,
				core_ip_stats[i].ip_name,
				MAX_IP_NAME_LENGTH - 1);
			memcpy(p[i].times,
			       core_ip_stats[i].times,
			       sizeof(struct ip_times));
		}
		spin_unlock_irqrestore(&swpm_sp_spinlock, flags);
	}
	return 0;
}
static int32_t swpm_vcore_vol_duration(int32_t vol_num,
				       struct vol_duration *duration)
{
	unsigned long flags;

	if (duration && vol_num == NR_CORE_VOLT) {
		spin_lock_irqsave(&swpm_sp_spinlock, flags);
		memcpy(duration, core_vol_duration,
		       sizeof(struct vol_duration) * NR_CORE_VOLT);
		spin_unlock_irqrestore(&swpm_sp_spinlock, flags);
	}
	return 0;
}
static int32_t swpm_plat_nums(enum swpm_num_type type)
{
	switch (type) {
	case DDR_DATA_IP:
		return NR_DDR_BC_IP;
	case DDR_FREQ:
		return NR_DDR_FREQ;
	case CORE_IP:
		return NR_CORE_IP;
	case CORE_VOL:
		return NR_CORE_VOLT;
	default:
		return 0;
	}
	return 0;
}

static struct swpm_internal_ops plat_ops = {
	.cmd = swpm_sp_dispatcher,
	.ddr_act_times_get = swpm_ddr_act_times,
	.ddr_sr_pd_times_get = swpm_ddr_sr_pd_times,
	.ddr_freq_data_ip_stats_get =
		swpm_ddr_freq_data_ip_stats,
	.vcore_ip_vol_stats_get =
		swpm_vcore_ip_vol_stats,
	.vcore_vol_duration_get =
		swpm_vcore_vol_duration,
	.num_get = swpm_plat_nums,
};

/* critical section function */
static void swpm_sp_timer_init(void)
{
	swpm_sp_timer.function = swpm_sp_routine;
	timer_setup(&swpm_sp_timer, swpm_sp_routine, TIMER_DEFERRABLE);
	mod_timer(&swpm_sp_timer,
		  jiffies + msecs_to_jiffies(update_interval_ms));
}

void swpm_v6985_ext_init(void)
{
	int i, j;

	/* init extension index address */
	if (wrap_d) {
		share_idx_ref_ext =
		(struct share_index_ext *)
		sspm_sbuf_get(wrap_d->share_index_ext_addr);
		share_idx_ctrl_ext =
		(struct share_ctrl_ext *)
		sspm_sbuf_get(wrap_d->share_ctrl_ext_addr);
		subsys_swpm_data_ptr =
		(struct subsys_swpm_data *)
		sspm_sbuf_get(wrap_d->subsys_swpm_data_addr);
	} else {
		share_idx_ref_ext = NULL;
		share_idx_ctrl_ext = NULL;
		subsys_swpm_data_ptr = NULL;
	}

	if (subsys_swpm_data_ptr) {
		mem_swpm_data_ptr = (struct mem_swpm_data *)
		sspm_sbuf_get(subsys_swpm_data_ptr->mem_swpm_data_addr);
		core_swpm_data_ptr = (struct core_swpm_data *)
		sspm_sbuf_get(subsys_swpm_data_ptr->core_swpm_data_addr);
	} else {
		mem_swpm_data_ptr = NULL;
		core_swpm_data_ptr = NULL;
	}

	/* core_ip_stats initialize */
	for (i = 0; i < NR_CORE_IP; i++) {
		strncpy(core_ip_stats[i].ip_name,
			core_ip_str[i], MAX_IP_NAME_LENGTH - 1);
		core_ip_stats[i].times =
		kmalloc(sizeof(struct ip_vol_times), GFP_KERNEL);

		core_ip_stats[i].times->active_time = 0;
		core_ip_stats[i].times->idle_time = 0;
		core_ip_stats[i].times->off_time = 0;
	}
	/* core duration initialize */
	for (i = 0; core_swpm_data_ptr && i < NR_CORE_VOLT; i++) {
		core_vol_duration[i].duration = 0;
		core_vol_duration[i].vol =
			core_swpm_data_ptr->core_volt_tbl[i];
	}

	/* ddr act duration initialize */
	for (i = 0; mem_swpm_data_ptr && i < NR_DDR_FREQ; i++) {
		ddr_act_duration[i].active_time = 0;
		ddr_act_duration[i].freq =
		OPP_FREQ_TO_DDR(mem_swpm_data_ptr->ddr_opp_freq[i]);
	}
	/* ddr sr pd duration initialize */
	ddr_sr_pd_duration.sr_time = 0;
	ddr_sr_pd_duration.pd_time = 0;

	/* ddr bc ip initialize */
	for (i = 0; i < NR_DDR_BC_IP; i++) {
		strncpy(ddr_ip_stats[i].ip_name,
			ddr_bc_ip_str[i], MAX_IP_NAME_LENGTH - 1);
		ddr_ip_stats[i].bc_stats =
		kmalloc(sizeof(struct ddr_bc_stats) * NR_DDR_FREQ, GFP_KERNEL);
		if (ddr_ip_stats[i].bc_stats) {
			for (j = 0; mem_swpm_data_ptr &&
			     j < NR_DDR_FREQ; j++) {
				ddr_ip_stats[i].bc_stats[j].value = 0;
				ddr_ip_stats[i].bc_stats[j].freq =
				OPP_FREQ_TO_DDR(mem_swpm_data_ptr->ddr_opp_freq[j]);
			}
		}
	}
	total_suspend_us = 0;

	swpm_v6985_sub_ext_init();

#if SWPM_TEST
	pr_notice("share_index_ext size = %zu bytes\n",
		sizeof(struct share_index_ext));
#endif

	swpm_sp_timer_init();

	mtk_register_swpm_ops(&plat_ops);
}

void swpm_v6985_ext_exit(void)
{
	del_timer_sync(&swpm_sp_timer);
}
