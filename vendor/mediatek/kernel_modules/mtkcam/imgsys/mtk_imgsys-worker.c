// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 *
 * Author: Johnson-CH Chiu <johnson-ch.chiu@mediatek.com>
 *
 */
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#include "mtk_imgsys-worker.h"
#include "mtk_imgsys-trace.h"
#include "mtk_imgsys-v4l2-debug.h"

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif

#ifdef OPLUS_FEATURE_CAMERA_COMMON
struct imgsys_daemon_info
{
	struct task_struct *daemon_task;
	wait_queue_head_t wq;
	atomic_t is_thread_run;
};

extern int imgsys_daemon_create(struct imgsys_daemon_info *info, int (*threadfn)(void *data), const char name[], bool is_fifo);
static struct imgsys_daemon_info s_daemon_info = {.daemon_task = NULL};
static struct imgsys_queue *s_imgsys_que = NULL;
static int worker_func(void *data);

int imgsys_worker_daemon_create(void)
{
	imgsys_daemon_create(&s_daemon_info, worker_func, "imgsys-cmdq-daemon", false);
	return 0;
}
#endif

int imgsys_queue_init(struct imgsys_queue *que, struct device *dev, char *name)
{
	int ret = 0;

	if ((!que) || (!dev)) {
		ret = -1;
		goto EXIT;
	}

	que->name = name;
	que->dev = dev;
	INIT_LIST_HEAD(&que->queue);
	init_waitqueue_head(&que->wq);
	init_waitqueue_head(&que->dis_wq);
	spin_lock_init(&que->lock);
	atomic_set(&que->nr, 0);
	que->peak = 0;
	mutex_init(&que->task_lock);

EXIT:
	return ret;
}

static int worker_func(void *data)
{
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	struct imgsys_queue *head = NULL;
#else
	struct imgsys_queue *head = data;
#endif
	struct imgsys_work *node;
	struct list_head *list;
	u64 start;
	u64 end;

#ifdef OPLUS_FEATURE_CAMERA_COMMON
	printk("%s: launch", __func__);
	while (1) {
		printk("%s: wait", __func__);
		wait_event_interruptible(s_daemon_info.wq, atomic_read(&s_daemon_info.is_thread_run));
		head = s_imgsys_que;
		node = NULL;
		list = NULL;
		printk("%s: task enqueue", __func__);
#endif
	while (1) {
        if (imgsys_dbg_enable())
		dev_dbg(head->dev, "%s: %s kthread sleeps\n", __func__,
								head->name);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		wait_event_interruptible(s_daemon_info.wq,
#else
		wait_event_interruptible(head->wq,
#endif
			atomic_read(&head->nr) || atomic_read(&head->disable));
        if (imgsys_dbg_enable())
		dev_dbg(head->dev, "%s: %s kthread wakes dis/nr(%d/%d)\n", __func__,
				head->name, atomic_read(&head->disable), atomic_read(&head->nr));

		spin_lock(&head->lock);
		if (atomic_read(&head->disable) || !atomic_read(&head->nr)) {
			spin_unlock(&head->lock);
			dev_info(head->dev, "%s: %s: nr(%d) dis(%d)\n", __func__,
					head->name, atomic_read(&head->nr),
						atomic_read(&head->disable));
#ifdef OPLUS_FEATURE_CAMERA_COMMON
			atomic_set(&s_daemon_info.is_thread_run, 0);
#endif
			goto next;
		}

		list = head->queue.next;
		list_del(list);
		atomic_dec(&head->nr);
		spin_unlock(&head->lock);

		node = list_entry(list, struct imgsys_work, entry);

		IMGSYS_SYSTRACE_BEGIN("%s work:%p nr:%d\n", __func__, node, atomic_read(&head->nr));

		start = ktime_get_boottime_ns();
		if (node->run)
			node->run(node);
		end = ktime_get_boottime_ns();
		if ((end - start) > 2000000)
			dev_info(head->dev, "%s: work run time %lld > 2ms\n",
			__func__, (end - start));

		IMGSYS_SYSTRACE_END();

next:
#ifdef OPLUS_FEATURE_CAMERA_COMMON
		if (imgsys_dbg_enable()) {
			dev_dbg(head->dev, "%s: %s kthread exits\n", __func__, head->name);
		}
		break;
	}
	}
#else
		if (kthread_should_stop()) {
            if (imgsys_dbg_enable())
			dev_dbg(head->dev, "%s: %s kthread exits\n", __func__, head->name);
			break;
		}
	}
#endif

	dev_info(head->dev, "%s: %s exited\n", __func__, head->name);

	return 0;
}

int imgsys_queue_enable(struct imgsys_queue *que)
{
	if (!que)
		return -1;

	mutex_lock(&que->task_lock);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	if (IS_ERR(s_daemon_info.daemon_task)) {
		dev_info(que->dev, "%s: imgsys-cmdq-daemon NULL error\n", __func__);
		mutex_unlock(&que->task_lock);
		return PTR_ERR(s_daemon_info.daemon_task);
	}
	que->task = s_daemon_info.daemon_task;
	s_imgsys_que = que;
	atomic_set(&que->disable, 0);
	atomic_set(&s_daemon_info.is_thread_run, 1);
	wake_up(&s_daemon_info.wq);
#else
	que->task = kthread_create(worker_func, (void *)que, que->name);
	if (IS_ERR(que->task)) {
		mutex_unlock(&que->task_lock);
		dev_info(que->dev, "%s: kthread_run failed\n", __func__);
		return PTR_ERR(que->task);
	}
	sched_set_normal(que->task, -20);
	get_task_struct(que->task);
	atomic_set(&que->disable, 0);
	wake_up_process(que->task);
#endif
	mutex_unlock(&que->task_lock);

	return 0;
}

#define TIMEOUT (300)
int imgsys_queue_disable(struct imgsys_queue *que)
{
	int ret;

	if ((!que) || IS_ERR_OR_NULL(que->task))
		return -1;

	ret = wait_event_interruptible_timeout(que->dis_wq, !atomic_read(&que->nr),
						msecs_to_jiffies(TIMEOUT));
	if (!ret)
		dev_info(que->dev, "%s: timeout", __func__);
	else if (ret == -ERESTARTSYS)
		dev_info(que->dev, "%s: signal interrupt", __func__);

	mutex_lock(&que->task_lock);
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	atomic_set(&que->disable, 1);
	atomic_set(&s_daemon_info.is_thread_run, 0);
	wake_up(&s_daemon_info.wq);
	que->task = NULL;
#else
	if (que->task != NULL) {
	ret = kthread_stop(que->task);
	if (ret)
		dev_info(que->dev, "%s: kthread_stop failed %d\n",
						__func__, ret);

	put_task_struct(que->task);
	que->task = NULL;
	}
#endif

	dev_info(que->dev, "%s: kthread(%s) queue peak(%d)\n",
		__func__, que->name, que->peak);

	mutex_unlock(&que->task_lock);

	mutex_destroy(&que->task_lock);

	return ret;
}

int imgsys_queue_add(struct imgsys_queue *que, struct imgsys_work *work)
{
	int size;

	if ((!que) || (!work) || (!que->task))
		return -1;

	if (!que->task) {
		dev_info(que->dev, "%s %s not enabled\n", __func__, que->name);
		return -1;
	}

	if (!work->run)
		dev_info(que->dev, "%s no work func added\n", __func__);

	spin_lock(&que->lock);
	list_add_tail(&work->entry, &que->queue);
	size = atomic_inc_return(&que->nr);
	if (size > que->peak)
		que->peak = size;
	spin_unlock(&que->lock);

    if (imgsys_dbg_enable())
	dev_dbg(que->dev, "%s try wakeup dis/nr(%d/%d)\n", __func__,
		atomic_read(&que->disable), atomic_read(&que->nr));
#ifdef OPLUS_FEATURE_CAMERA_COMMON
	wake_up(&s_daemon_info.wq);
#else
	wake_up(&que->wq);
#endif

    if (imgsys_dbg_enable())
	dev_dbg(que->dev, "%s: raising %s\n", __func__, que->name);

	return 0;
}

int imgsys_queue_timeout(struct imgsys_queue *que)
{
	struct imgsys_work *work, *tmp;

	spin_lock(&que->lock);

	dev_info(que->dev, "%s: stalled work+\n", __func__);
	list_for_each_entry_safe(work, tmp,
		&que->queue, entry){
		dev_info(que->dev, "%s: work %p\n", __func__, work);
	}
	dev_info(que->dev, "%s: stalled work-\n", __func__);

	spin_unlock(&que->lock);

	return 0;
}
