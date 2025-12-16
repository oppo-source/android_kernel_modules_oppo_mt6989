// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2017-2020, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2019-2020.
 */

#include <linux/kthread.h>
//#include <linux/debugfs..h>
#include <linux/seq_file.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <linux/cpufreq.h>
#include "pw_iris_log.h"
#include "dsi_iris_boost.h"

static LIST_HEAD(disp_policy_list);
struct disp_policy {
		struct freq_qos_request qos_req;
		struct cpufreq_policy *policy;
		struct list_head list;
};

int iris_boost_init(void)
{
	struct disp_policy *req_policy;
	struct cpufreq_policy *policy;
	int cpu, ret;

	if (list_empty(&disp_policy_list)) {
		for_each_possible_cpu(cpu) {
			policy = cpufreq_cpu_get(cpu);
			if (!policy)
				continue;

			IRIS_LOGI("%s, policy: first:%d, min:%d, max:%d",
				__func__, policy->cpu, policy->min, policy->max);
			req_policy = kzalloc(sizeof(*req_policy), GFP_KERNEL);
			if (!req_policy) {
				IRIS_LOGE("%s: fail to kzalloc disp_policy", __func__);
				ret = -ENOMEM;
				goto err;
			}

			req_policy->policy = policy;
			ret = freq_qos_add_request(&policy->constraints, &req_policy->qos_req, FREQ_QOS_MIN, 0);
			if (ret < 0) {
				IRIS_LOGE("%s: fail to add freq constraint (%d)", __func__, ret);
				goto err;

			}
			list_add_tail(&req_policy->list, &disp_policy_list);
			cpu = cpumask_last(policy->related_cpus);
			cpufreq_cpu_put(policy);
		}
	}
		return 0;

err:
	iris_boost_deinit();
	return ret;
}

int iris_boost_deinit(void)
{
	struct disp_policy *req_policy, *tmp;

	list_for_each_entry_safe(req_policy, tmp, &disp_policy_list, list) {
		freq_qos_remove_request(&req_policy->qos_req);
		list_del(&req_policy->list);
		kfree(req_policy);
	}

	return 0;
}

int iris_boost_enable(void)
{
	struct disp_policy *req_policy;
	int ret;
	int cpu = task_cpu(current);

	if (list_empty(&disp_policy_list)) {
		ret = iris_boost_init();
		if (ret < 0)
			return ret;
	}

	/* Boost current cpu's frequency to maximum */
	list_for_each_entry(req_policy, &disp_policy_list, list) {
		if (cpumask_test_cpu(cpu, req_policy->policy->related_cpus)) {
			freq_qos_update_request(&req_policy->qos_req, req_policy->policy->max);
			IRIS_LOGD("%s: update cput-%d min frequency to %d",
				__func__, req_policy->policy->cpu, req_policy->policy->max);
		}
	}
	return ret;
}

int iris_boost_disable(void)
{
	struct disp_policy *req_policy;

	list_for_each_entry(req_policy, &disp_policy_list, list) {
		freq_qos_update_request(&req_policy->qos_req, 0);
		IRIS_LOGD("%s: update cput-%d min frequency to %d", __func__, req_policy->policy->cpu, 0);
	}
	return 0;
}
