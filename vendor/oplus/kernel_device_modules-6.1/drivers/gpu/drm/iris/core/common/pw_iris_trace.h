/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2023, Pixelworks, Inc.
 *
 * These files contain modifications made by Pixelworks, Inc., in 2023.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM iris

#if !defined(_PW_IRIS_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _PW_IRIS_TRACE_H

#include <linux/types.h>
#include <linux/tracepoint.h>

TRACE_EVENT(iris_tracing_mark_write,
	TP_PROTO(char trace_type, const struct task_struct *task,
		const char *name, int value),
	TP_ARGS(trace_type, task, name, value),
	TP_STRUCT__entry(
			__field(char, trace_type)
			__field(int, pid)
			__string(trace_name, name)
			__field(int, value)
	),
	TP_fast_assign(
			__entry->trace_type = trace_type;
			__entry->pid = task ? task->tgid : 0;
			__assign_str(trace_name, name);
			__entry->value = value;
	),
	TP_printk("%c|%d|%s|%d", __entry->trace_type,
			__entry->pid, __get_str(trace_name), __entry->value)
);

#define PW_IRIS_ATRACE_END(name) trace_iris_tracing_mark_write('E', current, name, 0)
#define PW_IRIS_ATRACE_BEGIN(name) trace_iris_tracing_mark_write('B', current, name, 0)
#define PW_IRIS_ATRACE_FUNC() IRIS_ATRACE_BEGIN(__func__)

#define PW_IRIS_ATRACE_INT(name, value) trace_iris_tracing_mark_write('C', current, name, value)

#endif /* _PW_IRIS_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE pw_iris_trace
#include <trace/define_trace.h>
