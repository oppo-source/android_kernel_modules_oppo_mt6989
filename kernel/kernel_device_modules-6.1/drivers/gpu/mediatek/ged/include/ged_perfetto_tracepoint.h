/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM ged_perfetto

#if !defined(_TRACE_GED_PERFETTO_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_GED_PERFETTO_H

#include <linux/tracepoint.h>

TRACE_EVENT(perfetto_tracing_mark_write,

	TP_PROTO(const char *name, long long value),

	TP_ARGS(name, value),

	TP_STRUCT__entry(
		__string(name, name)
		__field(long long, value)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->value = value;
	),

	TP_printk("%s=%lld",__get_str(name), __entry->value)
);

TRACE_EVENT(perfetto_log,

	TP_PROTO(unsigned int active, unsigned int tiler, unsigned int frag,
		unsigned int comp, unsigned int iter, unsigned int mcu),

	TP_ARGS(active, tiler, frag, comp, iter, mcu),

	TP_STRUCT__entry(
		__field(unsigned int, active)
		__field(unsigned int, tiler)
		__field(unsigned int, frag)
		__field(unsigned int, comp)
		__field(unsigned int, iter)
		__field(unsigned int, mcu)
	),

	TP_fast_assign(
		__entry->active = active;
		__entry->tiler = tiler;
		__entry->frag = frag;
		__entry->comp = comp;
		__entry->iter = iter;
		__entry->mcu = mcu;
	),

	TP_printk("active=%u, tiler=%u, frag=%u, comp=%u, iter=%u, mcu=%u",
		__entry->active, __entry->tiler, __entry->frag, __entry->comp,
		__entry->iter, __entry->mcu)
);

TRACE_EVENT(oplus_tracing_mark_write,

	TP_PROTO(int pid, const char *name, long long value),

	TP_ARGS(pid, name, value),

	TP_STRUCT__entry(
		__field(int, pid)
		__string(name, name)
		__field(long long, value)
	),

	TP_fast_assign(
		__entry->pid = pid;
		__assign_str(name, name);
		__entry->value = value;
	),

	TP_printk("C|%d|%s|%lld", __entry->pid, __get_str(name), __entry->value)
);


#endif /* _TRACE_GED_PERFETTO_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/gpu/mediatek/ged/include
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE ged_perfetto_tracepoint
#include <trace/define_trace.h>
