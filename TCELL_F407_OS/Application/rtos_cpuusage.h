#ifndef _RTOS_CPUUSAGE_H_
#define _RTOS_CPUUSAGE_H_

#include <rtthread.h>
#include <rthw.h>

extern void cpu_usage_init(void);
extern void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);

volatile extern rt_uint8_t cpu_usage_major;
volatile extern rt_uint8_t cpu_usage_minor;

#endif
