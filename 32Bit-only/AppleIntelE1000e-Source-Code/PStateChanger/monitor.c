/*
 *  monitor.cpp
 *
 */

#include "monitor.h"

#include <mach/mach.h>
#include <mach/processor_info.h>
#include <mach/mach_host.h>

static integer_t prevTicks[2];
static int entry;

void monitor_init()
{
	prevTicks[0] = 0;
	prevTicks[1] = 0;
	entry = 0;
}

float monitor_probe()
{
	if(entry > 0)
		return 0;
	
	entry += 1;
	
	natural_t numProcessors = 0U;
	natural_t numProcessorInfo;
	processor_info_array_t processorInfo;
	float inUse = 0, total = 0;
	
	kern_return_t err = host_processor_info(mach_host_self(), PROCESSOR_CPU_LOAD_INFO, &numProcessors, &processorInfo, &numProcessorInfo);
	if(err == KERN_SUCCESS) {
		integer_t usedTicks = 0, idleTicks = 0;
		for(int i = 0; i < numProcessors; ++i) {
			int k = (CPU_STATE_MAX * i);
			
			usedTicks += processorInfo[k + CPU_STATE_USER] + processorInfo[k + CPU_STATE_SYSTEM] + processorInfo[k + CPU_STATE_NICE];
			idleTicks += processorInfo[k + CPU_STATE_IDLE];
		}
		inUse = usedTicks - prevTicks[0];
		total = inUse + idleTicks - prevTicks[1];
		prevTicks[0] = usedTicks;
		prevTicks[1] = idleTicks;
		vm_deallocate(mach_task_self(), (vm_address_t)processorInfo, sizeof(integer_t) * numProcessorInfo);
	} else {	// only to prevent zero-division
		total = 1;
	}
	entry -= 1;
	return inUse / total;
}