/*
 *  intel.cpp
 *  VoodooPState
 *
 *  Created by hnak on 09/03/14.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "Support.h"
#include "VoodooPState.h"
#include "probe.h"

#include <IOKit/IORegistryEntry.h>
#include <IOKit/acpi/IOACPIPlatformDevice.h>
#include <architecture/i386/pio.h>>

// *** MSR's

#define MSR_P4_EBC_FREQUENCY_ID		0x002C
#define MSR_M_PSB_CLOCK_STS			0x00CD
#define MSR_IA32_MPERF				0x00E7
#define MSR_IA32_APERF				0x00E8
#define MSR_IA32_EXT_CONFIG			0x00EE
#define MSR_IA32_CLOCK_MODULATION	0x019A
#define MSR_IA32_THERM_INTERRUPT	0x019B
#define MSR_IA32_THERM_STATUS		0x019C
#define MSR_IA32_TEMPERATURE_TARGET	0x01A2

#define FID(ctl)        (((ctl) & 0xff00) >> 8)
#define VID(ctl)        ((ctl) & 0x00ff)
static int find_acpi_pss_table(PStateClass*);

// Inlines
inline void IntelWaitForSts(void) {
	UInt32 inline_timeout = 100000;
	while (rdmsr64(MSR_IA32_PERF_STS) & (1 << 21)) { if (!inline_timeout--) break; }
}

inline UInt8 IntelGetPentiumMVID(UInt32 Voltage) {
	return (Voltage - 700) >> 4;
}

inline UInt8 IntelGetNetburstVID(UInt32 Voltage) {
	UInt32 ret = (Voltage < 1100) ? ((Voltage << 2) - 1198) / 50 : ((Voltage << 2) - 4298) / 50;
	return (ret >> 1) | ((ret & 0x1) * 0x20);
}

inline UInt8 IntelGetCoreOldVID(UInt32 Voltage, bool Mobile) {
	return Mobile ? ((Voltage << 1) - 1424) / 25 : ((Voltage << 1) - 1649) / 25;
}

inline UInt8 IntelGetCoreNewVID(UInt32 Voltage) {
	return ((Voltage << 1) - 1424) / 25;
}

inline UInt32 IntelGetFrequency(UInt8 fid, UInt32 fsb) {
	UInt32 multiplier = fid & 0x1f;					// = 0x08
	bool half = fid & 0x40;							// = 0x01
	bool dfsb = fid & 0x80;							// = 0x00
	UInt32 halffsb = (fsb + 1) >> 1;				// = 200
	UInt32 frequency = (multiplier * fsb);			// = 3200
	return (frequency + (half * halffsb)) >> dfsb;	// = 3200 + 200 = 3400
}

inline UInt32 IntelGetPentiumMVoltage(UInt8 vid) {
	return 700 + ((vid & 0x3F) << 4);
}

inline UInt32 IntelGetNetburstVoltage(UInt8 vid) {
	return (((vid & 0x1F) <= 0x15 ? 2150 + ((vid & 0x1F) * 50) : 600 + ((vid & 0x1F) * 50)) + ((bool)(vid & 0x20)) * 25) >> 1;
}

inline UInt32 IntelGetCoreOldVoltage(UInt8 vid, bool Mobile) {
	return Mobile ? (1425 + ((vid & 0x3F) * 25)) >> 1 : (1650 + ((vid & 0x3F) * 25)) >> 1;
}

inline UInt32 IntelGetCoreNewVoltage(UInt8 vid) {
	return (1425 + ((vid & 0x3F) * 25)) >> 1;
}

inline UInt32 IntelGetNehalemVoltage(UInt8 vid) {
	// 1.60000 mV at VID = 2, step = 0.00625
	return (6450 - ((vid & 0x7F) * 25)) / 4;
}

// callbacks
static void IntelReadProc(void*);
static void IntelWriteProc(void*);
static UInt32 IntelFrequencyProc(void*,PStateClass*);
static UInt32 IntelVoltageProc(void*,PStateClass*);

bool intel_probe(VoodooPState* vp)
{
	// assign callbacks
	vp->VoodooReadProc = IntelReadProc;
	vp->VoodooWriteProc = IntelWriteProc;
	vp->VoodooFrequencyProc = IntelFrequencyProc;
	vp->VoodooVoltageProc = IntelVoltageProc;

	// Enhanced Intel Speedstep support
	if (vp->Features & CPUID_FEATURE_EST) {
		wrmsr64(MSR_IA32_MISC_ENABLE, (rdmsr64(MSR_IA32_MISC_ENABLE) | (1 << 16))); IOSleep(1);
		vp->PStateControl = rdmsr64(MSR_IA32_MISC_ENABLE) & (1 << 16);
	}
	
	// Non-integer Bus Ratio Support
	vp->NonIntegerBusRatio = (rdmsr64(MSR_IA32_PERF_STS) & (1ULL << 46));
	
	// TState Control & Non Stopping TSC in CState 2+
	UInt32 cpuid;
	__asm__ volatile ("cpuid" : "=d" (cpuid) : "a" (0x00000001) : "ebx", "ecx" );
	vp->TStateControl = cpuid & (1<<22); 
	__asm__ volatile ("cpuid" : "=d" (cpuid) : "a" (0x80000007) : "ebx", "ecx" );
	vp->Intel_NoStopCStateTSC = cpuid & (1<<8); 
	
	
	UInt8 core = Unknown;
	// Netburst
	switch (vp->CpuSignature & 0x000FF0) {
		case 0x000F10:
		case 0x000F20:
			core = IntelNetburstOld;
			break;
		case 0x000F30:
		case 0x000F40:
		case 0x000F60:
			core = IntelNetburstNew;
			break;
	}
	// Core/P-M
	switch (vp->CpuSignature & 0x0006F0) {
		case 0x000690:
		case 0x0006D0:
			core = IntelPentiumM;
			break;
		case 0x0006E0:
		case 0x0006F0:
			core = IntelCore;
			break;
	}
	// Core/Core45/i7
	switch (vp->CpuSignature & 0x0106F0) {
		case 0x010660:
			core = IntelCore;
			break;
		case 0x010670:
		case 0x0106D0:
			core = IntelCore45;
			break;
		case 0x0106C0:
			core = IntelAtom;
			break;
		case 0x0106A0:
			core = IntelCoreI7;
			break;
		case 0x0106E0:
			core = IntelCoreI5;
			break;
	}

	if (core == Unknown)
		return false;

	
	vp->CpuCoreTech = core;

	// Set CPU control flags
	switch (core) {
		case IntelPentiumM:
			vp->MobileCpu = true;
			vp->ConstantTSC = false;
			break;
		case IntelNetburstOld:
			vp->ConstantTSC = false;
			vp->MobileCpu = (rdmsr64(MSR_P4_EBC_FREQUENCY_ID) & (1 << 21));
			break;
		case IntelNetburstNew:
			vp->ConstantTSC = true;
			vp->MobileCpu = (rdmsr64(MSR_P4_EBC_FREQUENCY_ID) & (1 << 21));
			break;
		case IntelCoreI7:
		case IntelCoreI5:
			vp->IndividualCoreControl = true;
			vp->ConstantTSC = true;
			vp->MobileCpu = (rdmsr64(MSR_IA32_PLATFORM_ID) & (1 << 28));
			// it's safe not to support.
			return false;
			break;

		case IntelCore:
		case IntelCore45:
		case IntelAtom:
			vp->ConstantTSC = true;
			vp->MobileCpu = (rdmsr64(MSR_IA32_PLATFORM_ID) & (1 << 28));
			if (rdmsr64(MSR_IA32_EXT_CONFIG) & (1 << 27)) {
				wrmsr64(MSR_IA32_EXT_CONFIG, (rdmsr64(MSR_IA32_EXT_CONFIG) | (1 << 28))); IOSleep(1);
				vp->Intel_DynamicFSB = rdmsr64(MSR_IA32_EXT_CONFIG) & (1 << 28);
			}
			vp->Intel_TjMAX15 = (rdmsr64(MSR_IA32_EXT_CONFIG) & (1 << 30));
			break;
	}
	
	
	// Find a TjMAX value
	switch (core) {
		case IntelCore45:
			vp->TjMAX = 100;
			if (vp->MobileCpu) {
				vp->TjMAX += 5;
				if (vp->Intel_TjMAX15) {
					vp->TjMAX -= 15;
				}
			}
			break;
		case IntelAtom:
			vp->TjMAX = 100;
			if (vp->MobileCpu) vp->TjMAX -= 10;
			break;
		case IntelCore:
			vp->TjMAX = 100;
			if ((vp->MobileCpu) && (vp->Intel_TjMAX15)) vp->TjMAX -= 15;
			break;
		case IntelCoreI7:
		case IntelCoreI5:
			vp->TjMAX = ((rdmsr64(MSR_IA32_TEMPERATURE_TARGET) >> 16) & 0xFF);
			break;
		default:
			vp->TjMAX = 0;
			break;
	}
	DebugLog("TjMAX %u TjMAX15 %u Mobile %u", vp->TjMAX, vp->TjMAX, vp->Intel_TjMAX15, vp->MobileCpu);

	// Create P-States
	PStateClass Maximum, Minimum;
	// Store our loading state for unloading
	vp->Initial.ctl = rdmsr64(MSR_IA32_PERF_STS) & 0xFFFF;

	// ACPI_PSS first.
	if(vp->UseACPI || vp->CpuCoreTech >= IntelCoreI7){
		vp->PStateCount = find_acpi_pss_table(vp->PState);
		if(vp->PStateCount > 0){
			return true;
		}
	}


	// Find maximum and minimum states
	Maximum.ctl = ((rdmsr64(MSR_IA32_PERF_STS) >> 32) & 0x1F3F) | (0x4000 * vp->NonIntegerBusRatio);
	Maximum.cid = ((Maximum.fid & 0x1F) << 1) | vp->NonIntegerBusRatio;
	Minimum.fid = ((rdmsr64(MSR_IA32_PERF_STS) >> 24) & 0x1F) | (0x80 * vp->Intel_DynamicFSB);
	Minimum.vid = ((rdmsr64(MSR_IA32_PERF_STS) >> 48) & 0x3F);
	if (Minimum.fid == 0) {
		if (vp->VoltageProbe) {	// Probe for lowest fid
			for (UInt8 i = Maximum.fid; i >= 0x6; i--) {
				wrmsr64(MSR_IA32_PERF_CTL,
						(rdmsr64(MSR_IA32_PERF_CTL) & 0xFFFFFFFFFFFF0000ULL) | (i << 8) | Minimum.vid);
				IntelWaitForSts();
				Minimum.fid = (rdmsr64(MSR_IA32_PERF_STS) >> 8) & 0x1F; IOSleep(1);
			}
		} else {
			Minimum.fid = Maximum.fid;
		}
	}
	if (vp->VoltageProbe && (Minimum.vid == Maximum.vid)) {	// Probe for lowest vid
		for (UInt8 i = Maximum.vid; i > 0xA; i--) {
			wrmsr64(MSR_IA32_PERF_CTL,
					(rdmsr64(MSR_IA32_PERF_CTL) & 0xFFFFFFFFFFFF0000ULL) | (Minimum.fid << 8) | i);
			IntelWaitForSts();
			Minimum.vid = rdmsr64(MSR_IA32_PERF_STS) & 0x3F; IOSleep(1);
		}
	}
	Minimum.cid = ((Minimum.fid & 0x1F) << 1) >> vp->Intel_DynamicFSB;
	// Sanity check
	if (Maximum.cid < Minimum.cid) {
		ErrorLog("Insane FID values");
		return 0;
	}
	// Voltage override
	if (vp->VoltageOverride) {
		switch (core) {
			case IntelPentiumM:
				Maximum.vid = IntelGetPentiumMVID(vp->UserVoltageMax);
				Minimum.vid = IntelGetPentiumMVID(vp->UserVoltageMin);
				break;
			case IntelNetburstOld:
			case IntelNetburstNew:
				Maximum.vid = IntelGetNetburstVID(vp->UserVoltageMax);
				Minimum.vid = IntelGetNetburstVID(vp->UserVoltageMin);
				break;
			case IntelCore:
				Maximum.vid = IntelGetCoreOldVID(vp->UserVoltageMax, vp->MobileCpu);
				Minimum.vid = IntelGetCoreOldVID(vp->UserVoltageMin, vp->MobileCpu);
				break;
			case IntelCore45:
			case IntelAtom:
			case IntelCoreI7:
			case IntelCoreI5:
				Maximum.vid = IntelGetCoreNewVID(vp->UserVoltageMax);
				Minimum.vid = IntelGetCoreNewVID(vp->UserVoltageMin);						
				break;
		}
	}
	// Finalize P-States
	// Find how many P-States machine supports
	if (vp->PStateControl) {
		vp->PStateCount = Maximum.cid - Minimum.cid + 1;
		if (vp->PStateCount > PStateCountMax) vp->PStateCount = PStateCountMax;
		UInt8 vidstep;
		UInt8 i = 0, invalid = 0;
		vidstep = ((Maximum.vid << 2) - (Minimum.vid << 2)) / (vp->PStateCount - 1);
		for (UInt8 u = 0; u < vp->PStateCount; u++) {
			i = u - invalid;
			vp->PState[i].cid = Maximum.cid - u;
			vp->PState[i].fid = (vp->PState[i].cid >> 1);
			if (vp->PState[i].fid < 0x6) {
				if (vp->Intel_DynamicFSB) vp->PState[i].fid = (vp->PState[i].fid << 1) | 0x80;
			} else {
				if (vp->NonIntegerBusRatio) vp->PState[i].fid = vp->PState[i].fid | (0x40 * (vp->PState[i].cid & 0x1));
			}
			if (i) {
				if (vp->PState[i].fid == vp->PState[i-1].fid) {
					invalid++;
				}
			}
			vp->PState[i].vid = ((Maximum.vid << 2) - (vidstep * u)) >> 2;
		}
		vp->PStateCount -= invalid;
		if (!vp->PStateCount) {
			ErrorLog("Insane P-State values");
			return false;
		}
	} else {
		return false;
	}
	for( int k = 0; k < vp->PStateCount; k++ ){
		vp->PState[k].frequency = IntelFrequencyProc(vp, &vp->PState[k]);
		vp->PState[k].voltage = IntelVoltageProc(vp, &vp->PState[k]);
	}
	return true;
}


static void IntelReadProc(void* magic)
{
	VoodooPState* vp = (VoodooPState*)magic;
	// Thermal read
	switch (vp->CpuCoreTech) {
		case IntelCore:
		case IntelCore45:
		case IntelAtom:
		case IntelCoreI7:
		case IntelCoreI5:
			UInt64 msr = rdmsr64(MSR_IA32_THERM_STATUS);
			if (msr & 0x80000000){
				int t = (msr >> 16) & 0x7F;
				vp->Thermal = vp->TjMAX - t;
			}
	}			
	GlobalCurrent.ctl = rdmsr64(MSR_IA32_PERF_STS) & 0xFFFF;
}

static void IntelWriteProc(void* magic)
{	
	UInt64 msr = rdmsr64(MSR_IA32_PERF_CTL) & 0xFFFFFFFFFFFF0000ULL;
	if ((GlobalCurrent.ctl != GlobalRequest.ctl) && (GlobalRequest.ctl)) {
		wrmsr64(MSR_IA32_PERF_CTL, msr | GlobalRequest.ctl);
		IntelWaitForSts();
	}
	bool throttle = rdmsr64(MSR_IA32_CLOCK_MODULATION) & 0x10;
	if (GlobalThrottle) {
		UInt8 local = (~GlobalThrottle) & 0x7;
		if (local == 0) local = 1;
		wrmsr64(MSR_IA32_CLOCK_MODULATION, 0x10 | (local << 1));
	} else {
		if (throttle) {
			wrmsr64(MSR_IA32_CLOCK_MODULATION, 0x8);
		}
	}
}


static UInt32 IntelFrequencyProc(void* magic,PStateClass* pstate)
{
	VoodooPState* vp = (VoodooPState*)magic;
	return IntelGetFrequency(pstate->fid, vp->CpuFSB);

}

static UInt32 IntelVoltageProc(void* magic,PStateClass* pstate)
{
	VoodooPState* vp = (VoodooPState*)magic;
	UInt32 voltage = 0;
	switch (vp->CpuCoreTech) {
		case IntelPentiumM:
			voltage = IntelGetPentiumMVoltage(pstate->vid);
			break;
		case IntelNetburstOld:
		case IntelNetburstNew:
			voltage = IntelGetNetburstVoltage(pstate->vid);
			break;
		case IntelCore:
			voltage = IntelGetCoreOldVoltage(pstate->vid, vp->MobileCpu);
			break;
		case IntelCore45:
		case IntelAtom:
			voltage = IntelGetCoreNewVoltage(pstate->vid);
			break;
		case IntelCoreI7:
		case IntelCoreI5:
			voltage = IntelGetNehalemVoltage(pstate->vid);
			break;
	}
	return voltage;
}

// Just for reference
static int find_acpi_pss_table(PStateClass* PState)
{
	// Use ACPI to extract fid/vid
	// borrowed from xnu-speedstep
	/* Find CPUs in the IODeviceTree plane */
	IORegistryEntry* ioreg = IORegistryEntry::fromPath("/cpus", IORegistryEntry::getPlane("IODeviceTree"));
	if (ioreg == 0) {
		DebugLog("No CPU!");
		return 0;
	}
	
	/* Get the first CPU - we assume all CPUs share the same P-State */
	IOACPIPlatformDevice* cpu = (IOACPIPlatformDevice*) ioreg->getChildEntry(IORegistryEntry::getPlane("IODeviceTree"));
	if (cpu == 0) {
		DebugLog("CPU = 0!");
		return 0;
	}
	
	/* Now try to find the performance state table */
	OSObject* PSS;
	cpu->evaluateObject("_PSS", &PSS);
	if(PSS == 0) {
		InfoLog("No PState table in ACPI.");
		return 0;
	}
	
	OSArray* PSSArray = (OSArray*) PSS;
	UInt32 numps = PSSArray->getCount();
	for(int k=0; k < numps; k++){
		OSArray* onestate = ( OSArray* )(PSSArray->getObject(k));
		PState[k].frequency = ((OSNumber*) onestate->getObject(0))->unsigned32BitValue();
		PState[k].voltage = 0; // Not yet figured out
		PState[k].ctl = ((OSNumber*) onestate->getObject(4))->unsigned32BitValue();
	}
	return numps;
}

