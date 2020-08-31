/*
 *  amd.cpp
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

// constants copied from AMD Linux driver (only needed ones)
enum {
	CPUID_XFAM = 0x0ff00000,	/* extended family */
	CPUID_XFAM_K8 = 0,
	CPUID_XMOD = 0x000f0000,	/* extended model */
	CPUID_XMOD_REV_MASK = 0x00080000,
	CPUID_XFAM_10H = 0x00100000,	/* family 0x10 */
	CPUID_XFAM_11H = 0x00200000,	/* family 0x11 */
	CPUID_USE_XFAM_XMOD = 0x00000f00,
	CPUID_FREQ_VOLT_CAPABILITIES = 0x80000007,
	P_STATE_TRANSITION_CAPABLE = 6,
	/* Field definitions within the FID VID Low Control MSR : */
	MSR_C_LO_INIT_FID_VID = 0x00010000,
	MSR_C_LO_VID_SHIFT = 8,
	/* Field definitions within the FID VID Low Status MSR : */
	MSR_S_LO_CHANGE_PENDING = 0x80000000,   /* cleared when completed */
	MSR_S_LO_CURRENT_FID = 0x0000003f,
	/* Field definitions within the FID VID High Status MSR : */
	MSR_S_HI_CURRENT_VID = 0x0000003f,

	MSR_AMD_CLKCTL = 0xc001001b,
	MSR_AMD_FIDVID_CTL = 0xc0010041,
	MSR_AMD_FIDVID_STS = 0xc0010042,
	MSR_AMD_PSTATE_LIMIT = 0xc0010061,
	MSR_AMD_PSTATE_CTL = 0xc0010062,
	MSR_AMD_PSTATE_STS = 0xc0010063,
	MSR_AMD_PSTATE_BASE = 0xc0010064,
	MSR_AMD_COFVID_CTL = 0xc0010070,
	MSR_AMD_COFVID_STS = 0xc0010071,
	
	/* Hardware Pstate _PSS and MSR definitions */
	HW_PSTATE_FID_MASK = 0x0000003f,
	HW_PSTATE_DID_MASK = 0x000001c0,
	HW_PSTATE_DID_SHIFT = 6,
	HW_PSTATE_VID_MASK = 0x0000fe00,
	HW_PSTATE_VID_SHIFT = 9,
	
	LO_FID_TABLE_TOP = 7,	/* fid values marking the boundary    */
	HI_FID_TABLE_BOTTOM = 8,	/* between the low and high tables    */
	
	// ACPI (PPS)
	IRT_SHIFT = 30,
	RVO_SHIFT = 28,
	PLL_L_SHIFT = 20,
	MVS_SHIFT = 18,
	VST_SHIFT = 11,
	VID_SHIFT = 6,
	IRT_MASK = 3,
	RVO_MASK = 3,
	PLL_L_MASK = 0x7f,
	MVS_MASK = 3,
	VST_MASK = 0x7f,
	EXT_VID_MASK = 0x3f,
	EXT_FID_MASK = 0x3f,
	VID_MASK = 0x1f,
	FID_MASK = 0x1f,
	
	// Cool'n'Quiet transition
	STOP_GRANT_5NS = 1, /* min poss memory access latency for voltage change */
	PLL_LOCK_CONVERSION = (1000/5), /* ms to ns, then divide by clock period */
	MAXIMUM_VID_STEPS = 1,  /* Current cpus only allow a single step of 25mV */
	VST_UNITS_20US = 20,   /* Voltage Stabalization Time is in units of 20us */
	
	CONF_ADR_REG = 0x0cf8,
	CONF_DATA_REG = 0x0cfc,
};

typedef struct {
	UInt32 rvo;     /* ramp voltage offset */
	UInt32 irt;     /* isochronous relief time */
	UInt32 vidmvs;  /* usable value calculated from mvs */
	UInt32 vstable; /* voltage stabilization time, units 20 us */
	UInt32 plllock; /* pll lock time, units 1 us */
} K8_ACPI_DATA;
static K8_ACPI_DATA k8_data;
static UInt32 pStateOffset;

static int find_psb_table(PStateClass*);
static int find_acpi_pss_table(PStateClass*);

// callbacks
static void AmdK8Read(void*);
static void AmdK10Read(void*);

static void AmdK8Write(void*);
static void AmdK10Write(void*);

static UInt32 AmdK8FrequencyProc(void*,PStateClass*);
static UInt32 AmdK8VoltageProc(void*,PStateClass*);
static UInt32 AmdK10FrequencyProc(void*,PStateClass*);
static UInt32 AmdK10VoltageProc(void*,PStateClass*);
static UInt32 AmdK11FrequencyProc(void*,PStateClass*);




// Inlines
inline UInt32 AMDGetK8Frequency(UInt8 fid) { return (800 + (fid * 100));}

inline UInt32 AMDGetK10Frequency(UInt8 fid, UInt8 did) { return (100 * (fid + 0x10) >> did);}

inline UInt32 AMDGetK11Frequency(UInt8 fid, UInt8 did) { return (100 * (fid + 0x08) >> did);}


inline UInt8 AMDGetK8VID(UInt32 Voltage) {
	return (1550 - Voltage) / 25;
}

inline UInt8 AMDGetK10VID(UInt32 Voltage) {
	return (3100 - (Voltage << 1)) / 25;
}

inline UInt32 AMDGetK8Voltage(UInt8 vid) {
	return 1550 - (vid * 25);
}

inline UInt32 AMDGetK10Voltage(UInt8 vid) {
	return (3100 - (vid * 25)) >> 1;
}


static UInt64 readIOConfig(UInt32 bus, UInt32 func, UInt32 reg )
{
	// Thermal read (F3xA4[21:31] + ?)/8
	outl(CONF_ADR_REG, 0x80000000 | (bus << 11) | (func << 8) | reg);
	return inl(CONF_DATA_REG);
}

// if ml_phys_read_byte does not fail to link...
#define	USE_PSB	0

bool amd_probe(VoodooPState* vp)
{
	
	// 920   = 100f42, 1f9
	// 4050e = 060fb2, 7f
	// Turion Z8x = 200f31
	if ((vp->CpuSignature & CPUID_XFAM) == CPUID_XFAM_K8) {
		if (((vp->CpuSignature & CPUID_USE_XFAM_XMOD) != CPUID_USE_XFAM_XMOD) ||
			((vp->CpuSignature & CPUID_XMOD) > CPUID_XMOD_REV_MASK)) {
			InfoLog("Processor cpuid %x not supported", (unsigned int)(vp->CpuSignature));
			return false;
		}
		
		uint32_t data[4];
		do_cpuid(CPUID_FREQ_VOLT_CAPABILITIES, data);
		if ((data[3] & P_STATE_TRANSITION_CAPABLE) != P_STATE_TRANSITION_CAPABLE) {
			InfoLog("Power state transitions not supported");
			return false;
		}
		vp->ConstantTSC = (data[3] & 0x100) != 0;
		vp->IndividualCoreControl = false;
		//
		// Use ACPI to extract fid/vid
		vp->PStateCount = find_acpi_pss_table(vp->PState);
#if	USE_PSB
		if(vp->PStateCount == 0)
			vp->PStateCount = find_psb_table(PState);
#endif
		
		if(vp->PStateCount <= 1){
			InfoLog("No power state.");
			return 0;
		}
		
		// check multiplier tweak
		UInt32 lo, hi;
		rdmsr(MSR_AMD_FIDVID_STS, lo, hi);
		lo &= MSR_S_LO_CURRENT_FID;
		hi &= MSR_S_HI_CURRENT_VID;
		if( lo > vp->PState[0].fid ){
			vp->PState[0].fid = lo;
			vp->PState[0].vid = hi;
		}
		// End of ACPI
		switch ((vp->CpuSignature & CPUID_XMOD) >> 16) {
			case 0:
				vp->CpuCoreTech = AMDK8BC;
				break;
			case 0x01:
				vp->CpuCoreTech = AMDK8D;
				break;
			case 0x02:
			case 0x03:
				vp->CpuCoreTech = AMDK8E;
				break;
			case 0x04:
			case 0x05:
			case 0x06:
			case 0x07:
			case 0x0C:
				vp->CpuCoreTech = AMDK8NPT;
				break;
		}
		vp->VoodooReadProc = AmdK8Read;
		vp->VoodooWriteProc = AmdK8Write;
		vp->VoodooFrequencyProc = AmdK8FrequencyProc;
		vp->VoodooVoltageProc = AmdK8VoltageProc;
	} else { /* HW Pstate capable processor */
		vp->ConstantTSC = true;
		vp->IndividualCoreControl = true;

		vp->VoodooReadProc = AmdK10Read;
		vp->VoodooWriteProc = AmdK10Write;
		vp->VoodooVoltageProc = AmdK10VoltageProc;

		if((vp->CpuSignature & CPUID_XFAM) >= CPUID_XFAM_11H){
			vp->CpuCoreTech = AMDK11;
			vp->VoodooFrequencyProc = AmdK11FrequencyProc;
		} else {
			vp->CpuCoreTech = AMDK10;
			vp->VoodooFrequencyProc = AmdK10FrequencyProc;
		}
		pStateOffset = (rdmsr64(MSR_AMD_COFVID_STS) >> 32) & 0x07;
		// Create P-States
		vp->PStateCount = ((rdmsr64(MSR_AMD_PSTATE_LIMIT) >> 4) & 0x7) + 1;
		for (UInt8 i = 0; i < vp->PStateCount; i++) {
			UInt64 val64 = rdmsr64(MSR_AMD_PSTATE_BASE + pStateOffset + i);
			vp->PState[i].fid = (val64) & HW_PSTATE_FID_MASK;
			vp->PState[i].did = (val64 >> HW_PSTATE_DID_SHIFT) & (HW_PSTATE_DID_MASK>>HW_PSTATE_DID_SHIFT);
			vp->PState[i].vid = (val64 >> HW_PSTATE_VID_SHIFT) & (HW_PSTATE_VID_MASK>>HW_PSTATE_VID_SHIFT);
			vp->PState[i].cid = i;
		}
	}
	vp->PStateControl = 1;

	for( int k = 0; k < vp->PStateCount; k++ ){
		vp->PState[k].frequency = vp->VoodooFrequencyProc(vp, &vp->PState[k]);
		vp->PState[k].voltage = vp->VoodooVoltageProc(vp, &vp->PState[k]);
	}
	
	return true;
}

// port from Linux driver
static int query_current_values_with_pending_wait(PStateClass *pstate)
{
	UInt32 lo, hi;
	int i = 0;
	
	do {
		if (i++ > 10000) {
			return 1;
		}
		rdmsr(MSR_AMD_FIDVID_STS, lo, hi);
	} while (lo & MSR_S_LO_CHANGE_PENDING);
	
	pstate->vid = hi & MSR_S_HI_CURRENT_VID;
	pstate->fid = lo & MSR_S_LO_CURRENT_FID;
	
	return 0;
}

static int write_new_fid(PStateClass *pstate, UInt32 fid)
{
	int i = 0;
	
	UInt32 lo = fid | (pstate->vid << MSR_C_LO_VID_SHIFT) | MSR_C_LO_INIT_FID_VID;
	do {
		wrmsr(MSR_AMD_FIDVID_CTL, lo, k8_data.plllock * PLL_LOCK_CONVERSION);
		if (i++ > 100) {
			DebugLog("Hardware error - pending bit very stuck.");
			return 1;
		}
	} while (query_current_values_with_pending_wait(pstate));
	
	IODelay((1 << k8_data.irt) * 10);
	
	if (fid != pstate->fid) {
		DebugLog("fid trans failed, fid %X, curr %X", (unsigned int)fid,  (unsigned int)pstate->fid );
		return 1;
	}
	
	return 0;
}

/* Write a new vid to the hardware */
static int write_new_vid(PStateClass* pstate, UInt32 vid)
{
	int i = 0;
	
	UInt32 lo = pstate->fid | (vid << MSR_C_LO_VID_SHIFT) | MSR_C_LO_INIT_FID_VID;
	
	do {
		wrmsr(MSR_AMD_FIDVID_CTL, lo, STOP_GRANT_5NS);
		if (i++ > 100) {
			DebugLog("internal error - pending bit very stuck.");
			return 1;
		}
	} while (query_current_values_with_pending_wait(pstate));
	
	if (vid != pstate->vid) {
		DebugLog("vid trans failed, vid %X, curr %X", (unsigned int)vid, (unsigned int)pstate->vid);
		return 1;
	}
	
	return 0;
}

static int decrease_vid_code_by_step(PStateClass* state, UInt32 reqvid, UInt32 step)
{
	if ((state->vid - reqvid) > step)
		reqvid = state->vid - step;
	
	if (write_new_vid(state,reqvid))
		return 1;
	
	IODelay(k8_data.vstable * VST_UNITS_20US);
	
	return 0;
}

/* Phase 1 - core voltage transition ... setup voltage */
static int core_voltage_pre_transition(PStateClass* pstate, UInt32 reqvid)
{
	UInt32 rvosteps = k8_data.rvo;
	UInt32 maxvid, lo;
	
	rdmsr(MSR_AMD_FIDVID_STS, lo, maxvid);
	maxvid = VID_MASK & (maxvid >> 16);
	if (reqvid < maxvid) /* lower numbers are higher voltages */
		reqvid = maxvid;
	
	while (pstate->vid > reqvid) {
		if (decrease_vid_code_by_step(pstate, reqvid, k8_data.vidmvs))
			return 1;
	}
	
	while ((rvosteps > 0) && ((k8_data.rvo + pstate->vid) > reqvid)) {
		if ( pstate->vid == maxvid)
			break;
		
		if (decrease_vid_code_by_step(pstate,pstate->vid - 1, 1))
			return 1;
		rvosteps--;
	}
	
	if (query_current_values_with_pending_wait(pstate))
		return 1;
	
	return 0;
}

static UInt32 convert_fid_to_vco_fid(UInt32 fid)
{
	if (fid < HI_FID_TABLE_BOTTOM)
		return 8 + (2 * fid);
	return fid;
}

/* Phase 2 - core frequency transition */
static int core_frequency_transition(PStateClass* pstate, UInt32 reqfid)
{
	
	if (pstate->fid == reqfid) {
		return 0;
	}
	
	UInt32 vcoreqfid = convert_fid_to_vco_fid(reqfid);
	UInt32 vcocurrfid = convert_fid_to_vco_fid(pstate->fid);
	UInt32 vcofiddiff = vcocurrfid > vcoreqfid ? vcocurrfid - vcoreqfid
	: vcoreqfid - vcocurrfid;
	

	// non-constant TSC CPU must adjust here

	while (vcofiddiff > 2) {
		UInt32 fid_interval;
		(pstate->fid & 1) ? (fid_interval = 1) : (fid_interval = 2);
		
		if (reqfid > pstate->fid) {
			if (pstate->fid > LO_FID_TABLE_TOP) {
				if (write_new_fid(pstate, pstate->fid + fid_interval)) {
					return 1;
				}
			} else {
				if (write_new_fid(pstate, 2 + convert_fid_to_vco_fid(pstate->fid))) {
					return 1;
				}
			}
		} else {
			if (write_new_fid(pstate, pstate->fid - fid_interval))
				return 1;
		}
		vcocurrfid = convert_fid_to_vco_fid(pstate->fid);
		vcofiddiff = vcocurrfid > vcoreqfid ? vcocurrfid - vcoreqfid
		: vcoreqfid - vcocurrfid;
	}
	
	if (write_new_fid(pstate, reqfid))
		return 1;

	// non-constant TSC CPU must adjust here

	if (pstate->fid != reqfid) {
		DebugLog("ph2: failed fid transition, curr %X, req %X", (unsigned int)pstate->fid, (unsigned int)reqfid);
		return 1;
	}
	return 0;
}

/* Phase 3 - core voltage transition flow ... jump to the final vid. */
static int core_voltage_post_transition(PStateClass* pstate, UInt32 reqvid)
{
	if (reqvid != pstate->vid) {
		if (write_new_vid(pstate, reqvid))
			return 1;
		
		if (pstate->vid != reqvid) {
			DebugLog("ph3: failed vid transition req %X, cur %X", (unsigned int)reqvid, (unsigned int)pstate->vid);
			return 1;
		}
	}
	
	return query_current_values_with_pending_wait(pstate);
}
// copy end

void AmdK8Write(void*)
{
	if (GlobalCurrent.ctl != GlobalRequest.ctl) {
		PStateClass state;
		UInt32 reqfid, reqvid;
		reqfid = GlobalRequest.fid;
		reqvid = GlobalRequest.vid;

		/* Change Opteron/Athlon64 fid and vid, by the 3 phases. */
		if (query_current_values_with_pending_wait(&state))
			return;
		
		if (core_voltage_pre_transition(&state, reqvid))
			return;
		
		if (core_frequency_transition(&state, reqfid))
			return;
		
		if (core_voltage_post_transition(&state, reqvid))
			return;
		
		if (query_current_values_with_pending_wait(&state))
			return;
		
	}
}

void AmdK8Read(void* magic)
{
	VoodooPState* vp = (VoodooPState*)magic;
	UInt32 lo, hi;
	rdmsr(MSR_AMD_FIDVID_STS, lo, hi);
	GlobalCurrent.fid = lo & MSR_S_LO_CURRENT_FID;
	GlobalCurrent.vid = hi & MSR_S_HI_CURRENT_VID;
	// Thermal read F3xE4[14:23]/4 - 49 : revision F/G only
	UInt32 val = readIOConfig(24,3,0xe4);
	vp->Thermal = ((val >> 16) & 0xff) - 49; // ignore 0.25
}

static UInt32 AmdK8FrequencyProc(void* magic,PStateClass* pstate)
{
	VoodooPState* vp = (VoodooPState*)magic;
	return AMDGetK8Frequency(pstate->fid) * vp->CpuFSB / 200;
}

static UInt32 AmdK8VoltageProc(void*,PStateClass* pstate)
{
	return AMDGetK8Voltage(pstate->vid);
}


static void AmdK10Read(void* magic)
{
	VoodooPState* vp = (VoodooPState*)magic;
	UInt64 msr = rdmsr64(MSR_AMD_COFVID_STS);
	GlobalCurrent.fid = (msr >> 0) & HW_PSTATE_FID_MASK;
	GlobalCurrent.did = (msr >> HW_PSTATE_DID_SHIFT) & (HW_PSTATE_DID_MASK>>HW_PSTATE_DID_SHIFT);
	GlobalCurrent.vid = (msr >> HW_PSTATE_VID_SHIFT) & (HW_PSTATE_VID_MASK>>HW_PSTATE_VID_SHIFT);
	
	// Thermal read (F3xA4[21:31] + ?)/8
	UInt32 curTmp = readIOConfig(24,3,0xa4)  >> 21;
	curTmp /= 8;	// 0.125 c/t
	vp->Thermal = curTmp;
}


static void AmdK10Write(void* magic)
{
	wrmsr64(MSR_AMD_PSTATE_CTL, GlobalRequest.cid);
}

static UInt32 AmdK10FrequencyProc(void* magic,PStateClass* pstate)
{
	return AMDGetK10Frequency(pstate->fid, pstate->did);
}
					  
static UInt32 AmdK10VoltageProc(void*,PStateClass* pstate)
{
	return AMDGetK10Voltage(pstate->vid);
}
					  
				  

static UInt32 AmdK11FrequencyProc(void* magic,PStateClass* pstate)
{
	return AMDGetK11Frequency(pstate->fid, pstate->did);
}
				  


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
		if(k==0){
			UInt32 ctl = ((OSNumber*) onestate->getObject(4))->unsigned32BitValue();
			k8_data.irt = (ctl >> IRT_SHIFT) & IRT_MASK;
			k8_data.rvo = (ctl >> RVO_SHIFT) & RVO_MASK;
			k8_data.plllock = (ctl >> PLL_L_SHIFT) & PLL_L_MASK;
			k8_data.vidmvs = 1 << ((ctl >> MVS_SHIFT) & MVS_MASK);
			k8_data.vstable = (ctl >> VST_SHIFT) & VST_MASK;
		}
		// extract various params....
		UInt32 state = ((OSNumber*) onestate->getObject(5))->unsigned32BitValue();
		PState[k].fid = state & EXT_FID_MASK;
		PState[k].did = 0;
		PState[k].vid = (state >> VID_SHIFT) & EXT_VID_MASK;;
		PState[k].cid = k;
	}
	return numps;
}

#if	USE_PSB
static int find_psb_table(PStateClass* PState)
{
	psb_s psb;
	UInt32 i, j;
	UInt32 numps;
	UInt8* ptr = (UInt8*)&psb;
	for (i = 0xc0000; i < 0xffff0; i += 0x10) {
		/* Scan BIOS looking for the signature. */
		/* It can not be at ffff0 - it is too big. */
		for(j = 0; j < PSB_ID_STRING_LEN; j++ )
			psb.signature[j] = ml_phys_read_byte((vm_offset_t)(i+j));
		if (memcmp(psb.signature, PSB_ID_STRING, PSB_ID_STRING_LEN) != 0)
			continue;
		InfoLog("found PSB header at 0x%x", i);
		for(j=PSB_ID_STRING_LEN; j < sizeof(psb); j++){
			ptr[j] = ml_phys_read_byte((vm_offset_t)(i+j));
		}
		if (psb.tableversion != 0x14) {
			InfoLog("PSB table is not v1.4");
			return 0;
		}
		if (psb.flags1) {
			DebugLog("PSB unknown flags");
			return 0;
		}
		k8_data.vstable = psb.vstable;
		k8_data.rvo = psb.flags2 & 3;
		k8_data.irt = (psb.flags2 >> 2) & 3;
		UInt32 mvs = (psb.flags2 >> 4) & 3;
		k8_data.vidmvs = 1 << mvs;
		k8_data.plllock = psb.plllocktime;
		
		numps = (psb.flags2 >> 6) & 3; // batps
		if(numps == 0)
			numps = psb.numps;
		
		UInt32 cpst = psb.num_tables;
		if ((psb.cpuid == 0x00000fc0) || (psb.cpuid == 0x00000fe0) ){
			UInt32 cpuid = cpuid_signature();
			if ((cpuid == 0x00000fc0) || (cpuid == 0x00000fe0) ) {
				cpst = 1;
			}
		}
		if (cpst != 1) {
			DebugLog("numpst must be 1");
			return 0;
		}
		// ommit sanity check
		for( j = 0; j < numps; j++ ){
			PState[j].fid = ml_phys_read_byte((vm_offset_t)(j*2+i+sizeof(psb)));
			PState[j].did = 0;
			PState[j].vid = ml_phys_read_byte((vm_offset_t)(j*2+i+sizeof(psb)+1));
			PState[j].cid = j;
		}
		return numps; // will be PStateCount
	}
	return 0;
}
#endif

