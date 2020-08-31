/*
 --- VoodooPState ---
 (C) 2008 Superhai
 
 Contact	http://
 www.superhai.com/
 
 */

#include <IOKit/IOService.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IOTimerEventSource.h>
#include <i386/cpuid.h>
#include <i386/proc_reg.h>
#include <kern/host.h>

#ifndef	SUPPORT_VOODOO_KERNEL
#define	SUPPORT_VOODOO_KERNEL	0
#endif
// *** Const values
const UInt8		PStateCountMax	= 0x20;		// Limit number of P-States
const UInt8		ThrottleMax		= 0x20;
const UInt32	IOCpuCountMax	= 0xFF;		// Limit number of CPU's

// *** String defines
extern const char *	keyPowerControl;
extern const char *	keyVoltageOverride;
extern const char *	keyVoltageProbe;
extern const char *	keyUserVoltageMin;
extern const char *	keyUserVoltageMax;
extern const char *	keyUseEfiFsb;
extern const char *	keyPStateLimitHigh;
extern const char *	keyPStateLimitLow;
extern const char *	keyTStateControl;
extern const char *	keyPStates;
extern const char *	keyFid;
extern const char *	keyDid;
extern const char *	keyVid;
extern const char *	keyProductName;
extern const char *	keyProductVersion;
extern const char *	keyBuildConfig;
extern const char *	keyBuildDate;
extern const char *	keyBuildTime;
extern const char *	keyCpuCoreTech;
extern const char *	keyFrontSideBus;
extern const char *	keyCharacteristics;
extern const char *	keySystem;
extern const char *	keyUser;
extern const char *	keyIdle;
extern const char *	keyNice;
extern const char *	keyLoad;
extern const char *	keyCurrentPState;
extern const char *	keyCurrentThrottle;
extern const char *	keyTemperature;
extern const char *	keyCurrentFrequency;
extern const char *	keyCurrentVoltage;
extern const char *	keyCpu;
extern const char *	keyCurrentProfile;
extern const char *	keyStatus;
extern const char *	keyEfiPlatform;
extern const char *	keyEfiFsbFrequency;
extern const char *	keyColdStart;
extern const char *	keyTimerInterval;
extern const char *	keyUseACPI;


// CPU Core Tech
enum  {
	Unknown,
	Intel386,
	Intel486,
	IntelPentium,
	IntelPentiumPro,
	IntelPentiumM,
	IntelNetburstOld,
	IntelNetburstNew,
	IntelCore,
	IntelAtom,
	IntelCore45,
	IntelCoreI7,
	IntelCoreI5,
	AMDK5,
	AMDK6,
	AMDK7,
	AMDK8BC,
	AMDK8D,
	AMDK8E,
	AMDK8NPT,
	AMDK10,
	AMDK11,
	CoreTechCount			
};

// P-State Class
struct PStateClass {
	union {
		UInt16 ctl;
		struct {
			UInt8 vid;	// Voltage ID
			UInt8 fid;	// Frequency ID
		};
	};
	UInt8	did;		// DID
	UInt8	cid;		// Compare ID
	UInt16	frequency;
	UInt16	voltage;
};

	
class VoodooPState : public IOService {
	OSDeclareDefaultStructors(VoodooPState)
public:
	bool					Ready;
	bool					UseEfiFsb;
	bool					VoltageOverride;
	bool					VoltageProbe;
	bool					AutoPowerManager;
	bool					PStateControl;
	bool					TStateControl;
	bool					MobileCpu;
	bool					ConstantTSC;
	bool					NonIntegerBusRatio;
	bool					IndividualCoreControl;
	bool					Intel_NoStopCStateTSC;
	bool					Intel_DynamicFSB;
	bool					Intel_DynamicAcceleration;
	bool					Intel_TjMAX15;
	bool					ReEntry;
	bool					ColdStart;
	bool					UseACPI;
#if	SUPPORT_VOODOO_KERNEL
	bool					VoodooKernel;
#endif
	UInt8					CpuCoreTech;
	UInt8					PStateCount;
	UInt8					LoadCalcShift;
	UInt8					Request;
	UInt8					Current;
	UInt8					Throttle;
	UInt8					TjMAX;
	UInt8					Thermal;
	UInt32					CpuSignature;
	UInt64					Features;
	char vendor[16];
	char brand_string[64];
	UInt32					CpuCount;
	UInt32					TimerInterval;
	UInt32					UserVoltageMax;
	UInt32					UserVoltageMin;
	UInt32					CpuFSB;
	UInt32					Frequency;
	UInt32					Voltage;
	PStateClass				PState[PStateCountMax];
	PStateClass				Initial;
private:
	uint32_t				UniqueFeature;
	IOTimerEventSource *	TimerEventSource;
	IOSimpleLock *			SimpleLock;
	UInt32	getPlistValue(OSDictionary * dictionary, const char * symbol);
	UInt32	getPlistValue(OSDictionary * dictionary, const char * symbol, const char * subdictionary);
	bool	setDictionaryNumber(const char * symbol, UInt32 value, OSDictionary * dictionary);
	bool	setDictionaryString(const char * symbol, char * value, OSDictionary * dictionary);
	bool	setDictionaryString(const char * symbol, const char * value, OSDictionary * dictionary);
	bool	setDictionaryString(const char * symbol, const OSSymbol * value, OSDictionary * dictionary);
	bool	setDictionaryBoolean(const char * symbol, bool value, OSDictionary * dictionary);
	bool	setDictionaryArray(const char * symbol, OSArray * value, OSDictionary * dictionary);
	bool	setDictionaryDictionary(const char * symbol, OSDictionary * value, OSDictionary * dictionary);
	void	ProcessorDriver(void);
	void	PublishStatus();
public:
	void (*VoodooReadProc)(void*);
	void (*VoodooWriteProc)(void*);
	UInt32 (*VoodooFrequencyProc)(void*,PStateClass*);
	UInt32 (*VoodooVoltageProc)(void*,PStateClass*);
public:
	virtual IOReturn	setProperties(OSObject * properties);
	virtual IOReturn	LoopTimerEvent(void);
	virtual IOService * probe(IOService * provider, SInt32 * score);
	virtual bool		start(IOService * provider);
	virtual void		stop(IOService * provider);
	virtual void		free(void);
};

extern PStateClass	GlobalCurrent;
extern PStateClass	GlobalRequest;
extern UInt8		GlobalThrottle;

// Externs from Unsupported
extern "C" void mp_rendezvous_no_intrs(void (*action_func)(void *), void * arg);
extern "C" int cpu_number(void);
#if	SUPPORT_VOODOO_KERNEL
extern "C" void rtc_clock_stepping(uint32_t new_frequency, uint32_t old_frequency);
extern "C" void rtc_clock_stepped (uint32_t new_frequency, uint32_t old_frequency);
#endif
