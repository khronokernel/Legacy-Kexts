/*
 --- VoodooPState ---
 (C) 2008-09 Superhai
 
 */

#include "Support.h"
#include "VoodooPState.h"
#include "probe.h"

#include <sys/sysctl.h>


#pragma mark VoodooPState
#pragma mark -
#pragma mark Initialization

// Global
PStateClass	GlobalCurrent;
PStateClass	GlobalRequest;
UInt8		GlobalThrottle;

OSDefineMetaClassAndStructors(VoodooPState, IOService)

static UInt32 getCpuCount()
{
	size_t len;
	integer_t count = 0;
	len = sizeof(count);
	sysctlbyname("hw.physicalcpu", &count, &len, NULL, 0);
	return count;
}

// Class probe
IOService * 
VoodooPState::probe(IOService * provider,
				   SInt32 * score)
{
	Ready = false;

	// Probe the superclass
	if (IOService::probe(provider, score) != this) return NULL;
	
	// Read our own values from the property list
	OSDictionary * dictionary = OSDynamicCast(OSDictionary, getProperty(keyPowerControl));
	if (!dictionary) return NULL;
	UseEfiFsb			= getPlistValue(dictionary, keyUseEfiFsb);
	VoltageOverride		= getPlistValue(dictionary, keyVoltageOverride);
	VoltageProbe		= getPlistValue(dictionary, keyVoltageProbe);
	UserVoltageMax		= getPlistValue(dictionary, keyUserVoltageMax);
	UserVoltageMin		= getPlistValue(dictionary, keyUserVoltageMin);
	ColdStart			= getPlistValue(dictionary, keyColdStart);
	TimerInterval		= getPlistValue(dictionary, keyTimerInterval);
	UseACPI				= getPlistValue(dictionary, keyUseACPI);
	if(TimerInterval < 50){
		TimerInterval = 50;
	}
	
	// Get CPU's from I/O Kit
	CpuCount = getCpuCount();
	
	// No CPU's found -> bailout
	if (CpuCount == 0) return NULL;
	
	// Get FSB from /efi/platform
	CpuFSB = gPEClockFrequencyInfo.bus_frequency_max_hz >> 2;
	if (UseEfiFsb) {
		IORegistryEntry * entry = fromPath(keyEfiPlatform, gIODTPlane);
		if (entry) {
			OSObject * object = entry->getProperty(keyEfiFsbFrequency);
			if (object && (OSTypeIDInst(object) == OSTypeID(OSData))) {
				OSData * data = OSDynamicCast(OSData, object);
				if (data) {
					CpuFSB = * (UInt32 *) data->getBytesNoCopy();
					gPEClockFrequencyInfo.bus_frequency_max_hz = CpuFSB << 2;
				}
			}
		}		
	}
	CpuFSB = (CpuFSB+Mega/2) / Mega;	// Mega is enough

#if	SUPPORT_VOODOO_KERNEL
	{
		UInt64 magic;
		nanoseconds_to_absolutetime(~(0), &magic);
		VoodooKernel = (magic == 2);
	}
#endif
	// Enumerate CPU's
	CpuCoreTech = Unknown;
	{
		uint32_t data[4];

		do_cpuid(0, data);
		((uint32_t*)vendor)[0] = data[1];
		((uint32_t*)vendor)[1] = data[3];
		((uint32_t*)vendor)[2] = data[2];
		vendor[15] = 0;

		do_cpuid(1, data);
		CpuSignature = data[0];

		// Features
		((uint32_t*)&Features)[0] = data[3];
		((uint32_t*)&Features)[1] = data[2];

		for( int i = 0; i < 3; i++ ){
			do_cpuid(0x80000002+i, data);
			memcpy( &brand_string[i*16], data, 16 );
		}
		brand_string[16*3] = 0;
	}

	// Find core technology and cross core vendor specifics
	// Intel
	if (!strncmp(vendor, CPUID_VID_INTEL, sizeof(CPUID_VID_INTEL))) {
		if(!intel_probe(this)) return NULL;
	}
	// AMD
	else if (!strncmp(vendor, CPUID_VID_AMD, sizeof(CPUID_VID_AMD))) {
		if(!amd_probe(this)) return NULL;
	}
	// Unknown CPU or core technology
	else {
		ErrorLog("CPU: Core Technology Unknown - Signature %x (%s)(%s)",
				 (unsigned int)CpuSignature,
				 vendor,
				 brand_string);
		return NULL;
	}
	
	return this;
}

// Class start
bool
VoodooPState::start(IOService * provider)
{
	if (!IOService::start(provider)) return false;

	// Printout banner
	InfoLog("%s %s (%s) %s %s [%s]",
			KextProductName,
			KextVersion,
			KextConfig,
			KextBuildDate,
			KextBuildTime,
			KextOSX);
	InfoLog("based on VoodooPower 1.2.3.");
	
	
	
	// Setup workloop and timer
	IOWorkLoop* WorkLoop = getWorkLoop();
	if (!WorkLoop) return false;
	TimerEventSource = 
	IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action,
																	this,
																	&VoodooPState::LoopTimerEvent));
	if (!TimerEventSource) return false;
	
	if (kIOReturnSuccess != WorkLoop->addEventSource(TimerEventSource)) {
		return false;
	}
	
	// Get a SimpleLock
	SimpleLock = IOSimpleLockAlloc();
	if (!SimpleLock) return false;
	
	// Publish the static characteristics
	OSDictionary * dictionary = OSDictionary::withCapacity(13);
	if (!dictionary) return false;
	OSArray * array = OSArray::withCapacity(PStateCount);
	if (!array) return false;
	setDictionaryNumber(keyCpuCoreTech, CpuCoreTech, dictionary);
	setDictionaryNumber(keyFrontSideBus, CpuFSB, dictionary);
	for (int i = 0; i < PStateCount; i++) {
		OSDictionary * pdictionary = OSDictionary::withCapacity(3);
		if (!pdictionary) return false;
		setDictionaryNumber(keyCurrentFrequency, PState[i].frequency, pdictionary);
		setDictionaryNumber(keyCurrentVoltage, PState[i].voltage, pdictionary);
		setDictionaryNumber(keyFid, PState[i].fid, pdictionary);
		setDictionaryNumber(keyDid, PState[i].did, pdictionary);
		setDictionaryNumber(keyVid, PState[i].vid, pdictionary);
		array->setObject(i, pdictionary);
		pdictionary->release();
	}
	setDictionaryArray(keyPStates, array, dictionary);
	setDictionaryString(keyProductName, KextProductName, dictionary);
	setDictionaryString(keyProductVersion, KextVersion, dictionary);
	setDictionaryString(keyBuildConfig, KextConfig, dictionary);
	setDictionaryString(keyBuildDate, KextBuildDate, dictionary);
	setDictionaryString(keyBuildTime, KextBuildTime, dictionary);
	setDictionaryNumber(keyTimerInterval, TimerInterval, dictionary);
	setProperty(keyCharacteristics, dictionary);

	array->release();
	dictionary->release();

	// set initial pstate
	Request = ColdStart ? (PStateCount-1) : 0;	// hot/cold start

	gPEClockFrequencyInfo.cpu_frequency_max_hz = VoodooFrequencyProc(this,&PState[0]) * Mega;
	gPEClockFrequencyInfo.cpu_frequency_min_hz = VoodooFrequencyProc(this,&PState[PStateCount-1]) * Mega;

	LoopTimerEvent();
	
	// Finalize and kick off the loop
	this->registerService(0);
	Ready = true;

	return true;
}

// Class stop
void
VoodooPState::stop(IOService * provider)
{
	Request = ColdStart ? (PStateCount-1) : 0;	// hot/cold start
	ProcessorDriver();

	Ready = false;
	
	if (TimerEventSource) {
		TimerEventSource->cancelTimeout();
	}

	if (SimpleLock) {
		IOSimpleLockFree(SimpleLock);
	}
	
	IOService::stop(provider);
}

// Class free
void
VoodooPState::free(void)
{
	WarningLog("Unloading");
	IOService::free();
}

#pragma mark -
#pragma mark Main Methods

IOReturn
VoodooPState::setProperties(OSObject * properties)
{
	if (!Ready) {
		return kIOReturnNotReady;
	}
	OSDictionary * dictionary = OSDynamicCast(OSDictionary, properties);

	// to fix p-state
	OSObject * object = dictionary->getObject(keyCurrentPState);
	if (object) {
		// reset
		CpuCount = getCpuCount();

		OSNumber * number = OSDynamicCast(OSNumber, object);
		UInt32 pstate = number->unsigned32BitValue();
		if(pstate >= PStateCount)
			pstate = PStateCount-1;
		Request = pstate;

		// InfoLog("Requested : %d",Request);
		return kIOReturnSuccess;
	}
	return kIOReturnUnsupported;
}

IOReturn
VoodooPState::LoopTimerEvent(void)
{
	// Schedule loop of timer
	TimerEventSource->setTimeoutMS(TimerInterval);
	if (ReEntry) return kIOReturnTimeout;
	ReEntry = true;

	ProcessorDriver();
	PublishStatus();
	
	ReEntry = false;
	return kIOReturnSuccess;
}

void
VoodooPState::PublishStatus(void)
{
	// Publish the current status
	OSDictionary * dictionary = OSDictionary::withCapacity(0);
	if (dictionary) {
		setDictionaryNumber(keyCurrentPState, Current, dictionary);
		setDictionaryNumber(keyCurrentThrottle, Throttle, dictionary);
		setDictionaryNumber(keyCurrentFrequency, Frequency, dictionary);
		setDictionaryNumber(keyCurrentVoltage, Voltage, dictionary);
		setDictionaryNumber(keyTemperature, Thermal, dictionary);
		setProperty(keyStatus, dictionary);
		dictionary->release();
	}
}

typedef void (*void_func)(void *);	// to call mp_rendezvous_no_intrs

//  Processor Driver
void
VoodooPState::ProcessorDriver(void)
{

	if(Current != Request){
		// Prepare values
		if (PStateControl) GlobalRequest = PState[Request];
		if (TStateControl) GlobalThrottle = Throttle >> 2;

		// Read state values
		// only 1 core
		VoodooReadProc(this);
		
#if	SUPPORT_VOODOO_KERNEL
		UInt32 NewFrequency, OldFrequency;
		bool doStepping = (!ConstantTSC && VoodooKernel);
		if( doStepping ){
			NewFrequency = VoodooFrequencyProc(this,&GlobalRequest);
			OldFrequency = VoodooFrequencyProc(this, &GlobalCurrent);
			rtc_clock_stepping(NewFrequency, OldFrequency);
		}
#endif
		// Write state values
		// all the cores.
		IOSimpleLockLock(SimpleLock);
		mp_rendezvous_no_intrs(VoodooWriteProc, this);
		IOSimpleLockUnlock(SimpleLock);
#if	SUPPORT_VOODOO_KERNEL
		if( doStepping ){
			rtc_clock_stepped(NewFrequency, OldFrequency);
		}
#endif
	}

	// Read state values to GlobalCurrent
	// only 1 core
	VoodooReadProc(this);
	
	// Update final values
	Current = Request;
#if	1
	Frequency = PState[Current].frequency;
	Voltage = PState[Current].voltage;
#else
	// convert fit/vid/did to frequency/voltage
	VoodooFrequencyProc(this, &GlobalCurrent);
	Voltage = VoodooVoltageProc(this, &GlobalCurrent);
#endif
	// Update kernel frequency
	// gPEClockFrequencyInfo.bus_to_cpu_rate_num = GlobalCurrent[0].fid & 0x3F;	
	gPEClockFrequencyInfo.cpu_clock_rate_hz = Frequency * Mega;
	gPEClockFrequencyInfo.cpu_frequency_hz = gPEClockFrequencyInfo.cpu_clock_rate_hz;
}


#pragma mark -
#pragma mark Helper Methods

// GetPlistValue
UInt32 
VoodooPState::getPlistValue(OSDictionary * dictionary,
						   const char * symbol)
{
	OSObject * object = 0;
	OSBoolean * boolean = false;
	OSNumber * number = 0;
	OSString * string = 0;
	object = dictionary->getObject(symbol);
	if (object && (OSTypeIDInst(object) == OSTypeID(OSBoolean)))
	{
		boolean = OSDynamicCast(OSBoolean, object);
		return boolean->getValue();
	}
	if (object && (OSTypeIDInst(object) == OSTypeID(OSNumber)))
	{
		number = OSDynamicCast(OSNumber, object);
		return number->unsigned32BitValue();
	}
	if (object && (OSTypeIDInst(object) == OSTypeID(OSString)))
	{
		string = OSDynamicCast(OSString, object);
		// Implement string to number conversion
	}
	return 0;
}

// GetPlistValue
UInt32 
VoodooPState::getPlistValue(OSDictionary * dictionary,
						   const char * symbol,
						   const char * subdictionary)
{
	OSObject * object = dictionary->getObject(subdictionary);
	if (object && (OSTypeIDInst(object) == OSTypeID(OSDictionary)))
	{
		OSDictionary * newdictionary = OSDynamicCast(OSDictionary, object);
		return getPlistValue(newdictionary, symbol);
	}
	return 0;
}

// Adding values into the ioreg dictionary
bool
VoodooPState::setDictionaryNumber(const char * symbol,
								 UInt32 value,
								 OSDictionary * dictionary)
{
	OSNumber * number = OSNumber::withNumber(value, 32);
	if (number)
	{
		dictionary->setObject(symbol, number);
		return true;
	}
	return false;
}

bool
VoodooPState::setDictionaryString(const char * symbol,
								 char * value,
								 OSDictionary * dictionary)
{
	OSString * string = OSString::withCString(value);
	if (string)
	{
		dictionary->setObject(symbol, string);
		return true;
	}
	return false;
}

bool
VoodooPState::setDictionaryString(const char * symbol,
								 const char * value,
								 OSDictionary * dictionary)
{
	OSString * string = OSString::withCString(value);
	if (string)
	{
		dictionary->setObject(symbol, string);
		return true;
	}
	return false;
}

bool
VoodooPState::setDictionaryString(const char * symbol,
								 const OSSymbol * value,
								 OSDictionary * dictionary)
{
	dictionary->setObject(symbol, value);
	return true;
}

bool
VoodooPState::setDictionaryBoolean(const char * symbol,
								  bool value,
								  OSDictionary * dictionary)
{
	OSBoolean * boolean = value ? kOSBooleanTrue : kOSBooleanFalse;
	if (boolean)
	{
		dictionary->setObject(symbol, boolean);
		return true;
	}
	return false;
}

bool
VoodooPState::setDictionaryArray(const char * symbol,
								OSArray * value,
								OSDictionary * dictionary)
{
	dictionary->setObject(symbol, value);
	return true;
}

bool
VoodooPState::setDictionaryDictionary(const char * symbol,
									 OSDictionary * value,
									 OSDictionary * dictionary)
{
	dictionary->setObject(symbol, value);
	return true;
}

