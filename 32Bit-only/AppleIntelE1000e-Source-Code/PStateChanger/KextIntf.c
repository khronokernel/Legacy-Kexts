/*
 *  KextIntf.c
 *  PStateChanger
 *
 *
 */

#include "KextIntf.h"
#include <IOKit/IOKitLib.h>
#include <CoreFoundation/CoreFoundation.h>

#define	SERVICE_NAME	"VoodooPState"

static unsigned int GetNumber (CFStringRef key, CFDictionaryRef dictionary) {
	unsigned int ret;
	if (CFNumberGetValue((CFNumberRef) CFDictionaryGetValue(dictionary, key), kCFNumberIntType, &ret)) {
		return ret;
	}
	return 0;
}

int getPstateCurrent(PSTATE_STATUS_INFO* pState){
	io_service_t	IOService  = IOServiceGetMatchingService(0, IOServiceMatching(SERVICE_NAME));
	if (! IOService )
		return 0;
	CFDictionaryRef SDictionary = (CFDictionaryRef) IORegistryEntryCreateCFProperty(IOService,
								CFSTR("Status"),
								kCFAllocatorDefault,
								0);
	if (! SDictionary )
		return 0;

	int pstate = GetNumber(CFSTR("P-State"), SDictionary);
	if(pState){
		pState->pstate = pstate;
		pState->frequency = GetNumber(CFSTR("Frequency"), SDictionary);
		pState->voltage = GetNumber(CFSTR("Voltage"), SDictionary);
		pState->temperature = GetNumber(CFSTR("Temperature"), SDictionary);
	}
	CFRelease(SDictionary);
	return pstate;
}

int getPstateInterval(){
	int interval = 200; // default
	io_service_t	IOService  = IOServiceGetMatchingService(0, IOServiceMatching(SERVICE_NAME));
	if ( IOService ){
		CFDictionaryRef CDictionary = (CFDictionaryRef) IORegistryEntryCreateCFProperty(IOService,					
				CFSTR("Characteristics"),kCFAllocatorDefault,0);
		if(CDictionary){
			int n = GetNumber(CFSTR("TimerInterval"), CDictionary);
			if(n)
				interval = n;
		}
		CFRelease(CDictionary);
	}
	return interval;
}

int getPstateTable(PSTATE_CTL_INFO* info){
	int count = 0;
	io_service_t	IOService  = IOServiceGetMatchingService(0, IOServiceMatching(SERVICE_NAME));
	if (! IOService )
		return 0;

	CFDictionaryRef CDictionary = (CFDictionaryRef) IORegistryEntryCreateCFProperty(IOService,					
				CFSTR("Characteristics"),kCFAllocatorDefault,0);

	CFArrayRef PSArray = CFArrayCreateCopy(kCFAllocatorDefault, 
								(CFArrayRef) CFDictionaryGetValue(CDictionary, CFSTR("PStates")));
	if (PSArray) {
		count = CFArrayGetCount(PSArray);
		for( int k = 0; k < count; k++ ){
			CFDictionaryRef PSDictionary = CFDictionaryCreateCopy(kCFAllocatorDefault,
																  (CFDictionaryRef) CFArrayGetValueAtIndex(PSArray, k));
			info->frequency = GetNumber(CFSTR("Frequency"), PSDictionary);
			info->voltage = GetNumber(CFSTR("Voltage"), PSDictionary);
			info->fid = GetNumber(CFSTR("FID"), PSDictionary);
			info->did = GetNumber(CFSTR("DID"), PSDictionary);
			info->vid = GetNumber(CFSTR("VID"), PSDictionary);
			info->pstate = k;
			info++;
			CFRelease(PSDictionary);
		}
		CFRelease(PSArray);
	}
	CFRelease(CDictionary);
	return count;
}

void setPstate(unsigned int newState){
	io_service_t	IOService  = IOServiceGetMatchingService(0, IOServiceMatching(SERVICE_NAME));
	if (! IOService )
		return;

	integer_t state = newState;
	IORegistryEntrySetCFProperty(IOService, CFSTR("P-State"),
		CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &state));
}