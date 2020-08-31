/*
 *  preference.c
 *  PStateChanger
 *
 */

#include "preference.h"
#include <CoreFoundation/CoreFoundation.h>

#define kKeyCFPrefStr CFSTR("Preference")
#define kKeyCFIntervalStr CFSTR("Interval")
#define kKeyCFThresHighestStr CFSTR("Threshold Higest")
#define kKeyCFThresLowestStr CFSTR("Threshold Lowest")
#define kKeyCFThresUpStr CFSTR("Threshold Up")
#define kKeyCFThresDownStr CFSTR("Threshold Down")
#define kKeyCFPStateMinStr CFSTR("PState Min")
#define kKeyCFPStateMaxStr CFSTR("PState Max")
#define kKeyCFDocMethod CFSTR("Doc Method")

enum {
	MIN_INTERVAL = 200,
	MAX_INTERVAL = 2000,
};

static void GetNumber (CFStringRef key, CFDictionaryRef dictionary, int* pNum) {
	int ret;
	const void* ref; 
	if(CFDictionaryGetValueIfPresent(dictionary, key, &ref)){
		if (CFNumberGetValue((CFNumberRef)ref, kCFNumberIntType, &ret)) {
			*pNum = ret;
		}
	}
}

static void SetNumber (CFStringRef key, CFMutableDictionaryRef dictionary, int num) {
	CFDictionaryAddValue(dictionary, key, CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &num));
}


int checkPref(PSTATECHANGER_PREF* pPref, int pstateCount, int minInterval){
	int invalid = 0;
	if(minInterval == 0)
		minInterval = MIN_INTERVAL;
	if(pPref->interval < minInterval || pPref->interval > MAX_INTERVAL ){
		pPref->interval = -1;
		invalid += 1;
	}
	if( pPref->threshold_highest > 100  ){
		pPref->threshold_highest = -1;
		invalid += 1;
	}
	if( pPref->threshold_highest < pPref->threshold_up  ){
		pPref->threshold_up = -1;
		invalid += 1;
	}
	if( pPref->threshold_up < pPref->threshold_down ){
		pPref->threshold_down = -1;
		invalid += 1;
	}
	if( pPref->threshold_down < pPref->threshold_lowest || pPref->threshold_lowest < 0 ){
		pPref->threshold_lowest = -1;
		invalid += 1;
	}
	if(pPref->pstate_min < 0  ){
		pPref->pstate_min = -1;
		invalid += 1;
	}
	if( pPref->pstate_max >= pstateCount ||pPref->pstate_min > pPref->pstate_max ){
		pPref->pstate_max = -1;
		invalid += 1;
	}
	if(pPref->doc_method < 0 || pPref->doc_method > 1 ){
		pPref->doc_method = -1;
		invalid += 1;
	}
	return invalid;
}

int savePref(PSTATECHANGER_PREF* pPref){
    CFMutableDictionaryRef	prefCFDicRef = CFDictionaryCreateMutable(NULL, 0, NULL, NULL);
	SetNumber(kKeyCFIntervalStr, prefCFDicRef, pPref->interval);
	SetNumber(kKeyCFThresHighestStr, prefCFDicRef, pPref->threshold_highest);
	SetNumber(kKeyCFThresUpStr, prefCFDicRef, pPref->threshold_up);
	SetNumber(kKeyCFThresDownStr, prefCFDicRef, pPref->threshold_down);
	SetNumber(kKeyCFThresLowestStr, prefCFDicRef, pPref->threshold_lowest);
	SetNumber(kKeyCFPStateMinStr, prefCFDicRef, pPref->pstate_min);
	SetNumber(kKeyCFPStateMaxStr, prefCFDicRef, pPref->pstate_max);
	SetNumber(kKeyCFDocMethod, prefCFDicRef, pPref->doc_method);
	CFPreferencesSetAppValue(kKeyCFPrefStr,prefCFDicRef,kCFPreferencesCurrentApplication);
	CFPreferencesAppSynchronize(kCFPreferencesCurrentApplication);
	CFRelease(prefCFDicRef);
	return 1;
}

void loadPref(PSTATECHANGER_PREF* pPref, int pstateCount){
	defaultPref(pPref, pstateCount);
	CFDictionaryRef	prefCFDicRef = CFPreferencesCopyAppValue(kKeyCFPrefStr,kCFPreferencesCurrentApplication);
	if(prefCFDicRef){
		GetNumber(kKeyCFIntervalStr, prefCFDicRef, &pPref->interval);
		GetNumber(kKeyCFThresHighestStr, prefCFDicRef, &pPref->threshold_highest);
		GetNumber(kKeyCFThresUpStr, prefCFDicRef, &pPref->threshold_up);
		GetNumber(kKeyCFThresDownStr, prefCFDicRef, &pPref->threshold_down);
		GetNumber(kKeyCFThresLowestStr, prefCFDicRef, &pPref->threshold_lowest);
		GetNumber(kKeyCFPStateMinStr, prefCFDicRef, &pPref->pstate_min);
		GetNumber(kKeyCFPStateMaxStr, prefCFDicRef, &pPref->pstate_max);
		GetNumber(kKeyCFDocMethod, prefCFDicRef, &pPref->doc_method);
		CFRelease(prefCFDicRef);
		if(pPref->pstate_max > pstateCount-1)
			pPref->pstate_max = pstateCount-1;
		if(pPref->pstate_min > pPref->pstate_max)
			pPref->pstate_min = pPref->pstate_max;
	}
}

void defaultPref(PSTATECHANGER_PREF* pPref, int pstateCount){
	pPref->interval = 400;	// ms
	pPref->threshold_highest = 90;	// %
	pPref->threshold_up = 50;	// %
	pPref->threshold_down = 25;	// %
	pPref->threshold_lowest = 10;	// %
	pPref->pstate_min = 0;
	pPref->pstate_max = pstateCount-1;
	pPref->doc_method = 1;	// show
}
