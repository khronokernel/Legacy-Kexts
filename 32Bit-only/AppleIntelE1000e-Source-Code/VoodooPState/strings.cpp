/*
 *  strings.cpp
 *  VoodooPState
 *
 *  Created by hnak on 09/03/14.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */
#include "Support.h"
#include "VoodooPState.h"

#define defString(s) defXString(s)
#define defXString(s) #s

const char * KextVersion = defString(KEXT_VERSION);
const char * KextProductName = defString(KEXT_PRODUCTNAME);
const char * KextOSX = defString(KEXT_OSX);
const char * KextConfig = defString(KEXT_CONFIG);
const char * KextBuildDate = __DATE__;
const char * KextBuildTime = __TIME__;

const char *	keyPowerControl		=	"PowerControl";
const char *	keyVoltageOverride	=	"VoltageOverride";
const char *	keyVoltageProbe		=	"VoltageProbe";
const char *	keyUserVoltageMin	=	"UserVoltageMin";
const char *	keyUserVoltageMax	=	"UserVoltageMax";
const char *	keyUseEfiFsb		=	"UseEfiFsb";
const char *	keyPStateLimitHigh	=	"PStateLimitHigh";
const char *	keyPStateLimitLow	=	"PStateLimitLow";
const char *	keyTStateControl	=	"TStateControl";
const char *	keyPStates			=	"PStates";
const char *	keyFid				=	"FID";
const char *	keyDid				=	"DID";
const char *	keyVid				=	"VID";
const char *	keyProductName		=	"Product Name";
const char *	keyProductVersion	=	"Product Version";
const char *	keyBuildConfig		=	"Build Config";
const char *	keyBuildDate		=	"Build Date";
const char *	keyBuildTime		=	"Build Time";
const char *	keyCpuCoreTech		=	"CPU Core Technology";
const char *	keyFrontSideBus		=	"Front Side Bus";
const char *	keyCharacteristics	=	"Characteristics";
const char *	keyCurrentPState	=	"P-State";
const char *	keyCurrentThrottle	=	"Throttle";
const char *	keyTemperature	=	"Temperature";
const char *	keyCurrentFrequency	=	"Frequency";
const char *	keyCurrentVoltage	=	"Voltage";
const char *	keyCurrentProfile	=	"Profile";
const char *	keyStatus			=	"Status";
const char *	keyEfiPlatform		=	"/efi/platform";
const char *	keyEfiFsbFrequency	=	"FSBFrequency";
const char *	keyColdStart		=   "ColdStart";
const char *	keyTimerInterval	=   "TimerInterval";
const char *	keyUseACPI			=   "UseACPI";
