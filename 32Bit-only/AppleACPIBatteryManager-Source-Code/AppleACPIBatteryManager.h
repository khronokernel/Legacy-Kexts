/*
 * Copyright (c) 2005 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this
 * file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */

#ifndef __AppleACPIBatteryManager__
#define __AppleACPIBatteryManager__

#include <IOKit/IOService.h>
#include <IOKit/acpi/IOACPIPlatformDevice.h>

#include "AppleACPIBatteryDevice.h"

#if 0
#define DEBUG_LOG(args...)  IOLog(args)
#else
#define DEBUG_LOG(args...)
#endif

class AppleACPIBatteryDevice;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

class AppleACPIBatteryManager : public IOService {
	OSDeclareDefaultStructors(AppleACPIBatteryManager)

public:
    bool start(IOService *provider);

    IOReturn setPowerState(unsigned long which, IOService *whom);

    IOReturn message(UInt32 type, IOService *provider, void *argument);

private:
    IOReturn setPollingInterval(int milliSeconds);
    
    IOCommandGate          * fManagerGate;
    IOCommandGate          * fBatteryGate;
	IOACPIPlatformDevice   * fProvider;
	AppleACPIBatteryDevice * fBattery;

public:
    UInt32 fBatterySTA;

	IOReturn getBatterySTA(void);

	IOReturn getBatteryBIF(void);

	IOReturn getBatteryBST(void);

};

#endif