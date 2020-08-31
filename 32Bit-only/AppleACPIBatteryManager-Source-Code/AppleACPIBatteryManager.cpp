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

#include <IOKit/pwr_mgt/RootDomain.h>
#include <IOKit/IOCommandGate.h>

#include "AppleACPIBatteryManager.h"
#include "AppleACPIBatteryDevice.h"

enum {
    kMyOnPowerState = 1
};

static IOPMPowerState myTwoStates[2] = {
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMPowerOn, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
};

#define super IOService

OSDefineMetaClassAndStructors(AppleACPIBatteryManager, IOService)

/******************************************************************************
 * AppleACPIBatteryManager::start
 *
 ******************************************************************************/

bool AppleACPIBatteryManager::start(IOService *provider)
{
    fProvider = OSDynamicCast(IOACPIPlatformDevice, provider);

    if (!fProvider || !super::start(provider)) {
        return false;
    }

    IOWorkLoop *wl = getWorkLoop();
    if (!wl) {
        return false;
    }

    // Join power management so that we can get a notification early during
    // wakeup to re-sample our battery data. We don't actually power manage
    // any devices.
	PMinit();
    registerPowerDriver(this, myTwoStates, 2);
    provider->joinPMtree(this);

	int value = getPlatform()->numBatteriesSupported();
	DEBUG_LOG("AppleACPIBatteryManager: Battery Supported Count(s) %d.\n", value);

	if (value > 1) { //FIXME: More than one battery bay ?
		if (kIOReturnSuccess == fProvider->evaluateInteger("_STA", &fBatterySTA)) {
			if (fBatterySTA & 0x10) {
				goto populateBattery;
			} else {
				goto skipBattery;
			}
		}
	}

populateBattery:

	fBattery = AppleACPIBatteryDevice::ACPIBattery();

	if(!fBattery) return false;

	fBattery->attach(this);

	fBattery->start(this);

    // Command gate for ACPIBatteryManager
    fManagerGate = IOCommandGate::commandGate(this);
    if (!fManagerGate) {
        return false;
    }
    wl->addEventSource(fManagerGate);

    // Command gate for ACPIBattery
    fBatteryGate = IOCommandGate::commandGate(fBattery);
    if (!fBatteryGate) {
        return false;
    }
    wl->addEventSource(fBatteryGate);

	fBattery->registerService(0);

skipBattery:

	this->registerService(0);

    return true;
}

/******************************************************************************
 * AppleACPIBatteryManager::setPollingInterval
 *
 ******************************************************************************/

IOReturn AppleACPIBatteryManager::setPollingInterval(int milliSeconds)
{
    // Discard any negatize or zero arguments
    if (milliSeconds <= 0) return kIOReturnBadArgument;

    if (fBattery)
        fBattery->setPollingInterval(milliSeconds);

    setProperty("PollingInterval_msec", milliSeconds, 32);

    return kIOReturnSuccess;
}

/******************************************************************************
 * AppleACPIBatteryManager::setPowerState
 *
 ******************************************************************************/

IOReturn AppleACPIBatteryManager::setPowerState(unsigned long which, IOService *whom)
{
    if( (kMyOnPowerState == which)
        && fBatteryGate )
	{
        // We are waking from sleep - kick off a battery read to make sure
        // our battery concept is in line with reality.
        fBatteryGate->runAction(OSMemberFunctionCast(IOCommandGate::Action,
                           fBattery, &AppleACPIBatteryDevice::pollBatteryState),
                           (void *)1, NULL, NULL, NULL); // kNewBatteryPath = 1
    }

    return IOPMAckImplied;
}

/******************************************************************************
 * AppleACPIBatteryManager::message
 *
 ******************************************************************************/

IOReturn AppleACPIBatteryManager::message(UInt32 type, IOService *provider, void *argument)
{
    UInt32 batterySTA;

	if( (kIOACPIMessageDeviceNotification == type)
        && (kIOReturnSuccess == fProvider->evaluateInteger("_STA", &batterySTA))
		&& fBatteryGate )
	{
		if (batterySTA ^ fBatterySTA) {
			if (batterySTA & 0x10) {
				// Battery inserted
				fBatteryGate->runAction(OSMemberFunctionCast(IOCommandGate::Action,
                               fBattery, &AppleACPIBatteryDevice::handleBatteryInserted),
                               NULL, NULL, NULL, NULL);
			} else {
				// Battery removed
				fBatteryGate->runAction(OSMemberFunctionCast(IOCommandGate::Action,
                               fBattery, &AppleACPIBatteryDevice::handleBatteryRemoved),
                               NULL, NULL, NULL, NULL);
			}
		} else {
            // Just an alarm; re-read battery state.
            fBatteryGate->runAction(OSMemberFunctionCast(IOCommandGate::Action,
                               fBattery, &AppleACPIBatteryDevice::pollBatteryState),
                               NULL, NULL, NULL, NULL);
		}
	}

    return kIOReturnSuccess;
}

/******************************************************************************
 * AppleACPIBatteryManager::getBatterySTA
 *
 ******************************************************************************/

IOReturn AppleACPIBatteryManager::getBatterySTA(void)
{
	if (kIOReturnSuccess == fProvider->evaluateInteger("_STA", &fBatterySTA)) {
		return fBattery->setBatterySTA(fBatterySTA);
	} else {
		return kIOReturnError;
	}
}

/******************************************************************************
 * AppleACPIBatteryManager::getBatteryBIF
 *
 ******************************************************************************/

IOReturn AppleACPIBatteryManager::getBatteryBIF(void)
{
	OSObject * batteryBIF;

	if (kIOReturnSuccess == fProvider->evaluateObject("_BIF", &batteryBIF)) {
		OSArray * acpibat_bif = OSDynamicCast(OSArray, batteryBIF);
		setProperty("Battery Information", acpibat_bif);
		IOReturn value = fBattery->setBatteryBIF(acpibat_bif);
		acpibat_bif->release();
		return value;
	} else {
		return kIOReturnError;
	}
}

/******************************************************************************
 * AppleACPIBatteryManager::getBatteryBST
 *
 ******************************************************************************/

IOReturn AppleACPIBatteryManager::getBatteryBST(void)
{
	OSObject * batteryBST;

	if (kIOReturnSuccess == fProvider->evaluateObject("_BST", &batteryBST)) {
		OSArray * acpibat_bst = OSDynamicCast(OSArray, batteryBST);
		setProperty("Battery Status", acpibat_bst);
		IOReturn value = fBattery->setBatteryBST(acpibat_bst);
		acpibat_bst->release();
		return value;
	} else {
		return kIOReturnError;
	}
}
