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

#include <IOKit/IOTimerEventSource.h>

#include "AppleACPIBatteryManager.h"
#include "AppleACPIBatteryDevice.h"

// Defines the order of reading properties in the power source state machine
// Bitfield
enum {
    kExistingBatteryPath    = 1,
    kNewBatteryPath         = 2
};

enum {
    kSecondsUntilValidOnWake    = 30,
    kPostChargeWaitSeconds      = 120,
    kPostDischargeWaitSeconds   = 120
};

enum {
    kDefaultPollInterval = 0,
    kQuickPollInterval = 1
};

#define kErrorRetryAttemptsExceeded         "Read Retry Attempts Exceeded"
#define kErrorOverallTimeoutExpired         "Overall Read Timeout Expired"
#define kErrorZeroCapacity                  "Capacity Read Zero"
#define kErrorPermanentFailure              "Permanent Battery Failure"
#define kErrorNonRecoverableStatus          "Non-recoverable status failure"

// Polling intervals
// The battery kext switches between polling frequencies depending on
// battery load
static uint32_t milliSecPollingTable[2] =
    { 
      30000,    // 0 == Regular 30 second polling
      1000      // 1 == Quick 1 second polling
    };

static const uint32_t kBatteryReadAllTimeout = 10000;       // 10 seconds

// Keys we use to publish battery state in our IOPMPowerSource::properties array
static const OSSymbol *_MaxErrSym = 
                        OSSymbol::withCString(kIOPMPSMaxErrKey);
static const OSSymbol *_DeviceNameSym = 
                        OSSymbol::withCString(kIOPMDeviceNameKey);
static const OSSymbol *_FullyChargedSym = 
                        OSSymbol::withCString(kIOPMFullyChargedKey);
static const OSSymbol *_AvgTimeToEmptySym = 
                        OSSymbol::withCString("AvgTimeToEmpty");
static const OSSymbol *_AvgTimeToFullSym = 
                        OSSymbol::withCString("AvgTimeToFull");
static const OSSymbol *_InstantTimeToEmptySym = 
                        OSSymbol::withCString("InstantTimeToEmpty");
static const OSSymbol *_InstantTimeToFullSym = 
                        OSSymbol::withCString("InstantTimeToFull");
static const OSSymbol *_InstantAmperageSym =
                        OSSymbol::withCString("InstantAmperage");
static const OSSymbol *_ManufactureDateSym = 
                        OSSymbol::withCString(kIOPMPSManufactureDateKey);
static const OSSymbol *_DesignCapacitySym = 
                        OSSymbol::withCString(kIOPMPSDesignCapacityKey);
static const OSSymbol *_QuickPollSym = 
                        OSSymbol::withCString("Quick Poll");
static const OSSymbol *_TemperatureSym = 
                        OSSymbol::withCString("Temperature");
static const OSSymbol *_CellVoltageSym = 
                        OSSymbol::withCString("CellVoltage");
static const OSSymbol *_ManufacturerDataSym = 
                        OSSymbol::withCString("ManufacturerData");
static const OSSymbol *_TypeSym = 
                        OSSymbol::withCString("Type");

/* _SerialNumberSym represents the manufacturer's 16-bit serial number in
    numeric format. 
 */
static const OSSymbol *_SerialNumberSym =
                        OSSymbol::withCString("SerialNumber");

/* _SoftwareSerialSym == AppleSoftwareSerial
   represents the Apple-generated user readable serial number that will appear
   in the OS and is accessible to users.
 */
static const OSSymbol *_SoftwareSerialSym =
                        OSSymbol::withCString("BatterySerialNumber");

#define super IOPMPowerSource
OSDefineMetaClassAndStructors(AppleACPIBatteryDevice, IOPMPowerSource)

/******************************************************************************
 * AppleACPIBatteryDevice::ACPIBattery
 *     
 ******************************************************************************/

AppleACPIBatteryDevice * 
AppleACPIBatteryDevice::ACPIBattery(void)
{
	AppleACPIBatteryDevice * me;
	me = new AppleACPIBatteryDevice;

	if (me && !me->init())
	{
		me->release();
		return NULL;
	}

	return me;
}

/******************************************************************************
 * AppleACPIBatteryDevice::init
 *
 ******************************************************************************/

bool AppleACPIBatteryDevice::init(void) 
{
    if (!super::init()) {
        return false;
    }

    fProvider = NULL;
    fWorkLoop = NULL;
    fPollTimer = NULL;

    return true;
}

/******************************************************************************
 * AppleACPIBatteryDevice::start
 *
 ******************************************************************************/

bool AppleACPIBatteryDevice::start(IOService *provider)
{
    OSNumber        *debugPollingSetting;

    fProvider = OSDynamicCast(AppleACPIBatteryManager, provider);

    if (!fProvider || !super::start(provider)) {
        return false;
    }

    debugPollingSetting = (OSNumber *)fProvider->getProperty(kBatteryPollingDebugKey);
    if( debugPollingSetting && OSDynamicCast(OSNumber, debugPollingSetting) )
    {
        /* We set our polling interval to the "BatteryPollingPeriodOverride" property's value,
            in seconds.
            Polling Period of 0 causes us to poll endlessly in a loop for testing.
         */
        fPollingInterval = debugPollingSetting->unsigned32BitValue();
        fPollingOverridden = true;
    } else {
        fPollingInterval = kDefaultPollInterval;
        fPollingOverridden = false;
    }

    fBatteryPresent = false;
    fACConnected = false;
    fACChargeCapable = false;

    fWorkLoop = getWorkLoop();

    fPollTimer = IOTimerEventSource::timerEventSource( this, 
                    OSMemberFunctionCast( IOTimerEventSource::Action, 
                    this, &AppleACPIBatteryDevice::pollingTimeOut) );

    fBatteryReadAllTimer = IOTimerEventSource::timerEventSource( this,
                    OSMemberFunctionCast( IOTimerEventSource::Action,
                    this, &AppleACPIBatteryDevice::incompleteReadTimeOut) );

    if( !fWorkLoop || !fPollTimer
      || (kIOReturnSuccess != fWorkLoop->addEventSource(fPollTimer)) )
    {
        return false;
    }

    // Publish the intended period in seconds that our "time remaining"
    // estimate is wildly inaccurate after wake from sleep.
    setProperty( kIOPMPSInvalidWakeSecondsKey, 
                 kSecondsUntilValidOnWake, 32);

    // Publish the necessary time period (in seconds) that a battery
    // calibrating tool must wait to allow the battery to settle after
    // charge and after discharge.
    setProperty( kIOPMPSPostChargeWaitSecondsKey, 
                 kPostChargeWaitSeconds, 32);
    setProperty( kIOPMPSPostDishargeWaitSecondsKey, 
                 kPostDischargeWaitSeconds, 32);

    // zero out battery state with argument (do_set == true)
    clearBatteryState(false);

    // Kick off the 30 second timer and do an initial poll
    pollBatteryState( kNewBatteryPath );

    return true;
}

/******************************************************************************
 * AppleACPIBatteryDevice::logReadError
 *
 ******************************************************************************/

void AppleACPIBatteryDevice::logReadError(
    const char *error_type, 
    uint16_t additional_error,
    void *t)
{
    if(!error_type) return;

    setProperty((const char *)"LatestErrorType", error_type);

    IOLog("AppleACPIBatteryDevice Error: %s (%d)\n", error_type, additional_error);  

    return;
}

/******************************************************************************
 * AppleACPIBatteryDevice::setPollingInterval
 *
 ******************************************************************************/

void AppleACPIBatteryDevice::setPollingInterval(
    int milliSeconds)
{
    if (!fPollingOverridden) {
        milliSecPollingTable[kDefaultPollInterval] = milliSeconds;
        fPollingInterval = kDefaultPollInterval;
    }
}

/******************************************************************************
 * AppleACPIBatteryDevice::pollBatteryState
 *
 * Asynchronously kicks off the register poll.
 ******************************************************************************/

bool AppleACPIBatteryDevice::pollBatteryState(int path)
{
    // This must be called under workloop synchronization
    if (kNewBatteryPath == path) {
		/* Cancel polling timer in case this round of reads was initiated
		   by an alarm. We re-set the 30 second poll later. */
		fPollTimer->cancelTimeout();

		/* Initialize battery read timeout to catch any longstanding stalls. */           
		fBatteryReadAllTimer->cancelTimeout();
		fBatteryReadAllTimer->setTimeoutMS( kBatteryReadAllTimeout );

		pollBatteryState( kExistingBatteryPath );
	} else {
        fProvider->getBatterySTA();

        if (fBatteryPresent) {
            fProvider->getBatteryBIF();
			fProvider->getBatteryBST();
        } else {
            setFullyCharged(false);
            clearBatteryState(true);
        }

		if (!fPollingOverridden) {
			/* Restart timer with standard polling interval */
			fPollTimer->setTimeoutMS( milliSecPollingTable[fPollingInterval] );
		} else {
			/* restart timer with debug value */
			fPollTimer->setTimeoutMS( 1000 * fPollingInterval );
		}
	}

    return true;
}

void AppleACPIBatteryDevice::handleBatteryInserted(void)
{
    // This must be called under workloop synchronization
    pollBatteryState( kNewBatteryPath );

    return;
}

void AppleACPIBatteryDevice::handleBatteryRemoved(void)
{
    // This must be called under workloop synchronization
    clearBatteryState(true);

    return;
}

/******************************************************************************
 * pollingTimeOut
 *
 * Regular 30 second poll expiration handler.
 ******************************************************************************/

void AppleACPIBatteryDevice::pollingTimeOut(void)
{
	pollBatteryState( kExistingBatteryPath );
}

/******************************************************************************
 * incompleteReadTimeOut
 * 
 * The complete battery read has not completed in the allowed timeframe.
 * We assume this is for several reasons:
 *    - The EC has dropped an SMBus packet (probably recoverable)
 *    - The EC has stalled an SMBus request; IOSMBusController is hung (probably not recoverable)
 *
 * Start the battery read over from scratch.
 *****************************************************************************/

void AppleACPIBatteryDevice::incompleteReadTimeOut(void)
{
    logReadError(kErrorOverallTimeoutExpired, 0, NULL);

	pollBatteryState( kExistingBatteryPath );
}

/******************************************************************************
 * AppleACPIBatteryDevice::clearBatteryState
 *
 ******************************************************************************/

void AppleACPIBatteryDevice::clearBatteryState(bool do_set)
{
    // Only clear out battery state; don't clear manager state like AC Power.
    // We just zero out the int and bool values, but remove the OSType values.
    fBatteryPresent = false;
    fACConnected = false;
    fACChargeCapable = false;

    setBatteryInstalled(false);
    setIsCharging(false);
    setCurrentCapacity(0);
    setMaxCapacity(0);
    setTimeRemaining(0);
    setAmperage(0);
    setVoltage(0);
    setCycleCount(0);
    setAdapterInfo(0);
    setLocation(0);

    properties->removeObject(manufacturerKey);
    removeProperty(manufacturerKey);
    properties->removeObject(serialKey);
    removeProperty(serialKey);
    properties->removeObject(batteryInfoKey);
    removeProperty(batteryInfoKey);
    properties->removeObject(errorConditionKey);
    removeProperty(errorConditionKey);
// setBatteryBIF
	properties->removeObject(_DesignCapacitySym);
    removeProperty(_DesignCapacitySym);
	properties->removeObject(_DeviceNameSym);
    removeProperty(_DeviceNameSym);
	properties->removeObject(_TypeSym);
    removeProperty(_TypeSym);
	properties->removeObject(_MaxErrSym);
    removeProperty(_MaxErrSym);
	properties->removeObject(_ManufactureDateSym);
    removeProperty(_ManufactureDateSym);
	properties->removeObject(_SerialNumberSym);
    removeProperty(_SerialNumberSym);
	properties->removeObject(_ManufacturerDataSym);
    removeProperty(_ManufacturerDataSym);
// setBatteryBST
	properties->removeObject(_AvgTimeToEmptySym);
    removeProperty(_AvgTimeToEmptySym);
	properties->removeObject(_AvgTimeToFullSym);
    removeProperty(_AvgTimeToFullSym);
	properties->removeObject(_InstantTimeToEmptySym);
    removeProperty(_InstantTimeToEmptySym);
	properties->removeObject(_InstantTimeToFullSym);
    removeProperty(_InstantTimeToFullSym);
	properties->removeObject(_InstantAmperageSym);
    removeProperty(_InstantAmperageSym);
	properties->removeObject(_QuickPollSym);
    removeProperty(_QuickPollSym);
	properties->removeObject(_CellVoltageSym);
    removeProperty(_CellVoltageSym);
	properties->removeObject(_TemperatureSym);
    removeProperty(_TemperatureSym);
	properties->removeObject(_SoftwareSerialSym);
    removeProperty(_SoftwareSerialSym);

    rebuildLegacyIOBatteryInfo();

    if(do_set) {
        updateStatus();
    }
}

/******************************************************************************
 *  Package battery data in "legacy battery info" format, readable by
 *  any applications using the not-so-friendly IOPMCopyBatteryInfo()
 ******************************************************************************/

void AppleACPIBatteryDevice::rebuildLegacyIOBatteryInfo(void)
{
    OSDictionary        *legacyDict = OSDictionary::withCapacity(5);
    uint32_t            flags = 0;
    OSNumber            *flags_num = NULL;

    if (externalConnected()) flags |= kIOPMACInstalled;
    if (batteryInstalled()) flags |= kIOPMBatteryInstalled;
    if (isCharging()) flags |= kIOPMBatteryCharging;

    flags_num = OSNumber::withNumber((unsigned long long)flags, 32);
    legacyDict->setObject(kIOBatteryFlagsKey, flags_num);
    flags_num->release();

    legacyDict->setObject(kIOBatteryCurrentChargeKey, properties->getObject(kIOPMPSCurrentCapacityKey));
    legacyDict->setObject(kIOBatteryCapacityKey, properties->getObject(kIOPMPSMaxCapacityKey));
    legacyDict->setObject(kIOBatteryVoltageKey, properties->getObject(kIOPMPSVoltageKey));
    legacyDict->setObject(kIOBatteryAmperageKey, properties->getObject(kIOPMPSAmperageKey));
    legacyDict->setObject(kIOBatteryCycleCountKey, properties->getObject(kIOPMPSCycleCountKey));

    setLegacyIOBatteryInfo(legacyDict);

    legacyDict->release();
}

/******************************************************************************
 *  Fabricate a serial number from our manufacturer, model, manufacture date,
 *  and factory serial numbers. This is unique, and mappable back to Apple's
 *  independently assigned serial number.
 ******************************************************************************/

#define kMaxGeneratedSerialSize (16+16+4+4+4)
void AppleACPIBatteryDevice::constructAppleSerialNumber(void)
{
    OSSymbol        *manf_string = fManufacturer;
    const char *    manf_cstring_ptr;
    OSSymbol        *device_string = fDeviceName;
    const char *    device_cstring_ptr;
    OSSymbol        *type_string = fType;
    const char *    type_cstring_ptr;
    OSSymbol        *serial_string = fSerial;
    const char *    serial_cstring_ptr;

    const OSSymbol  *printableSerial = NULL;
    char            serialBuf[kMaxGeneratedSerialSize];

    if (manf_string) {
        manf_cstring_ptr = manf_string->getCStringNoCopy();
    } else {
        manf_cstring_ptr = "Unknown";
    }

    if (device_string) {
        device_cstring_ptr = device_string->getCStringNoCopy();
    } else {
        device_cstring_ptr = "Unknown";
    }

    if (type_string) {
        type_cstring_ptr = type_string->getCStringNoCopy();
    } else {
        type_cstring_ptr = "Unknown";
    }

    if (serial_string) {
        serial_cstring_ptr = serial_string->getCStringNoCopy();
    } else {
        serial_cstring_ptr = "Unknown";
    }

    bzero(serialBuf, kMaxGeneratedSerialSize);

    snprintf(serialBuf, kMaxGeneratedSerialSize, "%s-%s-%s-%s", 
                manf_cstring_ptr, device_cstring_ptr, type_cstring_ptr, serial_cstring_ptr);

    printableSerial = OSSymbol::withCString(serialBuf);
    if (printableSerial) {
        properties->setObject(_SoftwareSerialSym, (OSObject *)printableSerial);
        printableSerial->release();
    }

    return;
}

/******************************************************************************
 *  Power Source value accessors
 *  These supplement the built-in accessors in IOPMPowerSource.h, and should 
 *  arguably be added back into the superclass IOPMPowerSource
 ******************************************************************************/

void AppleACPIBatteryDevice::setMaxErr(int error)
{
    OSNumber *n = OSNumber::withNumber(error, 32);
    if (n) {
        properties->setObject(_MaxErrSym, n);
        n->release();
    }
}

int AppleACPIBatteryDevice::maxErr(void)
{
    OSNumber *n = OSDynamicCast(OSNumber, properties->getObject(_MaxErrSym));
    if (n) {
        return n->unsigned32BitValue();
    } else {
        return 0;
    }
}

void AppleACPIBatteryDevice::setDeviceName(OSSymbol *sym)
{
    if (sym)
        properties->setObject(_DeviceNameSym, (OSObject *)sym);
}

OSSymbol * AppleACPIBatteryDevice::deviceName(void)
{
    return OSDynamicCast(OSSymbol, properties->getObject(_DeviceNameSym));
}

void AppleACPIBatteryDevice::setFullyCharged(bool charged)
{
    properties->setObject(_FullyChargedSym, 
                   (charged ? kOSBooleanTrue:kOSBooleanFalse));
}

bool AppleACPIBatteryDevice::fullyCharged(void) 
{
    return (kOSBooleanTrue == properties->getObject(_FullyChargedSym));
}

void AppleACPIBatteryDevice::setInstantaneousTimeToEmpty(int seconds)
{
    OSNumber *n = OSNumber::withNumber(seconds, 32);
    if (n) {
        properties->setObject(_InstantTimeToEmptySym, n);
        n->release();
    }
}

void AppleACPIBatteryDevice::setInstantaneousTimeToFull(int seconds)
{
    OSNumber *n = OSNumber::withNumber(seconds, 32);
    if (n) {
        properties->setObject(_InstantTimeToFullSym, n);
        n->release();
    }
}

void AppleACPIBatteryDevice::setInstantAmperage(int mA)
{
    OSNumber *n = OSNumber::withNumber(mA, 32);
    if (n) {
        properties->setObject(_InstantAmperageSym, n);
        n->release();
    }
}

void AppleACPIBatteryDevice::setAverageTimeToEmpty(int seconds)
{
    OSNumber *n = OSNumber::withNumber(seconds, 32);
    if (n) {
        properties->setObject(_AvgTimeToEmptySym, n);
        n->release();
    }
}

int AppleACPIBatteryDevice::averageTimeToEmpty(void)
{
    OSNumber *n = OSDynamicCast(OSNumber, properties->getObject(_AvgTimeToEmptySym));
    if (n) {
        return n->unsigned32BitValue();
    } else {
        return 0;
    }
}

void AppleACPIBatteryDevice::setAverageTimeToFull(int seconds)
{
    OSNumber *n = OSNumber::withNumber(seconds, 32);
    if (n) {
        properties->setObject(_AvgTimeToFullSym, n);
        n->release();
    }
}

int AppleACPIBatteryDevice::averageTimeToFull(void)
{
    OSNumber *n = OSDynamicCast(OSNumber, properties->getObject(_AvgTimeToFullSym));
    if (n) {
        return n->unsigned32BitValue();
    } else {
        return 0;
    }
}

void AppleACPIBatteryDevice::setManufactureDate(int date)
{
    OSNumber *n = OSNumber::withNumber(date, 32);
    if (n) {
        properties->setObject(_ManufactureDateSym, n);
        n->release();
    }
}

int AppleACPIBatteryDevice::manufactureDate(void)
{
    OSNumber *n = OSDynamicCast(OSNumber, properties->getObject(_ManufactureDateSym));
    if (n) {
        return n->unsigned32BitValue();
    } else {
        return 0;
    }
}

void AppleACPIBatteryDevice::setSerialNumber(int sernum)
{
    OSNumber *n = OSNumber::withNumber(sernum, 32);
    if (n) {
        properties->setObject(_SerialNumberSym, n);
        n->release();
    }
}

int AppleACPIBatteryDevice::serialNumber(void)
{
    OSNumber *n = OSDynamicCast(OSNumber, properties->getObject(_SerialNumberSym));
    if (n) {
        return n->unsigned16BitValue();
    } else {
        return 0;
    }
}

void AppleACPIBatteryDevice::setManufacturerData(uint8_t *buffer, uint32_t bufferSize)
{
    OSData *newData = OSData::withBytes( buffer, bufferSize );
    if (newData) {
        properties->setObject(_ManufacturerDataSym, newData);
        newData->release();
    }
}

void AppleACPIBatteryDevice::setDesignCapacity(unsigned int val)
{
    OSNumber *n = OSNumber::withNumber(val, 32);
    properties->setObject(_DesignCapacitySym, n);
    n->release();
}

unsigned int AppleACPIBatteryDevice::designCapacity(void) {
    OSNumber *n;
    n = OSDynamicCast(OSNumber, properties->getObject(_DesignCapacitySym));
    if(!n) return 0;
    else return (unsigned int)n->unsigned32BitValue();
}

void AppleACPIBatteryDevice::setType(OSSymbol *sym)
{
    if (sym)
        properties->setObject(_TypeSym, (OSObject *)sym);
}

OSSymbol * AppleACPIBatteryDevice::Type(void)
{
    return OSDynamicCast(OSSymbol, properties->getObject(_TypeSym));
}

/******************************************************************************
 * AppleACPIBatteryDevice::setBatterySTA
 *
 ******************************************************************************/

IOReturn AppleACPIBatteryDevice::setBatterySTA(UInt32 acpibat_bif)
{
	if (acpibat_bif & 0x10) {
		fBatteryPresent = true;
		setBatteryInstalled(fBatteryPresent);
	} else {
		fBatteryPresent = false;
		setBatteryInstalled(fBatteryPresent);
	}

	return kIOReturnSuccess;
}

/******************************************************************************
 * AppleACPIBatteryDevice::setBatteryBIF
 *
 ******************************************************************************/
/*
 * _BIF (Battery InFormation)
 * Arguments: none
 * Results  : package _BIF (Battery InFormation)
 * Package {
 * 	// ASCIIZ is ASCII character string terminated with a 0x00.
 * 	Power Unit						//DWORD
 * 	Design Capacity					//DWORD
 * 	Last Full Charge Capacity		//DWORD
 * 	Battery Technology				//DWORD
 * 	Design Voltage					//DWORD
 * 	Design Capacity of Warning		//DWORD
 * 	Design Capacity of Low			//DWORD
 * 	Battery Capacity Granularity 1	//DWORD
 * 	Battery Capacity Granularity 2	//DWORD
 * 	Model Number					//ASCIIZ
 * 	Serial Number					//ASCIIZ
 * 	Battery Type					//ASCIIZ
 * 	OEM Information					//ASCIIZ
 * }
 */

IOReturn AppleACPIBatteryDevice::setBatteryBIF(OSArray *acpibat_bif)
{
	fDesignVoltage = OSDynamicCast(OSNumber, acpibat_bif->getObject(4))->unsigned32BitValue();

	fUnitFactor = OSDynamicCast(OSNumber, acpibat_bif->getObject(0))->unsigned32BitValue();
	if (fUnitFactor == 0) fUnitFactor = fDesignVoltage / 1000;
	if (fUnitFactor == 0) fUnitFactor = 1; //FIXME: 0 Power Unit ?

	fDesignCapacity = OSDynamicCast(OSNumber, acpibat_bif->getObject(1))->unsigned32BitValue();
	if ((fUnitFactor > 1) && (fDesignCapacity < 10000)) { //FIXME: Wrong Power Unit ?
		fUnitFactor = 1;
	}

	fDesignCapacity = OSDynamicCast(OSNumber, acpibat_bif->getObject(1))->unsigned32BitValue() / fUnitFactor;
	setDesignCapacity(fDesignCapacity);

	fMaxCapacity = OSDynamicCast(OSNumber, acpibat_bif->getObject(2))->unsigned32BitValue() / fUnitFactor;
	setMaxCapacity(fMaxCapacity);

	if ((fDesignCapacity == 0) || (fMaxCapacity == 0))  {
		logReadError(kErrorZeroCapacity, 0, NULL);
	}

    const OSMetaClass *typeID;
    const OSSymbol *stringSym;
    char stringBuf[255];

    typeID = OSTypeIDInst(acpibat_bif->getObject(9)); //FIXME: OSString or OSData ?
	if (typeID == OSTypeID(OSString)) {
		fDeviceName = (OSSymbol *)OSDynamicCast(OSString, acpibat_bif->getObject(9));
	} else if (typeID == OSTypeID(OSData)) {
		bzero(stringBuf, sizeof(stringBuf));
		snprintf(stringBuf, sizeof(stringBuf), "%s", 
					OSDynamicCast(OSData, acpibat_bif->getObject(9))->getBytesNoCopy());
		stringSym = OSSymbol::withCString(stringBuf);
		fDeviceName = (OSSymbol *)stringSym;
	}
	setDeviceName(fDeviceName);

    typeID = OSTypeIDInst(acpibat_bif->getObject(10)); //FIXME: OSString or OSData ?
	if (typeID == OSTypeID(OSString)) {
		fSerial = (OSSymbol *)OSDynamicCast(OSString, acpibat_bif->getObject(10));
	} else if (typeID == OSTypeID(OSData)) {
		bzero(stringBuf, sizeof(stringBuf));
		snprintf(stringBuf, sizeof(stringBuf), "%s", 
					OSDynamicCast(OSData, acpibat_bif->getObject(10))->getBytesNoCopy());
		stringSym = OSSymbol::withCString(stringBuf);
		fSerial = (OSSymbol *)stringSym;
	}
	setSerial(fSerial);

    typeID = OSTypeIDInst(acpibat_bif->getObject(11)); //FIXME: OSString or OSData ?
	if (typeID == OSTypeID(OSString)) {
		fType = (OSSymbol *)OSDynamicCast(OSString, acpibat_bif->getObject(11));
	} else if (typeID == OSTypeID(OSData)) {
		bzero(stringBuf, sizeof(stringBuf));
		snprintf(stringBuf, sizeof(stringBuf), "%s", 
					OSDynamicCast(OSData, acpibat_bif->getObject(11))->getBytesNoCopy());
		stringSym = OSSymbol::withCString(stringBuf);
		fType = (OSSymbol *)stringSym;
	}
	setType(fType);

    typeID = OSTypeIDInst(acpibat_bif->getObject(12)); //FIXME: OSString or OSData ?
	if (typeID == OSTypeID(OSString)) {
		fManufacturer = (OSSymbol *)OSDynamicCast(OSString, acpibat_bif->getObject(12));
	} else if (typeID == OSTypeID(OSData)) {
		bzero(stringBuf, sizeof(stringBuf));
		snprintf(stringBuf, sizeof(stringBuf), "%s", 
					OSDynamicCast(OSData, acpibat_bif->getObject(12))->getBytesNoCopy());
		stringSym = OSSymbol::withCString(stringBuf);
		fManufacturer = (OSSymbol *)stringSym;
	}
	setManufacturer(fManufacturer);

	if (stringSym) stringSym->release();

	setMaxErr(0);
	setManufactureDate(0);
	setSerialNumber(0);

	fManufacturerData = OSData::withCapacity(10);
	setManufacturerData((uint8_t *)fManufacturerData, fManufacturerData->getLength());

	return kIOReturnSuccess;
}

/******************************************************************************
 * AppleACPIBatteryDevice::setBatteryBST
 *
 ******************************************************************************/
/*
 * _BST (Battery STatus)
 * Arguments: none
 * Results  : package _BST (Battery STatus)
 * Package {
 * 	Battery State				//DWORD
 * 	Battery Present Rate		//DWORD
 * 	Battery Remaining Capacity	//DWORD
 * 	Battery Present Voltage		//DWORD
 * }
 */

IOReturn AppleACPIBatteryDevice::setBatteryBST(OSArray *acpibat_bst)
{
	fCurrentVoltage = OSDynamicCast(OSNumber, acpibat_bst->getObject(3))->unsigned32BitValue();
	setVoltage(fCurrentVoltage);

	fCurentCapacity = OSDynamicCast(OSNumber, acpibat_bst->getObject(2))->unsigned32BitValue() / fUnitFactor;
	setCurrentCapacity(fCurentCapacity);

	fCurrentRate = (OSDynamicCast(OSNumber, acpibat_bst->getObject(1))->unsigned32BitValue()) / fUnitFactor;
	if (fCurrentRate <= 0x00000000)
		fCurrentRate = fMaxCapacity / 2;
	else
		fCurrentRate &= 0x00000FFF;

	if (fAverageRate) {
		fAverageRate = (fAverageRate + fCurrentRate) / 2;
	} else {
		fAverageRate = fCurrentRate;
	}

	UInt32 value = OSDynamicCast(OSNumber, acpibat_bst->getObject(0))->unsigned32BitValue();
	DEBUG_LOG("AppleACPIBatteryDevice: Battery State 0x%x.\n", value);

	if (value ^ fStatus) {
		fStatus = value;
		fAverageRate = 0;
	}

	if ((value & 0x01) && (value & 0x02)) {
		const OSSymbol *permanentFailureSym =
			OSSymbol::withCString(kErrorPermanentFailure);

		logReadError( kErrorPermanentFailure, 0, NULL);
		setErrorCondition( (OSSymbol *)permanentFailureSym );
		permanentFailureSym->release();

		/* We want to display the battery as present & completely discharged, not charging */
		setFullyCharged(false);
		setIsCharging(false);

		fACConnected = true;
		setExternalConnected(fACConnected);
		fACChargeCapable = false;
		setExternalChargeCapable(fACChargeCapable);

		setAmperage(0);
		setInstantAmperage(0);

		setTimeRemaining(0);
		setAverageTimeToEmpty(0);
		setAverageTimeToFull(0);
		setInstantaneousTimeToFull(0);
		setInstantaneousTimeToEmpty(0);

		DEBUG_LOG("AppleACPIBatteryDevice: Battery Charging and Discharging?\n");
	} else if (value & 0x01) {
		setFullyCharged(false);
		setIsCharging(false);

		fACConnected = false;
		setExternalConnected(fACConnected);
		fACChargeCapable = false;
		setExternalChargeCapable(fACChargeCapable);

		setAmperage(fAverageRate * -1);
		setInstantAmperage(fCurrentRate * -1);

		if (fAverageRate)
			setTimeRemaining((60 * fCurentCapacity) / fAverageRate);
		else
			setTimeRemaining(0xffff);

		setAverageTimeToFull(0xffff);

		if (fAverageRate)
			setAverageTimeToEmpty((60 * fCurentCapacity) / fAverageRate);
		else
			setAverageTimeToEmpty(0xffff);

		setInstantaneousTimeToFull(0xffff);

		if (fCurrentRate)
			setInstantaneousTimeToEmpty((60 * fCurentCapacity) / fCurrentRate);
		else
			setInstantaneousTimeToEmpty(0xffff);

		DEBUG_LOG("AppleACPIBatteryDevice: Battery is discharging.\n");
	} else if (value & 0x02) {
		setFullyCharged(false);
		setIsCharging(true);

		fACConnected = true;
		setExternalConnected(fACConnected);
		fACChargeCapable = true;
		setExternalChargeCapable(fACChargeCapable);

		setAmperage(fAverageRate);
		setInstantAmperage(fCurrentRate);

		if (fAverageRate)
			setTimeRemaining((60 * (fMaxCapacity - fCurentCapacity)) / fAverageRate);
		else
			setTimeRemaining(0xffff);

		if (fAverageRate)
			setAverageTimeToFull((60 * (fMaxCapacity - fCurentCapacity)) / fAverageRate);
		else
			setAverageTimeToFull(0xffff);

		setAverageTimeToEmpty(0xffff);

		if (fCurrentRate)
			setInstantaneousTimeToFull((60 * (fMaxCapacity - fCurentCapacity)) / fCurrentRate);
		else
			setInstantaneousTimeToFull(0xffff);

		setInstantaneousTimeToEmpty(0xffff);

		DEBUG_LOG("AppleACPIBatteryDevice: Battery is charging.\n");
	} else {
		setFullyCharged(true);
		setIsCharging(false);

		fACConnected = true;
		setExternalConnected(fACConnected);
		fACChargeCapable = true;
		setExternalChargeCapable(fACChargeCapable);

		setAmperage(0);
		setInstantAmperage(0);

		setTimeRemaining(0xffff);
		setAverageTimeToFull(0xffff);
		setAverageTimeToEmpty(0xffff);
		setInstantaneousTimeToFull(0xffff);
		setInstantaneousTimeToEmpty(0xffff);

		fCurentCapacity = fMaxCapacity;
		setCurrentCapacity(fCurentCapacity);

		DEBUG_LOG("AppleACPIBatteryDevice: Battery is charged.\n");
	}

	if (!fPollingOverridden && fMaxCapacity) {
		/*
		 * Conditionally set polling interval to 1 second if we're
		 *     discharging && below 5% && on AC power
		 * i.e. we're doing an Inflow Disabled discharge
		 */
		if ((((100*fCurentCapacity) / fMaxCapacity) < 5) && fACConnected) {
			setProperty("Quick Poll", true);
			fPollingInterval = kQuickPollInterval;
		} else {
			setProperty("Quick Poll", false);
			fPollingInterval = kDefaultPollInterval;
		}
	}
#if 1
	if (fDesignCapacity > fMaxCapacity) { // FIXME: Unknown denominator ?
		setCycleCount((fDesignCapacity - fMaxCapacity) / 7);
	} else {
		setCycleCount((fMaxCapacity - fDesignCapacity) / 7);
	}
#endif
    OSNumber *num;
	fCellVoltages = OSArray::withCapacity(4); // FIXME: Assuming 3 cells ?

	fCellVoltage1 = fCurrentVoltage / 3;
    num = OSNumber::withNumber((unsigned long long)fCellVoltage1 , 16);
    fCellVoltages->setObject(num);

	fCellVoltage2 = fCurrentVoltage  /3;
    num = OSNumber::withNumber((unsigned long long)fCellVoltage2 , 16);
    fCellVoltages->setObject(num);

	fCellVoltage3 = fCurrentVoltage - fCellVoltage1 - fCellVoltage2;
    num = OSNumber::withNumber((unsigned long long)fCellVoltage3 , 16);
    fCellVoltages->setObject(num);

	fCellVoltage4 = 0;
    num = OSNumber::withNumber((unsigned long long)fCellVoltage4 , 16);
    fCellVoltages->setObject(num);

	setProperty("CellVoltage", fCellVoltages);

	fTemperature = fCurrentVoltage / 4; // FIXME: Skipping Amperage variable ?
	setProperty("Temperature", (long long unsigned int)fTemperature, (unsigned int)16);

	/* construct and publish our battery serial number here */
	constructAppleSerialNumber();

	/* Cancel read-completion timeout; Successfully read battery state */
	fBatteryReadAllTimer->cancelTimeout();

	rebuildLegacyIOBatteryInfo();

	updateStatus();

	return kIOReturnSuccess;
}
