/*
 * Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * The contents of this file constitute Original Code as defined in and
 * are subject to the Apple Public Source License Version 1.1 (the
 * "License").  You may not use this file except in compliance with the
 * License.  Please obtain a copy of the License at
 * http://www.apple.com/publicsource and read it before using this file.
 * 
 * This Original Code and all software distributed under the License are
 * distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
 * License for the specific language governing rights and limitations
 * under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */

#include <IOKit/IOLib.h>
#include <IOKit/IODeviceTreeSupport.h>

#include <sys/systm.h>    // snprintf
#include <IOKit/assert.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>

#include "ApplePCIIDE.h"
#include "ApplePCIIDEHardware.h"

// I could not make UDMA work with IT821x.
#define	ITE_FORCE_PIO	1

static bool isSataLink(UInt16 device){
	return (device == PCI_SIL_3112 ||device == PCI_SIL_3512 || device == PCI_SIL_3114);
}

OSDefineMetaClassAndStructors( ApplePCIIDERoot, IOService )

//---------------------------------------------------------------------------
//
// Probe for PCI device and verify that I/O space decoding is enabled.
//

IOService * ApplePCIIDERoot::probe( IOService * provider, SInt32 * score )
{
    IOPCIDevice * pciDevice;
	
    // Let superclass probe first.
	
    if (IOService::probe( provider, score ) == 0)
    {
        return 0;
    }
	
    // Verify the provider is an IOPCIDevice.
	
    pciDevice = OSDynamicCast( IOPCIDevice, provider );
    if (pciDevice == 0)
    {
        return 0;
    }
	
    // Fail if I/O space decoding is disabled.

    if ((pciDevice->configRead16( kIOPCIConfigCommand ) &
         kIOPCICommandIOSpace) == 0)
    {
        return 0;
    }
	
    return this;
}

//---------------------------------------------------------------------------
//
// Start the Root ATA driver. Probe both primary and secondary ATA channels.
//

static void registerClientApplier( IOService * service, void * context )
{
    if ( service ) service->registerService();
}

bool ApplePCIIDERoot::initHardware()
{
	UInt16 vendor = fProvider->configRead16(kIOPCIConfigVendorID);
	UInt16 device = fProvider->configRead16(kIOPCIConfigDeviceID);
	//IOLog("ApplePCIIDERoot::initHardware %4x:%4x\n", vendor, device);

	if(vendor == PCI_SIL_ID){	// SiliconImage
		if(device == PCI_SIL_0680){	// SiL0680
			// sequence ported from NetBSD
			fProvider->configWrite8(0x80, 0x00);
			fProvider->configWrite8(0x84, 0x00);
			fProvider->configWrite8(0x8a, fProvider->configRead8(0x8a) | 0x01);
		} else if(isSataLink(device)){
			UInt8 scs_cmd = fProvider->configRead8(SII3112_SCS_CMD);
			fProvider->configWrite8(SII3112_SCS_CMD,scs_cmd | SII3112_RESET_BITS);
			IODelay(50 * 1000);
			fProvider->configWrite8(SII3112_SCS_CMD,scs_cmd & SCS_CMD_BA5_EN);
			IODelay(50 * 1000);
			
			if(device == PCI_SIL_3114){	// SATALink BA5 is always enabled
				nChannels = 4;
				// BA5_WRITE_4(sc, 2, ba5_IDEDMA_CMD, IDEDMA_CMD_INT_STEER);
				pciConfigWrite16(SII3112_BA5_IND_ADDR, 0x200);
				pciConfigWrite32(SII3112_BA5_IND_DATA, IDEDMA_CMD_INT_STEER);
			} else if (scs_cmd & SCS_CMD_BA5_EN) {
				IOLog("SATALink BA5 register space enabled\n");
#if	0
				fProvider->setMemoryEnable(true);
#endif
			} else {
				IOLog("SATALink BA5 register space disabled\n");
				
				UInt8 cfgctl = fProvider->configRead8(SII3112_PCI_CFGCTL);
				fProvider->configWrite8(SII3112_PCI_CFGCTL, cfgctl | CFGCTL_BA5INDEN);
			}
		}

		
	} else if(vendor == PCI_ITE_ID){
		// reset local CPU
		fProvider->configWrite8(0x5e, 0x01);
		/* Disable RAID/CPU firmware mode */
		fProvider->configWrite8(IT_MODE, 0);
		fProvider->configWrite16(IT_CFG, 0xa0f3);	// IT_CFG_IORDY
		fProvider->configWrite32(0x4c, 0x02040204);
		fProvider->configWrite8(0x42, 0x36);
		//fProvider->configWrite8(kIOPCIConfigLatencyTimer, 0);
	} else if(vendor == PCI_VIA_ID){
		if(device == PCI_VIA_6421){
			nChannels = 3;
		}
		/* make sure SATA channels are enabled */
		UInt8 tmp8;
		tmp8 = fProvider->configRead8(VIA_IDE_ENABLE);
		if ((tmp8 & 0x03) != 0x03) {
			tmp8 |= 0x03;
			fProvider->configWrite8(VIA_IDE_ENABLE,tmp8);
		}
		
		/* make sure interrupts for each channel sent to us */
		tmp8 = fProvider->configRead8(VIA_IDE_CONFIG);
		tmp8 |= 0x03;
		// Turn on prefetch and post write buffers for both primary/secondary channels.
		tmp8 |= 0xf0;
		fProvider->configWrite8(VIA_IDE_CONFIG,tmp8);
		
		/* make sure native mode is enabled */
		tmp8 = fProvider->configRead8(VIA_SATA_NATIVE);
		if ((tmp8 & 0xf0) != 0xf0) {
			tmp8 |= 0xf0;
			fProvider->configWrite8(VIA_SATA_NATIVE,tmp8);
		}
	}
	return true;
}

bool ApplePCIIDERoot::start( IOService * provider )
{
    if (IOService::start(provider) != true)
        return false;
	
    fProvider = OSDynamicCast( IOPCIDevice, provider );
    if (fProvider == 0)
        return false;
	
    fProvider->retain();
	
    // Enable bus master.
	
    fProvider->setBusMasterEnable( true );
	
	nChannels = 2;
	do {
		// Allocate a mutex to serialize access to PCI config space between
		// the primary and secondary ATA channels.
		fPCILock = IOLockAlloc();
		if (fPCILock == 0)
			break;

		if(!initHardware())
			break;

		fChannels = createATAChannels();
		if (fChannels == 0)
			break;

		fOpenChannels = OSSet::withCapacity( fChannels->getCount() );
		if (fOpenChannels == 0)
			break;

		applyToClients( registerClientApplier, 0 );
		
		return true;

	} while(false);

	// Failure
	fProvider->release();
	return false;
}

//---------------------------------------------------------------------------
//
// Release allocated resources before this object is destroyed.
//

void ApplePCIIDERoot::free( void )
{
    if (fChannels)
    {
        fChannels->release();
        fChannels = 0;
    }
	
    if (fOpenChannels)
    {
        fOpenChannels->release();
        fOpenChannels = 0;
    }

    if (fProvider)
    {
        fProvider->release();
        fProvider = 0;
    }
	
    if (fPCILock)
    {
        IOLockFree( fPCILock );
        fPCILock = 0;
    }
	
    IOService::free();
}


//---------------------------------------------------------------------------
//
// Locate an entry in the device tree that correspond to the channels
// behind the ATA controller. This allows discovery of the ACPI entry
// for ACPI method evaluation, and also uses the ACPI assigned device
// name for a persistent path to the root device.
//

IORegistryEntry * ApplePCIIDERoot::getDTChannelEntry( int channelID )
{
    IORegistryEntry * entry = 0;
    const char *      location;
	
    OSIterator * iter = fProvider->getChildIterator( gIODTPlane );
    if (iter == 0) return 0;
	
    while (( entry = (IORegistryEntry *) iter->getNextObject() ))
    {
        location = entry->getLocation();
        if ( location && strtol(location, 0, 10) == channelID )
        {
            entry->retain();
            break;
        }
    }
	
    iter->release();
    
    return entry;  // retain held on the entry
}

//---------------------------------------------------------------------------
//
// Create nubs based on the channel information in the driver personality.
//

OSSet * ApplePCIIDERoot::createATAChannels( void )
{
    OSSet *           nubSet;
    OSDictionary *    channelInfo;
    IORegistryEntry * dtEntry;
	
    do {
        nubSet = OSSet::withCapacity(nChannels);
        if (nubSet == 0)
            break;

        if (fProvider->open(this) != true)
            break;
		
        for ( UInt32 channelID = 0; channelID < nChannels; channelID++ )
        {        
            // Create a dictionary for the channel info. Use native mode
            // settings if possible, else default to legacy mode.
			
            channelInfo = createNativeModeChannelInfo( channelID );

            if (channelInfo == 0)
                continue;

            // Create a nub for each ATA channel.
            ApplePCIIDEChannel * nub = new ApplePCIIDEChannel;
            if ( nub )
            {
                dtEntry = getDTChannelEntry( channelID );
				
                // Invoke special init method in channel nub.
				
                if (nub->init( this, channelInfo, dtEntry ) &&
                    nub->attach( this ))
                {
                    nubSet->setObject( nub );
                }
				
                if ( dtEntry )
                {
                    dtEntry->release();
                }
                else
                {
                    // Platform did not create a device tree entry for
                    // this ATA channel. Do it here.
					
                    char channelName[5] = {'C','H','N','_','\0'};
					
                    channelName[3] = '0' + channelID;
                    nub->setName( channelName );
					
                    if (fProvider->inPlane(gIODTPlane))
                    {
                        nub->attachToParent( fProvider, gIODTPlane );
                    }
                }
				
                nub->release();
            }
			
            channelInfo->release();
        }
        
        fProvider->close( this );
    }
    while ( false );
	
    // Release and invalidate an empty set.

    if (nubSet && (nubSet->getCount() == 0))
    {
        nubSet->release();
        nubSet = 0;
    }
	
    return nubSet;
}

//---------------------------------------------------------------------------

OSDictionary *
ApplePCIIDERoot::createNativeModeChannelInfo( UInt32 ataChannel )
{
    UInt16 cmdPort, ctrPort;
	
    // Force native mode configuration.
	UInt16 vendor = fProvider->configRead16(kIOPCIConfigVendorID);
	UInt16 device = fProvider->configRead16(kIOPCIConfigDeviceID);

	if( vendor == PCI_VIA_ID && device == PCI_VIA_6421 ){
		cmdPort = fProvider->configRead16( kIOPCIConfigBaseAddress0 + ataChannel * 4 );
		ctrPort = cmdPort + 8;
	} else {
		cmdPort = fProvider->configRead16( kIOPCIConfigBaseAddress0 + ataChannel * 8 );
		ctrPort = fProvider->configRead16( kIOPCIConfigBaseAddress1 + ataChannel * 8 );
	}
	cmdPort &= ~0x1;  // clear PCI I/O space indicator bit
	ctrPort &= ~0x1;

	// Programming interface byte indicate that native mode
	// is supported and active, but the controller has been
	// assigned legacy ranges. Force legacy mode configuration
	// which is safest. PCI INT# interrupts are not wired
	// properly for some machines in this state.
	
	if ( cmdPort == PRI_CMD_ADDR && ctrPort == PRI_CTR_ADDR )
	{
		cmdPort = ctrPort = 0;
	}
	
    if (cmdPort && ctrPort)
        return createChannelInfo( ataChannel, cmdPort, ctrPort,
								 fProvider->configRead8(kIOPCIConfigInterruptLine) );
    return 0;
}


//---------------------------------------------------------------------------

OSDictionary *
ApplePCIIDERoot::createChannelInfo( UInt32 ataChannel,
								   UInt16 commandPort,
								   UInt16 controlPort,
								   UInt8 interruptVector )
{
    OSDictionary * dict = OSDictionary::withCapacity( 4 );
    OSNumber *     num;
	
    if ( dict == 0 || commandPort == 0 || controlPort == 0 || 
		interruptVector == 0 || interruptVector == 0xFF )
    {
        if ( dict ) dict->release();
        return 0;
    }
	
    num = OSNumber::withNumber( ataChannel, 32 );
    if (num)
    {
        dict->setObject( kChannelNumberKey, num );
        num->release();
    }
    
    num = OSNumber::withNumber( commandPort, 16 );
    if (num)
    {
        dict->setObject( kCommandBlockAddressKey, num );
        num->release();
    }
	
    num = OSNumber::withNumber( controlPort, 16 );
    if (num)
    {
        dict->setObject( kControlBlockAddressKey, num );
        num->release();
    }
	
    num = OSNumber::withNumber( interruptVector, 32 );
    if (num)
    {
        dict->setObject( kInterruptVectorKey, num );
        num->release();
    }
	
    return dict;
}

//---------------------------------------------------------------------------
//
// Handle an open request from a client. Multiple channel nubs can hold
// an open on the root driver.
//

bool ApplePCIIDERoot::handleOpen( IOService *  client,
								 IOOptionBits options,
								 void *       arg )
{
    bool ret = true;
	
    // Reject open request from unknown clients, or if the client
    // already holds an open.
	
    if ((fChannels->containsObject(client) == false) ||
        (fOpenChannels->containsObject(client) == true))
        return false;
	
    // First client open will trigger an open to our provider.
	
    if (fOpenChannels->getCount() == 0)
        ret = fProvider->open(this);
	
    if (ret == true)
    {
        fOpenChannels->setObject(client);
		
        // Return the PCI device to the client
        if ( arg ) *((IOService **) arg) = fProvider;
    }
	
    return ret;
}

//---------------------------------------------------------------------------
//
// Handle a close request from a client.
//

void ApplePCIIDERoot::handleClose( IOService *  client,
								  IOOptionBits options )
{
    // Reject close request from clients that do not hold an open.
	
    if (fOpenChannels->containsObject(client) == false) return;
	
    fOpenChannels->removeObject(client);
	
    // Last client close will trigger a close to our provider.
	
    if (fOpenChannels->getCount() == 0)
        fProvider->close(this);
}

//---------------------------------------------------------------------------
//
// Report if the specified client (or any client) has an open on us.
//

bool ApplePCIIDERoot::handleIsOpen( const IOService * client ) const
{
    if (client)
        return fOpenChannels->containsObject(client);
    else
        return (fOpenChannels->getCount() != 0);
}

//---------------------------------------------------------------------------


UInt32 ApplePCIIDERoot::getHardwareFlags( void ) const
{
    return 0;
}

UInt32 ApplePCIIDERoot::getUltraDMAModeMask( void ) const
{
	return 0x7F;
}

//---------------------------------------------------------------------------

void ApplePCIIDERoot::pciConfigWrite8( UInt8 offset, UInt8 data, UInt8 mask )
{
    UInt8 u8;
	
    IOLockLock( fPCILock );
	
    u8 = fProvider->configRead8( offset );
    u8 &= ~mask;
    u8 |= (mask & data);
    fProvider->configWrite8( offset, u8 );
	
    IOLockUnlock( fPCILock );
}

void ApplePCIIDERoot::pciConfigWrite16( UInt8 offset, UInt16 data, UInt16 mask )
{
    UInt16 u16;
	
    IOLockLock( fPCILock );
	
    u16 = fProvider->configRead16( offset );
    u16 &= ~mask;
    u16 |= (mask & data);
    fProvider->configWrite16( offset, u16 );
	
    IOLockUnlock( fPCILock );
}

void ApplePCIIDERoot::pciConfigWrite32( UInt8 offset, UInt32 data, UInt32 mask )
{
    UInt8 u32;
	
    IOLockLock( fPCILock );
	
    u32 = fProvider->configRead32( offset );
    u32 &= ~mask;
    u32 |= (mask & data);
    fProvider->configWrite32( offset, u32 );
	
    IOLockUnlock( fPCILock );
}

UInt8 ApplePCIIDERoot::pciConfigRead8( UInt8 offset )
{
    return fProvider->configRead8( offset );
}

UInt16 ApplePCIIDERoot::pciConfigRead16( UInt8 offset )
{
    return fProvider->configRead16( offset );
}

UInt32 ApplePCIIDERoot::pciConfigRead32( UInt8 offset )
{
    return fProvider->configRead32( offset );
}

OSDefineMetaClassAndStructors( ApplePCIIDEChannel, IOService )

//---------------------------------------------------------------------------

bool ApplePCIIDEChannel::mergeProperties( OSDictionary * properties )
{
    bool success = true;
    OSCollectionIterator * propIter =
	OSCollectionIterator::withCollection( properties );
	
    if ( propIter )
    {
        const OSSymbol * propKey;
        while ((propKey = (const OSSymbol *) propIter->getNextObject()))
        {
            if (setProperty(propKey, properties->getObject(propKey)) == false)
            {
                success = false;
                break;
            }
        }
        propIter->release();
    }
    return success;
}

//---------------------------------------------------------------------------
//
// Initialize the ATA channel nub.
//

bool ApplePCIIDEChannel::init( IOService *       provider,
							  OSDictionary *    properties,
							  IORegistryEntry * dtEntry )
{
    if ( dtEntry )
    {
        if ( IOService::init( dtEntry, gIODTPlane ) == false ||
			mergeProperties( properties ) == false )
			return false;
    }
    else
    {
        if ( IOService::init( properties ) == false )
			return false;
    }
	
    fProvider = OSDynamicCast(ApplePCIIDERoot, provider);
    if (fProvider == 0)
        return false;
	
    // Call platform to register the interrupt assigned to each ATA
    // channel. For PCI interrupts (native mode), each channel will
    // share the same interrupt vector assigned to the PCI device.
    // Legacy mode channels will attempt to register IRQ 14 and 15.
	
    UInt32 vector = getInterruptVector();
    if (provider->callPlatformFunction( "SetDeviceInterrupts",
									   /* waitForFunction */ false,
									   /* nub             */ this,
									   /* vectors         */ (void *) &vector,
									   /* vectorCount     */ (void *) 1,
									   /* exclusive       */ (void *) false )
		!= kIOReturnSuccess)
    {
        return false;
    }
	
	char temp[] = {'0',0};
	temp[0] = '0' + getChannelNumber();
    setLocation( temp );
	
    return true;
}

//---------------------------------------------------------------------------
//
// Handle open and close from our exclusive client.
//

bool ApplePCIIDEChannel::handleOpen( IOService *  client,
									IOOptionBits options,
									void *       arg )
{
    bool ret = false;
	
    if ( fProvider && fProvider->open( this, options, arg ) )
    {
        ret = IOService::handleOpen( client, options, arg );
        if ( ret == false )
            fProvider->close( this );
    }
    
    return ret;
}

void ApplePCIIDEChannel::handleClose( IOService *  client,
									 IOOptionBits options )
{
    IOService::handleClose( client, options );
    if ( fProvider ) fProvider->close( this );
}

//---------------------------------------------------------------------------

UInt16 ApplePCIIDEChannel::getCommandBlockAddress( void ) const
{
    UInt16 value = 0xFFF0;
    getNumberValue( kCommandBlockAddressKey, &value, 16 );
    return value;
}

UInt16 ApplePCIIDEChannel::getControlBlockAddress( void ) const
{
    UInt16 value = 0xFFF0;
    getNumberValue( kControlBlockAddressKey, &value, 16 );
    return value;
}

UInt32 ApplePCIIDEChannel::getInterruptVector( void ) const
{
    UInt32 value = 0xFF;
    getNumberValue( kInterruptVectorKey, &value, 32 );
    return value;
}

UInt32 ApplePCIIDEChannel::getChannelNumber( void ) const
{
    UInt32 value = 0xFF;
    getNumberValue( kChannelNumberKey, &value, 32 );
    return value;
}

UInt32 ApplePCIIDEChannel::getUltraDMAModeMask( void ) const
{
    return fProvider->getUltraDMAModeMask();
}


UInt32 ApplePCIIDEChannel::getHardwareFlags( void ) const
{
    return fProvider->getHardwareFlags();
}

const char * ApplePCIIDEChannel::getHardwareName( void ) const
{
    OSString * name;
	
    name = OSDynamicCast(OSString,
                         fProvider->getProperty(kHardwareNameKey));
    if (name)
        return name->getCStringNoCopy();
    else
        return "UNKNOWN";
}

void ApplePCIIDEChannel::pciConfigWrite8( UInt8 offset,
										 UInt8 data,
										 UInt8 mask )
{
    fProvider->pciConfigWrite8( offset, data, mask );
}

void ApplePCIIDEChannel::pciConfigWrite16( UInt8 offset,
										  UInt16 data,
										  UInt16 mask )
{
    fProvider->pciConfigWrite16( offset, data, mask );
}

void ApplePCIIDEChannel::pciConfigWrite32( UInt8  offset,
										  UInt32 data,
										  UInt32 mask )
{
    fProvider->pciConfigWrite32( offset, data, mask );
}

UInt8 ApplePCIIDEChannel::pciConfigRead8( UInt8 offset )
{
    return fProvider->pciConfigRead8( offset );
}

UInt16 ApplePCIIDEChannel::pciConfigRead16( UInt8 offset )
{
    return fProvider->pciConfigRead16( offset );
}

UInt32 ApplePCIIDEChannel::pciConfigRead32( UInt8 offset )
{
    return fProvider->pciConfigRead32( offset );
}

//---------------------------------------------------------------------------

bool ApplePCIIDEChannel::getNumberValue( const char * propKey,
										void       * outValue,
										UInt32       outBits ) const
{
    OSNumber * num = OSDynamicCast( OSNumber, getProperty( propKey ) );
    bool   success = false;
	
    if ( num )
    {
        success = true;
		
        switch ( outBits )
        {
			case 64:
				*(UInt64 *) outValue = num->unsigned32BitValue();
                break;
				
            case 32:
                *(UInt32 *) outValue = num->unsigned32BitValue();
                break;
				
            case 16:
                *(UInt16 *) outValue = num->unsigned16BitValue();
                break;
				
            case 8:
                *(UInt8 *) outValue = num->unsigned8BitValue();
                break;
				
            default:
                success = false;
                break;
        }
    }
    return success;
}

/************************************************************
 * Real driver part.
 ************************************************************/
OSDefineMetaClassAndStructors( ApplePCIIDEDriver, IOPCIATA )

#if     0
#define DEBUG_LOG(fmt, args...) kprintf(fmt, ## args)
#define ERROR_LOG(fmt, args...) kprintf(fmt, ## args)
#else
#define DEBUG_LOG(fmt, args...)
#define ERROR_LOG(fmt, args...) IOLog(fmt, ## args)
#endif

#if	0
#define FEDE_LOG(fmt,args...) IOLog(fmt, ## args); kprintf(fmt, ## args);
#define FEDE_RELOG(fmt,args...) IOLog(fmt, ## args); kprintf(fmt, ## args);
#else
#define FEDE_LOG(fmt,args...) 
#define FEDE_RELOG(fmt,args...)
#endif



#define kPIOModeMask   ((1 << kPIOModeCount) - 1)
#define kDMAModeMask   ((1 << kDMAModeCount) - 1)
#define kUDMAModeMask  (fProvider->getUltraDMAModeMask())

#define DRIVE_IS_PRESENT(u) \
(_devInfo[u].type != kUnknownATADeviceType)

#define TIMING_PARAM_IS_VALID(p) \
((p) != 0)

// Increase the PRD table size to one full page or 4096 descriptors for
// large transfers via DMA.  2048 are required for 1 megabyte transfers
// assuming no fragmentation and no alignment issues on the buffer.  We
// allocate twice that since there are more issues than simple alignment
// for this DMA engine.

#define kATAXferDMADesc  512
#define kATAMaxDMADesc   kATAXferDMADesc

// up to 2048 ATA sectors per transfer

#define kMaxATAXfer      512 * 2048

/*---------------------------------------------------------------------------
 *
 * Start the single-channel ATA controller driver.
 *
 ---------------------------------------------------------------------------*/

bool ApplePCIIDEDriver::start( IOService * provider )
{
	FEDE_LOG("FEDE - %s: %s( %p, %p )\n", getName(), __FUNCTION__, this, provider);
    bool superStarted = false;
	
    DEBUG_LOG("%s: %s( %p, %p )\n", getName(), __FUNCTION__, this, provider);
	
    // Our provider is a 'nub' that represents a single channel PCI ATA
    // controller, and not an IOPCIDevice.
	
    fProvider = OSDynamicCast( ApplePCIIDEChannel, provider );
    if ( fProvider == 0 )
        goto fail;
	
    // Retain and open our provider.
	
    fProvider->retain();
	
    if ( fProvider->open( this ) != true )
    {
        DEBUG_LOG("%s: provider open failed\n", getName());
        goto fail;
    }
	
    // Create a work loop.
	
    fWorkLoop = IOWorkLoop::workLoop();
    if ( fWorkLoop == 0 )
    {
        DEBUG_LOG("%s: new work loop failed\n", getName());
        goto fail;
    }
	
    // Cache static controller properties.
	
    fChannelNumber = fProvider->getChannelNumber();
	

    // Probe for 80-pin conductors on drive 0 and 1.
	f80PinCable = true;
	
    // Get the base address for the bus master registers in I/O space.
	
    if ( getBMBaseAddress( fChannelNumber, &fBMBaseAddr ) != true )
    {
        DEBUG_LOG("%s: invalid bus-master base address\n", getName());
        goto fail;
    }
	
	// Override P-ATA reporting in IOATAController::start()
    // for SystemProfiler.
	{
		UInt16 vendor = fProvider->pciConfigRead16(kIOPCIConfigVendorID);
		UInt16 device = fProvider->pciConfigRead16(kIOPCIConfigDeviceID);
		if(vendor == PCI_SIL_ID && isSataLink(device) && fChannelNumber >= 2){
			fBMBaseAddr += 0x200;
		}
		if (vendor == PCI_SIL_ID && isSataLink(device) ||
			vendor == PCI_VIA_ID && device == PCI_VIA_6421 && fChannelNumber < 2)
		{
			setProperty( kIOPropertyPhysicalInterconnectTypeKey,
						kIOPropertyPhysicalInterconnectTypeSerialATA );
		}
	}
	

    // Must setup these variables inherited from IOPCIATA before it is started.
#if defined(__i386__) || defined(__x86_64__)
	_bmCommandReg   = IOATAIOReg8::withAddress( fBMBaseAddr + BM_COMMAND );
	_bmStatusReg    = IOATAIOReg8::withAddress( fBMBaseAddr + BM_STATUS );
	_bmPRDAddresReg = IOATAIOReg32::withAddress( fBMBaseAddr + BM_PRD_TABLE );
	
#else
	_bmCommandReg = (UInt8*)fBMBaseAddr + BM_COMMAND;
	_bmStatusReg = (UInt8*)fBMBaseAddr + BM_STATUS;
	_bmPRDAddresReg = (volatile UInt32 *)((UInt8*)fBMBaseAddr + BM_PRD_TABLE);
#endif
    // Reset bus timings for both drives.
	
    initializeHardware();
    resetBusTimings();
	
	
    // Now we are ready to call super::start
	
    if ( IOPCIATA::start(_provider) == false )
    {
        goto fail;
    }
    superStarted = true;
	
    // This driver will handle interrupts using a work loop.
    // Create interrupt event source that will signal the
    // work loop (thread) when a device interrupt occurs.
	
    if ( fProvider->getInterruptVector() == 14 ||
		fProvider->getInterruptVector() == 15 )
    {
        // Legacy IRQ are never shared, no need for an interrupt filter.
		
        fInterruptSource = IOInterruptEventSource::interruptEventSource(
																		this, &interruptOccurred,
																		fProvider, 0 );
    }
    else
    {
        fInterruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(
																					this, &interruptOccurred, &interruptFilter,
																					fProvider, 0 );
    }
	
    if ( !fInterruptSource ||
		(fWorkLoop->addEventSource(fInterruptSource) != kIOReturnSuccess) )
    {
        DEBUG_LOG("%s: interrupt registration error\n", getName());
        goto fail;
    }
    fInterruptSource->enable();
	
    // Attach to power management.
	
    initForPM( provider );
	
    // For each device discovered on the ATA bus (by super),
    // create a nub for that device and call registerService() to
    // trigger matching against that device.
	
    for ( UInt32 i = 0; i < kMaxDriveCount; i++ )
    {
        if ( _devInfo[i].type != kUnknownATADeviceType )
        {
            ATADeviceNub * nub;
			
            nub = ATADeviceNub::ataDeviceNub( (IOATAController*) this,
											 (ataUnitID) i,
											 _devInfo[i].type );
			
            if ( nub )
            {
                if ( _devInfo[i].type == kATAPIDeviceType )
                {
                    nub->setProperty( kIOMaximumSegmentCountReadKey,
									 kATAMaxDMADesc / 2, 64 );
					
                    nub->setProperty( kIOMaximumSegmentCountWriteKey,
									 kATAMaxDMADesc / 2, 64 );
					
                    nub->setProperty( kIOMaximumSegmentByteCountReadKey,
									 0x10000, 64 );
					
                    nub->setProperty( kIOMaximumSegmentByteCountWriteKey,
									 0x10000, 64 );
                }
				
                if ( nub->attach( this ) )
                {
                    _nub[i] = (IOATADevice *) nub;
                    _nub[i]->retain();
                    _nub[i]->registerService();
                }
                nub->release();
            }
        }
    }
	
    // Successful start, announce useful properties.
	
    IOLog("%s: IDE %s (CMD 0x%x, CTR 0x%x, IRQ %u, BM 0x%x)\n", getName(),
          fProvider->getHardwareName(),
          fProvider->getCommandBlockAddress(),
          fProvider->getControlBlockAddress(),
          (unsigned int)fProvider->getInterruptVector(),
          fBMBaseAddr);
	
    return true;
	
fail:
    if ( fProvider )
        fProvider->close( this );
	
    if ( superStarted )
        IOPCIATA::stop( provider );
	
    return false;
}

/*---------------------------------------------------------------------------
 *
 * Stop the single-channel VIA ATA controller driver.
 *
 ---------------------------------------------------------------------------*/

void ApplePCIIDEDriver::stop( IOService * provider )
{
	FEDE_LOG("FEDE - %s: %s( %p, %p )\n", getName(), __FUNCTION__, this, provider);
    PMstop();
    IOPCIATA::stop( provider );
}

/*---------------------------------------------------------------------------
 *
 * Release resources before this driver is destroyed.
 *
 ---------------------------------------------------------------------------*/

void ApplePCIIDEDriver::free( void )
{
	FEDE_LOG("FEDE - %s: %s( %p )\n", getName(), __FUNCTION__, this);
#define RELEASE(x) do { if(x) { (x)->release(); (x) = 0; } } while(0)
	
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);
	
    // Release resources created by start().
	
    if (fInterruptSource && fWorkLoop)
    {
        fWorkLoop->removeEventSource(fInterruptSource);
    }
	
    RELEASE( fProvider        );
    RELEASE( fInterruptSource );
    RELEASE( fWorkLoop        );
    RELEASE( _nub[0]          );
    RELEASE( _nub[1]          );
    RELEASE( _bmCommandReg    );
    RELEASE( _bmStatusReg     );
    RELEASE( _bmPRDAddresReg  );
	
    // Release registers created by configureTFPointers().
	
    RELEASE( _tfDataReg       );
    RELEASE( _tfFeatureReg    );
    RELEASE( _tfSCountReg     );
    RELEASE( _tfSectorNReg    );
    RELEASE( _tfCylLoReg      );
    RELEASE( _tfCylHiReg      );
    RELEASE( _tfSDHReg        );
    RELEASE( _tfStatusCmdReg  );
    RELEASE( _tfAltSDevCReg   );
	
    // IOATAController should release this.
	
    if ( _doubleBuffer.logicalBuffer )
    {
        IOFree( (void *) _doubleBuffer.logicalBuffer,
			   _doubleBuffer.bufferSize );
        _doubleBuffer.bufferSize     = 0;
        _doubleBuffer.logicalBuffer  = 0;
        _doubleBuffer.physicalBuffer = 0;
    }
	
    // What about _cmdGate, and _timer in the superclass?
	
    IOPCIATA::free();
}

/*---------------------------------------------------------------------------
 *
 * Return the driver's work loop
 *
 ---------------------------------------------------------------------------*/

IOWorkLoop * ApplePCIIDEDriver::getWorkLoop( void ) const
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    return fWorkLoop;
}

/*---------------------------------------------------------------------------
 *
 * Override IOATAController::synchronousIO()
 *
 ---------------------------------------------------------------------------*/

IOReturn ApplePCIIDEDriver::synchronousIO( void )
{
	FEDE_LOG("FEDE - %s: %s( %p )\n", getName(), __FUNCTION__, this);
    IOReturn ret;
    
    // IOATAController::synchronousIO() asserts nIEN bit in order to disable
    // drive interrupts during polled mode command execution. The problem is
    // that this will float the INTRQ line and put it in high impedance state,
    // which on certain systems has the undesirable effect of latching a false
    // interrupt on the interrupt controller. Perhaps those systems lack a
    // strong pull down resistor on the INTRQ line. Experiment shows that the
    // interrupt event source is signalled, and its producerCount incremented
    // after every synchronousIO() call. This false interrupt can become
    // catastrophic after reverting to async operations since software can
    // issue a command, handle the false interrupt, and issue another command
    // to the drive before the actual completion of the first command, leading
    // to a irrecoverable bus hang. This function is called after an ATA bus
    // reset. Waking from system sleep will exercise this path.
    // The workaround is to mask the interrupt line while the INTRQ line is
    // floating (or bouncing).
	
    if (fInterruptSource) fInterruptSource->disable();
    ret = IOPCIATA::synchronousIO();
    if (fInterruptSource) fInterruptSource->enable();
	
    return ret;
}

/*---------------------------------------------------------------------------
 *
 * Determine the start of the I/O mapped Bus-Master registers.
 *
 ---------------------------------------------------------------------------*/

bool ApplePCIIDEDriver::getBMBaseAddress( UInt32  channel,
										 UInt16 * baseAddr )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    UInt32 bmiba;
	
    DEBUG_LOG("%s::%s( %p, %u, %p )\n", getName(), __FUNCTION__,
              this, (unsigned int)channel, baseAddr);
	
    bmiba = fProvider->pciConfigRead32( PCI_BMIBA );
	
    if ((bmiba & PCI_BMIBA_RTE) == 0)
    {
        DEBUG_LOG("%s: PCI BAR 0x%02x (0x%08x) is not an I/O range\n",
                  getName(), PCI_BMIBA, (unsigned int)bmiba);
        return false;
    }
	
    bmiba &= PCI_BMIBA_MASK;  // get the address portion
    if (bmiba == 0)
    {
        DEBUG_LOG("%s: BMIBA is zero\n", getName());
        return false;
    }
	
    // if (channel == SEC_CHANNEL_ID)
    bmiba += BM_SEC_OFFSET * channel;
	
    *baseAddr = (UInt16) bmiba;
    DEBUG_LOG("%s: BMBaseAddr = %04x\n", getName(), *baseAddr);
	
    return true;
}

/*---------------------------------------------------------------------------
 *
 * Reset all timing registers to the slowest (most compatible) timing.
 * DMA modes are disabled.
 *
 ---------------------------------------------------------------------------*/

void ApplePCIIDEDriver::resetBusTimings( void )
{
	
    programTimingRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Setup the location of the task file registers.
 *
 ---------------------------------------------------------------------------*/

bool ApplePCIIDEDriver::configureTFPointers( void )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);
	
    UInt16 cmdBlockAddr = fProvider->getCommandBlockAddress();
    UInt16 ctrBlockAddr = fProvider->getControlBlockAddress();
	
    _tfDataReg      = IOATAIOReg16::withAddress( cmdBlockAddr + 0 );
    _tfFeatureReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 1 );
    _tfSCountReg    = IOATAIOReg8::withAddress(  cmdBlockAddr + 2 );
    _tfSectorNReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 3 );
    _tfCylLoReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 4 );
    _tfCylHiReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 5 );
    _tfSDHReg       = IOATAIOReg8::withAddress(  cmdBlockAddr + 6 );
    _tfStatusCmdReg = IOATAIOReg8::withAddress(  cmdBlockAddr + 7 );
    _tfAltSDevCReg  = IOATAIOReg8::withAddress(  ctrBlockAddr + 2 );
	
    if ( !_tfDataReg || !_tfFeatureReg || !_tfSCountReg ||
		!_tfSectorNReg || !_tfCylLoReg || !_tfCylHiReg ||
		!_tfSDHReg || !_tfStatusCmdReg || !_tfAltSDevCReg )
    {
        return false;
    }
	
    return true;
}

/*---------------------------------------------------------------------------
 *
 * Filter interrupts that are not originated by our hardware. This will help
 * prevent waking up our work loop thread when sharing a interrupt line with
 * another driver.
 *
 ---------------------------------------------------------------------------*/

bool ApplePCIIDEDriver::interruptFilter( OSObject * owner,
										IOFilterInterruptEventSource * src )
{
    ApplePCIIDEDriver * self = (ApplePCIIDEDriver *) owner;
	
    if ( *(self->_bmStatusReg) & BM_STATUS_INT )
        return true;   // signal the work loop
    else
        return false;  // ignore this spurious interrupt
}

/*---------------------------------------------------------------------------
 *
 * The work loop based interrupt handler called by our interrupt event
 * source.
 *
 ---------------------------------------------------------------------------*/

void ApplePCIIDEDriver::interruptOccurred( OSObject *               owner,
										  IOInterruptEventSource * source,
										  int                      count )
{
    ApplePCIIDEDriver * self = (ApplePCIIDEDriver *) owner;
	
    // Clear interrupt latch
	
    *(self->_bmStatusReg) = BM_STATUS_INT;
	
    // Let our superclass handle the interrupt to advance to the next state
    // in the state machine.
	
    self->handleDeviceInterrupt();
}

/*---------------------------------------------------------------------------
 *
 * Extend the implementation of scanForDrives() from IOATAController
 * to issue a soft reset before scanning for ATA/ATAPI drive signatures.
 *
 ---------------------------------------------------------------------------*/

UInt32 ApplePCIIDEDriver::scanForDrives( void )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    UInt32 unitsFound;
	
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);
	
    *_tfAltSDevCReg = mATADCRReset;
	
    IODelay( 100 );
	
    *_tfAltSDevCReg = 0x0;
	
    IOSleep( 10 );
	
    unitsFound = IOPCIATA::scanForDrives();

    *_tfSDHReg = 0x00;  // Initialize device selection to device 0.
	
	// FEDE_LOG("FEDE - %s: %s %d drives\n", getName(), __FUNCTION__, (int)unitsFound);
    return unitsFound;
}

/*---------------------------------------------------------------------------
 *
 * Provide information on the ATA bus capability.
 *
 ---------------------------------------------------------------------------*/

IOReturn ApplePCIIDEDriver::provideBusInfo( IOATABusInfo * infoOut )
{
	FEDE_LOG("FEDE - %s: %s( %p, %p )\n", getName(), __FUNCTION__, this, infoOut);
    DEBUG_LOG("%s::%s( %p, %p )\n", getName(), __FUNCTION__, this, infoOut);
	
    if ( infoOut == 0 )
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }
	
    infoOut->zeroData();
	
	infoOut->setSocketType( kInternalATASocket );
	
    infoOut->setPIOModes( kPIOModeMask );
    infoOut->setDMAModes( kDMAModeMask );
    infoOut->setUltraModes( kUDMAModeMask );
    infoOut->setExtendedLBA( true );
    infoOut->setMaxBlocksExtended( 0x0800 );  // 2048 sectors for ext LBA
	
    UInt8 units = 0;
    if ( _devInfo[0].type != kUnknownATADeviceType ) units++;
    if ( _devInfo[1].type != kUnknownATADeviceType ) units++;
    infoOut->setUnits( units );
	
    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Returns the currently configured timings for the drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn ApplePCIIDEDriver::getConfig( IOATADevConfig * configOut,
									  UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %p, %d )\n", getName(), __FUNCTION__,
              this, configOut, (int)unit);
	
    if ((configOut == 0) || (unit > kATADevice1DeviceID))
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }
	
    configOut->setPIOMode( 1 << fBusTimings[unit].pioModeNumber );
    configOut->setDMAMode( 1 << fBusTimings[unit].dmaModeNumber );
    configOut->setUltraMode( 1 << fBusTimings[unit].ultraModeNumber );
	
	
    configOut->setPacketConfig( _devInfo[unit].packetSend );
	
    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Select the bus timings for a given drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn ApplePCIIDEDriver::selectConfig( IOATADevConfig * configRequest,
										 UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %p, %d )\n", getName(), __FUNCTION__,
              this, configRequest, (int)unit);
	
    if ((configRequest == 0) || (unit > kATADevice1DeviceID))
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }
	
    // All config requests must include a supported PIO mode
	
    if ((configRequest->getPIOMode() & kPIOModeMask) == 0)
    {
        DEBUG_LOG("%s: PIO mode unsupported\n", getName());
        return kATAModeNotSupported;
    }
	
    if (configRequest->getDMAMode() & ~kDMAModeMask)
    {
        DEBUG_LOG("%s: DMA mode unsupported (0x%x)\n",
                  getName(), configRequest->getDMAMode());
        return kATAModeNotSupported;
    }
	
    if (configRequest->getUltraMode() & ~kUDMAModeMask)
    {
        DEBUG_LOG("%s: UDMA mode unsupported (0x%x)\n",
                  getName(), configRequest->getUltraMode());
        return kATAModeNotSupported;
    }
	
    if (configRequest->getDMAMode() && configRequest->getUltraMode())
    {
        DEBUG_LOG("%s: multiple DMA mode selection error\n", getName());
        return kATAModeNotSupported;
    }
	
    _devInfo[unit].packetSend = configRequest->getPacketConfig();
	
    selectTimingParameter( configRequest, unit );
	
    return getConfig( configRequest, unit );
}

/*---------------------------------------------------------------------------
 *
 * Select timing parameters based on config request.
 *
 ---------------------------------------------------------------------------*/

void ApplePCIIDEDriver::selectTimingParameter( IOATADevConfig * configRequest,
											  UInt32           unit )
{
	FEDE_LOG("FEDE - %s: %s( %p, %d )\n", getName(), __FUNCTION__, this, (int)unit);
    DEBUG_LOG("%s::%s( %p, %d )\n", getName(), __FUNCTION__, this, (int)unit);
	
    // Reset existing parameters for this unit.
	
    fBusTimings[unit].ultraEnabled = false;
    fBusTimings[unit].dmaEnabled = false;
	
    if ( configRequest->getPIOMode() )
    {
        UInt32  pioModeNumber;
		
        pioModeNumber = bitSigToNumeric( configRequest->getPIOMode() );
        pioModeNumber = min(pioModeNumber, kPIOModeCount - 1);
		
        // Look for the fastest entry in the PIOTimingTable with a cycle time
        // which is larger than or equal to pioCycleTime.
		
        fBusTimings[unit].pioModeNumber = pioModeNumber;
        DEBUG_LOG("%s: selected PIO mode %d\n", getName(), (int)pioModeNumber);
        setDriveProperty(unit, kSelectedPIOModeKey, pioModeNumber, 8);
    }
#if	ITE_FORCE_PIO
	if( fProvider->pciConfigRead16(kIOPCIConfigVendorID) == PCI_ITE_ID ){
		programTimingRegisters();
		return;
	}
#endif
    if ( configRequest->getDMAMode() )
    {
        UInt32  dmaModeNumber;
		
        dmaModeNumber = bitSigToNumeric( configRequest->getDMAMode() );
        dmaModeNumber = min(dmaModeNumber, kDMAModeCount - 1);
		
		fBusTimings[unit].dmaEnabled = true;
        fBusTimings[unit].dmaModeNumber = dmaModeNumber;
        DEBUG_LOG("%s: selected DMA mode %d\n", getName(), (int)dmaModeNumber);
        setDriveProperty(unit, kSelectedDMAModeKey, dmaModeNumber, 8);
    }
	
    if ( configRequest->getUltraMode() )
    {
        UInt32  ultraModeNumber;
		
        ultraModeNumber = bitSigToNumeric( configRequest->getUltraMode() );
        ultraModeNumber = min(ultraModeNumber, kUDMAModeCount - 1);
		
        // For Ultra DMA mode 3 or higher, 80 pin cable must be present.
        // Otherwise, the drive will be limited to UDMA mode 2.
		
        if ( ultraModeNumber > 2 )
        {
            if ( f80PinCable == false )
            {
                DEBUG_LOG("%s: 80-conductor cable not detected\n", getName());
                ultraModeNumber = 2;
            }
        }
		
        fBusTimings[unit].ultraEnabled = true;
        fBusTimings[unit].ultraModeNumber = ultraModeNumber;
        DEBUG_LOG("%s: selected Ultra mode %d\n", getName(), (int)ultraModeNumber);
        setDriveProperty(unit, kSelectedUltraDMAModeKey, ultraModeNumber, 8);
    }
	
    programTimingRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Program timing registers for both drives.
 *
 ---------------------------------------------------------------------------*/
void ApplePCIIDEDriver::setITEUDMA( void ){
	static const UInt8 udma[][2] = { {0x44,0x33}, {0x42,0x31}, {0x31,0x21}, {0x21,0x21}, {0x11,0x11}, {0x22|0x80,0x11|0x80}, {0x11|0x80,0x11|0x80}};
	static const UInt8 udma_clk[] = { 0, 1, 0, 0, 0, 1, 0 }; // 1 = 50
	
	bool timing10 =(fProvider->pciConfigRead8(kIOPCIConfigRevisionID) == 0x10);
	
	for (int unit = 0; unit < 2; unit++)
	{
		if (DRIVE_IS_PRESENT(unit) == false)
			continue;

		if (fBusTimings[unit].ultraEnabled)
		{
			int t_index = udma_clk[fBusTimings[unit].ultraModeNumber];
			if(timing10){
				fProvider->pciConfigWrite8(IT_TIM_UDMA(fChannelNumber,0), udma[fBusTimings[unit].ultraModeNumber][t_index]);
				fProvider->pciConfigWrite8(IT_TIM_UDMA(fChannelNumber,1), udma[fBusTimings[unit].ultraModeNumber][t_index]);
			} else{
				fProvider->pciConfigWrite8(IT_TIM_UDMA(fChannelNumber,unit), udma[fBusTimings[unit].ultraModeNumber][t_index]);
			}
		}
	}
}

void ApplePCIIDEDriver::programTimingRegisters( void )
{
	UInt16 vendor = fProvider->pciConfigRead16(kIOPCIConfigVendorID);
	UInt16 device = fProvider->pciConfigRead16(kIOPCIConfigDeviceID);

	if(vendor == PCI_SIL_ID && device == PCI_SIL_0680){
        // We now have all the information need to program the registers.
		UInt8 mode = fProvider->pciConfigRead8(0x80 + fChannelNumber * 4);

		for (int unit = 0; unit < 2; unit++)
        {
            if (DRIVE_IS_PRESENT(unit) == false)
                continue;

			mode &= ~(0x03 << (unit * 4));
            if (fBusTimings[unit].ultraEnabled){
				static const UInt8 udma2_tbl[] = { 0x0f, 0x0b, 0x07, 0x06, 0x03, 0x02, 0x01 };
				static const UInt8 udma_tbl[] = { 0x0c, 0x07, 0x05, 0x04, 0x02, 0x01, 0x00 };
				
				UInt8 scsc = fProvider->pciConfigRead8(0x8a);
				if (fBusTimings[unit].ultraModeNumber > 2 && (scsc & 0x30) == 0) {
					fProvider->pciConfigWrite8(0x8a, scsc | 0x01);
					scsc = fProvider->pciConfigRead8(0x8a);
					if ((scsc & 0x30) == 0){	// UDMA100 = 5
						fBusTimings[unit].ultraModeNumber = 5;
					}
				}
				mode |= 0x03 << (unit * 4);
				UInt8 off = 0xac + fChannelNumber * 16 + unit * 2;
				UInt8 val = fProvider->pciConfigRead8(off) & ~0x3f;
				if (scsc & 0x30)
					val |= udma2_tbl[fBusTimings[unit].ultraModeNumber];
				else
					val |= udma_tbl[fBusTimings[unit].ultraModeNumber];
				fProvider->pciConfigWrite8(off, val);
            } else if(fBusTimings[unit].dmaEnabled){	// DMA
				static const UInt16 dma_tbl[] = { 0x2208, 0x10c2, 0x10c1 };
				mode |= 0x02 << (unit * 4);
				UInt8 off = 0xa8 + fChannelNumber * 16 + unit * 2;
				UInt16 val = dma_tbl[fBusTimings[unit].dmaModeNumber];
				fProvider->pciConfigWrite8(off, val & 0xff);
				fProvider->pciConfigWrite8(off+1, val >> 8);
            } else {
				static const UInt16 pio_tbl[] = { 0x328a, 0x2283, 0x1104, 0x10c3, 0x10c1 };
				mode |= 0x01 << (unit * 4);
				UInt8 off = 0xa4 + fChannelNumber * 16 + unit * 2;
				UInt16 val = pio_tbl[fBusTimings[unit].pioModeNumber];
				fProvider->pciConfigWrite8(off, val & 0xff);
				fProvider->pciConfigWrite8(off+1, val >> 8);
				
			}
        }
		fProvider->pciConfigWrite8(0x80 + fChannelNumber * 4, mode);
	} else if(vendor == PCI_SIL_ID && isSataLink(device)){
		UInt8 idedma_ctl = 0;
		UInt8 dtm = 0;
		for (int unit = 0; unit < 2; unit++)
        {
            if (DRIVE_IS_PRESENT(unit) == false)
                continue;
			
			idedma_ctl |= IDEDMA_CTL_DRV_DMA(unit);
			dtm |= DTM_IDEx_DMA;
        }
		if (idedma_ctl != 0) {
			/* Add software bits in status register */
			// bus_space_write_1(, cp->dma_iohs[IDEDMA_CTL], 0, idedma_ctl);
			UInt16 dtmadr[] = {0x0b4,0x0f4,0x2b4,0x2f4}; 
			fProvider->pciConfigWrite16(SII3112_BA5_IND_ADDR, dtmadr[fChannelNumber]);
			fProvider->pciConfigWrite8(SII3112_BA5_IND_DATA, dtm);
		}
	} else if(vendor == PCI_VIA_ID){	// 6421
		if(fChannelNumber == 2){	// PATA
			static const UInt8 pio_bits[] = { 0xa8, 0x65, 0x65, 0x31, 0x20 };
			static const UInt8 udma_bits[] = { 0xee, 0xe8, 0xe6, 0xe4, 0xe2, 0xe1, 0xe0, 0xe0 };
			for (int unit = 0; unit < 2; unit++)
			{
				if (DRIVE_IS_PRESENT(unit) == false)
					continue;
				fProvider->pciConfigWrite8(VIA_PATA_PIO_TIMING, pio_bits[fBusTimings[unit].pioModeNumber]);
				if(fBusTimings[0].ultraEnabled)
					fProvider->pciConfigWrite8(VIA_PATA_UDMA_TIMING, udma_bits[fBusTimings[unit].ultraModeNumber]);
			}
		}
	} else if(vendor == PCI_ITE_ID){	// 821x
		static const UInt8 udma_clk[] = { 0, 1, 0, 0, 0, 1, 0 }; // 1 = 50
		static const UInt8 dma[] = { 0x88, 0x32, 0x31 };
		static const UInt8 pio[][2] = { {0xaa,0x88}, {0xa3,0x82}, {0xa1,0x81}, {0x33,0x32}, {0x31,0x21} };

		
		setITEUDMA();

		bool timing10 =(fProvider->pciConfigRead8(kIOPCIConfigRevisionID) == 0x10);

		UInt8 mode = fProvider->pciConfigRead8(IT_MODE);
		UInt8 tim = 0;

		mode  &= ~IT_MODE_50MHZ(fChannelNumber);
		for (int unit = 0; unit < 2; unit++)
        {
            if (DRIVE_IS_PRESENT(unit) == false)
                continue;
			
			DEBUG_LOG("ITE channel = %d, unit = %d, ", (int)fChannelNumber, unit );
            if (fBusTimings[unit].ultraEnabled)
            {
				DEBUG_LOG("UDMA = %d, PIO = %d\n", (int)fBusTimings[unit].ultraModeNumber, (int)fBusTimings[unit].pioModeNumber );
				int t_index = udma_clk[fBusTimings[unit].ultraModeNumber];
				if(timing10){
					mode &= ~(IT_MODE_DMA(fChannelNumber, 0) | IT_MODE_DMA(fChannelNumber, 1));
				} else{
					mode &= ~IT_MODE_DMA(fChannelNumber, unit);
				}
				if(t_index){
					mode  |= IT_MODE_50MHZ(fChannelNumber);
				}
				tim = pio[fBusTimings[unit].pioModeNumber][t_index];
            } else if(fBusTimings[unit].dmaEnabled){
				DEBUG_LOG("MWDMA = %d\n", (int)fBusTimings[unit].dmaModeNumber );

				if(timing10){
					mode |= (IT_MODE_DMA(fChannelNumber, 0)|IT_MODE_DMA(fChannelNumber, 1));
				} else {
					mode |= IT_MODE_DMA(fChannelNumber, unit);
				}
				tim = dma[fBusTimings[unit].dmaModeNumber];
            } else {
				DEBUG_LOG("PIO = %d\n", (int)fBusTimings[unit].pioModeNumber );
				tim = pio[fBusTimings[unit].pioModeNumber][0];
			}
        }
		fProvider->pciConfigWrite8(IT_MODE, mode);
		fProvider->pciConfigWrite8(IT_TIM(fChannelNumber), tim);
	}
}


/*---------------------------------------------------------------------------
 *
 * Hardware initialization.
 *
 ---------------------------------------------------------------------------*/

void ApplePCIIDEDriver::initializeHardware( void )
{
	UInt16 vendor = fProvider->pciConfigRead16(kIOPCIConfigVendorID);
	UInt16 device = fProvider->pciConfigRead16(kIOPCIConfigDeviceID);
	
	
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);

	if(vendor == PCI_SIL_ID){	// SiliconImage
		if(device == PCI_SIL_0680){	// SiL0680
			f80PinCable = ((fProvider->pciConfigRead8(0xa0 + fChannelNumber * 16) & 0x1) != 0);	// from NetBSD cmdide.c
			static const u_int8_t init_val[] =
			{ 0x8a, 0x32, 0x8a, 0x32, 0x8a, 0x32, 0x92, 0x43, 0x92, 0x43, 0x09, 0x40, 0x09, 0x40 };
			UInt8 reg = 0xa2 + fChannelNumber * 16;
			for (int i = 0; i < sizeof(init_val); i++)
				fProvider->pciConfigWrite8(reg + i, init_val[i]);
		} else if(isSataLink(device)){
			UInt16 scs_cmd = fProvider->pciConfigRead16(SII3112_SCS_CMD);
			static const UInt16 scs_reset[] = {
				SCS_CMD_FF0_RESET | SCS_CMD_IDE0_RESET,
				SCS_CMD_FF1_RESET | SCS_CMD_IDE1_RESET,
				SCS_CMD_FF2_RESET | SCS_CMD_IDE2_RESET,
				SCS_CMD_FF3_RESET | SCS_CMD_IDE3_RESET
			};
			fProvider->pciConfigWrite16(SII3112_SCS_CMD,scs_reset[fChannelNumber] | scs_cmd);
			IODelay(50 * 1000);
			fProvider->pciConfigWrite16(SII3112_SCS_CMD,scs_cmd & SCS_CMD_BA5_EN);
			IODelay(50 * 1000);
		}
	} else if(vendor == PCI_VIA_ID){	// VIA 6141
		f80PinCable = ((fProvider->pciConfigRead8(VIA_PATA_UDMA_TIMING) & 0x10) == 0);
	}
}

/*---------------------------------------------------------------------------
 *
 * Dynamically select the bus timings for a drive unit.
 *
 ---------------------------------------------------------------------------*/

void ApplePCIIDEDriver::selectIOTiming( ataUnitID unit )
{
    /* Timings was already applied by selectConfig() */
}

/*---------------------------------------------------------------------------
 *
 * Flush the outstanding commands in the command queue.
 * Implementation borrowed from MacIOATA in IOATAFamily.
 *
 ---------------------------------------------------------------------------*/

IOReturn ApplePCIIDEDriver::handleQueueFlush( void )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    UInt32 savedQstate = _queueState;
	
    DEBUG_LOG("%s::%s()\n", getName(), __FUNCTION__);
	
    _queueState = IOATAController::kQueueLocked;
	
    IOATABusCommand * cmdPtr = 0;
	
    while ( cmdPtr = dequeueFirstCommand() )
    {
        cmdPtr->setResult( kIOReturnError );
        cmdPtr->executeCallback();
    }
	
    _queueState = savedQstate;
	
    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Handle termination notification from the provider.
 *
 ---------------------------------------------------------------------------*/

IOReturn ApplePCIIDEDriver::message( UInt32      type,
									IOService * provider,
									void *      argument )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    if ( ( provider == fProvider ) &&
		( type == kIOMessageServiceIsTerminated ) )
    {
        fProvider->close( this );
        return kIOReturnSuccess;
    }
	
    return IOPCIATA::message( type, provider, argument );
}

/*---------------------------------------------------------------------------
 *
 * Publish a numeric property pertaining to a drive to the registry.
 *
 ---------------------------------------------------------------------------*/

bool ApplePCIIDEDriver::setDriveProperty( UInt32       driveUnit,
										 const char * key,
										 UInt32       value,
										 UInt32       numberOfBits)
{
    char keyString[40];
    
    snprintf(keyString, 40, "Drive %u %s", (unsigned int)driveUnit, key);
    
    return IOPCIATA::setProperty( keyString, value, numberOfBits );
}

//---------------------------------------------------------------------------

IOReturn ApplePCIIDEDriver::createChannelCommands( void )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
	FEDE_RELOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
	
	FEDE_RELOG("FEDE Casteo a IOAABUSCommand64\n");
	IOATABusCommand64* currentCommand64 = OSDynamicCast( IOATABusCommand64, _currentCommand );
	FEDE_RELOG("FEDE Obtengo el comaqndo DMA\n ");
	IODMACommand* currentDMACmd = currentCommand64->GetDMACommand();
    IODMACommand::Segment32 elSegmento;
	IOReturn DMAStatus = 0;
	
	
	if ( NULL == currentDMACmd
		|| currentDMACmd->getMemoryDescriptor() == NULL)
    {
        FEDE_RELOG("FEDE DMA BUFFER NOT SET ON COMMAND\n ");
		IOLog("%s: DMA buffer not set on command\n", getName());
		return -1;
    }
	
	
	FEDE_RELOG("FEDE obtengo el descriptor de memoria\n ");
    IOMemoryDescriptor* descriptor = _currentCommand->getBuffer();
    //IOMemoryCursor::PhysicalSegment physSegment;
    UInt32 index = 0;
    UInt8  *xferDataPtr, *ptr2EndData, *next64KBlock, *starting64KBlock;
    UInt32 xferCount, count2Next64KBlock;
    
    if ( !descriptor )
    {
        return -1;
    }
	
	
	// This form of DMA engine can only do 1 pass.
    // It cannot execute multiple chains.
	
	UInt32 numSegmentos = 1;
	
    IOByteCount bytesRemaining = _currentCommand->getByteCount() ;
    IOByteCount xfrPosition    = _currentCommand->getPosition() ;
    // UInt64  transferSize  = 0;
    UInt64  transferSize  = xfrPosition;
	
    // There's a unique problem with pci-style controllers, in that each
    // dma transaction is not allowed to cross a 64K boundary. This leaves
    // us with the yucky task of picking apart any descriptor segments that
    // cross such a boundary ourselves.  
	//
	//    while ( _DMACursor->getPhysicalSegments(
	//                           /* descriptor */ descriptor,
	//							/* position   */ xfrPosition,
	//                           /* segments   */ &physSegment,
	//                           /* max segs   */ 1,
	//                           /* max xfer   */ bytesRemaining,
	//                           /* xfer size  */ &transferSize) )
	//
	FEDE_RELOG("FEDE entro al ciclo\n");
	while ( bytesRemaining )
	{
		FEDE_RELOG("FEDE genero 32IOVMSegments\n");
		DMAStatus = currentDMACmd->gen32IOVMSegments( &transferSize, &elSegmento, &numSegmentos);
		if ( ( DMAStatus != kIOReturnSuccess ) || ( numSegmentos != 1 ) || ( elSegmento.fLength == 0 ) )
		{
			
			panic ( "ApplePCIIDE::createChannelCommands [%d] status %x segs %d phys %p:%x \n", __LINE__, DMAStatus, (unsigned int)numSegmentos, (void*)elSegmento.fIOVMAddr, (unsigned int)elSegmento.fLength );
		    break;
		    
		}
		
        xferDataPtr = (UInt8 *) elSegmento.fIOVMAddr;
        xferCount   = elSegmento.fLength;
		
        if ( elSegmento.fIOVMAddr & 0x01 )
        {
            IOLog("%s: DMA buffer %p not 2 byte aligned\n",
                  getName(), xferDataPtr);
            return kIOReturnNotAligned;        
        }
		
        if ( xferCount & 0x01 )
        {
            IOLog("%s: DMA buffer length %u is odd\n",
                  getName(), (unsigned int)xferCount);
        }
		
        // Update bytes remaining count after this pass.
        bytesRemaining -= xferCount;
        xfrPosition += xferCount;
		
        // Examine the segment to see whether it crosses (a) 64k boundary(s)
        starting64KBlock = (UInt8*) ( elSegmento.fIOVMAddr & 0xffff0000);
        ptr2EndData  = xferDataPtr + xferCount;
        next64KBlock = starting64KBlock + 0x10000;
		
        // Loop until this physical segment is fully accounted for.
        // It is possible to have a memory descriptor which crosses more
        // than one 64K boundary in a single span.
        
        while ( xferCount > 0 )
        {
            if (ptr2EndData > next64KBlock)
            {
                count2Next64KBlock = next64KBlock - xferDataPtr;
                if ( index < kATAMaxDMADesc )
                {
                    setPRD( xferDataPtr, (UInt16)count2Next64KBlock,
						   &_prdTable[index], kContinue_PRD);
                    
                    xferDataPtr = next64KBlock;
                    next64KBlock += 0x10000;
                    xferCount -= count2Next64KBlock;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 1\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
            else
            {
                if (index < kATAMaxDMADesc)
                {
                    setPRD( xferDataPtr, (UInt16) xferCount,
						   &_prdTable[index],
						   (bytesRemaining == 0) ? kLast_PRD : kContinue_PRD);
                    xferCount = 0;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 2\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
        }
    } // end of segment counting loop.
	FEDE_RELOG("FEDE exit de ciclo\n");

    if (index == 0)
    {
        IOLog("%s: rejected command with zero PRD count (0x%x bytes)\n",
              getName(), (unsigned int)_currentCommand->getByteCount());
        return kATADeviceError;
    }
	
    // Transfer is satisfied and only need to check status on interrupt.
    _dmaState = kATADMAStatus;
    
    // Chain is now ready for execution.
    return kATANoErr;
}

//---------------------------------------------------------------------------

bool ApplePCIIDEDriver::allocDMAChannel( void )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
#if defined(MAC_OS_X_VERSION_10_6)
	_prdBuffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
																  kernel_task,
																  kIODirectionInOut | kIOMemoryPhysicallyContiguous,
																  sizeof(PRD) * kATAMaxDMADesc,
																  0xFFFF0000UL );
    
    if ( !_prdBuffer )
    {
        IOLog("%s: PRD buffer allocation failed\n", getName());
        return false;
    }
 	
	_prdBuffer->prepare ( );
	_prdTable			= (PRD *) _prdBuffer->getBytesNoCopy();
	_prdTablePhysical	= _prdBuffer->getPhysicalAddress();
#else
    _prdTable = (PRD *) IOMallocContiguous(
										   /* size  */ sizeof(PRD) * kATAMaxDMADesc, 
										   /* align */ 0x10000, 
										   /* phys  */ &_prdTablePhysical );

    if ( !_prdTable )
    {
        IOLog("%s: PRD table allocation failed\n", getName());
        return false;
    }
#endif	

    _DMACursor = IONaturalMemoryCursor::withSpecification(
														  /* max segment size  */ 0x10000,
														  /* max transfer size */ kMaxATAXfer );
    
    if ( !_DMACursor )
    {
        freeDMAChannel();
        IOLog("%s: Memory cursor allocation failed\n", getName());
        return false;
    }
	
    // fill the chain with stop commands to initialize it.    
    initATADMAChains( _prdTable );
	
    return true;
}

//---------------------------------------------------------------------------

bool ApplePCIIDEDriver::freeDMAChannel( void )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    if ( _prdTable )
    {
        // make sure the engine is stopped.
        stopDMA();
		
        // free the descriptor table.
#if defined(MAC_OS_X_VERSION_10_6)
        _prdBuffer->complete();
        _prdBuffer->release();
        _prdBuffer = NULL;
        _prdTable = NULL;
        _prdTablePhysical = 0;
#else
        IOFreeContiguous(_prdTable, sizeof(PRD) * kATAMaxDMADesc);
#endif
    }
    return true;
}

//---------------------------------------------------------------------------

void ApplePCIIDEDriver::initATADMAChains( PRD * descPtr )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    UInt32 i;
	
    /* Initialize the data-transfer PRD channel command descriptors. */
	
    for (i = 0; i < kATAMaxDMADesc; i++)
    {
        descPtr->bufferPtr = 0;
        descPtr->byteCount = 1;
        descPtr->flags = OSSwapHostToLittleConstInt16( kLast_PRD );
        descPtr++;
    }
}

//---------------------------------------------------------------------------

enum {
    kPCIIDEPowerStateOff = 0,
    kPCIIDEPowerStateDoze,
    kPCIIDEPowerStateOn,
    kPCIIDEPowerStateCount
};

void ApplePCIIDEDriver::initForPM( IOService * provider )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    static const IOPMPowerState powerStates[ kPCIIDEPowerStateCount ] =
    {
        { 1, 0, 0,             0,             0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, IOPMSoftSleep, IOPMSoftSleep, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, IOPMPowerOn,   IOPMPowerOn,   0, 0, 0, 0, 0, 0, 0, 0 }
    };
	
	if(fProvider->pciConfigRead16(kIOPCIConfigVendorID) != PCI_SIL_ID){
		PMinit();
	
		registerPowerDriver( this, (IOPMPowerState *) powerStates, kPCIIDEPowerStateCount );
	
		provider->joinPMtree( this );
	}
}

//---------------------------------------------------------------------------

IOReturn ApplePCIIDEDriver::setPowerState( unsigned long stateIndex,
										  IOService *   whatDevice )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
    if ( stateIndex == kPCIIDEPowerStateOff )
    {
        fHardwareLostContext = true;
    }
    else if ( fHardwareLostContext )
    {
        initializeHardware();
        programTimingRegisters();
        fHardwareLostContext = false;
    }
	
    return IOPMAckImplied;
}

//---------------------------------------------------------------------------
IOReturn ApplePCIIDEDriver::startDMA( void ){
#if ! ITE_FORCE_PIO
	if(fProvider->pciConfigRead16(kIOPCIConfigVendorID) == PCI_ITE_ID){
		setITEUDMA();
	}
#endif
	return IOPCIATA::startDMA();
}

//---------------------------------------------------------------------------
IOReturn ApplePCIIDEDriver::stopDMA( void ){
	IOReturn rc = IOPCIATA::stopDMA();
#if ! ITE_FORCE_PIO
	if(fProvider->pciConfigRead16(kIOPCIConfigVendorID) == PCI_ITE_ID){
		;
	}
#endif
	return rc;
}
