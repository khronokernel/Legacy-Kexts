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

#ifndef _APPLEPCIIDE_H
#define _APPLEPCIIDE_H

#include <AvailabilityMacros.h>

#include <IOKit/IOLocks.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/IOService.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/ata/IOATATypes.h>
#include <IOKit/ata/IOPCIATA.h>
#include <IOKit/ata/IOATAController.h>
#include <IOKit/ata/ATADeviceNub.h>
#include "ApplePCIIDEHardware.h"

class ApplePCIIDERoot : public IOService
{
    OSDeclareDefaultStructors( ApplePCIIDERoot )
	
protected:
    IOPCIDevice *  fProvider;
    IOLock *       fPCILock;
    OSSet *        fChannels;
    OSSet *        fOpenChannels;
	
    virtual OSSet * createATAChannels( void );
	
    virtual OSDictionary * createNativeModeChannelInfo( UInt32 ataChannel );
	
    virtual OSDictionary * createChannelInfo( UInt32 ataChannel,
											 UInt16 commandPort,
											 UInt16 controlPort,
											 UInt8  interruptVector );
	
    virtual IORegistryEntry * getDTChannelEntry( int channelID );
	
public:
    virtual IOService * probe( IOService * provider,
							  SInt32 *    score );
	
    virtual bool   start( IOService * provider );
	
    virtual void   free( void );
	
    virtual bool   handleOpen( IOService *  client,
							  IOOptionBits options,
							  void *       arg );
    
    virtual void   handleClose( IOService *  client,
							   IOOptionBits options );
	
    virtual bool   handleIsOpen( const IOService * client ) const;
	
    virtual UInt32 getHardwareFlags( void ) const;
	
    virtual UInt32 getUltraDMAModeMask( void ) const;
	
    virtual void   pciConfigWrite8( UInt8 offset,
								   UInt8 data,
								   UInt8 mask = 0xff );
	
	virtual void   pciConfigWrite16( UInt8  offset,
									UInt16 data,
									UInt16 mask = 0xffff );
	
    virtual void   pciConfigWrite32( UInt8  offset,
									UInt32 data,
									UInt32 mask = 0xffffffff );
	
    virtual UInt8  pciConfigRead8( UInt8 offset );
	
    virtual UInt16 pciConfigRead16( UInt8 offset );
	
    virtual UInt32 pciConfigRead32( UInt8 offset );

private:
	bool initHardware();
	int nChannels;
};

class ApplePCIIDEChannel : public IOService
{
    OSDeclareDefaultStructors( ApplePCIIDEChannel )
	
protected:
    ApplePCIIDERoot * fProvider;
    virtual bool   mergeProperties( OSDictionary * properties );
	
    virtual bool   getNumberValue( const char * propKey,
								  void       * outValue,
								  UInt32       outBits ) const;
	
public:
    virtual bool   init( IOService *       provider,
						OSDictionary *    properties,
						IORegistryEntry * dtEntry = 0 );
	
    virtual UInt16 getCommandBlockAddress( void ) const;
	
    virtual UInt16 getControlBlockAddress( void ) const;
	
    virtual UInt32 getInterruptVector( void ) const;
	
    virtual UInt32 getUltraDMAModeMask( void ) const;
	
    virtual UInt32 getChannelNumber( void ) const;
	
    virtual UInt32 getHardwareFlags( void ) const;
	
    virtual const char * getHardwareName( void ) const;
	
    virtual bool   handleOpen( IOService *  client,
							  IOOptionBits options,
							  void *       arg );
	
    virtual void   handleClose( IOService *  client,
							   IOOptionBits options );
	
    virtual void   pciConfigWrite8( UInt8 offset,
								   UInt8 data,
								   UInt8 mask = 0xff );
	
	virtual void   pciConfigWrite16( UInt8  offset,
									UInt16 data,
									UInt16 mask = 0xffff );
	
    virtual void   pciConfigWrite32( UInt8  offset,
									UInt32 data,
									UInt32 mask = 0xffffffff );
	
    virtual UInt8  pciConfigRead8( UInt8 offset );
	
    virtual UInt16 pciConfigRead16( UInt8 offset );
	
    virtual UInt32 pciConfigRead32( UInt8 offset );
};

struct ATABusTimings
{
    UInt8                      pioModeNumber;
    bool                       dmaEnabled;
    UInt8                      dmaModeNumber;
    UInt8                      ultraModeNumber;
    bool                       ultraEnabled;
};

class ApplePCIIDEDriver : public IOPCIATA
{
    OSDeclareDefaultStructors( ApplePCIIDEDriver )
	
protected:
    ApplePCIIDEChannel *       fProvider;
	IODMACommand	*		   comando;
	
    IOInterruptEventSource *   fInterruptSource;
    IOWorkLoop *               fWorkLoop;
    ATABusTimings              fBusTimings[ kMaxDriveCount ];
    bool                       f80PinCable;
    UInt16                     fBMBaseAddr;
    UInt32                     fChannelNumber;
    bool                       fHardwareLostContext;
	
#if defined(MAC_OS_X_VERSION_10_6)
	IOBufferMemoryDescriptor*		_prdBuffer;
#endif
    /* Interrupt event source action */
	
    static void      interruptOccurred( OSObject *               owner,
									   IOInterruptEventSource * src,
									   int                      count );
	
    /* Interrupt event source filter */
	
    static bool      interruptFilter( OSObject * owner,
									 IOFilterInterruptEventSource * src );
	
    /* Driver functions */
	
    virtual bool     getBMBaseAddress( UInt32   channel,
									  UInt16 * baseAddr );
	
    virtual void     resetBusTimings( void );
	
    virtual void     selectIOTiming( ataUnitID unit );
	
    virtual bool     setDriveProperty( UInt32       driveUnit,
									  const char * key,
									  UInt32       value,
									  UInt32       numberOfBits);
	
    virtual IOReturn synchronousIO( void );
	
    virtual void     initForPM( IOService * provider );
	
    virtual void     selectTimingParameter( IOATADevConfig * configRequest,
										   UInt32           unitNumber );
	
    virtual void     programTimingRegisters( void );
	
	
    virtual void     initializeHardware( void );
	
public:
    /* IOService overrides */
	
    virtual bool     start( IOService * provider );
	
    virtual void     stop( IOService * provider );
	
    virtual void     free( void );
	
    virtual IOWorkLoop * getWorkLoop( void ) const;
	
    virtual IOReturn message( UInt32      type,
							 IOService * provider,
							 void *      argument );
	
    virtual IOReturn setPowerState( unsigned long stateIndex,
								   IOService *   whatDevice );
	
    /* Mandatory IOATAController overrides */
	
    virtual bool     configureTFPointers( void );
	
    virtual IOReturn provideBusInfo( IOATABusInfo * infoOut );
	
    virtual IOReturn getConfig( IOATADevConfig * configOut,
							   UInt32           unit );
	
    virtual IOReturn selectConfig( IOATADevConfig * config,
								  UInt32           unit );
	
    /* Optional IOATAController overrides */
	
    virtual UInt32   scanForDrives( void );
	
    virtual IOReturn handleQueueFlush( void );
	
    /* Optional IOPCIATA overrides to support large transfers */
	
    virtual bool     allocDMAChannel( void );
	
    virtual void     initATADMAChains( PRD * descPtr );
	
    virtual IOReturn createChannelCommands( void );
	
    virtual bool     freeDMAChannel( void );

	virtual IOReturn startDMA( void );
	virtual IOReturn stopDMA( void );
private:
	void setITEUDMA( void );
};

#endif /* !_APPLEPCIIDE_H */
