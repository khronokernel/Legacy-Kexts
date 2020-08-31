/* add your code here */
#include "AgereET131x.h"

/******************************************************************************
 ROUTINE:  EnablePhyComa
 ******************************************************************************
 DESCRIPTION:
 This routine is called when network cable is unplugged -- driver 
 receive an phy status change interrupt while in D0 and check that 
 phy_status is down. 
 
 -- gate off JAGCore; 
 -- set gigE PHY in Coma mode
 -- wake on phy_interrupt; Perform software reset JAGCore, 
 re-initialize jagcore and gigE PHY
 
 Add D0-ASPM-PhyLinkDown Support:
 -- while in D0, when there is a phy_interrupt indicating phy link 
 down status, call the MPSetPhyComa routine to enter this active 
 state power saving mode
 -- while in D0-ASPM-PhyLinkDown mode, when there is a phy_interrupt
 indicating linkup status, call the MPDisablePhyComa routine to 
 restore JAGCore and gigE PHY
 
 PARAMETERS :
 pAdapter   - pointer to our adapter structure
 
 RETURNS    :
 NONE
 
 *****************************************************************************/



#pragma mark IOKit Framework
OSDefineMetaClassAndStructors(AgereET131x, super);


static uint32_t jiffies()
{
#if defined(MAC_OS_X_VERSION_10_6)
	clock_sec_t seconds;
	clock_usec_t microsecs;
#else
	uint32_t seconds;
	uint32_t microsecs;
#endif
	clock_get_system_microtime(&seconds, &microsecs);
	return  seconds * 100 + microsecs / 10000; // 10 ms
}



void AgereET131x::free()
{
	//IOLog("AgereET131x::free()\n");
    IOSimpleLockFree( adapter.Lock );
    IOSimpleLockFree( adapter.TCBSendQLock );
    IOSimpleLockFree( adapter.TCBReadyQLock );
    IOSimpleLockFree( adapter.SendHWLock );
    IOSimpleLockFree( adapter.RcvLock );
    IOSimpleLockFree( adapter.FbrLock );
    IOSimpleLockFree( adapter.PHYLock );
	
	RELEASE(netif);
	RELEASE(interruptSource);
	RELEASE(watchdogSource);
	
	RELEASE(workLoop);
	RELEASE(pciDevice);
	RELEASE(mediumDict);
	
	int i;
	for (i = 0; i < MEDIUM_INDEX_COUNT; i++) {
		RELEASE(mediumTable[i]);
	}
	
	RELEASE(csrPCIAddress);
	
	RELEASE(rxMbufCursor);
	RELEASE(txMbufCursor);
	
	super::free();
}

bool AgereET131x::init(OSDictionary *properties)
{
	//IOLog("AgereET131x::init()\n");
	if (super::init(properties) == false) 
		return false;
	
	enabledForNetif = false;
	
	pciDevice = NULL;
	mediumDict = NULL;
	csrPCIAddress = NULL;
	interruptSource = NULL;
	watchdogSource = NULL;
	netif = NULL;
	
	transmitQueue = NULL;
	rxMbufCursor = NULL;
	txMbufCursor = NULL;
	
	for (int i = 0; i < MEDIUM_INDEX_COUNT; i++) {
		mediumTable[i] = NULL;
	}

	suspend = false;

    adapter.Lock = IOSimpleLockAlloc();
    adapter.TCBSendQLock = IOSimpleLockAlloc();
    adapter.TCBReadyQLock = IOSimpleLockAlloc();
    adapter.SendHWLock = IOSimpleLockAlloc();
    adapter.RcvLock = IOSimpleLockAlloc();
    adapter.FbrLock = IOSimpleLockAlloc();
    adapter.PHYLock = IOSimpleLockAlloc();

	return true;
}

bool AgereET131x::createWorkLoop()
{
	workLoop = IOWorkLoop::workLoop();	
	return (workLoop !=  NULL);
}

IOWorkLoop* AgereET131x::getWorkLoop() const
{
	return workLoop;
}

//---------------------------------------------------------------------------
bool AgereET131x::initEventSources( IOService* provider )
{
	// Get a handle to our superclass' workloop.
	//
	IOWorkLoop* myWorkLoop = (IOWorkLoop *) getWorkLoop();
	if (myWorkLoop == NULL) {
		IOLog(" myWorkLoop is NULL.\n");
		return false;
	}
	
	transmitQueue = getOutputQueue();
	if (transmitQueue == NULL) {
		IOLog("getOutputQueue failed.\n");
		return false;
	}
	transmitQueue->setCapacity(NUM_TCB);
	
	interruptSource = IOFilterInterruptEventSource::filterInterruptEventSource( this, &AgereET131x::interruptHandler, &AgereET131x::interruptFilter, provider);
	
	if (!interruptSource ||
		(myWorkLoop->addEventSource(interruptSource) != kIOReturnSuccess)) {
		IOLog("workloop add eventsource interrupt source.\n");
		return false;
	}
	
	// This is important. If the interrupt line is shared with other devices,
	// then the interrupt vector will be enabled only if all corresponding
	// interrupt event sources are enabled. To avoid masking interrupts for
	// other devices that are sharing the interrupt line, the event source
	// is enabled immediately.
	interruptSource->enable();
	
	// Register a timer event source. This is used as a watchdog timer.
	//
	watchdogSource = IOTimerEventSource::timerEventSource(this, &AgereET131x::timeoutHandler );
	if (!watchdogSource || (myWorkLoop->addEventSource(watchdogSource) != kIOReturnSuccess)) {
		IOLog("watchdogSource create failed.\n");
		return false;
	}
	
	mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT + 1);
	if (mediumDict == NULL) {
		return false;
	}
	return true;
}

void AgereET131x::stop(IOService* provider)
{
	//IOLog("%s::%s()\n",getName(),__FUNCTION__);
	if(workLoop){
		if (watchdogSource) {
			workLoop->removeEventSource(watchdogSource);
			watchdogSource->release();
			watchdogSource = NULL;
		}
		
		if (interruptSource) {
			workLoop->removeEventSource(interruptSource);
			interruptSource->release();
			interruptSource = NULL;
		}
	}
	adapter_memory_free();

	super::stop(provider);
}


bool AgereET131x::start(IOService* provider)
{
	//IOLog("%s::%s()\n",getName(),__FUNCTION__);
    bool success = false;
	
	if (super::start(provider) == false) {
		IOLog("supper::start failed.\n");
		return false;
	}
	pciDevice = OSDynamicCast(IOPCIDevice, provider);
	if (pciDevice == NULL)
		return false;
	
	
	pciDevice->retain();
	if (pciDevice->open(this) == false)
		return false;

	adapter.VendorID = pciDevice->configRead16(kIOPCIConfigVendorID);
	adapter.DeviceID = pciDevice->configRead16(kIOPCIConfigDeviceID);
	adapter.SubVendorID = pciDevice->configRead16(kIOPCIConfigSubSystemVendorID);
	adapter.SubSystemID = pciDevice->configRead16(kIOPCIConfigSubSystemID);
	adapter.RevisionID = pciDevice->configRead8(kIOPCIConfigRevisionID);
	
	
	// adapter.hw.device_id will be used later
	IOLog("vendor:device: 0x%x:0x%x.\n", adapter.VendorID, adapter.DeviceID);
	
	do {
		if(!initEventSources(provider)){
			break;
		}
		
		csrPCIAddress = pciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
		if (csrPCIAddress == NULL) {
			IOLog("csrPCIAddress.\n");
			break;
		}
		adapter.CSRAddress = (ADDRESS_MAP_t*)csrPCIAddress->getVirtualAddress();
		
		adapter.netdev = this;
		adapter.pdev = pciDevice;
		
		set_mtu( ETH_DATA_LEN );

		// Init PCI config space:
        if ( false == initPCIConfigSpace() )
            break;
		
		
		// reset the hardware with the new settings
		addNetworkMedium(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO);
		addNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex,
						 10 * MBit, MEDIUM_INDEX_10HD);
		addNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex,
						 10 * MBit, MEDIUM_INDEX_10FD);
		addNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex,
						 100 * MBit, MEDIUM_INDEX_100HD);
		addNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex,
						 100 * MBit, MEDIUM_INDEX_100FD);
		addNetworkMedium(kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex,
						 1000 * MBit, MEDIUM_INDEX_1000FD);
		
		if (!publishMediumDictionary(mediumDict)) {
			IOLog("publishMediumDictionary.\n");
			break;
		}
		
		success = true;
	} while(false);
	
	// Close our provider, it will be re-opened on demand when
	// our enable() is called by a client.
	pciDevice->close(this);

	do {
		if ( false == success )
			break;
		
		// Allocate and attach an IOEthernetInterface instance.
		if (attachInterface((IONetworkInterface **)(&netif), true) == false) {
			IOLog("Failed to attach data link layer.\n");
			break;
		}
		
		success = true;
	} while(false);

	if(false == success){
		adapter_memory_free();
	}
	return success;
}

IOReturn AgereET131x::selectMedium(const IONetworkMedium * medium)
{
	
	if (OSDynamicCast(IONetworkMedium, medium) == 0) {
		// Defaults to Auto.
		medium = mediumTable[MEDIUM_INDEX_AUTO];
	}
	
	if (medium){
		setCurrentMedium(medium);
		return kIOReturnSuccess;
	}

	return kIOReturnIOError;
}


bool AgereET131x::configureInterface(IONetworkInterface * interface)
{
	IONetworkData * data = NULL;
	
	if (super::configureInterface(interface) == false) {
		return false;
	}
	
	// Get the generic network statistics structure.
	data = interface->getParameter(kIONetworkStatsKey);
	if (!data || !(netStats = (IONetworkStats *) data->getBuffer())) {
		return false;
	}
	
	// Get the Ethernet statistics structure.
	
	data = interface->getParameter(kIOEthernetStatsKey);
	if (!data || !(etherStats = (IOEthernetStats *) data->getBuffer())) {
		return false;
	}
	
	return true;
}

IOReturn AgereET131x::enable(IONetworkInterface * netif)
{
	//IOLog("AgereET131x::enable()\n");

	if (pciDevice->open(this) == false)
		return kIOReturnError;

    /**************************************************************************
	 Enable the Tx and Rx DMA engines (if not already enabled)
     *************************************************************************/
    rx_dma_enable( );
    tx_dma_enable();
	
	
    /**************************************************************************
	 Enable device interrupts
     *************************************************************************/
    enable_interrupts();
	
    MP_SET_FLAG( &adapter, fMP_ADAPTER_INTERRUPT_IN_USE );
	
	
    /**************************************************************************
	 We're ready to move some data, so start the queue
     *************************************************************************/
	selectMedium(getSelectedMedium());
	
	watchdogSource->setTimeoutMS(1000);
	
	transmitQueue->start();
	
	workLoop->enableAllInterrupts();
	
	enabledForNetif = true;
	
	return kIOReturnSuccess;
}

IOReturn AgereET131x::disable(IONetworkInterface * netif)
{
	//IOLog("AgereET131x::disable()\n");
	if(!enabledForNetif)
		return kIOReturnSuccess;

	enabledForNetif = false;
	
    /**************************************************************************
	 First thing is to stop the queue
     *************************************************************************/
	workLoop->disableAllInterrupts();
	
	if (transmitQueue) {
		transmitQueue->stop();
		transmitQueue->flush();
	}
	
	//stop watchdog timer
	watchdogSource->cancelTimeout();		// Stop the timer event source.
	
	
    /**************************************************************************
	 Stop the Tx and Rx DMA engines
     *************************************************************************/
    rx_dma_disable();
    tx_dma_disable();
	
	
    /**************************************************************************
	 Disable device interrupts
     *************************************************************************/
    disable_interrupts( );
	
	
    /**************************************************************************
	 Deregistering ISR
     *************************************************************************/
    MP_CLEAR_FLAG( &adapter, fMP_ADAPTER_INTERRUPT_IN_USE );
	
	setLinkStatus( kIONetworkLinkValid );	// Valid sans kIONetworkLinkActive
	
	
    /**************************************************************************
	 Stop the error timer
     *************************************************************************/
    //del_timer_sync( &adapter->ErrorTimer );
	pciDevice->close(this);
	
	return kIOReturnSuccess;
}

UInt32 AgereET131x::outputPacket(mbuf_t m, void * param)
{
	int status = 0;

    /**************************************************************************
	 Queue is not empty or TCB is not available
     *************************************************************************/
    if( MP_TCB_RESOURCES_NOT_AVAILABLE( &adapter )) {
        /**********************************************************************
		 NOTE - If there's an error on send, no need to queue the
		 packet under Linux; if we just send an error up to the netif
		 layer, it will resend the skb to us.
         *********************************************************************/
       // IOLog("TCB Resources Not Available\n" );
		freePacket(m);
		netStats->outputErrors += 1;
		return kIOReturnOutputDropped;
    }

	/**********************************************************************
	 We need to see if the link is up; if it's not, make the netif layer
	 think we're good and drop the packet
	 *********************************************************************/
	if( MP_SHOULD_FAIL_SEND( &adapter ) ){
		freePacket( m );
		netStats->outputErrors += 1;
		return kIOReturnOutputDropped;
	}

	status = send_packet( m );
	if( status != 0 ){
		IOLog( "General error, drop packet(%d)\n", status );
		freePacket( m );
		netStats->outputErrors += 1;
		return kIOReturnOutputDropped;
	}

	return kIOReturnSuccess;
}

void AgereET131x::getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const
{
	constraints->alignStart = kIOPacketBufferAlign4;
	constraints->alignLength = kIOPacketBufferAlign4;
}

IOOutputQueue * AgereET131x::createOutputQueue()
{
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

const OSString * AgereET131x::newVendorString() const
{
	return OSString::withCString("Agere (LSI Logic)");
}

const OSString * AgereET131x::newModelString() const
{
	return OSString::withCString("et1310");
}


IOReturn AgereET131x::getHardwareAddress(IOEthernetAddress * addr)
{
	bcopy( adapter.CurrentAddress, addr->bytes,  kIOEthernetAddressSize );
	return kIOReturnSuccess;
}

IOReturn AgereET131x::setHardwareAddress(const IOEthernetAddress * addr)
{
	set_mac_addr(addr->bytes);
	return kIOReturnSuccess;
}

IOReturn AgereET131x::setPromiscuousMode(bool active)
{
    if( active )
    {
        adapter.PacketFilter |= ET131X_PACKET_TYPE_PROMISCUOUS;
    }
    else
    {
        adapter.PacketFilter &= ~ET131X_PACKET_TYPE_PROMISCUOUS;
    }
	set_packet_filter();
	return kIOReturnSuccess;
}

IOReturn AgereET131x::setMulticastMode(bool active)
{
	
    if( active )
    {
        adapter.PacketFilter |= ET131X_PACKET_TYPE_MULTICAST;
    } else {
        adapter.PacketFilter &= ~(ET131X_PACKET_TYPE_MULTICAST|ET131X_PACKET_TYPE_ALL_MULTICAST);
	}
	set_packet_filter();
	return kIOReturnSuccess;
}

IOReturn AgereET131x::setMulticastList(IOEthernetAddress * addrs, UInt32 count)
{
    UINT32              PacketFilter = 0;

    IOSimpleLockLock( adapter.Lock );
	
	
    /**************************************************************************
	 Before we modify the platform-independent filter flags, store them 
	 locally. This allows us to determine if anything's changed and if we
	 even need to bother the hardware
     *************************************************************************/
    PacketFilter = adapter.PacketFilter;
	
	
    /**************************************************************************
	 Clear the 'multicast' flag locally; becuase we only have a single flag
	 to check multicast, and multiple multicast addresses can be set, this is
	 the easiest way to determine if more than one multicast address is being
	 set.
     *************************************************************************/
    PacketFilter &= ~ET131X_PACKET_TYPE_MULTICAST;
	
	
    /**************************************************************************
	 Check the net_device flags and set the device independent flags
	 accordingly
     *************************************************************************/
	
	
	
	
    if( count > NIC_MAX_MCAST_LIST )
    {
        adapter.PacketFilter |= ET131X_PACKET_TYPE_ALL_MULTICAST;
    } else  if( count < 1 ) {
        adapter.PacketFilter &= ~ET131X_PACKET_TYPE_ALL_MULTICAST;
        adapter.PacketFilter &= ~ET131X_PACKET_TYPE_MULTICAST;
    } else {
		/**************************************************************************
		 Set values in the private adapter struct
		 *************************************************************************/
		adapter.MCAddressCount = count;
		
		for( UInt32 k = 0; k < count; k++ )
		{
			bcopy( addrs[k].bytes, adapter.MCList[k], ETH_ALEN );
		}
	}
	
    /**************************************************************************
	 Are the new flags different from the previous ones? If not, then no
	 action is required
	 
	 NOTE - This block will always update the MCList with the hardware, even
	 if the addresses aren't the same.
     *************************************************************************/
    if( PacketFilter != adapter.PacketFilter )
    {
        /**********************************************************************
		 Call the device's filter function
         *********************************************************************/
		
        set_packet_filter();
    }
	
    IOSimpleLockUnlock( adapter.Lock );

	return kIOReturnSuccess;
}


IOReturn AgereET131x::getMaxPacketSize (UInt32 *maxSize) const {
	if (maxSize)
		*maxSize = 9216 - 14;

	return kIOReturnSuccess;
}

IOReturn AgereET131x::getMinPacketSize (UInt32 *minSize) const {
	if(minSize)
		*minSize = ETH_ZLEN + ETH_FCS_LEN + VLAN_HLEN;
	
	return kIOReturnSuccess;
}

IOReturn AgereET131x::setMaxPacketSize (UInt32 maxSize){
    UInt32 newMtu = maxSize - (ETH_HLEN + ETH_FCS_LEN);
	if(newMtu == mtu)
		return kIOReturnSuccess;

	adapter_memory_free();

	set_mtu(newMtu);

	adapter_memory_alloc();

	return kIOReturnSuccess;
}

UInt32 AgereET131x::getFeatures() const
{
	return kIONetworkFeatureMultiPages | kIONetworkFeatureSoftwareVlan;
}

IOReturn AgereET131x::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) 
{
	if( checksumFamily == kChecksumFamilyTCPIP ) {
#if	USE_TX_CSUM
		*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
		return kIOReturnSuccess;
#else
		if( ! isOutput ) {
			*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
			return kIOReturnSuccess;
		}
#endif
	}
	*checksumMask = 0;
	return kIOReturnUnsupported;
}


IOReturn AgereET131x::registerWithPolicyMaker(IOService* policyMaker)
{
	static IOPMPowerState powerStateArray[ 2 ] = {
		{ 1,0,0,0,0,0,0,0,0,0,0,0 },
		{ 1,kIOPMDeviceUsable,kIOPMPowerOn,kIOPMPowerOn,0,0,0,0,0,0,0,0 }
	};
	powerState = 1;
	return policyMaker->registerPowerDriver( this, powerStateArray, 2 );
}

IOReturn AgereET131x::setPowerState( unsigned long powerStateOrdinal, IOService *policyMaker )
{
	if(powerState == 1){	//
		;
	} else {
		suspend = true;
	}
    return IOPMAckImplied;
}

#pragma mark interrupt Handler
void AgereET131x::interruptHandler(OSObject * target, IOInterruptEventSource * src, int count)
{
	//IOLog("AgereET131x::interruptHandler(%d)\n", count);
	((AgereET131x*) target)->interruptOccurred(src);
}

bool AgereET131x::interruptFilter(OSObject * target, IOFilterInterruptEventSource * src )
{
	bool rc = ((AgereET131x*) target)->filterCheck(src);
	//IOLog("AgereET131x::interruptFilter(%d)\n", rc);
	return rc;
}

void AgereET131x::timeoutHandler(OSObject * target, IOTimerEventSource * src)
{
	//IOLog("AgereET131x::timeoutHandler()\n");
	((AgereET131x*) target)->timeoutOccurred( src );
}

