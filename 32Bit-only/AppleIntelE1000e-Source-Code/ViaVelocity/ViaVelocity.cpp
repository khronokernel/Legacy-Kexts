#include "ViaVelocity.h"

#define	NO_LOCK_FREE	0

static IOBufferMemoryDescriptor* allocPool( mach_vm_size_t size,  mach_vm_address_t mask = MALLOC_MASK )
{
	return IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionOut | kIOMemoryPhysicallyContiguous, size, mask);
}

//////////
//////////
//////////

#define ONE_SECOND_TICKS			1000
#define LOAD_STATISTICS_INTERVAL	(4 * ONE_SECOND_TICKS)

#define super IOEthernetController
OSDefineMetaClassAndStructors( ViaVelocity, IOEthernetController )

bool ViaVelocity::init(OSDictionary *properties)
{
	
	if (super::init(properties) == false) 
		return false;

	bzero(&velocity, sizeof(velocity));
#if	NO_LOCK_FREE
	velocity.lock = IOSimpleLockAlloc();
#endif
	mediumDict = NULL;
	interruptSrc = NULL;
	timerSrc = NULL;
	netif = NULL;
	
	transmitQueue = NULL;
	rxMbufCursor = NULL;
	tqActive = FALSE;
	suspend = FALSE;

	bzero(mediumTable, sizeof(mediumTable));

	return true;
}

//---------------------------------------------------------------------------
// Function: free <IOService>
//
// Deallocate all resources and destroy the instance.

void ViaVelocity::free()
{
#define RELEASE(x) do { if(x) { (x)->release(); (x) = 0; } } while(0)
	
	RELEASE( debugger     );
	RELEASE( netif        );
	
	RELEASE( interruptSrc );
	RELEASE( timerSrc     );
	RELEASE( rxMbufCursor );
	RELEASE( csrMap );
	RELEASE( mediumDict   );
	RELEASE( pciNub       );
	RELEASE( workLoop     );
#if	NO_LOCK_FREE	
	IOSimpleLockFree(velocity.lock);
#endif
	super::free();	// pass it to our superclass
}

//---------------------------------------------------------------------------
// Function: newVendorString(), newModelString()
//           <IONetworkController>
//
// Report human readable hardware information strings.

const OSString * ViaVelocity::newVendorString() const
{
	return OSString::withCString("Via Technologies");
}

const OSString * ViaVelocity::newModelString() const
{
	if( velocity.hw.byRevId < 0x10 )
		return OSString::withCString("VT6110");
	if( velocity.hw.byRevId < 0x20 )
		return OSString::withCString("VT612x");
	if( velocity.hw.byRevId >= 0x80 )
		return OSString::withCString("VT613x");
	
	return OSString::withCString("Velocity (Unknown)");
}

void ViaVelocity::netif_start_queue(){ tqActive = true; }
void ViaVelocity::netif_stop_queue() { tqActive = false; }
BOOL ViaVelocity::netif_running(){ return tqActive; }

//
// Initialiation of adapter
//
void ViaVelocity::velocity_init_adapter( VELOCITY_INIT_TYPE  InitType )
{
    int                 i;
	
    mac_wol_reset(&velocity.hw);
	
    switch (InitType) {
		case VELOCITY_INIT_RESET:
		case VELOCITY_INIT_WOL:
			
			netif_stop_queue();
			
			velocity_init_register_reset(&velocity.hw);
			
			if (!(velocity.hw.mii_status & VELOCITY_LINK_FAIL))
				netif_start_queue();
			
			break;
			
		case VELOCITY_INIT_COLD:
		default:
			mac_eeprom_reload(&velocity.hw);
			// write dev->dev_addr to MAC address field for MAC address override
			for (i = 0; i < 6; i++)
				CSR_WRITE_1(&velocity.hw, myAddress.bytes[i], MAC_REG_PAR+i);
			
			// clear Pre_ACPI bit.
			BYTE_REG_BITS_OFF(&velocity.hw, CFGA_PACPI, MAC_REG_CFGA);
			
			// set packet filter
			// receive directed and broadcast address
			velocity_set_multi();
			netif_stop_queue();

			velocity_init_register_cold(&velocity.hw, (struct pci_dev*)velocity.pcid);
			

			if (!(velocity.hw.mii_status & VELOCITY_LINK_FAIL))
				netif_start_queue();

			break;
    } // switch (InitType)
}

//---------------------------------------------------------------------------
// Function: pciConfigInit
//
// Update PCI command register to enable the memory-mapped range,
// and bus-master interface.

bool ViaVelocity::pciConfigInit(IOPCIDevice * provider)
{
	velocity.hw.SubSystemID = provider->configRead16(kIOPCIConfigSubSystemID);
	velocity.hw.SubVendorID = provider->configRead16(kIOPCIConfigSubSystemVendorID);
	velocity.hw.byRevId = provider->configRead8(kIOPCIConfigRevisionID);

	provider->setMemoryEnable(true);
	provider->setBusMasterEnable(true);

	// moved from velocity_init_pci()
	// turn this on to avoid retry forever
	provider->configWrite8( PCI_REG_MODE2, pciNub->configRead8(PCI_REG_MODE2) | MODE2_PCEROPT );
	// turn this on to detect MII coding error
	provider->configWrite8( PCI_REG_MODE3, pciNub->configRead8(PCI_REG_MODE3) | MODE3_MIION );
	
	csrMap = pciNub->mapDeviceMemoryWithRegister( kIOPCIConfigBaseAddress1 );
	if ( csrMap == 0 ){
		return false;
	}
	
	velocity.hw.memaddr = csrMap->getVirtualAddress();
	velocity.hw.hw_addr = (PU8)velocity.hw.memaddr;

	IOByteCount offset = 0;
	if(provider->extendedFindPCICapability( kIOPCIPCIExpressCapability, &offset)){
		is64bit = true;
		IOLog("%s: PCI Express.\n",getName());
	} else {
		is64bit = false;
		IOLog("%s: PCI 32bit.\n",getName());
	}
	
	velocity.pcid = provider;

	return true;
}

//---------------------------------------------------------------------------
// Function: initDriver
//
// Create and initialize driver objects before the hardware is 
// enabled.
//
// Returns true on sucess, and false if initialization failed.

bool ViaVelocity::initDriver(IOService * provider)
{
	currentMediumType = MEDIUM_TYPE_INVALID;
	
	// This driver will allocate and use an IOGatedOutputQueue.
	//
	transmitQueue = getOutputQueue();
	if ( transmitQueue == 0 ) return false;

	transmitQueue->setCapacity(TRANSMIT_QUEUE_SIZE);

	// Get a handle to our superclass' workloop.
	//
	IOWorkLoop * myWorkLoop = (IOWorkLoop *) getWorkLoop();
	if (!myWorkLoop)
		return false;
	
	// Create and register an interrupt event source. The provider will
	// take care of the low-level interrupt registration stuff.
	//
	interruptSrc = IOFilterInterruptEventSource::filterInterruptEventSource(this,
							&ViaVelocity::interruptHandler,&ViaVelocity::interruptFilter,
							provider);
	
	if (!interruptSrc ||
		(myWorkLoop->addEventSource(interruptSrc) != kIOReturnSuccess))
		return false;
	
	// This is important. If the interrupt line is shared with other devices,
    // then the interrupt vector will be enabled only if all corresponding
    // interrupt event sources are enabled. To avoid masking interrupts for
    // other devices that are sharing the interrupt line, the event source
    // is enabled immediately.
	
    interruptSrc->enable();
	
	// Register a timer event source. This is used as a watchdog timer.
	//
	timerSrc = IOTimerEventSource::timerEventSource( this, &ViaVelocity::timeoutHandler );
	if (!timerSrc ||
		(myWorkLoop->addEventSource(timerSrc) != kIOReturnSuccess))
		return false;
	
	// Create a dictionary to hold IONetworkMedium objects.
	//
	mediumDict = OSDictionary::withCapacity(5);
	if (!mediumDict)
		return false;
	
	return true;
}

//---------------------------------------------------------------------------
// Function: getDefaultSettings
//
// Get the default driver settings chosen by the user. The properties
// are all stored in our property table (an OSDictionary).

int ViaVelocity::velocity_get_int_opt( int min, int max, int def, const char *name )
{
	int val = def;
	OSNumber* numObj = OSDynamicCast( OSNumber, getProperty(name) );
	if ( numObj ){
		val = (int)numObj->unsigned32BitValue();
		if( val < min )
			val = min;
		else if(val > max )
			val = max;
	}
	return val;
}

bool ViaVelocity::velocity_get_bool_opt( const char *name, bool defVal )
{
	OSBoolean* rc = OSDynamicCast( OSBoolean, getProperty(name));
	if( rc ){
		return (rc == kOSBooleanTrue );
	}
	return defVal;
}


void ViaVelocity::getDefaultSettings()
{
	maxPacketSize = velocity_get_int_opt( kIOEthernetMaxPacketSize,
										 VELOCITY_MAX_MTU+(ETH_HLEN + ETH_FCS_LEN),
										 kIOEthernetMaxPacketSize, "MAX_PACKET_SIZE");
	
	velocity.hw.sOpts.rx_thresh = velocity_get_int_opt( RX_THRESH_MIN, RX_THRESH_MAX, RX_THRESH_DEF, "RX_THRESH");
	velocity.hw.sOpts.DMA_length = velocity_get_int_opt( DMA_LENGTH_MIN, DMA_LENGTH_MAX, DMA_LENGTH_DEF, "DMA_LENGTH");
	velocity.hw.sOpts.nRxDescs = velocity_get_int_opt( RX_DESC_MIN, RX_DESC_MAX, RX_DESC_DEF, "RX_DESC");
	velocity.hw.sOpts.nTxDescs = velocity_get_int_opt( TX_DESC_MIN, TX_DESC_MAX, TX_DESC_DEF, "TX_DESC");

	velocity.hw.sOpts.vid = velocity_get_int_opt( VLAN_ID_MIN, VLAN_ID_MAX, VLAN_ID_DEF, "VID");
	velocity.hw.sOpts.tagging = velocity_get_int_opt( TAGGING_MIN, TAGGING_MAX, TAGGING_DEF, "TAGGING");
	if(velocity_get_bool_opt("TX_CSUM_OFFLOAD", 1))
		velocity.hw.flags |= VELOCITY_FLAGS_TX_CSUM;	

	velocity.hw.sOpts.flow_cntl = velocity_get_int_opt( FLOW_CNTL_MIN, FLOW_CNTL_MAX, FLOW_CNTL_DEF, "FLOW_CNTL");
	velocity.hw.sOpts.spd_dpx = SPD_DPX_AUTO; // auto
	velocity.hw.sOpts.wol_opts = velocity_get_int_opt( WOL_OPT_MIN, WOL_OPT_MAX, WOL_OPT_DEF, "WOL");
	velocity.hw.sOpts.int_works = velocity_get_int_opt( INT_WORKS_MIN, INT_WORKS_MAX, INT_WORKS_DEF, "INT_WORKS");

	if(velocity_get_bool_opt("ENABLE_MRDPL", MRDPL_DEF))
		velocity.hw.flags |= VELOCITY_FLAGS_MRDPL;	

	if(velocity_get_bool_opt("ENABLE_AI", AI_DEF))
		velocity.hw.flags |= VELOCITY_FLAGS_AI;	

	velocity.hw.sOpts.txque_timer = velocity_get_int_opt( TXQUE_TIMER_MIN, TXQUE_TIMER_MAX, TXQUE_TIMER_DEF, "Tx Queue Empty defer timer");
	velocity.hw.sOpts.rxque_timer = velocity_get_int_opt( RXQUE_TIMER_MIN, RXQUE_TIMER_MAX, RXQUE_TIMER_DEF, "Rx Queue Empty defer timer");

	velocity.hw.sOpts.tx_intsup = velocity_get_int_opt( TX_INTSUP_MIN, TX_INTSUP_MAX, TX_INTSUP_DEF, "Tx Interrupt Suppression Threshold");
	velocity.hw.sOpts.rx_intsup = velocity_get_int_opt( RX_INTSUP_MIN, RX_INTSUP_MAX, RX_INTSUP_DEF, "Rx Interrupt Suppression Threshold");

	
	velocity.hw.sOpts.nRxDescs = (velocity.hw.sOpts.nRxDescs & ~3);
	
	verbose = velocity_get_bool_opt("Verbose",0);
}

//---------------------------------------------------------------------------
// Function: configureInterface <IONetworkController>
//
// Configure a newly instantiated IONetworkInterface object.

bool ViaVelocity::configureInterface(IONetworkInterface * netif)
{
	IONetworkData * data;
	
	if ( super::configureInterface(netif) == false )
		return false;
	
	// Get the generic network statistics structure.
	
	data = netif->getParameter(kIONetworkStatsKey);
	if (!data || !(netStats = (IONetworkStats *) data->getBuffer())) {
		return false;
	}
	
	// Get the Ethernet statistics structure.
	
	data = netif->getParameter(kIOEthernetStatsKey);
	if (!data || !(etherStats = (IOEthernetStats *) data->getBuffer())) {
		return false;
	}
	
	return true;
}

//---------------------------------------------------------------------------
// Function: start <IOService>
//
// Hardware was detected and initialized, start the driver.

bool ViaVelocity::start( IOService * provider )
{
    bool ret     = false;
    bool started = false;
	
	//IOLog("%s: start()\n", getName());
	do {
		// Start our superclass first.
		
		if ( super::start(provider) == false )
			break;
		
		started = true;
		
		// Cache our provider to an instance variable.

		pciNub = OSDynamicCast(IOPCIDevice, provider);
		if ( pciNub == 0 ) break;
		
		// Retain provider, released in free().
		
		pciNub->retain();
		
		// Open our provider.
		if ( pciNub->open(this) == false ) break;
		
		// Initialize the driver's event sources and other support objects.
		
		if ( initDriver(provider) == false ){
			IOLog("initDriver() failed.\n");
			break;
		}
		
		// Setup our PCI config space.
		
		if ( pciConfigInit(pciNub) == false ){
			IOLog("pciConfigInit() failed.\n");
			break;
		}

		
		velocity.hw.nTxQueues = 1;
		velocity.hw.multicast_limit = MCAM_SIZE;
		// velocity.hw.io_size = 256;

		velocity.hw.msglevel = MSG_LEVEL_INFO;

		mac_wol_reset(&velocity.hw);

		// get MII PHY Id
		velocity_mii_read(&velocity.hw, MII_REG_PHYID2, (PU16)&velocity.hw.dwPHYId);
		velocity_mii_read(&velocity.hw, MII_REG_PHYID1, ((PU16)&velocity.hw.dwPHYId)+1);

		// software reset
		velocity_soft_reset(&velocity.hw);
		mdelay(5);

		//david add
		mii_reset(&velocity.hw);
		mdelay(5);

		// EEPROM reload
		mac_eeprom_reload(&velocity.hw);

		// copy MAC
		for (int i = 0; i < 6; i++)
			myAddress.bytes[i] = CSR_READ_1(&velocity.hw, MAC_REG_PAR+i);

		// Get default driver settings (stored in property table).
		getDefaultSettings();
		
		/*
		 *	Enable the chip specified capbilities
		 */
		
		velocity.wol_opts = velocity.hw.sOpts.wol_opts;
#if		0
		velocity.hw.flags |= VELOCITY_FLAGS_WOL_ENABLED;
#endif
        // Publish our media capabilities.
        _phyPublishMedia();
        if ( publishMediumDictionary(mediumDict) == false )
        {
            IOLog("%s: publishMediumDictionary failed\n", getName());
			break;
        }

		velocity_change_mtu(ETH_DATA_LEN);	// set mtu, velocity.hw.rx_buf_sz, alloc rxMbufCursor

		ret = true;
	} while ( false );
	
    // Close our provider, it will be re-opened on demand when
    // our enable() method is called.
	
    if ( pciNub ) pciNub->close(this);
	
    do {
        if ( ret == false ) break;
		
        ret = false;
		
        // Allocate and attach an IOEthernetInterface instance to this driver
        // object.
		
        if ( attachInterface((IONetworkInterface **) &netif, false) == false )
            break;
		
        // Attach a kernel debugger client. This is not an essential service,
        // and the return is not checked.
		
        attachDebuggerClient(&debugger);
		
        // Start matching for clients of IONetworkInterface.
		
        netif->registerService();
        
        ret = true;
    }
    while ( false );
	
    // Issue a stop on failure.
	
    if (started && !ret) super::stop(provider);
	
    return ret;
}

//---------------------------------------------------------------------------
// Function: stop <IOService>
//
// Stop all activities and prepare for termination.

void ViaVelocity::stop(IOService * provider)
{
	if ( interruptSrc && workLoop ){
		workLoop->removeEventSource( interruptSrc );
	}
	
    super::stop(provider);
}

//---------------------------------------------------------------------------
// Function: enable <IONetworkController>
//
// A request from our interface client to enable the adapter.

IOReturn ViaVelocity::enable(IONetworkInterface * /*netif*/)
{
	//IOLog("%s::%s()\n", getName(),__FUNCTION__);
	if ( velocity.hw.flags & VELOCITY_FLAGS_OPENED )
		return kIOReturnSuccess;
	
	pciNub->open(this);

    if (!velocity_init_rings())
        return kIOReturnNoMemory;

    if (!velocity_init_rd_ring())
        return kIOReturnNoMemory;

    if (!velocity_init_td_ring())
        return kIOReturnNoMemory;

	velocity_init_adapter(VELOCITY_INIT_COLD);
	
	
	// Set current medium.
	//
	selectMedium(getCurrentMedium());

	// Start our IOOutputQueue object.
	//
	transmitQueue->start();
	netif_start_queue();
	
	// initial timer
	prevStat = VELOCITY_LINK_FAIL;
	timerSrc->setTimeoutMS(LOAD_STATISTICS_INTERVAL);
	
    mac_enable_int(&velocity.hw);
	
	velocity.hw.flags |= VELOCITY_FLAGS_OPENED;
	
	return kIOReturnSuccess;
}

//---------------------------------------------------------------------------
// Function: disable <IONetworkController>
//
// A request from our interface client to disable the adapter.

IOReturn ViaVelocity::disable(IONetworkInterface * /*netif*/)
{
	//IOLog("%s::%s()\n", getName(),__FUNCTION__);
	velocity.hw.flags &= (~VELOCITY_FLAGS_OPENED);

	timerSrc->cancelTimeout();

	// Report link status: valid and down.
	//
	setLinkStatus( kIONetworkLinkValid );
	
	// Flush all packets held in the queue and prevent it
	// from accumulating any additional packets.
	transmitQueue->stop();
	transmitQueue->flush();
	
	velocity_shutdown(&velocity.hw);

	if (velocity.hw.flags & VELOCITY_FLAGS_WOL_ENABLED){
		velocity_get_ip(&velocity);
		velocity_set_wol(&velocity);
	}

	velocity_free_td_ring();
	velocity_free_rd_ring();
	velocity_free_rings();
	
	velocity_free_rings();
	
	
	pciNub->close(this);
	
	return kIOReturnSuccess;
}

/**
 *	velocity_free_tx_buf	-	free transmit buffer
 *	@vptr: velocity
 *	@tdinfo: buffer
 *
 *	Release an transmit buffer. If the buffer was preallocated then
 *	recycle it, if not then unmap the buffer.
 */

void ViaVelocity::velocity_free_tx_buf( PVELOCITY_TD_INFO pTDInfo )
{
	if(pTDInfo->skb){
		freePacket(pTDInfo->skb);
		pTDInfo->skb = 0;
	}
	if(pTDInfo->pool){
		pTDInfo->pool->complete();
		pTDInfo->pool->release();
		pTDInfo->pool = 0;
	}
}

/**
 *	velocity_free_rd_ring	-	free receive ring
 *	@vptr: velocity to clean up
 *
 *	Free the receive buffers for each ring slot and any
 *	attached socket buffers that need to go away.
 */

void ViaVelocity::velocity_free_rd_ring()
{
	int i;
	PVELOCITY_RD_INFO  pRDInfo;
	
	if (velocity.aRDInfo == NULL)
		return;
	
	for (i = 0; i < velocity.hw.sOpts.nRxDescs; i++) {
		pRDInfo = &(velocity.aRDInfo[i]);
#if USE_RX_BUFFER
		if (pRDInfo->pool) {
			pRDInfo->pool->complete();
			pRDInfo->pool->release();
            pRDInfo->pool=NULL;
		}
#else
		if (pRDInfo->skb) {
            freePacket(pRDInfo->skb);
            pRDInfo->skb=NULL;
		}
#endif
	}
	
    if (velocity.aRDInfo)
        IOFree(velocity.aRDInfo, sizeof(VELOCITY_RD_INFO)*velocity.hw.sOpts.nRxDescs);
    velocity.aRDInfo = NULL;
}

/**
 *	velocity_init_rd_ring	-	set up receive ring
 *	@vptr: velocity to configure
 *
 *	Allocate and set up the receive buffers for each ring slot and
 *	assign them to the network adapter.
 */

int ViaVelocity::velocity_init_rd_ring()
{
    int i;
    PRX_DESC        pDesc;
    PVELOCITY_RD_INFO  pRDInfo;
	size_t size = sizeof(VELOCITY_RD_INFO)*velocity.hw.sOpts.nRxDescs;

    velocity.aRDInfo = (PVELOCITY_RD_INFO)IOMalloc(size);
	if(velocity.aRDInfo ==NULL){
		IOLog("velocity.aRDInfo alloc failed (%d).\n", (int)size);
		return FALSE;
	}
	bzero(velocity.aRDInfo, size);

    /* Init the RD ring entries */
    for (i = 0; i < velocity.hw.sOpts.nRxDescs; i++) {
        pDesc=&(velocity.hw.aRDRing[i]);
        pRDInfo=&(velocity.aRDInfo[i]);
		
        if (!velocity_alloc_rx_buf(i)) {
            IOLog("%s: can not alloc rx bufs\n", getName());
            return FALSE;
        }
        pDesc->rdesc0 |= cpu_to_le32(RDESC0_OWN);
    }
	
    velocity.hw.iCurrRDIdx = 0;
    return TRUE;
}

/**
 *	velocity_init_td_ring	-	set up transmit ring
 *	@vptr:	velocity
 *
 *	Set up the transmit ring and chain the ring pointers together.
 *	Returns zero on success or a negative posix errno code for
 *	failure.
 */

BOOL ViaVelocity::velocity_init_td_ring()
{
    int i,j;
	
    /* Init the TD ring entries */
    for (j=0;j<velocity.hw.nTxQueues;j++) {
        velocity.apTDInfos[j] = (PVELOCITY_TD_INFO)IOMalloc(sizeof(VELOCITY_TD_INFO)*velocity.hw.sOpts.nTxDescs);
		
        bzero(velocity.apTDInfos[j], sizeof(VELOCITY_TD_INFO)*velocity.hw.sOpts.nTxDescs);
		U8* vadr = (U8*)velocity.tx_bufs[j]->getBytesNoCopy();
		dma_addr_t padr = velocity.tx_bufs[j]->getPhysicalAddress();
        for (i = 0; i < velocity.hw.sOpts.nTxDescs; i++) {
            velocity.apTDInfos[j][i].buf= vadr;
            velocity.apTDInfos[j][i].buf_dma=padr;
			vadr += PKT_BUF_SZ;
			padr += PKT_BUF_SZ;
        }
        velocity.hw.aiTailTDIdx[j]=velocity.hw.aiCurrTDIdx[j]=velocity.hw.iTDUsed[j]=0;
    }
    return TRUE;
}


/**
 *	velocity_free_td_ring	-	free td ring
 *	@vptr: velocity
 *
 *	Free up the transmit ring for this particular velocity adapter.
 *	We free the ring contents but not the ring itself.
 */

void ViaVelocity::velocity_free_td_ring()
{
    int i, j;
	
    for (j=0; j<velocity.hw.nTxQueues; j++) {
		PVELOCITY_TD_INFO apTDInfo = velocity.apTDInfos[j];
        if (apTDInfo){

			for (i = 0; i < velocity.hw.sOpts.nTxDescs; i++) {
				velocity_free_tx_buf(&(apTDInfo[i]));
			}
			
			IOFree(velocity.apTDInfos[j], sizeof(VELOCITY_TD_INFO)*velocity.hw.sOpts.nTxDescs);
			velocity.apTDInfos[j] = NULL;
		}
    }
}


BOOL ViaVelocity::velocity_init_rings()
{
	int     i;
	
	size_t size;
    /*allocate all RD/TD rings a single pool*/
	size = velocity.hw.sOpts.nRxDescs * sizeof(RX_DESC) + velocity.hw.sOpts.nTxDescs * sizeof(TX_DESC)*velocity.hw.nTxQueues;
	velocity.pool = allocPool(size);
	
	if (velocity.pool == NULL) {
        IOLog("%s : allocate dma memory failed\n",getName());
        return FALSE;
	}
	velocity.pool->prepare();
	velocity.hw.pool_dma = velocity.pool->getPhysicalAddress();
	PU8 vadrs = (PU8)velocity.pool->getBytesNoCopy();

	bzero(vadrs, size);
	
	velocity.hw.rd_pool_dma = velocity.hw.pool_dma;
	velocity.hw.aRDRing = (PRX_DESC)vadrs;

	dma_addr_t td_pool_dma = velocity.hw.rd_pool_dma + velocity.hw.sOpts.nRxDescs*sizeof(RX_DESC);
	PTX_DESC pTDRings = (PTX_DESC)(vadrs + velocity.hw.sOpts.nRxDescs*sizeof(RX_DESC));
	for (i=0;i<velocity.hw.nTxQueues;i++) {
		velocity.hw.td_pool_dma[i]= td_pool_dma;
		velocity.hw.apTDRings[i]= pTDRings;
		
		td_pool_dma += sizeof(TX_DESC)*velocity.hw.sOpts.nTxDescs;
		pTDRings += velocity.hw.sOpts.nTxDescs;
	}

	// pre-allocate tx buffer
	size = velocity.hw.sOpts.nTxDescs * PKT_BUF_SZ*velocity.hw.nTxQueues;
	for (i=0;i<velocity.hw.nTxQueues;i++) {
		velocity.tx_bufs[i] = allocPool(size);
		if (velocity.tx_bufs[i] == NULL) {
			IOLog("%s: allocate dma memory failed\n", getName());
			velocity.pool->complete();
			velocity.pool->release();
			velocity.pool = NULL;
			for(int j = 0; j < i; j++ ){
				velocity.tx_bufs[i]->complete();
				velocity.tx_bufs[i]->release();
				velocity.tx_bufs[i] = NULL;
			}
			return FALSE;
		}
		velocity.tx_bufs[i]->prepare();
		bzero(velocity.tx_bufs[i]->getBytesNoCopy(), size);
	}
	
	return TRUE;
}

void ViaVelocity::velocity_free_rings()
{
	if(velocity.pool){
		velocity.pool->complete();
		velocity.pool->release();
		velocity.pool = NULL;
	}
	
	for (int i=0;i<velocity.hw.nTxQueues;i++) {
		if (velocity.tx_bufs[i]){
			velocity.tx_bufs[i]->complete();
			velocity.tx_bufs[i]->release();
			velocity.tx_bufs[i] = NULL;
		}
	}
}


/**
 *	velocity_alloc_rx_buf	-	allocate aligned receive buffer
 *	@vptr: velocity
 *	@idx: ring index
 *
 *	Allocate a new full sized buffer for the reception of a frame and
 *	map it into PCI space for the hardware to use. The hardware
 *	requires *64* byte alignment of the buffer which makes life
 *	less fun than would be ideal.
 */

BOOL ViaVelocity::velocity_alloc_rx_buf(int idx)
{
	PRX_DESC        pRD=&(velocity.hw.aRDRing[idx]);
	PVELOCITY_RD_INFO  pRDInfo=&(velocity.aRDInfo[idx]);
	struct IOPhysicalSegment vector;

	// already allocated ?
#if USE_RX_BUFFER
	if (pRDInfo->pool)
		return TRUE;

    pRDInfo->pool = allocPool( velocity.hw.rx_buf_sz, RXBUF_MASK ); // 32-bit, 64-bytes boundary

	if (pRDInfo->pool == NULL)
		return FALSE;

	pRDInfo->pool->prepare();
	vector.location = pRDInfo->pool->getPhysicalAddress();
#else
	if (pRDInfo->skb)
		return TRUE;
	
    pRDInfo->skb = velocity_alloc_packet(velocity.hw.rx_buf_sz);
	if (pRDInfo->skb == NULL)
		return FALSE;
	
	int count = rxMbufCursor->getPhysicalSegmentsWithCoalesce(pRDInfo->skb, &vector, 1);
	if(count == 0){
		freePacket(pRDInfo->skb);
		return FALSE;
	}
#endif

    *((PU32)&(pRD->rdesc0)) = 0;

    VELOCITY_SET_RD_BUFFER_SIZE(pRD, velocity.hw.rx_buf_sz);
    pRD->rdesc3 |= cpu_to_le32(RDESC3_INTCTL);
    pRD->dwBufAddrLo = cpu_to_le32(vector.location);
    // mask off RD data buffer address high to zero
    pRD->rdesc3 &= cpu_to_le32(0xffff0000L);
	
    return TRUE;	
}


//---------------------------------------------------------------------------
// Function: _phyGetMediumWithCode
//
// Purpose:
//   Returns the IONetworkMedium object associated with the given type.

IONetworkMedium * ViaVelocity::_phyGetMediumWithType(UInt32 type)
{
	if (type < MEDIUM_TYPE_INVALID)
		return mediumTable[type];
	else
		return 0;
}


//---------------------------------------------------------------------------
// Function: _phyAddMediumType
//
// Purpose:
//   Add a single medium object to the medium dictionary.
//   Also add the medium object to an array for fast lookup.

bool ViaVelocity::_phyAddMediumType(UInt32 type, UInt32 speed, UInt32 code)
{	
	IONetworkMedium	* medium;
	bool              ret = false;
	
	medium = IONetworkMedium::medium(type, speed, 0, code);
	if (medium) {
		ret = IONetworkMedium::addMedium(mediumDict, medium);
		if (ret)
			mediumTable[code] = medium;
		medium->release();
	}
	return ret;
}

//---------------------------------------------------------------------------
// Function: _phyPublishMedia
//
// Purpose:
//   Examine the PHY capabilities and advertise all supported medium types.
//
// FIXME: Non PHY medium types are not probed.

#define MBPS 1000000 

void ViaVelocity::_phyPublishMedia()
{
	
	_phyAddMediumType(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO);
	
	_phyAddMediumType(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex,10 * MBPS, MEDIUM_INDEX_10HD);
	_phyAddMediumType(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex,10 * MBPS, MEDIUM_INDEX_10FD);
	_phyAddMediumType(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex,100 * MBPS, MEDIUM_INDEX_100HD);
	_phyAddMediumType(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex,100 * MBPS, MEDIUM_INDEX_100FD);
	_phyAddMediumType(kIOMediumEthernet1000BaseT,1000 * MBPS, MEDIUM_INDEX_1000FD);
}


bool ViaVelocity::_phySetMedium(mediumType_t medium)
{
	if(medium == MEDIUM_INDEX_AUTO){
		velocity.hw.sOpts.spd_dpx = SPD_DPX_AUTO;
	} else {
		if(medium == MEDIUM_INDEX_100FD){
			velocity.hw.sOpts.spd_dpx = SPD_DPX_100_FULL;
		} else if(medium == MEDIUM_INDEX_100HD){
			velocity.hw.sOpts.spd_dpx = SPD_DPX_100_HALF;
		} else if(medium == MEDIUM_INDEX_10FD){
			velocity.hw.sOpts.spd_dpx = SPD_DPX_10_FULL;
		} else if(medium == MEDIUM_INDEX_10HD){
			velocity.hw.sOpts.spd_dpx = SPD_DPX_10_HALF;
		}
	}
	int stat = velocity_set_media_mode(&velocity.hw, velocity.hw.sOpts.spd_dpx);
	return stat == VELOCITY_LINK_CHANGE;
}

//---------------------------------------------------------------------------
// Function: setPromiscuousMode <IOEthernetController>

IOReturn ViaVelocity::setPromiscuousMode( bool active )
{
	if(active)
		flags |= IFF_PROMISC;
	else
		flags &= ~IFF_PROMISC;

	velocity_set_multi();
	return kIOReturnSuccess;
}

//---------------------------------------------------------------------------
// Function: setMulticastMode <IOEthernetController>

IOReturn ViaVelocity::setMulticastMode( bool active )
{
	if(active)
		flags |= IFF_MULTICAST;
	else
		flags &= ~IFF_MULTICAST;
	velocity_set_multi();
	return kIOReturnSuccess;
}

//---------------------------------------------------------------------------
// Function: setMulticastList <IOEthernetController>

IOReturn ViaVelocity::setMulticastList(IOEthernetAddress * addrs, UInt32 count)
{
	
	int offset = MCAM_SIZE - velocity.hw.multicast_limit;
	mac_get_cam_mask(&velocity.hw,velocity.hw.abyMCAMMask, VELOCITY_MULTICAST_CAM);
	for (UInt32 i = 0; i < count; i++) {
		mac_set_cam(&velocity.hw, i + offset, addrs[i].bytes, VELOCITY_MULTICAST_CAM);
		velocity.hw.abyMCAMMask[(offset + i) / 8] |= 1 << ((offset + i) & 7);
	}
	
	mac_set_cam_mask(&velocity.hw,velocity.hw.abyMCAMMask,VELOCITY_MULTICAST_CAM);
	
	return kIOReturnSuccess;
}

//---------------------------------------------------------------------------
// Function: getPacketBufferConstraints <IONetworkController>
//
// Return our driver's packet alignment requirements.

void
ViaVelocity::getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const
{
	constraints->alignStart  = kIOPacketBufferAlign32;
	constraints->alignLength = kIOPacketBufferAlign32;
}

//---------------------------------------------------------------------------
// Function: getChecksumSupport <IONetworkController>
//
// Return our driver's checksum capability.

IOReturn ViaVelocity::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) 
{
	if( checksumFamily != kChecksumFamilyTCPIP )
		return kIOReturnUnsupported;
	
	*checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;
	if( isOutput ) {
		if((velocity.hw.flags & VELOCITY_FLAGS_TX_CSUM) == 0)
			*checksumMask = 0;
	}
	return kIOReturnSuccess;
}


//---------------------------------------------------------------------------
// Function: getHardwareAddress <IOEthernetController>
//
// Return the adapter's hardware/Ethernet address.

IOReturn ViaVelocity::getHardwareAddress(IOEthernetAddress * addrs)
{
	*addrs = myAddress;
	return kIOReturnSuccess;
}

//---------------------------------------------------------------------------
// Function: setHardwareAddress <IOEthernetController>
//
// Change the address filtered by the unicast filter.

IOReturn ViaVelocity::setHardwareAddress( const IOEthernetAddress * addr )
{
	myAddress = *addr;
	mac_eeprom_reload(&velocity.hw);
	for (int i = 0; i < 6; i++) {
		CSR_WRITE_1(&velocity.hw, myAddress.bytes[i], MAC_REG_PAR+i);
	}
	
	return kIOReturnSuccess;
}

IOReturn ViaVelocity::getMaxPacketSize (UInt32 *maxSize) const {
	*maxSize = maxPacketSize;
	return kIOReturnSuccess;
}

IOReturn ViaVelocity::getMinPacketSize (UInt32 *minSize) const {
	return super::getMinPacketSize(minSize);
}

IOReturn ViaVelocity::setMaxPacketSize (UInt32 maxSize){
	if(maxSize == mtu + (ETH_HLEN + ETH_FCS_LEN))
		return kIOReturnSuccess;
	
	if(velocity.hw.flags & VELOCITY_FLAGS_OPENED){
		IOLog("%s::%s(%d) failed.",getName(),__FUNCTION__,(int)maxSize);
		;
	} else {
		velocity_change_mtu(maxSize - (ETH_HLEN + ETH_FCS_LEN));
	}
	
	return kIOReturnSuccess;
}

UInt32 ViaVelocity::getFeatures() const {
	UInt32 features = kIONetworkFeatureMultiPages;
	if(velocity.hw.sOpts.tagging)
		features |= kIONetworkFeatureHardwareVlan;
	return features;
}

//---------------------------------------------------------------------------
// Function: createOutputQueue <IONetworkController>
//
// Allocate an IOGatedOutputQueue instance.

IOOutputQueue * ViaVelocity::createOutputQueue()
{	
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

//---------------------------------------------------------------------------
// Function: createWorkLoop <IONetworkController>
//
// Override IONetworkController::createWorkLoop() method to create and return
// a new work loop object.

bool ViaVelocity::createWorkLoop()
{
	workLoop = IOWorkLoop::workLoop();
	
	return ( workLoop != 0 );
}

//---------------------------------------------------------------------------
// Function: getWorkLoop <IOService>
//
// Override IOService::getWorkLoop() method and return a reference to our
// work loop.

IOWorkLoop * ViaVelocity::getWorkLoop() const
{
	return workLoop;
}



//---------------------------------------------------------------------------
// Function: selectMedium <IONetworkController>
//
// Transition the controller/PHY to use a new medium. Note that
// this function can be called my the driver, or by our client.

IOReturn ViaVelocity::selectMedium(const IONetworkMedium * medium)
{
	
	//IOLog("%s::%s(%p)\n",getName(),__FUNCTION__,medium);
	if ( OSDynamicCast(IONetworkMedium, medium) == 0 )
	{
		// Defaults to Auto.
		medium = _phyGetMediumWithType( MEDIUM_INDEX_AUTO );
		if ( medium == 0 ) return kIOReturnError;
	}
	
	// Program PHY to select the desired medium.
	//
	_phySetMedium( (mediumType_t) medium->getIndex() );
	
	// Update the current medium property.
	//
	if (  !setSelectedMedium(medium) ){
		IOLog("%s::%s error\n", getName(), __FUNCTION__);
		return kIOReturnIOError;
	}
	return kIOReturnSuccess;
}


/**
 *	velocity_set_multi	-	filter list change callback
 *	@dev: network device
 *
 *	Called by the network layer when the filter lists need to change
 *	for a velocity adapter. Reload the CAMs with the new address
 *	filter ruleset.
 */

void ViaVelocity::velocity_set_multi()
{
	U8 rx_mode;
	
	if (flags & IFF_PROMISC) {	/* Set promiscuous. */
		velocity_set_all_multi(&velocity.hw);
		rx_mode = (RCR_AM|RCR_AB|RCR_PROM);
	} else if (flags & IFF_ALLMULTI) {
		velocity_set_all_multi(&velocity.hw);
		rx_mode = (RCR_AM|RCR_AB);
	} else {
		velocity_clear_all_multi(&velocity.hw);
		rx_mode = (RCR_AP|RCR_AM|RCR_AB);
	}
	if (mtu > 1500)
		rx_mode |= RCR_AL;
	
	BYTE_REG_BITS_ON(&velocity.hw, rx_mode, MAC_REG_RCR);
}

/**
 *	velocity_change_mtu	-	MTU change callback
 *	@dev: network device
 *	@new_mtu: desired MTU
 *
 *	Handle requests from the networking layer for MTU change on
 *	this interface. It gets called on a change by the network layer.
 *	Return zero for success or negative posix error code.
 */

int ViaVelocity::velocity_change_mtu(int new_mtu)
{
	if (mtu == new_mtu) {
		return 0;
	}

	// assume que is not running
	mtu = new_mtu;
	if (new_mtu>8192)
		velocity.hw.rx_buf_sz=9*1024;
	else if (new_mtu>4096)
		velocity.hw.rx_buf_sz=8192;
	else if (new_mtu>2048)
		velocity.hw.rx_buf_sz=4*1024;
	else
		velocity.hw.rx_buf_sz=2*1024;

#if	USE_RX_BUFFER
#else
	RELEASE(rxMbufCursor);
	rxMbufCursor = IOMbufLittleMemoryCursor::withSpecification(velocity.hw.rx_buf_sz, 1);
#endif
	return 0;
}



//---------------------------------------------------------------------------
// Function: outputPacket <IONetworkController>
//
// Purpose:
//   Transmit the packet handed by our IOOutputQueue.
//   TCBs have the suspend bit set, so that the CU goes into the suspend
//   state when done.  We use the CU_RESUME optimization that allows us to
//   issue CU_RESUMES without waiting for SCB command to clear.
//
UInt32 ViaVelocity::outputPacket(mbuf_t skb, void * param)
{
    int                 iQNo = 0;
    PTX_DESC            pTD;
    PVELOCITY_TD_INFO   pTDInfo;
    int                 iCurrTDIdx;
    int                 iPrevTDIdx;
	
	if (!netif_running()){
		freePacket(skb);
		netStats->outputErrors += 1;
		return kIOReturnOutputDropped;
	}

#ifdef VELOCITY_TSO_SUPPORT
    unsigned int        mss = tcp_skb_mss(skb);
#endif
	UInt32 vlanTag;
	if(!getVlanTagDemand(skb, &vlanTag))
		vlanTag = 0;
	UInt32 checksumDemanded;
	getChecksumDemand(skb, kChecksumFamilyTCPIP, &checksumDemanded);

#if	NO_LOCK_FREE
	IOSimpleLockLock(velocity.lock);
#endif
    iCurrTDIdx = velocity.hw.aiCurrTDIdx[iQNo];
    pTD = &(velocity.hw.apTDRings[iQNo][iCurrTDIdx]);
    pTDInfo = &(velocity.apTDInfos[iQNo][iCurrTDIdx]);
	
    // Init TDESC0,1
    pTD->tdesc0 = 0x00000000UL;
    pTD->tdesc1 = 0x00000000UL;
	
	VELOCITY_SET_TD_TCPLS(pTD, TCPLS_NORMAL);
	pTD->tdesc1 |= cpu_to_le32(TCR_TIC);
	pTD->aTdBufs[0].dwBufaddrHi &= cpu_to_le32(~TDTXBUF_QUE);
	
	size_t pktlen = mbuf_pkthdr_len(skb);
	PU8 buf;
	dma_addr_t buf_dma;
	if(pktlen > PKT_BUF_SZ){
		// packet size is bigger than pre-allocated buf
		IOLog("%s::%s, packet = %d > %d",getName(),__FUNCTION__,(int)pktlen,PKT_BUF_SZ);
		pTDInfo->pool = allocPool(pktlen);
		if(pTDInfo->pool == NULL){
			freePacket(skb);
			netStats->outputErrors += 1;
			return kIOReturnOutputDropped;
		}
		pTDInfo->pool->prepare();
		buf = (PU8)pTDInfo->pool->getBytesNoCopy();
		buf_dma = pTDInfo->pool->getPhysicalAddress();
	} else {
		// packet size is less than pre-allocated buf
		buf = pTDInfo->buf;
		buf_dma = pTDInfo->buf_dma;		
	}
	pTDInfo->skb = skb;
	pktlen = 0;
	// copy mbuf data
	do {
		size_t len = mbuf_len(skb);
		bcopy( mbuf_data(skb), buf+pktlen, len );
		pktlen += len;
	} while ( (skb = mbuf_next(skb)) != 0 );
	// padding
	if(pktlen < ETH_ZLEN){
		// padding zero
		bzero(buf+pktlen, ETH_ZLEN-pktlen);
		pktlen = ETH_ZLEN;
	}
	VELOCITY_SET_TD_PACKET_SIZE(pTD, pktlen);
	pTD->aTdBufs[0].dwBufAddrLo = cpu_to_le32(buf_dma);
	// mask off TD data buffer address high to zero
	pTD->aTdBufs[0].dwBufaddrHi &= cpu_to_le32(0xffff0000L);
	VELOCITY_SET_TD_BUFFER_SIZE(pTD->aTdBufs[0], pktlen);
	// pTDInfo->nskb_dma = 1;
	VELOCITY_SET_TD_CMDZ(pTD, 2);

	if(vlanTag) {
        // clear CFI and priority
        pTD->tdesc1 &= cpu_to_le32(0xffff0000L);
        VELOCITY_SET_TD_VLANID(pTD, vlanTag & 0xfff);
        pTD->tdesc1 |= cpu_to_le32(TCR_VETAG);
    }

    if ( checksumDemanded & (kChecksumIP|kChecksumTCP|kChecksumUDP) ) {
		
        if (checksumDemanded & kChecksumTCP) {
            // request TCP checksum calculation
            pTD->tdesc1 |= cpu_to_le32(TCR_TCPCK);
        }
        else if (checksumDemanded & kChecksumUDP) {
            // request UDP checksum calculation
            pTD->tdesc1 |= cpu_to_le32(TCR_UDPCK);
        }
		
        // request IP checksum calculation
        pTD->tdesc1 |= cpu_to_le32(TCR_IPCK);
    }
	
    // Set OWN bit of current TD
    pTD->tdesc0 |= cpu_to_le32(TDESC0_OWN);
	
    velocity.hw.iTDUsed[iQNo]++;
    velocity.hw.aiCurrTDIdx[iQNo] = (iCurrTDIdx + 1) % velocity.hw.sOpts.nTxDescs;
	
	if (AVAIL_TD(&velocity.hw, iQNo) < 2)
		netif_stop_queue();

	iPrevTDIdx = (iCurrTDIdx + velocity.hw.sOpts.nTxDescs - 1) % velocity.hw.sOpts.nTxDescs;
	pTD = &(velocity.hw.apTDRings[iQNo][iPrevTDIdx]);
	pTD->aTdBufs[0].dwBufaddrHi |= cpu_to_le32(TDTXBUF_QUE);
	
	mac_tx_queue_wake(&velocity.hw, iQNo);

#if	NO_LOCK_FREE
	IOSimpleLockUnlock(velocity.lock);
#endif
	return kIOReturnOutputSuccess;
}
/**
 *	velocity_error	-	handle error from controller
 *	@vptr: velocity
 *	@status: card status
 *
 *	Process an error report from the hardware and attempt to recover
 *	the card itself. At the moment we cannot recover from some
 *	theoretically impossible errors but this could be fixed using
 *	the pci_device_failed logic to bounce the hardware
 *
 */

void ViaVelocity::velocity_error(int status)
{
	
	// (1) LSTEI
	if (status & ISR_TXSTLI) {
		IOLog("TD structure errror TDindex=%X\n", CSR_READ_2(&velocity.hw, MAC_REG_TDIDX0));
		
		BYTE_REG_BITS_ON(&velocity.hw, TXESR_TDSTR, MAC_REG_TXE_SR);
		CSR_WRITE_2(&velocity.hw, TRDCSR_RUN, MAC_REG_TDCSR_CLR);
		transmitQueue->stop();
	}
	
	// (2) SRCI
	if (status & ISR_SRCI) {
		
		if (velocity.hw.sOpts.spd_dpx == SPD_DPX_AUTO) {
			velocity.hw.mii_status = check_connectiontype(&velocity.hw);
			
			// if it's 3119, disable frame bursting in halfduplex mode and
			// enable it in fullduplex mode
			if (velocity.hw.byRevId < REV_ID_VT3216_A0) {
				if (velocity.hw.mii_status | VELOCITY_DUPLEX_FULL)
            		BYTE_REG_BITS_ON(&velocity.hw, TCR_TB2BDIS, MAC_REG_TCR);
				else
					BYTE_REG_BITS_OFF(&velocity.hw, TCR_TB2BDIS, MAC_REG_TCR);
			}
			
			/*
			 * only enable CD heart beat counter in 10HD mode
			 * This PATCH is for VT3119/VT3216
			 */
			if (velocity.hw.byRevId < REV_ID_VT3284_A0) {
				if (!(velocity.hw.mii_status & VELOCITY_DUPLEX_FULL) && (velocity.hw.mii_status & VELOCITY_SPEED_10)) {
					BYTE_REG_BITS_OFF(&velocity.hw, TESTCFG_HBDIS, MAC_REG_TESTCFG);
				}
				else {
					BYTE_REG_BITS_ON(&velocity.hw, TESTCFG_HBDIS, MAC_REG_TESTCFG);
				}
			}
			
			//--------------------------------------------------------
			// [1.18] Adaptive Interrupt
			if (velocity.hw.byRevId >= REV_ID_VT3216_A0) {
				if (velocity.hw.mii_status & VELOCITY_SPEED_1000) {
					if (velocity.hw.flags & VELOCITY_FLAGS_AI) {
						CSR_WRITE_1(&velocity.hw, velocity.hw.sOpts.txque_timer, MAC_REG_TQETMR);
						CSR_WRITE_1(&velocity.hw, velocity.hw.sOpts.rxque_timer, MAC_REG_RQETMR);
					}
					else {
						CSR_WRITE_1(&velocity.hw, 0x00, MAC_REG_TQETMR);
						CSR_WRITE_1(&velocity.hw, 0x00, MAC_REG_RQETMR);
					}
				}
				else {
					CSR_WRITE_1(&velocity.hw, 0x00, MAC_REG_TQETMR);
					CSR_WRITE_1(&velocity.hw, 0x00, MAC_REG_RQETMR);
				}
			}
			//--------------------------------------------------------
		}
		
		// get link status from PHYSR0
		
        velocity.hw.mii_status = check_connectiontype(&velocity.hw);
		
		// velocity_print_link_status(&velocity.hw);
	    enable_flow_control_ability(&velocity.hw);
		
		// re-enable auto-polling because SRCI will disable auto-polling
		EnableMiiAutoPoll(&velocity.hw);
#if	1
		if (velocity.hw.mii_status & VELOCITY_LINK_FAIL) {
			netif_stop_queue();
		} else {
			netif_start_queue();
		}
#endif
    } // ISR_SRCI
	
    // (3) MIBFI
    if (status & ISR_MIBFI)
        velocity_update_hw_mibs(&velocity.hw);
	
    // (4) LSTEI: RD used up, re-wake RD ring
    if (status & ISR_LSTEI) {
        mac_rx_queue_wake(&velocity.hw);
    }
}


void ViaVelocity::velocity_rx_csum(PRX_DESC pRD, mbuf_t skb)
{
	UInt32 ckResult = 0;
	U8  byCSM = (U8)(cpu_to_le32(pRD->rdesc1) >> 16);
	
	if ((byCSM & CSM_IPKT) && (byCSM & CSM_IPOK)) {
		ckResult |= kChecksumIP;
		if (byCSM & CSM_TUPOK){
			if (byCSM & CSM_TCPKT)
				ckResult |= kChecksumTCP;
			if (byCSM & CSM_UDPKT)
				ckResult |= kChecksumTCP;
		}
		setChecksumResult(skb, kChecksumFamilyTCPIP, kChecksumIP | kChecksumTCP | kChecksumUDP, ckResult);
	}
}

/**
 *	velocity_receive_frame	-	received packet processor
 *	@vptr: velocity we are handling
 *	@idx: ring index
 *
 *	A packet has arrived. We process the packet and if appropriate
 *	pass the frame up the network stack
 */

BOOL ViaVelocity::velocity_receive_frame(int idx)
{
    PVELOCITY_RD_INFO   pRDInfo = &(velocity.aRDInfo[idx]);
    PRX_DESC            pRD = &(velocity.hw.aRDRing[idx]);
    mbuf_t     skb;
    U16                 wRSR, wLength;
	
    wRSR = (U16)(cpu_to_le32(pRD->rdesc0));
    wLength = VELOCITY_GET_RD_PACKET_SIZE(pRD);
	
	if (wRSR & (RSR_STP|RSR_EDP)) {
		IOLog("%s: the received frame span multple RDs\n", getName());
		etherStats->dot3RxExtraEntry.frameTooShorts += 1;
		return FALSE;
	}
	/***
    if (wRSR & RSR_MAR)
        pInfo->stats.multicast++;
	
    if (wRSR & RSR_BAR)
        pInfo->adwRMONStats[RMON_BroadcastPkts]++;
	 ***/
#if	USE_RX_BUFFER
	skb = allocatePacket(wLength);
	if(skb == NULL){
		netStats->inputErrors += 1;
		return FALSE;
	}
	mbuf_copyback(skb, 0, wLength, pRDInfo->pool->getBytesNoCopy(), MBUF_WAITOK);
#else
	skb = pRDInfo->skb;

	if(wLength < 256){
		mbuf_t m_in = copyPacket(skb, wLength);
		if(m_in){
			//mbuf_setlen(skb, 0);
			skb = m_in;
		} else {
			pRDInfo->skb = NULL;
		}
	} else {
		pRDInfo->skb = NULL;
	}
#endif

	
	//drop frame not met IEEE 802.3
	if (velocity.hw.flags & VELOCITY_FLAGS_VAL_PKT_LEN) {
		if (wRSR & RSR_RL) {
			etherStats->dot3RxExtraEntry.frameTooShorts += 1;
			freePacket(skb);
			return FALSE;
		}
	}
	
	velocity_rx_csum(pRD, skb);
	netif->inputPacket(skb, wLength, IONetworkInterface::kInputOptionQueuePacket);
	
    return TRUE;
}

/**
 *	velocity_rx_srv		-	service RX interrupt
 *	@vptr: velocity
 *	@status: adapter status (unused)
 *
 *	Walk the receive ring of the velocity adapter and remove
 *	any received packets from the receive queue. Hand the ring
 *	slots back to the adapter for reuse.
 */

int ViaVelocity::velocity_rx_srv( int status)
{
	PRX_DESC	pRD;
	int			iCurrRDIdx = velocity.hw.iCurrRDIdx;
	int			works = 0;
	U16			wRSR;
	BOOL		bVIDM = FALSE;
	WORD		wRBRDU;
	WORD		wCurrDeltaRBRDU;

    if(velocity.hw.sOpts.tagging == 2)
 		bVIDM = TRUE;

    while (TRUE) {
        pRD = &(velocity.hw.aRDRing[iCurrRDIdx]);

#if	USE_RX_BUFFER

#else
        if ((velocity.aRDInfo[iCurrRDIdx]).skb == NULL) {
            if (!velocity_alloc_rx_buf(iCurrRDIdx))
                break;
        }
#endif

        if (works++ > velocity.hw.sOpts.int_works)
            break;
		
        if (pRD->rdesc0 & cpu_to_le32(RDESC0_OWN))
            break;
		
		// velocity.adwRMONStats[RMON_Octets] += (U16)((cpu_to_le32(pRD->rdesc0) >> 16) & 0x00003fffL);
		
        wRSR = (U16)(cpu_to_le32(pRD->rdesc0));
		
        // don't drop CE or RL error frame although RXOK is off
        if( (wRSR & RSR_RXOK) || (!(wRSR & RSR_RXOK) && (wRSR & (RSR_CE | RSR_RL)))||(bVIDM && (wRSR & RSR_VIDM))) {
            if (velocity_receive_frame(iCurrRDIdx)) {
				netStats->inputPackets += 1;
#if	USE_RX_BUFFER
#else
                if (!velocity_alloc_rx_buf(iCurrRDIdx)) {
                    IOLog("%s: can not allocate rx buf\n", getName());
                    break;
                }
#endif
            }
            else {
				netStats->inputErrors++;
            }
        }
        else {
            if (wRSR & RSR_CRC)
                etherStats->dot3StatsEntry.fcsErrors += 1;
            if (wRSR & RSR_FAE)
                etherStats->dot3StatsEntry.fcsErrors += 1;
			
			netStats->inputErrors++;
        }
		
        if ((iCurrRDIdx % 4) == 3) {
            int i, iPrevRDIdx = iCurrRDIdx;
            for (i=0; i<4; i++) {
                pRD = &(velocity.hw.aRDRing[iPrevRDIdx]);
                pRD->rdesc0 |= cpu_to_le32(RDESC0_OWN);
                SUB_ONE_WITH_WRAP_AROUND(iPrevRDIdx, velocity.hw.sOpts.nRxDescs);
            }
            if (velocity.hw.byRevId >= REV_ID_VT3286_A0) {
                wRBRDU = CSR_READ_2(&velocity.hw, MAC_REG_RBRDU);
                wCurrDeltaRBRDU = velocity.hw.sOpts.nRxDescs - wRBRDU;
                if(4 <= wCurrDeltaRBRDU) {
                    CSR_WRITE_2(&velocity.hw, 4, MAC_REG_RBRDU);
                }
                else {
                    CSR_WRITE_2(&velocity.hw, wCurrDeltaRBRDU, MAC_REG_RBRDU);
                }
            }
            else {
                CSR_WRITE_2(&velocity.hw, 4, MAC_REG_RBRDU);
            }
        }
		
        //velocity.dev->last_rx = jiffies;
		
        ADD_ONE_WITH_WRAP_AROUND(iCurrRDIdx, velocity.hw.sOpts.nRxDescs);
    }
	
	velocity.hw.iCurrRDIdx = iCurrRDIdx;
	return works;
}

/**
 *	tx_srv		-	transmit interrupt service
 *	@vptr; Velocity
 *	@status:
 *
 *	Scan the queues looking for transmitted packets that
 *	we can complete and clean up. Update any statistics as
 *	necessary/
 */

int ViaVelocity::velocity_tx_srv(UInt32 status)
{
    PTX_DESC                    pTD;
    int                         iQNo;
    BOOL                        bFull = FALSE;
    int                         idx;
    int                         works = 0;
    PVELOCITY_TD_INFO           pTDInfo;
    U16                         wTSR;
	
	
    for (iQNo=0; iQNo<velocity.hw.nTxQueues; iQNo++) {
        for (idx = velocity.hw.aiTailTDIdx[iQNo];
             velocity.hw.iTDUsed[iQNo]>0;
             idx = (idx+1) % velocity.hw.sOpts.nTxDescs)
        {
            // Get Tx Descriptor
            pTD = &(velocity.hw.apTDRings[iQNo][idx]);
            pTDInfo = &(velocity.apTDInfos[iQNo][idx]);
			
            if (pTD->tdesc0 & cpu_to_le32(TDESC0_OWN)) {
                break;
            }
			
            if (works++ > velocity.hw.sOpts.int_works) {
                break;
            }
			
            wTSR = (U16)cpu_to_le32(pTD->tdesc0);
			
            if (wTSR & TSR0_TERR) {
                netStats->outputErrors++;
#if	0
                if (wTSR & TSR0_CDH)
                    pStats->tx_heartbeat_errors++;
                if (wTSR & TSR0_CRS)
                    pStats->tx_carrier_errors++;
                if (wTSR & TSR0_ABT)
                    pStats->tx_aborted_errors++;
                if (wTSR & TSR0_OWC)
                    pStats->tx_window_errors++;
#endif
            }
            else {
                netStats->outputPackets++;
            }
			
            velocity_free_tx_buf(pTDInfo);
            velocity.hw.iTDUsed[iQNo]--;
        } // for (idx)
		
        velocity.hw.aiTailTDIdx[iQNo]=idx;
		
        if (AVAIL_TD(&velocity.hw, iQNo) < 1 ) {
            bFull = TRUE;
        }
    } // for (iQNo)

    if (!netif_running() && (bFull==FALSE)
        && (!(velocity.hw.mii_status & VELOCITY_LINK_FAIL))) {
        netif_start_queue();
    }

    return works;
}


//---------------------------------------------------------------------------
// Function: handleInterrupt
//
// Purpose:
//   Field an interrupt.

int ViaVelocity::handleInterrupt(IOInterruptEventSource * src, int count)
{
	U32                 isr_status;
	int                 tx_count = 0;
	int                 rx_count = 0;

	if(( velocity.hw.flags & VELOCITY_FLAGS_OPENED) == 0)
		return 0;
	
	if(count > 1){
		IOLog("%s::%s(%d)",getName(),__FUNCTION__,count);
	}
	/*
	 * Loop until the interrupt line becomes deasserted.
	 */
	
#if	NO_LOCK_FREE
	IOSimpleLockLock(velocity.lock);
#endif
	isr_status = mac_read_isr(&velocity.hw);
	if (isr_status == 0) {
#if	NO_LOCK_FREE
		IOSimpleLockUnlock(velocity.lock);
#endif
		return 0;
	}
	
	mac_disable_int(&velocity.hw);

	mac_write_isr(&velocity.hw, isr_status);
	
	velocity_error(isr_status);
	
	rx_count += velocity_rx_srv(isr_status);
	tx_count += velocity_tx_srv(isr_status);
	
	// [1.18], for performance
	rx_count += velocity_rx_srv(isr_status);
	tx_count += velocity_tx_srv(isr_status);
	
	
	// Flush all packets received and pass them to the network stack.
	//
	if (rx_count)
		netif->flushInputQueue();
	if (tx_count)
		transmitQueue->service();

	mac_enable_int(&velocity.hw);
#if	NO_LOCK_FREE
	IOSimpleLockUnlock(velocity.lock);
#endif

	return tx_count + rx_count;
}

void ViaVelocity::interruptHandler( OSObject* target,
								   IOInterruptEventSource * src,
								   int count )
{
	// C++ glue to eliminate compiler warnings
	ViaVelocity* me = (ViaVelocity *) target;
	me->handleInterrupt( src, count );
}

bool ViaVelocity::interruptFilter(OSObject* target, IOFilterInterruptEventSource* src )
{
	ViaVelocity* me = (ViaVelocity *) target;
	U32 isr_status = mac_read_isr(&(me->velocity.hw));
	if(isr_status == 0)
		return false;
	
	return true;
}

//---------------------------------------------------------------------------
// Function: timeoutOccurred
//
// Periodic timer that monitors the receiver status, updates error
// and collision statistics, and update the current link status.

void ViaVelocity::timeoutOccurred( IOTimerEventSource * /*timer*/ )
{
	timerSrc->setTimeoutMS(LOAD_STATISTICS_INTERVAL);

	velocity.hw.mii_status = check_connectiontype(&velocity.hw);

	if( velocity.hw.mii_status != prevStat ){
		prevStat = velocity.hw.mii_status;
		if( velocity.hw.mii_status & VELOCITY_LINK_FAIL ){
			setLinkStatus( kIONetworkLinkValid );
		} else {
			UInt32 speed = 10 * MBPS;
        	if (velocity.hw.mii_status & VELOCITY_SPEED_1000)
            	speed = 1000 * MBPS;
	        else if (velocity.hw.mii_status & VELOCITY_SPEED_100)
            	speed = 100 * MBPS;
			setLinkStatus( kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), speed );
		}
	}
	if (!netif_running()){
		if(velocity.hw.mii_status & VELOCITY_LINK_FAIL){
			IOLog("%s::%s tx stopped for VELOCITY_LINK_FAIL",getName(),__FUNCTION__);
		} else {
			int n = AVAIL_TD(&velocity.hw, 0);
			IOLog("%s::%s AVAIL_TD(%d)\n",getName(),__FUNCTION__,n);
			if (n >= 1 ) {
				netif_start_queue();
			}
		}
    }
	
}

void ViaVelocity::timeoutHandler( OSObject * target, IOTimerEventSource * src )
{
	ViaVelocity * me = (ViaVelocity *) target;
	me->timeoutOccurred(src);
}

IOReturn ViaVelocity::registerWithPolicyMaker ( IOService * policyMaker )
{
	//IOLog("%s::%s()\n",getName(),__FUNCTION__);
	static IOPMPowerState powerStateArray[ 2 ] = {
		{ 1,0,0,0,0,0,0,0,0,0,0,0 },
		{ 1,kIOPMDeviceUsable,kIOPMPowerOn,kIOPMPowerOn,0,0,0,0,0,0,0,0 }
	};
	pmPowerState = 1;
	return policyMaker->registerPowerDriver( this, powerStateArray, 2 );
}

IOReturn ViaVelocity::setPowerState( unsigned long powerStateOrdinal,
										 IOService *policyMaker )
{
	//IOLog("%s::%s(%d)\n",getName(),__FUNCTION__,(int)powerStateOrdinal);
	if (pmPowerState == powerStateOrdinal)
		return IOPMAckImplied;
	pmPowerState = powerStateOrdinal;
	
	if(pmPowerState == 1){	//
        velocity_restore_mac_context(&velocity.hw, &velocity.mac_context);
	} else {
        velocity_save_mac_context(&velocity.hw, &velocity.mac_context);
		suspend = true;
	}
    return IOPMAckImplied;
}


IOReturn ViaVelocity::setWakeOnMagicPacket( bool active )
{
	if( active )
		velocity.wol_opts |= VELOCITY_FLAGS_WOL_ENABLED;
	else
		velocity.wol_opts &= ~VELOCITY_FLAGS_WOL_ENABLED;

	return kIOReturnSuccess;
}
