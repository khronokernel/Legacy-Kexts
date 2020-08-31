#ifndef	__AGERE_ET131X_H__
#define	__AGERE_ET131X_H__

#include "et131x_osx.h"

enum
{
	MEDIUM_INDEX_AUTO = 0,
	MEDIUM_INDEX_10HD,
	MEDIUM_INDEX_10FD,
	MEDIUM_INDEX_100HD,
	MEDIUM_INDEX_100FD,
	MEDIUM_INDEX_1000FD,
	MEDIUM_INDEX_COUNT
};

#define MBit 1000000
#define TBDS_PER_TCB 12
#define USE_TX_CSUM	0

#define	RELEASE(x)	if(x)x->release();x=NULL;

#define super IOEthernetController

class AgereET131x: public IOEthernetController
{
		
	OSDeclareDefaultStructors(AgereET131x);
		
public:
	// --------------------------------------------------
	// IOService (or its superclass) methods.
	// --------------------------------------------------
		
	virtual bool start(IOService * provider);
	virtual void stop(IOService * provider);
	virtual bool init(OSDictionary *properties);
	virtual void free();
	// --------------------------------------------------
	// Power Management Support
	// --------------------------------------------------
	virtual IOReturn registerWithPolicyMaker(IOService* policyMaker);
    virtual IOReturn setPowerState( unsigned long powerStateOrdinal, IOService *policyMaker );
	
	// --------------------------------------------------
	// IONetworkController methods.
	// --------------------------------------------------
	
	virtual IOReturn enable(IONetworkInterface * netif);
	virtual IOReturn disable(IONetworkInterface * netif);
	
	virtual UInt32 outputPacket(mbuf_t m, void * param);
	
	virtual void getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const;
	
	virtual IOOutputQueue * createOutputQueue();
	
	virtual const OSString * newVendorString() const;
	virtual const OSString * newModelString() const;
	
	virtual IOReturn selectMedium(const IONetworkMedium * medium);
	virtual bool configureInterface(IONetworkInterface * interface);
	
	virtual bool createWorkLoop();
	virtual IOWorkLoop * getWorkLoop() const;
	
	//-----------------------------------------------------------------------
	// Methods inherited from IOEthernetController.
	//-----------------------------------------------------------------------
	
	virtual IOReturn getHardwareAddress(IOEthernetAddress * addr);
	virtual IOReturn setHardwareAddress(const IOEthernetAddress * addr);
	virtual IOReturn setPromiscuousMode(bool active);
	virtual IOReturn setMulticastMode(bool active);
	virtual IOReturn setMulticastList(IOEthernetAddress * addrs, UInt32 count);
	virtual IOReturn getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput);
	virtual IOReturn setMaxPacketSize (UInt32 maxSize);
	virtual IOReturn getMaxPacketSize (UInt32 *maxSize) const;
	virtual IOReturn getMinPacketSize (UInt32 *minSize) const;
	virtual UInt32 getFeatures() const;

public:
	void interruptOccurred(IOInterruptEventSource * src);
	void timeoutOccurred(IOTimerEventSource* src);
	bool filterCheck(IOInterruptEventSource * src);

private:
	static void interruptHandler(OSObject * target, IOInterruptEventSource * src, int count );
	static bool interruptFilter(OSObject * target , IOFilterInterruptEventSource * src);
	static void timeoutHandler(OSObject * target, IOTimerEventSource * src);

	bool initEventSources( IOService* provider );
	bool addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index);


	bool initPCIConfigSpace();

	int adapter_memory_alloc();
	void adapter_memory_free();
	// 
	void soft_reset();
	void disable_interrupts();
	void enable_interrupts();
	int send_packet( mbuf_t skb );
	int nic_send_packet( PMP_TCB pMpTcb );
	void free_send_packet( PMP_TCB pMpTcb );
	void free_busy_send_packets();
	void update_tcb_list();
	void check_send_wait_list();
	PMP_RFD nic_rx_pkts();
	void handle_recv_interrupt();
	void DisablePhyComa();
	void EnablePhyComa();
	void rx_dma_enable();
	void rx_dma_disable();
	void tx_dma_enable();
	void tx_dma_disable();

	void HandleMacStatInterrupt();
	void HandlePhyInterrupt();
	void UpdateMacStatHostCounters();

	int adapter_setup();
	void reset_recv();
	void init_send();
	void set_mac_addr(const  u8* addr);
	void set_mtu(UInt32);

	void Mii_check( MI_BMSR_t bmsr, MI_BMSR_t bmsr_ints );
	void PhyInit();
	void PhyReset();
	void PhyPowerDown();
	void PhyPowerUp();
    int setphy_normal();
	int xcvr_find();
	
	void set_packet_filter();
private:
	IOWorkLoop * workLoop;
	IOPCIDevice * pciDevice;
	OSDictionary * mediumDict;
	IONetworkMedium * mediumTable[MEDIUM_INDEX_COUNT];
	IOOutputQueue * transmitQueue;
	
	IOInterruptEventSource * interruptSource;
	IOTimerEventSource * watchdogSource;
	
	IOEthernetInterface * netif;
	IONetworkStats * netStats;
	IOEthernetStats * etherStats;
    
	IOMemoryMap * csrPCIAddress;
	
	IOMbufNaturalMemoryCursor * rxMbufCursor;
	IOMbufNaturalMemoryCursor * txMbufCursor;
	
	bool enabledForNetif;
	UInt32 mtu;
	UInt32 powerState;
	
	bool suspend;

	ET131X_ADAPTER	adapter;
};

#endif
