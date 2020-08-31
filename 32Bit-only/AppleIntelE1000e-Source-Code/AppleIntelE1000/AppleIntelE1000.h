#ifndef __INTEL_E1000_H__
#define __INTEL_E1000_H__

#define MBit 1000000

enum {
	eePowerStateOff = 0,
	eePowerStateOn,
	eePowerStateCount
};

enum {
	kFiveSeconds = 5000000
};


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

enum 
{
	kActivationLevelNone = 0,  /* adapter shut off */
	kActivationLevelKDP,       /* adapter partially up to support KDP */
	kActivationLevelBSD        /* adapter fully up to support KDP and BSD */
};

enum
{
	kFullInitialization = 0,
	kResetChip          = 1
};

#define	MAX_RX_SIZE	(kIOEthernetAddressSize+kIOEthernetMaxPacketSize)
#define	SIZE_RING_DESC	PAGE_SIZE
#define NUM_RING_FRAME	256

class AppleIntelE1000: public IOEthernetController
{
	
	OSDeclareDefaultStructors(AppleIntelE1000);
	
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
	
private:
	IOWorkLoop * workLoop;
	OSDictionary * mediumDict;
	IONetworkMedium * mediumTable[MEDIUM_INDEX_COUNT];
	IOOutputQueue * transmitQueue;
	bool txqActive;
	IOInterruptEventSource * interruptSource;
public:
	IOTimerEventSource * watchdogSource;
	IOTimerEventSource * phyinfoSource;
	IOTimerEventSource * fifostallSource;
	IOPCIDevice * pciDevice;

private:
	IOEthernetInterface * netif;
	IONetworkStats * netStats;
	IOEthernetStats * etherStats;
    
public:
	IOMemoryMap * csrPCIAddress;
	
	IOMbufNaturalMemoryCursor * txMbufCursor;
	
private:
	bool enabledForNetif;
	UInt32 preLinkStatus;
	UInt32 powerState;
	
	struct e1000_adapter adapter;
	UInt32 icr;
public:
	void e1000_update_phy_info_task();
	void e1000_82547_tx_fifo_stall_task();
	void e1000_watchdog_task();
	void e1000_reset_task();
	int e1000_intr();

	void receive(mbuf_t skb, UInt32 vlanTag);

private:
	void timeoutOccurred(IOTimerEventSource* src);
	bool addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index);
	
	bool initEventSources( IOService* provider );
	
	void e1000_clean_rx_ring();
public:
	void e1000_set_mtu(UInt32 maxSize);
	void e1000_rx_err( int  );
	void e1000_tx_err( int  );
	void initTimers();
	void delTimers();
	UInt32 features;
	UInt32 flags;
	UInt32 mtu;
	UInt32 last_rx;
	UInt32 trans_start;
	const char* name;
	e1000_adapter* priv(){ return &adapter;}
	bool running(){ return enabledForNetif; }
	void startQueue();
	void stopQueue();
	void wakeQueue();
	bool queueStopped();
	void carrierOn();
	void carrierOff();
	bool carrierOK(){ return preLinkStatus != 0; }

public:
	static void interruptHandler(OSObject * target, IOInterruptEventSource * src, int count );
	
	static bool interruptFilter(OSObject * target , IOFilterInterruptEventSource * src);
	
	static void timeoutHandler(OSObject * target, IOTimerEventSource * src);
	
};


#endif //__INTEL_E1000_H__
