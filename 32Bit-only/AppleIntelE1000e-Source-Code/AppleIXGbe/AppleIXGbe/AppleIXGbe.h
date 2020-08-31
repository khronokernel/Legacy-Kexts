enum
{
	MEDIUM_INDEX_AUTO = 0,
	MEDIUM_INDEX_100FD,
	MEDIUM_INDEX_1000FD,
	MEDIUM_INDEX_10000FD,
	MEDIUM_INDEX_COUNT
};
#define MBit 1000000


class AppleIXGbe: public IOEthernetController
{
	
	OSDeclareDefaultStructors(AppleIXGbe);
	
public:
	// --------------------------------------------------
	// IOService (or its superclass) methods.
	// --------------------------------------------------
	virtual bool start(IOService * provider);
	virtual void stop(IOService * provider);
	virtual bool init(OSDictionary *properties);
	virtual void free();

	// --------------------------------------------------
	// IONetworkController methods.
	// --------------------------------------------------
	virtual IOReturn enable(IONetworkInterface * netif);
	virtual IOReturn disable(IONetworkInterface * netif);

	virtual UInt32 outputPacket(mbuf_t m, void * param);
	virtual void getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const;
	virtual IOOutputQueue * createOutputQueue();
	virtual bool createWorkLoop();
	virtual IOWorkLoop * getWorkLoop() const;

	virtual const OSString * newVendorString() const;
	virtual const OSString * newModelString() const;
	virtual IOReturn selectMedium(const IONetworkMedium * medium);
	virtual bool configureInterface(IONetworkInterface * interface);
	

	//-----------------------------------------------------------------------
	// IOEthernetController methods.
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
    virtual IOReturn setWakeOnMagicPacket(bool active);
    virtual IOReturn getPacketFilters(const OSSymbol * group, UInt32 * filters) const;
    virtual UInt32 getFeatures() const;
	// --------------------------------------------------
	// Power Management Support
	// --------------------------------------------------
	virtual IOReturn registerWithPolicyMaker(IOService* policyMaker);
    virtual IOReturn setPowerState( unsigned long powerStateOrdinal, IOService *policyMaker );
private:
	UInt32 ixgbe_xmit_frame_ring(mbuf_t skb,
								struct ixgbe_adapter *adapter,
								struct ixgbe_ring *tx_ring);
	
	bool addNetworkMedium(UInt32 type, UInt64 bps, UInt32 index);
public:
	struct ixgbe_adapter priv_adapter;
	UInt32 flags;
	UInt32 features;
	UInt32 mtu;
	UInt32 gso;
	UInt32 trans_start;
private:
	IOWorkLoop* workLoop;
	IOPCIDevice* pdev;
	OSDictionary * mediumDict;
	IONetworkMedium * mediumTable[MEDIUM_INDEX_COUNT];
	IOOutputQueue * transmitQueue;
	IOInterruptEventSource * interruptSource;
	IOTimerEventSource * timerSource;
	IOTimerEventSource * serviceSource;
	IOEthernetInterface * netif;
	IONetworkStats * netStats;
	IOEthernetStats * etherStats;

	IOMemoryMap * csrPCIAddress;
	IOMbufNaturalMemoryCursor * txMbufCursor;

	bool enabledForNetif;
	bool bQueueStopped;
	UInt32 preLinkStatus;
	UInt32 powerState;
public:
	void detachNetIf();
	void receive(mbuf_t m, UInt32 vlanTag, UInt32 csumFlag);
	bool running(){return enabledForNetif;}
	bool queueStopped(){return bQueueStopped;}
	bool carrier();
	void setCarrier(bool);
	void startTxQueue();
	void stopTxQueue();
	void scheduleService(){serviceSource->setTimeoutMS(0);}
	void stopService(){serviceSource->cancelTimeout();}
	IONetworkStats* stats(){ return netStats; }
private:
	void handleInterrupt( int count );
	void handleTimer();
	void handleService();

public:
	static void interruptHandler(OSObject * target, IOInterruptEventSource * src, int count );
	static void timerHandler(OSObject * target, IOTimerEventSource * src);
	static void serviceHandler(OSObject * target, IOTimerEventSource * src);

};