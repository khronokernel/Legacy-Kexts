
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IONetworkInterface.h>
#include <IOKit/network/IONetworkStats.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/network/IOPacketQueue.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/assert.h>

extern "C" {
#include <sys/kpi_mbuf.h>
}

extern "C" {
#include "velocity.h"
}

#define TRANSMIT_QUEUE_SIZE		256

typedef enum {
	MEDIUM_INDEX_AUTO = 0,
	MEDIUM_INDEX_10HD,
	MEDIUM_INDEX_10FD,
	MEDIUM_INDEX_100HD,
	MEDIUM_INDEX_100FD,
	MEDIUM_INDEX_1000FD,
	MEDIUM_INDEX_COUNT,
	MEDIUM_TYPE_INVALID = MEDIUM_INDEX_COUNT,
} mediumType_t;

#define RX_DESC_MIN     64
#define RX_DESC_MAX     255
#define RX_DESC_DEF     252

#define TX_DESC_MIN     16
#define TX_DESC_MAX     256
#define TX_DESC_DEF     256

#define VLAN_ID_MIN         0
#define VLAN_ID_MAX         4094
#define VLAN_ID_DEF         0
/* VID_setting[] is used for setting the VID of NIC.
 0: default VID.
 1-4094: other VIDs.
 */


#define RX_THRESH_MIN   0
#define RX_THRESH_MAX   3
#define RX_THRESH_DEF   0
/* rx_thresh[] is used for controlling the receive fifo threshold.
 0: indicate the rxfifo threshold is 128 bytes.
 1: indicate the rxfifo threshold is 512 bytes.
 2: indicate the rxfifo threshold is 1024 bytes.
 3: indicate the rxfifo threshold is store & forward.
 */

#define DMA_LENGTH_MIN  0
#define DMA_LENGTH_MAX  7
#define DMA_LENGTH_DEF  6

/* DMA_length[] is used for controlling the DMA length
 0: 8 DWORDs
 1: 16 DWORDs
 2: 32 DWORDs
 3: 64 DWORDs
 4: 128 DWORDs
 5: 256 DWORDs
 6: SF(flush till emply)
 7: SF(flush till emply)
 */

#define TAGGING_MIN         0
#define TAGGING_MAX         2
#define TAGGING_DEF         0
/* enable_tagging[] is used for enabling 802.1Q VID tagging.
 0: disable VID seeting(default).
 1: enable VID setting.
 */

#define IP_ALIG_DEF     0
/* IP_byte_align[] is used for IP header DWORD byte aligned
 0: indicate the IP header won't be DWORD byte aligned.(Default) .
 1: indicate the IP header will be DWORD byte aligned.
 In some enviroment, the IP header should be DWORD byte aligned,
 or the packet will be droped when we receive it. (eg: IPVS)
 */

#ifdef VELOCITY_TX_CSUM_SUPPORT
#define TX_CSUM_DEF     1
/* txcsum_offload[] is used for setting the Tx checksum offload ability of NIC.
 0: disable csum_offload[checksum offload
 1: enable checksum offload. (Default)
 */
#endif

#define FLOW_CNTL_DEF   1
#define FLOW_CNTL_MIN   1
#define FLOW_CNTL_MAX   5

/* flow_control[] is used for setting the flow control ability of NIC.
 1: hardware deafult - AUTO (default). Use Hardware default value in ANAR.
 2: enable TX flow control.
 3: enable RX flow control.
 4: enable RX/TX flow control.
 5: disable
 */

#define MED_LNK_DEF 0
#define MED_LNK_MIN 0
#define MED_LNK_MAX	5
/* speed_duplex[] is used for setting the speed and duplex mode of NIC.
 0: indicate autonegotiation for both speed and duplex mode
 1: indicate 100Mbps half duplex mode
 2: indicate 100Mbps full duplex mode
 3: indicate 10Mbps half duplex mode
 4: indicate 10Mbps full duplex mode
 5: indicate 1000Mbps full duplex mode

 Note:
 if EEPROM have been set to the force mode, this option is ignored
 by driver.
 */

#define VAL_PKT_LEN_DEF     0
/* ValPktLen[] is used for setting the checksum offload ability of NIC.
 0: Receive frame with invalid layer 2 length (Default)
 1: Drop frame with invalid layer 2 length
 */

#define WOL_OPT_DEF     0
#define WOL_OPT_MIN     0
#define WOL_OPT_MAX     7
/* wol_opts[] is used for controlling wake on lan behavior.
 0: Wake up if recevied a magic packet. (Default)
 1: Wake up if link status is on/off.
 2: Wake up if recevied an arp packet.
 4: Wake up if recevied any unicast packet.
 Those value can be sumed up to support more than one option.
 */

#define INT_WORKS_DEF   64
#define INT_WORKS_MIN   10
#define INT_WORKS_MAX   256

// EnableMRDPL[] is used for setting the Memory-Read-Multiple ability of NIC
// 0: Disable (default)
// 1: Enable
#define MRDPL_DEF           0

// EnableAI[] is used for setting the Adaptive-Interrupt ability of NIC
// 0: Disable
// 1: Enable (default)
#define AI_DEF              1

#define TXQUE_TIMER_DEF     0x59
#define TXQUE_TIMER_MIN     0x00
#define TXQUE_TIMER_MAX     0xFF

#define RXQUE_TIMER_DEF     0x14
#define RXQUE_TIMER_MIN     0x00
#define RXQUE_TIMER_MAX     0xFF

#define TX_INTSUP_DEF       0x1F
#define TX_INTSUP_MIN       0x01
#define TX_INTSUP_MAX       0x3F

#define RX_INTSUP_DEF       0x1F
#define RX_INTSUP_MIN       0x01
#define RX_INTSUP_MAX       0x3F

#define	MALLOC_MASK	0xFFFFF000UL
#define	RXBUF_MASK	0xFFFFFF00UL

class ViaVelocity : public IOEthernetController
{
    OSDeclareDefaultStructors(ViaVelocity)
	
public:
    IOEthernetAddress              myAddress;
    IOEthernetInterface	*          netif;
    IOKernelDebugger *             debugger;
    IOPCIDevice *                  pciNub;
    IOWorkLoop *                   workLoop;
	
    IOInterruptEventSource *       interruptSrc;
    IOOutputQueue *                transmitQueue;
    IOTimerEventSource *           timerSrc;
    IONetworkStats *               netStats;
    IOEthernetStats *              etherStats;
    IOMemoryMap *                  csrMap;
    OSDictionary *                 mediumDict;
    IONetworkMedium *              mediumTable[MEDIUM_TYPE_INVALID];
	
    IOMbufLittleMemoryCursor *     rxMbufCursor;
	
    bool                           multicastEnabled;
	UInt32						flags;
    bool                           verbose;
    mediumType_t                   currentMediumType;
	
    /* power management support */
    unsigned long                  pmPowerState;
	
    static void interruptHandler( OSObject* target, IOInterruptEventSource* src, int count );
    static bool interruptFilter( OSObject* target, IOFilterInterruptEventSource* src );
	
    static void timeoutHandler( OSObject* target,IOTimerEventSource * src );
	
	// --------------------------------------------------
	// IOService (or its superclass) methods.
	// --------------------------------------------------
	virtual bool init(OSDictionary *properties);
	virtual bool start(IOService * provider);
	virtual void stop(IOService * provider);
	virtual void free();
	
	// --------------------------------------------------
	// IONetworkController methods.
	// --------------------------------------------------
	
	virtual IOReturn enable(IONetworkInterface * netif);
	virtual IOReturn disable(IONetworkInterface * netif);
	
	
	virtual UInt32 outputPacket(mbuf_t m, void * param);
	virtual IOReturn getMaxPacketSize (UInt32 *maxSize) const;
	virtual IOReturn getMinPacketSize (UInt32 *minSize) const;
	virtual IOReturn setMaxPacketSize (UInt32 maxSize);
    virtual UInt32 getFeatures() const;

	virtual void getPacketBufferConstraints(IOPacketBufferConstraints * constraints) const;
	virtual IOReturn getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput);
	
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
	
	virtual IOReturn getHardwareAddress( IOEthernetAddress * addr );
	virtual IOReturn setHardwareAddress( const IOEthernetAddress * addr );
	virtual IOReturn setPromiscuousMode( bool active );
	virtual IOReturn setMulticastMode( bool active );
	virtual IOReturn setMulticastList( IOEthernetAddress * addrs, UInt32 count );
	
	// --------------------------------------------------
	// driver specific methods.
	// --------------------------------------------------
	
	bool pciConfigInit(IOPCIDevice * provider);
	bool initDriver(IOService * provider);
	
	void getDefaultSettings();
	
	int handleInterrupt(IOInterruptEventSource * src, int count);
	void timeoutOccurred(IOTimerEventSource * timer);
	
	// --------------------------------------------------
	// PHY methods.
	// --------------------------------------------------
	
	bool   _phySetMedium(mediumType_t medium);
	void   _phyPublishMedia();
	bool   _phyAddMediumType(UInt32 type, UInt32 speed, UInt32 code);
	IONetworkMedium * _phyGetMediumWithType(UInt32 type);
	
	void netif_start_queue();
	void netif_stop_queue();
	BOOL netif_running();
	//-----------------------------------------------------------------------
	// Power management support.
	//-----------------------------------------------------------------------
	
	virtual IOReturn registerWithPolicyMaker( IOService * policyMaker );
	
    virtual IOReturn setPowerState( unsigned long powerStateOrdinal, IOService *policyMaker);

    virtual IOReturn setWakeOnMagicPacket( bool active );
	
private:
	int velocity_get_int_opt( int min, int max, int def, const char *name );
	bool velocity_get_bool_opt( const char *name, bool defVal );
	void velocity_free_tx_buf( PVELOCITY_TD_INFO pTDInfo );
	int velocity_init_rd_ring();
	BOOL velocity_init_td_ring();
	BOOL velocity_init_rings();

	void velocity_free_rings();
	void velocity_free_rd_ring();
	void velocity_free_td_ring();
	
	BOOL velocity_alloc_rx_buf(int idx);
	
	void velocity_set_multi();

	void velocity_init_adapter(VELOCITY_INIT_TYPE);

	
	int velocity_change_mtu(int new_mtu);
	
	void velocity_error(int status);
	int velocity_rx_srv(int status);
	int velocity_tx_srv(UInt32 status);
	BOOL velocity_receive_frame(int idx);
	void velocity_rx_csum(PRX_DESC pRD, mbuf_t skb);
#if ! USE_RX_BUFFER
	mbuf_t velocity_alloc_packet( UInt32 size );
	mbuf_t velocity_copy_packet(mbuf_t m, UInt32 size);
#endif
	VELOCITY_INFO  velocity;
	UInt32 mtu;
	UInt32 maxPacketSize;
	UInt32 prevStat;
	bool is64bit;
	bool tqActive;
	bool suspend;
};
