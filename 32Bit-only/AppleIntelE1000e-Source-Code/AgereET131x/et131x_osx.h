#ifndef __ET131X_OSX_H__
#define __ET131X_OSX_H__

#include <AvailabilityMacros.h>
#include <sys/types.h>
#include <libkern/OSByteOrder.h>
#include <sys/kpi_mbuf.h>
#include <IOKit/IOLib.h>
#include <libkern/OSAtomic.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/network/IOPacketQueue.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/assert.h>

#include "kcompat.h"

#define DBG_ERROR(A,S...)
#define DBG_WARNING(A,S...)
#define DBG_NOTICE(A,S...)
#define DBG_TRACE(A,S...)
#define DBG_VERBOSE(A,S...)
#define DBG_ASSERT(C)


/******************************************************************************
 Define a boolean type
 *****************************************************************************/
typedef u8 BOOL_t;

/******************************************************************************
 Packet and header sizes
 *****************************************************************************/
#define NIC_MIN_PACKET_SIZE             60
#define NIC_HEADER_SIZE                 ETH_HLEN    //14


/******************************************************************************
 Multicast list size
 *****************************************************************************/
#define NIC_MAX_MCAST_LIST              128

/******************************************************************************
 Supported Filters
 *****************************************************************************/
#define ET131X_PACKET_TYPE_DIRECTED      0x0001
#define ET131X_PACKET_TYPE_MULTICAST     0x0002
#define ET131X_PACKET_TYPE_BROADCAST     0x0004
#define ET131X_PACKET_TYPE_PROMISCUOUS   0x0008
#define ET131X_PACKET_TYPE_ALL_MULTICAST 0x0010



/******************************************************************************
 MP_TCB flags
 *****************************************************************************/
#define fMP_DEST_MULTI                          0x00000001
#define fMP_DEST_BROAD                          0x00000002




/******************************************************************************
 MP_ADAPTER flags
 *****************************************************************************/
#define fMP_ADAPTER_RECV_LOOKASIDE              0x00000004
#define fMP_ADAPTER_INTERRUPT_IN_USE            0x00000008
#define fMP_ADAPTER_SECONDARY                   0x00000010




/******************************************************************************
 MP_SHARED flags
 *****************************************************************************/
#define fMP_ADAPTER_SHUTDOWN                    0x00100000
#define fMP_ADAPTER_LOWER_POWER                 0x00200000

#define fMP_ADAPTER_NON_RECOVER_ERROR           0x00800000
#define fMP_ADAPTER_RESET_IN_PROGRESS           0x01000000
#define fMP_ADAPTER_NO_CABLE                    0x02000000 
#define fMP_ADAPTER_HARDWARE_ERROR              0x04000000
#define fMP_ADAPTER_REMOVE_IN_PROGRESS          0x08000000
#define fMP_ADAPTER_HALT_IN_PROGRESS            0x10000000
#define fMP_ADAPTER_LINK_DETECTION              0x20000000

#define fMP_ADAPTER_FAIL_SEND_MASK              0x3ff00000                
#define fMP_ADAPTER_NOT_READY_MASK              0x3ff00000




/******************************************************************************
 Some offsets in PCI config space that are actually used.
 *****************************************************************************/
#define ET1310_PCI_PM_CAPABILITY    (UINT32)0x40
#define ET1310_PCI_PM_CSR           (UINT32)0x44
#define ET1310_PCI_MAX_PYLD         (UINT32)0x4C
#define ET1310_PCI_DEV_CTRL         (UINT32)0x50
#define ET1310_PCI_DEV_STAT         (UINT32)0x52
#define ET1310_NMI_DISABLE          (UINT32)0x61
#define ET1310_PCI_MAC_ADDRESS      (UINT32)0xA4
#define ET1310_PCI_EEPROM_STATUS    (UINT32)0xB2
#define ET1310_PCI_PHY_INDEX_REG    (UINT32)0xB4
#define ET1310_PCI_ACK_NACK         (UINT32)0xC0
#define ET1310_PCI_REPLAY           (UINT32)0xC2
#define ET1310_PCI_L0L1LATENCY      (UINT32)0xCF
#define ET1310_PCI_SEL_PHY_CTRL     (UINT32)0xE4
#define ET1310_PCI_ADVANCED_ERR     (UINT32)0x100




/******************************************************************************
 PCI Vendor/Product IDs
 *****************************************************************************/
#define ET131X_PCI_VENDOR_ID      0x11C1  // Agere Systems
#define ET131X_PCI_DEVICE_ID_GIG  0xED00  // ET1310 1000 Base-T
#define ET131X_PCI_DEVICE_ID_FAST 0xED01  // ET1310 100  Base-T


/******************************************************************************
 Define FIELD_OFFSET macro
 *****************************************************************************/
//#define FIELD_OFFSET(type,field)    ((int)(&((type *)0)->field))
#define FIELD_OFFSET(type,field)    (offsetof(type,field))


/******************************************************************************
 Handle name change of some regsiter bits
 *****************************************************************************/
#define phy_sw_coma     pm_phy_sw_coma


/******************************************************************************
 Define order of magnitude converter
 *****************************************************************************/
#define NANO_IN_A_MICRO 1000


/******************************************************************************
 Do not change these values: if changed, then change also in respective 
 TXdma and Rxdma engines
 *****************************************************************************/
#define NUM_DESC_PER_RING_TX         512    // TX Do not change these values
#define NUM_TCB                      64


/******************************************************************************
 some typedefs for compiler
 *****************************************************************************/
typedef s8  INT8,   *PINT8;
typedef s16 INT16,  *PINT16;
typedef s32 INT32,  *PINT32;
typedef s64 INT64,  *PINT64;

typedef u8  UINT8,  *PUINT8;
typedef u16 UINT16, *PUINT16;
typedef u32 UINT32, *PUINT32;
typedef u64 UINT64, *PUINT64;

typedef u8  UCHAR,  *PUCHAR;




/*===========================================================================*/
/*===========================================================================*/
/*===                START OF GLOBAL REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 structure for tx queue start address reg in global address map 
 located at address 0x0000 
 *****************************************************************************/
typedef union _TXQ_START_ADDR_t
	{
		UINT32 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:22;                           //bits 10-31
			UINT32 txq_start:10;                        //bits 0-9
#else
			UINT32 txq_start:10;                        //bits 0-9
			UINT32 unused:22;                           //bits 10-31
#endif
		} bits;
	} 
	TXQ_START_ADDR_t, *PTXQ_START_ADDR_t;


/******************************************************************************
 structure for tx queue end address reg in global address map
 located at address 0x0004
 *****************************************************************************/
typedef union _TXQ_END_ADDR_t
	{
		UINT32 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:22;                           //bits 10-31
			UINT32 txq_end:10;                          //bits 0-9
#else
			UINT32 txq_end:10;                          //bits 0-9
			UINT32 unused:22;                           //bits 10-31
#endif    
		} bits;
	} 
	TXQ_END_ADDR_t, *PTXQ_END_ADDR_t;


/******************************************************************************
 structure for rx queue start address reg in global address map
 located at address 0x0008
 *****************************************************************************/
typedef union _RXQ_START_ADDR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:22;                           //bits 10-31
			UINT32 rxq_start_addr:10;                   //bits 0-9
#else
			UINT32 rxq_start_addr:10;                   //bits 0-9
			UINT32 unused:22;                           //bits 10-31
#endif
		} bits;
	} 
	RXQ_START_ADDR_t, *PRXQ_START_ADDR_t;


/******************************************************************************
 structure for rx queue end address reg in global address map
 located at address 0x000C
 *****************************************************************************/
typedef union _RXQ_END_ADDR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:22;                           //bits 10-31
			UINT32 rxq_end_addr:10;                     //bits 0-9
#else
			UINT32 rxq_end_addr:10;                     //bits 0-9
			UINT32 unused:22;                           //bits 10-31
#endif
		} bits;
	} 
	RXQ_END_ADDR_t, *PRXQ_END_ADDR_t;


/******************************************************************************
 structure for power management control status reg in global address map
 located at address 0x0010
 *****************************************************************************/
typedef union _PM_CSR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:22;                           //bits 10-31
			UINT32 pm_jagcore_rx_rdy:1;                 //bit 9
			UINT32 pm_jagcore_tx_rdy:1;                 //bit 8
			UINT32 pm_phy_lped_en:1;                    //bit 7
			UINT32 pm_phy_sw_coma:1;                    //bit 6
			UINT32 pm_rxclk_gate:1;                     //bit 5
			UINT32 pm_txclk_gate:1;                     //bit 4
			UINT32 pm_sysclk_gate:1;                    //bit 3
			UINT32 pm_jagcore_rx_en:1;                  //bit 2
			UINT32 pm_jagcore_tx_en:1;                  //bit 1
			UINT32 pm_gigephy_en:1;                     //bit 0
#else
			UINT32 pm_gigephy_en:1;                     //bit 0
			UINT32 pm_jagcore_tx_en:1;                  //bit 1
			UINT32 pm_jagcore_rx_en:1;                  //bit 2
			UINT32 pm_sysclk_gate:1;                    //bit 3
			UINT32 pm_txclk_gate:1;                     //bit 4
			UINT32 pm_rxclk_gate:1;                     //bit 5
			UINT32 pm_phy_sw_coma:1;                    //bit 6
			UINT32 pm_phy_lped_en:1;                    //bit 7
			UINT32 pm_jagcore_tx_rdy:1;                 //bit 8
			UINT32 pm_jagcore_rx_rdy:1;                 //bit 9
			UINT32 unused:22;                           //bits 10-31
#endif
		} bits;
	} 
	PM_CSR_t, *PPM_CSR_t;


/******************************************************************************
 structure for interrupt status reg in global address map
 located at address 0x0018
 *****************************************************************************/
typedef union _INT_STATUS_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused5:11;                          //bits 21-31
			UINT32 slv_timeout:1;                       //bit 20
			UINT32 mac_stat_interrupt:1;                //bit 19
			UINT32 rxmac_interrupt:1;                   //bit 18
			UINT32 txmac_interrupt:1;                   //bit 17
			UINT32 phy_interrupt:1;                     //bit 16
			UINT32 wake_on_lan:1;                       //bit 15
			UINT32 watchdog_interrupt:1;                //bit 14
			UINT32 unused4:4;                           //bits 10-13
			UINT32 rxdma_err:1;                         //bit 9
			UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
			UINT32 rxdma_fb_ring1_low:1;                //bit 7
			UINT32 rxdma_fb_ring0_low:1;                //bit 6
			UINT32 rxdma_xfr_done:1;                    //bit 5
			UINT32 txdma_err:1;                         //bit 4
			UINT32 txdma_isr:1;                         //bit 3
			UINT32 unused3:1;                           //bit 2
			UINT32 unused2:1;                           //bit 1
			UINT32 unused1:1;                           //bit 0
#else
			UINT32 unused1:1;                           //bit 0
			UINT32 unused2:1;                           //bit 1
			UINT32 unused3:1;                           //bit 2
			UINT32 txdma_isr:1;                         //bit 3
			UINT32 txdma_err:1;                         //bit 4
			UINT32 rxdma_xfr_done:1;                    //bit 5
			UINT32 rxdma_fb_ring0_low:1;                //bit 6
			UINT32 rxdma_fb_ring1_low:1;                //bit 7
			UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
			UINT32 rxdma_err:1;                         //bit 9
			UINT32 unused4:4;                           //bits 10-13
			UINT32 watchdog_interrupt:1;                //bit 14
			UINT32 wake_on_lan:1;                       //bit 15
			UINT32 phy_interrupt:1;                     //bit 16
			UINT32 txmac_interrupt:1;                   //bit 17
			UINT32 rxmac_interrupt:1;                   //bit 18
			UINT32 mac_stat_interrupt:1;                //bit 19
			UINT32 slv_timeout:1;                       //bit 20
			UINT32 unused5:11;                          //bits 21-31
#endif
		} bits;
	} 
	INT_STATUS_t, *PINT_STATUS_t;


/******************************************************************************
 structure for interrupt mask reg in global address map
 located at address 0x001C
 *****************************************************************************/
typedef union _INT_MASK_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused5:11;                          //bits 21-31
			UINT32 slv_timeout:1;                       //bit 20
			UINT32 mac_stat_interrupt:1;                //bit 19
			UINT32 rxmac_interrupt:1;                   //bit 18
			UINT32 txmac_interrupt:1;                   //bit 17
			UINT32 phy_interrupt:1;                     //bit 16
			UINT32 wake_on_lan:1;                       //bit 15
			UINT32 unused4:5;                           //bits 10-14
			UINT32 rxdma_err:1;                         //bit 9
			UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
			UINT32 rxdma_fb_ring1_low:1;                //bit 7
			UINT32 rxdma_fb_ring0_low:1;                //bit 6
			UINT32 rxdma_xfr_done:1;                    //bit 5
			UINT32 txdma_err:1;                         //bit 4
			UINT32 txdma_isr:1;                         //bit 3
			UINT32 unused3:1;                           //bit 2
			UINT32 unused2:1;                           //bit 1
			UINT32 unused1:1;                           //bit 0
#else
			UINT32 unused1:1;                           //bit 0
			UINT32 unused2:1;                           //bit 1
			UINT32 unused3:1;                           //bit 2
			UINT32 txdma_isr:1;                         //bit 3
			UINT32 txdma_err:1;                         //bit 4
			UINT32 rxdma_xfr_done:1;                    //bit 5
			UINT32 rxdma_fb_ring0_low:1;                //bit 6
			UINT32 rxdma_fb_ring1_low:1;                //bit 7
			UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
			UINT32 rxdma_err:1;                         //bit 9
			UINT32 unused4:5;                           //bits 10-14
			UINT32 wake_on_lan:1;                       //bit 15
			UINT32 phy_interrupt:1;                     //bit 16
			UINT32 txmac_interrupt:1;                   //bit 17
			UINT32 rxmac_interrupt:1;                   //bit 18
			UINT32 mac_stat_interrupt:1;                //bit 19
			UINT32 slv_timeout:1;                       //bit 20
			UINT32 unused5:11;                          //bits 21-31
#endif
		} bits;
	} 
	INT_MASK_t, *PINT_MASK_t;


/******************************************************************************
 structure for interrupt alias clear mask reg in global address map
 located at address 0x0020
 *****************************************************************************/
typedef union _INT_ALIAS_CLR_EN_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused5:11;                          //bits 21-31
			UINT32 slv_timeout:1;                       //bit 20
			UINT32 mac_stat_interrupt:1;                //bit 19
			UINT32 rxmac_interrupt:1;                   //bit 18
			UINT32 txmac_interrupt:1;                   //bit 17
			UINT32 phy_interrupt:1;                     //bit 16
			UINT32 wake_on_lan:1;                       //bit 15
			UINT32 watchdog_interrupt:1;                //bit 14
			UINT32 unused4:4;                           //bits 10-13
			UINT32 rxdma_err:1;                         //bit 9
			UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
			UINT32 rxdma_fb_ring1_low:1;                //bit 7
			UINT32 rxdma_fb_ring0_low:1;                //bit 6
			UINT32 rxdma_xfr_done:1;                    //bit 5
			UINT32 txdma_err:1;                         //bit 4
			UINT32 txdma_isr:1;                         //bit 3
			UINT32 unused3:1;                           //bit 2
			UINT32 unused2:1;                           //bit 1
			UINT32 unused1:1;                           //bit 0
#else
			UINT32 unused1:1;                           //bit 0
			UINT32 unused2:1;                           //bit 1
			UINT32 unused3:1;                           //bit 2
			UINT32 txdma_isr:1;                         //bit 3
			UINT32 txdma_err:1;                         //bit 4
			UINT32 rxdma_xfr_done:1;                    //bit 5
			UINT32 rxdma_fb_ring0_low:1;                //bit 6
			UINT32 rxdma_fb_ring1_low:1;                //bit 7
			UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
			UINT32 rxdma_err:1;                         //bit 9
			UINT32 unused4:4;                           //bits 10-13
			UINT32 watchdog_interrupt:1;                //bit 14
			UINT32 wake_on_lan:1;                       //bit 15
			UINT32 phy_interrupt:1;                     //bit 16
			UINT32 txmac_interrupt:1;                   //bit 17
			UINT32 rxmac_interrupt:1;                   //bit 18
			UINT32 mac_stat_interrupt:1;                //bit 19
			UINT32 slv_timeout:1;                       //bit 20
			UINT32 unused5:11;                          //bits 21-31
#endif
		} bits;
	} 
	INT_ALIAS_CLR_EN_t, *PINT_ALIAS_CLR_EN_t;


/******************************************************************************
 structure for interrupt status alias reg in global address map
 located at address 0x0024
 *****************************************************************************/
typedef union _INT_STATUS_ALIAS_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused5:11;                          //bits 21-31
			UINT32 slv_timeout:1;                       //bit 20
			UINT32 mac_stat_interrupt:1;                //bit 19
			UINT32 rxmac_interrupt:1;                   //bit 18
			UINT32 txmac_interrupt:1;                   //bit 17
			UINT32 phy_interrupt:1;                     //bit 16
			UINT32 wake_on_lan:1;                       //bit 15
			UINT32 watchdog_interrupt:1;                //bit 14
			UINT32 unused4:4;                           //bits 10-13
			UINT32 rxdma_err:1;                         //bit 9
			UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
			UINT32 rxdma_fb_ring1_low:1;                //bit 7
			UINT32 rxdma_fb_ring0_low:1;                //bit 6
			UINT32 rxdma_xfr_done:1;                    //bit 5
			UINT32 txdma_err:1;                         //bit 4
			UINT32 txdma_isr:1;                         //bit 3
			UINT32 unused3:1;                           //bit 2
			UINT32 unused2:1;                           //bit 1
			UINT32 unused1:1;                           //bit 0
#else
			UINT32 unused1:1;                           //bit 0
			UINT32 unused2:1;                           //bit 1
			UINT32 unused3:1;                           //bit 2
			UINT32 txdma_isr:1;                         //bit 3
			UINT32 txdma_err:1;                         //bit 4
			UINT32 rxdma_xfr_done:1;                    //bit 5
			UINT32 rxdma_fb_ring0_low:1;                //bit 6
			UINT32 rxdma_fb_ring1_low:1;                //bit 7
			UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
			UINT32 rxdma_err:1;                         //bit 9
			UINT32 unused4:4;                           //bits 10-13
			UINT32 watchdog_interrupt:1;                //bit 14
			UINT32 wake_on_lan:1;                       //bit 15
			UINT32 phy_interrupt:1;                     //bit 16
			UINT32 txmac_interrupt:1;                   //bit 17
			UINT32 rxmac_interrupt:1;                   //bit 18
			UINT32 mac_stat_interrupt:1;                //bit 19
			UINT32 slv_timeout:1;                       //bit 20
			UINT32 unused5:11;                          //bits 21-31
#endif
		} bits;
	} 
	INT_STATUS_ALIAS_t, *PINT_STATUS_ALIAS_t;


/******************************************************************************
 structure for software reset reg in global address map
 located at address 0x0028
 *****************************************************************************/
typedef union _SW_RESET_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 selfclr_disable:1;                   //bit 31
			UINT32 unused:24;                           //bits 7-30
			UINT32 mmc_sw_reset:1;                      //bit 6
			UINT32 mac_stat_sw_reset:1;                 //bit 5
			UINT32 mac_sw_reset:1;                      //bit 4
			UINT32 rxmac_sw_reset:1;                    //bit 3
			UINT32 txmac_sw_reset:1;                    //bit 2
			UINT32 rxdma_sw_reset:1;                    //bit 1
			UINT32 txdma_sw_reset:1;                    //bit 0
#else
			UINT32 txdma_sw_reset:1;                    //bit 0
			UINT32 rxdma_sw_reset:1;                    //bit 1
			UINT32 txmac_sw_reset:1;                    //bit 2
			UINT32 rxmac_sw_reset:1;                    //bit 3
			UINT32 mac_sw_reset:1;                      //bit 4
			UINT32 mac_stat_sw_reset:1;                 //bit 5
			UINT32 mmc_sw_reset:1;                      //bit 6
			UINT32 unused:24;                           //bits 7-30
			UINT32 selfclr_disable:1;                   //bit 31
#endif
		} bits;
	} 
	SW_RESET_t, *PSW_RESET_t;


/******************************************************************************
 structure for SLV Timer reg in global address map
 located at address 0x002C
 *****************************************************************************/
typedef union _SLV_TIMER_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:8;                            //bits 24-31
			UINT32 timer_ini:24;                        //bits 0-23
#else
			UINT32 timer_ini:24;                        //bits 0-23
			UINT32 unused:8;                            //bits 24-31
#endif
		} bits;
	} 
	SLV_TIMER_t, *PSLV_TIMER_t;


/******************************************************************************
 structure for MSI Configuration reg in global address map
 located at address 0x0030
 *****************************************************************************/
typedef union _MSI_CONFIG_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused1:13;                          //bits 19-31
			UINT32 msi_tc:3;                            //bits 16-18
			UINT32 unused2:11;                          //bits 5-15
			UINT32 msi_vector:5;                        //bits 0-4
#else
			UINT32 msi_vector:5;                        //bits 0-4
			UINT32 unused2:11;                          //bits 5-15
			UINT32 msi_tc:3;                            //bits 16-18
			UINT32 unused1:13;                          //bits 19-31
#endif
		} bits;
	} 
	MSI_CONFIG_t, *PMSI_CONFIG_t;


/******************************************************************************
 structure for Loopback reg in global address map
 located at address 0x0034
 *****************************************************************************/
typedef union _LOOPBACK_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:30;                           //bits 2-31
			UINT32 dma_loopback:1;                      //bit 1
			UINT32 mac_loopback:1;                      //bit 0
#else
			UINT32 mac_loopback:1;                      //bit 0
			UINT32 dma_loopback:1;                      //bit 1
			UINT32 unused:30;                           //bits 2-31
#endif
		} bits;
	} 
	LOOPBACK_t, *PLOOPBACK_t;


/******************************************************************************
 GLOBAL Module of JAGCore Address Mapping
 Located at address 0x0000
 *****************************************************************************/
typedef struct _GLOBAL_t
	{                                                   //Location:
		TXQ_START_ADDR_t    txq_start_addr;             //  0x0000
		TXQ_END_ADDR_t      txq_end_addr;               //  0x0004
		RXQ_START_ADDR_t    rxq_start_addr;             //  0x0008
		RXQ_END_ADDR_t      rxq_end_addr;               //  0x000C
		PM_CSR_t            pm_csr;                     //  0x0010
		UINT32              unused;                     //  0x0014
		INT_STATUS_t        int_status;                 //  0x0018
		INT_MASK_t          int_mask;                   //  0x001C
		INT_ALIAS_CLR_EN_t  int_alias_clr_en;           //  0x0020
		INT_STATUS_ALIAS_t  int_status_alias;           //  0x0024
		SW_RESET_t          sw_reset;                   //  0x0028
		SLV_TIMER_t         slv_timer;                  //  0x002C
		MSI_CONFIG_t        msi_config;                 //  0x0030
		LOOPBACK_t          loopback;                   //  0x0034
		UINT32              watchdog_timer;             //  0x0038
	} 
	GLOBAL_t, *PGLOBAL_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                 END OF GLOBAL REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/




/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF TXDMA REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 structure for txdma control status reg in txdma address map
 located at address 0x1000
 *****************************************************************************/
typedef union _TXDMA_CSR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused2:19;                          //bits 13-31
			UINT32 traffic_class:4;                     //bits 9-12
			UINT32 sngl_epkt_mode:1;                    //bit 8
			UINT32 cache_thrshld:4;                     //bits 4-7
			UINT32 unused1:2;                           //bits 2-3
			UINT32 drop_TLP_disable:1;                  //bit 1
			UINT32 halt:1;                              //bit 0
#else
			UINT32 halt:1;                              //bit 0
			UINT32 drop_TLP_disable:1;                  //bit 1
			UINT32 unused1:2;                           //bits 2-3
			UINT32 cache_thrshld:4;                     //bits 4-7
			UINT32 sngl_epkt_mode:1;                    //bit 8
			UINT32 traffic_class:4;                     //bits 9-12
			UINT32 unused2:19;                          //bits 13-31
#endif
		} bits;
	} 
	TXDMA_CSR_t, *PTXDMA_CSR_t;


/******************************************************************************
 structure for txdma packet ring base address hi reg in txdma address map
 located at address 0x1004
 *****************************************************************************/
typedef struct _TXDMA_PR_BASE_HI_t
	{
		UINT32 addr_hi;                                 //bits 0-31
	} 
	TXDMA_PR_BASE_HI_t, *PTXDMA_PR_BASE_HI_t;


/******************************************************************************
 structure for txdma packet ring base address low reg in txdma address map
 located at address 0x1008
 *****************************************************************************/
typedef struct _TXDMA_PR_BASE_LO_t
	{
		UINT32 addr_lo;                                 //bits 0-31
	} 
	TXDMA_PR_BASE_LO_t, *PTXDMA_PR_BASE_LO_t;


/******************************************************************************
 structure for txdma packet ring number of descriptor reg in txdma address 
 map.  Located at address 0x100C
 *****************************************************************************/
typedef union _TXDMA_PR_NUM_DES_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:22;                           //bits 10-31
			UINT32 pr_ndes:10;                          //bits 0-9
#else
			UINT32 pr_ndes:10;                          //bits 0-9
			UINT32 unused:22;                           //bits 10-31
#endif
		} bits;
	} 
	TXDMA_PR_NUM_DES_t, *PTXDMA_PR_NUM_DES_t;


/******************************************************************************
 structure for txdma tx queue write address reg in txdma address map
 located at address 0x1010
 *****************************************************************************/
typedef  union _TXDMA_TXQ_WR_ADDR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 txq_wr_wrap:1;                       //bit 10
			UINT32 txq_wr:10;                           //bits 0-9
#else
			UINT32 txq_wr:10;                           //bits 0-9
			UINT32 txq_wr_wrap:1;                       //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	TXDMA_TXQ_WR_ADDR_t, *PTXDMA_TXQ_WR_ADDR_t;


/******************************************************************************
 structure for txdma tx queue write address external reg in txdma address map
 located at address 0x1014
 *****************************************************************************/
typedef union _TXDMA_TXQ_WR_ADDR_EXT_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 txq_wr_ext_wrap:1;                   //bit 10
			UINT32 txq_wr_ext:10;                       //bits 0-9
#else
			UINT32 txq_wr_ext:10;                       //bits 0-9
			UINT32 txq_wr_ext_wrap:1;                   //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	TXDMA_TXQ_WR_ADDR_EXT_t, *PTXDMA_TXQ_WR_ADDR_EXT_t;


/******************************************************************************
 structure for txdma tx queue read address reg in txdma address map
 located at address 0x1018
 *****************************************************************************/
typedef union _TXDMA_TXQ_RD_ADDR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 txq_rd_wrap:1;                       //bit 10
			UINT32 txq_rd:10;                           //bits 0-9
#else
			UINT32 txq_rd:10;                           //bits 0-9
			UINT32 txq_rd_wrap:1;                       //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	TXDMA_TXQ_RD_ADDR_t, *PTXDMA_TXQ_RD_ADDR_t;


/******************************************************************************
 structure for txdma status writeback address hi reg in txdma address map
 located at address 0x101C
 *****************************************************************************/
typedef struct _TXDMA_DMA_WB_ADDR_HI_t
	{
		UINT32 addr_hi;                              //bits 0-31
	} 
	TXDMA_DMA_WB_ADDR_HI_t, *PTXDMA_DMA_WB_ADDR_HI_t;


/******************************************************************************
 structure for txdma status writeback address lo reg in txdma address map
 located at address 0x1020
 *****************************************************************************/
typedef struct _TXDMA_DMA_WB_ADDR_LO_t
	{
		UINT32 addr_lo;                              //bits 0-31
	} 
	TXDMA_DMA_WB_ADDR_LO_t, *PTXDMA_DMA_WB_ADDR_LO_t;


/******************************************************************************
 structure for txdma service request reg in txdma address map
 located at address 0x1024
 *****************************************************************************/
typedef union _TXDMA_SERVICE_REQUEST_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 serv_req_wrap:1;                     //bit 10
			UINT32 serv_req:10;                         //bits 0-9
#else
			UINT32 serv_req:10;                         //bits 0-9
			UINT32 serv_req_wrap:1;                     //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	TXDMA_SERVICE_REQUEST_t, *PTXDMA_SERVICE_REQUEST_t;


/******************************************************************************
 structure for txdma service complete reg in txdma address map
 located at address 0x1028
 *****************************************************************************/
typedef union _TXDMA_SERVICE_COMPLETE_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 serv_cpl_wrap:1;                     //bit 10
			UINT32 serv_cpl:10;                         //bits 0-9
#else
			UINT32 serv_cpl:10;                         //bits 0-9
			UINT32 serv_cpl_wrap:1;                     //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	TXDMA_SERVICE_COMPLETE_t, *PTXDMA_SERVICE_COMPLETE_t;


/******************************************************************************
 structure for txdma tx descriptor cache read index reg in txdma address map
 located at address 0x102C
 *****************************************************************************/
typedef union _TXDMA_CACHE_RD_INDEX_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:27;                           //bits 5-31
			UINT32 rdi_wrap:1;                          //bit 4
			UINT32 rdi:4;                               //bit 0-3
#else
			UINT32 rdi:4;                               //bits 0-3
			UINT32 rdi_wrap:1;                          //bit 4
			UINT32 unused:27;                           //bits 5-31
#endif
		} bits;
	} 
	TXDMA_CACHE_RD_INDEX_t, *PTXDMA_CACHE_RD_INDEX_t;


/******************************************************************************
 structure for txdma tx descriptor cache write index reg in txdma address map
 located at address 0x1030
 *****************************************************************************/
typedef union _TXDMA_CACHE_WR_INDEX_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:27;                           //bits 5-31
			UINT32 wri_wrap:1;                          //bit 4
			UINT32 wri:4;                               //bit 0-3
#else
			UINT32 wri:4;                               //bits 0-3
			UINT32 wri_wrap:1;                          //bit 4
			UINT32 unused:27;                           //bits 5-31
#endif
		} bits;
	} 
	TXDMA_CACHE_WR_INDEX_t, *PTXDMA_CACHE_WR_INDEX_t;


/******************************************************************************
 structure for txdma error reg in txdma address map
 located at address 0x1034
 *****************************************************************************/
typedef union _TXDMA_ERROR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused3:22;                          //bits 10-31
			UINT32 WrbkRewind:1;                        //bit 9
			UINT32 WrbkResend:1;                        //bit 8
			UINT32 unused2:2;                           //bits 6-7
			UINT32 DescrRewind:1;                       //bit 5
			UINT32 DescrResend:1;                       //bit 4
			UINT32 unused1:2;                           //bits 2-3
			UINT32 PyldRewind:1;                        //bit 1
			UINT32 PyldResend:1;                        //bit 0
#else
			UINT32 PyldResend:1;                        //bit 0
			UINT32 PyldRewind:1;                        //bit 1
			UINT32 unused1:2;                           //bits 2-3
			UINT32 DescrResend:1;                       //bit 4
			UINT32 DescrRewind:1;                       //bit 5
			UINT32 unused2:2;                           //bits 6-7
			UINT32 WrbkResend:1;                        //bit 8
			UINT32 WrbkRewind:1;                        //bit 9
			UINT32 unused3:22;                          //bits 10-31
#endif
		} bits;
	}
	TXDMA_ERROR_t, *PTXDMA_ERROR_t;


/******************************************************************************
 Tx DMA Module of JAGCore Address Mapping
 Located at address 0x1000
 *****************************************************************************/
typedef struct _TXDMA_t
	{                                                   //Location:
		TXDMA_CSR_t                 csr;                //  0x1000
		TXDMA_PR_BASE_HI_t          pr_base_hi;         //  0x1004
		TXDMA_PR_BASE_LO_t          pr_base_lo;         //  0x1008
		TXDMA_PR_NUM_DES_t          pr_num_des;         //  0x100C
		TXDMA_TXQ_WR_ADDR_t         txq_wr_addr;        //  0x1010
		TXDMA_TXQ_WR_ADDR_EXT_t     txq_wr_addr_ext;    //  0x1014
		TXDMA_TXQ_RD_ADDR_t         txq_rd_addr;        //  0x1018
		TXDMA_DMA_WB_ADDR_HI_t      dma_wb_base_hi;     //  0x101C
		TXDMA_DMA_WB_ADDR_LO_t      dma_wb_base_lo;     //  0x1020
		TXDMA_SERVICE_REQUEST_t     service_request;    //  0x1024
		TXDMA_SERVICE_COMPLETE_t    service_complete;   //  0x1028
		TXDMA_CACHE_RD_INDEX_t      cache_rd_index;     //  0x102C
		TXDMA_CACHE_WR_INDEX_t      cache_wr_index;     //  0x1030
		TXDMA_ERROR_t               TxDmaError;         //  0x1034
		UINT32                      DescAbortCount;     //  0x1038
		UINT32                      PayloadAbortCnt;    //  0x103c
		UINT32                      WriteBackAbortCnt;  //  0x1040
		UINT32                      DescTimeoutCnt;     //  0x1044
		UINT32                      PayloadTimeoutCnt;  //  0x1048
		UINT32                      WriteBackTimeoutCnt;//  0x104c
		UINT32                      DescErrorCount;     //  0x1050
		UINT32                      PayloadErrorCnt;    //  0x1054
		UINT32                      WriteBackErrorCnt;  //  0x1058
		UINT32                      DroppedTLPCount;    //  0x105c
		TXDMA_SERVICE_COMPLETE_t    NewServiceComplete; //  0x1060
		UINT32                      EthernetPacketCount;//  0x1064
	}
	TXDMA_t, *PTXDMA_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF TXDMA REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF RXDMA REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 structure for control status reg in rxdma address map
 Located at address 0x2000
 *****************************************************************************/
typedef union _RXDMA_CSR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused2:14;                          //bits 18-31
			UINT32 halt_status:1;                       //bit 17
			UINT32 pkt_done_flush:1;                    //bit 16
			UINT32 pkt_drop_disable:1;                  //bit 15
			UINT32 unused1:1;                           //bit 14
			UINT32 fbr1_enable:1;                       //bit 13
			UINT32 fbr1_size:2;                         //bits 11-12
			UINT32 fbr0_enable:1;                       //bit 10
			UINT32 fbr0_size:2;                         //bits 8-9
			UINT32 dma_big_endian:1;                    //bit 7
			UINT32 pkt_big_endian:1;                    //bit 6
			UINT32 psr_big_endian:1;                    //bit 5
			UINT32 fbr_big_endian:1;                    //bit 4
			UINT32 tc:3;                                //bits 1-3
			UINT32 halt:1;                              //bit 0
#else
			UINT32 halt:1;                              //bit 0
			UINT32 tc:3;                                //bits 1-3
			UINT32 fbr_big_endian:1;                    //bit 4
			UINT32 psr_big_endian:1;                    //bit 5
			UINT32 pkt_big_endian:1;                    //bit 6
			UINT32 dma_big_endian:1;                    //bit 7
			UINT32 fbr0_size:2;                         //bits 8-9
			UINT32 fbr0_enable:1;                       //bit 10
			UINT32 fbr1_size:2;                         //bits 11-12
			UINT32 fbr1_enable:1;                       //bit 13
			UINT32 unused1:1;                           //bit 14
			UINT32 pkt_drop_disable:1;                  //bit 15
			UINT32 pkt_done_flush:1;                    //bit 16
			UINT32 halt_status:1;                       //bit 17
			UINT32 unused2:14;                          //bits 18-31
#endif
		} bits;
	} 
	RXDMA_CSR_t, *PRXDMA_CSR_t;


/******************************************************************************
 structure for dma writeback lo reg in rxdma address map
 located at address 0x2004
 *****************************************************************************/
typedef struct _RXDMA_DMA_WB_BASE_LO_t
	{
		UINT32 addr_lo;                                 //bits 0-31
	} 
	RXDMA_DMA_WB_BASE_LO_t, *PRXDMA_DMA_WB_BASE_LO_t;


/******************************************************************************
 structure for dma writeback hi reg in rxdma address map
 located at address 0x2008
 *****************************************************************************/
typedef struct _RXDMA_DMA_WB_BASE_HI_t
	{
		UINT32 addr_hi;                                 //bits 0-31
	} 
	RXDMA_DMA_WB_BASE_HI_t, *PRXDMA_DMA_WB_BASE_HI_t;


/******************************************************************************
 structure for number of packets done reg in rxdma address map
 located at address 0x200C
 *****************************************************************************/
typedef union _RXDMA_NUM_PKT_DONE_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:24;                           //bits 8-31
			UINT32 num_done:8;                          //bits 0-7
#else
			UINT32 num_done:8;                          //bits 0-7
			UINT32 unused:24;                           //bits 8-31
#endif
		} bits;
	} 
	RXDMA_NUM_PKT_DONE_t, *PRXDMA_NUM_PKT_DONE_t;


/******************************************************************************
 structure for max packet time reg in rxdma address map
 located at address 0x2010
 *****************************************************************************/
typedef union _RXDMA_MAX_PKT_TIME_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:14;                           //bits 18-31
			UINT32 time_done:18;                        //bits 0-17
#else
			UINT32 time_done:18;                        //bits 0-17
			UINT32 unused:14;                           //bits 18-31
#endif
		} bits;
	} 
	RXDMA_MAX_PKT_TIME_t, *PRXDMA_MAX_PKT_TIME_t;


/******************************************************************************
 structure for rx queue read address reg in rxdma address map
 located at address 0x2014
 *****************************************************************************/
typedef union _RXDMA_RXQ_RD_ADDR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 rxq_rd_wrap:1;                       //bit 10
			UINT32 rxq_rd:10;                           //bits 0-9
#else
			UINT32 rxq_rd:10;                           //bits 0-9
			UINT32 rxq_rd_wrap:1;                       //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	RXDMA_RXQ_RD_ADDR_t, *PRXDMA_RXQ_RD_ADDR_t;


/******************************************************************************
 structure for rx queue read address external reg in rxdma address map
 located at address 0x2018
 *****************************************************************************/
typedef union _RXDMA_RXQ_RD_ADDR_EXT_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 rxq_rd_ext_wrap:1;                   //bit 10
			UINT32 rxq_rd_ext:10;                       //bits 0-9
#else
			UINT32 rxq_rd_ext:10;                       //bits 0-9
			UINT32 rxq_rd_ext_wrap:1;                   //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	RXDMA_RXQ_RD_ADDR_EXT_t, *PRXDMA_RXQ_RD_ADDR_EXT_t;


/******************************************************************************
 structure for rx queue write address reg in rxdma address map
 located at address 0x201C
 *****************************************************************************/
typedef union _RXDMA_RXQ_WR_ADDR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 rxq_wr_wrap:1;                       //bit 10
			UINT32 rxq_wr:10;                           //bits 0-9
#else
			UINT32 rxq_wr:10;                           //bits 0-9
			UINT32 rxq_wr_wrap:1;                       //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	RXDMA_RXQ_WR_ADDR_t, *PRXDMA_RXQ_WR_ADDR_t;


/******************************************************************************
 structure for packet status ring base address lo reg in rxdma address map
 located at address 0x2020
 *****************************************************************************/
typedef struct _RXDMA_PSR_BASE_LO_t
	{
		UINT32 addr_lo;                                 //bits 0-31
	} 
	RXDMA_PSR_BASE_LO_t, *PRXDMA_PSR_BASE_LO_t;


/******************************************************************************
 structure for packet status ring base address hi reg in rxdma address map
 located at address 0x2024
 *****************************************************************************/
typedef struct _RXDMA_PSR_BASE_HI_t
	{
		UINT32 addr_hi;                                  //bits 0-31
	} 
	RXDMA_PSR_BASE_HI_t, *PRXDMA_PSR_BASE_HI_t;


/******************************************************************************
 structure for packet status ring number of descriptors reg in rxdma address 
 map.  Located at address 0x2028
 *****************************************************************************/
typedef union _RXDMA_PSR_NUM_DES_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:20;                           //bits 12-31
			UINT32 psr_ndes:12;                         //bit 0-11
#else
			UINT32 psr_ndes:12;                         //bit 0-11
			UINT32 unused:20;                           //bits 12-31
#endif
		} bits;
	} 
	RXDMA_PSR_NUM_DES_t, *PRXDMA_PSR_NUM_DES_t;


/******************************************************************************
 structure for packet status ring available offset reg in rxdma address map
 located at address 0x202C
 *****************************************************************************/
typedef union _RXDMA_PSR_AVAIL_OFFSET_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:19;                           //bits 13-31
			UINT32 psr_avail_wrap:1;                    //bit 12
			UINT32 psr_avail:12;                        //bit 0-11
#else
			UINT32 psr_avail:12;                        //bit 0-11
			UINT32 psr_avail_wrap:1;                    //bit 12
			UINT32 unused:19;                           //bits 13-31
#endif
		} bits;
	} 
	RXDMA_PSR_AVAIL_OFFSET_t, *PRXDMA_PSR_AVAIL_OFFSET_t;


/******************************************************************************
 structure for packet status ring full offset reg in rxdma address map
 located at address 0x2030
 *****************************************************************************/
typedef union _RXDMA_PSR_FULL_OFFSET_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:19;                           //bits 13-31
			UINT32 psr_full_wrap:1;                     //bit 12
			UINT32 psr_full:12;                         //bit 0-11
#else
			UINT32 psr_full:12;                         //bit 0-11
			UINT32 psr_full_wrap:1;                     //bit 12
			UINT32 unused:19;                           //bits 13-31
#endif
		} bits;
	} 
	RXDMA_PSR_FULL_OFFSET_t, *PRXDMA_PSR_FULL_OFFSET_t;


/******************************************************************************
 structure for packet status ring access index reg in rxdma address map
 located at address 0x2034
 *****************************************************************************/
typedef union _RXDMA_PSR_ACCESS_INDEX_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:27;                           //bits 5-31
			UINT32 psr_ai:5;                            //bits 0-4
#else
			UINT32 psr_ai:5;                            //bits 0-4
			UINT32 unused:27;                           //bits 5-31
#endif
		} bits;
	} 
	RXDMA_PSR_ACCESS_INDEX_t, *PRXDMA_PSR_ACCESS_INDEX_t;


/******************************************************************************
 structure for packet status ring minimum descriptors reg in rxdma address 
 map.  Located at address 0x2038
 *****************************************************************************/
typedef union _RXDMA_PSR_MIN_DES_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:20;                           //bits 12-31
			UINT32 psr_min:12;                          //bits 0-11
#else
			UINT32 psr_min:12;                          //bits 0-11
			UINT32 unused:20;                           //bits 12-31
#endif
		} bits;
	} 
	RXDMA_PSR_MIN_DES_t, *PRXDMA_PSR_MIN_DES_t;


/******************************************************************************
 structure for free buffer ring base lo address reg in rxdma address map
 located at address 0x203C
 *****************************************************************************/
typedef struct _RXDMA_FBR_BASE_LO_t
	{
		UINT32 addr_lo;                                 //bits 0-31
	} 
	RXDMA_FBR_BASE_LO_t, *PRXDMA_FBR_BASE_LO_t;


/******************************************************************************
 structure for free buffer ring base hi address reg in rxdma address map
 located at address 0x2040
 *****************************************************************************/
typedef struct _RXDMA_FBR_BASE_HI_t
	{
		UINT32 addr_hi;                                 //bits 0-31
	} 
	RXDMA_FBR_BASE_HI_t, *PRXDMA_FBR_BASE_HI_t;


/******************************************************************************
 structure for free buffer ring number of descriptors reg in rxdma address 
 map.  Located at address 0x2044
 *****************************************************************************/
typedef union _RXDMA_FBR_NUM_DES_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:22;                           //bits 10-31
			UINT32 fbr_ndesc:10;                        //bits 0-9
#else
			UINT32 fbr_ndesc:10;                        //bits 0-9
			UINT32 unused:22;                           //bits 10-31
#endif
		} bits;
	} 
	RXDMA_FBR_NUM_DES_t, *PRXDMA_FBR_NUM_DES_t;


/******************************************************************************
 structure for free buffer ring 0 available offset reg in rxdma address map  
 located at address 0x2048
 *****************************************************************************/
typedef union _RXDMA_FBR_AVAIL_OFFSET_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 fbr_avail_wrap:1;                    //bit 10
			UINT32 fbr_avail:10;                        //bit 0-9
#else
			UINT32 fbr_avail:10;                        //bit 0-9
			UINT32 fbr_avail_wrap:1;                    //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	RXDMA_FBR_AVAIL_OFFSET_t, *PRXDMA_FBR_AVAIL_OFFSET_t;


/******************************************************************************
 structure for free buffer ring 0 full offset reg in rxdma address map  
 located at address 0x204C
 *****************************************************************************/
typedef union _RXDMA_FBR_FULL_OFFSET_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 fbr_full_wrap:1;                     //bit 10
			UINT32 fbr_full:10;                         //bit 0-9
#else
			UINT32 fbr_full:10;                         //bit 0-9
			UINT32 fbr_full_wrap:1;                     //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
	} 
	RXDMA_FBR_FULL_OFFSET_t, *PRXDMA_FBR_FULL_OFFSET_t;


/******************************************************************************
 structure for free buffer cache 0 full offset reg in rxdma address map  
 located at address 0x2050
 *****************************************************************************/
typedef union _RXDMA_FBC_RD_INDEX_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:27;                           //bits 5-31
			UINT32 fbc_rdi:5;                          //bit 0-4
#else
			UINT32 fbc_rdi:5;                          //bit 0-4
			UINT32 unused:27;                           //bits 5-31
#endif
		} bits;
	} 
	RXDMA_FBC_RD_INDEX_t, *PRXDMA_FBC_RD_INDEX_t;


/******************************************************************************
 structure for free buffer ring 0 minimum descriptor reg in rxdma address map
 located at address 0x2054
 *****************************************************************************/
typedef union _RXDMA_FBR_MIN_DES_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:22;                           //bits 10-31
			UINT32 fbr_min:10;                          //bits 0-9
#else
			UINT32 fbr_min:10;                          //bits 0-9
			UINT32 unused:22;                           //bits 10-31
#endif
		} bits;
	} 
	RXDMA_FBR_MIN_DES_t, *PRXDMA_FBR_MIN_DES_t;


/******************************************************************************
 structure for free buffer ring 1 base address lo reg in rxdma address map
 located at address 0x2058 - 0x205C
 Defined earlier (RXDMA_FBR_BASE_LO_t and RXDMA_FBR_BASE_HI_t)
 *****************************************************************************/

/******************************************************************************
 structure for free buffer ring 1 number of descriptors reg in rxdma address 
 map.  Located at address 0x2060
 Defined earlier (RXDMA_FBR_NUM_DES_t)
 *****************************************************************************/

/******************************************************************************
 structure for free buffer ring 1 available offset reg in rxdma address map
 located at address 0x2064
 Defined Earlier (RXDMA_FBR_AVAIL_OFFSET_t)
 *****************************************************************************/

/******************************************************************************
 structure for free buffer ring 1 full offset reg in rxdma address map
 located at address 0x2068
 Defined Earlier (RXDMA_FBR_FULL_OFFSET_t)
 *****************************************************************************/

/******************************************************************************
 structure for free buffer cache 1 read index reg in rxdma address map
 located at address 0x206C
 Defined Earlier (RXDMA_FBC_RD_INDEX_t)
 *****************************************************************************/

/******************************************************************************
 structure for free buffer ring 1 minimum descriptor reg in rxdma address map
 located at address 0x2070
 Defined Earlier (RXDMA_FBR_MIN_DES_t)
 *****************************************************************************/


/******************************************************************************
 Rx DMA Module of JAGCore Address Mapping  
 Located at address 0x2000
 *****************************************************************************/
typedef struct _RXDMA_t
	{                                                   //Location:
		RXDMA_CSR_t                 csr;                //  0x2000
		RXDMA_DMA_WB_BASE_LO_t      dma_wb_base_lo;     //  0x2004
		RXDMA_DMA_WB_BASE_HI_t      dma_wb_base_hi;     //  0x2008
		RXDMA_NUM_PKT_DONE_t        num_pkt_done;       //  0x200C
		RXDMA_MAX_PKT_TIME_t        max_pkt_time;       //  0x2010
		RXDMA_RXQ_RD_ADDR_t         rxq_rd_addr;        //  0x2014
		RXDMA_RXQ_RD_ADDR_EXT_t     rxq_rd_addr_ext;    //  0x2018
		RXDMA_RXQ_WR_ADDR_t         rxq_wr_addr;        //  0x201C
		RXDMA_PSR_BASE_LO_t         psr_base_lo;        //  0x2020
		RXDMA_PSR_BASE_HI_t         psr_base_hi;        //  0x2024
		RXDMA_PSR_NUM_DES_t         psr_num_des;        //  0x2028
		RXDMA_PSR_AVAIL_OFFSET_t    psr_avail_offset;   //  0x202C
		RXDMA_PSR_FULL_OFFSET_t     psr_full_offset;    //  0x2030
		RXDMA_PSR_ACCESS_INDEX_t    psr_access_index;   //  0x2034
		RXDMA_PSR_MIN_DES_t         psr_min_des;        //  0x2038
		RXDMA_FBR_BASE_LO_t         fbr0_base_lo;       //  0x203C
		RXDMA_FBR_BASE_HI_t         fbr0_base_hi;       //  0x2040
		RXDMA_FBR_NUM_DES_t         fbr0_num_des;       //  0x2044
		RXDMA_FBR_AVAIL_OFFSET_t    fbr0_avail_offset;  //  0x2048
		RXDMA_FBR_FULL_OFFSET_t     fbr0_full_offset;   //  0x204C
		RXDMA_FBC_RD_INDEX_t        fbr0_rd_index;      //  0x2050
		RXDMA_FBR_MIN_DES_t         fbr0_min_des;       //  0x2054
		RXDMA_FBR_BASE_LO_t         fbr1_base_lo;       //  0x2058
		RXDMA_FBR_BASE_HI_t         fbr1_base_hi;       //  0x205C
		RXDMA_FBR_NUM_DES_t         fbr1_num_des;       //  0x2060
		RXDMA_FBR_AVAIL_OFFSET_t    fbr1_avail_offset;  //  0x2064
		RXDMA_FBR_FULL_OFFSET_t     fbr1_full_offset;   //  0x2068
		RXDMA_FBC_RD_INDEX_t        fbr1_rd_index;      //  0x206C
		RXDMA_FBR_MIN_DES_t         fbr1_min_des;       //  0x2070
	}
	RXDMA_t, *PRXDMA_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF RXDMA REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF TXMAC REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 structure for control reg in txmac address map
 located at address 0x3000
 *****************************************************************************/
typedef union _TXMAC_CTL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:24;                           //bits 8-31
			UINT32 cklseg_diable:1;                     //bit 7
			UINT32 ckbcnt_disable:1;                    //bit 6
			UINT32 cksegnum:1;                          //bit 5
			UINT32 async_disable:1;                     //bit 4
			UINT32 fc_disable:1;                        //bit 3
			UINT32 mcif_disable:1;                      //bit 2
			UINT32 mif_disable:1;                       //bit 1
			UINT32 txmac_en:1;                          //bit 0
#else
			UINT32 txmac_en:1;                          //bit 0
			UINT32 mif_disable:1;                       //bit 1 mac interface
			UINT32 mcif_disable:1;                      //bit 2 memory controller interface
			UINT32 fc_disable:1;                        //bit 3
			UINT32 async_disable:1;                     //bit 4
			UINT32 cksegnum:1;                          //bit 5
			UINT32 ckbcnt_disable:1;                    //bit 6
			UINT32 cklseg_diable:1;                     //bit 7
			UINT32 unused:24;                           //bits 8-31
#endif
		} bits;
	} 
	TXMAC_CTL_t, *PTXMAC_CTL_t;


/******************************************************************************
 structure for shadow pointer reg in txmac address map
 located at address 0x3004
 *****************************************************************************/
typedef union _TXMAC_SHADOW_PTR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved2:5;                         //bits 27-31
			UINT32 txq_rd_ptr:11;                       //bits 16-26
			UINT32 reserved:5;                          //bits 11-15
			UINT32 txq_wr_ptr:11;                       //bits 0-10
#else
			UINT32 txq_wr_ptr:11;                       //bits 0-10
			UINT32 reserved:5;                          //bits 11-15
			UINT32 txq_rd_ptr:11;                       //bits 16-26
			UINT32 reserved2:5;                         //bits 27-31
#endif
		} bits;
	} 
	TXMAC_SHADOW_PTR_t, *PTXMAC_SHADOW_PTR_t;


/******************************************************************************
 structure for error count reg in txmac address map
 located at address 0x3008
 *****************************************************************************/
typedef union _TXMAC_ERR_CNT_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:20;                           //bits 12-31
			UINT32 reserved:4;                          //bits 8-11
			UINT32 txq_underrun:4;                      //bits 4-7
			UINT32 fifo_underrun:4;                     //bits 0-3
#else
			UINT32 fifo_underrun:4;                     //bits 0-3
			UINT32 txq_underrun:4;                      //bits 4-7
			UINT32 reserved:4;                          //bits 8-11
			UINT32 unused:20;                           //bits 12-31
#endif
		} bits;
	} TXMAC_ERR_CNT_t, *PTXMAC_ERR_CNT_t;


/******************************************************************************
 structure for max fill reg in txmac address map
 located at address 0x300C
 *****************************************************************************/
typedef union _TXMAC_MAX_FILL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:20;                           //bits 12-31
			UINT32 max_fill:12;                         //bits 0-11
#else
			UINT32 max_fill:12;                         //bits 0-11
			UINT32 unused:20;                           //bits 12-31
#endif
		} bits;
	} 
	TXMAC_MAX_FILL_t, *PTXMAC_MAX_FILL_t;


/******************************************************************************
 structure for cf parameter reg in txmac address map
 located at address 0x3010
 *****************************************************************************/
typedef union _TXMAC_CF_PARAM_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 cfep:16;                             //bits 16-31
			UINT32 cfpt:16;                             //bits 0-15
#else
			UINT32 cfpt:16;                             //bits 0-15
			UINT32 cfep:16;                             //bits 16-31
#endif
		} bits;
	} 
	TXMAC_CF_PARAM_t, *PTXMAC_CF_PARAM_t;


/******************************************************************************
 structure for tx test reg in txmac address map
 located at address 0x3014
 *****************************************************************************/
typedef union _TXMAC_TXTEST_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused2:15;                          //bits 17-31
			UINT32 reserved1:1;                         //bit 16
			UINT32 txtest_en:1;                         //bit 15
			UINT32 unused1:4;                           //bits 11-14
			UINT32 txqtest_ptr:11;                      //bits 0-11
#else
			UINT32 txqtest_ptr:11;                      //bits 0-10
			UINT32 unused1:4;                           //bits 11-14
			UINT32 txtest_en:1;                         //bit 15
			UINT32 reserved1:1;                         //bit 16
			UINT32 unused2:15;                          //bits 17-31
#endif
		} bits;
	} 
	TXMAC_TXTEST_t, *PTXMAC_TXTEST_t;


/******************************************************************************
 structure for error reg in txmac address map
 located at address 0x3018
 *****************************************************************************/
typedef union _TXMAC_ERR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused2:23;                          //bits 9-31
			UINT32 fifo_underrun:1;                     //bit 8
			UINT32 unused1:2;                           //bits 6-7
			UINT32 ctrl2_err:1;                         //bit 5
			UINT32 txq_underrun:1;                      //bit 4
			UINT32 bcnt_err:1;                          //bit 3
			UINT32 lseg_err:1;                          //bit 2
			UINT32 segnum_err:1;                        //bit 1
			UINT32 seg0_err:1;                          //bit 0
#else
			UINT32 seg0_err:1;                          //bit 0
			UINT32 segnum_err:1;                        //bit 1
			UINT32 lseg_err:1;                          //bit 2
			UINT32 bcnt_err:1;                          //bit 3
			UINT32 txq_underrun:1;                      //bit 4
			UINT32 ctrl2_err:1;                         //bit 5
			UINT32 unused1:2;                           //bits 6-7
			UINT32 fifo_underrun:1;                     //bit 8
			UINT32 unused2:23;                          //bits 9-31
#endif
		} bits;
	} 
	TXMAC_ERR_t, *PTXMAC_ERR_t;


/******************************************************************************
 structure for error interrupt reg in txmac address map
 located at address 0x301C
 *****************************************************************************/
typedef union _TXMAC_ERR_INT_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused2:23;                          //bits 9-31
			UINT32 fifo_underrun:1;                     //bit 8
			UINT32 unused1:2;                           //bits 6-7
			UINT32 ctrl2_err:1;                         //bit 5
			UINT32 txq_underrun:1;                      //bit 4
			UINT32 bcnt_err:1;                          //bit 3
			UINT32 lseg_err:1;                          //bit 2
			UINT32 segnum_err:1;                        //bit 1
			UINT32 seg0_err:1;                          //bit 0
#else
			UINT32 seg0_err:1;                          //bit 0
			UINT32 segnum_err:1;                        //bit 1
			UINT32 lseg_err:1;                          //bit 2
			UINT32 bcnt_err:1;                          //bit 3
			UINT32 txq_underrun:1;                      //bit 4
			UINT32 ctrl2_err:1;                         //bit 5
			UINT32 unused1:2;                           //bits 6-7
			UINT32 fifo_underrun:1;                     //bit 8
			UINT32 unused2:23;                          //bits 9-31
#endif
		} bits;
	} 
	TXMAC_ERR_INT_t, *PTXMAC_ERR_INT_t;


/******************************************************************************
 structure for error interrupt reg in txmac address map
 located at address 0x3020
 *****************************************************************************/
typedef union _TXMAC_CP_CTRL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:30;                           //bits 2-31
			UINT32 bp_req:1;                            //bit 1
			UINT32 bp_xonxoff:1;                        //bit 0
#else
			UINT32 bp_xonxoff:1;                        //bit 0
			UINT32 bp_req:1;                            //bit 1
			UINT32 unused:30;                           //bits 2-31
#endif
		} bits;
	} 
	TXMAC_BP_CTRL_t, *PTXMAC_BP_CTRL_t;


/******************************************************************************
 Tx MAC Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _TXMAC_t
	{                                                   //Location:
		TXMAC_CTL_t             ctl;                    //  0x3000
		TXMAC_SHADOW_PTR_t      shadow_ptr;             //  0x3004
		TXMAC_ERR_CNT_t         err_cnt;                //  0x3008
		TXMAC_MAX_FILL_t        max_fill;               //  0x300C
		TXMAC_CF_PARAM_t        cf_param;               //  0x3010
		TXMAC_TXTEST_t          tx_test;                //  0x3014
		TXMAC_ERR_t             err;                    //  0x3018
		TXMAC_ERR_INT_t         err_int;                //  0x301C
		TXMAC_BP_CTRL_t         bp_ctrl;                //  0x3020
	}
	TXMAC_t, *PTXMAC_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF TXMAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF RXMAC REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 structure for rxmac control reg in rxmac address map
 located at address 0x4000
 *****************************************************************************/
typedef union _RXMAC_CTRL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:25;                         //bits 7-31
			UINT32 rxmac_int_disable:1;                 //bit 6
			UINT32 async_disable:1;                     //bit 5
			UINT32 mif_disable:1;                       //bit 4
			UINT32 wol_disable:1;                       //bit 3
			UINT32 pkt_filter_disable:1;                //bit 2
			UINT32 mcif_disable:1;                      //bit 1
			UINT32 rxmac_en:1;                          //bit 0
#else
			UINT32 rxmac_en:1;                          //bit 0
			UINT32 mcif_disable:1;                      //bit 1
			UINT32 pkt_filter_disable:1;                //bit 2
			UINT32 wol_disable:1;                       //bit 3
			UINT32 mif_disable:1;                       //bit 4
			UINT32 async_disable:1;                     //bit 5
			UINT32 rxmac_int_disable:1;                 //bit 6
			UINT32 reserved:25;                         //bits 7-31
#endif
		} bits;
	} 
	RXMAC_CTRL_t, *PRXMAC_CTRL_t;


/******************************************************************************
 structure for Wake On Lan Control and CRC 0 reg in rxmac address map
 located at address 0x4004
 *****************************************************************************/
typedef union _RXMAC_WOL_CTL_CRC0_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 crc0:16;                             //bits 16-31
			UINT32 reserve:4;                           //bits 12-15
			UINT32 ignore_pp:1;                         //bit 11
			UINT32 ignore_mp:1;                         //bit 10
			UINT32 clr_intr:1;                          //bit 9
			UINT32 ignore_link_chg:1;                   //bit 8
			UINT32 ignore_uni:1;                        //bit 7
			UINT32 ignore_multi:1;                      //bit 6
			UINT32 ignore_broad:1;                      //bit 5
			UINT32 valid_crc4:1;                        //bit 4
			UINT32 valid_crc3:1;                        //bit 3
			UINT32 valid_crc2:1;                        //bit 2
			UINT32 valid_crc1:1;                        //bit 1
			UINT32 valid_crc0:1;                        //bit 0
#else
			UINT32 valid_crc0:1;                        //bit 0
			UINT32 valid_crc1:1;                        //bit 1
			UINT32 valid_crc2:1;                        //bit 2
			UINT32 valid_crc3:1;                        //bit 3
			UINT32 valid_crc4:1;                        //bit 4
			UINT32 ignore_broad:1;                      //bit 5
			UINT32 ignore_multi:1;                      //bit 6
			UINT32 ignore_uni:1;                        //bit 7
			UINT32 ignore_link_chg:1;                   //bit 8
			UINT32 clr_intr:1;                          //bit 9
			UINT32 ignore_mp:1;                         //bit 10
			UINT32 ignore_pp:1;                         //bit 11
			UINT32 reserve:4;                           //bits 12-15
			UINT32 crc0:16;                             //bits 16-31
#endif
		} bits;
	} 
	RXMAC_WOL_CTL_CRC0_t, *PRXMAC_WOL_CTL_CRC0_t;


/******************************************************************************
 structure for CRC 1 and CRC 2 reg in rxmac address map
 located at address 0x4008
 *****************************************************************************/
typedef union _RXMAC_WOL_CRC12_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 crc2:16;                             //bits 16-31
			UINT32 crc1:16;                             //bits 0-15
#else
			UINT32 crc1:16;                             //bits 0-15
			UINT32 crc2:16;                             //bits 16-31
#endif
		} bits;
	} 
	RXMAC_WOL_CRC12_t, *PRXMAC_WOL_CRC12_t;


/******************************************************************************
 structure for CRC 3 and CRC 4 reg in rxmac address map
 located at address 0x400C
 *****************************************************************************/
typedef union _RXMAC_WOL_CRC34_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 crc4:16;                             //bits 16-31
			UINT32 crc3:16;                             //bits 0-15
#else
			UINT32 crc3:16;                             //bits 0-15
			UINT32 crc4:16;                             //bits 16-31
#endif
		} bits;
	} 
	RXMAC_WOL_CRC34_t, *PRXMAC_WOL_CRC34_t;


/******************************************************************************
 structure for Wake On Lan Source Address Lo reg in rxmac address map
 located at address 0x4010
 *****************************************************************************/
typedef union _RXMAC_WOL_SA_LO_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 sa3:8;                               //bits 24-31
			UINT32 sa4:8;                               //bits 16-23
			UINT32 sa5:8;                               //bits 8-15
			UINT32 sa6:8;                               //bits 0-7
#else
			UINT32 sa6:8;                               //bits 0-7
			UINT32 sa5:8;                               //bits 8-15
			UINT32 sa4:8;                               //bits 16-23
			UINT32 sa3:8;                               //bits 24-31
#endif
		} bits;
	} 
	RXMAC_WOL_SA_LO_t, *PRXMAC_WOL_SA_LO_t;


/******************************************************************************
 structure for Wake On Lan Source Address Hi reg in rxmac address map
 located at address 0x4014
 *****************************************************************************/
typedef union _RXMAC_WOL_SA_HI_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:16;                         //bits 16-31
			UINT32 sa1:8;                               //bits 8-15
			UINT32 sa2:8;                               //bits 0-7
#else
			UINT32 sa2:8;                               //bits 0-7
			UINT32 sa1:8;                               //bits 8-15
			UINT32 reserved:16;                         //bits 16-31
#endif
		} bits;
	} 
	RXMAC_WOL_SA_HI_t, *PRXMAC_WOL_SA_HI_t;


/******************************************************************************
 structure for Wake On Lan mask reg in rxmac address map
 located at address 0x4018 - 0x4064
 *****************************************************************************/
typedef struct _RXMAC_WOL_MASK_t
	{
		UINT32 mask;                                    //bits 0-31
	} 
	RXMAC_WOL_MASK_t, *PRXMAC_WOL_MASK_t;


/******************************************************************************
 structure for Unicast Paket Filter Address 1 reg in rxmac address map
 located at address 0x4068
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR1_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 addr1_3:8;                           //bits 24-31
			UINT32 addr1_4:8;                           //bits 16-23
			UINT32 addr1_5:8;                           //bits 8-15
			UINT32 addr1_6:8;                           //bits 0-7
#else
			UINT32 addr1_6:8;                           //bits 0-7
			UINT32 addr1_5:8;                           //bits 8-15
			UINT32 addr1_4:8;                           //bits 16-23
			UINT32 addr1_3:8;                           //bits 24-31
#endif
		} bits;
	} 
	RXMAC_UNI_PF_ADDR1_t, *PRXMAC_UNI_PF_ADDR1_t;


/******************************************************************************
 structure for Unicast Paket Filter Address 2 reg in rxmac address map
 located at address 0x406C
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR2_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 addr2_3:8;                           //bits 24-31
			UINT32 addr2_4:8;                           //bits 16-23
			UINT32 addr2_5:8;                           //bits 8-15
			UINT32 addr2_6:8;                           //bits 0-7
#else
			UINT32 addr2_6:8;                           //bits 0-7
			UINT32 addr2_5:8;                           //bits 8-15
			UINT32 addr2_4:8;                           //bits 16-23
			UINT32 addr2_3:8;                           //bits 24-31
#endif
		} bits;
	} 
	RXMAC_UNI_PF_ADDR2_t, *PRXMAC_UNI_PF_ADDR2_t;


/******************************************************************************
 structure for Unicast Paket Filter Address 1 & 2 reg in rxmac address map
 located at address 0x4070
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR3_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 addr2_1:8;                           //bits 24-31
			UINT32 addr2_2:8;                           //bits 16-23
			UINT32 addr1_1:8;                           //bits 8-15
			UINT32 addr1_2:8;                           //bits 0-7
#else
			UINT32 addr1_2:8;                           //bits 0-7
			UINT32 addr1_1:8;                           //bits 8-15
			UINT32 addr2_2:8;                           //bits 16-23
			UINT32 addr2_1:8;                           //bits 24-31
#endif
		} bits;
	} 
	RXMAC_UNI_PF_ADDR3_t, *PRXMAC_UNI_PF_ADDR3_t;


/******************************************************************************
 structure for Multicast Hash reg in rxmac address map
 located at address 0x4074 - 0x4080
 *****************************************************************************/
typedef struct _RXMAC_MULTI_HASH_t
	{
		UINT32 hash;                                    //bits 0-31
	} 
	RXMAC_MULTI_HASH_t, *PRXMAC_MULTI_HASH_t;


/******************************************************************************
 structure for Packet Filter Control reg in rxmac address map
 located at address 0x4084
 *****************************************************************************/
typedef union _RXMAC_PF_CTRL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused2:9;                           //bits 23-31
			UINT32 min_pkt_size:7;                      //bits 16-22
			UINT32 unused1:12;                          //bits 4-15
			UINT32 filter_frag_en:1;                    //bit 3
			UINT32 filter_uni_en:1;                     //bit 2
			UINT32 filter_multi_en:1;                   //bit 1
			UINT32 filter_broad_en:1;                   //bit 0
#else
			UINT32 filter_broad_en:1;                   //bit 0
			UINT32 filter_multi_en:1;                   //bit 1
			UINT32 filter_uni_en:1;                     //bit 2
			UINT32 filter_frag_en:1;                    //bit 3
			UINT32 unused1:12;                          //bits 4-15
			UINT32 min_pkt_size:7;                      //bits 16-22
			UINT32 unused2:9;                           //bits 23-31
#endif
		} bits;
	} 
	RXMAC_PF_CTRL_t, *PRXMAC_PF_CTRL_t;


/******************************************************************************
 structure for Memory Controller Interface Control Max Segment reg in rxmac 
 address map.  Located at address 0x4088
 *****************************************************************************/
typedef union _RXMAC_MCIF_CTRL_MAX_SEG_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:22;                         //bits 10-31
			UINT32 max_size:8;                          //bits 2-9
			UINT32 fc_en:1;                             //bit 1
			UINT32 seg_en:1;                            //bit 0
#else
			UINT32 seg_en:1;                            //bit 0
			UINT32 fc_en:1;                             //bit 1
			UINT32 max_size:8;                          //bits 2-9
			UINT32 reserved:22;                         //bits 10-31
#endif
		} bits;
	} 
	RXMAC_MCIF_CTRL_MAX_SEG_t, *PRXMAC_MCIF_CTRL_MAX_SEG_t;


/******************************************************************************
 structure for Memory Controller Interface Water Mark reg in rxmac address 
 map.  Located at address 0x408C
 *****************************************************************************/
typedef union _RXMAC_MCIF_WATER_MARK_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved2:6;                         //bits 26-31
			UINT32 mark_hi:10;                          //bits 16-25
			UINT32 reserved1:6;                         //bits 10-15
			UINT32 mark_lo:10;                          //bits 0-9
#else
			UINT32 mark_lo:10;                          //bits 0-9
			UINT32 reserved1:6;                         //bits 10-15
			UINT32 mark_hi:10;                          //bits 16-25
			UINT32 reserved2:6;                         //bits 26-31
#endif
		} bits;
	} 
	RXMAC_MCIF_WATER_MARK_t, *PRXMAC_MCIF_WATER_MARK_t;


/******************************************************************************
 structure for Rx Queue Dialog reg in rxmac address map.  
 located at address 0x4090
 *****************************************************************************/
typedef union _RXMAC_RXQ_DIAG_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved2:6;                         //bits 26-31
			UINT32 rd_ptr:10;                           //bits 16-25
			UINT32 reserved1:6;                         //bits 10-15
			UINT32 wr_ptr:10;                           //bits 0-9
#else
			UINT32 wr_ptr:10;                           //bits 0-9
			UINT32 reserved1:6;                         //bits 10-15
			UINT32 rd_ptr:10;                           //bits 16-25
			UINT32 reserved2:6;                         //bits 26-31
#endif
		} bits;
	} 
	RXMAC_RXQ_DIAG_t, *PRXMAC_RXQ_DIAG_t;


/******************************************************************************
 structure for space availiable reg in rxmac address map.  
 located at address 0x4094
 *****************************************************************************/
typedef union _RXMAC_SPACE_AVAIL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved2:15;                        //bits 17-31
			UINT32 space_avail_en:1;                    //bit 16
			UINT32 reserved1:6;                         //bits 10-15
			UINT32 space_avail:10;                      //bits 0-9
#else
			UINT32 space_avail:10;                      //bits 0-9
			UINT32 reserved1:6;                         //bits 10-15
			UINT32 space_avail_en:1;                    //bit 16
			UINT32 reserved2:15;                        //bits 17-31
#endif
		} bits;
	} 
	RXMAC_SPACE_AVAIL_t, *PRXMAC_SPACE_AVAIL_t;


/******************************************************************************
 structure for management interface reg in rxmac address map.  
 located at address 0x4098
 *****************************************************************************/
typedef union _RXMAC_MIF_CTL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserve:14;                          //bits 18-31
			UINT32 drop_pkt_en:1;                       //bit 17
			UINT32 drop_pkt_mask:17;                    //bits 0-16
#else
			UINT32 drop_pkt_mask:17;                    //bits 0-16
			UINT32 drop_pkt_en:1;                       //bit 17
			UINT32 reserve:14;                          //bits 18-31
#endif
		} bits;
	} 
	RXMAC_MIF_CTL_t, *PRXMAC_MIF_CTL_t;


/******************************************************************************
 structure for Error reg in rxmac address map.  
 located at address 0x409C
 *****************************************************************************/
typedef union _RXMAC_ERROR_REG_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserve:28;                          //bits 4-31
			UINT32 mif:1;                               //bit 3
			UINT32 async:1;                             //bit 2
			UINT32 pkt_filter:1;                        //bit 1
			UINT32 mcif:1;                              //bit 0
#else
			UINT32 mcif:1;                              //bit 0
			UINT32 pkt_filter:1;                        //bit 1
			UINT32 async:1;                             //bit 2
			UINT32 mif:1;                               //bit 3
			UINT32 reserve:28;                          //bits 4-31 
#endif
		} bits;
	} 
	RXMAC_ERROR_REG_t, *PRXMAC_ERROR_REG_t;


/******************************************************************************
 Rx MAC Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _RXMAC_t
	{                                                   //Location:
		RXMAC_CTRL_t                ctrl;               //  0x4000
		RXMAC_WOL_CTL_CRC0_t        crc0;               //  0x4004
		RXMAC_WOL_CRC12_t           crc12;              //  0x4008
		RXMAC_WOL_CRC34_t           crc34;              //  0x400C
		RXMAC_WOL_SA_LO_t           sa_lo;              //  0x4010
		RXMAC_WOL_SA_HI_t           sa_hi;              //  0x4014
		RXMAC_WOL_MASK_t            mask0_word0;        //  0x4018
		RXMAC_WOL_MASK_t            mask0_word1;        //  0x401C
		RXMAC_WOL_MASK_t            mask0_word2;        //  0x4020
		RXMAC_WOL_MASK_t            mask0_word3;        //  0x4024
		RXMAC_WOL_MASK_t            mask1_word0;        //  0x4028
		RXMAC_WOL_MASK_t            mask1_word1;        //  0x402C
		RXMAC_WOL_MASK_t            mask1_word2;        //  0x4030
		RXMAC_WOL_MASK_t            mask1_word3;        //  0x4034
		RXMAC_WOL_MASK_t            mask2_word0;        //  0x4038
		RXMAC_WOL_MASK_t            mask2_word1;        //  0x403C
		RXMAC_WOL_MASK_t            mask2_word2;        //  0x4040
		RXMAC_WOL_MASK_t            mask2_word3;        //  0x4044
		RXMAC_WOL_MASK_t            mask3_word0;        //  0x4048
		RXMAC_WOL_MASK_t            mask3_word1;        //  0x404C
		RXMAC_WOL_MASK_t            mask3_word2;        //  0x4050
		RXMAC_WOL_MASK_t            mask3_word3;        //  0x4054
		RXMAC_WOL_MASK_t            mask4_word0;        //  0x4058
		RXMAC_WOL_MASK_t            mask4_word1;        //  0x405C
		RXMAC_WOL_MASK_t            mask4_word2;        //  0x4060
		RXMAC_WOL_MASK_t            mask4_word3;        //  0x4064
		RXMAC_UNI_PF_ADDR1_t        uni_pf_addr1;       //  0x4068
		RXMAC_UNI_PF_ADDR2_t        uni_pf_addr2;       //  0x406C
		RXMAC_UNI_PF_ADDR3_t        uni_pf_addr3;       //  0x4070
		RXMAC_MULTI_HASH_t          multi_hash1;        //  0x4074
		RXMAC_MULTI_HASH_t          multi_hash2;        //  0x4078
		RXMAC_MULTI_HASH_t          multi_hash3;        //  0x407C
		RXMAC_MULTI_HASH_t          multi_hash4;        //  0x4080
		RXMAC_PF_CTRL_t             pf_ctrl;            //  0x4084
		RXMAC_MCIF_CTRL_MAX_SEG_t   mcif_ctrl_max_seg;  //  0x4088
		RXMAC_MCIF_WATER_MARK_t     mcif_water_mark;    //  0x408C
		RXMAC_RXQ_DIAG_t            rxq_diag;           //  0x4090
		RXMAC_SPACE_AVAIL_t         space_avail;        //  0x4094
		
		RXMAC_MIF_CTL_t             mif_ctrl;           //  0x4098
		RXMAC_ERROR_REG_t           err_reg;            //  0x409C
	}
	RXMAC_t, *PRXMAC_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF TXMAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                  START OF MAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 structure for configuration #1 reg in mac address map.  
 located at address 0x5000
 *****************************************************************************/
typedef union _MAC_CFG1_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 soft_reset:1;                        //bit 31
			UINT32 sim_reset:1;                         //bit 30
			UINT32 reserved3:10;                        //bits 20-29
			UINT32 reset_rx_mc:1;                       //bit 19
			UINT32 reset_tx_mc:1;                       //bit 18
			UINT32 reset_rx_fun:1;                      //bit 17
			UINT32 reset_tx_fun:1;                      //bit 16
			UINT32 reserved2:7;                         //bits 9-15
			UINT32 loop_back:1;                         //bit 8
			UINT32 reserved1:2;                         //bits 6-7
			UINT32 rx_flow:1;                           //bit 5
			UINT32 tx_flow:1;                           //bit 4
			UINT32 syncd_rx_en:1;                       //bit 3
			UINT32 rx_enable:1;                         //bit 2
			UINT32 syncd_tx_en:1;                       //bit 1
			UINT32 tx_enable:1;                         //bit 0
#else
			UINT32 tx_enable:1;                         //bit 0
			UINT32 syncd_tx_en:1;                       //bit 1
			UINT32 rx_enable:1;                         //bit 2
			UINT32 syncd_rx_en:1;                       //bit 3
			UINT32 tx_flow:1;                           //bit 4
			UINT32 rx_flow:1;                           //bit 5
			UINT32 reserved1:2;                         //bits 6-7
			UINT32 loop_back:1;                         //bit 8
			UINT32 reserved2:7;                         //bits 9-15
			UINT32 reset_tx_fun:1;                      //bit 16
			UINT32 reset_rx_fun:1;                      //bit 17
			UINT32 reset_tx_mc:1;                       //bit 18
			UINT32 reset_rx_mc:1;                       //bit 19
			UINT32 reserved3:10;                        //bits 20-29
			UINT32 sim_reset:1;                         //bit 30
			UINT32 soft_reset:1;                        //bit 31
#endif
		} bits;
	} 
	MAC_CFG1_t, *PMAC_CFG1_t;


/******************************************************************************
 structure for configuration #2 reg in mac address map.  
 located at address 0x5004
 *****************************************************************************/
typedef union _MAC_CFG2_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved3:16;                        //bits 16-31
			UINT32 preamble_len:4;                      //bits 12-15
			UINT32 reserved2:2;                         //bits 10-11
			UINT32 if_mode:2;                           //bits 8-9
			UINT32 reserved1:2;                         //bits 6-7
			UINT32 huge_frame:1;                        //bit 5
			UINT32 len_check:1;                         //bit 4
			UINT32 undefined:1;                         //bit 3
			UINT32 pad_crc:1;                           //bit 2
			UINT32 crc_enable:1;                        //bit 1
			UINT32 full_duplex:1;                       //bit 0
#else
			UINT32 full_duplex:1;                       //bit 0
			UINT32 crc_enable:1;                        //bit 1
			UINT32 pad_crc:1;                           //bit 2
			UINT32 undefined:1;                         //bit 3
			UINT32 len_check:1;                         //bit 4
			UINT32 huge_frame:1;                        //bit 5
			UINT32 reserved1:2;                         //bits 6-7
			UINT32 if_mode:2;                           //bits 8-9
			UINT32 reserved2:2;                         //bits 10-11
			UINT32 preamble_len:4;                      //bits 12-15
			UINT32 reserved3:16;                        //bits 16-31
#endif
		} bits;
	} 
	MAC_CFG2_t, *PMAC_CFG2_t;


/******************************************************************************
 structure for Interpacket gap reg in mac address map.  
 located at address 0x5008
 *****************************************************************************/
typedef union _MAC_IPG_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:1;                          //bit 31
			UINT32 non_B2B_ipg_1:7;                     //bits 24-30
			UINT32 undefined2:1;                        //bit 23
			UINT32 non_B2B_ipg_2:7;                     //bits 16-22
			UINT32 min_ifg_enforce:8;                   //bits 8-15
			UINT32 undefined1:1;                        //bit 7
			UINT32 B2B_ipg:7;                           //bits 0-6
#else
			UINT32 B2B_ipg:7;                           //bits 0-6
			UINT32 undefined1:1;                        //bit 7
			UINT32 min_ifg_enforce:8;                   //bits 8-15
			UINT32 non_B2B_ipg_2:7;                     //bits 16-22
			UINT32 undefined2:1;                        //bit 23
			UINT32 non_B2B_ipg_1:7;                     //bits 24-30
			UINT32 reserved:1;                          //bit 31
#endif
		} bits;
	} 
	MAC_IPG_t, *PMAC_IPG_t;


/******************************************************************************
 structure for half duplex reg in mac address map.  
 located at address 0x500C
 *****************************************************************************/
typedef union _MAC_HFDP_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved2:8;                         //bits 24-31
			UINT32 alt_beb_trunc:4;                     //bits 23-20
			UINT32 alt_beb_enable:1;                    //bit 19
			UINT32 bp_no_backoff:1;                     //bit 18
			UINT32 no_backoff:1;                        //bit 17
			UINT32 excess_defer:1;                      //bit 16
			UINT32 rexmit_max:4;                        //bits 12-15
			UINT32 reserved1:2;                         //bits 10-11
			UINT32 coll_window:10;                      //bits 0-9
#else
			UINT32 coll_window:10;                      //bits 0-9
			UINT32 reserved1:2;                         //bits 10-11
			UINT32 rexmit_max:4;                        //bits 12-15
			UINT32 excess_defer:1;                      //bit 16
			UINT32 no_backoff:1;                        //bit 17
			UINT32 bp_no_backoff:1;                     //bit 18
			UINT32 alt_beb_enable:1;                    //bit 19
			UINT32 alt_beb_trunc:4;                     //bits 23-20
			UINT32 reserved2:8;                         //bits 24-31
#endif
		} bits;
	} 
	MAC_HFDP_t, *PMAC_HFDP_t;


/******************************************************************************
 structure for Maximum Frame Length reg in mac address map.  
 located at address 0x5010
 *****************************************************************************/
typedef union _MAC_MAX_FM_LEN_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:16;                         //bits 16-31
			UINT32 max_len:16;                          //bits 0-15
#else
			UINT32 max_len:16;                          //bits 0-15
			UINT32 reserved:16;                         //bits 16-31
#endif
		} bits;
	} 
	MAC_MAX_FM_LEN_t, *PMAC_MAX_FM_LEN_t;


/******************************************************************************
 structure for Reserve 1 reg in mac address map.  
 located at address 0x5014 - 0x5018
 *****************************************************************************/
typedef struct _MAC_RSV_t
	{
		UINT32 value;                                   //bits 0-31
	} 
	MAC_RSV_t, *PMAC_RSV_t;


/******************************************************************************
 structure for Test reg in mac address map.  
 located at address 0x501C
 *****************************************************************************/
typedef union _MAC_TEST_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:29;                           //bits 3-31
			UINT32 mac_test:3;                          //bits 0-2
#else
			UINT32 mac_test:3;                          //bits 0-2
			UINT32 unused:29;                           //bits 3-31
#endif
		} bits;
	} 
	MAC_TEST_t, *PMAC_TEST_t;


/******************************************************************************
 structure for MII Management Configuration reg in mac address map.  
 located at address 0x5020
 *****************************************************************************/
typedef union _MII_MGMT_CFG_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reset_mii_mgmt:1;                    //bit 31
			UINT32 reserved:25;                         //bits 6-30
			UINT32 scan_auto_incremt:1;                 //bit 5
			UINT32 preamble_suppress:1;                 //bit 4
			UINT32 undefined:1;                         //bit 3
			UINT32 mgmt_clk_reset:3;                    //bits 0-2
#else
			UINT32 mgmt_clk_reset:3;                    //bits 0-2
			UINT32 undefined:1;                         //bit 3
			UINT32 preamble_suppress:1;                 //bit 4
			UINT32 scan_auto_incremt:1;                 //bit 5
			UINT32 reserved:25;                         //bits 6-30
			UINT32 reset_mii_mgmt:1;                    //bit 31
#endif
		} bits;
	} 
	MII_MGMT_CFG_t, *PMII_MGMT_CFG_t;


/******************************************************************************
 structure for MII Management Command reg in mac address map.  
 located at address 0x5024
 *****************************************************************************/
typedef union _MII_MGMT_CMD_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:30;                         //bits 2-31
			UINT32 scan_cycle:1;                        //bit 1
			UINT32 read_cycle:1;                        //bit 0
#else
			UINT32 read_cycle:1;                        //bit 0
			UINT32 scan_cycle:1;                        //bit 1
			UINT32 reserved:30;                         //bits 2-31    
#endif
		} bits;
	} 
	MII_MGMT_CMD_t, *PMII_MGMT_CMD_t;


/******************************************************************************
 structure for MII Management Address reg in mac address map.  
 located at address 0x5028
 *****************************************************************************/
typedef union _MII_MGMT_ADDR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved2:19;                        //bit 13-31
			UINT32 phy_addr:5;                          //bits 8-12
			UINT32 reserved1:3;                         //bits 5-7
			UINT32 reg_addr:5;                          //bits 0-4
#else
			UINT32 reg_addr:5;                          //bits 0-4
			UINT32 reserved1:3;                         //bits 5-7
			UINT32 phy_addr:5;                          //bits 8-12
			UINT32 reserved2:19;                        //bit 13-31
#endif
		} bits;
	} 
	MII_MGMT_ADDR_t, *PMII_MGMT_ADDR_t;


/******************************************************************************
 structure for MII Management Control reg in mac address map.  
 located at address 0x502C
 *****************************************************************************/
typedef union _MII_MGMT_CTRL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:16;                         //bits 16-31
			UINT32 phy_ctrl:16;                         //bits 0-15
#else
			UINT32 phy_ctrl:16;                         //bits 0-15
			UINT32 reserved:16;                         //bits 16-31
#endif
		} bits;
	} 
	MII_MGMT_CTRL_t, *PMII_MGMT_CTRL_t;


/******************************************************************************
 structure for MII Management Status reg in mac address map.  
 located at address 0x5030
 *****************************************************************************/
typedef union _MII_MGMT_STAT_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:16;                         //bits 16-31
			UINT32 phy_stat:16;                         //bits 0-15
#else
			UINT32 phy_stat:16;                         //bits 0-15
			UINT32 reserved:16;                         //bits 16-31
#endif
		} bits;
	} 
	MII_MGMT_STAT_t, *PMII_MGMT_STAT_t;


/******************************************************************************
 structure for MII Management Indicators reg in mac address map.  
 located at address 0x5034
 *****************************************************************************/
typedef union _MII_MGMT_INDICATOR_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:29;                         //bits 3-31
			UINT32 not_valid:1;                         //bit 2
			UINT32 scanning:1;                          //bit 1
			UINT32 busy:1;                              //bit 0
#else
			UINT32 busy:1;                              //bit 0
			UINT32 scanning:1;                          //bit 1
			UINT32 not_valid:1;                         //bit 2
			UINT32 reserved:29;                         //bits 3-31
#endif
		} bits;
	} 
	MII_MGMT_INDICATOR_t, *PMII_MGMT_INDICATOR_t;


/******************************************************************************
 structure for Interface Control reg in mac address map.  
 located at address 0x5038
 *****************************************************************************/
typedef union _MAC_IF_CTRL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reset_if_module:1;                   //bit 31
			UINT32 reserved4:3;                         //bit 28-30
			UINT32 tbi_mode:1;                          //bit 27
			UINT32 ghd_mode:1;                          //bit 26
			UINT32 lhd_mode:1;                          //bit 25
			UINT32 phy_mode:1;                          //bit 24
			UINT32 reset_per_mii:1;                     //bit 23
			UINT32 reserved3:6;                         //bits 17-22
			UINT32 speed:1;                             //bit 16
			UINT32 reset_pe100x:1;                      //bit 15
			UINT32 reserved2:4;                         //bits 11-14
			UINT32 force_quiet:1;                       //bit 10
			UINT32 no_cipher:1;                         //bit 9
			UINT32 disable_link_fail:1;                 //bit 8
			UINT32 reset_gpsi:1;                        //bit 7
			UINT32 reserved1:6;                         //bits 1-6
			UINT32 enab_jab_protect:1;                  //bit 0
#else
			UINT32 enab_jab_protect:1;                  //bit 0
			UINT32 reserved1:6;                         //bits 1-6
			UINT32 reset_gpsi:1;                        //bit 7
			UINT32 disable_link_fail:1;                 //bit 8
			UINT32 no_cipher:1;                         //bit 9
			UINT32 force_quiet:1;                       //bit 10
			UINT32 reserved2:4;                         //bits 11-14
			UINT32 reset_pe100x:1;                      //bit 15
			UINT32 speed:1;                             //bit 16
			UINT32 reserved3:6;                         //bits 17-22
			UINT32 reset_per_mii:1;                     //bit 23
			UINT32 phy_mode:1;                          //bit 24
			UINT32 lhd_mode:1;                          //bit 25
			UINT32 ghd_mode:1;                          //bit 26
			UINT32 tbi_mode:1;                          //bit 27
			UINT32 reserved4:3;                         //bit 28-30
			UINT32 reset_if_module:1;                   //bit 31
#endif
		} bits;
	} 
	MAC_IF_CTRL_t, *PMAC_IF_CTRL_t;


/******************************************************************************
 structure for Interface Status reg in mac address map.  
 located at address 0x503C
 *****************************************************************************/
typedef union _MAC_IF_STAT_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:22;                         //bits 10-31
			UINT32 excess_defer:1;                      //bit 9
			UINT32 clash:1;                             //bit 8
			UINT32 phy_jabber:1;                        //bit 7
			UINT32 phy_link_ok:1;                       //bit 6
			UINT32 phy_full_duplex:1;                   //bit 5
			UINT32 phy_speed:1;                         //bit 4
			UINT32 pe100x_link_fail:1;                  //bit 3
			UINT32 pe10t_loss_carrie:1;                 //bit 2
			UINT32 pe10t_sqe_error:1;                   //bit 1
			UINT32 pe10t_jabber:1;                      //bit 0
#else
			UINT32 pe10t_jabber:1;                      //bit 0
			UINT32 pe10t_sqe_error:1;                   //bit 1
			UINT32 pe10t_loss_carrie:1;                 //bit 2
			UINT32 pe100x_link_fail:1;                  //bit 3
			UINT32 phy_speed:1;                         //bit 4
			UINT32 phy_full_duplex:1;                   //bit 5
			UINT32 phy_link_ok:1;                       //bit 6
			UINT32 phy_jabber:1;                        //bit 7
			UINT32 clash:1;                             //bit 8
			UINT32 excess_defer:1;                      //bit 9
			UINT32 reserved:22;                         //bits 10-31
#endif
		} bits;
	} 
	MAC_IF_STAT_t, *PMAC_IF_STAT_t;


/******************************************************************************
 structure for Mac Station Address, Part 1 reg in mac address map.  
 located at address 0x5040
 *****************************************************************************/
typedef union _MAC_STATION_ADDR1_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 Octet6:8;                            //bits 24-31
			UINT32 Octet5:8;                            //bits 16-23
			UINT32 Octet4:8;                            //bits 8-15
			UINT32 Octet3:8;                            //bits 0-7
#else
			UINT32 Octet3:8;                            //bits 0-7
			UINT32 Octet4:8;                            //bits 8-15
			UINT32 Octet5:8;                            //bits 16-23
			UINT32 Octet6:8;                            //bits 24-31
#endif
		} bits;
	} 
	MAC_STATION_ADDR1_t, *PMAC_STATION_ADDR1_t;


/******************************************************************************
 structure for Mac Station Address, Part 2 reg in mac address map.  
 located at address 0x5044
 *****************************************************************************/
typedef union _MAC_STATION_ADDR2_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 Octet2:8;                            //bits 24-31
			UINT32 Octet1:8;                            //bits 16-23
			UINT32 reserved:16;                         //bits 0-15
#else
			UINT32 reserved:16;                         //bit 0-15
			UINT32 Octet1:8;                            //bits 16-23
			UINT32 Octet2:8;                            //bits 24-31
#endif
		} bits;
	} 
	MAC_STATION_ADDR2_t, *PMAC_STATION_ADDR2_t;


/******************************************************************************
 MAC Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _MAC_t {                                                   //Location:
		MAC_CFG1_t                  cfg1;               //  0x5000
		MAC_CFG2_t                  cfg2;               //  0x5004
		MAC_IPG_t                   ipg;                //  0x5008
		MAC_HFDP_t                  hfdp;               //  0x500C
		MAC_MAX_FM_LEN_t            max_fm_len;         //  0x5010
		MAC_RSV_t                   rsv1;               //  0x5014
		MAC_RSV_t                   rsv2;               //  0x5018
		MAC_TEST_t                  mac_test;           //  0x501C
		MII_MGMT_CFG_t              mii_mgmt_cfg;       //  0x5020
		MII_MGMT_CMD_t              mii_mgmt_cmd;       //  0x5024
		MII_MGMT_ADDR_t             mii_mgmt_addr;      //  0x5028
		MII_MGMT_CTRL_t             mii_mgmt_ctrl;      //  0x502C
		MII_MGMT_STAT_t             mii_mgmt_stat;      //  0x5030
		MII_MGMT_INDICATOR_t        mii_mgmt_indicator; //  0x5034
		MAC_IF_CTRL_t               if_ctrl;            //  0x5038
		MAC_IF_STAT_t               if_stat;            //  0x503C
		MAC_STATION_ADDR1_t         station_addr_1;     //  0x5040
		MAC_STATION_ADDR2_t         station_addr_2;     //  0x5044
} MAC_t, *PMAC_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                   END OF MAC REGISTER ADDRESS MAP                   ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                START OF MAC STAT REGISTER ADDRESS MAP               ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 structure for Carry Register One and it's Mask Register reg located in mac
 stat address map address 0x6130 and 0x6138.
 *****************************************************************************/
typedef union _MAC_STAT_REG_1_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 tr64:1;                              //bit 31
			UINT32 tr127:1;                             //bit 30
			UINT32 tr255:1;                             //bit 29
			UINT32 tr511:1;                             //bit 28
			UINT32 tr1k:1;                              //bit 27
			UINT32 trmax:1;                             //bit 26
			UINT32 trmgv:1;                             //bit 25
			UINT32 unused:8;                            //bits 17-24
			UINT32 rbyt:1;                             //bit 16
			UINT32 rpkt:1;                             //bit 15
			UINT32 rfcs:1;                             //bit 14
			UINT32 rmca:1;                             //bit 13
			UINT32 rbca:1;                             //bit 12
			UINT32 rxcf:1;                             //bit 11
			UINT32 rxpf:1;                             //bit 10
			UINT32 rxuo:1;                             //bit 9
			UINT32 raln:1;                             //bit 8
			UINT32 rflr:1;                             //bit 7
			UINT32 rcde:1;                             //bit 6
			UINT32 rcse:1;                             //bit 5
			UINT32 rund:1;                             //bit 4
			UINT32 rovr:1;                             //bit 3
			UINT32 rfrg:1;                             //bit 2
			UINT32 rjbr:1;                             //bit 1
			UINT32 rdrp:1;                             //bit 0
#else
			UINT32 rdrp:1;                             //bit 0
			UINT32 rjbr:1;                             //bit 1
			UINT32 rfrg:1;                             //bit 2
			UINT32 rovr:1;                             //bit 3
			UINT32 rund:1;                             //bit 4
			UINT32 rcse:1;                             //bit 5
			UINT32 rcde:1;                             //bit 6 
			UINT32 rflr:1;                             //bit 7
			UINT32 raln:1;                             //bit 8
			UINT32 rxuo:1;                             //bit 9
			UINT32 rxpf:1;                             //bit 10
			UINT32 rxcf:1;                             //bit 11
			UINT32 rbca:1;                             //bit 12
			UINT32 rmca:1;                             //bit 13
			UINT32 rfcs:1;                             //bit 14
			UINT32 rpkt:1;                             //bit 15
			UINT32 rbyt:1;                             //bit 16
			UINT32 unused:8;                            //bits 17-24
			UINT32 trmgv:1;                             //bit 25
			UINT32 trmax:1;                             //bit 26
			UINT32 tr1k:1;                              //bit 27
			UINT32 tr511:1;                             //bit 28
			UINT32 tr255:1;                             //bit 29
			UINT32 tr127:1;                             //bit 30
			UINT32 tr64:1;                              //bit 31
#endif
		} bits;
	} 
	MAC_STAT_REG_1_t, *PMAC_STAT_REG_1_t;


/******************************************************************************
 structure for Carry Register Two Mask Register reg in mac stat address map.  
 located at address 0x613C
 *****************************************************************************/
typedef union _MAC_STAT_REG_2_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:12;                           //bit 20-31
			UINT32 tjbr:1;                             //bit 19
			UINT32 tfcs:1;                             //bit 18
			UINT32 txcf:1;                             //bit 17        
			UINT32 tovr:1;                             //bit 16
			UINT32 tund:1;                             //bit 15
			UINT32 tfrg:1;                             //bit 14
			UINT32 tbyt:1;                             //bit 13
			UINT32 tpkt:1;                             //bit 12
			UINT32 tmca:1;                             //bit 11
			UINT32 tbca:1;                             //bit 10
			UINT32 txpf:1;                             //bit 9
			UINT32 tdfr:1;                             //bit 8
			UINT32 tedf:1;                             //bit 7
			UINT32 tscl:1;                             //bit 6
			UINT32 tmcl:1;                             //bit 5
			UINT32 tlcl:1;                             //bit 4
			UINT32 txcl:1;                             //bit 3
			UINT32 tncl:1;                             //bit 2
			UINT32 tpfh:1;                             //bit 1
			UINT32 tdrp:1;                             //bit 0
#else
			UINT32 tdrp:1;                             //bit 0
			UINT32 tpfh:1;                             //bit 1
			UINT32 tncl:1;                             //bit 2
			UINT32 txcl:1;                             //bit 3
			UINT32 tlcl:1;                             //bit 4
			UINT32 tmcl:1;                             //bit 5
			UINT32 tscl:1;                             //bit 6
			UINT32 tedf:1;                             //bit 7
			UINT32 tdfr:1;                             //bit 8
			UINT32 txpf:1;                             //bit 9
			UINT32 tbca:1;                             //bit 10
			UINT32 tmca:1;                             //bit 11
			UINT32 tpkt:1;                             //bit 12
			UINT32 tbyt:1;                             //bit 13
			UINT32 tfrg:1;                             //bit 14
			UINT32 tund:1;                             //bit 15
			UINT32 tovr:1;                             //bit 16
			UINT32 txcf:1;                             //bit 17    
			UINT32 tfcs:1;                             //bit 18
			UINT32 tjbr:1;                             //bit 19
			UINT32 unused:12;                         //bit 20-31
#endif
		} bits;
	} 
	MAC_STAT_REG_2_t, *PMAC_STAT_REG_2_t;




/******************************************************************************
 MAC STATS Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _MAC_STAT_t {                                                   //Location:
		UINT32 pad[32];                                 //  0x6000 - 607C
		
		//Tx/Rx 0-64 Byte Frame Counter
		UINT32 TR64;                                    //  0x6080
		
		//Tx/Rx 65-127 Byte Frame Counter
		UINT32 TR127;                                   //  0x6084
		
		//Tx/Rx 128-255 Byte Frame Counter
		UINT32 TR255;                                   //  0x6088
		
		//Tx/Rx 256-511 Byte Frame Counter
		UINT32 TR511;                                   //  0x608C
		
		//Tx/Rx 512-1023 Byte Frame Counter
		UINT32 TR1K;                                    //  0x6090
		
		//Tx/Rx 1024-1518 Byte Frame Counter
		UINT32 TRMax;                                   //  0x6094
		
		//Tx/Rx 1519-1522 Byte Good VLAN Frame Count
		UINT32 TRMgv;                                   //  0x6098
		
		//Rx Byte Counter
		UINT32 RByt;                                    //  0x609C
		
		//Rx Packet Counter
		UINT32 RPkt;                                    //  0x60A0
		
		//Rx FCS Error Counter
		UINT32 RFcs;                                    //  0x60A4
		
		//Rx Multicast Packet Counter
		UINT32 RMca;                                    //  0x60A8
		
		//Rx Broadcast Packet Counter
		UINT32 RBca;                                    //  0x60AC
		
		//Rx Control Frame Packet Counter
		UINT32 RxCf;                                    //  0x60B0
		
		//Rx Pause Frame Packet Counter
		UINT32 RxPf;                                    //  0x60B4
		
		//Rx Unknown OP Code Counter
		UINT32 RxUo;                                    //  0x60B8
		
		//Rx Alignment Error Counter
		UINT32 RAln;                                    //  0x60BC
		
		//Rx Frame Length Error Counter
		UINT32 RFlr;                                    //  0x60C0
		
		//Rx Code Error Counter
		UINT32 RCde;                                    //  0x60C4
		
		//Rx Carrier Sense Error Counter
		UINT32 RCse;                                    //  0x60C8
		
		//Rx Undersize Packet Counter
		UINT32 RUnd;                                    //  0x60CC
		
		//Rx Oversize Packet Counter
		UINT32 ROvr;                                    //  0x60D0
		
		//Rx Fragment Counter
		UINT32 RFrg;                                    //  0x60D4
		
		//Rx Jabber Counter
		UINT32 RJbr;                                    //  0x60D8
		
		//Rx Drop
		UINT32 RDrp;                                    //  0x60DC
		
		//Tx Byte Counter
		UINT32 TByt;                                    //  0x60E0
		
		//Tx Packet Counter
		UINT32 TPkt;                                    //  0x60E4
		
		//Tx Multicast Packet Counter
		UINT32 TMca;                                    //  0x60E8
		
		//Tx Broadcast Packet Counter
		UINT32 TBca;                                    //  0x60EC
		
		//Tx Pause Control Frame Counter
		UINT32 TxPf;                                    //  0x60F0
		
		//Tx Deferral Packet Counter
		UINT32 TDfr;                                    //  0x60F4
		
		//Tx Excessive Deferral Packet Counter
		UINT32 TEdf;                                    //  0x60F8
		
		//Tx Single Collision Packet Counter
		UINT32 TScl;                                    //  0x60FC
		
		//Tx Multiple Collision Packet Counter
		UINT32 TMcl;                                    //  0x6100
		
		//Tx Late Collision Packet Counter
		UINT32 TLcl;                                    //  0x6104
		
		//Tx Excessive Collision Packet Counter
		UINT32 TXcl;                                    //  0x6108
		
		//Tx Total Collision Packet Counter
		UINT32 TNcl;                                    //  0x610C
		
		//Tx Pause Frame Honored Counter
		UINT32 TPfh;                                    //  0x6110
		
		//Tx Drop Frame Counter
		UINT32 TDrp;                                    //  0x6114
		
		//Tx Jabber Frame Counter
		UINT32 TJbr;                                    //  0x6118
		
		//Tx FCS Error Counter
		UINT32 TFcs;                                    //  0x611C
		
		//Tx Control Frame Counter
		UINT32 TxCf;                                    //  0x6120
		
		//Tx Oversize Frame Counter
		UINT32 TOvr;                                    //  0x6124
		
		//Tx Undersize Frame Counter
		UINT32 TUnd;                                    //  0x6128
		
		//Tx Fragments Frame Counter
		UINT32 TFrg;                                    //  0x612C
		
		//Carry Register One Register
		MAC_STAT_REG_1_t Carry1;                        //  0x6130
		
		//Carry Register Two Register
		MAC_STAT_REG_2_t Carry2;                           //  0x6134
		
		//Carry Register One Mask Register
		MAC_STAT_REG_1_t Carry1M;                       //  0x6138
		
		//Carry Register Two Mask Register
		MAC_STAT_REG_2_t Carry2M;                       //  0x613C
} MAC_STAT_t, *PMAC_STAT_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                END OF MAC STAT REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                  START OF MMC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 structure for Main Memory Controller Control reg in mmc address map.  
 located at address 0x7000
 *****************************************************************************/
typedef union _MMC_CTRL_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:25;                         //bits 7-31
			UINT32 force_ce:1;                          //bit 6
			UINT32 rxdma_disable:1;                     //bit 5
			UINT32 txdma_disable:1;                     //bit 4
			UINT32 txmac_disable:1;                     //bit 3
			UINT32 rxmac_disable:1;                     //bit 2
			UINT32 arb_disable:1;                       //bit 1
			UINT32 mmc_enable:1;                        //bit 0
#else
			UINT32 mmc_enable:1;                        //bit 0
			UINT32 arb_disable:1;                       //bit 1
			UINT32 rxmac_disable:1;                     //bit 2
			UINT32 txmac_disable:1;                     //bit 3
			UINT32 txdma_disable:1;                     //bit 4
			UINT32 rxdma_disable:1;                     //bit 5
			UINT32 force_ce:1;                          //bit 6
			UINT32 reserved:25;                         //bits 7-31
#endif
		} bits;
	} 
	MMC_CTRL_t, *PMMC_CTRL_t;


/******************************************************************************
 structure for Main Memory Controller Host Memory Access Address reg in mmc 
 address map.  Located at address 0x7004
 *****************************************************************************/
typedef union _MMC_SRAM_ACCESS_t
	{
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 byte_enable:16;                      //bits 16-31
			UINT32 reserved2:2;                         //bits 14-15
			UINT32 req_addr:10;                         //bits 4-13
			UINT32 reserved1:1;                         //bit 3
			UINT32 is_ctrl_word:1;                      //bit 2
			UINT32 wr_access:1;                         //bit 1
			UINT32 req_access:1;                        //bit 0
#else
			UINT32 req_access:1;                        //bit 0
			UINT32 wr_access:1;                         //bit 1
			UINT32 is_ctrl_word:1;                      //bit 2
			UINT32 reserved1:1;                         //bit 3
			UINT32 req_addr:10;                         //bits 4-13
			UINT32 reserved2:2;                         //bits 14-15
			UINT32 byte_enable:16;                      //bits 16-31
#endif
		} bits;
	} 
	MMC_SRAM_ACCESS_t, *PMMC_SRAM_ACCESS_t;


/******************************************************************************
 structure for Main Memory Controller Host Memory Access Data reg in mmc 
 address map.  Located at address 0x7008 - 0x7014
 *****************************************************************************/
typedef struct _MMC_SRAM_WORD_t {
		UINT32 data;                                    //bits 0-31
} MMC_SRAM_WORD_t, *PMMC_SRAM_WORD_t;


/******************************************************************************
 Memory Control Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _MMC_t {	//Location:
		MMC_CTRL_t          mmc_ctrl;                   //  0x7000
		MMC_SRAM_ACCESS_t   sram_access;                //  0x7004
		MMC_SRAM_WORD_t     sram_word1;                 //  0x7008
		MMC_SRAM_WORD_t     sram_word2;                 //  0x700C
		MMC_SRAM_WORD_t     sram_word3;                 //  0x7010
		MMC_SRAM_WORD_t     sram_word4;                 //  0x7014
} MMC_t, *PMMC_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                   END OF MMC REGISTER ADDRESS MAP                   ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                START OF EXP ROM REGISTER ADDRESS MAP                ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
 Expansion ROM Module of JAGCore Address Mapping
 *****************************************************************************/


/******************************************************************************
 JAGCore Address Mapping
 *****************************************************************************/
typedef struct _ADDRESS_MAP_t {
		GLOBAL_t    global;
		UCHAR       unused_global[4096 - sizeof (GLOBAL_t)];    //unused section of global address map
		TXDMA_t     txdma;
		UCHAR       unused_txdma[4096 - sizeof (TXDMA_t)];     //unused section of txdma address map
		RXDMA_t     rxdma;
		UCHAR       unused_rxdma[4096 - sizeof (RXDMA_t)];     //unused section of rxdma address map
		TXMAC_t     txmac;
		UCHAR       unused_txmac[4096 - sizeof (TXMAC_t)];     //unused section of txmac address map
		RXMAC_t     rxmac;
		UCHAR       unused_rxmac[4096 - sizeof (RXMAC_t)];     //unused section of rxmac address map
		MAC_t       mac;
		UCHAR       unused_mac[4096 - sizeof (MAC_t)];       //unused section of mac address map
		MAC_STAT_t  macStat;
		UCHAR       unused_mac_stat[4096 - sizeof (MAC_STAT_t)];  //unused section of mac stat address map
		MMC_t       mmc;
		UCHAR       unused_mmc[4096 - sizeof (MMC_t)];       //unused section of mmc address map
		UCHAR       unused_[1015808];       //unused section of address map
		
		UCHAR       unused_exp_rom[4096];   //MGS-size TBD
		UCHAR       unused__[524288];       //unused section of address map
} ADDRESS_MAP_t, *PADDRESS_MAP_t;
/*===========================================================================*/


/******************************************************************************
 CONSTANTS FOR POWER MANAGEMENT
 *****************************************************************************/
#define MAX_WOL_PACKET_SIZE    0x80
#define MAX_WOL_MASK_SIZE      ( MAX_WOL_PACKET_SIZE / 8 )
#define NUM_WOL_PATTERNS       0x5
#define CRC16_POLY             0x1021




/******************************************************************************
 Definition of NDIS_DEVICE_POWER_STATE
 *****************************************************************************/
typedef enum
	{
		NdisDeviceStateUnspecified = 0,
		NdisDeviceStateD0,
		NdisDeviceStateD1,
		NdisDeviceStateD2,
		NdisDeviceStateD3
	} NDIS_DEVICE_POWER_STATE;




/******************************************************************************
 Structure for Power Management Info
 *****************************************************************************/
typedef struct _MP_PM_CONFIG_SPACE_INFO_t {
		UCHAR  capId;
		UCHAR  nextItemPtr;
		UINT16 pmcr;
		UINT16 pmcsr;
		UCHAR  pmscr_bse;
		UCHAR  pm_data_regs;
} MP_PM_CONFIG_SPACE_INFO_t, *pMP_PM_CONFIG_SPACE_INFO_t;




typedef struct _MP_POWER_MGMT {
		/**************************************************************************
		 variable putting the phy into coma mode when boot up with no cable 
		 plugged in after 5 seconds
		 *************************************************************************/
		UCHAR               TransPhyComaModeOnBoot;
		
		
		/**************************************************************************
		 Array holding the five CRC values that the device is currently using
		 for WOL.  This will be queried when a pattern is to be removed.
		 *************************************************************************/
		UINT32              localWolAndCrc0;
		UINT16              WOLPatternList[ NUM_WOL_PATTERNS ];
		UCHAR               WOLMaskList[ NUM_WOL_PATTERNS ][ MAX_WOL_MASK_SIZE ];
		UINT32              WOLMaskSize[ NUM_WOL_PATTERNS ];
		
		
		/**************************************************************************
		 IP address   
		 *************************************************************************/
		union
		{
			UINT32  u32;
			UCHAR   u8[4];
		} IPAddress;
		
		
		/**************************************************************************
		 Current Power state of the adapter.
		 *************************************************************************/
		NDIS_DEVICE_POWER_STATE PowerState;
		BOOL_t                  WOLState;
		BOOL_t                  WOLEnabled;
		BOOL_t                  Failed10Half;
		BOOL_t                  bFailedStateTransition;
		
		/**************************************************************************
		 Next two used to save power information at power down.  
		 This information will be used during power up to set up parts of Power 
		 Management in JAGCore
		 *************************************************************************/
		UINT32  tx_en;
		UINT32  rx_en;
		UINT16  PowerDownSpeed;
		UCHAR   PowerDownDuplex;
		
		MP_PM_CONFIG_SPACE_INFO_t   pmConfigRegs;
	} MP_POWER_MGMT, *PMP_POWER_MGMT;



/******************************************************************************
 Typedefs for Tx Descriptor Ring
 *****************************************************************************/
/******************************************************************************
 TXDESC_WORD2_t structure holds part of the control bits in the Tx Descriptor
 ring for the ET-1310 
 *****************************************************************************/
typedef union _txdesc_word2_t 
	{
		UINT32 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
            UINT32 vlan_prio:3;                     //bits 29-31(VLAN priority)
            UINT32 vlan_cfi:1;                      //bit 28(cfi)
            UINT32 vlan_tag:12;                     //bits 16-27(VLAN tag)
            UINT32 length_in_bytes:16;              //bits  0-15(packet length)
#else
            UINT32 length_in_bytes:16;              //bits  0-15(packet length)
            UINT32 vlan_tag:12;                     //bits 16-27(VLAN tag)
            UINT32 vlan_cfi:1;                      //bit 28(cfi)
            UINT32 vlan_prio:3;                     //bits 29-31(VLAN priority)
#endif /* __BIG_ENDIAN__ */
		} bits;
	}TXDESC_WORD2_t, *PTXDESC_WORD2_t;




/******************************************************************************
 TXDESC_WORD3_t structure holds part of the control bits in the Tx Descriptor
 ring for the ET-1310 
 *****************************************************************************/
typedef union  _txdesc_word3_t {
		UINT32 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
            UINT32 unused:17;                       //bits 15-31
            UINT32 udpa:1;                          //bit 14(UDP checksum assist)
            UINT32 tcpa:1;                          //bit 13(TCP checksum assist)
            UINT32 ipa:1;                           //bit 12(IP checksum assist)
            UINT32 vlan:1;                          //bit 11(append VLAN tag)
            UINT32 hp:1;                            //bit 10(Packet is a Huge packet)
            UINT32 pp:1;                            //bit  9(pad packet)
            UINT32 mac:1;                           //bit  8(MAC override)
            UINT32 crc:1;                           //bit  7(append CRC)
            UINT32 e:1;                             //bit  6(Tx frame has error)
            UINT32 pf:1;                            //bit  5(send pause frame)
            UINT32 bp:1;                            //bit  4(Issue half-duplex backpressure (XON/XOFF)
            UINT32 cw:1;                            //bit  3(Control word - no packet data)
            UINT32 ir:1;                            //bit  2(interrupt the processor when this pkt sent)
            UINT32 f:1;                             //bit  1(first packet in the sequence)
            UINT32 l:1;                             //bit  0(last packet in the sequence)
#else
            UINT32 l:1;                             //bit  0(last packet in the sequence)
            UINT32 f:1;                             //bit  1(first packet in the sequence)
            UINT32 ir:1;                            //bit  2(interrupt the processor when this pkt sent)
            UINT32 cw:1;                            //bit  3(Control word - no packet data)
            UINT32 bp:1;                            //bit  4(Issue half-duplex backpressure (XON/XOFF)
            UINT32 pf:1;                            //bit  5(send pause frame)
            UINT32 e:1;                             //bit  6(Tx frame has error)
            UINT32 crc:1;                           //bit  7(append CRC)
            UINT32 mac:1;                           //bit  8(MAC override)
            UINT32 pp:1;                            //bit  9(pad packet)
            UINT32 hp:1;                            //bit 10(Packet is a Huge packet)
            UINT32 vlan:1;                          //bit 11(append VLAN tag)
            UINT32 ipa:1;                           //bit 12(IP checksum assist)
            UINT32 tcpa:1;                          //bit 13(TCP checksum assist)
            UINT32 udpa:1;                          //bit 14(UDP checksum assist)
            UINT32 unused:17;                       //bits 15-31
#endif /* __BIG_ENDIAN__ */
		} bits;
} TXDESC_WORD3_t, *PTXDESC_WORD3_t;




/******************************************************************************
 TX_DESC_ENTRY_t is sructure representing each descriptor on the ring
 *****************************************************************************/
typedef struct _tx_desc_entry_t {
		UINT32         DataBufferPtrHigh;
		UINT32         DataBufferPtrLow;
		TXDESC_WORD2_t word2;                           // control words how to xmit the
		TXDESC_WORD3_t word3;                           // data (detailed above)
} TX_DESC_ENTRY_t, *PTX_DESC_ENTRY_t;
/*===========================================================================*/




/******************************************************************************
 Typedefs for Tx DMA engine status writeback
 *****************************************************************************/
/******************************************************************************
 TX_STATUS_BLOCK_t is sructure representing the status of the Tx DMA engine
 it sits in free memory, and is pointed to by 0x101c / 0x1020
 *****************************************************************************/

typedef union _tx_status_block_t {
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:21;                           //bits 11-31
			UINT32 serv_cpl_wrap:1;                     //bit 10
			UINT32 serv_cpl:10;                         //bits 0-9
#else
			UINT32 serv_cpl:10;                         //bits 0-9
			UINT32 serv_cpl_wrap:1;                     //bit 10
			UINT32 unused:21;                           //bits 11-31
#endif
		} bits;
} TX_STATUS_BLOCK_t, *PTX_STATUS_BLOCK_t;



/******************************************************************************
 TCB (Transmit Control Block)
 *****************************************************************************/
typedef struct _MP_TCB {
		struct _MP_TCB          *Next;
		UINT32                  PacketStaleCount;
		mbuf_t          Packet;
		TXDMA_SERVICE_REQUEST_t WrIndex;
		TXDMA_SERVICE_REQUEST_t WrIndexStart;
} MP_TCB, *PMP_TCB;



/******************************************************************************
 TX_RING_t is sructure representing our local reference(s) to the ring
 *****************************************************************************/
typedef struct _tx_ring_t {
		/**************************************************************************
		 TCB (Transmit Control Block) memory and lists
		 *************************************************************************/
		MP_TCB	MpTcbMem[NUM_TCB];
		
		/**************************************************************************
		 List of TCBs that are ready to be used
		 *************************************************************************/
		PMP_TCB                  TCBReadyQueueHead;
		PMP_TCB                  TCBReadyQueueTail;
		
		
		/**************************************************************************
		 list of TCBs that are currently being sent.  NOTE that access to all
		 three of these (including nBusySend) are controlled via the
		 TCBSendQLock.  This lock should be secured prior to incementing /
		 decrementing nBusySend, or any queue manipulation on CurrSendHead / Tail
		 *************************************************************************/
		PMP_TCB                  CurrSendHead;
		PMP_TCB                  CurrSendTail;
		INT32                    nBusySend;
		
		
		/**************************************************************************
		 The actual descriptor ring
		 *************************************************************************/
		IOBufferMemoryDescriptor* pTxDescRingPool;

		/**************************************************************************
		 ReadyToSend indicates where we last wrote to in the descriptor ring.
		 *************************************************************************/
		TXDMA_SERVICE_REQUEST_t  txDmaReadyToSend;
		
		
		/**************************************************************************
		 The location of the write-back status block
		 *************************************************************************/
		IOBufferMemoryDescriptor* pTxStatusPool;
		
		/**************************************************************************
		 Variables to track the Tx interrupt coalescing features
		 *************************************************************************/
		INT32                    TxPacketsSinceLastinterrupt;
} TX_RING_t, *PTX_RING_t;


#define USE_FBR0 TRUE

#ifdef USE_FBR0
//#define FBR0_BUFFER_SIZE 256
#endif

//#define FBR1_BUFFER_SIZE 2048

#define FBR_CHUNKS 32

#define MAX_DESC_PER_RING_RX         1024 




/******************************************************************************
 number of RFDs - default and min
 *****************************************************************************/
#ifdef USE_FBR0
#define RFD_LOW_WATER_MARK              40
#define NIC_MIN_NUM_RFD                 64
#define NIC_DEFAULT_NUM_RFD             1024
#else
#define RFD_LOW_WATER_MARK              20
#define NIC_MIN_NUM_RFD                 64
#define NIC_DEFAULT_NUM_RFD             256
#endif

#define NUM_PACKETS_HANDLED 256

#define ALCATEL_BAD_STATUS       0xe47f0000
#define ALCATEL_MULTICAST_PKT    0x01000000
#define ALCATEL_BROADCAST_PKT    0x02000000




/******************************************************************************
 typedefs for Free Buffer Descriptors
 *****************************************************************************/
typedef union _FBR_WORD2_t {
		UINT32 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT32 reserved:22;                         //bits 10-31
			UINT32 bi:10;                               //bits 0-9(Buffer Index)
#else
			UINT32 bi:10;                               //bits 0-9(Buffer Index)
			UINT32 reserved:22;                         //bit 10-31
#endif
		} bits;
} FBR_WORD2_t, *PFBR_WORD2_t;

typedef struct _FBR_DESC_t {
		UINT32         addr_lo;
		UINT32         addr_hi;
		FBR_WORD2_t    word2;
} FBR_DESC_t, *PFBR_DESC_t;
/*===========================================================================*/




/******************************************************************************
 Typedefs for Packet Status Ring Descriptors
 *****************************************************************************/
typedef union _PKT_STAT_DESC_WORD0_t {
		UINT32 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			// top 16 bits are from the Alcatel Status Word as enumerated in
			// PE-MCXMAC Data Sheet IPD DS54 0210-1 (also IPD-DS80 0205-2)
#if 0
			UINT32 asw_trunc:1;                         //bit 31(Rx frame truncated)
#endif
			UINT32 asw_long_evt:1;                      //bit 31(Rx long event)
			UINT32 asw_VLAN_tag:1;                      //bit 30(VLAN tag detected)
			UINT32 asw_unsupported_op:1;                //bit 29(unsupported OP code)
			UINT32 asw_pause_frame:1;                   //bit 28(is a pause frame)
			UINT32 asw_control_frame:1;                 //bit 27(is a control frame)
			UINT32 asw_dribble_nibble:1;                //bit 26(spurious bits after EOP)
			UINT32 asw_broadcast:1;                     //bit 25(has a broadcast address)
			UINT32 asw_multicast:1;                     //bit 24(has a multicast address)
			UINT32 asw_OK:1;                            //bit 23(valid CRC + no code error)
			UINT32 asw_too_long:1;                      //bit 22(frame length > 1518 bytes)
			UINT32 asw_len_chk_err:1;                   //bit 21(frame length field incorrect)
			UINT32 asw_CRC_err:1;                       //bit 20(CRC error)
			UINT32 asw_code_err:1;                      //bit 19(one or more nibbles signalled as errors)
			UINT32 asw_false_carrier_event:1;           //bit 18(bad carrier since last good packet)
			UINT32 asw_RX_DV_event:1;                   //bit 17(short receive event detected)
			UINT32 asw_prev_pkt_dropped:1;              //bit 16(e.g. IFG too small on previous)
			UINT32 unused:5;                            //bits 11-15
			UINT32 vp:1;                                //bit 10(VLAN Packet)
			UINT32 jp:1;                                //bit 9(Jumbo Packet)
			UINT32 ft:1;                                //bit 8(Frame Truncated)
			UINT32 drop:1;                              //bit 7(Drop packet)
			UINT32 rxmac_error:1;                       //bit 6(RXMAC Error Indicator)
			UINT32 wol:1;                               //bit 5(WOL Event)
			UINT32 tcpp:1;                              //bit 4(TCP checksum pass)
			UINT32 tcpa:1;                              //bit 3(TCP checksum assist)
			UINT32 ipp:1;                               //bit 2(IP checksum pass)
			UINT32 ipa:1;                               //bit 1(IP checksum assist)
			UINT32 hp:1;                                //bit 0(hash pass)
#else
			UINT32 hp:1;                                //bit 0(hash pass)
			UINT32 ipa:1;                               //bit 1(IP checksum assist)
			UINT32 ipp:1;                               //bit 2(IP checksum pass)
			UINT32 tcpa:1;                              //bit 3(TCP checksum assist)
			UINT32 tcpp:1;                              //bit 4(TCP checksum pass)
			UINT32 wol:1;                               //bit 5(WOL Event)
			UINT32 rxmac_error:1;                       //bit 6(RXMAC Error Indicator)
			UINT32 drop:1;                              //bit 7(Drop packet)
			UINT32 ft:1;                                //bit 8(Frame Truncated)
			UINT32 jp:1;                                //bit 9(Jumbo Packet)
			UINT32 vp:1;                                //bit 10(VLAN Packet)
			UINT32 unused:5;                            //bits 11-15
			UINT32 asw_prev_pkt_dropped:1;              //bit 16(e.g. IFG too small on previous)
			UINT32 asw_RX_DV_event:1;                   //bit 17(short receive event detected)
			UINT32 asw_false_carrier_event:1;           //bit 18(bad carrier since last good packet)
			UINT32 asw_code_err:1;                      //bit 19(one or more nibbles signalled as errors)
			UINT32 asw_CRC_err:1;                       //bit 20(CRC error)
			UINT32 asw_len_chk_err:1;                   //bit 21(frame length field incorrect)
			UINT32 asw_too_long:1;                      //bit 22(frame length > 1518 bytes)
			UINT32 asw_OK:1;                            //bit 23(valid CRC + no code error)
			UINT32 asw_multicast:1;                     //bit 24(has a multicast address)
			UINT32 asw_broadcast:1;                     //bit 25(has a broadcast address)
			UINT32 asw_dribble_nibble:1;                //bit 26(spurious bits after EOP)
			UINT32 asw_control_frame:1;                 //bit 27(is a control frame)
			UINT32 asw_pause_frame:1;                   //bit 28(is a pause frame)
			UINT32 asw_unsupported_op:1;                //bit 29(unsupported OP code)
			UINT32 asw_VLAN_tag:1;                      //bit 30(VLAN tag detected)
			UINT32 asw_long_evt:1;                      //bit 31(Rx long event)
#if 0
			UINT32 asw_trunc:1;                         //bit 31(Rx frame truncated)
#endif
#endif
		} bits;
} PKT_STAT_DESC_WORD0_t, *PPKT_STAT_WORD0_t;

typedef union _PKT_STAT_DESC_WORD1_t {
		UINT32 value;
		struct
		{
#ifdef __BIG_ENDIAN__
			UINT32 unused:4;                            //bits 28-31
			UINT32 ri:2;                                //bits 26-27(Ring Index)
			UINT32 bi:10;                               //bits 16-25(Buffer Index)
			UINT32 length:16;                           //bit 0-15(length in bytes)
#else
			UINT32 length:16;                           //bit 0-15(length in bytes)
			UINT32 bi:10;                               //bits 16-25(Buffer Index)
			UINT32 ri:2;                                //bits 26-27(Ring Index)
			UINT32 unused:4;                            //bits 28-31
#endif
		} bits;
} PKT_STAT_DESC_WORD1_t, *PPKT_STAT_WORD1_t;

typedef struct _PKT_STAT_DESC_t {
		PKT_STAT_DESC_WORD0_t   word0;
		PKT_STAT_DESC_WORD1_t   word1;
} PKT_STAT_DESC_t, *PPKT_STAT_DESC_t;
/*===========================================================================*/




/******************************************************************************
 Typedefs for the RX DMA status word
 *****************************************************************************/
/******************************************************************************
 RXSTAT_WORD0_t structure holds part of the status bits of the Rx DMA engine
 that get copied out to memory by the ET-1310.  Word 0 is a 32 bit word which
 contains Free Buffer ring 0 and 1 available offset.
 *****************************************************************************/
typedef union _rxstat_word0_t {
		UINT32 value;
		struct     
		{
#ifdef __BIG_ENDIAN__
			UINT32 FBR1unused:5;                        //bits 27-31
			UINT32 FBR1wrap:1;                          //bit 26
			UINT32 FBR1offset:10;                       //bits 16-25
			UINT32 FBR0unused:5;                        //bits 11-15
			UINT32 FBR0wrap:1;                          //bit 10
			UINT32 FBR0offset:10;                       //bits 0-9
#else
			UINT32 FBR0offset:10;                       //bits 0-9
			UINT32 FBR0wrap:1;                          //bit 10
			UINT32 FBR0unused:5;                        //bits 11-15
			UINT32 FBR1offset:10;                       //bits 16-25
			UINT32 FBR1wrap:1;                          //bit 26
			UINT32 FBR1unused:5;                        //bits 27-31
#endif
		} bits;
} RXSTAT_WORD0_t, *PRXSTAT_WORD0_t;




/******************************************************************************
 RXSTAT_WORD1_t structure holds part of the status bits of the Rx DMA engine
 that get copied out to memory by the ET-1310.  Word 3 is a 32 bit word which
 contains the Packet Status Ring available offset.
 *****************************************************************************/
typedef union _rxstat_word1_t {
	UINT32 value;
	struct {
#ifdef __BIG_ENDIAN__
			UINT32 PSRunused:3;                            //bits 29-31
			UINT32 PSRwrap:1;                              //bit 28
			UINT32 PSRoffset:12;                           //bits 16-27
			UINT32 reserved:16;                            //bits 0-15
#else
			UINT32 reserved:16;                            //bits 0-15
			UINT32 PSRoffset:12;                           //bits 16-27
			UINT32 PSRwrap:1;                              //bit 28
			UINT32 PSRunused:3;                            //bits 29-31
#endif
	} bits;
} RXSTAT_WORD1_t, *PRXSTAT_WORD1_t;




/******************************************************************************
 RX_STATUS_BLOCK_t is sructure representing the status of the Rx DMA engine
 it sits in free memory, and is pointed to by 0x101c / 0x1020
 *****************************************************************************/

typedef struct _rx_status_block_t {
		RXSTAT_WORD0_t    Word0;
		RXSTAT_WORD1_t    Word1;
} RX_STATUS_BLOCK_t, *PRX_STATUS_BLOCK_t;




/******************************************************************************
 Structure for look-up table holding free buffer ring pointers
 *****************************************************************************/
typedef struct _FbrLookupTable {
	void                 *Va            [MAX_DESC_PER_RING_RX];
	UINT32                PAHigh        [MAX_DESC_PER_RING_RX];
	UINT32                PALow         [MAX_DESC_PER_RING_RX];
} FBRLOOKUPTABLE, *PFBRLOOKUPTABLE;




/******************************************************************************
 RX_RING_t is sructure representing the adaptor's local reference(s) to the
 rings
 *****************************************************************************/
typedef struct _rx_ring_t {
	IOBufferMemoryDescriptor* pFbr1RingPool;

	IOBufferMemoryDescriptor* pFbr1MemPool[ MAX_DESC_PER_RING_RX / FBR_CHUNKS ];
	FBRLOOKUPTABLE             Fbr1;

	RXDMA_FBR_FULL_OFFSET_t     local_Fbr1_full;
	UINT32                      Fbr1NumEntries;
	UINT32                      Fbr1BufferSize;
		
	IOBufferMemoryDescriptor* pPSRingPool;

	RXDMA_PSR_FULL_OFFSET_t     local_psr_full;
	UINT32                      PsrNumEntries;
		
	IOBufferMemoryDescriptor* pRxStatusPool;
		
	/**************************************************************************
	 RECV
	 *************************************************************************/
	struct list_head            RecvList;
	UINT32                      nReadyRecv;
	UINT32                      NumRfd;
		
	BOOL_t                      UnfinishedReceives;
} RX_RING_t, *PRX_RING_t;




#define TRUEPHY_SUCCESS 0
#define TRUEPHY_FAILURE 1
typedef void    *TRUEPHY_HANDLE;
typedef void    *TRUEPHY_PLATFORM_HANDLE;
typedef void    *TRUEPHY_OSAL_HANDLE;




/******************************************************************************
 CONSTANTS for PHY Register
 *****************************************************************************/
/* MI Register Addresses */
#define MI_CONTROL_REG                      0
#define MI_STATUS_REG                       1
#define MI_PHY_IDENTIFIER_1_REG             2
#define MI_PHY_IDENTIFIER_2_REG             3
#define MI_AUTONEG_ADVERTISEMENT_REG        4
#define MI_AUTONEG_LINK_PARTNER_ABILITY_REG 5
#define MI_AUTONEG_EXPANSION_REG            6
#define MI_AUTONEG_NEXT_PAGE_TRANSMIT_REG   7
#define MI_LINK_PARTNER_NEXT_PAGE_REG       8
#define MI_1000BASET_CONTROL_REG            9
#define MI_1000BASET_STATUS_REG             10
#define MI_RESERVED11_REG                   11
#define MI_RESERVED12_REG                   12
#define MI_RESERVED13_REG                   13
#define MI_RESERVED14_REG                   14
#define MI_EXTENDED_STATUS_REG              15

/* VMI Register Addresses */
#define VMI_RESERVED16_REG                  16
#define VMI_RESERVED17_REG                  17
#define VMI_RESERVED18_REG                  18
#define VMI_LOOPBACK_CONTROL_REG            19
#define VMI_RESERVED20_REG                  20
#define VMI_MI_CONTROL_REG                  21
#define VMI_PHY_CONFIGURATION_REG           22
#define VMI_PHY_CONTROL_REG                 23
#define VMI_INTERRUPT_MASK_REG              24
#define VMI_INTERRUPT_STATUS_REG            25
#define VMI_PHY_STATUS_REG                  26
#define VMI_LED_CONTROL_1_REG               27
#define VMI_LED_CONTROL_2_REG               28
#define VMI_RESERVED29_REG                  29
#define VMI_RESERVED30_REG                  30
#define VMI_RESERVED31_REG                  31




/******************************************************************************
 PHY Register Mapping(MI) Management Interface Regs
 *****************************************************************************/
typedef struct _MI_REGS_t {
		UCHAR bmcr;         // Basic mode control reg(Reg 0x00)
		UCHAR bmsr;         // Basic mode status reg(Reg 0x01)
		UCHAR idr1;         // Phy identifier reg 1(Reg 0x02)
		UCHAR idr2;         // Phy identifier reg 2(Reg 0x03)
		UCHAR anar;         // Auto-Negotiation advertisement(Reg 0x04)
		UCHAR anlpar;       // Auto-Negotiation link Partner Ability(Reg 0x05)
		UCHAR aner;         // Auto-Negotiation expansion reg(Reg 0x06)
		UCHAR annptr;       // Auto-Negotiation next page transmit reg(Reg 0x07)
		UCHAR lpnpr;        // link partner next page reg(Reg 0x08)
		UCHAR gcr;          // Gigabit basic mode control reg(Reg 0x09)
		UCHAR gsr;          // Gigabit basic mode status reg(Reg 0x0A)
		UCHAR mi_res1[4];   // Future use by MI working group(Reg 0x0B - 0x0E)
		UCHAR esr;          // Extended status reg(Reg 0x0F)
		UCHAR mi_res2[3];   // Future use by MI working group(Reg 0x10 - 0x12)
		UCHAR loop_ctl;     // Loopback Control Reg(Reg 0x13)
		UCHAR mi_res3;      // Future use by MI working group(Reg 0x14)
		UCHAR mcr;          // MI Control Reg(Reg 0x15)
		UCHAR pcr;          // Configuration Reg(Reg 0x16)
		UCHAR phy_ctl;      // PHY Control Reg(Reg 0x17)
		UCHAR imr;          // Interrupt Mask Reg(Reg 0x18)
		UCHAR isr;          // Interrupt Status Reg(Reg 0x19)
		UCHAR psr;          // PHY Status Reg(Reg 0x1A)
		UCHAR lcr1;         // LED Control 1 Reg(Reg 0x1B)
		UCHAR lcr2;         // LED Control 2 Reg(Reg 0x1C)
		UCHAR mi_res4[3];   // Future use by MI working group(Reg 0x1D - 0x1F)
} MI_REGS_t, *PMI_REGS_t;




/******************************************************************************
 MI Register 0: Basic mode control register
 *****************************************************************************/
typedef union _MI_BMCR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 reset:1;                             //bit 15
			UINT16 loopback:1;                          //bit 14
			UINT16 speed_sel:1;                         //bit 13
			UINT16 enable_autoneg:1;                    //bit 12
			UINT16 power_down:1;                        //bit 11
			UINT16 isolate:1;                           //bit 10
			UINT16 restart_autoneg:1;                   //bit 9
			UINT16 duplex_mode:1;                       //bit 8
			UINT16 col_test:1;                          //bit 7
			UINT16 speed_1000_sel:1;                    //bit 6
			UINT16 res1:6;                              //bits 0-5
#else
			UINT16 res1:6;                              //bits 0-5
			UINT16 speed_1000_sel:1;                    //bit 6
			UINT16 col_test:1;                          //bit 7
			UINT16 duplex_mode:1;                       //bit 8
			UINT16 restart_autoneg:1;                   //bit 9
			UINT16 isolate:1;                           //bit 10
			UINT16 power_down:1;                        //bit 11
			UINT16 enable_autoneg:1;                    //bit 12
			UINT16 speed_sel:1;                         //bit 13
			UINT16 loopback:1;                          //bit 14
			UINT16 reset:1;                             //bit 15
#endif
		} bits;
	}
	MI_BMCR_t, *PMI_BMCR_t;




/******************************************************************************
 MI Register 1:  Basic mode status register
 *****************************************************************************/
typedef union _MI_BMSR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 link_100T4:1;                        //bit 15
			UINT16 link_100fdx:1;                       //bit 14
			UINT16 link_100hdx:1;                       //bit 13
			UINT16 link_10fdx:1;                        //bit 12
			UINT16 link_10hdx:1;                        //bit 11
			UINT16 link_100T2fdx:1;                     //bit 10
			UINT16 link_100T2hdx:1;                     //bit 9
			UINT16 extend_status:1;                     //bit 8
			UINT16 res1:1;                              //bit 7
			UINT16 preamble_supress:1;                  //bit 6
			UINT16 auto_neg_complete:1;                 //bit 5
			UINT16 remote_fault:1;                      //bit 4
			UINT16 auto_neg_able:1;                     //bit 3
			UINT16 link_status:1;                       //bit 2
			UINT16 jabber_detect:1;                     //bit 1
			UINT16 ext_cap:1;                           //bit 0
#else
			UINT16 ext_cap:1;                           //bit 0
			UINT16 jabber_detect:1;                     //bit 1
			UINT16 link_status:1;                       //bit 2
			UINT16 auto_neg_able:1;                     //bit 3
			UINT16 remote_fault:1;                      //bit 4
			UINT16 auto_neg_complete:1;                 //bit 5
			UINT16 preamble_supress:1;                  //bit 6
			UINT16 res1:1;                              //bit 7
			UINT16 extend_status:1;                     //bit 8
			UINT16 link_100T2hdx:1;                     //bit 9
			UINT16 link_100T2fdx:1;                     //bit 10
			UINT16 link_10hdx:1;                        //bit 11
			UINT16 link_10fdx:1;                        //bit 12
			UINT16 link_100hdx:1;                       //bit 13
			UINT16 link_100fdx:1;                       //bit 14
			UINT16 link_100T4:1;                        //bit 15
#endif
		} bits;
	} 
	MI_BMSR_t, *PMI_BMSR_t;




/******************************************************************************
 MI Register 2: Physical Identifier 1
 *****************************************************************************/
typedef union _MI_IDR1_t 
	{
		UINT16 value;
		struct 
		{
			UINT16 ieee_address:16;                     //0x0282 default(bits 0-15)
		} bits;
	}
	MI_IDR1_t, *PMI_IDR1_t;




/******************************************************************************
 MI Register 3: Physical Identifier 2
 *****************************************************************************/
typedef union _MI_IDR2_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 ieee_address:6;                      //111100 default(bits 10-15)
			UINT16 model_no:6;                          //000001 default(bits 4-9)
			UINT16 rev_no:4;                            //0010   default(bits 0-3)
#else
			UINT16 rev_no:4;                            //0010   default(bits 0-3)
			UINT16 model_no:6;                          //000001 default(bits 4-9)
			UINT16 ieee_address:6;                      //111100 default(bits 10-15)
#endif
		} bits;
	}
	MI_IDR2_t, *PMI_IDR2_t; 




/******************************************************************************
 MI Register 4: Auto-negotiation advertisement register
 *****************************************************************************/
typedef union _MI_ANAR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 np_indication:1;                     //bit 15
			UINT16 res2:1;                              //bit 14
			UINT16 remote_fault:1;                      //bit 13
			UINT16 res1:1;                              //bit 12
			UINT16 cap_asmpause:1;                      //bit 11
			UINT16 cap_pause:1;                         //bit 10
			UINT16 cap_100T4:1;                         //bit 9
			UINT16 cap_100fdx:1;                        //bit 8
			UINT16 cap_100hdx:1;                        //bit 7
			UINT16 cap_10fdx:1;                         //bit 6
			UINT16 cap_10hdx:1;                         //bit 5
			UINT16 selector:5;                          //bits 0-4
#else
			UINT16 selector:5;                          //bits 0-4
			UINT16 cap_10hdx:1;                         //bit 5
			UINT16 cap_10fdx:1;                         //bit 6
			UINT16 cap_100hdx:1;                        //bit 7
			UINT16 cap_100fdx:1;                        //bit 8
			UINT16 cap_100T4:1;                         //bit 9
			UINT16 cap_pause:1;                         //bit 10
			UINT16 cap_asmpause:1;                      //bit 11
			UINT16 res1:1;                              //bit 12
			UINT16 remote_fault:1;                      //bit 13
			UINT16 res2:1;                              //bit 14
			UINT16 np_indication:1;                     //bit 15
#endif
		} bits;
	} 
	MI_ANAR_t, *PMI_ANAR_t;




/******************************************************************************
 MI Register 5: Auto-negotiation link partner advertisement register
 *****************************************************************************/
typedef struct _MI_ANLPAR_t
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 np_indication:1;                     //bit 15
			UINT16 acknowledge:1;                       //bit 14
			UINT16 remote_fault:1;                      //bit 13
			UINT16 res1:1;                              //bit 12
			UINT16 cap_asmpause:1;                      //bit 11
			UINT16 cap_pause:1;                         //bit 10
			UINT16 cap_100T4:1;                         //bit 9
			UINT16 cap_100fdx:1;                        //bit 8
			UINT16 cap_100hdx:1;                        //bit 7
			UINT16 cap_10fdx:1;                         //bit 6
			UINT16 cap_10hdx:1;                         //bit 5
			UINT16 selector:5;                          //bits 0-4
#else
			UINT16 selector:5;                          //bits 0-4
			UINT16 cap_10hdx:1;                         //bit 5
			UINT16 cap_10fdx:1;                         //bit 6
			UINT16 cap_100hdx:1;                        //bit 7
			UINT16 cap_100fdx:1;                        //bit 8
			UINT16 cap_100T4:1;                         //bit 9
			UINT16 cap_pause:1;                         //bit 10
			UINT16 cap_asmpause:1;                      //bit 11
			UINT16 res1:1;                              //bit 12
			UINT16 remote_fault:1;                      //bit 13
			UINT16 acknowledge:1;                       //bit 14
			UINT16 np_indication:1;                     //bit 15
#endif
		} bits;
	} 
	MI_ANLPAR_t, *PMI_ANLPAR_t;




/******************************************************************************
 MI Register 6: Auto-negotiation expansion register
 *****************************************************************************/
typedef union _MI_ANER_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 res:11;                              //bits 5-15
			UINT16 pdf:1;                               //bit 4
			UINT16 lp_np_able:1;                        //bit 3
			UINT16 np_able:1;                           //bit 2
			UINT16 page_rx:1;                           //bit 1
			UINT16 lp_an_able:1;                        //bit 0
#else
			UINT16 lp_an_able:1;                        //bit 0
			UINT16 page_rx:1;                           //bit 1
			UINT16 np_able:1;                           //bit 2
			UINT16 lp_np_able:1;                        //bit 3
			UINT16 pdf:1;                               //bit 4
			UINT16 res:11;                              //bits 5-15
#endif
		} bits;
	} 
	MI_ANER_t, *PMI_ANER_t;




/******************************************************************************
 MI Register 7: Auto-negotiation next page transmit reg(0x07)
 *****************************************************************************/
typedef union _MI_ANNPTR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 np:1;                                //bit 15
			UINT16 res1:1;                              //bit 14
			UINT16 msg_page:1;                          //bit 13
			UINT16 ack2:1;                              //bit 12
			UINT16 toggle:1;                            //bit 11
			UINT16 msg:11;                              //bits 0-10
#else
			UINT16 msg:11;                              //bits 0-10
			UINT16 toggle:1;                            //bit 11
			UINT16 ack2:1;                              //bit 12
			UINT16 msg_page:1;                          //bit 13
			UINT16 res1:1;                              //bit 14
			UINT16 np:1;                                //bit 15
#endif
		} bits;
	} 
	MI_ANNPTR_t, *PMI_ANNPTR_t;




/******************************************************************************
 MI Register 8: Link Partner Next Page Reg(0x08)
 *****************************************************************************/
typedef union _MI_LPNPR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 np:1;                                //bit 15
			UINT16 ack:1;                               //bit 14
			UINT16 msg_page:1;                          //bit 13
			UINT16 ack2:1;                              //bit 12
			UINT16 toggle:1;                            //bit 11
			UINT16 msg:11;                              //bits 0-10
#else
			UINT16 msg:11;                              //bits 0-10
			UINT16 toggle:1;                            //bit 11
			UINT16 ack2:1;                              //bit 12
			UINT16 msg_page:1;                          //bit 13
			UINT16 ack:1;                               //bit 14
			UINT16 np:1;                                //bit 15
#endif
		} bits;
	} 
	MI_LPNPR_t, *PMI_LPNPR_t;




/******************************************************************************
 MI Register 9: 1000BaseT Control Reg(0x09)
 *****************************************************************************/
typedef union _MI_GCR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 test_mode:3;                         //bits 13-15
			UINT16 ms_config_en:1;                      //bit 12
			UINT16 ms_value:1;                          //bit 11
			UINT16 port_type:1;                         //bit 10
			UINT16 link_1000fdx:1;                      //bit 9
			UINT16 link_1000hdx:1;                      //bit 8
			UINT16 res:8;                               //bit 0-7
#else
			UINT16 res:8;                               //bit 0-7
			UINT16 link_1000hdx:1;                      //bit 8
			UINT16 link_1000fdx:1;                      //bit 9
			UINT16 port_type:1;                         //bit 10
			UINT16 ms_value:1;                          //bit 11
			UINT16 ms_config_en:1;                      //bit 12
			UINT16 test_mode:3;                         //bits 13-15
#endif
		} bits;
	} 
	MI_GCR_t, *PMI_GCR_t;




/******************************************************************************
 MI Register 10: 1000BaseT Status Reg(0x0A)
 *****************************************************************************/
typedef union _MI_GSR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 ms_config_fault:1;                   //bit 15
			UINT16 ms_resolve:1;                        //bit 14
			UINT16 local_rx_status:1;                   //bit 13
			UINT16 remote_rx_status:1;                  //bit 12
			UINT16 link_1000fdx:1;                      //bit 11
			UINT16 link_1000hdx:1;                      //bit 10
			UINT16 res:2;                               //bits 8-9
			UINT16 idle_err_cnt:8;                      //bits 0-7
#else
			UINT16 idle_err_cnt:8;                      //bits 0-7
			UINT16 res:2;                               //bits 8-9
			UINT16 link_1000hdx:1;                      //bit 10
			UINT16 link_1000fdx:1;                      //bit 11
			UINT16 remote_rx_status:1;                  //bit 12
			UINT16 local_rx_status:1;                   //bit 13
			UINT16 ms_resolve:1;                        //bit 14
			UINT16 ms_config_fault:1;                   //bit 15
#endif
		} bits;
	} 
	MI_GSR_t, *PMI_GSR_t;




/******************************************************************************
 MI Register 11 - 14: Reserved Regs(0x0B - 0x0E)
 *****************************************************************************/
typedef union _MI_RES_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 res15:1;                             //bit 15
			UINT16 res14:1;                             //bit 14
			UINT16 res13:1;                             //bit 13
			UINT16 res12:1;                             //bit 12
			UINT16 res11:1;                             //bit 11
			UINT16 res10:1;                             //bit 10
			UINT16 res9:1;                              //bit 9
			UINT16 res8:1;                              //bit 8
			UINT16 res7:1;                              //bit 7
			UINT16 res6:1;                              //bit 6
			UINT16 res5:1;                              //bit 5
			UINT16 res4:1;                              //bit 4
			UINT16 res3:1;                              //bit 3
			UINT16 res2:1;                              //bit 2
			UINT16 res1:1;                              //bit 1
			UINT16 res0:1;                              //bit 0
#else
			UINT16 res0:1;                              //bit 0
			UINT16 res1:1;                              //bit 1
			UINT16 res2:1;                              //bit 2
			UINT16 res3:1;                              //bit 3
			UINT16 res4:1;                              //bit 4
			UINT16 res5:1;                              //bit 5
			UINT16 res6:1;                              //bit 6
			UINT16 res7:1;                              //bit 7
			UINT16 res8:1;                              //bit 8
			UINT16 res9:1;                              //bit 9
			UINT16 res10:1;                             //bit 10
			UINT16 res11:1;                             //bit 11
			UINT16 res12:1;                             //bit 12
			UINT16 res13:1;                             //bit 13
			UINT16 res14:1;                             //bit 14
			UINT16 res15:1;                             //bit 15
#endif
		} bits;
	}
	MI_RES_t, *PMI_RES_t;




/******************************************************************************
 MI Register 15: Extended status Reg(0x0F)
 *****************************************************************************/
typedef union _MI_ESR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 link_1000Xfdx:1;                     //bit 15
			UINT16 link_1000Xhdx:1;                     //bit 14
			UINT16 link_1000fdx:1;                      //bit 13
			UINT16 link_1000hdx:1;                      //bit 12
			UINT16 res:12;                              //bit 0-11
#else
			UINT16 res:12;                              //bit 0-11
			UINT16 link_1000hdx:1;                      //bit 12
			UINT16 link_1000fdx:1;                      //bit 13
			UINT16 link_1000Xhdx:1;                     //bit 14
			UINT16 link_1000Xfdx:1;                     //bit 15
#endif
		} bits;
	}
	MI_ESR_t, *PMI_ESR_t;




/******************************************************************************
 MI Register 16 - 18: Reserved Reg(0x10-0x12)
 *****************************************************************************/




/******************************************************************************
 MI Register 19: Loopback Control Reg(0x13)
 *****************************************************************************/
typedef union _MI_LCR_t 
	{
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 mii_en:1;                            //bit 15
			UINT16 pcs_en:1;                            //bit 14
			UINT16 pmd_en:1;                            //bit 13
			UINT16 all_digital_en:1;                    //bit 12
			UINT16 replica_en:1;                        //bit 11
			UINT16 line_driver_en:1;                    //bit 10
			UINT16 res:10;                              //bit 0-9
#else
			UINT16 res:10;                              //bit 0-9
			UINT16 line_driver_en:1;                    //bit 10
			UINT16 replica_en:1;                        //bit 11
			UINT16 all_digital_en:1;                    //bit 12
			UINT16 pmd_en:1;                            //bit 13
			UINT16 pcs_en:1;                            //bit 14
			UINT16 mii_en:1;                            //bit 15
#endif
		} bits;
	}
	MI_LCR_t, *PMI_LCR_t;




/******************************************************************************
 MI Register 20: Reserved Reg(0x14)
 *****************************************************************************/




/******************************************************************************
 MI Register 21: Management Interface Control Reg(0x15)
 *****************************************************************************/
typedef union _MI_MICR_t {
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 res1:5;                              //bits 11-15
			UINT16 mi_error_count:7;                    //bits 4-10
			UINT16 res2:1;                              //bit 3
			UINT16 ignore_10g_fr:1;                     //bit 2
			UINT16 res3:1;                              //bit 1
			UINT16 preamble_supress_en:1;               //bit 0
#else
			UINT16 preamble_supress_en:1;               //bit 0
			UINT16 res3:1;                              //bit 1
			UINT16 ignore_10g_fr:1;                     //bit 2
			UINT16 res2:1;                              //bit 3
			UINT16 mi_error_count:7;                    //bits 4-10
			UINT16 res1:5;                              //bits 11-15
#endif
		} bits;
} MI_MICR_t, *PMI_MICR_t;




/******************************************************************************
 MI Register 22: PHY Configuration Reg(0x16)
 *****************************************************************************/
typedef union _MI_PHY_CONFIG_t {
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 crs_tx_en:1;                         //bit 15
			UINT16 res1:1;                              //bit 14
			UINT16 tx_fifo_depth:2;                     //bits 12-13
			UINT16 speed_downshift:2;                   //bits 10-11
			UINT16 pbi_detect:1;                        //bit 9
			UINT16 tbi_rate:1;                          //bit 8
			UINT16 alternate_np:1;                      //bit 7
			UINT16 group_mdio_en:1;                     //bit 6
			UINT16 tx_clock_en:1;                       //bit 5
			UINT16 sys_clock_en:1;                      //bit 4
			UINT16 res2:1;                              //bit 3
			UINT16 mac_if_mode:3;                       //bits 0-2
#else
			UINT16 mac_if_mode:3;                       //bits 0-2
			UINT16 res2:1;                              //bit 3
			UINT16 sys_clock_en:1;                      //bit 4
			UINT16 tx_clock_en:1;                       //bit 5
			UINT16 group_mdio_en:1;                     //bit 6
			UINT16 alternate_np:1;                      //bit 7
			UINT16 tbi_rate:1;                          //bit 8
			UINT16 pbi_detect:1;                        //bit 9
			UINT16 speed_downshift:2;                   //bits 10-11
			UINT16 tx_fifo_depth:2;                     //bits 12-13
			UINT16 res1:1;                              //bit 14
			UINT16 crs_tx_en:1;                         //bit 15
#endif
		} bits;
} MI_PHY_CONFIG_t, *PMI_PHY_CONFIG_t;




/******************************************************************************
 MI Register 23: PHY CONTROL Reg(0x17)
 *****************************************************************************/
typedef union _MI_PHY_CONTROL_t {
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 res1:1;                              //bit 15
			UINT16 tdr_en:1;                            //bit 14
			UINT16 res2:1;                              //bit 13
			UINT16 downshift_attempts:2;                //bits 11-12
			UINT16 res3:5;                              //bit 6-10
			UINT16 jabber_10baseT:1;                    //bit 5
			UINT16 sqe_10baseT:1;                       //bit 4
			UINT16 tp_loopback_10baseT:1;               //bit 3
			UINT16 preamble_gen_en:1;                   //bit 2
			UINT16 res4:1;                              //bit 1
			UINT16 force_int:1;                         //bit 0
#else
			UINT16 force_int:1;                         //bit 0
			UINT16 res4:1;                              //bit 1
			UINT16 preamble_gen_en:1;                   //bit 2
			UINT16 tp_loopback_10baseT:1;               //bit 3
			UINT16 sqe_10baseT:1;                       //bit 4
			UINT16 jabber_10baseT:1;                    //bit 5
			UINT16 res3:5;                              //bit 6-10
			UINT16 downshift_attempts:2;                //bits 11-12
			UINT16 res2:1;                              //bit 13
			UINT16 tdr_en:1;                            //bit 14
			UINT16 res1:1;                              //bit 15
#endif
		} bits;
} MI_PHY_CONTROL_t, *PMI_PHY_CONTROL_t;




/******************************************************************************
 MI Register 24: Interrupt Mask Reg(0x18)
 *****************************************************************************/
typedef union _MI_IMR_t {
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 res1:6;                              //bits 10-15
			UINT16 mdio_sync_lost:1;                    //bit 9
			UINT16 autoneg_status:1;                    //bit 8
			UINT16 hi_bit_err:1;                        //bit 7
			UINT16 np_rx:1;                             //bit 6
			UINT16 err_counter_full:1;                  //bit 5
			UINT16 fifo_over_underflow:1;               //bit 4
			UINT16 rx_status:1;                         //bit 3
			UINT16 link_status:1;                       //bit 2
			UINT16 automatic_speed:1;                   //bit 1
			UINT16 int_en:1;                            //bit 0
#else
			UINT16 int_en:1;                            //bit 0
			UINT16 automatic_speed:1;                   //bit 1
			UINT16 link_status:1;                       //bit 2
			UINT16 rx_status:1;                         //bit 3
			UINT16 fifo_over_underflow:1;               //bit 4
			UINT16 err_counter_full:1;                  //bit 5
			UINT16 np_rx:1;                             //bit 6
			UINT16 hi_bit_err:1;                        //bit 7
			UINT16 autoneg_status:1;                    //bit 8
			UINT16 mdio_sync_lost:1;                    //bit 9
			UINT16 res1:6;                              //bits 10-15
#endif
		} bits;
} MI_IMR_t, *PMI_IMR_t;




/******************************************************************************
 MI Register 25: Interrupt Status Reg(0x19)
 *****************************************************************************/
typedef union _MI_ISR_t {
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 res1:6;                              //bits 10-15
			UINT16 mdio_sync_lost:1;                    //bit 9
			UINT16 autoneg_status:1;                    //bit 8
			UINT16 hi_bit_err:1;                        //bit 7
			UINT16 np_rx:1;                             //bit 6
			UINT16 err_counter_full:1;                  //bit 5
			UINT16 fifo_over_underflow:1;               //bit 4
			UINT16 rx_status:1;                         //bit 3
			UINT16 link_status:1;                       //bit 2
			UINT16 automatic_speed:1;                   //bit 1
			UINT16 int_en:1;                            //bit 0
#else
			UINT16 int_en:1;                            //bit 0
			UINT16 automatic_speed:1;                   //bit 1
			UINT16 link_status:1;                       //bit 2
			UINT16 rx_status:1;                         //bit 3
			UINT16 fifo_over_underflow:1;               //bit 4
			UINT16 err_counter_full:1;                  //bit 5
			UINT16 np_rx:1;                             //bit 6
			UINT16 hi_bit_err:1;                        //bit 7
			UINT16 autoneg_status:1;                    //bit 8
			UINT16 mdio_sync_lost:1;                    //bit 9
			UINT16 res1:6;                              //bits 10-15
#endif
		} bits;
} MI_ISR_t, *PMI_ISR_t;




/******************************************************************************
 MI Register 26: PHY Status Reg(0x1A)
 *****************************************************************************/
typedef union _MI_PSR_t {
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 res1:1;                              //bit 15
			UINT16 autoneg_fault:2;                     //bit 13-14
			UINT16 autoneg_status:1;                    //bit 12
			UINT16 mdi_x_status:1;                      //bit 11
			UINT16 polarity_status:1;                   //bit 10
			UINT16 speed_status:2;                      //bits 8-9
			UINT16 duplex_status:1;                     //bit 7
			UINT16 link_status:1;                       //bit 6
			UINT16 tx_status:1;                         //bit 5
			UINT16 rx_status:1;                         //bit 4
			UINT16 collision_status:1;                  //bit 3
			UINT16 autoneg_en:1;                        //bit 2
			UINT16 pause_en:1;                          //bit 1
			UINT16 asymmetric_dir:1;                    //bit 0
#else
			UINT16 asymmetric_dir:1;                    //bit 0
			UINT16 pause_en:1;                          //bit 1
			UINT16 autoneg_en:1;                        //bit 2
			UINT16 collision_status:1;                  //bit 3
			UINT16 rx_status:1;                         //bit 4
			UINT16 tx_status:1;                         //bit 5
			UINT16 link_status:1;                       //bit 6
			UINT16 duplex_status:1;                     //bit 7
			UINT16 speed_status:2;                      //bits 8-9
			UINT16 polarity_status:1;                   //bit 10
			UINT16 mdi_x_status:1;                      //bit 11
			UINT16 autoneg_status:1;                    //bit 12
			UINT16 autoneg_fault:2;                     //bit 13-14
			UINT16 res1:1;                              //bit 15
#endif
		} bits;
} MI_PSR_t, *PMI_PSR_t;




/******************************************************************************
 MI Register 27: LED Control Reg 1(0x1B)
 *****************************************************************************/
typedef union _MI_LCR1_t {
		UINT16 value;
		struct 
		{
#ifdef __BIG_ENDIAN__
			UINT16 res1:2;                              //bits 14-15
			UINT16 led_dup_indicate:2;                  //bits 12-13
			UINT16 led_10baseT:2;                       //bits 10-11
			UINT16 led_collision:2;                     //bits 8-9
			UINT16 res2:2;                              //bits 6-7
			UINT16 res3:2;                              //bits 4-5
			UINT16 pulse_dur:2;                         //bits 2-3
			UINT16 pulse_stretch1:1;                    //bit 1
			UINT16 pulse_stretch0:1;                    //bit 0
#else
			UINT16 pulse_stretch0:1;                    //bit 0
			UINT16 pulse_stretch1:1;                    //bit 1
			UINT16 pulse_dur:2;                         //bits 2-3
			UINT16 res3:2;                              //bits 4-5
			UINT16 res2:2;                              //bits 6-7
			UINT16 led_collision:2;                     //bits 8-9
			UINT16 led_10baseT:2;                       //bits 10-11
			UINT16 led_dup_indicate:2;                  //bits 12-13
			UINT16 res1:2;                              //bits 14-15
#endif
		} bits;
} MI_LCR1_t, *PMI_LCR1_t;




/******************************************************************************
 MI Register 28: LED Control Reg 2(0x1C)
 *****************************************************************************/
typedef union _MI_LCR2_t {
	UINT16 value;
	struct {
#ifdef __BIG_ENDIAN__
			UINT16 led_link:4;                          //bits 12-15
			UINT16 led_tx_rx:4;                         //bits 8-11
			UINT16 led_100BaseTX:4;                     //bits 4-7
			UINT16 led_1000BaseT:4;                     //bits 0-3
#else
			UINT16 led_1000BaseT:4;                     //bits 0-3
			UINT16 led_100BaseTX:4;                     //bits 4-7
			UINT16 led_tx_rx:4;                         //bits 8-11
			UINT16 led_link:4;                          //bits 12-15
#endif
	} bits;
} MI_LCR2_t, *PMI_LCR2_t;




/******************************************************************************
 MI Register 29 - 31: Reserved Reg(0x1D - 0x1E)
 *****************************************************************************/



/******************************************************************************
 Defines for PHY access routines
 *****************************************************************************/
// Define bit operation flags
#define TRUEPHY_BIT_CLEAR               0
#define TRUEPHY_BIT_SET                 1
#define TRUEPHY_BIT_READ                2

// Define read/write operation flags 
#ifndef TRUEPHY_READ
#define TRUEPHY_READ                    0
#define TRUEPHY_WRITE                   1
#define TRUEPHY_MASK                    2
#endif

// Define speeds
#define TRUEPHY_SPEED_10MBPS            0
#define TRUEPHY_SPEED_100MBPS           1
#define TRUEPHY_SPEED_1000MBPS          2

// Define duplex modes
#define TRUEPHY_DUPLEX_HALF             0
#define TRUEPHY_DUPLEX_FULL             1

// Define master/slave configuration values
#define TRUEPHY_CFG_SLAVE               0
#define TRUEPHY_CFG_MASTER              1

// Define MDI/MDI-X settings
#define TRUEPHY_MDI                     0
#define TRUEPHY_MDIX                    1
#define TRUEPHY_AUTO_MDI_MDIX           2

// Define 10Base-T link polarities
#define TRUEPHY_POLARITY_NORMAL         0
#define TRUEPHY_POLARITY_INVERTED       1

// Define auto-negotiation results
#define TRUEPHY_ANEG_NOT_COMPLETE       0
#define TRUEPHY_ANEG_COMPLETE           1
#define TRUEPHY_ANEG_DISABLED           2

/* Define duplex advertisment flags */
#define TRUEPHY_ADV_DUPLEX_NONE         0x00
#define TRUEPHY_ADV_DUPLEX_FULL         0x01
#define TRUEPHY_ADV_DUPLEX_HALF         0x02
#define TRUEPHY_ADV_DUPLEX_BOTH     \
(TRUEPHY_ADV_DUPLEX_FULL | TRUEPHY_ADV_DUPLEX_HALF)

#define PHY_CONTROL                0x00     //#define TRU_MI_CONTROL_REGISTER                 0
#define PHY_STATUS                 0x01     //#define TRU_MI_STATUS_REGISTER                  1
#define PHY_ID_1                   0x02     //#define TRU_MI_PHY_IDENTIFIER_1_REGISTER        2
#define PHY_ID_2                   0x03     //#define TRU_MI_PHY_IDENTIFIER_2_REGISTER        3
#define PHY_AUTO_ADVERTISEMENT     0x04     //#define TRU_MI_ADVERTISEMENT_REGISTER           4
#define PHY_AUTO_LINK_PARTNER      0x05     //#define TRU_MI_LINK_PARTNER_ABILITY_REGISTER    5
#define PHY_AUTO_EXPANSION         0x06     //#define TRU_MI_EXPANSION_REGISTER               6
#define PHY_AUTO_NEXT_PAGE_TX      0x07     //#define TRU_MI_NEXT_PAGE_TRANSMIT_REGISTER      7
#define PHY_LINK_PARTNER_NEXT_PAGE 0x08     //#define TRU_MI_LINK_PARTNER_NEXT_PAGE_REGISTER  8
#define PHY_1000_CONTROL           0x09     //#define TRU_MI_1000BASET_CONTROL_REGISTER       9
#define PHY_1000_STATUS            0x0A     //#define TRU_MI_1000BASET_STATUS_REGISTER        10


#define PHY_EXTENDED_STATUS        0x0F     //#define TRU_MI_EXTENDED_STATUS_REGISTER         15

// some defines for modem registers that seem to be 'reserved'
#define PHY_INDEX_REG              0x10
#define PHY_DATA_REG               0x11

#define PHY_MPHY_CONTROL_REG       0x12     //#define TRU_VMI_MPHY_CONTROL_REGISTER           18

#define PHY_LOOPBACK_CONTROL       0x13     //#define TRU_VMI_LOOPBACK_CONTROL_1_REGISTER     19
//#define TRU_VMI_LOOPBACK_CONTROL_2_REGISTER     20
#define PHY_REGISTER_MGMT_CONTROL  0x15     //#define TRU_VMI_MI_SEQ_CONTROL_REGISTER         21
#define PHY_CONFIG                 0x16     //#define TRU_VMI_CONFIGURATION_REGISTER          22
#define PHY_PHY_CONTROL            0x17     //#define TRU_VMI_PHY_CONTROL_REGISTER            23
#define PHY_INTERRUPT_MASK         0x18     //#define TRU_VMI_INTERRUPT_MASK_REGISTER         24
#define PHY_INTERRUPT_STATUS       0x19     //#define TRU_VMI_INTERRUPT_STATUS_REGISTER       25
#define PHY_PHY_STATUS             0x1A     //#define TRU_VMI_PHY_STATUS_REGISTER             26
#define PHY_LED_1                  0x1B     //#define TRU_VMI_LED_CONTROL_1_REGISTER          27
#define PHY_LED_2                  0x1C     //#define TRU_VMI_LED_CONTROL_2_REGISTER          28
//#define TRU_VMI_LINK_CONTROL_REGISTER           29
//#define TRU_VMI_TIMING_CONTROL_REGISTER 


/******************************************************************************
 Enum for use with netif_indicate_status
 *****************************************************************************/
typedef enum netif_status {
		NETIF_STATUS_INVALID           = 0,
		NETIF_STATUS_MEDIA_CONNECT,
		NETIF_STATUS_MEDIA_DISCONNECT,
		NETIF_STATUS_MAX
} NETIF_STATUS;




/******************************************************************************
 CONSTANTS FOR JAGCORE
 *****************************************************************************/
#define INTERNAL_MEM_SIZE       0x400  //1024 of internal memory
#define INTERNAL_MEM_RX_OFFSET  0x1FF  //50%   Tx, 50%   Rx

#define REGS_MAX_ARRAY          4096




/******************************************************************************
 For interrupts, normal running is:
 rxdma_xfr_done, phy_interrupt, mac_stat_interrupt, 
 watchdog_interrupt & txdma_xfer_done
 
 In both cases, when flow control is enabled for either Tx or bi-direction,
 we additional enable rx_fbr0_low and rx_fbr1_low, so we know when the 
 buffer rings are running low.
 *****************************************************************************/
#define INT_MASK_DISABLE            0xffffffff

// NOTE: Masking out MAC_STAT Interrupt for now...
//#define INT_MASK_ENABLE             0xfff6bf17
//#define INT_MASK_ENABLE_NO_FLOW     0xfff6bfd7
#define INT_MASK_ENABLE             0xfffebf17
#define INT_MASK_ENABLE_NO_FLOW     0xfffebfd7



/******************************************************************************
 There are three ways of counting errors - if there are more than X errors
 in Y packets (represented by the "SAMPLE" macros), if there are more than
 N errors in a S mSec time period (the "PERIOD" macros), or if there are
 consecutive packets with errors (CONSEC_ERRORED_THRESH).  This last covers
 for "Bursty" errors, and the errored packets may well not be contiguous,
 but several errors where the packet counter has changed by less than a
 small amount will cause this count to increment.
 *****************************************************************************/
#define TX_PACKETS_IN_SAMPLE        10000
#define TX_MAX_ERRORS_IN_SAMPLE     50

#define TX_ERROR_PERIOD             1000
#define TX_MAX_ERRORS_IN_PERIOD     10

#define LINK_DETECTION_TIMER        5000

#define TX_CONSEC_RANGE             5
#define TX_CONSEC_ERRORED_THRESH    10

#define LO_MARK_PERCENT_FOR_PSR     15
#define LO_MARK_PERCENT_FOR_RX      15




/******************************************************************************
 Macros for flag and ref count operations       
 *****************************************************************************/
#define MP_SET_FLAG(_M, _F)         ((_M)->Flags |= (_F))   
#define MP_CLEAR_FLAG(_M, _F)       ((_M)->Flags &= ~(_F))
#define MP_CLEAR_FLAGS(_M)          ((_M)->Flags = 0)
#define MP_TEST_FLAG(_M, _F)        (((_M)->Flags & (_F)) != 0)
#define MP_TEST_FLAGS(_M, _F)       (((_M)->Flags & (_F)) == (_F))
#define MP_IS_FLAG_CLEAR(_M, _F)    (((_M)->Flags & (_F)) == 0)

#define MP_INC_RCV_REF(_A)          atomic_inc(&(_A)->RcvRefCount)
#define MP_DEC_RCV_REF(_A)          atomic_dec(&(_A)->RcvRefCount)
#define MP_GET_RCV_REF(_A)          atomic_read(&(_A)->RcvRefCount)




/******************************************************************************
 Macros specific to the private adapter structure
 *****************************************************************************/
#define MP_TCB_RESOURCES_AVAILABLE(_M) ((_M)->TxRing.nBusySend < NUM_TCB)
#define MP_TCB_RESOURCES_NOT_AVAILABLE(_M) ((_M)->TxRing.nBusySend >= NUM_TCB)

#define MP_SHOULD_FAIL_SEND(_M)   ((_M)->Flags & fMP_ADAPTER_FAIL_SEND_MASK) 
#define MP_IS_NOT_READY(_M)       ((_M)->Flags & fMP_ADAPTER_NOT_READY_MASK)
#define MP_IS_READY(_M)           !((_M)->Flags & fMP_ADAPTER_NOT_READY_MASK)

#define MP_HAS_CABLE(_M)           !((_M)->Flags & fMP_ADAPTER_NO_CABLE)
#define MP_LINK_DETECTED(_M)       !((_M)->Flags & fMP_ADAPTER_LINK_DETECTION)




/******************************************************************************
 RFD (Receive Frame Descriptor)
 *****************************************************************************/
typedef struct _MP_RFD {
		struct list_head        list_node;
		//struct sk_buff          *Packet;
		UINT32                  PacketSize;         // total size of receive frame
		UINT16                  iBufferIndex;
		UINT8                   iRingIndex;
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
		BOOL_t                  bHasVLANTag;
		UINT16                  VLANTag;
#endif
} MP_RFD, *PMP_RFD;




/******************************************************************************
 Enum for Flow Control
 *****************************************************************************/
typedef enum _eflow_control_t {
    Both   = 0,
    TxOnly = 1,
    RxOnly = 2,
    None   = 3
} eFLOW_CONTROL_t, *PeFLOW_CONTROL_t;



/******************************************************************************
 Struct to define some device statistics
 *****************************************************************************/
typedef struct _ce_stats_t {
		
		/**************************************************************************
		 MIB II variables
		 *************************************************************************/
		UINT32    unircv;      // # multicast packets received
		UINT32    multircv;    // # multicast packets received
		UINT32    brdcstrcv;   // # broadcast packets received
		
		
		/**************************************************************************
		 Transciever state informations.
		 *************************************************************************/
		UINT32    xcvr_addr;
		UINT32    xcvr_id;
		
		
		INT_STATUS_t InterruptStatus;
} CE_STATS_t, *PCE_STATS_t;




/******************************************************************************
 The private adapter structure
 *****************************************************************************/
typedef struct et131x_adapter {
		struct net_device       *netdev;
		struct pci_dev          *pdev;

	PMP_RFD					RecvLookaside;
		
		/**************************************************************************
		 Flags that indicate current state of the adapter
		 *************************************************************************/
		UINT32                  Flags;
		
		
		/**************************************************************************
		 Configuration 
		 *************************************************************************/
		UCHAR                   PermanentAddress[ETH_ALEN];
		UCHAR                   CurrentAddress[ETH_ALEN];
		BOOL_t                  bEepromPresent;
		UCHAR                   eepromData[2];
		
		
		/**************************************************************************
		 Spinlocks
		 *************************************************************************/
		spinlock_t              Lock;
		
		spinlock_t              TCBSendQLock;
		spinlock_t              TCBReadyQLock;
		spinlock_t              SendHWLock;
		
		spinlock_t              RcvLock;
		spinlock_t              FbrLock;
		
		spinlock_t              PHYLock;
		
		
		/**************************************************************************
		 Packet Filter and look ahead size
		 *************************************************************************/
		UINT32                  PacketFilter;
		UINT32                  ulLookAhead;
		UINT32                  uiLinkSpeed;
		UINT32                  uiDuplexMode;
		UINT32                  uiAutoNegStatus;
		UCHAR                   ucLinkStatus;
		
		/**************************************************************************
		 multicast list
		 *************************************************************************/
		UINT32                  MCAddressCount;
		UCHAR                   MCList[NIC_MAX_MCAST_LIST][ETH_ALEN];
		
		/**************************************************************************
		 MAC test
		 *************************************************************************/
		TXMAC_TXTEST_t          TxMacTest;
		
		
		/**************************************************************************
		 Pointer to the device's PCI register space
		 *************************************************************************/
		ADDRESS_MAP_t          *CSRAddress;
		
		
		/**************************************************************************
		 PCI config space info, for debug purposes only.
		 *************************************************************************/
		UCHAR                   RevisionID;
		UINT16                  VendorID;
		UINT16                  DeviceID;
		UINT16                  SubVendorID;
		UINT16                  SubSystemID;
		UINT16                  PciXDevCtl;
		
		/**************************************************************************
		 Registry parameters
		 *************************************************************************/
		UCHAR                   SpeedDuplex;            // speed/duplex
		eFLOW_CONTROL_t         RegistryFlowControl;    // for 802.3x flow control
		UCHAR                   RegistryWOLMatch;       // Enable WOL pattern-matching
		UCHAR                   RegistryWOLLink;        // Link state change is independant
		UCHAR                   RegistryPhyComa;        // Phy Coma mode enable/disable
		
		UINT32                  RegistryRxMemEnd;       // Size of internal rx memory
		UCHAR                   RegistryMACStat;        // If set, read MACSTAT, else don't
		UINT32                  RegistryVlanTag;        // 802.1q Vlan TAG
		UINT32                  RegistryJumboPacket;    // Max supported ethernet packet size
		
		UINT32                  RegistryTxNumBuffers;
		UINT32                  RegistryTxTimeInterval;
		
		UINT32                  RegistryRxNumBuffers;
		UINT32                  RegistryRxTimeInterval;
		
		
		/**************************************************************************
		 Validation helpers
		 *************************************************************************/
		UCHAR                   RegistryPMWOL;
		UCHAR                   RegistryNMIDisable;
		UINT32                  RegistryDMACache;
		UCHAR                   RegistryPhyLoopbk;      // Enable Phy loopback
		
		/**************************************************************************
		 Derived from the registry:
		 *************************************************************************/
		UCHAR                   AiForceDpx;         // duplex setting
		UINT16                  AiForceSpeed;       // 'Speed', user over-ride of line speed
		eFLOW_CONTROL_t         FlowControl;        // flow control validated by the far-end
		NETIF_STATUS            MediaState;
		
		/**************************************************************************
		 Minimize init-time
		 *************************************************************************/
		MP_POWER_MGMT           PoMgmt;
		
		
		/**************************************************************************
		 Xcvr status at last poll
		 *************************************************************************/
		MI_BMSR_t               Bmsr;
		
		
		/**************************************************************************
		 Tx Memory Variables
		 *************************************************************************/
		TX_RING_t               TxRing;
		
		
		/**************************************************************************
		 Rx Memory Variables
		 *************************************************************************/
		RX_RING_t               RxRing;
		
		
		/**************************************************************************
		 Loopback specifics
		 *************************************************************************/
		UCHAR                   ReplicaPhyLoopbk;       // Replica Enable 
		UCHAR                   ReplicaPhyLoopbkPF;     // Replica Enable Pass/Fail 
		
		
		/**************************************************************************
		 Stats
		 *************************************************************************/
		CE_STATS_t              Stats;
		
		/**************************************************************************
		 VLAN
		 *************************************************************************/
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
		struct vlan_group      *vlgrp;
#endif
} ET131X_ADAPTER, *PET131X_ADAPTER;



#endif
