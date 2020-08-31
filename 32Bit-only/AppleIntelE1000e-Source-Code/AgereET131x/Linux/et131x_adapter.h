/*******************************************************************************
 * Agere Systems Inc.
 * 10/100/1000 Base-T Ethernet Driver for the ET1301 and ET131x series MACs
 *
 * Copyright © 2005 Agere Systems Inc. 
 * All rights reserved.
 *   http://www.agere.com
 *
 *------------------------------------------------------------------------------
 *
 * et131x_adapter.h - Header which includes the private adapter structure, along
 *                    with related support structures, macros, definitions, etc.
 *
 *------------------------------------------------------------------------------
 *
 * SOFTWARE LICENSE
 *
 * This software is provided subject to the following terms and conditions,
 * which you should read carefully before using the software.  Using this
 * software indicates your acceptance of these terms and conditions.  If you do
 * not agree with these terms and conditions, do not use the software.
 *
 * Copyright © 2005 Agere Systems Inc.
 * All rights reserved.
 *
 * Redistribution and use in source or binary forms, with or without
 * modifications, are permitted provided that the following conditions are met:
 *
 * . Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following Disclaimer as comments in the code as
 *    well as in the documentation and/or other materials provided with the
 *    distribution.
 * 
 * . Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following Disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * . Neither the name of Agere Systems Inc. nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Disclaimer
 *
 * THIS SOFTWARE IS PROVIDED ìAS ISî AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, INFRINGEMENT AND THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  ANY
 * USE, MODIFICATION OR DISTRIBUTION OF THIS SOFTWARE IS SOLELY AT THE USERS OWN
 * RISK. IN NO EVENT SHALL AGERE SYSTEMS INC. OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, INCLUDING, BUT NOT LIMITED TO, CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *   
 ******************************************************************************/




/******************************************************************************
 *  VERSION CONTROL INFORMATION
 ******************************************************************************

      $RCSFile: $
         $Date: 2006/01/25 20:48:56 $
     $Revision: 1.15 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef __ET131X_ADAPTER_H__
#define __ET131X_ADAPTER_H__




#include "ET1310_address_map.h"
#include "ET1310_pm.h"
#include "ET1310_tx.h"
#include "ET1310_rx.h"
#include "ET1310_phy.h"
#include "et131x_supp.h"
#include "ET1310_jagcore.h"







/******************************************************************************
   These values are all superseded by registry entries to facilitate tuning.
   Once the desired performance has been achieved, the optimal registry values
   should be re-populated to these #defines:
 *****************************************************************************/
#define NUM_TRAFFIC_CLASSES          1




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
   Counters for error rate monitoring
 *****************************************************************************/
typedef struct _MP_ERR_COUNTERS
{
    UINT32                  PktCountTxPackets;
    UINT32                  PktCountTxErrors;
    UINT32                  TimerBasedTxErrors;
    UINT32                  PktCountLastError;
    UINT32                  ErredConsecPackets;
} MP_ERR_COUNTERS, *PMP_ERR_COUNTERS;




/******************************************************************************
   RFD (Receive Frame Descriptor)
 *****************************************************************************/
typedef struct _MP_RFD
{
    struct list_head        list_node;
#ifndef	__APPLE__
    struct sk_buff          *Packet;
#endif
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
typedef struct _ce_stats_t 
{
    /**************************************************************************
       Link Input/Output stats
     *************************************************************************/
    UINT64    ipackets;    // # of in packets
    UINT64    opackets;    // # of out packets


    /**************************************************************************
       MIB II variables
     *************************************************************************/
    /**************************************************************************
       NOTE - atomic_t types are only guaranteed to store 24-bits; if we MUST
              have 32, then we'll need another way to perform atomic operations
     *************************************************************************/
    UINT32    unircv;      // # multicast packets received
    atomic_t  unixmt;      // # multicast packets for Tx
    UINT32    multircv;    // # multicast packets received
    atomic_t  multixmt;    // # multicast packets for Tx
    UINT32    brdcstrcv;   // # broadcast packets received
    atomic_t  brdcstxmt;   // # broadcast packets for Tx
    UINT32    norcvbuf;    // # Rx packets discarded
    UINT32    noxmtbuf;    // # Tx packets discarded


    /**************************************************************************
       Transciever state informations.
     *************************************************************************/
    UINT32    xcvr_addr;
    UINT32    xcvr_id;


    /**************************************************************************
       Tx Statistics.
     *************************************************************************/
    UINT32    tx_uflo;                   //Tx Underruns

    UINT32    collisions;
    UINT32    excessive_collisions;
    UINT32    first_collision;
    UINT32    late_collisions;
    UINT32    max_pkt_error;
    UINT32    tx_deferred;
    

    /**************************************************************************
       Rx Statistics.
     *************************************************************************/
    UINT32    rx_ov_flow;     //Rx Over Flow

    UINT32    length_err;
    UINT32    alignment_err;
    UINT32    crc_err;
    UINT32    code_violations;
    UINT32    other_errors;

#if ET131X_DBG
    UINT32    UnhandledInterruptsPerSec;
    UINT32    RxDmaInterruptsPerSec;
    UINT32    TxDmaInterruptsPerSec;
    UINT32    WatchDogInterruptsPerSec;
#endif  /* ET131X_DBG */

    UINT32    SynchrounousIterations;
    INT_STATUS_t InterruptStatus;
}
CE_STATS_t, *PCE_STATS_t;




/******************************************************************************
   The private adapter structure
 *****************************************************************************/
typedef struct et131x_adapter
{
    struct net_device       *netdev;
    struct pci_dev          *pdev;
#ifdef	__APPLE__
	PMP_RFD					RecvLookaside;
#endif

    struct work_struct      task;


    /**************************************************************************
       Flags that indicate current state of the adapter
     *************************************************************************/
    UINT32                  Flags;
    UINT32                  HwErrCount;


    /**************************************************************************
       Configuration 
     *************************************************************************/
    UCHAR                   PermanentAddress[ETH_ALEN];
    UCHAR                   CurrentAddress[ETH_ALEN];
    BOOL_t                  bOverrideAddress;
    BOOL_t                  bEepromPresent;
    UCHAR                   eepromData[2];


    /**************************************************************************
       Spinlocks
     *************************************************************************/
    spinlock_t              Lock;

    spinlock_t              TCBSendQLock;
    spinlock_t              TCBReadyQLock;
    spinlock_t              SendHWLock;
    spinlock_t              SendWaitLock;

    spinlock_t              RcvLock;
    spinlock_t              RcvPendLock;
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
    UINT32                  CacheFillSize;
    UINT16                  PciXDevCtl;
#ifndef	__APPLE__
	UCHAR   		        pci_lat_timer;
	UCHAR			        pci_hdr_type;
	UCHAR			        pci_bist;
	UINT32			        pci_cfg_state[64 / sizeof(u32)];
#endif

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
    UINT32                  RegistrySCGain;
    UCHAR                   RegistryPhyLoopbk;      // Enable Phy loopback


    /**************************************************************************
       Derived from the registry:
     *************************************************************************/
    UCHAR                   AiForceDpx;         // duplex setting
    UINT16                  AiForceSpeed;       // 'Speed', user over-ride of line speed
    eFLOW_CONTROL_t         FlowControl;        // flow control validated by the far-end
    NETIF_STATUS            MediaState;
#ifndef	__APPLE__
    UCHAR                   DriverNoPhyAccess;
#endif

    /**************************************************************************
       Minimize init-time
     *************************************************************************/
#ifndef	__APPLE__
    BOOL_t                  bQueryPending;
    BOOL_t                  bSetPending;
    BOOL_t                  bResetPending;
    struct timer_list       ErrorTimer;
    BOOL_t                  bLinkTimerActive;
    INT_MASK_t              CachedMaskValue;
    atomic_t                RcvRefCount;    // Num packets not yet returned
#endif


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
       ET1310 register Access 
     *************************************************************************/
	JAGCORE_ACCESS_REGS     JagCoreRegs;
	PCI_CFG_SPACE_REGS		PciCfgRegs;


    /**************************************************************************
       Loopback specifics
     *************************************************************************/
    UCHAR                   ReplicaPhyLoopbk;       // Replica Enable 
    UCHAR                   ReplicaPhyLoopbkPF;     // Replica Enable Pass/Fail 


    /**************************************************************************
       Stats
     *************************************************************************/
    CE_STATS_t              Stats;

    struct net_device_stats	net_stats;
	struct net_device_stats	net_stats_prev;


    /**************************************************************************
       VLAN
     *************************************************************************/
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
    struct vlan_group      *vlgrp;
#endif

    /**************************************************************************
       Data to support workaround for bad config space addresses; see
       et131x_pci_setup() for more information
     *************************************************************************/
    BOOL_t                  pci_bar_workaround;
    unsigned long           pci_bar_addr_orig;
} ET131X_ADAPTER, *PET131X_ADAPTER;


#ifndef	__APPLE__

#define MPSendPacketsHandler  MPSendPackets
#define  MP_FREE_SEND_PACKET_FUN(Adapter, pMpTcb)  et131x_free_send_packet(Adapter, pMpTcb)
#define  MpSendPacketFun(Adapter,Packet) MpSendPacket(Adapter, Packet) 


#endif

#endif  /* __ET131X_ADAPTER_H__ */
