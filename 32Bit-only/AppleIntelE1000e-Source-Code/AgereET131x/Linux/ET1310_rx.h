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
 * ET1310_rx.h - Defines, structs, enums, prototypes, etc. pertaining to data
 *               reception.
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
         $Date: 2005/10/28 18:43:45 $
     $Revision: 1.10 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef __ET1310_RX_H__
#define __ET1310_RX_H__


#ifdef __cplusplus
extern "C" {
#endif




/******************************************************************************
   INCLUDES
 *****************************************************************************/
#include "ET1310_common.h"

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
typedef union _FBR_WORD2_t
{
    UINT32 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:22;                         //bits 10-31
        UINT32 bi:10;                               //bits 0-9(Buffer Index)
    #else
        UINT32 bi:10;                               //bits 0-9(Buffer Index)
        UINT32 reserved:22;                         //bit 10-31
    #endif
    } bits;
}
FBR_WORD2_t, *PFBR_WORD2_t;

typedef struct _FBR_DESC_t
{
    UINT32         addr_lo;
    UINT32         addr_hi;
    FBR_WORD2_t    word2;
}
FBR_DESC_t, *PFBR_DESC_t;
/*===========================================================================*/




/******************************************************************************
   Typedefs for Packet Status Ring Descriptors
 *****************************************************************************/
typedef union _PKT_STAT_DESC_WORD0_t
{
    UINT32 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
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
}
PKT_STAT_DESC_WORD0_t, *PPKT_STAT_WORD0_t;

typedef union _PKT_STAT_DESC_WORD1_t
{
    UINT32 value;
    struct
    {
    #ifdef _BIT_FIELDS_HTOL
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
}
PKT_STAT_DESC_WORD1_t, *PPKT_STAT_WORD1_t;

typedef struct _PKT_STAT_DESC_t
{
    PKT_STAT_DESC_WORD0_t   word0;
    PKT_STAT_DESC_WORD1_t   word1;
}
PKT_STAT_DESC_t, *PPKT_STAT_DESC_t;
/*===========================================================================*/




/******************************************************************************
   Typedefs for the RX DMA status word
 *****************************************************************************/
/******************************************************************************
   RXSTAT_WORD0_t structure holds part of the status bits of the Rx DMA engine
   that get copied out to memory by the ET-1310.  Word 0 is a 32 bit word which
   contains Free Buffer ring 0 and 1 available offset.
 *****************************************************************************/
typedef union _rxstat_word0_t 
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
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
}RXSTAT_WORD0_t, *PRXSTAT_WORD0_t;




/******************************************************************************
   RXSTAT_WORD1_t structure holds part of the status bits of the Rx DMA engine
   that get copied out to memory by the ET-1310.  Word 3 is a 32 bit word which
   contains the Packet Status Ring available offset.
 *****************************************************************************/
typedef union _rxstat_word1_t 
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
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
}RXSTAT_WORD1_t, *PRXSTAT_WORD1_t;




/******************************************************************************
   RX_STATUS_BLOCK_t is sructure representing the status of the Rx DMA engine
   it sits in free memory, and is pointed to by 0x101c / 0x1020
 *****************************************************************************/

typedef struct _rx_status_block_t 
{
    RXSTAT_WORD0_t    Word0;
    RXSTAT_WORD1_t    Word1;
} 
RX_STATUS_BLOCK_t, *PRX_STATUS_BLOCK_t;




/******************************************************************************
   Structure for look-up table holding free buffer ring pointers
 *****************************************************************************/
typedef struct _FbrLookupTable
{
    void                 *Va            [MAX_DESC_PER_RING_RX];
    void                 *Buffer1       [MAX_DESC_PER_RING_RX];
    void                 *Buffer2       [MAX_DESC_PER_RING_RX];
    UINT32                PAHigh        [MAX_DESC_PER_RING_RX];
    UINT32                PALow         [MAX_DESC_PER_RING_RX];
} FBRLOOKUPTABLE, *PFBRLOOKUPTABLE;

typedef enum {
    ONE_PACKET_INTERRUPT,
    FOUR_PACKET_INTERRUPT
} 
eRX_INTERRUPT_STATE_t, *PeRX_INTERRUPT_STATE_t;




/******************************************************************************
   Structure to hold the skb's in a list
 *****************************************************************************/
typedef struct rx_skb_list_elem
{
	struct list_head  skb_list_elem;
	dma_addr_t        dma_addr;
	struct sk_buff   *skb;
} RX_SKB_LIST_ELEM, *PRX_SKB_LIST_ELEM;




/******************************************************************************
   RX_RING_t is sructure representing the adaptor's local reference(s) to the
   rings
 *****************************************************************************/
typedef struct _rx_ring_t 
{
    PFBR_DESC_t					pFbr1RingVa;
    dma_addr_t                  pFbr1RingPa;
    void                       *Fbr1MemVa[ MAX_DESC_PER_RING_RX / FBR_CHUNKS ];
    dma_addr_t                  Fbr1MemPa[ MAX_DESC_PER_RING_RX / FBR_CHUNKS ];
    UINT64                      Fbr1Realpa;
    UINT64                      Fbr1offset;
#ifdef	__APPLE__
    FBRLOOKUPTABLE             Fbr1;
#else
    FBRLOOKUPTABLE             *Fbr[2];
#endif
    RXDMA_FBR_FULL_OFFSET_t     local_Fbr1_full;
    UINT32                      Fbr1NumEntries;
    UINT32                      Fbr1BufferSize;

    PPKT_STAT_DESC_t			pPSRingVa;
    dma_addr_t                  pPSRingPa;
    UINT64                      pPSRingRealPa;
#ifndef	__APPLE__
    UINT64                      pPSRingOffset;
#endif
    RXDMA_PSR_FULL_OFFSET_t     local_psr_full;
    UINT32                      PsrNumEntries;

    void                       *pRxStatusVa;
    dma_addr_t                  pRxStatusPa;
    UINT64                      RxStatusRealPA;
#ifndef	__APPLE__
    UINT64                      RxStatusOffset;
#endif

#ifndef	__APPLE__
    struct list_head            RecvBufferPool;
#endif

    /**************************************************************************
       RECV
     *************************************************************************/
    struct list_head            RecvList;
#ifndef	__APPLE__
    struct list_head            RecvPendingList;
#endif
    UINT32                      nReadyRecv;

    UINT32                      NumRfd;

    BOOL_t                      UnfinishedReceives;

    struct list_head            RecvPacketPool;


    /**************************************************************************
       lookaside lists
     *************************************************************************/
#ifndef	__APPLE__
    kmem_cache_t             *RecvLookaside;
#endif
}
RX_RING_t, *PRX_RING_t;



#ifndef	__APPLE__

/******************************************************************************
   Forward reference of RFD
 *****************************************************************************/
struct _MP_RFD;




/******************************************************************************
   Forward declaration of the private adapter structure
 *****************************************************************************/
struct et131x_adapter;




/******************************************************************************
   PROTOTYPES for Initialization
 *****************************************************************************/
int et131x_rx_dma_memory_alloc( struct et131x_adapter *adapter );
void et131x_rx_dma_memory_free( struct et131x_adapter *adapter );
int et131x_rfd_resources_alloc( struct et131x_adapter *adapter, struct _MP_RFD *pMpRfd );
void et131x_rfd_resources_free( struct et131x_adapter *adapter, struct _MP_RFD *pMpRfd );
int et131x_init_recv( struct et131x_adapter *adapter );

void ConfigRxDmaRegs( struct et131x_adapter *pAdapter );
void SetRxDmaTimer( struct et131x_adapter *pAdapter );
void et131x_rx_dma_disable( struct et131x_adapter *pAdapter );
void et131x_rx_dma_enable( struct et131x_adapter *pAdapter );

void et131x_reset_recv( struct et131x_adapter *pAdapter );

void et131x_handle_recv_interrupt( struct et131x_adapter *pAdapter );
#endif



#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ET1310_RX_H__ */
