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
 * ET1310_tx.h - Defines, structs, enums, prototypes, etc. pertaining to data
 *               transmission.
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
         $Date: 2005/10/07 21:26:45 $
     $Revision: 1.9 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef __ET1310_TX_H__
#define __ET1310_TX_H__

#ifdef __cplusplus
extern "C" {
#endif




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
        #ifdef _BIT_FIELDS_HTOL
            UINT32 vlan_prio:3;                     //bits 29-31(VLAN priority)
            UINT32 vlan_cfi:1;                      //bit 28(cfi)
            UINT32 vlan_tag:12;                     //bits 16-27(VLAN tag)
            UINT32 length_in_bytes:16;              //bits  0-15(packet length)
        #else
            UINT32 length_in_bytes:16;              //bits  0-15(packet length)
            UINT32 vlan_tag:12;                     //bits 16-27(VLAN tag)
            UINT32 vlan_cfi:1;                      //bit 28(cfi)
            UINT32 vlan_prio:3;                     //bits 29-31(VLAN priority)
        #endif /* _BIT_FIELDS_HTOL */
    } bits;
}TXDESC_WORD2_t, *PTXDESC_WORD2_t;




/******************************************************************************
   TXDESC_WORD3_t structure holds part of the control bits in the Tx Descriptor
   ring for the ET-1310 
 *****************************************************************************/
typedef union  _txdesc_word3_t 
{
    UINT32 value;
    struct 
    {
        #ifdef _BIT_FIELDS_HTOL
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
        #endif /* _BIT_FIELDS_HTOL */
    } bits;
}TXDESC_WORD3_t, *PTXDESC_WORD3_t;




/******************************************************************************
   TX_DESC_ENTRY_t is sructure representing each descriptor on the ring
 *****************************************************************************/
typedef struct _tx_desc_entry_t 
{
    UINT32         DataBufferPtrHigh;
    UINT32         DataBufferPtrLow;
    TXDESC_WORD2_t word2;                           // control words how to xmit the
    TXDESC_WORD3_t word3;                           // data (detailed above)
}
TX_DESC_ENTRY_t, *PTX_DESC_ENTRY_t;
/*===========================================================================*/




/******************************************************************************
   Typedefs for Tx DMA engine status writeback
 *****************************************************************************/
/******************************************************************************
   TX_STATUS_BLOCK_t is sructure representing the status of the Tx DMA engine
   it sits in free memory, and is pointed to by 0x101c / 0x1020
 *****************************************************************************/

typedef union _tx_status_block_t 
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
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
TX_STATUS_BLOCK_t, *PTX_STATUS_BLOCK_t;
/*===========================================================================*/




/******************************************************************************
   TCB (Transmit Control Block)
 *****************************************************************************/
typedef struct _MP_TCB
{
    struct _MP_TCB          *Next;
    UINT32                  Flags;
    UINT32                  Count;
    UINT32                  PacketStaleCount;
    struct sk_buff          *Packet;
#ifndef	__APPLE__
    UINT32                  PacketLength;
#endif
    TXDMA_SERVICE_REQUEST_t WrIndex;
    TXDMA_SERVICE_REQUEST_t WrIndexStart;
} MP_TCB, *PMP_TCB;




/******************************************************************************
   Structure to hold the skb's in a list
 *****************************************************************************/
typedef struct tx_skb_list_elem
{
	struct list_head  skb_list_elem;
	struct sk_buff   *skb;
} TX_SKB_LIST_ELEM, *PTX_SKB_LIST_ELEM;




/******************************************************************************
   TX_RING_t is sructure representing our local reference(s) to the ring
 *****************************************************************************/
typedef struct _tx_ring_t 
{
    /**************************************************************************
        TCB (Transmit Control Block) memory and lists
     *************************************************************************/
#ifdef	__APPLE__
	MP_TCB	MpTcbMem[NUM_TCB];
#else
    PUCHAR                   MpTcbMem;
#endif

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
        List of packets (not TCBs) that were queued for lack of resources
     *************************************************************************/
    struct list_head         SendWaitQueue;
    INT32                    nWaitSend;


    /**************************************************************************
        The actual descriptor ring
     *************************************************************************/
    PTX_DESC_ENTRY_t         pTxDescRingVa;
    dma_addr_t               pTxDescRingPa;
#ifndef	__APPLE__
    UINT64                   pTxDescRingAdjustedPa;
    UINT64                   TxDescOffset;
#endif

    /**************************************************************************
        ReadyToSend indicates where we last wrote to in the descriptor ring.
     *************************************************************************/
    TXDMA_SERVICE_REQUEST_t  txDmaReadyToSend;


    /**************************************************************************
        The location of the write-back status block
     *************************************************************************/
    PTX_STATUS_BLOCK_t       pTxStatusVa;
    dma_addr_t               pTxStatusPa;


    /**************************************************************************
        A Block of zeroes, used to pad packets that are less than 60 bytes.
     *************************************************************************/
    void                    *pTxDummyBlkVa;
    dma_addr_t               pTxDummyBlkPa;

    TXMAC_ERR_t              TxMacErr;


    /**************************************************************************
        Variables to track the Tx interrupt coalescing features
     *************************************************************************/
    INT32                    TxPacketsSinceLastinterrupt;
}
TX_RING_t, *PTX_RING_t;


#ifndef	__APPLE__


/******************************************************************************
   Forward declaration of the frag-list for the following prototypes
 *****************************************************************************/
typedef struct _MP_FRAG_LIST  MP_FRAG_LIST, *PMP_FRAG_LIST;




/******************************************************************************
   Forward declaration of the private adapter structure
 *****************************************************************************/
struct et131x_adapter;




/******************************************************************************
   PROTOTYPES for et1310_tx.c
 *****************************************************************************/
int et131x_tx_dma_memory_alloc( struct et131x_adapter *adapter );
void et131x_tx_dma_memory_free( struct et131x_adapter *adapter );
void ConfigTxDmaRegs( struct et131x_adapter *pAdapter );
void et131x_init_send( struct et131x_adapter *adapter );
void et131x_tx_dma_disable( struct et131x_adapter *pAdapter );
void et131x_tx_dma_enable( struct et131x_adapter *pAdapter );
void et131x_handle_send_interrupt( struct et131x_adapter *pAdapter );
void et131x_free_busy_send_packets( struct et131x_adapter *pAdapter );
int et131x_send_packets( struct sk_buff *skb, struct net_device *netdev );
#endif



#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ET1310_TX_H__ */
