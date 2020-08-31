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
 * ET1310_rx.c - Routines used to perform data reception
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
         $Date: 2006/01/20 21:29:44 $
     $Revision: 1.21 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




/******************************************************************************
   Includes
 *****************************************************************************/
#include "et131x_version.h"
#include "et131x_debug.h"
#include "et131x_defs.h"


#include "ET1310_phy.h"
#include "ET1310_pm.h"
#include "ET1310_jagcore.h"

#include "et131x_supp.h"
#include "et131x_adapter.h"
#include "et131x_initpci.h"

#include "ET1310_rx.h"




/******************************************************************************
   Data for debugging facilities
 *****************************************************************************/
#if ET131X_DBG
extern dbg_info_t *et131x_dbginfo;
#endif  /* ET131X_DBG */




/******************************************************************************
   Prototypes for functions local to this module
 *****************************************************************************/
void nic_return_rfd( ET131X_ADAPTER *pAdapter, PMP_RFD pMpRfd );




/******************************************************************************
   ROUTINE :  et131x_rx_dma_memory_alloc
 ******************************************************************************

   DESCRIPTION       : Allocates Free buffer ring 1 for sure, free buffer ring
                       0 if required, and the Packet Status Ring
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_rx_dma_memory_alloc( ET131X_ADAPTER *adapter )
{
    UINT32          OuterLoop, InnerLoop;
    UINT32          bufsize;
    UINT32          pktStatRingSize, FBRChunkSize;
    RX_RING_t      *rx_ring;
    /*-----------------------------------------------------------------------*/

    
    DBG_FUNC( "et131x_rx_dma_memory_alloc" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Setup some convenience pointers
     *************************************************************************/
    rx_ring = (RX_RING_t *)&adapter->RxRing;


    /**************************************************************************
       Alloc memory for the lookup table
     *************************************************************************/
#ifdef USE_FBR0
    rx_ring->Fbr[0] = kmalloc( sizeof( FBRLOOKUPTABLE ), GFP_KERNEL );
#endif

    rx_ring->Fbr[1] = kmalloc( sizeof( FBRLOOKUPTABLE ), GFP_KERNEL );


    /**************************************************************************
        The first thing we will do is configure the sizes of the buffer rings.
        These will change based on jumbo packet support.  Larger jumbo packets
        increases the size of each entry in FBR0, and the number of entries in
        FBR0, while at the same time decreasing the number of entries in FBR1.

        FBR1 holds "large" frames, FBR0 holds "small" frames.  If FBR1 entries
        are huge in order to accomodate a "jumbo" frame, then it will have less
        entries.  Conversely, FBR1 will now be relied upon to carry more 
        "normal" frames, thus it's entry size also increases and the number
        of entries goes up too (since it now carries "small" + "regular"
        packets.

        In this scheme, we try to maintain 512 entries between the two rings.
        Also, FBR1 remains a constant size - when it's size doubles the
        number of entries halves.  FBR0 increases in size, however.
     *************************************************************************/

    if( adapter->RegistryJumboPacket < 2048 )
    {
#ifdef USE_FBR0
        rx_ring->Fbr0BufferSize = 256;
        rx_ring->Fbr0NumEntries = 512;
#endif
        rx_ring->Fbr1BufferSize = 2048;
        rx_ring->Fbr1NumEntries = 512;
    }
    else if( adapter->RegistryJumboPacket < 4096 )
    {
#ifdef USE_FBR0
        rx_ring->Fbr0BufferSize = 512;
        rx_ring->Fbr0NumEntries = 1024;
#endif
        rx_ring->Fbr1BufferSize = 4096;
        rx_ring->Fbr1NumEntries = 512;
    }
    else
    {
#ifdef USE_FBR0
        rx_ring->Fbr0BufferSize = 1024;
        rx_ring->Fbr0NumEntries = 768;
#endif
        rx_ring->Fbr1BufferSize = 16384;
        rx_ring->Fbr1NumEntries = 128;
    }

#ifdef USE_FBR0
    adapter->RxRing.PsrNumEntries = adapter->RxRing.Fbr0NumEntries +
                                    adapter->RxRing.Fbr1NumEntries;
#else
    adapter->RxRing.PsrNumEntries = adapter->RxRing.Fbr1NumEntries;
#endif


    /**************************************************************************
       Allocate an area of memory for Free Buffer Ring 1
     *************************************************************************/
    bufsize = ( sizeof( FBR_DESC_t ) * rx_ring->Fbr1NumEntries) + 0xfff;
    rx_ring->pFbr1RingVa = pci_alloc_consistent( adapter->pdev,
                                                 bufsize,
                                                 &rx_ring->pFbr1RingPa );
    if( !rx_ring->pFbr1RingVa )
    {
        DBG_ERROR( et131x_dbginfo, "Cannot alloc memory for Free Buffer Ring 1\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -ENOMEM;
    }
                 

    /**************************************************************************
       Save physical address

       NOTE : pci_alloc_consistent(), used above to alloc DMA regions, ALWAYS
              returns SAC (32-bit) addresses. If DAC (64-bit) addresses are
              ever returned, make sure the high part is retrieved here before
              storing the adjusted address.
     *************************************************************************/
    rx_ring->Fbr1Realpa = rx_ring->pFbr1RingPa;


    /**************************************************************************
       Align Free Buffer Ring 1 on a 4K boundary
     *************************************************************************/
    et131x_align_allocated_memory( adapter,
                                   &rx_ring->Fbr1Realpa,
                                   &rx_ring->Fbr1offset,
                                   0x0FFF );

    rx_ring->pFbr1RingVa = (void *)( (PUCHAR)rx_ring->pFbr1RingVa +
                                             rx_ring->Fbr1offset );


#ifdef USE_FBR0
    /**************************************************************************
       Allocate an area of memory for Free Buffer Ring 0
     *************************************************************************/
    bufsize = ( sizeof( FBR_DESC_t ) * rx_ring->Fbr0NumEntries ) + 0xfff;
    rx_ring->pFbr0RingVa = pci_alloc_consistent( adapter->pdev,
                                                 bufsize,
                                                 &rx_ring->pFbr0RingPa );
    if( !rx_ring->pFbr0RingVa )
    {
        DBG_ERROR( et131x_dbginfo, "Cannot alloc memory for Free Buffer Ring 0\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -ENOMEM;
    }


    /**************************************************************************
       Save physical address

       NOTE : pci_alloc_consistent(), used above to alloc DMA regions, ALWAYS
              returns SAC (32-bit) addresses. If DAC (64-bit) addresses are
              ever returned, make sure the high part is retrieved here before
              storing the adjusted address.
     *************************************************************************/
    rx_ring->Fbr0Realpa =  rx_ring->pFbr0RingPa;


    /**************************************************************************
       Align Free Buffer Ring 0 on a 4K boundary
     *************************************************************************/
    et131x_align_allocated_memory( adapter,
                                   &rx_ring->Fbr0Realpa,
                                   &rx_ring->Fbr0offset,
                                   0x0FFF );

    rx_ring->pFbr0RingVa = (void *)( (PUCHAR)rx_ring->pFbr0RingVa +
                                             rx_ring->Fbr0offset );

#endif
    

    for( OuterLoop = 0; OuterLoop < (rx_ring->Fbr1NumEntries / FBR_CHUNKS); OuterLoop++ )
    {
        UINT64  Fbr1Offset;
        UINT64  Fbr1TempPa;
        UINT32  Fbr1Align;

        /**********************************************************************
           This code allocates an area of memory big enough for N free
           buffers + (buffer_size - 1) so that the buffers can be aligned 
           on 4k boundaries.  If each buffer were aligned
           to a buffer_size boundary, the effect would be to double the size
           of FBR0.  By allocating N buffers at once, we reduce this overhead.
         *********************************************************************/
        if( rx_ring->Fbr1BufferSize > 4096 )
        {
            Fbr1Align = 4096;
        }
        else
        {
            Fbr1Align = rx_ring->Fbr1BufferSize;
        }

        FBRChunkSize = ( FBR_CHUNKS * rx_ring->Fbr1BufferSize ) + Fbr1Align - 1;
        rx_ring->Fbr1MemVa[OuterLoop] = pci_alloc_consistent( adapter->pdev,
                                                              FBRChunkSize,
                                                              &rx_ring->Fbr1MemPa[OuterLoop] );

        if( !rx_ring->Fbr1MemVa[OuterLoop] )
        {
            DBG_ERROR( et131x_dbginfo, "Could not alloc memory\n" );
            DBG_LEAVE( et131x_dbginfo );
            return -ENOMEM;
        }


        /**********************************************************************
           See NOTE in "Save Physical Address" comment above
         *********************************************************************/
        Fbr1TempPa = rx_ring->Fbr1MemPa[OuterLoop];

        et131x_align_allocated_memory( adapter,
                                       &Fbr1TempPa,
                                       &Fbr1Offset,
                                       (Fbr1Align - 1));


        for( InnerLoop = 0; InnerLoop < FBR_CHUNKS; InnerLoop++ )
        {
            UINT32 index = (OuterLoop * FBR_CHUNKS) + InnerLoop;


            /******************************************************************
                Save the Virtual address of this index for quick access later
             *****************************************************************/
            rx_ring->Fbr[1]->Va[index] = (PUCHAR)rx_ring->Fbr1MemVa[OuterLoop] + 
                                        ( InnerLoop * rx_ring->Fbr1BufferSize ) +
                                        Fbr1Offset;


            /******************************************************************
                now store the physical address in the descriptor so the device
                can access it
             *****************************************************************/
            rx_ring->Fbr[1]->PAHigh[index] = (UINT32)(Fbr1TempPa >> 32);
            rx_ring->Fbr[1]->PALow[index]  = (UINT32) Fbr1TempPa;
            
            Fbr1TempPa += rx_ring->Fbr1BufferSize;

            rx_ring->Fbr[1]->Buffer1[index] = rx_ring->Fbr[1]->Va[index];
            rx_ring->Fbr[1]->Buffer2[index] = rx_ring->Fbr[1]->Va[index] - 4;
        }
    }

#ifdef USE_FBR0
    /**************************************************************************
        Same for FBR0 (if in use)
     *************************************************************************/
    for( OuterLoop = 0; OuterLoop < (rx_ring->Fbr0NumEntries / FBR_CHUNKS); OuterLoop++ )
    {
        UINT64  Fbr0Offset;
        UINT64  Fbr0TempPa;

        FBRChunkSize = (( FBR_CHUNKS + 1 ) * rx_ring->Fbr0BufferSize ) - 1;
        rx_ring->Fbr0MemVa[OuterLoop] = pci_alloc_consistent( adapter->pdev,
                                                              FBRChunkSize,
                                                              &rx_ring->Fbr0MemPa[OuterLoop] );

        if( !rx_ring->Fbr0MemVa[OuterLoop] )
        {
            DBG_ERROR( et131x_dbginfo, "Could not alloc memory\n" );
            DBG_LEAVE( et131x_dbginfo );
            return -ENOMEM;
        }


        /**********************************************************************
           See NOTE in "Save Physical Address" comment above
         *********************************************************************/
        Fbr0TempPa = rx_ring->Fbr0MemPa[OuterLoop];
        
        et131x_align_allocated_memory( adapter,
                                       &Fbr0TempPa,
                                       &Fbr0Offset,
                                       rx_ring->Fbr0BufferSize - 1 );


        for( InnerLoop = 0; InnerLoop < FBR_CHUNKS; InnerLoop++ )
        {
            UINT32 index = (OuterLoop * FBR_CHUNKS) + InnerLoop;

            rx_ring->Fbr[0]->Va[index] = (PUCHAR)rx_ring->Fbr0MemVa[OuterLoop] +
                                        ( InnerLoop * rx_ring->Fbr0BufferSize ) +
                                        Fbr0Offset;

            rx_ring->Fbr[0]->PAHigh[index] = (UINT32)(Fbr0TempPa >> 32);                              
            rx_ring->Fbr[0]->PALow[index]  = (UINT32) Fbr0TempPa;

            Fbr0TempPa += rx_ring->Fbr0BufferSize;

            rx_ring->Fbr[0]->Buffer1[index] = rx_ring->Fbr[0]->Va[index];
            rx_ring->Fbr[0]->Buffer2[index] = rx_ring->Fbr[0]->Va[index] - 4;
        }
    }
#endif

    /**************************************************************************
       Allocate an area of memory for the FIFO of Packet Status ring entries
     *************************************************************************/
    pktStatRingSize = sizeof( PKT_STAT_DESC_t ) * adapter->RxRing.PsrNumEntries;
   
    rx_ring->pPSRingVa = pci_alloc_consistent( adapter->pdev,
                                               pktStatRingSize + 0x0fff,
                                               &rx_ring->pPSRingPa );

    if( !rx_ring->pPSRingVa )
    {
        DBG_ERROR( et131x_dbginfo, "Cannot alloc memory for Packet Status Ring\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -ENOMEM;
    }


    /**************************************************************************
       Save physical address

       NOTE : pci_alloc_consistent(), used above to alloc DMA regions, ALWAYS
              returns SAC (32-bit) addresses. If DAC (64-bit) addresses are
              ever returned, make sure the high part is retrieved here before
              storing the adjusted address.
     *************************************************************************/
    rx_ring->pPSRingRealPa = rx_ring->pPSRingPa;


    /**************************************************************************
       Align Packet Status Ring on a 4K boundary
     *************************************************************************/
    et131x_align_allocated_memory( adapter,
                                   &rx_ring->pPSRingRealPa,
                                   &rx_ring->pPSRingOffset,
                                   0x0FFF );

    rx_ring->pPSRingVa = (void *)( (PUCHAR)rx_ring->pPSRingVa +
                                           rx_ring->pPSRingOffset );


    /**************************************************************************
       Allocate an area of memory for the writeback of status information
     *************************************************************************/
    rx_ring->pRxStatusVa = pci_alloc_consistent( adapter->pdev,
                                                 sizeof( RX_STATUS_BLOCK_t ) + 0x7,
                                                 &rx_ring->pRxStatusPa );
    if( !rx_ring->pRxStatusVa )
    {
        DBG_ERROR( et131x_dbginfo, "Cannot alloc memory for Status Block\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -ENOMEM;
    }
                                                 

    /**************************************************************************
       Save physical address
     *************************************************************************/
    rx_ring->RxStatusRealPA = rx_ring->pRxStatusPa;


    /**************************************************************************
       Align write back on an 8 byte boundary
     *************************************************************************/
    et131x_align_allocated_memory( adapter,
                                   &rx_ring->RxStatusRealPA,
                                   &rx_ring->RxStatusOffset,
                                   0x07 );

    rx_ring->pRxStatusVa = (void *)( (PUCHAR)rx_ring->pRxStatusVa + 
                                             rx_ring->RxStatusOffset );
    rx_ring->NumRfd = NIC_DEFAULT_NUM_RFD;


    /**************************************************************************
        Recv
        pci_pool_create initializes a lookaside list. 
        After successful creation, nonpaged fixed-size blocks can be
        allocated from and freed to the lookaside list.

        RFDs will be allocated from this pool. 
     *************************************************************************/
    rx_ring->RecvLookaside = kmem_cache_create( adapter->netdev->name,
                                                sizeof( MP_RFD ),
                                                0,
                                                SLAB_CACHE_DMA |
                                                SLAB_HWCACHE_ALIGN, 
                                                NULL,
                                                NULL );


    MP_SET_FLAG( adapter, fMP_ADAPTER_RECV_LOOKASIDE );


    /**************************************************************************
        The RFDs are going to be put on lists later on, so initialize the lists
        now.
     *************************************************************************/
    INIT_LIST_HEAD( &rx_ring->RecvList );
    INIT_LIST_HEAD( &rx_ring->RecvPendingList );


    DBG_LEAVE( et131x_dbginfo );
    return 0;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_rx_dma_memory_free
 ******************************************************************************

   DESCRIPTION       : Should basically free all memory allocated within this
                       module.
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_rx_dma_memory_free( ET131X_ADAPTER *adapter )
{
    UINT32     index;
    UINT32     bufsize;
    UINT32     pktStatRingSize;
    PMP_RFD    pMpRfd;
    RX_RING_t *rx_ring;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_rx_dma_memory_free" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Setup some convenience pointers
     *************************************************************************/
    rx_ring = (RX_RING_t *)&adapter->RxRing;


    /**************************************************************************
       Free RFDs and associated packet descriptors
     *************************************************************************/
    DBG_ASSERT( rx_ring->nReadyRecv == rx_ring->NumRfd );

    while( !list_empty( &rx_ring->RecvList ))
    {
        pMpRfd = (MP_RFD *)list_entry( rx_ring->RecvList.next,
                                       MP_RFD,
                                       list_node );

        list_del( &pMpRfd->list_node );
        et131x_rfd_resources_free( adapter, pMpRfd );
    }

    while( !list_empty( &rx_ring->RecvPendingList ))
    {
        pMpRfd = (MP_RFD *)list_entry( rx_ring->RecvPendingList.next,
                                       MP_RFD,
                                       list_node );
        list_del( &pMpRfd->list_node );
        et131x_rfd_resources_free( adapter, pMpRfd );
    }


    /**************************************************************************
        Free Free Buffer Ring 1
     *************************************************************************/
    if( rx_ring->pFbr1RingVa )
    {
        /**********************************************************************
            First the packet memory
         *********************************************************************/
        for( index = 0; index < 
                      (rx_ring->Fbr1NumEntries / FBR_CHUNKS); index++ )
        {
            if( rx_ring->Fbr1MemVa[index] )
            {
                UINT32 Fbr1Align;

                if( rx_ring->Fbr1BufferSize > 4096 )
                {
                    Fbr1Align = 4096;
                }
                else
                {
                    Fbr1Align = rx_ring->Fbr1BufferSize;
                }

                bufsize = ( rx_ring->Fbr1BufferSize * FBR_CHUNKS ) +
                            Fbr1Align - 1;

                pci_free_consistent( adapter->pdev,
                                     bufsize,
                                     rx_ring->Fbr1MemVa[index],
                                     rx_ring->Fbr1MemPa[index] );
                                     
                rx_ring->Fbr1MemVa[index] = 0;
            }
        }


        /**********************************************************************
            Now the FIFO itself
         *********************************************************************/
        rx_ring->pFbr1RingVa = (void *)( (PUCHAR)rx_ring->pFbr1RingVa -
                                                 rx_ring->Fbr1offset );

        bufsize = ( sizeof( FBR_DESC_t ) * rx_ring->Fbr1NumEntries ) + 0xfff;

        pci_free_consistent( adapter->pdev,
                             bufsize,
                             rx_ring->pFbr1RingVa,
                             rx_ring->pFbr1RingPa );

        rx_ring->pFbr1RingVa = NULL;
    }


#ifdef USE_FBR0
    /**********************************************************************
        Now the same for Free Buffer Ring 0
     *********************************************************************/
    if( rx_ring->pFbr0RingVa )
    {
        /**********************************************************************
            First the packet memory
         *********************************************************************/
        for( index = 0; index <
                      (rx_ring->Fbr0NumEntries / FBR_CHUNKS); index++ )
        {
            if( rx_ring->Fbr0MemVa[index] )
            {
                bufsize = ( rx_ring->Fbr0BufferSize * ( FBR_CHUNKS + 1 )) - 1;

                pci_free_consistent( adapter->pdev,
                                     bufsize,
                                     rx_ring->Fbr0MemVa[index],
                                     rx_ring->Fbr0MemPa[index] );
                                     
                rx_ring->Fbr0MemVa[index] = 0;
            }
        }


        /**********************************************************************
            Now the FIFO itself
         *********************************************************************/
        rx_ring->pFbr0RingVa = (void *)( (PUCHAR)rx_ring->pFbr0RingVa -
                                                 rx_ring->Fbr0offset );

        bufsize = ( sizeof( FBR_DESC_t ) * rx_ring->Fbr0NumEntries ) + 0xfff;

        pci_free_consistent( adapter->pdev,
                             bufsize,
                             rx_ring->pFbr0RingVa,
                             rx_ring->pFbr0RingPa );

        rx_ring->pFbr0RingVa = NULL;
    }
#endif


    /**************************************************************************
        Free Packet Status Ring
     *************************************************************************/
    if( rx_ring->pPSRingVa )
    {
        rx_ring->pPSRingVa = (void *)( (PUCHAR)rx_ring->pPSRingVa -
                                               rx_ring->pPSRingOffset );

        pktStatRingSize = sizeof( PKT_STAT_DESC_t ) * adapter->RxRing.PsrNumEntries;
       
        pci_free_consistent( adapter->pdev,
                             pktStatRingSize + 0x0fff,
                             rx_ring->pPSRingVa,
                             rx_ring->pPSRingPa );
                             
        rx_ring->pPSRingVa = NULL;
    }


    /**********************************************************************
        Free area of memory for the writeback of status information
     *********************************************************************/
    if( rx_ring->pRxStatusVa )
    {
        rx_ring->pRxStatusVa = (void *)( (PUCHAR)rx_ring->pRxStatusVa -
                                                 rx_ring->RxStatusOffset );

        pci_free_consistent( adapter->pdev,
                             sizeof( RX_STATUS_BLOCK_t ) + 0x7,
                             rx_ring->pRxStatusVa,
                             rx_ring->pRxStatusPa );

        rx_ring->pRxStatusVa = NULL;
    }


    /**************************************************************************
        Free receive buffer pool
     *************************************************************************/


    /**************************************************************************
       Free receive packet pool
     *************************************************************************/


    /**************************************************************************
       Destroy the lookaside (RFD) pool
     *************************************************************************/
    if( MP_TEST_FLAG( adapter, fMP_ADAPTER_RECV_LOOKASIDE ))
    {
        kmem_cache_destroy( rx_ring->RecvLookaside );
        MP_CLEAR_FLAG( adapter, fMP_ADAPTER_RECV_LOOKASIDE );
    }


    /**************************************************************************
       Free the FBR Lookup Table
     *************************************************************************/
#ifdef USE_FBR0
    kfree( rx_ring->Fbr[0] );
#endif

    kfree( rx_ring->Fbr[1] );


    /**************************************************************************
       Reset Counters
     *************************************************************************/
    rx_ring->nReadyRecv = 0;


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_init_recv
 ******************************************************************************

   DESCRIPTION       : Initialize receive data structures.
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_init_recv( ET131X_ADAPTER *adapter )
{
    int               status        = -ENOMEM;
    PMP_RFD           pMpRfd        = NULL;      
    UINT32            RfdCount;
    UINT32            TotalNumRfd   = 0;
    RX_RING_t        *rx_ring       = NULL;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_init_recv" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Setup some convenience pointers
     *************************************************************************/
    rx_ring = (RX_RING_t *)&adapter->RxRing;


    /**************************************************************************
       Setup each RFD
     *************************************************************************/
    for( RfdCount = 0; RfdCount < rx_ring->NumRfd; RfdCount++ )
    {
        pMpRfd = ( MP_RFD * )kmem_cache_alloc( rx_ring->RecvLookaside,
                                               GFP_ATOMIC | GFP_DMA );

        if( !pMpRfd )
        {
            DBG_ERROR( et131x_dbginfo, "Couldn't alloc RFD out of kmem_cache\n" );
            
            status = -ENOMEM;
            continue;
        }
        
        status = et131x_rfd_resources_alloc( adapter,pMpRfd );
        if( status != 0 )
        {
            DBG_ERROR( et131x_dbginfo, "Couldn't alloc packet for RFD\n" );
            kmem_cache_free( rx_ring->RecvLookaside, pMpRfd );
            continue;
        }


        /**********************************************************************
           Add this RFD to the RecvList
         *********************************************************************/
        list_add_tail( &pMpRfd->list_node, &rx_ring->RecvList );


        /**********************************************************************
           Increment both the available RFD's, and the total RFD's.
         *********************************************************************/
        rx_ring->nReadyRecv++;
        TotalNumRfd++;                      
    }

    if( TotalNumRfd > NIC_MIN_NUM_RFD )
    {
        status = 0;
    }

    rx_ring->NumRfd = TotalNumRfd;

    if( status != 0 )
    {
        kmem_cache_free( rx_ring->RecvLookaside, pMpRfd );
        DBG_ERROR( et131x_dbginfo, "Allocation problems in et131x_init_recv\n" );
    }


    DBG_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_rfd_resources_alloc
 ******************************************************************************

   DESCRIPTION       :
        
   PARAMETERS        : adapter - pointer to our private adapter structure
                       pMpRfd  - pointer to a RFD
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_rfd_resources_alloc( ET131X_ADAPTER *adapter, MP_RFD *pMpRfd )
{
    pMpRfd->Packet = NULL;

    return 0;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_rfd_resources_free
 ******************************************************************************

   DESCRIPTION       : Free the packet allocated for the given RFD
        
   PARAMETERS        : adapter - pointer to our private adapter structure
                       pMpRfd  - pointer to a RFD

   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_rfd_resources_free( ET131X_ADAPTER *adapter, MP_RFD *pMpRfd )
{
    pMpRfd->Packet = NULL;
    kmem_cache_free( adapter->RxRing.RecvLookaside, pMpRfd );
    
    return;
}
/*===========================================================================*/





/******************************************************************************
   ROUTINE:  ConfigRxDmaRegs
 ******************************************************************************
   DESCRIPTION:
        START OF Rx_DMA INIT SEQUENCE

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :


 *****************************************************************************/
void ConfigRxDmaRegs( ET131X_ADAPTER *pAdapter )
{
    PRXDMA_t      pRxDma;
    PFBR_DESC_t   pFbrEntry;
    UINT32        iEntry;
    unsigned long lockflags;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "ConfigRxDmaRegs" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Let's get our pointer to the RxDma section of regs
     *************************************************************************/
    pRxDma = &pAdapter->CSRAddress->rxdma;


    /**************************************************************************
       Halt RXDMA to perform the reconfigure. 
     *************************************************************************/
    et131x_rx_dma_disable( pAdapter );


    /**************************************************************************
       Load the completion writeback physical address

       NOTE : pci_alloc_consistent(), used above to alloc DMA regions, ALWAYS
              returns SAC (32-bit) addresses. If DAC (64-bit) addresses are
              ever returned, make sure the high part is retrieved here before
              storing the adjusted address.
     *************************************************************************/
    pRxDma->dma_wb_base_hi.addr_hi = (UINT32)( pAdapter->RxRing.RxStatusRealPA >> 32 );
    pRxDma->dma_wb_base_lo.addr_lo = (UINT32)( pAdapter->RxRing.RxStatusRealPA );

    memset( pAdapter->RxRing.pRxStatusVa, 0, sizeof( RX_STATUS_BLOCK_t ));


    /**************************************************************************
       Set the address and parameters of the packet status ring into the 1310's
       registers
     *************************************************************************/
    pRxDma->psr_base_hi.addr_hi   = (UINT32)( pAdapter->RxRing.pPSRingRealPa >> 32 );
    pRxDma->psr_base_lo.addr_lo   = (UINT32)( pAdapter->RxRing.pPSRingRealPa );
  
    pRxDma->psr_num_des.value     = pAdapter->RxRing.PsrNumEntries - 1;

    pRxDma->psr_full_offset.value = 0;

    pRxDma->psr_min_des.value     = 
         ( pRxDma->psr_num_des.bits.psr_ndes * LO_MARK_PERCENT_FOR_PSR ) / 100;

    spin_lock_irqsave( &pAdapter->RcvLock, lockflags );


    /**************************************************************************
       These local variables track the PSR in the adapter structure
     *************************************************************************/
    pAdapter->RxRing.local_psr_full.bits.psr_full      = 0;
    pAdapter->RxRing.local_psr_full.bits.psr_full_wrap = 0;


    /**************************************************************************
       Now's the best time to initialize FBR1 contents
     *************************************************************************/
    pFbrEntry = (PFBR_DESC_t) pAdapter->RxRing.pFbr1RingVa;

    for (iEntry=0; iEntry < pAdapter->RxRing.Fbr1NumEntries; iEntry++)
    {
        pFbrEntry->addr_hi = pAdapter->RxRing.Fbr[1]->PAHigh[ iEntry ];
        pFbrEntry->addr_lo = pAdapter->RxRing.Fbr[1]->PALow [ iEntry ];
        pFbrEntry->word2.bits.bi = iEntry;
        pFbrEntry++;
    }


    /**************************************************************************
       Set the address and parameters of Free buffer ring 1 (and 0 if required) 
       into the 1310's registers
     *************************************************************************/
    pRxDma->fbr1_base_hi.addr_hi = (UINT32)( pAdapter->RxRing.Fbr1Realpa >> 32 );
    pRxDma->fbr1_base_lo.addr_lo = (UINT32)( pAdapter->RxRing.Fbr1Realpa );

    pRxDma->fbr1_num_des.value = pAdapter->RxRing.Fbr1NumEntries - 1;

    {
        RXDMA_FBR_FULL_OFFSET_t fbr1_full;

        fbr1_full.bits.fbr_full        = 0;
        fbr1_full.bits.fbr_full_wrap   = 1;

        pRxDma->fbr1_full_offset = fbr1_full;
    }


    /**************************************************************************
       This variable tracks the free buffer ring 1 full position, so it has to
       match the above.
     *************************************************************************/
    pAdapter->RxRing.local_Fbr1_full.bits.fbr_full      = 0;
    pAdapter->RxRing.local_Fbr1_full.bits.fbr_full_wrap = 1;

    pRxDma->fbr1_min_des.bits.fbr_min = 
      (( pAdapter->RxRing.Fbr1NumEntries * LO_MARK_PERCENT_FOR_RX ) / 100) - 1;


#ifdef USE_FBR0
    /**************************************************************************
       Now's the best time to initialize FBR0 contents
     *************************************************************************/
    pFbrEntry = (PFBR_DESC_t) pAdapter->RxRing.pFbr0RingVa;

    for (iEntry=0; iEntry < pAdapter->RxRing.Fbr0NumEntries; iEntry++)
    {
        pFbrEntry->addr_hi = pAdapter->RxRing.Fbr[0]->PAHigh[iEntry];
        pFbrEntry->addr_lo = pAdapter->RxRing.Fbr[0]->PALow [iEntry];
        pFbrEntry->word2.bits.bi = iEntry;
        pFbrEntry++;
    }

    pRxDma->fbr0_base_hi.addr_hi = (UINT32)( pAdapter->RxRing.Fbr0Realpa >> 32 );
    pRxDma->fbr0_base_lo.addr_lo = (UINT32)( pAdapter->RxRing.Fbr0Realpa );
    
    pRxDma->fbr0_num_des.bits.fbr_ndesc = pAdapter->RxRing.Fbr0NumEntries - 1;

    {
        RXDMA_FBR_FULL_OFFSET_t fbr0_full;

        fbr0_full.bits.fbr_full        = 0;
        fbr0_full.bits.fbr_full_wrap   = 1;

        pRxDma->fbr0_full_offset = fbr0_full;
    }


    /**************************************************************************
       This variable tracks the free buffer ring 0 full position, so it has to
       match the above.
     *************************************************************************/
    pAdapter->RxRing.local_Fbr0_full.bits.fbr_full      = 0;
    pAdapter->RxRing.local_Fbr0_full.bits.fbr_full_wrap = 1;

    pRxDma->fbr0_min_des.bits.fbr_min   = 
      (( pAdapter->RxRing.Fbr0NumEntries * LO_MARK_PERCENT_FOR_RX ) / 100) - 1;
#endif


    /**************************************************************************
       Program the number of packets we will receive before generating an 
       interrupt.

      For version B silicon, this value gets updated once autoneg is complete.
     *************************************************************************/
    pRxDma->num_pkt_done.value  = pAdapter->RegistryRxNumBuffers;


    /**************************************************************************
       The "time_done" is not working correctly to coalesce interrupts after
       a given time period, but rather is giving us an interrupt regardless
       of whether we have received packets.

      This value gets updated once autoneg is complete.
     *************************************************************************/
    pRxDma->max_pkt_time.value = pAdapter->RegistryRxTimeInterval;

    spin_unlock_irqrestore( &pAdapter->RcvLock, lockflags );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  SetRxDmaTimer
 ******************************************************************************
   DESCRIPTION:
        SET the heartbeat timer according to line rate.

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        N/A

 *****************************************************************************/
void SetRxDmaTimer( ET131X_ADAPTER *pAdapter )
{
    /**************************************************************************
       For version B silicon, we do not use the RxDMA timer for 10 and 100
       Mbits/s line rates.  We do not enable and RxDMA interrupt coalescing.
     *************************************************************************/
    if(( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_100MBPS ) ||
       ( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_10MBPS ))
    {
        pAdapter->CSRAddress->rxdma.max_pkt_time.value = 0;
        pAdapter->CSRAddress->rxdma.num_pkt_done.value = 1;
    }
    
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  et131x_rx_dma_disable
 ******************************************************************************
   DESCRIPTION:
        Stop OF Rx_DMA on the ET1310

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        

 *****************************************************************************/
void et131x_rx_dma_disable( ET131X_ADAPTER *pAdapter )
{
    DBG_FUNC( "et131x_rx_dma_disable" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Setup the receive dma configuration register
     *************************************************************************/
    pAdapter->CSRAddress->rxdma.csr.value = 0x00002001;

    if( pAdapter->CSRAddress->rxdma.csr.bits.halt_status != 1 )
    {
        udelay( 5 );
        if( pAdapter->CSRAddress->rxdma.csr.bits.halt_status != 1 )
        {
            DBG_ERROR( et131x_dbginfo,
                       "RX Dma failed to enter halt state. CSR 0x%08x\n",
                       pAdapter->CSRAddress->rxdma.csr.value );
        }
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  et131x_rx_dma_enable
 ******************************************************************************
   DESCRIPTION:
        re-start OF Rx_DMA on the ET1310.

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        

 *****************************************************************************/
void et131x_rx_dma_enable( ET131X_ADAPTER *pAdapter )
{
    DBG_FUNC( "et131x_rx_dma_enable" );
    DBG_RX_ENTER( et131x_dbginfo );


    if( pAdapter->RegistryPhyLoopbk )
    {
        /**********************************************************************
            RxDMA is disabled for loopback operation.
         *********************************************************************/
        pAdapter->CSRAddress->rxdma.csr.value = 0x1;
    }
    else
    {
        /**********************************************************************
            Setup the receive dma configuration register for normal operation.
         *********************************************************************/
        {
            RXDMA_CSR_t csr = {0};

            csr.bits.fbr1_enable = 1;

            if( pAdapter->RxRing.Fbr1BufferSize == 4096 )
            {
                csr.bits.fbr1_size = 1;
            }
            else if( pAdapter->RxRing.Fbr1BufferSize == 8192 )
            {
                csr.bits.fbr1_size = 2;
            }
            else if( pAdapter->RxRing.Fbr1BufferSize == 16384 )
            {
                csr.bits.fbr1_size = 3;
            }
#ifdef USE_FBR0
            csr.bits.fbr0_enable = 1;

            if( pAdapter->RxRing.Fbr0BufferSize == 256 )
            {
                csr.bits.fbr0_size = 1;
            }
            else if( pAdapter->RxRing.Fbr0BufferSize == 512 )
            {
                csr.bits.fbr0_size = 2;
            }
            else if( pAdapter->RxRing.Fbr0BufferSize == 1024 )
            {
                csr.bits.fbr0_size = 3;
            }
#endif
            pAdapter->CSRAddress->rxdma.csr = csr;
        }

        if( pAdapter->CSRAddress->rxdma.csr.bits.halt_status != 0 )
        {
            udelay( 5 );
            if( pAdapter->CSRAddress->rxdma.csr.bits.halt_status != 0 )
            {
                DBG_ERROR( et131x_dbginfo,
                           "RX Dma failed to exit halt state.  CSR 0x%08x\n",
                           pAdapter->CSRAddress->rxdma.csr.value );
            }
        }
    }


    DBG_RX_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  nic_rx_pkts
 ******************************************************************************
   DESCRIPTION:
        Checks the hardware for available packets, using completion ring
        If packets are available, it gets an RFD from the RecvList, attaches
        the packet to it, puts the RFD in the RecvPendList, and also returns
        the pointer to the RFD.
    
    PARAMETERS :
        pAdapter - pointer to our adapter

   RETURNS    :
        pMpRfd   - pointer to our MPRFD

 *****************************************************************************/
PMP_RFD nic_rx_pkts( ET131X_ADAPTER *pAdapter )
{
    PRX_STATUS_BLOCK_t      pRxStatusBlock;
    PPKT_STAT_DESC_t        pPSREntry;
    PMP_RFD                 pMpRfd;
    UINT32                  nIndex;
    PUCHAR                  pBufVa;
    PUINT16                 pShBufVa;
    unsigned long           lockflags;
    struct list_head       *element;


    /**************************************************************************
       RX Status block is written by the DMA engine prior to every interrupt.
       It contains the next to be used entry in the Packet Status Ring, and
       also the two Free Buffer rings.
     *************************************************************************/
    pRxStatusBlock = (PRX_STATUS_BLOCK_t)pAdapter->RxRing.pRxStatusVa;

    if(( pRxStatusBlock->Word1.bits.PSRoffset != 
                             pAdapter->RxRing.local_psr_full.bits.psr_full ) ||
       ( pRxStatusBlock->Word1.bits.PSRwrap != 
                          pAdapter->RxRing.local_psr_full.bits.psr_full_wrap ))
    {
        UINT8                   ringIndex;
        UINT16                  bufferIndex;
        UINT32                  localLen;
        PKT_STAT_DESC_WORD0_t   Word0;


        /**********************************************************************
           The packet status ring indicates that data is available. 
         *********************************************************************/
        pPSREntry = (PPKT_STAT_DESC_t)( pAdapter->RxRing.pPSRingVa ) + 
                                 pAdapter->RxRing.local_psr_full.bits.psr_full;


        /**********************************************************************
           Grab any information that is required once the PSR is advanced,
           since we can no longer rely on the memory being accurate
         *********************************************************************/
        localLen    = pPSREntry->word1.bits.length;
        ringIndex   = (UINT8)pPSREntry->word1.bits.ri;
        bufferIndex = (UINT16)pPSREntry->word1.bits.bi;
        Word0       = pPSREntry->word0;

        /**********************************************************************
           Indicate that we have used this PSR entry.
         *********************************************************************/
        if( ++pAdapter->RxRing.local_psr_full.bits.psr_full >
                             ( pAdapter->RxRing.PsrNumEntries - 1 ))
        {
            pAdapter->RxRing.local_psr_full.bits.psr_full       = 0;
            pAdapter->RxRing.local_psr_full.bits.psr_full_wrap ^= 1;
        }

        pAdapter->CSRAddress->rxdma.psr_full_offset =
                                               pAdapter->RxRing.local_psr_full;

#ifndef USE_FBR0
        if( ringIndex != 1 )
        {
            DBG_ERROR( et131x_dbginfo, 
                       "NICRxPkts PSR Entry %d indicates "
                       "Buffer Ring 0 in use\n",
                       pAdapter->RxRing.local_psr_full.bits.psr_full );

            return NULL;
        }
#endif

#ifdef USE_FBR0
        if(( ringIndex > 1 ) ||
          (( ringIndex == 0 ) && ( bufferIndex > ( pAdapter->RxRing.Fbr0NumEntries - 1 ))) ||
          (( ringIndex == 1 ) && ( bufferIndex > ( pAdapter->RxRing.Fbr1NumEntries - 1 ))))
#else
        if(( ringIndex != 1 ) || ( bufferIndex > ( pAdapter->RxRing.Fbr1NumEntries - 1 )))
#endif
        {
            /******************************************************************
               Illegal buffer or ring index cannot be used by the S/W
             *****************************************************************/
            DBG_ERROR( et131x_dbginfo, 
                       "NICRxPkts PSR Entry %d indicates "
                       "length of %d and/or bad bi(%d)\n",
                       pAdapter->RxRing.local_psr_full.bits.psr_full,
                       localLen,
                       bufferIndex );
            return NULL;
        }


        /**********************************************************************
           Get and fill the RFD.
         *********************************************************************/
		IOSimpleLockLock( pAdapter->RcvLock );

        pMpRfd = NULL;
        element = pAdapter->RxRing.RecvList.next;
        pMpRfd  = (PMP_RFD)list_entry( element, MP_RFD, list_node );

        if( pMpRfd == NULL )
        {
            DBG_RX( et131x_dbginfo,
                    "NULL RFD returned from RecvList via list_entry()\n" );
            DBG_RX_LEAVE( et131x_dbginfo );
            return NULL;
        }

        list_del( &pMpRfd->list_node );
        pAdapter->RxRing.nReadyRecv--;

		IOSimpleLockUnlock( pAdapter->RcvLock );

        pMpRfd->iBufferIndex = bufferIndex;
        pMpRfd->iRingIndex   = ringIndex;


        /**********************************************************************
           In V1 silicon, there is a bug which screws up filtering of runt
           packets.  Therefore runt packet filtering is disabled in the MAC
           and the packets are dropped here.  They are also counted here.
         *********************************************************************/
        if( localLen < ( NIC_MIN_PACKET_SIZE + 4 ))
        {
            pAdapter->Stats.other_errors++;
            localLen = 0;
        }


#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
        if( localLen )
        {
            pShBufVa = pAdapter->RxRing.Fbr[ringIndex]->Va[bufferIndex];

            pMpRfd->bHasVLANTag = FALSE;


            /******************************************************************
                The protocol value of 0x8100 means there is a VLAN tag in the
                packet.  Also the original protocol value will be present four
                bytes further on.
             *****************************************************************/
            if( pShBufVa[6] == 0x0081 )
            {
                UINT16 LocalShort = (( pShBufVa[7] & 0xff00 ) >> 8 ) +
                                    (( pShBufVa[7] & 0x00ff ) << 8 );
                UINT16 vlan_tag   = LocalShort & 0x0fff;

                pMpRfd->bHasVLANTag = TRUE;

                DBG_RX( et131x_dbginfo,
                        "VLAN: RX packet Tag: %d\n", vlan_tag );


                /**************************************************************
                    The rules are:
                        - if our VLAN tag is zero we can pass anything up
                        - if our VLAN tag matches the incoming we can pass it
                        - If the packet is a protocol 802.3ad we pass it 
                           regardless (802.3ad had protocol val of 0x8809.  
                           proto val is now in ShBuf [8])
                        - If the packet is a GARP VLAN Registration Protocol
                          (GVRP) packet, we pass it regardless.  
                          01:80:c2:00:00:21 is the GVRP address.

                    NOTE: Because the ET1310 doesn't perform VLAN tagging
                          (it's done in the kernel) always pass packets up.
                          We'll leave this code in, however, just in case it's
                          needed in the future.
                 *************************************************************/
                if(( 1 ) ||
                   (  pShBufVa[8] == 0x0988 ) ||
                   (( pShBufVa[0] == 0x8001 ) &&
                    ( pShBufVa[1] == 0x00c2 ) && 
                    ( pShBufVa[2] == 0x2100 )))
                {
                    DBG_RX( et131x_dbginfo,
                            "VLAN: Passed test, send pkt up\n" );
                    pMpRfd->VLANTag = vlan_tag;
                }
                else
                {
                    /*********************************************************
                        Our VLAN tag is non-zero, AND the incoming tag does
                        not match it.  Drop the packet.
                     ********************************************************/
                    DBG_RX( et131x_dbginfo,
                            "VLAN: no match, drop pkt\n" );
                    localLen = 0;
                }
            }
            else if((  pShBufVa[6] == 0x0988 ) ||
                    (( pShBufVa[0] == 0x8001 ) && 
                     ( pShBufVa[1] == 0x00c2 ) && 
                     ( pShBufVa[2] == 0x2100 )))
            {
                /******************************************************************
                    The protocol type (ethertype) of 0x8809 corresponds to 802.3ad 
                    The MAC address of 01:80:c2:00:00:21 is the GARP VLAN 
                    registration protocol (GVRP) address.

                    Both of these message types should be passed up regardless
                    of their VLAN tagging.
                *****************************************************************/
                DBG_RX( et131x_dbginfo,
                        "VLAN: No tag, but 802.3ad/GVRP, send pkt up\n" );
            }
            else
            {
                /******************************************************************
                    Our VLAN tag is non-zero.  no VLAN header on incoming.
                    Packet is not GVRP or 802.3ad.  Drop the packet.
                *****************************************************************/
                DBG_RX( et131x_dbginfo,
                        "VLAN: No RX packet tag\n" );
                // NOTE: Same as the note above; never drop a packet for now.
                // localLen = 0;
            }
        }
#endif

        if( localLen )
        {
            if ( pAdapter->ReplicaPhyLoopbk == 1 )
            {
                pBufVa = pAdapter->RxRing.Fbr[ringIndex]->Va[bufferIndex];
                
                if( memcmp( &pBufVa[6], &pAdapter->CurrentAddress[0], ETH_ALEN ) == 0 )
                {
                    if( memcmp( &pBufVa[42], "Replica packet", ETH_HLEN ))
                    {
                        pAdapter->ReplicaPhyLoopbkPF = 1;
                    }
                }
                DBG_WARNING( et131x_dbginfo, 
                             "pBufVa:\t%02x:%02x:%02x:%02x:%02x:%02x\n",
                             pBufVa[6],
                             pBufVa[7],
                             pBufVa[8],
                             pBufVa[9],
                             pBufVa[10],
                             pBufVa[11] );

                DBG_WARNING( et131x_dbginfo,
                             "CurrentAddr:\t%02x:%02x:%02x:%02x:%02x:%02x\n",
                             pAdapter->CurrentAddress[0],
                             pAdapter->CurrentAddress[1],
                             pAdapter->CurrentAddress[2],
                             pAdapter->CurrentAddress[3],
                             pAdapter->CurrentAddress[4],
                             pAdapter->CurrentAddress[5] );
            }

            /******************************************************************
               Determine if this is a multicast packet coming in
             *****************************************************************/
            if(( Word0.value & ALCATEL_MULTICAST_PKT ) &&
               !( Word0.value & ALCATEL_BROADCAST_PKT ))
            {
                /**************************************************************
                   Promiscuous mode and Multicast mode are not mutually
                   exclusive as was first thought.  I guess Promiscuous is
                   just considered a super-set of the other filters.
                   Generally filter is 0x2b when in promiscuous mode.
                 *************************************************************/
                if(( pAdapter->PacketFilter & ET131X_PACKET_TYPE_MULTICAST ) &&
                   !( pAdapter->PacketFilter & ET131X_PACKET_TYPE_PROMISCUOUS ) &&
                   !( pAdapter->PacketFilter & ET131X_PACKET_TYPE_ALL_MULTICAST ))
                {
                    pBufVa = pAdapter->RxRing.Fbr[ringIndex]->Va[bufferIndex];

                    /**********************************************************
                       Loop through our list to see if the destination address
                       of this packet matches one in our list.
                     *********************************************************/
                    for( nIndex = 0; nIndex < pAdapter->MCAddressCount; nIndex++ )
                    {
                        if( pBufVa[0] == pAdapter->MCList[nIndex][0] &&
                            pBufVa[1] == pAdapter->MCList[nIndex][1] &&
                            pBufVa[2] == pAdapter->MCList[nIndex][2] &&
                            pBufVa[3] == pAdapter->MCList[nIndex][3] &&
                            pBufVa[4] == pAdapter->MCList[nIndex][4] &&
                            pBufVa[5] == pAdapter->MCList[nIndex][5] )
                        {
                            break;
                        }
                    }
                    
                    /**********************************************************
                       If our index is equal to the number of Multicast
                       address we have, then this means we did not find this
                       packet's matching address in our list.  Set the
                       PacketSize to zero, so we free our RFD when we return
                       from this function.
                     *********************************************************/
                    if( nIndex == pAdapter->MCAddressCount )
                    {
                        localLen = 0;
                    }
                }

                if( localLen > 0 )
                {
                    pAdapter->Stats.multircv++;
                }
            }
            else if( Word0.value & ALCATEL_BROADCAST_PKT )
            {
                pAdapter->Stats.brdcstrcv++;
            }
            else
            {
                /**************************************************************
                   Not sure what this counter measures in promiscuous mode.
                   Perhaps we should check the MAC address to see if it is 
                   directed to us in promiscuous mode.
                 *************************************************************/
                pAdapter->Stats.unircv++;
            }
        }

        if( localLen > 0 )
        {
            struct sk_buff *skb = NULL;

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
            unsigned short  vlan_tag = 0;
#endif

            //vlan_tag = 0; 

            //pMpRfd->PacketSize = localLen - 4;
            pMpRfd->PacketSize = localLen;

            skb = dev_alloc_skb( pMpRfd->PacketSize + 2 );
            if( !skb )
            {
                DBG_ERROR( et131x_dbginfo, "Couldn't alloc an SKB for Rx\n" );
                DBG_RX_LEAVE( et131x_dbginfo );
                return NULL;
            }

            pAdapter->net_stats.rx_bytes += pMpRfd->PacketSize;

            memcpy( skb_put( skb, pMpRfd->PacketSize ), 
                    pAdapter->RxRing.Fbr[ringIndex]->Va[bufferIndex],
                    pMpRfd->PacketSize );

            skb->dev       = pAdapter->netdev;
            skb->protocol  = eth_type_trans( skb, pAdapter->netdev );
            skb->ip_summed = CHECKSUM_NONE;


#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#if defined __vlan_get_tag
            if( __vlan_get_tag( skb, &vlan_tag ) == -EINVAL )
            {
                DBG_RX( et131x_dbginfo,
                        "VLAN: No Rx packet tag\n" );
            }
            else
            {
                DBG_RX( et131x_dbginfo,
                        "VLAN: Rx packet tag: %d\n", vlan_tag );
            }
#endif
#endif

            netif_rx( skb );
        }
        else
        {
            pMpRfd->PacketSize   = 0;
        }

        nic_return_rfd( pAdapter, pMpRfd );


        DBG_RX( et131x_dbginfo, "(1)\n" );
        DBG_RX_LEAVE( et131x_dbginfo );
        return pMpRfd;
    }
    else
    {
        /**********************************************************************
           Looks like this ring is not updated yet
         *********************************************************************/
        DBG_RX( et131x_dbginfo, "(0)\n" );
        DBG_RX_LEAVE( et131x_dbginfo );
        return NULL;
    }
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  et131x_reset_recv
 ******************************************************************************
   DESCRIPTION:
        Reset the receive list                    

        Assumption: Rcv spinlock has been acquired 

   PARAMETERS :
        pAdapter - pointer to our adapter

   RETURNS    :
        NONE

 *****************************************************************************/
void et131x_reset_recv( ET131X_ADAPTER *pAdapter )
{
    PMP_RFD           pMpRfd;      
    struct list_head *element;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_reset_recv" );
    DBG_ENTER( et131x_dbginfo );


    DBG_ASSERT( !list_empty( &pAdapter->RxRing.RecvList ));


    /**************************************************************************
       Take all the RFD's from the pending list, and stick them on the
       RecvList.
     *************************************************************************/
    while( !list_empty( &pAdapter->RxRing.RecvPendingList ) )
    {
        element = pAdapter->RxRing.RecvPendingList.next;

        pMpRfd = (PMP_RFD)list_entry( element, MP_RFD, list_node );

        list_del( &pMpRfd->list_node );
        list_add_tail( &pMpRfd->list_node, &pAdapter->RxRing.RecvList );
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  et131x_handle_recv_interrupt
 ******************************************************************************
   DESCRIPTION:
        Interrupt handler for receive processing
    
        Assumption: Rcv spinlock has been acquired 
 
   PARAMETERS :
        pAdapter  - pointer to our adapter
    
   RETURNS    :
        NONE

 *****************************************************************************/
void et131x_handle_recv_interrupt( ET131X_ADAPTER *pAdapter )
{
    PMP_RFD         pMpRfd = NULL;
    struct sk_buff *PacketArray [NUM_PACKETS_HANDLED];              
    PMP_RFD         RFDFreeArray[NUM_PACKETS_HANDLED];
    UINT32          PacketArrayCount = 0;
    UINT32          PacketsToHandle;
    UINT32          PacketFreeCount  = 0;
    BOOL_t          TempUnfinishedRec = FALSE;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_handle_recv_interrupt" );
    DBG_RX_ENTER( et131x_dbginfo );
    
    
    PacketsToHandle = NUM_PACKETS_HANDLED;


    /**************************************************************************
       Process up to available RFD's
     *************************************************************************/
    while( PacketArrayCount < PacketsToHandle )
    {
        if( list_empty( &pAdapter->RxRing.RecvList ))
        {
            DBG_ASSERT( pAdapter->RxRing.nReadyRecv == 0 );
            DBG_ERROR( et131x_dbginfo, "NO RFD's !!!!!!!!!!!!!\n" );
            TempUnfinishedRec = TRUE;
            break;
        }

        pMpRfd = nic_rx_pkts( pAdapter );

        if( pMpRfd == NULL)
        {
            break;
        }


        /**********************************************************************
           Do not receive any packets until a filter has been set.
           Do not receive any packets until we are at D0.
           Do not receive any packets until we have link.
           If length is zero, return the RFD in order to advance the Free
           buffer ring.
         *********************************************************************/
        if(( !pAdapter->PacketFilter ) || 
           ( pAdapter->PoMgmt.PowerState != NdisDeviceStateD0 ) ||
           ( !MP_LINK_DETECTED( pAdapter )) ||
           ( pMpRfd->PacketSize == 0 ))
        {
            continue;
        }


        /**********************************************************************
           Increment the number of packets we received
         *********************************************************************/
        pAdapter->Stats.ipackets++;


        /**********************************************************************
           Set the status on the packet, either resources or success
         *********************************************************************/
        if( pAdapter->RxRing.nReadyRecv >= RFD_LOW_WATER_MARK )
        {
            /******************************************************************
                Put this RFD on the pending list

                NOTE - nic_rx_pkts() above is already returning the RFD to the
                RecvList, so don't additionally do that here.

                Besides, we don't really need (at this point) the pending list
                anyway.
             *****************************************************************/
            //spin_lock_irqsave( &pAdapter->RcvPendLock, lockflags );
            //list_add_tail( &pMpRfd->list_node, &pAdapter->RxRing.RecvPendingList );
            //spin_unlock_irqrestore( &pAdapter->RcvPendLock, lockflags );


            /******************************************************************
                Update the number of outstanding Recvs
             *****************************************************************/
            //MP_INC_RCV_REF( pAdapter );
        }
        else
        {   
            RFDFreeArray[PacketFreeCount] = pMpRfd;
            PacketFreeCount++;

            DBG_WARNING( et131x_dbginfo, "RFD's are running out !!!!!!!!!!!!!\n" );
        }

        PacketArray[PacketArrayCount] = pMpRfd->Packet;
        PacketArrayCount++;
    }


    if(( PacketArrayCount == NUM_PACKETS_HANDLED ) || TempUnfinishedRec )
    {
        pAdapter->RxRing.UnfinishedReceives = TRUE;
        pAdapter->CSRAddress->global.watchdog_timer = 
                            pAdapter->RegistryTxTimeInterval * NANO_IN_A_MICRO;
    }
    else
    {
        /**********************************************************************
           Watchdog timer will disable itself if appropriate.
         *********************************************************************/
        pAdapter->RxRing.UnfinishedReceives = FALSE;
    }


    DBG_RX_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  NICReturnRFD
 ******************************************************************************
   DESCRIPTION:
        Recycle a RFD and put it back onto the receive list 
    
   PARAMETERS :
        pAdapter - pointer to our adapter
        pMpRfd   - pointer to the RFD 

   RETURNS    :
        NONE

 *****************************************************************************/
void nic_return_rfd( ET131X_ADAPTER *pAdapter, PMP_RFD pMpRfd )
{
    UINT16        ReturnedBI = pMpRfd->iBufferIndex;
    UINT8         ReturnedRI = pMpRfd->iRingIndex;
    unsigned long lockflags; 
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "nic_return_rfd" );
    DBG_RX_ENTER( et131x_dbginfo );


    /**************************************************************************
       We don't use any of the OOB data besides status
       Otherwise, we need to clean up OOB data
     *************************************************************************/
        if(
#ifdef USE_FBR0
            (( ReturnedRI == 0 ) && ( ReturnedBI < pAdapter->RxRing.Fbr0NumEntries )) ||
#endif
            (( ReturnedRI == 1 ) && ( ReturnedBI < pAdapter->RxRing.Fbr1NumEntries )))
    {
        spin_lock_irqsave( &pAdapter->FbrLock, lockflags );

        if( ReturnedRI == 1 )
        {
            PFBR_DESC_t pNextDesc = (PFBR_DESC_t)(pAdapter->RxRing.pFbr1RingVa) +
                                    pAdapter->RxRing.local_Fbr1_full.bits.fbr_full;

            /******************************************************************
                Handle the Free Buffer Ring advancement here.  Write the 
                PA / Buffer Index for the returned buffer into the oldest
                (next to be freed)FBR entry
             *****************************************************************/
            pNextDesc->addr_hi = pAdapter->RxRing.Fbr[1]->PAHigh[ReturnedBI];
            pNextDesc->addr_lo = pAdapter->RxRing.Fbr[1]->PALow[ReturnedBI];
            pNextDesc->word2.value = ReturnedBI;

            if( ++pAdapter->RxRing.local_Fbr1_full.bits.fbr_full >
                                       ( pAdapter->RxRing.Fbr1NumEntries - 1 ))
            {
                pAdapter->RxRing.local_Fbr1_full.bits.fbr_full = 0;
                pAdapter->RxRing.local_Fbr1_full.bits.fbr_full_wrap ^= 1;
            }

            pAdapter->CSRAddress->rxdma.fbr1_full_offset =
                                                pAdapter->RxRing.local_Fbr1_full;
        }
#ifdef USE_FBR0
        else
        {
            PFBR_DESC_t pNextDesc = (PFBR_DESC_t)(pAdapter->RxRing.pFbr0RingVa) +
                                    pAdapter->RxRing.local_Fbr0_full.bits.fbr_full;

            /******************************************************************
                Handle the Free Buffer Ring advancement here.  Write the
                PA / Buffer Index for the returned buffer into the oldest
                (next to be freed) FBR entry
             *****************************************************************/
            pNextDesc->addr_hi = pAdapter->RxRing.Fbr[0]->PAHigh[ReturnedBI];
            pNextDesc->addr_lo = pAdapter->RxRing.Fbr[0]->PALow[ReturnedBI];
            pNextDesc->word2.value = ReturnedBI;

            if( ++pAdapter->RxRing.local_Fbr0_full.bits.fbr_full >
                                         (pAdapter->RxRing.Fbr0NumEntries - 1))
            {
                pAdapter->RxRing.local_Fbr0_full.bits.fbr_full = 0;
                pAdapter->RxRing.local_Fbr0_full.bits.fbr_full_wrap ^= 1;
            }

            pAdapter->CSRAddress->rxdma.fbr0_full_offset =
                                                pAdapter->RxRing.local_Fbr0_full;
        }
#endif
        spin_unlock_irqrestore( &pAdapter->FbrLock, lockflags );
    }
    else
    {
        DBG_ERROR( et131x_dbginfo,
                   "NICReturnRFD illegal Buffer Index returned\n" );
    }


    /**************************************************************************
        The processing on this RFD is done, so put it back on the tail of
        our list
     *************************************************************************/
    spin_lock_irqsave( &pAdapter->RcvLock, lockflags );

    list_add_tail( &pMpRfd->list_node, &pAdapter->RxRing.RecvList );
    pAdapter->RxRing.nReadyRecv++;

    spin_unlock_irqrestore( &pAdapter->RcvLock, lockflags );

    DBG_ASSERT( pAdapter->RxRing.nReadyRecv <= pAdapter->RxRing.NumRfd );

    return;
}
/*===========================================================================*/
