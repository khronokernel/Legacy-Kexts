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
 * et131x_debug.c - Routines used for debugging.
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
 * THIS SOFTWARE IS PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
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
         $Date: 2005/08/01 19:35:12 $
     $Revision: 1.6 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#if ET131X_DBG




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
#include "et131x_netdev.h"
#include "et131x_config.h"
#include "et131x_isr.h"

#include "ET1310_address_map.h"
#include "ET1310_jagcore.h"
#include "ET1310_tx.h"
#include "ET1310_rx.h"
#include "ET1310_mac.h"




/******************************************************************************
   Data for debugging facilities
 *****************************************************************************/
extern dbg_info_t *et131x_dbginfo;




/******************************************************************************
   ROUTINE:  DumpTxQueueContents
 ******************************************************************************
   DESCRIPTION:
        Used to dump out hte tx queue and the shadow pointers

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void DumpTxQueueContents( int dbgLvl, ET131X_ADAPTER *pAdapter )
{
    UINT32 TxQueueAddr;
    /*-----------------------------------------------------------------------*/


    if( DBG_FLAGS( et131x_dbginfo ) & dbgLvl )
    {

        for( TxQueueAddr = 0x200; TxQueueAddr< 0x3ff; TxQueueAddr++ )
        {
            pAdapter->CSRAddress->mmc.sram_access.bits.req_addr = TxQueueAddr;
            pAdapter->CSRAddress->mmc.sram_access.bits.req_access = 1;
            

            DBG_PRINT( "Addr 0x%x, Access 0x%08x\t"
                    "Value 1 0x%08x, Value 2 0x%08x, "
                    "Value 3 0x%08x, Value 4 0x%08x, \n",
                    TxQueueAddr, 
                    pAdapter->CSRAddress->mmc.sram_access.value,
                    pAdapter->CSRAddress->mmc.sram_word1.data, 
                    pAdapter->CSRAddress->mmc.sram_word2.data, 
                    pAdapter->CSRAddress->mmc.sram_word3.data, 
                    pAdapter->CSRAddress->mmc.sram_word4.data );

        }

        DBG_PRINT( "Shadow Pointers 0x%08x\n",
                pAdapter->CSRAddress->txmac.shadow_ptr.value );
    }
    
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  DumpDeviceBlock
 ******************************************************************************

   DESCRIPTION:
        Dumps the first 64 regs of each block of the et-1310 (each block is
        mapped to a new page, each page is 4096 bytes).

   PARAMETERS :
        pAdapter - pointer to our adapter

   RETURN     :
        VOID
 *****************************************************************************/
#define NUM_BLOCKS 8
void DumpDeviceBlock( int dbgLvl, ET131X_ADAPTER *pAdapter, UINT32 Block )
{
    UINT32 Address1, Address2;
    UINT32 *BigDevicePointer = (UINT32 *)pAdapter->CSRAddress;
    /*-----------------------------------------------------------------------*/


    char* BlockNames[NUM_BLOCKS] = 
            {"Global", "Tx DMA", "Rx DMA", "Tx MAC", 
            "Rx MAC", "MAC", "MAC Stat", "MMC" };
    /*-----------------------------------------------------------------------*/


    /**************************************************************************
       Output the debug counters to the debug terminal
     *************************************************************************/
    if( DBG_FLAGS( et131x_dbginfo ) & dbgLvl )
    {
        DBG_PRINT( "%s block\n", BlockNames[Block] );

        for( Address1 = 0; Address1 < 8; Address1++ )
        {
            for( Address2 = 0; Address2 < 8; Address2++ )
            {
                if( Block == 0 )
                {
                    if((( Address1 * 8 ) + Address2 ) == 6 )
                    {
                        DBG_PRINT( "  ISR    , " );
                    }
                    else
                    {
                        DBG_PRINT( "0x%08x, ", 
                                BigDevicePointer[( Block * 1024 ) + 
                                                    ( Address1 * 8 ) + Address2] );

                    }
                }
                else
                {
                    DBG_PRINT( "0x%08x, ", 
                            BigDevicePointer[( Block * 1024 ) + 
                                                ( Address1 * 8 ) + Address2] );
                }
            }

            DBG_PRINT( "\n" );
        }

        DBG_PRINT( "\n" );
    }

    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  DumpDeviceReg
 ******************************************************************************

   DESCRIPTION:
        Dumps the first 64 regs of each block of the et-1310 (each block is
        mapped to a new page, each page is 4096 bytes).

   PARAMETERS :
        pAdapter - pointer to our adapter

   RETURN     :
        VOID
 *****************************************************************************/
void DumpDeviceReg( int dbgLvl, ET131X_ADAPTER *pAdapter )
{
    UINT32  Address1, Address2;
    UINT32  Block;
    UINT32 *BigDevicePointer = (UINT32 *)pAdapter->CSRAddress;

    char* BlockNames[NUM_BLOCKS] = 
            {"Global", "Tx DMA", "Rx DMA", "Tx MAC", 
            "Rx MAC", "MAC", "MAC Stat", "MMC" };
    /*-----------------------------------------------------------------------*/


    /**************************************************************************
       Output the debug counters to the debug terminal
     *************************************************************************/
    if( DBG_FLAGS( et131x_dbginfo ) & dbgLvl )
    {
        for( Block = 0; Block < NUM_BLOCKS; Block++ )
        {
            DBG_PRINT( "%s block\n", BlockNames[Block] );

            for( Address1 = 0; Address1 < 8; Address1++ )
            {
                for( Address2 = 0; Address2 < 8; Address2++ )
                {
                    DBG_PRINT( "0x%08x, ", 
                            BigDevicePointer[( Block * 1024 ) + 
                            ( Address1 * 8 ) + Address2] );
                }

                DBG_PRINT( "\n" );
            }

            DBG_PRINT( "\n" );
        }
    }

    return;
}
/*==========================================================================*/




#endif // ET131X_DBG
