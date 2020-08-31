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
 * ET1310_jagcore.c - All code pertaining to the ET1301/ET131x's JAGcore
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
         $Date: 2005/10/28 18:43:44 $
     $Revision: 1.9 $
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




/******************************************************************************
   Data for debugging facilities
 *****************************************************************************/
#if ET131X_DBG
extern dbg_info_t *et131x_dbginfo;
#endif  /* ET131X_DBG */




/******************************************************************************
   ROUTINE:  ConfigGlobalRegs
 ******************************************************************************

   DESCRIPTION:
        Used to configure the global registers on the JAGCore

   PARAMETERS :
        pAdpater - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void ConfigGlobalRegs( ET131X_ADAPTER *pAdapter )
{
    PGLOBAL_t pGbl;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "ConfigGlobalRegs" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
        Map a local pointer to the global section of the JAGCore
     *************************************************************************/
    pGbl = &pAdapter->CSRAddress->global;

    if( pAdapter->RegistryPhyLoopbk == FALSE )
    {
        if( pAdapter->RegistryJumboPacket < 2048 )
        {
            /******************************************************************
                Tx / RxDMA and Tx/Rx MAC interfaces have a 1k word block of RAM
                that the driver can split between Tx and Rx as it desires.  Our
                default is to split it 50/50:
             *****************************************************************/
            pGbl->rxq_start_addr.value = 0;
            pGbl->rxq_end_addr.value   = pAdapter->RegistryRxMemEnd;
            pGbl->txq_start_addr.value = pGbl->rxq_end_addr.bits.rxq_end_addr + 1;
            pGbl->txq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
        }
        else if( pAdapter->RegistryJumboPacket < 8192 )
        {
            /******************************************************************
                For jumbo packets > 2k in length, but < 8k, split 50-50.
             *****************************************************************/
            pGbl->rxq_start_addr.value = 0;
            pGbl->rxq_end_addr.value   = INTERNAL_MEM_RX_OFFSET;
            pGbl->txq_start_addr.value = INTERNAL_MEM_RX_OFFSET + 1;
            pGbl->txq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
        }
        else
        {
            /******************************************************************
                9216 is the only packet size greater than 8k that is available.
                The Tx buffer has to be big enough for one whole packet on the
                Tx side.  We'll make the Tx 9408, and give the rest to Rx
             *****************************************************************/
            pGbl->rxq_start_addr.value = 0x0000;
            pGbl->rxq_end_addr.value   = 0x01b3;
            pGbl->txq_start_addr.value = 0x01b4;
            pGbl->txq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
        }

        /**********************************************************************
            Initialize the loopback register.  Disable all loopbacks.
         *********************************************************************/
        pGbl->loopback.value       = 0x0;
    }
    else
    {
        /**************************************************************************
            For PHY Line loopback, the memory is configured as if Tx and Rx both
            have all the memory.  This is because the RxMAC will write data into
            the space, and the TxMAC will read it out.
        *************************************************************************/
        pGbl->rxq_start_addr.value = 0;
        pGbl->rxq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
        pGbl->txq_start_addr.value = 0;
        pGbl->txq_end_addr.value   = INTERNAL_MEM_SIZE - 1;

        /**************************************************************************
            Initialize the loopback register (MAC loopback).
         *************************************************************************/
        pGbl->loopback.value       = 0x1;
    }

    /**************************************************************************
       MSI Register
     *************************************************************************/
    pGbl->msi_config.value     = 0x0;


    /**************************************************************************
       By default, disable the watchdog timer.  It will be enabled when 
       a packet is queued.
     *************************************************************************/
    pGbl->watchdog_timer     = 0;


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  ConfigMMCRegs
 ******************************************************************************

   DESCRIPTION:
        Used to configure the main memory registers in the JAGCore

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void ConfigMMCRegs( ET131X_ADAPTER *pAdapter )
{
    MMC_CTRL_t  mmc_ctrl = {0};
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "ConfigMMCRegs" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       All we need to do is initialize the Memory Control Register 
     *************************************************************************/
    mmc_ctrl.bits.force_ce        = 0x0;
    mmc_ctrl.bits.rxdma_disable   = 0x0;
    mmc_ctrl.bits.txdma_disable   = 0x0;
    mmc_ctrl.bits.txmac_disable   = 0x0;
    mmc_ctrl.bits.rxmac_disable   = 0x0;
    mmc_ctrl.bits.arb_disable     = 0x0;
    mmc_ctrl.bits.mmc_enable      = 0x1;

    pAdapter->CSRAddress->mmc.mmc_ctrl.value = mmc_ctrl.value;


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_enable_interrupts
 ******************************************************************************

   DESCRIPTION       : Enable interupts on the ET131x
        
   PARAMETERS        : adapter - a pointer to our private adapter structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_enable_interrupts( ET131X_ADAPTER *adapter )
{
    UINT32 MaskValue;


    /**************************************************************************
       Enable all global interrupts
     *************************************************************************/
    if(( adapter->FlowControl == TxOnly ) ||
       ( adapter->FlowControl == Both ))
    {
        MaskValue = INT_MASK_ENABLE;
    }
    else
    {
        MaskValue = INT_MASK_ENABLE_NO_FLOW;
    }

    if( adapter->DriverNoPhyAccess )
    {
        MaskValue |= 0x10000;
    }

    adapter->CachedMaskValue.value = MaskValue;
    adapter->CSRAddress->global.int_mask.value = MaskValue;

    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_disable_interrupts
 ******************************************************************************

   DESCRIPTION       : Enable interrupts on the ET131x
        
   PARAMETERS        : adapter - a pointer to our private adapter structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/

void et131x_disable_interrupts( ET131X_ADAPTER *adapter )
{
    /**************************************************************************
       Disable all global interrupts
     *************************************************************************/
    adapter->CachedMaskValue.value             = INT_MASK_DISABLE;
    adapter->CSRAddress->global.int_mask.value = INT_MASK_DISABLE;

    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  NICGetRegs
 ******************************************************************************

   DESCRIPTION:
        function used to get register data

   PARAMETERS :
        pAdapter         - pointer to our adapter structure
        InfoBuf          - pointer to data struct containing registers to get
        ulBytesAvailable - 
        ulInfoLen -

   RETURN     :
        NONE

 *****************************************************************************/
void * et131x_get_regs( ET131X_ADAPTER *pAdapter, void *InfoBuf,
                        PUINT32 ulBytesAvailable, PUINT32 ulInfoLen )
{
    INT32   nRegCount;
    PINT8   pJCBase;
    INT32   x;
    PUINT32 pReg;
    /*-----------------------------------------------------------------------*/

    
    DBG_FUNC( "et131x_get_regs" );
    DBG_ENTER( et131x_dbginfo );


    memset( (void *)&pAdapter->JagCoreRegs, 0, sizeof( JAGCORE_ACCESS_REGS ));


    /**************************************************************************
       Get the user supplied data 
     *************************************************************************/
    pAdapter->JagCoreRegs = *(PJAGCORE_ACCESS_REGS)InfoBuf;
    nRegCount             = pAdapter->JagCoreRegs.nRegCount;

    pJCBase = (PINT8)&pAdapter->CSRAddress->global; 
    
    for( x = 0; x < nRegCount; x++ ) 
    {
        pReg = (PUINT32)( pJCBase+pAdapter->JagCoreRegs.nOffsets[x] );
        
        pAdapter->JagCoreRegs.nData[x] = *pReg;
    }

    *ulBytesAvailable = sizeof( JAGCORE_ACCESS_REGS );
    *ulInfoLen        = sizeof( JAGCORE_ACCESS_REGS );


    DBG_LEAVE( et131x_dbginfo );
    return( (void *)&pAdapter->JagCoreRegs );
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  et131x_set_regs
 ******************************************************************************

   DESCRIPTION:
        function used to set register data

   PARAMETERS :
        pAdapter - pointer to our adapter structure
        InfoBuf  - pointer to data struct containing register offset and data 
                   to set
   RETURN     :
        N/A

 *****************************************************************************/
void et131x_set_regs( ET131X_ADAPTER *pAdapter, void *InfoBuf )
{
    INT32   nRegCount;
    PINT8   pJCBase;
    INT32   x;
    PUINT32 pReg;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_set_regs" );
    DBG_ENTER( et131x_dbginfo );


    memset( (void *)&pAdapter->JagCoreRegs, 0, sizeof( JAGCORE_ACCESS_REGS ));


    /**************************************************************************
       Get the user supplied data 
     *************************************************************************/
    pAdapter->JagCoreRegs = *(PJAGCORE_ACCESS_REGS)InfoBuf;

    nRegCount = pAdapter->JagCoreRegs.nRegCount;

    pJCBase = (PINT8)&pAdapter->CSRAddress->global; 
    
    for( x = 0; x < nRegCount; x++ ) 
    {
         pReg = (PUINT32)(pJCBase+pAdapter->JagCoreRegs.nOffsets[x]);
        *pReg = pAdapter->JagCoreRegs.nData[x];
    }
    

    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  GetPciCfgRegs
 ******************************************************************************

   DESCRIPTION:
        function used to get register data

   PARAMETERS :
        pAdapter         - pointer to our adapter structure
        InfoBuf          - pointer to data struct containing registers to get
        ulBytesAvailable - 
        ulInfoLen -

   RETURN     :
        NONE

 *****************************************************************************/
void * GetPciCfgRegs( ET131X_ADAPTER *pAdapter, void * InfoBuf,
                      PUINT32 ulBytesAvailable, PUINT32 ulInfoLen )
{
    INT32   nRegCount;
    UINT32  ByteLength = 0;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "GetPciCfgRegs" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Get the user supplied data 
     *************************************************************************/
    pAdapter->PciCfgRegs  = *(PPCI_CFG_SPACE_REGS)InfoBuf;
    nRegCount             = pAdapter->PciCfgRegs.nRegCount;


	/**************************************************************************
       Support for 8,16,32 bit widths
     *************************************************************************/
	ByteLength = pAdapter->PciCfgRegs.nDataWidth/8;

    pci_slot_information_read( pAdapter->pdev,
                               pAdapter->PciCfgRegs.nOffsets[0],
                               (UINT8 *)&pAdapter->PciCfgRegs.nData,
                               ByteLength * nRegCount );

   *ulBytesAvailable = sizeof( PCI_CFG_SPACE_REGS );
   *ulInfoLen        = sizeof( PCI_CFG_SPACE_REGS );
	

    DBG_LEAVE( et131x_dbginfo );
	return( (void *)&pAdapter->PciCfgRegs );
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  SetPciCfgRegs
 ******************************************************************************

   DESCRIPTION:
        function used to set register data

   PARAMETERS :
        pAdapter - pointer to our adapter structure
        InfoBuf  - pointer to data struct containing register offset and data 
                   to set
   RETURN     :
        N/A

 *****************************************************************************/
void SetPciCfgRegs( ET131X_ADAPTER *pAdapter, void * InfoBuf )
{
    INT32  nRegCount;
    INT32  x;
    UINT32 ByteLength = 0;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "SetPciCfgRegs" );
    DBG_ENTER( et131x_dbginfo );


	memset( (void *)&pAdapter->PciCfgRegs, 0, sizeof( PCI_CFG_SPACE_REGS ));


    /**************************************************************************
       Get the user supplied data 
     *************************************************************************/
	pAdapter->PciCfgRegs = *(PPCI_CFG_SPACE_REGS)InfoBuf;

	nRegCount = pAdapter->PciCfgRegs.nRegCount;
	ByteLength = pAdapter->PciCfgRegs.nDataWidth/8;

	for( x = 0; x < nRegCount; x++ ) 
	{
        pci_slot_information_write( pAdapter->pdev,
                                    pAdapter->PciCfgRegs.nOffsets[x],
                                    (UINT8 *)&pAdapter->PciCfgRegs.nData[x * ByteLength],
                                    ByteLength );
	}


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  MemTest
 ******************************************************************************

   DESCRIPTION:
        function used to test rx/tx queue memory at 0x0 and 0x200

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURN     :
        TRUE or FALSE

 *****************************************************************************/
BOOL_t MemTest( ET131X_ADAPTER *pAdapter, UINT32 addr )
{
    UINT32 data;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "MemTest" );
    DBG_ENTER( et131x_dbginfo );


    // Read initial value    
    pAdapter->CSRAddress->mmc.sram_access.value = 1;
    data = pAdapter->CSRAddress->mmc.sram_word1.data;


    // Write test value
    pAdapter->CSRAddress->mmc.sram_word1.data = 0xdeadbeef;   
    pAdapter->CSRAddress->mmc.sram_access.value = 0xf003;
        

    // Read back test value
    pAdapter->CSRAddress->mmc.sram_access.bits.req_access  = 1;
    if ( pAdapter->CSRAddress->mmc.sram_word1.data != 0xdeadbeef )
    {
        DBG_LEAVE( et131x_dbginfo );
        return FALSE;
    }

    // Restore initial value
    pAdapter->CSRAddress->mmc.sram_word1.data = data;
    pAdapter->CSRAddress->mmc.sram_access.value = 0xf003;


    DBG_LEAVE( et131x_dbginfo );
    return TRUE;
}
/*===========================================================================*/
