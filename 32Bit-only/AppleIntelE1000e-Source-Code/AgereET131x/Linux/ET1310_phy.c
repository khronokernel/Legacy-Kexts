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
 * ET1310_phy.c - Routines for configuring and accessing the PHY
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
     $Revision: 1.13 $
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
#include "ET1310_mac.h"

#include "et131x_supp.h"
#include "et131x_adapter.h"
#include "et131x_netdev.h"
#include "et131x_initpci.h"

#include "ET1310_address_map.h"
#include "ET1310_jagcore.h"
#include "ET1310_tx.h"
#include "ET1310_rx.h"
#include "ET1310_mac.h"




/******************************************************************************
   Data for debugging facilities
 *****************************************************************************/
#if ET131X_DBG
extern dbg_info_t *et131x_dbginfo;
#endif  /* ET131X_DBG */




/******************************************************************************
   Prototypes for functions with local scope 
 *****************************************************************************/
int et131x_xcvr_init( ET131X_ADAPTER *adapter );




/******************************************************************************
   ROUTINE :  MiRead
 ******************************************************************************

   DESCRIPTION       : Used to read from the PHY through the MII Interface on
                       the MAC.
        
   PARAMETERS        : adapter  - pointer to our private adapter structure
                       xcvrAddr - the address of the transciever
                       xcvrReg  - the register to read
                       value    - pointer to a 16-bit value in which the value
                                  will be stored.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int MiRead( ET131X_ADAPTER *adapter, UINT8 xcvrAddr, UINT8 xcvrReg, UINT16 *value )
{
    int                 status = 0;
	UINT32              delay;
    MII_MGMT_ADDR_t     miiAddr;
    MII_MGMT_CMD_t      miiCmd;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "MiRead" );


    /**************************************************************************
       Save a local copy of the registers we are dealing with so we can set 
       them back
     *************************************************************************/
    miiAddr.value = adapter->CSRAddress->mac.mii_mgmt_addr.value;
    miiCmd.value  = adapter->CSRAddress->mac.mii_mgmt_cmd.value;


    /**************************************************************************
        Stop the current operation
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x0;


    /**************************************************************************
        Set up the register we need to read from on the correct PHY
     *************************************************************************/
    {
        MII_MGMT_ADDR_t mii_mgmt_addr = {0};

        mii_mgmt_addr.bits.phy_addr = xcvrAddr;
        mii_mgmt_addr.bits.reg_addr = xcvrReg;

        adapter->CSRAddress->mac.mii_mgmt_addr = mii_mgmt_addr;
    }


    /**************************************************************************
        Kick the read cycle off
     *************************************************************************/
    delay = 0;

    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x1;

    do 
    {
        udelay( 50 );
        delay++;
    } while(( adapter->CSRAddress->mac.mii_mgmt_indicator.bits.not_valid ||
              adapter->CSRAddress->mac.mii_mgmt_indicator.bits.busy ) && 
            ( delay < 50 ));


    /**************************************************************************
        If we hit the max delay, we could not read the register
     *************************************************************************/
	if( delay >= 50 )
    {
        DBG_WARNING( et131x_dbginfo, "xcvrReg 0x%08x could not be read\n", xcvrReg );
        DBG_WARNING( et131x_dbginfo, "status is  0x%08x\n", 
                     adapter->CSRAddress->mac.mii_mgmt_indicator.value );

        status = -EIO;
    }


    /**************************************************************************
       If we hit here we were able to read the register and we need to return
       the value to the caller
     *************************************************************************/
    *value = (UINT16)adapter->CSRAddress->mac.mii_mgmt_stat.bits.phy_stat;


    /**************************************************************************
        Stop the read operation
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x0;

    DBG_VERBOSE( et131x_dbginfo, "  xcvr_addr = 0x%02x, "
					             "xcvr_reg  = 0x%02x, "
					             "value     = 0x%04x.\n",
					             xcvrAddr, xcvrReg, *value );


    /**************************************************************************
       set the registers we touched back to the state at which we entered 
       this function
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_addr.value = miiAddr.value;
    adapter->CSRAddress->mac.mii_mgmt_cmd.value  = miiCmd.value;


    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  MiWrite
 ******************************************************************************

   DESCRIPTION       : Used to write to a PHY register through the MII
                       interface of the MAC. Updated for the ET1310.
        
   PARAMETERS        : adapter  - pointer to our private adapter structure
                       xcvrAddr - the address of the transciever
                       xcvrReg  - the register to read
                       value    - 16-bit value to write
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int MiWrite( ET131X_ADAPTER *adapter, UINT8 xcvrAddr, UINT8 xcvrReg, UINT16 value )
{
    int             status = 0;
	UINT32          delay;
    MII_MGMT_ADDR_t miiAddr;
    MII_MGMT_CMD_t  miiCmd;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "MiWrite" );


    /**************************************************************************
       Save a local copy of the registers we are dealing with so we can set 
       them back
     *************************************************************************/
    miiAddr.value = adapter->CSRAddress->mac.mii_mgmt_addr.value;
    miiCmd.value  = adapter->CSRAddress->mac.mii_mgmt_cmd.value;


    /**************************************************************************
       Stop the current operation
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x0;


    /**************************************************************************
       Set up the register we need to write to on the correct PHY
     *************************************************************************/
    {
        MII_MGMT_ADDR_t mii_mgmt_addr = {0};

        mii_mgmt_addr.bits.phy_addr = xcvrAddr;
        mii_mgmt_addr.bits.reg_addr = xcvrReg;

        adapter->CSRAddress->mac.mii_mgmt_addr = mii_mgmt_addr;
    }


    /**************************************************************************
       Add the value to write to the registers to the mac
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_ctrl.value = value;
    delay = 0;

    do
    {
        udelay( 50 );
        delay++;
    } while(( adapter->CSRAddress->mac.mii_mgmt_indicator.bits.busy ) && 
            ( delay < 100 ));


    /**************************************************************************
       If we hit the max delay, we could not write the register
     *************************************************************************/
	if( delay == 100 )
    {
        UINT16 TempValue;

        DBG_WARNING( et131x_dbginfo, "xcvrReg 0x%08x could not be written",
                     xcvrReg );
        DBG_WARNING( et131x_dbginfo, "status is  0x%08x\n", 
                     adapter->CSRAddress->mac.mii_mgmt_indicator.value );
        DBG_WARNING( et131x_dbginfo, "command is  0x%08x\n", 
                     adapter->CSRAddress->mac.mii_mgmt_cmd.value );
 
        MiRead( adapter, xcvrAddr, xcvrReg, &TempValue );
        
        status = -EIO;
    }


    /**************************************************************************
       Stop the write operation
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x0;


    /**************************************************************************
       set the registers we touched back to the state at which we entered 
       this function
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_addr.value = miiAddr.value;
    adapter->CSRAddress->mac.mii_mgmt_cmd.value  = miiCmd.value;


    DBG_VERBOSE( et131x_dbginfo, " xcvr_addr = 0x%02x, "
					             "xcvr_reg  = 0x%02x, "
					             "value     = 0x%04x.\n",
					             xcvrAddr, xcvrReg, value );

    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_xcvr_find
 ******************************************************************************

   DESCRIPTION       : Used to find the PHY ID
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_xcvr_find( ET131X_ADAPTER *adapter )
{
    int           status = -ENODEV;
    UINT8         xcvr_addr;
	MI_IDR1_t     idr1;
	MI_IDR2_t     idr2;
	UINT32        xcvr_id;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_xcvr_find" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       We need to get xcvr id and address we just get the first one
     *************************************************************************/
    for( xcvr_addr = 0; xcvr_addr < 32; xcvr_addr++ ) 
    {
        /**********************************************************************
           Read the ID from the PHY
         *********************************************************************/
        MiRead( adapter, xcvr_addr, (UINT8)FIELD_OFFSET(MI_REGS_t, idr1), &idr1.value );
        MiRead( adapter, xcvr_addr, (UINT8)FIELD_OFFSET(MI_REGS_t, idr2), &idr2.value );
			
        xcvr_id = (UINT32)(( idr1.value << 16 ) | idr2.value );

        if(( idr1.value != 0) && ( idr1.value != 0xffff )) 
        {
            DBG_TRACE( et131x_dbginfo, "Xcvr addr: 0x%02x\tXcvr_id: 0x%08x\n",
                       xcvr_addr, xcvr_id );

            adapter->Stats.xcvr_id   = xcvr_id;
            adapter->Stats.xcvr_addr = (UINT32)xcvr_addr;

            status = 0;
            break;
        }
	} 
    

    DBG_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_setphy_normal
 ******************************************************************************

   DESCRIPTION       : Used by Power Management to force the PHY into 10 Base T
                       half-duplex mode, when going to D3 in WOL mode. Also
                       used during initialization to set the PHY for normal
                       operation.
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_setphy_normal( ET131X_ADAPTER *adapter )
{
    int status;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_setphy_normal" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Make sure the PHY is powered up
     *************************************************************************/
    ET1310_PhyPowerDown( adapter, 0 );
    status = et131x_xcvr_init( adapter );


    DBG_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_xcvr_init
 ******************************************************************************

   DESCRIPTION       : Used to init the phy if we are setting it into force mode
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_xcvr_init( ET131X_ADAPTER *adapter )
{
    int         status = 0;
    MI_IMR_t    imr;
    MI_ISR_t    isr;
    MI_LCR2_t   lcr2;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_xcvr_init" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
        Zero out the adapter structure variable representing BMSR
     *************************************************************************/
    adapter->Bmsr.value = 0;

    MiRead( adapter, (UINT8)adapter->Stats.xcvr_addr, 
            (UINT8)FIELD_OFFSET( MI_REGS_t, isr ), &isr.value );

    MiRead( adapter, (UINT8)adapter->Stats.xcvr_addr, 
            (UINT8)FIELD_OFFSET( MI_REGS_t, imr ), &imr.value );


    /**************************************************************************
       Set the link status interrupt only.  Bad behavior when link status and 
       auto neg are set, we run into a nested interrupt problem
     *************************************************************************/
    imr.bits.int_en         = 0x1;
    imr.bits.link_status    = 0x1;
    imr.bits.autoneg_status = 0x1;

    MiWrite( adapter, (UINT8)adapter->Stats.xcvr_addr, 
             (UINT8)FIELD_OFFSET( MI_REGS_t, imr ), imr.value );


    /**************************************************************************
       Set the LED behavior such that LED 1 indicates speed (off = 10Mbits,
       blink = 100Mbits, on = 1000Mbits) and LED 2 indicates link and
       activity (on for link, blink off for activity).

       NOTE: Some customizations have been added here for specific vendors;
       The LED behavior is now determined by vendor data in the EEPROM. However,
       the above description is the default.
     *************************************************************************/
    if(( adapter->eepromData[1] & 0x4 ) == 0 )
    {
        MiRead( adapter, (UINT8)adapter->Stats.xcvr_addr, 
                (UINT8)FIELD_OFFSET( MI_REGS_t, lcr2 ), &lcr2.value );

        if(( adapter->eepromData[1] & 0x8 ) == 0 )
        {
            lcr2.bits.led_tx_rx = 0x3;
        }
        else
        {
            lcr2.bits.led_tx_rx = 0x4;
        }

        lcr2.bits.led_link = 0xa;

        MiWrite( adapter, (UINT8)adapter->Stats.xcvr_addr, 
                (UINT8)FIELD_OFFSET( MI_REGS_t, lcr2 ), lcr2.value );
    }


    /**************************************************************************
       Determine if we need to go into a force mode and set it
     *************************************************************************/
    if( adapter->AiForceSpeed == 0 && adapter->AiForceDpx == 0 )
    {
        if(( adapter->RegistryFlowControl == TxOnly ) ||
            ( adapter->RegistryFlowControl == Both ))
        {
            ET1310_PhyAccessMiBit( adapter,
                                    TRUEPHY_BIT_SET, 4, 11, NULL );
        }
        else
        {
            ET1310_PhyAccessMiBit( adapter,
                                    TRUEPHY_BIT_CLEAR, 4, 11, NULL );
        }

        if( adapter->RegistryFlowControl == Both )
        {
            ET1310_PhyAccessMiBit( adapter,
                                    TRUEPHY_BIT_SET, 4, 10, NULL );
        }
        else
        {
            ET1310_PhyAccessMiBit( adapter,
                                    TRUEPHY_BIT_CLEAR, 4, 10, NULL );
        }


        /**********************************************************************
           Set the phy to autonegotiation
         *********************************************************************/
        ET1310_PhyAutoNeg( adapter, TRUE );


        /* NOTE - Do we need this? */
        ET1310_PhyAccessMiBit( adapter,
                               TRUEPHY_BIT_SET, 0, 9, NULL );

        DBG_LEAVE( et131x_dbginfo );
        return status;
    }
    else
    {
        ET1310_PhyAutoNeg( adapter, FALSE );

        /**********************************************************************
           Set to the correct force mode.
         *********************************************************************/
        if( adapter->AiForceDpx != 1 )
        {
            if(( adapter->RegistryFlowControl == TxOnly ) ||
               ( adapter->RegistryFlowControl == Both ))
            {
                ET1310_PhyAccessMiBit( adapter,
                                       TRUEPHY_BIT_SET, 4, 11, NULL );
            }
            else
            {
                ET1310_PhyAccessMiBit( adapter,
                                       TRUEPHY_BIT_CLEAR, 4, 11, NULL );
            }

            if( adapter->RegistryFlowControl == Both )
            {
                ET1310_PhyAccessMiBit( adapter,
                                       TRUEPHY_BIT_SET, 4, 10, NULL );
            }
            else
            {
                ET1310_PhyAccessMiBit( adapter,
                                       TRUEPHY_BIT_CLEAR, 4, 10, NULL );
            }
        }
        else
        {
            ET1310_PhyAccessMiBit( adapter,
                                   TRUEPHY_BIT_CLEAR, 4, 10, NULL );
            ET1310_PhyAccessMiBit( adapter,
                                   TRUEPHY_BIT_CLEAR, 4, 11, NULL );
        }

        switch( adapter->AiForceSpeed )
        {
        case 10:
            if( adapter->AiForceDpx == 1 )
            {
                TPAL_SetPhy10HalfDuplex( adapter );
            }
            else if( adapter->AiForceDpx == 2 )
            {
                TPAL_SetPhy10FullDuplex( adapter );
            }
            else
            {
                TPAL_SetPhy10Force( adapter );
            }
            break;

        case 100:
            if( adapter->AiForceDpx == 1 )
            {
                TPAL_SetPhy100HalfDuplex( adapter );
            }
            else if( adapter->AiForceDpx == 2 )
            {
                TPAL_SetPhy100FullDuplex( adapter );
            }
            else
            {
                TPAL_SetPhy100Force( adapter );
            }
            break;

        case 1000:
            TPAL_SetPhy1000FullDuplex( adapter );
            break;
        }


        DBG_LEAVE( et131x_dbginfo );
        return status;
    }
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  et131x_Mii_check
 ******************************************************************************

 DESCRIPTION:
    used to 

 PARAMETERS :
    pAdapter - pointer to our adapter
    bmsr   - 

 RETURNS    :
    Nothing

 *****************************************************************************/
void et131x_Mii_check( ET131X_ADAPTER *pAdapter,
                       MI_BMSR_t bmsr,
                       MI_BMSR_t bmsr_ints )
{
    UCHAR    ucLinkStatus;
    INT32    nAutoNegStatus;
    INT32    nSpeed;
    INT32    nDuplex;
    INT32    nMdiMdix;
    INT32    nMasterSlave;
    INT32    nPolarity;
    unsigned long lockflags;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_Mii_check" );
    DBG_ENTER( et131x_dbginfo );


    if( bmsr_ints.bits.link_status ) 
	{
		if( bmsr.bits.link_status ) 
		{
            pAdapter->PoMgmt.TransPhyComaModeOnBoot = 20;


            /******************************************************************
               Update our state variables and indicate the connected state
             *****************************************************************/
            spin_lock_irqsave( &pAdapter->Lock, lockflags );
            
            pAdapter->MediaState = NETIF_STATUS_MEDIA_CONNECT;
            MP_CLEAR_FLAG( pAdapter, fMP_ADAPTER_LINK_DETECTION );
            
            spin_unlock_irqrestore( &pAdapter->Lock, lockflags );

            if( pAdapter->RegistryPhyLoopbk == FALSE )
            {
                /**************************************************************
                    Don't indicate state if we're in loopback mode
                 *************************************************************/
                netif_indicate_status( pAdapter->netdev, pAdapter->MediaState );
            }
		} 
		else 
		{
            DBG_WARNING( et131x_dbginfo, "Link down cable problem\n" );

            if( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_10MBPS )
            {
                // NOTE - Is there a way to query this without TruePHY?
                //if( TRU_QueryCoreType ( pAdapter->hTruePhy, 0 ) == EMI_TRUEPHY_A13O )
                {
                    UINT16 Register18;

                    MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                           0x12, &Register18 );
                    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                      0x12, Register18 | 0x4 );
                    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                   0x10, Register18 | 0x8402 );
                    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                      0x11, Register18 | 511 );
                    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                            0x12, Register18 );
                }
            }
			
            /******************************************************************
                For the first N seconds of life, we are in "link detection"
                When we are in this state, we should only report "connected". 
                When the LinkDetection Timer expires, we can report
                disconnected (handled in the LinkDetectionDPC).
             *****************************************************************/
            if(( MP_IS_FLAG_CLEAR( pAdapter, fMP_ADAPTER_LINK_DETECTION )) ||
               ( pAdapter->MediaState == NETIF_STATUS_MEDIA_DISCONNECT ))
            {
                spin_lock_irqsave( &pAdapter->Lock, lockflags );
                pAdapter->MediaState = NETIF_STATUS_MEDIA_DISCONNECT;
                spin_unlock_irqrestore( &pAdapter->Lock, lockflags );

                /**************************************************************
                   Only indicate state if we're in loopback mode
                 *************************************************************/
                if( pAdapter->RegistryPhyLoopbk == FALSE )
                {
                    netif_indicate_status( pAdapter->netdev, pAdapter->MediaState );
                }
            }

            pAdapter->uiLinkSpeed  = 0;
            pAdapter->uiDuplexMode = 0;


			/******************************************************************
			   Free the packets being actively sent & stopped
			 *****************************************************************/
			et131x_free_busy_send_packets( pAdapter );


            /******************************************************************
			   Re-initialize the send structures
			 *****************************************************************/
			et131x_init_send( pAdapter );


			/******************************************************************
			   Reset the RFD list and re-start RU 
			 *****************************************************************/
			et131x_reset_recv( pAdapter );


			/******************************************************************
			   Bring the device back to the state it was during init prior
               to autonegotiation being complete.  This way, when we get the
               auto-neg complete interrupt, we can complete init by calling
               ConfigMacREGS2.
			 *****************************************************************/
			et131x_soft_reset( pAdapter );


            /******************************************************************
               Setup ET1310 as per the documentation
             *****************************************************************/
            et131x_adapter_setup( pAdapter );


            /******************************************************************
               Setup the PHY into coma mode until the cable is plugged back in
             *****************************************************************/
            if( pAdapter->RegistryPhyComa == 1 )
            {
                EnablePhyComa( pAdapter );
            }
		}
	}

    if( bmsr_ints.bits.auto_neg_complete ||
        (( pAdapter->AiForceDpx == 3 ) && ( bmsr_ints.bits.link_status )))
    {
        if( bmsr.bits.auto_neg_complete  ||
           ( pAdapter->AiForceDpx == 3 ))
        {
            ET1310_PhyLinkStatus( pAdapter,
                                  &ucLinkStatus,
                                  &nAutoNegStatus,
                                  &nSpeed,
                                  &nDuplex,
                                  &nMdiMdix,
                                  &nMasterSlave,
                                  &nPolarity );
                                
                                    
            pAdapter->uiLinkSpeed  = nSpeed;
            pAdapter->uiDuplexMode = nDuplex;

            DBG_TRACE( et131x_dbginfo,
                       "pAdapter->uiLinkSpeed 0x%04x, pAdapter->uiDuplex 0x%08x\n",
                       pAdapter->uiLinkSpeed, pAdapter->uiDuplexMode );

            pAdapter->PoMgmt.TransPhyComaModeOnBoot = 20;

            if( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_10MBPS )
            {
                // NOTE - Is there a way to query this without TruePHY?
                //if( TRU_QueryCoreType ( pAdapter->hTruePhy, 0 ) == EMI_TRUEPHY_A13O )
                {
                    UINT16 Register18;

                    MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                           0x12, &Register18 );
                    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                      0x12, Register18 | 0x4 );
                    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                   0x10, Register18 | 0x8402 );
                    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                      0x11, Register18 | 511 );
                    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
                                                            0x12, Register18 );
                }
            }

            ConfigFlowControl( pAdapter );


            if(( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_1000MBPS ) &&
                ( pAdapter->RegistryJumboPacket > 2048 ))

            {
                ET1310_PhyAndOrReg( pAdapter, 0x16, 0xcfff, 0x2000 );
            }

            SetRxDmaTimer( pAdapter );
            ConfigMACRegs2( pAdapter );
        }
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  TPAL_MiAccessRegs
 ******************************************************************************

   DESCRIPTION       : Used to perform a single or series of MI Register
                       transactions on the specified PHY
        
   PARAMETERS        : hPlatform      - handle to platform resources
                       nPhyId         - logical PHY ID (if >= 0) or 
                                        logical PHY group ID (if <0)
                       pbyAccessFlags - Pointer to a list of flags, which
                                        specify the transaction type associated
                                        with each register contained in the
                                        'pbyRegisters' array.  Valid values:
                                            TRUEPHY_READ
                                            TRUEPHY_WRITE
                                            TRUEPHY_MASK
                                        If a logical PHY Group ID is specified
                                        in 'nPhyId', the value of these flags
                                        must be TRUEPHY_WRITE
                       pbyRegisters   - Pointer to a list of MI registers
                                        addressess (0-31), one of each register
                                        specified by 'nRegCount'
                       pwData         - Pointer to a buffer that contains the
                                        data required by or resulting from each
                                        transaction, the contents of which are 
                                        dependent upon the 'pbyAccessFlags'
                                        parameter. Specifically:

                                        If 'pbyAccessFlags[i]' is TRUEPHY_READ,
                                        the contents of the register specified 
                                        by 'pbyRegisters[i]' is read and stored
                                        in 'pwData[i]'.

                                        If 'pbyAccessFlags[i]' is TRUEPHY_WRITE,
                                        the contents of 'pwData[i]' is written
                                        to the register specified by 
                                        'pbyRegisters[i]'.

                                        If 'pbyAccessFlags[i]' is TRUEPHY_MASK,
                                        the contents of the register specified
                                        by 'pbyRegisters[i]' is read and stored
                                        in 'pwData [i]', which is then
                                        logically AND'ed with the contents of 
                                        'pwAndMasks[i]' and logically OR'ed
                                        with the contents of 'pwOrMasks[i]' 
                                        before it is written back to the 
                                        register specified by 'pbyRegisters[i]'

                                        This allows the calling function to
                                        interleave a register read, logical 
                                        and/or, and write operation within a 
                                        single transaction.

                        pwAndMasks    - Pointer to buffer containing AND masks,
                                        which is required if the corresponding 
                                        transaction type is TRUEPHY_MASK; 
                                        otherwise, this parameter is ignored.
                        pwOrMasks     - Pointer to buffer containing OR masks,
                                        which is required if the corresponding 
                                        transaction type is TRUEPHY_MASK; 
                                        otherwise, this parameter is ignored.
                        nRegCount     - Specifies the number of register
                                        transactions to be performed.
        
   RETURNS           :  TRUEPHY_SUCCESS
                        TRUEPHY_FAILURE
        
   REUSE INFORMATION :
        
 *****************************************************************************/
INT32 TPAL_MiAccessRegs( TPAL_HANDLE hPlatform,
                         INT32       nPhyId,
                         PUCHAR      pbyAccessFlags,
                         PUCHAR      pbyRegisters,
                         PUINT16     pwData,
                         PUINT16     pwAndMasks,
                         PUINT16     pwOrMasks,
                         INT32       nRegCount )
{
    INT32           nStatus   = TRUEPHY_FAILURE;
    INT32           index;
    INT16           wTemp;
    UINT8           xcvrAddr;
    ET131X_ADAPTER *pAdapter  = hPlatform;
    unsigned long   lockflags;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "TPAL_MiAccessRegs" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Validate Input parameters.  We are not worried about nPhyId because we 
       can only access the one we control
     *************************************************************************/
    if( pbyAccessFlags != NULL && pwData != NULL )
    {
        /**********************************************************************
           Convert nPhyId into a PHY physical address(nPhyAddress)
         *********************************************************************/


        /**********************************************************************
           Convert nPhyId into a MI bus address (nBusAddress).  
           
           NOTE: This is only needed for platforms that support multiple MI 
                 buses
         *********************************************************************/


        /**********************************************************************
           Get the xcvr addr for all transactions to go to
         *********************************************************************/
        xcvrAddr = (UINT8)pAdapter->Stats.xcvr_addr;


        /**********************************************************************
           Go through all registers performing specific transaction indicated
           in the corresponding pbyAccessFlags array.
         *********************************************************************/
        for( index = 0; index < nRegCount; index++ )
        {
            /******************************************************************
               Validate MI register
             *****************************************************************/
            if( pbyRegisters[index] > 31 )
            {
                break;
            }


            /******************************************************************
               Is this a read?
             *****************************************************************/
            if( pbyAccessFlags[index] == TRUEPHY_READ )
            {
                /**************************************************************
                   Group reads are not allowed
                 *************************************************************/
                if( nPhyId < 0 )
                {
                    break;
                }


                /**************************************************************
                   Make sure we acquire the spin lock
                 *************************************************************/
                spin_lock_irqsave( &pAdapter->PHYLock, lockflags );


                /**************************************************************
                   Let's perform the read operation
                 *************************************************************/
                MiRead( pAdapter, xcvrAddr, pbyRegisters[index], 
                        (PUINT16)&pwData[index] );

                spin_unlock_irqrestore( &pAdapter->PHYLock, lockflags );

            }
            else if( pbyAccessFlags[index] == TRUEPHY_WRITE )
            {
                /**************************************************************
                   Make sure we acquire the spin lock
                 *************************************************************/
                spin_lock_irqsave( &pAdapter->PHYLock, lockflags );


                /**************************************************************
                   Let's perform the write operation
                 *************************************************************/
                MiWrite( pAdapter, xcvrAddr, pbyRegisters[index], 
                         (UINT16)pwData[index] );

                spin_unlock_irqrestore( &pAdapter->PHYLock, lockflags );
            }
            else if( pbyAccessFlags[index] == TRUEPHY_MASK )
            {
                /**************************************************************
                   Group masks are not allowed
                 *************************************************************/
                if( nPhyId < 0 )
                {
                    break;
                }

                /**************************************************************
                   Pointer to AND and OR masks must not be NULL
                 *************************************************************/
                if(( pwOrMasks == NULL ) || ( pwAndMasks == NULL ))
                {
                    break;
                }

                /**************************************************************
                   Make sure we acquire the spin lock
                 *************************************************************/
                spin_lock_irqsave( &pAdapter->PHYLock, lockflags );


                /**************************************************************
                   Perform read
                 *************************************************************/
                MiRead( pAdapter, xcvrAddr, pbyRegisters[index], 
                        (PUINT16)&pwData[index] );


                /**************************************************************
                   Perform and/or masks and write it back
                 *************************************************************/
                wTemp = (( pwData[index] & pwAndMasks[index] ) | 
                           pwOrMasks[index] );

                MiWrite( pAdapter, xcvrAddr, pbyRegisters[index], 
                         (UINT16)wTemp );

                spin_unlock_irqrestore( &pAdapter->PHYLock, lockflags );
            }
            else
            {
                /**************************************************************
                   Invalid transaction type, just break out of the loop
                 *************************************************************/
                break;
            }
        }//end for loop

        /**********************************************************************
           If we completed all the transactions, indicate success
         *********************************************************************/
        if( index == nRegCount )
        {
            nStatus = TRUEPHY_SUCCESS;
        }
    }//end if


    DBG_LEAVE( et131x_dbginfo );
    return nStatus;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  TPAL_PlatformExit
 ******************************************************************************

   DESCRIPTION       : Used to exit the TRUEPHY library which will de-allocate
                       any system resources allocated.
        
   PARAMETERS        : hPlatform - handle to the platform resources
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/

void TPAL_PlatformExit( TRUEPHY_OSAL_HANDLE hPlatform )
{
    DBG_FUNC( "TPAL_PlatformExit" );
    DBG_ENTER( et131x_dbginfo );

    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/



///////////////////////////////////////////////////////////////////////////////
////                         OS Specific Functions                         ////
///////////////////////////////////////////////////////////////////////////////
/******************************************************************************
   ROUTINE :  TPAL_AllocMem
 ******************************************************************************

   DESCRIPTION       : Used so the TRUEPHY library can allocate memory
        
   PARAMETERS        : ulNumBytes - number of bytes to allocate
        
   RETURNS           : pointer to allocated memory block on success, or
                       NULL on failure
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void * TPAL_AllocMem( TRUEPHY_OSAL_HANDLE hPlatform, u_long ulNumBytes )
{
    void     *pBuffer = NULL;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "TPAL_AllocMem" );
    DBG_ENTER( et131x_dbginfo );


    pBuffer = kmalloc( ulNumBytes, GFP_ATOMIC );


    DBG_LEAVE( et131x_dbginfo );
    return pBuffer;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  TPAL_FreeMem
 ******************************************************************************

   DESCRIPTION       : Used to free a previously allocated block of memory
        
   PARAMETERS        : pMemBlock - the pointer to the buffer to free
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void TPAL_FreeMem( TRUEPHY_OSAL_HANDLE hPlatform, void *pMemBlock )
{
    DBG_FUNC( "TPAL_FreeMem" );
    DBG_ENTER( et131x_dbginfo );


    kfree( pMemBlock );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_Sleep
 ******************************************************************************

   DESCRIPTION:
        Used to delay execution for a specified number of milliseconds

   PARAMETERS :
        ulMsec - Number of milliseconds to delay.  This parameter can be zero

   RETURNS    :
        NONE

 *****************************************************************************/
void TPAL_Sleep( TRUEPHY_OSAL_HANDLE hPlatform, u_long ulMsec )
{
    DBG_FUNC( "TPAL_Sleep" );
    DBG_ENTER( et131x_dbginfo );


    mdelay( ulMsec );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_GetSystemUpTime
 ******************************************************************************

   DESCRIPTION:
        Used to get the numberr of milliseconds that have elapsed since the 
        system was started

   PARAMETERS :
        NONE

   RETURNS    :
        number of milliseconds since system was started

 *****************************************************************************/
u_long TPAL_GetSystemUpTime( TRUEPHY_OSAL_HANDLE hPlatform )
{
    UINT32 uptime;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "TPAL_GetSystemUpTime" );
    DBG_ENTER( et131x_dbginfo );


    uptime = 10 * jiffies / HZ;


    DBG_LEAVE( et131x_dbginfo );
    return uptime;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_GetLinkStatusInfo
 ******************************************************************************

   DESCRIPTION:
        used to determine what link speed and duplex the phy is set to 

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void TPAL_GetLinkStatusInfo( ET131X_ADAPTER *pAdapter )
{
    UINT32 index = 0;
    INT32  MdiMdix;
    INT32  MasterSlave;
    INT32  Polarity;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "TPAL_GetLinkStatusInfo" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Get the link status information from the phy.  Loop until the 
       autonegstatus is complete or link status is up.

       If looping for 50 uSec, break out, we have a problem
     *************************************************************************/
    do
    {
        ET1310_PhyLinkStatus( pAdapter,
                                  &pAdapter->ucLinkStatus,
                                  &pAdapter->uiAutoNegStatus,
                                  &pAdapter->uiLinkSpeed,
                                  &pAdapter->uiDuplexMode,
                                  &MdiMdix,
                                  &MasterSlave,
                                  &Polarity );

        DBG_VERBOSE( et131x_dbginfo, 
                     "uiAutoNegStatus 0x%08x ucLinkStatus 0x%04x\n "
                     "uiLinkSpeed 0x%04x, uiDuplexMode 0x%08x\n",
                     pAdapter->uiAutoNegStatus, 
                     pAdapter->ucLinkStatus,
                     pAdapter->uiLinkSpeed, 
                     pAdapter->uiDuplexMode );

        udelay( 100 );
        index++;

        if( index == 10000 )
        {
            /******************************************************************
               We hit our limit, we need to set a variable so during power 
               management we know to try 100/half
             *****************************************************************/
            pAdapter->PoMgmt.Failed10Half = TRUE;
            break;
        }
    } while( pAdapter->uiAutoNegStatus != TRUEPHY_ANEG_COMPLETE  || 
             pAdapter->ucLinkStatus == 0 );
    
    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_SetPhy10HalfDuplex
 ******************************************************************************

   DESCRIPTION:
        Used to force the phy into 10 Base T Half Duplex mode.  Also sets the 
        MAC so it is syncd up properly

   PARAMETERS :
        pAdapter - pointer to the adapter structure

   RETURNS    :
        TRUEPHY_SUCCESS - if everything goes according to plan
        TRUEPHY_FALIURE  -if something goes wrong

 *****************************************************************************/
INT32 TPAL_SetPhy10HalfDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "TPAL_SetPhy10HalfDuplex" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power down the PHY\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    /**************************************************************************
       First we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 1000 BaseT\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 100 BaseT "
                   "for forcing 10 BaseT Half Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        Set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_HALF );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not set Advertise of 10 Half\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    /**************************************************************************
       Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power up the PHY\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    DBG_LEAVE( et131x_dbginfo );
    return returnValue ;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_SetPhy10FullDuplex
 ******************************************************************************

   DESCRIPTION:
        Used to force the phy into 10 Base T Full Duplex mode.  Also sets the 
        MAC so it is syncd up properly

   PARAMETERS :
        pAdapter - pointer to the adapter structure

   RETURNS    :
        TRUEPHY_SUCCESS - if everything goes according to plan
        TRUEPHY_FALIURE  -if somethign goes wrong during the procedures

 *****************************************************************************/
INT32 TPAL_SetPhy10FullDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/

    
    DBG_FUNC( "TPAL_SetPhy10FullDuplex" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power down the PHY\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        First we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 1000 BaseT "
                   "for Forcing 10 BaseT Full Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 100 BaseT "
                   "for Forcing 10 BaseT Full Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        Set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not set Advertise of 10 Full\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
       Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power up the phy\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    DBG_LEAVE( et131x_dbginfo );
    return returnValue;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_SetPhy10Force
 ******************************************************************************

   DESCRIPTION:
        Used to force the phy into 10 Base T Full Duplex mode WITHOUT using
        autonegotiation. 

   PARAMETERS :
        pAdapter - pointer to the adapter structure

   RETURNS    :

 *****************************************************************************/
void TPAL_SetPhy10Force( ET131X_ADAPTER *pAdapter )
{
    DBG_FUNC( "TPAL_SetPhy10Force" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );


    /**************************************************************************
       Disable autoneg
     *************************************************************************/
    ET1310_PhyAutoNeg( pAdapter, FALSE );


    /**************************************************************************
       Disable all advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );

	
    /**************************************************************************
       Force 10 Mbps
     *************************************************************************/
    ET1310_PhySpeedSelect( pAdapter, TRUEPHY_SPEED_10MBPS );
 

    /**************************************************************************
       Force Full duplex
     *************************************************************************/
    ET1310_PhyDuplexMode( pAdapter, TRUEPHY_DUPLEX_FULL );


    /**************************************************************************
       Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_SetPhy100HalfDuplex
 ******************************************************************************

   DESCRIPTION:
        Used to force the phy into 100 Base T Half Duplex mode.  Also sets the 
        MAC so it is syncd up properly

   PARAMETERS :
        pAdapter - pointer to the adapter structure

   RETURNS    :
        TRUEPHY_SUCCESS - if everything goes according to plan
        TRUEPHY_FALIURE  -if somethign goes wrong during the procedures

 *****************************************************************************/
INT32 TPAL_SetPhy100HalfDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/

    
    DBG_FUNC( "TPAL_SetPhy100HalfDuplex" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        
        DBG_ERROR( et131x_dbginfo, "Could not power down the PHY\n" );
        DBG_LEAVE( et131x_dbginfo );
        return( returnValue );
    }


    /**************************************************************************
        first we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 1000 BaseT "
                   "for Forcing 100 BaseT Half Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 10 BaseT "
                   "for Forcing 100 BaseT Half Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        Set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_HALF );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not set Advertise of 100 Half\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /* Set speed */
    ET1310_PhySpeedSelect( pAdapter, TRUEPHY_SPEED_100MBPS );


    /**************************************************************************
       Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power up the PHY\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    DBG_LEAVE( et131x_dbginfo );
    return returnValue;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_SetPhy100FullDuplex
 ******************************************************************************

   DESCRIPTION:
        Used to force the phy into 100 Base T Full Duplex mode.  Also sets the 
        MAC so it is syncd up properly

   PARAMETERS :
        pAdapter - pointer to the adapter structure

   RETURNS    :
        TRUEPHY_SUCCESS - if everything goes according to plan
        TRUEPHY_FALIURE  -if somethign goes wrong during the procedures

 *****************************************************************************/
INT32 TPAL_SetPhy100FullDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "TPAL_SetPhy100FullDuplex" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power down PHY\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        First we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 1000 BaseT "
                   "for Forcing 100 BaseT Full Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 10 BaseT "
                             "for Forcing 100 BaseT Full Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        Set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not set Advertise of 100 Full\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
       Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power up PHY\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    DBG_LEAVE( et131x_dbginfo );
    return returnValue;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_SetPhy100Force
 ******************************************************************************

   DESCRIPTION:
        Used to force the phy into 100 Base T Full Duplex mode WITHOUT using
        autonegotiation. 

   PARAMETERS :
        pAdapter - pointer to the adapter structure

   RETURNS    :

 *****************************************************************************/
void TPAL_SetPhy100Force( ET131X_ADAPTER *pAdapter )
{
    DBG_FUNC( "TPAL_SetPhy100Force" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );


    /**************************************************************************
       Disable autoneg
     *************************************************************************/
    ET1310_PhyAutoNeg( pAdapter, FALSE );


    /**************************************************************************
       Disable all advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	

    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	

    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );

	
    /**************************************************************************
       Force 100 Mbps
     *************************************************************************/
    ET1310_PhySpeedSelect( pAdapter, TRUEPHY_SPEED_100MBPS );
 

    /**************************************************************************
       Force Full duplex
     *************************************************************************/
    ET1310_PhyDuplexMode( pAdapter, TRUEPHY_DUPLEX_FULL );


    /**************************************************************************
       Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_SetPhy1000FullDuplex
 ******************************************************************************

   DESCRIPTION:
        Used to force the phy into 1000 Base T Full Duplex mode.  Also sets the 
        MAC so it is syncd up properly

   PARAMETERS :
        pAdapter - pointer to the adapter structure

   RETURNS    :
        TRUEPHY_SUCCESS - if everything goes according to plan
        TRUEPHY_FALIURE  -if somethign goes wrong during the procedures

 *****************************************************************************/
INT32 TPAL_SetPhy1000FullDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/

    
    DBG_FUNC( "TPAL_SetPhy1000FullDuplex" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power down phy\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        first we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 100 BaseT "
                   "for Forcing 1000 BaseT Full Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not turn off advertisement of 10 BaseT "
                   "for Forcing 1000 BaseT Full Duplex\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not set Advertise of 1000 Full\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
       power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power up PHY\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    DBG_LEAVE( et131x_dbginfo );
    return returnValue;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  TPAL_SetPhyAutoNeg
 ******************************************************************************

   DESCRIPTION:
        Used to set phy to autonegotiation mode.  

   PARAMETERS :
        pAdapter - pointer to the adapter structure

   RETURNS    :
        TRUEPHY_SUCCESS - if everything goes according to plan
        TRUEPHY_FALIURE  -if somethign goes wrong during the procedures

 *****************************************************************************/
INT32 TPAL_SetPhyAutoNeg( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "TPAL_SetPhyAutoNeg" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power down phy\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
        Turn on advertisement of all capabilities
     *************************************************************************/
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_BOTH );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, 
                   "Could not Turn on Advertisement of 10 BaseT "
                   "from Setting to Auto Negotiation\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_BOTH );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not Turn on Advertisement of 100 BaseT "
                   "from Setting to Auto Negotiation\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }

    if( pAdapter->DeviceID != ET131X_PCI_DEVICE_ID_FAST )
    {
        ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
    }
    else
    {
        ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
    }
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo,
                   "Could not Turn on Advertisement of 1000 BaseT "
                   "from Setting to Auto Negotiation\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    /**************************************************************************
       Make sure auto-neg is ON (it is disabled in FORCE modes)
     *************************************************************************/
    ET1310_PhyAutoNeg( pAdapter, TRUE );


    /**************************************************************************
       Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );

    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
           HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( et131x_dbginfo, "Could not power up phy\n" );
        DBG_LEAVE( et131x_dbginfo );
        return returnValue;
    }


    DBG_LEAVE( et131x_dbginfo );
    return returnValue;
}
/*===========================================================================*/




/******************************************************************************
*******************************************************************************

  The routines which follow provide low-level access to the PHY, and are used
  primarily by the routines above (although there are a few places elsewhere in
  the driver where this level of access is required).

*******************************************************************************
******************************************************************************/

static UINT16  ConfigPhy[25][2] =
{
    /* Reg      Value      Register */
    /* Addr                         */
    {0x880B,    0x0926}, /* AfeIfCreg4B1000Msbs */
    {0x880C,    0x0926}, /* AfeIfCreg4B100Msbs */
    {0x880D,    0x0926}, /* AfeIfCreg4B10Msbs */

    {0x880E,    0xB4D3}, /* AfeIfCreg4B1000Lsbs */
    {0x880F,    0xB4D3}, /* AfeIfCreg4B100Lsbs */
    {0x8810,    0xB4D3}, /* AfeIfCreg4B10Lsbs */

    {0x8805,    0xB03E}, /* AfeIfCreg3B1000Msbs */
    {0x8806,    0xB03E}, /* AfeIfCreg3B100Msbs */
    {0x8807,    0xFF00}, /* AfeIfCreg3B10Msbs */

    {0x8808,    0xE090}, /* AfeIfCreg3B1000Lsbs */
    {0x8809,    0xE110}, /* AfeIfCreg3B100Lsbs */
    {0x880A,    0x0000}, /* AfeIfCreg3B10Lsbs */
    
    {0x300D,    1     }, /* DisableNorm */
    
    {0x280C,    0x0180}, /* LinkHoldEnd */

    {0x1C21,    0x0002}, /* AlphaM */

    {0x3821,    6     }, /* FfeLkgTx0 */
    {0x381D,    1     }, /* FfeLkg1g4 */
    {0x381E,    1     }, /* FfeLkg1g5 */
    {0x381F,    1     }, /* FfeLkg1g6 */
    {0x3820,    1     }, /* FfeLkg1g7 */

    {0x8402,    0x01F0}, /* Btinact */
    {0x800E,    20    }, /* LftrainTime */
    {0x800F,    24    }, /* DvguardTime */
    {0x8010,    46    }, /* IdlguardTime */

    {0,         0     } 

};
//
// condensed version of the phy initialization routine
//
void ET1310_PhyInit( ET131X_ADAPTER *pAdapter )
{
    UINT16  usData, usIndex;


    if( pAdapter == NULL )
    {
        return;
    }

    
    // get the identity (again ?)
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_ID_1, &usData );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_ID_2, &usData );
    
    // what does this do/achieve ?
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, &usData );   // should read 0002
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0006 );
    
    // read modem register 0402, should I do something with the return data ?
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_INDEX_REG, 0x0402 );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_DATA_REG, &usData );

    // what does this do/achieve ?    
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0002 );

    // get the identity (again ?)
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_ID_1, &usData );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_ID_2, &usData );

    // what does this achieve ?    
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, &usData );   // should read 0002
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0006 );
    
    // read modem register 0402, should I do something with the return data ?
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_INDEX_REG, 0x0402 );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_DATA_REG, &usData );

    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0002 );

    // what does this achieve (should return 0x1040)
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, &usData );   // should read 0002
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, 0x1840 );
    
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0007 );
    
    // here the writing of the array starts....
    usIndex = 0;
    while( ConfigPhy[usIndex][0] != 0x0000 )
    {
        // write value
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_INDEX_REG, ConfigPhy[usIndex][0] );
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_DATA_REG, ConfigPhy[usIndex][1] );

        // read it back
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_INDEX_REG, ConfigPhy[usIndex][0] );
        MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_DATA_REG, &usData );

        // do a check on the value read back ?
        usIndex++;
    }
    // here the writing of the array ends...
    
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );   // 0x1840
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, &usData );   // should read 0007
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, 0x1040 );
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0002 );

    return;

}




void ET1310_PhyReset( ET131X_ADAPTER *pAdapter )
{
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, 0x8000 );
    return;
}  




void ET1310_PhyPowerDown( ET131X_ADAPTER *pAdapter, BOOL_t down )
{
    UINT16  usData;

    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );

    if( down == FALSE ) 
    {
        // Power UP
        usData &= ~0x0800;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
    else
    {
        // Power DOWN
        usData |= 0x0800;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }

    return;
}


void ET1310_PhyAutoNeg( ET131X_ADAPTER *pAdapter, BOOL_t enable )
{
    UINT16 usData;

    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );

    if( enable == TRUE )
    {
        // Autonegotiation ON
        usData |= 0x1000;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
    else
    {
        // Autonegotiation OFF
        usData &= ~0x1000;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }

    return;
}

void ET1310_PhyDuplexMode( ET131X_ADAPTER *pAdapter, UINT16 duplex )
{
    UINT16 usData;

    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );

    if( duplex == TRUEPHY_DUPLEX_FULL )
    {
        // Set Full Duplex
        usData |= 0x100;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
    else
    {
        // Set Half Duplex
        usData &= ~0x100;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }

    return;
}


void ET1310_PhySpeedSelect( ET131X_ADAPTER *pAdapter, UINT16 speed )
{
    UINT16 usData;

    // Read the PHY control register
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );

    // Clear all Speed settings (Bits 6, 13)
    usData &= ~0x2040;

    // Reset the speed bits based on user selection
    switch( speed )
    {
    case TRUEPHY_SPEED_10MBPS:
        // Bits already cleared above, do nothing
        break;

    case TRUEPHY_SPEED_100MBPS:
        // 100M == Set bit 13
        usData |= 0x2000;
        break;

    case TRUEPHY_SPEED_1000MBPS:
    default:
        usData |= 0x0040;
        break;
    }

    // Write back the new speed
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );

    return;
}


void ET1310_PhyAdvertise1000BaseT( ET131X_ADAPTER *pAdapter, UINT16 duplex )
{
    UINT16 usData;

    // Read the PHY 1000 Base-T Control Register
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_1000_CONTROL, &usData );

    // Clear Bits 8,9
    usData &= ~0x0300;

    switch( duplex )
    {
    case TRUEPHY_ADV_DUPLEX_NONE:
        // Duplex already cleared, do nothing
        break;

    case TRUEPHY_ADV_DUPLEX_FULL:
        // Set Bit 9
        usData |= 0x0200;
        break;

    case TRUEPHY_ADV_DUPLEX_HALF:
        // Set Bit 8
        usData |= 0x0100;
        break;

    case TRUEPHY_ADV_DUPLEX_BOTH:
    default:
        usData |= 0x0300;
        break;
    }

    // Write back advertisement
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_1000_CONTROL, usData );

    return;
}


void ET1310_PhyAdvertise100BaseT( ET131X_ADAPTER *pAdapter, UINT16 duplex )
{
    UINT16 usData;

    // Read the Autonegotiation Register (10/100)
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_AUTO_ADVERTISEMENT, &usData );

    // Clear bits 7,8
    usData &= ~0x0180;

    switch( duplex )
    {
    case TRUEPHY_ADV_DUPLEX_NONE:
        // Duplex already cleared, do nothing
        break;

    case TRUEPHY_ADV_DUPLEX_FULL:
        // Set Bit 8
        usData |= 0x0100;
        break;

    case TRUEPHY_ADV_DUPLEX_HALF:
        // Set Bit 7
        usData |= 0x0080;
        break;

    case TRUEPHY_ADV_DUPLEX_BOTH:
    default:
        // Set Bits 7,8
        usData |= 0x0180;
        break;
    }

    // Write back advertisement
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_AUTO_ADVERTISEMENT, usData );

    return;
}


void ET1310_PhyAdvertise10BaseT( ET131X_ADAPTER *pAdapter, UINT16 duplex )
{
    UINT16 usData;

    // Read the Autonegotiation Register (10/100)
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_AUTO_ADVERTISEMENT, &usData );

    // Clear bits 5,6
    usData &= ~0x0060;

    switch( duplex )
    {
    case TRUEPHY_ADV_DUPLEX_NONE:
        // Duplex already cleared, do nothing
        break;

    case TRUEPHY_ADV_DUPLEX_FULL:
        // Set Bit 6
        usData |= 0x0040;
        break;

    case TRUEPHY_ADV_DUPLEX_HALF:
        // Set Bit 5
        usData |= 0x0020;
        break;

    case TRUEPHY_ADV_DUPLEX_BOTH:
    default:
        // Set Bits 5,6
        usData |= 0x0060;
        break;
    }

    // Write back advertisement
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_AUTO_ADVERTISEMENT, usData );

    return;
}




void ET1310_PhyLinkStatus( ET131X_ADAPTER *pAdapter, 
                           UCHAR  *ucLinkStatus,
                           UINT32 *uiAutoNeg,
                           UINT32 *uiLinkSpeed,
                           UINT32 *uiDuplexMode,
                           UINT32 *uiMdiMdix,
                           UINT32 *uiMasterSlave,
                           UINT32 *uiPolarity )
{
    UINT16 usMiStatus     = 0;
    UINT16 us1000BaseT    = 0;
    UINT16 usVmiPhyStatus = 0;
    UINT16 usControl      = 0;


    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_STATUS, &usMiStatus );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_1000_STATUS, &us1000BaseT );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_PHY_STATUS, &usVmiPhyStatus );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usControl );

    if( ucLinkStatus )
    {
        *ucLinkStatus  = (unsigned char)(( usVmiPhyStatus & 0x0040 ) ? 1 : 0 );
    }

    if( uiAutoNeg )
    {
        *uiAutoNeg     = ( usControl & 0x1000 ) ? (( usVmiPhyStatus & 0x0020 ) ? TRUEPHY_ANEG_COMPLETE : TRUEPHY_ANEG_NOT_COMPLETE ) : TRUEPHY_ANEG_DISABLED;
    }

    if( uiLinkSpeed )
    {
        *uiLinkSpeed   = ( usVmiPhyStatus & 0x0300 ) >> 8;
    }

    if( uiDuplexMode )
    {
        *uiDuplexMode  = ( usVmiPhyStatus & 0x0080 ) >> 7;
    }

    if( uiMdiMdix )
    {
        /* NOTE: Need to complete this */
        *uiMdiMdix     = 0;
    }

    if( uiMasterSlave )
    {
        *uiMasterSlave = ( us1000BaseT & 0x4000 ) ? TRUEPHY_CFG_MASTER : TRUEPHY_CFG_SLAVE;
    }

    if( uiPolarity )
    {
        *uiPolarity    = ( usVmiPhyStatus & 0x0400 ) ? TRUEPHY_POLARITY_INVERTED : TRUEPHY_POLARITY_NORMAL;
    }

    return;
}


void ET1310_PhyAndOrReg( ET131X_ADAPTER *pAdapter,
                         UINT16 regnum,
                         UINT16 andMask,
                         UINT16 orMask )
{
    UINT16 reg;

    // Read the requested register
    MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
            regnum, &reg );

    // Apply the AND mask
    reg &= andMask;

    // Apply the OR mask
    reg |= orMask;

    // Write the value back to the register
    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
             regnum, reg );

    return;
}


void ET1310_PhyAccessMiBit( ET131X_ADAPTER *pAdapter,
                            UINT16 action,
                            UINT16 regnum,
                            UINT16 bitnum,
                            UINT8 *value )
{
    UINT16 reg;
    UINT16 mask = 0;

    // Create a mask to isolate the requested bit
    mask = 0x0001 << bitnum;

    // Read the requested register
    MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
            regnum, &reg );

    switch( action )
    {
    case TRUEPHY_BIT_READ:
        if( value != NULL )
        {
            *value = ( reg & mask ) >> bitnum;
        }

        break;

    case TRUEPHY_BIT_SET:
        reg |= mask;
        MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr, regnum, reg );
        break;

    case TRUEPHY_BIT_CLEAR:
        reg &= ~mask;
        MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr, regnum, reg );
        break;

    default:
        break;
    }

    return;
}

