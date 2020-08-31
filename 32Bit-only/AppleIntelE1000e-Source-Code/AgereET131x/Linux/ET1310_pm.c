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
 * ET1310_pm.c - All power management related code (not completely implemented)
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
         $Date: 2006/01/20 21:29:44 $
     $Revision: 1.8 $
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
#include "ET1310_rx.h"

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
   ROUTINE:  CalculateCCITCRC16
 ******************************************************************************
   DESCRIPTION:
        This routine calculates the CCIT CRC-16 value required for power
        management.

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
UINT16 CalculateCCITCRC16( PUCHAR Pattern, PUCHAR Mask, UINT32 MaskSize )
{
    UINT32      i, j, k, HitBit;
    UINT16      RemainderCRC = 0xFFFF;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "CalculateCCITCRC16" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       There is one bit in the mask for every byte in the pattern.  Therefore,
       this first loop (the "i" loop) iterates over every byte in the mask,
       the inner "j" loop iterates over every bit in each byte.  If that bit
       in the mask is clear, then we skip over this byte of packet date (the
       "continue").
     *************************************************************************/
    for( i = 0; i < MaskSize; i++ )
    {
        for( j = 0; j < 8; j++ )
        {
            if(( Mask[i] & ( 1 << j )) == 0 )
            {
                continue;
            }

            /******************************************************************
                Following 10 lines iterates over every bit in this byte of the
                pattern, and applies the CRC-16 calculation as defined by CCIT.
                It could be optimized using a look-up-table, but this function
                will be called infrequently, so the memory overhead of 
                optimization can be avoided.
             *****************************************************************/
            for( k = 0; k < 8; k++ )
            {
                HitBit = (( RemainderCRC >> 15 ) & 1 );
                HitBit ^= ( Pattern[(i*8)+j] >> k ) & 1;

                RemainderCRC <<= 1;

                if( HitBit )
                {
                    RemainderCRC ^= CRC16_POLY;
                }
            }
        }
    }

    DBG_LEAVE( et131x_dbginfo );
    return( RemainderCRC );
}

/*===========================================================================*/



#if 0
/******************************************************************************
   ROUTINE:  MPSetPowerD0
 ******************************************************************************
   DESCRIPTION:
        Used to set the power state to D0, Updated for the ET1310

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
VOID MPSetPowerD0( IN PMP_ADAPTER pAdapter )
{
    UINT32                  uPmCsr;
    NDIS_DEVICE_POWER_STATE ComingBackFrom;
    PM_CSR_t                pm_csr = pAdapter->CSRAddress->global.pm_csr;
    RXMAC_CTRL_t            ctrl;
    MI_ISR_t                isr;
    MI_BMSR_t               Bmsr, BmsrInts;
    UINT32                  delay = 0;

    /*-----------------------------------------------------------------------*/

    DBGPRINT( MP_TRACE, ( "----> MPSetPowerD0\n" ));

    /**************************************************************************
       Archive the power-state we are returning from, and then set the
       "current" power state back to fully operational
     *************************************************************************/
    ComingBackFrom              = pAdapter->PoMgmt.PowerState;
    pAdapter->PoMgmt.PowerState = NdisDeviceStateD0;


    /**********************************************************************
        Allow disabling of Non-Maskable Interrupts in I/O space, to
        support validation.
        *********************************************************************/
    if( pAdapter->RegistryNMIDisable )
    {
        UCHAR RegisterVal;

        NdisRawReadPortUchar( ET1310_NMI_DISABLE, &RegisterVal );

        RegisterVal &= 0xf3;

        if( pAdapter->RegistryNMIDisable == 2 )
        {
            RegisterVal |= 0xc;
        }

        NdisRawWritePortUchar( ET1310_NMI_DISABLE, RegisterVal );
    }

    {
        UCHAR read_size_reg;

        /******************************************************************
            Change the max read size to 2k
         *****************************************************************/
        NdisReadPciSlotInformation( pAdapter->AdapterHandle,
                                    0,          // not used
                                    0x51,
                                    (PVOID)&read_size_reg,
                                    sizeof( UCHAR ));

        read_size_reg &= 0x8f;
        read_size_reg |= 0x40;

        NdisWritePciSlotInformation( pAdapter->AdapterHandle,
                                     0,
                                     0x51,
                                     &read_size_reg,
                                     sizeof( UCHAR ));
    }


    /**************************************************************************
       Sample read of pmcsr to aid validation.  Will be catured by PCI-e
       analyzer
     *************************************************************************/
    NdisReadPciSlotInformation( pAdapter->AdapterHandle,
                            0,  // Slot no.  Reserved.  Ndis ignores.
                            ET1310_PCI_PM_CSR,
                            (PVOID)&uPmCsr,
                            sizeof( UINT32 ));

    /**************************************************************************
       we now need to set the JAGCore gating control registers for this power 
       state.  Bypass this for nonwake device since in that case the JAGCore 
	   and gigE PHY have been reset

       This is only supposeto be for wake mode, when in non-wake mode, the
       device enters L1 which causes a reset of the jagcore which basically
       clears the PM_STATE.  But due to issue with system not staying in L1 the
       hardware reset is not happening.  So we take care of it here.
     *************************************************************************/
    if( TRUE ) //pAdapter->PoMgmt.WOLEnabled )
    {
		pm_csr.bits.pm_gigephy_en  = 0;
        pm_csr.bits.pm_jagcore_rx_en = 0;
        pm_csr.bits.pm_jagcore_tx_en = 0;
        pm_csr.bits.pm_phy_lped_en  = 0;
        pm_csr.bits.pm_phy_sw_coma  = 0;

        /**********************************************************************
           enable clock first 
         *********************************************************************/
		pm_csr.bits.pm_sysclk_gate = 1;
		pm_csr.bits.pm_txclk_gate  = 1;
		pm_csr.bits.pm_rxclk_gate  = 1;

		/**********************************************************************
           tx_en and rx_en should remain the same as in power down
         *********************************************************************/
		pAdapter->CSRAddress->global.pm_csr = pm_csr;
    
		/**********************************************************************
		   poll the PMSTATE until JAGCore is in normal state
		 *********************************************************************/
        while(( pAdapter->CSRAddress->global.pm_csr.bits.pm_jagcore_rx_rdy != 0 ) && 
              ( pAdapter->CSRAddress->global.pm_csr.bits.pm_jagcore_tx_rdy != 0 ) && 
              ( delay < 100 ))
		{
			NdisStallExecution( 10 );
            delay++;
		}

        if( delay >= 100 )
        {
            pm_csr = pAdapter->CSRAddress->global.pm_csr;

            pm_csr.bits.pm_jagcore_rx_en = 0;
            pm_csr.bits.pm_jagcore_tx_en = 0;
            pAdapter->CSRAddress->global.pm_csr.value = pm_csr.value;
        }
	}

	/**************************************************************************
	   now lets set up JAGcore pm control for D0
	 *************************************************************************/
	pm_csr.bits.pm_gigephy_en  = 0;

    pm_csr.bits.pm_jagcore_rx_en = pAdapter->PoMgmt.rx_en;
    pm_csr.bits.pm_jagcore_tx_en = pAdapter->PoMgmt.tx_en;

	pAdapter->CSRAddress->global.pm_csr = pm_csr;

    /**************************************************************************
		we also need to read and clear the phy_interrupt register in case of 
        wakeup on link status change
	*************************************************************************/
    MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr, 
            (UINT8)FIELD_OFFSET( MI_REGS_t, isr ), &isr.value );

    /**************************************************************************
       disable WOL
     *************************************************************************/
    ctrl   = pAdapter->CSRAddress->rxmac.ctrl;

    ctrl.bits.wol_disable   = 1;
    ctrl.bits.mif_disable   = 0;
    ctrl.bits.async_disable = 0;

    pAdapter->CSRAddress->rxmac.ctrl = ctrl;

	/**************************************************************************
       Set the power state to 0 (d0), and flip the PME status pin.  Clear the
       PME Enable pin, if it were set.
     *************************************************************************/
    uPmCsr = 0x00008000;
    NdisWritePciSlotInformation( pAdapter->AdapterHandle, 
                                 0, // Slot no.  Reserved.  Ndis ignores.
                                 ET1310_PCI_PM_CSR,
                                 &uPmCsr,
                                 sizeof( UINT32 ));

    /**************************************************************************
       When the adapter is placed in D3 wake, it is set in 10 Base-T
       half-duplex mode.  However, we archived the speed and duplex settings
       that were current prior to that, so restore them now and then tell
       the phy to do it's thing.
     *************************************************************************/
    if(( ComingBackFrom == NdisDeviceStateD3 ) || 
       ( ComingBackFrom == NdisDeviceStateD1 && !pAdapter->PoMgmt.WOLEnabledByCurrentState ))
    {
        pAdapter->AiForceSpeed = pAdapter->PoMgmt.PowerDownSpeed;
        pAdapter->AiForceDpx   = pAdapter->PoMgmt.PowerDownDuplex;

        /**********************************************************************
		    Re-initialize the send structures
         **********************************************************************/
	    MPInitSend( pAdapter );

	    /**********************************************************************
		    Reset the RFD list and re-start RU 
         *********************************************************************/
	    NICResetRecv( pAdapter );

	    /**********************************************************************
		    Bring the device back to the state it was during init prior
            to autonegotiation being complete.  This way, when we get the
            auto-neg complete interrupt, we can complete init by calling
            ConfigMacREGS2.
         *********************************************************************/
	    HwSoftwareReset( pAdapter );

        /**********************************************************************
            setup et1310 as per the documentation
         *********************************************************************/
        NICSetAdapterUp( pAdapter );
    }

    /**************************************************************************
        Allow Tx to restart 
     *************************************************************************/
    MP_CLEAR_FLAG( pAdapter, fMP_ADAPTER_LOWER_POWER );

    /**************************************************************************
       Check to see if the link status bit is set in the BMSR register, if it
       is not, this means we had a cable pull when at a lower power state and 
       need to indicate that we are disconnected to NDIS
     *************************************************************************/
    MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr, 
            (UINT8)FIELD_OFFSET( MI_REGS_t, bmsr ), &Bmsr.value );

    if( !Bmsr.bits.link_status )
    {

        pAdapter->Bmsr.value = Bmsr.value;

        NdisAcquireSpinLock( &pAdapter->Lock );
        MP_SET_FLAG( pAdapter, fMP_ADAPTER_LINK_DETECTION );
        pAdapter->MediaState = 0;

        NdisReleaseSpinLock( &pAdapter->Lock );

        // get timer going again...
        NdisMSetPeriodicTimer( &pAdapter->PeriodicTimer, 
                               TX_ERROR_PERIOD );
        
        // setup for possible disconnect
        NdisMSetTimer( &pAdapter->LinkDetectionTimer, 
                               LINK_DETECTION_TIMER );

        pAdapter->bLinkTimerActive = TRUE;

        if( pAdapter->RegistryPhyComa )
        {
            MPSetPhyComa( pAdapter );
        }
        /**********************************************************************
           In the case of link-in before sleep, link-out after sleep, we need
           to re-start the start-of-day timer so that we eventually flop
           into PhyComa.  In this case, the above state change gets overwritten
           since we get a PHY interrupt almost straight away, and PHY interrupt
           handling pulls us out of PHY coma mode.
         *********************************************************************/
        pAdapter->PoMgmt.TransPhyComaModeOnBoot = 0;
    }

    /**************************************************************************
       we need to enable interrupts
     *************************************************************************/
    NdisMSynchronizeWithInterrupt( &pAdapter->Interrupt,
                                (PVOID)NICEnableInterrupts,
                                pAdapter );

    DBGPRINT( MP_TRACE, ( "<---- MPSetPowerD0\n" ));
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  MPSetLowPower
 ******************************************************************************
   DESCRIPTION:
        This routine is called when the adapter receives a SetPower to any
        state other than d0.  Most of the common stuff for setting to a lower
        power state should be handled in here.

   PARAMETERS :
        pAdapter   - pointer to our adapter structure
        PowerState - NewPowerState

   RETURNS    :
        NONE

 *****************************************************************************/
VOID MPSetLowPower( IN PMP_ADAPTER pAdapter, 
                    IN NDIS_DEVICE_POWER_STATE PowerState )
{
    PM_CSR_t     GlobalPmCSR;
    RXMAC_CTRL_t ctrl;
    MI_ISR_t     isr;
    MI_BMSR_t    lBmsr;
    BOOLEAN     bDummy;
    /*-----------------------------------------------------------------------*/

    /**************************************************************************
       if we are in coma mode when we get this request, we need to disable it
     *************************************************************************/
    if( pAdapter->CSRAddress->global.pm_csr.bits.pm_phy_sw_coma == 1 )
    {
        /**********************************************************************
           check to see if we are in coma mode and if so, disable it because 
           we will not be able to read phy values until we are out.
         *********************************************************************/
        MPDisablePhyComa( pAdapter );
    }

    /**************************************************************************
        interrupts will be queued until after this routine completes.  May as
        well disable them now, since we do not want any of the queued interrupts.
     *************************************************************************/
    NdisMSynchronizeWithInterrupt( &pAdapter->Interrupt,
                                (PVOID)NICDisableInterrupts,
                                pAdapter );

    /**************************************************************************
       Set the adapter power state to requested, lower state
     *************************************************************************/
    pAdapter->PoMgmt.PowerState = PowerState;

    /**************************************************************************
       Cancel timers - start them again when power restored
     *************************************************************************/
    NdisMCancelTimer( &pAdapter->PeriodicTimer, &bDummy );

    if( pAdapter->bLinkTimerActive == TRUE ){
        NdisMCancelTimer( &pAdapter->LinkDetectionTimer, &bDummy );
    }

    /**************************************************************************
       Save the GbE PHY speed and duplex modes 
		
       Need to restore this for: D1 nonwake, D3 wake and nonwake modes
	 *************************************************************************/
	pAdapter->PoMgmt.PowerDownSpeed  = pAdapter->AiForceSpeed;
    pAdapter->PoMgmt.PowerDownDuplex = pAdapter->AiForceDpx;

    NdisAcquireSpinLock( &pAdapter->SendHWLock );

    /**************************************************************************
       Stop sending packets. 
     *************************************************************************/
    MP_SET_FLAG( pAdapter, fMP_ADAPTER_LOWER_POWER );

    NdisReleaseSpinLock( &pAdapter->SendHWLock );

    /**************************************************************************
		Free the packets being actively sent & stopped
    **************************************************************************/
	MpFreeBusySendPackets( pAdapter );

    /**************************************************************************
       Save Rx/Tx enable condition.  Used during restore to D0 state.
     *************************************************************************/
    pAdapter->PoMgmt.tx_en = pAdapter->CSRAddress->global.pm_csr.bits.pm_jagcore_tx_en;
    pAdapter->PoMgmt.rx_en = pAdapter->CSRAddress->global.pm_csr.bits.pm_jagcore_rx_en;

    /**************************************************************************
       set the JAGCore gating control registers for this power state
     *************************************************************************/
    GlobalPmCSR = pAdapter->CSRAddress->global.pm_csr;

    if( pAdapter->PoMgmt.WOLEnabledByCurrentState )
    {
        GlobalPmCSR.bits.pm_gigephy_en = 0;
    }
    else
    {
        GlobalPmCSR.bits.pm_gigephy_en = 1;
    }
    
    /**************************************************************************
       only exercise the PM state machine when the link is up
     *************************************************************************/
	if( pAdapter->Bmsr.bits.link_status )
	{
        GlobalPmCSR.bits.pm_jagcore_tx_en = 1;
        GlobalPmCSR.bits.pm_jagcore_rx_en = 1;

	}
    pAdapter->CSRAddress->global.pm_csr = GlobalPmCSR;

    ctrl = pAdapter->CSRAddress->rxmac.ctrl;

    /**************************************************************************
       only exercise the PM state machine when the link is up
     *************************************************************************/
	if( pAdapter->Bmsr.bits.link_status )
	{
        /**********************************************************************
           disable MIF so power state can transition 
         *********************************************************************/
        ctrl.bits.mif_disable            = 1;
	    pAdapter->CSRAddress->rxmac.ctrl = ctrl;

        while(( pAdapter->CSRAddress->global.pm_csr.bits.pm_jagcore_rx_rdy != 1 )
           && ( pAdapter->CSRAddress->global.pm_csr.bits.pm_jagcore_tx_rdy != 1 ))
		{
			NdisStallExecution( 10 );
		}
	}

    /**************************************************************************
        Stop hardware from receiving packets - Set the RU to idle
     *************************************************************************/
    NICRxDmaDisable( pAdapter );

    /**************************************************************************
        Wait for outstanding Receive packets
     *************************************************************************/
    while( MP_GET_RCV_REF( pAdapter ) != 0 )
    {
        /**********************************************************************
            Sleep for 2 Ms;
         *********************************************************************/
        NdisStallExecution( 2000 );
    }

    /**************************************************************************
       D3Wake only - set the MAC and Phy to 10BaseT half-duplex operation.
       Archive the current settings in the pAdapter structure so we can
       restore them when done.
     *************************************************************************/
    if(( PowerState == NdisDeviceStateD3 ) && pAdapter->PoMgmt.WOLEnabledByCurrentState &&
        pAdapter->Bmsr.bits.link_status )
    {
        /**********************************************************************
           initialize variable for testing if we failed to go to specific link
         *********************************************************************/
        pAdapter->PoMgmt.Failed10Half = FALSE;

        /**********************************************************************
           set the phy properly
         *********************************************************************/
        pAdapter->AiForceSpeed = 10;
        pAdapter->AiForceDpx   = 1;
        
        TPAL_SetPhy10HalfDuplex( pAdapter );

        /**********************************************************************
           get the link status here, we need to wait until the link comes back 
           after reconfiguring it.
         *********************************************************************/
        TPAL_GetLinkStatusInfo( pAdapter );

        if( pAdapter->PoMgmt.Failed10Half )
        {
            /******************************************************************
               re-initialize variable for testing if we failed to go to 
               specific link speed
             *****************************************************************/
            pAdapter->PoMgmt.Failed10Half = FALSE;

            /******************************************************************
               set the phy properly
             *****************************************************************/
            pAdapter->AiForceSpeed = 100;
            pAdapter->AiForceDpx   = 1;
            
            TPAL_SetPhy100HalfDuplex( pAdapter );

            /******************************************************************
               get the link status here, we need to wait until the link comes 
               back after reconfiguring it.
             *****************************************************************/
            TPAL_GetLinkStatusInfo( pAdapter );
        }

        // SetRxDmaTimer( pAdapter );
        ConfigMACRegs2( pAdapter );
    }

    if( pAdapter->PoMgmt.WOLEnabledByCurrentState )
    {
        RXMAC_WOL_CTL_CRC0_t    rxmac_ctl_crc0 = pAdapter->CSRAddress->rxmac.crc0;
        RXMAC_WOL_CRC12_t       crc12;
        RXMAC_WOL_CRC34_t       crc34;
        ULONG               ulResult;
        UINT32              SerdesPhyControl;
        UINT                    mask;

        ulResult = NdisReadPciSlotInformation( pAdapter->AdapterHandle,
                                    0,          // not used
                                    ET1310_PCI_PHY_INDEX_REG,
                                    (PVOID)&SerdesPhyControl,
                                    sizeof( UINT32 ));

        if( pAdapter->RegistryPMWOL )
        {
            SerdesPhyControl |= 0x00010000;
        }
        else
        {
            SerdesPhyControl &= 0xfffeffff;
        }

        ulResult = NdisWritePciSlotInformation( 
                                pAdapter->AdapterHandle,
                                0,
                                ET1310_PCI_PHY_INDEX_REG,
                                (PVOID)&SerdesPhyControl,
                                sizeof( UINT32 ));

        if( pAdapter->Bmsr.bits.link_status )
        {
            /**********************************************************************
                need to do this for all wake devices
                make sure all ignore bits are set
             *********************************************************************/
            rxmac_ctl_crc0.bits.ignore_broad       = 1;
            rxmac_ctl_crc0.bits.ignore_multi       = 1;
            rxmac_ctl_crc0.bits.ignore_uni         = 1;

            if( pAdapter->RegistryWOLMatch & 0x1 )
            {
                rxmac_ctl_crc0.bits.ignore_mp          = 0;
            }
            else
            {
                rxmac_ctl_crc0.bits.ignore_mp          = 1;
            }

            if( pAdapter->RegistryWOLMatch & 0x2 )
            {
                rxmac_ctl_crc0.bits.ignore_pp          = 0;
            }
            else
            {
                rxmac_ctl_crc0.bits.ignore_pp          = 1;
            }

            if( pAdapter->RegistryWOLLink & 0x1 )
            {
                rxmac_ctl_crc0.bits.ignore_link_chg    = 0;
            }
            else
            {
                rxmac_ctl_crc0.bits.ignore_link_chg    = 1;
            }

            /******************************************************************
                Clear the pattern match validity bits - i.e. make all patterns
                invalid
             *****************************************************************/
            rxmac_ctl_crc0.value &= ~0x1f;

            if(( pAdapter->RegistryWOLMatch & 0x2 ) &&
                ( pAdapter->PoMgmt.localWolAndCrc0 & 0x1f ))
            {
                /**************************************************************
                    set the validity bits based on what has been enabled in
                    the adapter
                 *************************************************************/
                rxmac_ctl_crc0.value |= (pAdapter->PoMgmt.localWolAndCrc0 & 0x1f);

                /**************************************************************
                    Copy the five CRCs from the adapter
                 *************************************************************/
                rxmac_ctl_crc0.bits.crc0  = pAdapter->PoMgmt.WOLPatternList [0];
                crc12.bits.crc1 = pAdapter->PoMgmt.WOLPatternList [1];
                crc12.bits.crc2 = pAdapter->PoMgmt.WOLPatternList [2];
                crc34.bits.crc3 = pAdapter->PoMgmt.WOLPatternList [3];
                crc34.bits.crc4 = pAdapter->PoMgmt.WOLPatternList [4];

                for( mask = 0; mask < 5; mask++)
                {
                    UINT        i;
                    PUINT32     pDevicePatternMask = (PUINT32)
                        (&pAdapter->CSRAddress->rxmac.mask0_word0) +
                                                        (mask * MAX_WOL_MASK_SIZE / 4);
                    if(( pAdapter->PoMgmt.localWolAndCrc0 >> mask ) & 0x1 )
                    {
                        UINT32      Temp[ MAX_WOL_MASK_SIZE / 4 ];
                        PUCHAR      pNdisPatternMask;

                        pNdisPatternMask = (PUCHAR)&pAdapter->PoMgmt.WOLMaskList[ mask ][ 0 ];

                        NdisZeroMemory( Temp, sizeof( Temp ));
                        NdisMoveMemory( Temp, pNdisPatternMask,
                                        pAdapter->PoMgmt.WOLMaskSize[ mask ] );

                        /**************************************************************
                            Write the mask to the device using the pointer calculated
                            above.
                        *************************************************************/
                        for( i=0; i<( MAX_WOL_MASK_SIZE / 4 ); i++ )
                        {
                            pDevicePatternMask[ i ] = Temp[ i ];
                        }
                    }
                }
            }

            pAdapter->CSRAddress->rxmac.crc0 = rxmac_ctl_crc0;
            pAdapter->CSRAddress->rxmac.crc12 = crc12;
            pAdapter->CSRAddress->rxmac.crc34 = crc34;

            /******************************************************************
		        we also need to read and clear the phy_interrupt register in
                case of wakeup on link status change
	         *****************************************************************/
            MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr, 
                    (UINT8)FIELD_OFFSET( MI_REGS_t, isr ), &isr.value );

            MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr, 
                    (UINT8)FIELD_OFFSET( MI_REGS_t, bmsr ), &lBmsr.value );

            /******************************************************************
                disable mif and async
             *****************************************************************/
		    ctrl.bits.mif_disable            = 1;
		    ctrl.bits.async_disable          = 1;		
            ctrl.bits.wol_disable            = 0;
            pAdapter->CSRAddress->rxmac.ctrl = ctrl;

            rxmac_ctl_crc0.bits.clr_intr                = 1;
            pAdapter->CSRAddress->rxmac.crc0.value = rxmac_ctl_crc0.value;

            DBGPRINT( MP_SPEC, ("Going down - ctrl 0x%x, crc0_ctl 0x%x\n",
                pAdapter->CSRAddress->rxmac.ctrl.value,
                pAdapter->CSRAddress->rxmac.crc0.value ));

            pAdapter->Stats.InterruptStatus.value = pAdapter->CSRAddress->global.int_status.value;

            DBGPRINT( MP_SPEC, ("Going down - int stat was 0x%x, now 0x%x\n",
                pAdapter->Stats.InterruptStatus.value,
                pAdapter->CSRAddress->global.int_status.value ));
        }
        else if( pAdapter->RegistryWOLLink & 0x1 )
        {
            GlobalPmCSR = pAdapter->CSRAddress->global.pm_csr;

	        /**************************************************************************
                Gate off JAGCore 3 clock domains
            *************************************************************************/
	        GlobalPmCSR.bits.pm_sysclk_gate     = 0;
            GlobalPmCSR.bits.pm_txclk_gate      = 0;
            GlobalPmCSR.bits.pm_rxclk_gate      = 0;
            pAdapter->CSRAddress->global.pm_csr = GlobalPmCSR;

	        /**************************************************************************
                Program gigE PHY in to Coma mode
            *************************************************************************/
	        GlobalPmCSR.bits.pm_phy_sw_coma     = 1;
            GlobalPmCSR.bits.pm_phy_lped_en     = 1;

            pAdapter->CSRAddress->global.pm_csr = GlobalPmCSR;
        }
    }
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  MPSetPowerD1Wake
 ******************************************************************************
   DESCRIPTION:
        This routine is called when the adapter receives a SetPower to  
        D1 PowerState, and there is a set of wake-up patterns programmed
        in the adapter.  The set could consist of a single wake-up pattern.

   PARAMETERS :
        pAdapter   - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
VOID MPSetPowerD1Wake( PMP_ADAPTER pAdapter )
{
    UINT32        uPmCsr;
    PM_CSR_t     pm_csr;
    /*-----------------------------------------------------------------------*/

    DBGPRINT( MP_TRACE, ( "----> MPSetPowerD1Wake\n" ));

    /**************************************************************************
        common power down functionality; disable Tx & Rx, disable wol while
        it is being provisioned, archive Tx and Rx enable, and do some of the
        gating into jagcore.
     *************************************************************************/
    MPSetLowPower( pAdapter, NdisDeviceStateD1 );

    if((( pAdapter->RegistryWOLLink & 0x1) == 0 ) || ( pAdapter->Bmsr.bits.link_status ))
    {
        /**********************************************************************
        gate off paths we do not want active and leave ones we do
         *********************************************************************/
        pm_csr = pAdapter->CSRAddress->global.pm_csr;

    pm_csr.bits.pm_sysclk_gate = 0;
    pm_csr.bits.pm_txclk_gate  = 0;
    pm_csr.bits.pm_rxclk_gate  = 1;

    pAdapter->CSRAddress->global.pm_csr = pm_csr;
    }

    /**************************************************************************
       Set the power state to 1 (d1), and blip the PME status pin.  Set the
       PME Enable pin, which indicates wake state to PCI.
     *************************************************************************/
    uPmCsr = 0x00008100;
    NdisWritePciSlotInformation( pAdapter->AdapterHandle,
                                 0, // Slot no.  Reserved.  Ndis ignores.
                                 ET1310_PCI_PM_CSR,
                                 &uPmCsr,
                                 sizeof( UINT32 ));

    DBGPRINT( MP_TRACE, ( "<---- MPSetPowerD1Wake\n" ));
    return;
}
/*===========================================================================*/





/******************************************************************************
   ROUTINE:  MPSetPowerD1NonWake
 ******************************************************************************
   DESCRIPTION:
        This routine is called when the adapter receives a SetPower to  
        D1 PowerState, and there are no wake-up patterns programmed
        in the adapter. 

   PARAMETERS :
        pAdapter   - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
VOID MPSetPowerD1NonWake( PMP_ADAPTER pAdapter )
{
    UINT32   uPmCsr;
    PM_CSR_t pm_csr;
    /*-----------------------------------------------------------------------*/

    DBGPRINT( MP_TRACE, ( "----> MPSetPowerD1NonWake\n" ));

    /**************************************************************************
        common power down functionality; disable Tx & Rx, disable wol,
        archive Tx and Rx enable, and do some of the gating into jagcore.
     *************************************************************************/
	MPSetLowPower( pAdapter, NdisDeviceStateD1 );

    /**************************************************************************
        gate off paths we do not want active and leave ones we do
     *************************************************************************/
    pm_csr = pAdapter->CSRAddress->global.pm_csr;

    pm_csr.bits.pm_sysclk_gate = 1;
    pm_csr.bits.pm_txclk_gate  = 1;
    pm_csr.bits.pm_rxclk_gate  = 1;

    pAdapter->CSRAddress->global.pm_csr = pm_csr;

    /**************************************************************************
       Set the power state to 1 (d1), and blip the PME status pin.  Leave the
       PME Enable pin clear, which indicates non-wake state to PCI.
     *************************************************************************/
    uPmCsr = 0x00008000;
    NdisWritePciSlotInformation( pAdapter->AdapterHandle,
                                 0, // Slot no.  Reserved.  Ndis ignores.
                                 ET1310_PCI_PM_CSR,
                                 &uPmCsr,
                                 sizeof( UINT32 ));

    DBGPRINT( MP_TRACE, ( "<---- MPSetPowerD1NonWake\n" ));
    return;
}
/*===========================================================================*/





/******************************************************************************
   ROUTINE:  MPSetPowerD3Wake
 ******************************************************************************
   DESCRIPTION:
        This routine is called when the adapter receives a SetPower to  
        D3 PowerState, and there is a set of wake-up patterns programmed
        in the adapter.  The set could consist of a single wake-up pattern.

   PARAMETERS :
        pAdapter   - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
VOID MPSetPowerD3Wake( PMP_ADAPTER pAdapter )
{
    UINT32       uPmCsr;
    PM_CSR_t     pm_csr;
    /*-----------------------------------------------------------------------*/

    DBGPRINT( MP_TRACE, ( "----> MPSetPowerD3Wake\n" ));

    /**************************************************************************
        common power down functionality; disable Tx & Rx, disable wol while
        it is being provisioned, archive Tx and Rx enable, and do some of the
        gating into jagcore.
     *************************************************************************/
    MPSetLowPower( pAdapter, NdisDeviceStateD3 );

    if((( pAdapter->RegistryWOLLink & 0x1) == 0 ) || ( pAdapter->Bmsr.bits.link_status ))
    {
        /**********************************************************************
           gate off paths we do not want active and leave ones we do
         *********************************************************************/
        pm_csr = pAdapter->CSRAddress->global.pm_csr;

        pm_csr.bits.pm_sysclk_gate = 0;
        pm_csr.bits.pm_txclk_gate  = 0;
        pm_csr.bits.pm_rxclk_gate  = 1;

        pAdapter->CSRAddress->global.pm_csr = pm_csr;
    }

    /**************************************************************************
       Set the power state to 3 (d3), and blip the PME status pin.  Set the
       PME Enable pin, which indicates wake state to PCI.
     *************************************************************************/
    uPmCsr = 0x00008100;
    NdisWritePciSlotInformation( pAdapter->AdapterHandle,
                                 0, // Slot no.  Reserved.  Ndis ignores.
                                 ET1310_PCI_PM_CSR,
                                 &uPmCsr,
                                 sizeof( UINT32 ));

    DBGPRINT( MP_TRACE, ( "<---- MPSetPowerD3Wake\n" ));
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  MPSetPowerD3NonWake
 ******************************************************************************
   DESCRIPTION:
        This routine is called when the adapter receives a SetPower to  
        D3 PowerState, and there are no wake-up patterns programmed
        in the adapter.

   PARAMETERS :
        pAdapter   - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
VOID MPSetPowerD3NonWake( PMP_ADAPTER pAdapter )
{
    UINT32   uPmCsr;
    PM_CSR_t pm_csr;
    /*-----------------------------------------------------------------------*/

    DBGPRINT( MP_TRACE, ( "----> MPSetPowerD3NonWake\n" ));

    /**************************************************************************
        common power down functionality; disable Tx & Rx, disable wol, 
        archive Tx and Rx enable, and do some of the gating into jagcore.
     *************************************************************************/
    MPSetLowPower( pAdapter, NdisDeviceStateD3 );

    /**************************************************************************
        gate off paths we do not want active and leave ones we do
     *************************************************************************/
    pm_csr = pAdapter->CSRAddress->global.pm_csr;
    pm_csr.bits.pm_sysclk_gate = 1;
    pm_csr.bits.pm_txclk_gate  = 1;
    pm_csr.bits.pm_rxclk_gate  = 1;

    pAdapter->CSRAddress->global.pm_csr = pm_csr;

    /**************************************************************************
       Set the power state to 3 (d3), and blip the PME status pin.  Leave the
       PME Enable pin clear, which indicates non-wake state to PCI.
     *************************************************************************/
    uPmCsr = 0x00008000;
    NdisWritePciSlotInformation( pAdapter->AdapterHandle,
                                 0, // Slot no.  Reserved.  Ndis ignores.
                                 ET1310_PCI_PM_CSR,
                                 &uPmCsr,
                                 sizeof( UINT32 ));

    DBGPRINT( MP_TRACE, ( "<---- MPSetPowerD3NonWake\n" ));

    return;
}
/*===========================================================================*/

#endif


/******************************************************************************
   ROUTINE:  EnablePhyComa
 ******************************************************************************
   DESCRIPTION:
        This routine is called when network cable is unplugged -- driver 
        receive an phy status change interrupt while in D0 and check that 
        phy_status is down. 
		
            -- gate off JAGCore; 
            -- set gigE PHY in Coma mode
            -- wake on phy_interrupt; Perform software reset JAGCore, 
               re-initialize jagcore and gigE PHY

        Add D0-ASPM-PhyLinkDown Support:
            -- while in D0, when there is a phy_interrupt indicating phy link 
               down status, call the MPSetPhyComa routine to enter this active 
               state power saving mode
            -- while in D0-ASPM-PhyLinkDown mode, when there is a phy_interrupt
		       indicating linkup status, call the MPDisablePhyComa routine to 
               restore JAGCore and gigE PHY

   PARAMETERS :
        pAdapter   - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void EnablePhyComa( ET131X_ADAPTER *pAdapter )
{
    unsigned long lockflags;
    PM_CSR_t      GlobalPmCSR = pAdapter->CSRAddress->global.pm_csr;
    INT32         LoopCounter = 10;
    /*-----------------------------------------------------------------------*/

    
    DBG_FUNC( "EnablePhyComa" );
    DBG_ENTER( et131x_dbginfo );


	/**************************************************************************
	   Save the GbE PHY speed and duplex modes 
	   Need to restore this when cable is plugged back in
	 *************************************************************************/
	pAdapter->PoMgmt.PowerDownSpeed  = pAdapter->AiForceSpeed;
    pAdapter->PoMgmt.PowerDownDuplex = pAdapter->AiForceDpx;


    /**************************************************************************
       Stop sending packets. 
     *************************************************************************/
    spin_lock_irqsave( &pAdapter->SendHWLock, lockflags );

    MP_SET_FLAG( pAdapter, fMP_ADAPTER_LOWER_POWER );

    spin_unlock_irqrestore( &pAdapter->SendHWLock, lockflags );


	/**************************************************************************
       Wait for outstanding Receive packets
     *************************************************************************/
    while(( MP_GET_RCV_REF( pAdapter ) != 0 ) && ( LoopCounter-- > 0 ))
    {
        /**********************************************************************
           Sleep for 2 Ms;
         *********************************************************************/
        mdelay( 2 );
    }


	/**************************************************************************
       Gate off JAGCore 3 clock domains
     *************************************************************************/
	GlobalPmCSR.bits.pm_sysclk_gate     = 0;
    GlobalPmCSR.bits.pm_txclk_gate      = 0;
    GlobalPmCSR.bits.pm_rxclk_gate      = 0;
    pAdapter->CSRAddress->global.pm_csr = GlobalPmCSR;


	/**************************************************************************
       Program gigE PHY in to Coma mode
     *************************************************************************/
	GlobalPmCSR.bits.pm_phy_sw_coma     = 1;

    pAdapter->CSRAddress->global.pm_csr = GlobalPmCSR;


	DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  DisablePhyComa
 ******************************************************************************
   DESCRIPTION:
        This routine is used to disable the Phy Coma Mode

   PARAMETERS :
        pAdapter   - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void DisablePhyComa( ET131X_ADAPTER *pAdapter )
{
    PM_CSR_t     GlobalPmCSR = pAdapter->CSRAddress->global.pm_csr;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "DisablePhyComa" );
    DBG_ENTER( et131x_dbginfo );


    /*************************************************************************
       Disable phy_sw_coma register and re-enable JAGCore clocks
     ************************************************************************/
    GlobalPmCSR.bits.pm_sysclk_gate     = 1;
    GlobalPmCSR.bits.pm_txclk_gate      = 1;
    GlobalPmCSR.bits.pm_rxclk_gate      = 1;
    GlobalPmCSR.bits.pm_phy_sw_coma     = 0;

    pAdapter->CSRAddress->global.pm_csr = GlobalPmCSR;


    /**************************************************************************
       Restore the GbE PHY speed and duplex modes; 
       Reset JAGCore; re-configure and initialize JAGCore and gigE PHY
     *************************************************************************/
    pAdapter->AiForceSpeed = pAdapter->PoMgmt.PowerDownSpeed;
    pAdapter->AiForceDpx   = pAdapter->PoMgmt.PowerDownDuplex;


    /**************************************************************************
       Re-initialize the send structures
     *************************************************************************/
    et131x_init_send( pAdapter );


    /**************************************************************************
       Reset the RFD list and re-start RU 
     *************************************************************************/
    et131x_reset_recv( pAdapter );


    /**************************************************************************
       Bring the device back to the state it was during init prior to 
       autonegotiation being complete.  This way, when we get the auto-neg 
       complete interrupt, we can complete init by calling ConfigMacREGS2.
     *************************************************************************/
    et131x_soft_reset( pAdapter );


    /**************************************************************************
       setup et1310 as per the documentation ??
     *************************************************************************/
    et131x_adapter_setup( pAdapter );


    /**************************************************************************
        Allow Tx to restart 
     *************************************************************************/
    MP_CLEAR_FLAG( pAdapter, fMP_ADAPTER_LOWER_POWER );


    /**************************************************************************
       Need to re-enable Rx. 
     *************************************************************************/
    et131x_rx_dma_enable( pAdapter );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/



#if 0
/******************************************************************************
   ROUTINE:  MPSetPower
 ******************************************************************************

   DESCRIPTION:
        This routine is called when the adapter receives a SetPower 
        request. It redirects the call to an appropriate routine to
        Set the New PowerState

   PARAMETERS :
         pAdapter   - pointer to the adapter structure
         PowerState - NewPowerState

   RETURN     :
        NONE

 *****************************************************************************/
VOID MPSetPower( IN PMP_ADAPTER             pAdapter, 
                 IN NDIS_DEVICE_POWER_STATE PowerState )
{
    DBGPRINT( MP_TRACE, ( "====> MPSetPower()\n" ));

    /**************************************************************************
       determine what power state we are going to and go there
     *************************************************************************/
    pAdapter->PoMgmt.WOLEnabledByCurrentState = TRUE;

    if( !pAdapter->PoMgmt.WOLEnabledByNdis )
    {
        pAdapter->PoMgmt.WOLEnabledByCurrentState = FALSE;
    }

    if(( pAdapter->RegistryWOLMatch == 0 ) && ( pAdapter->RegistryWOLLink == 0 ))
    {
        pAdapter->PoMgmt.WOLEnabledByCurrentState = FALSE;
    }

    if(( pAdapter->PoMgmt.localWolAndCrc0 & 0x1f ) == 0 )
    {
        /**********************************************************************
            There are no wake patterns programmed.
         *********************************************************************/
        if( pAdapter->RegistryWOLLink == 0 )
        {
            pAdapter->PoMgmt.WOLEnabledByCurrentState = FALSE;
        }
    }

    /**************************************************************************
       If the link is pulled, then the only wake event that will wake us
       up is wake on link state change.  If this change is disabled (0), go
       to a non-wake state.
     *************************************************************************/
    if( !pAdapter->ucLinkStatus && ( pAdapter->RegistryWOLLink == 0 ))
    {
        pAdapter->PoMgmt.WOLEnabledByCurrentState = 0;
    }

    if( PowerState == NdisDeviceStateD0 )
    {
        MPSetPowerD0( pAdapter );
    }
    else if( PowerState == NdisDeviceStateD1 )
    {
        if( pAdapter->PoMgmt.WOLEnabledByCurrentState )
        {
            MPSetPowerD1Wake( pAdapter );
        }
        else
        {
            MPSetPowerD1NonWake( pAdapter );
        }
    }
    else if( PowerState == NdisDeviceStateD3 )
    {
        if( pAdapter->PoMgmt.WOLEnabledByCurrentState )
        {
            MPSetPowerD3Wake( pAdapter );
        }
        else
        {
            MPSetPowerD3NonWake( pAdapter );
        }
    }

    DBGPRINT( MP_TRACE, ( "<==== MPSetPower()\n" ));
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  MPFillPoMgmtCaps
 ******************************************************************************

   DESCRIPTION:
        Fills in the Power Managment structure depending the capabilities of 
        the software driver and the card.  MGS CURRENTLY NOT SUPPORTED


   PARAMETERS :
         pAdapter       - pointer to the adapter structure
         pPowManageCaps - power management struct as defined in the DDK, 
         pStatus        - status to be returned by the request,
         pulInfoLen     - length of the pPowerManagmentCapabilites

   RETURN     :
        Success or failure 

   NOTE:
        If the driver returns NDIS_STATUS_SUCCESS in response to a query of 
        this OID, NDIS treats a miniport driver as power management-aware. 
        If the driver returns NDIS_STATUS_NOT_SUPPORTED, NDIS treats the 
        miniport driver as a legacy driver that is not power 
        management-aware. 

        The Bus driver is the power policy owner here so we do not have to 
        perform our mapping of System States to Device states, that is 
        accomplished by NDIS, we need to worry more about our wake-up
        capabilities

 *****************************************************************************/
VOID MPFillPoMgmtCaps( IN     PMP_ADAPTER             pAdapter, 
                       IN OUT PNDIS_PNP_CAPABILITIES  pPowMgmtCaps, 
                       IN OUT PNDIS_STATUS            pStatus,
                       IN OUT PULONG                  pulInfoLen )
{
    BOOLEAN bIsPoMgmtSupported = TRUE; 
    /*-----------------------------------------------------------------------*/

    bIsPoMgmtSupported = MPIsPoMgmtSupported( pAdapter );

    if( bIsPoMgmtSupported )
    {
        /**********************************************************************
           NDIS_DEVICE_WAKE_UP_ENABLE;
         *********************************************************************/
        pPowMgmtCaps->Flags                                   = 0;

        /**********************************************************************
           Magic Packet wakeups
         *********************************************************************/
        pPowMgmtCaps->WakeUpCapabilities.MinMagicPacketWakeUp = 
                                                             NdisDeviceStateD3;
        
        /**********************************************************************
           NdisDeviceStateD3;
         *********************************************************************/
        pPowMgmtCaps->WakeUpCapabilities.MinPatternWakeUp     = 
                                                             NdisDeviceStateD3; 
        pPowMgmtCaps->WakeUpCapabilities.MinLinkChangeWakeUp  = 
                                                             NdisDeviceStateD3;
        
        *pulInfoLen = sizeof( *pPowMgmtCaps );
        *pStatus    = NDIS_STATUS_SUCCESS;
    }
    else
    {
        NdisZeroMemory( pPowMgmtCaps, sizeof( *pPowMgmtCaps ));
        
        *pStatus    = NDIS_STATUS_NOT_SUPPORTED;
        *pulInfoLen = 0;
    }
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  MPAddWakeUpPattern
 ******************************************************************************

   DESCRIPTION:
        This routine will allocate a local memory structure, copy the pattern, 
        insert the pattern into a linked list and return success

        We are gauranteed that we wll get only one request at a time, so this 
        is implemented without locks.

   PARAMETERS :
         pAdapter                - pointer to the adapter structure
         InformationBuffer       - Wake up Pattern
         InformationBufferLength - Wake Up Pattern Length

   RETURN     :
        Success - if successful
        NDIS_STATUS_FAILURE - if memory allocation fails

 *****************************************************************************/
NDIS_STATUS MPAddWakeUpPattern( IN PMP_ADAPTER  pAdapter,
                                IN PVOID        InfoBuf, 
                                IN UINT         InfoBufLen )
{
    NDIS_STATUS             Status;
    PNDIS_PM_PACKET_PATTERN pPmPattern       = NULL;
    UINT                    Slot = 0, FreeSlot = 0xff;
    PUCHAR                  pNdisPatternMask;
    PUINT32                 pDevicePatternMask;
    UINT16                  DevMagicNumber;
    UINT                    i;
 
    PUCHAR TempBuf;
    ULONG  offset;
 
    /*-----------------------------------------------------------------------*/

    pPmPattern = (PNDIS_PM_PACKET_PATTERN)InfoBuf;

    /**************************************************************************
        Check that the wake-up pattern is not too large for us.  Also, not
        sure how to deal with a pattern whose mask is smaller than the pattern
     *************************************************************************/
    if(( pPmPattern->MaskSize < (( pPmPattern->PatternSize / 8) + 1)) ||
       ( pPmPattern->PatternSize > MAX_WOL_PACKET_SIZE))
    {
        return( NDIS_STATUS_RESOURCES );
    }

    NdisAcquireSpinLock( &pAdapter->Lock );

    /**************************************************************************
        Check to see if there is an empty slot on the device to store this
        pattern.  Store the slot number for later use.
     *************************************************************************/
    while ((FreeSlot == 0xff) && (Slot < NUM_WOL_PATTERNS))
    {
        if (!((pAdapter->PoMgmt.localWolAndCrc0 >> Slot) & 0x1))
        {
            FreeSlot = Slot;
        }
        else
        {
            Slot++;
        }
    }

    if( FreeSlot == 0xff )
    {
        /**********************************************************************
            Failed to find a free slot.
         *********************************************************************/
        NdisReleaseSpinLock( &pAdapter->Lock );
        return( NDIS_STATUS_RESOURCES );
    }

    pNdisPatternMask = (PUCHAR)InfoBuf + sizeof( NDIS_PM_PACKET_PATTERN );

    /**************************************************************************
        Make a copy of what Ndis sent us
     *************************************************************************/
    for( i=0; i<pPmPattern->MaskSize; i++ )
    {
        pAdapter->PoMgmt.WOLMaskList[ FreeSlot ][ i ] = pNdisPatternMask[ i ];
    }

    /**************************************************************************
        Calculate the CRC that will be used by the device to identify this
        packet
     *************************************************************************/
    offset  = 0;
    TempBuf = (PUCHAR)( InfoBuf ) + pPmPattern->PatternOffset;

    DevMagicNumber = MPCalculateCCITCRC16( TempBuf,
                                           (PUCHAR )( InfoBuf ) +
                                           sizeof( NDIS_PM_PACKET_PATTERN ),
                                           pPmPattern->MaskSize );

    DBGPRINT( MP_SPEC, ( "DevMagicNumber 0x%08x\n", DevMagicNumber ));
    DBGPRINT( MP_SPEC, ( "Pattern Size : 0x%08x, MaskSize : 0x%08x\n",
        pPmPattern->PatternSize, pPmPattern->MaskSize ));

    pAdapter->PoMgmt.localWolAndCrc0 |= ( 1 << FreeSlot );

    /**************************************************************************
        Now handle storing the pattern in our adapter.  Only store what makes
        the pattern unique to our device. (the CRC, the mask and the mask
        size.  The pattern is irrelevant, since it is not stored in the
        device).  The mask is stored above.
     *************************************************************************/
    pAdapter->PoMgmt.WOLPatternList[ FreeSlot ] = DevMagicNumber;
    pAdapter->PoMgmt.WOLMaskSize[ FreeSlot ]    = pPmPattern->MaskSize;

    NdisReleaseSpinLock( &pAdapter->Lock );
    return( NDIS_STATUS_SUCCESS );
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  MPRemoveWakeUpPattern
 ******************************************************************************

   DESCRIPTION:
        This routine will scan the array of wake up patterns and attempt to
        match the wake up pattern.  If it finds a copy , it will remove that 
        WakeUpPattern

   PARAMETERS:
        pAdapter                - pointer to the Adapter structure
        InformationBuffer       - Wake up Pattern
        InformationBufferLength - Wake Up Pattern Length
    
   RETURN:
        Success             - if successful.
        NDIS_STATUS_FAILURE - if memory allocation fails. 

 *****************************************************************************/
NDIS_STATUS MPRemoveWakeUpPattern( IN PMP_ADAPTER  pAdapter,
                                   IN PVOID        InformationBuffer, 
                                   IN UINT         InformationBufferLength )
{

    PNDIS_PM_PACKET_PATTERN pPmPattern          = NULL;
    UINT                    Slot = 0, FoundSlot = 0xff;
    UINT16                  DevMagicNumber;
    /*-----------------------------------------------------------------------*/

    pPmPattern = (PNDIS_PM_PACKET_PATTERN)InformationBuffer;

    /**************************************************************************
        Check that the wake-up pattern is not too large for us.  Also, not
        sure how to deal with a pattern whose mask is not the same size
        as the pattern.
     *************************************************************************/
    if(( pPmPattern->MaskSize < (( pPmPattern->PatternSize / 8) + 1)) ||
       ( pPmPattern->PatternSize > MAX_WOL_PACKET_SIZE))
    {
        return NDIS_STATUS_FAILURE;
    }

    /**************************************************************************
        Calculate the CRC that will be used by the device to identify this
        packet
     *************************************************************************/
    DevMagicNumber = MPCalculateCCITCRC16( (PUCHAR )( InformationBuffer ) +
                                                    pPmPattern->PatternOffset ,
                                           (PUCHAR )( InformationBuffer ) +
                                              sizeof( NDIS_PM_PACKET_PATTERN ),
                                           pPmPattern->MaskSize );

    NdisAcquireSpinLock( &pAdapter->Lock );

    while ((FoundSlot == 0xff) && (Slot < NUM_WOL_PATTERNS))
    {
        if(( pAdapter->PoMgmt.WOLPatternList[ Slot ] == DevMagicNumber ) &&
           ( pAdapter->PoMgmt.WOLMaskSize[ Slot ] == pPmPattern->MaskSize ) &&
           (( pAdapter->PoMgmt.localWolAndCrc0 >> Slot ) & 0x1) )
        {
            UINT    i;
            PUCHAR   pIncomingMask = (PUCHAR)( InformationBuffer ) +
                                              sizeof( NDIS_PM_PACKET_PATTERN );

            for( i=0; i<pPmPattern->MaskSize; i++ )
            {
                if( pAdapter->PoMgmt.WOLMaskList[ Slot ][ i ] !=
                                                           pIncomingMask[ i ] )
                {
                    break;
                }
            }
            if( i >= pPmPattern->MaskSize )
            {
                FoundSlot = Slot;
            }
        }
        
        Slot++;
    }

    if( FoundSlot == 0xff )
    {
        /**********************************************************************
            Failed to find this packet in the list coinciding with a valid
            entry in the device.
         *********************************************************************/
        NdisReleaseSpinLock( &pAdapter->Lock );
        return( NDIS_STATUS_FAILURE );
    }

    pAdapter->PoMgmt.localWolAndCrc0 &= ~( 1 << FoundSlot );

    NdisReleaseSpinLock( &pAdapter->Lock );

    return( NDIS_STATUS_SUCCESS );
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  MPRemoveAllWakeUpPatterns
 ******************************************************************************

   DESCRIPTION:
        This routine disables all wake-up on the device.  Probably not
        required.

   PARAMETERS:
        pAdapter - pointer to the adapter structure

   RETURN:
        Nothing

 *****************************************************************************/
VOID MPRemoveAllWakeUpPatterns( IN PMP_ADAPTER pAdapter )
{
    RXMAC_WOL_CTL_CRC0_t    crc0;
    /*-----------------------------------------------------------------------*/

    if( pAdapter )
    {
        NdisAcquireSpinLock( &pAdapter->Lock );

        pAdapter->PoMgmt.localWolAndCrc0 = 0;

        NdisReleaseSpinLock( &pAdapter->Lock );
    }
}
/*===========================================================================*/

#endif
