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
 * et131x_initpci.c - Routines and data used to register the driver with the
 *                    PCI (and PCI Express) subsystem, as well as basic driver
 *                    init and startup.
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
     $Revision: 1.22 $
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
#include "et131x_netdev.h"
#include "et131x_config.h"
#include "et131x_isr.h"

#include "ET1310_address_map.h"
#include "ET1310_jagcore.h"
#include "ET1310_tx.h"
#include "ET1310_rx.h"
#include "ET1310_mac.h"
#include "ET1310_eeprom.h"




/******************************************************************************
   Data for debugging facilities
 *****************************************************************************/
#if ET131X_DBG
extern dbg_info_t *et131x_dbginfo;
#endif  /* ET131X_DBG */




/******************************************************************************
   Prototypes for functions with local scope 
 *****************************************************************************/
int __devinit et131x_pci_probe( struct pci_dev *pdev,
                                const struct pci_device_id *ent );

void __devexit et131x_pci_remove( struct pci_dev *pdev );

int et131x_pci_setup( struct pci_dev *pdev );




/******************************************************************************
   Data for PCI registration
 *****************************************************************************/
enum et131x_pci_versions
{
    Agere_Systems_PCI_V1 = 0,
};


static struct pci_device_id et131x_pci_table[] __devinitdata =
{
    { ET131X_PCI_VENDOR_ID, ET131X_PCI_DEVICE_ID_GIG, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0UL },
    { ET131X_PCI_VENDOR_ID, ET131X_PCI_DEVICE_ID_FAST, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0UL },
    { 0, }
};

MODULE_DEVICE_TABLE( pci, et131x_pci_table );


static struct pci_driver et131x_driver = 
{
    name:       DRIVER_NAME,
    id_table:   et131x_pci_table,
    probe:      et131x_pci_probe,
    remove:     __devexit_p( et131x_pci_remove ),
    suspend:    NULL, //et131x_pci_suspend,
    resume:     NULL, //et131x_pci_resume,
};




/******************************************************************************
   ROUTINE :  et131x_find_adapter
 ******************************************************************************

   DESCRIPTION       : Find the adapter and get all the assigned resources
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_find_adapter( ET131X_ADAPTER *adapter, struct pci_dev *pdev )
{
    int    result;
    UCHAR  eepromStat         = 0;
    UCHAR  maxPayload         = 0;
    UCHAR  latencyTimers      = 0;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_find_adapter" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
        Allow disabling of Non-Maskable Interrupts in I/O space, to
        support validation.
     *************************************************************************/
    if( adapter->RegistryNMIDisable )
    {
        UCHAR RegisterVal;

        RegisterVal  = inb( ET1310_NMI_DISABLE );
        RegisterVal &= 0xf3;

        if( adapter->RegistryNMIDisable == 2 )
        {
            RegisterVal |= 0xc;
        }

        outb( ET1310_NMI_DISABLE, RegisterVal );
    }


    /**************************************************************************
       We first need to check the EEPROM Status code located at offset 0xB2
       of config space
     *************************************************************************/
   
    result = pci_slot_information_read( pdev, 
                                        ET1310_PCI_EEPROM_STATUS,
                                        &eepromStat,
                                        sizeof( UCHAR ));
    /*************************************************************************
       THIS IS A WORKAROUND:
 	 * I need to call this function twice to get my card in a 
	   LG M1 Express Dual running. I tried also a msleep before this 
	   function, because I thougth there could be some time condidions
	   but it didn't work. Call the whole function twice also work.
     *************************************************************************/
    result = pci_slot_information_read( pdev, 
                                        ET1310_PCI_EEPROM_STATUS,
                                        &eepromStat,
                                        sizeof( UCHAR ));

    if( result != sizeof( UCHAR ))
    {
        DBG_ERROR( et131x_dbginfo, "Could not read PCI config space for "
                                   "EEPROM Status\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -EIO;
    }


    /**************************************************************************
       Determine if the error(s) we care about are present.  If they are
       present, we need to fail.
     *************************************************************************/
    if( eepromStat & 0x4C )
    { 
        result = pci_slot_information_read( pdev,
                                            PCI_REVISION_ID,
                                            &adapter->RevisionID,
                                            sizeof( UCHAR ));
        if( result != sizeof( UCHAR ))
        {
            DBG_ERROR( et131x_dbginfo,
                        "Could not read PCI config space for "
                        "Revision ID\n" );
            DBG_LEAVE( et131x_dbginfo );
            return -EIO;
        }
        else if( adapter->RevisionID == 0x01 )
        {
            INT32   nLoop;
            UCHAR   ucTemp[4] = {0xFE, 0x13, 0x10, 0xFF};

            /******************************************************************
               Re-write the first 4 bytes if we have an eeprom present and 
               the revision id is 1, this fixes the corruption seen with 
               1310 B Silicon
             *****************************************************************/
			for( nLoop = 0; nLoop < 3; nLoop++ )
			{
                EepromWriteByte( adapter, nLoop, ucTemp[nLoop], 0, SINGLE_BYTE );
			}
        }
                                            
        DBG_ERROR( et131x_dbginfo, "Fatal EEPROM Status Error - 0x%04x\n",
                   eepromStat );

        /**********************************************************************
           This error could mean that there was an error reading the eeprom
           or that the eeprom doesn't exist.  We will treat each case the 
           same and not try to gather additional information that normally
           would come from the eeprom, like MAC Address
         *********************************************************************/
        adapter->bEepromPresent = FALSE;

        DBG_LEAVE( et131x_dbginfo );
        return -EIO;
    }
    else
    {
        DBG_TRACE( et131x_dbginfo, "EEPROM Status Code - 0x%04x\n", eepromStat );
        adapter->bEepromPresent = TRUE;
    }


    /**************************************************************************
       Read the EEPROM for information regarding LED behavior. Refer to 
       ET1310_phy.c, et131x_xcvr_init(), for its use.
     *************************************************************************/
    EepromReadByte( adapter, 0x70, &adapter->eepromData [0], 0, SINGLE_BYTE );
    EepromReadByte( adapter, 0x71, &adapter->eepromData [1], 0, SINGLE_BYTE );

    if( adapter->eepromData[0] != 0xcd )
    {
        adapter->eepromData[1] = 0x00;  // Disable all optional features
    }


    /**************************************************************************
       Let's set up the PORT LOGIC Register.  First we need to know what the 
       max_payload_size is
     *************************************************************************/
    result = pci_slot_information_read( pdev,
                                        ET1310_PCI_MAX_PYLD,
                                        &maxPayload,
                                        sizeof( UCHAR ));
    
    if( result != sizeof( UCHAR ))
    {
        DBG_ERROR( et131x_dbginfo, "Could not read PCI config space for "
                                   "Max Payload Size\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -EIO;
    }
    else
    {
        UINT16 AckNak [2] = {0x76,  0xD0};
        UINT16 Replay [2] = {0x1E0, 0x2ED};


        /**********************************************************************
           Program the Ack/Nak latency and replay timers
         *********************************************************************/
        maxPayload &= 0x07;     // Only the lower 3 bits are valid

        if( maxPayload < 2 )
        {
            result = pci_slot_information_write( pdev,
                                                 ET1310_PCI_ACK_NACK,
                                                 (UINT8 *)&AckNak[maxPayload],
                                                 sizeof( UINT16 ));
            if( result != sizeof( UINT16 ))
            {
                DBG_ERROR( et131x_dbginfo, "Could not write PCI config space "
                                           "for ACK/NAK\n" );
                DBG_LEAVE( et131x_dbginfo );
                return -EIO;
            }

            result = pci_slot_information_write( pdev,
                                                 ET1310_PCI_REPLAY,
                                                 (UINT8 *)&Replay[maxPayload],
                                                 sizeof( UINT16 ));
            if( result != sizeof( UINT16 ))
            {
                DBG_ERROR( et131x_dbginfo, "Could not write PCI config space "
                                           "for Replay Timer\n" );
                DBG_LEAVE( et131x_dbginfo );
                return -EIO;
            }
        }
    }


    /**************************************************************************
       l0s and l1 latency timers.  We are using default values.
     *************************************************************************/
    latencyTimers = 0x11;   // Representing 001 for L0s and 010 for L1

    result = pci_slot_information_write( pdev,
                                         ET1310_PCI_L0L1LATENCY,
                                         (UINT8 *)&latencyTimers,
                                         sizeof( UCHAR ));
    if( result != sizeof( UCHAR ))
    {
        DBG_ERROR( et131x_dbginfo, "Could not write PCI config space for "
                                   "Latency Timers\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -EIO;
    }


    /**************************************************************************
       Archive Power management capability values for later use
     *************************************************************************/
    result = pci_slot_information_read( pdev,
                                        ET1310_PCI_PM_CAPABILITY,
                                        (UINT8 *)&adapter->PoMgmt.pmConfigRegs,
                                        sizeof( MP_PM_CONFIG_SPACE_INFO_t ));
    if( result != sizeof( MP_PM_CONFIG_SPACE_INFO_t ))
    {
        DBG_ERROR( et131x_dbginfo,
                   "Could not read PCI config space for PM Capability\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -EIO;
    }
    else
    {
        UCHAR read_size_reg;

        /******************************************************************
           Change the max read size to 2k
         *****************************************************************/
        result = pci_slot_information_read( pdev,
                                            0x51,
                                            (void *)&read_size_reg,
                                            sizeof( UCHAR ));

        if( result != sizeof( UCHAR ))
        {
            DBG_ERROR( et131x_dbginfo,
                       "Could not read PCI config space for Max read size\n" );
            DBG_LEAVE( et131x_dbginfo );
            return -EIO;
        }

        read_size_reg &= 0x8f;
        read_size_reg |= 0x40;

        result = pci_slot_information_write( pdev,
                                             0x51,
                                             &read_size_reg,
                                             sizeof( UCHAR ));
        if( result != sizeof( UCHAR ))
        {
            DBG_ERROR( et131x_dbginfo,
                       "Could not write PCI config space for Max read size\n" );
            DBG_LEAVE( et131x_dbginfo );
            return -EIO;
        }
    }


    /**************************************************************************
       PCI Express Configuration registers 0x48-0x5B (Device Control)
     *************************************************************************/
    result = pci_slot_information_read( pdev,
                                        ET1310_PCI_DEV_CTRL,
                                        (UINT8 *)&adapter->PciXDevCtl,
                                        sizeof( UINT16 ));

    if( result != sizeof( UINT16 ))
    {
        DBG_ERROR( et131x_dbginfo,
                   "Could not read PCI config space for PCI Express Dev Ctl\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -EIO;
    }


    /**************************************************************************
       Get MAC address from config space if an eeprom exists, otherwise the
       MAC address there will not be valid
     *************************************************************************/
    if( adapter->bEepromPresent )
    {
        result = pci_slot_information_read( pdev,
                                            ET1310_PCI_MAC_ADDRESS,
                                            (UINT8 *)adapter->PermanentAddress,
                                            ETH_ALEN );
        if( result != ETH_ALEN )
        {
            DBG_ERROR( et131x_dbginfo,
                       "Could not read PCI config space for MAC address\n" );
            DBG_LEAVE( et131x_dbginfo );
            return -EIO;
        }
    }


    DBG_LEAVE( et131x_dbginfo );
    return 0;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_error_timer_handler
 ******************************************************************************

   DESCRIPTION       : The routine called when the error timer expires, to
                       track the number of recurring errors.
        
   PARAMETERS        : data - a timer-specific variable; in this case, a 
                              pointer to our private adapter structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_error_timer_handler( unsigned long data )
{
    ET131X_ADAPTER  *pAdapter = (ET131X_ADAPTER *)data;
    PM_CSR_t        pm_csr;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_error_timer_handler" );


    pm_csr = pAdapter->CSRAddress->global.pm_csr;

    if( pm_csr.bits.pm_phy_sw_coma == 0 )
    {
        if( pAdapter->RegistryMACStat )
        {
            UpdateMacStatHostCounters( pAdapter );
        }
    }
    else
    {
        DBG_VERBOSE( et131x_dbginfo,
                     "No interrupts, in PHY coma, pm_csr = 0x%x\n",
                     pm_csr.value );
    }


    if( !pAdapter->Bmsr.bits.link_status && 
         pAdapter->RegistryPhyComa && 
         pAdapter->PoMgmt.TransPhyComaModeOnBoot < 11 )
    {
        pAdapter->PoMgmt.TransPhyComaModeOnBoot++;
    }

    if( pAdapter->PoMgmt.TransPhyComaModeOnBoot == 10 )
    {
        if( !pAdapter->Bmsr.bits.link_status && pAdapter->RegistryPhyComa )
        {
            if( pAdapter->CSRAddress->global.pm_csr.bits.phy_sw_coma == 0 )
            {
                // NOTE - This was originally a 'sync with interrupt'. How
                //        to do that under Linux?
                et131x_enable_interrupts( pAdapter );
                EnablePhyComa( pAdapter );
            }
        }
    }


    /**************************************************************************
       This is a periodic timer, so reschedule
     *************************************************************************/
    add_timer( &pAdapter->ErrorTimer );

    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  et131x_link_detection_handler
 ******************************************************************************
   DESCRIPTION:
        Timer function for handling link up at driver load time

   PARAMETERS :
        SystemSpecific1     Not used
        FunctionContext     Pointer to our adapter
        SystemSpecific2     Not used
        SystemSpecific3     Not used

   RETURNS    :
        NONE

 *****************************************************************************/
void et131x_link_detection_handler( unsigned long data )
{
    ET131X_ADAPTER *pAdapter   = (ET131X_ADAPTER *)data;
    unsigned long   lockflags;
    /*-----------------------------------------------------------------------*/


    /**************************************************************************
       Let everyone know that we have run
     *************************************************************************/
    pAdapter->bLinkTimerActive = FALSE;

    if( pAdapter->MediaState == 0 )
    {
        spin_lock_irqsave( &pAdapter->Lock, lockflags );

        pAdapter->MediaState = NETIF_STATUS_MEDIA_DISCONNECT;
        MP_CLEAR_FLAG( pAdapter, fMP_ADAPTER_LINK_DETECTION );
        
        spin_unlock_irqrestore( &pAdapter->Lock, lockflags );

        netif_indicate_status( pAdapter->netdev, pAdapter->MediaState );

        if( pAdapter->bSetPending )
        {
            pAdapter->bSetPending = FALSE;
        }
    }
    
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_adapter_setup
 ******************************************************************************

   DESCRIPTION       : Used to set the adapter up as per cassini+ documentation
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_adapter_setup( ET131X_ADAPTER *pAdapter )
{
    int   status = 0;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_adapter_setup" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Configure the JAGCore
     *************************************************************************/
    ConfigGlobalRegs( pAdapter );

    ConfigMACRegs1( pAdapter );
    ConfigMMCRegs( pAdapter );

    ConfigRxMacRegs( pAdapter );
    ConfigTxMacRegs( pAdapter );

    ConfigRxDmaRegs( pAdapter );
    ConfigTxDmaRegs( pAdapter );

    ConfigMacStatRegs( pAdapter );


    /**************************************************************************
       Move the following code to Timer function??
     *************************************************************************/
    status = et131x_xcvr_find( pAdapter );

    if( status != 0 )
    {
        DBG_WARNING( et131x_dbginfo, "Could not find the xcvr\n" );
    }


    /**********************************************************************
        Prepare the TRUEPHY library.
     *********************************************************************/
    ET1310_PhyInit( pAdapter );


    /**************************************************************************
        Reset the phy now so changes take place
     *************************************************************************/
    ET1310_PhyReset( pAdapter );
    

    /**************************************************************************
       Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );


    /**************************************************************************
        We need to turn off 1000 base half dulplex, the mac does not 
        support it
        For the 10/100 part, turn off all gig advertisement
     *************************************************************************/
    if( pAdapter->DeviceID != ET131X_PCI_DEVICE_ID_FAST )
    {
        ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
    }
    else
    {
        ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
    }


    /**************************************************************************
       Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );

    et131x_setphy_normal( pAdapter );


    DBG_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_setup_hardware_properties
 ******************************************************************************

   DESCRIPTION       : Used to set up the MAC Address on the ET1310
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_setup_hardware_properties( ET131X_ADAPTER *adapter )
{
    DBG_FUNC( "et131x_setup_hardware_properties" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       If have our default mac from registry and no mac address from EEPROM
       then we need to generate the last octet and set it on the device
     *************************************************************************/
    if( !adapter->bOverrideAddress )
    {
        if (adapter->PermanentAddress[0] == 0x00 &&
            adapter->PermanentAddress[1] == 0x00 &&
            adapter->PermanentAddress[2] == 0x00 &&
            adapter->PermanentAddress[3] == 0x00 &&
            adapter->PermanentAddress[4] == 0x00 &&
            adapter->PermanentAddress[5] == 0x00 )
        {
            /******************************************************************
                We need to randomly generate the last octet so we decrease our
                chances of setting the mac address to same as another one of 
                our cards in the system
             *****************************************************************/
            get_random_bytes( &adapter->CurrentAddress[5], 1 );


            /******************************************************************
                We have the default value in the register we are working with
                so we need to copy the current address into the permanent
                address
             *****************************************************************/
            memcpy( adapter->PermanentAddress,
                    adapter->CurrentAddress,
                    ETH_ALEN );
        }
        else
        {
            /******************************************************************
                We do not have an override address, so set the current address
                to the permanent address and add it to the device
             *****************************************************************/
            memcpy( adapter->CurrentAddress,
                    adapter->PermanentAddress,
                    ETH_ALEN );
        }
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_soft_reset
 ******************************************************************************

   DESCRIPTION       : Issue a soft reset to the hardware, complete for ET1310.
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_soft_reset( ET131X_ADAPTER *adapter )
{
    DBG_FUNC( "et131x_soft_reset" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Disable MAC Core
     *************************************************************************/
    adapter->CSRAddress->mac.cfg1.value = 0xc00f0000;


    /**************************************************************************
       Set everything to a reset value
     *************************************************************************/
    adapter->CSRAddress->global.sw_reset.value = 0x7F;
    adapter->CSRAddress->mac.cfg1.value        = 0x000f0000;
    adapter->CSRAddress->mac.cfg1.value        = 0x00000000;


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_align_allocated_memory
 ******************************************************************************

   DESCRIPTION       : Align an allocated block of memory on a given boundary
        
   PARAMETERS        : adapter   - pointer to our adapter structure
                       phys_addr - pointer to Physical address
                       offset    - pointer to the offset variable
                       mask      - correct mask
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_align_allocated_memory( ET131X_ADAPTER *adapter,
                                    UINT64         *phys_addr,
                                    UINT64         *offset,
                                    UINT64          mask )
{
    UINT64 new_addr;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_align_allocated_memory" );
    DBG_ENTER( et131x_dbginfo );


    *offset = 0;

    new_addr = *phys_addr & ~mask;

    if( new_addr != *phys_addr )
    {
        new_addr  += mask+1;                 // Move to next aligned block
        *offset    = new_addr - *phys_addr;  // Return offset for adjusting virt addr
        *phys_addr = new_addr;               // Return new physical address
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_adapter_memory_alloc
 ******************************************************************************

   DESCRIPTION       : Allocate all the memory blocks for send, receive and
                       others.
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_adapter_memory_alloc( ET131X_ADAPTER *adapter )
{
    int status    = 0;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_adapter_memory_alloc" );
    DBG_ENTER( et131x_dbginfo );

    do
    {
        /**********************************************************************
           Allocate memory for the Tx Ring
         *********************************************************************/
        status = et131x_tx_dma_memory_alloc( adapter );
        if( status != 0 )
        {
            DBG_ERROR( et131x_dbginfo, "et131x_tx_dma_memory_alloc FAILED\n" );
            break;
        }


        /**********************************************************************
           Receive buffer memory allocation
         *********************************************************************/
        status = et131x_rx_dma_memory_alloc( adapter );
        if( status != 0 )
        {
            DBG_ERROR( et131x_dbginfo, "et131x_rx_dma_memory_alloc FAILED\n" );
            et131x_tx_dma_memory_free( adapter );
            break;
        }


        /**********************************************************************
           Init receive data structures
         *********************************************************************/
        status = et131x_init_recv( adapter );
        if( status != 0 )
        {
            DBG_ERROR( et131x_dbginfo, "et131x_init_recv FAILED\n" );
            et131x_tx_dma_memory_free( adapter );
            et131x_rx_dma_memory_free( adapter );
            break;
        }
    } while( 0 );


    DBG_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_adapter_memory_free
 ******************************************************************************

   DESCRIPTION       : Free all memory allocated for use by Tx & Rx code
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_adapter_memory_free( ET131X_ADAPTER *adapter )
{
    DBG_FUNC( "et131x_adapter_memory_free" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Free DMA memory
     *************************************************************************/
    et131x_tx_dma_memory_free( adapter );
    et131x_rx_dma_memory_free( adapter );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_pci_register
 ******************************************************************************

   DESCRIPTION       : This function uses the above data to regsiter the PCI
                       function table and PCI Vendor/Product ID(s) with the PCI
                       subsystem to match corresponding devices to this driver.
        
   PARAMETERS        : N/A
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_pci_register( void )
{
    int result;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_pci_register" );
    DBG_ENTER( et131x_dbginfo );


    result = pci_register_driver( &et131x_driver );
    DBG_TRACE( et131x_dbginfo,
               " pci_register_driver( ) returns %d \n",
               result );


    DBG_LEAVE( et131x_dbginfo );
    return result;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_pci_cleanup
 ******************************************************************************

   DESCRIPTION       : This function deregisters the PCI function table and
                       related PCI Vendor/Product ID(s) with the PCI subsytem.
        
   PARAMETERS        : N/A
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_pci_unregister( void )
{
    DBG_FUNC( "et131x_pci_unregister" );
    DBG_ENTER( et131x_dbginfo );


    pci_unregister_driver( &et131x_driver );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_pci_probe
 ******************************************************************************

   DESCRIPTION       : Registered in the pci_driver structure, this function is
                       called when the PCI subsystem finds a new PCI device
                       which matches the information contained in the
                       pci_device_id table. This routine is the equivalent to
                       a device insertion routine.
        
   PARAMETERS        : pdev - a pointer to the device's pci_dev structure
                       ent  - this device's entry in the pci_device_id table
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int __devinit et131x_pci_probe( struct pci_dev *pdev,
                            const struct pci_device_id *ent )
{
    int result;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_pci_probe" );
    DBG_ENTER( et131x_dbginfo );


    result = et131x_pci_setup( pdev );


    DBG_LEAVE( et131x_dbginfo );
    return 0;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_pci_remove
 ******************************************************************************

   DESCRIPTION       : Registered in the pci_driver structure, this function is
                       called when the PCI subsystem detects that a PCI device
                       which matches the information contained in the
                       pci_device_id table has been removed.
        
   PARAMETERS        : pdev - a pointer to the device's pci_dev structure
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void __devexit et131x_pci_remove( struct pci_dev *pdev )
{
    struct net_device *netdev = NULL;
    ET131X_ADAPTER    *adapter = NULL;
    BOOL_t             bar_workaround;
    unsigned long      bar_addr_orig = 0;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_pci_remove" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Make sure the pci_dev pointer is valid
     *************************************************************************/
    if( pdev == NULL )
    {
        DBG_ERROR( et131x_dbginfo,
                   "PCI subsys passed in an invalid pci_dev pointer\n" );
        DBG_LEAVE( et131x_dbginfo );
        return;
    }


    /**************************************************************************
       Retrieve the net_device pointer from the pci_dev struct, as well as the
       private adapter struct
     *************************************************************************/
    netdev = (struct net_device *)pci_get_drvdata( pdev );
    if( netdev == NULL )
    {
        DBG_ERROR( et131x_dbginfo,
                   "Could not retrieve net_device struct\n" );
        DBG_LEAVE( et131x_dbginfo );
        return;
    }

    adapter = netdev_priv( netdev );
    if( adapter == NULL )
    {
        DBG_ERROR( et131x_dbginfo,
                   "Could not retrieve private adapter struct\n" );
        DBG_LEAVE( et131x_dbginfo );
        return;
    }


    /**************************************************************************
       Retrieve config space workaround info before deleting the private
       adapter struct
     *************************************************************************/
    bar_workaround = adapter->pci_bar_workaround;
    bar_addr_orig  = adapter->pci_bar_addr_orig;


    /**************************************************************************
       Perform device cleanup
     *************************************************************************/
    unregister_netdev( netdev );
    et131x_adapter_memory_free( adapter );
    iounmap( (void *)adapter->CSRAddress );
    et131x_device_free( netdev );

    if( bar_workaround == FALSE )
    {
        pci_release_regions( pdev );
    }
    else
    {
        pdev->resource[0].start = bar_addr_orig;
    }

    pci_disable_device( pdev );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_pci_setup
 ******************************************************************************

   DESCRIPTION       : Called by et131x_pci_probe() to perform device
                       initialization.
        
   PARAMETERS        : pdev - a pointer to the device's pci_dev structure
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_pci_setup( struct pci_dev *pdev )
{
    int                result = 0;
    int                pm_cap;
    BOOL_t             pci_using_dac;
    unsigned long      et131x_reg_base; 
    unsigned long      et131x_reg_len;
    struct net_device *netdev = NULL;
    ET131X_ADAPTER    *adapter = NULL;
    BOOL_t             bar_workaround = FALSE;
    unsigned long      bar_addr_orig = 0;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_pci_setup" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Make sure the device pointer is valid
     *************************************************************************/
    if( pdev == NULL )
    {
        DBG_ERROR( et131x_dbginfo, 
                   "PCI subsys passed in an invalid pci_dev pointer\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -ENODEV;
    }


    /**************************************************************************
       On some systems, the base address for a PCI device's config space which
       is stored in the pci_dev structure is incorrect and different from 
       what's actually in the config space. If they're different, workaround
       the issue by correcting the address.
     *************************************************************************/
    pci_read_config_dword( pdev, PCI_BASE_ADDRESS_0, (u32 *)&et131x_reg_base );
    et131x_reg_base &= ~0x07;

    if( (u32)pdev->resource[0].start != (u32)et131x_reg_base )
    {
        DBG_WARNING( et131x_dbginfo, "PCI CONFIG SPACE WORKAROUND REQUIRED\n" );
        DBG_WARNING( et131x_dbginfo, "pdev->resource[0].start : 0x%08x\n",
                     (unsigned int)pdev->resource[0].start );
        DBG_WARNING( et131x_dbginfo, "et131x_reg_base         : 0x%08x\n",
                     (unsigned int)et131x_reg_base );
        bar_workaround = TRUE;
        bar_addr_orig  = pdev->resource[0].start;
        pdev->resource[0].start = et131x_reg_base;
    }


    /**************************************************************************
       Enable the device via the PCI subsystem
     *************************************************************************/
    result = pci_enable_device( pdev );
    if( result != 0 )
    {
        if( bar_workaround )
        {
            pdev->resource[0].start = bar_addr_orig;
        }

        DBG_ERROR( et131x_dbginfo, "pci_enable_device() failed\n" );
        DBG_LEAVE( et131x_dbginfo );
        return result;
    }


    /**************************************************************************
       Perform some basic PCI checks
     *************************************************************************/
    if( !( pci_resource_flags( pdev, 0 ) & IORESOURCE_MEM ))
    {
        if( bar_workaround )
        {
            pdev->resource[0].start = bar_addr_orig;
        }

        DBG_ERROR( et131x_dbginfo,
                   "Can't find PCI device's base address\n" );
        DBG_LEAVE( et131x_dbginfo );
        return -ENODEV;
    }

    if( bar_workaround == FALSE )
    {    
        result = pci_request_regions( pdev, DRIVER_NAME );
        if( result != 0 )
        {
            DBG_ERROR( et131x_dbginfo,
                       "Can't get PCI resources\n" );

            pci_disable_device( pdev );

            DBG_LEAVE( et131x_dbginfo );
            return result;
        }
    }

    
    /**************************************************************************
       Enable PCI bus mastering
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Setting PCI Bus Mastering...\n" );
    pci_set_master( pdev );

    
    /**************************************************************************
       Query PCI for Power Mgmt Capabilities

       NOTE: Now reading PowerMgmt in another location; is this still needed?
     *************************************************************************/
    pm_cap = pci_find_capability( pdev, PCI_CAP_ID_PM );
    if( pm_cap == 0 )
    {
        DBG_ERROR( et131x_dbginfo,
                   "Cannot find Power Management capabilities\n" );

        if( bar_workaround == FALSE )
        {
            pci_release_regions( pdev );
        }
        else
        {
            pdev->resource[0].start = bar_addr_orig;
        }

        pci_disable_device( pdev );

        DBG_LEAVE( et131x_dbginfo );
        return -EIO;
    }

    
    /**************************************************************************
       Check the DMA addressing support of this device
     *************************************************************************/
    if( !pci_set_dma_mask( pdev, 0xffffffffffffffffULL ))
    {
        DBG_TRACE( et131x_dbginfo,
                   "64-bit DMA addressing supported\n" );
        pci_using_dac = TRUE;

        result = pci_set_consistent_dma_mask( pdev, 0xffffffffffffffffULL );
		if( result != 0 )
        {
			DBG_ERROR( et131x_dbginfo,
                       "Unable to obtain 64 bit DMA for consistent allocations\n" );
            if( bar_workaround == FALSE )
            {
                pci_release_regions( pdev );
            }
            else
            {
                pdev->resource[0].start = bar_addr_orig;
            }
            
            pci_disable_device( pdev );

			DBG_LEAVE( et131x_dbginfo );
            return -EIO;
        }
    }
    else if( !pci_set_dma_mask( pdev, 0xffffffffULL ))
    {
        DBG_TRACE( et131x_dbginfo,
                   "64-bit DMA addressing NOT supported\n" );
        DBG_TRACE( et131x_dbginfo,
                   "32-bit DMA addressing will be used\n" );
        pci_using_dac = FALSE;
    }
    else
    {
        DBG_ERROR( et131x_dbginfo, "No usable DMA addressing method\n" );

        if( bar_workaround == FALSE )
        {
            pci_release_regions( pdev );
        }
        else
        {
            pdev->resource[0].start = bar_addr_orig;
        }

        pci_disable_device( pdev );

        DBG_LEAVE( et131x_dbginfo );
        return -EIO;
    }


    /**************************************************************************
       Allocate netdev and private adapter structs
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Allocate netdev and private adapter structs...\n" );
    netdev = et131x_device_alloc( );

    if( netdev == NULL )
    {
        DBG_ERROR( et131x_dbginfo, "Couldn't alloc netdev struct\n" );

        if( bar_workaround == FALSE )
        {
            pci_release_regions( pdev );
        }
        else
        {
            pdev->resource[0].start = bar_addr_orig;
        }

        pci_disable_device( pdev );

        DBG_LEAVE( et131x_dbginfo );
        return -ENOMEM;
    }


    /**************************************************************************
       Setup the fundamental net_device and private adapter structure elements 
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Setting fundamental net_device info...\n" );

    SET_MODULE_OWNER( netdev );
    SET_NETDEV_DEV( netdev, &pdev->dev );

    if( pci_using_dac )
    {
        //netdev->features |= NETIF_F_HIGHDMA;
    }


    /**************************************************************************
       NOTE - Turn this on when we're ready to deal with SG-DMA

       NOTE: According to "Linux Device Drivers", 3rd ed, Rubini et al, if 
       checksumming is not performed in HW, then the kernel will not use SG.
       From pp 510-511:

       "Note that the kernel does not perform scatter/gather I/O to your device
       if it does not also provide some form of checksumming as well. The
       reason is that, if the kernel has to make a pass over a fragmented
       ("nonlinear") packet to calculate the checksum, it might as well copy
       the data and coalesce the packet at the same time."

       This has been verified by setting the flags below and still not 
       receiving a scattered buffer from the network stack, so leave it off
       until checksums are calculated in HW.
     *************************************************************************/
    //netdev->features |= NETIF_F_SG;
    //netdev->features |= NETIF_F_NO_CSUM;

    //netdev->features |= NETIF_F_LLTX;

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
    /**************************************************************************
       The ET1310 does not perform VLAN tagging in hardware, so these flags are
       not set.
     *************************************************************************/
    /*
    netdev->features   |= NETIF_F_HW_VLAN_TX |
                          NETIF_F_HW_VLAN_RX |
                          NETIF_F_HW_VLAN_FILTER;
    */
#endif

    /**************************************************************************
       Allocate private adapter struct and copy in relevant information 
     *************************************************************************/
    adapter           = netdev_priv( netdev );
    adapter->pdev     = pdev;
    adapter->netdev   = netdev;
    adapter->VendorID = pdev->vendor;
    adapter->DeviceID = pdev->device;

    adapter->pci_bar_workaround = bar_workaround;
    adapter->pci_bar_addr_orig  = bar_addr_orig;


    /**************************************************************************
       Do the same for the netdev struct 
     *************************************************************************/
    netdev->irq       = pdev->irq;
    netdev->base_addr = pdev->resource[0].start;


    /**************************************************************************
       Initialize spinlocks here
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Initialize spinlocks...\n" );

    spin_lock_init( &adapter->Lock );
    spin_lock_init( &adapter->TCBSendQLock );
    spin_lock_init( &adapter->TCBReadyQLock );
    spin_lock_init( &adapter->SendHWLock );
    spin_lock_init( &adapter->SendWaitLock );
    spin_lock_init( &adapter->RcvLock );
    spin_lock_init( &adapter->RcvPendLock );
    spin_lock_init( &adapter->FbrLock );
    spin_lock_init( &adapter->PHYLock );


    /**************************************************************************
       Parse configuration parameters into the private adapter struct
     *************************************************************************/
    et131x_config_parse( adapter );


    /**************************************************************************
       Find the physical adapter

       NOTE: This is the equivalent of the MpFindAdapter() routine; can we
             lump it's init with the device specific init below into a single
             init function?
     *************************************************************************/
    //while(et131x_find_adapter( adapter, pdev ) != 0);
    et131x_find_adapter( adapter, pdev );

    /**************************************************************************
       Map the bus-relative registers to system virtual memory
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Mapping bus-relative registers to virtual memory...\n" );

    if( bar_workaround == FALSE )
    { 
        et131x_reg_base = pci_resource_start( pdev, 0 );
        et131x_reg_len  = pci_resource_len( pdev, 0 );
    }
    else
    {
        et131x_reg_len  = 0x00200000;
    }

    adapter->CSRAddress = (ADDRESS_MAP_t *)ioremap_nocache( et131x_reg_base,
                                                            et131x_reg_len );
    if( adapter->CSRAddress == NULL )
    {
        DBG_ERROR( et131x_dbginfo, "Cannot map device registers\n" );

        et131x_device_free( netdev );

        if( bar_workaround == FALSE )
        {
            pci_release_regions( pdev );
        }
        else
        {
            pdev->resource[0].start = bar_addr_orig;
        }

        pci_disable_device( pdev );

        DBG_LEAVE( et131x_dbginfo );
        return -ENOMEM;
    }


    /**************************************************************************
       Perform device-specific initialization here (See code below)
     *************************************************************************/

    /**************************************************************************
       If Phy COMA mode was enabled when we went down, disable it here.
     *************************************************************************/
    {
        PM_CSR_t     GlobalPmCSR = {0};

        GlobalPmCSR.bits.pm_sysclk_gate     = 1;
        GlobalPmCSR.bits.pm_txclk_gate      = 1;
        GlobalPmCSR.bits.pm_rxclk_gate      = 1;

        adapter->CSRAddress->global.pm_csr = GlobalPmCSR;
    }


    /**************************************************************************
       Issue a global reset to the et1310
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Issuing soft reset...\n" );
    et131x_soft_reset( adapter );


    /**************************************************************************
       Disable all interrupts (paranoid)
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Disable device interrupts...\n" );
    et131x_disable_interrupts( adapter );


    /**************************************************************************
       Allocate DMA memory
     *************************************************************************/
    result = et131x_adapter_memory_alloc( adapter );
    if( result != 0 )
    {
        DBG_ERROR( et131x_dbginfo,
                   "Could not alloc adapater memory (DMA)\n" );

        iounmap( (void *)adapter->CSRAddress );

        et131x_device_free( netdev );

        if( bar_workaround == FALSE )
        {
            pci_release_regions( pdev );
        }
        else
        {
            pdev->resource[0].start = bar_addr_orig;
        }

        pci_disable_device( pdev );

        DBG_LEAVE( et131x_dbginfo );
        return -ENOMEM;
    }


    /**************************************************************************
       Init send data structures
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Init send data structures...\n" );
    et131x_init_send( adapter );

    adapter->PoMgmt.PowerState = NdisDeviceStateD0;


    /**************************************************************************
       Register the interrupt

       NOTE - This is being done in the open routine, where most other Linux
              drivers setup IRQ handlers. Make sure device interrupts are not
              turned on before the IRQ is registered!!!!

              What we will do here is setup the task structure for the ISR's
              deferred handler
     *************************************************************************/
    INIT_WORK( &adapter->task, et131x_isr_handler );


    /**************************************************************************
       Determine MAC Address, and copy into the net_device struct
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Retrieve MAC address...\n" );
    et131x_setup_hardware_properties( adapter );

    memcpy( netdev->dev_addr, adapter->CurrentAddress, ETH_ALEN );


    /**************************************************************************
       Setup up our lookup table for CRC Calculations
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Setup CRC lookup table...\n" );
    et131x_init_enet_crc_calc( );


    /**************************************************************************
       Setup et1310 as per the documentation
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Setup the adapter...\n" );
    et131x_adapter_setup( adapter );


    /**************************************************************************
       Create a timer to count errors received by the NIC
     *************************************************************************/
    init_timer( &adapter->ErrorTimer );
    
    adapter->ErrorTimer.expires  = jiffies + TX_ERROR_PERIOD * HZ / 1000;
    adapter->ErrorTimer.function = et131x_error_timer_handler;
    adapter->ErrorTimer.data     = (unsigned long)adapter;


    /**************************************************************************
       Initialize link state
     *************************************************************************/
    et131x_link_detection_handler( (unsigned long)adapter );


    /**************************************************************************
       Intialize variable for counting how long we do not have link status
     *************************************************************************/
    adapter->PoMgmt.TransPhyComaModeOnBoot = 0;


    /**************************************************************************
       We can enable interrupts now

       NOTE - Because registration of interrupt handler is done in the device's
              open(), defer enabling device interrupts to that point 
     *************************************************************************/


    /**************************************************************************
       Register the net_device struct with the Linux network layer
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Registering net_device...\n" );


    if(( result = register_netdev( netdev )) != 0 )
    {
        DBG_ERROR( et131x_dbginfo, "register_netdev() failed\n" );

        et131x_adapter_memory_free( adapter );

        iounmap( (void *)adapter->CSRAddress );

        et131x_device_free( netdev );

        if( bar_workaround == FALSE )
        {
            pci_release_regions( pdev );
        }
        else
        {
            pdev->resource[0].start = bar_addr_orig;
        }

        pci_disable_device( pdev );

        DBG_LEAVE( et131x_dbginfo );
        return result;
    }


    /**************************************************************************
       Register the net_device struct with the PCI subsystem. Save a copy
       of the PCI config space for this device now that the device has been 
       initialized, just in case it needs to be quickly restored.
     *************************************************************************/
    pci_set_drvdata( pdev, netdev );
    
    pci_save_state( adapter->pdev );


    /**************************************************************************
       Print out some information about this device
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo,
               "DEVICE FOUND\n" );
    DBG_TRACE( et131x_dbginfo,
               "------------------------------\n" );
    DBG_TRACE( et131x_dbginfo,
               "Device Vendor ID     : 0x%04x\n",
               pdev->vendor );
    DBG_TRACE( et131x_dbginfo,
               "Device Product ID    : 0x%04x\n",
               pdev->device );
    DBG_TRACE( et131x_dbginfo,
               "Device SubVendor ID  : 0x%04x\n",
               pdev->subsystem_vendor );
    DBG_TRACE( et131x_dbginfo,
               "Device SubProduct ID : 0x%04x\n",
               pdev->subsystem_device );


    DBG_TRACE( et131x_dbginfo,
               "Device on Bus #      : %d\n",
               pdev->bus->number );
    DBG_TRACE( et131x_dbginfo,
               "          Bus Name   : %s\n",
               pdev->bus->name );
    DBG_TRACE( et131x_dbginfo,
               "Device in Slot #     : %d\n",
               PCI_SLOT( pdev->devfn ));


    DBG_TRACE( et131x_dbginfo,
               "Device Base Address  : 0x%#03lx\n",
               netdev->base_addr );
    DBG_TRACE( et131x_dbginfo,
               "Device IRQ           : %d\n",
               netdev->irq );
    DBG_TRACE( et131x_dbginfo,
               "Device MAC Address   : %02x:%02x:%02x:%02x:%02x:%02x\n",
               adapter->CurrentAddress[0], adapter->CurrentAddress[1],
               adapter->CurrentAddress[2], adapter->CurrentAddress[3],
               adapter->CurrentAddress[4], adapter->CurrentAddress[5] );


    DBG_LEAVE( et131x_dbginfo );
    return 0;   
}
/*===========================================================================*/
