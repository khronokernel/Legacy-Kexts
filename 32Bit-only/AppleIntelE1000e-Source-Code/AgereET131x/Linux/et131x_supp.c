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
 * et131x_supp.c - Misc. support routines.
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
         $Date: 2005/08/01 19:35:13 $
     $Revision: 1.6 $
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




/******************************************************************************
   Data for debugging facilities
 *****************************************************************************/
#if ET131X_DBG
extern dbg_info_t *et131x_dbginfo;
#endif  /* ET131X_DBG */




/******************************************************************************
   Global look-up table used to calculate CRC.  Preset with the appropriate
   XORs from the 256 possible byte values that might be in the message.
 *****************************************************************************/
static UINT32 SynDrome [256];




/******************************************************************************
   Prototypes for functions with local scope
 *****************************************************************************/
static UINT32 crc32byte( UINT32 remainder, UCHAR data );




/******************************************************************************
   ROUTINE :  et131x_init_enet_crc_calc
 ******************************************************************************

   DESCRIPTION       : Initializes the look-up table (syndrome) for CRC
                       calculation.
        
   PARAMETERS        : N/A
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_init_enet_crc_calc( void )
{
    UINT32  EthernetKey = 0x04c11db7;
    UINT32  i;
    UINT32  j;
    UINT32  reg;
    BOOL_t  topBit;
    /*-----------------------------------------------------------------------*/


    for( i = 0; i < 256; i++ )
    {
        reg = i << 24;

        for( j = 0; j < 8; j++ )
        {
            topBit = ( reg & 0x80000000 ) != 0;

            reg <<= 1;
            
            if( topBit )
            {
                reg ^= EthernetKey;
            }
        }
        SynDrome[i] = reg;
    }
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  crc32byte
 ******************************************************************************

   DESCRIPTION       : 
        
   PARAMETERS        : 
        
   RETURNS           : 
        
   REUSE INFORMATION :
        
 *****************************************************************************/
#define CRC32_POLY 0x4C11DB7

static UINT32 crc32byte( UINT32 remainder, UCHAR data )
{
    int	   index;
	UINT32 remndr, hitbit;
    /*-----------------------------------------------------------------------*/


	remndr = remainder;

	for( index = 0; index < 8; index++ )
    {
		hitbit = (remndr >> 31) ^ (data & 0x01);

		data   = data >> 1;     // get the next data bit
		remndr = remndr << 1;   // get the next remainder bit

		if( hitbit )
        {
			remndr = remndr ^ CRC32_POLY;
		}
	}

	return remndr;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_calc_enet_crc
 ******************************************************************************

   DESCRIPTION       : Used to calculate the CRC of an ethernet message. Used
                       in the device's filtering of multi-cast packets.
        
   PARAMETERS        : Message     - A pointer to the Ethernet bytes to be encoded
                       MessageSize - The number of bytes in the message
        
   RETURNS           : The computed CRC
        
   REUSE INFORMATION :
        
 *****************************************************************************/
UINT32 et131x_calc_enet_crc( PUCHAR Message, UINT32 MessageSize )
{
    UINT32 Result = 0xFFFFFFFF;
    UINT32 i;
    /*-----------------------------------------------------------------------*/

    for( i = 0; i < MessageSize; i++ )
    {
        Result = crc32byte( Result, *Message );
        Message++;
    }

    return Result;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  pci_slot_information_read
 ******************************************************************************

   DESCRIPTION       : Reads a segment of the PCI configuration space for a
                       device.

                       This is a port of the NDIS function
                       NdisReadPciSlotInformation 

                       This function does not begin with 'et131x_' in order to
                       remain consistent with other system pci_xxx() calls.
        
   PARAMETERS        : pdev  - pointer to a pci_dev structure for the device
                               whose PCI config space is to be read
                       where - the offset to read
                       buf   - a pointer to a buffer in which the data will be
                               returned
                       len   - the length of the above buffer
        
   RETURNS           : The length of the buffer read
        
   REUSE INFORMATION :
        
 *****************************************************************************/
UINT32 pci_slot_information_read( struct pci_dev *pdev, UINT32 where, 
                                  UINT8 *buf, UINT32 len )
{
	int i;
    int status;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "pci_slot_information_read" );


	for( i = 0; i < len; i++ )
    {
        status = pci_read_config_byte( pdev, where+i, &buf[i] );

        if( status != 0 )
        {
            DBG_ERROR( et131x_dbginfo, "Cannot Read PCI config space...\n" );
            break;
        }
    }

	return len;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  pci_slot_information_write
 ******************************************************************************

   DESCRIPTION       : Writes a segment of the PCI configuration space for a
                       device.

                       This is a port of the NDIS function
                       NdisWritePciSlotInformation

                       This function does not begin with 'et131x_' in order to
                       remain consistent with other system pci_xxx() calls.
        
   PARAMETERS        : pdev  - pointer to a pci_dev structure for the device
                               whose PCI config space is to be written
                       where - the offset to write
                       buf   - a pointer to a buffer containing the data to be
                               written
                       len   - the length of the above buffer
        
   RETURNS           : The length of the buffer written
        
   REUSE INFORMATION :
        
 *****************************************************************************/
UINT32 pci_slot_information_write( struct pci_dev *pdev, UINT32 where,
                                   UINT8 *buf, UINT32 len )
{
	int i;
    int status;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "pci_slot_information_write" );


	for( i = 0; i < len; i++ )
    {
        status = pci_write_config_byte( pdev, where+i, buf[i] );

        if( status != 0 )
        {
            DBG_ERROR( et131x_dbginfo, "Cannot Write PCI config space...\n" );
            break;
        }
    }

	return len;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  netif_indicate_status
 ******************************************************************************

   DESCRIPTION       : Provides the network stack with link status for the
                       given device.

                       This is a port of the NDIS function
                       NdisMIndicateStatus. Unlike NDIS, there is no
                       corresponding status completion function.

                       This function does not begin with 'et131x_' in order to
                       remain consistent with other system netif_xxx() calls.
        
   PARAMETERS        : netdev - a pointer to the net_device struct representing
                                the device whose status we wosh to indicate.
                       status - the link status
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void netif_indicate_status( struct net_device *netdev, NETIF_STATUS status )
{
    DBG_FUNC( "netif_indicate_status" );


    if( netdev != NULL )
    {
        switch( status )
        {
        case NETIF_STATUS_MEDIA_CONNECT:
            DBG_VERBOSE( et131x_dbginfo, "Indicating Link UP\n" );
            netif_carrier_on( netdev );
            break;

        case NETIF_STATUS_MEDIA_DISCONNECT:
            DBG_VERBOSE( et131x_dbginfo, "Indicating Link DOWN\n" );
            netif_carrier_off( netdev );
            break;

        default:
            DBG_WARNING( et131x_dbginfo, 
                        "Unknown link status code: %d\n",
                        status );
            break;
        }
    }
    else
    {
        DBG_WARNING( et131x_dbginfo, "net_device pointer is NULL\n" );
    }

    return;
}
/*===========================================================================*/
