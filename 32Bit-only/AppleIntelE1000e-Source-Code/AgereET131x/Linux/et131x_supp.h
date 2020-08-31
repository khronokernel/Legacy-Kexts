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
 * et131x_supp.h - Misc. defines, structs, enums, prototypes, etc.
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
         $Date: 2005/08/01 19:35:13 $
     $Revision: 1.5 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/

#ifndef __ET131X_SUPP_H__
#define __ET131X_SUPP_H__




/******************************************************************************
   Enum for use with netif_indicate_status
 *****************************************************************************/
typedef enum netif_status
{
    NETIF_STATUS_INVALID           = 0,
    NETIF_STATUS_MEDIA_CONNECT,
    NETIF_STATUS_MEDIA_DISCONNECT,
    NETIF_STATUS_MAX
} NETIF_STATUS;




/******************************************************************************
   Definitions to maintain compatibility with older versions of the kernel
 *****************************************************************************/
#ifndef netdev_priv
#define netdev_priv(x)             (ET131X_ADAPTER *)netdev->priv
#endif

#ifndef SET_NETDEV_DEV
#define SET_NETDEV_DEV(net, pdev) 
#endif

#ifndef free_netdev
#define free_netdev(x)             kfree(x)
#endif

#ifndef if_mii
#define if_mii(x)                  (struct mii_ioctl_data *)&x->ifr_ifru
#endif




/******************************************************************************
   PROTOTYPES
 *****************************************************************************/
#ifndef	__APPLE__
void et131x_init_enet_crc_calc( void );

UINT32 et131x_calc_enet_crc( PUCHAR Message, UINT32 MessageSize );

UINT32 pci_slot_information_read( struct pci_dev *pdev, UINT32 where, 
                                  UINT8 *buf, UINT32 len );

UINT32 pci_slot_information_write( struct pci_dev *pdev, UINT32 where,
                                   UINT8 *buf, UINT32 len );

void netif_indicate_status( struct net_device *netdev, NETIF_STATUS status );

#endif


#endif  /* __ET131X_SUPP_H__ */
