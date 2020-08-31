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
 * et131x_netdev.c - Routines and data required by all Linux network devices.
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
         $Date: 2007/01/22 23:13:56 $
     $Revision: 1.21 $
         $Name: T_20060131_v1-2-3 $
       $Author: vjs $

 *****************************************************************************/




/*******************************************************************************
   Includes
 ******************************************************************************/
#include "et131x_version.h"
#include "et131x_debug.h"
#include "et131x_defs.h"


#include "ET1310_phy.h"
#include "ET1310_pm.h"
#include "ET1310_jagcore.h"
#include "ET1310_mac.h"
#include "ET1310_tx.h"

#include "et131x_supp.h"
#include "et131x_adapter.h"
#include "et131x_isr.h"
#include "et131x_initpci.h"




/******************************************************************************
   Data for debugging facilities
 *****************************************************************************/
#if ET131X_DBG
extern dbg_info_t *et131x_dbginfo;
#endif  /* ET131X_DBG */




/******************************************************************************
   Prototypes for functions with local scope 
 *****************************************************************************/
int et131x_init( struct net_device *netdev );

int et131x_config( struct net_device *netdev, struct ifmap *map );

struct net_device_stats *et131x_stats( struct net_device *netdev );

int et131x_open( struct net_device *netdev );

int et131x_close( struct net_device *netdev );

int et131x_ioctl( struct net_device *netdev, struct ifreq *reqbuf, int cmd );

void et131x_multicast( struct net_device *netdev );

int et131x_tx( struct sk_buff *skb, struct net_device *netdev );

void et131x_tx_timeout( struct net_device *netdev );

int et131x_change_mtu( struct net_device *netdev, int new_mtu );

int et131x_set_mac_addr( struct net_device *netdev, void *new_mac );

void et131x_vlan_rx_register( struct net_device *netdev, struct vlan_group *grp );

void et131x_vlan_rx_add_vid( struct net_device *netdev, UINT16 vid );

void et131x_vlan_rx_kill_vid( struct net_device *netdev, UINT16 vid );




/******************************************************************************
   ROUTINE :  et131x_device_alloc()
 ******************************************************************************

   DESCRIPTION       : Create instances of net_device and wl_private for the
                       new adapter and register the device's entry points in
                       the net_device structure.
        
   PARAMETERS        : N/A
        
   RETURNS           : pointer to the allocated and initialized net_device
                       struct for this device.
        
   REUSE INFORMATION : 
        
 *****************************************************************************/
struct net_device * et131x_device_alloc( void )
{
    struct net_device *netdev  = NULL;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_device_alloc" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Alloc net_device and adapter structs
     *************************************************************************/
    netdev = alloc_etherdev( sizeof( ET131X_ADAPTER ));

    if( netdev == NULL )
    {
        DBG_ERROR( et131x_dbginfo,
                   "Alloc of net_device struct failed\n" );
        DBG_LEAVE( et131x_dbginfo );
        return NULL;
    }


    /**************************************************************************
       Setup the function registration table (and other data) for a net_device
     *************************************************************************/
    //netdev->init               = &et131x_init;
    netdev->set_config         = &et131x_config;
    netdev->get_stats          = &et131x_stats;
    netdev->open               = &et131x_open;
    netdev->stop               = &et131x_close;
    netdev->do_ioctl           = &et131x_ioctl;
    netdev->set_multicast_list = &et131x_multicast;
    netdev->hard_start_xmit    = &et131x_tx;
    netdev->tx_timeout         = &et131x_tx_timeout;
    netdev->watchdog_timeo     = ET131X_TX_TIMEOUT;
    netdev->change_mtu         = &et131x_change_mtu;
    netdev->set_mac_address    = &et131x_set_mac_addr;

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
    netdev->vlan_rx_register   = &et131x_vlan_rx_register;
    netdev->vlan_rx_add_vid    = &et131x_vlan_rx_add_vid;
    netdev->vlan_rx_kill_vid   = &et131x_vlan_rx_kill_vid;
#endif

    //netdev->ethtool_ops        = &et131x_ethtool_ops;
    
    // Poll?
    //netdev->poll               = &et131x_poll;
    //netdev->poll_controller    = &et131x_poll_controller;


    DBG_LEAVE( et131x_dbginfo );
    return netdev;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_device_free()
 ******************************************************************************

   DESCRIPTION       : Free the net_device and adapter private resources for
                       an adapter and perform basic cleanup.
        
   PARAMETERS        : netdev - a pointer to the net_device structure
                                representing the device whose resources should
                                be freed.
                                
   RETURNS           : N/A
        
   REUSE INFORMATION : 
        
 *****************************************************************************/
void et131x_device_free( struct net_device *netdev )
{

    DBG_FUNC( "et131x_device_free" );
    DBG_ENTER( et131x_dbginfo );

    if( netdev == NULL )
    {
        DBG_WARNING( et131x_dbginfo, "Pointer to net_device == NULL\n" );
        DBG_LEAVE( et131x_dbginfo );
        return;
    }
    else
    {
        free_netdev( netdev );
    }

    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_init
 ******************************************************************************

   DESCRIPTION       : This function is called by the kernel to initialize a
                       device
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing
                       the device to be initialized.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION : At the moment, this routine does nothing, as most init
                       processing needs to be performed before this point.
        
 *****************************************************************************/
int et131x_init( struct net_device *netdev )
{

    DBG_FUNC( "et131x_init" );
    DBG_ENTER( et131x_dbginfo );

    
    DBG_LEAVE( et131x_dbginfo );
    return 0;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_config
 ******************************************************************************

   DESCRIPTION       : Implement the SIOCSIFMAP interface.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing 
                                the device to be configured.
                       map    - a pointer to an ifmap struct
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION : 
        
 *****************************************************************************/
int et131x_config( struct net_device *netdev, struct ifmap *map )
{
    DBG_FUNC( "et131x_config" );
    DBG_ENTER( et131x_dbginfo );


    DBG_LEAVE( et131x_dbginfo );
    return 0;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_stats
 ******************************************************************************

   DESCRIPTION       : Return the current device statistics.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing 
                                the device whose stats are being queried.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
struct net_device_stats *et131x_stats( struct net_device *netdev )
{
    ET131X_ADAPTER          *adapter = netdev_priv(netdev);
	struct net_device_stats *stats   = &adapter->net_stats;
    CE_STATS_t              *devstat = &adapter->Stats;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_stats" );
    DBG_ENTER( et131x_dbginfo );


    stats->rx_packets          = devstat->ipackets;
    stats->tx_packets          = devstat->opackets;
    stats->rx_errors           = devstat->length_err + devstat->alignment_err +
                                 devstat->crc_err + devstat->code_violations +
                                 devstat->other_errors;
    stats->tx_errors           = devstat->max_pkt_error;
    stats->multicast           = devstat->multircv;
    stats->collisions          = devstat->collisions;

    stats->rx_length_errors    = devstat->length_err;
    stats->rx_over_errors      = devstat->rx_ov_flow;
    stats->rx_crc_errors       = devstat->crc_err;

    // NOTE: These stats don't have corresponding values in CE_STATS, so we're
    //       going to have to update these directly from within the TX/RX code
    //stats->rx_bytes            = 20; //devstat->;
    //stats->tx_bytes            = 20; //devstat->;
    //stats->rx_dropped          = devstat->;
    //stats->tx_dropped          = devstat->;

    // NOTE: Not used, can't find analogous statistics
    //stats->rx_frame_errors     = devstat->;
    //stats->rx_fifo_errors      = devstat->;
    //stats->rx_missed_errors    = devstat->;

    //stats->tx_aborted_errors   = devstat->;
    //stats->tx_carrier_errors   = devstat->;
    //stats->tx_fifo_errors      = devstat->;
    //stats->tx_heartbeat_errors = devstat->;
    //stats->tx_window_errors    = devstat->;


    DBG_LEAVE( et131x_dbginfo );
    return stats;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_open
 ******************************************************************************

   DESCRIPTION       : Open the device for use.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing 
                                the device to be opened.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_open( struct net_device *netdev )
{
    int             result  = 0;
    ET131X_ADAPTER *adapter = NULL;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_open" );
    DBG_ENTER( et131x_dbginfo );


    adapter = netdev_priv( netdev );


    /**************************************************************************
       Start the timer to track NIC errors
     *************************************************************************/
    add_timer( &adapter->ErrorTimer );


    /**************************************************************************
       Register our ISR
     *************************************************************************/
    DBG_TRACE( et131x_dbginfo, "Registering ISR...\n" );

    result = request_irq( netdev->irq, et131x_isr, SA_SHIRQ, netdev->name, netdev );
    if( result )
    {
        DBG_ERROR( et131x_dbginfo, "Could not register ISR\n" );
        DBG_LEAVE( et131x_dbginfo );
        return result;
    }
    

    /**************************************************************************
       Enable the Tx and Rx DMA engines (if not already enabled)
     *************************************************************************/
    et131x_rx_dma_enable( adapter );
    et131x_tx_dma_enable( adapter );


    /**************************************************************************
       Enable device interrupts
     *************************************************************************/
    et131x_enable_interrupts( adapter );

    MP_SET_FLAG( adapter, fMP_ADAPTER_INTERRUPT_IN_USE );


    /**************************************************************************
       We're ready to move some data, so start the queue
     *************************************************************************/
    netif_start_queue( netdev );


    DBG_LEAVE( et131x_dbginfo );
    return result;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_close
 ******************************************************************************

   DESCRIPTION       : Close the device.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing 
                                the device to be opened.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_close( struct net_device *netdev )
{
    ET131X_ADAPTER *adapter = NULL;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_close" );
    DBG_ENTER( et131x_dbginfo );


    adapter = netdev_priv( netdev );


    /**************************************************************************
       First thing is to stop the queue
     *************************************************************************/
    netif_stop_queue( netdev );


    /**************************************************************************
       Stop the Tx and Rx DMA engines
     *************************************************************************/
    et131x_rx_dma_disable( adapter );
    et131x_tx_dma_disable( adapter );


    /**************************************************************************
       Disable device interrupts
     *************************************************************************/
    et131x_disable_interrupts( adapter );


    /**************************************************************************
       Deregistering ISR
     *************************************************************************/
    MP_CLEAR_FLAG( adapter, fMP_ADAPTER_INTERRUPT_IN_USE );

    DBG_TRACE( et131x_dbginfo, "Deregistering ISR...\n" );
    free_irq( netdev->irq, netdev );


    /**************************************************************************
       Stop the error timer
     *************************************************************************/
    del_timer_sync( &adapter->ErrorTimer );


    DBG_LEAVE( et131x_dbginfo );
    return 0;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_ioctl_mii
 ******************************************************************************

   DESCRIPTION       : The function which handles MII IOCTLs
        
   PARAMETERS        : netdev - the net_device struct corresponding to the
                                device on which the query is being made
                       reqbuf - the request-specific data buffer
                       cmd    - the command request code
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_ioctl_mii( struct net_device *netdev, struct ifreq *reqbuf, int cmd )
{
    int                    status   = 0;
    ET131X_ADAPTER        *pAdapter = netdev_priv(netdev);
    struct mii_ioctl_data *data     = if_mii( reqbuf );
    UINT16                 mii_reg;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_ioctl_mii" );
    DBG_ENTER( et131x_dbginfo );


    switch( cmd )
    {
    case SIOCGMIIPHY:
        DBG_VERBOSE( et131x_dbginfo, "SIOCGMIIPHY\n" );
        data->phy_id = pAdapter->Stats.xcvr_addr;
        break;


    case SIOCGMIIREG:
        DBG_VERBOSE( et131x_dbginfo, "SIOCGMIIREG\n" );
        if( !capable( CAP_NET_ADMIN ))
        {
            status = -EPERM;
        }
        else
        {
            status = MiRead( pAdapter,
                             pAdapter->Stats.xcvr_addr,
                             data->reg_num,
                            &data->val_out );
        }

        break;


    case SIOCSMIIREG:
        DBG_VERBOSE( et131x_dbginfo, "SIOCSMIIREG\n" );
        if( !capable( CAP_NET_ADMIN ))
        {
            status = -EPERM;
        }
        else
        {
            mii_reg = data->val_in;

            status = MiWrite( pAdapter,
                              pAdapter->Stats.xcvr_addr,
                              data->reg_num,
                              mii_reg );
        }

        break;


    default:
        status = -EOPNOTSUPP;
    }


    DBG_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_ioctl
 ******************************************************************************

   DESCRIPTION       : The I/O Control handler for the driver.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing 
                                the device on which the control request is 
                                being made.
                       reqbuf - a pointer to the IOCTL request buffer.
                       cmd    - the IOCTL command code.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_ioctl( struct net_device *netdev, struct ifreq *reqbuf, int cmd )
{
    int status = 0;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_ioctl" );
    DBG_ENTER( et131x_dbginfo );


    switch( cmd )
    {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
        status = et131x_ioctl_mii( netdev, reqbuf, cmd );
        break;
	
    default:
        DBG_WARNING( et131x_dbginfo, "Unhandled IOCTL Code: 0x%04x\n", cmd );
		status = -EOPNOTSUPP;
	}


    DBG_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_set_packet_filter
 ******************************************************************************

   DESCRIPTION       : Configures the Rx Packet filtering on the device
        
   PARAMETERS        : adapter - pointer to our private adapter structure
        
   RETURNS           : 0 on success, errno on failure
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_set_packet_filter( ET131X_ADAPTER *adapter )
{
    int             status  = 0;
    UINT32          filter  = adapter->PacketFilter;
    RXMAC_CTRL_t    ctrl    = adapter->CSRAddress->rxmac.ctrl;
    RXMAC_PF_CTRL_t pf_ctrl = adapter->CSRAddress->rxmac.pf_ctrl;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_set_packet_filter" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Default to disabled packet filtering.  Enable it in the individual case
       statements that require the device to filter something
     *************************************************************************/
    ctrl.bits.pkt_filter_disable = 1;


    /**************************************************************************
       Set us to be in promiscuous mode so we receive everything, 
       this is also true when we get a packet filter of 0
     *************************************************************************/
    if(( filter & ET131X_PACKET_TYPE_PROMISCUOUS ) || filter == 0 )
    {
        pf_ctrl.bits.filter_broad_en = 0;
        pf_ctrl.bits.filter_multi_en = 0;
        pf_ctrl.bits.filter_uni_en   = 0;
    }
    else 
    {
        /**********************************************************************
           Set us up with Multicast packet filtering.  Three cases are
           possible - (1) we have a multi-cast list, (2) we receive ALL
           multicast entries or (3) we receive none.
         *********************************************************************/
        if( filter & ET131X_PACKET_TYPE_ALL_MULTICAST )
        {
            DBG_VERBOSE( et131x_dbginfo,
                         "Multicast filtering OFF (Rx ALL MULTICAST)\n" );
            pf_ctrl.bits.filter_multi_en = 0;
        }
        else
        {
            DBG_VERBOSE( et131x_dbginfo, "Multicast filtering ON\n" );
            SetupDeviceForMulticast( adapter );
            pf_ctrl.bits.filter_multi_en    = 1;
            ctrl.bits.pkt_filter_disable    = 0;
        }


        /**********************************************************************
           Set us up with Unicast packet filtering
         *********************************************************************/
        if( filter & ET131X_PACKET_TYPE_DIRECTED )
        {
            DBG_VERBOSE( et131x_dbginfo, "Unicast Filtering ON\n" );
            SetupDeviceForUnicast( adapter );
            pf_ctrl.bits.filter_uni_en = 1;
            ctrl.bits.pkt_filter_disable    = 0;
        }


        /**********************************************************************
           Set us up with Broadcast packet filtering
         *********************************************************************/
        if( filter & ET131X_PACKET_TYPE_BROADCAST )
        {
            DBG_VERBOSE( et131x_dbginfo, "Broadcast Filtering ON\n" );
            pf_ctrl.bits.filter_broad_en = 1;
            ctrl.bits.pkt_filter_disable    = 0;
        }
        else
        {
            DBG_VERBOSE( et131x_dbginfo, "Broadcast Filtering OFF\n" );
            pf_ctrl.bits.filter_broad_en = 0;
        }
 

        /**********************************************************************
           Setup the receive mac configuration registers - Packet Filter
           control + the enable / disable for packet filter in the control
           reg.
         *********************************************************************/
        adapter->CSRAddress->rxmac.pf_ctrl.value = pf_ctrl.value;
        adapter->CSRAddress->rxmac.ctrl = ctrl;
    }

    DBG_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_multicast
 ******************************************************************************

   DESCRIPTION       : The handler to configure multicasting on the interface.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing 
                                the device.
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_multicast( struct net_device *netdev )
{
    ET131X_ADAPTER     *adapter = NULL;
    UINT32              PacketFilter = 0;
    UINT32              count;
    unsigned long       lockflags;
    struct dev_mc_list *mclist = netdev->mc_list;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_multicast" );
    DBG_ENTER( et131x_dbginfo );


    adapter = netdev_priv( netdev );


    spin_lock_irqsave( &adapter->Lock, lockflags );


    /**************************************************************************
       Before we modify the platform-independent filter flags, store them 
       locally. This allows us to determine if anything's changed and if we
       even need to bother the hardware
     *************************************************************************/
    PacketFilter = adapter->PacketFilter;


    /**************************************************************************
       Clear the 'multicast' flag locally; becuase we only have a single flag
       to check multicast, and multiple multicast addresses can be set, this is
       the easiest way to determine if more than one multicast address is being
       set.
     *************************************************************************/
    PacketFilter &= ~ET131X_PACKET_TYPE_MULTICAST;


    /**************************************************************************
       Check the net_device flags and set the device independent flags
       accordingly
     *************************************************************************/
    DBG_VERBOSE( et131x_dbginfo,
                 "MULTICAST ADDR COUNT: %d\n",
                 netdev->mc_count );

    if( netdev->flags & IFF_PROMISC )
    {
        DBG_VERBOSE( et131x_dbginfo, "Request: PROMISCUOUS MODE ON\n" );
        adapter->PacketFilter |= ET131X_PACKET_TYPE_PROMISCUOUS;
    }
    else
    {
        DBG_VERBOSE( et131x_dbginfo, "Request: PROMISCUOUS MODE OFF\n" );
        adapter->PacketFilter &= ~ET131X_PACKET_TYPE_PROMISCUOUS;
    }


    if( netdev->flags & IFF_ALLMULTI )
    {
        DBG_VERBOSE( et131x_dbginfo, "Request: ACCEPT ALL MULTICAST\n" );
        adapter->PacketFilter |= ET131X_PACKET_TYPE_ALL_MULTICAST;
    }


    if( netdev->mc_count > NIC_MAX_MCAST_LIST )
    {
        DBG_WARNING( et131x_dbginfo,
                     "ACCEPT ALL MULTICAST for now, as there's more Multicast "
                     "addresses than the HW supports\n" );

        adapter->PacketFilter |= ET131X_PACKET_TYPE_ALL_MULTICAST;
    }


    if( netdev->mc_count < 1 )
    {
        DBG_VERBOSE( et131x_dbginfo, "Request: REJECT ALL MULTICAST\n" );
        adapter->PacketFilter &= ~ET131X_PACKET_TYPE_ALL_MULTICAST;
        adapter->PacketFilter &= ~ET131X_PACKET_TYPE_MULTICAST;
    }
    else
    {
        DBG_VERBOSE( et131x_dbginfo, "Request: SET MULTICAST FILTER(S)\n" );
        adapter->PacketFilter |= ET131X_PACKET_TYPE_MULTICAST;
    }


    /**************************************************************************
       Set values in the private adapter struct
     *************************************************************************/
    adapter->MCAddressCount = netdev->mc_count;

    if( netdev->mc_count )
    {
        if( mclist->dmi_addrlen != ETH_ALEN )
        {
            DBG_WARNING( et131x_dbginfo, "Multicast addrs are not ETH_ALEN in size\n" );
        }
        else
        {
            count = netdev->mc_count - 1;
            memcpy( adapter->MCList[count], mclist->dmi_addr, ETH_ALEN );
        }
    }


    /**************************************************************************
       Are the new flags different from the previous ones? If not, then no
       action is required

       NOTE - This block will always update the MCList with the hardware, even
              if the addresses aren't the same.
     *************************************************************************/
    if( PacketFilter != adapter->PacketFilter )
    {
        /**********************************************************************
           Call the device's filter function
         *********************************************************************/
        DBG_VERBOSE( et131x_dbginfo,
                     "UPDATE REQUIRED, FLAGS changed\n" );

        et131x_set_packet_filter( adapter );
    }
    else
    {
        DBG_VERBOSE( et131x_dbginfo,
                     "NO UPDATE REQUIRED, FLAGS didn't change\n" );
    }


    spin_unlock_irqrestore( &adapter->Lock, lockflags );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_tx
 ******************************************************************************

   DESCRIPTION       : The handler called when the Linux network layer wants
                       to a Tx a packet on the device.
        
   PARAMETERS        : skb - a pointer to the sk_buff structure which
                             represents the data to be Tx'd. 
                       netdev - a pointer to a net_device struct representing 
                                the device on which data is to be Tx'd.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_tx( struct sk_buff *skb, struct net_device *netdev )
{
    int             status = 0;
    ET131X_ADAPTER *adapter;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_tx" );
    DBG_TX_ENTER( et131x_dbginfo );


    adapter = netdev_priv( netdev );


    /**************************************************************************
       Save the timestamp for the TX timeout watchdog
     *************************************************************************/
    netdev->trans_start = jiffies;


    /**************************************************************************
       Call the device-specific data Tx routine
     *************************************************************************/
    status = et131x_send_packets( skb, netdev );


    /**************************************************************************
       Check status and manage the netif queue if necessary
     *************************************************************************/
    if( status != 0 )
    {
        if( status == -ENOMEM )
        {
            DBG_VERBOSE( et131x_dbginfo, "OUT OF TCBs; STOP NETIF QUEUE\n" );

            /* Put the queue to sleep until resources are available */
            netif_stop_queue( netdev );
            status = 1;
        }
        else
        {
            DBG_WARNING( et131x_dbginfo, "Misc error; drop packet\n" );
            status = 0;
        }
    }

    DBG_TX_LEAVE( et131x_dbginfo );
    return status;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_tx_timeout
 ******************************************************************************

   DESCRIPTION       : The handler called when a Tx request times out. The
                       timeout period is specified by the 'tx_timeo" element in
                       the net_device structure (see et131x_alloc_device() to
                       see how this value is set).
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing 
                                the device.
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_tx_timeout( struct net_device *netdev )
{
    ET131X_ADAPTER     *pAdapter = netdev_priv( netdev );
    PMP_TCB             pMpTcb;
    unsigned long       lockflags;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_tx_timeout" );
    DBG_WARNING( et131x_dbginfo, "TX TIMEOUT\n" );


    /**************************************************************************
       Just skip this part if the adapter is doing link detection
     *************************************************************************/
    if( MP_TEST_FLAG( pAdapter, fMP_ADAPTER_LINK_DETECTION ))
    {
        DBG_ERROR( et131x_dbginfo, "Still doing link detection\n" );
        return;   
    }


    /**************************************************************************
       Any nonrecoverable hardware error?
       Checks adapter->flags for any failure in phy reading
     *************************************************************************/
    if( MP_TEST_FLAG( pAdapter, fMP_ADAPTER_NON_RECOVER_ERROR ))   
    {
        DBG_WARNING( et131x_dbginfo, "Non recoverable error - remove\n" );
        return;
    }       


    /**************************************************************************
       Hardware failure?
     *************************************************************************/
    if( MP_TEST_FLAG( pAdapter, fMP_ADAPTER_HARDWARE_ERROR ))
    {
        DBG_WARNING( et131x_dbginfo, "hardware error - reset\n" );
        return;
    }

    /**************************************************************************
        Is send stuck?
     *************************************************************************/
    spin_lock_irqsave( &pAdapter->TCBSendQLock, lockflags );

    pMpTcb = pAdapter->TxRing.CurrSendHead;

    if( pMpTcb != NULL )
    {
        pMpTcb->Count++;

        if( pMpTcb->Count > NIC_SEND_HANG_THRESHOLD )
        {
#if ( ET131X_DBG == 1 )
            TX_STATUS_BLOCK_t   txDmaComplete = *( pAdapter->TxRing.pTxStatusVa );
            PTX_DESC_ENTRY_t    pDesc = pAdapter->TxRing.pTxDescRingVa + 
                                        pMpTcb->WrIndex.bits.serv_req;
#endif
            TX_DESC_ENTRY_t     StuckDescriptors[10];

            if( pMpTcb->WrIndex.bits.serv_req > 7 )
            {
                memcpy( StuckDescriptors, 
                        pAdapter->TxRing.pTxDescRingVa + pMpTcb->WrIndex.bits.serv_req - 6,
                        sizeof( TX_DESC_ENTRY_t ) * 10 );
            }

            spin_unlock_irqrestore( &pAdapter->TCBSendQLock, lockflags );

            DBG_WARNING( et131x_dbginfo, 
                         "Send stuck - reset.  pMpTcb->WrIndex %x, Flags 0x%08x\n",
                         pMpTcb->WrIndex.bits.serv_req, pMpTcb->Flags );

            DBG_WARNING( et131x_dbginfo,
                         "pDesc 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
                         pDesc->DataBufferPtrHigh, pDesc->DataBufferPtrLow,
                         pDesc->word2.value, pDesc->word3.value );

            DBG_WARNING( et131x_dbginfo, 
                         "WbStatus 0x%08x\n",
                         txDmaComplete.value );

#if ( ET131X_DBG == 1 )
            DumpDeviceBlock( DBG_WARNING_ON, pAdapter, 0 );
            DumpDeviceBlock( DBG_WARNING_ON, pAdapter, 1 );
            DumpDeviceBlock( DBG_WARNING_ON, pAdapter, 3 );
            DumpDeviceBlock( DBG_WARNING_ON, pAdapter, 5 );
#endif

            return;
        }
    }

    spin_unlock_irqrestore( &pAdapter->TCBSendQLock, lockflags );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_change_mtu
 ******************************************************************************

   DESCRIPTION       : The handler called to change the MTU for the device.
        
   PARAMETERS        : netdev  - a pointer to a net_device struct representing
                                 the device whose MTU is to be changed.
                               
                       new_mtu - the desired MTU.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_change_mtu( struct net_device *netdev, int new_mtu )
{
    int             result = 0;
    ET131X_ADAPTER *adapter = NULL;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_change_mtu" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Get the private adapter structure
     *************************************************************************/
    adapter = netdev_priv( netdev );

    if( adapter == NULL )
    {
        DBG_LEAVE( et131x_dbginfo );
        return -ENODEV;
    }


    /**************************************************************************
       Make sure the requested MTU is valid
     *************************************************************************/
    if( new_mtu == 0 || new_mtu > 9216 )
    {
        DBG_LEAVE( et131x_dbginfo );
        return -EINVAL;
    }


    /**************************************************************************
       Stop the netif queue
     *************************************************************************/
    netif_stop_queue( netdev );


    /**************************************************************************
       Stop the Tx and Rx DMA engines
     *************************************************************************/
    et131x_rx_dma_disable( adapter );
    et131x_tx_dma_disable( adapter );


    /**************************************************************************
       Disable device interrupts
     *************************************************************************/
    et131x_disable_interrupts( adapter );
    et131x_handle_send_interrupt( adapter );
    et131x_handle_recv_interrupt( adapter );


    /**************************************************************************
       Set the new MTU
     *************************************************************************/
    netdev->mtu = new_mtu;


    /**************************************************************************
       Free Rx DMA memory
     *************************************************************************/
    et131x_adapter_memory_free( adapter );


    /**************************************************************************
       Set the config parameter for Jumbo Packet support
     *************************************************************************/
    adapter->RegistryJumboPacket = new_mtu + 14;
    et131x_soft_reset( adapter );


    /**************************************************************************
       Alloc and init Rx DMA memory
     *************************************************************************/
    result = et131x_adapter_memory_alloc( adapter );
    if( result != 0 )
    {
        DBG_WARNING( et131x_dbginfo, 
                     "Change MTU failed; couldn't re-alloc DMA memory\n" );
        return result;
    }

    et131x_init_send( adapter );


    et131x_setup_hardware_properties( adapter );
    memcpy( netdev->dev_addr, adapter->CurrentAddress, ETH_ALEN );
    et131x_init_enet_crc_calc( );


    /**************************************************************************
       Init the device with the new settings
     *************************************************************************/
    et131x_adapter_setup( adapter );


    /**************************************************************************
       Enable interrupts
     *************************************************************************/
    if( MP_TEST_FLAG( adapter, fMP_ADAPTER_INTERRUPT_IN_USE ))
    {
        et131x_enable_interrupts( adapter );
    }


    /**************************************************************************
       Restart the Tx and Rx DMA engines
     *************************************************************************/
    et131x_rx_dma_enable( adapter );
    et131x_tx_dma_enable( adapter );
    
    
    /**************************************************************************
       Restart the netif queue
     *************************************************************************/
    netif_wake_queue( netdev );


    DBG_LEAVE( et131x_dbginfo );
    return result;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_set_mac_addr
 ******************************************************************************

   DESCRIPTION       : The handler called to change the MAC address for the
                       device.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing
                                 the device whose MAC is to be changed.
                       
                       new_mac  - a buffer containing a sock_addr struct in which
                                the desired MAC address is stored.
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :

   IMPLEMENTED BY : blux http://berndlux.de 22.01.2007 21:14
        
 *****************************************************************************/
int et131x_set_mac_addr( struct net_device *netdev, void *new_mac )
{
    DBG_FUNC( "et131x_set_mac_addr" );
    DBG_ENTER( et131x_dbginfo );
    // begin blux
    // DBG_VERBOSE( et131x_dbginfo, "Function not implemented!!\n" );

    int             result = 0;
    ET131X_ADAPTER *adapter = NULL;
    struct sockaddr *address = new_mac;
    /*-----------------------------------------------------------------------*/


    /**************************************************************************
       Get the private adapter structure
     *************************************************************************/
    adapter = netdev_priv( netdev );

    if( adapter == NULL )
    {
        DBG_LEAVE( et131x_dbginfo );
        return -ENODEV;
    }


    /**************************************************************************
       Make sure the requested MAC is valid
     *************************************************************************/
   if (!is_valid_ether_addr(address->sa_data))
    {
        DBG_LEAVE( et131x_dbginfo );
        return -EINVAL;
    }


    /**************************************************************************
       Stop the netif queue
     *************************************************************************/
    netif_stop_queue( netdev );


    /**************************************************************************
       Stop the Tx and Rx DMA engines
     *************************************************************************/
    et131x_rx_dma_disable( adapter );
    et131x_tx_dma_disable( adapter );


    /**************************************************************************
       Disable device interrupts
     *************************************************************************/
    et131x_disable_interrupts( adapter );
    et131x_handle_send_interrupt( adapter );
    et131x_handle_recv_interrupt( adapter );


    /**************************************************************************
       Set the new MAC
     *************************************************************************/
     //    netdev->set_mac_address  = &new_mac;
     //  netdev->mtu = new_mtu;


        memcpy(netdev->dev_addr, address->sa_data, netdev->addr_len);

        printk("%s: Setting MAC address to %02x:%02x:%02x:%02x:%02x:%02x\n", netdev->name,
           netdev->dev_addr[0], netdev->dev_addr[1], netdev->dev_addr[2],
           netdev->dev_addr[3], netdev->dev_addr[4], netdev->dev_addr[5]);


    /**************************************************************************
       Free Rx DMA memory
     *************************************************************************/
    et131x_adapter_memory_free( adapter );


    /**************************************************************************
       Set the config parameter for Jumbo Packet support
     *************************************************************************/
    //adapter->RegistryJumboPacket = new_mtu + 14;
    // blux: not needet here, w'll change the MAC

    et131x_soft_reset( adapter );


    /**************************************************************************
       Alloc and init Rx DMA memory
     *************************************************************************/
    result = et131x_adapter_memory_alloc( adapter );
    if( result != 0 )
    {
        DBG_WARNING( et131x_dbginfo, 
                     "Change MAC failed; couldn't re-alloc DMA memory\n" );
        return result;
    }

    et131x_init_send( adapter );


    et131x_setup_hardware_properties( adapter );
   // memcpy( netdev->dev_addr, adapter->CurrentAddress, ETH_ALEN );
   // blux: no, do not override our nice address
    et131x_init_enet_crc_calc( );


    /**************************************************************************
       Init the device with the new settings
     *************************************************************************/
    et131x_adapter_setup( adapter );


    /**************************************************************************
       Enable interrupts
     *************************************************************************/
    if( MP_TEST_FLAG( adapter, fMP_ADAPTER_INTERRUPT_IN_USE ))
    {
        et131x_enable_interrupts( adapter );
    }


    /**************************************************************************
       Restart the Tx and Rx DMA engines
     *************************************************************************/
    et131x_rx_dma_enable( adapter );
    et131x_tx_dma_enable( adapter );
    
    
    /**************************************************************************
       Restart the netif queue
     *************************************************************************/
    netif_wake_queue( netdev );



    DBG_LEAVE( et131x_dbginfo );
    return result;
}
/*===========================================================================*/




/******************************************************************************
   NOTE: The ET1310 doesn't support hardware VLAN tagging, so these functions
         are currently not used; they are in place to eventually support the
         feature if needed.
 *****************************************************************************/

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)

/******************************************************************************
   ROUTINE :  et131x_vlan_rx_register
 ******************************************************************************

   DESCRIPTION       : The handler called to enable or disable VLAN support on
                       the device.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing
                                the device on which VLAN should be enabled or
                                disabled.
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_vlan_rx_register( struct net_device *netdev, struct vlan_group *grp )
{
    ET131X_ADAPTER     *pAdapter = netdev_priv( netdev );
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_vlan_rx_register" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Track the vlan_group struct in the adapter private structure; if this 
       element is NULL, then VLAN is disabled; otherwise, it is enabled.
     *************************************************************************/
    if( grp )
    {
        DBG_VERBOSE( et131x_dbginfo, "VLAN: Enable, 0x%p\n", grp );
    }
    else
    {
        DBG_VERBOSE( et131x_dbginfo, "VLAN: Disable\n" );
    }

    pAdapter->vlgrp = grp;


    /**************************************************************************
       This is where any interfacing with the hardware to enable or disable
       VLAN would be done. Since the ET1310 doesn't handle this in hardware,
       nothing else needs to be done.
     *************************************************************************/


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_vlan_rx_add_vid
 ******************************************************************************

   DESCRIPTION       : The handler called to register a VLAN tag.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing
                                the device on which a VLAN tag should be added.
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_vlan_rx_add_vid( struct net_device *netdev, UINT16 vid )
{
    DBG_FUNC( "et131x_vlan_rx_add_vid" );
    DBG_ENTER( et131x_dbginfo );


    DBG_VERBOSE( et131x_dbginfo, "VLAN, Add VID: %d\n", vid );


    /**************************************************************************
       This is where any interfacing with the hardware to register VLAN IDs
       would take place. Since the ET1310 doesn't handle this in hardware,
       nothing else needs to be done here; the vlan_group structure's 
       vlan_devices element can be used in the TX/RX routines to determine if 
       a VLAN tag has been 'registered' with the device.
     *************************************************************************/


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_vlan_rx_kill_vid
 ******************************************************************************

   DESCRIPTION       : The handler called to deregister a VLAN tag.
        
   PARAMETERS        : netdev - a pointer to a net_device struct representing
                                the device on which a VLAN tag should be
                                removed.
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_vlan_rx_kill_vid( struct net_device *netdev, UINT16 vid )
{
    ET131X_ADAPTER     *pAdapter = netdev_priv( netdev );
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_vlan_rx_kill_vid" );
    DBG_ENTER( et131x_dbginfo );


    DBG_VERBOSE( et131x_dbginfo, "VLAN, Remove VID: %d\n", vid );

    if( pAdapter->vlgrp )
    {
        pAdapter->vlgrp->vlan_devices[vid] = NULL;
    }


    /**************************************************************************
       This is where any interfacing with the hardware to deregister VLAN IDs
       would take place. Since the ET1310 doesn't handle this in hardware,
       nothing else needs to be done here.
     *************************************************************************/


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/

#endif
