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
 * ET1310_mac.c - All code and routines pertaining to the MAC
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
     $Revision: 1.11 $
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
#include "et131x_initpci.h"
#include "et131x_supp.h"




/******************************************************************************
   Data for debugging facilities
 *****************************************************************************/
#if ET131X_DBG
extern dbg_info_t *et131x_dbginfo;
#endif  /* ET131X_DBG */




/******************************************************************************
   ROUTINE:  ConfigMacRegs1
 ******************************************************************************

   DESCRIPTION:
        Used to configure the first part of MAC regs to a known initialized 
        state

   PARAMETERS :
        pAdpater - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void ConfigMACRegs1( ET131X_ADAPTER *pAdapter )
{
    PMAC_t              pMac;
    MAC_STATION_ADDR1_t station1;
    MAC_STATION_ADDR2_t station2;
    MAC_IPG_t           ipg;
    MAC_HFDP_t          hfdp;
    MII_MGMT_CFG_t      mii_mgmt_cfg;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "ConfigMACRegs1" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Let's get our pointer to the MAC regs
     *************************************************************************/
    pMac = &pAdapter->CSRAddress->mac;


    /**************************************************************************
       First we need to reset everything.  Write to MAC configuration register 
       1 to perform reset.
     *************************************************************************/
    pMac->cfg1.value = 0xC00F0000;


    /**************************************************************************
       Next lets configure the MAC Inter-packet gap register
     *************************************************************************/
    ipg.bits.non_B2B_ipg_1        = 0x38; //58d
    ipg.bits.non_B2B_ipg_2        = 0x58; //88d
    ipg.bits.min_ifg_enforce      = 0x50; //80d
    ipg.bits.B2B_ipg              = 0x60; //96d

    pMac->ipg.value = ipg.value;


    /**************************************************************************
       Next lets configure the MAC Half Duplex register
     *************************************************************************/
    hfdp.bits.alt_beb_trunc       = 0xA;
    hfdp.bits.alt_beb_enable      = 0x0;
    hfdp.bits.bp_no_backoff       = 0x0;
    hfdp.bits.no_backoff          = 0x0;
    hfdp.bits.excess_defer        = 0x1;
    hfdp.bits.rexmit_max          = 0xF;
    hfdp.bits.coll_window         = 0x37; //55d

    pMac->hfdp.value = hfdp.value;


    /**************************************************************************
       Next lets configure the MAC Interface Control register
     *************************************************************************/
    pMac->if_ctrl.value = 0x0;


    /**************************************************************************
       Let's move on to setting up the mii managment configuration
     *************************************************************************/
    mii_mgmt_cfg.bits.reset_mii_mgmt    = 0;
    mii_mgmt_cfg.bits.scan_auto_incremt = 0;
    mii_mgmt_cfg.bits.preamble_suppress = 0;
    mii_mgmt_cfg.bits.mgmt_clk_reset    = 0x7;

    pMac->mii_mgmt_cfg.value = mii_mgmt_cfg.value;


    /**************************************************************************
       Next lets configure the MAC Station Address register.  These values are 
       read from the EEPROM during initialization and stored in the adapter 
       structure.  We write what is stored in the adapter structure to the MAC 
       Station Address registers high and low.  This station address is used
       for generating and checking pause control packets.
     *************************************************************************/
    station2.bits.Octet1    = pAdapter->CurrentAddress[0];
    station2.bits.Octet2    = pAdapter->CurrentAddress[1];
    station1.bits.Octet3    = pAdapter->CurrentAddress[2];
    station1.bits.Octet4    = pAdapter->CurrentAddress[3];
    station1.bits.Octet5    = pAdapter->CurrentAddress[4];
    station1.bits.Octet6    = pAdapter->CurrentAddress[5]; 

    pMac->station_addr_1.value = station1.value;
    pMac->station_addr_2.value = station2.value;


    /**************************************************************************
       Max ethernet packet in bytes that will passed by the mac without being
       truncated.  Allow the MAC to pass 8 more than our max packet size.  This
       is 4 for the Ethernet CRC and 4 for the VLAN ID.

       Packets larger than (RegistryJumboPacket) that do not contain a VLAN
       ID will be dropped by the Rx function.
     *************************************************************************/
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
    pMac->max_fm_len.value = pAdapter->RegistryJumboPacket + 8;
#else
    pMac->max_fm_len.value = pAdapter->RegistryJumboPacket + 4;
#endif


    /**************************************************************************
       clear out MAC config reset
     *************************************************************************/
    pAdapter->CSRAddress->mac.cfg1.value = 0x0;


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  ConfigMacRegs2
 ******************************************************************************

   DESCRIPTION:
        Used to configure the second part of MAC regs to a known initialized 
        state

   PARAMETERS :
        pAdpater - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void ConfigMACRegs2( ET131X_ADAPTER *pAdapter )
{
    INT32         delay = 0;
    PMAC_t        pMac;
    MAC_CFG1_t    cfg1;
    MAC_CFG2_t    cfg2;
    MAC_IF_CTRL_t ifctrl;
    TXMAC_CTL_t   ctl = pAdapter->CSRAddress->txmac.ctl;
    /*-----------------------------------------------------------------------*/

    
    DBG_FUNC( "ConfigMACRegs2" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Let's get our pointer to the MAC regs
     *************************************************************************/
    pMac = &pAdapter->CSRAddress->mac;

    cfg1.value   = pMac->cfg1.value;
    cfg2.value   = pMac->cfg2.value;
    ifctrl.value = pMac->if_ctrl.value; 
    
    if( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_1000MBPS )
    {
        cfg2.bits.if_mode    = 0x2;
        ifctrl.bits.phy_mode = 0x0;
    }
    else
    {
        cfg2.bits.if_mode    = 0x1;
        ifctrl.bits.phy_mode = 0x1;
    }


    /**************************************************************************
       We need to enable Rx/Tx
     *************************************************************************/
    cfg1.bits.rx_enable	    = 0x1;
    cfg1.bits.tx_enable	    = 0x1;


    /**************************************************************************
       Set up flow control
     *************************************************************************/
    cfg1.bits.tx_flow	    = 0x1;

    if( ( pAdapter->FlowControl == RxOnly ) ||
        ( pAdapter->FlowControl == Both ))
    {
        cfg1.bits.rx_flow	    = 0x1;
    }
    else
    {
        cfg1.bits.rx_flow	    = 0x0;
    }


    /**************************************************************************
       Initialize loop back to off
     *************************************************************************/
    cfg1.bits.loop_back	    = 0;

    pAdapter->CSRAddress->mac.cfg1.value = cfg1.value;


    /**************************************************************************
       Now we need to initialize the MAC Configuration 2 register
     *************************************************************************/ 
    cfg2.bits.preamble_len        = 0x7;
    cfg2.bits.huge_frame          = 0x0;
    /* LENGTH FIELD CHECKING bit4: Set this bit to cause the MAC to check the
     * frame’s length field to ensure it matches the actual data field length. Clear this bit if no
     * length field checking is desired. Its default is ‘0’.
     */
    cfg2.bits.len_check           = 0x1;

    if ( pAdapter->RegistryPhyLoopbk == FALSE )
    {
        cfg2.bits.pad_crc         = 0x1;
        cfg2.bits.crc_enable      = 0x1;
    }
    else
    {
        cfg2.bits.pad_crc         = 0;
        cfg2.bits.crc_enable      = 0;
    }


    /**************************************************************************
       1 – full duplex, 0 – half-duplex
     *************************************************************************/
    cfg2.bits.full_duplex	        = pAdapter->uiDuplexMode;
    ifctrl.bits.ghd_mode            = !pAdapter->uiDuplexMode;

    pAdapter->CSRAddress->mac.if_ctrl = ifctrl;
    pAdapter->CSRAddress->mac.cfg2.value = cfg2.value;

    do
    {
        udelay( 10 );
        delay++;
    } while(( !pAdapter->CSRAddress->mac.cfg1.bits.syncd_rx_en ||
              !pAdapter->CSRAddress->mac.cfg1.bits.syncd_tx_en ) && 
              ( delay < 100 ));

    if( delay == 100 )
    {
        DBG_ERROR( et131x_dbginfo,
                   "Syncd bits did not respond correctly cfg1 word 0x%08x\n",
                   pAdapter->CSRAddress->mac.cfg1.value );
    }

    DBG_TRACE( et131x_dbginfo,
               "Speed %d, Dup %d, CFG1 0x%08x, CFG2 0x%08x, if_ctrl 0x%08x\n",
               pAdapter->uiLinkSpeed, pAdapter->uiDuplexMode,
               pAdapter->CSRAddress->mac.cfg1.value,
               pAdapter->CSRAddress->mac.cfg2.value,
               pAdapter->CSRAddress->mac.if_ctrl.value );


    /**************************************************************************
       Enable TXMAC
     *************************************************************************/
    ctl.bits.txmac_en   = 0x1;
    ctl.bits.fc_disable = 0x1;
    pAdapter->CSRAddress->txmac.ctl = ctl;


    /**************************************************************************
       Ready to start the RXDMA/TXDMA engine
     *************************************************************************/
    if( !MP_TEST_FLAG( pAdapter, fMP_ADAPTER_LOWER_POWER ))
    {
        et131x_rx_dma_enable( pAdapter );
        et131x_tx_dma_enable( pAdapter );
    }
    else
    {
        DBG_WARNING( et131x_dbginfo,
                     "Didn't enable Rx/Tx due to low-power mode\n" );
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  ConfigRxMacRegs
 ******************************************************************************

   DESCRIPTION:
        Used to configure the RX MAC registers in the JAGCore

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void ConfigRxMacRegs( ET131X_ADAPTER *pAdapter )
{
    PRXMAC_t                  pRxMac;
    RXMAC_WOL_SA_LO_t         sa_lo;
    RXMAC_WOL_SA_HI_t         sa_hi;
    RXMAC_PF_CTRL_t           pf_ctrl = {0};
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "ConfigRxMacRegs" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Let's get a local pointer to the RX MAC Registers
     *************************************************************************/
    pRxMac = &pAdapter->CSRAddress->rxmac;


    /**************************************************************************
       Disable the MAC while it is being configured (also disable WOL)
     *************************************************************************/
    pRxMac->ctrl.value = 0x8;


    /**************************************************************************
       Initialize WOL to disabled.
     *************************************************************************/

    pRxMac->crc0.value      = 0x0;
    pRxMac->crc12.value     = 0x0000;
    pRxMac->crc34.value     = 0x0000;


    /**************************************************************************
        We need to set the WOL mask0 – mask4 next.  We initialize it to its
        default Values of 0x00000000 because there are not WOL masks as of
        this time.
     *************************************************************************/
    pRxMac->mask0_word0.mask                = 0x00000000;
    pRxMac->mask0_word1.mask                = 0x00000000;
    pRxMac->mask0_word2.mask                = 0x00000000;
    pRxMac->mask0_word3.mask                = 0x00000000;

    pRxMac->mask1_word0.mask                = 0x00000000;
    pRxMac->mask1_word1.mask                = 0x00000000;
    pRxMac->mask1_word2.mask                = 0x00000000;
    pRxMac->mask1_word3.mask                = 0x00000000;

    pRxMac->mask2_word0.mask                = 0x00000000;
    pRxMac->mask2_word1.mask                = 0x00000000;
    pRxMac->mask2_word2.mask                = 0x00000000;
    pRxMac->mask2_word3.mask                = 0x00000000;

    pRxMac->mask3_word0.mask                = 0x00000000;
    pRxMac->mask3_word1.mask                = 0x00000000;
    pRxMac->mask3_word2.mask                = 0x00000000;
    pRxMac->mask3_word3.mask                = 0x00000000;

    pRxMac->mask4_word0.mask                = 0x00000000;
    pRxMac->mask4_word1.mask                = 0x00000000;
    pRxMac->mask4_word2.mask                = 0x00000000;
    pRxMac->mask4_word3.mask                = 0x00000000;


    /**************************************************************************
       Lets setup the WOL Source Address
     *************************************************************************/
    sa_lo.bits.sa3                  = pAdapter->CurrentAddress[2]; 
    sa_lo.bits.sa4                  = pAdapter->CurrentAddress[3];
    sa_lo.bits.sa5                  = pAdapter->CurrentAddress[4];
    sa_lo.bits.sa6                  = pAdapter->CurrentAddress[5];
    pRxMac->sa_lo.value = sa_lo.value;
    
    sa_hi.bits.sa1                  = pAdapter->CurrentAddress[0];
    sa_hi.bits.sa2                  = pAdapter->CurrentAddress[1];
    pRxMac->sa_hi.value = sa_hi.value;


    /**************************************************************************
       Disable all Packet Filtering
     *************************************************************************/
    pRxMac->pf_ctrl.value = 0;


    /**************************************************************************
       Let's initialize the Unicast Packet filtering address
     *************************************************************************/
    if( pAdapter->PacketFilter & ET131X_PACKET_TYPE_DIRECTED )
    {
        SetupDeviceForUnicast( pAdapter );
        pf_ctrl.bits.filter_uni_en = 1;
    }
    else
    {
        pRxMac->uni_pf_addr1.value       = 0x00000000;
        pRxMac->uni_pf_addr2.value       = 0x00000000;
        pRxMac->uni_pf_addr3.value       = 0x00000000;
    }


    /**************************************************************************
       Let's initialize the Multicast hash
     *************************************************************************/
    if( pAdapter->PacketFilter & ET131X_PACKET_TYPE_ALL_MULTICAST )
    {
        pf_ctrl.bits.filter_multi_en = 0;
    }
    else
    {
        pf_ctrl.bits.filter_multi_en = 1;
        SetupDeviceForMulticast( pAdapter );
    }


    /**************************************************************************
       Runt packet filtering.  Didn't work in version A silicon.
     *************************************************************************/
    pf_ctrl.bits.min_pkt_size   = NIC_MIN_PACKET_SIZE + 4;
    pf_ctrl.bits.filter_frag_en = 1;

    if( pAdapter->RegistryJumboPacket > 8192 )
    {
        RXMAC_MCIF_CTRL_MAX_SEG_t mcif_ctrl_max_seg;


        /**********************************************************************
            In order to transmit jumbo packets greater than 8k, the FIFO
            between RxMAC and RxDMA needs to be reduced in size to (16k -
            Jumbo packet size).  In order to implement this, we must use
            "cut through" mode in the RxMAC, which chops packets down into
            segments which are (max_size * 16).  In this case we selected
            256 bytes, since this is the size of the PCI-Express TLP's that
            the 1310 uses.
         *********************************************************************/
        mcif_ctrl_max_seg.bits.seg_en   = 0x1;
        mcif_ctrl_max_seg.bits.fc_en    = 0x0;
        mcif_ctrl_max_seg.bits.max_size = 0x10;

        pRxMac->mcif_ctrl_max_seg.value = mcif_ctrl_max_seg.value;
    }
    else
    {
        pRxMac->mcif_ctrl_max_seg.value = 0x0;
    }


    /**************************************************************************
       Initialize the MCIF water marks
     *************************************************************************/
    pRxMac->mcif_water_mark.value    = 0x0;


    /**************************************************************************
       Initialize the MIF control
     *************************************************************************/
    pRxMac->mif_ctrl.value     = 0x0;


    /**************************************************************************
       Initialize the Space Available Register
     *************************************************************************/
    pRxMac->space_avail.value    = 0x0;

    /* Initialize the the mif_ctrl register
     * bit 3  - Receive code error. One or more nibbles were signaled as errors
                during the reception of the packet.  Clear this bit in Gigabit,
                set it in 100Mbit.  This was derived experimentally at UNH.
     * bit 4  - Receive CRC error. The packet’s CRC did not match the
                internally generated CRC.
     * bit 5  - Receive length check error. Indicates that frame length field
                value in the packet does not match the actual data byte length
                and is not a type field.
     * bit 16 - Receive frame truncated.
     * bit 17 - Drop packet enable
     */
    if( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_100MBPS )
    {
        pRxMac->mif_ctrl.value = 0x30038;
    }
    else
    {
        pRxMac->mif_ctrl.value = 0x30030;
    }


    /**************************************************************************
       Finally we initialize RxMac to be enabled & WOL disabled.  Packet filter
       is always enabled since it is where the runt packets are supposed to be
       dropped.  For version A silicon, runt packet dropping doesn't work, so
       it is disabled in the pf_ctrl register, but we still leave the packet
       filter on.
     *************************************************************************/
    pRxMac->pf_ctrl = pf_ctrl;
    pRxMac->ctrl.value = 0x9;


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  ConfigTxMacRegs
 ******************************************************************************

   DESCRIPTION:
        used to configure the TX MAC registers of the JAGCore

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void ConfigTxMacRegs( ET131X_ADAPTER *pAdapter )
{
    PTXMAC_t pTxMac;
    TXMAC_CF_PARAM_t Local;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "ConfigTxMacRegs" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Let's get the pointer to tx mac section of regs
     *************************************************************************/
    pTxMac = &pAdapter->CSRAddress->txmac;


    /**************************************************************************
       We need to update the Control Frame Parameters
       cfpt - control frame pause timer set to 64 (0x40)
       cfep - control frame extended pause timer set to 0x0
     *************************************************************************/
    if( pAdapter->FlowControl == None )
    {
        pTxMac->cf_param.value  = 0x0;
    }
    else
    {
        Local.bits.cfpt = 0x40;
        Local.bits.cfep = 0x0;
        pTxMac->cf_param.value  = Local.value;
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  ConfigMacStatRegs
 ******************************************************************************

   DESCRIPTION:
        Used to configure the MAC STAT section of the JAGCore

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void ConfigMacStatRegs( ET131X_ADAPTER *pAdapter )
{
    PMAC_STAT_t pDevMacStat;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "ConfigMacStatRegs" );
    DBG_ENTER( et131x_dbginfo );

    
    pDevMacStat = &pAdapter->CSRAddress->macStat;


    /**************************************************************************
       Next we need to initialize all the MAC_STAT registers to zero on the 
       device.
     *************************************************************************/
    pDevMacStat->RFcs = 0x0;
    pDevMacStat->RAln = 0x0;
    pDevMacStat->RFlr = 0x0;
    pDevMacStat->RDrp = 0x0;
    pDevMacStat->RCde = 0x0;
    pDevMacStat->ROvr = 0x0;
    pDevMacStat->RFrg = 0x0;

    pDevMacStat->TScl = 0x0;
    pDevMacStat->TDfr = 0x0;
    pDevMacStat->TMcl = 0x0;
    pDevMacStat->TLcl = 0x0;
    pDevMacStat->TNcl = 0x0;
    pDevMacStat->TOvr = 0x0;
    pDevMacStat->TUnd = 0x0;


    /***************************************************************************
       Unmask any counters that we want to track the overflow of.  Initially
       this will be all counters.  It may become clear later that we do not
       need to track all counters.
     **************************************************************************/
    {
        MAC_STAT_REG_1_t Carry1M = {0xffffffff};

        Carry1M.bits.rdrp         = 0x0;
        Carry1M.bits.rjbr         = 0x1;
        Carry1M.bits.rfrg         = 0x0;
        Carry1M.bits.rovr         = 0x0;
        Carry1M.bits.rund         = 0x1;
        Carry1M.bits.rcse         = 0x1;
        Carry1M.bits.rcde         = 0x0;
        Carry1M.bits.rflr         = 0x0;
        Carry1M.bits.raln         = 0x0;
        Carry1M.bits.rxuo         = 0x1;
        Carry1M.bits.rxpf         = 0x1;
        Carry1M.bits.rxcf         = 0x1;
        Carry1M.bits.rbca         = 0x1;
        Carry1M.bits.rmca         = 0x1;
        Carry1M.bits.rfcs         = 0x0;
        Carry1M.bits.rpkt         = 0x1;
        Carry1M.bits.rbyt         = 0x1;
        Carry1M.bits.trmgv        = 0x1;
        Carry1M.bits.trmax        = 0x1;
        Carry1M.bits.tr1k         = 0x1;
        Carry1M.bits.tr511        = 0x1;
        Carry1M.bits.tr255        = 0x1;
        Carry1M.bits.tr127        = 0x1;
        Carry1M.bits.tr64         = 0x1;

        pDevMacStat->Carry1M = Carry1M;
    }

    {
        MAC_STAT_REG_2_t Carry2M = {0xffffffff};

        Carry2M.bits.tdrp         = 0x1;
        Carry2M.bits.tpfh         = 0x1;
        Carry2M.bits.tncl         = 0x0;
        Carry2M.bits.txcl         = 0x1;
        Carry2M.bits.tlcl         = 0x0;
        Carry2M.bits.tmcl         = 0x0;
        Carry2M.bits.tscl         = 0x0;
        Carry2M.bits.tedf         = 0x1;
        Carry2M.bits.tdfr         = 0x0;
        Carry2M.bits.txpf         = 0x1;
        Carry2M.bits.tbca         = 0x1;
        Carry2M.bits.tmca         = 0x1;
        Carry2M.bits.tpkt         = 0x1;
        Carry2M.bits.tbyt         = 0x1;
        Carry2M.bits.tfrg         = 0x1;
        Carry2M.bits.tund         = 0x0;
        Carry2M.bits.tovr         = 0x0;
        Carry2M.bits.txcf         = 0x1;
        Carry2M.bits.tfcs         = 0x1;
        Carry2M.bits.tjbr         = 0x1;

        pDevMacStat->Carry2M = Carry2M;
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE:  ConfigFlowControl
 ******************************************************************************

   DESCRIPTION:
        Used to configure the MAC STAT section of the JAGCore

   PARAMETERS :
        pAdapter - pointer to our adapter structure

   RETURNS    :
        NONE

 *****************************************************************************/
void ConfigFlowControl( ET131X_ADAPTER *pAdapter )
{
    if( pAdapter->uiDuplexMode == 0 )
    {
        pAdapter->FlowControl = None;
    }
    else
    {
        char RemotePause, RemoteAsyncPause;

        ET1310_PhyAccessMiBit( pAdapter,
                               TRUEPHY_BIT_READ, 5, 10, &RemotePause );
        ET1310_PhyAccessMiBit( pAdapter,
                               TRUEPHY_BIT_READ, 5, 11, &RemoteAsyncPause );

        if(( RemotePause == TRUEPHY_BIT_SET ) &&
           ( RemoteAsyncPause == TRUEPHY_BIT_SET ))
        {
            pAdapter->FlowControl = pAdapter->RegistryFlowControl;
        }
        else if(( RemotePause      == TRUEPHY_BIT_SET ) && 
                ( RemoteAsyncPause == TRUEPHY_BIT_CLEAR ))
        {
            if( pAdapter->RegistryFlowControl == Both )
            {
                pAdapter->FlowControl = Both;
            }
            else
            {
                pAdapter->FlowControl = None;
            }
        }
        else if(( RemotePause      == TRUEPHY_BIT_CLEAR ) && 
                ( RemoteAsyncPause == TRUEPHY_BIT_CLEAR ))
        {
            pAdapter->FlowControl = None;
        }
        else /* if (( RemotePause      == TRUEPHY_CLEAR_BIT ) && 
                    ( RemoteAsyncPause == TRUEPHY_SET_BIT )) */
        {
            if( pAdapter->RegistryFlowControl == Both )
            {
                pAdapter->FlowControl = RxOnly;
            }
            else
            {
                pAdapter->FlowControl = None;
            }
        }
    }
}
/*===========================================================================*/






/******************************************************************************
   ROUTINE:  UpdateMacStatHostCounters
 ******************************************************************************

   DESCRIPTION:
        used to update the local copy of the statistics held in the adapter 
        structure

   PARAMETERS :
        pAdapter - pointer to the adapter structure 

   RETURNS    :
        NONE

 *****************************************************************************/
void UpdateMacStatHostCounters( ET131X_ADAPTER *pAdapter )
{
    PMAC_STAT_t pDevMacStat;
    /*-----------------------------------------------------------------------*/


    /**************************************************************************
       Get a local pointer to the adapter macstat regs and update stats
     *************************************************************************/
    pDevMacStat = &pAdapter->CSRAddress->macStat;

    pAdapter->Stats.collisions              += pDevMacStat->TNcl;
    pAdapter->Stats.first_collision         += pDevMacStat->TScl;
    pAdapter->Stats.tx_deferred             += pDevMacStat->TDfr;
    pAdapter->Stats.excessive_collisions    += pDevMacStat->TMcl;
    pAdapter->Stats.late_collisions         += pDevMacStat->TLcl;
    pAdapter->Stats.tx_uflo                 += pDevMacStat->TUnd;
    pAdapter->Stats.max_pkt_error           += pDevMacStat->TOvr;

    pAdapter->Stats.alignment_err           += pDevMacStat->RAln;
    pAdapter->Stats.crc_err                 += pDevMacStat->RCde;
    pAdapter->Stats.norcvbuf                += pDevMacStat->RDrp;
    pAdapter->Stats.rx_ov_flow              += pDevMacStat->ROvr;
    pAdapter->Stats.code_violations         += pDevMacStat->RFcs;
    pAdapter->Stats.length_err              += pDevMacStat->RFlr;

    pAdapter->Stats.other_errors            += pDevMacStat->RFrg;

    return;
}
/*===========================================================================*/





/******************************************************************************
   ROUTINE:  HandleMacStatInterrupt
 ******************************************************************************

   DESCRIPTION:
        One of the MACSTAT counters has wrapped.  Update the local copy of
        the statistics held in the adapter structure, checking the "wrap"
        bit for each counter.

   PARAMETERS :
        pAdapter - pointer to the adapter structure 

   RETURNS    :
        NONE

 *****************************************************************************/
void HandleMacStatInterrupt( ET131X_ADAPTER *pAdapter )
{
    MAC_STAT_REG_1_t Carry1;
    MAC_STAT_REG_2_t Carry2;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "HandleMacStatInterrupt" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Read the interrupt bits from the register(s).  These are Clear On Write.
     *************************************************************************/
    Carry1 = pAdapter->CSRAddress->macStat.Carry1;
    Carry2 = pAdapter->CSRAddress->macStat.Carry2;

    pAdapter->CSRAddress->macStat.Carry1 = Carry1;
    pAdapter->CSRAddress->macStat.Carry2 = Carry2;


    /**************************************************************************
       We need to do update the host copy of all the MAC_STAT counters.  For
       each counter, check it's overflow bit.  If the overflow bit is set, then
       increment the host version of the count by one complete revolution of the
       counter.  This routine is called when the counter block indicates that
       one of the counters has wrapped.
     *************************************************************************/
    if( Carry1.bits.rfcs )
    {
        pAdapter->Stats.code_violations += COUNTER_WRAP_16_BIT;
    }

    if( Carry1.bits.raln )
    {
        pAdapter->Stats.alignment_err += COUNTER_WRAP_12_BIT;
    }

    if( Carry1.bits.rflr )
    {
        pAdapter->Stats.length_err += COUNTER_WRAP_16_BIT;
    }

    if( Carry1.bits.rfrg )
    {
        pAdapter->Stats.other_errors += COUNTER_WRAP_16_BIT;
    }

    if( Carry1.bits.rcde )
    {
        pAdapter->Stats.crc_err += COUNTER_WRAP_16_BIT;
    }

    if( Carry1.bits.rovr )
    {
        pAdapter->Stats.rx_ov_flow += COUNTER_WRAP_16_BIT;
    }

    if( Carry1.bits.rdrp )
    {
        pAdapter->Stats.norcvbuf += COUNTER_WRAP_16_BIT;
    }

    if( Carry2.bits.tovr )
    {
        pAdapter->Stats.max_pkt_error += COUNTER_WRAP_12_BIT;
    }

    if( Carry2.bits.tund )
    {
        pAdapter->Stats.tx_uflo += COUNTER_WRAP_12_BIT;
    }

    if( Carry2.bits.tscl )
    {
        pAdapter->Stats.first_collision += COUNTER_WRAP_12_BIT;
    }

    if( Carry2.bits.tdfr )
    {
        pAdapter->Stats.tx_deferred += COUNTER_WRAP_12_BIT;
    }

    if( Carry2.bits.tmcl )
    {
        pAdapter->Stats.excessive_collisions += COUNTER_WRAP_12_BIT;
    }

    if( Carry2.bits.tlcl )
    {
        pAdapter->Stats.late_collisions += COUNTER_WRAP_12_BIT;
    }

    if( Carry2.bits.tncl )
    {
        pAdapter->Stats.collisions += COUNTER_WRAP_12_BIT;
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  SetupDeviceForMulticast
 ******************************************************************************

   DESCRIPTION       :
        Use to set the ET1310 to do multicast filtering 

   PARAMETERS        :
        pAdapter - pointer to our adapter structure

   RETURNS           :
        NONE

   REUSE INFORMATION :
        NONE
        
 *****************************************************************************/
void SetupDeviceForMulticast( ET131X_ADAPTER *pAdapter )
{
    UINT32             nIndex;
    UINT32             result;
    RXMAC_MULTI_HASH_t hash1 = {0};
    RXMAC_MULTI_HASH_t hash2 = {0};
    RXMAC_MULTI_HASH_t hash3 = {0};
    RXMAC_MULTI_HASH_t hash4 = {0};
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "SetupDeviceForMulticast" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       If ET131X_PACKET_TYPE_MULTICAST is specified, then we provision the
       multi-cast LIST.  If it is NOT specified, (and "ALL" is not specified)
       then we should pass NO multi-cast addresses to the driver.
     *************************************************************************/
    if( pAdapter->PacketFilter & ET131X_PACKET_TYPE_MULTICAST )
    {
        DBG_VERBOSE( et131x_dbginfo, "MULTICAST flag is set, MCCount: %d\n",
                     pAdapter->MCAddressCount );


        /**********************************************************************
           Loop through our multicast array and set up the device
        **********************************************************************/
        for( nIndex = 0; nIndex < pAdapter->MCAddressCount; nIndex++ )
        {
            DBG_VERBOSE( et131x_dbginfo,
                         "MCList[%d]: %02x:%02x:%02x:%02x:%02x:%02x\n",
                         nIndex,
                         pAdapter->MCList[nIndex][0],
                         pAdapter->MCList[nIndex][1],
                         pAdapter->MCList[nIndex][2],
                         pAdapter->MCList[nIndex][3],
                         pAdapter->MCList[nIndex][4],
                         pAdapter->MCList[nIndex][5] );

            result = et131x_calc_enet_crc( pAdapter->MCList[nIndex], 6 );

            result = ( result & 0x3F800000 ) >> 23;

            if( result < 32 )
            {
                hash1.hash |= ( 1 << result );
            }
            else if(( 31 < result ) && ( result < 64 ))
            {
                result -= 32;
                hash2.hash |= ( 1 << result );
            }
            else if(( 63 < result ) && ( result < 96 ))
            {
                result -= 64;
                hash3.hash |= ( 1 << result );
            }
            else
            {
                result -= 96;
                hash4.hash |= ( 1 << result );
            }
        }
    }


    /**************************************************************************
       Write out the new hash to the device
     *************************************************************************/
    if( pAdapter->CSRAddress->global.pm_csr.bits.pm_phy_sw_coma == 0 )
    {
    pAdapter->CSRAddress->rxmac.multi_hash1.hash = hash1.hash;
    pAdapter->CSRAddress->rxmac.multi_hash2.hash = hash2.hash;
    pAdapter->CSRAddress->rxmac.multi_hash3.hash = hash3.hash;
    pAdapter->CSRAddress->rxmac.multi_hash4.hash = hash4.hash;
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  SetupDeviceForUnicast
 ******************************************************************************

   DESCRIPTION       :
        Use to set the ET1310 to do unicast filtering 

   PARAMETERS        :
        pAdapter - pointer to our adapter structure

   RETURNS           :
        NONE

   REUSE INFORMATION :
        NONE

 *****************************************************************************/
void SetupDeviceForUnicast( ET131X_ADAPTER *pAdapter )
{
    RXMAC_UNI_PF_ADDR1_t uni_pf1;
    RXMAC_UNI_PF_ADDR2_t uni_pf2;
    RXMAC_UNI_PF_ADDR3_t uni_pf3;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "SetupDeviceForUnicast" );
    DBG_ENTER( et131x_dbginfo );


    /**************************************************************************
       Set up unicast packet filter reg 3 to be the first two octets of the 
       MAC address for both address
     *************************************************************************/
    /**************************************************************************
       Set up unicast packet filter reg 2 to be the octets 2 - 5 of the 
       MAC address for second address
     *************************************************************************/
    /**************************************************************************
       Set up unicast packet filter reg 3 to be the octets 2 - 5 of the 
       MAC address for first address
     *************************************************************************/
    uni_pf3.bits.addr1_1 = pAdapter->CurrentAddress[0];
    uni_pf3.bits.addr1_2 = pAdapter->CurrentAddress[1];
    uni_pf3.bits.addr2_1 = pAdapter->CurrentAddress[0];
    uni_pf3.bits.addr2_2 = pAdapter->CurrentAddress[1];

    uni_pf2.bits.addr2_3 = pAdapter->CurrentAddress[2];
    uni_pf2.bits.addr2_4 = pAdapter->CurrentAddress[3];
    uni_pf2.bits.addr2_5 = pAdapter->CurrentAddress[4];
    uni_pf2.bits.addr2_6 = pAdapter->CurrentAddress[5];

    uni_pf1.bits.addr1_3 = pAdapter->CurrentAddress[2];
    uni_pf1.bits.addr1_4 = pAdapter->CurrentAddress[3];
    uni_pf1.bits.addr1_5 = pAdapter->CurrentAddress[4];
    uni_pf1.bits.addr1_6 = pAdapter->CurrentAddress[5];

    if( pAdapter->CSRAddress->global.pm_csr.bits.pm_phy_sw_coma == 0 )
    {
        pAdapter->CSRAddress->rxmac.uni_pf_addr1 = uni_pf1;
        pAdapter->CSRAddress->rxmac.uni_pf_addr2 = uni_pf2;
        pAdapter->CSRAddress->rxmac.uni_pf_addr3 = uni_pf3;
    }


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/
