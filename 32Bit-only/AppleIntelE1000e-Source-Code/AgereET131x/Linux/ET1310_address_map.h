/*******************************************************************************
 * Agere Systems Inc.
 * 10/100/1000 Base-T Ethernet Driver for the ET1301 and ET131x series MACs
 *
 * Copyright � 2005 Agere Systems Inc. 
 * All rights reserved.
 *   http://www.agere.com
 *
 *------------------------------------------------------------------------------
 *
 * ET1310_address_map.h - Contains the register mapping for the ET1310
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
 * Copyright � 2005 Agere Systems Inc.
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
 * THIS SOFTWARE IS PROVIDED �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES,
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
         $Date: 2005/10/28 18:43:43 $
     $Revision: 1.8 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef _ET1310_ADDRESS_MAP_H_
#define _ET1310_ADDRESS_MAP_H_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
   some typedefs for compiler
 *****************************************************************************/
typedef s8  INT8,   *PINT8;
typedef s16 INT16,  *PINT16;
typedef s32 INT32,  *PINT32;
typedef s64 INT64,  *PINT64;

typedef u8  UINT8,  *PUINT8;
typedef u16 UINT16, *PUINT16;
typedef u32 UINT32, *PUINT32;
typedef u64 UINT64, *PUINT64;

typedef u8  UCHAR,  *PUCHAR;




/*===========================================================================*/
/*===========================================================================*/
/*===                START OF GLOBAL REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for tx queue start address reg in global address map 
   located at address 0x0000 
 *****************************************************************************/
typedef union _TXQ_START_ADDR_t
{
    UINT32 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:22;                           //bits 10-31
        UINT32 txq_start:10;                        //bits 0-9
    #else
        UINT32 txq_start:10;                        //bits 0-9
        UINT32 unused:22;                           //bits 10-31
    #endif
    } bits;
} 
TXQ_START_ADDR_t, *PTXQ_START_ADDR_t;


/******************************************************************************
   structure for tx queue end address reg in global address map
   located at address 0x0004
 *****************************************************************************/
typedef union _TXQ_END_ADDR_t
{
    UINT32 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:22;                           //bits 10-31
        UINT32 txq_end:10;                          //bits 0-9
    #else
        UINT32 txq_end:10;                          //bits 0-9
        UINT32 unused:22;                           //bits 10-31
    #endif    
    } bits;
} 
TXQ_END_ADDR_t, *PTXQ_END_ADDR_t;


/******************************************************************************
   structure for rx queue start address reg in global address map
   located at address 0x0008
 *****************************************************************************/
typedef union _RXQ_START_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:22;                           //bits 10-31
        UINT32 rxq_start_addr:10;                   //bits 0-9
    #else
        UINT32 rxq_start_addr:10;                   //bits 0-9
        UINT32 unused:22;                           //bits 10-31
    #endif
    } bits;
} 
RXQ_START_ADDR_t, *PRXQ_START_ADDR_t;


/******************************************************************************
   structure for rx queue end address reg in global address map
   located at address 0x000C
 *****************************************************************************/
typedef union _RXQ_END_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:22;                           //bits 10-31
        UINT32 rxq_end_addr:10;                     //bits 0-9
    #else
        UINT32 rxq_end_addr:10;                     //bits 0-9
        UINT32 unused:22;                           //bits 10-31
    #endif
    } bits;
} 
RXQ_END_ADDR_t, *PRXQ_END_ADDR_t;


/******************************************************************************
   structure for power management control status reg in global address map
   located at address 0x0010
 *****************************************************************************/
typedef union _PM_CSR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:22;                           //bits 10-31
        UINT32 pm_jagcore_rx_rdy:1;                 //bit 9
        UINT32 pm_jagcore_tx_rdy:1;                 //bit 8
        UINT32 pm_phy_lped_en:1;                    //bit 7
        UINT32 pm_phy_sw_coma:1;                    //bit 6
        UINT32 pm_rxclk_gate:1;                     //bit 5
        UINT32 pm_txclk_gate:1;                     //bit 4
        UINT32 pm_sysclk_gate:1;                    //bit 3
        UINT32 pm_jagcore_rx_en:1;                  //bit 2
        UINT32 pm_jagcore_tx_en:1;                  //bit 1
        UINT32 pm_gigephy_en:1;                     //bit 0
    #else
        UINT32 pm_gigephy_en:1;                     //bit 0
        UINT32 pm_jagcore_tx_en:1;                  //bit 1
        UINT32 pm_jagcore_rx_en:1;                  //bit 2
        UINT32 pm_sysclk_gate:1;                    //bit 3
        UINT32 pm_txclk_gate:1;                     //bit 4
        UINT32 pm_rxclk_gate:1;                     //bit 5
        UINT32 pm_phy_sw_coma:1;                    //bit 6
        UINT32 pm_phy_lped_en:1;                    //bit 7
        UINT32 pm_jagcore_tx_rdy:1;                 //bit 8
        UINT32 pm_jagcore_rx_rdy:1;                 //bit 9
        UINT32 unused:22;                           //bits 10-31
    #endif
    } bits;
} 
PM_CSR_t, *PPM_CSR_t;


/******************************************************************************
   structure for interrupt status reg in global address map
   located at address 0x0018
 *****************************************************************************/
typedef union _INT_STATUS_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused5:11;                          //bits 21-31
        UINT32 slv_timeout:1;                       //bit 20
        UINT32 mac_stat_interrupt:1;                //bit 19
        UINT32 rxmac_interrupt:1;                   //bit 18
        UINT32 txmac_interrupt:1;                   //bit 17
        UINT32 phy_interrupt:1;                     //bit 16
        UINT32 wake_on_lan:1;                       //bit 15
        UINT32 watchdog_interrupt:1;                //bit 14
        UINT32 unused4:4;                           //bits 10-13
        UINT32 rxdma_err:1;                         //bit 9
        UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
        UINT32 rxdma_fb_ring1_low:1;                //bit 7
        UINT32 rxdma_fb_ring0_low:1;                //bit 6
        UINT32 rxdma_xfr_done:1;                    //bit 5
        UINT32 txdma_err:1;                         //bit 4
        UINT32 txdma_isr:1;                         //bit 3
        UINT32 unused3:1;                           //bit 2
        UINT32 unused2:1;                           //bit 1
        UINT32 unused1:1;                           //bit 0
    #else
        UINT32 unused1:1;                           //bit 0
        UINT32 unused2:1;                           //bit 1
        UINT32 unused3:1;                           //bit 2
        UINT32 txdma_isr:1;                         //bit 3
        UINT32 txdma_err:1;                         //bit 4
        UINT32 rxdma_xfr_done:1;                    //bit 5
        UINT32 rxdma_fb_ring0_low:1;                //bit 6
        UINT32 rxdma_fb_ring1_low:1;                //bit 7
        UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
        UINT32 rxdma_err:1;                         //bit 9
        UINT32 unused4:4;                           //bits 10-13
        UINT32 watchdog_interrupt:1;                //bit 14
        UINT32 wake_on_lan:1;                       //bit 15
        UINT32 phy_interrupt:1;                     //bit 16
        UINT32 txmac_interrupt:1;                   //bit 17
        UINT32 rxmac_interrupt:1;                   //bit 18
        UINT32 mac_stat_interrupt:1;                //bit 19
        UINT32 slv_timeout:1;                       //bit 20
        UINT32 unused5:11;                          //bits 21-31
    #endif
    } bits;
} 
INT_STATUS_t, *PINT_STATUS_t;


/******************************************************************************
   structure for interrupt mask reg in global address map
   located at address 0x001C
 *****************************************************************************/
typedef union _INT_MASK_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused5:11;                          //bits 21-31
        UINT32 slv_timeout:1;                       //bit 20
        UINT32 mac_stat_interrupt:1;                //bit 19
        UINT32 rxmac_interrupt:1;                   //bit 18
        UINT32 txmac_interrupt:1;                   //bit 17
        UINT32 phy_interrupt:1;                     //bit 16
        UINT32 wake_on_lan:1;                       //bit 15
        UINT32 unused4:5;                           //bits 10-14
        UINT32 rxdma_err:1;                         //bit 9
        UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
        UINT32 rxdma_fb_ring1_low:1;                //bit 7
        UINT32 rxdma_fb_ring0_low:1;                //bit 6
        UINT32 rxdma_xfr_done:1;                    //bit 5
        UINT32 txdma_err:1;                         //bit 4
        UINT32 txdma_isr:1;                         //bit 3
        UINT32 unused3:1;                           //bit 2
        UINT32 unused2:1;                           //bit 1
        UINT32 unused1:1;                           //bit 0
    #else
        UINT32 unused1:1;                           //bit 0
        UINT32 unused2:1;                           //bit 1
        UINT32 unused3:1;                           //bit 2
        UINT32 txdma_isr:1;                         //bit 3
        UINT32 txdma_err:1;                         //bit 4
        UINT32 rxdma_xfr_done:1;                    //bit 5
        UINT32 rxdma_fb_ring0_low:1;                //bit 6
        UINT32 rxdma_fb_ring1_low:1;                //bit 7
        UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
        UINT32 rxdma_err:1;                         //bit 9
        UINT32 unused4:5;                           //bits 10-14
        UINT32 wake_on_lan:1;                       //bit 15
        UINT32 phy_interrupt:1;                     //bit 16
        UINT32 txmac_interrupt:1;                   //bit 17
        UINT32 rxmac_interrupt:1;                   //bit 18
        UINT32 mac_stat_interrupt:1;                //bit 19
        UINT32 slv_timeout:1;                       //bit 20
        UINT32 unused5:11;                          //bits 21-31
    #endif
    } bits;
} 
INT_MASK_t, *PINT_MASK_t;


/******************************************************************************
   structure for interrupt alias clear mask reg in global address map
   located at address 0x0020
 *****************************************************************************/
typedef union _INT_ALIAS_CLR_EN_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused5:11;                          //bits 21-31
        UINT32 slv_timeout:1;                       //bit 20
        UINT32 mac_stat_interrupt:1;                //bit 19
        UINT32 rxmac_interrupt:1;                   //bit 18
        UINT32 txmac_interrupt:1;                   //bit 17
        UINT32 phy_interrupt:1;                     //bit 16
        UINT32 wake_on_lan:1;                       //bit 15
        UINT32 watchdog_interrupt:1;                //bit 14
        UINT32 unused4:4;                           //bits 10-13
        UINT32 rxdma_err:1;                         //bit 9
        UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
        UINT32 rxdma_fb_ring1_low:1;                //bit 7
        UINT32 rxdma_fb_ring0_low:1;                //bit 6
        UINT32 rxdma_xfr_done:1;                    //bit 5
        UINT32 txdma_err:1;                         //bit 4
        UINT32 txdma_isr:1;                         //bit 3
        UINT32 unused3:1;                           //bit 2
        UINT32 unused2:1;                           //bit 1
        UINT32 unused1:1;                           //bit 0
    #else
        UINT32 unused1:1;                           //bit 0
        UINT32 unused2:1;                           //bit 1
        UINT32 unused3:1;                           //bit 2
        UINT32 txdma_isr:1;                         //bit 3
        UINT32 txdma_err:1;                         //bit 4
        UINT32 rxdma_xfr_done:1;                    //bit 5
        UINT32 rxdma_fb_ring0_low:1;                //bit 6
        UINT32 rxdma_fb_ring1_low:1;                //bit 7
        UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
        UINT32 rxdma_err:1;                         //bit 9
        UINT32 unused4:4;                           //bits 10-13
        UINT32 watchdog_interrupt:1;                //bit 14
        UINT32 wake_on_lan:1;                       //bit 15
        UINT32 phy_interrupt:1;                     //bit 16
        UINT32 txmac_interrupt:1;                   //bit 17
        UINT32 rxmac_interrupt:1;                   //bit 18
        UINT32 mac_stat_interrupt:1;                //bit 19
        UINT32 slv_timeout:1;                       //bit 20
        UINT32 unused5:11;                          //bits 21-31
    #endif
    } bits;
} 
INT_ALIAS_CLR_EN_t, *PINT_ALIAS_CLR_EN_t;


/******************************************************************************
   structure for interrupt status alias reg in global address map
   located at address 0x0024
 *****************************************************************************/
typedef union _INT_STATUS_ALIAS_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused5:11;                          //bits 21-31
        UINT32 slv_timeout:1;                       //bit 20
        UINT32 mac_stat_interrupt:1;                //bit 19
        UINT32 rxmac_interrupt:1;                   //bit 18
        UINT32 txmac_interrupt:1;                   //bit 17
        UINT32 phy_interrupt:1;                     //bit 16
        UINT32 wake_on_lan:1;                       //bit 15
        UINT32 watchdog_interrupt:1;                //bit 14
        UINT32 unused4:4;                           //bits 10-13
        UINT32 rxdma_err:1;                         //bit 9
        UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
        UINT32 rxdma_fb_ring1_low:1;                //bit 7
        UINT32 rxdma_fb_ring0_low:1;                //bit 6
        UINT32 rxdma_xfr_done:1;                    //bit 5
        UINT32 txdma_err:1;                         //bit 4
        UINT32 txdma_isr:1;                         //bit 3
        UINT32 unused3:1;                           //bit 2
        UINT32 unused2:1;                           //bit 1
        UINT32 unused1:1;                           //bit 0
    #else
        UINT32 unused1:1;                           //bit 0
        UINT32 unused2:1;                           //bit 1
        UINT32 unused3:1;                           //bit 2
        UINT32 txdma_isr:1;                         //bit 3
        UINT32 txdma_err:1;                         //bit 4
        UINT32 rxdma_xfr_done:1;                    //bit 5
        UINT32 rxdma_fb_ring0_low:1;                //bit 6
        UINT32 rxdma_fb_ring1_low:1;                //bit 7
        UINT32 rxdma_pkt_stat_ring_low:1;           //bit 8
        UINT32 rxdma_err:1;                         //bit 9
        UINT32 unused4:4;                           //bits 10-13
        UINT32 watchdog_interrupt:1;                //bit 14
        UINT32 wake_on_lan:1;                       //bit 15
        UINT32 phy_interrupt:1;                     //bit 16
        UINT32 txmac_interrupt:1;                   //bit 17
        UINT32 rxmac_interrupt:1;                   //bit 18
        UINT32 mac_stat_interrupt:1;                //bit 19
        UINT32 slv_timeout:1;                       //bit 20
        UINT32 unused5:11;                          //bits 21-31
    #endif
    } bits;
} 
INT_STATUS_ALIAS_t, *PINT_STATUS_ALIAS_t;


/******************************************************************************
   structure for software reset reg in global address map
   located at address 0x0028
 *****************************************************************************/
typedef union _SW_RESET_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 selfclr_disable:1;                   //bit 31
        UINT32 unused:24;                           //bits 7-30
        UINT32 mmc_sw_reset:1;                      //bit 6
        UINT32 mac_stat_sw_reset:1;                 //bit 5
        UINT32 mac_sw_reset:1;                      //bit 4
        UINT32 rxmac_sw_reset:1;                    //bit 3
        UINT32 txmac_sw_reset:1;                    //bit 2
        UINT32 rxdma_sw_reset:1;                    //bit 1
        UINT32 txdma_sw_reset:1;                    //bit 0
    #else
        UINT32 txdma_sw_reset:1;                    //bit 0
        UINT32 rxdma_sw_reset:1;                    //bit 1
        UINT32 txmac_sw_reset:1;                    //bit 2
        UINT32 rxmac_sw_reset:1;                    //bit 3
        UINT32 mac_sw_reset:1;                      //bit 4
        UINT32 mac_stat_sw_reset:1;                 //bit 5
        UINT32 mmc_sw_reset:1;                      //bit 6
        UINT32 unused:24;                           //bits 7-30
        UINT32 selfclr_disable:1;                   //bit 31
    #endif
    } bits;
} 
SW_RESET_t, *PSW_RESET_t;


/******************************************************************************
   structure for SLV Timer reg in global address map
   located at address 0x002C
 *****************************************************************************/
typedef union _SLV_TIMER_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:8;                            //bits 24-31
        UINT32 timer_ini:24;                        //bits 0-23
    #else
        UINT32 timer_ini:24;                        //bits 0-23
        UINT32 unused:8;                            //bits 24-31
    #endif
    } bits;
} 
SLV_TIMER_t, *PSLV_TIMER_t;


/******************************************************************************
   structure for MSI Configuration reg in global address map
   located at address 0x0030
 *****************************************************************************/
typedef union _MSI_CONFIG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused1:13;                          //bits 19-31
        UINT32 msi_tc:3;                            //bits 16-18
        UINT32 unused2:11;                          //bits 5-15
        UINT32 msi_vector:5;                        //bits 0-4
    #else
        UINT32 msi_vector:5;                        //bits 0-4
        UINT32 unused2:11;                          //bits 5-15
        UINT32 msi_tc:3;                            //bits 16-18
        UINT32 unused1:13;                          //bits 19-31
    #endif
    } bits;
} 
MSI_CONFIG_t, *PMSI_CONFIG_t;


/******************************************************************************
   structure for Loopback reg in global address map
   located at address 0x0034
 *****************************************************************************/
typedef union _LOOPBACK_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:30;                           //bits 2-31
        UINT32 dma_loopback:1;                      //bit 1
        UINT32 mac_loopback:1;                      //bit 0
    #else
        UINT32 mac_loopback:1;                      //bit 0
        UINT32 dma_loopback:1;                      //bit 1
        UINT32 unused:30;                           //bits 2-31
    #endif
    } bits;
} 
LOOPBACK_t, *PLOOPBACK_t;


/******************************************************************************
   GLOBAL Module of JAGCore Address Mapping
   Located at address 0x0000
 *****************************************************************************/
typedef struct _GLOBAL_t
{                                                   //Location:
    TXQ_START_ADDR_t    txq_start_addr;             //  0x0000
    TXQ_END_ADDR_t      txq_end_addr;               //  0x0004
    RXQ_START_ADDR_t    rxq_start_addr;             //  0x0008
    RXQ_END_ADDR_t      rxq_end_addr;               //  0x000C
    PM_CSR_t            pm_csr;                     //  0x0010
    UINT32              unused;                     //  0x0014
    INT_STATUS_t        int_status;                 //  0x0018
    INT_MASK_t          int_mask;                   //  0x001C
    INT_ALIAS_CLR_EN_t  int_alias_clr_en;           //  0x0020
    INT_STATUS_ALIAS_t  int_status_alias;           //  0x0024
    SW_RESET_t          sw_reset;                   //  0x0028
    SLV_TIMER_t         slv_timer;                  //  0x002C
    MSI_CONFIG_t        msi_config;                 //  0x0030
    LOOPBACK_t          loopback;                   //  0x0034
    UINT32              watchdog_timer;             //  0x0038
} 
GLOBAL_t, *PGLOBAL_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                 END OF GLOBAL REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/




/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF TXDMA REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for txdma control status reg in txdma address map
   located at address 0x1000
 *****************************************************************************/
typedef union _TXDMA_CSR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:19;                          //bits 13-31
        UINT32 traffic_class:4;                     //bits 9-12
        UINT32 sngl_epkt_mode:1;                    //bit 8
        UINT32 cache_thrshld:4;                     //bits 4-7
        UINT32 unused1:2;                           //bits 2-3
        UINT32 drop_TLP_disable:1;                  //bit 1
        UINT32 halt:1;                              //bit 0
    #else
        UINT32 halt:1;                              //bit 0
        UINT32 drop_TLP_disable:1;                  //bit 1
        UINT32 unused1:2;                           //bits 2-3
        UINT32 cache_thrshld:4;                     //bits 4-7
        UINT32 sngl_epkt_mode:1;                    //bit 8
        UINT32 traffic_class:4;                     //bits 9-12
        UINT32 unused2:19;                          //bits 13-31
    #endif
    } bits;
} 
TXDMA_CSR_t, *PTXDMA_CSR_t;


/******************************************************************************
   structure for txdma packet ring base address hi reg in txdma address map
   located at address 0x1004
 *****************************************************************************/
typedef struct _TXDMA_PR_BASE_HI_t
{
    UINT32 addr_hi;                                 //bits 0-31
} 
TXDMA_PR_BASE_HI_t, *PTXDMA_PR_BASE_HI_t;


/******************************************************************************
   structure for txdma packet ring base address low reg in txdma address map
   located at address 0x1008
 *****************************************************************************/
typedef struct _TXDMA_PR_BASE_LO_t
{
    UINT32 addr_lo;                                 //bits 0-31
} 
TXDMA_PR_BASE_LO_t, *PTXDMA_PR_BASE_LO_t;


/******************************************************************************
   structure for txdma packet ring number of descriptor reg in txdma address 
   map.  Located at address 0x100C
 *****************************************************************************/
typedef union _TXDMA_PR_NUM_DES_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:22;                           //bits 10-31
        UINT32 pr_ndes:10;                          //bits 0-9
    #else
        UINT32 pr_ndes:10;                          //bits 0-9
        UINT32 unused:22;                           //bits 10-31
    #endif
    } bits;
} 
TXDMA_PR_NUM_DES_t, *PTXDMA_PR_NUM_DES_t;


/******************************************************************************
   structure for txdma tx queue write address reg in txdma address map
   located at address 0x1010
 *****************************************************************************/
typedef  union _TXDMA_TXQ_WR_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 txq_wr_wrap:1;                       //bit 10
        UINT32 txq_wr:10;                           //bits 0-9
    #else
        UINT32 txq_wr:10;                           //bits 0-9
        UINT32 txq_wr_wrap:1;                       //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
TXDMA_TXQ_WR_ADDR_t, *PTXDMA_TXQ_WR_ADDR_t;


/******************************************************************************
   structure for txdma tx queue write address external reg in txdma address map
   located at address 0x1014
 *****************************************************************************/
typedef union _TXDMA_TXQ_WR_ADDR_EXT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 txq_wr_ext_wrap:1;                   //bit 10
        UINT32 txq_wr_ext:10;                       //bits 0-9
    #else
        UINT32 txq_wr_ext:10;                       //bits 0-9
        UINT32 txq_wr_ext_wrap:1;                   //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
TXDMA_TXQ_WR_ADDR_EXT_t, *PTXDMA_TXQ_WR_ADDR_EXT_t;


/******************************************************************************
   structure for txdma tx queue read address reg in txdma address map
   located at address 0x1018
 *****************************************************************************/
typedef union _TXDMA_TXQ_RD_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 txq_rd_wrap:1;                       //bit 10
        UINT32 txq_rd:10;                           //bits 0-9
    #else
        UINT32 txq_rd:10;                           //bits 0-9
        UINT32 txq_rd_wrap:1;                       //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
TXDMA_TXQ_RD_ADDR_t, *PTXDMA_TXQ_RD_ADDR_t;


/******************************************************************************
   structure for txdma status writeback address hi reg in txdma address map
   located at address 0x101C
 *****************************************************************************/
typedef struct _TXDMA_DMA_WB_ADDR_HI_t
{
    UINT32 addr_hi;                              //bits 0-31
} 
TXDMA_DMA_WB_ADDR_HI_t, *PTXDMA_DMA_WB_ADDR_HI_t;


/******************************************************************************
   structure for txdma status writeback address lo reg in txdma address map
   located at address 0x1020
 *****************************************************************************/
typedef struct _TXDMA_DMA_WB_ADDR_LO_t
{
    UINT32 addr_lo;                              //bits 0-31
} 
TXDMA_DMA_WB_ADDR_LO_t, *PTXDMA_DMA_WB_ADDR_LO_t;


/******************************************************************************
   structure for txdma service request reg in txdma address map
   located at address 0x1024
 *****************************************************************************/
typedef union _TXDMA_SERVICE_REQUEST_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 serv_req_wrap:1;                     //bit 10
        UINT32 serv_req:10;                         //bits 0-9
    #else
        UINT32 serv_req:10;                         //bits 0-9
        UINT32 serv_req_wrap:1;                     //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
TXDMA_SERVICE_REQUEST_t, *PTXDMA_SERVICE_REQUEST_t;


/******************************************************************************
   structure for txdma service complete reg in txdma address map
   located at address 0x1028
 *****************************************************************************/
typedef union _TXDMA_SERVICE_COMPLETE_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 serv_cpl_wrap:1;                     //bit 10
        UINT32 serv_cpl:10;                         //bits 0-9
    #else
        UINT32 serv_cpl:10;                         //bits 0-9
        UINT32 serv_cpl_wrap:1;                     //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
TXDMA_SERVICE_COMPLETE_t, *PTXDMA_SERVICE_COMPLETE_t;


/******************************************************************************
   structure for txdma tx descriptor cache read index reg in txdma address map
   located at address 0x102C
 *****************************************************************************/
typedef union _TXDMA_CACHE_RD_INDEX_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:27;                           //bits 5-31
        UINT32 rdi_wrap:1;                          //bit 4
        UINT32 rdi:4;                               //bit 0-3
    #else
        UINT32 rdi:4;                               //bits 0-3
        UINT32 rdi_wrap:1;                          //bit 4
        UINT32 unused:27;                           //bits 5-31
    #endif
    } bits;
} 
TXDMA_CACHE_RD_INDEX_t, *PTXDMA_CACHE_RD_INDEX_t;


/******************************************************************************
   structure for txdma tx descriptor cache write index reg in txdma address map
   located at address 0x1030
 *****************************************************************************/
typedef union _TXDMA_CACHE_WR_INDEX_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:27;                           //bits 5-31
        UINT32 wri_wrap:1;                          //bit 4
        UINT32 wri:4;                               //bit 0-3
    #else
        UINT32 wri:4;                               //bits 0-3
        UINT32 wri_wrap:1;                          //bit 4
        UINT32 unused:27;                           //bits 5-31
    #endif
    } bits;
} 
TXDMA_CACHE_WR_INDEX_t, *PTXDMA_CACHE_WR_INDEX_t;


/******************************************************************************
   structure for txdma error reg in txdma address map
   located at address 0x1034
 *****************************************************************************/
typedef union _TXDMA_ERROR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused3:22;                          //bits 10-31
        UINT32 WrbkRewind:1;                        //bit 9
        UINT32 WrbkResend:1;                        //bit 8
        UINT32 unused2:2;                           //bits 6-7
        UINT32 DescrRewind:1;                       //bit 5
        UINT32 DescrResend:1;                       //bit 4
        UINT32 unused1:2;                           //bits 2-3
        UINT32 PyldRewind:1;                        //bit 1
        UINT32 PyldResend:1;                        //bit 0
    #else
        UINT32 PyldResend:1;                        //bit 0
        UINT32 PyldRewind:1;                        //bit 1
        UINT32 unused1:2;                           //bits 2-3
        UINT32 DescrResend:1;                       //bit 4
        UINT32 DescrRewind:1;                       //bit 5
        UINT32 unused2:2;                           //bits 6-7
        UINT32 WrbkResend:1;                        //bit 8
        UINT32 WrbkRewind:1;                        //bit 9
        UINT32 unused3:22;                          //bits 10-31
    #endif
    } bits;
}
TXDMA_ERROR_t, *PTXDMA_ERROR_t;


/******************************************************************************
   Tx DMA Module of JAGCore Address Mapping
   Located at address 0x1000
 *****************************************************************************/
typedef struct _TXDMA_t
{                                                   //Location:
    TXDMA_CSR_t                 csr;                //  0x1000
    TXDMA_PR_BASE_HI_t          pr_base_hi;         //  0x1004
    TXDMA_PR_BASE_LO_t          pr_base_lo;         //  0x1008
    TXDMA_PR_NUM_DES_t          pr_num_des;         //  0x100C
    TXDMA_TXQ_WR_ADDR_t         txq_wr_addr;        //  0x1010
    TXDMA_TXQ_WR_ADDR_EXT_t     txq_wr_addr_ext;    //  0x1014
    TXDMA_TXQ_RD_ADDR_t         txq_rd_addr;        //  0x1018
    TXDMA_DMA_WB_ADDR_HI_t      dma_wb_base_hi;     //  0x101C
    TXDMA_DMA_WB_ADDR_LO_t      dma_wb_base_lo;     //  0x1020
    TXDMA_SERVICE_REQUEST_t     service_request;    //  0x1024
    TXDMA_SERVICE_COMPLETE_t    service_complete;   //  0x1028
    TXDMA_CACHE_RD_INDEX_t      cache_rd_index;     //  0x102C
    TXDMA_CACHE_WR_INDEX_t      cache_wr_index;     //  0x1030
    TXDMA_ERROR_t               TxDmaError;         //  0x1034
    UINT32                      DescAbortCount;     //  0x1038
    UINT32                      PayloadAbortCnt;    //  0x103c
    UINT32                      WriteBackAbortCnt;  //  0x1040
    UINT32                      DescTimeoutCnt;     //  0x1044
    UINT32                      PayloadTimeoutCnt;  //  0x1048
    UINT32                      WriteBackTimeoutCnt;//  0x104c
    UINT32                      DescErrorCount;     //  0x1050
    UINT32                      PayloadErrorCnt;    //  0x1054
    UINT32                      WriteBackErrorCnt;  //  0x1058
    UINT32                      DroppedTLPCount;    //  0x105c
    TXDMA_SERVICE_COMPLETE_t    NewServiceComplete; //  0x1060
    UINT32                      EthernetPacketCount;//  0x1064
}
TXDMA_t, *PTXDMA_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF TXDMA REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF RXDMA REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for control status reg in rxdma address map
   Located at address 0x2000
 *****************************************************************************/
typedef union _RXDMA_CSR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:14;                          //bits 18-31
        UINT32 halt_status:1;                       //bit 17
        UINT32 pkt_done_flush:1;                    //bit 16
        UINT32 pkt_drop_disable:1;                  //bit 15
        UINT32 unused1:1;                           //bit 14
        UINT32 fbr1_enable:1;                       //bit 13
        UINT32 fbr1_size:2;                         //bits 11-12
        UINT32 fbr0_enable:1;                       //bit 10
        UINT32 fbr0_size:2;                         //bits 8-9
        UINT32 dma_big_endian:1;                    //bit 7
        UINT32 pkt_big_endian:1;                    //bit 6
        UINT32 psr_big_endian:1;                    //bit 5
        UINT32 fbr_big_endian:1;                    //bit 4
        UINT32 tc:3;                                //bits 1-3
        UINT32 halt:1;                              //bit 0
    #else
        UINT32 halt:1;                              //bit 0
        UINT32 tc:3;                                //bits 1-3
        UINT32 fbr_big_endian:1;                    //bit 4
        UINT32 psr_big_endian:1;                    //bit 5
        UINT32 pkt_big_endian:1;                    //bit 6
        UINT32 dma_big_endian:1;                    //bit 7
        UINT32 fbr0_size:2;                         //bits 8-9
        UINT32 fbr0_enable:1;                       //bit 10
        UINT32 fbr1_size:2;                         //bits 11-12
        UINT32 fbr1_enable:1;                       //bit 13
        UINT32 unused1:1;                           //bit 14
        UINT32 pkt_drop_disable:1;                  //bit 15
        UINT32 pkt_done_flush:1;                    //bit 16
        UINT32 halt_status:1;                       //bit 17
        UINT32 unused2:14;                          //bits 18-31
    #endif
    } bits;
} 
RXDMA_CSR_t, *PRXDMA_CSR_t;


/******************************************************************************
   structure for dma writeback lo reg in rxdma address map
   located at address 0x2004
 *****************************************************************************/
typedef struct _RXDMA_DMA_WB_BASE_LO_t
{
    UINT32 addr_lo;                                 //bits 0-31
} 
RXDMA_DMA_WB_BASE_LO_t, *PRXDMA_DMA_WB_BASE_LO_t;


/******************************************************************************
   structure for dma writeback hi reg in rxdma address map
   located at address 0x2008
 *****************************************************************************/
typedef struct _RXDMA_DMA_WB_BASE_HI_t
{
    UINT32 addr_hi;                                 //bits 0-31
} 
RXDMA_DMA_WB_BASE_HI_t, *PRXDMA_DMA_WB_BASE_HI_t;


/******************************************************************************
   structure for number of packets done reg in rxdma address map
   located at address 0x200C
 *****************************************************************************/
typedef union _RXDMA_NUM_PKT_DONE_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:24;                           //bits 8-31
        UINT32 num_done:8;                          //bits 0-7
    #else
        UINT32 num_done:8;                          //bits 0-7
        UINT32 unused:24;                           //bits 8-31
    #endif
    } bits;
} 
RXDMA_NUM_PKT_DONE_t, *PRXDMA_NUM_PKT_DONE_t;


/******************************************************************************
   structure for max packet time reg in rxdma address map
   located at address 0x2010
 *****************************************************************************/
typedef union _RXDMA_MAX_PKT_TIME_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:14;                           //bits 18-31
        UINT32 time_done:18;                        //bits 0-17
    #else
        UINT32 time_done:18;                        //bits 0-17
        UINT32 unused:14;                           //bits 18-31
    #endif
    } bits;
} 
RXDMA_MAX_PKT_TIME_t, *PRXDMA_MAX_PKT_TIME_t;


/******************************************************************************
   structure for rx queue read address reg in rxdma address map
   located at address 0x2014
 *****************************************************************************/
typedef union _RXDMA_RXQ_RD_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 rxq_rd_wrap:1;                       //bit 10
        UINT32 rxq_rd:10;                           //bits 0-9
    #else
        UINT32 rxq_rd:10;                           //bits 0-9
        UINT32 rxq_rd_wrap:1;                       //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
RXDMA_RXQ_RD_ADDR_t, *PRXDMA_RXQ_RD_ADDR_t;


/******************************************************************************
   structure for rx queue read address external reg in rxdma address map
   located at address 0x2018
 *****************************************************************************/
typedef union _RXDMA_RXQ_RD_ADDR_EXT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 rxq_rd_ext_wrap:1;                   //bit 10
        UINT32 rxq_rd_ext:10;                       //bits 0-9
    #else
        UINT32 rxq_rd_ext:10;                       //bits 0-9
        UINT32 rxq_rd_ext_wrap:1;                   //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
RXDMA_RXQ_RD_ADDR_EXT_t, *PRXDMA_RXQ_RD_ADDR_EXT_t;


/******************************************************************************
   structure for rx queue write address reg in rxdma address map
   located at address 0x201C
 *****************************************************************************/
typedef union _RXDMA_RXQ_WR_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 rxq_wr_wrap:1;                       //bit 10
        UINT32 rxq_wr:10;                           //bits 0-9
    #else
        UINT32 rxq_wr:10;                           //bits 0-9
        UINT32 rxq_wr_wrap:1;                       //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
RXDMA_RXQ_WR_ADDR_t, *PRXDMA_RXQ_WR_ADDR_t;


/******************************************************************************
   structure for packet status ring base address lo reg in rxdma address map
   located at address 0x2020
 *****************************************************************************/
typedef struct _RXDMA_PSR_BASE_LO_t
{
    UINT32 addr_lo;                                 //bits 0-31
} 
RXDMA_PSR_BASE_LO_t, *PRXDMA_PSR_BASE_LO_t;


/******************************************************************************
   structure for packet status ring base address hi reg in rxdma address map
   located at address 0x2024
 *****************************************************************************/
typedef struct _RXDMA_PSR_BASE_HI_t
{
    UINT32 addr_hi;                                  //bits 0-31
} 
RXDMA_PSR_BASE_HI_t, *PRXDMA_PSR_BASE_HI_t;


/******************************************************************************
   structure for packet status ring number of descriptors reg in rxdma address 
   map.  Located at address 0x2028
 *****************************************************************************/
typedef union _RXDMA_PSR_NUM_DES_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:20;                           //bits 12-31
        UINT32 psr_ndes:12;                         //bit 0-11
    #else
        UINT32 psr_ndes:12;                         //bit 0-11
        UINT32 unused:20;                           //bits 12-31
    #endif
    } bits;
} 
RXDMA_PSR_NUM_DES_t, *PRXDMA_PSR_NUM_DES_t;


/******************************************************************************
   structure for packet status ring available offset reg in rxdma address map
   located at address 0x202C
 *****************************************************************************/
typedef union _RXDMA_PSR_AVAIL_OFFSET_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:19;                           //bits 13-31
        UINT32 psr_avail_wrap:1;                    //bit 12
        UINT32 psr_avail:12;                        //bit 0-11
    #else
        UINT32 psr_avail:12;                        //bit 0-11
        UINT32 psr_avail_wrap:1;                    //bit 12
        UINT32 unused:19;                           //bits 13-31
    #endif
    } bits;
} 
RXDMA_PSR_AVAIL_OFFSET_t, *PRXDMA_PSR_AVAIL_OFFSET_t;


/******************************************************************************
   structure for packet status ring full offset reg in rxdma address map
   located at address 0x2030
 *****************************************************************************/
typedef union _RXDMA_PSR_FULL_OFFSET_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:19;                           //bits 13-31
        UINT32 psr_full_wrap:1;                     //bit 12
        UINT32 psr_full:12;                         //bit 0-11
    #else
        UINT32 psr_full:12;                         //bit 0-11
        UINT32 psr_full_wrap:1;                     //bit 12
        UINT32 unused:19;                           //bits 13-31
    #endif
    } bits;
} 
RXDMA_PSR_FULL_OFFSET_t, *PRXDMA_PSR_FULL_OFFSET_t;


/******************************************************************************
   structure for packet status ring access index reg in rxdma address map
   located at address 0x2034
 *****************************************************************************/
typedef union _RXDMA_PSR_ACCESS_INDEX_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:27;                           //bits 5-31
        UINT32 psr_ai:5;                            //bits 0-4
    #else
        UINT32 psr_ai:5;                            //bits 0-4
        UINT32 unused:27;                           //bits 5-31
    #endif
    } bits;
} 
RXDMA_PSR_ACCESS_INDEX_t, *PRXDMA_PSR_ACCESS_INDEX_t;


/******************************************************************************
   structure for packet status ring minimum descriptors reg in rxdma address 
   map.  Located at address 0x2038
 *****************************************************************************/
typedef union _RXDMA_PSR_MIN_DES_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:20;                           //bits 12-31
        UINT32 psr_min:12;                          //bits 0-11
    #else
        UINT32 psr_min:12;                          //bits 0-11
        UINT32 unused:20;                           //bits 12-31
    #endif
    } bits;
} 
RXDMA_PSR_MIN_DES_t, *PRXDMA_PSR_MIN_DES_t;


/******************************************************************************
   structure for free buffer ring base lo address reg in rxdma address map
   located at address 0x203C
 *****************************************************************************/
typedef struct _RXDMA_FBR_BASE_LO_t
{
    UINT32 addr_lo;                                 //bits 0-31
} 
RXDMA_FBR_BASE_LO_t, *PRXDMA_FBR_BASE_LO_t;


/******************************************************************************
   structure for free buffer ring base hi address reg in rxdma address map
   located at address 0x2040
 *****************************************************************************/
typedef struct _RXDMA_FBR_BASE_HI_t
{
    UINT32 addr_hi;                                 //bits 0-31
} 
RXDMA_FBR_BASE_HI_t, *PRXDMA_FBR_BASE_HI_t;


/******************************************************************************
   structure for free buffer ring number of descriptors reg in rxdma address 
   map.  Located at address 0x2044
 *****************************************************************************/
typedef union _RXDMA_FBR_NUM_DES_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:22;                           //bits 10-31
        UINT32 fbr_ndesc:10;                        //bits 0-9
    #else
        UINT32 fbr_ndesc:10;                        //bits 0-9
        UINT32 unused:22;                           //bits 10-31
    #endif
    } bits;
} 
RXDMA_FBR_NUM_DES_t, *PRXDMA_FBR_NUM_DES_t;


/******************************************************************************
   structure for free buffer ring 0 available offset reg in rxdma address map  
   located at address 0x2048
 *****************************************************************************/
typedef union _RXDMA_FBR_AVAIL_OFFSET_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 fbr_avail_wrap:1;                    //bit 10
        UINT32 fbr_avail:10;                        //bit 0-9
    #else
        UINT32 fbr_avail:10;                        //bit 0-9
        UINT32 fbr_avail_wrap:1;                    //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
RXDMA_FBR_AVAIL_OFFSET_t, *PRXDMA_FBR_AVAIL_OFFSET_t;


/******************************************************************************
   structure for free buffer ring 0 full offset reg in rxdma address map  
   located at address 0x204C
 *****************************************************************************/
typedef union _RXDMA_FBR_FULL_OFFSET_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:21;                           //bits 11-31
        UINT32 fbr_full_wrap:1;                     //bit 10
        UINT32 fbr_full:10;                         //bit 0-9
    #else
        UINT32 fbr_full:10;                         //bit 0-9
        UINT32 fbr_full_wrap:1;                     //bit 10
        UINT32 unused:21;                           //bits 11-31
    #endif
    } bits;
} 
RXDMA_FBR_FULL_OFFSET_t, *PRXDMA_FBR_FULL_OFFSET_t;


/******************************************************************************
   structure for free buffer cache 0 full offset reg in rxdma address map  
   located at address 0x2050
 *****************************************************************************/
typedef union _RXDMA_FBC_RD_INDEX_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:27;                           //bits 5-31
        UINT32 fbc_rdi:5;                          //bit 0-4
    #else
        UINT32 fbc_rdi:5;                          //bit 0-4
        UINT32 unused:27;                           //bits 5-31
    #endif
    } bits;
} 
RXDMA_FBC_RD_INDEX_t, *PRXDMA_FBC_RD_INDEX_t;


/******************************************************************************
   structure for free buffer ring 0 minimum descriptor reg in rxdma address map
   located at address 0x2054
 *****************************************************************************/
typedef union _RXDMA_FBR_MIN_DES_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:22;                           //bits 10-31
        UINT32 fbr_min:10;                          //bits 0-9
    #else
        UINT32 fbr_min:10;                          //bits 0-9
        UINT32 unused:22;                           //bits 10-31
    #endif
    } bits;
} 
RXDMA_FBR_MIN_DES_t, *PRXDMA_FBR_MIN_DES_t;


/******************************************************************************
   structure for free buffer ring 1 base address lo reg in rxdma address map
   located at address 0x2058 - 0x205C
   Defined earlier (RXDMA_FBR_BASE_LO_t and RXDMA_FBR_BASE_HI_t)
 *****************************************************************************/

/******************************************************************************
   structure for free buffer ring 1 number of descriptors reg in rxdma address 
   map.  Located at address 0x2060
   Defined earlier (RXDMA_FBR_NUM_DES_t)
 *****************************************************************************/

/******************************************************************************
   structure for free buffer ring 1 available offset reg in rxdma address map
   located at address 0x2064
   Defined Earlier (RXDMA_FBR_AVAIL_OFFSET_t)
 *****************************************************************************/

/******************************************************************************
   structure for free buffer ring 1 full offset reg in rxdma address map
   located at address 0x2068
   Defined Earlier (RXDMA_FBR_FULL_OFFSET_t)
 *****************************************************************************/

/******************************************************************************
   structure for free buffer cache 1 read index reg in rxdma address map
   located at address 0x206C
   Defined Earlier (RXDMA_FBC_RD_INDEX_t)
 *****************************************************************************/

/******************************************************************************
   structure for free buffer ring 1 minimum descriptor reg in rxdma address map
   located at address 0x2070
   Defined Earlier (RXDMA_FBR_MIN_DES_t)
 *****************************************************************************/


/******************************************************************************
   Rx DMA Module of JAGCore Address Mapping  
   Located at address 0x2000
 *****************************************************************************/
typedef struct _RXDMA_t
{                                                   //Location:
    RXDMA_CSR_t                 csr;                //  0x2000
    RXDMA_DMA_WB_BASE_LO_t      dma_wb_base_lo;     //  0x2004
    RXDMA_DMA_WB_BASE_HI_t      dma_wb_base_hi;     //  0x2008
    RXDMA_NUM_PKT_DONE_t        num_pkt_done;       //  0x200C
    RXDMA_MAX_PKT_TIME_t        max_pkt_time;       //  0x2010
    RXDMA_RXQ_RD_ADDR_t         rxq_rd_addr;        //  0x2014
    RXDMA_RXQ_RD_ADDR_EXT_t     rxq_rd_addr_ext;    //  0x2018
    RXDMA_RXQ_WR_ADDR_t         rxq_wr_addr;        //  0x201C
    RXDMA_PSR_BASE_LO_t         psr_base_lo;        //  0x2020
    RXDMA_PSR_BASE_HI_t         psr_base_hi;        //  0x2024
    RXDMA_PSR_NUM_DES_t         psr_num_des;        //  0x2028
    RXDMA_PSR_AVAIL_OFFSET_t    psr_avail_offset;   //  0x202C
    RXDMA_PSR_FULL_OFFSET_t     psr_full_offset;    //  0x2030
    RXDMA_PSR_ACCESS_INDEX_t    psr_access_index;   //  0x2034
    RXDMA_PSR_MIN_DES_t         psr_min_des;        //  0x2038
    RXDMA_FBR_BASE_LO_t         fbr0_base_lo;       //  0x203C
    RXDMA_FBR_BASE_HI_t         fbr0_base_hi;       //  0x2040
    RXDMA_FBR_NUM_DES_t         fbr0_num_des;       //  0x2044
    RXDMA_FBR_AVAIL_OFFSET_t    fbr0_avail_offset;  //  0x2048
    RXDMA_FBR_FULL_OFFSET_t     fbr0_full_offset;   //  0x204C
    RXDMA_FBC_RD_INDEX_t        fbr0_rd_index;      //  0x2050
    RXDMA_FBR_MIN_DES_t         fbr0_min_des;       //  0x2054
    RXDMA_FBR_BASE_LO_t         fbr1_base_lo;       //  0x2058
    RXDMA_FBR_BASE_HI_t         fbr1_base_hi;       //  0x205C
    RXDMA_FBR_NUM_DES_t         fbr1_num_des;       //  0x2060
    RXDMA_FBR_AVAIL_OFFSET_t    fbr1_avail_offset;  //  0x2064
    RXDMA_FBR_FULL_OFFSET_t     fbr1_full_offset;   //  0x2068
    RXDMA_FBC_RD_INDEX_t        fbr1_rd_index;      //  0x206C
    RXDMA_FBR_MIN_DES_t         fbr1_min_des;       //  0x2070
}
RXDMA_t, *PRXDMA_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF RXDMA REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF TXMAC REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for control reg in txmac address map
   located at address 0x3000
 *****************************************************************************/
typedef union _TXMAC_CTL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:24;                           //bits 8-31
        UINT32 cklseg_diable:1;                     //bit 7
        UINT32 ckbcnt_disable:1;                    //bit 6
        UINT32 cksegnum:1;                          //bit 5
        UINT32 async_disable:1;                     //bit 4
        UINT32 fc_disable:1;                        //bit 3
        UINT32 mcif_disable:1;                      //bit 2
        UINT32 mif_disable:1;                       //bit 1
        UINT32 txmac_en:1;                          //bit 0
    #else
        UINT32 txmac_en:1;                          //bit 0
        UINT32 mif_disable:1;                       //bit 1 mac interface
        UINT32 mcif_disable:1;                      //bit 2 memory controller interface
        UINT32 fc_disable:1;                        //bit 3
        UINT32 async_disable:1;                     //bit 4
        UINT32 cksegnum:1;                          //bit 5
        UINT32 ckbcnt_disable:1;                    //bit 6
        UINT32 cklseg_diable:1;                     //bit 7
        UINT32 unused:24;                           //bits 8-31
    #endif
    } bits;
} 
TXMAC_CTL_t, *PTXMAC_CTL_t;


/******************************************************************************
   structure for shadow pointer reg in txmac address map
   located at address 0x3004
 *****************************************************************************/
typedef union _TXMAC_SHADOW_PTR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:5;                         //bits 27-31
        UINT32 txq_rd_ptr:11;                       //bits 16-26
        UINT32 reserved:5;                          //bits 11-15
        UINT32 txq_wr_ptr:11;                       //bits 0-10
    #else
        UINT32 txq_wr_ptr:11;                       //bits 0-10
        UINT32 reserved:5;                          //bits 11-15
        UINT32 txq_rd_ptr:11;                       //bits 16-26
        UINT32 reserved2:5;                         //bits 27-31
    #endif
    } bits;
} 
TXMAC_SHADOW_PTR_t, *PTXMAC_SHADOW_PTR_t;


/******************************************************************************
   structure for error count reg in txmac address map
   located at address 0x3008
 *****************************************************************************/
typedef union _TXMAC_ERR_CNT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:20;                           //bits 12-31
        UINT32 reserved:4;                          //bits 8-11
        UINT32 txq_underrun:4;                      //bits 4-7
        UINT32 fifo_underrun:4;                     //bits 0-3
    #else
        UINT32 fifo_underrun:4;                     //bits 0-3
        UINT32 txq_underrun:4;                      //bits 4-7
        UINT32 reserved:4;                          //bits 8-11
        UINT32 unused:20;                           //bits 12-31
    #endif
    } bits;
} TXMAC_ERR_CNT_t, *PTXMAC_ERR_CNT_t;


/******************************************************************************
   structure for max fill reg in txmac address map
   located at address 0x300C
 *****************************************************************************/
typedef union _TXMAC_MAX_FILL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:20;                           //bits 12-31
        UINT32 max_fill:12;                         //bits 0-11
    #else
        UINT32 max_fill:12;                         //bits 0-11
        UINT32 unused:20;                           //bits 12-31
    #endif
    } bits;
} 
TXMAC_MAX_FILL_t, *PTXMAC_MAX_FILL_t;


/******************************************************************************
   structure for cf parameter reg in txmac address map
   located at address 0x3010
 *****************************************************************************/
typedef union _TXMAC_CF_PARAM_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 cfep:16;                             //bits 16-31
        UINT32 cfpt:16;                             //bits 0-15
    #else
        UINT32 cfpt:16;                             //bits 0-15
        UINT32 cfep:16;                             //bits 16-31
    #endif
    } bits;
} 
TXMAC_CF_PARAM_t, *PTXMAC_CF_PARAM_t;


/******************************************************************************
   structure for tx test reg in txmac address map
   located at address 0x3014
 *****************************************************************************/
typedef union _TXMAC_TXTEST_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:15;                          //bits 17-31
        UINT32 reserved1:1;                         //bit 16
        UINT32 txtest_en:1;                         //bit 15
        UINT32 unused1:4;                           //bits 11-14
        UINT32 txqtest_ptr:11;                      //bits 0-11
    #else
        UINT32 txqtest_ptr:11;                      //bits 0-10
        UINT32 unused1:4;                           //bits 11-14
        UINT32 txtest_en:1;                         //bit 15
        UINT32 reserved1:1;                         //bit 16
        UINT32 unused2:15;                          //bits 17-31
    #endif
    } bits;
} 
TXMAC_TXTEST_t, *PTXMAC_TXTEST_t;


/******************************************************************************
   structure for error reg in txmac address map
   located at address 0x3018
 *****************************************************************************/
typedef union _TXMAC_ERR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:23;                          //bits 9-31
        UINT32 fifo_underrun:1;                     //bit 8
        UINT32 unused1:2;                           //bits 6-7
        UINT32 ctrl2_err:1;                         //bit 5
        UINT32 txq_underrun:1;                      //bit 4
        UINT32 bcnt_err:1;                          //bit 3
        UINT32 lseg_err:1;                          //bit 2
        UINT32 segnum_err:1;                        //bit 1
        UINT32 seg0_err:1;                          //bit 0
    #else
        UINT32 seg0_err:1;                          //bit 0
        UINT32 segnum_err:1;                        //bit 1
        UINT32 lseg_err:1;                          //bit 2
        UINT32 bcnt_err:1;                          //bit 3
        UINT32 txq_underrun:1;                      //bit 4
        UINT32 ctrl2_err:1;                         //bit 5
        UINT32 unused1:2;                           //bits 6-7
        UINT32 fifo_underrun:1;                     //bit 8
        UINT32 unused2:23;                          //bits 9-31
    #endif
    } bits;
} 
TXMAC_ERR_t, *PTXMAC_ERR_t;


/******************************************************************************
   structure for error interrupt reg in txmac address map
   located at address 0x301C
 *****************************************************************************/
typedef union _TXMAC_ERR_INT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:23;                          //bits 9-31
        UINT32 fifo_underrun:1;                     //bit 8
        UINT32 unused1:2;                           //bits 6-7
        UINT32 ctrl2_err:1;                         //bit 5
        UINT32 txq_underrun:1;                      //bit 4
        UINT32 bcnt_err:1;                          //bit 3
        UINT32 lseg_err:1;                          //bit 2
        UINT32 segnum_err:1;                        //bit 1
        UINT32 seg0_err:1;                          //bit 0
    #else
        UINT32 seg0_err:1;                          //bit 0
        UINT32 segnum_err:1;                        //bit 1
        UINT32 lseg_err:1;                          //bit 2
        UINT32 bcnt_err:1;                          //bit 3
        UINT32 txq_underrun:1;                      //bit 4
        UINT32 ctrl2_err:1;                         //bit 5
        UINT32 unused1:2;                           //bits 6-7
        UINT32 fifo_underrun:1;                     //bit 8
        UINT32 unused2:23;                          //bits 9-31
    #endif
    } bits;
} 
TXMAC_ERR_INT_t, *PTXMAC_ERR_INT_t;


/******************************************************************************
   structure for error interrupt reg in txmac address map
   located at address 0x3020
 *****************************************************************************/
typedef union _TXMAC_CP_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:30;                           //bits 2-31
        UINT32 bp_req:1;                            //bit 1
        UINT32 bp_xonxoff:1;                        //bit 0
    #else
        UINT32 bp_xonxoff:1;                        //bit 0
        UINT32 bp_req:1;                            //bit 1
        UINT32 unused:30;                           //bits 2-31
    #endif
    } bits;
} 
TXMAC_BP_CTRL_t, *PTXMAC_BP_CTRL_t;


/******************************************************************************
   Tx MAC Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _TXMAC_t
{                                                   //Location:
    TXMAC_CTL_t             ctl;                    //  0x3000
    TXMAC_SHADOW_PTR_t      shadow_ptr;             //  0x3004
    TXMAC_ERR_CNT_t         err_cnt;                //  0x3008
    TXMAC_MAX_FILL_t        max_fill;               //  0x300C
    TXMAC_CF_PARAM_t        cf_param;               //  0x3010
    TXMAC_TXTEST_t          tx_test;                //  0x3014
    TXMAC_ERR_t             err;                    //  0x3018
    TXMAC_ERR_INT_t         err_int;                //  0x301C
    TXMAC_BP_CTRL_t         bp_ctrl;                //  0x3020
}
TXMAC_t, *PTXMAC_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF TXMAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                 START OF RXMAC REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for rxmac control reg in rxmac address map
   located at address 0x4000
 *****************************************************************************/
typedef union _RXMAC_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:25;                         //bits 7-31
        UINT32 rxmac_int_disable:1;                 //bit 6
        UINT32 async_disable:1;                     //bit 5
        UINT32 mif_disable:1;                       //bit 4
        UINT32 wol_disable:1;                       //bit 3
        UINT32 pkt_filter_disable:1;                //bit 2
        UINT32 mcif_disable:1;                      //bit 1
        UINT32 rxmac_en:1;                          //bit 0
    #else
        UINT32 rxmac_en:1;                          //bit 0
        UINT32 mcif_disable:1;                      //bit 1
        UINT32 pkt_filter_disable:1;                //bit 2
        UINT32 wol_disable:1;                       //bit 3
        UINT32 mif_disable:1;                       //bit 4
        UINT32 async_disable:1;                     //bit 5
        UINT32 rxmac_int_disable:1;                 //bit 6
        UINT32 reserved:25;                         //bits 7-31
    #endif
    } bits;
} 
RXMAC_CTRL_t, *PRXMAC_CTRL_t;


/******************************************************************************
   structure for Wake On Lan Control and CRC 0 reg in rxmac address map
   located at address 0x4004
 *****************************************************************************/
typedef union _RXMAC_WOL_CTL_CRC0_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 crc0:16;                             //bits 16-31
        UINT32 reserve:4;                           //bits 12-15
        UINT32 ignore_pp:1;                         //bit 11
        UINT32 ignore_mp:1;                         //bit 10
        UINT32 clr_intr:1;                          //bit 9
        UINT32 ignore_link_chg:1;                   //bit 8
        UINT32 ignore_uni:1;                        //bit 7
        UINT32 ignore_multi:1;                      //bit 6
        UINT32 ignore_broad:1;                      //bit 5
        UINT32 valid_crc4:1;                        //bit 4
        UINT32 valid_crc3:1;                        //bit 3
        UINT32 valid_crc2:1;                        //bit 2
        UINT32 valid_crc1:1;                        //bit 1
        UINT32 valid_crc0:1;                        //bit 0
    #else
        UINT32 valid_crc0:1;                        //bit 0
        UINT32 valid_crc1:1;                        //bit 1
        UINT32 valid_crc2:1;                        //bit 2
        UINT32 valid_crc3:1;                        //bit 3
        UINT32 valid_crc4:1;                        //bit 4
        UINT32 ignore_broad:1;                      //bit 5
        UINT32 ignore_multi:1;                      //bit 6
        UINT32 ignore_uni:1;                        //bit 7
        UINT32 ignore_link_chg:1;                   //bit 8
        UINT32 clr_intr:1;                          //bit 9
        UINT32 ignore_mp:1;                         //bit 10
        UINT32 ignore_pp:1;                         //bit 11
        UINT32 reserve:4;                           //bits 12-15
        UINT32 crc0:16;                             //bits 16-31
    #endif
    } bits;
} 
RXMAC_WOL_CTL_CRC0_t, *PRXMAC_WOL_CTL_CRC0_t;


/******************************************************************************
   structure for CRC 1 and CRC 2 reg in rxmac address map
   located at address 0x4008
 *****************************************************************************/
typedef union _RXMAC_WOL_CRC12_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 crc2:16;                             //bits 16-31
        UINT32 crc1:16;                             //bits 0-15
    #else
        UINT32 crc1:16;                             //bits 0-15
        UINT32 crc2:16;                             //bits 16-31
    #endif
    } bits;
} 
RXMAC_WOL_CRC12_t, *PRXMAC_WOL_CRC12_t;


/******************************************************************************
   structure for CRC 3 and CRC 4 reg in rxmac address map
   located at address 0x400C
 *****************************************************************************/
typedef union _RXMAC_WOL_CRC34_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 crc4:16;                             //bits 16-31
        UINT32 crc3:16;                             //bits 0-15
    #else
        UINT32 crc3:16;                             //bits 0-15
        UINT32 crc4:16;                             //bits 16-31
    #endif
    } bits;
} 
RXMAC_WOL_CRC34_t, *PRXMAC_WOL_CRC34_t;


/******************************************************************************
   structure for Wake On Lan Source Address Lo reg in rxmac address map
   located at address 0x4010
 *****************************************************************************/
typedef union _RXMAC_WOL_SA_LO_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 sa3:8;                               //bits 24-31
        UINT32 sa4:8;                               //bits 16-23
        UINT32 sa5:8;                               //bits 8-15
        UINT32 sa6:8;                               //bits 0-7
    #else
        UINT32 sa6:8;                               //bits 0-7
        UINT32 sa5:8;                               //bits 8-15
        UINT32 sa4:8;                               //bits 16-23
        UINT32 sa3:8;                               //bits 24-31
    #endif
    } bits;
} 
RXMAC_WOL_SA_LO_t, *PRXMAC_WOL_SA_LO_t;


/******************************************************************************
   structure for Wake On Lan Source Address Hi reg in rxmac address map
   located at address 0x4014
 *****************************************************************************/
typedef union _RXMAC_WOL_SA_HI_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:16;                         //bits 16-31
        UINT32 sa1:8;                               //bits 8-15
        UINT32 sa2:8;                               //bits 0-7
    #else
        UINT32 sa2:8;                               //bits 0-7
        UINT32 sa1:8;                               //bits 8-15
        UINT32 reserved:16;                         //bits 16-31
    #endif
    } bits;
} 
RXMAC_WOL_SA_HI_t, *PRXMAC_WOL_SA_HI_t;


/******************************************************************************
   structure for Wake On Lan mask reg in rxmac address map
   located at address 0x4018 - 0x4064
 *****************************************************************************/
typedef struct _RXMAC_WOL_MASK_t
{
    UINT32 mask;                                    //bits 0-31
} 
RXMAC_WOL_MASK_t, *PRXMAC_WOL_MASK_t;


/******************************************************************************
   structure for Unicast Paket Filter Address 1 reg in rxmac address map
   located at address 0x4068
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR1_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 addr1_3:8;                           //bits 24-31
        UINT32 addr1_4:8;                           //bits 16-23
        UINT32 addr1_5:8;                           //bits 8-15
        UINT32 addr1_6:8;                           //bits 0-7
    #else
        UINT32 addr1_6:8;                           //bits 0-7
        UINT32 addr1_5:8;                           //bits 8-15
        UINT32 addr1_4:8;                           //bits 16-23
        UINT32 addr1_3:8;                           //bits 24-31
    #endif
    } bits;
} 
RXMAC_UNI_PF_ADDR1_t, *PRXMAC_UNI_PF_ADDR1_t;


/******************************************************************************
   structure for Unicast Paket Filter Address 2 reg in rxmac address map
   located at address 0x406C
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR2_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 addr2_3:8;                           //bits 24-31
        UINT32 addr2_4:8;                           //bits 16-23
        UINT32 addr2_5:8;                           //bits 8-15
        UINT32 addr2_6:8;                           //bits 0-7
    #else
        UINT32 addr2_6:8;                           //bits 0-7
        UINT32 addr2_5:8;                           //bits 8-15
        UINT32 addr2_4:8;                           //bits 16-23
        UINT32 addr2_3:8;                           //bits 24-31
    #endif
    } bits;
} 
RXMAC_UNI_PF_ADDR2_t, *PRXMAC_UNI_PF_ADDR2_t;


/******************************************************************************
   structure for Unicast Paket Filter Address 1 & 2 reg in rxmac address map
   located at address 0x4070
 *****************************************************************************/
typedef union _RXMAC_UNI_PF_ADDR3_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 addr2_1:8;                           //bits 24-31
        UINT32 addr2_2:8;                           //bits 16-23
        UINT32 addr1_1:8;                           //bits 8-15
        UINT32 addr1_2:8;                           //bits 0-7
    #else
        UINT32 addr1_2:8;                           //bits 0-7
        UINT32 addr1_1:8;                           //bits 8-15
        UINT32 addr2_2:8;                           //bits 16-23
        UINT32 addr2_1:8;                           //bits 24-31
    #endif
    } bits;
} 
RXMAC_UNI_PF_ADDR3_t, *PRXMAC_UNI_PF_ADDR3_t;


/******************************************************************************
   structure for Multicast Hash reg in rxmac address map
   located at address 0x4074 - 0x4080
 *****************************************************************************/
typedef struct _RXMAC_MULTI_HASH_t
{
    UINT32 hash;                                    //bits 0-31
} 
RXMAC_MULTI_HASH_t, *PRXMAC_MULTI_HASH_t;


/******************************************************************************
   structure for Packet Filter Control reg in rxmac address map
   located at address 0x4084
 *****************************************************************************/
typedef union _RXMAC_PF_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused2:9;                           //bits 23-31
        UINT32 min_pkt_size:7;                      //bits 16-22
        UINT32 unused1:12;                          //bits 4-15
        UINT32 filter_frag_en:1;                    //bit 3
        UINT32 filter_uni_en:1;                     //bit 2
        UINT32 filter_multi_en:1;                   //bit 1
        UINT32 filter_broad_en:1;                   //bit 0
    #else
        UINT32 filter_broad_en:1;                   //bit 0
        UINT32 filter_multi_en:1;                   //bit 1
        UINT32 filter_uni_en:1;                     //bit 2
        UINT32 filter_frag_en:1;                    //bit 3
        UINT32 unused1:12;                          //bits 4-15
        UINT32 min_pkt_size:7;                      //bits 16-22
        UINT32 unused2:9;                           //bits 23-31
    #endif
    } bits;
} 
RXMAC_PF_CTRL_t, *PRXMAC_PF_CTRL_t;


/******************************************************************************
   structure for Memory Controller Interface Control Max Segment reg in rxmac 
   address map.  Located at address 0x4088
 *****************************************************************************/
typedef union _RXMAC_MCIF_CTRL_MAX_SEG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:22;                         //bits 10-31
        UINT32 max_size:8;                          //bits 2-9
        UINT32 fc_en:1;                             //bit 1
        UINT32 seg_en:1;                            //bit 0
    #else
        UINT32 seg_en:1;                            //bit 0
        UINT32 fc_en:1;                             //bit 1
        UINT32 max_size:8;                          //bits 2-9
        UINT32 reserved:22;                         //bits 10-31
    #endif
    } bits;
} 
RXMAC_MCIF_CTRL_MAX_SEG_t, *PRXMAC_MCIF_CTRL_MAX_SEG_t;


/******************************************************************************
   structure for Memory Controller Interface Water Mark reg in rxmac address 
   map.  Located at address 0x408C
 *****************************************************************************/
typedef union _RXMAC_MCIF_WATER_MARK_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:6;                         //bits 26-31
        UINT32 mark_hi:10;                          //bits 16-25
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 mark_lo:10;                          //bits 0-9
    #else
        UINT32 mark_lo:10;                          //bits 0-9
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 mark_hi:10;                          //bits 16-25
        UINT32 reserved2:6;                         //bits 26-31
    #endif
    } bits;
} 
RXMAC_MCIF_WATER_MARK_t, *PRXMAC_MCIF_WATER_MARK_t;


/******************************************************************************
   structure for Rx Queue Dialog reg in rxmac address map.  
   located at address 0x4090
 *****************************************************************************/
typedef union _RXMAC_RXQ_DIAG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:6;                         //bits 26-31
        UINT32 rd_ptr:10;                           //bits 16-25
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 wr_ptr:10;                           //bits 0-9
    #else
        UINT32 wr_ptr:10;                           //bits 0-9
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 rd_ptr:10;                           //bits 16-25
        UINT32 reserved2:6;                         //bits 26-31
    #endif
    } bits;
} 
RXMAC_RXQ_DIAG_t, *PRXMAC_RXQ_DIAG_t;


/******************************************************************************
   structure for space availiable reg in rxmac address map.  
   located at address 0x4094
 *****************************************************************************/
typedef union _RXMAC_SPACE_AVAIL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:15;                        //bits 17-31
        UINT32 space_avail_en:1;                    //bit 16
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 space_avail:10;                      //bits 0-9
    #else
        UINT32 space_avail:10;                      //bits 0-9
        UINT32 reserved1:6;                         //bits 10-15
        UINT32 space_avail_en:1;                    //bit 16
        UINT32 reserved2:15;                        //bits 17-31
    #endif
    } bits;
} 
RXMAC_SPACE_AVAIL_t, *PRXMAC_SPACE_AVAIL_t;


/******************************************************************************
   structure for management interface reg in rxmac address map.  
   located at address 0x4098
 *****************************************************************************/
typedef union _RXMAC_MIF_CTL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserve:14;                          //bits 18-31
        UINT32 drop_pkt_en:1;                       //bit 17
        UINT32 drop_pkt_mask:17;                    //bits 0-16
    #else
        UINT32 drop_pkt_mask:17;                    //bits 0-16
        UINT32 drop_pkt_en:1;                       //bit 17
        UINT32 reserve:14;                          //bits 18-31
    #endif
    } bits;
} 
RXMAC_MIF_CTL_t, *PRXMAC_MIF_CTL_t;


/******************************************************************************
   structure for Error reg in rxmac address map.  
   located at address 0x409C
 *****************************************************************************/
typedef union _RXMAC_ERROR_REG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserve:28;                          //bits 4-31
        UINT32 mif:1;                               //bit 3
        UINT32 async:1;                             //bit 2
        UINT32 pkt_filter:1;                        //bit 1
        UINT32 mcif:1;                              //bit 0
    #else
        UINT32 mcif:1;                              //bit 0
        UINT32 pkt_filter:1;                        //bit 1
        UINT32 async:1;                             //bit 2
        UINT32 mif:1;                               //bit 3
        UINT32 reserve:28;                          //bits 4-31 
    #endif
    } bits;
} 
RXMAC_ERROR_REG_t, *PRXMAC_ERROR_REG_t;


/******************************************************************************
   Rx MAC Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _RXMAC_t
{                                                   //Location:
    RXMAC_CTRL_t                ctrl;               //  0x4000
    RXMAC_WOL_CTL_CRC0_t        crc0;               //  0x4004
    RXMAC_WOL_CRC12_t           crc12;              //  0x4008
    RXMAC_WOL_CRC34_t           crc34;              //  0x400C
    RXMAC_WOL_SA_LO_t           sa_lo;              //  0x4010
    RXMAC_WOL_SA_HI_t           sa_hi;              //  0x4014
    RXMAC_WOL_MASK_t            mask0_word0;        //  0x4018
    RXMAC_WOL_MASK_t            mask0_word1;        //  0x401C
    RXMAC_WOL_MASK_t            mask0_word2;        //  0x4020
    RXMAC_WOL_MASK_t            mask0_word3;        //  0x4024
    RXMAC_WOL_MASK_t            mask1_word0;        //  0x4028
    RXMAC_WOL_MASK_t            mask1_word1;        //  0x402C
    RXMAC_WOL_MASK_t            mask1_word2;        //  0x4030
    RXMAC_WOL_MASK_t            mask1_word3;        //  0x4034
    RXMAC_WOL_MASK_t            mask2_word0;        //  0x4038
    RXMAC_WOL_MASK_t            mask2_word1;        //  0x403C
    RXMAC_WOL_MASK_t            mask2_word2;        //  0x4040
    RXMAC_WOL_MASK_t            mask2_word3;        //  0x4044
    RXMAC_WOL_MASK_t            mask3_word0;        //  0x4048
    RXMAC_WOL_MASK_t            mask3_word1;        //  0x404C
    RXMAC_WOL_MASK_t            mask3_word2;        //  0x4050
    RXMAC_WOL_MASK_t            mask3_word3;        //  0x4054
    RXMAC_WOL_MASK_t            mask4_word0;        //  0x4058
    RXMAC_WOL_MASK_t            mask4_word1;        //  0x405C
    RXMAC_WOL_MASK_t            mask4_word2;        //  0x4060
    RXMAC_WOL_MASK_t            mask4_word3;        //  0x4064
    RXMAC_UNI_PF_ADDR1_t        uni_pf_addr1;       //  0x4068
    RXMAC_UNI_PF_ADDR2_t        uni_pf_addr2;       //  0x406C
    RXMAC_UNI_PF_ADDR3_t        uni_pf_addr3;       //  0x4070
    RXMAC_MULTI_HASH_t          multi_hash1;        //  0x4074
    RXMAC_MULTI_HASH_t          multi_hash2;        //  0x4078
    RXMAC_MULTI_HASH_t          multi_hash3;        //  0x407C
    RXMAC_MULTI_HASH_t          multi_hash4;        //  0x4080
    RXMAC_PF_CTRL_t             pf_ctrl;            //  0x4084
    RXMAC_MCIF_CTRL_MAX_SEG_t   mcif_ctrl_max_seg;  //  0x4088
    RXMAC_MCIF_WATER_MARK_t     mcif_water_mark;    //  0x408C
    RXMAC_RXQ_DIAG_t            rxq_diag;           //  0x4090
    RXMAC_SPACE_AVAIL_t         space_avail;        //  0x4094

    RXMAC_MIF_CTL_t             mif_ctrl;           //  0x4098
    RXMAC_ERROR_REG_t           err_reg;            //  0x409C
}
RXMAC_t, *PRXMAC_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                  END OF TXMAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                  START OF MAC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for configuration #1 reg in mac address map.  
   located at address 0x5000
 *****************************************************************************/
typedef union _MAC_CFG1_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 soft_reset:1;                        //bit 31
        UINT32 sim_reset:1;                         //bit 30
        UINT32 reserved3:10;                        //bits 20-29
        UINT32 reset_rx_mc:1;                       //bit 19
        UINT32 reset_tx_mc:1;                       //bit 18
        UINT32 reset_rx_fun:1;                      //bit 17
        UINT32 reset_tx_fun:1;                      //bit 16
        UINT32 reserved2:7;                         //bits 9-15
        UINT32 loop_back:1;                         //bit 8
        UINT32 reserved1:2;                         //bits 6-7
        UINT32 rx_flow:1;                           //bit 5
        UINT32 tx_flow:1;                           //bit 4
        UINT32 syncd_rx_en:1;                       //bit 3
        UINT32 rx_enable:1;                         //bit 2
        UINT32 syncd_tx_en:1;                       //bit 1
        UINT32 tx_enable:1;                         //bit 0
    #else
        UINT32 tx_enable:1;                         //bit 0
        UINT32 syncd_tx_en:1;                       //bit 1
        UINT32 rx_enable:1;                         //bit 2
        UINT32 syncd_rx_en:1;                       //bit 3
        UINT32 tx_flow:1;                           //bit 4
        UINT32 rx_flow:1;                           //bit 5
        UINT32 reserved1:2;                         //bits 6-7
        UINT32 loop_back:1;                         //bit 8
        UINT32 reserved2:7;                         //bits 9-15
        UINT32 reset_tx_fun:1;                      //bit 16
        UINT32 reset_rx_fun:1;                      //bit 17
        UINT32 reset_tx_mc:1;                       //bit 18
        UINT32 reset_rx_mc:1;                       //bit 19
        UINT32 reserved3:10;                        //bits 20-29
        UINT32 sim_reset:1;                         //bit 30
        UINT32 soft_reset:1;                        //bit 31
    #endif
    } bits;
} 
MAC_CFG1_t, *PMAC_CFG1_t;


/******************************************************************************
   structure for configuration #2 reg in mac address map.  
   located at address 0x5004
 *****************************************************************************/
typedef union _MAC_CFG2_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved3:16;                        //bits 16-31
        UINT32 preamble_len:4;                      //bits 12-15
        UINT32 reserved2:2;                         //bits 10-11
        UINT32 if_mode:2;                           //bits 8-9
        UINT32 reserved1:2;                         //bits 6-7
        UINT32 huge_frame:1;                        //bit 5
        UINT32 len_check:1;                         //bit 4
        UINT32 undefined:1;                         //bit 3
        UINT32 pad_crc:1;                           //bit 2
        UINT32 crc_enable:1;                        //bit 1
        UINT32 full_duplex:1;                       //bit 0
    #else
        UINT32 full_duplex:1;                       //bit 0
        UINT32 crc_enable:1;                        //bit 1
        UINT32 pad_crc:1;                           //bit 2
        UINT32 undefined:1;                         //bit 3
        UINT32 len_check:1;                         //bit 4
        UINT32 huge_frame:1;                        //bit 5
        UINT32 reserved1:2;                         //bits 6-7
        UINT32 if_mode:2;                           //bits 8-9
        UINT32 reserved2:2;                         //bits 10-11
        UINT32 preamble_len:4;                      //bits 12-15
        UINT32 reserved3:16;                        //bits 16-31
    #endif
    } bits;
} 
MAC_CFG2_t, *PMAC_CFG2_t;


/******************************************************************************
   structure for Interpacket gap reg in mac address map.  
   located at address 0x5008
 *****************************************************************************/
typedef union _MAC_IPG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:1;                          //bit 31
        UINT32 non_B2B_ipg_1:7;                     //bits 24-30
        UINT32 undefined2:1;                        //bit 23
        UINT32 non_B2B_ipg_2:7;                     //bits 16-22
        UINT32 min_ifg_enforce:8;                   //bits 8-15
        UINT32 undefined1:1;                        //bit 7
        UINT32 B2B_ipg:7;                           //bits 0-6
    #else
        UINT32 B2B_ipg:7;                           //bits 0-6
        UINT32 undefined1:1;                        //bit 7
        UINT32 min_ifg_enforce:8;                   //bits 8-15
        UINT32 non_B2B_ipg_2:7;                     //bits 16-22
        UINT32 undefined2:1;                        //bit 23
        UINT32 non_B2B_ipg_1:7;                     //bits 24-30
        UINT32 reserved:1;                          //bit 31
    #endif
    } bits;
} 
MAC_IPG_t, *PMAC_IPG_t;


/******************************************************************************
   structure for half duplex reg in mac address map.  
   located at address 0x500C
 *****************************************************************************/
typedef union _MAC_HFDP_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:8;                         //bits 24-31
        UINT32 alt_beb_trunc:4;                     //bits 23-20
        UINT32 alt_beb_enable:1;                    //bit 19
        UINT32 bp_no_backoff:1;                     //bit 18
        UINT32 no_backoff:1;                        //bit 17
        UINT32 excess_defer:1;                      //bit 16
        UINT32 rexmit_max:4;                        //bits 12-15
        UINT32 reserved1:2;                         //bits 10-11
        UINT32 coll_window:10;                      //bits 0-9
    #else
        UINT32 coll_window:10;                      //bits 0-9
        UINT32 reserved1:2;                         //bits 10-11
        UINT32 rexmit_max:4;                        //bits 12-15
        UINT32 excess_defer:1;                      //bit 16
        UINT32 no_backoff:1;                        //bit 17
        UINT32 bp_no_backoff:1;                     //bit 18
        UINT32 alt_beb_enable:1;                    //bit 19
        UINT32 alt_beb_trunc:4;                     //bits 23-20
        UINT32 reserved2:8;                         //bits 24-31
    #endif
    } bits;
} 
MAC_HFDP_t, *PMAC_HFDP_t;


/******************************************************************************
   structure for Maximum Frame Length reg in mac address map.  
   located at address 0x5010
 *****************************************************************************/
typedef union _MAC_MAX_FM_LEN_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:16;                         //bits 16-31
        UINT32 max_len:16;                          //bits 0-15
    #else
        UINT32 max_len:16;                          //bits 0-15
        UINT32 reserved:16;                         //bits 16-31
    #endif
    } bits;
} 
MAC_MAX_FM_LEN_t, *PMAC_MAX_FM_LEN_t;


/******************************************************************************
   structure for Reserve 1 reg in mac address map.  
   located at address 0x5014 - 0x5018
 *****************************************************************************/
typedef struct _MAC_RSV_t
{
    UINT32 value;                                   //bits 0-31
} 
MAC_RSV_t, *PMAC_RSV_t;


/******************************************************************************
   structure for Test reg in mac address map.  
   located at address 0x501C
 *****************************************************************************/
typedef union _MAC_TEST_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:29;                           //bits 3-31
        UINT32 mac_test:3;                          //bits 0-2
    #else
        UINT32 mac_test:3;                          //bits 0-2
        UINT32 unused:29;                           //bits 3-31
    #endif
    } bits;
} 
MAC_TEST_t, *PMAC_TEST_t;


/******************************************************************************
   structure for MII Management Configuration reg in mac address map.  
   located at address 0x5020
 *****************************************************************************/
typedef union _MII_MGMT_CFG_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reset_mii_mgmt:1;                    //bit 31
        UINT32 reserved:25;                         //bits 6-30
        UINT32 scan_auto_incremt:1;                 //bit 5
        UINT32 preamble_suppress:1;                 //bit 4
        UINT32 undefined:1;                         //bit 3
        UINT32 mgmt_clk_reset:3;                    //bits 0-2
    #else
        UINT32 mgmt_clk_reset:3;                    //bits 0-2
        UINT32 undefined:1;                         //bit 3
        UINT32 preamble_suppress:1;                 //bit 4
        UINT32 scan_auto_incremt:1;                 //bit 5
        UINT32 reserved:25;                         //bits 6-30
        UINT32 reset_mii_mgmt:1;                    //bit 31
    #endif
    } bits;
} 
MII_MGMT_CFG_t, *PMII_MGMT_CFG_t;


/******************************************************************************
   structure for MII Management Command reg in mac address map.  
   located at address 0x5024
 *****************************************************************************/
typedef union _MII_MGMT_CMD_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:30;                         //bits 2-31
        UINT32 scan_cycle:1;                        //bit 1
        UINT32 read_cycle:1;                        //bit 0
    #else
        UINT32 read_cycle:1;                        //bit 0
        UINT32 scan_cycle:1;                        //bit 1
        UINT32 reserved:30;                         //bits 2-31    
    #endif
    } bits;
} 
MII_MGMT_CMD_t, *PMII_MGMT_CMD_t;


/******************************************************************************
   structure for MII Management Address reg in mac address map.  
   located at address 0x5028
 *****************************************************************************/
typedef union _MII_MGMT_ADDR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved2:19;                        //bit 13-31
        UINT32 phy_addr:5;                          //bits 8-12
        UINT32 reserved1:3;                         //bits 5-7
        UINT32 reg_addr:5;                          //bits 0-4
    #else
        UINT32 reg_addr:5;                          //bits 0-4
        UINT32 reserved1:3;                         //bits 5-7
        UINT32 phy_addr:5;                          //bits 8-12
        UINT32 reserved2:19;                        //bit 13-31
    #endif
    } bits;
} 
MII_MGMT_ADDR_t, *PMII_MGMT_ADDR_t;


/******************************************************************************
   structure for MII Management Control reg in mac address map.  
   located at address 0x502C
 *****************************************************************************/
typedef union _MII_MGMT_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:16;                         //bits 16-31
        UINT32 phy_ctrl:16;                         //bits 0-15
    #else
        UINT32 phy_ctrl:16;                         //bits 0-15
        UINT32 reserved:16;                         //bits 16-31
    #endif
    } bits;
} 
MII_MGMT_CTRL_t, *PMII_MGMT_CTRL_t;


/******************************************************************************
   structure for MII Management Status reg in mac address map.  
   located at address 0x5030
 *****************************************************************************/
typedef union _MII_MGMT_STAT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:16;                         //bits 16-31
        UINT32 phy_stat:16;                         //bits 0-15
    #else
        UINT32 phy_stat:16;                         //bits 0-15
        UINT32 reserved:16;                         //bits 16-31
    #endif
    } bits;
} 
MII_MGMT_STAT_t, *PMII_MGMT_STAT_t;


/******************************************************************************
   structure for MII Management Indicators reg in mac address map.  
   located at address 0x5034
 *****************************************************************************/
typedef union _MII_MGMT_INDICATOR_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:29;                         //bits 3-31
        UINT32 not_valid:1;                         //bit 2
        UINT32 scanning:1;                          //bit 1
        UINT32 busy:1;                              //bit 0
    #else
        UINT32 busy:1;                              //bit 0
        UINT32 scanning:1;                          //bit 1
        UINT32 not_valid:1;                         //bit 2
        UINT32 reserved:29;                         //bits 3-31
    #endif
    } bits;
} 
MII_MGMT_INDICATOR_t, *PMII_MGMT_INDICATOR_t;


/******************************************************************************
   structure for Interface Control reg in mac address map.  
   located at address 0x5038
 *****************************************************************************/
typedef union _MAC_IF_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reset_if_module:1;                   //bit 31
        UINT32 reserved4:3;                         //bit 28-30
        UINT32 tbi_mode:1;                          //bit 27
        UINT32 ghd_mode:1;                          //bit 26
        UINT32 lhd_mode:1;                          //bit 25
        UINT32 phy_mode:1;                          //bit 24
        UINT32 reset_per_mii:1;                     //bit 23
        UINT32 reserved3:6;                         //bits 17-22
        UINT32 speed:1;                             //bit 16
        UINT32 reset_pe100x:1;                      //bit 15
        UINT32 reserved2:4;                         //bits 11-14
        UINT32 force_quiet:1;                       //bit 10
        UINT32 no_cipher:1;                         //bit 9
        UINT32 disable_link_fail:1;                 //bit 8
        UINT32 reset_gpsi:1;                        //bit 7
        UINT32 reserved1:6;                         //bits 1-6
        UINT32 enab_jab_protect:1;                  //bit 0
    #else
        UINT32 enab_jab_protect:1;                  //bit 0
        UINT32 reserved1:6;                         //bits 1-6
        UINT32 reset_gpsi:1;                        //bit 7
        UINT32 disable_link_fail:1;                 //bit 8
        UINT32 no_cipher:1;                         //bit 9
        UINT32 force_quiet:1;                       //bit 10
        UINT32 reserved2:4;                         //bits 11-14
        UINT32 reset_pe100x:1;                      //bit 15
        UINT32 speed:1;                             //bit 16
        UINT32 reserved3:6;                         //bits 17-22
        UINT32 reset_per_mii:1;                     //bit 23
        UINT32 phy_mode:1;                          //bit 24
        UINT32 lhd_mode:1;                          //bit 25
        UINT32 ghd_mode:1;                          //bit 26
        UINT32 tbi_mode:1;                          //bit 27
        UINT32 reserved4:3;                         //bit 28-30
        UINT32 reset_if_module:1;                   //bit 31
    #endif
    } bits;
} 
MAC_IF_CTRL_t, *PMAC_IF_CTRL_t;


/******************************************************************************
   structure for Interface Status reg in mac address map.  
   located at address 0x503C
 *****************************************************************************/
typedef union _MAC_IF_STAT_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:22;                         //bits 10-31
        UINT32 excess_defer:1;                      //bit 9
        UINT32 clash:1;                             //bit 8
        UINT32 phy_jabber:1;                        //bit 7
        UINT32 phy_link_ok:1;                       //bit 6
        UINT32 phy_full_duplex:1;                   //bit 5
        UINT32 phy_speed:1;                         //bit 4
        UINT32 pe100x_link_fail:1;                  //bit 3
        UINT32 pe10t_loss_carrie:1;                 //bit 2
        UINT32 pe10t_sqe_error:1;                   //bit 1
        UINT32 pe10t_jabber:1;                      //bit 0
    #else
        UINT32 pe10t_jabber:1;                      //bit 0
        UINT32 pe10t_sqe_error:1;                   //bit 1
        UINT32 pe10t_loss_carrie:1;                 //bit 2
        UINT32 pe100x_link_fail:1;                  //bit 3
        UINT32 phy_speed:1;                         //bit 4
        UINT32 phy_full_duplex:1;                   //bit 5
        UINT32 phy_link_ok:1;                       //bit 6
        UINT32 phy_jabber:1;                        //bit 7
        UINT32 clash:1;                             //bit 8
        UINT32 excess_defer:1;                      //bit 9
        UINT32 reserved:22;                         //bits 10-31
    #endif
    } bits;
} 
MAC_IF_STAT_t, *PMAC_IF_STAT_t;


/******************************************************************************
   structure for Mac Station Address, Part 1 reg in mac address map.  
   located at address 0x5040
 *****************************************************************************/
typedef union _MAC_STATION_ADDR1_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 Octet6:8;                            //bits 24-31
        UINT32 Octet5:8;                            //bits 16-23
        UINT32 Octet4:8;                            //bits 8-15
        UINT32 Octet3:8;                            //bits 0-7
    #else
        UINT32 Octet3:8;                            //bits 0-7
        UINT32 Octet4:8;                            //bits 8-15
        UINT32 Octet5:8;                            //bits 16-23
        UINT32 Octet6:8;                            //bits 24-31
    #endif
    } bits;
} 
MAC_STATION_ADDR1_t, *PMAC_STATION_ADDR1_t;


/******************************************************************************
   structure for Mac Station Address, Part 2 reg in mac address map.  
   located at address 0x5044
 *****************************************************************************/
typedef union _MAC_STATION_ADDR2_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 Octet2:8;                            //bits 24-31
        UINT32 Octet1:8;                            //bits 16-23
        UINT32 reserved:16;                         //bits 0-15
    #else
        UINT32 reserved:16;                         //bit 0-15
        UINT32 Octet1:8;                            //bits 16-23
        UINT32 Octet2:8;                            //bits 24-31
    #endif
    } bits;
} 
MAC_STATION_ADDR2_t, *PMAC_STATION_ADDR2_t;


/******************************************************************************
   MAC Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _MAC_t
{                                                   //Location:
    MAC_CFG1_t                  cfg1;               //  0x5000
    MAC_CFG2_t                  cfg2;               //  0x5004
    MAC_IPG_t                   ipg;                //  0x5008
    MAC_HFDP_t                  hfdp;               //  0x500C
    MAC_MAX_FM_LEN_t            max_fm_len;         //  0x5010
    MAC_RSV_t                   rsv1;               //  0x5014
    MAC_RSV_t                   rsv2;               //  0x5018
    MAC_TEST_t                  mac_test;           //  0x501C
    MII_MGMT_CFG_t              mii_mgmt_cfg;       //  0x5020
    MII_MGMT_CMD_t              mii_mgmt_cmd;       //  0x5024
    MII_MGMT_ADDR_t             mii_mgmt_addr;      //  0x5028
    MII_MGMT_CTRL_t             mii_mgmt_ctrl;      //  0x502C
    MII_MGMT_STAT_t             mii_mgmt_stat;      //  0x5030
    MII_MGMT_INDICATOR_t        mii_mgmt_indicator; //  0x5034
    MAC_IF_CTRL_t               if_ctrl;            //  0x5038
    MAC_IF_STAT_t               if_stat;            //  0x503C
    MAC_STATION_ADDR1_t         station_addr_1;     //  0x5040
    MAC_STATION_ADDR2_t         station_addr_2;     //  0x5044
}
MAC_t, *PMAC_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                   END OF MAC REGISTER ADDRESS MAP                   ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                START OF MAC STAT REGISTER ADDRESS MAP               ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for Carry Register One and it's Mask Register reg located in mac
   stat address map address 0x6130 and 0x6138.
 *****************************************************************************/
typedef union _MAC_STAT_REG_1_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 tr64:1;                              //bit 31
        UINT32 tr127:1;                             //bit 30
        UINT32 tr255:1;                             //bit 29
        UINT32 tr511:1;                             //bit 28
        UINT32 tr1k:1;                              //bit 27
        UINT32 trmax:1;                             //bit 26
        UINT32 trmgv:1;                             //bit 25
        UINT32 unused:8;                            //bits 17-24
        UINT32 rbyt:1;                             //bit 16
        UINT32 rpkt:1;                             //bit 15
        UINT32 rfcs:1;                             //bit 14
        UINT32 rmca:1;                             //bit 13
        UINT32 rbca:1;                             //bit 12
        UINT32 rxcf:1;                             //bit 11
        UINT32 rxpf:1;                             //bit 10
        UINT32 rxuo:1;                             //bit 9
        UINT32 raln:1;                             //bit 8
        UINT32 rflr:1;                             //bit 7
        UINT32 rcde:1;                             //bit 6
        UINT32 rcse:1;                             //bit 5
        UINT32 rund:1;                             //bit 4
        UINT32 rovr:1;                             //bit 3
        UINT32 rfrg:1;                             //bit 2
        UINT32 rjbr:1;                             //bit 1
        UINT32 rdrp:1;                             //bit 0
    #else
        UINT32 rdrp:1;                             //bit 0
        UINT32 rjbr:1;                             //bit 1
        UINT32 rfrg:1;                             //bit 2
        UINT32 rovr:1;                             //bit 3
        UINT32 rund:1;                             //bit 4
        UINT32 rcse:1;                             //bit 5
        UINT32 rcde:1;                             //bit 6 
        UINT32 rflr:1;                             //bit 7
        UINT32 raln:1;                             //bit 8
        UINT32 rxuo:1;                             //bit 9
        UINT32 rxpf:1;                             //bit 10
        UINT32 rxcf:1;                             //bit 11
        UINT32 rbca:1;                             //bit 12
        UINT32 rmca:1;                             //bit 13
        UINT32 rfcs:1;                             //bit 14
        UINT32 rpkt:1;                             //bit 15
        UINT32 rbyt:1;                             //bit 16
        UINT32 unused:8;                            //bits 17-24
        UINT32 trmgv:1;                             //bit 25
        UINT32 trmax:1;                             //bit 26
        UINT32 tr1k:1;                              //bit 27
        UINT32 tr511:1;                             //bit 28
        UINT32 tr255:1;                             //bit 29
        UINT32 tr127:1;                             //bit 30
        UINT32 tr64:1;                              //bit 31
    #endif
    } bits;
} 
MAC_STAT_REG_1_t, *PMAC_STAT_REG_1_t;


/******************************************************************************
   structure for Carry Register Two Mask Register reg in mac stat address map.  
   located at address 0x613C
 *****************************************************************************/
typedef union _MAC_STAT_REG_2_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 unused:12;                           //bit 20-31
        UINT32 tjbr:1;                             //bit 19
        UINT32 tfcs:1;                             //bit 18
        UINT32 txcf:1;                             //bit 17        
        UINT32 tovr:1;                             //bit 16
        UINT32 tund:1;                             //bit 15
        UINT32 tfrg:1;                             //bit 14
        UINT32 tbyt:1;                             //bit 13
        UINT32 tpkt:1;                             //bit 12
        UINT32 tmca:1;                             //bit 11
        UINT32 tbca:1;                             //bit 10
        UINT32 txpf:1;                             //bit 9
        UINT32 tdfr:1;                             //bit 8
        UINT32 tedf:1;                             //bit 7
        UINT32 tscl:1;                             //bit 6
        UINT32 tmcl:1;                             //bit 5
        UINT32 tlcl:1;                             //bit 4
        UINT32 txcl:1;                             //bit 3
        UINT32 tncl:1;                             //bit 2
        UINT32 tpfh:1;                             //bit 1
        UINT32 tdrp:1;                             //bit 0
    #else
        UINT32 tdrp:1;                             //bit 0
        UINT32 tpfh:1;                             //bit 1
        UINT32 tncl:1;                             //bit 2
        UINT32 txcl:1;                             //bit 3
        UINT32 tlcl:1;                             //bit 4
        UINT32 tmcl:1;                             //bit 5
        UINT32 tscl:1;                             //bit 6
        UINT32 tedf:1;                             //bit 7
        UINT32 tdfr:1;                             //bit 8
        UINT32 txpf:1;                             //bit 9
        UINT32 tbca:1;                             //bit 10
        UINT32 tmca:1;                             //bit 11
        UINT32 tpkt:1;                             //bit 12
        UINT32 tbyt:1;                             //bit 13
        UINT32 tfrg:1;                             //bit 14
        UINT32 tund:1;                             //bit 15
        UINT32 tovr:1;                             //bit 16
        UINT32 txcf:1;                             //bit 17    
        UINT32 tfcs:1;                             //bit 18
        UINT32 tjbr:1;                             //bit 19
        UINT32 unused:12;                         //bit 20-31
    #endif
    } bits;
} 
MAC_STAT_REG_2_t, *PMAC_STAT_REG_2_t;




/******************************************************************************
   MAC STATS Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _MAC_STAT_t
{                                                   //Location:
    UINT32 pad[32];                                 //  0x6000 - 607C

    //Tx/Rx 0-64 Byte Frame Counter
    UINT32 TR64;                                    //  0x6080
    
    //Tx/Rx 65-127 Byte Frame Counter
    UINT32 TR127;                                   //  0x6084
    
    //Tx/Rx 128-255 Byte Frame Counter
    UINT32 TR255;                                   //  0x6088
    
    //Tx/Rx 256-511 Byte Frame Counter
    UINT32 TR511;                                   //  0x608C
    
    //Tx/Rx 512-1023 Byte Frame Counter
    UINT32 TR1K;                                    //  0x6090
    
    //Tx/Rx 1024-1518 Byte Frame Counter
    UINT32 TRMax;                                   //  0x6094
    
    //Tx/Rx 1519-1522 Byte Good VLAN Frame Count
    UINT32 TRMgv;                                   //  0x6098
    
    //Rx Byte Counter
    UINT32 RByt;                                    //  0x609C
    
    //Rx Packet Counter
    UINT32 RPkt;                                    //  0x60A0
    
    //Rx FCS Error Counter
    UINT32 RFcs;                                    //  0x60A4
    
    //Rx Multicast Packet Counter
    UINT32 RMca;                                    //  0x60A8
    
    //Rx Broadcast Packet Counter
    UINT32 RBca;                                    //  0x60AC
    
    //Rx Control Frame Packet Counter
    UINT32 RxCf;                                    //  0x60B0
    
    //Rx Pause Frame Packet Counter
    UINT32 RxPf;                                    //  0x60B4
    
    //Rx Unknown OP Code Counter
    UINT32 RxUo;                                    //  0x60B8
    
    //Rx Alignment Error Counter
    UINT32 RAln;                                    //  0x60BC
    
    //Rx Frame Length Error Counter
    UINT32 RFlr;                                    //  0x60C0
    
    //Rx Code Error Counter
    UINT32 RCde;                                    //  0x60C4
    
    //Rx Carrier Sense Error Counter
    UINT32 RCse;                                    //  0x60C8
    
    //Rx Undersize Packet Counter
    UINT32 RUnd;                                    //  0x60CC
    
    //Rx Oversize Packet Counter
    UINT32 ROvr;                                    //  0x60D0
    
    //Rx Fragment Counter
    UINT32 RFrg;                                    //  0x60D4
    
    //Rx Jabber Counter
    UINT32 RJbr;                                    //  0x60D8
    
    //Rx Drop
    UINT32 RDrp;                                    //  0x60DC
    
    //Tx Byte Counter
    UINT32 TByt;                                    //  0x60E0
    
    //Tx Packet Counter
    UINT32 TPkt;                                    //  0x60E4
    
    //Tx Multicast Packet Counter
    UINT32 TMca;                                    //  0x60E8
    
    //Tx Broadcast Packet Counter
    UINT32 TBca;                                    //  0x60EC
    
    //Tx Pause Control Frame Counter
    UINT32 TxPf;                                    //  0x60F0
    
    //Tx Deferral Packet Counter
    UINT32 TDfr;                                    //  0x60F4
    
    //Tx Excessive Deferral Packet Counter
    UINT32 TEdf;                                    //  0x60F8
    
    //Tx Single Collision Packet Counter
    UINT32 TScl;                                    //  0x60FC
    
    //Tx Multiple Collision Packet Counter
    UINT32 TMcl;                                    //  0x6100
    
    //Tx Late Collision Packet Counter
    UINT32 TLcl;                                    //  0x6104
    
    //Tx Excessive Collision Packet Counter
    UINT32 TXcl;                                    //  0x6108
    
    //Tx Total Collision Packet Counter
    UINT32 TNcl;                                    //  0x610C
    
    //Tx Pause Frame Honored Counter
    UINT32 TPfh;                                    //  0x6110
    
    //Tx Drop Frame Counter
    UINT32 TDrp;                                    //  0x6114
    
    //Tx Jabber Frame Counter
    UINT32 TJbr;                                    //  0x6118
    
    //Tx FCS Error Counter
    UINT32 TFcs;                                    //  0x611C
    
    //Tx Control Frame Counter
    UINT32 TxCf;                                    //  0x6120
    
    //Tx Oversize Frame Counter
    UINT32 TOvr;                                    //  0x6124
    
    //Tx Undersize Frame Counter
    UINT32 TUnd;                                    //  0x6128
    
    //Tx Fragments Frame Counter
    UINT32 TFrg;                                    //  0x612C
    
    //Carry Register One Register
    MAC_STAT_REG_1_t Carry1;                        //  0x6130
    
    //Carry Register Two Register
    MAC_STAT_REG_2_t Carry2;                           //  0x6134
    
    //Carry Register One Mask Register
    MAC_STAT_REG_1_t Carry1M;                       //  0x6138
    
    //Carry Register Two Mask Register
    MAC_STAT_REG_2_t Carry2M;                       //  0x613C
}
MAC_STAT_t, *PMAC_STAT_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                END OF MAC STAT REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                  START OF MMC REGISTER ADDRESS MAP                  ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   structure for Main Memory Controller Control reg in mmc address map.  
   located at address 0x7000
 *****************************************************************************/
typedef union _MMC_CTRL_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 reserved:25;                         //bits 7-31
        UINT32 force_ce:1;                          //bit 6
        UINT32 rxdma_disable:1;                     //bit 5
        UINT32 txdma_disable:1;                     //bit 4
        UINT32 txmac_disable:1;                     //bit 3
        UINT32 rxmac_disable:1;                     //bit 2
        UINT32 arb_disable:1;                       //bit 1
        UINT32 mmc_enable:1;                        //bit 0
    #else
        UINT32 mmc_enable:1;                        //bit 0
        UINT32 arb_disable:1;                       //bit 1
        UINT32 rxmac_disable:1;                     //bit 2
        UINT32 txmac_disable:1;                     //bit 3
        UINT32 txdma_disable:1;                     //bit 4
        UINT32 rxdma_disable:1;                     //bit 5
        UINT32 force_ce:1;                          //bit 6
        UINT32 reserved:25;                         //bits 7-31
    #endif
    } bits;
} 
MMC_CTRL_t, *PMMC_CTRL_t;


/******************************************************************************
   structure for Main Memory Controller Host Memory Access Address reg in mmc 
   address map.  Located at address 0x7004
 *****************************************************************************/
typedef union _MMC_SRAM_ACCESS_t
{
    UINT32 value;
    struct     
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT32 byte_enable:16;                      //bits 16-31
        UINT32 reserved2:2;                         //bits 14-15
        UINT32 req_addr:10;                         //bits 4-13
        UINT32 reserved1:1;                         //bit 3
        UINT32 is_ctrl_word:1;                      //bit 2
        UINT32 wr_access:1;                         //bit 1
        UINT32 req_access:1;                        //bit 0
    #else
        UINT32 req_access:1;                        //bit 0
        UINT32 wr_access:1;                         //bit 1
        UINT32 is_ctrl_word:1;                      //bit 2
        UINT32 reserved1:1;                         //bit 3
        UINT32 req_addr:10;                         //bits 4-13
        UINT32 reserved2:2;                         //bits 14-15
        UINT32 byte_enable:16;                      //bits 16-31
    #endif
    } bits;
} 
MMC_SRAM_ACCESS_t, *PMMC_SRAM_ACCESS_t;


/******************************************************************************
   structure for Main Memory Controller Host Memory Access Data reg in mmc 
   address map.  Located at address 0x7008 - 0x7014
 *****************************************************************************/
typedef struct _MMC_SRAM_WORD_t
{
    UINT32 data;                                    //bits 0-31
} MMC_SRAM_WORD_t, *PMMC_SRAM_WORD_t;


/******************************************************************************
   Memory Control Module of JAGCore Address Mapping
 *****************************************************************************/
typedef struct _MMC_t
{                                                   //Location:
    MMC_CTRL_t          mmc_ctrl;                   //  0x7000
    MMC_SRAM_ACCESS_t   sram_access;                //  0x7004
    MMC_SRAM_WORD_t     sram_word1;                 //  0x7008
    MMC_SRAM_WORD_t     sram_word2;                 //  0x700C
    MMC_SRAM_WORD_t     sram_word3;                 //  0x7010
    MMC_SRAM_WORD_t     sram_word4;                 //  0x7014
}
MMC_t, *PMMC_t;
/*===========================================================================*/
/*===========================================================================*/
/*===                   END OF MMC REGISTER ADDRESS MAP                   ===*/
/*===========================================================================*/
/*===========================================================================*/



/*===========================================================================*/
/*===========================================================================*/
/*===                START OF EXP ROM REGISTER ADDRESS MAP                ===*/
/*===========================================================================*/
/*===========================================================================*/
/******************************************************************************
   Expansion ROM Module of JAGCore Address Mapping
 *****************************************************************************/

/* Take this out until it is not empty */
#if 0
typedef struct _EXP_ROM_t
{

}
EXP_ROM_t, *PEXP_ROM_t;
#endif
/*===========================================================================*/
/*===========================================================================*/
/*===                 END OF EXP ROM REGISTER ADDRESS MAP                 ===*/
/*===========================================================================*/
/*===========================================================================*/




/******************************************************************************
   JAGCore Address Mapping
 *****************************************************************************/
typedef struct _ADDRESS_MAP_t
{
    GLOBAL_t    global;
    UCHAR       unused_global[4096 - sizeof (GLOBAL_t)];    //unused section of global address map
    TXDMA_t     txdma;
    UCHAR       unused_txdma[4096 - sizeof (TXDMA_t)];     //unused section of txdma address map
    RXDMA_t     rxdma;
    UCHAR       unused_rxdma[4096 - sizeof (RXDMA_t)];     //unused section of rxdma address map
    TXMAC_t     txmac;
    UCHAR       unused_txmac[4096 - sizeof (TXMAC_t)];     //unused section of txmac address map
    RXMAC_t     rxmac;
    UCHAR       unused_rxmac[4096 - sizeof (RXMAC_t)];     //unused section of rxmac address map
    MAC_t       mac;
    UCHAR       unused_mac[4096 - sizeof (MAC_t)];       //unused section of mac address map
    MAC_STAT_t  macStat;
    UCHAR       unused_mac_stat[4096 - sizeof (MAC_STAT_t)];  //unused section of mac stat address map
    MMC_t       mmc;
    UCHAR       unused_mmc[4096 - sizeof (MMC_t)];       //unused section of mmc address map
    UCHAR       unused_[1015808];       //unused section of address map

/* Take this out until it is not empty */
#if 0
    EXP_ROM_t   exp_rom;
#endif

    UCHAR       unused_exp_rom[4096];   //MGS-size TBD
    UCHAR       unused__[524288];       //unused section of address map
} 
ADDRESS_MAP_t, *PADDRESS_MAP_t;
/*===========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _ET1310_ADDRESS_MAP_H_ */
