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
 * ET1310_phy.h - Defines, structs, enums, prototypes, etc. pertaining to the
 *                PHY.
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
         $Date: 2006/01/18 22:18:33 $
     $Revision: 1.10 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef _ET1310_PHY_H_
#define _ET1310_PHY_H_

#ifdef __cplusplus
extern "C" {
#endif




/******************************************************************************
   INCLUDES
 *****************************************************************************/
#include "ET1310_common.h"




#define TRUEPHY_SUCCESS 0
#define TRUEPHY_FAILURE 1
typedef void    *TRUEPHY_HANDLE;
typedef void    *TRUEPHY_PLATFORM_HANDLE;
typedef void    *TRUEPHY_OSAL_HANDLE;




/******************************************************************************
   CONSTANTS for PHY Register
 *****************************************************************************/
/* MI Register Addresses */
#define MI_CONTROL_REG                      0
#define MI_STATUS_REG                       1
#define MI_PHY_IDENTIFIER_1_REG             2
#define MI_PHY_IDENTIFIER_2_REG             3
#define MI_AUTONEG_ADVERTISEMENT_REG        4
#define MI_AUTONEG_LINK_PARTNER_ABILITY_REG 5
#define MI_AUTONEG_EXPANSION_REG            6
#define MI_AUTONEG_NEXT_PAGE_TRANSMIT_REG   7
#define MI_LINK_PARTNER_NEXT_PAGE_REG       8
#define MI_1000BASET_CONTROL_REG            9
#define MI_1000BASET_STATUS_REG             10
#define MI_RESERVED11_REG                   11
#define MI_RESERVED12_REG                   12
#define MI_RESERVED13_REG                   13
#define MI_RESERVED14_REG                   14
#define MI_EXTENDED_STATUS_REG              15

/* VMI Register Addresses */
#define VMI_RESERVED16_REG                  16
#define VMI_RESERVED17_REG                  17
#define VMI_RESERVED18_REG                  18
#define VMI_LOOPBACK_CONTROL_REG            19
#define VMI_RESERVED20_REG                  20
#define VMI_MI_CONTROL_REG                  21
#define VMI_PHY_CONFIGURATION_REG           22
#define VMI_PHY_CONTROL_REG                 23
#define VMI_INTERRUPT_MASK_REG              24
#define VMI_INTERRUPT_STATUS_REG            25
#define VMI_PHY_STATUS_REG                  26
#define VMI_LED_CONTROL_1_REG               27
#define VMI_LED_CONTROL_2_REG               28
#define VMI_RESERVED29_REG                  29
#define VMI_RESERVED30_REG                  30
#define VMI_RESERVED31_REG                  31




/******************************************************************************
   PHY Register Mapping(MI) Management Interface Regs
 *****************************************************************************/
typedef struct _MI_REGS_t 
{
    UCHAR bmcr;         // Basic mode control reg(Reg 0x00)
    UCHAR bmsr;         // Basic mode status reg(Reg 0x01)
    UCHAR idr1;         // Phy identifier reg 1(Reg 0x02)
    UCHAR idr2;         // Phy identifier reg 2(Reg 0x03)
    UCHAR anar;         // Auto-Negotiation advertisement(Reg 0x04)
    UCHAR anlpar;       // Auto-Negotiation link Partner Ability(Reg 0x05)
    UCHAR aner;         // Auto-Negotiation expansion reg(Reg 0x06)
    UCHAR annptr;       // Auto-Negotiation next page transmit reg(Reg 0x07)
    UCHAR lpnpr;        // link partner next page reg(Reg 0x08)
    UCHAR gcr;          // Gigabit basic mode control reg(Reg 0x09)
    UCHAR gsr;          // Gigabit basic mode status reg(Reg 0x0A)
    UCHAR mi_res1[4];   // Future use by MI working group(Reg 0x0B - 0x0E)
    UCHAR esr;          // Extended status reg(Reg 0x0F)
    UCHAR mi_res2[3];   // Future use by MI working group(Reg 0x10 - 0x12)
    UCHAR loop_ctl;     // Loopback Control Reg(Reg 0x13)
    UCHAR mi_res3;      // Future use by MI working group(Reg 0x14)
    UCHAR mcr;          // MI Control Reg(Reg 0x15)
    UCHAR pcr;          // Configuration Reg(Reg 0x16)
    UCHAR phy_ctl;      // PHY Control Reg(Reg 0x17)
    UCHAR imr;          // Interrupt Mask Reg(Reg 0x18)
    UCHAR isr;          // Interrupt Status Reg(Reg 0x19)
    UCHAR psr;          // PHY Status Reg(Reg 0x1A)
    UCHAR lcr1;         // LED Control 1 Reg(Reg 0x1B)
    UCHAR lcr2;         // LED Control 2 Reg(Reg 0x1C)
    UCHAR mi_res4[3];   // Future use by MI working group(Reg 0x1D - 0x1F)
} 
MI_REGS_t, *PMI_REGS_t;




/******************************************************************************
   MI Register 0: Basic mode control register
 *****************************************************************************/
typedef union _MI_BMCR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 reset:1;                             //bit 15
        UINT16 loopback:1;                          //bit 14
        UINT16 speed_sel:1;                         //bit 13
        UINT16 enable_autoneg:1;                    //bit 12
        UINT16 power_down:1;                        //bit 11
        UINT16 isolate:1;                           //bit 10
        UINT16 restart_autoneg:1;                   //bit 9
        UINT16 duplex_mode:1;                       //bit 8
        UINT16 col_test:1;                          //bit 7
        UINT16 speed_1000_sel:1;                    //bit 6
        UINT16 res1:6;                              //bits 0-5
    #else
        UINT16 res1:6;                              //bits 0-5
        UINT16 speed_1000_sel:1;                    //bit 6
        UINT16 col_test:1;                          //bit 7
        UINT16 duplex_mode:1;                       //bit 8
        UINT16 restart_autoneg:1;                   //bit 9
        UINT16 isolate:1;                           //bit 10
        UINT16 power_down:1;                        //bit 11
        UINT16 enable_autoneg:1;                    //bit 12
        UINT16 speed_sel:1;                         //bit 13
        UINT16 loopback:1;                          //bit 14
        UINT16 reset:1;                             //bit 15
    #endif
    } bits;
}
MI_BMCR_t, *PMI_BMCR_t;




/******************************************************************************
   MI Register 1:  Basic mode status register
 *****************************************************************************/
typedef union _MI_BMSR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 link_100T4:1;                        //bit 15
        UINT16 link_100fdx:1;                       //bit 14
        UINT16 link_100hdx:1;                       //bit 13
        UINT16 link_10fdx:1;                        //bit 12
        UINT16 link_10hdx:1;                        //bit 11
        UINT16 link_100T2fdx:1;                     //bit 10
        UINT16 link_100T2hdx:1;                     //bit 9
        UINT16 extend_status:1;                     //bit 8
        UINT16 res1:1;                              //bit 7
        UINT16 preamble_supress:1;                  //bit 6
        UINT16 auto_neg_complete:1;                 //bit 5
        UINT16 remote_fault:1;                      //bit 4
        UINT16 auto_neg_able:1;                     //bit 3
        UINT16 link_status:1;                       //bit 2
        UINT16 jabber_detect:1;                     //bit 1
        UINT16 ext_cap:1;                           //bit 0
    #else
        UINT16 ext_cap:1;                           //bit 0
        UINT16 jabber_detect:1;                     //bit 1
        UINT16 link_status:1;                       //bit 2
        UINT16 auto_neg_able:1;                     //bit 3
        UINT16 remote_fault:1;                      //bit 4
        UINT16 auto_neg_complete:1;                 //bit 5
        UINT16 preamble_supress:1;                  //bit 6
        UINT16 res1:1;                              //bit 7
        UINT16 extend_status:1;                     //bit 8
        UINT16 link_100T2hdx:1;                     //bit 9
        UINT16 link_100T2fdx:1;                     //bit 10
        UINT16 link_10hdx:1;                        //bit 11
        UINT16 link_10fdx:1;                        //bit 12
        UINT16 link_100hdx:1;                       //bit 13
        UINT16 link_100fdx:1;                       //bit 14
        UINT16 link_100T4:1;                        //bit 15
    #endif
    } bits;
} 
MI_BMSR_t, *PMI_BMSR_t;




/******************************************************************************
   MI Register 2: Physical Identifier 1
 *****************************************************************************/
typedef union _MI_IDR1_t 
{
    UINT16 value;
    struct 
    {
        UINT16 ieee_address:16;                     //0x0282 default(bits 0-15)
    } bits;
}
MI_IDR1_t, *PMI_IDR1_t;




/******************************************************************************
   MI Register 3: Physical Identifier 2
 *****************************************************************************/
typedef union _MI_IDR2_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 ieee_address:6;                      //111100 default(bits 10-15)
        UINT16 model_no:6;                          //000001 default(bits 4-9)
        UINT16 rev_no:4;                            //0010   default(bits 0-3)
    #else
        UINT16 rev_no:4;                            //0010   default(bits 0-3)
        UINT16 model_no:6;                          //000001 default(bits 4-9)
        UINT16 ieee_address:6;                      //111100 default(bits 10-15)
    #endif
    } bits;
}
MI_IDR2_t, *PMI_IDR2_t; 




/******************************************************************************
   MI Register 4: Auto-negotiation advertisement register
 *****************************************************************************/
typedef union _MI_ANAR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 np_indication:1;                     //bit 15
        UINT16 res2:1;                              //bit 14
        UINT16 remote_fault:1;                      //bit 13
        UINT16 res1:1;                              //bit 12
        UINT16 cap_asmpause:1;                      //bit 11
        UINT16 cap_pause:1;                         //bit 10
        UINT16 cap_100T4:1;                         //bit 9
        UINT16 cap_100fdx:1;                        //bit 8
        UINT16 cap_100hdx:1;                        //bit 7
        UINT16 cap_10fdx:1;                         //bit 6
        UINT16 cap_10hdx:1;                         //bit 5
        UINT16 selector:5;                          //bits 0-4
    #else
        UINT16 selector:5;                          //bits 0-4
        UINT16 cap_10hdx:1;                         //bit 5
        UINT16 cap_10fdx:1;                         //bit 6
        UINT16 cap_100hdx:1;                        //bit 7
        UINT16 cap_100fdx:1;                        //bit 8
        UINT16 cap_100T4:1;                         //bit 9
        UINT16 cap_pause:1;                         //bit 10
        UINT16 cap_asmpause:1;                      //bit 11
        UINT16 res1:1;                              //bit 12
        UINT16 remote_fault:1;                      //bit 13
        UINT16 res2:1;                              //bit 14
        UINT16 np_indication:1;                     //bit 15
    #endif
    } bits;
} 
MI_ANAR_t, *PMI_ANAR_t;




/******************************************************************************
   MI Register 5: Auto-negotiation link partner advertisement register
 *****************************************************************************/
typedef struct _MI_ANLPAR_t
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 np_indication:1;                     //bit 15
        UINT16 acknowledge:1;                       //bit 14
        UINT16 remote_fault:1;                      //bit 13
        UINT16 res1:1;                              //bit 12
        UINT16 cap_asmpause:1;                      //bit 11
        UINT16 cap_pause:1;                         //bit 10
        UINT16 cap_100T4:1;                         //bit 9
        UINT16 cap_100fdx:1;                        //bit 8
        UINT16 cap_100hdx:1;                        //bit 7
        UINT16 cap_10fdx:1;                         //bit 6
        UINT16 cap_10hdx:1;                         //bit 5
        UINT16 selector:5;                          //bits 0-4
    #else
        UINT16 selector:5;                          //bits 0-4
        UINT16 cap_10hdx:1;                         //bit 5
        UINT16 cap_10fdx:1;                         //bit 6
        UINT16 cap_100hdx:1;                        //bit 7
        UINT16 cap_100fdx:1;                        //bit 8
        UINT16 cap_100T4:1;                         //bit 9
        UINT16 cap_pause:1;                         //bit 10
        UINT16 cap_asmpause:1;                      //bit 11
        UINT16 res1:1;                              //bit 12
        UINT16 remote_fault:1;                      //bit 13
        UINT16 acknowledge:1;                       //bit 14
        UINT16 np_indication:1;                     //bit 15
    #endif
    } bits;
} 
MI_ANLPAR_t, *PMI_ANLPAR_t;




/******************************************************************************
   MI Register 6: Auto-negotiation expansion register
 *****************************************************************************/
typedef union _MI_ANER_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 res:11;                              //bits 5-15
        UINT16 pdf:1;                               //bit 4
        UINT16 lp_np_able:1;                        //bit 3
        UINT16 np_able:1;                           //bit 2
        UINT16 page_rx:1;                           //bit 1
        UINT16 lp_an_able:1;                        //bit 0
    #else
        UINT16 lp_an_able:1;                        //bit 0
        UINT16 page_rx:1;                           //bit 1
        UINT16 np_able:1;                           //bit 2
        UINT16 lp_np_able:1;                        //bit 3
        UINT16 pdf:1;                               //bit 4
        UINT16 res:11;                              //bits 5-15
    #endif
    } bits;
} 
MI_ANER_t, *PMI_ANER_t;




/******************************************************************************
   MI Register 7: Auto-negotiation next page transmit reg(0x07)
 *****************************************************************************/
typedef union _MI_ANNPTR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 np:1;                                //bit 15
        UINT16 res1:1;                              //bit 14
        UINT16 msg_page:1;                          //bit 13
        UINT16 ack2:1;                              //bit 12
        UINT16 toggle:1;                            //bit 11
        UINT16 msg:11;                              //bits 0-10
    #else
        UINT16 msg:11;                              //bits 0-10
        UINT16 toggle:1;                            //bit 11
        UINT16 ack2:1;                              //bit 12
        UINT16 msg_page:1;                          //bit 13
        UINT16 res1:1;                              //bit 14
        UINT16 np:1;                                //bit 15
    #endif
    } bits;
} 
MI_ANNPTR_t, *PMI_ANNPTR_t;




/******************************************************************************
   MI Register 8: Link Partner Next Page Reg(0x08)
 *****************************************************************************/
typedef union _MI_LPNPR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 np:1;                                //bit 15
        UINT16 ack:1;                               //bit 14
        UINT16 msg_page:1;                          //bit 13
        UINT16 ack2:1;                              //bit 12
        UINT16 toggle:1;                            //bit 11
        UINT16 msg:11;                              //bits 0-10
    #else
        UINT16 msg:11;                              //bits 0-10
        UINT16 toggle:1;                            //bit 11
        UINT16 ack2:1;                              //bit 12
        UINT16 msg_page:1;                          //bit 13
        UINT16 ack:1;                               //bit 14
        UINT16 np:1;                                //bit 15
    #endif
    } bits;
} 
MI_LPNPR_t, *PMI_LPNPR_t;




/******************************************************************************
   MI Register 9: 1000BaseT Control Reg(0x09)
 *****************************************************************************/
typedef union _MI_GCR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 test_mode:3;                         //bits 13-15
        UINT16 ms_config_en:1;                      //bit 12
        UINT16 ms_value:1;                          //bit 11
        UINT16 port_type:1;                         //bit 10
        UINT16 link_1000fdx:1;                      //bit 9
        UINT16 link_1000hdx:1;                      //bit 8
        UINT16 res:8;                               //bit 0-7
    #else
        UINT16 res:8;                               //bit 0-7
        UINT16 link_1000hdx:1;                      //bit 8
        UINT16 link_1000fdx:1;                      //bit 9
        UINT16 port_type:1;                         //bit 10
        UINT16 ms_value:1;                          //bit 11
        UINT16 ms_config_en:1;                      //bit 12
        UINT16 test_mode:3;                         //bits 13-15
    #endif
    } bits;
} 
MI_GCR_t, *PMI_GCR_t;




/******************************************************************************
   MI Register 10: 1000BaseT Status Reg(0x0A)
 *****************************************************************************/
typedef union _MI_GSR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 ms_config_fault:1;                   //bit 15
        UINT16 ms_resolve:1;                        //bit 14
        UINT16 local_rx_status:1;                   //bit 13
        UINT16 remote_rx_status:1;                  //bit 12
        UINT16 link_1000fdx:1;                      //bit 11
        UINT16 link_1000hdx:1;                      //bit 10
        UINT16 res:2;                               //bits 8-9
        UINT16 idle_err_cnt:8;                      //bits 0-7
    #else
        UINT16 idle_err_cnt:8;                      //bits 0-7
        UINT16 res:2;                               //bits 8-9
        UINT16 link_1000hdx:1;                      //bit 10
        UINT16 link_1000fdx:1;                      //bit 11
        UINT16 remote_rx_status:1;                  //bit 12
        UINT16 local_rx_status:1;                   //bit 13
        UINT16 ms_resolve:1;                        //bit 14
        UINT16 ms_config_fault:1;                   //bit 15
    #endif
    } bits;
} 
MI_GSR_t, *PMI_GSR_t;




/******************************************************************************
   MI Register 11 - 14: Reserved Regs(0x0B - 0x0E)
 *****************************************************************************/
typedef union _MI_RES_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 res15:1;                             //bit 15
        UINT16 res14:1;                             //bit 14
        UINT16 res13:1;                             //bit 13
        UINT16 res12:1;                             //bit 12
        UINT16 res11:1;                             //bit 11
        UINT16 res10:1;                             //bit 10
        UINT16 res9:1;                              //bit 9
        UINT16 res8:1;                              //bit 8
        UINT16 res7:1;                              //bit 7
        UINT16 res6:1;                              //bit 6
        UINT16 res5:1;                              //bit 5
        UINT16 res4:1;                              //bit 4
        UINT16 res3:1;                              //bit 3
        UINT16 res2:1;                              //bit 2
        UINT16 res1:1;                              //bit 1
        UINT16 res0:1;                              //bit 0
    #else
        UINT16 res0:1;                              //bit 0
        UINT16 res1:1;                              //bit 1
        UINT16 res2:1;                              //bit 2
        UINT16 res3:1;                              //bit 3
        UINT16 res4:1;                              //bit 4
        UINT16 res5:1;                              //bit 5
        UINT16 res6:1;                              //bit 6
        UINT16 res7:1;                              //bit 7
        UINT16 res8:1;                              //bit 8
        UINT16 res9:1;                              //bit 9
        UINT16 res10:1;                             //bit 10
        UINT16 res11:1;                             //bit 11
        UINT16 res12:1;                             //bit 12
        UINT16 res13:1;                             //bit 13
        UINT16 res14:1;                             //bit 14
        UINT16 res15:1;                             //bit 15
    #endif
    } bits;
}
MI_RES_t, *PMI_RES_t;




/******************************************************************************
   MI Register 15: Extended status Reg(0x0F)
 *****************************************************************************/
typedef union _MI_ESR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 link_1000Xfdx:1;                     //bit 15
        UINT16 link_1000Xhdx:1;                     //bit 14
        UINT16 link_1000fdx:1;                      //bit 13
        UINT16 link_1000hdx:1;                      //bit 12
        UINT16 res:12;                              //bit 0-11
    #else
        UINT16 res:12;                              //bit 0-11
        UINT16 link_1000hdx:1;                      //bit 12
        UINT16 link_1000fdx:1;                      //bit 13
        UINT16 link_1000Xhdx:1;                     //bit 14
        UINT16 link_1000Xfdx:1;                     //bit 15
    #endif
    } bits;
}
MI_ESR_t, *PMI_ESR_t;




/******************************************************************************
   MI Register 16 - 18: Reserved Reg(0x10-0x12)
 *****************************************************************************/




/******************************************************************************
   MI Register 19: Loopback Control Reg(0x13)
 *****************************************************************************/
typedef union _MI_LCR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 mii_en:1;                            //bit 15
        UINT16 pcs_en:1;                            //bit 14
        UINT16 pmd_en:1;                            //bit 13
        UINT16 all_digital_en:1;                    //bit 12
        UINT16 replica_en:1;                        //bit 11
        UINT16 line_driver_en:1;                    //bit 10
        UINT16 res:10;                              //bit 0-9
    #else
        UINT16 res:10;                              //bit 0-9
        UINT16 line_driver_en:1;                    //bit 10
        UINT16 replica_en:1;                        //bit 11
        UINT16 all_digital_en:1;                    //bit 12
        UINT16 pmd_en:1;                            //bit 13
        UINT16 pcs_en:1;                            //bit 14
        UINT16 mii_en:1;                            //bit 15
    #endif
    } bits;
}
MI_LCR_t, *PMI_LCR_t;




/******************************************************************************
   MI Register 20: Reserved Reg(0x14)
 *****************************************************************************/




/******************************************************************************
   MI Register 21: Management Interface Control Reg(0x15)
 *****************************************************************************/
typedef union _MI_MICR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 res1:5;                              //bits 11-15
        UINT16 mi_error_count:7;                    //bits 4-10
        UINT16 res2:1;                              //bit 3
        UINT16 ignore_10g_fr:1;                     //bit 2
        UINT16 res3:1;                              //bit 1
        UINT16 preamble_supress_en:1;               //bit 0
    #else
        UINT16 preamble_supress_en:1;               //bit 0
        UINT16 res3:1;                              //bit 1
        UINT16 ignore_10g_fr:1;                     //bit 2
        UINT16 res2:1;                              //bit 3
        UINT16 mi_error_count:7;                    //bits 4-10
        UINT16 res1:5;                              //bits 11-15
    #endif
    } bits;
}
MI_MICR_t, *PMI_MICR_t;




/******************************************************************************
   MI Register 22: PHY Configuration Reg(0x16)
 *****************************************************************************/
typedef union _MI_PHY_CONFIG_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 crs_tx_en:1;                         //bit 15
        UINT16 res1:1;                              //bit 14
        UINT16 tx_fifo_depth:2;                     //bits 12-13
        UINT16 speed_downshift:2;                   //bits 10-11
        UINT16 pbi_detect:1;                        //bit 9
        UINT16 tbi_rate:1;                          //bit 8
        UINT16 alternate_np:1;                      //bit 7
        UINT16 group_mdio_en:1;                     //bit 6
        UINT16 tx_clock_en:1;                       //bit 5
        UINT16 sys_clock_en:1;                      //bit 4
        UINT16 res2:1;                              //bit 3
        UINT16 mac_if_mode:3;                       //bits 0-2
    #else
        UINT16 mac_if_mode:3;                       //bits 0-2
        UINT16 res2:1;                              //bit 3
        UINT16 sys_clock_en:1;                      //bit 4
        UINT16 tx_clock_en:1;                       //bit 5
        UINT16 group_mdio_en:1;                     //bit 6
        UINT16 alternate_np:1;                      //bit 7
        UINT16 tbi_rate:1;                          //bit 8
        UINT16 pbi_detect:1;                        //bit 9
        UINT16 speed_downshift:2;                   //bits 10-11
        UINT16 tx_fifo_depth:2;                     //bits 12-13
        UINT16 res1:1;                              //bit 14
        UINT16 crs_tx_en:1;                         //bit 15
    #endif
    } bits;
}
MI_PHY_CONFIG_t, *PMI_PHY_CONFIG_t;




/******************************************************************************
   MI Register 23: PHY CONTROL Reg(0x17)
 *****************************************************************************/
typedef union _MI_PHY_CONTROL_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 res1:1;                              //bit 15
        UINT16 tdr_en:1;                            //bit 14
        UINT16 res2:1;                              //bit 13
        UINT16 downshift_attempts:2;                //bits 11-12
        UINT16 res3:5;                              //bit 6-10
        UINT16 jabber_10baseT:1;                    //bit 5
        UINT16 sqe_10baseT:1;                       //bit 4
        UINT16 tp_loopback_10baseT:1;               //bit 3
        UINT16 preamble_gen_en:1;                   //bit 2
        UINT16 res4:1;                              //bit 1
        UINT16 force_int:1;                         //bit 0
    #else
        UINT16 force_int:1;                         //bit 0
        UINT16 res4:1;                              //bit 1
        UINT16 preamble_gen_en:1;                   //bit 2
        UINT16 tp_loopback_10baseT:1;               //bit 3
        UINT16 sqe_10baseT:1;                       //bit 4
        UINT16 jabber_10baseT:1;                    //bit 5
        UINT16 res3:5;                              //bit 6-10
        UINT16 downshift_attempts:2;                //bits 11-12
        UINT16 res2:1;                              //bit 13
        UINT16 tdr_en:1;                            //bit 14
        UINT16 res1:1;                              //bit 15
    #endif
    } bits;
}
MI_PHY_CONTROL_t, *PMI_PHY_CONTROL_t;




/******************************************************************************
   MI Register 24: Interrupt Mask Reg(0x18)
 *****************************************************************************/
typedef union _MI_IMR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 res1:6;                              //bits 10-15
        UINT16 mdio_sync_lost:1;                    //bit 9
        UINT16 autoneg_status:1;                    //bit 8
        UINT16 hi_bit_err:1;                        //bit 7
        UINT16 np_rx:1;                             //bit 6
        UINT16 err_counter_full:1;                  //bit 5
        UINT16 fifo_over_underflow:1;               //bit 4
        UINT16 rx_status:1;                         //bit 3
        UINT16 link_status:1;                       //bit 2
        UINT16 automatic_speed:1;                   //bit 1
        UINT16 int_en:1;                            //bit 0
    #else
        UINT16 int_en:1;                            //bit 0
        UINT16 automatic_speed:1;                   //bit 1
        UINT16 link_status:1;                       //bit 2
        UINT16 rx_status:1;                         //bit 3
        UINT16 fifo_over_underflow:1;               //bit 4
        UINT16 err_counter_full:1;                  //bit 5
        UINT16 np_rx:1;                             //bit 6
        UINT16 hi_bit_err:1;                        //bit 7
        UINT16 autoneg_status:1;                    //bit 8
        UINT16 mdio_sync_lost:1;                    //bit 9
        UINT16 res1:6;                              //bits 10-15
    #endif
    } bits;
}
MI_IMR_t, *PMI_IMR_t;




/******************************************************************************
   MI Register 25: Interrupt Status Reg(0x19)
 *****************************************************************************/
typedef union _MI_ISR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 res1:6;                              //bits 10-15
        UINT16 mdio_sync_lost:1;                    //bit 9
        UINT16 autoneg_status:1;                    //bit 8
        UINT16 hi_bit_err:1;                        //bit 7
        UINT16 np_rx:1;                             //bit 6
        UINT16 err_counter_full:1;                  //bit 5
        UINT16 fifo_over_underflow:1;               //bit 4
        UINT16 rx_status:1;                         //bit 3
        UINT16 link_status:1;                       //bit 2
        UINT16 automatic_speed:1;                   //bit 1
        UINT16 int_en:1;                            //bit 0
    #else
        UINT16 int_en:1;                            //bit 0
        UINT16 automatic_speed:1;                   //bit 1
        UINT16 link_status:1;                       //bit 2
        UINT16 rx_status:1;                         //bit 3
        UINT16 fifo_over_underflow:1;               //bit 4
        UINT16 err_counter_full:1;                  //bit 5
        UINT16 np_rx:1;                             //bit 6
        UINT16 hi_bit_err:1;                        //bit 7
        UINT16 autoneg_status:1;                    //bit 8
        UINT16 mdio_sync_lost:1;                    //bit 9
        UINT16 res1:6;                              //bits 10-15
    #endif
    } bits;
}
MI_ISR_t, *PMI_ISR_t;




/******************************************************************************
   MI Register 26: PHY Status Reg(0x1A)
 *****************************************************************************/
typedef union _MI_PSR_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 res1:1;                              //bit 15
        UINT16 autoneg_fault:2;                     //bit 13-14
        UINT16 autoneg_status:1;                    //bit 12
        UINT16 mdi_x_status:1;                      //bit 11
        UINT16 polarity_status:1;                   //bit 10
        UINT16 speed_status:2;                      //bits 8-9
        UINT16 duplex_status:1;                     //bit 7
        UINT16 link_status:1;                       //bit 6
        UINT16 tx_status:1;                         //bit 5
        UINT16 rx_status:1;                         //bit 4
        UINT16 collision_status:1;                  //bit 3
        UINT16 autoneg_en:1;                        //bit 2
        UINT16 pause_en:1;                          //bit 1
        UINT16 asymmetric_dir:1;                    //bit 0
    #else
        UINT16 asymmetric_dir:1;                    //bit 0
        UINT16 pause_en:1;                          //bit 1
        UINT16 autoneg_en:1;                        //bit 2
        UINT16 collision_status:1;                  //bit 3
        UINT16 rx_status:1;                         //bit 4
        UINT16 tx_status:1;                         //bit 5
        UINT16 link_status:1;                       //bit 6
        UINT16 duplex_status:1;                     //bit 7
        UINT16 speed_status:2;                      //bits 8-9
        UINT16 polarity_status:1;                   //bit 10
        UINT16 mdi_x_status:1;                      //bit 11
        UINT16 autoneg_status:1;                    //bit 12
        UINT16 autoneg_fault:2;                     //bit 13-14
        UINT16 res1:1;                              //bit 15
    #endif
    } bits;
}
MI_PSR_t, *PMI_PSR_t;




/******************************************************************************
   MI Register 27: LED Control Reg 1(0x1B)
 *****************************************************************************/
typedef union _MI_LCR1_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 res1:2;                              //bits 14-15
        UINT16 led_dup_indicate:2;                  //bits 12-13
        UINT16 led_10baseT:2;                       //bits 10-11
        UINT16 led_collision:2;                     //bits 8-9
        UINT16 res2:2;                              //bits 6-7
        UINT16 res3:2;                              //bits 4-5
        UINT16 pulse_dur:2;                         //bits 2-3
        UINT16 pulse_stretch1:1;                    //bit 1
        UINT16 pulse_stretch0:1;                    //bit 0
    #else
        UINT16 pulse_stretch0:1;                    //bit 0
        UINT16 pulse_stretch1:1;                    //bit 1
        UINT16 pulse_dur:2;                         //bits 2-3
        UINT16 res3:2;                              //bits 4-5
        UINT16 res2:2;                              //bits 6-7
        UINT16 led_collision:2;                     //bits 8-9
        UINT16 led_10baseT:2;                       //bits 10-11
        UINT16 led_dup_indicate:2;                  //bits 12-13
        UINT16 res1:2;                              //bits 14-15
    #endif
    } bits;
}
MI_LCR1_t, *PMI_LCR1_t;




/******************************************************************************
   MI Register 28: LED Control Reg 2(0x1C)
 *****************************************************************************/
typedef union _MI_LCR2_t 
{
    UINT16 value;
    struct 
    {
    #ifdef _BIT_FIELDS_HTOL
        UINT16 led_link:4;                          //bits 12-15
        UINT16 led_tx_rx:4;                         //bits 8-11
        UINT16 led_100BaseTX:4;                     //bits 4-7
        UINT16 led_1000BaseT:4;                     //bits 0-3
    #else
        UINT16 led_1000BaseT:4;                     //bits 0-3
        UINT16 led_100BaseTX:4;                     //bits 4-7
        UINT16 led_tx_rx:4;                         //bits 8-11
        UINT16 led_link:4;                          //bits 12-15
    #endif
    } bits;
}
MI_LCR2_t, *PMI_LCR2_t;




/******************************************************************************
   MI Register 29 - 31: Reserved Reg(0x1D - 0x1E)
 *****************************************************************************/




/******************************************************************************
   TruePHY headers
 *****************************************************************************/
typedef struct _TRUEPHY_ACCESS_MI_REGS_
{
    TRUEPHY_HANDLE hTruePhy;
    INT32          nPhyId;
    UCHAR          bReadWrite;
    PUCHAR         pbyRegs;
    PUCHAR         pwData;
    INT32          nRegCount;
} 
TRUEPHY_ACCESS_MI_REGS, *PTRUEPHY_ACCESS_MI_REGS;




/******************************************************************************
   TruePHY headers
 *****************************************************************************/
typedef struct _TAG_TPAL_ACCESS_MI_REGS_
{
    UINT32      nPhyId;
    UCHAR       bReadWrite;
    UINT32      nRegCount;
    UINT16      Data[4096];
    UCHAR       Regs[4096];
} TPAL_ACCESS_MI_REGS, *PTPAL_ACCESS_MI_REGS;




/******************************************************************************
   Required TruePHY PROTOTYPES
 *****************************************************************************/
/******************************************************************************
   TYPE DEFINITIONS
 *****************************************************************************/
typedef TRUEPHY_HANDLE TPAL_HANDLE;

#ifndef	__APPLE__

/******************************************************************************
   Forward declaration of the private adapter structure
 *****************************************************************************/
struct et131x_adapter;


///////////////////////////////////////////////////////////////////////////////
////                  TruePHY Platform Specific Functions                  ////
///////////////////////////////////////////////////////////////////////////////
//TRUEPHY_HANDLE TPAL_PlatformInit( void *                      pPlatformInfo,
//                                  PTRUEPHY_PLATFORM_FUNCTIONS pPlatFunctions );

INT32          TPAL_MiAccessRegs( TPAL_HANDLE hPlatform,
                                  INT32       nPhyId,
                                  PUCHAR      pbyAccessFlags,
                                  PUCHAR      pbyRegisters,
                                  PUINT16     pwData,
                                  PUINT16     pwAndMasks,
                                  PUINT16     pwOrMasks,
                                  INT32       nRegCount );

void           TPAL_PlatformExit( TRUEPHY_PLATFORM_HANDLE hPlatform );




///////////////////////////////////////////////////////////////////////////////
////                         OS Specific Functions                         ////
///////////////////////////////////////////////////////////////////////////////
void  *TPAL_AllocMem( TRUEPHY_PLATFORM_HANDLE hPlatform, u_long ulNumBytes );
void   TPAL_FreeMem( TRUEPHY_PLATFORM_HANDLE hPlatform, void *pMemBlock );
void   TPAL_Sleep( TRUEPHY_PLATFORM_HANDLE hPlatform, u_long ulMsec );
u_long TPAL_GetSystemUpTime( TRUEPHY_PLATFORM_HANDLE hPlatform );
void   TPAL_GetLinkStatusInfo( struct et131x_adapter *adapter );

INT32 TPAL_SetPhy10HalfDuplex( struct et131x_adapter *adapter );
INT32 TPAL_SetPhy10FullDuplex( struct et131x_adapter *adapter );
void  TPAL_SetPhy10Force( struct et131x_adapter * pAdapter );
INT32 TPAL_SetPhy100HalfDuplex( struct et131x_adapter *adapter );
INT32 TPAL_SetPhy100FullDuplex( struct et131x_adapter *adapter );
void  TPAL_SetPhy100Force( struct et131x_adapter * pAdapter );
INT32 TPAL_SetPhy1000FullDuplex( struct et131x_adapter *adapter);
INT32 TPAL_SetPhyAutoNeg( struct et131x_adapter *adapter );


/******************************************************************************
   PROTOTYPES for ET1310_phy.c
 *****************************************************************************/
int et131x_xcvr_find( struct et131x_adapter *adapter );

int et131x_setphy_normal( struct et131x_adapter *adapter );
INT32 et131x_xcvr_init( struct et131x_adapter *adapter );

INT32 MiRead( struct et131x_adapter *adapter,
              UINT8 xcvrAddr,
              UINT8 xcvrReg, 
              UINT16 *value );

INT32 MiWrite( struct et131x_adapter *adapter,
               UINT8 xcvrAddr,
               UINT8 xcvReg, 
               UINT16 value );

void et131x_Mii_check( struct et131x_adapter *pAdapter,
                       MI_BMSR_t bmsr,
                       MI_BMSR_t bmsr_ints );



/******************************************************************************
   This last is not strictly required (the driver could call the TPAL 
   version instead), but this sets the adapter up correctly, and calls the
   access routine indirectly.  This protects the driver from changes in TPAL.
 *****************************************************************************/
void SetPhy_10BaseTHalfDuplex( struct et131x_adapter *adapter );

#endif



/******************************************************************************
   Defines for PHY access routines
 *****************************************************************************/
// Define bit operation flags
#define TRUEPHY_BIT_CLEAR               0
#define TRUEPHY_BIT_SET                 1
#define TRUEPHY_BIT_READ                2

// Define read/write operation flags 
#ifndef TRUEPHY_READ
#define TRUEPHY_READ                    0
#define TRUEPHY_WRITE                   1
#define TRUEPHY_MASK                    2
#endif

// Define speeds
#define TRUEPHY_SPEED_10MBPS            0
#define TRUEPHY_SPEED_100MBPS           1
#define TRUEPHY_SPEED_1000MBPS          2

// Define duplex modes
#define TRUEPHY_DUPLEX_HALF             0
#define TRUEPHY_DUPLEX_FULL             1

// Define master/slave configuration values
#define TRUEPHY_CFG_SLAVE               0
#define TRUEPHY_CFG_MASTER              1

// Define MDI/MDI-X settings
#define TRUEPHY_MDI                     0
#define TRUEPHY_MDIX                    1
#define TRUEPHY_AUTO_MDI_MDIX           2

// Define 10Base-T link polarities
#define TRUEPHY_POLARITY_NORMAL         0
#define TRUEPHY_POLARITY_INVERTED       1

// Define auto-negotiation results
#define TRUEPHY_ANEG_NOT_COMPLETE       0
#define TRUEPHY_ANEG_COMPLETE           1
#define TRUEPHY_ANEG_DISABLED           2

/* Define duplex advertisment flags */
#define TRUEPHY_ADV_DUPLEX_NONE         0x00
#define TRUEPHY_ADV_DUPLEX_FULL         0x01
#define TRUEPHY_ADV_DUPLEX_HALF         0x02
#define TRUEPHY_ADV_DUPLEX_BOTH     \
    (TRUEPHY_ADV_DUPLEX_FULL | TRUEPHY_ADV_DUPLEX_HALF)

#define PHY_CONTROL                0x00     //#define TRU_MI_CONTROL_REGISTER                 0
#define PHY_STATUS                 0x01     //#define TRU_MI_STATUS_REGISTER                  1
#define PHY_ID_1                   0x02     //#define TRU_MI_PHY_IDENTIFIER_1_REGISTER        2
#define PHY_ID_2                   0x03     //#define TRU_MI_PHY_IDENTIFIER_2_REGISTER        3
#define PHY_AUTO_ADVERTISEMENT     0x04     //#define TRU_MI_ADVERTISEMENT_REGISTER           4
#define PHY_AUTO_LINK_PARTNER      0x05     //#define TRU_MI_LINK_PARTNER_ABILITY_REGISTER    5
#define PHY_AUTO_EXPANSION         0x06     //#define TRU_MI_EXPANSION_REGISTER               6
#define PHY_AUTO_NEXT_PAGE_TX      0x07     //#define TRU_MI_NEXT_PAGE_TRANSMIT_REGISTER      7
#define PHY_LINK_PARTNER_NEXT_PAGE 0x08     //#define TRU_MI_LINK_PARTNER_NEXT_PAGE_REGISTER  8
#define PHY_1000_CONTROL           0x09     //#define TRU_MI_1000BASET_CONTROL_REGISTER       9
#define PHY_1000_STATUS            0x0A     //#define TRU_MI_1000BASET_STATUS_REGISTER        10


#define PHY_EXTENDED_STATUS        0x0F     //#define TRU_MI_EXTENDED_STATUS_REGISTER         15

// some defines for modem registers that seem to be 'reserved'
#define PHY_INDEX_REG              0x10
#define PHY_DATA_REG               0x11

#define PHY_MPHY_CONTROL_REG       0x12     //#define TRU_VMI_MPHY_CONTROL_REGISTER           18
                                            
#define PHY_LOOPBACK_CONTROL       0x13     //#define TRU_VMI_LOOPBACK_CONTROL_1_REGISTER     19
                                            //#define TRU_VMI_LOOPBACK_CONTROL_2_REGISTER     20
#define PHY_REGISTER_MGMT_CONTROL  0x15     //#define TRU_VMI_MI_SEQ_CONTROL_REGISTER         21
#define PHY_CONFIG                 0x16     //#define TRU_VMI_CONFIGURATION_REGISTER          22
#define PHY_PHY_CONTROL            0x17     //#define TRU_VMI_PHY_CONTROL_REGISTER            23
#define PHY_INTERRUPT_MASK         0x18     //#define TRU_VMI_INTERRUPT_MASK_REGISTER         24
#define PHY_INTERRUPT_STATUS       0x19     //#define TRU_VMI_INTERRUPT_STATUS_REGISTER       25
#define PHY_PHY_STATUS             0x1A     //#define TRU_VMI_PHY_STATUS_REGISTER             26
#define PHY_LED_1                  0x1B     //#define TRU_VMI_LED_CONTROL_1_REGISTER          27
#define PHY_LED_2                  0x1C     //#define TRU_VMI_LED_CONTROL_2_REGISTER          28
                                            //#define TRU_VMI_LINK_CONTROL_REGISTER           29
                                            //#define TRU_VMI_TIMING_CONTROL_REGISTER 


#ifndef	__APPLE__

/******************************************************************************
   Prototypes for PHY access routines
 *****************************************************************************/
void ET1310_PhyInit( struct et131x_adapter *pAdapter );
void ET1310_PhyReset( struct et131x_adapter *pAdapter );
void ET1310_PhyPowerDown( struct et131x_adapter *pAdapter, BOOL_t down );
void ET1310_PhyAutoNeg( struct et131x_adapter *pAdapter, BOOL_t enable );
void ET1310_PhyDuplexMode( struct et131x_adapter *pAdapter, UINT16 duplex );
void ET1310_PhySpeedSelect( struct et131x_adapter *pAdapter, UINT16 speed );
void ET1310_PhyAdvertise1000BaseT( struct et131x_adapter *pAdapter, UINT16 duplex );
void ET1310_PhyAdvertise100BaseT( struct et131x_adapter *pAdapter, UINT16 duplex );
void ET1310_PhyAdvertise10BaseT( struct et131x_adapter *pAdapter, UINT16 duplex );
void ET1310_PhyLinkStatus( struct et131x_adapter *pAdapter, 
                           UCHAR  *ucLinkStatus,
                           UINT32 *uiAutoNeg,
                           UINT32 *uiLinkSpeed,
                           UINT32 *uiDuplexMode,
                           UINT32 *uiMdiMdix,
                           UINT32 *uiMasterSlave,
                           UINT32 *uiPolarity );
void ET1310_PhyAndOrReg( struct et131x_adapter *pAdapter,
                         UINT16 regnum,
                         UINT16 andMask,
                         UINT16 orMask );
void ET1310_PhyAccessMiBit( struct et131x_adapter *pAdapter,
                            UINT16 action,
                            UINT16 regnum,
                            UINT16 bitnum,
                            UINT8 *value );



#endif
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _ET1310_PHY_H_ */
