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
 * ET1310_pm.h - Defines, structs, enums, prototypes, etc. pertaining to power
 *               management.
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
         $Date: 2005/08/01 19:35:11 $
     $Revision: 1.5 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef _ET1310_PM_H_
#define _ET1310_PM_H_

#ifdef __cplusplus
extern "C" {
#endif




/******************************************************************************
   INCLUDES
 *****************************************************************************/
#include "ET1310_common.h"




/******************************************************************************
   CONSTANTS FOR POWER MANAGEMENT
 *****************************************************************************/
#define MAX_WOL_PACKET_SIZE    0x80
#define MAX_WOL_MASK_SIZE      ( MAX_WOL_PACKET_SIZE / 8 )
#define NUM_WOL_PATTERNS       0x5
#define CRC16_POLY             0x1021




/******************************************************************************
   Definition of NDIS_DEVICE_POWER_STATE
 *****************************************************************************/
typedef enum
{
    NdisDeviceStateUnspecified = 0,
    NdisDeviceStateD0,
    NdisDeviceStateD1,
    NdisDeviceStateD2,
    NdisDeviceStateD3
} NDIS_DEVICE_POWER_STATE;




/******************************************************************************
   Structure for Power Management Info
 *****************************************************************************/
typedef struct _MP_PM_CONFIG_SPACE_INFO_t
{
    UCHAR  capId;
    UCHAR  nextItemPtr;
    UINT16 pmcr;
    UINT16 pmcsr;
    UCHAR  pmscr_bse;
    UCHAR  pm_data_regs;
}
MP_PM_CONFIG_SPACE_INFO_t, *pMP_PM_CONFIG_SPACE_INFO_t;




typedef struct _MP_POWER_MGMT
{
    /**************************************************************************
        variable putting the phy into coma mode when boot up with no cable 
        plugged in after 5 seconds
     *************************************************************************/
    UCHAR               TransPhyComaModeOnBoot;


    /**************************************************************************
        Array holding the five CRC values that the device is currently using
        for WOL.  This will be queried when a pattern is to be removed.
     *************************************************************************/
    UINT32              localWolAndCrc0;
    UINT16              WOLPatternList[ NUM_WOL_PATTERNS ];
    UCHAR               WOLMaskList[ NUM_WOL_PATTERNS ][ MAX_WOL_MASK_SIZE ];
    UINT32              WOLMaskSize[ NUM_WOL_PATTERNS ];


    /**************************************************************************
       IP address   
     *************************************************************************/
    union
    {
        UINT32  u32;
        UCHAR   u8[4];
    } IPAddress;
  

    /**************************************************************************
       Current Power state of the adapter.
     *************************************************************************/
    NDIS_DEVICE_POWER_STATE PowerState;
    BOOL_t                  WOLState;
    BOOL_t                  WOLEnabled;
    BOOL_t                  Failed10Half;
    BOOL_t                  bFailedStateTransition;

    /**************************************************************************
        Next two used to save power information at power down.  
        This information will be used during power up to set up parts of Power 
        Management in JAGCore
     *************************************************************************/
    UINT32  tx_en;
    UINT32  rx_en;
    UINT16  PowerDownSpeed;
    UCHAR   PowerDownDuplex;

    MP_PM_CONFIG_SPACE_INFO_t   pmConfigRegs;
} MP_POWER_MGMT, *PMP_POWER_MGMT;



#ifndef	__APPLE__

/******************************************************************************
   Forward declaration of the private adapter structure
   ( IS THERE A WAY TO DO THIS WITH A TYPEDEF??? )
 *****************************************************************************/
struct et131x_adapter;




/******************************************************************************
   PROTOTYPES for ET1310_pm.c
 *****************************************************************************/
UINT16 CalculateCCITCRC16( PUCHAR Pattern, PUCHAR Mask, UINT32 MaskSize );

void EnablePhyComa( struct et131x_adapter *pAdapter );

void DisablePhyComa( struct et131x_adapter *pAdapter );
#endif



#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _ET1310_PM_H_ */
