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
 * ET1310_mac.h -  Defines, structs, enums, prototypes, etc. pertaining to the
 *                 MAC.
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
         $Date: 2005/08/01 19:35:11 $
     $Revision: 1.5 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef _ET1310_MAC_H_
#define _ET1310_MAC_H_

#ifdef __cplusplus
extern "C" {
#endif




/******************************************************************************
   INCLUDES
 *****************************************************************************/
#include "ET1310_common.h"




/******************************************************************************
   MACSTAT #defines
 *****************************************************************************/
#define COUNTER_WRAP_28_BIT 0x10000000
#define COUNTER_WRAP_22_BIT 0x400000
#define COUNTER_WRAP_16_BIT 0x10000
#define COUNTER_WRAP_12_BIT 0x1000

#define COUNTER_MASK_28_BIT (COUNTER_WRAP_28_BIT - 1)
#define COUNTER_MASK_22_BIT (COUNTER_WRAP_22_BIT - 1)
#define COUNTER_MASK_16_BIT (COUNTER_WRAP_16_BIT - 1)
#define COUNTER_MASK_12_BIT (COUNTER_WRAP_12_BIT - 1)

#define UPDATE_COUNTER(HostCnt,DevCnt) \
    HostCnt = HostCnt + DevCnt;



#ifndef	__APPLE__

/******************************************************************************
   Forward declaration of the private adapter structure
 *****************************************************************************/
struct et131x_adapter;


/******************************************************************************
   PROTOTYPES
 *****************************************************************************/
void ConfigMACRegs1( struct et131x_adapter *pAdapter );
void ConfigMACRegs2( struct et131x_adapter *pAdapter );
void ConfigRxMacRegs( struct et131x_adapter *pAdapter );
void ConfigTxMacRegs( struct et131x_adapter *pAdapter );
void ConfigMacStatRegs( struct et131x_adapter *pAdapter );
void ConfigFlowControl( struct et131x_adapter *pAdapter );
void UpdateMacStatHostCounters( struct et131x_adapter *pAdapter );
void HandleMacStatInterrupt( struct et131x_adapter *pAdapter );
void SetupDeviceForMulticast( struct et131x_adapter *pAdapter );
void SetupDeviceForUnicast( struct et131x_adapter *pAdapter );
#endif



#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _ET1310_MAC_H_ */
