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
 * ET1310_eeprom.h - Defines, structs, enums, prototypes, etc. used for EEPROM
 *                   access routines
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
         $Date: 2005/08/01 19:35:10 $
     $Revision: 1.4 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef __ET1310_EEPROM_H__
#define __ET1310_EEPROM_H__




/******************************************************************************
   INCLUDES
 *****************************************************************************/
#include "ET1310_common.h"


#ifndef SUCCESS
    #define SUCCESS         0
    #define FAILURE         1
#endif

#ifndef READ
    #define READ            0
    #define WRITE           1
#endif

#ifndef SINGLE_BYTE
    #define SINGLE_BYTE     0
    #define DUAL_BYTE       1
#endif



#ifndef	__APPLE__

/******************************************************************************
   Forward declaration of the private adapter structure
 *****************************************************************************/
struct et131x_adapter;


UINT32 EEPROM_access( struct et131x_adapter *pAdapter, UCHAR bAccessFlag, 
                      UINT32 unOffset, UCHAR bWidth, void *pData );

INT32  EepromWriteByte( struct et131x_adapter *pAdapter, UINT32 unAddress,
                        UCHAR bData, UINT32 unEepromId, UINT32 unAddressingMode );

INT32  EepromReadByte( struct et131x_adapter *pAdapter, UINT32 unAddress, 
                       PUCHAR pbData, UINT32 unEepromId, UINT32 unAddressingMode );

#endif


#endif /* _ET1310_EEPROM_H_ */
