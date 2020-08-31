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
 * ET1310_jagcore.h - Defines, structs, enums, prototypes, etc. pertaining to
 *                    the JAGCore
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
         $Date: 2005/10/28 18:43:44 $
     $Revision: 1.7 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef __ET1310_JAGCORE_H__
#define __ET1310_JAGCORE_H__

#ifdef __cplusplus
extern "C" {
#endif




/******************************************************************************
   INCLUDES
 *****************************************************************************/
#include "ET1310_common.h"




/******************************************************************************
   CONSTANTS FOR JAGCORE
 *****************************************************************************/
#define INTERNAL_MEM_SIZE       0x400  //1024 of internal memory
#define INTERNAL_MEM_RX_OFFSET  0x1FF  //50%   Tx, 50%   Rx

#define REGS_MAX_ARRAY          4096




/******************************************************************************
    For interrupts, normal running is:
        rxdma_xfr_done, phy_interrupt, mac_stat_interrupt, 
        watchdog_interrupt & txdma_xfer_done

    In both cases, when flow control is enabled for either Tx or bi-direction,
    we additional enable rx_fbr0_low and rx_fbr1_low, so we know when the 
    buffer rings are running low.
 *****************************************************************************/
#define INT_MASK_DISABLE            0xffffffff

// NOTE: Masking out MAC_STAT Interrupt for now...
//#define INT_MASK_ENABLE             0xfff6bf17
//#define INT_MASK_ENABLE_NO_FLOW     0xfff6bfd7
#define INT_MASK_ENABLE             0xfffebf17
#define INT_MASK_ENABLE_NO_FLOW     0xfffebfd7




/******************************************************************************
   DATA STRUCTURES FOR DIRECT REGISTER ACCESS
 *****************************************************************************/
typedef struct
{
    u8  bReadWrite;
    u32 nRegCount;
    u32 nData [REGS_MAX_ARRAY];
    u32 nOffsets [REGS_MAX_ARRAY];
} JAGCORE_ACCESS_REGS, *PJAGCORE_ACCESS_REGS;


typedef struct
{
    u8  bReadWrite;                            
    u32 nDataWidth;                            
    u32 nRegCount;                            
    u32 nOffsets [REGS_MAX_ARRAY];        
    u32 nData [REGS_MAX_ARRAY];            
} PCI_CFG_SPACE_REGS, *PPCI_CFG_SPACE_REGS;



#ifndef	__APPLE__

/******************************************************************************
   Forward declaration of the private adapter structure
 *****************************************************************************/
struct et131x_adapter;




/******************************************************************************
   PROTOTYPES
 *****************************************************************************/
void ConfigGlobalRegs(struct et131x_adapter *pAdapter );

void ConfigMMCRegs( struct et131x_adapter *pAdapter );

void et131x_enable_interrupts( struct et131x_adapter *adapter );

void et131x_disable_interrupts( struct et131x_adapter *adapter );

void * et131x_get_regs( struct et131x_adapter *pAdapter, void *InfoBuf,
                        PUINT32 ulBytesAvailable, PUINT32 ulInfoLen );

void et131x_set_regs( struct et131x_adapter *pAdapter, void *InfoBuf );
#endif



#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ET1310_JAGCORE_H__ */
