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
 * et131x_defs.h - Defines, structs, enums, prototypes, etc. to assist with OS
 *                 compatibility
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
         $Date: 2005/10/28 18:43:46 $
     $Revision: 1.8 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef __ET131X_DEFS_H__
#define __ET131X_DEFS_H__




/******************************************************************************
   Define a boolean type
 *****************************************************************************/
#ifdef	__APPLE__
typedef u8 BOOL_t;
#else
typedef enum { FALSE, TRUE } __attribute__ ((packed)) BOOL_t;
#endif



/******************************************************************************
   Packet and header sizes
 *****************************************************************************/
#define NIC_MIN_PACKET_SIZE             60
#define NIC_HEADER_SIZE                 ETH_HLEN    //14




/******************************************************************************
   Multicast list size
 *****************************************************************************/
#define NIC_MAX_MCAST_LIST              128




/******************************************************************************
   Supported Filters
 *****************************************************************************/
#define ET131X_PACKET_TYPE_DIRECTED      0x0001
#define ET131X_PACKET_TYPE_MULTICAST     0x0002
#define ET131X_PACKET_TYPE_BROADCAST     0x0004
#define ET131X_PACKET_TYPE_PROMISCUOUS   0x0008
#define ET131X_PACKET_TYPE_ALL_MULTICAST 0x0010




/******************************************************************************
   Tx Timeout
 *****************************************************************************/
#define ET131X_TX_TIMEOUT       (1 * HZ)
#define NIC_SEND_HANG_THRESHOLD 0




/******************************************************************************
   MP_TCB flags
 *****************************************************************************/
#define fMP_DEST_MULTI                          0x00000001
#define fMP_DEST_BROAD                          0x00000002




/******************************************************************************
   MP_ADAPTER flags
 *****************************************************************************/
#define fMP_ADAPTER_RECV_LOOKASIDE              0x00000004
#define fMP_ADAPTER_INTERRUPT_IN_USE            0x00000008
#define fMP_ADAPTER_SECONDARY                   0x00000010




/******************************************************************************
   MP_SHARED flags
 *****************************************************************************/
#define fMP_ADAPTER_SHUTDOWN                    0x00100000
#define fMP_ADAPTER_LOWER_POWER                 0x00200000

#define fMP_ADAPTER_NON_RECOVER_ERROR           0x00800000
#define fMP_ADAPTER_RESET_IN_PROGRESS           0x01000000
#define fMP_ADAPTER_NO_CABLE                    0x02000000 
#define fMP_ADAPTER_HARDWARE_ERROR              0x04000000
#define fMP_ADAPTER_REMOVE_IN_PROGRESS          0x08000000
#define fMP_ADAPTER_HALT_IN_PROGRESS            0x10000000
#define fMP_ADAPTER_LINK_DETECTION              0x20000000
                                 
#define fMP_ADAPTER_FAIL_SEND_MASK              0x3ff00000                
#define fMP_ADAPTER_NOT_READY_MASK              0x3ff00000




/******************************************************************************
   Some offsets in PCI config space that are actually used.
 *****************************************************************************/
#define ET1310_PCI_PM_CAPABILITY    (UINT32)0x40
#define ET1310_PCI_PM_CSR           (UINT32)0x44
#define ET1310_PCI_MAX_PYLD         (UINT32)0x4C
#define ET1310_PCI_DEV_CTRL         (UINT32)0x50
#define ET1310_PCI_DEV_STAT         (UINT32)0x52
#define ET1310_NMI_DISABLE          (UINT32)0x61
#define ET1310_PCI_MAC_ADDRESS      (UINT32)0xA4
#define ET1310_PCI_EEPROM_STATUS    (UINT32)0xB2
#define ET1310_PCI_PHY_INDEX_REG    (UINT32)0xB4
#define ET1310_PCI_ACK_NACK         (UINT32)0xC0
#define ET1310_PCI_REPLAY           (UINT32)0xC2
#define ET1310_PCI_L0L1LATENCY      (UINT32)0xCF
#define ET1310_PCI_SEL_PHY_CTRL     (UINT32)0xE4
#define ET1310_PCI_ADVANCED_ERR     (UINT32)0x100




/******************************************************************************
   PCI Vendor/Product IDs
 *****************************************************************************/
#define ET131X_PCI_VENDOR_ID      0x11C1  // Agere Systems
#define ET131X_PCI_DEVICE_ID_GIG  0xED00  // ET1310 1000 Base-T
#define ET131X_PCI_DEVICE_ID_FAST 0xED01  // ET1310 100  Base-T




/******************************************************************************
   Define FIELD_OFFSET macro
 *****************************************************************************/
#define FIELD_OFFSET(type,field)    ((int)(&((type *)0)->field))




/******************************************************************************
   Handle name change of some regsiter bits
 *****************************************************************************/
#define phy_sw_coma     pm_phy_sw_coma




/******************************************************************************
   Define order of magnitude converter
 *****************************************************************************/
#define NANO_IN_A_MICRO 1000


/******************************************************************************
 Do not change these values: if changed, then change also in respective 
 TXdma and Rxdma engines
 *****************************************************************************/
#define NUM_DESC_PER_RING_TX         512    // TX Do not change these values
#define NUM_TCB                      64



#endif  /* __ET131X_DEFS_H__ */
