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
 * et131x_version.h - This file provides system and device version information.
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
         $Date: 2006/01/31 20:58:43 $
     $Revision: 1.16 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




#ifndef __ET131X_VERSION_H__
#define __ET131X_VERSION_H__




/******************************************************************************
   Includes
 *****************************************************************************/

#ifdef	__APPLE__

#include <AvailabilityMacros.h>
#include <sys/types.h>
#include <libkern/OSByteOrder.h>
#include <sys/kpi_mbuf.h>
#include <IOKit/IOLib.h>
#include <libkern/OSAtomic.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/network/IOPacketQueue.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/assert.h>

#include "kcompat.h"

#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
//#include <linux/if_vlan.h>
#endif

#endif




/******************************************************************************
   Constant Definitions
 *****************************************************************************/
#define DRIVER_AUTHOR           "Victor Soriano (vjsoriano@agere.com)"
#define DRIVER_LICENSE          "BSD"
#define DRIVER_DEVICE_STRING    "ET1310"
#define DRIVER_NAME             "et131x"
#define DRIVER_MAJOR_VERSION    1
#define DRIVER_MINOR_VERSION    2
#define DRIVER_PATCH_VERSION    3
#define DRIVER_VERSION_STRING   "1.2.3"
#define DRIVER_VENDOR           "Agere Systems, http://www.agere.com"
#define DRIVER_BUILD_DATE       "01/31/2006 15:40:00"
#define DRIVER_DESC             "10/100/1000 Base-T Ethernet Driver"

#define STRUCT_MODULE           "net" // blux: missed by the kernel

#define DRIVER_INFO             DRIVER_DESC " for the "\
                                DRIVER_DEVICE_STRING ", v" \
                                DRIVER_VERSION_STRING " " \
                                DRIVER_BUILD_DATE " by " \
                                DRIVER_VENDOR


#define DRIVER_NAME_EXT         "et131x.ko"




#endif  /* __ET131X_VERSION_H__ */
