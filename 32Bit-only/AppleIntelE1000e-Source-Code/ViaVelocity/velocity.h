/*
 * Copyright (c) 1996, 2003 VIA Networking Technologies, Inc.
 * All rights reserved.
 *
 * This software may be redistributed and/or modified under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 *
 * File: velocity.h
 *
 * Purpose: Header file to define driver's private structures.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */


#ifndef __VELOCITY_H__
#define __VELOCITY_H__

#ifdef	__APPLE__
#define	USE_RX_BUFFER	1

#else

#ifdef MODULE
#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif // MODVERSIONS
#include <linux/module.h>
#endif // MODULE

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <asm/io.h>
#include <linux/if.h>
#include <linux/utsname.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <linux/config.h>
#endif

#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/inetdevice.h>
#include <linux/reboot.h>

#ifdef SIOCETHTOOL
#define VELOCITY_ETHTOOL_IOCTL_SUPPORT
#include <linux/ethtool.h>
#else
#undef VELOCITY_ETHTOOL_IOCTL_SUPPORT
#endif

#ifdef SIOCGMIIPHY
#define VELOCITY_MII_IOCTL_SUPPORT
#include <linux/mii.h>
#else
#undef VELOCITY_MII_IOCTL_SUPPORT
#endif

//#ifdef MAX_SKB_FRAGS
//#define VELOCITY_ZERO_COPY_SUPPORT
//#else
//#undef  VELOCITY_ZERO_COPY_SUPPORT
//#endif

#undef  VELOCITY_ZERO_COPY_SUPPORT

#ifdef NETIF_F_IP_CSUM
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#define VELOCITY_TX_CSUM_SUPPORT
#else
#undef VELOCITY_TX_CSUM_SUPPORT
#endif

#if 0
#ifdef NETIF_F_TSO
#include <net/checksum.h>
#include <net/tcp.h>
#define VELOCITY_TSO_SUPPORT
#else
#undef VELOCITY_TSO_SUPPORT
#endif
#endif

#endif	// __APPLE__

#include "velocity_hw.h"

#include "kcompat.h"
#include "velocity_cfg.h"
#include "velocity_desc.h"
#include "velocity_mac.h"
#ifndef	__APPLE__
#include "velocity_proc.h"
#endif
#include "velocity_wol.h"
#include "velocity_mii.h"
#include "velocity_dbg.h"


// flags for driver status
#define VELOCITY_FLAGS_OPENED       0x00010000UL
#define VELOCITY_FLAGS_WOL_ENABLED  0x00080000UL

//
// Registers in the PCI configuration space
//
#define PCI_REG_COMMAND         0x04 //
#define PCI_REG_MODE0           0x60 //
#define PCI_REG_MODE1           0x61 //
#define PCI_REG_MODE2           0x62 //
#define PCI_REG_MODE3           0x63 //
#define PCI_REG_DELAY_TIMER     0x64 //

//
// Registers for power management (offset)
//
#define PCI_REG_PM_BASE     0x50

#define PM_CAP_ID           0x00 // 0x50
#define PM_NEXT_ITEM_PTR    0x01 // 0x51
#define PM_PMC0             0x02 // 0x52
#define PM_PMC1             0x03 // 0x53
#define PM_PMCSR0           0x04 // 0x54
#define PM_PMCSR1           0x05 // 0x55
#define PM_CSR_BSE          0x06 // 0x56
#define PM_DATA             0x07 // 0x57

// Bits in the (COMMAND, 0x04) register
#define COMMAND_BUSM        0x04
#define COMMAND_WAIT        0x80

//
// Bits in the (MODE0, 0x60) register
//
#define MODE0_QPKTDIS       0x80 // DISable transmit PacKeT Queuing

//
// Bits in the (MODE2, 0x62) register
//
#define MODE2_PCEROPT       0x80 // take PCI bus ERror as a fatal and shutdown from software control
#define MODE2_TXQ16         0x40 // TX write-back Queue control. 0->32 entries available in Tx write-back queue, 1->16 entries
#define MODE2_TXPOST        0x08 // (Not support in VT3119)
#define MODE2_AUTOOPT       0x04 // (VT3119 GHCI without such behavior)
#define MODE2_MODE10T       0x02 // used to control tx Threshold for 10M case
#define MODE2_TCPLSOPT      0x01 // TCP large send field update disable, hardware will not update related fields, leave it to software.

//
// Bits in the MODE3 register
//
#define MODE3_MIION         0x04 // MII symbol codine error detect enable ??

//
// Bits in PMCSR0 register
//
#define PMCSR0_PW_STAT1     0x02 //
#define PMCSR0_PW_STAT0     0x01 //

//
// Bits in PMCSR1 register
//
#define PMCSR1_PME_STATUS   0x80 //
#define PMCSR1_PME_EN       0x01 //


typedef enum __velocity_init_type {
    VELOCITY_INIT_COLD = 0, // 0
    VELOCITY_INIT_RESET,    // 1
    VELOCITY_INIT_WOL       // 2
} VELOCITY_INIT_TYPE, *PVELOCITY_INIT_TYPE;


typedef
struct _velocity_rd_info {
#ifdef	__APPLE__
	#if USE_RX_BUFFER
    IOBufferMemoryDescriptor* pool;
	#else
    mbuf_t     skb;
	#endif
#else
    struct sk_buff*     skb;
    dma_addr_t          skb_dma;
#endif
} VELOCITY_RD_INFO,    *PVELOCITY_RD_INFO;

typedef
struct _velocity_td_info {
#ifdef	__APPLE__
    mbuf_t     skb;
    IOBufferMemoryDescriptor* pool;
#else
    struct sk_buff*     skb;
#endif
    PU8                 buf;
#ifndef	__APPLE__
    int                 nskb_dma;
    dma_addr_t          skb_dma[7];
#endif
    dma_addr_t          buf_dma;
} VELOCITY_TD_INFO,    *PVELOCITY_TD_INFO;



typedef struct __velocity_info {
#ifdef	__APPLE__
    IOPCIDevice*         pcid;
#else
    struct __velocity_info*     next;
    struct __velocity_info*     prev;

    struct pci_dev*             pcid;
    struct net_device*          dev;
    struct net_device_stats     stats;
#endif

#if CONFIG_PM
    U32                         pci_state[16];
#endif
#ifdef	__APPLE__
    IOBufferMemoryDescriptor*   pool;
    IOBufferMemoryDescriptor*   tx_bufs[TX_QUEUE_NO];
#else
    void*                       pool;
    PU8                         tx_bufs;
#endif

    U8                          abyIPAddr[4];
    CHIP_TYPE                   chip_id;

    PVELOCITY_TD_INFO           apTDInfos[TX_QUEUE_NO];

    PVELOCITY_RD_INFO           aRDInfo; //it's an array
#ifndef	__APPLE__
    U32                         adwRMONStats[RMON_TAB_SIZE];
#endif

    /* define in velocity_hw.h */
    struct velocity_hw          hw;

#ifndef	__APPLE__
    spinlock_t                  lock;
    spinlock_t                  xmit_lock;
#endif

    int                         wol_opts;
#ifndef	__APPLE__
    U8                          wol_passwd[6];
#endif

    VELOCITY_CONTEXT            mac_context;
    U32                         pci_context;

#ifdef CONFIG_PROC_FS
    PVELOCITY_PROC_ENTRY        pProcDir;
#endif
#ifndef	__APPLE__
    U32                         ticks;
    U32                         rx_bytes;
    struct em_osdep             osdep;
#endif

} VELOCITY_INFO, *PVELOCITY_INFO;

inline static BOOL velocity_get_ip(PVELOCITY_INFO pInfo) {
#ifdef	__APPLE__
	bzero(pInfo->abyIPAddr,4);
    return TRUE;
#else
    struct in_device* in_dev = (struct in_device*)pInfo->dev->ip_ptr;
    struct in_ifaddr* ifa;

    if (in_dev!=NULL) {
        ifa=(struct in_ifaddr*) in_dev->ifa_list;
        if (ifa!=NULL) {
            memcpy(pInfo->abyIPAddr,&ifa->ifa_address,4);
            return TRUE;
        }
    }
    return FALSE;
#endif
}



#define PCI_BYTE_REG_BITS_ON(x,i,p) do{\
    U8 byReg;\
    pci_read_config_byte((p), (i), &(byReg));\
    (byReg) |= (x);\
    pci_write_config_byte((p), (i), (byReg));\
} while (0)

#define PCI_BYTE_REG_BITS_OFF(x,i,p) do{\
    U8 byReg;\
    pci_read_config_byte((p), (i), &(byReg));\
    (byReg) &= (~(x));\
    pci_write_config_byte((p), (i), (byReg));\
} while (0)

#define ADD_ONE_WITH_WRAP_AROUND(uVar, uModulo) {   \
    if ((uVar) >= ((uModulo) - 1))                  \
        (uVar) = 0;                                 \
    else                                            \
        (uVar)++;                                   \
}

#define SUB_ONE_WITH_WRAP_AROUND(uVar, uModulo) {   \
    if ((uVar) <= 0)                                \
        (uVar) = ((uModulo) - 1);                   \
    else                                            \
        (uVar)--;                                   \
}


#endif // __VELOCITY_H__
