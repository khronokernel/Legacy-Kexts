/*
 * Copyright (c) 1996, 2003 VIA Networking Technologies, Inc.
 * All rights reserved.
 *
 * This software is copyrighted by and is the sole property of
 * VIA Networking Technologies, Inc. This software may only be used
 * in accordance with the corresponding license agreement. Any unauthorized
 * use, duplication, transmission, distribution, or disclosure of this
 * software is expressly forbidden.
 *
 * This software is provided by VIA Networking Technologies, Inc. "as is"
 * and any express or implied warranties, including, but not limited to, the
 * implied warranties of merchantability and fitness for a particular purpose
 * are disclaimed. In no event shall VIA Networking Technologies, Inc.
 * be liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages.
 *
 *
 * File: osdep.h
 *
 * Purpose: OS depended function and macro defined, including register accessing.
 *
 * Author: Guard Kuo
 *
 * Date: Jan 28, 2005
 *
 *
 */
#ifndef ___OSDEP_H__
#define ___OSDEP_H__

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/netdevice.h>
#include <linux/version.h>

typedef __u8    UCHAR,  *PUCHAR;
typedef __u16   U16,    *PU16;
typedef __u8    UINT8,  *PUINT8;
typedef __u32   U32,    *PU32;
typedef __u32   UINT32, *PUINT32;
typedef __u32   UINT,   *PUINT;
typedef __u8    BYTE,   *PBYTE;
typedef __u8    U8,     *PU8;
typedef __u32   BOOL,   *PBOOL;
typedef __u16   WORD,   *PWORD;
typedef __u32   DWORD,  *PDWORD;
typedef unsigned long   ULONG,  *PULONG;

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (!(FALSE))
#endif

// Power status setting
#ifndef PCI_D0
#define PCI_D0	0
#endif
#ifndef PCI_D1
#define PCI_D1	1
#endif
#ifndef PCI_D2
#define PCI_D2	2
#endif
#ifndef PCI_D3hot
#define PCI_D3hot	3
#endif
#ifndef PCI_D3cold
#define PCI_D3cold	4
#endif

/*
   little/big endian converting
*/
#if 0
#define cpu_to_le32(x) x
#define cpu_to_le16(x) x
#endif


/*
   delay function
*/
#if 0
#ifndef mdelay(x)
#define mdelay(x) x
#endif

#ifndef udelay(x)
#define udelay(x) x
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,18)
typedef unsigned long dma_addr_t;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
#define net_device		device
#endif

struct em_osdep
{
    struct net_device*          dev;
};


/*
 */
#define VELOCITY_PRT(msglevel, l, p, args...) do {if (l<=msglevel) printk( p ,##args);} while (0)
#define VELOCITY_HW_PRT(hw, l, p, args...)  \
        if(l<=(hw)->msglevel){ \
            printk( "%s: ", (hw)->back->dev->name);\
            printk( p, ##args); \
        }


//#define USE_PIO
#undef USE_PIO

#ifdef USE_PIO
#define CSR_WRITE_4(hw, val, reg) writel(val, (hw)->ioaddr+reg)
#define CSR_WRITE_2(hw, val, reg) writew(val, (hw)->ioaddr+reg)
#define CSR_WRITE_1(hw, val, reg) writeb(val, (hw)->ioaddr+reg)

#define CSR_READ_4(hw, reg)   readl((hw)->ioaddr+reg)
#define CSR_READ_2(hw, reg)   readw((hw)->ioaddr+reg)
#define CSR_READ_1(hw, reg)   readb((hw)->ioaddr+reg)

#else
/*
 * register space access macros
 */
#define CSR_WRITE_4(hw, val, reg) writel(val, (hw)->hw_addr+reg)
#define CSR_WRITE_2(hw, val, reg) writew(val, (hw)->hw_addr+reg)
#define CSR_WRITE_1(hw, val, reg) writeb(val, (hw)->hw_addr+reg)

#define CSR_READ_4(hw, reg)   readl((hw)->hw_addr+reg)
#define CSR_READ_2(hw, reg)   readw((hw)->hw_addr+reg)
#define CSR_READ_1(hw, reg)   readb((hw)->hw_addr+reg)
#endif

#define _INB(hw, reg)       inb((hw)->ioaddr+reg)
#define _OUTB(hw, val, reg) outb(val, (hw)->ioaddr+reg)

#define TX_QUEUE_NO         4

#define MALLOC(x,y)         kmalloc((x),(y))

#define PKT_BUF_SZ          1540

#endif

