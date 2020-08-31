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


typedef __uint8_t    UCHAR,  *PUCHAR;
typedef __uint16_t   U16,    *PU16;
typedef __uint8_t    UINT8,  *PUINT8;
typedef __uint32_t   U32,    *PU32;
typedef __uint32_t   UINT32, *PUINT32;
typedef __uint32_t   UINT,   *PUINT;
typedef __uint8_t    BYTE,   *PBYTE;
typedef __uint8_t    U8,     *PU8;
typedef __uint32_t   BOOL,   *PBOOL;
typedef __uint16_t   WORD,   *PWORD;
typedef __uint32_t   DWORD,  *PDWORD;
typedef unsigned long   ULONG,  *PULONG;

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (!(FALSE))
#endif

// Power status setting
enum {
	PCI_D0	= 0,
	PCI_D1,
	PCI_D2,
	PCI_D3hot,
	PCI_D3cold,
};

/*
 little/big endian converting
 */
#define cpu_to_le16(x)	OSSwapHostToLittleConstInt16(x)
#define cpu_to_le32(x)	OSSwapHostToLittleConstInt32(x)


/*
 delay function
 */
#define mdelay(x)	for(int i = 0; i < x; i++ )udelay(1000)

#define udelay(x)	IODelay(x)

#define	dma_addr_t	IOPhysicalAddress

#define	net_device	IONetworkDevice

struct pci_dev;


struct em_osdep
{
    struct net_device*          dev;
};

struct net_device_stats {
	int dummy;
};

typedef IOSimpleLock* spinlock_t;


/*
 */
#define VELOCITY_PRT(msglevel, l, p, args...) do {if (l<=msglevel) printk( p ,##args);} while (0)
#define VELOCITY_HW_PRT(hw, l, p, args...)  \
if(l<=(hw)->msglevel){ \
printk( "%s: ", "ViaVelocity");\
printk( p, ##args); \
}

static inline unsigned char inb (unsigned short port)
{
	unsigned char _v;
	
	__asm__ __volatile__ ("inb %w1,%0":"=a" (_v):"Nd" (port));
	return _v;
}

static inline unsigned int inl (unsigned short port)
{
	unsigned int _v;
	
	__asm__ __volatile__ ("inl %w1,%0":"=a" (_v):"Nd" (port));
	return _v;
}

static inline void outb (unsigned char value, unsigned short port)
{
	__asm__ __volatile__ ("outb %b0,%w1": :"a" (value), "Nd" (port));
}

static inline void outl (unsigned int value, unsigned short port)
{
	__asm__ __volatile__ ("outl %0,%w1": :"a" (value), "Nd" (port));
}

OS_INLINE
void OSWriteInt8(volatile void* base, uintptr_t byteOffset, uint8_t data)
{
    *(volatile uint8_t *)((uintptr_t)base + byteOffset) = data;
}

OS_INLINE
uint8_t OSReadInt8(const volatile void* base, uintptr_t byteOffset)
{
    return *(volatile uint8_t *)((uintptr_t)base + byteOffset);
}


#if	0
#define readb(addr) (*(volatile unsigned char *) __io_virt(addr))
#define readw(addr) (*(volatile unsigned short *) __io_virt(addr))
#define readl(addr) (*(volatile unsigned int *) __io_virt(addr))
#define writeb(b,addr) (*(volatile unsigned char *) __io_virt(addr) = (b))
#define writew(b,addr) (*(volatile unsigned short *) __io_virt(addr) = (b))
#define writel(b,addr) (*(volatile unsigned int *) __io_virt(addr) = (b))
#define __io_virt(x) ((void *)(x))
#endif

#define CSR_WRITE_4(hw, val, reg) OSWriteLittleInt32((hw)->hw_addr, reg, val)
#define CSR_WRITE_2(hw, val, reg) OSWriteLittleInt16((hw)->hw_addr, reg, val)
#define CSR_WRITE_1(hw, val, reg) OSWriteInt8((hw)->hw_addr, reg, val)


#define CSR_READ_4(hw, reg)   OSReadLittleInt32((hw)->hw_addr,reg)
#define CSR_READ_2(hw, reg)   OSReadLittleInt16((hw)->hw_addr,reg)
#define CSR_READ_1(hw, reg)   OSReadInt8((hw)->hw_addr,reg)


#define _INB(hw, reg)       inb((hw)->ioaddr+reg)
#define _OUTB(hw, val, reg) outb(val, (hw)->ioaddr+reg)

#define TX_QUEUE_NO         4

#define PKT_BUF_SZ          1540

#ifndef	__cplusplus
#define	IOBufferMemoryDescriptor	void
#define	IOPCIDevice	void
#endif

#endif

