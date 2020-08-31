/*
 * Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * The contents of this file constitute Original Code as defined in and
 * are subject to the Apple Public Source License Version 1.1 (the
 * "License").  You may not use this file except in compliance with the
 * License.  Please obtain a copy of the License at
 * http://www.apple.com/publicsource and read it before using this file.
 * 
 * This Original Code and all software distributed under the License are
 * distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
 * License for the specific language governing rights and limitations
 * under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */

#ifndef _APPLEPCIIDEHARDWARE_H
#define _APPLEPCIIDEHARDWARE_H

#define kMaxDriveCount      2
#define kMaxChannelCount    2

/*
 * I/O port addresses for primary and secondary channels.
 */
#define PRI_CMD_ADDR        0x1f0
#define PRI_CTR_ADDR        0x3f4
#define SEC_CMD_ADDR        0x170
#define SEC_CTR_ADDR        0x374

/*
 * IRQ assigned to primary and secondary channels.
 */
#define PRI_ISA_IRQ         14
#define SEC_ISA_IRQ         15

/*
 * Two ATA channels max.
 */
#define PRI_CHANNEL_ID      0
#define SEC_CHANNEL_ID      1

/*
 * PCI ATA config space registers.
 * Register size (bits) in parenthesis.
 */
#define PCI_CFID            0x00    // (32) PCI Device/Vendor ID
#define PCI_PCICMD          0x04    // (16) PCI command register
#define PCI_PCISTS          0x06    // (16) PCI device status register
#define PCI_RID             0x08    // (8)  Revision ID register
#define PCI_PI              0x09    // (8)  Programming interface
#define PCI_MLT             0x0d    // (8)  Master latency timer register
#define PCI_HEDT            0x0e    // (8)  Header type register
#define PCI_BMIBA           0x20    // (32) Bus-Master base address

#define PCI_BMIBA_RTE       0x01    // resource type indicator (I/O)
#define PCI_BMIBA_MASK      0xfff0  // base address mask

#define PCI_PCICMD_IOSE     0x01    // I/O space enable
#define PCI_PCICMD_BME      0x04    // bus-master enable

/*
 * VIA specific PCI config space registers.
 */
#define VIA_IDE_ENABLE      0x40
#define VIA_IDE_CONFIG      0x41
#define VIA_SATA_NATIVE     0x42
#define VIA_FIFO_CONFIG     0x43
#define VIA_MISC_1          0x44
#define VIA_MISC_2          0x45
#define VIA_MISC_3          0x46
#define VIA_DATA_TIMING     0x48
#define VIA_CMD_TIMING      0x4e
#define VIA_ADDRESS_SETUP   0x4c
#define VIA_ULTRA_TIMING    0x50
#define	VIA_PATA_UDMA_TIMING	0xB3 /* PATA timing for DMA/ cable detect */
#define	VIA_PATA_PIO_TIMING	0xAB /* PATA timing register */

/*
 * Supported devices (ATA controller and PCI-ISA bridges).
 */
#define PCI_VIA_ID          0x1106
#define PCI_SIL_ID          0x1095
#define PCI_ITE_ID          0x1283

#define	PCI_VIA_6421        0x3249

#define PCI_SIL_0680        0x0680
#define PCI_SIL_3112        0x3112
#define PCI_SIL_3114        0x3114
#define PCI_SIL_3512        0x3512

#define	SII3112_PCI_CFGCTL	0x40
#define	CFGCTL_CFGWREN		(1U << 0)	/* enable cfg writes */
#define	CFGCTL_BA5INDEN		(1U << 1)	/* BA5 indirect access enable */

#define	SII3112_PCI_SWDATA	0x44

#define	SII3112_PCI_BM_IDE0	0x70
/* == BAR4+0x00 */

#define	SII3112_PCI_PRD_IDE0	0x74
/* == BAR4+0x04 */

#define	SII3112_PCI_BM_IDE1	0x78
/* == BAR4+0x08 */

#define	SII3112_PCI_PRD_IDE1	0x7c
/* == BAR4+0x0c */

#define	SII3112_DTM_IDE0	0x80	/* Data Transfer Mode - IDE0 */
#define	SII3112_DTM_IDE1	0x84	/* Data Transfer Mode - IDE1 */
#define	DTM_IDEx_PIO		0x00000000	/* PCI DMA, IDE PIO (or 1) */
#define	DTM_IDEx_DMA		0x00000002	/* PCI DMA, IDE DMA (or 3) */


#define	SII3112_SCS_CMD		0x88	/* System Config Status */
#define	SCS_CMD_PBM_RESET	(1U << 0)	/* PBM module reset */
#define	SCS_CMD_ARB_RESET	(1U << 1)	/* ARB module reset */
#define	SCS_CMD_FF1_RESET	(1U << 4)	/* IDE1 FIFO reset */
#define	SCS_CMD_FF0_RESET	(1U << 5)	/* IDE0 FIFO reset */
#define	SCS_CMD_IDE1_RESET	(1U << 6)	/* IDE1 module reset */
#define	SCS_CMD_IDE0_RESET	(1U << 7)	/* IDE0 module reset */
#define	SCS_CMD_FF3_RESET	(1U << 8)	/* IDE3 FIFO reset (3114) */
#define	SCS_CMD_FF2_RESET	(1U << 9)	/* IDE2 FIFO reset (3114) */
#define	SCS_CMD_IDE3_RESET	(1U << 10)	/* IDE3 module reset (3114) */
#define	SCS_CMD_IDE2_RESET	(1U << 11)	/* IDE2 module reset (3114) */
#define	SCS_CMD_BA5_EN		(1U << 16)	/* BA5 is enabled (3112) */
#define	SCS_CMD_M66EN		(1U << 16)	/* 1=66MHz, 0=33MHz (3114) */
#define	SCS_CMD_IDE0_INT_BLOCK	(1U << 22)	/* IDE0 interrupt block */
#define	SCS_CMD_IDE1_INT_BLOCK	(1U << 23)	/* IDE1 interrupt block */
#define	SCS_CMD_IDE2_INT_BLOCK	(1U << 24)	/* IDE2 interrupt block */
#define	SCS_CMD_IDE3_INT_BLOCK	(1U << 25)	/* IDE3 interrupt block */

#define	SII3112_RESET_BITS	(SCS_CMD_PBM_RESET | SCS_CMD_ARB_RESET | SCS_CMD_FF1_RESET | SCS_CMD_FF0_RESET | SCS_CMD_IDE1_RESET | SCS_CMD_IDE0_RESET)

#define	SII3112_SSDR		0x8c	/* System SW Data Register */

#define	SII3112_FMA_CSR		0x90	/* Flash Memory Addr - CSR */

#define	SII3112_FM_DATA		0x94	/* Flash Memory Data */

#define	SII3112_EEA_CSR		0x98	/* EEPROM Memory Addr - CSR */

#define	SII3112_EE_DATA		0x9c	/* EEPROM Data */

#define	SII3112_TCS_IDE0	0xa0	/* IDEx config, status */
#define	SII3112_TCS_IDE1	0xb0
#define	TCS_IDEx_BCA		(1U << 1)	/* buffered command active */
#define	TCS_IDEx_CH_RESET	(1U << 2)	/* channel reset */
#define	TCS_IDEx_VDMA_INT	(1U << 10)	/* virtual DMA interrupt */
#define	TCS_IDEx_INT		(1U << 11)	/* interrupt status */
#define	TCS_IDEx_WTT		(1U << 12)	/* watchdog timer timeout */
#define	TCS_IDEx_WTEN		(1U << 13)	/* watchdog timer enable */
#define	TCS_IDEx_WTINTEN	(1U << 14)	/* watchdog timer int. enable */

#define	SII3112_BA5_IND_ADDR	0xc0	/* BA5 indirect address */

#define	SII3112_BA5_IND_DATA	0xc4	/* BA5 indirect data */

#define	IDEDMA_CMD_INT_STEER	(1U << 1)

#define PCI_ITE_8212        0x8212

#define IT_CFG			0x40	/* I/O configuration */
#define IT_CFG_MASK			0x0000ffff
#define IT_CFG_IORDY(chan)		(0x0001 << (chan))
#define IT_CFG_BLID(chan)		(0x0004 << (chan))
#define IT_CFG_CABLE(chan, drive)	(0x0010 << ((chan) * 2 + (drive)))
#define IT_CFG_DECODE(chan)		(0x8000 >> ((chan) * 2))

#define IT_MODE			0x50	/* mode control / RAID function */
#define IT_MODE_MASK			0x0000ffff
#define IT_MODE_CPU			0x0001
#define IT_MODE_50MHZ(chan)		(0x0002 << (chan))
#define IT_MODE_DMA(chan, drive)	(0x0008 << ((chan) * 2 + (drive)))
#define IT_MODE_RESET			0x0080
#define IT_MODE_RAID1			0x0100

#define IT_TIM(chan)		((chan) ? 0x58 : 0x54) /* timings */
#define IT_TIM_UDMA(chan,unit)		(((chan) ? 0x5a : 0x56)+unit) /* timings */
#define IT_TIM_UDMA5(drive)		(0x00800000 << (drive) * 8)

#define IDEDMA_CTL_DRV_DMA(d)	(0x20 << (d))
/*
 * Bus master registers are located in I/O space.
 * Register size (bits) indicated in parenthesis.
 *
 * Note:
 * For the primary channel, the base address is stored in PCI_BMIBA.
 * For the secondary channel, an offset (BM_SEC_OFFSET) is added to
 * the value stored in PCI_BMIBA.
 */
#define BM_COMMAND          0x00    // (8) Bus master command register
#define BM_STATUS           0x02    // (8) Bus master status register
#define BM_PRD_TABLE        0x04    // (32) Descriptor table register

#define BM_SEC_OFFSET       0x08    // offset to channel 1 registers
#define BM_ADDR_MASK        0xfff0  // BMIBA mask to get I/O base address
#define BM_STATUS_INT       0x04    // IDE device asserted its interrupt

/*
 * Property keys
 */
#define kChannelNumberKey         "Channel Number"
#define kCommandBlockAddressKey   "Command Block Address"
#define kControlBlockAddressKey   "Control Block Address"
#define kInterruptVectorKey       "Interrupt Vector"
#define kHardwareNameKey          "Controller Name"
#define kSelectedPIOModeKey       "PIO Mode"
#define kSelectedDMAModeKey       "DMA Mode"
#define kSelectedUltraDMAModeKey  "Ultra DMA Mode"

#define kPIOModeCount    5   /* PIO mode 0 to 4 */
#define kDMAModeCount    3   /* DMA mode 0 to 2 */
#define kUDMAModeCount   7   /* Ultra mode 0 to 6 */

#endif /* !_APPLEPCIIDEHARDWARE_H */
