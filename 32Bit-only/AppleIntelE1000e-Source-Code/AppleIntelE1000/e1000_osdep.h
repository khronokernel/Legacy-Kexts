/*******************************************************************************

  Intel PRO/1000 Linux driver
  Copyright(c) 1999 - 2010 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  Linux NICS <linux.nics@intel.com>
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


/* glue for the OS-dependent part of e1000
 * includes register access macros
 */

#ifndef _E1000_OSDEP_H_
#define _E1000_OSDEP_H_

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
#else
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>
#endif

#include "kcompat.h"

#ifdef	__APPLE__
#define usec_delay(x)	IODelay(x)
#define msec_delay(x)	IOSleep(x)
#define msec_delay_irq(x) mdelay(x)
#else
#define usec_delay(x) udelay(x)
#ifndef msec_delay
#define msec_delay(x)	do { if(in_interrupt()) { \
				/* Don't sleep in interrupt context! */ \
	                	BUG(); \
			} else { \
				msleep(x); \
			} } while (0)
#endif

/* Some workarounds require millisecond delays and are run during interrupt
 * context.  Most notably, when establishing link, the phy may need tweaking
 * but cannot process phy register reads/writes faster than millisecond
 * intervals...and we establish link due to a "link status change" interrupt.
 */
#define msec_delay_irq(x) mdelay(x)
#endif

#define PCI_COMMAND_REGISTER   PCI_COMMAND
#define CMD_MEM_WRT_INVALIDATE PCI_COMMAND_INVALIDATE
#define ETH_ADDR_LEN           ETH_ALEN

#ifdef __BIG_ENDIAN
#define E1000_BIG_ENDIAN __BIG_ENDIAN
#endif


#ifdef	DEBUG
#define DEBUGOUT(S)
#define DEBUGOUT1(S, A...)
#else
#define DEBUGOUT(S)
#define DEBUGOUT1(S, A...)
#endif

#define DEBUGFUNC(F) DEBUGOUT(F "\n")
#define DEBUGOUT2 DEBUGOUT1
#define DEBUGOUT3 DEBUGOUT2
#define DEBUGOUT7 DEBUGOUT3

#define E1000_REGISTER(a, reg) (((a)->mac.type >= e1000_82543) \
                               ? reg                           \
                               : e1000_translate_register_82542(reg))

static inline u32 er32(u8 __iomem *reg)
{
	return readl(reg);
}

static inline void ew32(u8 __iomem *reg, u32 val)
{
	writel(val, reg);
}

#define E1000_WRITE_REG(a, reg, value) \
    ew32(((a)->hw_addr + E1000_REGISTER(a, reg)),value)

#define E1000_READ_REG(a, reg) (er32((a)->hw_addr + E1000_REGISTER(a, reg)))

#define E1000_WRITE_REG_ARRAY(a, reg, offset, value) ( \
    writel((value), ((a)->hw_addr + E1000_REGISTER(a, reg) + ((offset) << 2))))

#define E1000_READ_REG_ARRAY(a, reg, offset) ( \
    readl((a)->hw_addr + E1000_REGISTER(a, reg) + ((offset) << 2)))

#define E1000_READ_REG_ARRAY_DWORD E1000_READ_REG_ARRAY
#define E1000_WRITE_REG_ARRAY_DWORD E1000_WRITE_REG_ARRAY

#define E1000_WRITE_REG_ARRAY_WORD(a, reg, offset, value) ( \
    writew((value), ((a)->hw_addr + E1000_REGISTER(a, reg) + ((offset) << 1))))

#define E1000_READ_REG_ARRAY_WORD(a, reg, offset) ( \
    readw((a)->hw_addr + E1000_REGISTER(a, reg) + ((offset) << 1)))

#define E1000_WRITE_REG_ARRAY_BYTE(a, reg, offset, value) ( \
    writeb((value), ((a)->hw_addr + E1000_REGISTER(a, reg) + (offset))))

#define E1000_READ_REG_ARRAY_BYTE(a, reg, offset) ( \
    readb((a)->hw_addr + E1000_REGISTER(a, reg) + (offset)))

#ifdef	__APPLE__
#define E1000_WRITE_REG_IO(a, reg, offset)	e1000_writeRegIO((a),reg,offset)

#define E1000_WRITE_FLUSH(a) E1000_READ_REG(a, E1000_STATUS)

#else	/* __APPLE__ */
extern void e1000_writeRegIO(e1000_hw* hw, u32 reg, u32 offset);
#define E1000_WRITE_REG_IO(a, reg, offset) do { \
    outl(reg, ((a)->io_base));                  \
    outl(offset, ((a)->io_base + 4));      } while(0)

#define E1000_WRITE_FLUSH(a) E1000_READ_REG(a, E1000_STATUS)

#define E1000_WRITE_FLASH_REG(a, reg, value) ( \
    writel((value), ((a)->flash_address + reg)))

#define E1000_WRITE_FLASH_REG16(a, reg, value) ( \
    writew((value), ((a)->flash_address + reg)))

#define E1000_READ_FLASH_REG(a, reg) (readl((a)->flash_address + reg))

#define E1000_READ_FLASH_REG16(a, reg) (readw((a)->flash_address + reg))
#endif

#endif /* _E1000_OSDEP_H_ */
