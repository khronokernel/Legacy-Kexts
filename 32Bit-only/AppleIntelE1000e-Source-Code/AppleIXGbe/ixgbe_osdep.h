/*******************************************************************************

  Intel 10 Gigabit PCI Express Linux driver
  Copyright(c) 1999 - 2013 Intel Corporation.

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
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


/* glue for the OS independent part of ixgbe
 * includes register access macros
 */

#ifndef _IXGBE_OSDEP_H_
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
#define _IXGBE_OSDEP_H_

#ifdef	__APPLE__
#else
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>
#include <linux/sched.h>
#endif
#include "kcompat.h"


#ifndef msleep
#define msleep(x)	do { if (in_interrupt()) { \
				/* Don't mdelay in interrupt context! */ \
				BUG(); \
			} else { \
				msleep(x); \
			} } while (0)

#endif

#undef ASSERT

#ifdef DBG
#define hw_dbg(hw, S, A...)	printk(KERN_DEBUG S, ## A)
#else
#define hw_dbg(hw, S, A...)	do {} while (0)
#endif

struct ixgbe_hw;
struct ixgbe_msg {
	u16 msg_enable;
};
struct net_device *ixgbe_hw_to_netdev(const struct ixgbe_hw *hw);
struct ixgbe_msg *ixgbe_hw_to_msg(const struct ixgbe_hw *hw);

#ifdef	__APPLE__
#define e_dev_info(format, arg...)	IOLog(format, ## arg)
#define e_dev_warn(format, arg...)	IOLog(format, ## arg)
#define e_dev_err(format, arg...)	IOLog(format, ## arg)
#define e_dev_notice(format, arg...)	IOLog(format, ## arg)
#define e_dbg(msglvl, format, arg...)	IOLog(format, ## arg)
#define e_info(msglvl, format, arg...)	IOLog(format, ## arg)
#define e_err(msglvl, format, arg...)	IOLog(format, ## arg)
#define e_warn(msglvl, format, arg...)	IOLog(format, ## arg)
#define e_crit(msglvl, format, arg...)	IOLog(format, ## arg)
#else /* __APPLE__ */
#define e_dev_info(format, arg...) \
	dev_info(pci_dev_to_dev(adapter->pdev), format, ## arg)
#define e_dev_warn(format, arg...) \
	dev_warn(pci_dev_to_dev(adapter->pdev), format, ## arg)
#define e_dev_err(format, arg...) \
	dev_err(pci_dev_to_dev(adapter->pdev), format, ## arg)
#define e_dev_notice(format, arg...) \
	dev_notice(pci_dev_to_dev(adapter->pdev), format, ## arg)
#define e_dbg(msglvl, format, arg...) \
	netif_dbg(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_info(msglvl, format, arg...) \
	netif_info(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_err(msglvl, format, arg...) \
	netif_err(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_warn(msglvl, format, arg...) \
	netif_warn(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_crit(msglvl, format, arg...) \
	netif_crit(adapter, msglvl, adapter->netdev, format, ## arg)
#endif /* __APPLE__ */


#ifdef DBG
#define IXGBE_WRITE_REG(a, reg, value) do {\
	switch (reg) { \
	case IXGBE_EIMS: \
	case IXGBE_EIMC: \
	case IXGBE_EIAM: \
	case IXGBE_EIAC: \
	case IXGBE_EICR: \
	case IXGBE_EICS: \
		printk("%s: Reg - 0x%05X, value - 0x%08X\n", __func__, \
		       reg, (u32)(value)); \
	default: \
		break; \
	} \
	writel((value), ((a)->hw_addr + (reg))); \
} while (0)
#else
#define IXGBE_WRITE_REG(a, reg, value) writel((value), ((a)->hw_addr + (reg)))
#endif

#define IXGBE_READ_REG(a, reg) readl((a)->hw_addr + (reg))

#define IXGBE_WRITE_REG_ARRAY(a, reg, offset, value) ( \
	writel((value), ((a)->hw_addr + (reg) + ((offset) << 2))))

#define IXGBE_READ_REG_ARRAY(a, reg, offset) ( \
	readl((a)->hw_addr + (reg) + ((offset) << 2)))

#ifndef writeq
#define writeq(val, addr)	do { writel((u32) (val), addr); \
				     writel((u32) (val >> 32), (addr + 4)); \
				} while (0);
#endif

#define IXGBE_WRITE_REG64(a, reg, value) writeq((value), ((a)->hw_addr + (reg)))

#define IXGBE_WRITE_FLUSH(a) IXGBE_READ_REG(a, IXGBE_STATUS)
extern u16 ixgbe_read_pci_cfg_word(struct ixgbe_hw *hw, u32 reg);
extern void ixgbe_write_pci_cfg_word(struct ixgbe_hw *hw, u32 reg, u16 value);
extern void ewarn(struct ixgbe_hw *hw, const char *str, u32 status);

#define IXGBE_READ_PCIE_WORD ixgbe_read_pci_cfg_word
#define IXGBE_WRITE_PCIE_WORD ixgbe_write_pci_cfg_word
#define IXGBE_EEPROM_GRANT_ATTEMPS 100
#define IXGBE_HTONL(_i) htonl(_i)
#define IXGBE_NTOHL(_i) ntohl(_i)
#define IXGBE_NTOHS(_i) ntohs(_i)
#define IXGBE_CPU_TO_LE32(_i) cpu_to_le32(_i)
#define IXGBE_LE32_TO_CPUS(_i) le32_to_cpus(_i)
#define EWARN(H, W, S) ewarn(H, W, S)

enum {
	IXGBE_ERROR_SOFTWARE,
	IXGBE_ERROR_POLLING,
	IXGBE_ERROR_INVALID_STATE,
	IXGBE_ERROR_UNSUPPORTED,
	IXGBE_ERROR_ARGUMENT,
	IXGBE_ERROR_CAUTION,
};

#ifdef	__APPLE__
#define ERROR_REPORT(level, format, arg...) do {			\
	switch (level) {						\
	case IXGBE_ERROR_SOFTWARE:					\
	case IXGBE_ERROR_CAUTION:					\
	case IXGBE_ERROR_POLLING:					\
		IOLog(format, ## arg);				\
		break;							\
	case IXGBE_ERROR_INVALID_STATE:					\
	case IXGBE_ERROR_UNSUPPORTED:					\
	case IXGBE_ERROR_ARGUMENT:					\
		IOLog(format, ## arg);				\
		break;							\
	default:							\
	break;							\
	}								\
} while (0)
#else	/* __APPLE__ */
#define ERROR_REPORT(level, format, arg...) do {			\
	switch (level) {						\
	case IXGBE_ERROR_SOFTWARE:					\
	case IXGBE_ERROR_CAUTION:					\
	case IXGBE_ERROR_POLLING:					\
		netif_dbg(ixgbe_hw_to_msg(hw), drv, ixgbe_hw_to_netdev(hw), \
			  format, ## arg);				\
		break;							\
	case IXGBE_ERROR_INVALID_STATE:					\
	case IXGBE_ERROR_UNSUPPORTED:					\
	case IXGBE_ERROR_ARGUMENT:					\
		netif_err(ixgbe_hw_to_msg(hw), hw, ixgbe_hw_to_netdev(hw), \
			  format, ## arg);				\
		break;							\
	default:							\
		break;							\
	}								\
} while (0)
#endif	/* __APPLE__ */

#define ERROR_REPORT1 ERROR_REPORT
#define ERROR_REPORT2 ERROR_REPORT
#define ERROR_REPORT3 ERROR_REPORT
#endif /* _IXGBE_OSDEP_H_ */
