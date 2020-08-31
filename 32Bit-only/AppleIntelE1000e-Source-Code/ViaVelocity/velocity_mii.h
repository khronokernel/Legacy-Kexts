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
 * File: velocity_mii.h
 *
 * Purpose: Header file for MII registers.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */


#ifndef __VELOCITY_MII_H__
#define __VELOCITY_MII_H__

#include "osdep.h"

/*
// Registers in the MII (offset unit is WORD)
*/
#define MII_REG_BMCR        0x00        /* physical address */
#define MII_REG_BMSR        0x01        
#define MII_REG_PHYID1      0x02        /* OUI */
#define MII_REG_PHYID2      0x03        /* OUI + Module ID + REV ID */
#define MII_REG_ANAR        0x04
#define MII_REG_ANLPAR      0x05
#define MII_REG_G1000CR     0x09
#define MII_REG_G1000SR     0x0A
#define MII_REG_MODCFG      0x10
#define MII_REG_TCSR        0x16
#define MII_REG_PLED        0x1B
/* NS, MYSON only */
#define MII_REG_PCR         0x17
/* ESI only */
#define MII_REG_PCSR        0x17
#define MII_REG_AUXCR       0x1C

/* Marvell 88E1000/88E1000S */
#define MII_REG_PSCR        0x10        /* PHY specific control register */

/*
// Bits in the BMCR register
*/
#define BMCR_RESET          0x8000
#define BMCR_LBK            0x4000
#define BMCR_SPEED100       0x2000
#define BMCR_AUTO           0x1000
#define BMCR_PD             0x0800
#define BMCR_ISO            0x0400
#define BMCR_REAUTO         0x0200
#define BMCR_FDX            0x0100
#define BMCR_SPEED1G        0x0040
/*
// Bits in the BMSR register
*/
#define BMSR_AUTOCM         0x0020
#define BMSR_LNK            0x0004

/*
// Bits in the ANAR register
*/
#define ANAR_ASMDIR         0x0800      /* Asymmetric PAUSE support */
#define ANAR_PAUSE          0x0400      /* Symmetric PAUSE Support */
#define ANAR_T4             0x0200
#define ANAR_TXFD           0x0100
#define ANAR_TX             0x0080
#define ANAR_10FD           0x0040
#define ANAR_10             0x0020
/*
// Bits in the ANLPAR register
*/
#define ANLPAR_ASMDIR       0x0800      /* Asymmetric PAUSE support */
#define ANLPAR_PAUSE        0x0400      /* Symmetric PAUSE Support */
#define ANLPAR_T4           0x0200
#define ANLPAR_TXFD         0x0100
#define ANLPAR_TX           0x0080
#define ANLPAR_10FD         0x0040
#define ANLPAR_10           0x0020

/*
// Bits in the G1000CR register
*/
#define G1000CR_1000FD      0x0200      /* PHY is 1000-T Full-duplex capable */
#define G1000CR_1000        0x0100      /* PHY is 1000-T Half-duplex capable */

/*
// Bits in the G1000SR register
*/
#define G1000SR_1000FD      0x0800      /* LP PHY is 1000-T Full-duplex capable */
#define G1000SR_1000        0x0400      /* LP PHY is 1000-T Half-duplex capable */

#define TCSR_ECHODIS        0x2000		
#define AUXCR_MDPPS         0x0004		

/* Bits in the PLED register */
#define PLED_LALBE			0x0004		

/* Marvell 88E1000/88E1000S Bits in the PHY specific control register (10h) */
#define PSCR_ACRSTX         0x0800  /* Assert CRS on Transmit */

#define PHYID_CICADA_CS8201 0x000FC410UL
#define PHYID_VT3216_32BIT  0x000FC610UL
#define PHYID_VT3216_64BIT  0x000FC600UL
#define PHYID_MARVELL_1000  0x01410C50UL
#define PHYID_MARVELL_1000S 0x01410C40UL

#define PHYID_REV_ID_MASK   0x0000000FUL

#define PHYID_GET_PHY_REV_ID(i)     ((i) & PHYID_REV_ID_MASK)
#define PHYID_GET_PHY_ID(i)         ((i) & ~PHYID_REV_ID_MASK)

#define MII_REG_BITS_ON(x,i,hw) do {\
    U16 w;\
    velocity_mii_read((hw),(i),&(w));\
    (w)|=(x);\
    velocity_mii_write((hw),(i),(w));\
} while (0)

#define MII_REG_BITS_OFF(x,i,hw) do {\
    U16 w;\
    velocity_mii_read((hw),(i),&(w));\
    (w)&=(~(x));\
    velocity_mii_write((hw),(i),(w));\
} while (0)

#define MII_REG_BITS_IS_ON(x,i,hw) ({\
    U16 w;\
    velocity_mii_read((hw),(i),&(w));\
    ((BOOL) ((w) & (x)));})

#define MII_GET_PHY_ID(hw) ({\
    U32 id;\
    velocity_mii_read(hw, MII_REG_PHYID2, (PU16)&id);\
    velocity_mii_read(hw, MII_REG_PHYID1, ((PU16)&id)+1);\
    (id);})

#endif /* __VELOCITY_MII_H__ */
