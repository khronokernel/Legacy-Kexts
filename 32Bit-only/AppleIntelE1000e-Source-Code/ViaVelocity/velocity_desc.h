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
 * File: velocity_desc.h
 *
 * Purpose: Structures for MAX RX/TX descriptors.
 *
 * Author: Chuang Liang-Shing, AJ Jiang, Ryan Fu
 *
 * Date: Jan 24, 2003
 *
 *
 */


#ifndef __VELOCITY_DESC_H__
#define __VELOCITY_DESC_H__

#include "osdep.h"

/*
// Bits in the RSR0 register
*/
#define RSR_DETAG          0x0080
#define RSR_SNTAG          0x0040
#define RSR_RXER           0x0020
#define RSR_RL             0x0010
#define RSR_CE             0x0008
#define RSR_FAE            0x0004
#define RSR_CRC            0x0002
#define RSR_VIDM           0x0001
/*
// Bits in the RSR1 register
*/
#define RSR_RXOK           0x8000        /* rx OK */
#define RSR_PFT            0x4000        /* Perfect filtering address match */
#define RSR_MAR            0x2000        /* MAC accept multicast address packet */
#define RSR_BAR            0x1000        /* MAC accept broadcast address packet */
#define RSR_PHY            0x0800        /* MAC accept physical address packet */
#define RSR_VTAG           0x0400        /* 802.1p/1Q tagging packet indicator */
#define RSR_STP            0x0200        /* start of packet */
#define RSR_EDP            0x0100        /* end of packet */
/*
// Bits in the RSR1 register
*/
#define RSR1_RXOK           0x80        /* rx OK */
#define RSR1_PFT            0x40        /* Perfect filtering address match */
#define RSR1_MAR            0x20        /* MAC accept multicast address packet */
#define RSR1_BAR            0x10        /* MAC accept broadcast address packet */
#define RSR1_PHY            0x08        /* MAC accept physical address packet */
#define RSR1_VTAG           0x04        /* 802.1p/1q tagging packet indicator */
#define RSR1_STP            0x02        /* start of packet */
#define RSR1_EDP            0x01        /* end of packet */

/*
// Bits in the CSM register
*/
#define CSM_IPOK            0x40        /*IP Checkusm validatiaon ok */
#define CSM_TUPOK           0x20        /*TCP/UDP Checkusm validatiaon ok */
#define CSM_FRAG            0x10        /*Fragment IP datagram */
#define CSM_IPKT            0x04        /*Received an IP packet */
#define CSM_TCPKT           0x02        /*Received a TCP packet */
#define CSM_UDPKT           0x01        /*Received a UDP packet */

/*
// Bits in the TSR0 register
*/
#define TSR0_ABT            0x0080        /* Tx abort because of excessive collision */
#define TSR0_OWT            0x0040        /* Jumbo frame Tx abort */
#define TSR0_OWC            0x0020        /* Out of window collision */
#define TSR0_COLS           0x0010        /* experience collision in this transmit event */
#define TSR0_NCR3           0x0008        /* collision retry counter[3] */
#define TSR0_NCR2           0x0004        /* collision retry counter[2] */
#define TSR0_NCR1           0x0002        /* collision retry counter[1] */
#define TSR0_NCR0           0x0001        /* collision retry counter[0] */
#define TSR0_TERR           0x8000        /* */
#define TSR0_FDX            0x4000        /* current transaction is serviced by full duplex mode */
#define TSR0_GMII           0x2000        /* current transaction is serviced by GMII mode */
#define TSR0_LNKFL          0x1000        /* packet serviced during link down */
#define TSR0_SHDN           0x0400        /* shutdown case */
#define TSR0_CRS            0x0200        /* carrier sense lost */
#define TSR0_CDH            0x0100        /* AQE test fail (CD heartbeat) */

/*
// Bits in the TSR1 register
*/
#define TSR1_TERR           0x80        /* */
#define TSR1_FDX            0x40        /* current transaction is serviced by full duplex mode */
#define TSR1_GMII           0x20        /* current transaction is serviced by GMII mode */
#define TSR1_LNKFL          0x10        /* packet serviced during link down */
#define TSR1_SHDN           0x04        /* shutdown case */
#define TSR1_CRS            0x02        /* carrier sense lost */
#define TSR1_CDH            0x01        /* AQE test fail (CD heartbeat) */

/*
// Bits in the TCR0 register
*/
#define TCR0_TIC            0x80        /* assert interrupt immediately while descriptor has been send complete */
#define TCR0_PIC            0x40        /* priority interrupt request, INA# is issued over adaptive interrupt scheme */
#define TCR0_VETAG          0x20        /* enable VLAN tag */
#define TCR0_IPCK           0x10        /* request IP  checksum calculation. */
#define TCR0_UDPCK          0x08        /* request UDP checksum calculation. */
#define TCR0_TCPCK          0x04        /* request TCP checksum calculation. */
#define TCR0_JMBO           0x02        /* indicate a jumbo packet in GMAC side */
#define TCR0_CRC            0x01        /* disable CRC generation */

#define TCR_TCPLS_SOF       0x02000000L
#define TCR_TCPLS_EOF       0x01000000L
#define TCR_TIC             0x00800000L
#define TCR_VETAG           0x00200000L        /* enable VLAN tag */
#define TCR_IPCK            0x00100000L        /* request IP  checksum calculation. */
#define TCR_UDPCK           0x00080000L        /* request UDP checksum calculation. */
#define TCR_TCPCK           0x00040000L        /* request TCP checksum calculation. */

#define TCPLS_NORMAL        3
#define TCPLS_START         2
#define TCPLS_END           1
#define TCPLS_MED           0

/* max transmit or receive buffer size */
#define CB_RX_BUF_SIZE     2048UL       /* max buffer size */
                                        /* NOTE: must be multiple of 4 */

#define CB_MAX_RD_NUM       512         /* MAX # of RD */
#define CB_MAX_TD_NUM       256         /* MAX # of TD */

#define CB_INIT_RD_NUM_3119 128         /* init # of RD, for setup VT3119 */
#define CB_INIT_TD_NUM_3119 64          /* init # of TD, for setup VT3119 */

#define CB_INIT_RD_NUM      128         /* init # of RD, for setup default */
#define CB_INIT_TD_NUM      64          /* init # of TD, for setup default */

/* for 3119 */
#define CB_TD_RING_NUM      4           /* # of TD rings. */
#define CB_MAX_SEG_PER_PKT  7           /* max data seg per packet (Tx) */


/* if collisions excess 15 times , tx will abort, and
// if tx fifo underflow, tx will fail
// we should try to resend it */
#define CB_MAX_TX_ABORT_RETRY   3

/*---------------------  Export Types  ------------------------------*/

/*
// receive descriptor
*/

#define RDESC0_OWN          0x80000000L
#define RDESC3_INTCTL       0x80000000L

typedef struct __rx_desc {
    U32     rdesc0;     /* [31]: Owner, [29:16]: RMBC(receive packet length */
                        /* [15:0]: RCR */
    U32     rdesc1;     /* [31:24]: IPKT, [23:16]: CSM, [15:0]: PQTAG */
    U32     dwBufAddrLo;
    U32     rdesc3;     /* [31]: Intr control enable, [29:16]: buffer size */
                        /* [15:0]: Rx buffer address(48 bit-33 bit) */
}
__attribute__ ((__packed__))
RX_DESC, * PRX_DESC;


/*
// transmit descriptor
*/

#define TDESC0_OWN      0x80000000L
#define TDTXBUF_QUE     0x80000000L
#define TDESC1_CMDZ     0xf0000000L

typedef struct _td_buf {
    U32     dwBufAddrLo;
    U32     dwBufaddrHi;    /* [31]: Queue bit, [29:16]: buffer size */
                            /* [15:0]: Tx data buffer address(48 bit-33 bit). */
}
__attribute__ ((__packed__))
TD_BUF, *PTD_BUF;

typedef struct _tx_desc {
    U32     tdesc0;         /* [31]: Owner, [29:16]: tx packet size */
                            /* [15:0]: TSR */
    U32     tdesc1;         /* [31:28]: CMDZ, [27:26]: reserved, [25:24]: TCPLS */
                            /* [23:16]: TCR, [15:13]: 802.1p priority bits, */
                            /* [12]: CFI(reserved control bit), [11:0]: 802.1Q Virtaul Lan identifier */
    TD_BUF  aTdBufs[7];
}
__attribute__ ((__packed__))
TX_DESC, *PTX_DESC;


typedef
enum {
    OWNED_BY_HOST=0,
    OWNED_BY_NIC=1
} VELOCITY_OWNER_TYPE, *PVELOCITY_OWNER_TYPE;


/*---------------------  Export Macros ------------------------------*/

#define VELOCITY_SET_RD_BUFFER_SIZE(pRD, size) {    \
    pRD->rdesc3 &= cpu_to_le32(0xc000ffffL);        \
    pRD->rdesc3 |= cpu_to_le32(((U32)size) << 16);  \
}

#define VELOCITY_SET_TD_BUFFER_SIZE(pTDBuf, size) {        \
    pTDBuf.dwBufaddrHi &= cpu_to_le32(0xc000ffffL);        \
    pTDBuf.dwBufaddrHi |= cpu_to_le32(((U32)size) << 16);  \
}

#define VELOCITY_SET_TD_PACKET_SIZE(pTD, size) {    \
    pTD->tdesc0 &= cpu_to_le32(0xc000ffffL);        \
    pTD->tdesc0 |= cpu_to_le32(((U32)size) << 16);  \
}

#define VELOCITY_SET_TD_CMDZ(pTD, cmdz) {           \
    pTD->tdesc1 &= cpu_to_le32(0x0fffffffL);        \
    pTD->tdesc1 |= cpu_to_le32(((U32)cmdz) << 28);  \
}

#define VELOCITY_SET_TD_TCPLS(pTD, tcpls) {         \
    pTD->tdesc1 &= cpu_to_le32(0xfcffffffL);        \
    pTD->tdesc1 |= cpu_to_le32(((U32)tcpls) << 24); \
}

#define VELOCITY_SET_TD_VLANID(pTD, vlanid) {       \
    pTD->tdesc1 &= cpu_to_le32(0xfffff000L);        \
    pTD->tdesc1 |= cpu_to_le32((U32)vlanid);        \
}

#define VELOCITY_SET_TD_PRIORITY(pTD, priority) {       \
    pTD->tdesc1 &= cpu_to_le32(0xffff1fffL);            \
    pTD->tdesc1 |= cpu_to_le32(((U32)priority) << 13);  \
}

#define VELOCITY_GET_RD_PACKET_SIZE(pRD) \
    (U16)((cpu_to_le32(pRD->rdesc0) >> 16) & 0x00003fffL);

#define VELOCITY_GET_TD_BUFFER_SIZE(pTDBuf)    \
    (U16)((cpu_to_le32(pTDBuf.dwBufaddrHi) >> 16) & 0x00003fffL);


#endif /* __VELOCITY_DESC_H__ */
