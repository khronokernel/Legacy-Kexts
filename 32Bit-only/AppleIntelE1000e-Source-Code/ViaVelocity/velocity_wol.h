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
 * File: velocity_wol.h
 *
 * Purpose: Hearder for WOL definitions.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */
 
#ifndef __VELOCITY_WOL_H__
#define __VELOCITY_WOL_H__

#include "velocity.h"

struct __velocity_info;

typedef U8  MCAM_ADDR[ETH_ALEN];

typedef
struct _arp_packet {
    U8  abyDMAC[ETH_ALEN];
    U8  abySMAC[ETH_ALEN];
    U16 wType;
    U16 ar_hrd;
    U16 ar_pro;
    U8  ar_hln;
    U8  ar_pln;
    U16 ar_op;
    U8  ar_sha[ETH_ALEN];
    U8  ar_sip[4];
    U8  ar_tha[ETH_ALEN];
    U8  ar_tip[4];
} __attribute__ ((__packed__))
ARP_PACKET, *PARP_PACKET;

typedef
struct _magic_packet {
    U8  abyDMAC[6];
    U8  abySMAC[6];
    U16 wType;
    U8  abyMAC[16][6];
    U8  abyPassword[6];
}
__attribute__ ((__packed__))
MAGIC_PACKET, *PMAGIC_PACKET;

typedef
struct __velocity_context {
    U8          abyMacRegs[256];
    MCAM_ADDR   aMcamAddr[MCAM_SIZE];
    U16         awVcam[VCAM_SIZE];
    U32         dwCammask[2];
    U32         dwPatCRC[2];
    U32         dwPattern[8];
} VELOCITY_CONTEXT, *PVELOCITY_CONTEXT;


BOOL velocity_set_wol(struct __velocity_info*);
void velocity_save_mac_context(struct velocity_hw* hw, PVELOCITY_CONTEXT pContext);
void velocity_restore_mac_context(struct velocity_hw* hw, PVELOCITY_CONTEXT pContext);
void velocity_save_pci_context(struct __velocity_info* pInfo, PU32 pContext);
void velocity_restore_pci_context(struct __velocity_info* pInfo, U32 pContext);

#endif // __VELOCITY_WOL_H__
