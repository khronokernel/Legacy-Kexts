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
 * File: velocity_wol.c
 *
 * Purpose: Functions to set WOL.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */

#ifndef	__APPLE__
#include <linux/if_arp.h>
#endif

#if !defined(__VELOCITY_H__)
#include "velocity.h"
#endif
#if !defined(__VELOCITY_WOL_H__)
#include "velocity_wol.h"
#endif

const static unsigned short Crc16Tab[256] = {
    0x0000,     0x1189,     0x2312,     0x329b,     0x4624,     0x57ad,     0x6536,     0x74bf,
    0x8c48,     0x9dc1,     0xaf5a,     0xbed3,     0xca6c,     0xdbe5,     0xe97e,     0xf8f7,
    0x1081,     0x0108,     0x3393,     0x221a,     0x56a5,     0x472c,     0x75b7,     0x643e,
    0x9cc9,     0x8d40,     0xbfdb,     0xae52,     0xdaed,     0xcb64,     0xf9ff,     0xe876,
    0x2102,     0x308b,     0x0210,     0x1399,     0x6726,     0x76af,     0x4434,     0x55bd,
    0xad4a,     0xbcc3,     0x8e58,     0x9fd1,     0xeb6e,     0xfae7,     0xc87c,     0xd9f5,
    0x3183,     0x200a,     0x1291,     0x0318,     0x77a7,     0x662e,     0x54b5,     0x453c,
    0xbdcb,     0xac42,     0x9ed9,     0x8f50,     0xfbef,     0xea66,     0xd8fd,     0xc974,
    0x4204,     0x538d,     0x6116,     0x709f,     0x0420,     0x15a9,     0x2732,     0x36bb,
    0xce4c,     0xdfc5,     0xed5e,     0xfcd7,     0x8868,     0x99e1,     0xab7a,     0xbaf3,
    0x5285,     0x430c,     0x7197,     0x601e,     0x14a1,     0x0528,     0x37b3,     0x263a,
    0xdecd,     0xcf44,     0xfddf,     0xec56,     0x98e9,     0x8960,     0xbbfb,     0xaa72,
    0x6306,     0x728f,     0x4014,     0x519d,     0x2522,     0x34ab,     0x0630,     0x17b9,
    0xef4e,     0xfec7,     0xcc5c,     0xddd5,     0xa96a,     0xb8e3,     0x8a78,     0x9bf1,
    0x7387,     0x620e,     0x5095,     0x411c,     0x35a3,     0x242a,     0x16b1,     0x0738,
    0xffcf,     0xee46,     0xdcdd,     0xcd54,     0xb9eb,     0xa862,     0x9af9,     0x8b70,
    0x8408,     0x9581,     0xa71a,     0xb693,     0xc22c,     0xd3a5,     0xe13e,     0xf0b7,
    0x0840,     0x19c9,     0x2b52,     0x3adb,     0x4e64,     0x5fed,     0x6d76,     0x7cff,
    0x9489,     0x8500,     0xb79b,     0xa612,     0xd2ad,     0xc324,     0xf1bf,     0xe036,
    0x18c1,     0x0948,     0x3bd3,     0x2a5a,     0x5ee5,     0x4f6c,     0x7df7,     0x6c7e,
    0xa50a,     0xb483,     0x8618,     0x9791,     0xe32e,     0xf2a7,     0xc03c,     0xd1b5,
    0x2942,     0x38cb,     0x0a50,     0x1bd9,     0x6f66,     0x7eef,     0x4c74,     0x5dfd,
    0xb58b,     0xa402,     0x9699,     0x8710,     0xf3af,     0xe226,     0xd0bd,     0xc134,
    0x39c3,     0x284a,     0x1ad1,     0x0b58,     0x7fe7,     0x6e6e,     0x5cf5,     0x4d7c,
    0xc60c,     0xd785,     0xe51e,     0xf497,     0x8028,     0x91a1,     0xa33a,     0xb2b3,
    0x4a44,     0x5bcd,     0x6956,     0x78df,     0x0c60,     0x1de9,     0x2f72,     0x3efb,
    0xd68d,     0xc704,     0xf59f,     0xe416,     0x90a9,     0x8120,     0xb3bb,     0xa232,
    0x5ac5,     0x4b4c,     0x79d7,     0x685e,     0x1ce1,     0x0d68,     0x3ff3,     0x2e7a,
    0xe70e,     0xf687,     0xc41c,     0xd595,     0xa12a,     0xb0a3,     0x8238,     0x93b1,
    0x6b46,     0x7acf,     0x4854,     0x59dd,     0x2d62,     0x3ceb,     0x0e70,     0x1ff9,
    0xf78f,     0xe606,     0xd49d,     0xc514,     0xb1ab,     0xa022,     0x92b9,     0x8330,
    0x7bc7,     0x6a4e,     0x58d5,     0x495c,     0x3de3,     0x2c6a,     0x1ef1,     0x0f78
};


static U32  adwMaskPattern[2][4]={
    {0x00203000, 0x000003C0, 0x00000000, 0x0000000}, //ARP
    {0xfffff000, 0xffffffff, 0xffffffff, 0x000ffff} //Magic Packet
};

static U16 ether_crc16( int len, PU8 cp , U16 Crc16) {
    while(len--)
        Crc16 = (Crc16 >> 8) ^ Crc16Tab[(Crc16 ^ *cp++) & 0xff];
    return (Crc16);
}

static U16 wBitReverseForWord (U16 wData)
{
    U32   dwNewData = 0x00000000;
    int     ii;


    for (ii = 0; ii < 16; ii++) {
        dwNewData |= ((U32) (wData & 1) << (31 - ii));
        wData>>=1;
    }

    return (U16)(dwNewData >> 16);
}

U16 WOLCalCRC (int size, PU8 pbyPattern, PU8 abyMaskPattern)
{
    U16   wCrc = 0xFFFF;
    U8    byMask;
    int     i, j;

    for (i = 0; i <size; i++) {
         byMask = abyMaskPattern[i];

     // skip this loop if the mask equals to zero
         if (byMask == 0x00) continue;

         for (j = 0; j < 8; j++) {
              if ((byMask & 0x01) == 0) {
                  byMask >>= 1;
                  continue;
                 }

              byMask >>= 1;
              wCrc = ether_crc16(1, &(pbyPattern[i * 8 + j]), wCrc);
             }
        }

    // finally, invert the result once to get the correct data
    wCrc = ~wCrc;

    return wBitReverseForWord(wCrc);
}

BOOL velocity_set_wol(PVELOCITY_INFO pInfo) {
    //PMAC_REGS       pMacRegs=pInfo->pMacRegs;
    static BYTE abyBuf[256];
    int             i;

    CSR_WRITE_2(&pInfo->hw, 0xFFFF, MAC_REG_WOLSR0_CLR);
    CSR_WRITE_1(&pInfo->hw, WOLCFG_SAB|WOLCFG_SAM, MAC_REG_WOLCFG_SET);
    CSR_WRITE_2(&pInfo->hw, WOLCR_MAGIC_EN, MAC_REG_WOLCR0_SET);

    if (pInfo->wol_opts & VELOCITY_WOL_PHY)
		CSR_WRITE_2(&pInfo->hw, WOLCR_LINKON_EN|WOLCR_LINKOFF_EN, MAC_REG_WOLCR0_SET);

    if (pInfo->wol_opts & VELOCITY_WOL_UCAST) {
        CSR_WRITE_2(&pInfo->hw, WOLCR_UNICAST_EN, MAC_REG_WOLCR0_SET);
    }

    if (pInfo->wol_opts & VELOCITY_WOL_ARP) {
        PARP_PACKET pPacket=(PARP_PACKET) abyBuf;
        U16             wCRC;
#ifdef	__APPLE__
        bzero(abyBuf,sizeof(ARP_PACKET)+7);
#else
        memset(abyBuf,0,sizeof(ARP_PACKET)+7);
#endif
        for (i=0;i<4;i++)
            CSR_WRITE_4(&pInfo->hw, adwMaskPattern[0][i], (MAC_REG_BYTEMSK0_0+i*4));

        pPacket->wType=htons(ETH_P_ARP);
        pPacket->ar_op=htons(1);
#ifdef	__APPLE__
        bcopy(pInfo->abyIPAddr,pPacket->ar_tip,4);
#else
        memcpy(pPacket->ar_tip,pInfo->abyIPAddr,4);
#endif
        wCRC=WOLCalCRC((sizeof(ARP_PACKET)+7)/8,
            abyBuf,(PU8)&adwMaskPattern[0][0]);

        CSR_WRITE_2(&pInfo->hw, wCRC, MAC_REG_PATRN_CRC0);
        CSR_WRITE_2(&pInfo->hw, WOLCR_ARP_EN, MAC_REG_WOLCR0_SET);
    }


    BYTE_REG_BITS_ON(&pInfo->hw, PWCFG_WOLTYPE, MAC_REG_PWCFG_SET);
    BYTE_REG_BITS_ON(&pInfo->hw, PWCFG_LEGACY_WOLEN, MAC_REG_PWCFG_SET);

    CSR_WRITE_2(&pInfo->hw, 0x0FFF, MAC_REG_WOLSR0_CLR);

	//david modify
	if(SPD_DPX_1000_FULL != pInfo->hw.sOpts.spd_dpx)
	{
		if(SPD_DPX_AUTO == pInfo->hw.sOpts.spd_dpx)
		{
			if (pInfo->hw.mii_status & VELOCITY_AUTONEG_ENABLE) {
		        if (PHYID_GET_PHY_ID(pInfo->hw.dwPHYId)==PHYID_CICADA_CS8201)
		            MII_REG_BITS_ON(AUXCR_MDPPS,MII_REG_AUXCR,&pInfo->hw);
				
		        MII_REG_BITS_OFF(G1000CR_1000FD|G1000CR_1000,
								 MII_REG_G1000CR,&pInfo->hw);
		    }
			
			if (pInfo->hw.mii_status & VELOCITY_SPEED_1000 || pInfo->hw.mii_status & VELOCITY_LINK_FAIL)
	        	MII_REG_BITS_ON(BMCR_REAUTO,MII_REG_BMCR,&pInfo->hw);
		}	    
		
	    BYTE_REG_BITS_ON(&pInfo->hw, CHIPGCR_FCMODE, MAC_REG_CHIPGCR);
		
	    {
	        U8  byGCR;
	        byGCR=CSR_READ_1(&pInfo->hw, MAC_REG_CHIPGCR);
	        byGCR=(byGCR & ~CHIPGCR_FCGMII) |CHIPGCR_FCFDX;
	        CSR_WRITE_1(&pInfo->hw, byGCR, MAC_REG_CHIPGCR);
	    }
	}
	
    BYTE_REG_BITS_OFF(&pInfo->hw, ISR_PWEI, MAC_REG_ISR);
    // Turn on SWPTAG just before entering power mode
    BYTE_REG_BITS_ON(&pInfo->hw, STICKHW_SWPTAG, MAC_REG_STICKHW);
	
	CSR_WRITE_1(&pInfo->hw, WOLCFG_PMEOVR, MAC_REG_WOLCFG_SET);
	
    //Go to bed .....
    BYTE_REG_BITS_ON(&pInfo->hw, (STICKHW_DS1|STICKHW_DS0),
            MAC_REG_STICKHW);

    return FALSE;
}

void velocity_save_mac_context(struct velocity_hw *hw,PVELOCITY_CONTEXT pContext) {
    //PMAC_REGS   pMacRegs=pInfo->pMacRegs;
    U16 i;

    for (i=MAC_REG_PAR;i<MAC_REG_CR0_CLR;i+=4)
        *((PU32)(pContext->abyMacRegs+i))=CSR_READ_4(hw, i);

    for (i=MAC_REG_MAR;i<MAC_REG_TDCSR_CLR;i+=4)
        *((PU32)(pContext->abyMacRegs+i))=CSR_READ_4(hw, i);

    for (i=MAC_REG_RDBASE_LO;i<MAC_REG_FIFO_TEST0;i+=4)
        *((PU32)(pContext->abyMacRegs+i))=CSR_READ_4(hw, i);

}

void velocity_restore_mac_context(struct velocity_hw *hw, PVELOCITY_CONTEXT pContext) {
    //PMAC_REGS   pMacRegs=pInfo->pMacRegs;
    int i;

    for (i=MAC_REG_PAR;i<MAC_REG_CR0_SET;i+=4) {
        CSR_WRITE_4(hw, *((PU32)(pContext->abyMacRegs+i)),i);
    }

    //Just skip cr0
    for (i=MAC_REG_CR1_SET;i<MAC_REG_CR0_CLR;i++) {
        //clear
        CSR_WRITE_1(hw, ~(*((PU8)(pContext->abyMacRegs+i))), i+4);
        //Set
        CSR_WRITE_1(hw, *((PU8)(pContext->abyMacRegs+i)), i);
    }

    for (i=MAC_REG_MAR;i<MAC_REG_IMR;i+=4) {
        CSR_WRITE_4(hw, *((PU32)(pContext->abyMacRegs+i)),i);
    }

    for (i=MAC_REG_RDBASE_LO;i<MAC_REG_FIFO_TEST0;i+=4) {
        CSR_WRITE_4(hw, *((PU32)(pContext->abyMacRegs+i)), i);
    }

    for (i=MAC_REG_TDCSR_SET;i<=MAC_REG_RDCSR_SET;i++) {
        CSR_WRITE_1(hw, *((PU8)(pContext->abyMacRegs+i)), i);
    }
}

void 
velocity_save_pci_context(PVELOCITY_INFO pInfo, PU32 pContext) {
    // save MODE0 ~ MODE3 registers
    pci_read_config_dword(pInfo->pcid, PCI_REG_MODE0, pContext);
}

void 
velocity_restore_pci_context(PVELOCITY_INFO pInfo, U32 Context) {
    // restore MODE0 ~ MODE3 registers
    pci_write_config_dword(pInfo->pcid, PCI_REG_MODE0, Context);
}
