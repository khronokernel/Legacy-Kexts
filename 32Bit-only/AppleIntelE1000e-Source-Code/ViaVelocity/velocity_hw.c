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
 * File: velocity_hw.c
 *
 * Purpose: Describe what this file is going to do.
 *
 * Author: Guard Kuo
 *
 * Date: May 04, 2005
 *
 * Functions:
 *      List all the functions this module provides.
 *      (This section is omitted in the header of ".h" files)
 *
 * Revision History:
 *      mm-dd-yyyy Revisor's Name: Whenever you made some changes,
 *                                 leave a record here.
 *      (This section is omitted in the header of ".h" files)
 *
 */

#include "velocity_hw.h"
#include "velocity_mac.h"
#include "velocity_desc.h"
#include "velocity_mii.h"
#include "velocity.h"

void velocity_shutdown(struct velocity_hw *hw) {
    mac_disable_int(hw);
    CSR_WRITE_4(hw, CR0_STOP,MAC_REG_CR0_SET);
    CSR_WRITE_2(hw, 0xFFFF, MAC_REG_TDCSR_CLR);
    CSR_WRITE_1(hw, 0xFF, MAC_REG_RDCSR_CLR);
    SafeDisableMiiAutoPoll(hw);
    mac_clear_isr(hw);
}

BOOL velocity_soft_reset(struct velocity_hw *hw) {

    int i=0;

    CSR_WRITE_4(hw, CR0_SFRST, MAC_REG_CR0_SET);

    for (i=0;i<W_MAX_TIMEOUT;i++) {
        udelay(5);
        if (!DWORD_REG_BITS_IS_ON(hw, CR0_SFRST, MAC_REG_CR0_SET))
            break;
    }

    if (i==W_MAX_TIMEOUT) {
        CSR_WRITE_4(hw, CR0_FORSRST, MAC_REG_CR0_SET);
        /* delay 2ms */
        mdelay(2);
    }
    return TRUE;
}

void enable_flow_control_ability(struct velocity_hw *hw)
{

    switch (hw->sOpts.flow_cntl) {

        case FLOW_CNTL_DEFAULT:
            if (BYTE_REG_BITS_IS_ON(hw, PHYSR0_RXFLC, MAC_REG_PHYSR0))
                CSR_WRITE_4(hw, CR0_FDXRFCEN, MAC_REG_CR0_SET);
            else
                CSR_WRITE_4(hw, CR0_FDXRFCEN, MAC_REG_CR0_CLR);

            if (BYTE_REG_BITS_IS_ON(hw, PHYSR0_TXFLC, MAC_REG_PHYSR0))
                CSR_WRITE_4(hw, CR0_FDXTFCEN, MAC_REG_CR0_SET);
            else
                CSR_WRITE_4(hw, CR0_FDXTFCEN, MAC_REG_CR0_CLR);
            break;

        case FLOW_CNTL_TX:
            CSR_WRITE_4(hw, CR0_FDXTFCEN, MAC_REG_CR0_SET);
            CSR_WRITE_4(hw, CR0_FDXRFCEN, MAC_REG_CR0_CLR);
            break;

        case FLOW_CNTL_RX:
            CSR_WRITE_4(hw, CR0_FDXRFCEN, MAC_REG_CR0_SET);
            CSR_WRITE_4(hw, CR0_FDXTFCEN, MAC_REG_CR0_CLR);
            break;

        case FLOW_CNTL_TX_RX:
            CSR_WRITE_4(hw, CR0_FDXTFCEN, MAC_REG_CR0_SET);
            CSR_WRITE_4(hw, CR0_FDXRFCEN, MAC_REG_CR0_SET);
            break;

        case FLOW_CNTL_DISABLE:
            CSR_WRITE_4(hw, CR0_FDXRFCEN, MAC_REG_CR0_CLR);
            CSR_WRITE_4(hw, CR0_FDXTFCEN, MAC_REG_CR0_CLR);
            break;

        default:
            break;
    }
}

U32 check_connectiontype(struct velocity_hw *hw)
{
    U32 status = 0;
    U8  byPHYSR0;

    byPHYSR0 = CSR_READ_1(hw, MAC_REG_PHYSR0);

    if(!(byPHYSR0 & PHYSR0_LINKGD))
    {
        status |= VELOCITY_LINK_FAIL;
	    return status;
    }

    if(hw->sOpts.spd_dpx == SPD_DPX_AUTO){
        status |= VELOCITY_AUTONEG_ENABLE;
        if (byPHYSR0 & PHYSR0_FDPX)
            status |= VELOCITY_DUPLEX_FULL;

        if (byPHYSR0 & PHYSR0_SPDG)
            status |= VELOCITY_SPEED_1000;
        else if (byPHYSR0 & PHYSR0_SPD10)
            status |= VELOCITY_SPEED_10;
        else
            status |= VELOCITY_SPEED_100;
    }
    else{
        switch(hw->sOpts.spd_dpx)
        {
        case SPD_DPX_1000_FULL:
            status |= VELOCITY_SPEED_1000|VELOCITY_DUPLEX_FULL;
            break;
        case SPD_DPX_100_HALF:
            status |= VELOCITY_SPEED_100;
            break;
        case SPD_DPX_100_FULL:
            status |= VELOCITY_SPEED_100|VELOCITY_DUPLEX_FULL;
            break;
        case SPD_DPX_10_HALF:
            status |= VELOCITY_SPEED_10;
            break;
        case SPD_DPX_10_FULL:
            status |= VELOCITY_SPEED_10|VELOCITY_DUPLEX_FULL;
            break;
        default:
            status = 0;
        }
    }

    return status;
}

U32 mii_check_media_mode(struct velocity_hw *hw)
{
    U32 status = 0;
    U16 wANAR;

    if (!MII_REG_BITS_IS_ON(BMSR_LNK,MII_REG_BMSR, hw))
        status |= VELOCITY_LINK_FAIL;

    if (MII_REG_BITS_IS_ON(G1000CR_1000FD, MII_REG_G1000CR, hw))
        status |= (VELOCITY_SPEED_1000|VELOCITY_DUPLEX_FULL);
    else if (MII_REG_BITS_IS_ON(G1000CR_1000, MII_REG_G1000CR, hw))
        status |= VELOCITY_SPEED_1000;
    else {
        velocity_mii_read(hw, MII_REG_ANAR, &wANAR);
        if (wANAR & ANAR_TXFD)
            status |= (VELOCITY_SPEED_100|VELOCITY_DUPLEX_FULL);
        else if (wANAR & ANAR_TX)
            status |= VELOCITY_SPEED_100;
        else if (wANAR & ANAR_10FD)
            status |= (VELOCITY_SPEED_10|VELOCITY_DUPLEX_FULL);
        else
            status |= (VELOCITY_SPEED_10);
    }

    if (MII_REG_BITS_IS_ON(BMCR_AUTO, MII_REG_BMCR, hw)) {
        velocity_mii_read(hw, MII_REG_ANAR, &wANAR);
        if ((wANAR & (ANAR_TXFD|ANAR_TX|ANAR_10FD|ANAR_10))
            ==(ANAR_TXFD|ANAR_TX|ANAR_10FD|ANAR_10)) {
            if (MII_REG_BITS_IS_ON(G1000CR_1000|G1000CR_1000FD,
                MII_REG_G1000CR, hw))
                status |= VELOCITY_AUTONEG_ENABLE;
        }
    }

    return status;
}


//david add
BOOL mii_reset (struct velocity_hw *hw)
{
    WORD    ww;

    // turn on reset only, do not write other bits
    MII_REG_BITS_ON(BMCR_RESET, MII_REG_BMCR, hw);
    // polling till MII reset complete
    // W_MAX_TIMEOUT is the timeout period
    for (ww = 0; ww < W_MAX_TIMEOUT; ww++) {
        if (!MII_REG_BITS_IS_ON(BMCR_RESET, MII_REG_BMCR, hw))
            break;
    }
    if (ww == W_MAX_TIMEOUT)
        return FALSE;
    return TRUE;
}





/************************************************************************
* MII access , media link mode setting functions
************************************************************************/
void mii_init(struct velocity_hw *hw, U32 mii_status) {
    U16 wBMCR;	

    switch (PHYID_GET_PHY_ID(hw->dwPHYId)) {
    case PHYID_CICADA_CS8201:
        /* Reset to hardware default */
        MII_REG_BITS_OFF((ANAR_ASMDIR|ANAR_PAUSE), MII_REG_ANAR, hw);

        /* turn on ECHODIS bit in NWay-forced full mode and turn off it in
        // NWay-forced half mode for NWay-forced v.s. legacy-forced issue */
        if (hw->mii_status & VELOCITY_DUPLEX_FULL)
        	MII_REG_BITS_ON(TCSR_ECHODIS, MII_REG_TCSR, hw);
        else
            MII_REG_BITS_OFF(TCSR_ECHODIS, MII_REG_TCSR, hw);

		/* Turn on Link/Activity LED enable bit for CIS8201 */
		MII_REG_BITS_ON(PLED_LALBE, MII_REG_PLED, hw);

        break;

    case PHYID_VT3216_32BIT:
    case PHYID_VT3216_64BIT:
        /* Reset to hardware default */
        MII_REG_BITS_ON((ANAR_ASMDIR|ANAR_PAUSE), MII_REG_ANAR, hw);

        /* turn on ECHODIS bit in NWay-forced full mode and turn off it in
        // NWay-forced half mode for NWay-forced v.s. legacy-forced issue */
        if (hw->mii_status & VELOCITY_DUPLEX_FULL)
        	MII_REG_BITS_ON(TCSR_ECHODIS, MII_REG_TCSR, hw);
        else
            MII_REG_BITS_OFF(TCSR_ECHODIS, MII_REG_TCSR, hw);

        break;

    case PHYID_MARVELL_1000:
    case PHYID_MARVELL_1000S:
        /* Assert CRS on Transmit */
        MII_REG_BITS_ON(PSCR_ACRSTX,MII_REG_PSCR, hw);
        /*Reset to hardware default*/
        MII_REG_BITS_ON((ANAR_ASMDIR|ANAR_PAUSE),MII_REG_ANAR, hw);
        break;
    default:
        ;
    }

    velocity_mii_read(hw, MII_REG_BMCR,&wBMCR);
    if (wBMCR & BMCR_ISO) {
        wBMCR &=~BMCR_ISO;
        velocity_mii_write(hw, MII_REG_BMCR, wBMCR);
    }
}


U16 set_mii_flow_control(struct velocity_hw *hw)
{
    /* Enable or Disable PAUSE in ANAR */
    switch(hw->sOpts.flow_cntl) {
    case FLOW_CNTL_DEFAULT:
        if (PHYID_GET_PHY_ID(hw->dwPHYId) == PHYID_CICADA_CS8201) {
            /* hardware default PAUSE/ASMDIR value in CIS8201 is (0,0) */
            /*MII_REG_BITS_OFF((ANAR_ASMDIR|ANAR_PAUSE), MII_REG_ANAR, hw);*/
            return ANAR_ASMDIR|ANAR_PAUSE;

        }
        else if ((PHYID_GET_PHY_ID(hw->dwPHYId) == PHYID_VT3216_32BIT) ||
                 (PHYID_GET_PHY_ID(hw->dwPHYId) == PHYID_VT3216_64BIT)) {
            /* hardware default PAUSE/ASMDIR value in VT3216 is (1,1) */
            /*MII_REG_BITS_ON((ANAR_ASMDIR|ANAR_PAUSE), MII_REG_ANAR, hw);*/
            return ANAR_ASMDIR|ANAR_PAUSE;
        }
        else {
            /* Do nothing */
        }
        break;
    case FLOW_CNTL_TX:
        /*MII_REG_BITS_OFF(ANAR_PAUSE, MII_REG_ANAR, hw);*/
        /*MII_REG_BITS_ON(ANAR_ASMDIR, MII_REG_ANAR, hw);*/
        return ANAR_ASMDIR;
        break;

    case FLOW_CNTL_RX:
        /*MII_REG_BITS_ON(ANAR_PAUSE, MII_REG_ANAR, hw);
        MII_REG_BITS_ON(ANAR_ASMDIR, MII_REG_ANAR, hw);*/
        return ANAR_PAUSE|ANAR_ASMDIR;

        break;

    case FLOW_CNTL_TX_RX:
        /*MII_REG_BITS_ON(ANAR_PAUSE, MII_REG_ANAR, hw);
        MII_REG_BITS_ON(ANAR_ASMDIR, MII_REG_ANAR, hw);*/
        return ANAR_PAUSE|ANAR_ASMDIR;
        break;

    case FLOW_CNTL_DISABLE:
        MII_REG_BITS_OFF(ANAR_PAUSE, MII_REG_ANAR, hw);
        MII_REG_BITS_OFF(ANAR_ASMDIR, MII_REG_ANAR, hw);
        return 0;
        break;
    default:
        return 0;
        break;
    }

    return 0;
}

/*
// Get the media mode stored in EEPROM or module options
*/
U32 velocity_get_opt_media_mode(struct velocity_hw *hw)
{
    U32 status = 0;

    switch (hw->sOpts.spd_dpx) {
    case SPD_DPX_AUTO:
        status=VELOCITY_AUTONEG_ENABLE;
        break;
    case SPD_DPX_100_FULL:
        status=VELOCITY_SPEED_100|VELOCITY_DUPLEX_FULL;
        break;
    case SPD_DPX_10_FULL:
        status=VELOCITY_SPEED_10|VELOCITY_DUPLEX_FULL;
        break;
    case SPD_DPX_100_HALF:
        status=VELOCITY_SPEED_100;
        break;
    case SPD_DPX_10_HALF:
        status=VELOCITY_SPEED_10;
        break;
    case SPD_DPX_1000_FULL:
        status=VELOCITY_SPEED_1000|VELOCITY_DUPLEX_FULL;
        break;
    }
    hw->mii_status=status;
    return status;
}

BOOL velocity_mii_read(struct velocity_hw *hw, U8 byIdx, PU16 pdata)
{
    WORD    ww;

    /* disable MIICR_MAUTO, so that mii addr can be set normally */
    SafeDisableMiiAutoPoll(hw);

    CSR_WRITE_1(hw, byIdx, MAC_REG_MIIADR);

    BYTE_REG_BITS_ON(hw, MIICR_RCMD, MAC_REG_MIICR);

    for (ww = 0; ww < W_MAX_TIMEOUT; ww++) {
        if (!(CSR_READ_1(hw, MAC_REG_MIICR) & MIICR_RCMD))
            break;
    }
    *pdata = CSR_READ_2(hw, MAC_REG_MIIDATA);

    EnableMiiAutoPoll(hw);

    if (ww == W_MAX_TIMEOUT)
        return FALSE;

    return TRUE;
}

BOOL velocity_mii_write(struct velocity_hw *hw, BYTE byMiiAddr, WORD wData)
{
    WORD    ww;

    /* disable MIICR_MAUTO, so that mii addr can be set normally */
    SafeDisableMiiAutoPoll(hw);

    /* MII reg offset */
    CSR_WRITE_1(hw, byMiiAddr, MAC_REG_MIIADR);
    /* set MII data */
    CSR_WRITE_2(hw, wData, MAC_REG_MIIDATA);

    /* turn on MIICR_WCMD */
    BYTE_REG_BITS_ON(hw, MIICR_WCMD, MAC_REG_MIICR);

    /* W_MAX_TIMEOUT is the timeout period */
    for (ww = 0; ww < W_MAX_TIMEOUT; ww++) {
        udelay(5);
        if (!(CSR_READ_1(hw, MAC_REG_MIICR) & MIICR_WCMD))
            break;
    }

    EnableMiiAutoPoll(hw);

    if (ww == W_MAX_TIMEOUT)
        return FALSE;

    return TRUE;
}

void init_flow_control_register(struct velocity_hw *hw) {

    /* Set {XHITH1, XHITH0, XLTH1, XLTH0} in FlowCR1 to {1, 0, 1, 1}
      depend on RD=64, and Turn on XNOEN in FlowCR1 */
    CSR_WRITE_4(hw, (CR0_XONEN|CR0_XHITH1|CR0_XLTH1|CR0_XLTH0), MAC_REG_CR0_SET);
    CSR_WRITE_4(hw, (CR0_FDXTFCEN|CR0_FDXRFCEN|CR0_HDXFCEN|CR0_XHITH0), MAC_REG_CR0_CLR);

    /* Set TxPauseTimer to 0xFFFF */
    CSR_WRITE_2(hw, 0xFFFF, MAC_REG_PAUSE_TIMER);

    /* Initialize RBRDU to Rx buffer count. */
    CSR_WRITE_2(hw, hw->sOpts.nRxDescs, MAC_REG_RBRDU);
}

void mac_get_cam_mask(struct velocity_hw *hw, PU8 pbyMask,
    VELOCITY_CAM_TYPE cam_type) {
    int i;
    /*Select CAM mask*/
    BYTE_REG_BITS_SET(hw, CAMCR_PS_CAM_MASK,CAMCR_PS1|CAMCR_PS0,
        MAC_REG_CAMCR);

    if (cam_type==VELOCITY_VLAN_ID_CAM)
        CSR_WRITE_1(hw, CAMADDR_VCAMSL, MAC_REG_CAMADDR);
    else
        CSR_WRITE_1(hw, 0, MAC_REG_CAMADDR);

    /* read mask */
    for (i=0;i<8;i++)
        *pbyMask++ = CSR_READ_1(hw, MAC_REG_MAR+i);

    /* disable CAMEN */
    CSR_WRITE_1(hw, 0, MAC_REG_CAMADDR);

    /*Select mar*/
    BYTE_REG_BITS_SET(hw, CAMCR_PS_MAR,CAMCR_PS1|CAMCR_PS0,
        MAC_REG_CAMCR);

}

void mac_set_cam_mask(struct velocity_hw *hw, PU8 pbyMask,
    VELOCITY_CAM_TYPE cam_type) {
    int i;
    /*Select CAM mask*/
    BYTE_REG_BITS_SET(hw, CAMCR_PS_CAM_MASK,CAMCR_PS1|CAMCR_PS0,
        MAC_REG_CAMCR);

    if (cam_type==VELOCITY_VLAN_ID_CAM)
        CSR_WRITE_1(hw, CAMADDR_CAMEN|CAMADDR_VCAMSL, MAC_REG_CAMADDR);
    else
        CSR_WRITE_1(hw, CAMADDR_CAMEN, MAC_REG_CAMADDR);

    for (i=0;i<8;i++) {
        CSR_WRITE_1(hw, *pbyMask++, MAC_REG_MAR+i);
    }
    /* disable CAMEN */
    CSR_WRITE_1(hw, 0, MAC_REG_CAMADDR);

    /*Select CAM mask */
    BYTE_REG_BITS_SET(hw, CAMCR_PS_MAR,CAMCR_PS1|CAMCR_PS0,
        MAC_REG_CAMCR);
}

void mac_set_cam(struct velocity_hw *hw, int idx, PU8 addr,
    VELOCITY_CAM_TYPE cam_type) {
    int i;

    /*Select CAM mask*/
    BYTE_REG_BITS_SET(hw, CAMCR_PS_CAM_DATA,CAMCR_PS1|CAMCR_PS0, MAC_REG_CAMCR);

    idx&=(64-1);

    if (cam_type==VELOCITY_VLAN_ID_CAM)
        CSR_WRITE_1(hw, CAMADDR_CAMEN | CAMADDR_VCAMSL | idx, MAC_REG_CAMADDR);
    else
        CSR_WRITE_1(hw, CAMADDR_CAMEN|idx, MAC_REG_CAMADDR);

    if (cam_type==VELOCITY_VLAN_ID_CAM)
        CSR_WRITE_2(hw, *((PU16) addr), MAC_REG_MAR);
    else {
        for (i=0;i<6;i++) {
            CSR_WRITE_1(hw, *addr++,MAC_REG_MAR+i);
        }
    }
    BYTE_REG_BITS_ON(hw, CAMCR_CAMWR, MAC_REG_CAMCR);

    udelay(10);

    CSR_WRITE_1(hw, 0, MAC_REG_CAMADDR);

    /*Select CAM mask*/
    BYTE_REG_BITS_SET(hw, CAMCR_PS_MAR,CAMCR_PS1|CAMCR_PS0, MAC_REG_CAMCR);
}

void mac_get_cam(struct velocity_hw *hw, int idx, PU8 addr,
    VELOCITY_CAM_TYPE cam_type) {
    int i;

    /*Select CAM mask*/
    BYTE_REG_BITS_SET(hw, CAMCR_PS_CAM_DATA,CAMCR_PS1|CAMCR_PS0,
        MAC_REG_CAMCR);

    idx&=(64-1);

    if (cam_type==VELOCITY_VLAN_ID_CAM)
        CSR_WRITE_1(hw, CAMADDR_CAMEN | CAMADDR_VCAMSL|idx, MAC_REG_CAMADDR);
    else
        CSR_WRITE_1(hw, CAMADDR_CAMEN |idx, MAC_REG_CAMADDR);

    BYTE_REG_BITS_ON(hw, CAMCR_CAMRD, MAC_REG_CAMCR);

    udelay(10);

    if (cam_type==VELOCITY_VLAN_ID_CAM)
        *((PU16) addr)=CSR_READ_2(hw, MAC_REG_MAR);
    else
        for (i=0;i<6;i++, addr++)
            *((PU8)addr)=CSR_READ_1(hw, MAC_REG_MAR+i);

    CSR_WRITE_1(hw, 0, MAC_REG_CAMADDR);

    /*Select CAM mask*/
    BYTE_REG_BITS_SET(hw, CAMCR_PS_MAR,CAMCR_PS1|CAMCR_PS0,
        MAC_REG_CAMCR);
}

void mac_wol_reset(struct velocity_hw *hw) {

    /* Turn off SWPTAG right after leaving power mode */
    BYTE_REG_BITS_OFF(hw, STICKHW_SWPTAG,MAC_REG_STICKHW);
    /* clear sticky bits */
    BYTE_REG_BITS_OFF(hw, (STICKHW_DS1|STICKHW_DS0),
                            MAC_REG_STICKHW);

    BYTE_REG_BITS_OFF(hw, CHIPGCR_FCGMII,MAC_REG_CHIPGCR);
    BYTE_REG_BITS_OFF(hw, CHIPGCR_FCMODE,MAC_REG_CHIPGCR);
    /* disable force PME-enable */
    CSR_WRITE_1(hw, WOLCFG_PMEOVR, MAC_REG_WOLCFG_CLR);
    /* disable power-event config bit */
    CSR_WRITE_2(hw, 0xFFFF, MAC_REG_WOLCR0_CLR);
    /* clear power status */
    CSR_WRITE_2(hw, 0xFFFF, MAC_REG_WOLSR0_CLR);
}

void SafeDisableMiiAutoPoll (struct velocity_hw *hw) {
    WORD    ww;

    /* turn off MAUTO */
    CSR_WRITE_1(hw, 0, MAC_REG_MIICR);
    for (ww = 0; ww < W_MAX_TIMEOUT; ww++) {
        udelay(1);
        if (BYTE_REG_BITS_IS_ON(hw, MIISR_MIDLE, MAC_REG_MIISR))
            break;
    }
}

void EnableMiiAutoPoll(struct velocity_hw* hw) {
    int     ii;

    CSR_WRITE_1(hw, 0, MAC_REG_MIICR);
    CSR_WRITE_1(hw, MIIADR_SWMPL, MAC_REG_MIIADR);

    for (ii=0; ii<W_MAX_TIMEOUT; ii++) {
        udelay(1);
        if (BYTE_REG_BITS_IS_ON(hw, MIISR_MIDLE, MAC_REG_MIISR))
            break;
    }

    CSR_WRITE_1(hw, MIICR_MAUTO, MAC_REG_MIICR);

    for (ii=0; ii<W_MAX_TIMEOUT; ii++) {
        udelay(1);
        if (!BYTE_REG_BITS_IS_ON(hw, MIISR_MIDLE, MAC_REG_MIISR))
            break;
    }
}

void velocity_init_cam_filter(struct velocity_hw *hw)
{

    /* turn on MCFG_PQEN, turn off MCFG_RTGOPT */
    if(hw->sOpts.tagging == 2)
    {
        WORD_REG_BITS_OFF(hw, MCFG_PQEN, MAC_REG_MCFG0);
        WORD_REG_BITS_OFF(hw, MCFG_RTGOPT, MAC_REG_MCFG0);
        WORD_REG_BITS_OFF(hw, MCFG_VIDFR, MAC_REG_MCFG0);
    }
    else
    {
        WORD_REG_BITS_SET(hw, MCFG_PQEN, MCFG_RTGOPT, MAC_REG_MCFG0);
        WORD_REG_BITS_ON(hw, MCFG_VIDFR, MAC_REG_MCFG0);
    }

    /* Disable all CAM */
    memset(hw->abyVCAMMask, 0, sizeof(U8)*8);
    memset(hw->abyMCAMMask, 0, sizeof(U8)*8);
    mac_set_cam_mask(hw,hw->abyVCAMMask,VELOCITY_VLAN_ID_CAM);
    mac_set_cam_mask(hw,hw->abyMCAMMask,VELOCITY_MULTICAST_CAM);

    /* Enable first VCAM */
    if (hw->sOpts.tagging == 1) {
        /* if Tagging option is enabled and VLAN ID is not zero, then turn on MCFG_RTGOPT also */
        if (hw->sOpts.vid != 0)
            WORD_REG_BITS_ON(hw, MCFG_RTGOPT, MAC_REG_MCFG0);

        mac_set_cam(hw, 0, (PU8)&(hw->sOpts.vid), VELOCITY_VLAN_ID_CAM);
        hw->abyVCAMMask[0] |= 1;
        mac_set_cam_mask(hw, hw->abyVCAMMask, VELOCITY_VLAN_ID_CAM);
    }
    else {
        U16	wTemp = 0;
        mac_set_cam(hw, 0, (PU8)&wTemp, VELOCITY_VLAN_ID_CAM);
        wTemp = 1;
        mac_set_cam_mask(hw, (PU8)&(wTemp), VELOCITY_VLAN_ID_CAM);
    }
}

void velocity_update_hw_mibs(struct velocity_hw *hw) {
    U32 dwTmp;
    int i;
    BYTE_REG_BITS_ON(hw, MIBCR_MIBFLSH, MAC_REG_MIBCR);

    while (BYTE_REG_BITS_IS_ON(hw, MIBCR_MIBFLSH, MAC_REG_MIBCR));

    BYTE_REG_BITS_ON(hw, MIBCR_MPTRINI, MAC_REG_MIBCR);
    for (i=0;i<HW_MIB_SIZE;i++) {
        dwTmp=CSR_READ_4(hw, MAC_REG_MIBREAD) & 0x00FFFFFFUL;
        hw->adwHWMIBCounters[i]+=dwTmp;
    }
}

void velocity_rx_reset(struct velocity_hw *hw)
{
    int         i;

    hw->iCurrRDIdx = 0;

    /* init state, all RD is chip's */
    for(i=0; i<hw->sOpts.nRxDescs; i++)
        hw->aRDRing[i].rdesc0 |= cpu_to_le32(RDESC0_OWN);

    CSR_WRITE_2(hw, hw->sOpts.nRxDescs, MAC_REG_RBRDU);
    CSR_WRITE_4(hw, (uint32_t)hw->rd_pool_dma, MAC_REG_RDBASE_LO);
    CSR_WRITE_2(hw, 0, MAC_REG_RDINDX);
    CSR_WRITE_2(hw, hw->sOpts.nRxDescs-1, MAC_REG_RDCSIZE);
}

void velocity_print_link_status(struct velocity_hw *hw) {

    if (hw->mii_status & VELOCITY_LINK_FAIL) {
        VELOCITY_HW_PRT(hw, MSG_LEVEL_INFO, "failed to detect cable link.\n");
    }
    else {
        if (hw->sOpts.spd_dpx == SPD_DPX_AUTO) {
            VELOCITY_HW_PRT(hw, MSG_LEVEL_INFO, "Link auto-negotiation");

        	if (hw->mii_status & VELOCITY_SPEED_1000)
            	VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO," speed 1000M bps");
	        else if (hw->mii_status & VELOCITY_SPEED_100)
    		    VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO," speed 100M bps");
	        else
    		    VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO," speed 10M bps");

	        if (hw->mii_status & VELOCITY_DUPLEX_FULL)
    	        VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO, " full duplex\n");
        	else
            	VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO, " half duplex\n");
    	}
    	else {
    		VELOCITY_HW_PRT(hw, MSG_LEVEL_INFO,	"Link forced");
            switch(hw->sOpts.spd_dpx) {
                case SPD_DPX_1000_FULL:
            		VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO, " speed 1000M bps full duplex\n");
            		break;
            	case SPD_DPX_100_HALF:
            		VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO, " speed 100M bps half duplex\n");
            		break;
            	case SPD_DPX_100_FULL:
            		VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO, " speed 100M bps full duplex\n");
            		break;
            	case SPD_DPX_10_HALF:
            		VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO, " speed 10M bps half duplex\n");
            		break;
            	case SPD_DPX_10_FULL:
            		VELOCITY_PRT(hw->msglevel, MSG_LEVEL_INFO, " speed 10M bps full duplex\n");
            		break;
            	default:
            		break;
			}
		}
	}
}

int velocity_set_media_mode(struct velocity_hw *hw, SPD_DPX_OPT spd_dpx)
{
    U16 wANARMask;
    U16 wOrigANAR, wOrigG1000CR;
    U16 wANAR=0, wG1000CR=0;
    U32 status=VELOCITY_LINK_UNCHANGE;


    velocity_mii_read(hw, MII_REG_ANAR, &wOrigANAR);
    velocity_mii_read(hw, MII_REG_G1000CR, &wOrigG1000CR);
    wANARMask = wOrigANAR;
    wANARMask &= ~(ANLPAR_ASMDIR|ANLPAR_PAUSE|ANAR_TXFD|ANAR_TX|ANAR_10FD|ANAR_10);
    wOrigANAR &= (ANLPAR_ASMDIR|ANLPAR_PAUSE|ANAR_TXFD|ANAR_TX|ANAR_10FD|ANAR_10);
    wOrigG1000CR &= (G1000CR_1000FD|G1000CR_1000);



    /* Set mii link status */
    wANAR = set_mii_flow_control(hw);

    if (PHYID_GET_PHY_ID(hw->dwPHYId) == PHYID_CICADA_CS8201) {
        MII_REG_BITS_ON(AUXCR_MDPPS, MII_REG_AUXCR, hw);
    }

    /* if connection type is AUTO */
    if (spd_dpx == SPD_DPX_AUTO) {

        /*VELOCITY_HW_PRT(hw, MSG_LEVEL_INFO, "Velocity is AUTO mode\n");*/

        /* auto */
        wANAR |= (ANAR_TXFD|ANAR_TX|ANAR_10FD|ANAR_10);
        wG1000CR |= (G1000CR_1000FD|G1000CR_1000);
    	/* clear force MAC mode bit */
    	BYTE_REG_BITS_OFF(hw, CHIPGCR_FCMODE, MAC_REG_CHIPGCR);
        /* set duplex mode of MAC according to duplex mode of MII */
        MII_REG_BITS_ON(BMCR_SPEED1G, MII_REG_BMCR, hw);
        if(wOrigANAR != wANAR || wOrigG1000CR != wG1000CR) {
            wANAR |= wANARMask;
            velocity_mii_write(hw, MII_REG_ANAR, wANAR);
            MII_REG_BITS_ON(G1000CR_1000FD|G1000CR_1000, MII_REG_G1000CR, hw);
            /* enable AUTO-NEGO mode */
            MII_REG_BITS_ON((BMCR_AUTO | BMCR_REAUTO), MII_REG_BMCR, hw);

            /* Link changed */
            status = VELOCITY_LINK_CHANGE;
        }
    }
    else {
        U8  byCHIPGCR;

        /* 1. if it's 3119, disable frame bursting in halfduplex mode and
        //    enable it in fullduplex mode
        // 2. set correct MII/GMII and half/full duplex mode in CHIPGCR
        // 3. only enable CD heart beat counter in 10HD mode

        // set force MAC mode bit */
        BYTE_REG_BITS_ON(hw, CHIPGCR_FCMODE, MAC_REG_CHIPGCR);


        byCHIPGCR = CSR_READ_1(hw, MAC_REG_CHIPGCR);

        if (spd_dpx == SPD_DPX_1000_FULL)
            byCHIPGCR |= CHIPGCR_FCGMII;
        else
            byCHIPGCR &= ~CHIPGCR_FCGMII;


        if( spd_dpx == SPD_DPX_1000_FULL || spd_dpx == SPD_DPX_100_FULL || spd_dpx == SPD_DPX_10_FULL ){
            byCHIPGCR |= CHIPGCR_FCFDX;
            CSR_WRITE_1(hw, byCHIPGCR, MAC_REG_CHIPGCR);
            /*VELOCITY_HW_PRT(hw, MSG_LEVEL_INFO, "Set Velocity to forced full mode\n");*/
            if (hw->byRevId < REV_ID_VT3216_A0)
                BYTE_REG_BITS_OFF(hw, TCR_TB2BDIS, MAC_REG_TCR);
        }
        else {
            byCHIPGCR &= ~CHIPGCR_FCFDX;
            /*VELOCITY_HW_PRT(hw, MSG_LEVEL_INFO, "Set Velocity to forced half mode\n");*/
            CSR_WRITE_1(hw, byCHIPGCR, MAC_REG_CHIPGCR);
            if (hw->byRevId < REV_ID_VT3216_A0)
                BYTE_REG_BITS_ON(hw, TCR_TB2BDIS, MAC_REG_TCR);
        }

        /* No force 1000Mbps */
        wG1000CR=0;

        switch(spd_dpx)
        {
            case SPD_DPX_1000_FULL:
                wANAR &= ~(ANLPAR_TX | ANLPAR_TXFD | ANLPAR_10 | ANLPAR_10FD);
                wG1000CR |= G1000CR_1000FD;
                break;
            case SPD_DPX_100_HALF:
                wANAR |= ANLPAR_TX;
                break;
            case SPD_DPX_100_FULL:
                wANAR |= ANLPAR_TXFD;
                break;
            case SPD_DPX_10_HALF:
                wANAR |= ANLPAR_10;
                break;
            case SPD_DPX_10_FULL:
                wANAR |= ANLPAR_10FD;
                break;
            default:
                break;
        }

        if(wANAR != wOrigANAR || wG1000CR != wOrigG1000CR)
        {
            wANAR |= wANARMask;
            velocity_mii_write(hw, MII_REG_ANAR, wANAR);

            if (spd_dpx == SPD_DPX_1000_FULL) {
                MII_REG_BITS_ON(G1000CR_1000FD, MII_REG_G1000CR, hw);
                MII_REG_BITS_OFF(G1000CR_1000, MII_REG_G1000CR, hw);
            }
            else {
                MII_REG_BITS_OFF(G1000CR_1000FD|G1000CR_1000, MII_REG_G1000CR, hw);
            }
            /* enable AUTO-NEGO mode */
            MII_REG_BITS_ON((BMCR_AUTO | BMCR_REAUTO), MII_REG_BMCR, hw);

            /* Link changed */
            status = VELOCITY_LINK_CHANGE;
        }
    }

    return status;
}


void velocity_init_interrupt_mask(struct velocity_hw *hw)
{
    hw->IntMask = INT_MASK_DEF;

    // [1.18] Adaptive Interrupt
    // Set Tx Interrupt Suppression Threshold
    CSR_WRITE_1(hw, CAMCR_PS0, MAC_REG_CAMCR);
    CSR_WRITE_2(hw, hw->sOpts.tx_intsup, MAC_REG_ISR_CTL);
    //pInfo->IntMask &= ~(ISR_PTXI | ISR_PTX0I | ISR_PTX1I | ISR_PTX2I | ISR_PTX3I);

    // Set Rx Interrupt Suppression Threshold
    CSR_WRITE_1(hw, CAMCR_PS1, MAC_REG_CAMCR);
    CSR_WRITE_2(hw, hw->sOpts.rx_intsup, MAC_REG_ISR_CTL);
    //pInfo->IntMask &= ~ISR_PRXI;

    /* Select page to interrupt hold timer */
    CSR_WRITE_1(hw, 0x00, MAC_REG_CAMCR);

    /* Modify IMR if adaptive interrupt is enabled */
    if (hw->flags & VELOCITY_FLAGS_AI)
        hw->IntMask &= ~(ISR_PTXI | ISR_PTX0I | ISR_PTX1I | ISR_PTX2I | ISR_PTX3I | ISR_PRXI);
}

void velocity_init_register_reset( struct velocity_hw *hw)
{

    /* reset RX to prevent RX pointer not on the 4X location */
    velocity_rx_reset(hw);
    mac_rx_queue_run(hw);
    mac_rx_queue_wake(hw);

    CSR_WRITE_4(hw, CR0_STOP, MAC_REG_CR0_CLR);
    CSR_WRITE_4(hw, (CR0_DPOLL|CR0_TXON|CR0_RXON|CR0_STRT), MAC_REG_CR0_SET);


    if (velocity_set_media_mode(hw,hw->sOpts.spd_dpx)!=VELOCITY_LINK_CHANGE) {
        hw->mii_status = check_connectiontype(hw);
        velocity_print_link_status(hw);
    }

    enable_flow_control_ability(hw);

    mac_clear_isr(hw);
}

void velocity_init_register_cold(struct velocity_hw *hw, struct pci_dev* pcid)
{
    int i;
    U32 mii_status;
    U8  byCommand;

    mac_set_rx_thresh(hw, hw->sOpts.rx_thresh);
    mac_set_dma_length(hw, hw->sOpts.DMA_length);

    CSR_WRITE_1(hw, (WOLCFG_SAM | WOLCFG_SAB), MAC_REG_WOLCFG_SET);

    /* back off algorithm use original IEEE standard */
    BYTE_REG_BITS_SET(hw, CFGB_OFSET,
        (CFGB_CRANDOM | CFGB_CAP | CFGB_MBA | CFGB_BAKOPT),
        MAC_REG_CFGB);


    /* enable MII auto-polling */
    EnableMiiAutoPoll(hw);
    
    velocity_init_interrupt_mask(hw);

    // Enable Memory-Read-Line, both VT3119 and VT3216
    // ==> DCFG1_XMRL = 0
    BYTE_REG_BITS_OFF(hw, DCFG1_XMRL, MAC_REG_DCFG1);

    // Enable Memory-Read-Multiple
    if ((hw->byRevId >= REV_ID_VT3284_A0) || (hw->flags & VELOCITY_FLAGS_MRDPL)) {
        // (1)Enable Memory-Read-Multiple ==> DCFG1_MRDPL = 1
        BYTE_REG_BITS_ON(hw, DCFG1_MRDPL, MAC_REG_DCFG1);

        if (hw->byRevId < REV_ID_VT3284_A0) {
            // (2)Disable DAC capability ==> CFGD_CFGDACEN = 0
            // Turn on CR3_DIAG
            CSR_WRITE_1(hw, CR3_DIAG, MAC_REG_CR3_SET);
            // Turn off CFGD_CFGDACEN
            BYTE_REG_BITS_OFF(hw, CFGD_CFGDACEN, MAC_REG_CFGD);
            // Turn off CR3_DIAG
            CSR_WRITE_1(hw, CR3_DIAG, MAC_REG_CR3_CLR);

            // (3)Enable PCI address stepping ==> PCI_COMMAND_WAIT(STEP) = 1
            pci_read_config_byte(pcid, PCI_REG_COMMAND, &byCommand);
            byCommand |= PCI_REG_COMMAND;
            pci_write_config_byte(pcid, PCI_REG_COMMAND, byCommand);
        }
    }

    CSR_WRITE_4(hw, (uint32_t)hw->rd_pool_dma, MAC_REG_RDBASE_LO);
    CSR_WRITE_2(hw, hw->sOpts.nRxDescs-1, MAC_REG_RDCSIZE);
    mac_rx_queue_run(hw);
    mac_rx_queue_wake(hw);

    CSR_WRITE_2(hw, hw->sOpts.nTxDescs-1, MAC_REG_TDCSIZE);

    for (i=0;i<hw->nTxQueues;i++) {
        CSR_WRITE_4(hw, (uint32_t)hw->td_pool_dma[i], (MAC_REG_TDBASE_LO+(i*4)) );
        mac_tx_queue_run(hw,i);
    }

    velocity_init_cam_filter(hw);

    init_flow_control_register(hw);

    CSR_WRITE_4(hw, CR0_STOP, MAC_REG_CR0_CLR);
    CSR_WRITE_4(hw, (CR0_DPOLL|CR0_TXON|CR0_RXON|CR0_STRT), MAC_REG_CR0_SET);

    mii_status = velocity_get_opt_media_mode(hw);
    mac_clear_isr(hw);

    mii_init(hw, mii_status);

#ifdef LINUX
    if (velocity_set_media_mode(hw,hw->sOpts.spd_dpx) != VELOCITY_LINK_CHANGE) {
        hw->mii_status = check_connectiontype(hw);
        velocity_print_link_status(hw);
    }
#else
    velocity_set_media_mode(hw,hw->sOpts.spd_dpx);
#endif

    enable_flow_control_ability(hw);
    mac_hw_mibs_init(hw);
    mac_write_int_mask(hw->IntMask, hw);
    mac_clear_isr(hw);

}
