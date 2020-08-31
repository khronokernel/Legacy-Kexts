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
 * File: velocity_hw.h
 *
 * Purpose: Describe what this file is going to do.
 *
 * Author: Guard Kuo
 *
 * Date: May 2, 2005
 *
 *
 */

#ifndef __VELOCITY_HW_H
#define __VELOCITY_HW_H

#include "osdep.h"
#include "velocity_desc.h"
#include "velocity_mac.h"

#define BYTE_REG_BITS_ON(hw,x,p)   do { CSR_WRITE_1(hw,(CSR_READ_1(hw,(p))|(x)),(p));} while (0)
#define WORD_REG_BITS_ON(hw,x,p)   do { CSR_WRITE_2(hw,(CSR_READ_2(hw,(p))|(x)),(p));} while (0)
#define DWORD_REG_BITS_ON(hw,x,p)  do { CSR_WRITE_4(hw,(CSR_READ_4(hw,(p))|(x)),(p));} while (0)

#define BYTE_REG_BITS_IS_ON(hw,x,p) (CSR_READ_1(hw,(p)) & (x))
#define WORD_REG_BITS_IS_ON(hw,x,p) (CSR_READ_2(hw,(p)) & (x))
#define DWORD_REG_BITS_IS_ON(hw,x,p) (CSR_READ_4(hw,(p)) & (x))

#define BYTE_REG_BITS_OFF(hw,x,p)  do { CSR_WRITE_1(hw,(CSR_READ_1(hw,(p)) & (~(x))),(p));} while (0)
#define WORD_REG_BITS_OFF(hw,x,p)  do { CSR_WRITE_2(hw,(CSR_READ_2(hw,(p)) & (~(x))),(p));} while (0)
#define DWORD_REG_BITS_OFF(hw,x,p) do { CSR_WRITE_4(hw,(CSR_READ_4(hw,(p)) & (~(x))),(p));} while (0)

#define BYTE_REG_BITS_SET(hw,x,m,p)    do { CSR_WRITE_1(hw,(CSR_READ_1(hw,(p)) & (~(m))) |(x),(p));} while (0)
#define WORD_REG_BITS_SET(hw,x,m,p)    do { CSR_WRITE_2(hw,(CSR_READ_2(hw,(p)) & (~(m))) |(x),(p));} while (0)
#define DWORD_REG_BITS_SET(hw,x,m,p)   do { CSR_WRITE_4(hw,(CSR_READ_4(hw,(p)) & (~(m)))|(x),(p));}  while (0)

#define VELOCITY_WOL_MAGIC          0x00000000UL
#define VELOCITY_WOL_PHY            0x00000001UL
#define VELOCITY_WOL_ARP            0x00000002UL
#define VELOCITY_WOL_UCAST          0x00000004UL
#define VELOCITY_WOL_BCAST          0x00000010UL
#define VELOCITY_WOL_MCAST          0x00000020UL
#define VELOCITY_WOL_MAGIC_SEC      0x00000040UL

/* flags for options */
#define VELOCITY_FLAGS_TAGGING      0x00000001UL
#define VELOCITY_FLAGS_TX_CSUM      0x00000002UL
//#define VELOCITY_FLAGS_RX_CSUM      0x00000004UL
#define VELOCITY_FLAGS_IP_ALIGN     0x00000008UL
#define VELOCITY_FLAGS_VAL_PKT_LEN  0x00000010UL
#define VELOCITY_FLAGS_MRDPL        0x00000020UL
#define VELOCITY_FLAGS_AI           0x00000040UL
//#define VELOCITY_FLAGS_FLOW_CTRL    0x01000000UL


/* flags for MII status */
#define VELOCITY_LINK_FAIL          0x00000001UL
#define VELOCITY_SPEED_10           0x00000002UL
#define VELOCITY_SPEED_100          0x00000004UL
#define VELOCITY_SPEED_1000         0x00000008UL
#define VELOCITY_DUPLEX_FULL        0x00000010UL
#define VELOCITY_AUTONEG_ENABLE     0x00000020UL
#define VELOCITY_FORCED_BY_EEPROM   0x00000040UL
#define VELOCITY_LINK_CHANGE        0x00000001UL /* for velocity_set_media_duplex */
#define VELOCITY_LINK_UNCHANGE      0x00000002UL /* for velocity_set_media_duplex */

typedef enum __velocity_flow_cntl_type {
    FLOW_CNTL_DEFAULT   =   1,
    FLOW_CNTL_TX        =   2,
    FLOW_CNTL_RX        =   3,
    FLOW_CNTL_TX_RX     =   4,
    FLOW_CNTL_DISABLE   =   5
}
VELOCITY_FLOW_CNTL_TYPE, *PVELOCITY_FLOW_CNTL_TYPE;


typedef enum __velocity_msg_level {
    MSG_LEVEL_ERR       =   0,          /* Errors that will cause abnormal operation. */
    MSG_LEVEL_NOTICE    =   1,          /* Some errors need users to be notified. */
    MSG_LEVEL_INFO      =   2,          /* Normal message. */
    MSG_LEVEL_VERBOSE   =   3,          /* Will report all trival errors. */
    MSG_LEVEL_DEBUG     =   4           /* Only for debug purpose. */
} VELOCITY_MSG_LEVEL, *PVELOCITY_MSG_LEVEL;


typedef enum _speed_opt {
    SPD_DPX_AUTO        =   0,
    SPD_DPX_100_HALF    =   1,
    SPD_DPX_100_FULL    =   2,
    SPD_DPX_10_HALF     =   3,
    SPD_DPX_10_FULL     =   4,
    SPD_DPX_1000_FULL   =   5
} SPD_DPX_OPT, *PSPD_DPX_OPT;


typedef struct __velocity_opt {
    int         nRxDescs;           // Number of RX descriptors
    int         nTxDescs;           // Number of TX descriptors
    SPD_DPX_OPT spd_dpx;            // Media link mode
    int         vid;                // VLAN Id
    int         tagging;                    //Tagging
    int         DMA_length;         // DMA length
    int         rx_thresh;          // Rx threshold
    int         flow_cntl;          // flow control
    int         wol_opts;           // Wake on lan options
    int         int_works;          // interrupt work count
    int         txque_timer;        // Tx queue empty defer delay timer
    int         rxque_timer;        // Rx queue empty defer delay timer
    int         tx_intsup;          // Tx interrupt suppression threshold
    int         rx_intsup;          // Rx interrupt suppression threshold
    //U32         flags;              // Boolean type options: 
                                    // 1.enable_tagging 
                                    // 2.IP_byte_align
                                    // 3.txcsum_offload
                                    // 4.ValPktLen
                                    // 5.EnableMRDPL
} OPTIONS, *POPTIONS;

struct velocity_hw {
#ifdef	__APPLE__
    IOVirtualAddress            memaddr;
#else
    long                        memaddr;
    long                        ioaddr;
    U32                         io_size;
#endif
    PU8                         hw_addr;

    U32                         dwPHYId;
    U8                          byRevId;
    U16                         SubSystemID;
    U16                         SubVendorID;
    U32                         mii_status;

#define AVAIL_TD(hw,q)   ((hw)->sOpts.nTxDescs-((hw)->iTDUsed[(q)]))
    int                         nTxQueues;
    volatile int                iTDUsed[TX_QUEUE_NO];
    int                         aiCurrTDIdx[TX_QUEUE_NO];
    int                         aiTailTDIdx[TX_QUEUE_NO];
    PTX_DESC                    apTDRings[TX_QUEUE_NO];
    int                         iCurrRDIdx;
#define GET_RD_BY_IDX(hw, idx)   (hw->aRDRing[idx])
    PRX_DESC                    aRDRing;

    dma_addr_t                  pool_dma;
    dma_addr_t                  rd_pool_dma;
    dma_addr_t                  td_pool_dma[TX_QUEUE_NO];
#ifndef	__APPLE__
    dma_addr_t                  tx_bufs_dma;
#endif

    OPTIONS                     sOpts;
    U32                         IntMask;
    U32                         flags;

    unsigned int                rx_buf_sz;
    int                         multicast_limit;

    U32                         adwHWMIBCounters[MAX_HW_MIB_COUNTER];

    U8                          abyVCAMMask[(VCAM_SIZE/8)];
    U8                          abyMCAMMask[(MCAM_SIZE/8)];

    int                         msglevel;   /* debug message level */

    /* operation-system-specific structure */
    struct em_osdep             *back;
};


#define velocity_set_all_multi(hw) \
    CSR_WRITE_4(hw, 0xffffffffL, MAC_REG_MAR); \
    CSR_WRITE_4(hw, 0xffffffffL, MAC_REG_MAR+4)

#define velocity_clear_all_multi(hw) \
    CSR_WRITE_4(hw, 0, MAC_REG_MAR); \
    CSR_WRITE_4(hw, 0, MAC_REG_MAR + 4)

U32 check_connectiontype(struct velocity_hw *hw);
U32 mii_check_media_mode(struct velocity_hw *hw);
U16 set_mii_flow_control(struct velocity_hw *hw);
void mii_init(struct velocity_hw *hw, U32 mii_status);
void init_flow_control_register(struct velocity_hw *hw);
void enable_flow_control_ability(struct velocity_hw *hw);
BOOL velocity_soft_reset(struct velocity_hw* hw);
BOOL mii_reset (struct velocity_hw *hw);
void velocity_shutdown(struct velocity_hw *hw);
U32 velocity_get_opt_media_mode(struct velocity_hw *hw);
BOOL velocity_mii_read(struct velocity_hw* hw, U8 byIdx, PU16 pdata);
BOOL velocity_mii_write(struct velocity_hw* hw, BYTE byMiiAddr, WORD wData);
void mac_wol_reset(struct velocity_hw *hw);
void mac_get_cam(struct velocity_hw *hw, int idx, PU8 addr, VELOCITY_CAM_TYPE cam_type);
void mac_set_cam(struct velocity_hw *hw, int idx, PU8 addr, VELOCITY_CAM_TYPE cam_type);
void mac_set_cam_mask(struct velocity_hw *hw, PU8 pbyMask, VELOCITY_CAM_TYPE cam_type);
void mac_get_cam_mask(struct velocity_hw *hw, PU8 pbyMask, VELOCITY_CAM_TYPE cam_type);
void SafeDisableMiiAutoPoll (struct velocity_hw *hw);
void EnableMiiAutoPoll(struct velocity_hw* hw);
void velocity_init_cam_filter(struct velocity_hw *hw);
void velocity_update_hw_mibs(struct velocity_hw *hw);
void velocity_rx_reset(struct velocity_hw *hw);
void velocity_print_link_status(struct velocity_hw *hw);
int velocity_set_media_mode(struct velocity_hw *hw, SPD_DPX_OPT spd_dpx);

void velocity_init_register_cold(struct velocity_hw *hw, struct pci_dev* pcid);
void velocity_init_register_reset(struct velocity_hw *hw);
void velocity_init_interrupt_mask(struct velocity_hw *hw);

#endif /* __VELOCITY_HW_H */
