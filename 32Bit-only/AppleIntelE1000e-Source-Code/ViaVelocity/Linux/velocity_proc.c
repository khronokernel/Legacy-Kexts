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
 * File: velocity_proc.c
 *
 * Purpose: proc entry process routines
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */

#include "velocity.h"
#include "velocity_proc.h"

static const char* VELOCITY_PROC_DIR_NAME = "Velocity_Gigabit_Adapters";
static struct proc_dir_entry *velocity_dir = NULL;

static int FunVerRead(char *page, char **start, off_t off, int count, int *eof, void *data);
static int FunStatRead(PVELOCITY_PROC_ENTRY pInfo,char* buf);
static int FunConfRead(PVELOCITY_PROC_ENTRY pInfo,char* buf);
static int FunConfWrite(PVELOCITY_PROC_ENTRY pInfo,const char* buf,unsigned long len);
static int FunRMONRead(PVELOCITY_PROC_ENTRY pInfo,char* buf);

typedef
enum _proc_conf_type {
    CONF_RX_DESC = 0, // 0
    CONF_TX_DESC,     // 1
    CONF_RX_THRESH,   // 2
    CONF_DMA_LEN,     // 3
    CONF_SPD_DPX,     // 4
    CONF_FLOW_CTRL,   // 5
    CONF_WOL_OPTS,    // 6
    CONF_ENABLE_TAG,  // 7
    CONF_VID_SETTING, // 8
    CONF_VAL_PKT,     // 9
    CONF_ENABLE_MRDPL,// 10
    CONF_ENABLE_AI    // 11
} PROC_CONF_TYPE, *PPROC_CONF_TYPE;

static const VELOCITY_PROC_ENTRY velocity_proc_tab_conf[] = {
{"RxDescriptors",  VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_RX_DESC,      NULL, 0},
{"TxDescriptors",  VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_TX_DESC,      NULL, 0},
{"rx_thresh",      VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_RX_THRESH,    NULL, 0},
{"speed_duplex",   VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_SPD_DPX,      NULL, 0},
{"flow_control",   VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_FLOW_CTRL,    NULL, 0},
{"DMA_length",     VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_DMA_LEN,      NULL, 0},
{"ValPktLen",      VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_VAL_PKT,      NULL, 0},
{"wol_opts",       VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_WOL_OPTS,     NULL, 0},
{"EnableMRDPL",    VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_ENABLE_MRDPL, NULL, 0},
{"EnableAI",       VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_ENABLE_AI,    NULL, 0},
{"enable_tagging", VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_ENABLE_TAG,   NULL, 0},
{"VID_setting",    VELOCITY_PROC_FILE, FunConfRead, FunConfWrite, CONF_VID_SETTING,  NULL, 0},
{"",               VELOCITY_PROC_EOT,  NULL,        NULL,         0,                 NULL}
};



static const VELOCITY_PROC_ENTRY velocity_proc_tab_rmon[]={
{"etherStatsDropEvents",            VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_DropEvents, NULL},
{"etherStatsOctets",                VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Octets,     NULL},
{"etherStatsPkts",                  VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Pkts,       NULL},
{"etherStatsBroadcastPkts",         VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_BroadcastPkts,  NULL},
{"etherStatsMulticastPkts",         VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_MulticastPkts,  NULL},
{"etherStatsCRCAlignErrors",        VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_CRCAlignErrors, NULL},
{"etherStatsUndersizePkts",         VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_UndersizePkts,NULL},
{"etherStatsOversizePkts",          VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_OversizePkts,NULL},
{"etherStatsFragments",             VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Fragments,NULL},
{"etherStatsJabbers",               VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Jabbers,NULL},
{"etherStatsCollisions",            VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Collisions, NULL},
{"etherStatsPkts64Octets",          VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Pkts64Octets,   NULL},
{"etherStatsPkts65to127Octets",     VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Pkts65to127Octets,  NULL},
{"etherStatsPkts128to255Octets",    VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Pkts128to255Octets, NULL},
{"etherStatsPkts256to511Octets",    VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Pkts256to511Octets, NULL},
{"etherStatsPkts512to1023Octets",   VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Pkts512to1023Octets,    NULL},
{"etherStatsPkts1024to1518Octets",  VELOCITY_PROC_READ,
    FunRMONRead, NULL, RMON_Pkts1024to1518Octets,   NULL},
{"",                                VELOCITY_PROC_EOT,
    NULL,   NULL,   0,  NULL}
};

static const VELOCITY_PROC_ENTRY velocity_proc_tab[]={
{"conf",    VELOCITY_PROC_DIR,     NULL,       NULL,   0,  velocity_proc_tab_conf},
{"rmon",    VELOCITY_PROC_DIR,     NULL,       NULL,   0,  velocity_proc_tab_rmon},
{"statics", VELOCITY_PROC_READ,    FunStatRead,NULL,   0,  NULL},
{"",    VELOCITY_PROC_EOT, NULL,   NULL,   0,  NULL}
};

static int velocity_proc_read(char *page, char **start, off_t off, int count,
    int *eof, void *data) {
    PVELOCITY_PROC_ENTRY pEntry=(PVELOCITY_PROC_ENTRY) data;
    int             len;

    len=pEntry->read_proc(pEntry, page);

    page[len++]='\n';

    if (len <= off + count)
        *eof = 1;

    *start = page + off;
   len -= off;

    if (len > count)
        len = count;

    if (len < 0)
        len = 0;
    return len;
}

static int velocity_proc_write(struct file *filp, const char* buffer,
        unsigned long count, void *data)
{
    int res=0;
    PVELOCITY_PROC_ENTRY pEntry=(PVELOCITY_PROC_ENTRY) data;
    res=pEntry->write_proc(pEntry,buffer,count);
    return res;
}

static void velocity_create_proc_tab(PVELOCITY_INFO pInfo,
    PVELOCITY_PROC_ENTRY   pParent,    const VELOCITY_PROC_ENTRY pTab[]) {

    int i;
    struct proc_dir_entry* ptr;
    PVELOCITY_PROC_ENTRY pEntry=NULL;
    PVELOCITY_PROC_ENTRY pRoot=NULL;

    if (pTab==NULL)
        return;

    for (i=0;pTab[i].type!=VELOCITY_PROC_EOT;i++) {
        //Skip some entries

        if ((pTab[i].byRevId!=0) && (pInfo->hw.byRevId<pTab[i].byRevId)) {
            continue;
        }
        pEntry=kmalloc(sizeof(VELOCITY_PROC_ENTRY),GFP_KERNEL);
        memcpy(pEntry,&pTab[i],sizeof(VELOCITY_PROC_ENTRY));
        pEntry->siblings=pRoot;
        pRoot=pEntry;

        if (pEntry->type & VELOCITY_PROC_DIR) {
            ptr=create_proc_entry(pEntry->name,S_IFDIR, pParent->pOsEntry);
            pEntry->pOsParent=pParent->pOsEntry;
            pEntry->pOsEntry=ptr;
            velocity_create_proc_tab(pInfo,pEntry,pTab[i].childs);
        }
        else {
            int flag=S_IFREG;
            if (pEntry->type & VELOCITY_PROC_READ)
                flag|=S_IRUGO;
            if (pEntry->type & VELOCITY_PROC_WRITE)
                flag|=S_IWUSR;
            ptr=create_proc_entry(pEntry->name,flag, pParent->pOsEntry);
            if (pEntry->type & VELOCITY_PROC_READ)
                ptr->read_proc=velocity_proc_read;
            if (pEntry->type & VELOCITY_PROC_WRITE)
                ptr->write_proc=velocity_proc_write;
            ptr->data=pEntry;
            pEntry->pOsEntry=ptr;
            pEntry->pInfo=pInfo;
            pEntry->pOsParent=pParent->pOsEntry;
        }
    }
    pParent->childs=pRoot;
}

static void velocity_delete_proc_tab(PVELOCITY_INFO pInfo,
    const PVELOCITY_PROC_ENTRY pEntry) {

    if (pEntry==NULL)
        return;

    if (pEntry->siblings)
        velocity_delete_proc_tab(pInfo, (PVELOCITY_PROC_ENTRY)pEntry->siblings);

    if (pEntry->type & VELOCITY_PROC_DIR)
        velocity_delete_proc_tab(pInfo, (PVELOCITY_PROC_ENTRY) pEntry->childs);


    remove_proc_entry(pEntry->name,pEntry->pOsParent);
    kfree(pEntry);
}

BOOL velocity_create_proc_entry(PVELOCITY_INFO pInfo)
{
    struct net_device *dev = pInfo->dev;

    //Create ethX directory as root directory
    pInfo->pProcDir=kmalloc(sizeof(VELOCITY_PROC_ENTRY),GFP_KERNEL);
    memset(pInfo->pProcDir,0,sizeof(VELOCITY_PROC_ENTRY));

    pInfo->pProcDir->pOsEntry=
        create_proc_entry(dev->name, S_IFDIR, velocity_dir);
    pInfo->pProcDir->pOsParent = velocity_dir;

    //Create all other directoires according the defined entries on table
    velocity_create_proc_tab(pInfo, pInfo->pProcDir, velocity_proc_tab);
    return TRUE;
}

void velocity_free_proc_entry(PVELOCITY_INFO pInfo)
{
    struct net_device *dev = pInfo->dev;

    velocity_delete_proc_tab(pInfo, (PVELOCITY_PROC_ENTRY) pInfo->pProcDir->childs);
    remove_proc_entry(dev->name, velocity_dir);
    kfree(pInfo->pProcDir);
}

BOOL    velocity_init_proc_fs(void) {
    struct proc_dir_entry* ptr;
    int len=strlen(VELOCITY_PROC_DIR_NAME);

    if (velocity_dir==NULL) {
        for (velocity_dir = proc_net->subdir; velocity_dir;velocity_dir = velocity_dir->next) {
            if ((velocity_dir->namelen == len) &&
                (!memcmp(velocity_dir->name, VELOCITY_PROC_DIR_NAME, len)))
                break;
        }

        if (velocity_dir==NULL) {
            velocity_dir=create_proc_entry(VELOCITY_PROC_DIR_NAME,S_IFDIR,proc_net);
            ptr=create_proc_entry("version",S_IFREG|S_IRUGO,velocity_dir);
            ptr->data=NULL;
            ptr->write_proc=NULL;
            ptr->read_proc=FunVerRead;
        }
    }

    if (velocity_dir==NULL)
        return FALSE;

    return TRUE;
}

void    velocity_free_proc_fs(void) {
    struct proc_dir_entry*  ptr;

//  remove_proc_entry(pInfo->pProcDir, velocity_dir);

    if (velocity_dir==NULL)
        return;

    //Check if other adapters's entry still exist
    for (ptr = velocity_dir->subdir; ptr; ptr = ptr->next) {
        if ((*(ptr->name) != '.') &&
            (strcmp(ptr->name,"version")))
            break;
    }

    if (ptr)
        return;

    remove_proc_entry("version",velocity_dir);
    remove_proc_entry(VELOCITY_PROC_DIR_NAME,proc_net);

    velocity_dir=NULL;
}

static long atol(const char* ptr,int len) {
    unsigned long l=0;
    while (*ptr!=0 && *ptr!='\n' && len-->0 ) {
        if (*ptr< '0' || *ptr >'9')
            return -1;
        l*=10;
        l+=(*ptr++-'0');
    }
    return l;
}

//----------------------------------
//
static int FunVerRead(char *page, char **start, off_t off, int count,
    int *eof, void *data) {
    int             len;

    len=sprintf(page,"%s",VELOCITY_VERSION);

    page[len++]='\n';

    if (len <= off + count)
        *eof = 1;

    *start = page + off;
    len -= off;

    if (len > count)
        len = count;

    if (len < 0)
        len = 0;
    return len;
}

static const char* MIB_STRINGS[HW_MIB_SIZE]={
    "RxAllPkts",
    "ifRxOkPkts",
    "ifTxOkPkts",
    "ifRxErrorPkts",
    "ifRxRuntOkPkt",
    "ifRxRuntErrPkt",
    "ifRx64Pkts",
    "ifTx64Pkts",
    "ifRx65To127Pkts",
    "ifTx65To127Pkts",
    "ifRx128To255Pkts",
    "ifTx128To255Pkts",
    "ifRx256To511Pkts",
    "ifTx256To511Pkts",
    "ifRx512To1023Pkts",
    "ifTx512To1023Pkts",
    "ifRx1024To1518Pkts",
    "ifTx1024To1518Pkts",
    "ifTxEtherCollisions",
    "ifRxPktCRCE",
    "ifRxJumboPkts",
    "ifTxJumboPkts",
    "ifRxMacControlFrames",
    "ifTxMacControlFrames",
    "ifRxPktFAE",
    "ifRxLongOkPkt",
    "ifRxLongPktErrPkt",
    "ifTXSQEErrors",
    "ifRxNobuf",
    "ifRxSymbolErrors",
    "ifInRangeLenthErrors",
    "ifLateCollisions"
};

static int FunStatRead(PVELOCITY_PROC_ENTRY pEntry,char* buf) {
    PVELOCITY_INFO pInfo=pEntry->pInfo;
    int         len=0;
    int         i;

    spin_lock_irq(&pInfo->lock);
    velocity_update_hw_mibs(&pInfo->hw);
    spin_unlock_irq(&pInfo->lock);
    len+=sprintf(&buf[len],"Hardware MIB Counter:\n");
    for (i=0;i<HW_MIB_SIZE;i++)
        len+=sprintf(&buf[len],"%d: %s\t%d\n",i+1,MIB_STRINGS[i], pInfo->hw.adwHWMIBCounters[i]);

    len=strlen(buf);

    return len;
}

static int FunConfRead(PVELOCITY_PROC_ENTRY pEntry,char* buf) {
    PVELOCITY_INFO pInfo=pEntry->pInfo;
    int len=0;
    PROC_CONF_TYPE  op=pEntry->data;
    switch(op) {
    case CONF_RX_DESC:
        len=sprintf(buf,"%d",pInfo->hw.sOpts.nRxDescs);
        break;

    case CONF_TX_DESC:
        len=sprintf(buf,"%d",pInfo->hw.sOpts.nTxDescs);
        break;

    case CONF_RX_THRESH:
        len=sprintf(buf,"%d",pInfo->hw.sOpts.rx_thresh);
        break;
    case CONF_SPD_DPX: {
        int mii_status=mii_check_media_mode(&pInfo->hw);
        int r=0;
        if (mii_status & VELOCITY_AUTONEG_ENABLE)
            r=0;
        else if ((mii_status & (VELOCITY_SPEED_100|VELOCITY_DUPLEX_FULL))
                ==(VELOCITY_SPEED_100|VELOCITY_DUPLEX_FULL))
            r=2;
        else if ((mii_status & (VELOCITY_SPEED_10|VELOCITY_DUPLEX_FULL))
                ==(VELOCITY_SPEED_10|VELOCITY_DUPLEX_FULL))
            r=4;
        else if (mii_status & (VELOCITY_SPEED_100))
            r=1;
        else if (mii_status & (VELOCITY_SPEED_10))
            r=3;
        else if (mii_status & (VELOCITY_SPEED_1000|VELOCITY_DUPLEX_FULL))
            r=5;
        len=sprintf(buf,"%d",r);
        }
        break;
    case CONF_DMA_LEN:
        len=sprintf(buf,"%d",pInfo->hw.sOpts.DMA_length);
        break;
    case CONF_WOL_OPTS:
        len=sprintf(buf,"%d",pInfo->hw.sOpts.wol_opts);
        break;
    case CONF_FLOW_CTRL:
        len=sprintf(buf,"%d",pInfo->hw.sOpts.flow_cntl);
        break;
    case CONF_VAL_PKT:
        len=sprintf(buf,"%d",
            (pInfo->hw.flags & VELOCITY_FLAGS_VAL_PKT_LEN) ? 1 : 0);
        break;
    case CONF_ENABLE_MRDPL:
        len=sprintf(buf,"%d",
            (pInfo->hw.flags & VELOCITY_FLAGS_MRDPL) ? 1 : 0);
        break;
    case CONF_ENABLE_AI:
        len=sprintf(buf,"%d",
            (pInfo->hw.flags & VELOCITY_FLAGS_AI) ? 1 : 0);
        break;
    case CONF_ENABLE_TAG:
        len=sprintf(buf,"%d",pInfo->hw.sOpts.tagging);
        break;
    case CONF_VID_SETTING:
        len=sprintf(buf,"%d",pInfo->hw.sOpts.vid);
        break;
    }
    return len;
}

static int FunConfWrite(PVELOCITY_PROC_ENTRY pEntry, const char* buf,
    unsigned long len) {
    PVELOCITY_INFO pInfo=pEntry->pInfo;
    PROC_CONF_TYPE  op=pEntry->data;
    long l;

    l=atol(buf,len);
    if (l<0)
        return -EINVAL;

    switch(op) {
    case CONF_RX_DESC:
        if (pInfo->hw.flags & VELOCITY_FLAGS_OPENED)
            return -EACCES;
        if ((l<4)||(l>128))
            return -EINVAL;
        pInfo->hw.sOpts.nRxDescs=(l +3 ) & ~3 ;
        break;
    case CONF_TX_DESC:
        if (pInfo->hw.flags & VELOCITY_FLAGS_OPENED)
            return -EACCES;
        if ((l<16)||(l>128))
            return -EINVAL;
        pInfo->hw.sOpts.nTxDescs=l;
        break;

    case CONF_RX_THRESH:
        if ((l<0)||(l>7))
            return -EINVAL;
        pInfo->hw.sOpts.rx_thresh=l;
        mac_set_rx_thresh(&pInfo->hw,pInfo->hw.sOpts.rx_thresh);
        break;

    case CONF_SPD_DPX: {
        int new_status=0;

        switch(l) {
        case 0:
            new_status=VELOCITY_AUTONEG_ENABLE;
            break;
        case 1:
            new_status=VELOCITY_SPEED_100;
            break;
        case 2:
            new_status=VELOCITY_SPEED_100|VELOCITY_DUPLEX_FULL;
            break;
        case 3:
            new_status=VELOCITY_SPEED_10;
            break;
        case 4:
            new_status=VELOCITY_SPEED_10|VELOCITY_DUPLEX_FULL;
            break;
        case 5:
            new_status=VELOCITY_SPEED_1000|VELOCITY_DUPLEX_FULL;
            break;
        }

        pInfo->hw.sOpts.spd_dpx = new_status;
        velocity_set_media_mode(&pInfo->hw, pInfo->hw.sOpts.spd_dpx);
        }
        break;

    case CONF_DMA_LEN:
        if ((l<0)||(l>7))
            return -EINVAL;
        pInfo->hw.sOpts.DMA_length=l;
        mac_set_dma_length(&pInfo->hw,pInfo->hw.sOpts.DMA_length);
        break;

    case CONF_WOL_OPTS:
        if ((l<0)||(l>16))
            return -EINVAL;
        pInfo->hw.sOpts.wol_opts=l;
        if (l==0)
            pInfo->hw.flags &=(~VELOCITY_FLAGS_WOL_ENABLED);
        else
            pInfo->hw.flags |=VELOCITY_FLAGS_WOL_ENABLED;
        break;

    case CONF_FLOW_CTRL:
        if (pInfo->hw.flags & VELOCITY_FLAGS_OPENED)
            return -EACCES;
        if ((l<1)||(l>3))
            return -EINVAL;
        pInfo->hw.sOpts.flow_cntl=l;
        break;

    case CONF_VAL_PKT:
        if (l==0) {
            pInfo->hw.flags &=~VELOCITY_FLAGS_VAL_PKT_LEN;
            //pInfo->hw.sOpts.flags &=~VELOCITY_FLAGS_VAL_PKT_LEN;
        }
        else if (l==1) {
            pInfo->hw.flags |=VELOCITY_FLAGS_VAL_PKT_LEN;
            //pInfo->hw.sOpts.flags |=VELOCITY_FLAGS_VAL_PKT_LEN;
        }
        else
            return -EINVAL;
        break;

    case CONF_ENABLE_MRDPL:
        if (l==0) {
            pInfo->hw.flags &= ~VELOCITY_FLAGS_MRDPL;
            //pInfo->hw.sOpts.flags &= ~VELOCITY_FLAGS_VAL_PKT_LEN;
        }
        else if (l==1) {
            pInfo->hw.flags |= VELOCITY_FLAGS_MRDPL;
            //pInfo->hw.sOpts.flags |= VELOCITY_FLAGS_MRDPL;
        }
        else
            return -EINVAL;
        break;
    case CONF_ENABLE_AI:
        if (l == 0)
            pInfo->hw.flags &= ~VELOCITY_FLAGS_AI;
        else if (l == 1)
            pInfo->hw.flags |= VELOCITY_FLAGS_AI;
        else
            return -EINVAL;
        break;
    case CONF_ENABLE_TAG:
        /*
        if (l==0) {
            pInfo->hw.flags &=~VELOCITY_FLAGS_TAGGING;
           pInfo->hw.sOpts.flags &=~VELOCITY_FLAGS_TAGGING;
        }
        else if (l==1) {
            pInfo->hw.flags |=VELOCITY_FLAGS_TAGGING;
            pInfo->hw.sOpts.flags |=VELOCITY_FLAGS_TAGGING;
        }
        else
            return -EINVAL;
        */
        if ((l<0) || (l>2))
            return -EINVAL;
        pInfo->hw.sOpts.tagging = l;
		break;
    case CONF_VID_SETTING:
        if ((l<0) || (l>4094))
            return -EINVAL;
        pInfo->hw.sOpts.vid=l;
        pInfo->hw.sOpts.tagging = 1;
        //pInfo->hw.sOpts.flags |=VELOCITY_FLAGS_TAGGING;
        velocity_init_cam_filter(&pInfo->hw);
        break;
    }
    return 0;
}

static void velocity_UpdateRMONStats(PVELOCITY_INFO pInfo) {
    spin_lock_irq(&pInfo->lock);
    velocity_update_hw_mibs(&pInfo->hw);
    spin_unlock_irq(&pInfo->lock);
/*
    pInfo->adwRMONStats[RMON_DropEvents]
    pInfo->adwRMONStats[RMON_CRCAlignErrors]
    pInfo->adwRMONStats[RMON_UndersizePkts]
    pInfo->adwRMONStats[RMON_OversizePkts]
    pInfo->adwRMONStats[RMON_Fragments]
    pInfo->adwRMONStats[RMON_Jabbers]
    pInfo->adwRMONStats[RMON_Collisions]*/

    pInfo->adwRMONStats[RMON_MulticastPkts]
        =pInfo->stats.multicast;
    pInfo->adwRMONStats[RMON_Pkts]
        =pInfo->hw.adwHWMIBCounters[HW_MIB_ifRxAllPkts];
    pInfo->adwRMONStats[RMON_Pkts64Octets]
        =pInfo->hw.adwHWMIBCounters[HW_MIB_ifRx64Pkts];
    pInfo->adwRMONStats[RMON_Pkts65to127Octets]
        =pInfo->hw.adwHWMIBCounters[HW_MIB_ifRx65To127Pkts];
    pInfo->adwRMONStats[RMON_Pkts128to255Octets]
        =pInfo->hw.adwHWMIBCounters[HW_MIB_ifRx128To255Pkts];
    pInfo->adwRMONStats[RMON_Pkts256to511Octets]
        =pInfo->hw.adwHWMIBCounters[HW_MIB_ifRx256To511Pkts];
    pInfo->adwRMONStats[RMON_Pkts512to1023Octets]
        =pInfo->hw.adwHWMIBCounters[HW_MIB_ifRx512To1023Pkts];
    pInfo->adwRMONStats[RMON_Pkts1024to1518Octets]
        =pInfo->hw.adwHWMIBCounters[HW_MIB_ifRx1024To1518Pkts];

}

static int FunRMONRead(PVELOCITY_PROC_ENTRY pEntry,char* buf) {
    PVELOCITY_INFO     pInfo   =pEntry->pInfo;
    PROC_RMON_TYPE  op      =pEntry->data;
    int len=0;

    if (op<0 || op>RMON_TAB_SIZE)
        return -EINVAL;

    velocity_UpdateRMONStats(pInfo);
    len=sprintf(buf,"%d",pInfo->adwRMONStats[op]);
    return len;
}
