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
 * File: velocity_main.c
 *
 * Purpose: Functions for Linux drver interfaces.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */


#undef __NO_VERSION__

#if !defined(__VELOCITY_H__)
#include "velocity.h"
#endif

static int              velocity_nics    = 0;
static PVELOCITY_INFO   pVelocity3_Infos = NULL;
static int              msglevel         = MSG_LEVEL_INFO;

#ifdef VELOCITY_ETHTOOL_IOCTL_SUPPORT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,25)
static int velocity_ethtool_ioctl(struct net_device* dev, struct ifreq* ifr);
#else
static struct ethtool_ops velocity_ethtool_ops;
#endif
#endif

#ifdef SIOCGMIIPHY
static int velocity_mii_ioctl(struct net_device* dev, struct ifreq* ifr, int cmd);
#endif


/*
    Define module options
*/

MODULE_AUTHOR("VIA Networking Technologies, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("VIA Networking Velocity Family Gigabit Ethernet Adapter Driver");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
#define VELOCITY_PARAM(N,D) \
        static int N[MAX_UINTS]=OPTION_DEFAULT;\
        module_param_array(N, int, NULL, 0);\
        MODULE_PARM_DESC(N, D);

#else
#define VELOCITY_PARAM(N,D) \
        static const int N[MAX_UINTS]=OPTION_DEFAULT;\
        MODULE_PARM(N, "1-" __MODULE_STRING(MAX_UINTS) "i");\
        MODULE_PARM_DESC(N, D);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,18)
#define RX_DESC_MIN         64
#define RX_DESC_MAX         255
#define RX_DESC_DEF         252
VELOCITY_PARAM(RxDescriptors, "Number of receive descriptors");

#define TX_DESC_MIN         16
#define TX_DESC_MAX         256
#define TX_DESC_DEF         256
VELOCITY_PARAM(TxDescriptors, "Number of transmit descriptors");
#else
#define RX_DESC_MIN         64
#define RX_DESC_MAX         64
#define RX_DESC_DEF         64
VELOCITY_PARAM(RxDescriptors, "Number of receive descriptors");

#define TX_DESC_MIN         16
#define TX_DESC_MAX         64
#define TX_DESC_DEF         64
VELOCITY_PARAM(TxDescriptors, "Number of transmit descriptors");
#endif

#define VLAN_ID_MIN         0
#define VLAN_ID_MAX         4094
#define VLAN_ID_DEF         0
/* VID_setting[] is used for setting the VID of NIC.
   0: default VID.
   1-4094: other VIDs.
*/
VELOCITY_PARAM(VID_setting, "802.1Q VLAN ID");

#define RX_THRESH_MIN       0
#define RX_THRESH_MAX       3
#define RX_THRESH_DEF       0
/* rx_thresh[] is used for controlling the receive fifo threshold.
   0: indicate the rxfifo threshold is 128 bytes.
   1: indicate the rxfifo threshold is 512 bytes.
   2: indicate the rxfifo threshold is 1024 bytes.
   3: indicate the rxfifo threshold is store & forward.
*/
VELOCITY_PARAM(rx_thresh, "Receive fifo threshold");

#define DMA_LENGTH_MIN      0
#define DMA_LENGTH_MAX      7
#define DMA_LENGTH_DEF      6   // [1.18] Change DMA default to S&F

/* DMA_length[] is used for controlling the DMA length
   0: 8 DWORDs
   1: 16 DWORDs
   2: 32 DWORDs
   3: 64 DWORDs
   4: 128 DWORDs
   5: 256 DWORDs
   6: SF(flush till emply)
   7: SF(flush till emply)
*/
VELOCITY_PARAM(DMA_length, "DMA length");

#define TAGGING_MIN         0
#define TAGGING_MAX         2
#define TAGGING_DEF         0
/* enable_tagging[] is used for enabling 802.1Q VID tagging.
   0: disable VID seeting(default).
   1: enable VID setting.
*/
VELOCITY_PARAM(enable_tagging, "Enable 802.1Q tagging");

#define IP_ALIG_DEF         0
/* IP_byte_align[] is used for IP header DWORD byte aligned
   0: indicate the IP header won't be DWORD byte aligned.(Default) .
   1: indicate the IP header will be DWORD byte aligned.
      In some enviroment, the IP header should be DWORD byte aligned,
      or the packet will be droped when we receive it. (eg: IPVS)
*/
VELOCITY_PARAM(IP_byte_align, "Enable IP header dword aligned");

#ifdef VELOCITY_TX_CSUM_SUPPORT
#define TX_CSUM_DEF         1
/* txcsum_offload[] is used for setting the Tx checksum offload ability of NIC.
   Rx checksum offload ability is default supported and no option for it.
   0: disable Tx checksum offload
   1: enable Tx checksum offload (Default)
*/
VELOCITY_PARAM(txcsum_offload, "Enable transmit packet checksum offload");
#endif

#define FLOW_CNTL_DEF       1
#define FLOW_CNTL_MIN       1
#define FLOW_CNTL_MAX       5
/* flow_control[] is used for setting the flow control ability of NIC.
   1: hardware deafult - AUTO (default). Use Hardware default value in ANAR.
   2: enable TX flow control.
   3: enable RX flow control.
   4: enable RX/TX flow control.
   5: disable
*/
VELOCITY_PARAM(flow_control, "Enable flow control ability");

#define MED_LNK_DEF         0
#define MED_LNK_MIN         0
#define MED_LNK_MAX         5
/* speed_duplex[] is used for setting the speed and duplex mode of NIC.
   0: indicate autonegotiation for both speed and duplex mode
   1: indicate 100Mbps half duplex mode
   2: indicate 100Mbps full duplex mode
   3: indicate 10Mbps half duplex mode
   4: indicate 10Mbps full duplex mode
   5: indicate 1000Mbps full duplex mode

   Note:
        if EEPROM have been set to the force mode, this option is ignored
            by driver.
*/
VELOCITY_PARAM(speed_duplex, "Setting the speed and duplex mode");

#define VAL_PKT_LEN_DEF     0
/* ValPktLen[] is used for setting the checksum offload ability of NIC.
   0: Receive frame with invalid layer 2 length (Default)
   1: Drop frame with invalid layer 2 length
*/
VELOCITY_PARAM(ValPktLen, "Receiving or Drop invalid 802.3 frame");

#define WOL_OPT_DEF         0
#define WOL_OPT_MIN         0
#define WOL_OPT_MAX         7
/* wol_opts[] is used for controlling wake on lan behavior.
   0: Wake up if recevied a magic packet. (Default)
   1: Wake up if link status is on/off.
   2: Wake up if recevied an arp packet.
   4: Wake up if recevied any unicast packet.
   Those value can be sumed up to support more than one option.
*/
VELOCITY_PARAM(wol_opts, "Wake On Lan options");

#define INT_WORKS_DEF       64
#define INT_WORKS_MIN       10
#define INT_WORKS_MAX       256
VELOCITY_PARAM(int_works, "Number of packets per interrupt services");

// EnableMRDPL[] is used for setting the Memory-Read-Multiple ability of NIC
// 0: Disable (default)
// 1: Enable
#define MRDPL_DEF           0
VELOCITY_PARAM(EnableMRDPL, "Memory-Read-Multiple ability");

// EnableAI[] is used for setting the Adaptive-Interrupt ability of NIC
// 0: Disable
// 1: Enable (default)
#define AI_DEF              1
VELOCITY_PARAM(EnableAI, "Adaptive-Interrupt ability");

#define TXQUE_TIMER_DEF     0x59
#define TXQUE_TIMER_MIN     0x00
#define TXQUE_TIMER_MAX     0xFF
VELOCITY_PARAM(txque_timer, "Tx Queue Empty defer timer");

#define RXQUE_TIMER_DEF     0x14
#define RXQUE_TIMER_MIN     0x00
#define RXQUE_TIMER_MAX     0xFF
VELOCITY_PARAM(rxque_timer, "Rx Queue Empty defer timer");

#define TX_INTSUP_DEF       0x1F
#define TX_INTSUP_MIN       0x01
#define TX_INTSUP_MAX       0x3F
VELOCITY_PARAM(tx_intsup, "Tx Interrupt Suppression Threshold");

#define RX_INTSUP_DEF       0x1F
#define RX_INTSUP_MIN       0x01
#define RX_INTSUP_MAX       0x3F
VELOCITY_PARAM(rx_intsup, "Rx Interrupt Suppression Threshold");

static int  velocity_found1(struct pci_dev *pcid, const struct pci_device_id *ent);
static void velocity_print_info(PVELOCITY_INFO pInfo);
static int  velocity_open(struct net_device *dev);
static int  velocity_change_mtu(struct net_device *dev,int mtu);
static int  velocity_xmit(struct sk_buff *skb, struct net_device *dev);
static irqreturn_t velocity_intr(int irq, void *dev_instance, struct pt_regs *regs);
static void velocity_set_multi(struct net_device *dev);
static struct net_device_stats *velocity_get_stats(struct net_device *dev);
static int  velocity_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
static int  velocity_close(struct net_device *dev);
static int  velocity_rx_srv(PVELOCITY_INFO pInfo,int status);
static BOOL velocity_receive_frame(PVELOCITY_INFO ,int idx);
static BOOL velocity_alloc_rx_buf(PVELOCITY_INFO, int idx);
static void velocity_init_adapter(PVELOCITY_INFO pInfo, VELOCITY_INIT_TYPE);
static void velocity_init_pci(PVELOCITY_INFO pInfo);
static void velocity_free_tx_buf(PVELOCITY_INFO pInfo, PVELOCITY_TD_INFO, PTX_DESC pTD);

#ifdef VELOCITY_ETHTOOL_IOCTL_SUPPORT
static int velocity_ethtool_gset(struct net_device*  dev, struct ethtool_cmd *ecmd);
static int velocity_ethtool_sset(struct net_device*  dev, struct ethtool_cmd *ecmd);
static u32 velocity_ethtool_glink(struct net_device*  dev);
static void velocity_ethtool_gdrvinfo(struct net_device*  dev, struct ethtool_drvinfo  *info);
static void velocity_ethtool_gwol(struct net_device*  dev, struct ethtool_wolinfo *wol);
static int velocity_ethtool_swol(struct net_device*  dev, struct ethtool_wolinfo *wol);
static u32 velocity_ethtool_gmsglvl(struct net_device*  dev);
static void velocity_ethtool_smsglvl(struct net_device*  dev, u32 value);
#endif



#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,9)
#ifdef CONFIG_PM
static int velocity_notify_reboot(struct notifier_block *, unsigned long event, void *ptr);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
static int velocity_suspend(struct pci_dev *pcid, pm_message_t state);
#else
static int velocity_suspend(struct pci_dev *pcid, u32 state);
#endif
static int velocity_resume(struct pci_dev *pcid);

struct notifier_block velocity_notifier = {
        notifier_call:  velocity_notify_reboot,
        next:           NULL,
        priority:       0
};

static int velocity_netdev_event(struct notifier_block *nb, unsigned long notification, void *ptr);

static struct notifier_block velocity_inetaddr_notifier = {
    notifier_call: velocity_netdev_event,
    };
#endif //CONFIG_PM
#endif //LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,9)


static CHIP_INFO chip_info_table[] = {
    {CHIP_TYPE_VT6110, "VIA Networking Velocity Family Gigabit Ethernet Adapter", 256, 4, 0x00FFFFFFUL},
    {0, NULL}
};

static struct pci_device_id velocity_id_table[] __devinitdata = {
    {0x1106, 0x3119, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long)&chip_info_table[0]},
    {0,}
};

static char* get_product_name(int chip_id)
{
    int i;
    for (i=0; chip_info_table[i].name!=NULL; i++)
        if (chip_info_table[i].chip_id==chip_id)
            break;
    return chip_info_table[i].name;
}

static void __devexit velocity_remove1(struct pci_dev *pcid)
{
    PVELOCITY_INFO      pInfo = pci_get_drvdata(pcid);
    PVELOCITY_INFO      ptr;
    struct net_device*  dev = pInfo->dev;

    if (pInfo == NULL)
        return;

    for (ptr=pVelocity3_Infos; ptr && (ptr!=pInfo); ptr=ptr->next)
        do {} while (0);

    if (ptr == pInfo) {
        if (ptr == pVelocity3_Infos)
            pVelocity3_Infos = ptr->next;
        else
            ptr->prev->next = ptr->next;
    }
    else {
        VELOCITY_PRT(msglevel, MSG_LEVEL_ERR, KERN_ERR "info struct not found\n");
        return;
    }

    if (dev)
        unregister_netdev(dev);

    /*if (pInfo->pMacRegs)
        iounmap(pInfo->pMacRegs);*/
    if (pInfo->hw.hw_addr)
        iounmap(pInfo->hw.hw_addr);

    pci_disable_device(pcid);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    pci_release_regions(pcid);
    free_netdev(dev);
#else
    if (pInfo->hw.ioaddr)
        release_region(pInfo->hw.ioaddr, pInfo->hw.io_size);
    kfree(dev);
#endif

    pci_set_drvdata(pcid, NULL);
}


static void
velocity_set_int_opt(
    int*    opt,
    int     val,
    int     min,
    int     max,
    int     def,
    char*   name
    )
{
    if (val == -1)
        *opt = def;
    else if ((val < min) || (val > max)) {
        VELOCITY_PRT(msglevel, MSG_LEVEL_INFO, KERN_NOTICE "the value of parameter %s is invalid, the valid range is (%d-%d)\n", name, min, max);
        *opt = def;
    }
    else {
        VELOCITY_PRT(msglevel, MSG_LEVEL_INFO, KERN_INFO "set value of parameter %s to %d\n", name, val);
        *opt = val;
    }
}

static void
velocity_set_bool_opt(PU32 opt, int val, BOOL def, U32 flag, char* name)
{
    (*opt) &= (~flag);
    if (val == -1)
        *opt |= (def ? flag : 0);
    else if (val<0 || val>1) {
        printk(KERN_NOTICE "the value of parameter %s is invalid, the valid range is (0-1)\n", name);
        *opt |= (def ? flag : 0);
    }
    else {
        printk(KERN_INFO "set parameter %s to %s\n", name , val ? "TRUE" : "FALSE");
        *opt |= (val ? flag : 0);
    }
}

static void
velocity_get_options (POPTIONS pOpts, PU32 flags, int index)
{
    velocity_set_int_opt(&pOpts->rx_thresh,rx_thresh[index],
        RX_THRESH_MIN, RX_THRESH_MAX, RX_THRESH_DEF,"rx_thresh");

    velocity_set_int_opt(&pOpts->DMA_length,DMA_length[index],
        DMA_LENGTH_MIN, DMA_LENGTH_MAX, DMA_LENGTH_DEF,"DMA_length");

    velocity_set_int_opt(&pOpts->nRxDescs,RxDescriptors[index],
        RX_DESC_MIN, RX_DESC_MAX, RX_DESC_DEF, "RxDescriptors");

    velocity_set_int_opt(&pOpts->nTxDescs,TxDescriptors[index],
        TX_DESC_MIN, TX_DESC_MAX, TX_DESC_DEF, "TxDescriptors");

    velocity_set_int_opt(&pOpts->vid,VID_setting[index],
        VLAN_ID_MIN, VLAN_ID_MAX, VLAN_ID_DEF,"VID_setting");

    velocity_set_int_opt(&pOpts->tagging,enable_tagging[index],
        TAGGING_MIN, TAGGING_MAX, TAGGING_DEF, "enable_tagging");

#ifdef VELOCITY_TX_CSUM_SUPPORT
    velocity_set_bool_opt(flags, txcsum_offload[index],
        TX_CSUM_DEF, VELOCITY_FLAGS_TX_CSUM, "txcsum_offload");
#endif

    velocity_set_int_opt(&pOpts->flow_cntl, flow_control[index],
        FLOW_CNTL_MIN,FLOW_CNTL_MAX, FLOW_CNTL_DEF, "flow_control");

    velocity_set_bool_opt(flags, IP_byte_align[index],
        IP_ALIG_DEF, VELOCITY_FLAGS_IP_ALIGN, "IP_byte_align");

    velocity_set_bool_opt(flags, ValPktLen[index],
        VAL_PKT_LEN_DEF, VELOCITY_FLAGS_VAL_PKT_LEN, "ValPktLen");

    velocity_set_int_opt((int*) &pOpts->spd_dpx, speed_duplex[index],
        MED_LNK_MIN, MED_LNK_MAX, MED_LNK_DEF,"Media link mode");

    velocity_set_int_opt((int*) &pOpts->wol_opts, wol_opts[index],
        WOL_OPT_MIN, WOL_OPT_MAX, WOL_OPT_DEF,"Wake On Lan options");

    velocity_set_int_opt((int*) &pOpts->int_works, int_works[index],
        INT_WORKS_MIN, INT_WORKS_MAX, INT_WORKS_DEF,"Interrupt service works");

    velocity_set_bool_opt(flags, EnableMRDPL[index],
        MRDPL_DEF, VELOCITY_FLAGS_MRDPL, "EnableMRDPL");

    velocity_set_bool_opt(flags, EnableAI[index],
        AI_DEF, VELOCITY_FLAGS_AI, "EnableAI");

    velocity_set_int_opt((int*) &pOpts->txque_timer, txque_timer[index],
        TXQUE_TIMER_MIN, TXQUE_TIMER_MAX, TXQUE_TIMER_DEF,"Tx Queue Empty defer timer");

    velocity_set_int_opt((int*) &pOpts->rxque_timer, rxque_timer[index],
        RXQUE_TIMER_MIN, RXQUE_TIMER_MAX, RXQUE_TIMER_DEF,"Rx Queue Empty defer timer");

    velocity_set_int_opt((int*) &pOpts->tx_intsup, tx_intsup[index],
        TX_INTSUP_MIN, TX_INTSUP_MAX, TX_INTSUP_DEF,"Tx Interrupt Suppression Threshold");

    velocity_set_int_opt((int*) &pOpts->rx_intsup, rx_intsup[index],
        RX_INTSUP_MIN, RX_INTSUP_MAX, RX_INTSUP_DEF,"Rx Interrupt Suppression Threshold");

    pOpts->nRxDescs = (pOpts->nRxDescs & ~3);
}

//
// Initialiation of adapter
//
static void
velocity_init_adapter (
    PVELOCITY_INFO      pInfo,
    VELOCITY_INIT_TYPE  InitType
    )
{
    struct net_device   *dev = pInfo->dev;
    int                 i;

    printk("%s: %s ver %s\n", pInfo->dev->name, VELOCITY_NAME, VELOCITY_VERSION);


    mac_wol_reset(&pInfo->hw);

    switch (InitType) {
    case VELOCITY_INIT_RESET:
    case VELOCITY_INIT_WOL:

        netif_stop_queue(dev);

        velocity_init_register_reset(&pInfo->hw);

        if (!(pInfo->hw.mii_status & VELOCITY_LINK_FAIL))
            netif_wake_queue(dev);

        break;

    case VELOCITY_INIT_COLD:
    default:
        mac_eeprom_reload(&pInfo->hw);
        // write dev->dev_addr to MAC address field for MAC address override
        for (i = 0; i < 6; i++)
            CSR_WRITE_1(&pInfo->hw, dev->dev_addr[i], MAC_REG_PAR+i);

        // clear Pre_ACPI bit.
        BYTE_REG_BITS_OFF(&pInfo->hw, CFGA_PACPI, MAC_REG_CFGA);

        // set packet filter
        // receive directed and broadcast address
        velocity_set_multi(dev);
        netif_stop_queue(dev);

        velocity_init_register_cold(&pInfo->hw, pInfo->pcid);

        if (!(pInfo->hw.mii_status & VELOCITY_LINK_FAIL))
            netif_wake_queue(dev);
        break;
    } // switch (InitType)
}

static void
velocity_init_pci(PVELOCITY_INFO pInfo) {

    // turn this on to avoid retry forever
    PCI_BYTE_REG_BITS_ON(MODE2_PCEROPT, PCI_REG_MODE2, pInfo->pcid);
    // for some legacy BIOS and OS don't open BusM
    // bit in PCI configuration space. So, turn it on.
    PCI_BYTE_REG_BITS_ON(COMMAND_BUSM, PCI_REG_COMMAND, pInfo->pcid);
    // turn this on to detect MII coding error
    PCI_BYTE_REG_BITS_ON(MODE3_MIION, PCI_REG_MODE3, pInfo->pcid);
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
static const struct net_device_ops velocity_netdev_ops = {
	.ndo_open				= velocity_open,
	.ndo_stop				= velocity_close,
	.ndo_get_stats			= velocity_get_stats,
	.ndo_start_xmit			= velocity_xmit,
	.ndo_set_multicast_list = velocity_set_multi,
	.ndo_do_ioctl			= velocity_ioctl,
	.ndo_change_mtu			= velocity_change_mtu,
};
#endif



static int
velocity_found1(
    struct pci_dev              *pcid,
    const struct pci_device_id  *ent
    )
{
    static BOOL         bFirst = TRUE;
    struct net_device*  dev = NULL;
    int                 i, rc;
    PCHIP_INFO          pChip_info = (PCHIP_INFO)ent->driver_data;
    PVELOCITY_INFO      pInfo, p;
    long                ioaddr, memaddr;
    PU8                 hw_addr;
    U32                 flags;


    if (velocity_nics++ >= MAX_UINTS) {
        printk(KERN_NOTICE VELOCITY_NAME ": already found %d NICs\n", velocity_nics);
        return -ENODEV;
    }

    rc = pci_enable_device(pcid);
    if (rc)
        goto err_out;

    rc = pci_set_dma_mask(pcid, 0xffffffffL);
    if (rc) {
        printk(KERN_ERR VELOCITY_NAME "PCI DMA not supported!\n");
        goto err_out;
    }

    ioaddr = pci_resource_start(pcid, 0);
    memaddr = pci_resource_start(pcid, 1);

    pci_set_master(pcid);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    dev = alloc_etherdev(sizeof(VELOCITY_INFO));
#else
    dev = init_etherdev(NULL, sizeof(VELOCITY_INFO));
#endif

    if (dev == NULL) {
        rc = -ENOMEM;
        printk(KERN_ERR VELOCITY_NAME ": allocate net device failed!\n");
        goto err_out;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    /* Chain it all together */
    //SET_MODULE_OWNER(dev);
    SET_NETDEV_DEV(dev, &pcid->dev);
#endif
    pInfo = netdev_priv(dev);

    if (bFirst) {
        printk(KERN_INFO "%s Ver. %s\n",VELOCITY_FULL_DRV_NAM, VELOCITY_VERSION);
        printk(KERN_INFO "Copyright (c) 2002, 2003 VIA Networking Technologies, Inc.\n");
        bFirst=FALSE;
    }

    // init pVelocity3_Infos information
    if (pVelocity3_Infos == NULL) {
        pVelocity3_Infos = pInfo;
    }
    else {
        for (p=pVelocity3_Infos; p->next!=NULL; p=p->next)
            do {} while (0);
        p->next = pInfo;
        pInfo->prev = p;
    }

    // init pInfo information
    pci_read_config_word(pcid, PCI_SUBSYSTEM_ID, &pInfo->hw.SubSystemID);
    pci_read_config_word(pcid, PCI_SUBSYSTEM_VENDOR_ID, &pInfo->hw.SubVendorID);
    pci_read_config_byte(pcid, PCI_REVISION_ID, &pInfo->hw.byRevId);

    pInfo->chip_id = pChip_info->chip_id;
    pInfo->hw.nTxQueues = 1;
    pInfo->hw.multicast_limit = MCAM_SIZE;
    pInfo->pcid = pcid;
    spin_lock_init(&(pInfo->lock));
    spin_lock_init(&(pInfo->xmit_lock));

    /* init velocity_hw */
    pInfo->hw.io_size = 256;
    pInfo->hw.ioaddr = ioaddr;
    pInfo->hw.memaddr = memaddr;

    /* assign message level */
    pInfo->hw.msglevel = msglevel;

    pInfo->dev = dev;
    pInfo->osdep.dev = dev;
    pInfo->hw.back = &pInfo->osdep;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    rc = pci_request_regions(pcid, VELOCITY_NAME);
    if (rc) {
        printk(KERN_ERR VELOCITY_NAME ": Failed to find PCI device\n");
        goto err_out_free_dev;
    }
#else
    if (check_region(pInfo->hw.ioaddr, pInfo->hw.io_size)) {
        printk(KERN_ERR VELOCITY_NAME ": Failed to find PCI device\n");
        goto err_out_free_dev;
    }
    request_region(pInfo->hw.ioaddr, pInfo->hw.io_size, VELOCITY_NAME);
#endif


    hw_addr = ioremap(pInfo->hw.memaddr & PCI_BASE_ADDRESS_MEM_MASK, pInfo->hw.io_size);
    if (!hw_addr) {
        rc = -EIO;
        printk(KERN_ERR VELOCITY_NAME ": ioremap failed for region 0x%lx\n", pInfo->hw.memaddr);
        goto err_out_free_res;
    }

    pInfo->hw.hw_addr = hw_addr;

    mac_wol_reset(&pInfo->hw);	

    // get MII PHY Id
    velocity_mii_read(&pInfo->hw, MII_REG_PHYID2, (PU16)&pInfo->hw.dwPHYId);
    velocity_mii_read(&pInfo->hw, MII_REG_PHYID1, ((PU16)&pInfo->hw.dwPHYId)+1);

    // software reset
    velocity_soft_reset(&pInfo->hw);
    mdelay(5);

	//david add
	mii_reset(&pInfo->hw);
	mdelay(5);

    // EEPROM reload
    mac_eeprom_reload(&pInfo->hw);

    // set net_device related stuffs
    dev->base_addr = pInfo->hw.ioaddr;
    for (i=0; i<6; i++)
        dev->dev_addr[i] = CSR_READ_1(&pInfo->hw, MAC_REG_PAR+i);
    dev->irq                = pcid->irq;	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	dev->netdev_ops 		= &velocity_netdev_ops;
#else
    dev->open               = velocity_open;
    dev->hard_start_xmit    = velocity_xmit;
    dev->stop               = velocity_close;
    dev->get_stats          = velocity_get_stats;
    dev->set_multicast_list = velocity_set_multi;
    dev->do_ioctl           = velocity_ioctl;
    dev->change_mtu         = velocity_change_mtu;
#endif

#ifdef VELOCITY_ETHTOOL_IOCTL_SUPPORT
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,25)
    dev->ethtool_ops = &velocity_ethtool_ops;
#endif
#endif

    velocity_get_options(&pInfo->hw.sOpts, &flags, velocity_nics-1);

    // Mask out the options cannot be set to the chip
    flags &= pChip_info->flags;

    // Enable the chip specified capbilities
    pInfo->hw.flags = flags | (pChip_info->flags & 0xFF000000L);

    pInfo->wol_opts = pInfo->hw.sOpts.wol_opts;

    pInfo->hw.flags |= VELOCITY_FLAGS_WOL_ENABLED;

#ifdef  VELOCITY_TX_CSUM_SUPPORT
    if (pInfo->hw.flags & VELOCITY_FLAGS_TX_CSUM) {
        dev->features |= NETIF_F_IP_CSUM;

#ifdef  VELOCITY_ZERO_COPY_SUPPORT
        // Checksum function must be enabled, or SG will be dropped
        dev->features |= NETIF_F_SG;
#endif
    }
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    rc = register_netdev(dev);
    if (rc) {
        printk(KERN_ERR VELOCITY_NAME ": Failed to register netdev\n");
        goto err_out_unmap;
    }
#endif

    velocity_print_info(pInfo);

    pci_set_drvdata(pcid, pInfo);

    return 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
err_out_unmap:
    iounmap(pInfo->hw.hw_addr);
err_out_free_res:
    pci_release_regions(pcid);
err_out_free_dev:
    free_netdev(dev);
#else
err_out_free_res:
    if (pInfo->hw.ioaddr)
        release_region(pInfo->hw.ioaddr, pInfo->hw.io_size);
err_out_free_dev:
    kfree(dev);
#endif

err_out:
    return rc;
}


static void velocity_print_info(PVELOCITY_INFO pInfo)
{
    struct net_device* dev = pInfo->dev;

    printk(KERN_INFO "%s: %s\n", dev->name, get_product_name(pInfo->chip_id));
    printk(KERN_INFO "%s: MAC=%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X",
        dev->name,
        dev->dev_addr[0],dev->dev_addr[1],dev->dev_addr[2],
        dev->dev_addr[3],dev->dev_addr[4],dev->dev_addr[5]);

    printk(" IO=0x%lx Mem=0x%lx ", pInfo->hw.ioaddr, pInfo->hw.memaddr);
    printk(" IRQ=%d \n", dev->irq);

}

static BOOL velocity_init_rings(PVELOCITY_INFO pInfo)
{
    int     i;
    struct net_device* dev = pInfo->dev;

    /*allocate all RD/TD rings a single pool*/
    pInfo->pool = pci_alloc_consistent(pInfo->pcid,
                    pInfo->hw.sOpts.nRxDescs * sizeof(RX_DESC) + 64 +
                    pInfo->hw.sOpts.nTxDescs * sizeof(TX_DESC)*pInfo->hw.nTxQueues,
                    &pInfo->hw.pool_dma);

    if (pInfo->pool == NULL) {
        printk(KERN_ERR "%s : allocate dma memory failed\n", dev->name);
        return FALSE;
    }

    memset(pInfo->pool,0, pInfo->hw.sOpts.nRxDescs * sizeof(RX_DESC) +
        pInfo->hw.sOpts.nTxDescs * sizeof(TX_DESC)*pInfo->hw.nTxQueues);

    pInfo->hw.aRDRing = (PRX_DESC)(((unsigned long)(((PU8) pInfo->pool) + 63)) & ~63);

    pInfo->hw.rd_pool_dma = pInfo->hw.pool_dma;

    pInfo->tx_bufs=pci_alloc_consistent(pInfo->pcid,
                pInfo->hw.sOpts.nTxDescs * PKT_BUF_SZ*pInfo->hw.nTxQueues,
                &pInfo->hw.tx_bufs_dma);

    if (pInfo->tx_bufs == NULL) {
        printk(KERN_ERR "%s: allocate dma memory failed\n", dev->name);
        pci_free_consistent(pInfo->pcid,
            pInfo->hw.sOpts.nRxDescs * sizeof(RX_DESC) + 64 +
            pInfo->hw.sOpts.nTxDescs * sizeof(TX_DESC)*pInfo->hw.nTxQueues,
            pInfo->pool, pInfo->hw.pool_dma);
        return FALSE;
    }

    memset(pInfo->tx_bufs, 0, pInfo->hw.sOpts.nTxDescs * PKT_BUF_SZ * pInfo->hw.nTxQueues);

    for (i=0;i<pInfo->hw.nTxQueues;i++) {

        pInfo->hw.td_pool_dma[i]=pInfo->hw.rd_pool_dma+
                pInfo->hw.sOpts.nRxDescs*sizeof(RX_DESC)+
                pInfo->hw.sOpts.nTxDescs*sizeof(TX_DESC)*i;

        pInfo->hw.apTDRings[i]=(PTX_DESC) (((PU8) pInfo->hw.aRDRing)+
                pInfo->hw.sOpts.nRxDescs*sizeof(RX_DESC)+
                pInfo->hw.sOpts.nTxDescs*sizeof(TX_DESC)*i);
    }

    return TRUE;
}

static void velocity_free_rings(PVELOCITY_INFO pInfo) {

    pci_free_consistent(pInfo->pcid,
        pInfo->hw.sOpts.nRxDescs * sizeof(RX_DESC) + 64 +
        pInfo->hw.sOpts.nTxDescs * sizeof(TX_DESC)*pInfo->hw.nTxQueues,
        pInfo->pool, pInfo->hw.pool_dma);

    if (pInfo->tx_bufs)
        pci_free_consistent(pInfo->pcid,
            pInfo->hw.sOpts.nTxDescs * PKT_BUF_SZ*pInfo->hw.nTxQueues,
            pInfo->tx_bufs, pInfo->hw.tx_bufs_dma);
}

static BOOL velocity_init_rd_ring(PVELOCITY_INFO pInfo)
{
    int i;
    PRX_DESC        pDesc;
    PVELOCITY_RD_INFO  pRDInfo;
    struct net_device* dev = pInfo->dev;

    pInfo->aRDInfo=MALLOC(sizeof(VELOCITY_RD_INFO)*pInfo->hw.sOpts.nRxDescs,
            GFP_KERNEL);
    memset(pInfo->aRDInfo,0,sizeof(VELOCITY_RD_INFO)*pInfo->hw.sOpts.nRxDescs);

    /* Init the RD ring entries */
    for (i = 0; i < pInfo->hw.sOpts.nRxDescs; i++) {
        pDesc=&(pInfo->hw.aRDRing[i]);
        pRDInfo=&(pInfo->aRDInfo[i]);

        if (!velocity_alloc_rx_buf(pInfo, i)) {
            VELOCITY_PRT(msglevel, MSG_LEVEL_ERR,KERN_ERR "%s: can not alloc rx bufs\n",
            dev->name);
            return FALSE;
        }
        pDesc->rdesc0 |= cpu_to_le32(RDESC0_OWN);
    }

    pInfo->hw.iCurrRDIdx = 0;
    return TRUE;
}

static void velocity_free_rd_ring(PVELOCITY_INFO pInfo) {
    int i;
    PVELOCITY_RD_INFO  pRDInfo;

    if (pInfo->aRDInfo == NULL)
        return;

    for (i = 0; i < pInfo->hw.sOpts.nRxDescs; i++) {
        pRDInfo = &(pInfo->aRDInfo[i]);

        if (pRDInfo->skb_dma) {
            pci_unmap_single(pInfo->pcid,pRDInfo->skb_dma,
               pInfo->hw.rx_buf_sz, PCI_DMA_FROMDEVICE);
            pRDInfo->skb_dma=(dma_addr_t)NULL;
        }
        if (pRDInfo->skb) {
            dev_kfree_skb(pRDInfo->skb);
            pRDInfo->skb=NULL;
        }
    }

    if (pInfo->aRDInfo)
        kfree(pInfo->aRDInfo);
    pInfo->aRDInfo = NULL;
}

static BOOL velocity_init_td_ring(PVELOCITY_INFO pInfo) {
    int i,j;
    dma_addr_t  curr;
    PTX_DESC        pDesc;
    PVELOCITY_TD_INFO  pTDInfo;

    /* Init the TD ring entries */
    for (j=0;j<pInfo->hw.nTxQueues;j++) {
        curr=pInfo->hw.td_pool_dma[j];

        pInfo->apTDInfos[j]=MALLOC(sizeof(VELOCITY_TD_INFO)*pInfo->hw.sOpts.nTxDescs,
            GFP_KERNEL);

        memset(pInfo->apTDInfos[j],0,
            sizeof(VELOCITY_TD_INFO)*pInfo->hw.sOpts.nTxDescs);

        for (i = 0; i < pInfo->hw.sOpts.nTxDescs; i++, curr+=sizeof(TX_DESC)) {
            pDesc=&(pInfo->hw.apTDRings[j][i]);
            pTDInfo=&(pInfo->apTDInfos[j][i]);
            pTDInfo->buf=pInfo->tx_bufs+(i+j)*PKT_BUF_SZ;
            pTDInfo->buf_dma=pInfo->hw.tx_bufs_dma+(i+j)*PKT_BUF_SZ;
        }
        pInfo->hw.aiTailTDIdx[j]=pInfo->hw.aiCurrTDIdx[j]=pInfo->hw.iTDUsed[j]=0;
    }
    return TRUE;
}

static void velocity_free_td_ring(PVELOCITY_INFO pInfo) {
    int i, j, k;
    PVELOCITY_TD_INFO   pTDInfo;

    for (j=0; j<pInfo->hw.nTxQueues; j++) {
        if (pInfo->apTDInfos[j] == NULL)
            continue;

        for (i = 0; i < pInfo->hw.sOpts.nTxDescs; i++) {
            pTDInfo = &(pInfo->apTDInfos[j][i]);
            if (pTDInfo == NULL)
                continue;

            for (k=0; k<pTDInfo->nskb_dma; k++)
                if (pTDInfo->skb_dma[k] && (pTDInfo->skb_dma[k] != pTDInfo->buf_dma)) {
                    pci_unmap_single(pInfo->pcid, pTDInfo->skb_dma[k],
                       pTDInfo->skb->len, PCI_DMA_TODEVICE);
                    pTDInfo->skb_dma[k] = (dma_addr_t)NULL;
                }

            if (pTDInfo->skb) {
                dev_kfree_skb(pTDInfo->skb);
                pTDInfo->skb = NULL;
            }
        }

        if (pInfo->apTDInfos[j]) {
            kfree(pInfo->apTDInfos[j]);
            pInfo->apTDInfos[j] = NULL;
        }
    }
}

/*-----------------------------------------------------------------*/
static int velocity_rx_srv(PVELOCITY_INFO pInfo, int status)
{
    PRX_DESC                    pRD;
    struct net_device_stats*    pStats = &pInfo->stats;
    //PMAC_REGS                   pMacRegs = pInfo->pMacRegs;
    int                         iCurrRDIdx = pInfo->hw.iCurrRDIdx;
    int                         works = 0;
    U16                         wRSR;
    BOOL			bVIDM = FALSE;\
    WORD                        wRBRDU;
    WORD                        wCurrDeltaRBRDU;

    if(pInfo->hw.sOpts.tagging == 2)
 		bVIDM = TRUE;

    while (TRUE) {
        pRD = &(pInfo->hw.aRDRing[iCurrRDIdx]);

        if ((pInfo->aRDInfo[iCurrRDIdx]).skb == NULL) {
            if (!velocity_alloc_rx_buf(pInfo, iCurrRDIdx))
                break;
        }

        if (works++ > pInfo->hw.sOpts.int_works)
            break;

        if (pRD->rdesc0 & cpu_to_le32(RDESC0_OWN))
            break;

        pInfo->adwRMONStats[RMON_Octets] += (U16)((cpu_to_le32(pRD->rdesc0) >> 16) & 0x00003fffL);

        wRSR = (U16)(cpu_to_le32(pRD->rdesc0));

        // don't drop CE or RL error frame although RXOK is off
        if( (wRSR & RSR_RXOK) || (!(wRSR & RSR_RXOK) && (wRSR & (RSR_CE | RSR_RL)))||(bVIDM && (wRSR & RSR_VIDM))) {
            if (velocity_receive_frame(pInfo, iCurrRDIdx)) {
                if (!velocity_alloc_rx_buf(pInfo, iCurrRDIdx)) {
                    VELOCITY_PRT(msglevel, MSG_LEVEL_ERR, KERN_ERR
                    "%s: can not allocate rx buf\n", pInfo->dev->name);
                    break;
                }
            }
            else {
                pStats->rx_dropped++;
            }
        }
        else {
            if (wRSR & RSR_CRC)
                pStats->rx_crc_errors++;
            if (wRSR & RSR_FAE)
                pStats->rx_frame_errors++;

            pStats->rx_dropped++;
        }

        if ((iCurrRDIdx % 4) == 3) {
            int i, iPrevRDIdx = iCurrRDIdx;
            for (i=0; i<4; i++) {
                pRD = &(pInfo->hw.aRDRing[iPrevRDIdx]);
                pRD->rdesc0 |= cpu_to_le32(RDESC0_OWN);
                SUB_ONE_WITH_WRAP_AROUND(iPrevRDIdx, pInfo->hw.sOpts.nRxDescs);
            }

            if (pInfo->hw.byRevId >= REV_ID_VT3286_A0) {
                wRBRDU = CSR_READ_2(&pInfo->hw, MAC_REG_RBRDU);
                wCurrDeltaRBRDU = pInfo->hw.sOpts.nRxDescs - wRBRDU;
                if(4 <= wCurrDeltaRBRDU) {
                    CSR_WRITE_2(&pInfo->hw, 4, MAC_REG_RBRDU);
                }
                else {
                    CSR_WRITE_2(&pInfo->hw, wCurrDeltaRBRDU, MAC_REG_RBRDU);
                }
            }
            else {
                CSR_WRITE_2(&pInfo->hw, 4, MAC_REG_RBRDU);
            }
        }

        pInfo->dev->last_rx = jiffies;

        ADD_ONE_WITH_WRAP_AROUND(iCurrRDIdx, pInfo->hw.sOpts.nRxDescs);
    }

    pInfo->hw.iCurrRDIdx = iCurrRDIdx;
    VAR_USED(pStats);
    return works;
}

static inline void
velocity_rx_csum(PRX_DESC pRD, struct sk_buff* skb)
{
    U8  byCSM;

    skb->ip_summed = CHECKSUM_NONE;
    byCSM = (U8)(cpu_to_le32(pRD->rdesc1) >> 16);

    if ((byCSM & CSM_IPKT) && (byCSM & CSM_IPOK)) {
        if ((byCSM & CSM_TCPKT) || (byCSM & CSM_UDPKT)) {
            if (!(byCSM & CSM_TUPOK))
                return;
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        }
    }
}

static BOOL
velocity_receive_frame(PVELOCITY_INFO pInfo, int idx)
{
    PVELOCITY_RD_INFO   pRDInfo = &(pInfo->aRDInfo[idx]);
    PRX_DESC            pRD = &(pInfo->hw.aRDRing[idx]);
    struct sk_buff*     skb;
    U16                 wRSR, wLength;

    wRSR = (U16)(cpu_to_le32(pRD->rdesc0));
    //wLength = (U16)((cpu_to_le32(pRD->rdesc0) >> 16) & 0x00003fffL);
    wLength = VELOCITY_GET_RD_PACKET_SIZE(pRD);

    if (wRSR & (RSR_STP|RSR_EDP)) {
        VELOCITY_PRT(msglevel, MSG_LEVEL_VERBOSE, KERN_NOTICE "%s: the received frame span multple RDs\n", pInfo->dev->name);
        pInfo->stats.rx_length_errors++;
        return FALSE;
    }

    if (wRSR & RSR_MAR)
        pInfo->stats.multicast++;

    if (wRSR & RSR_BAR)
        pInfo->adwRMONStats[RMON_BroadcastPkts]++;

    skb = pRDInfo->skb;
    skb->dev = pInfo->dev;

    pci_unmap_single(pInfo->pcid,pRDInfo->skb_dma, pInfo->hw.rx_buf_sz, PCI_DMA_FROMDEVICE);
    pRDInfo->skb_dma = (dma_addr_t)NULL;
    pRDInfo->skb = NULL;

    if (pInfo->hw.flags & VELOCITY_FLAGS_IP_ALIGN) {
        int i;
        for (i = wLength; i >= 0; i--)
            *(skb->data + i + 2) = *(skb->data + i);
        skb->data += 2;
        skb->tail += 2;
    }

    skb_put(skb, (wLength-4));

    skb->protocol = eth_type_trans(skb, skb->dev);

    //drop frame not met IEEE 802.3
    if (pInfo->hw.flags & VELOCITY_FLAGS_VAL_PKT_LEN) {
        if (wRSR & RSR_RL) {
            pInfo->stats.rx_length_errors++;
            return FALSE;
        }
    }

    velocity_rx_csum(pRD, skb);
    pInfo->stats.rx_bytes += skb->len;
    netif_rx(skb);

    return TRUE;
}

static BOOL velocity_alloc_rx_buf(PVELOCITY_INFO pInfo, int idx) {
    PRX_DESC        pRD=&(pInfo->hw.aRDRing[idx]);
    PVELOCITY_RD_INFO  pRDInfo=&(pInfo->aRDInfo[idx]);

    pRDInfo->skb = dev_alloc_skb(pInfo->hw.rx_buf_sz+64);

    if (pRDInfo->skb == NULL)
        return FALSE;

    ASSERT(pRDInfo->skb);
    pRDInfo->skb->dev = pInfo->dev;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
    skb_reserve(pRDInfo->skb, 64 - ((unsigned long)skb_tail_pointer(pRDInfo->skb) & 63));
    pRDInfo->skb_dma=
        pci_map_single(pInfo->pcid, skb_tail_pointer(pRDInfo->skb), pInfo->hw.rx_buf_sz,
            PCI_DMA_FROMDEVICE);
#else
    skb_reserve(pRDInfo->skb, 64 - ((unsigned long)pRDInfo->skb->tail & 63));
    pRDInfo->skb_dma=
        pci_map_single(pInfo->pcid, pRDInfo->skb->tail, pInfo->hw.rx_buf_sz,
            PCI_DMA_FROMDEVICE);
#endif
    *((PU32)&(pRD->rdesc0)) = 0;

    VELOCITY_SET_RD_BUFFER_SIZE(pRD, pInfo->hw.rx_buf_sz);
    pRD->rdesc3 |= cpu_to_le32(RDESC3_INTCTL);
    pRD->dwBufAddrLo = cpu_to_le32(pRDInfo->skb_dma);
    // mask off RD data buffer address high to zero
    pRD->rdesc3 &= cpu_to_le32(0xffff0000L);

    return TRUE;
}

static int velocity_tx_srv(PVELOCITY_INFO pInfo, U32 status)
{
    PTX_DESC                    pTD;
    int                         iQNo;
    BOOL                        bFull = FALSE;
    int                         idx;
    int                         works = 0;
    PVELOCITY_TD_INFO           pTDInfo;
    struct net_device_stats*    pStats = &pInfo->stats;
    U16                         wTSR;


    for (iQNo=0; iQNo<pInfo->hw.nTxQueues; iQNo++) {
        for (idx = pInfo->hw.aiTailTDIdx[iQNo];
             pInfo->hw.iTDUsed[iQNo]>0;
             idx = (idx+1) % pInfo->hw.sOpts.nTxDescs)
        {
            // Get Tx Descriptor
            pTD = &(pInfo->hw.apTDRings[iQNo][idx]);
            pTDInfo = &(pInfo->apTDInfos[iQNo][idx]);

            if (pTD->tdesc0 & cpu_to_le32(TDESC0_OWN)) {
                break;
            }

            if (works++ > pInfo->hw.sOpts.int_works) {
                break;
            }

            wTSR = (U16)cpu_to_le32(pTD->tdesc0);

            if (wTSR & TSR0_TERR) {
                pStats->tx_errors++;
                pStats->tx_dropped++;
                if (wTSR & TSR0_CDH)
                    pStats->tx_heartbeat_errors++;
                if (wTSR & TSR0_CRS)
                    pStats->tx_carrier_errors++;
                if (wTSR & TSR0_ABT)
                    pStats->tx_aborted_errors++;
                if (wTSR & TSR0_OWC)
                    pStats->tx_window_errors++;

            }
            else {
                pStats->tx_packets++;
                pStats->tx_bytes += pTDInfo->skb->len;
            }

            velocity_free_tx_buf(pInfo, pTDInfo, pTD);
            pInfo->hw.iTDUsed[iQNo]--;
        } // for (idx)

        pInfo->hw.aiTailTDIdx[iQNo]=idx;

        if (AVAIL_TD(&pInfo->hw, iQNo) < 1 ) {
            bFull = TRUE;
        }
    } // for (iQNo)

    if (netif_queue_stopped(pInfo->dev) && (bFull==FALSE)
        && (!(pInfo->hw.mii_status & VELOCITY_LINK_FAIL))) {
        netif_wake_queue(pInfo->dev);
    }
    return works;
}



static void velocity_error(PVELOCITY_INFO pInfo, int status) {
    struct net_device* dev = pInfo->dev;

    // (1) LSTEI
    if (status & ISR_TXSTLI) {
        printk("TD structure errror TDindex=%X\n", CSR_READ_2(&pInfo->hw, MAC_REG_TDIDX0));

        BYTE_REG_BITS_ON(&pInfo->hw, TXESR_TDSTR, MAC_REG_TXE_SR);
        CSR_WRITE_2(&pInfo->hw, TRDCSR_RUN, MAC_REG_TDCSR_CLR);
        netif_stop_queue(dev);
    }

    // (2) SRCI
    if (status & ISR_SRCI) {

        if (pInfo->hw.sOpts.spd_dpx == SPD_DPX_AUTO) {
            pInfo->hw.mii_status = check_connectiontype(&pInfo->hw);

            // if it's 3119, disable frame bursting in halfduplex mode and
            // enable it in fullduplex mode
            if (pInfo->hw.byRevId < REV_ID_VT3216_A0) {
            	if (pInfo->hw.mii_status | VELOCITY_DUPLEX_FULL)
                    BYTE_REG_BITS_ON(&pInfo->hw, TCR_TB2BDIS, MAC_REG_TCR);
                else
                    BYTE_REG_BITS_OFF(&pInfo->hw, TCR_TB2BDIS, MAC_REG_TCR);
            }

            /*
             * only enable CD heart beat counter in 10HD mode
             * This PATCH is for VT3119/VT3216
             */
            if (pInfo->hw.byRevId < REV_ID_VT3284_A0) {
                if (!(pInfo->hw.mii_status & VELOCITY_DUPLEX_FULL) && (pInfo->hw.mii_status & VELOCITY_SPEED_10))
                    BYTE_REG_BITS_OFF(&pInfo->hw, TESTCFG_HBDIS, MAC_REG_TESTCFG);
                else
                    BYTE_REG_BITS_ON(&pInfo->hw, TESTCFG_HBDIS, MAC_REG_TESTCFG);
            }
            //--------------------------------------------------------
            // [1.18] Adaptive Interrupt
            if (pInfo->hw.byRevId >= REV_ID_VT3216_A0) {
                if (pInfo->hw.mii_status & VELOCITY_SPEED_1000) {
                    if (pInfo->hw.flags & VELOCITY_FLAGS_AI) {
                        CSR_WRITE_1(&pInfo->hw, pInfo->hw.sOpts.txque_timer, MAC_REG_TQETMR);
                        CSR_WRITE_1(&pInfo->hw, pInfo->hw.sOpts.rxque_timer, MAC_REG_RQETMR);
                    }
                    else {
                        CSR_WRITE_1(&pInfo->hw, 0x00, MAC_REG_TQETMR);
                        CSR_WRITE_1(&pInfo->hw, 0x00, MAC_REG_RQETMR);
                    }
                }
                else {
                    CSR_WRITE_1(&pInfo->hw, 0x00, MAC_REG_TQETMR);
                    CSR_WRITE_1(&pInfo->hw, 0x00, MAC_REG_RQETMR);
                }
            }
            //--------------------------------------------------------
        }

		// get link status from PHYSR0

        pInfo->hw.mii_status = check_connectiontype(&pInfo->hw);

        velocity_print_link_status(&pInfo->hw);
	    enable_flow_control_ability(&pInfo->hw);

		// re-enable auto-polling because SRCI will disable auto-polling
		EnableMiiAutoPoll(&pInfo->hw);

        if (pInfo->hw.mii_status & VELOCITY_LINK_FAIL) {
            netif_carrier_off(dev);
            netif_stop_queue(dev);
        }
        else {
            netif_carrier_on(dev);
            netif_wake_queue(dev);
        }
    } // ISR_SRCI

    // (3) MIBFI
    if (status & ISR_MIBFI)
        velocity_update_hw_mibs(&pInfo->hw);

    // (4) LSTEI: RD used up, re-wake RD ring
    if (status & ISR_LSTEI) {
        mac_rx_queue_wake(&pInfo->hw);
    }
}

static void velocity_free_tx_buf(PVELOCITY_INFO pInfo, PVELOCITY_TD_INFO pTDInfo, PTX_DESC pTD)
{
    struct sk_buff* skb = pTDInfo->skb;
    int             i;

#ifdef VELOCITY_ZERO_COPY_SUPPORT
    WORD    wBufSize;
#endif


    // Don't unmap the pre-allocaed tx_bufs
    if (pTDInfo->skb_dma && (pTDInfo->skb_dma[0] != pTDInfo->buf_dma))  {
        for (i=0; i<pTDInfo->nskb_dma; i++) {
#ifdef VELOCITY_ZERO_COPY_SUPPORT
            wBufSize = VELOCITY_GET_TD_BUFFER_SIZE(pTD->aTdBufs[i]);
            pci_unmap_single(pInfo->pcid, pTDInfo->skb_dma[i], wBufSize, PCI_DMA_TODEVICE);
#else
            pci_unmap_single(pInfo->pcid, pTDInfo->skb_dma[i], skb->len, PCI_DMA_TODEVICE);
#endif
        }
    }

    dev_kfree_skb_irq(skb);

    for (i=0; i<pTDInfo->nskb_dma; i++)
        pTDInfo->skb_dma[i] = 0;

    pTDInfo->skb = 0;
}

static int  velocity_open(struct net_device *dev) {
    //PVELOCITY_INFO pInfo=(PVELOCITY_INFO) dev->priv;
    PVELOCITY_INFO pInfo=netdev_priv(dev);
    int i;

    pInfo->hw.rx_buf_sz=(dev->mtu <= 1504 ? PKT_BUF_SZ : dev->mtu + 32);

    if (!velocity_init_rings(pInfo))
        return -ENOMEM;

    if (!velocity_init_rd_ring(pInfo))
        return -ENOMEM;

    if (!velocity_init_td_ring(pInfo))
        return -ENOMEM;

    velocity_init_pci(pInfo);

    velocity_init_adapter(pInfo, VELOCITY_INIT_COLD);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
    i=request_irq(pInfo->pcid->irq, (irq_handler_t)velocity_intr, IRQF_SHARED, dev->name, dev);
#else
    i=request_irq(pInfo->pcid->irq, &velocity_intr, SA_SHIRQ, dev->name, dev);
#endif

    if (i)
        return i;

    mac_enable_int(&pInfo->hw);

    netif_start_queue(dev);

    pInfo->hw.flags |=VELOCITY_FLAGS_OPENED;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    MOD_INC_USE_COUNT;
#endif

#ifdef CONFIG_PROC_FS
    velocity_create_proc_entry(pInfo);
#endif
    return 0;
}

static int  velocity_change_mtu(struct net_device *dev,int new_mtu) {
    //PVELOCITY_INFO pInfo = (PVELOCITY_INFO)dev->priv;
    PVELOCITY_INFO pInfo = netdev_priv(dev);
    unsigned long    flags;
    int     oldmtu = dev->mtu;

    if ((new_mtu<VELOCITY_MIN_MTU)||new_mtu>(VELOCITY_MAX_MTU)) {
        VELOCITY_PRT(msglevel, MSG_LEVEL_ERR, KERN_NOTICE
                "%s: Invaild MTU\n",pInfo->dev->name);
        return -EINVAL;
    }

    if (new_mtu!=oldmtu) {
        spin_lock_irqsave(&pInfo->lock, flags);

        netif_stop_queue(dev);
        velocity_shutdown(&pInfo->hw);

        velocity_free_td_ring(pInfo);
        velocity_free_rd_ring(pInfo);

        dev->mtu=new_mtu;
        if (new_mtu>8192)
            pInfo->hw.rx_buf_sz=9*1024;
        else if (new_mtu>4096)
            pInfo->hw.rx_buf_sz=8192;
        else
            pInfo->hw.rx_buf_sz=4*1024;

        if (!velocity_init_rd_ring(pInfo))
            return -ENOMEM;

        if (!velocity_init_td_ring(pInfo))
            return -ENOMEM;

        velocity_init_adapter(pInfo, VELOCITY_INIT_COLD);

        mac_enable_int(&pInfo->hw);
        netif_start_queue(dev);
        spin_unlock_irqrestore(&pInfo->lock, flags);
    }

    return 0;
}

static int  velocity_close(struct net_device *dev) {
    PVELOCITY_INFO pInfo= netdev_priv(dev);

#ifdef CONFIG_PROC_FS
    velocity_free_proc_entry(pInfo);
    #endif

    netif_stop_queue(dev);

    velocity_shutdown(&pInfo->hw);

    if (pInfo->hw.flags & VELOCITY_FLAGS_WOL_ENABLED)
        velocity_get_ip(pInfo);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    MOD_DEC_USE_COUNT;
#endif

    if (dev->irq!=0)
        free_irq(dev->irq, dev);

    velocity_free_td_ring(pInfo);
    velocity_free_rd_ring(pInfo);
    velocity_free_rings(pInfo);

    pInfo->hw.flags &=(~VELOCITY_FLAGS_OPENED);
    return 0;
}


static int velocity_xmit(struct sk_buff *skb, struct net_device *dev)
{
    PVELOCITY_INFO      pInfo = netdev_priv(dev);
    int                 iQNo = 0;
    PTX_DESC            pTD;
    PVELOCITY_TD_INFO   pTDInfo;
    unsigned long       flags;
    int                 iCurrTDIdx;
    int                 iPrevTDIdx;

#ifdef VELOCITY_ZERO_COPY_SUPPORT
    unsigned int        nfrags = skb_shinfo(skb)->nr_frags;
#endif

#ifdef VELOCITY_TSO_SUPPORT
    unsigned int        mss = tcp_skb_mss(skb);
#endif


    spin_lock_irqsave(&pInfo->lock, flags);

    iCurrTDIdx = pInfo->hw.aiCurrTDIdx[iQNo];
    pTD = &(pInfo->hw.apTDRings[iQNo][iCurrTDIdx]);
    pTDInfo = &(pInfo->apTDInfos[iQNo][iCurrTDIdx]);

    // Init TDESC0,1
    pTD->tdesc0 = 0x00000000UL;
    pTD->tdesc1 = 0x00000000UL;

    VELOCITY_SET_TD_TCPLS(pTD, TCPLS_NORMAL);
    pTD->tdesc1 |= cpu_to_le32(TCR_TIC);
    pTD->aTdBufs[0].dwBufaddrHi &= cpu_to_le32(~TDTXBUF_QUE);

    if (skb->len < ETH_ZLEN) {
        // packet size is less than 60 bytes

#ifndef SUSE_10_2
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        skb_linearize(skb);
#else
        skb_linearize(skb, GFP_ATOMIC);
#endif
#else
	skb_linearize(skb);
#endif

        memcpy(pTDInfo->buf, skb->data, skb->len);
        VELOCITY_SET_TD_PACKET_SIZE(pTD, ETH_ZLEN);

        // padding zero
        memset(pTDInfo->buf+skb->len, 0, ETH_ZLEN-skb->len);

        pTDInfo->skb = skb;

        pTDInfo->skb_dma[0] = pTDInfo->buf_dma;
        pTD->aTdBufs[0].dwBufAddrLo = cpu_to_le32(pTDInfo->skb_dma[0]);
        // mask off TD data buffer address high to zero
        pTD->aTdBufs[0].dwBufaddrHi &= cpu_to_le32(0xffff0000L);

        VELOCITY_SET_TD_BUFFER_SIZE(pTD->aTdBufs[0], ETH_ZLEN);
        pTDInfo->nskb_dma = 1;
        VELOCITY_SET_TD_CMDZ(pTD, 2);
    }
    else
#ifdef VELOCITY_ZERO_COPY_SUPPORT
        if (nfrags > 0) {
            pTDInfo->skb = skb;

            if (nfrags > 6) {
                // elements > 7 ==> copy to a buffer

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
                skb_linearize(skb);
#else
                skb_linearize(skb, GFP_ATOMIC);
#endif

                memcpy(pTDInfo->buf, skb->data, skb->len);
                VELOCITY_SET_TD_PACKET_SIZE(pTD, skb->len);

                pTDInfo->skb_dma[0] = pTDInfo->buf_dma;
                pTD->aTdBufs[0].dwBufAddrLo = cpu_to_le32(pTDInfo->skb_dma[0]);
                // mask off TD data buffer address high to zero
                pTD->aTdBufs[0].dwBufaddrHi &= cpu_to_le32(0xffff0000L);

                VELOCITY_SET_TD_BUFFER_SIZE(pTD->aTdBufs[0], skb->len);
                pTDInfo->nskb_dma = 1;
                VELOCITY_SET_TD_CMDZ(pTD, 2);
            }
            else {
                // elements <= 7
                int         i = 0;

                pTDInfo->nskb_dma = 0;
                VELOCITY_SET_TD_PACKET_SIZE(pTD, skb->len);

                pTDInfo->skb_dma[i] = pci_map_single(pInfo->pcid, skb->data, skb->len - skb->data_len, PCI_DMA_TODEVICE);
                pTD->aTdBufs[i].dwBufAddrLo = cpu_to_le32(pTDInfo->skb_dma[i]);
                // mask off TD data buffer address high to zero
                pTD->aTdBufs[i].dwBufaddrHi &= cpu_to_le32(0xffff0000L);

                VELOCITY_SET_TD_BUFFER_SIZE(pTD->aTdBufs[i], skb->len - skb->data_len);

                for (i=0; i<nfrags; i++) {
                    skb_frag_t  *frag;
                    void*       addr;

                    frag = &skb_shinfo(skb)->frags[i];
                    addr = ((void *)page_address(frag->page + frag->page_offset));

                    pTDInfo->skb_dma[i+1] = pci_map_single(pInfo->pcid, addr, frag->size, PCI_DMA_TODEVICE);
                    pTD->aTdBufs[i+1].dwBufAddrLo = cpu_to_le32(pTDInfo->skb_dma[i+1]);
                    // mask off TD data buffer address high to zero
                    pTD->aTdBufs[i+1].dwBufaddrHi &= cpu_to_le32(0xffff0000L);

                    VELOCITY_SET_TD_BUFFER_SIZE(pTD->aTdBufs[i+1], frag->size);
                }

                pTDInfo->nskb_dma = i + 1;
                VELOCITY_SET_TD_CMDZ(pTD, i + 2);
            }
        }
        else
#endif
    {
        pTDInfo->skb = skb;
        pTDInfo->skb_dma[0] = pci_map_single(pInfo->pcid, skb->data, skb->len, PCI_DMA_TODEVICE);

        VELOCITY_SET_TD_PACKET_SIZE(pTD, skb->len);
        pTD->aTdBufs[0].dwBufAddrLo = cpu_to_le32(pTDInfo->skb_dma[0]);
        // mask off TD data buffer address high to zero
        pTD->aTdBufs[0].dwBufaddrHi &= cpu_to_le32(0xffff0000L);
        VELOCITY_SET_TD_BUFFER_SIZE(pTD->aTdBufs[0], skb->len);
        pTDInfo->nskb_dma = 1;
        VELOCITY_SET_TD_CMDZ(pTD, 2);
    }

    if (pInfo->hw.sOpts.tagging == 1) {
        // clear CFI and priority
        pTD->tdesc1 &= cpu_to_le32(0xffff0000L);
        VELOCITY_SET_TD_VLANID(pTD, pInfo->hw.sOpts.vid & 0xfff);
        pTD->tdesc1 |= cpu_to_le32(TCR_VETAG);
    }


#ifdef VELOCITY_TX_CSUM_SUPPORT
    if (pInfo->hw.flags & VELOCITY_FLAGS_TX_CSUM) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
        if (skb->ip_summed == CHECKSUM_PARTIAL) {
#else
        if (skb->ip_summed == CHECKSUM_HW) {
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
            struct iphdr* ip = (struct iphdr*)skb_network_header(skb);
#else
            struct iphdr* ip = skb->nh.iph;
#endif

            if (ip->protocol == IPPROTO_TCP) {
                // request TCP checksum calculation
               pTD->tdesc1 |= cpu_to_le32(TCR_TCPCK);
            }
            else if (ip->protocol == IPPROTO_UDP) {
                // request UDP checksum calculation
                pTD->tdesc1 |= cpu_to_le32(TCR_UDPCK);
            }

            // request IP checksum calculation
            pTD->tdesc1 |= cpu_to_le32(TCR_IPCK);
        }
    }
#endif

    // Set OWN bit of current TD
    pTD->tdesc0 |= cpu_to_le32(TDESC0_OWN);

    pInfo->hw.iTDUsed[iQNo]++;
    pInfo->hw.aiCurrTDIdx[iQNo] = (iCurrTDIdx + 1) % pInfo->hw.sOpts.nTxDescs;

    if (AVAIL_TD(&pInfo->hw, iQNo) < 1)
        netif_stop_queue(dev);

    iPrevTDIdx = (iCurrTDIdx + pInfo->hw.sOpts.nTxDescs - 1) % pInfo->hw.sOpts.nTxDescs;
    pTD = &(pInfo->hw.apTDRings[iQNo][iPrevTDIdx]);
    pTD->aTdBufs[0].dwBufaddrHi |= cpu_to_le32(TDTXBUF_QUE);

    mac_tx_queue_wake(&pInfo->hw, iQNo);

    dev->trans_start = jiffies;

    spin_unlock_irqrestore(&pInfo->lock, flags);
    return 0;
}

static irqreturn_t velocity_intr(int irq, void *dev_instance, struct pt_regs *regs)
{
    struct net_device*  dev = dev_instance;
    PVELOCITY_INFO      pInfo = netdev_priv(dev);
    U32                 isr_status;
    int                 max_count = 0;
    int                 handled = 0;


    if (!spin_trylock(&pInfo->lock))
        return IRQ_RETVAL(handled);

    isr_status = mac_read_isr(&pInfo->hw);

    if (isr_status == 0) {
        spin_unlock(&pInfo->lock);
        return IRQ_RETVAL(handled);
    }

    handled = 1;
    mac_disable_int(&pInfo->hw);

    //while (isr_status != 0) {
        mac_write_isr(&pInfo->hw, isr_status);

        velocity_error(pInfo, isr_status);

        max_count += velocity_rx_srv(pInfo, isr_status);
        max_count += velocity_tx_srv(pInfo, isr_status);

        // [1.18], for performance
        max_count += velocity_rx_srv(pInfo, isr_status);
        max_count += velocity_tx_srv(pInfo, isr_status);

        //isr_status = mac_read_isr(&pInfo->hw);

        //if (max_count > pInfo->hw.sOpts.int_works)
        //    break;
    //}

    mac_enable_int(&pInfo->hw);

    spin_unlock(&pInfo->lock);
    return IRQ_RETVAL(handled);
}

static unsigned const ethernet_polynomial = 0x04c11db7U;
static inline u32 ether_crc(int length, unsigned char *data)
{
    int crc = -1;

    while(--length >= 0) {
        unsigned char current_octet = *data++;
        int bit;
        for (bit = 0; bit < 8; bit++, current_octet >>= 1) {
            crc = (crc << 1) ^
                ((crc < 0) ^ (current_octet & 1) ? ethernet_polynomial : 0);
        }
    }
    return crc;
}


static void velocity_set_multi(struct net_device *dev)
{
    PVELOCITY_INFO      pInfo       =   netdev_priv(dev);
    U8                  rx_mode;
    int                 i;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
    struct netdev_hw_addr *ha;
#else
	struct dev_mc_list  *mclist;
#endif

    if (dev->flags & IFF_PROMISC) {         // Set promiscuous.
        // Unconditionally log net taps.
        printk(KERN_NOTICE "%s: Promiscuous mode enabled.\n", dev->name);
        velocity_set_all_multi(&pInfo->hw);
        rx_mode = (RCR_AM|RCR_AB|RCR_PROM);
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
    else if ((netdev_mc_count(dev) > pInfo->hw.multicast_limit) ||  (dev->flags & IFF_ALLMULTI)) {
#else
	else if ((dev->mc_count > pInfo->hw.multicast_limit) ||  (dev->flags & IFF_ALLMULTI)) {
#endif
        velocity_set_all_multi(&pInfo->hw);
        rx_mode = (RCR_AM|RCR_AB);
    }
    else {
        int offset=MCAM_SIZE-pInfo->hw.multicast_limit;

        // Clear multicast address registers
        velocity_clear_all_multi(&pInfo->hw);

        mac_get_cam_mask(&pInfo->hw,pInfo->hw.abyMCAMMask,VELOCITY_MULTICAST_CAM);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
		i = 0;
        netdev_for_each_mc_addr(ha, dev) {
            mac_set_cam(&pInfo->hw,i+offset,ha->addr,VELOCITY_MULTICAST_CAM);
            pInfo->hw.abyMCAMMask[(offset+i)/8]|=1<<((offset+i) & 7);
			i++;
        }
#else
		for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count; i++, mclist = mclist->next) {
			mac_set_cam(&pInfo->hw,i+offset,mclist->dmi_addr,VELOCITY_MULTICAST_CAM);
			pInfo->hw.abyMCAMMask[(offset+i)/8]|=1<<((offset+i) & 7);
}
#endif

        mac_set_cam_mask(&pInfo->hw,pInfo->hw.abyMCAMMask,VELOCITY_MULTICAST_CAM);

        rx_mode=(RCR_AP|RCR_AM|RCR_AB);
    }

    if (dev->mtu>1500)
        rx_mode|=RCR_AL;

    BYTE_REG_BITS_ON(&pInfo->hw, rx_mode, MAC_REG_RCR);
}


static struct net_device_stats *velocity_get_stats(struct net_device *dev)
{
    PVELOCITY_INFO  pInfo = netdev_priv(dev);

    spin_lock_irq(&pInfo->lock);
    velocity_update_hw_mibs(&pInfo->hw);
    spin_unlock_irq(&pInfo->lock);

    pInfo->stats.rx_packets=pInfo->hw.adwHWMIBCounters[HW_MIB_ifRxAllPkts];
    pInfo->stats.rx_errors=pInfo->hw.adwHWMIBCounters[HW_MIB_ifRxErrorPkts];
    pInfo->stats.rx_length_errors=pInfo->hw.adwHWMIBCounters[HW_MIB_ifInRangeLengthErrors];

    pInfo->stats.collisions=pInfo->hw.adwHWMIBCounters[HW_MIB_ifTxEtherCollisions];
    // detailed rx_errors:
    pInfo->stats.rx_crc_errors=pInfo->hw.adwHWMIBCounters[HW_MIB_ifRxPktCRCE];

    // detailed tx_errors

    return &pInfo->stats;
}


static int velocity_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    switch(cmd) {

#ifdef VELOCITY_ETHTOOL_IOCTL_SUPPORT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,25)
    case SIOCETHTOOL:
        return velocity_ethtool_ioctl(dev, rq);
        break;
#endif
#endif

#ifdef VELOCITY_MII_IOCTL_SUPPORT
    case SIOCGMIIPHY:       /* Get address of MII PHY in use. */
    case SIOCGMIIREG:       /* Read MII PHY register. */
    case SIOCSMIIREG:       /* Write to MII PHY register. */
        return velocity_mii_ioctl(dev, rq, cmd);
        break;
#endif

    default:
        return -EOPNOTSUPP;
    }
    return 0;
}



/*------------------------------------------------------------------*/

MODULE_DEVICE_TABLE(pci, velocity_id_table);

static struct pci_driver velocity_driver = {
        name:       VELOCITY_NAME,
        id_table:   velocity_id_table,
        probe:      velocity_found1,
        remove:     velocity_remove1,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,9)
#ifdef CONFIG_PM
        suspend:    velocity_suspend,
        resume:     velocity_resume,
#endif
#endif
};

static int __init velocity_init_module(void)
{
    int ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
    ret=pci_register_driver(&velocity_driver);
#else
    ret=pci_module_init(&velocity_driver);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,9)
#ifdef CONFIG_PM
    register_inetaddr_notifier(&velocity_inetaddr_notifier);
    if(ret >= 0)
        register_reboot_notifier(&velocity_notifier);

#endif
#endif

#ifdef CONFIG_PROC_FS
    velocity_init_proc_fs();
#endif
    return ret;
}

static void __exit velocity_cleanup_module(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,9)
#ifdef CONFIG_PM
    unregister_reboot_notifier(&velocity_notifier);
    unregister_inetaddr_notifier(&velocity_inetaddr_notifier);
#endif
#endif
    pci_unregister_driver(&velocity_driver);

#ifdef CONFIG_PROC_FS
    velocity_free_proc_fs();
#endif
}

module_init(velocity_init_module);
module_exit(velocity_cleanup_module);


/***************************************************************************
*    ETHTOOL ioctl support routine
****************************************************************************/
#ifdef VELOCITY_ETHTOOL_IOCTL_SUPPORT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,25)
static int velocity_ethtool_ioctl(struct net_device* dev, struct ifreq* ifr)
{
    struct ethtool_cmd  ecmd;
    PVELOCITY_INFO      pInfo = netdev_priv(dev);

    if (copy_from_user(&ecmd, ifr->ifr_data, sizeof(ecmd.cmd)))
        return -EFAULT;

    switch (ecmd.cmd) {
    case ETHTOOL_GSET: {
        velocity_ethtool_gset(dev, &ecmd);
        if(copy_to_user(ifr->ifr_data, &ecmd, sizeof(ecmd)))
            return -EFAULT;
        }
        break;

    case ETHTOOL_SSET: {
	velocity_ethtool_sset(dev, &ecmd);
        }
        break;

#ifdef ETHTOOL_GLINK
    case ETHTOOL_GLINK: {
        struct ethtool_value    info;
        memset((void *)&info, 0, sizeof(info));
        info.cmd = ETHTOOL_GLINK;
        info.data = velocity_ethtool_glink(dev);

        if (copy_to_user(ifr->ifr_data, &info, sizeof(info)))
            return -EFAULT;
        }
        break;
#endif

#ifdef ETHTOOL_GDRVINFO
    case ETHTOOL_GDRVINFO: {
        struct ethtool_drvinfo  info = {ETHTOOL_GDRVINFO};
	memset(&info, 0, sizeof(info));
	velocity_ethtool_gdrvinfo(dev, &info);
        if (copy_to_user(ifr->ifr_data, &info, sizeof(info)))
            return -EFAULT;
        }
        break;
#endif

#ifdef ETHTOOL_GWOL
    case ETHTOOL_GWOL: {
        struct ethtool_wolinfo  wol = {ETHTOOL_GWOL};
        memset(&wol, 0, sizeof(wol));
	velocity_ethtool_gwol(dev, &wol);
        if (copy_to_user(ifr->ifr_data, &wol, sizeof(wol)))
            return -EFAULT;
    }
    break;
#endif

#ifdef ETHTOOL_SWOL
    case ETHTOOL_SWOL: {
        struct ethtool_wolinfo  wol;
        memset(&wol, 0, sizeof(wol));
        if (copy_from_user(&wol, ifr->ifr_data, sizeof(wol)))
            return -EFAULT;

	if(velocity_ethtool_swol(dev, &wol) != 0)
            return -EFAULT;

        if (copy_to_user(ifr->ifr_data, &wol, sizeof(wol)))
            return -EFAULT;
    }
    break;
#endif

#ifdef ETHTOOL_GMSGLVL
    case ETHTOOL_GMSGLVL: {
        struct ethtool_value    edata = {ETHTOOL_GMSGLVL};
        edata.data = velocity_ethtool_gmsglvl(dev);

        if (copy_to_user(ifr->ifr_data, &edata, sizeof(edata)))
            return -EFAULT;
    }
    break;
#endif

#ifdef ETHTOOL_SMSGLVL
    case ETHTOOL_SMSGLVL: {
        struct ethtool_value edata={ETHTOOL_SMSGLVL};

        if (copy_from_user(&edata, ifr->ifr_data, sizeof(edata)))
            return -EFAULT;
	velocity_ethtool_smsglvl(dev, edata.data);
    }
    break;
#endif

    default:
        return -EOPNOTSUPP;
    }

    return 0;
}
#else
static struct ethtool_ops velocity_ethtool_ops = {
    .get_drvinfo    =   velocity_ethtool_gdrvinfo,
    .get_settings    =   velocity_ethtool_gset,
    .set_settings    =   velocity_ethtool_sset,
    .get_link       =   velocity_ethtool_glink,
    .get_msglevel   =   velocity_ethtool_gmsglvl,
    .set_msglevel   =   velocity_ethtool_smsglvl,
    .get_wol        =   velocity_ethtool_gwol,
    .set_wol        =   velocity_ethtool_swol,
};
#endif

static int velocity_ethtool_gset(struct net_device*  dev, struct ethtool_cmd *ecmd)
{
    PVELOCITY_INFO pInfo = netdev_priv(dev);
    U32 status=check_connectiontype(&pInfo->hw);

    ecmd->supported = SUPPORTED_TP  |
                    SUPPORTED_Autoneg   |
                    SUPPORTED_10baseT_Half  |
                    SUPPORTED_10baseT_Full  |
                    SUPPORTED_100baseT_Half  |
                    SUPPORTED_100baseT_Full |
                    SUPPORTED_1000baseT_Full;

    ecmd->autoneg = AUTONEG_ENABLE;
    ecmd->advertising |= ADVERTISED_Autoneg;
    ecmd->advertising |= ADVERTISED_TP;

    if (pInfo->hw.sOpts.spd_dpx == SPD_DPX_AUTO) {
        ecmd->advertising |= (ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
                              ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
                              ADVERTISED_1000baseT_Full);
    }
    else {
        switch(pInfo->hw.sOpts.spd_dpx) {
        case SPD_DPX_1000_FULL:
            ecmd->advertising |= ADVERTISED_1000baseT_Full;
            break;
        case SPD_DPX_100_HALF:
            ecmd->advertising |= ADVERTISED_100baseT_Half;
            break;
        case SPD_DPX_100_FULL:
            ecmd->advertising |= ADVERTISED_100baseT_Full;
            break;
        case SPD_DPX_10_HALF:
            ecmd->advertising |= ADVERTISED_10baseT_Half;
            break;
        case SPD_DPX_10_FULL:
            ecmd->advertising |= ADVERTISED_10baseT_Full;
            break;
        default:
            break;
        }
    }

    if(status & VELOCITY_LINK_FAIL) {
        ecmd->speed = -1;
        ecmd->duplex = -1;
    }
    else {
        if(status & VELOCITY_SPEED_1000)
           ecmd->speed = SPEED_1000;
        else
           ecmd->speed = (status & VELOCITY_SPEED_100) ? SPEED_100 : SPEED_10;

        ecmd->duplex = (status & VELOCITY_DUPLEX_FULL) ? DUPLEX_FULL : DUPLEX_HALF;
    }

    ecmd->port = PORT_TP;
    ecmd->transceiver = XCVR_INTERNAL;
    ecmd->phy_address = CSR_READ_1(&pInfo->hw, MAC_REG_MIIADR) & 0x1F;

    return 0;
}

static int velocity_ethtool_sset(struct net_device*  dev, struct ethtool_cmd *ecmd)
{
    PVELOCITY_INFO pInfo = netdev_priv(dev);
    U32 advertising;
    U32 diff_status;
    U32 curr_status;
    U32 new_status=0;

    if (!capable(CAP_NET_ADMIN)) {
        return -EPERM;
    }

    curr_status=check_connectiontype(&pInfo->hw);
    curr_status&=(~VELOCITY_LINK_FAIL);

    new_status|=((ecmd->autoneg) ? VELOCITY_AUTONEG_ENABLE : 0);
    new_status|=((ecmd->speed==SPEED_1000) ? VELOCITY_SPEED_1000 : 0);
    new_status|=((ecmd->speed==SPEED_100) ? VELOCITY_SPEED_100 : 0);
    new_status|=((ecmd->speed==SPEED_10) ? VELOCITY_SPEED_10 : 0);
    new_status|=((ecmd->duplex==DUPLEX_FULL) ? VELOCITY_DUPLEX_FULL : 0);

    if (!(new_status & VELOCITY_AUTONEG_ENABLE))
        return -EOPNOTSUPP;

    /* preserve the difference from the status */
    diff_status = new_status ^ curr_status;

    /* filter un-supported mode */
    if (ecmd->advertising & ~(ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
                              ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
                              ADVERTISED_1000baseT_Full |
                              ADVERTISED_Autoneg | ADVERTISED_TP))
        return -EOPNOTSUPP;

    advertising = ecmd->advertising & (ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
                                  ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
                                  ADVERTISED_1000baseT_Full);

    if (advertising == (ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
                        ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
                                                    ADVERTISED_1000baseT_Full)) {

        /* we don't provide changing speed or duplex mode with auto negotiation */
        if (diff_status & (VELOCITY_SPEED_1000 | VELOCITY_SPEED_100 | VELOCITY_SPEED_10))
            return -EINVAL;

        pInfo->hw.sOpts.spd_dpx = SPD_DPX_AUTO;
    }
    else{
        if ((new_status & VELOCITY_SPEED_1000) && (new_status & VELOCITY_DUPLEX_FULL)) {
            pInfo->hw.sOpts.spd_dpx = SPD_DPX_1000_FULL;
        }
        else if (new_status & VELOCITY_SPEED_100) {
            pInfo->hw.sOpts.spd_dpx = (new_status & VELOCITY_DUPLEX_FULL) ? SPD_DPX_100_FULL : SPD_DPX_100_HALF;
        }
        else if (new_status & VELOCITY_SPEED_10){
            pInfo->hw.sOpts.spd_dpx = (new_status & VELOCITY_DUPLEX_FULL) ? SPD_DPX_10_FULL : SPD_DPX_10_HALF;
        }
        else {
            return -EOPNOTSUPP;
        }
    }

    velocity_set_media_mode(&pInfo->hw, pInfo->hw.sOpts.spd_dpx);

    return 0;
}

static u32 velocity_ethtool_glink(struct net_device*  dev)
{
    PVELOCITY_INFO pInfo = netdev_priv(dev);
    int    link_ok = BYTE_REG_BITS_IS_ON(&pInfo->hw, PHYSR0_LINKGD,MAC_REG_PHYSR0) ? 1 : 0;
    return link_ok;
}

static void velocity_ethtool_gdrvinfo(struct net_device*  dev, struct ethtool_drvinfo  *info)
{
    PVELOCITY_INFO pInfo = netdev_priv(dev);
    strcpy(info->driver, VELOCITY_NAME);
    strcpy(info->version, VELOCITY_VERSION);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
    strcpy(info->bus_info, pci_name(pInfo->pcid));
#else
    strcpy(info->bus_info, pInfo->pcid->slot_name);
#endif
}

static void velocity_ethtool_gwol(struct net_device*  dev, struct ethtool_wolinfo *wol)
{
    PVELOCITY_INFO pInfo = netdev_priv(dev);

    wol->supported=WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_ARP;
    wol->wolopts|=WAKE_MAGIC;
    if (pInfo->wol_opts & VELOCITY_WOL_PHY)
        wol->wolopts|=WAKE_PHY;
    if (pInfo->wol_opts & VELOCITY_WOL_UCAST)
        wol->wolopts|=WAKE_UCAST;
    if (pInfo->wol_opts & VELOCITY_WOL_ARP)
        wol->wolopts|=WAKE_ARP;
    memcpy(&wol->sopass,pInfo->wol_passwd,6);
}

static int velocity_ethtool_swol(struct net_device*  dev, struct ethtool_wolinfo *wol)
{
    PVELOCITY_INFO pInfo = netdev_priv(dev);

    if (!(wol->wolopts & (WAKE_PHY|WAKE_MAGIC|WAKE_UCAST|WAKE_ARP)))
        return -EFAULT;

    pInfo->wol_opts=VELOCITY_WOL_MAGIC;
    if (wol->wolopts & WAKE_PHY) {
        pInfo->wol_opts|=VELOCITY_WOL_PHY;
        pInfo->hw.flags |=VELOCITY_FLAGS_WOL_ENABLED;
    }
    if (wol->wolopts & WAKE_MAGIC) {
        pInfo->wol_opts|=VELOCITY_WOL_MAGIC;
        pInfo->hw.flags |=VELOCITY_FLAGS_WOL_ENABLED;
    }
    if (wol->wolopts & WAKE_UCAST) {
        pInfo->wol_opts|=VELOCITY_WOL_UCAST;
        pInfo->hw.flags |=VELOCITY_FLAGS_WOL_ENABLED;
    }
    if (wol->wolopts & WAKE_ARP) {
        pInfo->wol_opts|=VELOCITY_WOL_ARP;
        pInfo->hw.flags |=VELOCITY_FLAGS_WOL_ENABLED;
    }
    memcpy(pInfo->wol_passwd,wol->sopass,6);

    return 0;
}

static u32 velocity_ethtool_gmsglvl(struct net_device*  dev)
{
    return msglevel;
}

static void velocity_ethtool_smsglvl(struct net_device*  dev, u32 value)
{
    msglevel = value;
}


#endif // VELOCITY_ETHTOOL_IOCTL_SUPPORT


/***************************************************************************
*    MII ioctl support routine
****************************************************************************/
#ifdef VELOCITY_MII_IOCTL_SUPPORT
static int velocity_mii_ioctl(struct net_device* dev, struct ifreq* ifr, int cmd)
{
    PVELOCITY_INFO          pInfo = netdev_priv(dev);
    //PMAC_REGS               pMacRegs = pInfo->pMacRegs;
    unsigned long           flags;
    struct mii_ioctl_data*  pMiiData = (struct mii_ioctl_data*)&(ifr->ifr_data);

    switch(cmd) {
    case SIOCGMIIPHY:
        pMiiData->phy_id=CSR_READ_1(&pInfo->hw, MAC_REG_MIIADR) & 0x1f;
        break;

    case SIOCGMIIREG:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;
        velocity_mii_read(&pInfo->hw,pMiiData->reg_num & 0x1f, &(pMiiData->val_out));
        break;

    case SIOCSMIIREG:
        if (!capable(CAP_NET_ADMIN))
            return -EPERM;
        spin_lock_irqsave(&pInfo->lock, flags);
        velocity_mii_write(&pInfo->hw,pMiiData->reg_num & 0x1f,pMiiData->val_in);
        spin_unlock_irqrestore(&pInfo->lock,flags);
        pInfo->hw.mii_status = check_connectiontype(&pInfo->hw);
        break;

    default:
        return -EOPNOTSUPP;
    }

    return 0;
}
#endif // VELOCITY_MII_IOCTL_SUPPORT


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,9)
#ifdef CONFIG_PM
static int
velocity_notify_reboot(struct notifier_block *nb, unsigned long event, void *p)
{
    struct pci_dev *pcid = NULL;

    switch(event) {
    case SYS_DOWN:
    case SYS_HALT:
    case SYS_POWER_OFF:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
        while ((pcid = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcid)) != NULL) {
#else
        pci_for_each_dev(pcid) {
#endif
            if(pci_dev_driver(pcid) == &velocity_driver) {
                if (pci_get_drvdata(pcid))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
                    velocity_suspend(pcid, PMSG_SUSPEND);
#else
                    velocity_suspend(pcid, PCI_D3hot);
#endif
            }
        }
    }
    return NOTIFY_DONE;
}


static int
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
velocity_suspend(struct pci_dev *pcid, pm_message_t state)
#else
velocity_suspend(struct pci_dev *pcid, u32 state)
#endif
{
    PVELOCITY_INFO      pInfo = pci_get_drvdata(pcid);
    struct net_device   *dev = pInfo->dev;
    unsigned long       flags;
    int power_status;       // to silence the compiler

    netif_stop_queue(dev);
    spin_lock_irqsave(&pInfo->lock, flags);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
    pci_save_state(pcid);
#else
    pci_save_state(pcid, pInfo->pci_state);
#endif

#ifdef ETHTOOL_GWOL
    if (pInfo->hw.flags & VELOCITY_FLAGS_WOL_ENABLED) {
        velocity_get_ip(pInfo);
        velocity_save_mac_context(&pInfo->hw, &pInfo->mac_context);
        velocity_save_pci_context(pInfo, &pInfo->pci_context);
        velocity_shutdown(&pInfo->hw);
        velocity_set_wol(pInfo);
        power_status = pci_enable_wake(pcid, PCI_D3hot, 1);
        power_status = pci_set_power_state(pcid, PCI_D3hot);
    }
    else {
        velocity_save_mac_context(&pInfo->hw, &pInfo->mac_context);
        velocity_save_pci_context(pInfo, &pInfo->pci_context);
        velocity_shutdown(&pInfo->hw);
        pci_disable_device(pcid);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
        power_status = pci_set_power_state(pcid, pci_choose_state(pcid, state));
#else
        power_status = pci_set_power_state(pcid, state);
#endif
    }

#else  // !ETHTOOL_GWOL
    pci_disable_device(pcid);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
    power_status = pci_set_power_state(pcid, pci_choose_state(pcid, state));
#else
    power_status = pci_set_power_state(pcid, state);
#endif

#endif

    spin_unlock_irqrestore(&pInfo->lock, flags);
    return 0;
}


static int
velocity_resume(struct pci_dev *pcid)
{
    PVELOCITY_INFO      pInfo = pci_get_drvdata(pcid);
    struct net_device   *dev = pInfo->dev;
    unsigned long       flags;
    int power_status;   // to silence the compiler

    power_status = pci_set_power_state(pcid, PCI_D0);
    power_status = pci_enable_wake(pcid, PCI_D0, 0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
    pci_restore_state(pcid);
#else
    pci_restore_state(pcid, pInfo->pci_state);
#endif

    mac_wol_reset(&pInfo->hw);

    if (netif_running(dev)) {
        int i;
        spin_lock_irqsave(&pInfo->lock, flags);
        velocity_restore_mac_context(&pInfo->hw, &pInfo->mac_context);
        velocity_restore_pci_context(pInfo, pInfo->pci_context);
        velocity_init_adapter(pInfo, VELOCITY_INIT_WOL);
        mac_disable_int(&pInfo->hw);

        velocity_tx_srv(pInfo, 0);

        for (i=0;i<pInfo->hw.nTxQueues;i++) {
            if (pInfo->hw.iTDUsed[i]) {
                mac_tx_queue_wake(&pInfo->hw, i);
            }
        }

        mac_enable_int(&pInfo->hw);
        netif_start_queue(dev);
        spin_unlock_irqrestore(&pInfo->lock, flags);
    }

    return 0;
}


static int
velocity_netdev_event(
    struct notifier_block   *nb,
    unsigned long           notification,
    void                    *ptr
    )
{
    struct in_ifaddr*   ifa = (struct in_ifaddr*)ptr;
    struct net_device*  dev;
    PVELOCITY_INFO      pInfo;

    if (ifa) {
        dev = ifa->ifa_dev->dev;
        for (pInfo=pVelocity3_Infos; pInfo; pInfo=pInfo->next) {
            if (pInfo->dev == dev)
                velocity_get_ip(pInfo);
        }
    }
    return NOTIFY_DONE;
}
#endif // CONFIG_PM
#endif // KERNEL_VERSION(2,4,9)
