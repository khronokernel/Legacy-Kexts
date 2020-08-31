/*******************************************************************************
 
 Macros to compile Intel PRO/1000 Linux driver almost-as-is for Mac OS X.
 
 *******************************************************************************/

#ifndef _KCOMPAT_H_
#define _KCOMPAT_H_

#define	s64	__int64_t
#define	s32	__int32_t
#define	s16	__int16_t
#define	s8	__int8_t
#define	u64	__uint64_t
#define	u32	__uint32_t
#define	u16	__uint16_t
#define	u8	__uint8_t

#ifndef __le16
#define __le16 __uint16_t
#endif
#ifndef __le32
#define __le32 __uint32_t
#endif
#ifndef __le64
#define __le64 __uint64_t
#endif
#ifndef __be16
#define __be16 __uint16_t
#endif
#ifndef __be32
#define __be32 __uint32_t
#endif
#ifndef __be64
#define __be64 __uint64_t
#endif

#define	sk_buff	__mbuf

#define	__iomem
#define __devinit
#define	__always_unused

#define	dma_addr_t	IOPhysicalAddress

#define	____cacheline_aligned_in_smp

#define true 1
#define false 0

#define min_t(type,x,y) \
({ type __x = (x); type __y = (y); __x < __y ? __x: __y; })

#define max_t(type, x, y) \
({ type __max1 = (x); type __max2 = (y); __max1 > __max2 ? __max1: __max2; })

#define cpu_to_le16(x)	OSSwapHostToLittleConstInt16(x)
#define cpu_to_le32(x)	OSSwapHostToLittleConstInt32(x)
#define	cpu_to_le64(x)	OSSwapHostToLittleConstInt64(x)
#define	le16_to_cpu(x)	OSSwapLittleToHostInt16(x)
#define	le32_to_cpu(x)	OSSwapLittleToHostInt32(x)
#if		defined(__BIG_ENDIAN__)
#define	le16_to_cpus(x)	(*(x)=le16_to_cpu(*(x)))
#else
#define	le16_to_cpus(x)
#endif

#define	writel(val, reg)	_OSWriteInt32(reg, 0, val)
#define	writew(val, reg)	_OSWriteInt16(reg, 0, val)
#define	readl(reg)	_OSReadInt32(reg, 0)
#define	readw(reg)	_OSReadInt16(reg, 0)

#define ALIGN(x,a) (((x)+(a)-1)&~((a)-1))

#define BITS_PER_LONG   32
#ifndef	BIT
#define	BIT(n)	(1<<(n))
#endif

#define BITS_TO_LONGS(bits) \
(((bits)+BITS_PER_LONG-1)/BITS_PER_LONG)

/* GFP_ATOMIC means both !wait (__GFP_WAIT not set) and use emergency pool */
#define GFP_ATOMIC      0

struct net_device {
	u32 mtu;
};
typedef struct pci_dev {
	u16 vendor;
	u16 device;
	void* provider;
} pci_dev;

struct net_device_stats {
	unsigned long	rx_packets;				/* total packets received       */
	unsigned long	tx_packets;				/* total packets transmitted    */
	unsigned long	rx_bytes;				/* total bytes received         */
	unsigned long	tx_bytes;				/* total bytes transmitted      */
	unsigned long	rx_errors;				/* bad packets received         */
	unsigned long	tx_errors;				/* packet transmit problems     */
	unsigned long	rx_dropped;				/* no space in linux buffers    */
	unsigned long	tx_dropped;				/* no space available in linux  */
	unsigned long	multicast;				/* multicast packets received   */
	unsigned long	collisions;
    
	/* detailed rx_errors: */
	unsigned long	rx_length_errors;
	unsigned long	rx_over_errors;			/* receiver ring buff overflow  */
	unsigned long	rx_crc_errors;			/* recved pkt with crc error    */
	unsigned long	rx_frame_errors;		/* recv'd frame alignment error */
	unsigned long	rx_fifo_errors;			/* recv'r fifo overrun          */
	unsigned long	rx_missed_errors;		/* receiver missed packet       */
    
	/* detailed tx_errors */
	unsigned long	tx_aborted_errors;
	unsigned long	tx_carrier_errors;
	unsigned long	tx_fifo_errors;
	unsigned long	tx_heartbeat_errors;
	unsigned long	tx_window_errors;
    
	/* for cslip etc */
	unsigned long	rx_compressed;
	unsigned long	tx_compressed;
};

struct list_head {
	struct list_head *next, *prev;
};

struct timer_list {
	struct list_head entry;
	unsigned long expires;
    
	//spinlock_t lock;
	unsigned long magic;
    
	void (*function)(unsigned long);
	unsigned long data;
    
	//struct tvec_t_base_s *base;
};

struct work_struct {
	void* src; // IOTimerEventSrc
};

#define ADVERTISED_10baseT_Half         (1 << 0)
#define ADVERTISED_10baseT_Full         (1 << 1)
#define ADVERTISED_100baseT_Half        (1 << 2)
#define ADVERTISED_100baseT_Full        (1 << 3)
#define ADVERTISED_1000baseT_Half       (1 << 4)
#define ADVERTISED_1000baseT_Full       (1 << 5)
#define ADVERTISED_Autoneg              (1 << 6)
#define ADVERTISED_TP                   (1 << 7)
#define ADVERTISED_MII                  (1 << 9)
#define ADVERTISED_FIBRE                (1 << 10)
#define ADVERTISED_10000baseT_Full      (1 << 12)
#define ADVERTISED_2500baseX_Full       (1 << 15)

#define	ADVERTISED_Default	(\
	ADVERTISED_10baseT_Half|ADVERTISED_10baseT_Full|\
	ADVERTISED_100baseT_Full|ADVERTISED_100baseT_Half|\
	ADVERTISED_1000baseT_Full|\
	ADVERTISED_Autoneg|ADVERTISED_TP|ADVERTISED_MII)

#define SPEED_10    10
#define SPEED_100   100
#define SPEED_1000  1000

#define IFNAMSIZ        16

#define ETH_ALEN		6			/* Octets in one ethernet addr   */
#define ETH_HLEN		14			/* Total octets in header.       */
#define ETH_ZLEN		60			/* Min. octets in frame sans FCS */
#define ETH_DATA_LEN	1500		/* Max. octets in payload        */
#define ETH_FRAME_LEN	1514		/* Max. octets in frame sans FCS */
#define ETH_FCS_LEN		4			/* Octets in the FCS             */

#define ETH_P_IP        0x0800
#define ETH_P_IPV6      0x86DD

#define VLAN_HLEN		4			/* The additional bytes (on top of the Ethernet header) that VLAN requires. */
#define VLAN_ETH_ALEN	6			/* Octets in one ethernet addr   */
#define VLAN_ETH_HLEN	18			/* Total octets in header.       */
#define VLAN_ETH_ZLEN	64			/* Min. octets in frame sans FCS */
#define	VLAN_ETH_FRAME_LEN	1518	/* Max. octets in frame sans FCS */
#define VLAN_N_VID          4096

/* Generic MII registers. */
#define MII_BMCR                0x00    /* Basic mode control register */
#define MII_BMSR                0x01    /* Basic mode status register  */
#define MII_PHYSID1             0x02    /* PHYS ID 1                   */
#define MII_PHYSID2             0x03    /* PHYS ID 2                   */
#define MII_ADVERTISE           0x04    /* Advertisement control reg   */
#define MII_LPA                 0x05    /* Link partner ability reg    */
#define MII_EXPANSION           0x06    /* Expansion register          */
#define MII_CTRL1000            0x09    /* 1000BASE-T control          */
#define MII_STAT1000            0x0a    /* 1000BASE-T status           */
#define MII_MMD_CTRL            0x0d    /* MMD Access Control Register */
#define MII_MMD_DATA            0x0e    /* MMD Access Data Register */
#define MII_ESTATUS             0x0f    /* Extended Status             */
#define MII_DCOUNTER            0x12    /* Disconnect counter          */
#define MII_FCSCOUNTER          0x13    /* False carrier counter       */
#define MII_NWAYTEST            0x14    /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER         0x15    /* Receive error counter       */
#define MII_SREVISION           0x16    /* Silicon revision            */
#define MII_RESV1               0x17    /* Reserved...                 */
#define MII_LBRERROR            0x18    /* Lpback, rx, bypass error    */
#define MII_PHYADDR             0x19    /* PHY address                 */
#define MII_RESV2               0x1a    /* Reserved...                 */
#define MII_TPISTATUS           0x1b    /* TPI status for 10mbps       */
#define MII_NCONFIG             0x1c    /* Network interface config    */

/* Basic mode control register. */
#define BMCR_RESV               0x003f  /* Unused...                   */
#define BMCR_SPEED1000          0x0040  /* MSB of Speed (1000)         */
#define BMCR_CTST               0x0080  /* Collision test              */
#define BMCR_FULLDPLX           0x0100  /* Full duplex                 */
#define BMCR_ANRESTART          0x0200  /* Auto negotiation restart    */
#define BMCR_ISOLATE            0x0400  /* Isolate data paths from MII */
#define BMCR_PDOWN              0x0800  /* Enable low power state      */
#define BMCR_ANENABLE           0x1000  /* Enable auto negotiation     */
#define BMCR_SPEED100           0x2000  /* Select 100Mbps              */
#define BMCR_LOOPBACK           0x4000  /* TXD loopback bits           */
#define BMCR_RESET              0x8000  /* Reset to default state      */

/* Basic mode status register. */
#define BMSR_ERCAP              0x0001  /* Ext-reg capability          */
#define BMSR_JCD                0x0002  /* Jabber detected             */
#define BMSR_LSTATUS            0x0004  /* Link status                 */
#define BMSR_ANEGCAPABLE        0x0008  /* Able to do auto-negotiation */
#define BMSR_RFAULT             0x0010  /* Remote fault detected       */
#define BMSR_ANEGCOMPLETE       0x0020  /* Auto-negotiation complete   */
#define BMSR_RESV               0x00c0  /* Unused...                   */
#define BMSR_ESTATEN            0x0100  /* Extended Status in R15      */
#define BMSR_100HALF2           0x0200  /* Can do 100BASE-T2 HDX       */
#define BMSR_100FULL2           0x0400  /* Can do 100BASE-T2 FDX       */
#define BMSR_10HALF             0x0800  /* Can do 10mbps, half-duplex  */
#define BMSR_10FULL             0x1000  /* Can do 10mbps, full-duplex  */
#define BMSR_100HALF            0x2000  /* Can do 100mbps, half-duplex */
#define BMSR_100FULL            0x4000  /* Can do 100mbps, full-duplex */
#define BMSR_100BASE4           0x8000  /* Can do 100mbps, 4k packets  */

/* Advertisement control register. */
#define ADVERTISE_SLCT          0x001f  /* Selector bits               */
#define ADVERTISE_CSMA          0x0001  /* Only selector supported     */
#define ADVERTISE_10HALF        0x0020  /* Try for 10mbps half-duplex  */
#define ADVERTISE_1000XFULL     0x0020  /* Try for 1000BASE-X full-duplex */
#define ADVERTISE_10FULL        0x0040  /* Try for 10mbps full-duplex  */
#define ADVERTISE_1000XHALF     0x0040  /* Try for 1000BASE-X half-duplex */
#define ADVERTISE_100HALF       0x0080  /* Try for 100mbps half-duplex */
#define ADVERTISE_1000XPAUSE    0x0080  /* Try for 1000BASE-X pause    */
#define ADVERTISE_100FULL       0x0100  /* Try for 100mbps full-duplex */
#define ADVERTISE_1000XPSE_ASYM 0x0100  /* Try for 1000BASE-X asym pause */
#define ADVERTISE_100BASE4      0x0200  /* Try for 100mbps 4k packets  */
#define ADVERTISE_PAUSE_CAP     0x0400  /* Try for pause               */
#define ADVERTISE_PAUSE_ASYM    0x0800  /* Try for asymetric pause     */
#define ADVERTISE_RESV          0x1000  /* Unused...                   */
#define ADVERTISE_RFAULT        0x2000  /* Say we can detect faults    */
#define ADVERTISE_LPACK         0x4000  /* Ack link partners response  */
#define ADVERTISE_NPAGE         0x8000  /* Next page bit               */

/* Link partner ability register. */
#define LPA_SLCT                0x001f  /* Same as advertise selector  */
#define LPA_10HALF              0x0020  /* Can do 10mbps half-duplex   */
#define LPA_1000XFULL           0x0020  /* Can do 1000BASE-X full-duplex */
#define LPA_10FULL              0x0040  /* Can do 10mbps full-duplex   */
#define LPA_1000XHALF           0x0040  /* Can do 1000BASE-X half-duplex */
#define LPA_100HALF             0x0080  /* Can do 100mbps half-duplex  */
#define LPA_1000XPAUSE          0x0080  /* Can do 1000BASE-X pause     */
#define LPA_100FULL             0x0100  /* Can do 100mbps full-duplex  */
#define LPA_1000XPAUSE_ASYM     0x0100  /* Can do 1000BASE-X pause asym*/
#define LPA_100BASE4            0x0200  /* Can do 100mbps 4k packets   */
#define LPA_PAUSE_CAP           0x0400  /* Can pause                   */
#define LPA_PAUSE_ASYM          0x0800  /* Can pause asymetrically     */
#define LPA_RESV                0x1000  /* Unused...                   */
#define LPA_RFAULT              0x2000  /* Link partner faulted        */
#define LPA_LPACK               0x4000  /* Link partner acked us       */
#define LPA_NPAGE               0x8000  /* Next page bit               */

#define LPA_DUPLEX              (LPA_10FULL | LPA_100FULL)
#define LPA_100                 (LPA_100FULL | LPA_100HALF | LPA_100BASE4)

#define EXPANSION_NWAY          0x0001  /* Can do N-way auto-nego      */
#define EXPANSION_LCWP          0x0002  /* Got new RX page code word   */
#define EXPANSION_ENABLENPAGE   0x0004  /* This enables npage words    */
#define EXPANSION_NPCAPABLE     0x0008  /* Link partner supports npage */
#define EXPANSION_MFAULTS       0x0010  /* Multiple faults detected    */
#define EXPANSION_RESV          0xffe0  /* Unused...                   */

/* 1000BASE-T Control register */
#define ADVERTISE_1000FULL      0x0200  /* Advertise 1000BASE-T full duplex */
#define ADVERTISE_1000HALF      0x0100  /* Advertise 1000BASE-T half duplex */
#define CTL1000_AS_MASTER       0x0800
#define CTL1000_ENABLE_MASTER   0x1000

/* 1000BASE-T Status register */
#define LPA_1000LOCALRXOK       0x2000  /* Link partner local receiver status */
#define LPA_1000REMRXOK         0x1000  /* Link partner remote receiver status */
#define LPA_1000FULL            0x0800  /* Link partner 1000BASE-T full duplex */
#define LPA_1000HALF            0x0400  /* Link partner 1000BASE-T half duplex */

/* EEE Supported/Advertisement/LP Advertisement registers.
 */
#define MDIO_AN_EEE_ADV_100TX   0x0002  /* Advertise 100TX EEE cap */
#define MDIO_AN_EEE_ADV_1000T   0x0004  /* Advertise 1000T EEE cap */

#define MDIO_EEE_100TX          MDIO_AN_EEE_ADV_100TX   /* 100TX EEE cap */
#define MDIO_EEE_1000T          MDIO_AN_EEE_ADV_1000T   /* 1000T EEE cap */
#define MDIO_EEE_10GT           0x0008  /* 10GT EEE cap */
#define MDIO_EEE_1000KX         0x0010  /* 1000KX EEE cap */
#define MDIO_EEE_10GKX4         0x0020  /* 10G KX4 EEE cap */
#define MDIO_EEE_10GKR          0x0040  /* 10G KR EEE cap */


#define NET_IP_ALIGN	2
#ifndef NETIF_F_RXFCS
#define NETIF_F_RXFCS	0
#endif /* NETIF_F_RXFCS */
#ifndef NETIF_F_RXALL
#define NETIF_F_RXALL	0
#endif /* NETIF_F_RXALL */


#define	PCI_CAP_ID_EXP			0x10	/* PCI Express */
#define	PCI_CAP_ID_AF			0x13	/* PCI Advanced Features */
#define	PCI_AF_CAP		3
#define	PCI_AF_CAP_FLR	0x02

#define	PCI_EXP_DEVCTL	8
#define	PCI_EXP_DEVCTL_CERE	0x0001	/* Correctable Error Reporting En. */
#define	PCI_EXP_LNKCTL	16
#define PCIE_LINK_STATE_L0S     1
#define PCIE_LINK_STATE_L1 2
#define	PCI_EXP_LNKCTL_ASPM_L0S  0x01  /* L0s Enable */
#define	PCI_EXP_LNKCTL_ASPM_L1   0x02  /* L1 Enable */

#define PCI_LTR_MAX_SNOOP_LAT	0x4
#define PCI_LTR_MAX_NOSNOOP_LAT	0x6
#define  PCI_LTR_VALUE_MASK	0x000003ff
#define  PCI_LTR_SCALE_MASK	0x00001c00
#define  PCI_LTR_SCALE_SHIFT	10

#define MAX_NUMNODES 1
#define first_online_node 0
#define node_online(node) ((node) == 0)
#define ether_crc_le(length, data) _kc_ether_crc_le(length, data)
#ifndef is_zero_ether_addr
#define is_zero_ether_addr _kc_is_zero_ether_addr
static inline int _kc_is_zero_ether_addr(const u8 *addr)
{
	return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}
#endif
#ifndef is_multicast_ether_addr
#define is_multicast_ether_addr _kc_is_multicast_ether_addr
static inline int _kc_is_multicast_ether_addr(const u8 *addr)
{
	return addr[0] & 0x01;
}
#endif /* is_multicast_ether_addr */

static inline unsigned int _kc_ether_crc_le(int length, unsigned char *data)
{
	unsigned int crc = 0xffffffff;  /* Initial value. */
	while(--length >= 0) {
		unsigned char current_octet = *data++;
		int bit;
		for (bit = 8; --bit >= 0; current_octet >>= 1) {
			if ((crc ^ current_octet) & 1) {
				crc >>= 1;
				crc ^= 0xedb88320U;
			} else
				crc >>= 1;
		}
	}
	return crc;
}

#define	EIO		5
#define	ENOMEM	12
#define	EBUSY	16
/*****************************************************************************/
#define msleep(x)	IOSleep(x)
#define udelay(x)	IODelay(x)
#define might_sleep()

#define mdelay(x)	for(int i = 0; i < x; i++ )udelay(1000)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#define usleep_range(min, max)	msleep(DIV_ROUND_UP(min, 1000))


/*****************************************************************************/

#define DMA_BIT_MASK(n)	(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

/************** Ugly macros to compile ich8lan.c *****************************/

#define	DEFINE_MUTEX(x)	void x##_dummy(){}
#define	mutex_lock(x)	IOLockLock(*x)
#define	mutex_unlock(x)	IOLockUnlock(*x)

#ifndef __cplusplus
typedef void IOBufferMemoryDescriptor;
#endif

#define	prefetch(x)
#define	unlikely(x)	(x)
#define	likely(x)	(x)
#define	BUG()
#define	BUG_ON(x)
#define	wmb()	OSSynchronizeIO()
#define	mmiowb()	OSSynchronizeIO()
#define	rmb()

#ifndef BIT
#define BIT(nr)         (1UL << (nr))
#endif

#define	__MODULE_STRING(s)	"x"
#define synchronize_irq(x)

static inline int test_bit(int nr, const volatile unsigned long * addr) {
	return (*addr & (1<<nr)) != 0;
}

static inline void set_bit(int nr, volatile unsigned long * addr) {
	*addr |= (1 << nr);
}

static inline void clear_bit(int nr, volatile unsigned long * addr) {
	*addr &= ~(1 << nr);
}

static inline int test_and_set_bit(int nr, volatile unsigned long * addr) {
	int rc = test_bit(nr,addr);
	set_bit(nr,addr);
	return rc;
}

#define do_div(lat_ns, speed) \
(lat_ns) = (UInt64)(lat_ns) / (speed)

#ifdef __cplusplus
extern "C" {
#endif
    int pci_read_config_word(pci_dev *dev, int where, u16 *val);
    u16 pci_find_capability(pci_dev *dev, u16 val);
#ifdef __cplusplus
}
#endif

#endif /* _KCOMPAT_H_ */
