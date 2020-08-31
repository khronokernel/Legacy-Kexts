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

#define	sk_buff	__mbuf

#define	__iomem
#define	__devinit

#define	dma_addr_t	IOPhysicalAddress

typedef struct { volatile int counter; } atomic_t;
typedef struct { int dummy; } spinlock_t;
#define atomic_read(v)          ((v)->counter)
#define atomic_set(v,x)          ((v)->counter)=(x)

#define	____cacheline_aligned_in_smp

#define true 1
#define false 0

#define cpu_to_le16(x)	OSSwapHostToLittleConstInt16(x)
#define cpu_to_le32(x)	OSSwapHostToLittleConstInt32(x)
#define	cpu_to_le64(x)	OSSwapHostToLittleConstInt64(x)
#define	le16_to_cpu(x)	OSSwapLittleToHostInt16(x)
#define	le32_to_cpu(x)	OSSwapLittleToHostInt32(x)

#define ALIGN(x,a) (((x)+(a)-1)&~((a)-1))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define MAX_SKB_FRAGS (65536/PAGE_SIZE + 2)

#define net_device	IONetworkController
#define	pci_dev	IOPCIDevice

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
	unsigned long pending;
	struct list_head entry;
	void (*func)(void *);
	void *data;
	void *wq_data;
	struct timer_list timer;
};


#define ETH_ALEN		6			/* Octets in one ethernet addr   */
#define ETH_HLEN		14			/* Total octets in header.       */
#define ETH_ZLEN		60			/* Min. octets in frame sans FCS */
#define ETH_DATA_LEN	1500		/* Max. octets in payload        */
#define ETH_FRAME_LEN	1514		/* Max. octets in frame sans FCS */
#define ETH_FCS_LEN		4			/* Octets in the FCS             */

#define VLAN_HLEN		4			/* The additional bytes (on top of the Ethernet header) that VLAN requires. */
#define VLAN_ETH_ALEN	6			/* Octets in one ethernet addr   */
#define VLAN_ETH_HLEN	18			/* Total octets in header.       */
#define VLAN_ETH_ZLEN	64			/* Min. octets in frame sans FCS */

#define NET_IP_ALIGN	2

#define	ENOMEM	12
#define	EIO	5
#define EBUSY           16      /* Device or resource busy */
#define EINVAL          22      /* Invalid argument */

#define NETDEV_TX_OK	0
#define NETDEV_TX_BUSY	1

#define  PCI_COMMAND_INVALIDATE 0x10    /* Use memory write and invalidate */

#define NETIF_F_SG              1       /* Scatter/gather IO. */
#define NETIF_F_IP_CSUM         2       /* Can checksum TCP/UDP over IPv4. */
#define NETIF_F_NO_CSUM         4       /* Does not require checksum. F.e. loopack. */
#define NETIF_F_HW_CSUM         8       /* Can checksum all the packets. */
#define NETIF_F_IPV6_CSUM       16      /* Can checksum TCP/UDP over IPV6 */
#define NETIF_F_HIGHDMA         32      /* Can DMA to high memory. */
#define NETIF_F_FRAGLIST        64      /* Scatter/gather IO. */
//#define NETIF_F_HW_VLAN_TX      128     /* Transmit VLAN hw acceleration */
//#define NETIF_F_HW_VLAN_RX      256     /* Receive VLAN hw acceleration */
//#define NETIF_F_HW_VLAN_FILTER  512     /* Receive filtering on VLAN */
//#define NETIF_F_VLAN_CHALLENGED 1024    /* Device cannot handle VLAN packets */
#define NETIF_F_GSO             2048    /* Enable software GSO. */
#define NETIF_F_NETNS_LOCAL     8192    /* Does not change network namespaces */
#define NETIF_F_GRO             16384   /* Generic receive offload */
#define NETIF_F_LRO             32768   /* large receive offload */

/* the GSO_MASK reserves bits 16 through 23 */
#define NETIF_F_FCOE_CRC        (1 << 24) /* FCoE CRC32 */
#define NETIF_F_SCTP_CSUM       (1 << 25) /* SCTP checksum offload */
#define NETIF_F_FCOE_MTU        (1 << 26) /* Supports max FCoE MTU, 2158 bytes*/
#define NETIF_F_NTUPLE          (1 << 27) /* N-tuple filters supported */

/* Segmentation offload features */
//#define NETIF_F_GSO_SHIFT       16
//#define NETIF_F_GSO_MASK        0x00ff0000
//#define NETIF_F_TSO             (SKB_GSO_TCPV4 << NETIF_F_GSO_SHIFT)
//#define NETIF_F_UFO             (SKB_GSO_UDP << NETIF_F_GSO_SHIFT)
//#define NETIF_F_GSO_ROBUST      (SKB_GSO_DODGY << NETIF_F_GSO_SHIFT)
//#define NETIF_F_TSO_ECN         (SKB_GSO_TCP_ECN << NETIF_F_GSO_SHIFT)
//#define NETIF_F_TSO6            (SKB_GSO_TCPV6 << NETIF_F_GSO_SHIFT)
//#define NETIF_F_FSO             (SKB_GSO_FCOE << NETIF_F_GSO_SHIFT)
#define IFF_PROMISC     0x100           /* receive all packets          */
#define IFF_ALLMULTI    0x200           /* receive all multicast packets*/

#define DUPLEX_HALF             0x00
#define DUPLEX_FULL             0x01

enum {
     SKB_GSO_TCPV4 = 1 << 0,
     SKB_GSO_UDP = 1 << 1,
     /* This indicates the skb is from an untrusted source. */
     SKB_GSO_DODGY = 1 << 2,
     /* This indicates the tcp segment has CWR set. */
     SKB_GSO_TCP_ECN = 1 << 3,
     SKB_GSO_TCPV6 = 1 << 4,
};
/*****************************************************************************/
#define msleep(x)	IOSleep(x)
#define udelay(x)	IODelay(x)

#define mdelay(x)	for(int i = 0; i < x; i++ )udelay(1000)


/*****************************************************************************/

#define DMA_BIT_MASK(n)	(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))

/************** Ugly macros to compile ich8lan.c *****************************/

#define	writel(val, reg)	_OSWriteInt32(reg, 0, val)
#define	writew(val, reg)	_OSWriteInt16(reg, 0, val)
#define	readl(reg)	_OSReadInt32(reg, 0)
#define	readw(reg)	_OSReadInt16(reg, 0)

#define	KERN_ERR
#define printk(args...)	IOLog(args)

#define	unlikely(s)	(s)
#define	likely(s)	(s)
#define	prefetch(x)
#define	spin_lock_irqsave(a,b)
#define	spin_unlock_irqrestore(a,b)
#define	spin_lock_init(a)


#define time_after(a,b)         \
	((long)(b) - (long)(a) < 0)
#define time_before(a,b)        time_after(b,a)

#define __stringify_1(x)        #x
#define __stringify(x)          __stringify_1(x)
#define __MODULE_STRING(x) __stringify(x)

#ifndef	__cplusplus
#define	IOBufferMemoryDescriptor	void

#endif

static inline void BUG() {}

#define	set_bit(b,a)	*(a)|=(1<<(b))
#define clear_bit(b,a)		*(a)&=~(1<<(b))
#define test_bit(b,a)	!!(*(a)&(1<<(b)))

#endif /* _KCOMPAT_H_ */
