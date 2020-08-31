/*******************************************************************************

  Macros to compile Intel PRO/1000 Linux driver almost-as-is for Mac OS X.
 
*******************************************************************************/

#ifndef _KCOMPAT_H_
#define _KCOMPAT_H_

typedef __int64_t s64;
typedef __int32_t s32;
typedef __int16_t s16;
typedef __int8_t s8;
typedef __uint64_t u64;
typedef __uint32_t u32;
typedef __uint16_t u16;
typedef __uint8_t u8;

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

#define	dma_addr_t	IOPhysicalAddress

#define	____cacheline_aligned_in_smp

#define	netdev_features_t	__uint32_t

#define cpu_to_le16(x)	OSSwapHostToLittleConstInt16(x)
#define cpu_to_le32(x)	OSSwapHostToLittleConstInt32(x)
#define	cpu_to_le64(x)	OSSwapHostToLittleConstInt64(x)
#define	le16_to_cpu(x)	OSSwapLittleToHostInt16(x)
#define	le32_to_cpu(x)	OSSwapLittleToHostInt32(x)
#define	be16_to_cpu(x)	OSSwapBigToHostInt16(x)

#define	writel(val, reg)	_OSWriteInt32(reg, 0, val)
#define	writew(val, reg)	_OSWriteInt16(reg, 0, val)
#define	readl(reg)	_OSReadInt32(reg, 0)
#define	readw(reg)	_OSReadInt16(reg, 0)
#define read_barrier_depends()

#ifdef	ALIGN
#undef	ALIGN
#endif
#define ALIGN(x,a) (((x)+(a)-1)&~((a)-1))

#define BITS_PER_LONG   32

#define BITS_TO_LONGS(bits) \
(((bits)+BITS_PER_LONG-1)/BITS_PER_LONG)

/* GFP_ATOMIC means both !wait (__GFP_WAIT not set) and use emergency pool */
#define GFP_ATOMIC      0

#define DEFINE_DMA_UNMAP_ADDR(ADDR_NAME)        dma_addr_t ADDR_NAME
#define DEFINE_DMA_UNMAP_LEN(LEN_NAME)          UInt32 LEN_NAME
#define dma_unmap_addr(PTR, ADDR_NAME)           ((PTR)->ADDR_NAME)
#define dma_unmap_addr_set(PTR, ADDR_NAME, VAL)  (((PTR)->ADDR_NAME) = (VAL))
#define dma_unmap_len(PTR, LEN_NAME)             ((PTR)->LEN_NAME)
#define dma_unmap_len_set(PTR, LEN_NAME, VAL)    (((PTR)->LEN_NAME) = (VAL))


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

#ifndef rcu_head
struct __kc_callback_head {
	struct __kc_callback_head *next;
	void (*func)(struct callback_head* head);
};
#define rcu_head __kc_callback_head
#endif
#ifndef kfree_rcu
/* this is placed here due to a lack of rcu_barrier in previous kernels */
#define kfree_rcu(_ptr, _offset) kfree(_ptr)
#endif /* kfree_rcu */

#ifndef rounddown_pow_of_two
#define rounddown_pow_of_two(n) \
__builtin_constant_p(n) ? ( \
(n == 1) ? 0 : \
(1UL << ilog2(n))) : \
(1UL << (fls_long(n) - 1))
#endif


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
#define VLAN_VID_MASK           0x0fff /* VLAN Identifier */
#define VLAN_N_VID              4096

#define IFF_PROMISC     0x100           /* receive all packets          */
#define IFF_ALLMULTI    0x200           /* receive all multicast packets*/

#define NET_IP_ALIGN	2

#define NETIF_F_SG              1       /* Scatter/gather IO. */
#define NETIF_F_IP_CSUM         2       /* Can checksum TCP/UDP over IPv4. */
#define NETIF_F_NO_CSUM         4       /* Does not require checksum. F.e. loopack. */
#define NETIF_F_HW_CSUM         8       /* Can checksum all the packets. */
#define NETIF_F_IPV6_CSUM       16      /* Can checksum TCP/UDP over IPV6 */
#define NETIF_F_HIGHDMA         32      /* Can DMA to high memory. */
#define NETIF_F_FRAGLIST        64      /* Scatter/gather IO. */
#define NETIF_F_HW_VLAN_TX      128     /* Transmit VLAN hw acceleration */
#define NETIF_F_HW_VLAN_RX      256     /* Receive VLAN hw acceleration */
#define NETIF_F_HW_VLAN_FILTER  512     /* Receive filtering on VLAN */
#define NETIF_F_VLAN_CHALLENGED 1024    /* Device cannot handle VLAN packets */
#define NETIF_F_GSO             2048    /* Enable software GSO. */

#define NETIF_F_GRO             16384   /* Generic receive offload */
#define NETIF_F_LRO             32768   /* large receive offload */

#define NETIF_F_SCTP_CSUM       (1 << 25) /* SCTP checksum offload */
//#define NETIF_F_RXHASH          (1 << 28) /* Receive hashing offload */
#define NETIF_F_RXCSUM          (1 << 29) /* Receive checksumming offload */

#define DUPLEX_HALF             0x00
#define DUPLEX_FULL             0x01

#if (65536/PAGE_SIZE + 2) < 16
#define MAX_SKB_FRAGS 16UL
#else
#define MAX_SKB_FRAGS (65536/PAGE_SIZE + 2)
#endif

#define PCI_COMMAND             0x04    /* 16 bits */
#define	PCI_EXP_DEVCTL	8
#define	PCI_EXP_DEVCTL_CERE	0x0001	/* Correctable Error Reporting En. */
#define	PCI_EXP_LNKCTL	16
#define PCIE_LINK_STATE_L0S     1
#define PCIE_LINK_STATE_L1 2

#define  PCI_STATUS_REC_TARGET_ABORT    0x1000 /* Master ack of " */
#define  PCI_STATUS_REC_MASTER_ABORT    0x2000 /* Set on master abort */
#define  PCI_STATUS_SIG_SYSTEM_ERROR    0x4000 /* Set when we drive SERR */

#define MDIO_EEE_100TX  0x0002  /* Advertise 100TX EEE cap */
#define MDIO_EEE_1000T  0x0004  /* Advertise 1000T EEE cap */

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
#define EINVAL  22  /* Invalid argument */
/*****************************************************************************/
#define msleep(x)	IOSleep(x)
#define udelay(x)	IODelay(x)

#define mdelay(x)	for(int i = 0; i < x; i++ )udelay(1000)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))
#define usleep_range(min, max)	msleep(DIV_ROUND_UP(min, 1000))	


/*****************************************************************************/

#define DMA_BIT_MASK(n)	(((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))


#ifdef __cplusplus
class AppleIGB;
#else
typedef void IOBufferMemoryDescriptor;
typedef void IOPCIDevice;
typedef void IOEthernetController;
typedef void IOTimerEventSource;
typedef void AppleIGB;
#endif

#define	prefetch(x)
#define	prefetchw(x)
#define	unlikely(x)	(x)
#define	likely(x)	(x)
#define	BUG()
#define	wmb()
#define	rmb()
#define	mmiowb()
#define	smp_mb()	mb()
#define mb()

#define	__MODULE_STRING(s)	"x"
#define	pr_debug(args...)	IOLog(args)
#define	pr_err(args...)	IOLog(args)
#define	dev_warn(dev,args...)	IOLog(args)
#define	dev_info(dev,args...)	IOLog(args)
#define	in_interrupt()	(0)

#define __stringify_1(x...)     #x
#define __stringify(x...)       __stringify_1(x)
#define	__devinit
#define	__devexit
#define WARN_ON(x)

#define min_t(type,x,y) \
	({ type __x = (x); type __y = (y); __x < __y ? __x: __y; })

#define		iphdr	ip
struct net_device { void* dummy; };

enum irqreturn {
	IRQ_NONE,
	IRQ_HANDLED,
	IRQ_WAKE_THREAD,
};
typedef enum irqreturn irqreturn_t;

typedef struct sk_buff_head {
	struct sk_buff	*next;
	struct sk_buff	*prev;
	u32		qlen;
	//spinlock_t	lock;
} sk_buff_head;

typedef struct napi_struct {
	struct list_head        poll_list;
	unsigned long           state;
	int                     weight;
	int                     (*poll)(struct napi_struct *, int);

	unsigned int            gro_count;
	//struct net_device       *dev;
	struct list_head        dev_list;
	struct sk_buff          *gro_list;
	struct sk_buff          *skb;
} napi_struct;

struct msix_entry {
	u32     vector; /* kernel uses to write allocated vector */
	u16     entry;  /* driver uses to specify entry, OS writes */
};

#define IFNAMSIZ        16
#define	____cacheline_internodealigned_in_smp

enum netdev_tx {
	__NETDEV_TX_MIN  = -100,     /* make sure enum is signed */
	NETDEV_TX_OK     = 0x00,        /* driver took care of packet */
	NETDEV_TX_BUSY   = 0x10,        /* driver tx path was busy*/
	NETDEV_TX_LOCKED = 0x20,        /* driver tx lock was already taken */
};
typedef enum netdev_tx netdev_tx_t;

#define max_t(type, x, y) ({                    \
	type __max1 = (x);                      \
	type __max2 = (y);                      \
	__max1 > __max2 ? __max1: __max2; })

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


static inline int is_valid_ether_addr(const u8 *addr)
{
    return !is_multicast_ether_addr(addr) && !is_zero_ether_addr(addr);
}

static inline void random_ether_addr(u8 *addr)
{
	u_int32_t temp[2];
	temp[0] = random();
	temp[1] = random();
	
	bcopy(temp,addr,ETH_ALEN);
	addr [0] &= 0xfe;       /* clear multicast bit */
	addr [0] |= 0x02;       /* set local assignment bit (IEEE802) */
}

static inline unsigned ether_addr_equal(const u8 *addr1, const u8 *addr2)
{
	const u16 *a = (const u16 *) addr1;
	const u16 *b = (const u16 *) addr2;
	return ((a[0] ^ b[0]) | (a[1] ^ b[1]) | (a[2] ^ b[2])) != 0;
}


#ifdef HAVE_VLAN_RX_REGISTER
#define VLAN_GROUP_ARRAY_LEN          4096
#define VLAN_GROUP_ARRAY_SPLIT_PARTS  8
#define VLAN_GROUP_ARRAY_PART_LEN     (VLAN_GROUP_ARRAY_LEN/VLAN_GROUP_ARRAY_SPLIT_PARTS)
 
struct vlan_group {
	struct IOEthernetController **vlan_devices_arrays[VLAN_GROUP_ARRAY_SPLIT_PARTS];
};


#endif

#define container_of(ptr, type, member) ({ \
	const typeof( ((type *)0)->member ) *__mptr = (ptr); \
	(type *)( (char *)__mptr - offsetof(type,member) );})

#define ACCESS_ONCE(x) (*(volatile typeof(x) *)&(x))

#endif /* _KCOMPAT_H_ */
