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

typedef int irqreturn_t;

#define	dma_addr_t	IOPhysicalAddress

typedef struct { volatile int counter; } atomic_t;
typedef struct { int dummy; } pt_regs;
typedef IOSimpleLock* spinlock_t;

#define atomic_read(v)          ((v)->counter)
#define atomic_set(v,x)          ((v)->counter)=(x)

#define	____cacheline_aligned_in_smp

#define true 1
#define false 0

#define cpu_to_le64(x)	OSSwapHostToLittleConstInt64(x)
#define cpu_to_le32(x)	OSSwapHostToLittleConstInt32(x)
#define	le16_to_cpu(x)	OSSwapLittleToHostInt16(x)

#define ALIGN(x,a) (((x)+(a)-1)&~((a)-1))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define MAX_SKB_FRAGS (65536/PAGE_SIZE + 2)

#ifndef __cplusplus
struct	IONetworkController;
struct	IOPCIDevice;
#endif

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

#define container_of(ptr, type, member) ({                      \
         const typeof(((type *)0)->member) * __mptr = (ptr);     \
         (type *)((char *)__mptr - offsetof(type, member)); })

#define INIT_LIST_HEAD(ptr) do { (ptr)->next = (ptr); (ptr)->prev = (ptr); } while (0)
#define LIST_POISON1  ((list_head *) 0x00100100)
#define LIST_POISON2  ((list_head *) 0x00200200)

struct list_head {
	struct list_head *next, *prev;
};

#define list_entry(ptr, type, member) \
         container_of(ptr, type, member)

static __inline__ int list_empty(struct list_head *head)
{
	return head->next == head;
}

static inline void __list_del(struct list_head * prev, struct list_head * next)
{
	next->prev = prev;
	prev->next = next;
}

static inline void list_del(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
	entry->next = LIST_POISON1;
	entry->prev = LIST_POISON2;
}

static inline void __list_add(struct list_head *newl, struct list_head *prev, struct list_head *next)
{
	next->prev = newl;
	newl->next = next;
	newl->prev = prev;
	prev->next = newl;
}

static inline void list_add_tail(struct list_head *newl, struct list_head *head)
{
         __list_add(newl, head->prev, head);
}

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
#define ENODEV          19

#define	GFP_KERNEL	0

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

static inline unsigned char
inb (unsigned short port)
{
	unsigned char _v;
	
	__asm__ __volatile__ ("inb %w1,%0":"=a" (_v):"Nd" (port));
	return _v;
}

static inline unsigned int
inl (unsigned short port)
{
	unsigned int _v;
	
	__asm__ __volatile__ ("inl %w1,%0":"=a" (_v):"Nd" (port));
	return _v;
}

static inline void
outb (unsigned char value, unsigned short port)
{
	__asm__ __volatile__ ("outb %b0,%w1": :"a" (value), "Nd" (port));
}
	
static inline void
outl (unsigned int value, unsigned short port)
{
	__asm__ __volatile__ ("outl %0,%w1": :"a" (value), "Nd" (port));
}



#define	writel(val, reg)	OSWriteLittleInt32(reg, 0, val)
#define	readl(reg)	OSReadLittleInt32(reg, 0)

#define	KERN_ERR
#define printk(args...)	IOLog(args)

#define	unlikely(s)	s
#define	likely(s)	s
#define	prefetch(x)
#define	spin_lock_irqsave(a,b)
#define	spin_unlock_irqrestore(a,b)


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

#define	kmem_cache_t	void

//#define	_BIT_FIELDS_HTOL	1

#endif /* _KCOMPAT_H_ */
