#ifndef _KCOMPAT_H_
#define _KCOMPAT_H_


#define ETH_ALEN		6			/* Octets in one ethernet addr   */
#define ETH_HLEN		14			/* Total octets in header.       */
#define ETH_ZLEN		60			/* Min. octets in frame sans FCS */
#define ETH_DATA_LEN	1500		/* Max. octets in payload        */
#define ETH_FRAME_LEN	1514		/* Max. octets in frame sans FCS */
#define ETH_FCS_LEN		4			/* Octets in the FCS             */
#define ETH_P_ARP		0x0806

#define VLAN_HLEN		4			/* The additional bytes (on top of the Ethernet header) that VLAN requires. */
#define VLAN_ETH_ALEN	6			/* Octets in one ethernet addr   */
#define VLAN_ETH_HLEN	18			/* Total octets in header.       */
#define VLAN_ETH_ZLEN	64			/* Min. octets in frame sans FCS */

#define NET_IP_ALIGN	2

#define	ENOMEM	12
#define	EIO	5
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

#define IFF_PROMISC     0x100           /* receive all packets          */
#define IFF_ALLMULTI    0x200           /* receive all multicast packets*/
#define IFF_MULTICAST   0x1000

#ifdef	__cplusplus
extern "C" {
#endif
	
	void pci_read_config_byte(void* dev, U8 offset, U8* pData);
	void pci_read_config_dword(void* dev, U8 offset, U32* pData);

	void pci_write_config_byte(void* dev, U8 offset, U8 data);
	void pci_write_config_dword(void* dev, U8 offset, U32 data);

#ifdef	__cplusplus
}
#endif


#endif