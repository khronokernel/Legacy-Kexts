/*
 *  ViaVelocity_mbuf.cpp
 *  ViaVelocity
 *
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#include "ViaVelocity.h"

#if ! USE_RX_BUFFER

// copied from sys/mbuf.h
#if	defined(MAC_OS_X_VERSION_10_6)
#define	MLEN		mbuf_get_mlen()		/* normal data len */
#define	MHLEN		mbuf_get_mhlen()	/* data len w/pkthdr */

#else

#define	MSIZE		256		/* size of an mbuf */
#define	MLEN		(MSIZE - sizeof(struct m_hdr))	/* normal data len */
#define	MHLEN		(MLEN - sizeof(struct pkthdr))	/* data len w/pkthdr */
/* header at beginning of each mbuf: */
struct m_hdr {
	struct	mbuf *mh_next;		/* next buffer in chain */
	struct	mbuf *mh_nextpkt;	/* next chain in queue/record */
	long	mh_len;			/* amount of data in this mbuf */
	caddr_t	mh_data;		/* location of data */
	short	mh_type;		/* type of data in this mbuf */
	short	mh_flags;		/* flags; see below */
};
#define	SLIST_ENTRY(type)						\
struct {								\
struct type *sle_next;	/* next element */			\
}
#define	SLIST_HEAD(name, type)						\
struct name {								\
struct type *slh_first;	/* first element */			\
}

struct m_tag {
	SLIST_ENTRY(m_tag)	m_tag_link;	/* List of packet tags */
	u_int16_t			m_tag_type;	/* Module specific type */
	u_int16_t			m_tag_len;	/* Length of data */
	u_int32_t			m_tag_id;	/* Module ID */
};


struct	pkthdr {
	int	len;			/* total packet length */
	struct	ifnet *rcvif;		/* rcv interface */
	
	/* variables for ip and tcp reassembly */
	void	*header;		/* pointer to packet header */
	/* variables for hardware checksum */
	int     csum_flags;             /* flags regarding checksum */       
	int     csum_data;              /* data field used by csum routines */
	void	*reserved0;		/* unused, for future use */
	u_short	vlan_tag;		/* VLAN tag, host byte order */
	u_short socket_id;		/* socket id */
	SLIST_HEAD(packet_tags, m_tag) tags; /* list of packet tags */
};
#endif
/*
 * The following _MLEN and _MHLEN macros are private to xnu.  Private code
 * that are outside of xnu must use the mbuf_get_{mlen,mhlen} routines since
 * the sizes of the structures are dependent upon specific xnu configs.
 */

#define	MINCLSIZE	(MHLEN + MLEN)	/* smallest amount to put in cluster */

// copied from IONetowkringFamily/IONetworkController.cpp

static mbuf_t getPacket( UInt32 size,
						UInt32 how,
						UInt32 smask,
						UInt32 lmask )
{
    mbuf_t packet;
	UInt32 reqSize =  size + smask + lmask; 	// we over-request so we can fulfill alignment needs.
	
	if(reqSize > MHLEN && reqSize <= MINCLSIZE)	//as protection from drivers that incorrectly assume they always get a single-mbuf packet
		reqSize = MINCLSIZE + 1;				//we force kernel to give us a cluster instead of chained small mbufs.
	
	if( 0 == mbuf_allocpacket(how, reqSize, NULL, &packet))
	{
		mbuf_t m = packet;
		mbuf_pkthdr_setlen(packet, size);
		//run the chain and apply alignment
		
		while(size && m)
		{
			uintptr_t alignedStart, originalStart;
			
			originalStart = (uintptr_t)mbuf_data(m);
			alignedStart = (originalStart + smask) & ~((uintptr_t)smask);
			mbuf_setdata(m,  (caddr_t)alignedStart, (mbuf_maxlen(m) - (alignedStart - originalStart)) & ~lmask);
			
			if(mbuf_len(m) > size)
				mbuf_setlen(m, size); //truncate to remaining portion of packet
			
			size -= mbuf_len(m);
			m = mbuf_next(m);
		}
		return packet;
	}
	else
		return NULL;
}

static inline bool IO_COPY_MBUF(mbuf_t src, mbuf_t dst, int length)
{
    caddr_t src_dat, dst_dat;
    int dst_len, src_len;
	
    assert(src && dst);
	
	// dupe the header to pick up internal things like csums and vlan tags
	mbuf_copy_pkthdr(dst, src);
	mbuf_pkthdr_setheader(dst, NULL); //otherwise it could be pointing into src's data
	
    dst_len = mbuf_len(dst);
    dst_dat = (caddr_t)mbuf_data(dst);
	
    while (src) {
		
        src_len = mbuf_len( src );
        src_dat = (caddr_t)mbuf_data( src );
		
        if (src_len > length)
            src_len = length;
		
        while (src_len) {
			
            if (dst_len >= src_len) {
                // copy entire src mbuf to dst mbuf.
				
                bcopy(src_dat, dst_dat, src_len);               
                length -= src_len;
                dst_len -= src_len;
                dst_dat += src_len;
                src_len = 0;
            }
            else {
                // fill up dst mbuf with some portion of the data in
                // the src mbuf.
				
                bcopy(src_dat, dst_dat, dst_len);       // dst_len = 0?             
                length -= dst_len;
                dst_len = 0;
                src_len -= dst_len;         
            }
            
            // Go to the next destination mbuf segment.
            
            if (dst_len == 0) {
                if (!(dst = mbuf_next(dst)))
                    return (length == 0);
                dst_len = mbuf_len(dst);
                dst_dat = (caddr_t)mbuf_data(dst);
            }
			
        } /* while (src_len) */
		
        src = mbuf_next(src);
		
    } /* while (src) */
    return (length == 0);   // returns true on success.
}

mbuf_t ViaVelocity::velocity_alloc_packet( UInt32 size )
{
    return getPacket( size, MBUF_WAITOK, 64, 64 );
}


mbuf_t ViaVelocity::velocity_copy_packet(mbuf_t m, UInt32 size)
{
	mbuf_t mn;
	
    assert(m != NULL);
	
    // If size is zero, then size is taken from the source mbuf.
	
    if (size == 0) size = mbuf_pkthdr_len(m);
	
    // Copy the current mbuf to the new mbuf, and return the new mbuf.
    // The input mbuf is left intact.
	
    if ( (mn = velocity_alloc_packet(size)) == 0 )
        return 0;
	
    if (!IO_COPY_MBUF(m, mn, size))
    {
        freePacket(mn); mn = 0;
    }
	
	// fill
	if( size < ETH_ZLEN ){
		caddr_t p = (caddr_t)mbuf_data(mn);
		bzero(p+size,(ETH_ZLEN-size));
		mbuf_setlen(mn,ETH_ZLEN);
	}
	
    return mn;
}

#endif
