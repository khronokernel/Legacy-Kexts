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
 * File: velocity_stats.h
 *
 * Purpose: Definitions for MAC hardware counters.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */


#ifndef __VELOCITY_STATS_H__
#define __VELOCITY_STATS_H__
/*
#if !defined(__TTYPE_H__)
#include "ttype.h"
#endif
#if !defined(__TTCHAR_H__)
#include "ttchar.h"
#endif
#if !defined(__TETHER_H__)
#include "tether.h"
#endif
#if !defined(__DESC_H__)
#include "desc.h"
#endif
*/
typedef struct tagSAdapterInfo
    SAdapterInfo, DEF* PSAdapterInfo;

/*---------------------  Export Definitions -------------------------*/
//
// Hardware counter
//
#define MAX_HW_MIB_COUNTER     32

typedef struct tagSHWMibCounter {
    DWORD       ifRxAllPkts;
    DWORD       ifRxOkPkts;
    DWORD       ifTxOkPkts;
    DWORD       ifRxErrorPkts;
    DWORD       ifRxRuntOkPkt;
    DWORD       ifRxRuntErrPkt;
    DWORD       ifRx64Pkts;
    DWORD       ifTx64Pkts;
    DWORD       ifRx65To127Pkts;
    DWORD       ifTx65To127Pkts;
    DWORD       ifRx128To255Pkts;
    DWORD       ifTx128To255Pkts;
    DWORD       ifRx256To511Pkts;
    DWORD       ifTx256To511Pkts;
    DWORD       ifRx512To1023Pkts;
    DWORD       ifTx512To1023Pkts;
    DWORD       ifRx1024To1518Pkts;
    DWORD       ifTx1024To1518Pkts;
    DWORD       ifTxEtherCollisions;
    DWORD       ifRxPktCRCE;
    DWORD       ifRxJumboPkts;
    DWORD       ifTxJumboPkts;
    DWORD       ifRxMacControlFrames;
    DWORD       ifTxMacControlFrames;
    DWORD       ifRxPktFAE;
    DWORD       ifRxLongOkPkt;
    DWORD       ifRxLongPktErrPkt;
    DWORD       ifTXSQEErrors;
    DWORD       ifRxNobuf;
    DWORD       ifRxSymbolErrors;
    DWORD       ifInRangeLenthErrors;
    DWORD       ifLateCollisions;
} SHWMibCounter, DEF* PSHWMibCounter;

//
// MIB2 counter
//
typedef struct tagSMib2Counter {
    LONG    ifIndex;
    TCHAR   ifDescr[256];               // max size 255 plus zero ending
                                        // e.g. "interface 1"
    LONG    ifType;
    LONG    ifMtu;
    DWORD   ifSpeed;
    BYTE    ifPhysAddress[U_ETHER_ADDR_LEN];
    LONG    ifAdminStatus;
    LONG    ifOperStatus;
    DWORD   ifLastChange;
    DWORD   ifInOctets;
    DWORD   ifInUcastPkts;
    DWORD   ifInNUcastPkts;
    DWORD   ifInDiscards;
    DWORD   ifInErrors;
    DWORD   ifInUnknownProtos;
    DWORD   ifOutOctets;
    DWORD   ifOutUcastPkts;
    DWORD   ifOutNUcastPkts;
    DWORD   ifOutDiscards;
    DWORD   ifOutErrors;
    DWORD   ifOutQLen;
    DWORD   ifSpecific;
} SMib2Counter, DEF* PSMib2Counter;


// Value in the ifType entry
#define ETHERNETCSMACD      6           //

// Value in the ifAdminStatus/ifOperStatus entry
#define UP                  1           //
#define DOWN                2           //
#define TESTING             3           //


//
// RMON counter
//
typedef struct tagSRmonCounter {
    LONG    etherStatsIndex;
    DWORD   etherStatsDataSource;
    DWORD   etherStatsDropEvents;
    DWORD   etherStatsOctets;
    DWORD   etherStatsPkts;
    DWORD   etherStatsBroadcastPkts;
    DWORD   etherStatsMulticastPkts;
    DWORD   etherStatsCRCAlignErrors;
    DWORD   etherStatsLengthErrPkts;
    DWORD   etherStatsJumbos;
    DWORD   etherStatsCollisions;
    DWORD   etherStatsPkt64Octets;
    DWORD   etherStatsPkt65to127Octets;
    DWORD   etherStatsPkt128to255Octets;
    DWORD   etherStatsPkt256to511Octets;
    DWORD   etherStatsPkt512to1023Octets;
    DWORD   etherStatsPkt1024to1518Octets;
    DWORD   etherStatsOwners;
    DWORD   etherStatsStatus;
} SRmonCounter, DEF* PSRmonCounter;

// Value in the etherStatsStatus entry
#define VALID               1           //
#define CREATE_REQUEST      2           //
#define UNDER_CREATION      3           //
#define INVALID             4           //

//
// statistic counter
//
typedef struct tagSStatCounter {
    // ISR0,1 status count
    //
    DWORD   dwIsrPRxOK;
    DWORD   dwIsrPTxOK;
    DWORD   dwIsrRxOK;
    DWORD   dwIsrTxOK;

    DWORD   dwIsrTxWB0I;
    DWORD   dwIsrTxWB1I;

    DWORD   dwIsrRxPktRace;
    DWORD   dwIsrRxFifoOvfl;
    DWORD   dwIsrRxFlowOn;
    DWORD   dwIsrRxNoBuf;
    DWORD   dwIsrRxNoBufP;

    DWORD   dwIsrLinkStatusChg;


    // MISR0
    DWORD   dwMisrSoftTimer0;
    DWORD   dwMisrSoftTimer1;
    DWORD   dwMisrPWE;
    DWORD   dwMisrPhyStatChg;
    DWORD   dwMisrShutDown;
    DWORD   dwMisrMibOvfl;
    DWORD   dwMisrUserDef;

     // MISR1
    DWORD   dwMisrTxDmaErr;
    DWORD   dwMisrRxDmaErr;

    // TXE_SR/RXE_SR
    DWORD   dwTxEsrDmaBusErr;
    DWORD   dwTxEsrTDRBusErr;
    DWORD   dwTxEsrTDWBusErr;
    DWORD   dwTxEsrTDStrErr;
    DWORD   dwRxEsrDmaBusErr;
    DWORD   dwRxEsrRDRBusErr;
    DWORD   dwRxEsrRDWBusErr;
    DWORD   dwRxEsrRDStrErr;


    DWORD   dwIsrUnknown;               // unknown interrupt count

    DWORD   dwIntLinkUp;
    DWORD   dwIntLinkDown;

    // RSR0,1 status count
    //
    DWORD   dwRsrCRCErr;
    DWORD   dwRsrFrmAlgnErr;
    DWORD   dwRsrCheckSumErr;
    DWORD   dwRsrLengthErr;
    DWORD   dwRsrPCSSymErr;
    DWORD   dwRsrSNTAG;
    DWORD   dwRsrDETAG;

    DWORD   dwRsrVIDMiss;
    DWORD   dwRsrRxTag;
    DWORD   dwRsrBroadcast;
    DWORD   dwRsrMulticast;
    DWORD   dwRsrDirected;
    DWORD   dwRsrPerfectMatch;
    DWORD   dwRsrOK;

    DWORD   dwRsrRxFrmLen64;
    DWORD   dwRsrRxFrmLen65_127;
    DWORD   dwRsrRxFrmLen128_255;
    DWORD   dwRsrRxFrmLen256_511;
    DWORD   dwRsrRxFrmLen512_1023;
    DWORD   dwRsrRxFrmLen1024_1518;

    DWORD   dwRsrRxPacket;
    DWORD   dwRsrRxOctet;
    DWORD   dwRsrRxTagFrame;

    // TSR0,1 status count
    //
    DWORD   dwTsrTotalColRetry;         // total collision retry count
    DWORD   dwTsrOnceCollision;         // this packet only occur one collision
    DWORD   dwTsrMoreThanOnceCollision; // this packet occur more than one collision

    DWORD   dwTsrCollision;             // this packet has ever occur collision,
                                        // that is (dwTsrOnceCollision + dwTsrMoreThanOnceCollision)
    DWORD   dwTsrHeartBeat;
    DWORD   dwTsrAbort;
    DWORD   dwTsrLateCollision;
    DWORD   dwTsrCarrierLost;
    DWORD   dwTsrJumboAbort;
    DWORD   dwTsrLNKFL;
    DWORD   dwTsrShutDown;
    DWORD   dwTsrGMII;
    DWORD   dwTsrFDX;

    DWORD   dwTsrErr;                   // GOWC | OWT | ABT | SHDN
    DWORD   dwTsrOK;

    DWORD   dwTsrTxPacket;
    DWORD   dwTsrTxOctet;

    DWORD   dwTsrBroadcast;
    DWORD   dwTsrMulticast;
    DWORD   dwTsrDirected;


    // RD/TD count
    DWORD   dwCntRxFrmLength;
    DWORD   dwCntTxBufLength;

    BYTE    abyCntRxPattern[16];
    BYTE    abyCntTxPattern[16];

    // for pingpong
    DWORD   dwCntNoResponse;
    DWORD   dwCntSerialNumErr;

    // PATCH....
    DWORD   dwCntRxDataErr;             // rx buffer data software compare CRC err count
    DWORD   dwIsrContinuePktRace;       // if continueous packet-race happen, should reset
    DWORD   dwIsrContinueNoBuf;         // if continueous no-buffer

} SStatCounter, DEF* PSStatCounter;

/*---------------------  Export Classes  ----------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Export Functions  --------------------------*/
#ifdef __cplusplus
extern "C" {                            /* Assume C declarations for C++ */
#endif /* __cplusplus */


VOID STAvClearAllCounter(PSStatCounter pStatistic);

VOID STAvUpdateEsrStatCounter(PSStatCounter pStatistic, BYTE byEsr, BOOL bTx);
VOID STAvUpdateIsrStatCounter(PSStatCounter pStatistic, DWORD dwIsr, WORD wEsr);

VOID STAvUpdateRDStatCounter(PSStatCounter pStatistic, PSRxDesc prdCurr,
                             PBYTE pbyBuffer, UINT cbFrameLength);

VOID STAvUpdateRDStatCounterEx(PSStatCounter pStatistic, PSRxDesc prdCurr,
                               PBYTE pbyBuffer, UINT cbFrameLength);

VOID STAvUpdateTDStatCounter(PSStatCounter pStatistic, PSTxDesc ptdCurr,
                             PBYTE pbyBuffer, UINT cbFrameLength);

VOID STAvUpdateTDStatCounterEx(PSStatCounter pStatistic, PSTxDesc ptdCurr,
                               PBYTE pbyBuffer, DWORD cbFrameLength);

VOID STAvClearHWMIBCounter(DWORD dwIoBase);
VOID STAvEnableHWMIBCounter(DWORD dwIoBase);
VOID STAvUpdateHWMIBCounter(DWORD dwIoBase, PSHWMibCounter pCounter);

#ifdef __cplusplus
}                                       /* End of extern "C" { */
#endif /* __cplusplus */


#endif // __VELOCITY_STATS_H__
