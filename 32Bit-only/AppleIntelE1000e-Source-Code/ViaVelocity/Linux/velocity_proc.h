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
 * File: velocity_proc.h
 *
 * Purpose: Header file for private proc structures.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */


#ifndef __VELOCITY_PROC_H__
#define __VELOCITY_PROC_H__

#include "velocity.h"

#define VELOCITY_PROC_READ      0x0001
#define VELOCITY_PROC_WRITE     0x0002
#define VELOCITY_PROC_DIR       0x0004
#define VELOCITY_PROC_EOT       0xFFFF
#define VELOCITY_PROC_FILE      (VELOCITY_PROC_READ|VELOCITY_PROC_WRITE)

struct __velocity_info;
struct __velocity_proc_entry;

typedef int (*VELOCITY_PROC_READ_FUNC)(struct __velocity_proc_entry*, char* buf);
typedef int (*VELOCITY_PROC_WRITE_FUNC)(struct __velocity_proc_entry*, const char* buf, unsigned long len);

typedef enum __velocity_proc_rmon_type {
    RMON_DropEvents           = 0,  // 0
    RMON_Octets,                    // 1
    RMON_Pkts,                      // 2
    RMON_BroadcastPkts,             // 3
    RMON_MulticastPkts,             // 4
    RMON_CRCAlignErrors,            // 5
    RMON_UndersizePkts,             // 6
    RMON_OversizePkts,              // 7
    RMON_Fragments,                 // 8
    RMON_Jabbers,                   // 9
    RMON_Collisions,                // 10
    RMON_Pkts64Octets,              // 11
    RMON_Pkts65to127Octets,         // 12
    RMON_Pkts128to255Octets,        // 13
    RMON_Pkts256to511Octets,        // 14
    RMON_Pkts512to1023Octets,       // 15
    RMON_Pkts1024to1518Octets,      // 16
    RMON_TAB_SIZE
} PROC_RMON_TYPE, *PPROC_RMON_TYPE;

typedef struct __velocity_proc_entry {
    char                                name[128];
    int                                 type;
    VELOCITY_PROC_READ_FUNC             read_proc;
    VELOCITY_PROC_WRITE_FUNC            write_proc;
    int                                 data;
    const struct __velocity_proc_entry* childs;
    U8                                  byRevId;
    struct proc_dir_entry*              pOsEntry;
    struct proc_dir_entry*              pOsParent;
    struct __velocity_info*             pInfo;
    const struct __velocity_proc_entry* siblings;
} VELOCITY_PROC_ENTRY, *PVELOCITY_PROC_ENTRY;


BOOL velocity_init_proc_fs(void);
void velocity_free_proc_fs(void);

BOOL velocity_create_proc_entry(struct __velocity_info* pInfo);
void velocity_free_proc_entry(struct __velocity_info* pInfo);

#endif // __VELOCITY_PROC_H__
