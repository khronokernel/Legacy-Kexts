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
 * File: velocity_dbg.h
 *
 * Purpose: Hearder file for inline debug routine
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */


#ifndef __VELOCITY_DBG_H__
#define __VELOCITY_DBG_H__

#ifdef	__APPLE__
#define	printk	IOLog
#define	KERN_ERR
#endif

#ifdef VELOCITY_DEBUG
#define ASSERT(x) { \
    if (!(x)) { \
        printk(KERN_ERR "assertion %s failed: file %s line %d\n", #x,\
        __FUNCTION__, __LINE__);\
        *(int*) 0=0;\
    }\
}
#define VELOCITY_DBG(p,args...) printk(p, ##args)
#else
#define ASSERT(x)
#define VELOCITY_DBG(x)
#endif


#define VELOCITY_PRT_CAMMASK(p,t) {\
    int i;\
    if ((t)==VELOCITY_MULTICAST_CAM) {\
        for (i=0;i<(MCAM_SIZE/8);i++)\
            printk("%02X",(p)->abyMCAMMask[i]);\
    }\
    else {\
        for (i=0;i<(VCAM_SIZE/8);i++)\
            printk("%02X",(p)->abyVCAMMask[i]);\
    }\
    printk("\n");\
}

#endif // __VELOCITY_DBG_H__
