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
 * File: velocity_cfg.h
 *
 * Purpose: The OS dependent types & constants.
 *
 * Author: Chuang Liang-Shing, AJ Jiang
 *
 * Date: Jan 24, 2003
 *
 */


/*
* Description: This file defined OS dependent macros & defines
*/

#ifndef __VELOCITY_CFG_H__
#define __VELOCITY_CFG_H__

#ifndef	__APPLE__

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <linux/config.h>
#endif

#include <linux/types.h>

typedef
struct _version {
    UINT8   major;
    UINT8   minor;
    UINT8   build;
} version_t, *pversion_t;

#define VELOCITY_NAME          "velocityget"
#define VELOCITY_FULL_DRV_NAM  "VIA Networking Velocity Family Gigabit Ethernet Adapter Driver"

#ifndef MAJOR_VERSION
#define MAJOR_VERSION       "1"
#endif

#ifndef MINOR_VERSION
#define MINOR_VERSION       "43"
#endif

#ifndef VELOCITY_VERSION
#define VELOCITY_VERSION    MAJOR_VERSION"."MINOR_VERSION
#endif



#define MAX_UINTS           8
#define OPTION_DEFAULT      { [0 ... MAX_UINTS-1] = -1}

#define REV_ID_VT6110       (0)
#define DEVICE_ID           (0x3119)

#define VAR_USED(p)     do {(p)=(p);} while (0)

#endif	// __APPLE__
#endif // __VELOCITY_CFG_H__
