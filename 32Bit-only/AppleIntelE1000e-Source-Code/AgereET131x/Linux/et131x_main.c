/*******************************************************************************
 * Agere Systems Inc.
 * 10/100/1000 Base-T Ethernet Driver for the ET1301 and ET131x series MACs
 *
 * Copyright © 2005 Agere Systems Inc. 
 * All rights reserved.
 *   http://www.agere.com
 *
 *------------------------------------------------------------------------------
 *
 * et131x_main.c - This file contains the driver's main Linux entry points.
 *
 *------------------------------------------------------------------------------
 *
 * SOFTWARE LICENSE
 *
 * This software is provided subject to the following terms and conditions,
 * which you should read carefully before using the software.  Using this
 * software indicates your acceptance of these terms and conditions.  If you do
 * not agree with these terms and conditions, do not use the software.
 *
 * Copyright © 2005 Agere Systems Inc.
 * All rights reserved.
 *
 * Redistribution and use in source or binary forms, with or without
 * modifications, are permitted provided that the following conditions are met:
 *
 * . Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following Disclaimer as comments in the code as
 *    well as in the documentation and/or other materials provided with the
 *    distribution.
 * 
 * . Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following Disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * . Neither the name of Agere Systems Inc. nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Disclaimer
 *
 * THIS SOFTWARE IS PROVIDED ìAS ISî AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, INFRINGEMENT AND THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  ANY
 * USE, MODIFICATION OR DISTRIBUTION OF THIS SOFTWARE IS SOLELY AT THE USERS OWN
 * RISK. IN NO EVENT SHALL AGERE SYSTEMS INC. OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, INCLUDING, BUT NOT LIMITED TO, CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *   
 ******************************************************************************/




/******************************************************************************
 *  VERSION CONTROL INFORMATION
 ******************************************************************************

      $RCSFile: $
         $Date: 2005/08/01 19:35:13 $
     $Revision: 1.6 $
         $Name: T_20060131_v1-2-2 $
       $Author: vjs $

 *****************************************************************************/




/******************************************************************************
   Includes
 *****************************************************************************/
#include "et131x_version.h"
#include "et131x_debug.h"
#include "et131x_defs.h"

#include "ET1310_phy.h"
#include "ET1310_pm.h"
#include "ET1310_jagcore.h"

#include "et131x_supp.h"
#include "et131x_adapter.h"
#include "et131x_initpci.h"




/******************************************************************************
   Modinfo parameters (filled out using defines from et131x_version.h)
 *****************************************************************************/
#ifndef	__APPLE__
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_INFO );
MODULE_LICENSE( DRIVER_LICENSE );

#endif


/******************************************************************************
   Module Parameters and related data for debugging facilities
 *****************************************************************************/
#if ET131X_DBG

static u32 et131x_debug_level = DBG_LVL;
static u32 et131x_debug_flags = DBG_DEFAULTS;

module_param( et131x_debug_level, uint, 0 );
module_param( et131x_debug_flags, uint, 0 );

MODULE_PARM_DESC( et131x_debug_level,
                 "Level of debugging desired (0-7)" );

dbg_info_t et131x_info     = { DRIVER_NAME_EXT, 0, 0 };
dbg_info_t *et131x_dbginfo = &et131x_info;

#endif  /* ET131X_DBG */




/******************************************************************************
   ROUTINE :  et131x_init_module
 ******************************************************************************

   DESCRIPTION       : The "main" entry point called on driver initialization
        
   PARAMETERS        : N/A
        
   RETURNS           : 0 on success
                       errno on failure (as defined in errno.h)
        
   REUSE INFORMATION :
        
 *****************************************************************************/
int et131x_init_module( void )
{
    int result;
    /*-----------------------------------------------------------------------*/


    DBG_FUNC( "et131x_init_module" );


#if ET131X_DBG
    /**************************************************************************
       Set the level of debug messages displayed using the module parameter
     *************************************************************************/
    et131x_dbginfo->dbgFlags = et131x_debug_flags;

    switch( et131x_debug_level )
    {
    case 7:
        et131x_dbginfo->dbgFlags |= ( DBG_RX_ON | DBG_TX_ON );

    case 6:
        et131x_dbginfo->dbgFlags |= DBG_PARAM_ON;

    case 5:
        et131x_dbginfo->dbgFlags |= DBG_VERBOSE_ON;

    case 4:
        et131x_dbginfo->dbgFlags |= DBG_TRACE_ON;

    case 3:
        et131x_dbginfo->dbgFlags |= DBG_NOTICE_ON;

    case 2:
    case 1:
    case 0:
    default:
        break;
    }
#endif  /* ET131X_DBG */

    DBG_ENTER( et131x_dbginfo );
    DBG_PRINT( "%s\n", DRIVER_INFO );


    result = et131x_pci_register( );


    DBG_LEAVE( et131x_dbginfo );
    return result;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  et131x_cleanup_module
 ******************************************************************************

   DESCRIPTION       : The entry point called on driver cleanup
        
   PARAMETERS        : N/A
        
   RETURNS           : N/A
        
   REUSE INFORMATION :
        
 *****************************************************************************/
void et131x_cleanup_module( void )
{
    DBG_FUNC( "et131x_cleanup_module" );
    DBG_ENTER( et131x_dbginfo );


    et131x_pci_unregister( );


    DBG_LEAVE( et131x_dbginfo );
    return;
}
/*===========================================================================*/




/******************************************************************************
   These macros map the driver-specific init_module() and cleanup_module() 
   routines so they can be called by the kernel.
 *****************************************************************************/
module_init(et131x_init_module);
module_exit(et131x_cleanup_module);
