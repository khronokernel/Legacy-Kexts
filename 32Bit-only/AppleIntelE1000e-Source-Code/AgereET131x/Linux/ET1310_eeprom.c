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
 * ET1310_eeprom.c - Code used to access the device's EEPROM
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
 * THIS SOFTWARE IS PROVIDED “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
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
         $Date: 2006/01/18 22:18:33 $
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
#include "ET1310_eeprom.h"

#include "et131x_supp.h"
#include "et131x_adapter.h"
#include "et131x_initpci.h"
#include "et131x_isr.h"

#include "ET1310_tx.h"




/******************************************************************************
   EEPROM Defines
 *****************************************************************************/
/* LBCIF Register Groups (addressed via 32-bit offsets) */
#define LBCIF_DWORD0_GROUP_OFFSET       0xAC
#define LBCIF_DWORD1_GROUP_OFFSET       0xB0

/* LBCIF Registers (addressed via 8-bit offsets) */
#define LBCIF_ADDRESS_REGISTER_OFFSET   0xAC
#define LBCIF_DATA_REGISTER_OFFSET      0xB0
#define LBCIF_CONTROL_REGISTER_OFFSET   0xB1
#define LBCIF_STATUS_REGISTER_OFFSET    0xB2

/* LBCIF Control Register Bits */
#define LBCIF_CONTROL_SEQUENTIAL_READ   0x01
#define LBCIF_CONTROL_PAGE_WRITE        0x02
#define LBCIF_CONTROL_UNUSED1           0x04
#define LBCIF_CONTROL_EEPROM_RELOAD     0x08
#define LBCIF_CONTROL_UNUSED2           0x10
#define LBCIF_CONTROL_TWO_BYTE_ADDR     0x20
#define LBCIF_CONTROL_I2C_WRITE         0x40
#define LBCIF_CONTROL_LBCIF_ENABLE      0x80

/* LBCIF Status Register Bits */
#define LBCIF_STATUS_PHY_QUEUE_AVAIL    0x01
#define LBCIF_STATUS_I2C_IDLE           0x02
#define LBCIF_STATUS_ACK_ERROR          0x04
#define LBCIF_STATUS_GENERAL_ERROR      0x08
#define LBCIF_STATUS_UNUSED             0x30
#define LBCIF_STATUS_CHECKSUM_ERROR     0x40
#define LBCIF_STATUS_EEPROM_PRESENT     0x80

/* Miscellaneous Constraints */
#define MAX_NUM_REGISTER_POLLS          1000
#define MAX_NUM_WRITE_RETRIES           2




/******************************************************************************
   Define macros that allow individual register values to be extracted from a
   DWORD1 register grouping
 *****************************************************************************/
#define EXTRACT_DATA_REGISTER(x)    (UCHAR)(x & 0xFF)
#define EXTRACT_STATUS_REGISTER(x)  (UCHAR)((x >> 16) & 0xFF)
#define EXTRACT_CONTROL_REG(x)      (UCHAR)((x >> 8) & 0xFF)




static INT32 LbcifWriteByte( ET131X_ADAPTER *pAdapter, UINT32 unOffset, UCHAR bByte )
{
    return EEPROM_access( pAdapter, WRITE, unOffset, 8, &bByte );
}


static INT32 LbcifReadDword( ET131X_ADAPTER *pAdapter, UINT32 unOffset, PUINT32 punData )
{
    return EEPROM_access( pAdapter, READ, unOffset, 32, punData );
}


static INT32 LbcifWriteDword( ET131X_ADAPTER *pAdapter,UINT32 unOffset, UINT32 unData )
{
    return EEPROM_access( pAdapter, WRITE, unOffset, 32, &unData );
}




/******************************************************************************
   ROUTINE :  EepromWriteByte
 ******************************************************************************

   DESCRIPTION       : Write a byte to the ET1310's EEPROM
        
   PARAMETERS        : pAdapter         - pointer to our private adapter structure
                       unAddress        - the address to write
                       bData            - the value to write
                       unEepronId       - the ID of the EEPROM
                       unAddressingMode - how the EEPROM is to be accessed
        
   RETURNS           : SUCCESS or FAILURE
        
   REUSE INFORMATION :
        
 *****************************************************************************/
INT32 EepromWriteByte( ET131X_ADAPTER *pAdapter, UINT32 unAddress, UCHAR bData,
                       UINT32 unEepromId, UINT32 unAddressingMode )
{
    INT32  nIndex;
    INT32  nRetries;
    INT32  nError = FALSE;
    INT32  nI2CWriteActive = 0;
    INT32  nWriteSuccessful = 0;
    UCHAR  bControl;
    UCHAR  bStatus = 0;
    UINT32 unDword1 = 0;

    UINT32 unData = 0;
    /*-----------------------------------------------------------------------*/


    /**************************************************************************
       The following excerpt is from "Serial EEPROM HW Design Specification"
       Version 0.92 (9/20/2004):
      
       Single Byte Writes
      
       For an EEPROM, an I2C single byte write is defined as a START
       condition followed by the device address, EEPROM address, one byte
       of data and a STOP condition.  The STOP condition will trigger the
       EEPROM's internally timed write cycle to the nonvolatile memory.
       All inputs are disabled during this write cycle and the EEPROM will
       not respond to any access until the internal write is complete.
       The steps to execute a single byte write are as follows:
      
       1. Check LBCIF Status Register for bits 6 & 3:2 all equal to 0 and
          bits 7,1:0 both equal to 1, at least once after reset. Subsequent
          operations need only to check that bits 1:0 are equal to 1 prior
          to starting a single byte write.
      
       2. Write to the LBCIF Control Register:  bit 7=1, bit 6=1, bit 3=0,
          and bits 1:0 both =0.  Bit 5 should be set according to the type
          of EEPROM being accessed (1=two byte addressing, 0=one byte
          addressing).
      
       3. Write the address to the LBCIF Address Register.
      
       4. Write the data to the LBCIF Data Register (the I2C write will
          begin).
      
       5. Monitor bit 1:0 of the LBCIF Status Register.  When bits 1:0 are
          both equal to 1, the I2C write has completed and the internal
          write cycle of the EEPROM is about to start. (bits 1:0 = 01 is a
          legal state while waiting from both equal to 1, but bits 1:0 = 10
          is invalid and implies that something is broken).
      
       6. Check bit 3 of the LBCIF Status Register.  If  equal to 1, an
           error has occurred.
      
       7. Check bit 2 of the LBCIF Status Register.  If equal to 1 an ACK
          error has occurred on the address phase of the write.  This could
          be due to an actual hardware failure or the EEPROM may still be in
          its internal write cycle from a previous write.  This write oper-
          ation was ignored and must be repeated later.
      
       8. Set bit 6 of the LBCIF Control Register = 0. If another write is
          required, go to step 1.
     *************************************************************************/


    /**************************************************************************
       Step 1:
     *************************************************************************/
    for( nIndex = 0; nIndex < MAX_NUM_REGISTER_POLLS; nIndex++)
    {
        /**********************************************************************
           Read registers grouped in DWORD1
         *********************************************************************/
        if( LbcifReadDword( pAdapter, LBCIF_DWORD1_GROUP_OFFSET, &unDword1 ))
        {
            nError = 1;
            break; 
        }

        bStatus = EXTRACT_STATUS_REGISTER( unDword1 );

        if( bStatus & LBCIF_STATUS_PHY_QUEUE_AVAIL &&
            bStatus & LBCIF_STATUS_I2C_IDLE )
        {
            /******************************************************************
               bits 1:0 are equal to 1
             *****************************************************************/
            break;
        }
    }

    if( nError || ( nIndex >= MAX_NUM_REGISTER_POLLS ))
    {
        return( FAILURE );
    }


    /**************************************************************************
       Step 2:
     *************************************************************************/
    bControl  = 0;
    bControl |= LBCIF_CONTROL_LBCIF_ENABLE | LBCIF_CONTROL_I2C_WRITE;

    if( unAddressingMode == DUAL_BYTE )
    {
        bControl |= LBCIF_CONTROL_TWO_BYTE_ADDR;
    }

    if( LbcifWriteByte( pAdapter, LBCIF_CONTROL_REGISTER_OFFSET, bControl ))
    {
        return( FAILURE );
    }
    
    nI2CWriteActive = 1;


    /**************************************************************************
       Prepare EEPROM address for Step 3
     *************************************************************************/
    unAddress |= ( unAddressingMode == DUAL_BYTE ) ?
                 ( unEepromId << 16 ) : ( unEepromId << 8 );
     
    for( nRetries = 0; nRetries < MAX_NUM_WRITE_RETRIES; nRetries++ )
    {
        /**********************************************************************
           Step 3:
         *********************************************************************/
        if( LbcifWriteDword( pAdapter, LBCIF_ADDRESS_REGISTER_OFFSET, unAddress ))
        {
            break;
        }
        
        /**********************************************************************
           Step 4:
         *********************************************************************/
        if( LbcifWriteByte( pAdapter, LBCIF_DATA_REGISTER_OFFSET, bData ))
        {
            break;
        }

        /**********************************************************************
           Step 5:
         *********************************************************************/
        for( nIndex = 0; nIndex < MAX_NUM_REGISTER_POLLS; nIndex++ )
        {
            /******************************************************************
               Read registers grouped in DWORD1
             *****************************************************************/
            if( LbcifReadDword( pAdapter, LBCIF_DWORD1_GROUP_OFFSET, 
                                &unDword1 ))
            {
                nError = 1;
                break;
            }

            bStatus = EXTRACT_STATUS_REGISTER( unDword1 );
            
            if( bStatus & LBCIF_STATUS_PHY_QUEUE_AVAIL &&
                bStatus & LBCIF_STATUS_I2C_IDLE )
            {
                /**************************************************************
                   I2C write complete
                 *************************************************************/
                break;
            }
        }

        if( nError || ( nIndex >= MAX_NUM_REGISTER_POLLS ))
        {
            break;
        }


        /**********************************************************************
           Step 6:  Don't break here if we are revision 1, this is so we do a 
                    blind write for load bug.
         *********************************************************************/
        if( bStatus & LBCIF_STATUS_GENERAL_ERROR && pAdapter->RevisionID == 0 )
        {
            break;
        }


        /**********************************************************************
           Step 7:
         *********************************************************************/
        if( bStatus & LBCIF_STATUS_ACK_ERROR )
        {
            /******************************************************************
               This could be due to an actual hardware failure or the EEPROM
               may still be in its internal write cycle from a previous write.
               This write operation was ignored and must be repeated later.
             *****************************************************************/
            udelay( 10 );
            continue;
        }

        nWriteSuccessful = 1;
        break;
    }


    /**************************************************************************
       Step 8:
     *************************************************************************/
    udelay( 10 );
    nIndex = 0;
    while( nI2CWriteActive )
    {
        bControl &= ~LBCIF_CONTROL_I2C_WRITE;

        if( LbcifWriteByte( pAdapter, LBCIF_CONTROL_REGISTER_OFFSET, bControl ))
        {
            nWriteSuccessful = 0;
        }
 
        /* Do read until internal ACK_ERROR goes away meaning write completed */
        for( ;; )
        {
            /* unAddress */
            LbcifWriteDword( pAdapter, LBCIF_ADDRESS_REGISTER_OFFSET, unAddress );
    
            for( ;; )
            {
                LbcifReadDword( pAdapter, LBCIF_DATA_REGISTER_OFFSET, &unData );
    
                if( unData & 0x00010000 )
                {
                    break;
                }
            }

            if(( unData & 0x00040000 ) == 0x0 )
            {
                break;
            }
        }

        bControl = EXTRACT_CONTROL_REG( unData );

        if( bControl != 0xC0 || nIndex == 10000 )
        {
            break;
        }

        nIndex++;
    }

    return(( nWriteSuccessful ) ? SUCCESS : FAILURE );
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  EepromReadByte
 ******************************************************************************

   DESCRIPTION       : Read a byte from the ET1310's EEPROM
        
   PARAMETERS        : pAdapter         - pointer to our private adapter structure
                       unAddress        - the address from which to read
                       pbData           - a pointer to a byte in which to store the 
                                          value of the read
                       unEepronId       - the ID of the EEPROM
                       unAddressingMode - how the EEPROM is to be accessed
        
   RETURNS           : SUCCESS or FAILURE
        
   REUSE INFORMATION :
        
 *****************************************************************************/
INT32 EepromReadByte( ET131X_ADAPTER *pAdapter, UINT32 unAddress, PUCHAR pbData,
                      UINT32 unEepromId, UINT32 unAddressingMode )
{
    INT32  nIndex;
    INT32  nError          = 0;
    INT32  nReadSuccessful = 0;
    UCHAR  bControl;
    UCHAR  bStatus         = 0;
    UINT32 unDword1        = 0;
    /*-----------------------------------------------------------------------*/


    /**************************************************************************
       The following excerpt is from "Serial EEPROM HW Design Specification"
       Version 0.92 (9/20/2004):
      
       Single Byte Reads
      
       A single byte read is similar to the single byte write, with the
       exception of the data flow:
      
       1. Check LBCIF Status Register for bits 6 & 3:2 all equal to 0 and
          bits 7,1:0 both equal to 1, at least once after reset. Subsequent
          operations need only to check that bits 1:0 are equal to 1 prior
          to starting a single byte read.
      
       2. Write to the LBCIF Control Register:  bit 7=1, bit 6=0, bit 3=0,
          and bits 1:0 both =0.  Bit 5 should be set according to the type
          of EEPROM being accessed (1=two byte addressing, 0=one byte addr-
          essing).
      
       3. Write the address to the LBCIF Address Register (I2C read will
          begin).
      
       4. Monitor bit 0 of the LBCIF Status Register.  When =1, I2C read is
          complete. (if bit 1 =1 and bit 0 stays =0, a hardware failure has
          occurred).
      
       5. Check bit 2 of the LBCIF Status Register.  If =1, then an error has
          occurred.  The data that has been returned from the PHY may be
          invalid.
      
       6. Regardless of error status, read data byte from LBCIF Data Register.
          If another byte is required, go to step 1.
     *************************************************************************/


    /**************************************************************************
       Step 1:
     *************************************************************************/
    for( nIndex = 0; nIndex < MAX_NUM_REGISTER_POLLS; nIndex++)
    {
        /**********************************************************************
           Read registers grouped in DWORD1
         *********************************************************************/
        if( LbcifReadDword( pAdapter, LBCIF_DWORD1_GROUP_OFFSET, &unDword1 ))
        {
            nError = 1;
            break;
        }

        bStatus = EXTRACT_STATUS_REGISTER( unDword1 );

        if( bStatus & LBCIF_STATUS_PHY_QUEUE_AVAIL &&
            bStatus & LBCIF_STATUS_I2C_IDLE )
        {
            /******************************************************************
               bits 1:0 are equal to 1
             *****************************************************************/
            break;
        }
    }

    if( nError || ( nIndex >= MAX_NUM_REGISTER_POLLS ))
    {
        return( FAILURE );
    }


    /**************************************************************************
       Step 2:
     *************************************************************************/
    bControl  = 0;
    bControl |= LBCIF_CONTROL_LBCIF_ENABLE;

    if( unAddressingMode == DUAL_BYTE )
    {
        bControl |= LBCIF_CONTROL_TWO_BYTE_ADDR;
    }

    if( LbcifWriteByte( pAdapter, LBCIF_CONTROL_REGISTER_OFFSET, bControl ))
    {
        return( FAILURE );
    }
    

    /**************************************************************************
       Step 3:
     *************************************************************************/
    unAddress |= ( unAddressingMode == DUAL_BYTE ) ?
                 ( unEepromId << 16 ) : ( unEepromId << 8 );

    if( LbcifWriteDword( pAdapter, LBCIF_ADDRESS_REGISTER_OFFSET, unAddress ))
    {
        return( FAILURE );
    }
   

    /**************************************************************************
       Step 4:
     *************************************************************************/
    for( nIndex = 0; nIndex < MAX_NUM_REGISTER_POLLS; nIndex++ )
    {
        /**********************************************************************
           Read registers grouped in DWORD1
         *********************************************************************/
        if( LbcifReadDword( pAdapter, LBCIF_DWORD1_GROUP_OFFSET, &unDword1 ))
        {
            nError = 1;
            break;
        }

        bStatus = EXTRACT_STATUS_REGISTER( unDword1 );

        if( bStatus & LBCIF_STATUS_PHY_QUEUE_AVAIL
            && bStatus & LBCIF_STATUS_I2C_IDLE )
        {
            /******************************************************************
               I2C read complete
             *****************************************************************/
            break;
        }
    }

    if( nError || ( nIndex >= MAX_NUM_REGISTER_POLLS ))
    {
        return( FAILURE );
    }


    /**************************************************************************
       Step 5:
     *************************************************************************/
    nReadSuccessful = ( bStatus & LBCIF_STATUS_ACK_ERROR ) ? 0 : 1;


    /**************************************************************************
       Step 6:
     *************************************************************************/
    *pbData = EXTRACT_DATA_REGISTER( unDword1 );

    return( nReadSuccessful ) ? SUCCESS : FAILURE;
}
/*===========================================================================*/




/******************************************************************************
   ROUTINE :  EEPROM_access
 ******************************************************************************

   DESCRIPTION       :
         Performs a read or write of a given data width at a specified offset
         from EEPROM
        
   PARAMETERS        :
         pAdapter    - [in] pointer to the adapter structure
         bAccessFlag - [in] A flag indicating what type of transaction
                            READ/WRITE
         unOffset    - [in] Specifies the offset within the EEPROM
         bWidth      - [in] Defines the width (in bits) of the requested 
                            access.  Valid values are 8 or 32.
         pData       - [in/out] Pointer to a buffer that contains the data 
                                requested by or resulting from the transaction
                                specified by 'bAccessFlag'.
                                Method in which this parameter is processed
                                (de-referenced) is determined by the value of
                                'bWidth', i.e. 8-bit or 32-bit read/write
                                accesses.
        
   RETURNS           :
        result from PCI Config space access
        
   REUSE INFORMATION :
        YES
        
 *****************************************************************************/
UINT32 EEPROM_access( ET131X_ADAPTER *pAdapter, UCHAR bAccessFlag,
                      UINT32 unOffset, UCHAR bWidth, void *pData )
{
	UINT32 ulResult;
    /*-----------------------------------------------------------------------*/
    
    if( bAccessFlag == WRITE )
    {
        /**********************************************************************
           we are doing a write
         *********************************************************************/
        ulResult = pci_slot_information_write( pAdapter->pdev,
                                               unOffset,
                                               pData,
                                               ( bWidth / 8 ));
    }
    else
    {
        /**********************************************************************
           do the read read
         *********************************************************************/
        ulResult = pci_slot_information_read( pAdapter->pdev,
                                              unOffset,
                                              pData,
                                              ( bWidth / 8 ));
    }
	if( ulResult != 0 )
	{
		return( 0 );
	}
	else
	{
		return( ulResult );
	}
}
/*===========================================================================*/
