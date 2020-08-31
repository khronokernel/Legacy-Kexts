#include "AgereET131x.h"


/******************************************************************************
 ROUTINE :  MiRead
 ******************************************************************************
 
 DESCRIPTION       : Used to read from the PHY through the MII Interface on
 the MAC.
 
 PARAMETERS        : adapter  - pointer to our private adapter structure
 xcvrAddr - the address of the transciever
 xcvrReg  - the register to read
 value    - pointer to a 16-bit value in which the value
 will be stored.
 
 *****************************************************************************/
static int MiRead( ET131X_ADAPTER *adapter, UINT8 xcvrAddr, UINT8 xcvrReg, UINT16 *value )
{
    int                 status = 0;
	UINT32              delay;
    MII_MGMT_ADDR_t     miiAddr;
    MII_MGMT_CMD_t      miiCmd;
	
	
	
    /**************************************************************************
	 Save a local copy of the registers we are dealing with so we can set 
	 them back
     *************************************************************************/
    miiAddr.value = adapter->CSRAddress->mac.mii_mgmt_addr.value;
    miiCmd.value  = adapter->CSRAddress->mac.mii_mgmt_cmd.value;
	
	
    /**************************************************************************
	 Stop the current operation
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x0;
	
	
    /**************************************************************************
	 Set up the register we need to read from on the correct PHY
     *************************************************************************/
    {
        MII_MGMT_ADDR_t mii_mgmt_addr = {0};
		
        mii_mgmt_addr.bits.phy_addr = xcvrAddr;
        mii_mgmt_addr.bits.reg_addr = xcvrReg;
		
        adapter->CSRAddress->mac.mii_mgmt_addr = mii_mgmt_addr;
    }
	
	
    /**************************************************************************
	 Kick the read cycle off
     *************************************************************************/
    delay = 0;
	
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x1;
	
    do 
    {
        udelay( 50 );
        delay++;
    } while(( adapter->CSRAddress->mac.mii_mgmt_indicator.bits.not_valid ||
			 adapter->CSRAddress->mac.mii_mgmt_indicator.bits.busy ) && 
            ( delay < 50 ));
	
	
    /**************************************************************************
	 If we hit the max delay, we could not read the register
     *************************************************************************/
	if( delay >= 50 )
    {
        DBG_WARNING( "xcvrReg 0x%08x could not be read\n", xcvrReg );
        DBG_WARNING( "status is  0x%08x\n", 
					adapter->CSRAddress->mac.mii_mgmt_indicator.value );
		
        status = -EIO;
    }
	
	
    /**************************************************************************
	 If we hit here we were able to read the register and we need to return
	 the value to the caller
     *************************************************************************/
    *value = (UINT16)adapter->CSRAddress->mac.mii_mgmt_stat.bits.phy_stat;
	
	
    /**************************************************************************
	 Stop the read operation
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x0;
	
	
    /**************************************************************************
	 set the registers we touched back to the state at which we entered 
	 this function
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_addr.value = miiAddr.value;
    adapter->CSRAddress->mac.mii_mgmt_cmd.value  = miiCmd.value;
	
	
    return status;
}



/******************************************************************************
 ROUTINE :  MiWrite
 ******************************************************************************
 
 DESCRIPTION       : Used to write to a PHY register through the MII
 interface of the MAC. Updated for the ET1310.
 
 PARAMETERS        : adapter  - pointer to our private adapter structure
 xcvrAddr - the address of the transciever
 xcvrReg  - the register to read
 value    - 16-bit value to write
 
 RETURNS           : 0 on success
 errno on failure (as defined in errno.h)
 
 
 *****************************************************************************/
static int MiWrite( ET131X_ADAPTER *adapter, UINT8 xcvrAddr, UINT8 xcvrReg, UINT16 value )
{
    int             status = 0;
	UINT32          delay;
    MII_MGMT_ADDR_t miiAddr;
    MII_MGMT_CMD_t  miiCmd;
	
    /**************************************************************************
	 Save a local copy of the registers we are dealing with so we can set 
	 them back
     *************************************************************************/
    miiAddr.value = adapter->CSRAddress->mac.mii_mgmt_addr.value;
    miiCmd.value  = adapter->CSRAddress->mac.mii_mgmt_cmd.value;
	
	
    /**************************************************************************
	 Stop the current operation
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x0;
	
	
    /**************************************************************************
	 Set up the register we need to write to on the correct PHY
     *************************************************************************/
    {
        MII_MGMT_ADDR_t mii_mgmt_addr = {0};
		
        mii_mgmt_addr.bits.phy_addr = xcvrAddr;
        mii_mgmt_addr.bits.reg_addr = xcvrReg;
		
        adapter->CSRAddress->mac.mii_mgmt_addr = mii_mgmt_addr;
    }
	
	
    /**************************************************************************
	 Add the value to write to the registers to the mac
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_ctrl.value = value;
    delay = 0;
	
    do
    {
        udelay( 50 );
        delay++;
    } while(( adapter->CSRAddress->mac.mii_mgmt_indicator.bits.busy ) && 
            ( delay < 100 ));
	
	
    /**************************************************************************
	 If we hit the max delay, we could not write the register
     *************************************************************************/
	if( delay == 100 )
    {
        UINT16 TempValue;
		
        MiRead( adapter, xcvrAddr, xcvrReg, &TempValue );
        
        status = -EIO;
    }
	
	
    /**************************************************************************
	 Stop the write operation
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_cmd.value = 0x0;
	
	
    /**************************************************************************
	 set the registers we touched back to the state at which we entered 
	 this function
     *************************************************************************/
    adapter->CSRAddress->mac.mii_mgmt_addr.value = miiAddr.value;
    adapter->CSRAddress->mac.mii_mgmt_cmd.value  = miiCmd.value;
	
    return status;
}

static void ET1310_PhyAccessMiBit( ET131X_ADAPTER *pAdapter,
								  UINT16 action,
								  UINT16 regnum,
								  UINT16 bitnum,
								  UINT8 *value )
{
    UINT16 reg;
    UINT16 mask = 0;
	
    // Create a mask to isolate the requested bit
    mask = 0x0001 << bitnum;
	
    // Read the requested register
    MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
		   regnum, &reg );
	
    switch( action )
    {
		case TRUEPHY_BIT_READ:
			if( value != NULL )
			{
				*value = ( reg & mask ) >> bitnum;
			}
			
			break;
			
		case TRUEPHY_BIT_SET:
			reg |= mask;
			MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr, regnum, reg );
			break;
			
		case TRUEPHY_BIT_CLEAR:
			reg &= ~mask;
			MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr, regnum, reg );
			break;
			
		default:
			break;
    }
	
    return;
}

static void ET1310_PhyAutoNeg( ET131X_ADAPTER *pAdapter, BOOL_t enable )
{
    UINT16 usData;
	
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );
	
    if( enable == TRUE )
    {
        // Autonegotiation ON
        usData |= 0x1000;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
    else
    {
        // Autonegotiation OFF
        usData &= ~0x1000;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
	
    return;
}


static void ET1310_PhyDuplexMode( ET131X_ADAPTER *pAdapter, UINT16 duplex )
{
    UINT16 usData;
	
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );
	
    if( duplex == TRUEPHY_DUPLEX_FULL )
    {
        // Set Full Duplex
        usData |= 0x100;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
    else
    {
        // Set Half Duplex
        usData &= ~0x100;
        MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
}


static void ET1310_PhySpeedSelect( ET131X_ADAPTER *pAdapter, UINT16 speed )
{
    UINT16 usData;
	
    // Read the PHY control register
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );
	
    // Clear all Speed settings (Bits 6, 13)
    usData &= ~0x2040;
	
    // Reset the speed bits based on user selection
    switch( speed )
    {
		case TRUEPHY_SPEED_10MBPS:
			// Bits already cleared above, do nothing
			break;
			
		case TRUEPHY_SPEED_100MBPS:
			// 100M == Set bit 13
			usData |= 0x2000;
			break;
			
		case TRUEPHY_SPEED_1000MBPS:
		default:
			usData |= 0x0040;
			break;
    }
	
    // Write back the new speed
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
}


static void ET1310_PhyAdvertise1000BaseT( ET131X_ADAPTER *pAdapter, UINT16 duplex )
{
    UINT16 usData;
	
    // Read the PHY 1000 Base-T Control Register
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_1000_CONTROL, &usData );
	
    // Clear Bits 8,9
    usData &= ~0x0300;
	
    switch( duplex )
    {
		case TRUEPHY_ADV_DUPLEX_NONE:
			// Duplex already cleared, do nothing
			break;
			
		case TRUEPHY_ADV_DUPLEX_FULL:
			// Set Bit 9
			usData |= 0x0200;
			break;
			
		case TRUEPHY_ADV_DUPLEX_HALF:
			// Set Bit 8
			usData |= 0x0100;
			break;
			
		case TRUEPHY_ADV_DUPLEX_BOTH:
		default:
			usData |= 0x0300;
			break;
    }
	
    // Write back advertisement
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_1000_CONTROL, usData );
	
    return;
}


static void ET1310_PhyAdvertise100BaseT( ET131X_ADAPTER *pAdapter, UINT16 duplex )
{
    UINT16 usData;
	
    // Read the Autonegotiation Register (10/100)
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_AUTO_ADVERTISEMENT, &usData );
	
    // Clear bits 7,8
    usData &= ~0x0180;
	
    switch( duplex )
    {
		case TRUEPHY_ADV_DUPLEX_NONE:
			// Duplex already cleared, do nothing
			break;
			
		case TRUEPHY_ADV_DUPLEX_FULL:
			// Set Bit 8
			usData |= 0x0100;
			break;
			
		case TRUEPHY_ADV_DUPLEX_HALF:
			// Set Bit 7
			usData |= 0x0080;
			break;
			
		case TRUEPHY_ADV_DUPLEX_BOTH:
		default:
			// Set Bits 7,8
			usData |= 0x0180;
			break;
    }
	
    // Write back advertisement
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_AUTO_ADVERTISEMENT, usData );
	
    return;
}


static void ET1310_PhyAdvertise10BaseT( ET131X_ADAPTER *pAdapter, UINT16 duplex )
{
    UINT16 usData;
	
    // Read the Autonegotiation Register (10/100)
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_AUTO_ADVERTISEMENT, &usData );
	
    // Clear bits 5,6
    usData &= ~0x0060;
	
    switch( duplex )
    {
		case TRUEPHY_ADV_DUPLEX_NONE:
			// Duplex already cleared, do nothing
			break;
			
		case TRUEPHY_ADV_DUPLEX_FULL:
			// Set Bit 6
			usData |= 0x0040;
			break;
			
		case TRUEPHY_ADV_DUPLEX_HALF:
			// Set Bit 5
			usData |= 0x0020;
			break;
			
		case TRUEPHY_ADV_DUPLEX_BOTH:
		default:
			// Set Bits 5,6
			usData |= 0x0060;
			break;
    }
	
    // Write back advertisement
    MiWrite( pAdapter, pAdapter->Stats.xcvr_addr, PHY_AUTO_ADVERTISEMENT, usData );
	
    return;
}


static void ET1310_PhyLinkStatus( ET131X_ADAPTER *pAdapter, 
								 UCHAR  *ucLinkStatus,
								 UINT32 *uiAutoNeg,
								 UINT32 *uiLinkSpeed,
								 UINT32 *uiDuplexMode,
								 UINT32 *uiMdiMdix,
								 UINT32 *uiMasterSlave,
								 UINT32 *uiPolarity )
{
    UINT16 usMiStatus     = 0;
    UINT16 us1000BaseT    = 0;
    UINT16 usVmiPhyStatus = 0;
    UINT16 usControl      = 0;
	
	
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_STATUS, &usMiStatus );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_1000_STATUS, &us1000BaseT );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_PHY_STATUS, &usVmiPhyStatus );
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usControl );
	
    if( ucLinkStatus )
    {
        *ucLinkStatus  = (unsigned char)(( usVmiPhyStatus & 0x0040 ) ? 1 : 0 );
    }
	
    if( uiAutoNeg )
    {
        *uiAutoNeg     = ( usControl & 0x1000 ) ? (( usVmiPhyStatus & 0x0020 ) ? TRUEPHY_ANEG_COMPLETE : TRUEPHY_ANEG_NOT_COMPLETE ) : TRUEPHY_ANEG_DISABLED;
    }
	
    if( uiLinkSpeed )
    {
        *uiLinkSpeed   = ( usVmiPhyStatus & 0x0300 ) >> 8;
    }
	
    if( uiDuplexMode )
    {
        *uiDuplexMode  = ( usVmiPhyStatus & 0x0080 ) >> 7;
    }
	
    if( uiMdiMdix )
    {
        /* NOTE: Need to complete this */
        *uiMdiMdix     = 0;
    }
	
    if( uiMasterSlave )
    {
        *uiMasterSlave = ( us1000BaseT & 0x4000 ) ? TRUEPHY_CFG_MASTER : TRUEPHY_CFG_SLAVE;
    }
	
    if( uiPolarity )
    {
        *uiPolarity    = ( usVmiPhyStatus & 0x0400 ) ? TRUEPHY_POLARITY_INVERTED : TRUEPHY_POLARITY_NORMAL;
    }
	
    return;
}


static void ET1310_PhyAndOrReg( ET131X_ADAPTER *pAdapter,
							   UINT16 regnum,
							   UINT16 andMask,
							   UINT16 orMask )
{
    UINT16 reg;
	
    // Read the requested register
    MiRead( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
		   regnum, &reg );
	
    // Apply the AND mask
    reg &= andMask;
	
    // Apply the OR mask
    reg |= orMask;
	
    // Write the value back to the register
    MiWrite( pAdapter, (UINT8)pAdapter->Stats.xcvr_addr,
			regnum, reg );
	
    return;
}


/******************************************************************************
 ROUTINE:  ConfigFlowControl
 ******************************************************************************
 
 DESCRIPTION:
 Used to configure the MAC STAT section of the JAGCore
 
 PARAMETERS :
 pAdapter - pointer to our adapter structure
 
 RETURNS    :
 NONE
 
 *****************************************************************************/
static void ConfigFlowControl( ET131X_ADAPTER *pAdapter )
{
    if( pAdapter->uiDuplexMode == 0 )
    {
        pAdapter->FlowControl = None;
    }
    else
    {
        UINT8 RemotePause, RemoteAsyncPause;
		
        ET1310_PhyAccessMiBit( pAdapter,
							  TRUEPHY_BIT_READ, 5, 10, &RemotePause );
        ET1310_PhyAccessMiBit( pAdapter,
							  TRUEPHY_BIT_READ, 5, 11, &RemoteAsyncPause );
		
        if(( RemotePause == TRUEPHY_BIT_SET ) &&
           ( RemoteAsyncPause == TRUEPHY_BIT_SET ))
        {
            pAdapter->FlowControl = pAdapter->RegistryFlowControl;
        }
        else if(( RemotePause      == TRUEPHY_BIT_SET ) && 
                ( RemoteAsyncPause == TRUEPHY_BIT_CLEAR ))
        {
            if( pAdapter->RegistryFlowControl == Both )
            {
                pAdapter->FlowControl = Both;
            }
            else
            {
                pAdapter->FlowControl = None;
            }
        }
        else if(( RemotePause      == TRUEPHY_BIT_CLEAR ) && 
                ( RemoteAsyncPause == TRUEPHY_BIT_CLEAR ))
        {
            pAdapter->FlowControl = None;
        }
        else /* if (( RemotePause      == TRUEPHY_CLEAR_BIT ) && 
		 ( RemoteAsyncPause == TRUEPHY_SET_BIT )) */
        {
            if( pAdapter->RegistryFlowControl == Both )
            {
                pAdapter->FlowControl = RxOnly;
            }
            else
            {
                pAdapter->FlowControl = None;
            }
        }
    }
}

/******************************************************************************
 ROUTINE:  ConfigMacRegs2
 ******************************************************************************
 
 DESCRIPTION:
 Used to configure the second part of MAC regs to a known initialized 
 state
 
 PARAMETERS :
 pAdpater - pointer to our adapter structure
 
 RETURNS    :
 NONE
 
 *****************************************************************************/
static void ConfigMACRegs2( ET131X_ADAPTER *pAdapter )
{
    INT32         delay = 0;
    PMAC_t        pMac;
    MAC_CFG1_t    cfg1;
    MAC_CFG2_t    cfg2;
    MAC_IF_CTRL_t ifctrl;
    TXMAC_CTL_t   ctl = pAdapter->CSRAddress->txmac.ctl;
    /*-----------------------------------------------------------------------*/
	
    
    /**************************************************************************
	 Let's get our pointer to the MAC regs
     *************************************************************************/
    pMac = &pAdapter->CSRAddress->mac;
	
    cfg1.value   = pMac->cfg1.value;
    cfg2.value   = pMac->cfg2.value;
    ifctrl.value = pMac->if_ctrl.value; 
    
    if( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_1000MBPS )
    {
        cfg2.bits.if_mode    = 0x2;
        ifctrl.bits.phy_mode = 0x0;
    }
    else
    {
        cfg2.bits.if_mode    = 0x1;
        ifctrl.bits.phy_mode = 0x1;
    }
	
	
    /**************************************************************************
	 We need to enable Rx/Tx
     *************************************************************************/
    cfg1.bits.rx_enable	    = 0x1;
    cfg1.bits.tx_enable	    = 0x1;
	
	
    /**************************************************************************
	 Set up flow control
     *************************************************************************/
    cfg1.bits.tx_flow	    = 0x1;
	
    if( ( pAdapter->FlowControl == RxOnly ) ||
	   ( pAdapter->FlowControl == Both ))
    {
        cfg1.bits.rx_flow	    = 0x1;
    }
    else
    {
        cfg1.bits.rx_flow	    = 0x0;
    }
	
	
    /**************************************************************************
	 Initialize loop back to off
     *************************************************************************/
    cfg1.bits.loop_back	    = 0;
	
    pAdapter->CSRAddress->mac.cfg1.value = cfg1.value;
	
	
    /**************************************************************************
	 Now we need to initialize the MAC Configuration 2 register
     *************************************************************************/ 
    cfg2.bits.preamble_len        = 0x7;
    cfg2.bits.huge_frame          = 0x0;
    /* LENGTH FIELD CHECKING bit4: Set this bit to cause the MAC to check the
     * frameís length field to ensure it matches the actual data field length. Clear this bit if no
     * length field checking is desired. Its default is ë0í.
     */
    cfg2.bits.len_check           = 0x1;
	
    if ( pAdapter->RegistryPhyLoopbk == FALSE )
    {
        cfg2.bits.pad_crc         = 0x1;
        cfg2.bits.crc_enable      = 0x1;
    }
    else
    {
        cfg2.bits.pad_crc         = 0;
        cfg2.bits.crc_enable      = 0;
    }
	
	
    /**************************************************************************
	 1 ñ full duplex, 0 ñ half-duplex
     *************************************************************************/
    cfg2.bits.full_duplex	        = pAdapter->uiDuplexMode;
    ifctrl.bits.ghd_mode            = !pAdapter->uiDuplexMode;
	
    pAdapter->CSRAddress->mac.if_ctrl = ifctrl;
    pAdapter->CSRAddress->mac.cfg2.value = cfg2.value;
	
    do
    {
        udelay( 10 );
        delay++;
    } while(( !pAdapter->CSRAddress->mac.cfg1.bits.syncd_rx_en ||
			 !pAdapter->CSRAddress->mac.cfg1.bits.syncd_tx_en ) && 
			( delay < 100 ));
	
    if( delay == 100 )
    {
        DBG_ERROR(
				  "Syncd bits did not respond correctly cfg1 word 0x%08x\n",
				  pAdapter->CSRAddress->mac.cfg1.value );
    }
	
    DBG_TRACE( "Speed %d, Dup %d, CFG1 0x%08x, CFG2 0x%08x, if_ctrl 0x%08x\n",
			  pAdapter->uiLinkSpeed, pAdapter->uiDuplexMode,
			  pAdapter->CSRAddress->mac.cfg1.value,
			  pAdapter->CSRAddress->mac.cfg2.value,
			  pAdapter->CSRAddress->mac.if_ctrl.value );
	
	
    /**************************************************************************
	 Enable TXMAC
     *************************************************************************/
    ctl.bits.txmac_en   = 0x1;
    ctl.bits.fc_disable = 0x1;
    pAdapter->CSRAddress->txmac.ctl = ctl;
	
	
}


static void ET1310_PhyPowerDown( ET131X_ADAPTER *pAdapter, BOOL_t down )
{
    UINT16  usData;
	
    MiRead( pAdapter, pAdapter->Stats.xcvr_addr, PHY_CONTROL, &usData );
	
    if( down == FALSE ) 
    {
        // Power UP
        usData &= ~0x0800;
        MiWrite( pAdapter,  pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
    else
    {
        // Power DOWN
        usData |= 0x0800;
        MiWrite( pAdapter,  pAdapter->Stats.xcvr_addr, PHY_CONTROL, usData );
    }
}


/******************************************************************************
 ROUTINE:  TPAL_SetPhy10HalfDuplex
 ******************************************************************************
 
 DESCRIPTION:
 Used to force the phy into 10 Base T Half Duplex mode.  Also sets the 
 MAC so it is syncd up properly
 
 PARAMETERS :
 pAdapter - pointer to the adapter structure
 
 RETURNS    :
 TRUEPHY_SUCCESS - if everything goes according to plan
 TRUEPHY_FALIURE  -if something goes wrong
 
 *****************************************************************************/
static INT32 TPAL_SetPhy10HalfDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/
	
	
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power down the PHY\n" );
        return returnValue;
    }
	
    /**************************************************************************
	 First we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 1000 BaseT\n" );
        return returnValue;
    }
	
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 100 BaseT "
				  "for forcing 10 BaseT Half Duplex\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 Set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_HALF );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not set Advertise of 10 Half\n" );
        return returnValue;
    }
	
    /**************************************************************************
	 Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power up the PHY\n" );
        return returnValue;
    }
	
	
    return returnValue ;
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE:  TPAL_SetPhy10FullDuplex
 ******************************************************************************
 
 DESCRIPTION:
 Used to force the phy into 10 Base T Full Duplex mode.  Also sets the 
 MAC so it is syncd up properly
 
 PARAMETERS :
 pAdapter - pointer to the adapter structure
 
 RETURNS    :
 TRUEPHY_SUCCESS - if everything goes according to plan
 TRUEPHY_FALIURE  -if somethign goes wrong during the procedures
 
 *****************************************************************************/
static INT32 TPAL_SetPhy10FullDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/
	
	
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power down the PHY\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 First we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 1000 BaseT "
				  "for Forcing 10 BaseT Full Duplex\n" );
        return returnValue;
    }
	
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 100 BaseT "
				  "for Forcing 10 BaseT Full Duplex\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 Set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not set Advertise of 10 Full\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power up the phy\n" );
        return returnValue;
    }

    return returnValue;
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE:  TPAL_SetPhy10Force
 ******************************************************************************
 
 DESCRIPTION:
 Used to force the phy into 10 Base T Full Duplex mode WITHOUT using
 autonegotiation. 
 
 PARAMETERS :
 pAdapter - pointer to the adapter structure
 
 RETURNS    :
 
 *****************************************************************************/
static void TPAL_SetPhy10Force( ET131X_ADAPTER *pAdapter )
{
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
	
    /**************************************************************************
	 Disable autoneg
     *************************************************************************/
    ET1310_PhyAutoNeg( pAdapter, FALSE );
	
	
    /**************************************************************************
	 Disable all advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
	
    /**************************************************************************
	 Force 10 Mbps
     *************************************************************************/
    ET1310_PhySpeedSelect( pAdapter, TRUEPHY_SPEED_10MBPS );
	
	
    /**************************************************************************
	 Force Full duplex
     *************************************************************************/
    ET1310_PhyDuplexMode( pAdapter, TRUEPHY_DUPLEX_FULL );
	
	
    /**************************************************************************
	 Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );
	
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE:  TPAL_SetPhy100HalfDuplex
 ******************************************************************************
 
 DESCRIPTION:
 Used to force the phy into 100 Base T Half Duplex mode.  Also sets the 
 MAC so it is syncd up properly
 
 PARAMETERS :
 pAdapter - pointer to the adapter structure
 
 RETURNS    :
 TRUEPHY_SUCCESS - if everything goes according to plan
 TRUEPHY_FALIURE  -if somethign goes wrong during the procedures
 
 *****************************************************************************/
static INT32 TPAL_SetPhy100HalfDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
	
	
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        
        DBG_ERROR( "Could not power down the PHY\n" );
        return( returnValue );
    }
	
	
    /**************************************************************************
	 first we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 1000 BaseT "
				  "for Forcing 100 BaseT Half Duplex\n" );
        return returnValue;
    }
	
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 10 BaseT "
				  "for Forcing 100 BaseT Half Duplex\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 Set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_HALF );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not set Advertise of 100 Half\n" );
        return returnValue;
    }
	
	
    /* Set speed */
    ET1310_PhySpeedSelect( pAdapter, TRUEPHY_SPEED_100MBPS );
	
	
    /**************************************************************************
	 Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power up the PHY\n" );
        return returnValue;
    }
	
	
    return returnValue;
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE:  TPAL_SetPhy100FullDuplex
 ******************************************************************************
 
 DESCRIPTION:
 Used to force the phy into 100 Base T Full Duplex mode.  Also sets the 
 MAC so it is syncd up properly
 
 PARAMETERS :
 pAdapter - pointer to the adapter structure
 
 RETURNS    :
 TRUEPHY_SUCCESS - if everything goes according to plan
 TRUEPHY_FALIURE  -if somethign goes wrong during the procedures
 
 *****************************************************************************/
static INT32 TPAL_SetPhy100FullDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
	
	
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power down PHY\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 First we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 1000 BaseT "
				  "for Forcing 100 BaseT Full Duplex\n" );
        return returnValue;
    }
	
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 10 BaseT "
				  "for Forcing 100 BaseT Full Duplex\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 Set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not set Advertise of 100 Full\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power up PHY\n" );
        return returnValue;
    }
	
	return returnValue;
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE:  TPAL_SetPhy100Force
 ******************************************************************************
 
 DESCRIPTION:
 Used to force the phy into 100 Base T Full Duplex mode WITHOUT using
 autonegotiation. 
 
 PARAMETERS :
 pAdapter - pointer to the adapter structure
 
 RETURNS    :
 
 *****************************************************************************/
static void TPAL_SetPhy100Force( ET131X_ADAPTER *pAdapter )
{
	
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
	
    /**************************************************************************
	 Disable autoneg
     *************************************************************************/
    ET1310_PhyAutoNeg( pAdapter, FALSE );
	
	
    /**************************************************************************
	 Disable all advertisement
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
	
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
	
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
	
    /**************************************************************************
	 Force 100 Mbps
     *************************************************************************/
    ET1310_PhySpeedSelect( pAdapter, TRUEPHY_SPEED_100MBPS );
	
	
    /**************************************************************************
	 Force Full duplex
     *************************************************************************/
    ET1310_PhyDuplexMode( pAdapter, TRUEPHY_DUPLEX_FULL );
	
	
    /**************************************************************************
	 Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE:  TPAL_SetPhy1000FullDuplex
 ******************************************************************************
 
 DESCRIPTION:
 Used to force the phy into 1000 Base T Full Duplex mode.  Also sets the 
 MAC so it is syncd up properly
 
 PARAMETERS :
 pAdapter - pointer to the adapter structure
 
 RETURNS    :
 TRUEPHY_SUCCESS - if everything goes according to plan
 TRUEPHY_FALIURE  -if somethign goes wrong during the procedures
 
 *****************************************************************************/
static INT32 TPAL_SetPhy1000FullDuplex( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
	
	
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power down phy\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 first we need to turn off all other advertisement
     *************************************************************************/
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 100 BaseT "
				  "for Forcing 1000 BaseT Full Duplex\n" );
        return returnValue;
    }
	
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not turn off advertisement of 10 BaseT "
				  "for Forcing 1000 BaseT Full Duplex\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 set our advertise values accordingly
     *************************************************************************/
    ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not set Advertise of 1000 Full\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power up PHY\n" );
        return returnValue;
    }
	
    return returnValue;
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE:  TPAL_SetPhyAutoNeg
 ******************************************************************************
 
 DESCRIPTION:
 Used to set phy to autonegotiation mode.  
 
 PARAMETERS :
 pAdapter - pointer to the adapter structure
 
 RETURNS    :
 TRUEPHY_SUCCESS - if everything goes according to plan
 TRUEPHY_FALIURE  -if somethign goes wrong during the procedures
 
 *****************************************************************************/
static INT32 TPAL_SetPhyAutoNeg( ET131X_ADAPTER *pAdapter )
{
    INT32   returnValue = TRUEPHY_SUCCESS;
    /*-----------------------------------------------------------------------*/
	
	
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 1 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power down phy\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 Turn on advertisement of all capabilities
     *************************************************************************/
    ET1310_PhyAdvertise10BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_BOTH );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( 
				  "Could not Turn on Advertisement of 10 BaseT "
				  "from Setting to Auto Negotiation\n" );
        return returnValue;
    }
	
    ET1310_PhyAdvertise100BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_BOTH );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not Turn on Advertisement of 100 BaseT "
				  "from Setting to Auto Negotiation\n" );
        return returnValue;
    }
	
    if( pAdapter->DeviceID != ET131X_PCI_DEVICE_ID_FAST )
    {
        ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_FULL );
    }
    else
    {
        ET1310_PhyAdvertise1000BaseT( pAdapter, TRUEPHY_ADV_DUPLEX_NONE );
    }
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR(
				  "Could not Turn on Advertisement of 1000 BaseT "
				  "from Setting to Auto Negotiation\n" );
        return returnValue;
    }
	
	
    /**************************************************************************
	 Make sure auto-neg is ON (it is disabled in FORCE modes)
     *************************************************************************/
    ET1310_PhyAutoNeg( pAdapter, TRUE );
	
	
    /**************************************************************************
	 Power up PHY
     *************************************************************************/
    ET1310_PhyPowerDown( pAdapter, 0 );
	
    if( returnValue == TRUEPHY_FAILURE )
    {
        /**********************************************************************
		 HANDLE ERROR HERE
         *********************************************************************/
        DBG_ERROR( "Could not power up phy\n" );
        return returnValue;
    }
	
    return returnValue;
}
/*===========================================================================*/

/******************************************************************************
 ROUTINE :  et131x_xcvr_init
 ******************************************************************************
 
 DESCRIPTION       : Used to init the phy if we are setting it into force mode
 
 *****************************************************************************/
static int et131x_xcvr_init( ET131X_ADAPTER *adapter )
{
    int         status = 0;
    MI_IMR_t    imr;
    MI_ISR_t    isr;
    MI_LCR2_t   lcr2;
    /*-----------------------------------------------------------------------*/
	
    /**************************************************************************
	 Zero out the adapter structure variable representing BMSR
     *************************************************************************/
    adapter->Bmsr.value = 0;
	
    MiRead( adapter, (UINT8)adapter->Stats.xcvr_addr, 
		   (UINT8)FIELD_OFFSET( MI_REGS_t, isr ), &isr.value );
	
    MiRead( adapter, (UINT8)adapter->Stats.xcvr_addr, 
		   (UINT8)FIELD_OFFSET( MI_REGS_t, imr ), &imr.value );
	
	
    /**************************************************************************
	 Set the link status interrupt only.  Bad behavior when link status and 
	 auto neg are set, we run into a nested interrupt problem
     *************************************************************************/
    imr.bits.int_en         = 0x1;
    imr.bits.link_status    = 0x1;
    imr.bits.autoneg_status = 0x1;
	
    MiWrite( adapter, (UINT8)adapter->Stats.xcvr_addr, 
			(UINT8)FIELD_OFFSET( MI_REGS_t, imr ), imr.value );
	
	
    /**************************************************************************
	 Set the LED behavior such that LED 1 indicates speed (off = 10Mbits,
	 blink = 100Mbits, on = 1000Mbits) and LED 2 indicates link and
	 activity (on for link, blink off for activity).
	 
	 NOTE: Some customizations have been added here for specific vendors;
	 The LED behavior is now determined by vendor data in the EEPROM. However,
	 the above description is the default.
     *************************************************************************/
    if(( adapter->eepromData[1] & 0x4 ) == 0 )
    {
        MiRead( adapter, (UINT8)adapter->Stats.xcvr_addr, 
			   (UINT8)FIELD_OFFSET( MI_REGS_t, lcr2 ), &lcr2.value );
		
        if(( adapter->eepromData[1] & 0x8 ) == 0 )
        {
            lcr2.bits.led_tx_rx = 0x3;
        }
        else
        {
            lcr2.bits.led_tx_rx = 0x4;
        }
		
        lcr2.bits.led_link = 0xa;
		
        MiWrite( adapter, (UINT8)adapter->Stats.xcvr_addr, 
                (UINT8)FIELD_OFFSET( MI_REGS_t, lcr2 ), lcr2.value );
    }
	
	
    /**************************************************************************
	 Determine if we need to go into a force mode and set it
     *************************************************************************/
    if( adapter->AiForceSpeed == 0 && adapter->AiForceDpx == 0 )
    {
        if(( adapter->RegistryFlowControl == TxOnly ) ||
		   ( adapter->RegistryFlowControl == Both ))
        {
            ET1310_PhyAccessMiBit( adapter,
								  TRUEPHY_BIT_SET, 4, 11, NULL );
        }
        else
        {
            ET1310_PhyAccessMiBit( adapter,
								  TRUEPHY_BIT_CLEAR, 4, 11, NULL );
        }
		
        if( adapter->RegistryFlowControl == Both )
        {
            ET1310_PhyAccessMiBit( adapter,
								  TRUEPHY_BIT_SET, 4, 10, NULL );
        }
        else
        {
            ET1310_PhyAccessMiBit( adapter,
								  TRUEPHY_BIT_CLEAR, 4, 10, NULL );
        }
		
		
        /**********************************************************************
		 Set the phy to autonegotiation
         *********************************************************************/
        ET1310_PhyAutoNeg( adapter, TRUE );
		
		
        /* NOTE - Do we need this? */
        ET1310_PhyAccessMiBit( adapter,
							  TRUEPHY_BIT_SET, 0, 9, NULL );
		
        return status;
    }
    else
    {
        ET1310_PhyAutoNeg( adapter, FALSE );
		
        /**********************************************************************
		 Set to the correct force mode.
         *********************************************************************/
        if( adapter->AiForceDpx != 1 )
        {
            if(( adapter->RegistryFlowControl == TxOnly ) ||
               ( adapter->RegistryFlowControl == Both ))
            {
                ET1310_PhyAccessMiBit( adapter,
									  TRUEPHY_BIT_SET, 4, 11, NULL );
            }
            else
            {
                ET1310_PhyAccessMiBit( adapter,
									  TRUEPHY_BIT_CLEAR, 4, 11, NULL );
            }
			
            if( adapter->RegistryFlowControl == Both )
            {
                ET1310_PhyAccessMiBit( adapter,
									  TRUEPHY_BIT_SET, 4, 10, NULL );
            }
            else
            {
                ET1310_PhyAccessMiBit( adapter,
									  TRUEPHY_BIT_CLEAR, 4, 10, NULL );
            }
        }
        else
        {
            ET1310_PhyAccessMiBit( adapter,
								  TRUEPHY_BIT_CLEAR, 4, 10, NULL );
            ET1310_PhyAccessMiBit( adapter,
								  TRUEPHY_BIT_CLEAR, 4, 11, NULL );
        }
		
        switch( adapter->AiForceSpeed )
        {
			case 10:
				if( adapter->AiForceDpx == 1 )
				{
					TPAL_SetPhy10HalfDuplex( adapter );
				}
				else if( adapter->AiForceDpx == 2 )
				{
					TPAL_SetPhy10FullDuplex( adapter );
				}
				else
				{
					TPAL_SetPhy10Force( adapter );
				}
				break;
				
			case 100:
				if( adapter->AiForceDpx == 1 )
				{
					TPAL_SetPhy100HalfDuplex( adapter );
				}
				else if( adapter->AiForceDpx == 2 )
				{
					TPAL_SetPhy100FullDuplex( adapter );
				}
				else
				{
					TPAL_SetPhy100Force( adapter );
				}
				break;
				
			case 1000:
				TPAL_SetPhy1000FullDuplex( adapter );
				break;
        }
		
		
        return status;
    }
}


/******************************************************************************
 *******************************************************************************
 
 The routines which follow provide low-level access to the PHY, and are used
 primarily by the routines above (although there are a few places elsewhere in
 the driver where this level of access is required).
 
 *******************************************************************************
 ******************************************************************************/

static UINT16  ConfigPhy[25][2] =
{
/* Reg      Value      Register */
/* Addr                         */
{0x880B,    0x0926}, /* AfeIfCreg4B1000Msbs */
{0x880C,    0x0926}, /* AfeIfCreg4B100Msbs */
{0x880D,    0x0926}, /* AfeIfCreg4B10Msbs */

{0x880E,    0xB4D3}, /* AfeIfCreg4B1000Lsbs */
{0x880F,    0xB4D3}, /* AfeIfCreg4B100Lsbs */
{0x8810,    0xB4D3}, /* AfeIfCreg4B10Lsbs */

{0x8805,    0xB03E}, /* AfeIfCreg3B1000Msbs */
{0x8806,    0xB03E}, /* AfeIfCreg3B100Msbs */
{0x8807,    0xFF00}, /* AfeIfCreg3B10Msbs */

{0x8808,    0xE090}, /* AfeIfCreg3B1000Lsbs */
{0x8809,    0xE110}, /* AfeIfCreg3B100Lsbs */
{0x880A,    0x0000}, /* AfeIfCreg3B10Lsbs */

{0x300D,    1     }, /* DisableNorm */

{0x280C,    0x0180}, /* LinkHoldEnd */

{0x1C21,    0x0002}, /* AlphaM */

{0x3821,    6     }, /* FfeLkgTx0 */
{0x381D,    1     }, /* FfeLkg1g4 */
{0x381E,    1     }, /* FfeLkg1g5 */
{0x381F,    1     }, /* FfeLkg1g6 */
{0x3820,    1     }, /* FfeLkg1g7 */

{0x8402,    0x01F0}, /* Btinact */
{0x800E,    20    }, /* LftrainTime */
{0x800F,    24    }, /* DvguardTime */
{0x8010,    46    }, /* IdlguardTime */

{0,         0     } 

};
//
// condensed version of the phy initialization routine
//
void AgereET131x::PhyInit()
{
    UINT16  usData, usIndex;
	
	
    // get the identity (again ?)
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_ID_1, &usData );
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_ID_2, &usData );
    
    // what does this do/achieve ?
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, &usData );   // should read 0002
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0006 );
    
    // read modem register 0402, should I do something with the return data ?
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_INDEX_REG, 0x0402 );
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_DATA_REG, &usData );
	
    // what does this do/achieve ?    
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0002 );
	
    // get the identity (again ?)
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_ID_1, &usData );
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_ID_2, &usData );
	
    // what does this achieve ?    
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, &usData );   // should read 0002
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0006 );
    
    // read modem register 0402, should I do something with the return data ?
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_INDEX_REG, 0x0402 );
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_DATA_REG, &usData );
	
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0002 );
	
    // what does this achieve (should return 0x1040)
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_CONTROL, &usData );
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, &usData );   // should read 0002
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_CONTROL, 0x1840 );
    
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0007 );
    
    // here the writing of the array starts....
    usIndex = 0;
    while( ConfigPhy[usIndex][0] != 0x0000 )
    {
        // write value
        MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_INDEX_REG, ConfigPhy[usIndex][0] );
        MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_DATA_REG, ConfigPhy[usIndex][1] );
		
        // read it back
        MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_INDEX_REG, ConfigPhy[usIndex][0] );
        MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_DATA_REG, &usData );
		
        // do a check on the value read back ?
        usIndex++;
    }
    // here the writing of the array ends...
    
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_CONTROL, &usData );   // 0x1840
    MiRead( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, &usData );   // should read 0007
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_CONTROL, 0x1040 );
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_MPHY_CONTROL_REG, 0x0002 );

}


void AgereET131x::PhyReset()
{
    MiWrite( &adapter, adapter.Stats.xcvr_addr, PHY_CONTROL, 0x8000 );
}  


void AgereET131x::PhyPowerDown()
{
	ET1310_PhyPowerDown( &adapter, 1 );
}

void AgereET131x::PhyPowerUp()
{
	ET1310_PhyPowerDown( &adapter, 0 );
}


/******************************************************************************
 ROUTINE :  et131x_setphy_normal
 ******************************************************************************
 
 DESCRIPTION  : Used by Power Management to force the PHY into 10 Base T
 half-duplex mode, when going to D3 in WOL mode. Also
 used during initialization to set the PHY for normal
 operation.
 
 *****************************************************************************/
int AgereET131x::setphy_normal()
{
    int status;
	
    /**************************************************************************
	 We need to turn off 1000 base half dulplex, the mac does not support it
	 For the 10/100 part, turn off all gig advertisement
     *************************************************************************/
    if( adapter.DeviceID != ET131X_PCI_DEVICE_ID_FAST )
    {
        ET1310_PhyAdvertise1000BaseT( &adapter, TRUEPHY_ADV_DUPLEX_FULL );
    }
    else
    {
        ET1310_PhyAdvertise1000BaseT( &adapter, TRUEPHY_ADV_DUPLEX_NONE );
    }
	
	
    /**************************************************************************
	 Make sure the PHY is powered up
     *************************************************************************/
    PhyPowerUp();
    status = et131x_xcvr_init( &adapter );
	
    return status;
}

/******************************************************************************
 ROUTINE:  et131x_Mii_check
 ******************************************************************************
 
 *****************************************************************************/
void AgereET131x::Mii_check( MI_BMSR_t bmsr, MI_BMSR_t bmsr_ints )
{
    UCHAR    ucLinkStatus;
    UINT32    nAutoNegStatus;
    UINT32    nSpeed;
    UINT32    nDuplex;
    UINT32    nMdiMdix;
    UINT32    nMasterSlave;
    UINT32    nPolarity;
    /*-----------------------------------------------------------------------*/
	
	
    if( bmsr_ints.bits.link_status ) 
	{
		if( bmsr.bits.link_status ) {
			adapter.PoMgmt.TransPhyComaModeOnBoot = 20;
			
            /******************************************************************
			 Update our state variables and indicate the connected state
             *****************************************************************/
            IOSimpleLockLock( adapter.Lock );
            
            adapter.MediaState = NETIF_STATUS_MEDIA_CONNECT;
            MP_CLEAR_FLAG( &adapter, fMP_ADAPTER_LINK_DETECTION );
            
            IOSimpleLockUnlock( adapter.Lock );
			
            if( adapter.RegistryPhyLoopbk == FALSE )
            {
				setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium());
            }
		} else {
            DBG_WARNING( "Link down cable problem\n" );
			
            if( adapter.uiLinkSpeed == TRUEPHY_SPEED_10MBPS ){
                // NOTE - Is there a way to query this without TruePHY?
                //if( TRU_QueryCoreType ( adapter.hTruePhy, 0 ) == EMI_TRUEPHY_A13O )
                {
                    UINT16 Register18;
					
                    MiRead( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x12, &Register18 );
                    MiWrite( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x12, Register18 | 0x4 );
                    MiWrite( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x10, Register18 | 0x8402 );
                    MiWrite( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x11, Register18 | 511 );
                    MiWrite( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x12, Register18 );
                }
            }
			
            /******************************************************************
			 For the first N seconds of life, we are in "link detection"
			 When we are in this state, we should only report "connected". 
			 When the LinkDetection Timer expires, we can report
			 disconnected (handled in the LinkDetectionDPC).
             *****************************************************************/
            if(( MP_IS_FLAG_CLEAR( &adapter, fMP_ADAPTER_LINK_DETECTION )) ||
               ( adapter.MediaState == NETIF_STATUS_MEDIA_DISCONNECT ))
            {
                spin_lock_irqsave( &adapter.Lock, lockflags );
                adapter.MediaState = NETIF_STATUS_MEDIA_DISCONNECT;
                spin_unlock_irqrestore( &adapter.Lock, lockflags );
				
                if( adapter.RegistryPhyLoopbk == FALSE )
                {
					setLinkStatus(kIONetworkLinkValid, 0);
                }
            }
			
            adapter.uiLinkSpeed  = 0;
            adapter.uiDuplexMode = 0;
			
			
			/******************************************************************
			 Free the packets being actively sent & stopped
			 *****************************************************************/
			free_busy_send_packets();
			
            /******************************************************************
			 Re-initialize the send structures
			 *****************************************************************/
			init_send();
			
			/******************************************************************
			 Reset the RFD list and re-start RU 
			 *****************************************************************/
			reset_recv();
			
			/******************************************************************
			 Bring the device back to the state it was during init prior
			 to autonegotiation being complete.  This way, when we get the
			 auto-neg complete interrupt, we can complete init by calling
			 ConfigMacREGS2.
			 *****************************************************************/
			soft_reset();
			
            /******************************************************************
			 Setup ET1310 as per the documentation
             *****************************************************************/
            adapter_setup();
			
			
            /******************************************************************
			 Setup the PHY into coma mode until the cable is plugged back in
             *****************************************************************/
            if( adapter.RegistryPhyComa == 1 ){
                EnablePhyComa();
            }
		}
	}
	
    if( bmsr_ints.bits.auto_neg_complete ||
	   (( adapter.AiForceDpx == 3 ) && ( bmsr_ints.bits.link_status )))
    {
        if( bmsr.bits.auto_neg_complete  ||
           ( adapter.AiForceDpx == 3 ))
        {
            ET1310_PhyLinkStatus( &adapter,
								 &ucLinkStatus,
								 &nAutoNegStatus,
								 &nSpeed,
								 &nDuplex,
								 &nMdiMdix,
								 &nMasterSlave,
								 &nPolarity );

			
            adapter.uiLinkSpeed  = nSpeed;
            adapter.uiDuplexMode = nDuplex;
			adapter.PoMgmt.TransPhyComaModeOnBoot = 20;
			
            if( adapter.uiLinkSpeed == TRUEPHY_SPEED_10MBPS )
            {
                // NOTE - Is there a way to query this without TruePHY?
                //if( TRU_QueryCoreType ( adapter.hTruePhy, 0 ) == EMI_TRUEPHY_A13O )
                {
                    UINT16 Register18;
					
                    MiRead( &adapter, (UINT8)adapter.Stats.xcvr_addr,  0x12, &Register18 );
                    MiWrite( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x12, Register18 | 0x4 );
                    MiWrite( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x10, Register18 | 0x8402 );
                    MiWrite( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x11, Register18 | 511 );
                    MiWrite( &adapter, (UINT8)adapter.Stats.xcvr_addr, 0x12, Register18 );
                }
            }
			
            ConfigFlowControl( &adapter );
			
			
            if(( adapter.uiLinkSpeed == TRUEPHY_SPEED_1000MBPS ) &&
			   ( adapter.RegistryJumboPacket > 2048 ))
				
            {
                ET1310_PhyAndOrReg( &adapter, 0x16, 0xcfff, 0x2000 );
            }
			
			if(( adapter.uiLinkSpeed == TRUEPHY_SPEED_100MBPS ) ||
			   ( adapter.uiLinkSpeed == TRUEPHY_SPEED_10MBPS ))
			{
				adapter.CSRAddress->rxdma.max_pkt_time.value = 0;
				adapter.CSRAddress->rxdma.num_pkt_done.value = 1;
			}
			
            ConfigMACRegs2( &adapter );
			/**************************************************************************
			 Ready to start the RXDMA/TXDMA engine
			 *************************************************************************/
			if( !MP_TEST_FLAG( &adapter, fMP_ADAPTER_LOWER_POWER )){
				rx_dma_enable();
				tx_dma_enable();
			}
#if	1
			// Notify
			UInt32 speed = 1000 * MBit;
			if( adapter.uiLinkSpeed == TRUEPHY_SPEED_10MBPS ) {
				speed = 10 * MBit;
			} else if(adapter.uiLinkSpeed == TRUEPHY_SPEED_100MBPS) {
				speed = 100 * MBit;
			}
			setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, getCurrentMedium(), speed);
#endif
        }
    }
}

/******************************************************************************
 ROUTINE :  et131x_xcvr_find
 ******************************************************************************
 
 DESCRIPTION       : Used to find the PHY ID
 
 PARAMETERS        : adapter - pointer to our private adapter structure
 
 RETURNS           : 0 on success
 errno on failure (as defined in errno.h)
 
 
 *****************************************************************************/
int AgereET131x::xcvr_find()
{
    int           status = -ENODEV;
    UINT8         xcvr_addr;
	MI_IDR1_t     idr1;
	MI_IDR2_t     idr2;
	UINT32        xcvr_id;

	
	
    /**************************************************************************
	 We need to get xcvr id and address we just get the first one
     *************************************************************************/
    for( xcvr_addr = 0; xcvr_addr < 32; xcvr_addr++ ) 
    {
        /**********************************************************************
		 Read the ID from the PHY
         *********************************************************************/
        MiRead( &adapter, xcvr_addr, (UINT8)FIELD_OFFSET(MI_REGS_t, idr1), &idr1.value );
        MiRead( &adapter, xcvr_addr, (UINT8)FIELD_OFFSET(MI_REGS_t, idr2), &idr2.value );
		
        xcvr_id = (UINT32)(( idr1.value << 16 ) | idr2.value );
		
        if(( idr1.value != 0) && ( idr1.value != 0xffff )) 
        {
            adapter.Stats.xcvr_id   = xcvr_id;
            adapter.Stats.xcvr_addr = (UINT32)xcvr_addr;
			
            status = 0;
            break;
        }
	} 
    return status;
}


void AgereET131x::EnablePhyComa()
{
    PM_CSR_t      GlobalPmCSR = adapter.CSRAddress->global.pm_csr;
    /*-----------------------------------------------------------------------*/
	
	/**************************************************************************
	 Save the GbE PHY speed and duplex modes 
	 Need to restore this when cable is plugged back in
	 *************************************************************************/

	adapter.PoMgmt.PowerDownSpeed  = adapter.AiForceSpeed;
    adapter.PoMgmt.PowerDownDuplex = adapter.AiForceDpx;

    /**************************************************************************
	 Stop sending packets. 
     *************************************************************************/
	IOSimpleLockLock( adapter.SendHWLock );
	
    MP_SET_FLAG( &adapter, fMP_ADAPTER_LOWER_POWER );
	
	IOSimpleLockUnlock( adapter.SendHWLock );
#if	0
	/**************************************************************************
	 Wait for outstanding Receive packets
     *************************************************************************/
    INT32         LoopCounter = 10;
    while(( MP_GET_RCV_REF( &adapter ) != 0 ) && ( LoopCounter-- > 0 ))
    {
        /**********************************************************************
		 Sleep for 2 Ms;
         *********************************************************************/
        mdelay( 2 );
    }
#endif
	
	/**************************************************************************
	 Gate off JAGCore 3 clock domains
     *************************************************************************/
	GlobalPmCSR.bits.pm_sysclk_gate     = 0;
    GlobalPmCSR.bits.pm_txclk_gate      = 0;
    GlobalPmCSR.bits.pm_rxclk_gate      = 0;
    adapter.CSRAddress->global.pm_csr = GlobalPmCSR;
	
	
	/**************************************************************************
	 Program gigE PHY in to Coma mode
     *************************************************************************/
	GlobalPmCSR.bits.pm_phy_sw_coma     = 1;
	
    adapter.CSRAddress->global.pm_csr = GlobalPmCSR;
}

/******************************************************************************
 ROUTINE:  DisablePhyComa
 ******************************************************************************
 DESCRIPTION:
 This routine is used to disable the Phy Coma Mode
 
 *****************************************************************************/
void AgereET131x::DisablePhyComa(  )
{
    PM_CSR_t     GlobalPmCSR = adapter.CSRAddress->global.pm_csr;
    /*-----------------------------------------------------------------------*/
	
	
    /*************************************************************************
	 Disable phy_sw_coma register and re-enable JAGCore clocks
     ************************************************************************/
    GlobalPmCSR.bits.pm_sysclk_gate     = 1;
    GlobalPmCSR.bits.pm_txclk_gate      = 1;
    GlobalPmCSR.bits.pm_rxclk_gate      = 1;
    GlobalPmCSR.bits.pm_phy_sw_coma     = 0;
	
    adapter.CSRAddress->global.pm_csr = GlobalPmCSR;
	
	
    /**************************************************************************
	 Restore the GbE PHY speed and duplex modes; 
	 Reset JAGCore; re-configure and initialize JAGCore and gigE PHY
     *************************************************************************/

    adapter.AiForceSpeed = adapter.PoMgmt.PowerDownSpeed;
    adapter.AiForceDpx   = adapter.PoMgmt.PowerDownDuplex;	
	
    /**************************************************************************
	 Re-initialize the send structures
     *************************************************************************/
    init_send();
	
	
    /**************************************************************************
	 Reset the RFD list and re-start RU 
     *************************************************************************/
    reset_recv();
	
	
    /**************************************************************************
	 Bring the device back to the state it was during init prior to 
	 autonegotiation being complete.  This way, when we get the auto-neg 
	 complete interrupt, we can complete init by calling ConfigMacREGS2.
     *************************************************************************/
    soft_reset();
	
	
    /**************************************************************************
	 setup et1310 as per the documentation ??
     *************************************************************************/
    adapter_setup();
	
	
    /**************************************************************************
	 Allow Tx to restart 
     *************************************************************************/
    MP_CLEAR_FLAG( &adapter, fMP_ADAPTER_LOWER_POWER );
	
	
    /**************************************************************************
	 Need to re-enable Rx. 
     *************************************************************************/
    rx_dma_enable();
}


void AgereET131x::HandlePhyInterrupt()
{
	MI_BMSR_t BmsrInts, BmsrData;
	MI_ISR_t  myIsr;
	
	/******************************************************************
	 If we are in coma mode when we get this interrupt, we need to 
	 disable it.
	 *****************************************************************/
	if( adapter.CSRAddress->global.pm_csr.bits.phy_sw_coma == 1 )
	{
		/**************************************************************
		 Check to see if we are in coma mode and if so, disable it 
		 because we will not be able to read PHY values until we are 
		 out.
		 *************************************************************/
		DBG_VERBOSE( "Device is in COMA mode, need to wake up\n" );
		DisablePhyComa();
	}
	
	
	/******************************************************************
	 Read the PHY ISR to clear the reason for the interrupt.
	 *****************************************************************/
	MiRead( &adapter, (UINT8)adapter.Stats.xcvr_addr, 
		   (UINT8)FIELD_OFFSET( MI_REGS_t, isr ), &myIsr.value );
	
	if( !adapter.ReplicaPhyLoopbk )
	{
		MiRead( &adapter, (UINT8)adapter.Stats.xcvr_addr, 
			   (UINT8)FIELD_OFFSET( MI_REGS_t, bmsr ), &BmsrData.value );
		
		BmsrInts.value       = adapter.Bmsr.value ^ BmsrData.value;
		adapter.Bmsr.value = BmsrData.value;
		
		DBG_VERBOSE( "Bmsr.value = 0x%04x, Bmsr_ints.value = 0x%04x\n", 
					BmsrData.value,  
					BmsrInts.value );
		
		
		/**************************************************************
		 Do all the cable in / cable out stuff
		 *************************************************************/
		Mii_check( BmsrData, BmsrInts );
	}
	
}

