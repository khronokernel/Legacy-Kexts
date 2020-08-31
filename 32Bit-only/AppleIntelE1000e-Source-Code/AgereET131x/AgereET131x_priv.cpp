#include "AgereET131x.h"



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



static void LbcifWriteByte( ET131X_ADAPTER *pAdapter, UINT32 unOffset, UCHAR bByte )
{
	pAdapter->pdev->configWrite8( unOffset, bByte);
}


static void LbcifReadDword( ET131X_ADAPTER *pAdapter, UINT32 unOffset, PUINT32 punData )
{
	*punData = pAdapter->pdev->configRead32( unOffset );
}


static void LbcifWriteDword( ET131X_ADAPTER *pAdapter,UINT32 unOffset, UINT32 unData )
{
	pAdapter->pdev->configWrite32( unOffset, unData );
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
 
 *****************************************************************************/
static void EepromWriteByte( ET131X_ADAPTER *pAdapter, UINT32 unAddress, UCHAR bData,
					  UINT32 unEepromId )
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
        LbcifReadDword( pAdapter, LBCIF_DWORD1_GROUP_OFFSET, &unDword1 );
		
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
        return;
    }
	
	
    /**************************************************************************
	 Step 2:
     *************************************************************************/
    bControl  = 0;
    bControl |= LBCIF_CONTROL_LBCIF_ENABLE | LBCIF_CONTROL_I2C_WRITE;
	
	
    LbcifWriteByte( pAdapter, LBCIF_CONTROL_REGISTER_OFFSET, bControl );
    
    nI2CWriteActive = 1;
	
	
    /**************************************************************************
	 Prepare EEPROM address for Step 3
     *************************************************************************/
    unAddress |= ( unEepromId << 8 );
	
    for( nRetries = 0; nRetries < MAX_NUM_WRITE_RETRIES; nRetries++ )
    {
        /**********************************************************************
		 Step 3:
         *********************************************************************/
        LbcifWriteDword( pAdapter, LBCIF_ADDRESS_REGISTER_OFFSET, unAddress );
        
        /**********************************************************************
		 Step 4:
         *********************************************************************/
        LbcifWriteByte( pAdapter, LBCIF_DATA_REGISTER_OFFSET, bData );
		
        /**********************************************************************
		 Step 5:
         *********************************************************************/
        for( nIndex = 0; nIndex < MAX_NUM_REGISTER_POLLS; nIndex++ )
        {
            /******************************************************************
			 Read registers grouped in DWORD1
             *****************************************************************/
            LbcifReadDword( pAdapter, LBCIF_DWORD1_GROUP_OFFSET, &unDword1 );
			
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
		
        LbcifWriteByte( pAdapter, LBCIF_CONTROL_REGISTER_OFFSET, bControl );
		
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
	
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE :  EepromReadByte
 ******************************************************************************
 
 DESCRIPTION       : Read a byte from the ET1310's EEPROM
 
 PARAMETERS        : pAdapter         - pointer to our private adapter structure
 unAddress        - the address from which to read
 pbData           - a pointer to a byte in which to store the value of the read
 unEepronId       - the ID of the EEPROM
 unAddressingMode - how the EEPROM is to be accessed
 
 RETURNS           : SUCCESS or FAILURE
 
 *****************************************************************************/
void EepromReadByte( ET131X_ADAPTER *pAdapter, UINT32 unAddress, PUCHAR pbData,
					 UINT32 unEepromId )
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
        LbcifReadDword( pAdapter, LBCIF_DWORD1_GROUP_OFFSET, &unDword1 );
		
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
        return;
    }
	
	
    /**************************************************************************
	 Step 2:
     *************************************************************************/
    bControl  = 0;
    bControl |= LBCIF_CONTROL_LBCIF_ENABLE;
	
    LbcifWriteByte( pAdapter, LBCIF_CONTROL_REGISTER_OFFSET, bControl );
    
	
    /**************************************************************************
	 Step 3:
     *************************************************************************/
    unAddress |= ( unEepromId << 8 );
	
    LbcifWriteDword( pAdapter, LBCIF_ADDRESS_REGISTER_OFFSET, unAddress );
	
    /**************************************************************************
	 Step 4:
     *************************************************************************/
    for( nIndex = 0; nIndex < MAX_NUM_REGISTER_POLLS; nIndex++ )
    {
        /**********************************************************************
		 Read registers grouped in DWORD1
         *********************************************************************/
        LbcifReadDword( pAdapter, LBCIF_DWORD1_GROUP_OFFSET, &unDword1 );
		
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
        return;
    }
	
	
    /**************************************************************************
	 Step 5:
     *************************************************************************/
    nReadSuccessful = ( bStatus & LBCIF_STATUS_ACK_ERROR ) ? 0 : 1;
	
	
    /**************************************************************************
	 Step 6:
     *************************************************************************/
    *pbData = EXTRACT_DATA_REGISTER( unDword1 );
	
}

/******************************************************************************
 ROUTINE :  et131x_config_parse
 ******************************************************************************
 
 DESCRIPTION       : Parses a configuration from some location (module
 parameters, for example) into the private adapter struct
 
 PARAMETERS        : pAdapter - pointer to the private adapter struct
 *****************************************************************************/



#define PARM_JUMBO_PKT_DEF      1514
#define PARM_JUMBO_PKT_MIN      1514
#define PARM_JUMBO_PKT_MAX      9216


#define PARM_RX_MEM_END_DEF     0x2bc
#define PARM_RX_MEM_END_MIN     0
#define PARM_RX_MEM_END_MAX     0x3ff


static void et131x_config_parse( ET131X_ADAPTER *pAdapter )
{
	pAdapter->SpeedDuplex  = 0;

	
    pAdapter->RegistryVlanTag        = 0;
    pAdapter->RegistryFlowControl    = Both;
    pAdapter->RegistryWOLLink        = 3;
    pAdapter->RegistryWOLMatch       = 7;
    pAdapter->RegistryJumboPacket    = PARM_JUMBO_PKT_DEF;
    pAdapter->RegistryPhyComa        = 0;
    pAdapter->RegistryRxNumBuffers   = 4; // 0..64
    pAdapter->RegistryRxTimeInterval = 10; // 2..320
    pAdapter->RegistryTxNumBuffers   = 4; // 1..40
    pAdapter->RegistryTxTimeInterval = 40; // 1..140
    pAdapter->RegistryRxMemEnd       = PARM_RX_MEM_END_DEF;
    pAdapter->RegistryMACStat        = 1; // 0..1
    pAdapter->RegistryPMWOL          = 0; // 0..1
	
	pAdapter->RegistryNMIDisable = 0; // 0..2
	
    pAdapter->RegistryDMACache       = 0; // 0..15
    pAdapter->RegistryPhyLoopbk      = 0; // 0..1
	
	
    /**************************************************************************
	 Set the MAC address to a default
     *************************************************************************/
    bzero( pAdapter->CurrentAddress, ETH_ALEN );
	
    /**************************************************************************
	 Decode SpeedDuplex
	 
	 Set up as if we are auto negotiating always and then change if we go 
	 into force mode
     *************************************************************************/
    pAdapter->AiForceSpeed = 0;         // Auto speed
    pAdapter->AiForceDpx   = 0;         // Auto FDX
	
	
    /**************************************************************************
	 If we are the 10/100 device, and gigabit is somehow requested then
	 knock it down to 100 full.
     *************************************************************************/
    if(( pAdapter->DeviceID == ET131X_PCI_DEVICE_ID_FAST ) &&
       ( pAdapter->SpeedDuplex == 5 ))
    {
        pAdapter->SpeedDuplex = 4;
    }
	
    switch( pAdapter->SpeedDuplex )
    {
		case 1:                             // 10Mb   Half-Duplex
			pAdapter->AiForceSpeed = 10; 
			pAdapter->AiForceDpx  = 1;
			break;
			
		case 2:                             // 10Mb   Full-Duplex
			pAdapter->AiForceSpeed = 10; 
			pAdapter->AiForceDpx  = 2;
			break;
			
		case 3:                             // 100Mb  Half-Duplex
			pAdapter->AiForceSpeed = 100; 
			pAdapter->AiForceDpx  = 1;
			break;
			
		case 4:                             // 100Mb  Full-Duplex
			pAdapter->AiForceSpeed = 100; 
			pAdapter->AiForceDpx  = 2;
			break;
			
		case 5:                             // 1000Mb Full-Duplex
			pAdapter->AiForceSpeed = 1000; 
			pAdapter->AiForceDpx  = 2;
			break;
    }
	
}


/******************************************************************************
 ROUTINE :  et131x_find_adapter
 ******************************************************************************
 
 DESCRIPTION       : Find the adapter and get all the assigned resources
 
 
 *****************************************************************************/

static int et131x_find_adapter( ET131X_ADAPTER *adapter, IOPCIDevice* pdev )
{
    UCHAR  eepromStat         = 0;
    UCHAR  maxPayload         = 0;
    UCHAR  latencyTimers      = 0;
    /*-----------------------------------------------------------------------*/
	
#if	0
    /**************************************************************************
	 Allow disabling of Non-Maskable Interrupts in I/O space, to
	 support validation.
     *************************************************************************/
    if( adapter->RegistryNMIDisable )
    {
        UCHAR RegisterVal;
		
        RegisterVal  = inb( ET1310_NMI_DISABLE );
        RegisterVal &= 0xf3;
		
        if( adapter->RegistryNMIDisable == 2 )
        {
            RegisterVal |= 0xc;
        }
		
        outb( ET1310_NMI_DISABLE, RegisterVal );
    }
#endif

    /**************************************************************************
	 We first need to check the EEPROM Status code located at offset 0xB2
	 of config space
     *************************************************************************/
	
    eepromStat = pdev->configRead8(ET1310_PCI_EEPROM_STATUS);
										   
    /*************************************************************************
	 THIS IS A WORKAROUND:
 	 * I need to call this function twice to get my card in a 
	 LG M1 Express Dual running. I tried also a msleep before this 
	 function, because I thougth there could be some time condidions
	 but it didn't work. Call the whole function twice also work.
     *************************************************************************/
    eepromStat = pdev->configRead8(ET1310_PCI_EEPROM_STATUS);

    	
	
    /**************************************************************************
	 Determine if the error(s) we care about are present.  If they are
	 present, we need to fail.
     *************************************************************************/
    if( eepromStat & 0x4C )
    { 
        if( adapter->RevisionID == 0x01 )
        {
            INT32   nLoop;
            UCHAR   ucTemp[4] = {0xFE, 0x13, 0x10, 0xFF};
			
            /******************************************************************
			 Re-write the first 4 bytes if we have an eeprom present and 
			 the revision id is 1, this fixes the corruption seen with 
			 1310 B Silicon
             *****************************************************************/
			for( nLoop = 0; nLoop < 3; nLoop++ )
			{
                EepromWriteByte( adapter, nLoop, ucTemp[nLoop], 0 );
			}
        }
		
        /**********************************************************************
		 This error could mean that there was an error reading the eeprom
		 or that the eeprom doesn't exist.  We will treat each case the 
		 same and not try to gather additional information that normally
		 would come from the eeprom, like MAC Address
         *********************************************************************/
        adapter->bEepromPresent = FALSE;
		
        return -EIO;
    }

	adapter->bEepromPresent = TRUE;
	
	
    /**************************************************************************
	 Read the EEPROM for information regarding LED behavior. Refer to 
	 ET1310_phy.c, et131x_xcvr_init(), for its use.
     *************************************************************************/
    EepromReadByte( adapter, 0x70, &adapter->eepromData [0], 0 );
    EepromReadByte( adapter, 0x71, &adapter->eepromData [1], 0 );
	
    if( adapter->eepromData[0] != 0xcd )
    {
        adapter->eepromData[1] = 0x00;  // Disable all optional features
    }
	
	
    /**************************************************************************
	 Let's set up the PORT LOGIC Register.  First we need to know what the 
	 max_payload_size is
     *************************************************************************/
    maxPayload = pdev->configRead8(ET1310_PCI_MAX_PYLD);
    
    {
        UINT16 AckNak [2] = {0x76,  0xD0};
        UINT16 Replay [2] = {0x1E0, 0x2ED};
		
		
        /**********************************************************************
		 Program the Ack/Nak latency and replay timers
         *********************************************************************/
        maxPayload &= 0x07;     // Only the lower 3 bits are valid
		
        if( maxPayload < 2 )
        {
            pdev->configWrite16(ET1310_PCI_ACK_NACK, AckNak[maxPayload]);
            pdev->configWrite16(ET1310_PCI_REPLAY, Replay[maxPayload]);
        }
    }
	
	
    /**************************************************************************
	 l0s and l1 latency timers.  We are using default values.
     *************************************************************************/
    latencyTimers = 0x11;   // Representing 001 for L0s and 010 for L1
	
	pdev->configWrite8(ET1310_PCI_L0L1LATENCY, latencyTimers);
	
    /**************************************************************************
	 Archive Power management capability values for later use
     *************************************************************************/

	adapter->PoMgmt.pmConfigRegs.capId = pdev->configRead8(ET1310_PCI_PM_CAPABILITY);
	adapter->PoMgmt.pmConfigRegs.nextItemPtr = pdev->configRead8(ET1310_PCI_PM_CAPABILITY + 1);
	adapter->PoMgmt.pmConfigRegs.pmcr = pdev->configRead16(ET1310_PCI_PM_CAPABILITY + 2);
	adapter->PoMgmt.pmConfigRegs.pmcsr = pdev->configRead16(ET1310_PCI_PM_CAPABILITY + 4);
	adapter->PoMgmt.pmConfigRegs.pmscr_bse = pdev->configRead8(ET1310_PCI_PM_CAPABILITY + 6);
	adapter->PoMgmt.pmConfigRegs.pm_data_regs = pdev->configRead8(ET1310_PCI_PM_CAPABILITY + 7);
	
    {
        UCHAR read_size_reg;
		
        /******************************************************************
		 Change the max read size to 2k
         *****************************************************************/
		read_size_reg = pdev->configRead8(0x51);
		
        read_size_reg &= 0x8f;
        read_size_reg |= 0x40;
		
		pdev->configWrite8(0x51,read_size_reg);
    }
	
	
    /**************************************************************************
	 PCI Express Configuration registers 0x48-0x5B (Device Control)
     *************************************************************************/
	adapter->PciXDevCtl = pdev->configRead16(ET1310_PCI_DEV_CTRL);

    /**************************************************************************
	 Get MAC address from config space if an eeprom exists, otherwise the
	 MAC address there will not be valid
     *************************************************************************/
    if( adapter->bEepromPresent )
    {
		for(int i = 0; i < ETH_ALEN; i++ ){
			adapter->PermanentAddress[i] = pdev->configRead8(ET1310_PCI_MAC_ADDRESS+i);
		}
    }
    return 0;
}

#if defined(__LP64__)
#define	MEMMASK_PAGE	0xFFFFFFFFFFFFF000UL
#define	MEMMASK_16		0xFFFFFFFFFFFFFFF0UL
#else
#define	MEMMASK_PAGE	0xFFFFF000UL
#define	MEMMASK_16		0xFFFFFFF0UL
#endif

int et131x_tx_dma_memory_alloc( ET131X_ADAPTER *adapter )
{
    int             desc_size = 0;
    TX_RING_t      *tx_ring;
    MP_TCB         *tcb;
	IOBufferMemoryDescriptor* pool;

    /*-----------------------------------------------------------------------*/
	
	
    /**************************************************************************
	 Allocate memory for the TCB's (Transmit Control Block)
     *************************************************************************/
    bzero( adapter->TxRing.MpTcbMem, ( NUM_TCB * sizeof( MP_TCB )));
	
	
    /**************************************************************************
	 Setup some convenience pointers
     *************************************************************************/
    tx_ring = (TX_RING_t *)&adapter->TxRing;
    tcb     = (MP_TCB *)adapter->TxRing.MpTcbMem;
	
    /**************************************************************************
	 Allocate enough memory for the Tx descriptor ring, and allocate some
	 extra so that the ring can be aligned on a 4k boundary.
     *************************************************************************/
    desc_size = ( sizeof( TX_DESC_ENTRY_t ) * NUM_DESC_PER_RING_TX );
    pool = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
									desc_size, MEMMASK_PAGE);	// 64-bits, 4K-aligned
	tx_ring->pTxDescRingPool = pool;
	if(pool){
		pool->prepare();
	} else {
        DBG_ERROR( "Cannot alloc memory for Tx Ring\n" );
        return -ENOMEM;
	}

	
    /**************************************************************************
	 Allocate memory for the Tx status block
     *************************************************************************/
    pool = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
								sizeof( TX_STATUS_BLOCK_t ), MEMMASK_16);	// 64-bits, 16-aligned
	tx_ring->pTxStatusPool = pool;
	if(pool){
		pool->prepare();
	} else {
        DBG_ERROR( "Cannot alloc memory for Tx status block\n" );
        return -ENOMEM;
	}
	
    return 0;
}


/******************************************************************************
 ROUTINE :  et131x_tx_dma_memory_free
 ******************************************************************************
 
 DESCRIPTION       : Free all memory allocated within this module
 
 *****************************************************************************/
void et131x_tx_dma_memory_free( ET131X_ADAPTER *adapter )
{


    if( adapter->TxRing.pTxDescRingPool )
    {
        /**********************************************************************
		 Free memory relating to Tx rings here
         *********************************************************************/
		
		adapter->TxRing.pTxDescRingPool->complete();
		adapter->TxRing.pTxDescRingPool->release();
 		adapter->TxRing.pTxDescRingPool = NULL ;
    }
	
	
    /**************************************************************************
	 Free memory for the Tx status block
     *************************************************************************/
    if( adapter->TxRing.pTxStatusPool )
    {
		adapter->TxRing.pTxStatusPool->complete();
		adapter->TxRing.pTxStatusPool->release();
 		adapter->TxRing.pTxStatusPool = NULL;
    }
	
}
/*===========================================================================*/


/******************************************************************************
 ROUTINE :  et131x_rx_dma_memory_alloc
 ******************************************************************************
 
 DESCRIPTION       : Allocates Free buffer ring 1 for sure, free buffer ring
 0 if required, and the Packet Status Ring
 
 
 *****************************************************************************/
static int et131x_rx_dma_memory_alloc( ET131X_ADAPTER *adapter )
{
    UINT32          OuterLoop, InnerLoop;
    UINT32          bufsize;
    UINT32          pktStatRingSize, FBRChunkSize;
    RX_RING_t      *rx_ring;
	IOBufferMemoryDescriptor* pool;

    /**************************************************************************
	 Setup some convenience pointers
     *************************************************************************/
    rx_ring = (RX_RING_t *)&adapter->RxRing;
	
	
	
    /**************************************************************************
	 The first thing we will do is configure the sizes of the buffer rings.
	 These will change based on jumbo packet support.  Larger jumbo packets
	 increases the size of each entry in FBR0, and the number of entries in
	 FBR0, while at the same time decreasing the number of entries in FBR1.
	 
	 FBR1 holds "large" frames, FBR0 holds "small" frames.  If FBR1 entries
	 are huge in order to accomodate a "jumbo" frame, then it will have less
	 entries.  Conversely, FBR1 will now be relied upon to carry more 
	 "normal" frames, thus it's entry size also increases and the number
	 of entries goes up too (since it now carries "small" + "regular"
	 packets.
	 
	 In this scheme, we try to maintain 512 entries between the two rings.
	 Also, FBR1 remains a constant size - when it's size doubles the
	 number of entries halves.  FBR0 increases in size, however.
     *************************************************************************/
	
    if( adapter->RegistryJumboPacket < 2048 )
    {
        rx_ring->Fbr1BufferSize = 2048;
        rx_ring->Fbr1NumEntries = 512;
    }
    else if( adapter->RegistryJumboPacket < 4096 )
    {
        rx_ring->Fbr1BufferSize = 4096;
        rx_ring->Fbr1NumEntries = 512;
    }
    else
    {
        rx_ring->Fbr1BufferSize = 16384;
        rx_ring->Fbr1NumEntries = 128;
    }
	
    adapter->RxRing.PsrNumEntries = adapter->RxRing.Fbr1NumEntries;
	
	
    /**************************************************************************
	 Allocate an area of memory for Free Buffer Ring 1
     *************************************************************************/
    bufsize = ( sizeof( FBR_DESC_t ) * rx_ring->Fbr1NumEntries);
    pool = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
										bufsize, MEMMASK_PAGE);	// 64-bits, 4K-aligned
	rx_ring->pFbr1RingPool = pool;
	if(pool){
		pool->prepare();
	} else {
        IOLog( "Cannot alloc memory for Free Buffer Ring 1\n" );
        return -ENOMEM;
	}
	
	
    /**************************************************************************
	 Save physical address
	 
	 NOTE : pci_alloc_consistent(), used above to alloc DMA regions, ALWAYS
	 returns SAC (32-bit) addresses. If DAC (64-bit) addresses are
	 ever returned, make sure the high part is retrieved here before
	 storing the adjusted address.
     *************************************************************************/
	
    for( OuterLoop = 0; OuterLoop < (rx_ring->Fbr1NumEntries / FBR_CHUNKS); OuterLoop++ )
    {
		
        /**********************************************************************
		 This code allocates an area of memory big enough for N free
		 buffers + (buffer_size - 1) so that the buffers can be aligned 
		 on 4k boundaries.  If each buffer were aligned
		 to a buffer_size boundary, the effect would be to double the size
		 of FBR0.  By allocating N buffers at once, we reduce this overhead.
         *********************************************************************/
		
        FBRChunkSize = ( FBR_CHUNKS * rx_ring->Fbr1BufferSize );
		pool = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
											FBRChunkSize, MEMMASK_PAGE);	// 64-bits, 4K-aligned
		rx_ring->pFbr1MemPool[OuterLoop] = pool;
		if(pool){
			pool->prepare();
		} else {
            DBG_ERROR( "Could not alloc memory\n" );
            return -ENOMEM;
		}
		
		
        /**********************************************************************
		 See NOTE in "Save Physical Address" comment above
         *********************************************************************/
        IOPhysicalAddress Fbr1TempPa = pool->getPhysicalAddress();
		PUCHAR Fbr1TempVa = (PUCHAR)pool->getBytesNoCopy();
        for( InnerLoop = 0; InnerLoop < FBR_CHUNKS; InnerLoop++ )
        {
            UINT32 index = (OuterLoop * FBR_CHUNKS) + InnerLoop;
			
			
            /******************************************************************
			 Save the Virtual address of this index for quick access later
             *****************************************************************/
            rx_ring->Fbr1.Va[index] = Fbr1TempVa;
			
			
            /******************************************************************
			 now store the physical address in the descriptor so the device
			 can access it
             *****************************************************************/
#ifdef	__LP64__
            rx_ring->Fbr1.PAHigh[index] = (UINT32)(Fbr1TempPa >> 32);
#else
            rx_ring->Fbr1.PAHigh[index] = 0;
#endif
            rx_ring->Fbr1.PALow[index]  = (UINT32) Fbr1TempPa;
            
            Fbr1TempPa += rx_ring->Fbr1BufferSize;
			Fbr1TempVa += rx_ring->Fbr1BufferSize;   
			
        }
    }
	
	
    /**************************************************************************
	 Allocate an area of memory for the FIFO of Packet Status ring entries
     *************************************************************************/
    pktStatRingSize = sizeof( PKT_STAT_DESC_t ) * adapter->RxRing.PsrNumEntries;
	
    pool = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
											pktStatRingSize, MEMMASK_PAGE);	// 64-bits, 4K-aligned
	rx_ring->pPSRingPool = pool;
	if(pool){
		pool->prepare();
	} else {
        DBG_ERROR( "Cannot alloc memory for Packet Status Ring\n" );
        return -ENOMEM;
	}
	
	
    /**************************************************************************
	 Allocate an area of memory for the writeback of status information
     *************************************************************************/
    pool = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIODirectionInOut | kIOMemoryPhysicallyContiguous,
															sizeof( RX_STATUS_BLOCK_t ), MEMMASK_16);	// 64-bits, 16-bytes
	rx_ring->pRxStatusPool = pool;
	if(pool){
		pool->prepare();
	} else {
        DBG_ERROR( "Cannot alloc memory for Status Block\n" );
        return -ENOMEM;
	}
	
    rx_ring->NumRfd = NIC_DEFAULT_NUM_RFD;
	
	
    /**************************************************************************
	 Recv
	 pci_pool_create initializes a lookaside list. 
	 After successful creation, nonpaged fixed-size blocks can be
	 allocated from and freed to the lookaside list.
	 
	 RFDs will be allocated from this pool. 
     *************************************************************************/
    adapter->RecvLookaside = (PMP_RFD)IOMalloc( sizeof( MP_RFD ) * rx_ring->NumRfd );
	
	
    MP_SET_FLAG( adapter, fMP_ADAPTER_RECV_LOOKASIDE );
	
	
    /**************************************************************************
	 The RFDs are going to be put on lists later on, so initialize the lists
	 now.
     *************************************************************************/
    INIT_LIST_HEAD( &rx_ring->RecvList );
	
    return 0;
}

static void et131x_rfd_resources_free( ET131X_ADAPTER *adapter, MP_RFD *pMpRfd )
{
	// pMpRfd->Packet = NULL;
    // kmem_cache_free( adapter->RxRing.RecvLookaside, pMpRfd );
}

/******************************************************************************
 ROUTINE :  et131x_rx_dma_memory_free
 ******************************************************************************/

static void et131x_rx_dma_memory_free( ET131X_ADAPTER *adapter )
{
    UINT32     index;
    PMP_RFD    pMpRfd;
    RX_RING_t *rx_ring;
    /*-----------------------------------------------------------------------*/
	
	
    /**************************************************************************
	 Setup some convenience pointers
     *************************************************************************/
    rx_ring = (RX_RING_t *)&adapter->RxRing;
	
	
    /**************************************************************************
	 Free RFDs and associated packet descriptors
     *************************************************************************/
    DBG_ASSERT( rx_ring->nReadyRecv == rx_ring->NumRfd );
	
    while( !list_empty( &rx_ring->RecvList ))
    {
        pMpRfd = (MP_RFD *)list_entry( rx_ring->RecvList.next,
									  MP_RFD,
									  list_node );
		
        list_del( &pMpRfd->list_node );
        et131x_rfd_resources_free( adapter, pMpRfd );
    }
	
	
    /**************************************************************************
	 Free Free Buffer Ring 1
     *************************************************************************/
    if( rx_ring->pFbr1RingPool )
    {
        /**********************************************************************
		 First the packet memory
         *********************************************************************/
        for( index = 0; index < 
			(rx_ring->Fbr1NumEntries / FBR_CHUNKS); index++ )
        {
            if( rx_ring->pFbr1MemPool[index] )
            {
				rx_ring->pFbr1MemPool[index]->complete();
				rx_ring->pFbr1MemPool[index]->release();
                rx_ring->pFbr1MemPool[index] = 0;
            }
        }
		
		
        /**********************************************************************
		 Now the FIFO itself
         *********************************************************************/
		rx_ring->pFbr1RingPool->complete();
		rx_ring->pFbr1RingPool->release();
        rx_ring->pFbr1RingPool = NULL;
    }
	
	
	
    /**************************************************************************
	 Free Packet Status Ring
     *************************************************************************/
    if( rx_ring->pPSRingPool )
    {
		rx_ring->pPSRingPool->complete();
		rx_ring->pPSRingPool->release();
        rx_ring->pPSRingPool = NULL;
    }
	
	
    /**********************************************************************
	 Free area of memory for the writeback of status information
     *********************************************************************/
    if( rx_ring->pRxStatusPool )
    {
		rx_ring->pRxStatusPool->complete();
		rx_ring->pRxStatusPool->release();
		rx_ring->pRxStatusPool = NULL;
    }
	
	
    /**************************************************************************
	 Free receive buffer pool
     *************************************************************************/
	
	
    /**************************************************************************
	 Free receive packet pool
     *************************************************************************/
	
	
    /**************************************************************************
	 Destroy the lookaside (RFD) pool
     *************************************************************************/
    if( MP_TEST_FLAG( adapter, fMP_ADAPTER_RECV_LOOKASIDE ))
    {
        IOFree( adapter->RecvLookaside, sizeof( MP_RFD ) * rx_ring->NumRfd );
        MP_CLEAR_FLAG( adapter, fMP_ADAPTER_RECV_LOOKASIDE );
    }
	
	
	
    /**************************************************************************
	 Reset Counters
     *************************************************************************/
    rx_ring->nReadyRecv = 0;
}


/******************************************************************************
 ROUTINE :  et131x_init_recv
 ******************************************************************************
 
 DESCRIPTION       : Initialize receive data structures.
 
 
 *****************************************************************************/
static int et131x_init_recv( ET131X_ADAPTER *adapter )
{
    int               status        = -ENOMEM;
    PMP_RFD           pMpRfd        = NULL;      
    UINT32            RfdCount;
    UINT32            TotalNumRfd   = 0;
    RX_RING_t        *rx_ring       = NULL;
	
	
    /**************************************************************************
	 Setup some convenience pointers
     *************************************************************************/
    rx_ring = (RX_RING_t *)&adapter->RxRing;
	
	
    /**************************************************************************
	 Setup each RFD
     *************************************************************************/
    for( RfdCount = 0; RfdCount < rx_ring->NumRfd; RfdCount++ )
    {
        pMpRfd = adapter->RecvLookaside + RfdCount;
		
        if( !pMpRfd )
        {
            DBG_ERROR( "Couldn't alloc RFD out of kmem_cache\n" );
            status = -ENOMEM;
            continue;
        }
        
		// init
		// pMpRfd->Packet = NULL;
		
		
        /**********************************************************************
		 Add this RFD to the RecvList
         *********************************************************************/
        list_add_tail( &pMpRfd->list_node, &rx_ring->RecvList );
		
		
        /**********************************************************************
		 Increment both the available RFD's, and the total RFD's.
         *********************************************************************/
        rx_ring->nReadyRecv++;
        TotalNumRfd++;                      
    }
	
    if( TotalNumRfd > NIC_MIN_NUM_RFD )
    {
        status = 0;
    }
	
    rx_ring->NumRfd = TotalNumRfd;
	
    if( status != 0 )
    {
        //kmem_cache_free( rx_ring->RecvLookaside, pMpRfd );
        DBG_ERROR( "Allocation problems in et131x_init_recv\n" );
    }
    return status;
}



#define CRC32_POLY 0x4C11DB7

static UINT32 crc32byte( UINT32 remainder, UCHAR data )
{
    int	   index;
	UINT32 remndr, hitbit;
    /*-----------------------------------------------------------------------*/
	
	
	remndr = remainder;
	
	for( index = 0; index < 8; index++ )
    {
		hitbit = (remndr >> 31) ^ (data & 0x01);
		
		data   = data >> 1;     // get the next data bit
		remndr = remndr << 1;   // get the next remainder bit
		
		if( hitbit )
        {
			remndr = remndr ^ CRC32_POLY;
		}
	}
	
	return remndr;
}

static UINT32 et131x_calc_enet_crc( PUCHAR Message, UINT32 MessageSize )
{
    UINT32 Result = 0xFFFFFFFF;
    UINT32 i;
    /*-----------------------------------------------------------------------*/
	
    for( i = 0; i < MessageSize; i++ )
    {
        Result = crc32byte( Result, *Message );
        Message++;
    }
	
    return Result;
}

/******************************************************************************
 ROUTINE:  ConfigMacRegs1
 ******************************************************************************
 
 DESCRIPTION:
 Used to configure the first part of MAC regs to a known initialized 
 state
 *****************************************************************************/
void ConfigMACRegs1( ET131X_ADAPTER *pAdapter )
{
    PMAC_t              pMac;
    MAC_STATION_ADDR1_t station1;
    MAC_STATION_ADDR2_t station2;
    MAC_IPG_t           ipg;
    MAC_HFDP_t          hfdp;
    MII_MGMT_CFG_t      mii_mgmt_cfg;
    /*-----------------------------------------------------------------------*/
	
	
    /**************************************************************************
	 Let's get our pointer to the MAC regs
     *************************************************************************/
    pMac = &pAdapter->CSRAddress->mac;
	
	
    /**************************************************************************
	 First we need to reset everything.  Write to MAC configuration register 
	 1 to perform reset.
     *************************************************************************/
    pMac->cfg1.value = 0xC00F0000;
	
	
    /**************************************************************************
	 Next lets configure the MAC Inter-packet gap register
     *************************************************************************/
    ipg.bits.non_B2B_ipg_1        = 0x38; //58d
    ipg.bits.non_B2B_ipg_2        = 0x58; //88d
    ipg.bits.min_ifg_enforce      = 0x50; //80d
    ipg.bits.B2B_ipg              = 0x60; //96d
	
    pMac->ipg.value = ipg.value;
	
	
    /**************************************************************************
	 Next lets configure the MAC Half Duplex register
     *************************************************************************/
    hfdp.bits.alt_beb_trunc       = 0xA;
    hfdp.bits.alt_beb_enable      = 0x0;
    hfdp.bits.bp_no_backoff       = 0x0;
    hfdp.bits.no_backoff          = 0x0;
    hfdp.bits.excess_defer        = 0x1;
    hfdp.bits.rexmit_max          = 0xF;
    hfdp.bits.coll_window         = 0x37; //55d
	
    pMac->hfdp.value = hfdp.value;
	
	
    /**************************************************************************
	 Next lets configure the MAC Interface Control register
     *************************************************************************/
    pMac->if_ctrl.value = 0x0;
	
	
    /**************************************************************************
	 Let's move on to setting up the mii managment configuration
     *************************************************************************/
    mii_mgmt_cfg.bits.reset_mii_mgmt    = 0;
    mii_mgmt_cfg.bits.scan_auto_incremt = 0;
    mii_mgmt_cfg.bits.preamble_suppress = 0;
    mii_mgmt_cfg.bits.mgmt_clk_reset    = 0x7;
	
    pMac->mii_mgmt_cfg.value = mii_mgmt_cfg.value;
	
	
    /**************************************************************************
	 Next lets configure the MAC Station Address register.  These values are 
	 read from the EEPROM during initialization and stored in the adapter 
	 structure.  We write what is stored in the adapter structure to the MAC 
	 Station Address registers high and low.  This station address is used
	 for generating and checking pause control packets.
     *************************************************************************/
    station2.bits.Octet1    = pAdapter->CurrentAddress[0];
    station2.bits.Octet2    = pAdapter->CurrentAddress[1];
    station1.bits.Octet3    = pAdapter->CurrentAddress[2];
    station1.bits.Octet4    = pAdapter->CurrentAddress[3];
    station1.bits.Octet5    = pAdapter->CurrentAddress[4];
    station1.bits.Octet6    = pAdapter->CurrentAddress[5]; 
	
    pMac->station_addr_1.value = station1.value;
    pMac->station_addr_2.value = station2.value;
	
	
    /**************************************************************************
	 Max ethernet packet in bytes that will passed by the mac without being
	 truncated.  Allow the MAC to pass 8 more than our max packet size.  This
	 is 4 for the Ethernet CRC and 4 for the VLAN ID.
	 
	 Packets larger than (RegistryJumboPacket) that do not contain a VLAN
	 ID will be dropped by the Rx function.
     *************************************************************************/
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
    pMac->max_fm_len.value = pAdapter->RegistryJumboPacket + 8;
#else
    pMac->max_fm_len.value = pAdapter->RegistryJumboPacket + 4;
#endif
	
	
    /**************************************************************************
	 clear out MAC config reset
     *************************************************************************/
    pAdapter->CSRAddress->mac.cfg1.value = 0x0;
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE :  SetupDeviceForUnicast
 ******************************************************************************
 
 DESCRIPTION       :
 Use to set the ET1310 to do unicast filtering 
 
 *****************************************************************************/
static void SetupDeviceForUnicast( ET131X_ADAPTER *pAdapter )
{
    RXMAC_UNI_PF_ADDR1_t uni_pf1;
    RXMAC_UNI_PF_ADDR2_t uni_pf2;
    RXMAC_UNI_PF_ADDR3_t uni_pf3;
    /*-----------------------------------------------------------------------*/
	
	
    /**************************************************************************
	 Set up unicast packet filter reg 3 to be the first two octets of the 
	 MAC address for both address
     *************************************************************************/
    /**************************************************************************
	 Set up unicast packet filter reg 2 to be the octets 2 - 5 of the 
	 MAC address for second address
     *************************************************************************/
    /**************************************************************************
	 Set up unicast packet filter reg 3 to be the octets 2 - 5 of the 
	 MAC address for first address
     *************************************************************************/
    uni_pf3.bits.addr1_1 = pAdapter->CurrentAddress[0];
    uni_pf3.bits.addr1_2 = pAdapter->CurrentAddress[1];
    uni_pf3.bits.addr2_1 = pAdapter->CurrentAddress[0];
    uni_pf3.bits.addr2_2 = pAdapter->CurrentAddress[1];
	
    uni_pf2.bits.addr2_3 = pAdapter->CurrentAddress[2];
    uni_pf2.bits.addr2_4 = pAdapter->CurrentAddress[3];
    uni_pf2.bits.addr2_5 = pAdapter->CurrentAddress[4];
    uni_pf2.bits.addr2_6 = pAdapter->CurrentAddress[5];
	
    uni_pf1.bits.addr1_3 = pAdapter->CurrentAddress[2];
    uni_pf1.bits.addr1_4 = pAdapter->CurrentAddress[3];
    uni_pf1.bits.addr1_5 = pAdapter->CurrentAddress[4];
    uni_pf1.bits.addr1_6 = pAdapter->CurrentAddress[5];
	
    if( pAdapter->CSRAddress->global.pm_csr.bits.pm_phy_sw_coma == 0 )
    {
        pAdapter->CSRAddress->rxmac.uni_pf_addr1 = uni_pf1;
        pAdapter->CSRAddress->rxmac.uni_pf_addr2 = uni_pf2;
        pAdapter->CSRAddress->rxmac.uni_pf_addr3 = uni_pf3;
    }
	
}

/******************************************************************************
 ROUTINE :  SetupDeviceForMulticast
 ******************************************************************************
 
 DESCRIPTION       :
 Use to set the ET1310 to do multicast filtering 
 
 PARAMETERS        :
 pAdapter - pointer to our adapter structure
 
 *****************************************************************************/
static void SetupDeviceForMulticast( ET131X_ADAPTER *pAdapter )
{
    UINT32             nIndex;
    UINT32             result;
    RXMAC_MULTI_HASH_t hash1 = {0};
    RXMAC_MULTI_HASH_t hash2 = {0};
    RXMAC_MULTI_HASH_t hash3 = {0};
    RXMAC_MULTI_HASH_t hash4 = {0};
    /*-----------------------------------------------------------------------*/
	
	
	
    /**************************************************************************
	 If ET131X_PACKET_TYPE_MULTICAST is specified, then we provision the
	 multi-cast LIST.  If it is NOT specified, (and "ALL" is not specified)
	 then we should pass NO multi-cast addresses to the driver.
     *************************************************************************/
    if( pAdapter->PacketFilter & ET131X_PACKET_TYPE_MULTICAST )
    {
        DBG_VERBOSE( "MULTICAST flag is set, MCCount: %d\n",
					pAdapter->MCAddressCount );
		
		
        /**********************************************************************
		 Loop through our multicast array and set up the device
		 **********************************************************************/
        for( nIndex = 0; nIndex < pAdapter->MCAddressCount; nIndex++ )
        {
            DBG_VERBOSE( "MCList[%d]: %02x:%02x:%02x:%02x:%02x:%02x\n",
						nIndex,
						pAdapter->MCList[nIndex][0],
						pAdapter->MCList[nIndex][1],
						pAdapter->MCList[nIndex][2],
						pAdapter->MCList[nIndex][3],
						pAdapter->MCList[nIndex][4],
						pAdapter->MCList[nIndex][5] );
			
            result = et131x_calc_enet_crc( pAdapter->MCList[nIndex], 6 );
			
            result = ( result & 0x3F800000 ) >> 23;
			
            if( result < 32 )
            {
                hash1.hash |= ( 1 << result );
            }
            else if(( 31 < result ) && ( result < 64 ))
            {
                result -= 32;
                hash2.hash |= ( 1 << result );
            }
            else if(( 63 < result ) && ( result < 96 ))
            {
                result -= 64;
                hash3.hash |= ( 1 << result );
            }
            else
            {
                result -= 96;
                hash4.hash |= ( 1 << result );
            }
        }
    }
	
	
    /**************************************************************************
	 Write out the new hash to the device
     *************************************************************************/
    if( pAdapter->CSRAddress->global.pm_csr.bits.pm_phy_sw_coma == 0 )
    {
		pAdapter->CSRAddress->rxmac.multi_hash1.hash = hash1.hash;
		pAdapter->CSRAddress->rxmac.multi_hash2.hash = hash2.hash;
		pAdapter->CSRAddress->rxmac.multi_hash3.hash = hash3.hash;
		pAdapter->CSRAddress->rxmac.multi_hash4.hash = hash4.hash;
    }
	
}
/*===========================================================================*/


/******************************************************************************
 ROUTINE:  ConfigRxMacRegs
 ******************************************************************************
 
 DESCRIPTION:
 Used to configure the RX MAC registers in the JAGCore
 
 
 *****************************************************************************/
void ConfigRxMacRegs( ET131X_ADAPTER *pAdapter )
{
    PRXMAC_t                  pRxMac;
    RXMAC_WOL_SA_LO_t         sa_lo;
    RXMAC_WOL_SA_HI_t         sa_hi;
    RXMAC_PF_CTRL_t           pf_ctrl = {0};
	
	
    /**************************************************************************
	 Let's get a local pointer to the RX MAC Registers
     *************************************************************************/
    pRxMac = &pAdapter->CSRAddress->rxmac;
	
	
    /**************************************************************************
	 Disable the MAC while it is being configured (also disable WOL)
     *************************************************************************/
    pRxMac->ctrl.value = 0x8;
	
	
    /**************************************************************************
	 Initialize WOL to disabled.
     *************************************************************************/
	
    pRxMac->crc0.value      = 0x0;
    pRxMac->crc12.value     = 0x0000;
    pRxMac->crc34.value     = 0x0000;
	
	
    /**************************************************************************
	 We need to set the WOL mask0 ñ mask4 next.  We initialize it to its
	 default Values of 0x00000000 because there are not WOL masks as of
	 this time.
     *************************************************************************/
    pRxMac->mask0_word0.mask                = 0x00000000;
    pRxMac->mask0_word1.mask                = 0x00000000;
    pRxMac->mask0_word2.mask                = 0x00000000;
    pRxMac->mask0_word3.mask                = 0x00000000;
	
    pRxMac->mask1_word0.mask                = 0x00000000;
    pRxMac->mask1_word1.mask                = 0x00000000;
    pRxMac->mask1_word2.mask                = 0x00000000;
    pRxMac->mask1_word3.mask                = 0x00000000;
	
    pRxMac->mask2_word0.mask                = 0x00000000;
    pRxMac->mask2_word1.mask                = 0x00000000;
    pRxMac->mask2_word2.mask                = 0x00000000;
    pRxMac->mask2_word3.mask                = 0x00000000;
	
    pRxMac->mask3_word0.mask                = 0x00000000;
    pRxMac->mask3_word1.mask                = 0x00000000;
    pRxMac->mask3_word2.mask                = 0x00000000;
    pRxMac->mask3_word3.mask                = 0x00000000;
	
    pRxMac->mask4_word0.mask                = 0x00000000;
    pRxMac->mask4_word1.mask                = 0x00000000;
    pRxMac->mask4_word2.mask                = 0x00000000;
    pRxMac->mask4_word3.mask                = 0x00000000;
	
	
    /**************************************************************************
	 Lets setup the WOL Source Address
     *************************************************************************/
    sa_lo.bits.sa3                  = pAdapter->CurrentAddress[2]; 
    sa_lo.bits.sa4                  = pAdapter->CurrentAddress[3];
    sa_lo.bits.sa5                  = pAdapter->CurrentAddress[4];
    sa_lo.bits.sa6                  = pAdapter->CurrentAddress[5];
    pRxMac->sa_lo.value = sa_lo.value;
    
    sa_hi.bits.sa1                  = pAdapter->CurrentAddress[0];
    sa_hi.bits.sa2                  = pAdapter->CurrentAddress[1];
    pRxMac->sa_hi.value = sa_hi.value;
	
	
    /**************************************************************************
	 Disable all Packet Filtering
     *************************************************************************/
    pRxMac->pf_ctrl.value = 0;
	
	
    /**************************************************************************
	 Let's initialize the Unicast Packet filtering address
     *************************************************************************/
    if( pAdapter->PacketFilter & ET131X_PACKET_TYPE_DIRECTED )
    {
        SetupDeviceForUnicast( pAdapter );
        pf_ctrl.bits.filter_uni_en = 1;
    }
    else
    {
        pRxMac->uni_pf_addr1.value       = 0x00000000;
        pRxMac->uni_pf_addr2.value       = 0x00000000;
        pRxMac->uni_pf_addr3.value       = 0x00000000;
    }
	
	
    /**************************************************************************
	 Let's initialize the Multicast hash
     *************************************************************************/
    if( pAdapter->PacketFilter & ET131X_PACKET_TYPE_ALL_MULTICAST )
    {
        pf_ctrl.bits.filter_multi_en = 0;
    }
    else
    {
        pf_ctrl.bits.filter_multi_en = 1;
        SetupDeviceForMulticast( pAdapter );
    }
	
	
    /**************************************************************************
	 Runt packet filtering.  Didn't work in version A silicon.
     *************************************************************************/
    pf_ctrl.bits.min_pkt_size   = NIC_MIN_PACKET_SIZE + 4;
    pf_ctrl.bits.filter_frag_en = 1;
	
    if( pAdapter->RegistryJumboPacket > 8192 )
    {
        RXMAC_MCIF_CTRL_MAX_SEG_t mcif_ctrl_max_seg;
		
		
        /**********************************************************************
		 In order to transmit jumbo packets greater than 8k, the FIFO
		 between RxMAC and RxDMA needs to be reduced in size to (16k -
		 Jumbo packet size).  In order to implement this, we must use
		 "cut through" mode in the RxMAC, which chops packets down into
		 segments which are (max_size * 16).  In this case we selected
		 256 bytes, since this is the size of the PCI-Express TLP's that
		 the 1310 uses.
         *********************************************************************/
        mcif_ctrl_max_seg.bits.seg_en   = 0x1;
        mcif_ctrl_max_seg.bits.fc_en    = 0x0;
        mcif_ctrl_max_seg.bits.max_size = 0x10;
		
        pRxMac->mcif_ctrl_max_seg.value = mcif_ctrl_max_seg.value;
    }
    else
    {
        pRxMac->mcif_ctrl_max_seg.value = 0x0;
    }
	
	
    /**************************************************************************
	 Initialize the MCIF water marks
     *************************************************************************/
    pRxMac->mcif_water_mark.value    = 0x0;
	
	
    /**************************************************************************
	 Initialize the MIF control
     *************************************************************************/
    pRxMac->mif_ctrl.value     = 0x0;
	
	
    /**************************************************************************
	 Initialize the Space Available Register
     *************************************************************************/
    pRxMac->space_avail.value    = 0x0;
	
    /* Initialize the the mif_ctrl register
     * bit 3  - Receive code error. One or more nibbles were signaled as errors
	 during the reception of the packet.  Clear this bit in Gigabit,
	 set it in 100Mbit.  This was derived experimentally at UNH.
     * bit 4  - Receive CRC error. The packetís CRC did not match the
	 internally generated CRC.
     * bit 5  - Receive length check error. Indicates that frame length field
	 value in the packet does not match the actual data byte length
	 and is not a type field.
     * bit 16 - Receive frame truncated.
     * bit 17 - Drop packet enable
     */
    if( pAdapter->uiLinkSpeed == TRUEPHY_SPEED_100MBPS )
    {
        pRxMac->mif_ctrl.value = 0x30038;
    }
    else
    {
        pRxMac->mif_ctrl.value = 0x30030;
    }
	
	
    /**************************************************************************
	 Finally we initialize RxMac to be enabled & WOL disabled.  Packet filter
	 is always enabled since it is where the runt packets are supposed to be
	 dropped.  For version A silicon, runt packet dropping doesn't work, so
	 it is disabled in the pf_ctrl register, but we still leave the packet
	 filter on.
     *************************************************************************/
    pRxMac->pf_ctrl = pf_ctrl;
    pRxMac->ctrl.value = 0x9;
}


/******************************************************************************
 ROUTINE:  ConfigTxMacRegs
 ******************************************************************************
 
 DESCRIPTION:
 used to configure the TX MAC registers of the JAGCore
 
 *****************************************************************************/
void ConfigTxMacRegs( ET131X_ADAPTER *pAdapter )
{
    PTXMAC_t pTxMac;
    TXMAC_CF_PARAM_t Local;
	
	
    /**************************************************************************
	 Let's get the pointer to tx mac section of regs
     *************************************************************************/
    pTxMac = &pAdapter->CSRAddress->txmac;
	
	
    /**************************************************************************
	 We need to update the Control Frame Parameters
	 cfpt - control frame pause timer set to 64 (0x40)
	 cfep - control frame extended pause timer set to 0x0
     *************************************************************************/
    if( pAdapter->FlowControl == None )
    {
        pTxMac->cf_param.value  = 0x0;
    }
    else
    {
        Local.bits.cfpt = 0x40;
        Local.bits.cfep = 0x0;
        pTxMac->cf_param.value  = Local.value;
    }
}
/*===========================================================================*/




/******************************************************************************
 ROUTINE:  ConfigMacStatRegs
 ******************************************************************************
 
 DESCRIPTION:
 Used to configure the MAC STAT section of the JAGCore
 
 *****************************************************************************/
void ConfigMacStatRegs( ET131X_ADAPTER *pAdapter )
{
    PMAC_STAT_t pDevMacStat;
    
    pDevMacStat = &pAdapter->CSRAddress->macStat;
	
	
    /**************************************************************************
	 Next we need to initialize all the MAC_STAT registers to zero on the 
	 device.
     *************************************************************************/
    pDevMacStat->RFcs = 0x0;
    pDevMacStat->RAln = 0x0;
    pDevMacStat->RFlr = 0x0;
    pDevMacStat->RDrp = 0x0;
    pDevMacStat->RCde = 0x0;
    pDevMacStat->ROvr = 0x0;
    pDevMacStat->RFrg = 0x0;
	
    pDevMacStat->TScl = 0x0;
    pDevMacStat->TDfr = 0x0;
    pDevMacStat->TMcl = 0x0;
    pDevMacStat->TLcl = 0x0;
    pDevMacStat->TNcl = 0x0;
    pDevMacStat->TOvr = 0x0;
    pDevMacStat->TUnd = 0x0;
	
	
    /***************************************************************************
	 Unmask any counters that we want to track the overflow of.  Initially
	 this will be all counters.  It may become clear later that we do not
	 need to track all counters.
     **************************************************************************/
    {
        MAC_STAT_REG_1_t Carry1M = {0xffffffff};
		
        Carry1M.bits.rdrp         = 0x0;
        Carry1M.bits.rjbr         = 0x1;
        Carry1M.bits.rfrg         = 0x0;
        Carry1M.bits.rovr         = 0x0;
        Carry1M.bits.rund         = 0x1;
        Carry1M.bits.rcse         = 0x1;
        Carry1M.bits.rcde         = 0x0;
        Carry1M.bits.rflr         = 0x0;
        Carry1M.bits.raln         = 0x0;
        Carry1M.bits.rxuo         = 0x1;
        Carry1M.bits.rxpf         = 0x1;
        Carry1M.bits.rxcf         = 0x1;
        Carry1M.bits.rbca         = 0x1;
        Carry1M.bits.rmca         = 0x1;
        Carry1M.bits.rfcs         = 0x0;
        Carry1M.bits.rpkt         = 0x1;
        Carry1M.bits.rbyt         = 0x1;
        Carry1M.bits.trmgv        = 0x1;
        Carry1M.bits.trmax        = 0x1;
        Carry1M.bits.tr1k         = 0x1;
        Carry1M.bits.tr511        = 0x1;
        Carry1M.bits.tr255        = 0x1;
        Carry1M.bits.tr127        = 0x1;
        Carry1M.bits.tr64         = 0x1;
		
        pDevMacStat->Carry1M = Carry1M;
    }
	
    {
        MAC_STAT_REG_2_t Carry2M = {0xffffffff};
		
        Carry2M.bits.tdrp         = 0x1;
        Carry2M.bits.tpfh         = 0x1;
        Carry2M.bits.tncl         = 0x0;
        Carry2M.bits.txcl         = 0x1;
        Carry2M.bits.tlcl         = 0x0;
        Carry2M.bits.tmcl         = 0x0;
        Carry2M.bits.tscl         = 0x0;
        Carry2M.bits.tedf         = 0x1;
        Carry2M.bits.tdfr         = 0x0;
        Carry2M.bits.txpf         = 0x1;
        Carry2M.bits.tbca         = 0x1;
        Carry2M.bits.tmca         = 0x1;
        Carry2M.bits.tpkt         = 0x1;
        Carry2M.bits.tbyt         = 0x1;
        Carry2M.bits.tfrg         = 0x1;
        Carry2M.bits.tund         = 0x0;
        Carry2M.bits.tovr         = 0x0;
        Carry2M.bits.txcf         = 0x1;
        Carry2M.bits.tfcs         = 0x1;
        Carry2M.bits.tjbr         = 0x1;
		
        pDevMacStat->Carry2M = Carry2M;
    }
}


/******************************************************************************
 ROUTINE:  ConfigGlobalRegs
 ******************************************************************************
 
 DESCRIPTION:
 Used to configure the global registers on the JAGCore
 
 *****************************************************************************/
static void ConfigGlobalRegs( ET131X_ADAPTER *pAdapter )
{
    PGLOBAL_t pGbl;

    /**************************************************************************
	 Map a local pointer to the global section of the JAGCore
     *************************************************************************/
    pGbl = &pAdapter->CSRAddress->global;
	
    if( pAdapter->RegistryPhyLoopbk == FALSE )
    {
        if( pAdapter->RegistryJumboPacket < 2048 )
        {
            /******************************************************************
			 Tx / RxDMA and Tx/Rx MAC interfaces have a 1k word block of RAM
			 that the driver can split between Tx and Rx as it desires.  Our
			 default is to split it 50/50:
             *****************************************************************/
            pGbl->rxq_start_addr.value = 0;
            pGbl->rxq_end_addr.value   = pAdapter->RegistryRxMemEnd;
            pGbl->txq_start_addr.value = pGbl->rxq_end_addr.bits.rxq_end_addr + 1;
            pGbl->txq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
        }
        else if( pAdapter->RegistryJumboPacket < 8192 )
        {
            /******************************************************************
			 For jumbo packets > 2k in length, but < 8k, split 50-50.
             *****************************************************************/
            pGbl->rxq_start_addr.value = 0;
            pGbl->rxq_end_addr.value   = INTERNAL_MEM_RX_OFFSET;
            pGbl->txq_start_addr.value = INTERNAL_MEM_RX_OFFSET + 1;
            pGbl->txq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
        }
        else
        {
            /******************************************************************
			 9216 is the only packet size greater than 8k that is available.
			 The Tx buffer has to be big enough for one whole packet on the
			 Tx side.  We'll make the Tx 9408, and give the rest to Rx
             *****************************************************************/
            pGbl->rxq_start_addr.value = 0x0000;
            pGbl->rxq_end_addr.value   = 0x01b3;
            pGbl->txq_start_addr.value = 0x01b4;
            pGbl->txq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
        }
		
        /**********************************************************************
		 Initialize the loopback register.  Disable all loopbacks.
         *********************************************************************/
        pGbl->loopback.value       = 0x0;
    }
    else
    {
        /**************************************************************************
		 For PHY Line loopback, the memory is configured as if Tx and Rx both
		 have all the memory.  This is because the RxMAC will write data into
		 the space, and the TxMAC will read it out.
		 *************************************************************************/
        pGbl->rxq_start_addr.value = 0;
        pGbl->rxq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
        pGbl->txq_start_addr.value = 0;
        pGbl->txq_end_addr.value   = INTERNAL_MEM_SIZE - 1;
		
        /**************************************************************************
		 Initialize the loopback register (MAC loopback).
         *************************************************************************/
        pGbl->loopback.value       = 0x1;
    }
	
    /**************************************************************************
	 MSI Register
     *************************************************************************/
    pGbl->msi_config.value     = 0x0;
	
	
    /**************************************************************************
	 By default, disable the watchdog timer.  It will be enabled when 
	 a packet is queued.
     *************************************************************************/
    pGbl->watchdog_timer     = 0;
	
}
/*===========================================================================*/

/******************************************************************************
 ROUTINE:  ConfigMMCRegs
 ******************************************************************************
 
 DESCRIPTION:
 Used to configure the main memory registers in the JAGCore
 
 PARAMETERS :
 pAdapter - pointer to our adapter structure
 
 RETURNS    :
 NONE
 
 *****************************************************************************/
static void ConfigMMCRegs( ET131X_ADAPTER *pAdapter )
{
    MMC_CTRL_t  mmc_ctrl = {0};
	
	
    /**************************************************************************
	 All we need to do is initialize the Memory Control Register 
     *************************************************************************/
    mmc_ctrl.bits.force_ce        = 0x0;
    mmc_ctrl.bits.rxdma_disable   = 0x0;
    mmc_ctrl.bits.txdma_disable   = 0x0;
    mmc_ctrl.bits.txmac_disable   = 0x0;
    mmc_ctrl.bits.rxmac_disable   = 0x0;
    mmc_ctrl.bits.arb_disable     = 0x0;
    mmc_ctrl.bits.mmc_enable      = 0x1;
	
    pAdapter->CSRAddress->mmc.mmc_ctrl.value = mmc_ctrl.value;
}


/******************************************************************************
 ROUTINE:  ConfigRxDmaRegs
 ******************************************************************************
 DESCRIPTION:
 START OF Rx_DMA INIT SEQUENCE
 
 *****************************************************************************/
static void ConfigRxDmaRegs( ET131X_ADAPTER *pAdapter )
{
    PRXDMA_t      pRxDma;
    PFBR_DESC_t   pFbrEntry;
    UINT32        iEntry;
	IOPhysicalAddress pAdrs;
	
    /**************************************************************************
	 Let's get our pointer to the RxDma section of regs
     *************************************************************************/
    pRxDma = &pAdapter->CSRAddress->rxdma;
	
	
    /**************************************************************************
	 Load the completion writeback physical address
	 
	 NOTE : pci_alloc_consistent(), used above to alloc DMA regions, ALWAYS
	 returns SAC (32-bit) addresses. If DAC (64-bit) addresses are
	 ever returned, make sure the high part is retrieved here before
	 storing the adjusted address.
     *************************************************************************/
	pAdrs = pAdapter->RxRing.pRxStatusPool->getPhysicalAddress();
#ifdef __LP64__
    pRxDma->dma_wb_base_hi.addr_hi = (UINT32)( pAdrs >> 32 );
#else
    pRxDma->dma_wb_base_hi.addr_hi = 0;
#endif
    pRxDma->dma_wb_base_lo.addr_lo = (UINT32)( pAdrs );
	
	bzero( pAdapter->RxRing.pRxStatusPool->getBytesNoCopy(), sizeof( RX_STATUS_BLOCK_t ));
	
	
    /**************************************************************************
	 Set the address and parameters of the packet status ring into the 1310's
	 registers
     *************************************************************************/
	pAdrs = pAdapter->RxRing.pPSRingPool->getPhysicalAddress();
#ifdef __LP64__
    pRxDma->psr_base_hi.addr_hi   = (UINT32)( pAdrs >> 32 );
#else
    pRxDma->psr_base_hi.addr_hi   = 0;
#endif
    pRxDma->psr_base_lo.addr_lo   = (UINT32)( pAdrs );
	
    pRxDma->psr_num_des.value     = pAdapter->RxRing.PsrNumEntries - 1;
	
    pRxDma->psr_full_offset.value = 0;
	
    pRxDma->psr_min_des.value     = 
	( pRxDma->psr_num_des.bits.psr_ndes * LO_MARK_PERCENT_FOR_PSR ) / 100;

	IOSimpleLockLock(pAdapter->RcvLock);
	
	
    /**************************************************************************
	 These local variables track the PSR in the adapter structure
     *************************************************************************/
    pAdapter->RxRing.local_psr_full.bits.psr_full      = 0;
    pAdapter->RxRing.local_psr_full.bits.psr_full_wrap = 0;
	
	
    /**************************************************************************
	 Now's the best time to initialize FBR1 contents
     *************************************************************************/
    pFbrEntry = (PFBR_DESC_t)pAdapter->RxRing.pFbr1RingPool->getBytesNoCopy();

    for (iEntry=0; iEntry < pAdapter->RxRing.Fbr1NumEntries; iEntry++)
    {
        pFbrEntry->addr_hi = pAdapter->RxRing.Fbr1.PAHigh[ iEntry ];
        pFbrEntry->addr_lo = pAdapter->RxRing.Fbr1.PALow [ iEntry ];
        pFbrEntry->word2.bits.bi = iEntry;
        pFbrEntry++;
    }
	
	
    /**************************************************************************
	 Set the address and parameters of Free buffer ring 1 (and 0 if required) 
	 into the 1310's registers
     *************************************************************************/
	pAdrs = pAdapter->RxRing.pFbr1RingPool->getPhysicalAddress();
#ifdef __LP64__
    pRxDma->fbr1_base_hi.addr_hi = (UINT32)( pAdrs >> 32 );
#else
    pRxDma->fbr1_base_hi.addr_hi = 0;
#endif
    pRxDma->fbr1_base_lo.addr_lo = (UINT32)( pAdrs );
	
    pRxDma->fbr1_num_des.value = pAdapter->RxRing.Fbr1NumEntries - 1;
	
    {
        RXDMA_FBR_FULL_OFFSET_t fbr1_full;
		
        fbr1_full.bits.fbr_full        = 0;
        fbr1_full.bits.fbr_full_wrap   = 1;
		
        pRxDma->fbr1_full_offset = fbr1_full;
    }
	
	
    /**************************************************************************
	 This variable tracks the free buffer ring 1 full position, so it has to
	 match the above.
     *************************************************************************/
    pAdapter->RxRing.local_Fbr1_full.bits.fbr_full      = 0;
    pAdapter->RxRing.local_Fbr1_full.bits.fbr_full_wrap = 1;
	
    pRxDma->fbr1_min_des.bits.fbr_min = 
	(( pAdapter->RxRing.Fbr1NumEntries * LO_MARK_PERCENT_FOR_RX ) / 100) - 1;
	
	
    /**************************************************************************
	 Program the number of packets we will receive before generating an 
	 interrupt.
	 
	 For version B silicon, this value gets updated once autoneg is complete.
     *************************************************************************/
    pRxDma->num_pkt_done.value  = pAdapter->RegistryRxNumBuffers;
	
	
    /**************************************************************************
	 The "time_done" is not working correctly to coalesce interrupts after
	 a given time period, but rather is giving us an interrupt regardless
	 of whether we have received packets.
	 
	 This value gets updated once autoneg is complete.
     *************************************************************************/
    pRxDma->max_pkt_time.value = pAdapter->RegistryRxTimeInterval;

	IOSimpleLockUnlock( pAdapter->RcvLock );
	
}
/*===========================================================================*/

/******************************************************************************
 ROUTINE :  ConfigTxDmaRegs
 ******************************************************************************
 
 DESCRIPTION       : Used to set up the tx dma section of the JAGCore.
 
 PARAMETERS        : adapter - pointer to our private adapter structure
 
 *****************************************************************************/
void ConfigTxDmaRegs( ET131X_ADAPTER *pAdapter )
{
    PTXDMA_t pTxDma;
	IOPhysicalAddress pAdrs;
	
    /**************************************************************************
	 First lets get a copy of the pointer
     *************************************************************************/
    pTxDma = &pAdapter->CSRAddress->txdma;
	
	
    /**************************************************************************
	 Load the hardware with the start of the transmit descriptor ring.
	 *************************************************************************/
	pAdrs = pAdapter->TxRing.pTxDescRingPool->getPhysicalAddress();
#ifdef __LP64__
    pTxDma->pr_base_hi.addr_hi =  (UINT32)( pAdrs >> 32 );
#else
    pTxDma->pr_base_hi.addr_hi =  0;
#endif
    pTxDma->pr_base_lo.addr_lo =  (UINT32)( pAdrs );
	
	
    /**************************************************************************
	 Initialise the transmit DMA engine
     *************************************************************************/
    pTxDma->pr_num_des.value   = NUM_DESC_PER_RING_TX - 1;
	
	
    /**************************************************************************
	 Load the completion writeback physical address
	 
	 NOTE : pci_alloc_consistent(), used above to alloc DMA regions, ALWAYS
	 returns SAC (32-bit) addresses. If DAC (64-bit) addresses are
	 ever returned, make sure the high part is retrieved here before
	 storing the adjusted address.
     *************************************************************************/
	pAdrs = pAdapter->TxRing.pTxStatusPool->getPhysicalAddress();
#ifdef __LP64__
    pTxDma->dma_wb_base_hi.addr_hi = (UINT32)(pAdrs >> 32);
#else
    pTxDma->dma_wb_base_hi.addr_hi = 0x0;
#endif
    pTxDma->dma_wb_base_lo.addr_lo = pAdrs;
	
	bzero( pAdapter->TxRing.pTxStatusPool->getBytesNoCopy(), sizeof( TX_STATUS_BLOCK_t ));
	
	
    pTxDma->service_request.value           = 0x0;
    pAdapter->TxRing.txDmaReadyToSend.value = 0x0;
}

/******************************************************************************
 ROUTINE:  UpdateMacStatHostCounters
 *****************************************************************************/
void AgereET131x::UpdateMacStatHostCounters()
{
	
    /**************************************************************************
	 Get a local pointer to the adapter macstat regs and update stats
     *************************************************************************/
	PMAC_STAT_t pDevMacStat = &adapter.CSRAddress->macStat;

	etherStats->dot3RxExtraEntry.collisionErrors += pDevMacStat->TNcl;
	etherStats->dot3StatsEntry.singleCollisionFrames += pDevMacStat->TScl;
	etherStats->dot3StatsEntry.deferredTransmissions += pDevMacStat->TDfr;
	etherStats->dot3StatsEntry.excessiveCollisions += pDevMacStat->TMcl;
	etherStats->dot3StatsEntry.lateCollisions += pDevMacStat->TLcl;
	etherStats->dot3TxExtraEntry.underruns += pDevMacStat->TUnd;
	etherStats->dot3TxExtraEntry.resourceErrors += pDevMacStat->TOvr;
	
	etherStats->dot3StatsEntry.alignmentErrors += pDevMacStat->RAln;
	etherStats->dot3StatsEntry.missedFrames += pDevMacStat->RCde; // crc error
	etherStats->dot3StatsEntry.frameTooLongs += pDevMacStat->RDrp;
	etherStats->dot3RxExtraEntry.overruns += pDevMacStat->ROvr;
	etherStats->dot3StatsEntry.fcsErrors += pDevMacStat->RFcs;
	etherStats->dot3RxExtraEntry.frameTooShorts += pDevMacStat->RFlr;
	
	etherStats->dot3StatsEntry.internalMacReceiveErrors += pDevMacStat->RFrg;
}

/******************************************************************************
 ROUTINE :  et131x_init_send
 DESCRIPTION       : Initialize send data structures
 *****************************************************************************/

void AgereET131x::init_send()
{
    PMP_TCB         pMpTcb;
    UINT32          TcbCount;
    TX_RING_t      *tx_ring;
	
	
    /**************************************************************************
	 Setup some convenience pointers
     *************************************************************************/
    tx_ring = (TX_RING_t *)&adapter.TxRing;
    pMpTcb  = (PMP_TCB)adapter.TxRing.MpTcbMem;
	
	
    tx_ring->TCBReadyQueueHead = pMpTcb;
	
	
    /**************************************************************************
	 Go through and set up each TCB
     *************************************************************************/
    for( TcbCount = 0; TcbCount < NUM_TCB; TcbCount++ )
    {
        memset( pMpTcb, 0, sizeof( MP_TCB ));
		
		
        /**********************************************************************
		 Set the link pointer in HW TCB to the next TCB in the chain.  If 
		 this is the last TCB in the chain, also set the tail pointer.
         *********************************************************************/
        if( TcbCount < NUM_TCB - 1 )
        { 
            pMpTcb->Next = pMpTcb + 1;
        }
        else
        {
            tx_ring->TCBReadyQueueTail = pMpTcb;
            pMpTcb->Next               = (PMP_TCB)NULL;
        }
		
        pMpTcb++; 
    }
	
    /**************************************************************************
	 Curr send queue should now be empty
     *************************************************************************/
    tx_ring->CurrSendHead = (PMP_TCB)NULL;
    tx_ring->CurrSendTail = (PMP_TCB)NULL;
	
}

void AgereET131x::adapter_memory_free()
{
	et131x_tx_dma_memory_free( &adapter );
	et131x_rx_dma_memory_free( &adapter );
}


/******************************************************************************
 ROUTINE :  et131x_adapter_memory_alloc
 ******************************************************************************
 
 DESCRIPTION       : Allocate all the memory blocks for send, receive and
 others.
  
 *****************************************************************************/
int AgereET131x::adapter_memory_alloc()
{
    int status    = 0;
	
    do
    {
        /**********************************************************************
		 Allocate memory for the Tx Ring
         *********************************************************************/
        status = et131x_tx_dma_memory_alloc( &adapter );
        if( status != 0 )
        {
            IOLog("et131x_tx_dma_memory_alloc FAILED\n" );
            break;
        }
		
		
        /**********************************************************************
		 Receive buffer memory allocation
         *********************************************************************/
        status = et131x_rx_dma_memory_alloc( &adapter );
        if( status != 0 )
        {
            IOLog( "et131x_rx_dma_memory_alloc FAILED\n" );
            et131x_tx_dma_memory_free( &adapter );
            break;
        }
		
		
        /**********************************************************************
		 Init receive data structures
         *********************************************************************/
        status = et131x_init_recv( &adapter );
        if( status != 0 )
        {
            IOLog(  "et131x_init_recv FAILED\n" );
            break;
        }
    } while( 0 );

	if(status ){	// error
		adapter_memory_free();
	}
    return status;
}

/******************************************************************************
 ROUTINE :  et131x_adapter_setup
 ******************************************************************************
 
 DESCRIPTION       : Used to set the adapter up as per cassini+ documentation
 
 *****************************************************************************/
int AgereET131x::adapter_setup()
{
    int   status = 0;

    /**************************************************************************
	 Configure the JAGCore
     *************************************************************************/
    ConfigGlobalRegs( &adapter );
	
    ConfigMACRegs1( &adapter );
    ConfigMMCRegs( &adapter );
	
    ConfigRxMacRegs( &adapter );
    ConfigTxMacRegs( &adapter );
	
    /**************************************************************************
	 Halt RXDMA to perform the reconfigure. 
     *************************************************************************/
	rx_dma_disable();
	
    ConfigRxDmaRegs( &adapter );
    ConfigTxDmaRegs( &adapter );
	
    ConfigMacStatRegs( &adapter );
	
	
    /**************************************************************************
	 Move the following code to Timer function??
     *************************************************************************/
    status = xcvr_find();
	
    if( status != 0 )
    {
        DBG_WARNING( "Could not find the xcvr\n" );
    }
	
	
    /**********************************************************************
	 Prepare the TRUEPHY library.
     *********************************************************************/
    PhyInit();
	
	
    /**************************************************************************
	 Reset the phy now so changes take place
     *************************************************************************/
    PhyReset();
    
	
    /**************************************************************************
	 Power down PHY
     *************************************************************************/
    PhyPowerDown();
	
	
    setphy_normal();
	
	return status;
}


/******************************************************************************
 ROUTINE:  NICReturnRFD
 ******************************************************************************
 DESCRIPTION:
 Recycle a RFD and put it back onto the receive list 
 
 *****************************************************************************/
static void nic_return_rfd( ET131X_ADAPTER *pAdapter, PMP_RFD pMpRfd )
{
    UINT16        ReturnedBI = pMpRfd->iBufferIndex;
    UINT8         ReturnedRI = pMpRfd->iRingIndex;

    /**************************************************************************
	 We don't use any of the OOB data besides status
	 Otherwise, we need to clean up OOB data
     *************************************************************************/
	if(
	   (( ReturnedRI == 1 ) && ( ReturnedBI < pAdapter->RxRing.Fbr1NumEntries )))
    {
		IOSimpleLockLock( pAdapter->FbrLock  );
		
        if( ReturnedRI == 1 )
        {
            PFBR_DESC_t pNextDesc = (PFBR_DESC_t)pAdapter->RxRing.pFbr1RingPool->getBytesNoCopy() + pAdapter->RxRing.local_Fbr1_full.bits.fbr_full;
			
            /******************************************************************
			 Handle the Free Buffer Ring advancement here.  Write the 
			 PA / Buffer Index for the returned buffer into the oldest
			 (next to be freed)FBR entry
             *****************************************************************/
            pNextDesc->addr_hi = pAdapter->RxRing.Fbr1.PAHigh[ReturnedBI];
            pNextDesc->addr_lo = pAdapter->RxRing.Fbr1.PALow[ReturnedBI];
            pNextDesc->word2.value = ReturnedBI;
			
            if( ++pAdapter->RxRing.local_Fbr1_full.bits.fbr_full >
			   ( pAdapter->RxRing.Fbr1NumEntries - 1 ))
            {
                pAdapter->RxRing.local_Fbr1_full.bits.fbr_full = 0;
                pAdapter->RxRing.local_Fbr1_full.bits.fbr_full_wrap ^= 1;
            }
			
            pAdapter->CSRAddress->rxdma.fbr1_full_offset =
			pAdapter->RxRing.local_Fbr1_full;
        }

		IOSimpleLockUnlock( pAdapter->FbrLock  );
    }
    else
    {
        DBG_ERROR( "NICReturnRFD illegal Buffer Index returned\n" );
    }
	
	
    /**************************************************************************
	 The processing on this RFD is done, so put it back on the tail of
	 our list
     *************************************************************************/
    IOSimpleLockLock( pAdapter->RcvLock );
	
    list_add_tail( &pMpRfd->list_node, &pAdapter->RxRing.RecvList );
    pAdapter->RxRing.nReadyRecv++;
	IOSimpleLockUnlock( pAdapter->RcvLock );
	
    DBG_ASSERT( pAdapter->RxRing.nReadyRecv <= pAdapter->RxRing.NumRfd );
}

void AgereET131x::reset_recv()
{
}

bool AgereET131x::initPCIConfigSpace()
{
	int                result = 0;
	
	UInt16	reg16;
	
	reg16 = pciDevice->configRead16( kIOPCIConfigCommand );
	
	reg16 |= ( kIOPCICommandBusMaster  |
			  kIOPCICommandMemorySpace |
			  kIOPCICommandMemWrInvalidate );
	reg16 &= ~kIOPCICommandIOSpace;  // disable I/O space
	pciDevice->configWrite16( kIOPCIConfigCommand, reg16 );
	
	
    /**************************************************************************
	 Parse configuration parameters into the private adapter struct
     *************************************************************************/
    et131x_config_parse( &adapter );
	
	
    /**************************************************************************
	 Find the physical adapter
     *************************************************************************/
	et131x_find_adapter( &adapter, pciDevice );
	
	
	bcopy( adapter.PermanentAddress, adapter.CurrentAddress, ETH_ALEN );
	
	
    /**************************************************************************
	 If Phy COMA mode was enabled when we went down, disable it here.
     *************************************************************************/
    {
        PM_CSR_t     GlobalPmCSR = {0};
		
        GlobalPmCSR.bits.pm_sysclk_gate     = 1;
        GlobalPmCSR.bits.pm_txclk_gate      = 1;
        GlobalPmCSR.bits.pm_rxclk_gate      = 1;
		
        adapter.CSRAddress->global.pm_csr = GlobalPmCSR;
    }
	
	
    /**************************************************************************
	 Issue a global reset to the et1310
     *************************************************************************/
    soft_reset();
	
	
    /**************************************************************************
	 Disable all interrupts (paranoid)
     *************************************************************************/
    disable_interrupts();
	
	
    /**************************************************************************
	 Allocate DMA memory
     *************************************************************************/
    result = adapter_memory_alloc();
    if( result != 0 )
    {
		IOLog("adapter_memory_alloc() failed.\n");
        return false;
    }
	
	
    /**************************************************************************
	 Init send data structures
     *************************************************************************/
    init_send();
	
	adapter.PoMgmt.PowerState = NdisDeviceStateD0;
	
	
    /**************************************************************************
	 Register the interrupt
	 
	 NOTE - This is being done in the open routine, where most other Linux
	 drivers setup IRQ handlers. Make sure device interrupts are not
	 turned on before the IRQ is registered!!!!
	 
	 What we will do here is setup the task structure for the ISR's
	 deferred handler
     *************************************************************************/
	
	
	
    /**************************************************************************
	 Setup et1310 as per the documentation
     *************************************************************************/
    adapter_setup();
	
#if	0
    /**************************************************************************
	 Create a timer to count errors received by the NIC
	 Initialize link state
     *************************************************************************/
    et131x_link_detection_handler( (unsigned long)adapter );
#endif
	
	
    /**************************************************************************
	 Intialize variable for counting how long we do not have link status
     *************************************************************************/
	adapter.PoMgmt.TransPhyComaModeOnBoot = 0;
	
	
    /**************************************************************************
	 We can enable interrupts now
	 
	 NOTE - Because registration of interrupt handler is done in the device's
	 open(), defer enabling device interrupts to that point 
     *************************************************************************/
	
    return true;   
}




/******************************************************************************
 ROUTINE:  et131x_free_send_packet 
 ******************************************************************************
 DESCRIPTION:
 Recycle a MP_TCB and complete the packet if necessary
 
 Assumption: Send spinlock has been acquired 
 
 *****************************************************************************/
void AgereET131x::free_send_packet( PMP_TCB pMpTcb )
{
    if( pMpTcb->Packet ) {
        freePacket( pMpTcb->Packet );
		netStats->outputPackets += 1;
    }
	
    bzero( pMpTcb, sizeof( MP_TCB ));
	
	
    /**************************************************************************
	 Add the TCB to the Ready Q
     *************************************************************************/
	IOSimpleLockLock(adapter.TCBReadyQLock);
	
	
    if( adapter.TxRing.TCBReadyQueueTail )
    {
        adapter.TxRing.TCBReadyQueueTail->Next = pMpTcb;
    }
    else
    {
        /**********************************************************************
		 Apparently ready Q is empty.
         *********************************************************************/
        adapter.TxRing.TCBReadyQueueHead = pMpTcb;
    }
	
    adapter.TxRing.TCBReadyQueueTail = pMpTcb;
	
	IOSimpleLockUnlock(adapter.TCBReadyQLock);
	
}




/******************************************************************************
 ROUTINE:  nic_send_packet
 ******************************************************************************
 DESCRIPTION:
 NIC specific send handler. This version of the send routine is designed
 for version B silicon.
 
 PARAMETERS :
 pMpTcb    - pointer to MP_TCB
 
 *****************************************************************************/

int AgereET131x::nic_send_packet( PMP_TCB pMpTcb )
{
    UINT32                  loopIndex;
    TX_DESC_ENTRY_t         CurDesc[TBDS_PER_TCB];
	IOPhysicalSegment		tx_segments[TBDS_PER_TCB];
    UINT32                  iThisCopy, iRemainder;
    UINT32                  FragListCount;

	FragListCount = txMbufCursor->getPhysicalSegmentsWithCoalesce(pMpTcb->Packet, &tx_segments[0], TBDS_PER_TCB);
	
    if( FragListCount == 0 ){
		//IOLog("%s::%s() failed in getPhysicalSeg(%p), len = %d\n",getName(),__FUNCTION__, pMpTcb->Packet, mbuf_len(pMpTcb->Packet));
        return -EIO;
	}

    bzero( CurDesc, sizeof( TX_DESC_ENTRY_t ) * FragListCount );

    for( loopIndex = 0; loopIndex < FragListCount; loopIndex++ )
    {
		/******************************************************************
		 NOTE - Here, the dma_addr_t returned from pci_map_page() is
		 implicitly cast as a UINT32. Although dma_addr_t can be 64-bit,
		 the address returned by pci_map_page() is always 32-bit 
		 addressable (as defined by the pci/dma subsystem)
		 ******************************************************************/
#ifdef	__LP64__
		CurDesc[loopIndex].DataBufferPtrHigh = (UInt32)(tx_segments[loopIndex].location >> 32);
#else
		CurDesc[loopIndex].DataBufferPtrHigh = 0;
#endif
		CurDesc[loopIndex].DataBufferPtrLow = (UInt32)tx_segments[loopIndex].location;
		CurDesc[loopIndex].word2.bits.length_in_bytes = tx_segments[loopIndex].length;
    }
	
    if( adapter.uiLinkSpeed == TRUEPHY_SPEED_1000MBPS ){
        if( ++adapter.TxRing.TxPacketsSinceLastinterrupt == 
		   adapter.RegistryTxNumBuffers )
        {
            CurDesc[FragListCount - 1].word3.value = 0x5;
            adapter.TxRing.TxPacketsSinceLastinterrupt = 0;
        }
        else
        {
            CurDesc[FragListCount - 1].word3.value = 0x1;
        }
    }
    else
    {
        CurDesc[FragListCount - 1].word3.value = 0x5;
    }
	
   CurDesc[0].word3.bits.f = 1;
	
#if	USE_TX_CSUM
	// Checksum
	UInt32 demand;
	getChecksumDemand(pMpTcb->Packet, kChecksumFamilyTCPIP, &demand);
	if(demand & kChecksumIP)
		CurDesc[0].word3.bits.ipa = 1;
	if(demand & kChecksumTCP)
		CurDesc[0].word3.bits.tcpa = 1;
	if(demand & kChecksumUDP)
		CurDesc[0].word3.bits.udpa = 1;
#endif
    pMpTcb->WrIndexStart = adapter.TxRing.txDmaReadyToSend;
	
    pMpTcb->PacketStaleCount = 0;

	IOSimpleLockLock(adapter.SendHWLock );
	
    iThisCopy = NUM_DESC_PER_RING_TX - adapter.TxRing.txDmaReadyToSend.bits.serv_req;
	
    if( iThisCopy >= FragListCount )
    {
        iRemainder = 0;
        iThisCopy = FragListCount;
    }
    else
    {
        iRemainder = FragListCount - iThisCopy;
    }
	
    memcpy( (PTX_DESC_ENTRY_t)adapter.TxRing.pTxDescRingPool->getBytesNoCopy() + adapter.TxRing.txDmaReadyToSend.bits.serv_req,
		   CurDesc,
		   sizeof( TX_DESC_ENTRY_t ) * iThisCopy );
	
	
    adapter.TxRing.txDmaReadyToSend.bits.serv_req += iThisCopy;
	
    if(( adapter.TxRing.txDmaReadyToSend.bits.serv_req == 0 ) ||
       ( adapter.TxRing.txDmaReadyToSend.bits.serv_req ==  NUM_DESC_PER_RING_TX))
    {
        if( adapter.TxRing.txDmaReadyToSend.bits.serv_req_wrap )
        {
            adapter.TxRing.txDmaReadyToSend.value = 0;
        }
        else
        {
            adapter.TxRing.txDmaReadyToSend.value = 0x400;
        }
    }
	
    if( iRemainder )
    {
        memcpy( adapter.TxRing.pTxDescRingPool->getBytesNoCopy(),
			   CurDesc + iThisCopy,
			   sizeof( TX_DESC_ENTRY_t ) * iRemainder );
		
        adapter.TxRing.txDmaReadyToSend.bits.serv_req += iRemainder;
    }
	
    if( adapter.TxRing.txDmaReadyToSend.bits.serv_req == 0 )
    {
        if( adapter.TxRing.txDmaReadyToSend.value )
        {
            pMpTcb->WrIndex.value = NUM_DESC_PER_RING_TX - 1;
        }   
        else
        {
            pMpTcb->WrIndex.value = 0x400 | ( NUM_DESC_PER_RING_TX - 1 );
        }
    }
    else 
    {
        pMpTcb->WrIndex.value = adapter.TxRing.txDmaReadyToSend.value - 1;
    }

	IOSimpleLockLock( adapter.TCBSendQLock );
	
    if( adapter.TxRing.CurrSendTail )
    {
        adapter.TxRing.CurrSendTail->Next = pMpTcb;
    }
    else
    {
        adapter.TxRing.CurrSendHead = pMpTcb;
    }
	
    adapter.TxRing.CurrSendTail = pMpTcb;
	
    DBG_ASSERT( pMpTcb->Next == NULL );
	
    adapter.TxRing.nBusySend++;

	IOSimpleLockUnlock( adapter.TCBSendQLock );
	
    /**************************************************************************
	 Write the new write pointer back to the device.
     *************************************************************************/
    adapter.CSRAddress->txdma.service_request.value = adapter.TxRing.txDmaReadyToSend.value;
	
	
    /**************************************************************************
	 For Gig only, we use Tx Interrupt coalescing.  Enable the software
	 timer to wake us up if this packet isn't followed by N more.
     *************************************************************************/
    if( adapter.uiLinkSpeed == TRUEPHY_SPEED_1000MBPS )
    {
        adapter.CSRAddress->global.watchdog_timer   = adapter.RegistryTxTimeInterval * NANO_IN_A_MICRO;
    }
	
	IOSimpleLockUnlock( adapter.SendHWLock );

    return 0;
}



/******************************************************************************
 ROUTINE:  et131x_free_busy_send_packets
 ******************************************************************************
 DESCRIPTION:
 Free and complete the stopped active sends
 
 Assumption: Send spinlock has been acquired 
 
 *****************************************************************************/
void AgereET131x::free_busy_send_packets()
{
	PMP_TCB pMpTcb;
    UINT32            FreeCounter = 0;
	
    /**************************************************************************
	 Any packets being sent? Check the first TCB on the send list
     *************************************************************************/
	IOSimpleLockLock( adapter.TCBSendQLock );
	
    pMpTcb = adapter.TxRing.CurrSendHead;
	
    while(( pMpTcb != NULL ) && ( FreeCounter < NUM_TCB ))
    {
        PMP_TCB pNext = pMpTcb->Next;
		
        adapter.TxRing.CurrSendHead = pNext;
		
        if( pNext == NULL )
        {
            adapter.TxRing.CurrSendTail = NULL;
        }
		
        adapter.TxRing.nBusySend--;
		
        IOSimpleLockUnlock(adapter.TCBSendQLock );
		
		
        FreeCounter++;
        free_send_packet( pMpTcb );
		
		IOSimpleLockLock( adapter.TCBSendQLock );

        pMpTcb = adapter.TxRing.CurrSendHead;
    }
	
    if( FreeCounter == NUM_TCB )
    {
        DBG_ERROR( "MpFreeBusySendPackets exitted loop for a bad reason\n" );
        BUG();
    }
	
	IOSimpleLockUnlock(adapter.TCBSendQLock );
	
    adapter.TxRing.nBusySend = 0;
	
}


/******************************************************************************
 ROUTINE:  et131x_update_tcb_list
 ******************************************************************************
 DESCRIPTION:
 Helper routine for Send Interrupt handler.  Re-claims the send
 resources and completes sends.  Can also be called as part of the NIC
 send routine when the "ServiceComplete" indication has wrapped.
 
 *****************************************************************************/
void AgereET131x::update_tcb_list()
{
    TXDMA_SERVICE_COMPLETE_t    ServiceComplete = adapter.CSRAddress->txdma.NewServiceComplete;
    PMP_TCB                     pMpTcb;
	
	
	/**************************************************************************
	 Has the ring wrapped?  Process any descriptors that do not have
	 the same "wrap" indicator as the current completion indicator
	 *************************************************************************/
    IOSimpleLockLock( adapter.TCBSendQLock );
	
    pMpTcb = adapter.TxRing.CurrSendHead;
	
    if( ServiceComplete.bits.serv_cpl_wrap ) {
        while(  pMpTcb && 
			  !pMpTcb->WrIndex.bits.serv_req_wrap &&
              ( pMpTcb->WrIndex.bits.serv_req > ServiceComplete.bits.serv_cpl ))
	    {
            PMP_TCB pNext = pMpTcb->Next;
			
            adapter.TxRing.CurrSendHead = pNext;
			
            if( pNext == NULL )
            {
                adapter.TxRing.CurrSendTail = NULL;
            }
			
            adapter.TxRing.nBusySend--;
			
			IOSimpleLockUnlock( adapter.TCBSendQLock );
            free_send_packet(  pMpTcb );
			IOSimpleLockLock( adapter.TCBSendQLock );
			
			
		    /******************************************************************
			 Goto the next packet
			 *****************************************************************/
		    pMpTcb = adapter.TxRing.CurrSendHead;
	    }
    }
    else
    {
        while( pMpTcb &&
			  pMpTcb->WrIndex.bits.serv_req_wrap &&
			  ( pMpTcb->WrIndex.bits.serv_req > ServiceComplete.bits.serv_cpl ))
	    {
            PMP_TCB pNext = pMpTcb->Next;
			
            adapter.TxRing.CurrSendHead = pNext;
			
            if( pNext == NULL )
            {
                adapter.TxRing.CurrSendTail = NULL;
            }
			
            adapter.TxRing.nBusySend--;
			
			IOSimpleLockUnlock( adapter.TCBSendQLock );
		    free_send_packet( pMpTcb );
			IOSimpleLockLock( adapter.TCBSendQLock );
			
			
		    /******************************************************************
			 Goto the next packet
			 *****************************************************************/
		    pMpTcb = adapter.TxRing.CurrSendHead;
	    }
    }
	
    while( pMpTcb  &&
		  ( ServiceComplete.bits.serv_cpl_wrap == pMpTcb->WrIndex.bits.serv_req_wrap ) &&
		  ( ServiceComplete.bits.serv_cpl > pMpTcb->WrIndex.bits.serv_req ))
    {
        PMP_TCB pNext = pMpTcb->Next;
		
        adapter.TxRing.CurrSendHead = pNext;
		
        if( pNext == NULL )
        {
            adapter.TxRing.CurrSendTail = NULL;
        }
		
        adapter.TxRing.nBusySend--;
		
		IOSimpleLockUnlock( adapter.TCBSendQLock );
        free_send_packet( pMpTcb );
		IOSimpleLockLock( adapter.TCBSendQLock );

		
		/**********************************************************************
		 Goto the next packet
         *********************************************************************/
        pMpTcb = adapter.TxRing.CurrSendHead;
    }
	
	
    /* Wake up the queue when we hit a low-water mark */
    if( adapter.TxRing.nBusySend <= ( NUM_TCB / 3 ))
    {
		//  netif_wake_queue( adapter.netdev );
    }
	
    IOSimpleLockUnlock( adapter.TCBSendQLock );
}

void AgereET131x::check_send_wait_list()
{
}

/******************************************************************************
 ROUTINE:  nic_rx_pkts
 ******************************************************************************
 DESCRIPTION:
 Checks the hardware for available packets, using completion ring
 If packets are available, it gets an RFD from the RecvList, attaches
 the packet to it, puts the RFD in the RecvPendList, and also returns
 the pointer to the RFD.
 *****************************************************************************/
PMP_RFD AgereET131x::nic_rx_pkts( )
{
    PRX_STATUS_BLOCK_t      pRxStatusBlock;
    PPKT_STAT_DESC_t        pPSREntry;
    PMP_RFD                 pMpRfd;
    UINT32                  nIndex;
    PUCHAR                  pBufVa;
    struct list_head       *element;
    /*-----------------------------------------------------------------------*/
	
	
    /**************************************************************************
	 RX Status block is written by the DMA engine prior to every interrupt.
	 It contains the next to be used entry in the Packet Status Ring, and
	 also the two Free Buffer rings.
     *************************************************************************/
    pRxStatusBlock = (PRX_STATUS_BLOCK_t)adapter.RxRing.pRxStatusPool->getBytesNoCopy();
	
    if(( pRxStatusBlock->Word1.bits.PSRoffset != 
		adapter.RxRing.local_psr_full.bits.psr_full ) ||
       ( pRxStatusBlock->Word1.bits.PSRwrap != 
		adapter.RxRing.local_psr_full.bits.psr_full_wrap ))
    {
        UINT8                   ringIndex;
        UINT16                  bufferIndex;
        UINT32                  localLen;
        PKT_STAT_DESC_WORD0_t   Word0;
		
        /**********************************************************************
		 The packet status ring indicates that data is available. 
         *********************************************************************/
        pPSREntry =  (PPKT_STAT_DESC_t)adapter.RxRing.pPSRingPool->getBytesNoCopy() + adapter.RxRing.local_psr_full.bits.psr_full;
		
		
        /**********************************************************************
		 Grab any information that is required once the PSR is advanced,
		 since we can no longer rely on the memory being accurate
         *********************************************************************/
        localLen    = pPSREntry->word1.bits.length;
        ringIndex   = (UINT8)pPSREntry->word1.bits.ri;
        bufferIndex = (UINT16)pPSREntry->word1.bits.bi;
        Word0       = pPSREntry->word0;
		

        /**********************************************************************
		 Check the Status Word that the MAC has appended to the PSR entry in
		 case the MAC has detected errors.
         *********************************************************************/
        if( Word0.value & ALCATEL_BAD_STATUS )
        {
#if	0
            IOLog("NICRxPkts >> Alcatel Status Word error. Value 0x%08x\n", pPSREntry->word0.value );
#endif
        }
		
        /**********************************************************************
		 Indicate that we have used this PSR entry.
         *********************************************************************/
        if( ++adapter.RxRing.local_psr_full.bits.psr_full > ( adapter.RxRing.PsrNumEntries - 1 ))
        {
            adapter.RxRing.local_psr_full.bits.psr_full       = 0;
            adapter.RxRing.local_psr_full.bits.psr_full_wrap ^= 1;
        }
		
        adapter.CSRAddress->rxdma.psr_full_offset = adapter.RxRing.local_psr_full;
		
		
			if(( ringIndex != 1 ) || ( bufferIndex > ( adapter.RxRing.Fbr1NumEntries - 1 )))
			{
				/******************************************************************
				 Illegal buffer or ring index cannot be used by the S/W
				 *****************************************************************/
				IOLog( "NICRxPkts PSR Entry %d indicates length of %d and/or bad bi(%d)\n",
						  adapter.RxRing.local_psr_full.bits.psr_full,
						  localLen,
						  bufferIndex );
				
				return NULL;
			}
		
		
        /**********************************************************************
		 Get and fill the RFD.
         *********************************************************************/
		IOSimpleLockLock(adapter.RcvLock);
		
        element = adapter.RxRing.RecvList.next;
        pMpRfd  = (PMP_RFD)list_entry( element, MP_RFD, list_node );
		
        if( pMpRfd == NULL )
        {
            return NULL;
        }
		
        list_del( &pMpRfd->list_node );
        adapter.RxRing.nReadyRecv--;
		
		IOSimpleLockUnlock(adapter.RcvLock);
		
        pMpRfd->iBufferIndex = bufferIndex;
        pMpRfd->iRingIndex   = ringIndex;
		
		
        /**********************************************************************
		 In V1 silicon, there is a bug which screws up filtering of runt
		 packets.  Therefore runt packet filtering is disabled in the MAC
		 and the packets are dropped here.  They are also counted here.
         *********************************************************************/
        if( localLen < ( NIC_MIN_PACKET_SIZE + 4 ))
        {
            etherStats->dot3StatsEntry.etherChipSet++;
            localLen = 0;
        }
		
		
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
        if( localLen ){
			PUINT16                 pShBufVa;
            pShBufVa = adapter.RxRing.Fbr.Va[bufferIndex];
			
            pMpRfd->bHasVLANTag = FALSE;
			
			
            /******************************************************************
			 The protocol value of 0x8100 means there is a VLAN tag in the
			 packet.  Also the original protocol value will be present four
			 bytes further on.
             *****************************************************************/
            if( pShBufVa[6] == 0x0081 ){
                UINT16 LocalShort = (( pShBufVa[7] & 0xff00 ) >> 8 ) +
				(( pShBufVa[7] & 0x00ff ) << 8 );
                UINT16 vlan_tag   = LocalShort & 0x0fff;
				
                pMpRfd->bHasVLANTag = TRUE;
				
				
                /**************************************************************
				 The rules are:
				 - if our VLAN tag is zero we can pass anything up
				 - if our VLAN tag matches the incoming we can pass it
				 - If the packet is a protocol 802.3ad we pass it 
				 regardless (802.3ad had protocol val of 0x8809.  
				 proto val is now in ShBuf [8])
				 - If the packet is a GARP VLAN Registration Protocol
				 (GVRP) packet, we pass it regardless.  
				 01:80:c2:00:00:21 is the GVRP address.
				 
				 NOTE: Because the ET1310 doesn't perform VLAN tagging
				 (it's done in the kernel) always pass packets up.
				 We'll leave this code in, however, just in case it's
				 needed in the future.
                 *************************************************************/
                if(( 1 ) ||
                   (  pShBufVa[8] == 0x0988 ) ||
                   (( pShBufVa[0] == 0x8001 ) &&
                    ( pShBufVa[1] == 0x00c2 ) && 
                    ( pShBufVa[2] == 0x2100 )))
                {
                    pMpRfd->VLANTag = vlan_tag;
                }
                else
                {
                    /*********************************************************
					 Our VLAN tag is non-zero, AND the incoming tag does
					 not match it.  Drop the packet.
                     ********************************************************/
                    localLen = 0;
                }
            }
            else if((  pShBufVa[6] == 0x0988 ) ||
                    (( pShBufVa[0] == 0x8001 ) && 
                     ( pShBufVa[1] == 0x00c2 ) && 
                     ( pShBufVa[2] == 0x2100 )))
            {
                /******************************************************************
				 The protocol type (ethertype) of 0x8809 corresponds to 802.3ad 
				 The MAC address of 01:80:c2:00:00:21 is the GARP VLAN 
				 registration protocol (GVRP) address.
				 
				 Both of these message types should be passed up regardless
				 of their VLAN tagging.
				 *****************************************************************/
            }
            else
            {
                /******************************************************************
				 Our VLAN tag is non-zero.  no VLAN header on incoming.
				 Packet is not GVRP or 802.3ad.  Drop the packet.
				 *****************************************************************/
                // NOTE: Same as the note above; never drop a packet for now.
                // localLen = 0;
            }
        }
#endif
		
        if( localLen ){
            if ( adapter.ReplicaPhyLoopbk == 1 ){
                pBufVa = (u8*)adapter.RxRing.Fbr1.Va[bufferIndex];
                
                if( memcmp( &pBufVa[6], &adapter.CurrentAddress[0], ETH_ALEN ) == 0 ){
                    if( memcmp( &pBufVa[42], "Replica packet", ETH_HLEN )){
                        adapter.ReplicaPhyLoopbkPF = 1;
                    }
                }
            }
			
            /******************************************************************
			 Determine if this is a multicast packet coming in
             *****************************************************************/
            if(( Word0.value & ALCATEL_MULTICAST_PKT ) &&
               !( Word0.value & ALCATEL_BROADCAST_PKT ))
            {
                /**************************************************************
				 Promiscuous mode and Multicast mode are not mutually
				 exclusive as was first thought.  I guess Promiscuous is
				 just considered a super-set of the other filters.
				 Generally filter is 0x2b when in promiscuous mode.
                 *************************************************************/
                if(( adapter.PacketFilter & ET131X_PACKET_TYPE_MULTICAST ) &&
                   !( adapter.PacketFilter & ET131X_PACKET_TYPE_PROMISCUOUS ) &&
                   !( adapter.PacketFilter & ET131X_PACKET_TYPE_ALL_MULTICAST ))
                {
                    pBufVa = (u8*)adapter.RxRing.Fbr1.Va[bufferIndex];
					
                    /**********************************************************
					 Loop through our list to see if the destination address
					 of this packet matches one in our list.
                     *********************************************************/
                    for( nIndex = 0; nIndex < adapter.MCAddressCount; nIndex++ ){
                        if( pBufVa[0] == adapter.MCList[nIndex][0] &&
						   pBufVa[1] == adapter.MCList[nIndex][1] &&
						   pBufVa[2] == adapter.MCList[nIndex][2] &&
						   pBufVa[3] == adapter.MCList[nIndex][3] &&
						   pBufVa[4] == adapter.MCList[nIndex][4] &&
						   pBufVa[5] == adapter.MCList[nIndex][5] )
                        {
                            break;
                        }
                    }
                    
                    /**********************************************************
					 If our index is equal to the number of Multicast
					 address we have, then this means we did not find this
					 packet's matching address in our list.  Set the
					 PacketSize to zero, so we free our RFD when we return
					 from this function.
                     *********************************************************/
                    if( nIndex == adapter.MCAddressCount )
                    {
                        localLen = 0;
                    }
                }
				
                if( localLen > 0 )
                {
                    adapter.Stats.multircv++;
                }
            }
            else if( Word0.value & ALCATEL_BROADCAST_PKT )
            {
                adapter.Stats.brdcstrcv++;
            }
            else
            {
                /**************************************************************
				 Not sure what this counter measures in promiscuous mode.
				 Perhaps we should check the MAC address to see if it is 
				 directed to us in promiscuous mode.
                 *************************************************************/
                adapter.Stats.unircv++;
            }
        }
		
        if( localLen > 0 )
        {
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
            unsigned short  vlan_tag = 0;
#endif

            pMpRfd->PacketSize = localLen;
			
            mbuf_t skb = allocatePacket( pMpRfd->PacketSize );
            if( !skb ){
                return NULL;
            }
			
			
			mbuf_copyback(skb, 0, pMpRfd->PacketSize, adapter.RxRing.Fbr1.Va[bufferIndex], MBUF_WAITOK);
			setChecksumResult(skb, kChecksumFamilyTCPIP, kChecksumIP | kChecksumTCP | kChecksumUDP, kChecksumIP | kChecksumTCP | kChecksumUDP);
			
            netif->inputPacket( skb, pMpRfd->PacketSize );
        }
        else
        {
            pMpRfd->PacketSize   = 0;
        }
		
        nic_return_rfd( &adapter, pMpRfd );
		
        return pMpRfd;
    }
	/**********************************************************************
	 Looks like this ring is not updated yet
	 *********************************************************************/
	return NULL;
}
/*===========================================================================*/


/******************************************************************************
 ROUTINE:  et131x_handle_recv_interrupt
 ******************************************************************************
 DESCRIPTION:
 Interrupt handler for receive processing
 
 Assumption: Rcv spinlock has been acquired 
 
 
 *****************************************************************************/
void AgereET131x::handle_recv_interrupt(  )
{
    PMP_RFD         pMpRfd = NULL;

    UINT32          PacketArrayCount = 0;
    BOOL_t          TempUnfinishedRec = FALSE;
	
	
    /**************************************************************************
	 Process up to available RFD's
     *************************************************************************/
    while( PacketArrayCount < NUM_PACKETS_HANDLED ){

        if( list_empty( &adapter.RxRing.RecvList )){
            DBG_ASSERT( adapter.RxRing.nReadyRecv == 0 );
            IOLog("NO RFD's !!!!!!!!!!!!!\n" );
            TempUnfinishedRec = TRUE;
            break;
        }
		
        pMpRfd = nic_rx_pkts();
		
        if( pMpRfd == NULL)
            break;
		
        /**********************************************************************
		 Do not receive any packets until a filter has been set.
		 Do not receive any packets until we are at D0.
		 Do not receive any packets until we have link.
		 If length is zero, return the RFD in order to advance the Free
		 buffer ring.
         *********************************************************************/
        if(( !adapter.PacketFilter ) || 
//           ( adapter.PoMgmt.PowerState != NdisDeviceStateD0 ) ||
           ( !MP_LINK_DETECTED( &adapter )) ||
           ( pMpRfd->PacketSize == 0 ))
        {
            continue;
        }
		
		
        /**********************************************************************
		 Increment the number of packets we received
         *********************************************************************/
        netStats->inputPackets++;

        PacketArrayCount++;
    }
	
	
    if(( PacketArrayCount == NUM_PACKETS_HANDLED ) || TempUnfinishedRec )
    {
        adapter.RxRing.UnfinishedReceives = TRUE;
        adapter.CSRAddress->global.watchdog_timer = 
		adapter.RegistryTxTimeInterval * NANO_IN_A_MICRO;
    }
    else
    {
        /**********************************************************************
		 Watchdog timer will disable itself if appropriate.
         *********************************************************************/
        adapter.RxRing.UnfinishedReceives = FALSE;
    }
	
}
/*===========================================================================*/


void AgereET131x::soft_reset(  )
{
	
    /**************************************************************************
	 Disable MAC Core
     *************************************************************************/
    adapter.CSRAddress->mac.cfg1.value = 0xc00f0000;
	
	
    /**************************************************************************
	 Set everything to a reset value
     *************************************************************************/
    adapter.CSRAddress->global.sw_reset.value = 0x7F;
    adapter.CSRAddress->mac.cfg1.value        = 0x000f0000;
    adapter.CSRAddress->mac.cfg1.value        = 0x00000000;
	
}


void AgereET131x::disable_interrupts()
{
    /**************************************************************************
	 Disable all global interrupts
     *************************************************************************/
    //adapter.CachedMaskValue.value             = INT_MASK_DISABLE;
    adapter.CSRAddress->global.int_mask.value = INT_MASK_DISABLE;
}

void AgereET131x::enable_interrupts( )
{
    UINT32 MaskValue;
	
	
    /**************************************************************************
	 Enable all global interrupts
     *************************************************************************/
    if(( adapter.FlowControl == TxOnly ) ||
       ( adapter.FlowControl == Both ))
    {
        MaskValue = INT_MASK_ENABLE;
    }
    else
    {
        MaskValue = INT_MASK_ENABLE_NO_FLOW;
    }
#if	0
    if( adapter.DriverNoPhyAccess )
    {
        MaskValue |= 0x10000;
    }
#endif
    //adapter.CachedMaskValue.value = MaskValue;
    adapter.CSRAddress->global.int_mask.value = MaskValue;
}





void AgereET131x::timeoutOccurred(IOTimerEventSource * src)
{
    PM_CSR_t        pm_csr;
    /*-----------------------------------------------------------------------*/
	
    pm_csr = adapter.CSRAddress->global.pm_csr;
	
    if( pm_csr.bits.pm_phy_sw_coma == 0 )
    {
        if( adapter.RegistryMACStat )
        {
            UpdateMacStatHostCounters();
        }
    }
    else
    {
        DBG_VERBOSE( "No interrupts, in PHY coma, pm_csr = 0x%x\n",
					pm_csr.value );
    }
	
    if( !adapter.Bmsr.bits.link_status && 
	   adapter.RegistryPhyComa && 
	   adapter.PoMgmt.TransPhyComaModeOnBoot < 11 )
    {
        adapter.PoMgmt.TransPhyComaModeOnBoot++;
    }
	
#if	1
    if( adapter.PoMgmt.TransPhyComaModeOnBoot == 10 )
    {
        if( !adapter.Bmsr.bits.link_status && adapter.RegistryPhyComa )
        {
            if( adapter.CSRAddress->global.pm_csr.bits.phy_sw_coma == 0 )
            {
                // NOTE - This was originally a 'sync with interrupt'. How
                //        to do that under Linux?
                enable_interrupts();
                EnablePhyComa();
            }
        }
    }
#endif
    /**************************************************************************
	 This is a periodic timer, so reschedule
     *************************************************************************/
	watchdogSource->setTimeoutMS(1000);
	
}


int AgereET131x::send_packet( mbuf_t skb  )
{
    int             status          = 0;
    PMP_TCB         pMpTcb;

    /**************************************************************************
	 Is this packet VLAN tagged? If so, is it a tag previously registered?
	 If not, drop the packet
	 
	 NOTE - We need not worry about the above note for now, as VLAN is handled
	 by the linux kernel (for the most part).
     *************************************************************************/
	
	
    /**************************************************************************
	 Get a TCB for this packet
     *************************************************************************/
	IOSimpleLockLock(adapter.TCBReadyQLock);

	pMpTcb = adapter.TxRing.TCBReadyQueueHead;
	if( pMpTcb == NULL ){
		status = ENOMEM;
	} else {
		adapter.TxRing.TCBReadyQueueHead = pMpTcb->Next;
		if( adapter.TxRing.TCBReadyQueueHead == NULL )
			adapter.TxRing.TCBReadyQueueTail = NULL;
	}

	IOSimpleLockUnlock(adapter.TCBReadyQLock); 

	if( status == 0 ){
		pMpTcb->Packet          = skb;
		pMpTcb->Next = NULL;

		status = nic_send_packet( pMpTcb );
		
		if( status != 0 ) {
			IOSimpleLockLock(adapter.TCBReadyQLock); 
			
			if( adapter.TxRing.TCBReadyQueueTail )
				adapter.TxRing.TCBReadyQueueTail->Next = pMpTcb;
			else
				//Apparently ready Q is empty.
				adapter.TxRing.TCBReadyQueueHead = pMpTcb;
			
			adapter.TxRing.TCBReadyQueueTail = pMpTcb;
			
			IOSimpleLockUnlock(adapter.TCBReadyQLock); 
		}
	}
	
	DBG_ASSERT( adapter.TxRing.nBusySend <= NUM_TCB );
    return status;
}
/*===========================================================================*/

bool AgereET131x::addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
{
	IONetworkMedium *medium;
	
	medium = IONetworkMedium::medium(type, bps, 0, index);
	if (!medium) {
		return false;
	}
	
	if (!IONetworkMedium::addMedium(mediumDict, medium)) {
		return false;
	}
	
	mediumTable[index] = medium;
	return true;
}

/******************************************************************************
 first part of et131x_isr
 ******************************************************************************/
bool AgereET131x::filterCheck(IOInterruptEventSource * src)
{
    INT_STATUS_t           status;

	if(! enabledForNetif )
		return false;

	/**********************************************************************
	 Get a copy of the value in the interrupt status register so we can 
	 process the interrupting section
	 *********************************************************************/
	status.value  =  adapter.CSRAddress->global.int_status.value;

	if(( adapter.FlowControl == TxOnly ) ||
	   ( adapter.FlowControl == Both ))
		status.value &= ~INT_MASK_ENABLE;
	else
		status.value &= ~INT_MASK_ENABLE_NO_FLOW;

	if( !status.value )
		return false;

	
	/**********************************************************************
	 This is our interrupt, so process accordingly
	 *********************************************************************/
	if( status.bits.watchdog_interrupt )
	{
		PMP_TCB pMpTcb = adapter.TxRing.CurrSendHead;
		
		if( pMpTcb )
		{
			if( ++pMpTcb->PacketStaleCount > 1 )
			{
				status.bits.txdma_isr = 1;
			}
		}
		
		if( adapter.RxRing.UnfinishedReceives )
		{
			status.bits.rxdma_xfr_done = 1;
		}
		else if( pMpTcb == 0 )
		{
			adapter.CSRAddress->global.watchdog_timer = 0;
		}
		
		status.bits.watchdog_interrupt = 0;
	}
	
	
	/**********************************************************************
	 We need to save the interrupt status value for use in our DPC. We
	 will clear the software copy of that in that routine.
	 *********************************************************************/
	adapter.Stats.InterruptStatus = status;
	
	return true;
}


/******************************************************************************
 ROUTINE :  et131x_isr_handler
 ******************************************************************************
 
 DESCRIPTION       : The ISR handler, scheduled to run in a deferred context
 by the ISR. This is where the ISR's work actually gets
 done.
 
 *****************************************************************************/

void AgereET131x::interruptOccurred(IOInterruptEventSource * src)
{
    INT_STATUS_t GlobStatus = adapter.Stats.InterruptStatus;

    /**************************************************************************
	 These first two are by far the most common.  Once handled, we clear
	 their two bits in the status word.  If the word is now zero, we exit.
     *************************************************************************/
    /**************************************************************************
	 Handle all the completed Transmit interrupts
     *************************************************************************/
    if( GlobStatus.bits.txdma_isr )
    {
		update_tcb_list();
		check_send_wait_list();
    }
	
    /**************************************************************************
	 Handle all the completed Receives interrupts
     *************************************************************************/
    if( GlobStatus.bits.rxdma_xfr_done )  
    {
        handle_recv_interrupt( );
    }
	
    GlobStatus.value &= 0xffffffd7;
	
    if( GlobStatus.value )
    {
		
        /**********************************************************************
		 Handle the TXDMA Error interrupt
         *********************************************************************/
        if( GlobStatus.bits.txdma_err )
        {
            TXDMA_ERROR_t   TxDmaErr;
			
			
            /******************************************************************
			 Following read also clears the register (COR register)
             *****************************************************************/
            TxDmaErr.value = adapter.CSRAddress->txdma.TxDmaError.value;
			
            DBG_WARNING( "TXDMA_ERR interrupt, error = %d\n",
						TxDmaErr.value );
        }
		
		
        /**********************************************************************
		 Handle Free Buffer Ring 0 and 1 Low interrupt
         *********************************************************************/
        if( GlobStatus.bits.rxdma_fb_ring0_low || 
		   GlobStatus.bits.rxdma_fb_ring1_low )
        {
            /******************************************************************
			 This indicates the number of unused buffers in RXDMA free buffer 
			 ring 0 is <= the limit you programmed. Free buffer resources
			 need to be returned.  Free buffers are consumed as packets are
			 passed from the network to the host. The host becomes aware of
			 the packets from the contents of the packet status ring. This
			 ring is queried when the packet done interrupt occurs. Packets
			 are then passed to the OS. When the OS is done with the packets
			 the resources can be returned to the ET1310 for re-use. This 
			 interrupt is one method of returning resources.
             *****************************************************************/
            DBG_WARNING( "RXDMA_FB_RING0_LOW or "
						"RXDMA_FB_RING1_LOW interrupt\n" );
			
			
            /******************************************************************
			 If the user has flow control on, then we will send a pause
			 packet, otherwise just exit
             *****************************************************************/
            if(( adapter.FlowControl == TxOnly ) ||
               ( adapter.FlowControl == Both ))
            {
                /**************************************************************
				 Tell the device to send a pause packet via the back pressure
				 register
                 *************************************************************/
                if( adapter.CSRAddress->global.pm_csr.bits.pm_phy_sw_coma == 0 )
                {
                    TXMAC_BP_CTRL_t     bp_ctrl;
					
                    bp_ctrl.bits.bp_req     = 1;
                    bp_ctrl.bits.bp_xonxoff = 1;
					
                    adapter.CSRAddress->txmac.bp_ctrl.value = bp_ctrl.value;
                }
            }
        }
		
		
        /**********************************************************************
		 Handle Packet Status Ring Low Interrupt
         *********************************************************************/
        if( GlobStatus.bits.rxdma_pkt_stat_ring_low )
        {
            DBG_WARNING( "RXDMA_PKT_STAT_RING_LOW interrupt\n" );
			
			
            /******************************************************************
			 Same idea as with the two Free Buffer Rings. Packets going from
			 the network to the host each consume a free buffer resource and 
			 a packet status resource.  These resoures are passed to the OS.
			 When the OS is done with the resources, they need to be returned 
			 to the ET1310. This is one method of returning the resources.
             *****************************************************************/
        }
		
		
        /**********************************************************************
		 Handle RXDMA Error Interrupt
         *********************************************************************/
        if( GlobStatus.bits.rxdma_err )
        {
            /******************************************************************
			 The rxdma_error interrupt is sent when a time-out on a request 
			 issued by the JAGCore has occurred or a completion is returned 
			 with an un-successful status.  In both cases the request is 
			 considered complete. The JAGCore will automatically re-try the
			 request in question. Normally information on events like these
			 are sent to the host using the "Advanced Error Reporting" 
			 capability. This interrupt is another way of getting similar
			 information. The only thing required is to clear the interrupt
			 by reading the ISR in the global resources. The JAGCore will do
			 a re-try on the request.  Normally you should never see this
			 interrupt. If you start to see this interrupt occurring
			 frequently then something bad has occurred. 
			 A reset might be the thing to do.
             *****************************************************************/
            // TRAP();
			
            adapter.TxMacTest = adapter.CSRAddress->txmac.tx_test;
            DBG_WARNING( "RxDMA_ERR interrupt, error %x\n",
						adapter.TxMacTest.value );
        }
		
		
        /**********************************************************************
		 Handle the Wake on LAN Event
         *********************************************************************/
        if( GlobStatus.bits.wake_on_lan ){
            /******************************************************************
			 This is a secondary interrupt for wake on LAN.  The driver  
			 should never see this, if it does, something serious is wrong.
			 We will TRAP the message when we are in DBG mode, otherwise we
			 will ignore it.
             *****************************************************************/
        }
		
		
        /**********************************************************************
		 Handle the PHY interrupt
         *********************************************************************/
        if( GlobStatus.bits.phy_interrupt ) 
        {
			HandlePhyInterrupt();
        }

		
        /**********************************************************************
		 Let's move on to the TxMac
         *********************************************************************/
        if( GlobStatus.bits.txmac_interrupt ) 
        {
			UINT32 value = adapter.CSRAddress->txmac.err.value;
			
            /******************************************************************
			 When any of the errors occur and TXMAC generates an interrupt to 
			 report these errors, it usually means that TXMAC has detected an 
			 error in the data stream retrieved from the on-chip Tx Q. All of
			 these errors are catastrophic and TXMAC wonít be able to recover 
			 data when these errors occur.  In a nutshell, the whole Tx path
			 will have to be reset and re-configured afterwards.  
             *****************************************************************/
            IOLog( "TXMAC interrupt, error 0x%08x\n", value );
			
			
            /*******************************************************************
			 If we are debugging, we want to see this error, otherwise we just
			 want the device to be reset and continue
             *****************************************************************/
            //DBG_TRAP();
            
        }
		
        /**********************************************************************
		 Handle RXMAC Interrupt
         *********************************************************************/
        if( GlobStatus.bits.rxmac_interrupt )
        {
            /******************************************************************
			 These interrupts are catastrophic to the device, what we need to 
			 do is disable the interrupts and set the flag to cause us to 
			 reset so we can solve this issue.
             ******************************************************************/
            // MP_SET_FLAG( pAdapter, fMP_ADAPTER_HARDWARE_ERROR );
			
            DBG_WARNING( "RXMAC interrupt, error 0x%08x.  Requesting reset\n",
						adapter.CSRAddress->rxmac.err_reg.value );
			
            DBG_WARNING( "Enable 0x%08x, Diag 0x%p\n",
						adapter.CSRAddress->rxmac.ctrl.value, 
						&adapter.CSRAddress->rxmac.rxq_diag );
			
			
            /*******************************************************************
			 If we are debugging, we want to see this error, otherwise we just
			 want the device to be reset and continue
             *****************************************************************/
            // TRAP();
        }
		
        /**********************************************************************
		 Handle MAC_STAT Interrupt
         *********************************************************************/
        if( GlobStatus.bits.mac_stat_interrupt )
        {
            /******************************************************************
			 This means at least one of the un-masked counters in the MAC_STAT 
			 block has rolled over.  Use this to maintain the top, software
			 managed bits of the counter(s).
             *****************************************************************/
            DBG_VERBOSE( "MAC_STAT interrupt\n" );
            HandleMacStatInterrupt();
        }
		
		
        /**********************************************************************
		 Handle SLV Timeout Interrupt
         *********************************************************************/
        if( GlobStatus.bits.slv_timeout )
        {
            /******************************************************************
			 This means a timeout has occured on a read or write request to 
			 one of the JAGCore registers. The Global Resources block has
			 terminated the request and on a read request, returned a "fake"
			 value.  The most likely reasons are: Bad Address or the 
			 addressed module is in a power-down state and can't respond.
             *****************************************************************/
            DBG_VERBOSE( "SLV_TIMEOUT interrupt\n" );
        }
    }
	
#if	0
    if( adapter.PoMgmt.PowerState == NdisDeviceStateD0 )
    {
        enable_interrupts();
    }
#endif
}


void AgereET131x::tx_dma_enable()
{
    TXDMA_CSR_t csr = {0};
    /*-----------------------------------------------------------------------*/
	
    if( adapter.RegistryPhyLoopbk )
    {
        /**********************************************************************
		 TxDMA is disabled for loopback operation.
         *********************************************************************/
        adapter.CSRAddress->txdma.csr.value = 0x101;
		
    }
    else
    {
        /**********************************************************************
		 Setup the transmit dma configuration register for normal operation
         *********************************************************************/
        csr.bits.sngl_epkt_mode = 1;
        csr.bits.halt           = 0;
        csr.bits.cache_thrshld  = adapter.RegistryDMACache;
		
        adapter.CSRAddress->txdma.csr.value = csr.value;
    }
}

void AgereET131x::rx_dma_enable()
{
    if( adapter.RegistryPhyLoopbk )
    {
        /**********************************************************************
		 RxDMA is disabled for loopback operation.
         *********************************************************************/
        adapter.CSRAddress->rxdma.csr.value = 0x1;
    }
    else
    {
        /**********************************************************************
		 Setup the receive dma configuration register for normal operation.
         *********************************************************************/
        {
            RXDMA_CSR_t csr = {0};
			
            csr.bits.fbr1_enable = 1;
			
            if( adapter.RxRing.Fbr1BufferSize == 4096 )
            {
                csr.bits.fbr1_size = 1;
            }
            else if( adapter.RxRing.Fbr1BufferSize == 8192 )
            {
                csr.bits.fbr1_size = 2;
            }
            else if( adapter.RxRing.Fbr1BufferSize == 16384 )
            {
                csr.bits.fbr1_size = 3;
            }
            adapter.CSRAddress->rxdma.csr = csr;
        }
		
        if( adapter.CSRAddress->rxdma.csr.bits.halt_status != 0 )
        {
            udelay( 5 );
            if( adapter.CSRAddress->rxdma.csr.bits.halt_status != 0 )
            {
                DBG_ERROR( "RX Dma failed to exit halt state.  CSR 0x%08x\n",
						  adapter.CSRAddress->rxdma.csr.value );
            }
        }
    }
}

/**************************************************************************
 Setup the receive dma configuration register
 *************************************************************************/
void AgereET131x::rx_dma_disable()
{
    adapter.CSRAddress->rxdma.csr.value = 0x00002001;
	
    if( adapter.CSRAddress->rxdma.csr.bits.halt_status != 1 )
    {
        udelay( 5 );
        if( adapter.CSRAddress->rxdma.csr.bits.halt_status != 1 )
        {
            DBG_ERROR( "RX Dma failed to enter halt state. CSR 0x%08x\n",
					  adapter.CSRAddress->rxdma.csr.value );
        }
    }
}

/**************************************************************************
 Setup the tramsmit dma configuration register
 *************************************************************************/
void AgereET131x::tx_dma_disable()
{
	adapter.CSRAddress->txdma.csr.value = 0x101;
}

void AgereET131x::HandleMacStatInterrupt()
{
    MAC_STAT_REG_1_t Carry1;
    MAC_STAT_REG_2_t Carry2;
    /*-----------------------------------------------------------------------*/
	
    /**************************************************************************
	 Read the interrupt bits from the register(s).  These are Clear On Write.
     *************************************************************************/
    Carry1 = adapter.CSRAddress->macStat.Carry1;
    Carry2 = adapter.CSRAddress->macStat.Carry2;
	
    adapter.CSRAddress->macStat.Carry1 = Carry1;
    adapter.CSRAddress->macStat.Carry2 = Carry2;
	

    if( Carry1.bits.rfcs )
		etherStats->dot3StatsEntry.fcsErrors += 1;

    if( Carry1.bits.raln )
		etherStats->dot3StatsEntry.alignmentErrors += 1;

    if( Carry1.bits.rflr )
		etherStats->dot3RxExtraEntry.frameTooShorts += 1;

	if( Carry1.bits.rfrg )
		etherStats->dot3StatsEntry.internalMacTransmitErrors += 1;
		
    if( Carry1.bits.rcde )
		etherStats->dot3StatsEntry.sqeTestErrors += 1;
	
    if( Carry1.bits.rovr )
        etherStats->dot3RxExtraEntry.overruns += 1;
	
    if( Carry1.bits.rdrp )
        etherStats->dot3RxExtraEntry.resourceErrors += 1;
	
    if( Carry2.bits.tovr )
        etherStats->dot3TxExtraEntry.resourceErrors += 1;
	
    if( Carry2.bits.tund )
        etherStats->dot3TxExtraEntry.underruns += 1;
	
    if( Carry2.bits.tscl )
        etherStats->dot3StatsEntry.singleCollisionFrames += 1;
	
    if( Carry2.bits.tdfr )
        etherStats->dot3StatsEntry.deferredTransmissions += 1;
	
    if( Carry2.bits.tmcl )
        etherStats->dot3StatsEntry.excessiveCollisions += 1;
	
    if( Carry2.bits.tlcl )
        etherStats->dot3StatsEntry.lateCollisions += 1;


    if( Carry2.bits.tncl )
        etherStats->dot3StatsEntry.multipleCollisionFrames += 1;

}


void AgereET131x::set_mac_addr( const u8* addr )
{

#if	0
    /**************************************************************************
	 Stop the Tx and Rx DMA engines
     *************************************************************************/
    rx_dma_disable();
	tx_dma_disable();
	
	
    /**************************************************************************
	 Disable device interrupts
     *************************************************************************/
    disable_interrupts();
    et131x_handle_send_interrupt( adapter );
    et131x_handle_recv_interrupt( adapter );
	
	
    /**************************************************************************
	 Free Rx DMA memory
     *************************************************************************/
	adapter_memory_free();
	
#endif
    /**************************************************************************
	 Set the config parameter for Jumbo Packet support
     *************************************************************************/
    //adapter->RegistryJumboPacket = new_mtu + 14;
    // blux: not needet here, w'll change the MAC
	
    soft_reset();
	
	
    /**************************************************************************
	 Alloc and init Rx DMA memory
     *************************************************************************/
#if	0
    result = adapter_memory_alloc();
    if( result != 0 )
    {
        DBG_WARNING( "Change MAC failed; couldn't re-alloc DMA memory\n" );
        return;
    }
#endif
    init_send( );
	
	
	bcopy( addr, adapter.CurrentAddress, ETH_ALEN );
	// memcpy( netdev->dev_addr, adapter->CurrentAddress, ETH_ALEN );
	// blux: no, do not override our nice address
	
#if	0

    /**************************************************************************
	 Init the device with the new settings
     *************************************************************************/
    adapter_setup();
	
	
    /**************************************************************************
	 Enable interrupts
     *************************************************************************/
    if( MP_TEST_FLAG( &adapter, fMP_ADAPTER_INTERRUPT_IN_USE ))
    {
        enable_interrupts(  );
    }
	
	
    /**************************************************************************
	 Restart the Tx and Rx DMA engines
     *************************************************************************/
    rx_dma_enable();
    tx_dma_enable();
    
    
    /**************************************************************************
	 Restart the netif queue
     *************************************************************************/
#endif
}

/******************************************************************************
 ROUTINE :  et131x_set_packet_filter
 ******************************************************************************
 
 DESCRIPTION       : Configures the Rx Packet filtering on the device
 
 *****************************************************************************/
void AgereET131x::set_packet_filter()
{
    UINT32          filter  = adapter.PacketFilter;
    RXMAC_CTRL_t    ctrl    = adapter.CSRAddress->rxmac.ctrl;
    RXMAC_PF_CTRL_t pf_ctrl = adapter.CSRAddress->rxmac.pf_ctrl;
    /*-----------------------------------------------------------------------*/
    /**************************************************************************
	 Default to disabled packet filtering.  Enable it in the individual case
	 statements that require the device to filter something
     *************************************************************************/
    ctrl.bits.pkt_filter_disable = 1;
	
	
    /**************************************************************************
	 Set us to be in promiscuous mode so we receive everything, 
	 this is also true when we get a packet filter of 0
     *************************************************************************/
    if(( filter & ET131X_PACKET_TYPE_PROMISCUOUS ) || filter == 0 )
    {
        pf_ctrl.bits.filter_broad_en = 0;
        pf_ctrl.bits.filter_multi_en = 0;
        pf_ctrl.bits.filter_uni_en   = 0;
    }
    else 
    {
        /**********************************************************************
		 Set us up with Multicast packet filtering.  Three cases are
		 possible - (1) we have a multi-cast list, (2) we receive ALL
		 multicast entries or (3) we receive none.
         *********************************************************************/
        if( filter & ET131X_PACKET_TYPE_ALL_MULTICAST )
        {
            pf_ctrl.bits.filter_multi_en = 0;
        }
        else
        {
            SetupDeviceForMulticast( &adapter );
            pf_ctrl.bits.filter_multi_en    = 1;
            ctrl.bits.pkt_filter_disable    = 0;
        }
		
		
        /**********************************************************************
		 Set us up with Unicast packet filtering
         *********************************************************************/
        if( filter & ET131X_PACKET_TYPE_DIRECTED )
        {
            SetupDeviceForUnicast( &adapter );
            pf_ctrl.bits.filter_uni_en = 1;
            ctrl.bits.pkt_filter_disable    = 0;
        }
		
		
        /**********************************************************************
		 Set us up with Broadcast packet filtering
         *********************************************************************/
        if( filter & ET131X_PACKET_TYPE_BROADCAST )
        {
            DBG_VERBOSE( "Broadcast Filtering ON\n" );
            pf_ctrl.bits.filter_broad_en = 1;
            ctrl.bits.pkt_filter_disable    = 0;
        }
        else
        {
            DBG_VERBOSE( "Broadcast Filtering OFF\n" );
            pf_ctrl.bits.filter_broad_en = 0;
        }
		
		
        /**********************************************************************
		 Setup the receive mac configuration registers - Packet Filter
		 control + the enable / disable for packet filter in the control
		 reg.
         *********************************************************************/
        adapter.CSRAddress->rxmac.pf_ctrl.value = pf_ctrl.value;
        adapter.CSRAddress->rxmac.ctrl = ctrl;
    }
}


void AgereET131x::set_mtu(UInt32 maxsize)
{
	mtu = maxsize;
	adapter.RegistryJumboPacket = maxsize + 14;
	
	RELEASE(txMbufCursor);
	txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(adapter.RegistryJumboPacket, TBDS_PER_TCB);
	
}
