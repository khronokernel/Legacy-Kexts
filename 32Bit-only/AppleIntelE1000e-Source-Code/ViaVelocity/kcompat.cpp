/*
 *  kcompat.cpp
 *  ViaVelocity
 *
 *
 */

#include "osdep.h"
#include "kcompat.h"

#include <IOKit/pci/IOPCIDevice.h>

extern "C" void pci_read_config_byte(void* dev, U8 offset, U8* pData)
{
	*pData = ((IOPCIDevice*)dev)->configRead8(offset);
}

extern "C" void pci_read_config_dword(void* dev, U8 offset, U32* pData)
{
	*pData = ((IOPCIDevice*)dev)->configRead32(offset);
}

extern "C" void pci_write_config_byte(void* dev, U8 offset, U8 data)
{
	((IOPCIDevice*)dev)->configWrite8(offset,data);
}

extern "C" void pci_write_config_dword(void* dev, U8 offset, U32 data)
{
	((IOPCIDevice*)dev)->configWrite32(offset,data);
}


