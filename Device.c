/************************************************************************
* Copyright (c) 2009-2011,  Microchip Technology Inc.
*
* Microchip licenses this software to you solely for use with Microchip
* products.  The software is owned by Microchip and its licensors, and
* is protected under applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED "AS IS."  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT
* NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL
* MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
* CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR
* EQUIPMENT, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY
* OR SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED
* TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION,
* OR OTHER SIMILAR COSTS.
*
* To the fullest extent allowed by law, Microchip and its licensors
* liability shall not exceed the amount of fees, if any, that you
* have paid directly to Microchip to use this software.
*
* MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
* OF THESE TERMS.
*
* Author        Date        Comment
*************************************************************************
* E. Schlunder  2010/02/01  Loading config bit fields/settings added.
* E. Schlunder  2009/04/14  Initial code ported from VB app.
* F. Schlunder  2011/06/13  Minor changes for USB HID Bootloader use.
************************************************************************/

#include "Device.h"


void Device_construct(TDevice * PDevice, TDeviceData * data)
{
    PDevice->deviceData = data;

    Device_setUnknown(PDevice);
}

void Device_setUnknown(TDevice * PDevice)
{
    PDevice->family = famUnknown;

    PDevice->bytesPerAddressFLASH = 1;
    PDevice->bytesPerAddressEEPROM = 1;
    PDevice->bytesPerAddressConfig = 1;

    PDevice->bytesPerWordEEPROM = 1;
    PDevice->bytesPerWordConfig = 2;
    PDevice->bytesPerWordFLASH = 2;
}


bool Device_hasEeprom(TDevice * PDevice)
{
    TMemoryRange range;
    int nRangeIndex;

    //foreach(range, deviceData->ranges)
    for (nRangeIndex = 0; nRangeIndex < PDevice->deviceData->RangeCount; ++nRangeIndex)
    {
        range = PDevice->deviceData->ranges[nRangeIndex];
        if (range.type == EEPROM_MEMORY)
            return true;
    }

    return false;
}


bool Device_hasConfig(TDevice * PDevice)
{
    TMemoryRange range;
    int nRangeIndex;

    //foreach(range, deviceData->ranges)
    for (nRangeIndex = 0; nRangeIndex < PDevice->deviceData->RangeCount; ++nRangeIndex)
    {
        range = PDevice->deviceData->ranges[nRangeIndex];
        if (range.type == CONFIG_MEMORY)
            return true;
    }

    return false;
}


/*
This method is useful for hex file parsing.  This function checks if the address
indicated in the hex file line is contained in one of the programmable regions
(ex: flash, eeprom, config words, etc.), as specified by the USB device,
(from the query response that we received earlier).  If the address from the
hex file is contained in the programmabl device address range, this function
returns bool includedInProgrammableRange = true, with the type set to the
programmable region type (which can be determined based on the address and
the query response).  This function also returns the new effective device
address (not from the .hex file, which is a raw linear byte address, but the
effective address the device should load into its self programming address
registers, in order to program the contents [PIC24 uses a 16-bit word addressed
flash array, so the hex file addresses don't match the microcontroller
addresses]).
This function also returns bool addressWasEndofRange = true, if the input
hexAddress corresponded to the very last byte of the last address of the
programmable memory region that the hexAddress corresponded to.
Otherwise this value returns false.  This provides an easy check later to
know when an end of a region has been completed during hex file parsing.
*/
unsigned int Device_GetDeviceAddressFromHexAddress(TDevice * PDevice,
    unsigned int hexAddress,
    TDeviceData * PData, unsigned char * type,
    bool * includedInProgrammableRange,
    bool * addressWasEndofRange, unsigned int * bytesPerAddressAndType,
    unsigned int * endDeviceAddressofRegion, unsigned char ** pPCRAMBuffer)
{
    TMemoryRange range;
    int nRangeIndex;
    unsigned int flashAddress = hexAddress / PDevice->bytesPerAddressFLASH;
    unsigned int eepromAddress = hexAddress / PDevice->bytesPerAddressEEPROM;
    unsigned int configAddress = hexAddress / PDevice->bytesPerAddressConfig;
    unsigned char * pRAMDataBuffer;
    unsigned int byteOffset;

    //Loop for each of the previously identified programmable regions, based on the results of the
    //previous Query device response packet.
    //foreach(range, pData->ranges)
    for (nRangeIndex = 0; nRangeIndex < PData->RangeCount; ++nRangeIndex)
    {
      range = PData->ranges[nRangeIndex];
      //Find what address range the hex address seems to contained within (if any, could be none, in
      //the case the .hex file contains info that is not part of the bootloader re-programmable region of flash).
      if ((range.type == PROGRAM_MEMORY) && (flashAddress >= range.start)
      && (flashAddress < range.end))
      {
          *includedInProgrammableRange = true;
          if (range.start != 0)
          {
              byteOffset = ((flashAddress - range.start) * PDevice->bytesPerAddressFLASH)
                  + (hexAddress % PDevice->bytesPerAddressFLASH);
              pRAMDataBuffer = range.pDataBuffer + byteOffset;
              *pPCRAMBuffer = pRAMDataBuffer;
          }
          else
          {
              *pPCRAMBuffer = 0;
          }

          *type = PROGRAM_MEMORY;
          *bytesPerAddressAndType = PDevice->bytesPerAddressFLASH;
          *endDeviceAddressofRegion = range.end;

          //Check if this was the very last byte of the very last address of the region.
          //We can determine this, using the below check.
          if ((flashAddress == (range.end - 1))
          && ((hexAddress % PDevice->bytesPerAddressFLASH) == (PDevice->bytesPerAddressFLASH - 1)))
          {
              *addressWasEndofRange = true;
          }
          else
          {
              *addressWasEndofRange = false;
          }
          return flashAddress;
      }

      if ((range.type == EEPROM_MEMORY)
      && (eepromAddress >= range.start) && (eepromAddress < range.end))
      {
          *includedInProgrammableRange = true;
          if (range.start != 0)
          {
              byteOffset = ((eepromAddress - range.start) * PDevice->bytesPerAddressEEPROM)
                  + (hexAddress % PDevice->bytesPerAddressEEPROM);
              pRAMDataBuffer = range.pDataBuffer + byteOffset;
              *pPCRAMBuffer = pRAMDataBuffer;
          }
          else
          {
              *pPCRAMBuffer = 0;
          }
          *type = EEPROM_MEMORY;
          *bytesPerAddressAndType = PDevice->bytesPerAddressEEPROM;
          *endDeviceAddressofRegion = range.end;

          //Check if this was the very last byte of the very last address of the region.
          //We can determine this, using the below check.
          if ((eepromAddress == (range.end - 1))
          && ((eepromAddress % PDevice->bytesPerAddressEEPROM) == (PDevice->bytesPerAddressEEPROM - 1)))
          {
              *addressWasEndofRange = true;
          }
          else
          {
              *addressWasEndofRange = false;
          }
          return eepromAddress;
      }

      if ((range.type == CONFIG_MEMORY)
      && (configAddress >= range.start) && (configAddress < range.end))
      {
          *includedInProgrammableRange = true;
          if (range.start != 0)
          {
              byteOffset = ((configAddress - range.start) * PDevice->bytesPerAddressConfig)
                  + (hexAddress % PDevice->bytesPerAddressConfig);
              pRAMDataBuffer = range.pDataBuffer + byteOffset;
              *pPCRAMBuffer = pRAMDataBuffer;
          }
          else
          {
              *pPCRAMBuffer = 0;
          }
          *type = CONFIG_MEMORY;
          *bytesPerAddressAndType = PDevice->bytesPerAddressConfig;
          *endDeviceAddressofRegion = range.end;

          //Check if this was the very last byte of the very last address of the region.
          //We can determine this, using the below check.
          if ((configAddress == (range.end - 1))
          && ((configAddress % PDevice->bytesPerAddressConfig) == (PDevice->bytesPerAddressConfig - 1)))
          {
              *addressWasEndofRange = true;
          }
          else
          {
              *addressWasEndofRange = false;
          }
          return configAddress;
      }
    } // for

    //If we get to here, that means the hex file address that was passed in was not included in any of the
    //device's reported programmable memory regions.
    *includedInProgrammableRange = false;
    *addressWasEndofRange = false;
    *pPCRAMBuffer = 0;
    return 0;
}
