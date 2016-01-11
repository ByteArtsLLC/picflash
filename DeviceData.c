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
* E. Schlunder  2009/05/07  Added code for CRC calculation.
* E. Schlunder  2009/04/29  Code ported from PicKit2 pk2cmd source code.
*************************************************************************/

#include "DeviceData.h"
#include <string.h>
#include <stdlib.h>

void DeviceData_construct(TDeviceData * PDeviceData)
{
  DeviceData_clearranges(PDeviceData);
}

void DeviceData_destruct(TDeviceData * PDeviceData)
{

}


void DeviceData_addrange(TDeviceData * PDeviceData, TMemoryRange * newrange)
{
  PDeviceData->ranges[PDeviceData->RangeCount] = *newrange;
  ++PDeviceData->RangeCount;
}



void DeviceData_clearranges(TDeviceData * PDeviceData)
{
  unsigned int nIndex;

  for (nIndex = 0; nIndex < MAX_MEMORY_RANGES; ++nIndex)
  {
    PDeviceData->ranges[nIndex].type = 0;
    PDeviceData->ranges[nIndex].start = 0;
    PDeviceData->ranges[nIndex].end = 0;
    PDeviceData->ranges[nIndex].dataBufferLength = 0;

    if ((nIndex < PDeviceData->RangeCount)
    && PDeviceData->ranges[nIndex].pDataBuffer)
    {
      free(PDeviceData->ranges[nIndex].pDataBuffer);
    }

    PDeviceData->ranges[nIndex].pDataBuffer = 0;
  }
  PDeviceData->RangeCount = 0;
}

void MemoryRange_construct(TMemoryRange * PRange)
{
  memset(PRange, 0, sizeof(TMemoryRange));
}

