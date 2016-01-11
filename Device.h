/************************************************************************
* Copyright (c) 2009-2010,  Microchip Technology Inc.
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
*/

#ifndef DEVICE_H
#define DEVICE_H

///#include <QString>
///#include <QVariant>
///#include <QLinkedList>
///#include <QList>

#include "typedefs.h"
#include "DeviceData.h"

/*!
 * Provides microcontroller device specific parameters, address calculations, and
 * assembly code tools.
 */
typedef enum Families
{
    famUnknown = 0,
    famPIC18 = 0x01,
    famPIC24 = 0x02,
    famPIC32 = 0x03,
    famPIC16 = 0x04
} TFamilies;


typedef struct {
  TFamilies family;

  unsigned int bytesPerPacket;
  unsigned int bytesPerWordFLASH;
  unsigned int bytesPerWordEEPROM;
  unsigned int bytesPerWordConfig;

  unsigned int bytesPerAddressFLASH;
  unsigned int bytesPerAddressEEPROM;
  unsigned int bytesPerAddressConfig;

  TDeviceData *deviceData;

} TDevice;


extern void Device_construct(TDevice * PDevice, TDeviceData * data);
extern void Device_setUnknown(TDevice * PDevice);

extern bool Device_hasEeprom(TDevice * PDevice);
extern bool Device_hasConfig(TDevice * PDevice);

extern bool Device_hasConfigAsFlash(TDevice * PDevice);
extern bool Device_hasConfigAsFuses(TDevice * PDevice);

extern unsigned int Device_GetDeviceAddressFromHexAddress(TDevice * PDevice,
    unsigned int hexAddress, TDeviceData * PData,
    unsigned char * type, bool * includedInProgrammableRange,
    bool * addressWasEndofRange, unsigned int * bytesPerAddressAndType,
    unsigned int * endDeviceAddressofRegion, unsigned char ** pPCRAMBuffer);

/*

class Device
{
public:

    Device(DeviceData* data);

    void setUnknown(void);

    Families family;

    unsigned int bytesPerPacket;
    unsigned int bytesPerWordFLASH;
    unsigned int bytesPerWordEEPROM;
    unsigned int bytesPerWordConfig;

    unsigned int bytesPerAddressFLASH;
    unsigned int bytesPerAddressEEPROM;
    unsigned int bytesPerAddressConfig;

    bool hasEeprom(void);
    bool hasConfig(void);

    bool hasConfigAsFlash(void);
    bool hasConfigAsFuses(void);

    unsigned int GetDeviceAddressFromHexAddress(unsigned int hexAddress, DeviceData* pData, unsigned char& type, bool& includedInProgrammableRange, bool& addressWasEndofRange, unsigned int& bytesPerAddressAndType, unsigned int& endDeviceAddressofRegion, unsigned char*& pPCRAMBuffer);

protected:
    DeviceData *deviceData;
};
*/
#endif // DEVICE_H
