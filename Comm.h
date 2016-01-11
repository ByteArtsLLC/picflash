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
*
* Author        Date        Comment
*************************************************************************
* T. Lawrence  2011/01/24  Initial code ported from AN1310.
************************************************************************/

#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include "typedefs.h"

#include "hidapi.h"
#include "Device.h"

// Device Vendor and Product IDs
#define VID 0x04d8
#define PID 0x003C

#define USB_PACKET_SIZE 64
#define USB_PACKET_SIZE_WITH_REPORT_ID (USB_PACKET_SIZE + 1)

// Packet commands
#define QUERY_DEVICE        0x02
#define UNLOCK_CONFIG       0x03
#define ERASE_DEVICE        0x04
#define PROGRAM_DEVICE      0x05
#define PROGRAM_COMPLETE    0x06
#define GET_DATA            0x07
#define RESET_DEVICE        0x08
#define SIGN_FLASH			0x09	//The host PC application should send this command after the verify operation has completed successfully.  If checksums are used instead of a true verify (due to ALLOW_GET_DATA_COMMAND being commented), then the host PC application should send SIGN_FLASH command after is has verified the checksums are as exected. The firmware will then program the SIGNATURE_WORD into flash at the SIGNATURE_ADDRESS.
#define QUERY_EXTENDED_INFO 0x0C    //Used by host PC app to get additional info about the device, beyond the basic NVM layout provided by the query device command

// Maximum number of memory regions that can be bootloaded
#define MAX_DATA_REGIONS    0x06
#define MAX_ERASE_BLOCK_SIZE 8196   //Increase this in the future if any microcontrollers with bigger than 8196 byte erase block is implemented

typedef enum
{
    ecSuccess = 0,
    ecNotConnected,
    ecFail,
    ecIncorrectCommand,
    ecTimeout,
    ecOther = 0xFF
} TCommErrorCode;


#pragma pack(1)
typedef struct
{
    unsigned char type;
    uint32_t address;
    uint32_t size;
} TMemoryRegion;
#pragma pack()

#pragma pack(1)
typedef struct
{
    unsigned char command;
    unsigned char bytesPerPacket;
    unsigned char deviceFamily;
    TMemoryRegion memoryRegions[MAX_DATA_REGIONS];
    unsigned char versionFlag;
    unsigned char pad[7];
} TBootInfo;
#pragma pack()

#pragma pack(1)
typedef struct
{
    unsigned char command;
    uint16_t bootloaderVersion;
    uint16_t applicationVersion;
    uint32_t signatureAddress;
    uint16_t signatureValue;
    uint32_t erasePageSize;
    unsigned char config1LMask;
    unsigned char config1HMask;
    unsigned char config2LMask;
    unsigned char config2HMask;
    unsigned char config3LMask;
    unsigned char config3HMask;
    unsigned char config4LMask;
    unsigned char config4HMask;
    unsigned char config5LMask;
    unsigned char config5HMask;
    unsigned char config6LMask;
    unsigned char config6HMask;
    unsigned char config7LMask;
    unsigned char config7HMask;
    unsigned char pad[USB_PACKET_SIZE_WITH_REPORT_ID - 29];
} TPIC18;
#pragma pack()

#pragma pack(1)
typedef struct
{
    unsigned char command;
    uint16_t bootloaderVersion;
    uint16_t applicationVersion;
    uint32_t signatureAddress;
    uint16_t signatureValue;
    uint32_t erasePageSize;
    //#warning Replace with real stuff when implemented.
    //uint16_t configxxMask...
    //unsigned char pad[USB_PACKET_SIZE_WITH_REPORT_ID - XX];
} TPIC24;
#pragma pack()

#pragma pack(1)
typedef struct
{
    unsigned char report;
    unsigned char command;
    union {
        uint32_t address;
        unsigned char LockedValue;
    };
    unsigned char bytesPerPacket;
    unsigned char data[58];
} TWritePacket;
#pragma pack()

#pragma pack(1)
typedef struct
{
    unsigned char command;
    uint32_t address;
    unsigned char bytesPerPacket;
    unsigned char data[59];
} TReadPacket;
#pragma pack()

#pragma pack(1)
typedef union
{
    unsigned char command;
    TPIC18 PIC18;
    TPIC24 PIC24;
} TExtendedQueryInfo;
#pragma pack()

typedef struct {
    hid_device * boot_device;
    bool connected;
    int SyncWaitTime;

    TCommErrorCode ErrorCode;

    //#pragma pack(1)
    TMemoryRegion MemoryRegion;
    TBootInfo BootInfo;

    //Structure for the response to the QUERY_EXTENDED_INFO command
    TExtendedQueryInfo ExtendedQueryInfo;

    TWritePacket WritePacket;
    TReadPacket ReadPacket;
    //#pragma pack()
} TComm;

extern void Comm_construct(TComm * PComm, unsigned SyncWait);
extern void Comm_destruct(TComm * PComm);
extern TCommErrorCode Comm_SendPacket(TComm * PComm, unsigned char * pData, int size);
extern TCommErrorCode Comm_Program(TComm * PComm, uint32_t address, unsigned char bytesPerPacket,
    unsigned char bytesPerAddress, unsigned char bytesPerWord,
    TFamilies deviceFamily,
    uint32_t endAddress, unsigned char *pData);
extern int Comm_Reset(TComm * PComm);
extern TCommErrorCode Comm_Erase(TComm * PComm);
extern TCommErrorCode Comm_GetData(TComm * PComm, uint32_t address, unsigned char bytesPerPacket,
    unsigned char bytesPerAddress, unsigned char bytesPerWord,
    uint32_t endAddress, unsigned char *pData);
extern TCommErrorCode Comm_LockUnlockConfig(TComm * PComm, bool lock);
extern void Comm_PollUSB(TComm * PComm);
extern TCommErrorCode Comm_ReadBootloaderInfo(TComm * PComm,
    TBootInfo * bootInfo);
extern TCommErrorCode Comm_ReadExtendedQueryInfo(TComm * PComm,
    TExtendedQueryInfo * extendedBootInfo);
extern TCommErrorCode Comm_ReceivePacket(TComm * PComm, unsigned char *data, int size);
extern TCommErrorCode Comm_SignFlash(TComm * PComm);
extern void Comm_close(TComm * PComm);
extern bool Comm_isConnected(TComm * PComm);
extern TCommErrorCode Comm_open(TComm * PComm);

#endif // COMM_H
