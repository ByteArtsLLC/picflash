/************************************************************************
* Copyright (c) 2010-2011,  Microchip Technology Inc.
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
************************************************************************/

#include <stdio.h>
#include <string.h>
#include "Comm.h"
#include "Logger.h"
#include "Delays.h"


void Comm_construct(TComm * PComm, unsigned SyncWait)
{
  PComm->SyncWaitTime = SyncWait;
  PComm->connected = false;
  PComm->boot_device = NULL;
}


void Comm_destruct(TComm * PComm)
{
  PComm->boot_device = NULL;
}


/**
 *
 */
void Comm_PollUSB(TComm * PComm)
{
    struct hid_device_info * dev;

    dev = hid_enumerate(VID, PID);

    PComm->connected = (dev != NULL);
    hid_free_enumeration(dev);
}

/**
 *
 */
bool Comm_isConnected(TComm * PComm)
{
    return PComm->connected;
}

/**
 *
 */
TCommErrorCode Comm_open(TComm * PComm)
{
    PComm->boot_device = hid_open(VID, PID, NULL);
    if (PComm->boot_device)
    {
        PComm->connected = true;
        hid_set_nonblocking(PComm->boot_device, true);
        LogMesg(levDEBUG_HI, "Device open.");
        return ecSuccess;
    }

    LogMesg(levERR, "Unable to open device.");
    return ecNotConnected;
}

/**
 *
 */
void Comm_close(TComm * PComm)
{
    if (PComm->boot_device)
    {
      hid_close(PComm->boot_device);
    }
    
    PComm->boot_device = NULL;
    PComm->connected = false;
}

/**
 *
 */
int Comm_Reset(TComm * PComm)
{
    unsigned char sendPacket[65];
    TCommErrorCode status;

    if (PComm->connected)
    {
        memset((void*)&sendPacket, 0x00, sizeof(sendPacket));
        sendPacket[1] = RESET_DEVICE;

        LogMesg(levINFO, "Sending Reset Command...");

        status = Comm_SendPacket(PComm, sendPacket, sizeof(sendPacket));

        if (status == ecSuccess)
        {
            LogMesg(levINFO, "Successfully sent reset command");
            return 0;
        }
        else
        {
            LogMesg(levERR, "Sending reset command failed.");
            return 10;
        }
    }
    else
      return 11;
}

/**
 *
 */
TCommErrorCode Comm_Program(TComm * PComm, uint32_t address, unsigned char bytesPerPacket,
    unsigned char bytesPerAddress, unsigned char bytesPerWord,
    TFamilies deviceFamily,
    uint32_t endAddress, unsigned char *pData)
{
    TWritePacket writePacket;
    TCommErrorCode ecResult = ecSuccess;
    uint32_t i;
    bool allPayloadBytesFF;
    bool firstAllFFPacketFound = false;
    uint32_t bytesToSend;
    unsigned char lastCommandSent = PROGRAM_DEVICE;
    uint32_t addressesToProgram;
    uint32_t startOfDataPayloadIndex;

    //Error check input parameters before using them
    if ((pData == NULL) || (bytesPerAddress == 0) || (address > endAddress) || (bytesPerWord == 0))
    {
        LogMesg(levERR, "Bad parameters specified when calling Program() function.");
        return ecFail;
    }

    //Error check to make sure the requested maximum data payload size is an exact multiple of the bytesPerAddress.
    //If not, shrink the number of bytes we actually send, so that it is always an exact multiple of the
    //programmable media bytesPerAddress.  This ensures that we don't "half" program any memory address (ex: if each
    //flash address is a 16-bit word address, we don't want to only program one byte of the address, we want to program
    //both bytes.
    while((bytesPerPacket % bytesPerWord) != 0)
    {
        bytesPerPacket--;
    }

    //Setup variable, used for progress bar updating computations.
    addressesToProgram = endAddress - address;
    if(addressesToProgram == 0) //Check to avoid potential divide by zero later.
        addressesToProgram++;

    // Make sure the device is still connected before we start trying to communicate with it.
    if (!PComm->connected)
      return ecNotConnected;

    // Loop through the entire data set/region, but break it into individual packets before sending it
    // to the device.
    while (address < endAddress)
    {
        //Prepare the packet to send to the device.
        memset((void*)&writePacket, 0x00, sizeof(writePacket)); //initialize all bytes clear, so unused pad bytes are = 0x00.
        writePacket.command = PROGRAM_DEVICE;
        writePacket.address = address;

        //Check if we are near the end of the programmable region, and need to send a "short packet" (with less than the maximum
        //allowed program data payload bytes).  In this case we need to notify the device by using the PROGRAM_COMPLETE command instead
        //of the normal PROGRAM_DEVICE command.  This lets the bootloader firmware in the device know it should flush any internal
        //buffers it may be using, by programming all of the bufferred data to NVM memory.
        if (((endAddress - address) * bytesPerAddress) < bytesPerPacket)
        {
            writePacket.bytesPerPacket = (endAddress - address) * bytesPerAddress;
            //Copy the packet data to the actual outgoing buffer and then send it over USB to the device.
            memcpy((unsigned char*)&writePacket.data[0] + 58 - writePacket.bytesPerPacket, pData, writePacket.bytesPerPacket);

            //Check to make sure we are completely programming all bytes of the destination address.  If not,
            //increase the data size and set the extra byte(s) to 0xFF (the default/blank value).
            while((writePacket.bytesPerPacket % bytesPerWord) != 0)
            {
                if (writePacket.bytesPerPacket >= bytesPerPacket)
                {
                    break; //should never hit this break, due to while((bytesPerPacket % bytesPerWord) != 0) check at start of function
                }

                //Shift all the data payload bytes in the packet to the left one (lower address),
                //so we can insert a new 0xFF byte.
                for(i = 0; i < (unsigned char)(bytesPerPacket - 1); i++)
                {
                    writePacket.data[i] = writePacket.data[i+1];
                }
                writePacket.data[writePacket.bytesPerPacket] = 0xFF;
                writePacket.bytesPerPacket++;

            }
            bytesToSend = writePacket.bytesPerPacket;
            LogMesg(levDEBUG_LO, "Preparing short packet of final program data with payload"); ///: 0x%x", (uint32_t)writePacket.bytesPerPacket);
        }
        else
        {
            //Else we are planning on sending a full length packet with the full size payload.
            writePacket.bytesPerPacket = bytesPerPacket;
            bytesToSend = bytesPerPacket;
            //Copy the packet data to the actual outgoing buffer and then prepare to send it.
            memcpy((unsigned char*)&writePacket.data[0] + 58 - writePacket.bytesPerPacket, pData, writePacket.bytesPerPacket);
        }


        //Check if all bytes of the data payload section of the packet are == 0xFF.  If so, we can save programming
        //time by skipping the packet by not sending it to the device.  The default/erased value is already = 0xFF, so
        //the contents of the flash memory will be correct (although if we want to be certain we should make sure
        //the 0xFF regions are still getting checked during the verify step, in case the erase procedure failed to set
        //all bytes = 0xFF).
        allPayloadBytesFF = true;   //assume true until we do the actual check below
        //Loop for all of the bytes in the data payload portion of the writePacket.  The data payload is little endian but is stored
        //"right justified" in the packet.  Therefore, writePacket.data[0] isn't necessarily the LSB data byte in the packet.
        startOfDataPayloadIndex = 58 - writePacket.bytesPerPacket;
        for (i = startOfDataPayloadIndex; i < (startOfDataPayloadIndex + writePacket.bytesPerPacket); i++)
        {
            if (writePacket.data[i] != 0xFF)
            {
                //Special check for PIC24, where every 4th byte from the .hex file is == 0x00,
                //which is the "phantom byte" (the upper byte of each odd address 16-bit word
                //is unimplemented, and is probably 0x00 in the .hex file).
                if((((i - startOfDataPayloadIndex) % bytesPerWord) == 3) && (deviceFamily == famPIC24))
                {
                    //We can ignore this "phantom byte", since it is unimplemented and effectively a "don't care" byte.
                }
                else
                {
                    //We found a non 0xFF (or blank value) byte.  We need to send and program the
                    //packet of useful data into the device.
                    allPayloadBytesFF = false;
                    break;
                }
            }
        }

        //Check if we need to send a normal packet of data to the device, if the packet was all 0xFF and
        //we need to send a PROGRAM_COMPLETE packet, or if it was all 0xFF and we can simply skip it without
        //doing anything.
        if (allPayloadBytesFF == false)
        {
            char sMesg[80];
            sprintf(sMesg, "Sending packet with address 0x%X", (uint32_t)writePacket.address);
            LogMesg(levDEBUG_LO, sMesg);

            //We need to send a normal PROGRAM_DEVICE packet worth of data to program.
            ecResult = Comm_SendPacket(PComm, (unsigned char*)&writePacket, sizeof(writePacket));

            //Verify the data was successfully received by the USB device.
            if (ecResult != ecSuccess)
            {
              sprintf(sMesg, "Error during program sending packet with address 0x%X", (uint32_t)writePacket.address);
              LogMesg(levERR, sMesg);
              return ecResult;
            }
            firstAllFFPacketFound = true; //reset flag so it will be true the next time a pure 0xFF packet is found
            lastCommandSent = PROGRAM_DEVICE;

        }
        else if ((allPayloadBytesFF == true) && (firstAllFFPacketFound == true))
        {
            //In this case we need to send a PROGRAM_COMPLETE command to let the firmware know it should flush
            //its buffer by programming all of it to flash, since we are about to skip to a new address range.
            writePacket.command = PROGRAM_COMPLETE;
            writePacket.bytesPerPacket = 0;
            firstAllFFPacketFound = false;
            LogMesg(levDEBUG_HI, "Sending program complete data packet to skip a packet with address"); ///: 0x%x", (uint32_t)writePacket.address);
            ecResult = Comm_SendPacket(PComm, (unsigned char*)&writePacket, sizeof(writePacket));
            //Verify the data was successfully received by the USB device.
            if (ecResult != ecSuccess)
            {
              char sError[80];
              sprintf(sError, "Error during program sending packet with address 0x%X", (uint32_t)writePacket.address);
              LogMesg(levERR, sError);
              return ecResult;
            }
            lastCommandSent = PROGRAM_COMPLETE;
        }
        else
        {
            //If we get to here, this means that (allPayloadBytesFF == true) && (firstAllFFPacketFound == false).
            //In this case, the last packet that we processed was all 0xFF, and all bytes of this packet are
            //also all 0xFF.  In this case, we don't need to send any packet data to the device.  All we need
            //to do is advance our pointers and keep checking for a new non-0xFF section.
            LogMesg(levDEBUG_LO, "Skipping data packet with all 0xFF with address"); ///: 0x%x", (uint32_t)writePacket.address);
        }

        //Increment pointers now that we successfully programmed (or deliberately skipped) a packet worth of data
        address += bytesPerPacket / bytesPerAddress;
        pData += bytesToSend;

        //Check if we just now exactly finished programming the memory region (in which case address will be exactly == endAddress)
        //region. (ex: we sent a PROGRAM_DEVICE instead of PROGRAM_COMPLETE for the last packet sent).
        //In this case, we still need to send the PROGRAM_COMPLETE command to let the firmware know that it is done,
        //and will not be receiving any subsequent program packets for this memory region.
        if ((uint32_t)address >= (uint32_t)endAddress)
        {
            //Check if we still need to send a PROGRAM_COMPLETE command (we don't need to send one if
            //the last command we sent was a PRORAM_COMPLETE already).
            if (lastCommandSent == PROGRAM_COMPLETE)
            {
                break;
            }

            memset((void*)&writePacket, 0x00, sizeof(writePacket));
            writePacket.command = PROGRAM_COMPLETE;
            writePacket.bytesPerPacket = 0;
            LogMesg(levDEBUG_HI, "Sending final program complete command for this region.");

            ecResult = Comm_SendPacket(PComm, (unsigned char*)&writePacket, sizeof(writePacket));
            break;
        }
    } //while(address < endAddress)

    return ecResult;
}

/*

*/
TCommErrorCode Comm_GetData(TComm * PComm, uint32_t address,
    unsigned char bytesPerPacket,
    unsigned char bytesPerAddress,
    unsigned char bytesPerWord,
    uint32_t endAddress, unsigned char * pData)
{
    TReadPacket readPacket;
    TWritePacket writePacket;
    TCommErrorCode ecResult;

    if (!PComm->connected)
    {
      return ecNotConnected;
    }

    //First error check the input parameters before using them
    if ((pData == NULL) || (endAddress < address) || (bytesPerPacket == 0))
    {
        LogMesg(levINFO, "Error, bad parameters provided to call of GetData()");
        return ecFail;
    }

    // Continue reading from device until the entire programmable region has been read
    while (address < endAddress)
    {
        // Set up the buffer packet with the appropriate address and with the get data command
        memset((void*)&writePacket, 0x00, sizeof(writePacket));
        writePacket.command = GET_DATA;
        writePacket.address = address;

        // Calculate to see if the entire buffer can be filled with data, or just partially
        if(((endAddress - address) * bytesPerAddress) < bytesPerPacket)
            // If the amount of bytes left over between current address and end address is less than
            //  the max amount of bytes per packet, then make sure the bytesPerPacket info is updated
            writePacket.bytesPerPacket = (endAddress - address) * bytesPerAddress;
        else
            // Otherwise keep it at its maximum
            writePacket.bytesPerPacket = bytesPerPacket;

        // Send the packet
        ecResult = Comm_SendPacket(PComm, (unsigned char*)&writePacket, sizeof(writePacket));

        // If it wasn't successful, then return with error
        if(ecResult != ecSuccess)
        {
            LogMesg(levINFO, "Error during verify sending packet with address"); ///: 0x%x", (uint32_t)writePacket.address);
            return ecResult;
        }

        // Otherwise, read back the packet from the device
        memset((void*)&readPacket, 0x00, sizeof(readPacket));
        ecResult = Comm_ReceivePacket(PComm, (unsigned char*)&readPacket, sizeof(readPacket));

        // If it wasn't successful, then return with error
        if(ecResult != ecSuccess)
        {
            LogMesg(levINFO, "Error reading packet with address"); ///: 0x%x", (uint32_t)readPacket.address);
            return ecResult;
        }

        // Copy contents from packet to data pointer
        memcpy(pData, readPacket.data + 58 - readPacket.bytesPerPacket, readPacket.bytesPerPacket);

        // Increment data pointer
        pData += readPacket.bytesPerPacket;

        // Increment address by however many bytes were received divided by how many bytes per address
        address += readPacket.bytesPerPacket / bytesPerAddress;
    }

    // if successfully received entire region, return success
    return ecSuccess;
}


/*

*/
TCommErrorCode Comm_Erase(TComm * PComm)
{
    TWritePacket sendPacket;
    TCommErrorCode status;

    if (PComm->connected)
    {
        memset((void*)&sendPacket, 0x00, sizeof(sendPacket));
        sendPacket.command = ERASE_DEVICE;

        LogMesg(levDEBUG_HI, "Sending Erase Command...");

        status = Comm_SendPacket(PComm, (unsigned char*)&sendPacket, sizeof(sendPacket));

        if(status == ecSuccess)
            LogMesg(levDEBUG_HI, "Successfully sent erase command");
        else
            LogMesg(levERR, "Erasing Device Failed");

        return status;
    }

    LogMesg(levERR, "Device not connected");
    return ecNotConnected;
}


/*
Sends command to USB device to lock or unlock the config bit region.  If 
the config bits overlapped an erase page with standard program memory, 
this will also affect the status of the erase page.  (for example locking 
the config bits on a PIC18FxxJ or PIC24FJ device, which stores config bits 
at the end of the last page of flash memory, will also prevent reprogramming 
of the last page of flash memory). The bool lock value input parameter is 
either true (lock/protect the config bits), or false (unlock/allow
reprogramming of config bits and any overlapping flash erase page, if 
relevant).
*/
TCommErrorCode Comm_LockUnlockConfig(TComm * PComm, bool lock)
{
    TWritePacket sendPacket;
    TCommErrorCode status;

    if(PComm->connected)
    {
        memset((void*)&sendPacket, 0x00, sizeof(sendPacket));
        sendPacket.command = UNLOCK_CONFIG;

        if (lock == false)
        {
            sendPacket.LockedValue = 0x00;  //0x00 is used in the bootloader firmware to mean unlock the config bits ("UNLOCKCONFIG")
        }
        else
        {
            //All other values should mean we are locking the config bits
            sendPacket.LockedValue = 0x01;  //lock the config bits
        }

        if (lock)
            LogMesg(levDEBUG_HI, "Locking Config Bits...");
        else
            LogMesg(levDEBUG_HI, "Unlocking Config Bits...");

        status = Comm_SendPacket(PComm, (unsigned char*)&sendPacket, sizeof(sendPacket));

        if(status == ecSuccess)
            LogMesg(levDEBUG_HI, "Successfully sent un/lock command"); /// (%fs)", (double)elapsed.elapsed() / 1000);
        else
            LogMesg(levWARN, "Unsuccessfully sent un/lock command");

        return status;
    }

    LogMesg(levWARN, "Device not connected");
    return ecNotConnected;
}

/*
 */
TCommErrorCode Comm_ReadBootloaderInfo(TComm * PComm, TBootInfo * bootInfo)
{
    int nRetries = 0;
    TWritePacket sendPacket;
    TCommErrorCode ecResult;

    LogMesg(levDEBUG_HI, "Getting query packet...");

    if (PComm->connected)
    {
RETRY_CMD:
        memset((void *)&sendPacket, 0x00, sizeof(sendPacket));
        sendPacket.command = QUERY_DEVICE;

        ecResult = Comm_SendPacket(PComm, (unsigned char *)&sendPacket, sizeof(sendPacket));

        switch (ecResult)
        {
            case ecFail:
                Comm_close(PComm);
                return ecResult;

            case ecTimeout:
                return ecResult;

            default:
                break;
        }

        LogMesg(levDEBUG_HI, "Successfully sent querying command");

        memset((void*)bootInfo, 0x00, sizeof(TBootInfo));

        LogMesg(levDEBUG_HI, "Receiving packet...");

        ecResult = Comm_ReceivePacket(PComm, (unsigned char*)bootInfo, sizeof(TBootInfo));

        switch (ecResult)
        {
            case ecFail:
                Comm_close(PComm);
                return ecResult;

            case ecTimeout:
                break; // return ecResult;

            default:
                break;
        }

        if ((ecResult == ecTimeout) && (++nRetries < 3))
        {
          LogMesg(levDEBUG_HI, "Retrying..");
          goto RETRY_CMD;
        }

        if (bootInfo->command != 0x02)
        {
            LogMesg(levWARN, "Received incorrect command.");
            return ecIncorrectCommand;
        }

        LogMesg(levDEBUG_HI, "Successfully received query packet");
        return ecSuccess;
    }
    else
    {
      LogMesg(levERR, "Device not connected");
      return ecNotConnected;
    }
}


TCommErrorCode Comm_ReadExtendedQueryInfo(TComm * PComm, TExtendedQueryInfo * extendedBootInfo)
{
    TWritePacket sendPacket;
    TCommErrorCode status;

    LogMesg(levINFO, "Getting Extended Query Info packet...");

    if(PComm->connected)
    {
        memset((void*)&sendPacket, 0x00, sizeof(sendPacket));
        sendPacket.command = QUERY_EXTENDED_INFO;

        ///elapsed.start();

        status = Comm_SendPacket(PComm, (unsigned char*)&sendPacket, sizeof(sendPacket));

        switch(status)
        {
            case ecFail:
                Comm_close(PComm);
            case ecTimeout:
                return status;
            default:
                break;
        }

        LogMesg(levINFO, "Successfully sent QUERY_EXTENDED_INFO command"); /// (%fs)", (double)elapsed.elapsed() / 1000);
        memset((void*)extendedBootInfo, 0x00, sizeof(TExtendedQueryInfo));

        status = Comm_ReceivePacket(PComm, (unsigned char*)extendedBootInfo, sizeof(TExtendedQueryInfo));

        if(extendedBootInfo->command != QUERY_EXTENDED_INFO)
        {
            LogMesg(levWARN, "Received incorrect command.");
            return ecIncorrectCommand;
        }

        switch(status)
        {
            case ecFail:
                Comm_close(PComm);
            case ecTimeout:
                return status;
            default:
                break;
        }

        LogMesg(levINFO, "Successfully received QUERY_EXTENDED_INFO response packet"); /// (%fs)", (double)elapsed.elapsed() / 1000);
        return ecSuccess;
    }

    return ecNotConnected;
}


// >>> SDP - this function has not been tested <<<
TCommErrorCode Comm_SignFlash(TComm * PComm)
{
    TWritePacket sendPacket;
    TCommErrorCode status;
    TBootInfo QueryInfoBuffer;

    LogMesg(levDEBUG_HI, "Sending SIGN_FLASH command...");

    if(PComm->connected)
    {
        memset((void*)&sendPacket, 0x00, sizeof(sendPacket));
        sendPacket.command = SIGN_FLASH;

        status = Comm_SendPacket(PComm, (unsigned char*)&sendPacket, sizeof(sendPacket));

        switch(status)
        {
            case ecFail:
                Comm_close(PComm);
            case ecTimeout:
                return status;

            default:
                break;
        }

        //Now issue a query command, so as to "poll" for the completion of
        //the prior request (which doesn't by itself generate a respone packet).
        status = Comm_ReadBootloaderInfo(PComm, &QueryInfoBuffer);
        switch(status)
        {
            case ecFail:
                Comm_close(PComm);
            case ecTimeout:
                return status;
            default:
                break;
        }

        LogMesg(levDEBUG_HI, "Successfully sent SIGN_FLASH command"); 

        return ecSuccess;
    }//if(connected)

    return ecNotConnected;
}


TCommErrorCode Comm_SendPacket(TComm * PComm, unsigned char * pData, int size)
{
    TTimeoutCounter timeoutTimer;
    int nBytesSent;

    LogMesg(levDEBUG_LO, "Sending packet..");

    TimeoutCounter_start(&timeoutTimer);

    while (true) // keep trying until success or timeout
    {
        nBytesSent = hid_write(PComm->boot_device, pData, size);

        if (nBytesSent > 0)
        {
          char sMesg[80];

          sprintf(sMesg, "Packet sent, byte count=%d", nBytesSent);
          LogMesg(levDEBUG_LO, sMesg);
          return ecSuccess;
        }
        else if (nBytesSent == -1) // write failed, could be waiting on overlapped i/o
        {
            LogMesg(levDEBUG_LO, "SendPacket write failed, retrying...");
        }

        // check for timeout
        if (TimeoutCounter_isexpired(&timeoutTimer, PComm->SyncWaitTime))
        {
            LogMesg(levDEBUG_HI, "SendPacket timeout.");
            return ecTimeout;
        }
    } // while (true)
}


TCommErrorCode Comm_ReceivePacket(TComm * PComm, unsigned char * PBuffer, int Size)
{
    TTimeoutCounter tcTimeout;
    int nBytesRead;

    LogMesg(levDEBUG_LO, "Receiving packet..");
    
    TimeoutCounter_start(&tcTimeout);

    while (true)
    {
        nBytesRead = hid_read(PComm->boot_device, PBuffer, Size);

        if (nBytesRead > 0)
        {
          char sMesg[80];

          sprintf(sMesg, "Packet recvd, byte count=%d", nBytesRead);
          LogMesg(levDEBUG_LO, sMesg);
          return ecSuccess;
        }
        else if (nBytesRead == -1) // read failed, could be waiting on overlapped i/o
        {
            LogMesg(levDEBUG_LO, "ReceivePacket failed, retrying...");
        }

        // check for timeout
        if (TimeoutCounter_isexpired(&tcTimeout, PComm->SyncWaitTime))
        {
            LogMesg(levDEBUG_HI, "ReceivePacket timeout");
            return ecTimeout;
        }
    } // while (true)
}

