/*
PROJECT: PICFLASH
Program to load firmware on a PIC microcontroller using the HID bootloader.
*/

/* === INCLUDES ============================================================= */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Comm.h"
#include "ImportExportHex.h"
#include "Logger.h"


/* === TYPES ================================================================ */

/* === MACROS =============================================================== */

/* === CONSTANTS =============================================================*/

// Value used for error checking device reponse values.
#define MAXIMUM_PROGRAMMABLE_MEMORY_SEGMENT_SIZE 0x0FFFFFFF

// Values returned by main()
#define errSUCCESS          0
#define errINVALID_PARAMS   1
#define errDEVICENOTFOUND   2
#define errFILEREAD_ERROR   3
#define errERASE_ERROR      4
#define errPROGRAM_ERROR    5
#define errVERIFY_ERROR     6
#define errRESET_ERROR      7

/* === GLOBAL DATA ===========================================================*/


/* ====== LOCAL DATA =========================================================*/

static TComm m_Comm;
static TDeviceData m_DeviceData;
static TDeviceData m_HexFileData;
static TDevice m_Device;
static TExtendedQueryInfo m_ExtBootInfo;
static THexImporter m_HexFileImporter;

static bool m_FirmwareIsAtLeast101 = false;
static bool m_CheckForDevice = false;
static bool m_EraseDevice = false;
static bool m_LoadHexFile = false;
static bool m_ResetDevice = false;
static bool m_VerifyDevice = false;
static bool m_WriteToFlash = false;
static bool m_WriteToEeprom = false;
static bool m_WriteToConfig = false;

/* === LOCAL PROTOTYPES ===================================================== */

static TCommErrorCode EraseDevice(void);
static TCommErrorCode GetDeviceInfo(void);
static THexImportErrorCode LoadFile(char * Filename);
static TCommErrorCode VerifyDevice(void);
static TCommErrorCode WriteDevice(void);


/* === IMPLEMENTATION ======================================================= */

static TCommErrorCode EraseDevice(void)
{
  TCommErrorCode ecResult;
  TBootInfo bootInfo;

  ecResult = Comm_Erase(&m_Comm); //result = comm->Erase();
  if (ecResult != ecSuccess)
  {
      LogMesg(levERR, "Erase failed");
      return ecResult;
  }

  ecResult = Comm_ReadBootloaderInfo(&m_Comm, &bootInfo);

  if (ecResult != ecSuccess)
  {
      LogMesg(levERR, "ReadBootloaderInfo after erase failed");
      return ecResult;
  }

  return ecSuccess;
}


static TCommErrorCode GetDeviceInfo(void)
{
    TBootInfo bootInfo;
    TMemoryRange range;
    int nIndex;
    TCommErrorCode ecResult;

    if (!Comm_isConnected(&m_Comm))
    {
        LogMesg(levINFO, "Device not connected");
        return ecNotConnected;
    }

    // Send the Query command to the device over USB, and check the result status.
    ecResult = Comm_ReadBootloaderInfo(&m_Comm, &bootInfo);
    switch (ecResult)
    {
        case ecFail:
        case ecIncorrectCommand:
            LogMesg(levWARN, "Unable to communicate with device");
            return ecResult;

        case ecTimeout:
            LogMesg(levWARN, "Operation timed out");
            return ecResult;

        case ecSuccess:
            LogMesg(levDEBUG_HI, "Device connected");
            break;

        default: // ?
            LogMesg(levDEBUG_HI, "unknown result");
            return ecOther;
    }

    DeviceData_clearranges(&m_DeviceData); // deviceData->ranges.clear();

    //Now start parsing the bootInfo packet to learn more about the device.  The bootInfo packet contains
    //contains the query response data from the USB device.  We will save these values into globabl variables
    //so other parts of the application can use the info when deciding how to do things.
    m_Device.family = bootInfo.deviceFamily; // device->family = (Device::Families) bootInfo.deviceFamily;
    m_Device.bytesPerPacket = bootInfo.bytesPerPacket; //  device->bytesPerPacket = bootInfo.bytesPerPacket;

    //Set some processor family specific global variables that will be used elsewhere (ex: during program/verify operations).
    switch (m_Device.family) //(device->family)
    {
        case famPIC18:
            LogMesg(levINFO, "Device family is PIC18");
            m_Device.bytesPerWordFLASH = 2;
            m_Device.bytesPerAddressFLASH = 1;
            break;

        case famPIC24:
            LogMesg(levINFO, "Device family is PIC24");
            m_Device.bytesPerWordFLASH = 4;
            m_Device.bytesPerAddressFLASH = 2;
            m_Device.bytesPerWordConfig = 4;
            m_Device.bytesPerAddressConfig = 2;
            break;

        case famPIC32:
            LogMesg(levINFO, "Device family is PIC32");
            m_Device.bytesPerWordFLASH = 4;
            m_Device.bytesPerAddressFLASH = 1;
            break;

        case famPIC16:
            LogMesg(levINFO, "Device family is PIC16");
            m_Device.bytesPerWordFLASH = 2;
            m_Device.bytesPerAddressFLASH = 2;

        default:
            LogMesg(levINFO, "Device family is ?");
            m_Device.bytesPerWordFLASH = 2;
            m_Device.bytesPerAddressFLASH = 1;
            break;
    }

    //Initialize the deviceData buffers and length variables, with the regions that the firmware claims are
    //reprogrammable.  We will need this information later, to decide what part(s) of the .hex file we
    //should look at/try to program into the device.  Data sections in the .hex file that are not included
    //in these regions should be ignored.
    for (nIndex = 0; nIndex < MAX_DATA_REGIONS; ++nIndex)
    {
        if (bootInfo.memoryRegions[nIndex].type == END_OF_TYPES_LIST)
        {
            //Before we quit, check the special versionFlag byte,
            //to see if the bootloader firmware is at least version 1.01.
            //If it is, then it will support the extended query command.
            //If the device is based on v1.00 bootloader firmware, it will have
            //loaded the versionFlag location with 0x00, which was a pad byte.
            if (bootInfo.versionFlag == BOOTLOADER_V1_01_OR_NEWER_FLAG)
            {
                m_FirmwareIsAtLeast101 = true;
                LogMesg(levDEBUG_HI, "Device bootloader firmware is v1.01 or newer and supports Extended Query.");
                // Now fetch the extended query information packet from the USB firmware.
                Comm_ReadExtendedQueryInfo(&m_Comm, &m_ExtBootInfo);  // comm->ReadExtendedQueryInfo(&m_ExtBootInfo);
                LogMesg(levDEBUG_HI, "Device bootloader firmware version is: " + m_ExtBootInfo.PIC18.bootloaderVersion);
            }
            else
            {
                LogMesg(levDEBUG_HI, "Device bootloader firmware version is older than V1.01");
                m_FirmwareIsAtLeast101 = false;
            }
            break;
        }

        //Error check: Check the firmware's reported size to make sure it is sensible.  This ensures
        //we don't try to allocate ourselves a massive amount of RAM (capable of crashing this PC app)
        //if the firmware claimed an improper value.
        if(bootInfo.memoryRegions[nIndex].size > MAXIMUM_PROGRAMMABLE_MEMORY_SEGMENT_SIZE)
        {
            bootInfo.memoryRegions[nIndex].size = MAXIMUM_PROGRAMMABLE_MEMORY_SEGMENT_SIZE;
        }

        // Parse the bootInfo response packet and allocate ourselves some RAM to hold the eventual data to program.
        MemoryRange_construct(&range);
        if (bootInfo.memoryRegions[nIndex].type == PROGRAM_MEMORY)
        {
            range.type = PROGRAM_MEMORY;
            range.dataBufferLength = bootInfo.memoryRegions[nIndex].size * m_Device.bytesPerAddressFLASH; //device->bytesPerAddressFLASH;

            range.pDataBuffer = (unsigned char *)malloc(range.dataBufferLength); //new unsigned char[range.dataBufferLength];
            memset(&range.pDataBuffer[0], 0xFF, range.dataBufferLength);
        }
        else if (bootInfo.memoryRegions[nIndex].type == EEPROM_MEMORY)
        {
            range.type = EEPROM_MEMORY;
            range.dataBufferLength = bootInfo.memoryRegions[nIndex].size * m_Device.bytesPerAddressEEPROM;  //device->bytesPerAddressEEPROM;
            range.pDataBuffer = (unsigned char *)malloc(range.dataBufferLength); //new unsigned char[range.dataBufferLength];
            memset(&range.pDataBuffer[0], 0xFF, range.dataBufferLength);
        }
        else if (bootInfo.memoryRegions[nIndex].type == CONFIG_MEMORY)
        {
            range.type = CONFIG_MEMORY;
            range.dataBufferLength = bootInfo.memoryRegions[nIndex].size * m_Device.bytesPerAddressConfig; //device->bytesPerAddressConfig;
            range.pDataBuffer = (unsigned char *)malloc(range.dataBufferLength); //new unsigned char[range.dataBufferLength];
            memset(&range.pDataBuffer[0], 0xFF, range.dataBufferLength);
        }

        //Notes regarding range.start and range.end: The range.start is defined as the starting address inside
        //the USB device that will get programmed.  For example, if the bootloader occupies 0x000-0xFFF flash
        //memory addresses (ex: on a PIC18), then the starting bootloader programmable address would typically
        //be = 0x1000 (ex: range.start = 0x1000).
        //The range.end is defined as the last address that actually gets programmed, plus one, in this programmable
        //region.  For example, for a 64kB PIC18 microcontroller, the last implemented flash memory address
        //is 0xFFFF.  If the last 1024 bytes are reserved by the bootloader (since that last page contains the config
        //bits for instance), then the bootloader firmware may only allow the last address to be programmed to
        //be = 0xFBFF.  In this scenario, the range.end value would be = 0xFBFF + 1 = 0xFC00.
        //When this application uses the range.end value, it should be aware that the actual address limit of
        //range.end does not actually get programmed into the device, but the address just below it does.
        //In this example, the programmed region would end up being 0x1000-0xFBFF (even though range.end = 0xFC00).
        //The proper code to program this would basically be something like this:
        //for(i = range.start; i < range.end; i++)
        //{
        //    //Insert code here that progams one device address.  Note: for PIC18 this will be one byte for flash memory.
        //    //For PIC24 this is actually 2 bytes, since the flash memory is addressed as a 16-bit word array.
        //}
        //In the above example, the for() loop exits just before the actual range.end value itself is programmed.

        range.start = bootInfo.memoryRegions[nIndex].address;
        range.end = bootInfo.memoryRegions[nIndex].address + bootInfo.memoryRegions[nIndex].size;

        // Add the new structure+buffer to the list
        DeviceData_addrange(&m_DeviceData, &range); // deviceData->ranges.append(range);
    }

    return ecSuccess; // success
} // GetDeviceInfo()


static THexImportErrorCode LoadFile(char * Filename)
{
  THexImportErrorCode hecHexImportError;
  unsigned int nRangeIndex;
  TMemoryRange dev_range, hex_range;

  DeviceData_clearranges(&m_HexFileData);

  //First duplicate the deviceData programmable region list and
  //allocate some RAM buffers to hold the hex data that we are about to import.
  //foreach(DeviceData::MemoryRange range, deviceData->ranges)
  for (nRangeIndex = 0; nRangeIndex < m_DeviceData.RangeCount; ++nRangeIndex)
  {
      dev_range = m_DeviceData.ranges[nRangeIndex];

      //Allocate some RAM for the hex file data we are about to import.
      //Initialize all bytes of the buffer to 0xFF, the default unprogrammed memory value,
      //which is also the "assumed" value, if a value is missing inside the .hex file, but
      //is still included in a programmable memory region.
      //range.pDataBuffer = new unsigned char[range.dataBufferLength];
      //hexData->ranges.append(range);
      MemoryRange_construct(&hex_range);
      hex_range.pDataBuffer = (unsigned char *)malloc(dev_range.dataBufferLength);
      hex_range.dataBufferLength = dev_range.dataBufferLength;
      memset(hex_range.pDataBuffer, 0xFF, hex_range.dataBufferLength);

      hex_range.type = dev_range.type;
      hex_range.start = dev_range.start;
      hex_range.end = dev_range.end;

      DeviceData_addrange(&m_HexFileData, &hex_range);
  }

  hecHexImportError = ImportHexFile(&m_HexFileImporter, Filename, &m_HexFileData, &m_Device);

  switch (hecHexImportError)
  {
    case hecSuccess:
      break;

    case hecCouldNotOpenFile:
      LogMesg(levERR, "Error: could not open hex file");
      break;

    case hecNoneInRange:
      LogMesg(levERR, "Error: no data in range");
      break;

    case hecErrorInHexFile:
      LogMesg(levERR, "Error: error in hex");
      break;

    case hecInsufficientMemory:
      LogMesg(levERR, "Error: insufficient memory");
      break;
  }

  return hecHexImportError;
}


/*
Command line usage:
picflash [options] <filename>
*/
int main(int argc, char * argv[])
{
  int nReturnVal = 0;
  int nArg;
  char * sHexFile = 0;

  LogMesg(levINFO, "PICFLASH V0.3");

  /*
  Validate command line arguments
  */

  for (nArg = 1; nArg < argc; ++nArg)
  {
    if (argv[nArg][0] != '-')
    {
      sHexFile = argv[nArg];
    }
    else
    {
      switch (argv[nArg][1])
      {
        case 'c':
          m_CheckForDevice = true;
          break;

        case 'e':  // erase
          m_EraseDevice = true;
          break;

        case 'l':  // log level
          g_LogLevel = argv[nArg][2] - '0';
          break;

        case 'r':  // reset
          m_ResetDevice = true;
          break;

        case 'v':  // verify
          m_VerifyDevice = true;
          m_LoadHexFile = true; // this is implied
          break;

        case 'w':  // write flash
          m_WriteToFlash = true;
          m_EraseDevice = true; // always erase before programming
          m_LoadHexFile = true; // this is implied
          break;
      }
    }
  }

  if (!(m_WriteToFlash || m_EraseDevice || m_ResetDevice || m_VerifyDevice || m_CheckForDevice))
  {
    LogMesg(levERR, "Missing or invalid command line options");
    LogMesg(levINFO, "Usage: picflash [options] [<filename>]");
    LogMesg(levINFO, "Options:\r\n-c     Check for device\r\n-e     Erase\r\n-l<N>  LogLevel<N>\r\n-r     Reset\r\n-v     Verify\r\n-w <filename>   Write");
    nReturnVal = errINVALID_PARAMS;
    goto EXIT_APP;
  }

  if (m_LoadHexFile && !sHexFile)
  {
    LogMesg(levERR, "-w or -v options require a filename");
    nReturnVal = errINVALID_PARAMS;
    goto EXIT_APP;
  }

  m_CheckForDevice = true; // always do check

  /*
  Check for device in bootload mode
  */
  Comm_construct(&m_Comm, 4000); // 4 sec timeout
  DeviceData_construct(&m_DeviceData);
  DeviceData_construct(&m_HexFileData);
  Device_construct(&m_Device, &m_DeviceData);

  // Make initial check to see if the USB device is attached
  Comm_PollUSB(&m_Comm);

  if (Comm_isConnected(&m_Comm))
  {
      TCommErrorCode ecResult;

      LogMesg(levDEBUG_HI, "Attempting to open device...");

      ecResult = Comm_open(&m_Comm);
      if (ecResult == ecSuccess)
      {
        LogMesg(levINFO, "Device Attached.");
      }
      else
      {
        LogMesg(levERR, "Comm_open failed");
        nReturnVal = errDEVICENOTFOUND;
        goto EXIT_APP;
      }
  }
  else
  {
      LogMesg(levWARN, "Device not detected. Verify device is attached and in firmware update mode.");
      nReturnVal = errDEVICENOTFOUND;
      goto EXIT_APP;
  }

  LogMesg(levINFO, "Getting device info..");
  if (GetDeviceInfo() != ecSuccess)
  {
    nReturnVal = errDEVICENOTFOUND;
    goto EXIT_APP;
  }

  // erase the device
  if (m_EraseDevice)
  {
    LogMesg(levINFO, "Erasing device..");
    if (EraseDevice() != ecSuccess)
    {
      LogMesg(levERR, "Error: erase failure");
      nReturnVal = errERASE_ERROR;
      goto EXIT_APP;
    }
    LogMesg(levINFO, "Erase complete.");
  }

  if (m_LoadHexFile)
  {
    LogMesg(levINFO, "Loading hex file..");
    if (LoadFile(sHexFile) != hecSuccess)
    {
      nReturnVal = errFILEREAD_ERROR;
      goto EXIT_APP;
    }
    LogMesg(levINFO, "Hex file loaded.");
  }

  if (m_WriteToFlash)
  {
    // program the device
    LogMesg(levINFO, "Programming device memory..");
    if (WriteDevice() != ecSuccess)
    {
      nReturnVal = errPROGRAM_ERROR;
      goto EXIT_APP;
    }
    LogMesg(levINFO, "Device memory programmed.");
  }

  // verify the device
  if (m_VerifyDevice)
  {
    LogMesg(levINFO, "Verifying device memory..");
    if (VerifyDevice() != ecSuccess)
    {
      nReturnVal = errVERIFY_ERROR;
      goto EXIT_APP;
    }
    LogMesg(levINFO, "Device memory verified.");
  }

  // reset the device
  if (m_ResetDevice)
  {
    LogMesg(levINFO, "Reseting device..");
    if (Comm_Reset(&m_Comm) != ecSuccess)
    {
      nReturnVal = errRESET_ERROR;
      goto EXIT_APP;
    }
    LogMesg(levINFO, "Device reset.");
  }

EXIT_APP:
  Comm_close(&m_Comm);
  {
    char sMesg[80];

    sprintf(sMesg, "Return value=%d", nReturnVal);
    LogMesg(levINFO, sMesg);
  }
  return nReturnVal;
} // main()


//This programs previously parsed .hex file data into the device's programmable memory regions.
static TCommErrorCode WriteDevice(void)
{
    TCommErrorCode ecResult;
    TMemoryRange hexRange;
    unsigned int nRangeIndex;

    //Now being re-programming each section based on the info we obtained when
    //we parsed the user's .hex file.

    for (nRangeIndex = 0; nRangeIndex < m_HexFileData.RangeCount; ++nRangeIndex)
    {
        hexRange = m_HexFileData.ranges[nRangeIndex];

        if (m_WriteToFlash && (hexRange.type == PROGRAM_MEMORY))
        {
            LogMesg(levDEBUG_HI, "Writing Device Program Memory...");
            ecResult = Comm_Program(&m_Comm,
                                   hexRange.start,
                                   m_Device.bytesPerPacket,
                                   m_Device.bytesPerAddressFLASH,
                                   m_Device.bytesPerWordFLASH,
                                   m_Device.family,
                                   hexRange.end,
                                   hexRange.pDataBuffer);
        }
        else if (m_WriteToEeprom && (hexRange.type == EEPROM_MEMORY))
        {
            LogMesg(levDEBUG_HI, "Writing Device EEPROM...");
                ecResult = Comm_Program(&m_Comm,
                                       hexRange.start,
                                       m_Device.bytesPerPacket,
                                       m_Device.bytesPerAddressEEPROM,
                                       m_Device.bytesPerWordEEPROM,
                                       m_Device.family,
                                       hexRange.end,
                                       hexRange.pDataBuffer);
        }
        else if (m_WriteToConfig && (hexRange.type == CONFIG_MEMORY))
        {
            LogMesg(levDEBUG_HI, "Writing Device Config Memory...");
            ecResult = Comm_Program(&m_Comm,
                                   hexRange.start,
                                   m_Device.bytesPerPacket,
                                   m_Device.bytesPerAddressConfig,
                                   m_Device.bytesPerWordConfig,
                                   m_Device.family,
                                   hexRange.end,
                                   hexRange.pDataBuffer);
        }
        else
        {
            continue;
        }

        if (ecResult != ecSuccess)
        {
            LogMesg(levERR, "Programming failed");
            return ecResult;
        }
    } // for nRange..

    return ecResult;
}

/*
Routine that verifies the contents of the non-voltaile memory regions in the
device, after an erase/programming cycle. This function requests the memory
contents of the device, then compares it against the parsed .hex file data
to make sure the locations that got programmed properly match.
*/
static TCommErrorCode VerifyDevice(void)
{
    TCommErrorCode ecResult;
    TMemoryRange deviceRange, hexRange;
    unsigned int i, j;
    unsigned int arrayIndex;
    bool failureDetected = false;
    unsigned char hexEraseBlockData[MAX_ERASE_BLOCK_SIZE];
    //unsigned char flashData[MAX_ERASE_BLOCK_SIZE];
    //uint32_t startOfEraseBlock;
    //uint32_t errorAddress = 0;
    //uint16_t expectedResult = 0;
    //uint16_t actualResult = 0;
    unsigned int nDevRangeIndex, nHexRangeIndex;

    /*
    Initialize an erase block sized buffer with 0xFF.
    Used later for post SIGN_FLASH verify operation.
    */
    memset(&hexEraseBlockData[0], 0xFF, MAX_ERASE_BLOCK_SIZE);

    // foreach(deviceRange, deviceData->ranges)
    for (nDevRangeIndex = 0; nDevRangeIndex < m_DeviceData.RangeCount; ++nDevRangeIndex)
    {
        deviceRange = m_DeviceData.ranges[nDevRangeIndex];

        if (m_WriteToFlash && (deviceRange.type == PROGRAM_MEMORY))
        {
            LogMesg(levDEBUG_HI, "Verifying device program memory...");
            
            ecResult = Comm_GetData(&m_Comm,
                          deviceRange.start,
                          m_Device.bytesPerPacket,
                          m_Device.bytesPerAddressFLASH,
                          m_Device.bytesPerWordFLASH,
                          deviceRange.end,
                          deviceRange.pDataBuffer);

            if (ecResult != ecSuccess)
            {
                failureDetected = true;
                LogMesg(levERR, "Error reading device program memory.");
            }

            /*
            Search through all of the programmable memory regions from the
            parsed .hex file data. For each of the programmable memory regions
            found, if the region also overlaps a region that was included in
            the device programmed area (which just got read back with GetData()),
            then verify both the parsed hex contents and read back data match.
            */
            for (nHexRangeIndex = 0; nHexRangeIndex < m_HexFileData.RangeCount; ++nHexRangeIndex)
            {
              hexRange = m_HexFileData.ranges[nHexRangeIndex];

                if (deviceRange.start == hexRange.start)
                {
                    //For this entire programmable memory address range, check to see if the data read from the device exactly
                    //matches what was in the hex file.
                    for (i = deviceRange.start; i < deviceRange.end; i++)
                    {
                        //For each byte of each device address (1 on PIC18, 2 on PIC24, since flash memory is 16-bit WORD array)
                        for (j = 0; j < m_Device.bytesPerAddressFLASH; j++)
                        {
                            //Check if the device response data matches the data we parsed from the original input .hex file.
                            if (deviceRange.pDataBuffer[((i - deviceRange.start) * m_Device.bytesPerAddressFLASH)+j]
                              != hexRange.pDataBuffer[((i - deviceRange.start) * m_Device.bytesPerAddressFLASH)+j])
                            {
                                /*
                                A mismatch was detected.
                                Check if this is a PIC24 device and we are looking at the "phantom byte"
                                (upper byte [j = 1] of odd address [i%2 == 1] 16-bit flash words).  If the hex data doesn't match
                                the device (which should be = 0x00 for these locations), this isn't a real verify
                                failure, since value is a don't care anyway.  This could occur if the hex file imported
                                doesn't contain all locations, and we "filled" the region with pure 0xFFFFFFFF, instead of 0x00FFFFFF
                                when parsing the hex file.
                                */
                                if((m_Device.family == famPIC24) && ((i % 2) == 1) && (j == 1))
                                {
                                    //Not a real verify failure, phantom byte is unimplemented and is a don't care.
                                }
                                else
                                {
                                    //If the data wasn't a match, and this wasn't a PIC24 phantom byte, then if we get
                                    //here this means we found a true verify failure.
                                    failureDetected = true;
                                    if (m_Device.family == famPIC24)
                                    {
                                        //qWarning("Device: 0x%x Hex: 0x%x", *(uint16_t*)&deviceRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressFLASH)+j], *(uint16_t*)&hexRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressFLASH)+j]);
                                    }
                                    else
                                    {
                                        //qWarning("Device: 0x%x Hex: 0x%x", deviceRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressFLASH)+j], hexRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressFLASH)+j]);
                                    }
                                    //qWarning("Failed verify at address 0x%x", i);
                                    //emit IoWithDeviceCompleted("Verify", Comm::Fail, ((double)elapsed.elapsed()) / 1000);
                                    LogMesg(levDEBUG_HI, "Verify error");
                                    return ecOther;
                                }
                            }//if(deviceRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressFLASH)+j] != hexRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressFLASH)+j])
                        }//for(j = 0; j < device->bytesPerAddressFLASH; j++)
                    }//for(i = deviceRange.start; i < deviceRange.end; i++)
                }//if(deviceRange.start == hexRange.start)
            }//foreach(hexRange, hexData->ranges)
            //emit IoWithDeviceCompleted("Verify", Comm::Success, ((double)elapsed.elapsed()) / 1000);
        }//if(writeFlash && (deviceRange.type == PROGRAM_MEMORY))
        else if (m_WriteToEeprom && (deviceRange.type == EEPROM_MEMORY))
        {
            LogMesg(levDEBUG_HI, "Verifying Device's EEPROM Memory...");

            ecResult = Comm_GetData(&m_Comm,
                deviceRange.start,
                m_Device.bytesPerPacket,
                m_Device.bytesPerAddressEEPROM,
                m_Device.bytesPerWordEEPROM,
                deviceRange.end,
                deviceRange.pDataBuffer);

            if (ecResult != ecSuccess)
            {
                failureDetected = true;
                LogMesg(levDEBUG_HI, "Error reading device EEPROM.");
            }

            /*
            Search through all of the programmable memory regions from the parsed .hex file data.
            For each of the programmable memory regions found, if the region also overlaps a region
            that was included in the device programmed area (which just got read back with GetData()),
            then verify both the parsed hex contents and read back data match.
            */
            for (nHexRangeIndex = 0; nHexRangeIndex < m_HexFileData.RangeCount; ++nHexRangeIndex)
            {
              hexRange = m_HexFileData.ranges[nHexRangeIndex];
                if(deviceRange.start == hexRange.start)
                {
                    //For this entire programmable memory address range, check to see if the data read from the device exactly
                    //matches what was in the hex file.
                    for (i = deviceRange.start; i < deviceRange.end; i++)
                    {
                        //For each byte of each device address (only 1 for EEPROM byte arrays, presumably 2 for EEPROM WORD arrays)
                        for (j = 0; j < m_Device.bytesPerAddressEEPROM; j++)
                        {
                            //Check if the device response data matches the data we parsed from the original input .hex file.
                            if(deviceRange.pDataBuffer[((i - deviceRange.start) * m_Device.bytesPerAddressEEPROM)+j]
                                != hexRange.pDataBuffer[((i - deviceRange.start) * m_Device.bytesPerAddressEEPROM)+j])
                            {
                                //A mismatch was detected.
                                failureDetected = true;
                                LogMesg(levDEBUG_HI, "EEPROM verify error");
                                //qWarning("Device: 0x%x Hex: 0x%x", deviceRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressFLASH)+j], hexRange.pDataBuffer[((i - deviceRange.start) * m_Device.bytesPerAddressFLASH)+j]);
                                //qWarning("Failed verify at address 0x%x", i);
                                //emit IoWithDeviceCompleted("Verify EEPROM Memory", Comm::Fail, ((double)elapsed.elapsed()) / 1000);
                                return ecOther;
                            }
                        }
                    }
                }
            } //foreach(hexRange, hexData->ranges)
            //emit IoWithDeviceCompleted("Verifying", Comm::Success, ((double)elapsed.elapsed()) / 1000);
        } //else if(writeEeprom && (deviceRange.type == EEPROM_MEMORY))
        else if (m_WriteToConfig && (deviceRange.type == CONFIG_MEMORY))
        {
            LogMesg(levDEBUG_HI, "Verifying device config memory...");

            ecResult = Comm_GetData(&m_Comm,
                 deviceRange.start,
                 m_Device.bytesPerPacket,
                 m_Device.bytesPerAddressConfig,
                 m_Device.bytesPerWordConfig,
                 deviceRange.end,
                 deviceRange.pDataBuffer);

            if (ecResult != ecSuccess)
            {
                failureDetected = true;
                LogMesg(levDEBUG_HI, "Error reading device config memory.");
            }

            /*
            Search through all of the programmable memory regions from the parsed .hex file data.
            For each of the programmable memory regions found, if the region also overlaps a region
            that was included in the device programmed area (which just got read back with GetData()),
            then verify both the parsed hex contents and read back data match.
            */
            for (nHexRangeIndex = 0; nHexRangeIndex < m_HexFileData.RangeCount; ++nHexRangeIndex)
            {
              hexRange = m_HexFileData.ranges[nHexRangeIndex];

                if(deviceRange.start == hexRange.start)
                {
                    //For this entire programmable memory address range, check to see if the data read from the device exactly
                    //matches what was in the hex file.
                    for(i = deviceRange.start; i < deviceRange.end; i++)
                    {
                        //For each byte of each device address (1 on PIC18, 2 on PIC24, since flash memory is 16-bit WORD array)
                        for(j = 0; j < m_Device.bytesPerAddressConfig; j++)
                        {
                            //Compute an index into the device and hex data arrays, based on the current i and j values.
                            arrayIndex = ((i - deviceRange.start) * m_Device.bytesPerAddressConfig)+j;

                            //Check if the device response data matches the data we parsed from the original input .hex file.
                            if(deviceRange.pDataBuffer[arrayIndex] != hexRange.pDataBuffer[arrayIndex])
                            {
                                //A mismatch was detected.  Perform additional checks to make sure it was a real/unexpected verify failure.

                                //Check if this is a PIC24 device and we are looking at the "phantom byte"
                                //(upper byte [j = 1] of odd address [i%2 == 1] 16-bit flash words).  If the hex data doesn't match
                                //the device (which should be = 0x00 for these locations), this isn't a real verify
                                //failure, since value is a don't care anyway.  This could occur if the hex file imported
                                //doesn't contain all locations, and we "filled" the region with pure 0xFFFFFFFF, instead of 0x00FFFFFF
                                //when parsing the hex file.
                                if((m_Device.family == famPIC24) && ((i % 2) == 1) && (j == 1))
                                {
                                    //Not a real verify failure, phantom byte is unimplemented and is a don't care.
                                }//Make further special checks for PIC18 non-J devices
                                else if((m_Device.family == famPIC18) && (deviceRange.start == 0x300000) && ((i == 0x300004) || (i == 0x300007)))
                                {
                                     //The "CONFIG3L" and "CONFIG4H" locations (0x300004 and 0x300007) on PIC18 non-J USB devices
                                     //are unimplemented and should be masked out from the verify operation.
                                }
                                else
                                {
                                    //If the data wasn't a match, and this wasn't a PIC24 phantom byte, then if we get
                                    //here this means we found a true verify failure.
                                    failureDetected = true;
                                    if(m_Device.family == famPIC24)
                                    {
                                        //qWarning("Device: 0x%x Hex: 0x%x", *(uint16_t*)&deviceRange.pDataBuffer[((i - deviceRange.start) * m_Device.bytesPerAddressConfig)+j], *(uint16_t*)&hexRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressConfig)+j]);
                                    }
                                    else
                                    {
                                        //qWarning("Device: 0x%x Hex: 0x%x", deviceRange.pDataBuffer[((i - deviceRange.start) * m_Device.bytesPerAddressConfig)+j], hexRange.pDataBuffer[((i - deviceRange.start) * device->bytesPerAddressConfig)+j]);
                                    }
                                    //qWarning("Failed verify at address 0x%x", i);
                                    //emit IoWithDeviceCompleted("Verify Config Bit Memory", Comm::Fail, ((double)elapsed.elapsed()) / 1000);
                                    LogMesg(levDEBUG_HI, "Verify Config Bit Memory error");
                                    return ecOther;
                                }
                            }
                        }
                    }
                }
            }//foreach(hexRange, hexData->ranges)
            //emit IoWithDeviceCompleted("Verifying", Comm::Success, ((double)elapsed.elapsed()) / 1000);
        }//else if(writeConfig && (deviceRange.type == CONFIG_MEMORY))
        else
        {
            continue;
        }
    } //foreach(deviceRange, deviceData->ranges)

#if 0  // extended info
    if (failureDetected == false)
    {
        //Successfully verified all regions without error.
        //If this is a v1.01 or later device, we now need to issue the SIGN_FLASH
        //command, and then re-verify the first erase page worth of flash memory
        //(but with the exclusion of the signature WORD address from the verify,
        //since the bootloader firmware will have changed it to the new/magic
        //value (probably 0x600D, or "good" in leet speak).
        if (m_FirmwareIsAtLeast101 == true)
        {
            comm->SignFlash();

            qDebug("Expected Signature Address: 0x%x", m_ExtBootInfo.PIC18.signatureAddress);
            qDebug("Expected Signature Value: 0x%x", m_ExtBootInfo.PIC18.signatureValue);


            //Now re-verify the first erase page of flash memory.
            if(device->family == famPIC18)
            {
                startOfEraseBlock = m_ExtBootInfo.PIC18.signatureAddress - (m_ExtBootInfo.PIC18.signatureAddress % m_ExtBootInfo.PIC18.erasePageSize);
                ecResult = comm->GetData(startOfEraseBlock,
                                       device->bytesPerPacket,
                                       device->bytesPerAddressFLASH,
                                       device->bytesPerWordFLASH,
                                       (startOfEraseBlock + m_ExtBootInfo.PIC18.erasePageSize),
                                       &flashData[0]);
                if(ecResult != ecSuccess)
                {
                    failureDetected = true;
                    qWarning("Error reading, post signing, flash data block.");
                }

                //Search through all of the programmable memory regions from the parsed .hex file data.
                //For each of the programmable memory regions found, if the region also overlaps a region
                //that is part of the erase block, copy out bytes into the hexEraseBlockData[] buffer,
                //for re-verification.
                foreach(hexRange, hexData->ranges)
                {
                    //Check if any portion of the range is within the erase block of interest in the device.
                    if((hexRange.start <= startOfEraseBlock) && (hexRange.end > startOfEraseBlock))
                    {
                        unsigned int rangeSize = hexRange.end - hexRange.start;
                        unsigned int address = hexRange.start;
                        unsigned int k = 0;

                        //Check every byte in the hex file range, to see if it is inside the erase block of interest
                        for(i = 0; i < rangeSize; i++)
                        {
                            //Check if the current byte we are looking at is inside the erase block of interst
                            if(((address+i) >= startOfEraseBlock) && ((address+i) < (startOfEraseBlock + m_ExtBootInfo.PIC18.erasePageSize)))
                            {
                                //The byte is in the erase block of interst.  Copy it out into a new buffer.
                                hexEraseBlockData[k] = *(hexRange.pDataBuffer + i);
                                //Check if this is a signature byte.  If so, replace the value in the buffer
                                //with the post-signing expected signature value, since this is now the expected
                                //value from the device, rather than the value from the hex file...
                                if((address+i) == m_ExtBootInfo.PIC18.signatureAddress)
                                {
                                    hexEraseBlockData[k] = (unsigned char)m_ExtBootInfo.PIC18.signatureValue;    //Write LSB of signature into buffer
                                }
                                if((address+i) == (m_ExtBootInfo.PIC18.signatureAddress + 1))
                                {
                                    hexEraseBlockData[k] = (unsigned char)(m_ExtBootInfo.PIC18.signatureValue >> 8); //Write MSB into buffer
                                }
                                k++;
                            }
                            if((k >= m_ExtBootInfo.PIC18.erasePageSize) || (k >= sizeof(hexEraseBlockData)))
                                break;
                        }//for(i = 0; i < rangeSize; i++)
                    }
                }//foreach(hexRange, hexData->ranges)

                //We now have both the hex data and the post signing flash erase block data
                //in two RAM buffers.  Compare them to each other to perform post-signing
                //verify.
                for (i = 0; i < m_ExtBootInfo.PIC18.erasePageSize; i++)
                {
                    if(flashData[i] != hexEraseBlockData[i])
                    {
                        failureDetected = true;
                        // qWarning("Post signing verify failure.");
                        EraseDevice();  //Send an erase command, to forcibly
                        //remove the signature (which might be valid), since
                        //there was a verify error and we can't trust the application
                        //firmware image integrity.  This ensures the device jumps
                        //back into bootloader mode always.

                        //errorAddress = startOfEraseBlock + i;
                        //expectedResult = hexEraseBlockData[i] + ((uint32_t)hexEraseBlockData[i+1] << 8);
                        //expectedResult = hexEraseBlockData[i];
                        //actualResult = flashData[i] + ((uint32_t)flashData[i+1] << 8);
                        //actualResult = flashData[i];

                        break;
                    }
                }//for(i = 0; i < m_ExtBootInfo.PIC18.erasePageSize; i++)
            }//if(device->family == Device::PIC18)

        }//if(deviceFirmwareIsAtLeast101 == true)
    }//if(failureDetected == false)
#endif

    if (failureDetected == true)
    {
        LogMesg(levERR, "Verify failed");
//        qDebug("Verify failed at address: 0x%x", errorAddress);
//        qDebug("Expected Result: 0x%x", expectedResult);
//        qDebug("Actual result: 0x%x", actualResult);
//        emit AppendString("Operation aborted due to error encountered during verify operation.");
//        emit AppendString("Please try the erase/program/verify sequence again.");
//        emit AppendString("If repeated failures are encountered, this may indicate the flash");
//        emit AppendString("memory has worn out, that the device has been damaged, or that");
//        emit AppendString("there is some other unidentified problem.");
//
//        emit IoWithDeviceCompleted("Verify", Comm::Fail, ((double)elapsed.elapsed()) / 1000);
    }
    else
    {
//        emit IoWithDeviceCompleted("Verify", Comm::Success, ((double)elapsed.elapsed()) / 1000);
//        emit AppendString("Erase/Program/Verify sequence completed successfully.");
//        emit AppendString("You may now unplug or reset the device.");
    }

    return ecResult;
} // VerifyDevice()

//---------------------------------------------------------------------------
