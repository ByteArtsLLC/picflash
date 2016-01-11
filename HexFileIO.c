#include "HexFileIO.h"
#include <string.h>
#include <ctype.h>

#define LINE_BUF_SIZE 1024

char LineBuffer[LINE_BUF_SIZE];

bool HexFile_Open(THexFileIO * PFile, char * Filename)
{
  PFile->PFileHandle = fopen(Filename, "r");
  PFile->CurrentLine = 0;
  PFile->CharIndex = 0;
  PFile->EndOfFile = false;

  return (PFile->PFileHandle != 0);
}


bool HexFile_atEnd(THexFileIO * PFile)
{
  return PFile->EndOfFile;
}

void HexFile_close(THexFileIO * PFile)
{
  if (PFile->PFileHandle)
  {
    fclose(PFile->PFileHandle);
    PFile->PFileHandle = 0;
  }
}


bool HexFile_ReadNextLine(THexFileIO * PFile, THexLine * PLine)
{
  int nChar, nBufferIndex;
  bool boolEndOfLine = false;

  if (!PFile->PFileHandle)
    return false;

  nBufferIndex = 0;
  memset(&LineBuffer[0], 0, LINE_BUF_SIZE);

  while ((!boolEndOfLine) && (nBufferIndex < LINE_BUF_SIZE - 1))
  {
    nChar = fgetc(PFile->PFileHandle);

    switch (nChar)
    {
      case 10:
      case 13:
        if (nBufferIndex)
        {
          boolEndOfLine = true;
        }
        break;

      case EOF:
        boolEndOfLine = true;
        PFile->EndOfFile = true;
        break;

      default:
        LineBuffer[nBufferIndex++] = (char)nChar;
        break;
    } // switch (nChar)
  } // while (true)

  PLine->Chars = &LineBuffer[0];
  PLine->Length = nBufferIndex;

  return (nBufferIndex > 0);
}


unsigned int AsciiHexToInt(char * Value, int start, int len)
{
  int Result = 0;
  unsigned char bByte;
  char * pChar;
  char chChar;

  pChar = Value + start;

  while (len)
  {
    chChar = toupper(*pChar);

    if ((chChar >= '0') && (chChar <= '9'))
    {
      bByte = chChar - '0';
    }
    else if ((chChar >= 'A') && (chChar <= 'F'))
    {
      bByte = chChar - 'A' + 10;
    }

    Result = (Result << 4) + bByte;
    ++pChar;
    --len;
  }

  return Result;
}




