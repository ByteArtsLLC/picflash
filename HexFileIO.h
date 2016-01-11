#ifndef HEXFILEIO_H
#define HEXFILEIO_H

#include <stdio.h>
#include "typedefs.h"

typedef struct {
  FILE * PFileHandle;
  int CurrentLine;
  int CharIndex;
  bool EndOfFile;
} THexFileIO;

typedef struct {
  char * Chars;
  int Length;
} THexLine;

extern bool HexFile_Open(THexFileIO * PFile, char * Filename);
extern bool HexFile_atEnd(THexFileIO * PFile);
extern void HexFile_close(THexFileIO * PFile);
extern bool HexFile_ReadNextLine(THexFileIO * PFile, THexLine * PLine);

extern unsigned int AsciiHexToInt(char * Value, int start, int len);

#endif