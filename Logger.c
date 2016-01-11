#include "Logger.h"
#include <stdio.h>

TLogLevel g_LogLevel = levINFO;

const char * sLOG_LEVELS[5] =
{
  "ERROR",    // 0
  "WARNING",  // 1
  "INFO",     // 2
  "DEBUG_HI", // 3
  "DEBUG_LO"  // 4
};

void LogMesg(TLogLevel Level, const char * Mesg)
{
  if (Level <= g_LogLevel)
  {
    printf("%s: %s\n", sLOG_LEVELS[Level], Mesg);
  }
}

