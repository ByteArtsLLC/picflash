#ifndef LOGGER_H
#define LOGGER_H

typedef enum
{
  levERR,
  levWARN,
  levINFO,
  levDEBUG_HI,
  levDEBUG_LO
} TLogLevel;

extern TLogLevel g_LogLevel;

void LogMesg(TLogLevel Level, const char * Mesg);

#endif
