#include "Delays.h"
#include "Logger.h"

#ifdef _WIN32
  #include "Windows.h"  // for GetTickCount()
#endif

#ifndef _WIN32

  #include <sys/time.h>

  unsigned GetTickCount(void)
  {
    struct timeval tv;

    if (gettimeofday(&tv, 0) != 0)
    {
      LogMesg(levDEBUG_LO, "gettimeofday failed");
      return 0;
    }

    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
  }

#endif

void DelayMillisecs(unsigned Value)
{
  TTimeoutCounter tcTimeout;

  if (!Value) {
    return;
  }

  TimeoutCounter_start(&tcTimeout);
  while (!TimeoutCounter_isexpired(&tcTimeout, Value))
    ;
}

void TimeoutCounter_start(TTimeoutCounter * Counter)
{
  Counter->elapsed = 0;
  Counter->StartTicks = GetTickCount();
}

unsigned TimeoutCounter_elapsed(TTimeoutCounter * Counter)
{
  Counter->elapsed = GetTickCount() - Counter->StartTicks;
  return Counter->elapsed;
}

bool TimeoutCounter_isexpired(TTimeoutCounter * Counter, unsigned Timeout)
{
  return TimeoutCounter_elapsed(Counter) >= Timeout;
}

