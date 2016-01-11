#ifndef DELAYS_H
#define DELAYS_H

#include "typedefs.h"

typedef struct
{
  unsigned elapsed;
  unsigned long StartTicks;
} TTimeoutCounter;


extern void DelayMillisecs(unsigned Value);
extern void TimeoutCounter_start(TTimeoutCounter * Counter);
extern unsigned TimeoutCounter_elapsed(TTimeoutCounter * Counter);
extern bool TimeoutCounter_isexpired(TTimeoutCounter * Counter, unsigned Timeout);


#endif
