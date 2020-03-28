#include <Arduino.h>
extern void (*mySysTickHook)(void);


int sysTickHook(void)
{
  if(mySysTickHook != NULL) mySysTickHook();
  return(false);
}
