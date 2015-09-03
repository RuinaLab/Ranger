/*

  tic_toc.c
  Utility functions for measuring the time elapsed for different
  exectutions.
  
  Nicolas Williamson - June 2009
  
*/

#include <includes.h>

unsigned int tt_elapsed = 0;
unsigned char tt_on = 0;

void tic(void)
{
     tt_elapsed = TT_CLOCK;
     tt_on = 1;
}

//Doesn't account for overflows in Timer 1
void toc(void)
{
     if (tt_on){
        tt_elapsed = TT_CLOCK - tt_elapsed;
        tt_on = 0;
     }
}

unsigned int tt_get_time_elapsed(void)
{
     return tt_elapsed;
}



