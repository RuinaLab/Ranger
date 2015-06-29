#include <mb_includes.h>

long unsigned int a9_clock_ms_counter = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void a9_clock_tick(void)
{
  static short int i = 0;
  
  ++i;
  if (i >= A9_CLOCK_TICKS_PER_MS)
  {
    i = 0;
    T0TC = 0;   //Reset timer0 watchdog/local time to zero
    T0MR0 = 19500 - 1;	//Timer counter will reset and interrupt at MR (match register) + 1 intervals
                        //Set watchdog/local timer to run slow, at 1.5 mS cycle time
                        //Thus, it should not be doing anything while SSP SSEL sync signals
                        //are arriving on time from the ARM7
                        
    //increment mS counter                    
    ++a9_clock_ms_counter;
    
    mb_schedule_tick();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Return time in seconds since start of schedule execution
float mb_clock_get_execution_time(void)
{
  return ((float)T0TC * 7.692308E-8);   //Assumes 13 MHz clock rate for T0
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
/////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long int mb_clock_get_time(void)
{
  return a9_clock_ms_counter;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt service routine for LPC3250 standard timer 0
void a9_timer0_isr(void)
{
  #ifdef DEBUG
  if (P3_INP_STATE & 1<<1)
  {
    MIC_ER = 0;
    return;
  }
  #endif
  
  T0IR = 0xFF;	//Clear Timer0 interrupts
   
  mb_schedule_tick();
  
  ++a9_clock_ms_counter;
  
  //error - not synchronized with ARM7, if this isr runs. (Running in local time mode.)
  
  //Set timer0 to run at 1 ms for accurate local mode time.
  //This should happen only in absence of SSP SSEL synchronization from ARM7. 
  T0MR0 = 13000 - 1;	//Timer counter will reset and interrupt at MR (match register) + 1 intervals
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////
