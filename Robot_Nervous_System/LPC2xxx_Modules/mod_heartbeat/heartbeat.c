/**
  @file heartbeat.c
  
  A simple file for periodic function calls either based on a timestamp
  or on a count.
  
  @author Nicolas Champagne-Williamson
  @date 2009
*/

#include <includes.h>

#define DEFAULT_HEARTBEAT_PERIOD (9) //in units of schedule periods (usually 1 ms)

static int hb_state = 0; /**< Whether we are in a heartbeat or not. Used for transition checking. 1 = hb, 0 = not hb. */
static int hb_period = DEFAULT_HEARTBEAT_PERIOD; /**< The period between function calls. */
static VOID_VOID_F hb_function = voidvoid; /**< The function called once at the beginning of a heartbeat. */
static INT_VOID_F hb_get_timestamp = intvoid; /**< The function called to get the timestamp. */

/**
  Initializes the heartbeat.
  @note SCHED_SPEED should be defined in hardware_setup.h and is the schedule speed in kHz. If it is
  not, @ref hb_get_count will not work properly.
  @param period The time between heartbeats is given as 2^(period-1) milliseconds.
  @param func The function to call once at the beginning of a heartbeat.
  @param get_time A function which returns the current timestamp.
*/
void hb_init(int period, VOID_VOID_F func, INT_VOID_F get_time){
  hb_period = period;
  hb_function = func;
  hb_get_timestamp = get_time;
}

/**
  This function checks the given value and heartbeat state to call the heartbeat function
  at a transition.
  @param count The value to check against the period for a heartbeat.
*/
static void hb_update(int count){
  int new_state = count & (1 << (hb_period - 1));
  if (!hb_state && new_state){
    hb_function();
  } 
  hb_state = new_state;
}

/**
  Use this function in every row of the scheduler.
*/
void hb_beat(void){
  hb_update(hb_get_timestamp());
}

/**
  Use this function as a parameter to the get_timestamp function in
  @ref hb_init to make the heartbeat act according to the internal clock
  as opposed to an external clock.
  @return The current internal clock of the heartbeat module.
*/
int hb_get_count(void){
  static int hb_prescale = -1;
  static unsigned int hb_count = 0;
  hb_prescale++;
  if (hb_prescale >= SCHED_SPEED){
    hb_count++;
    hb_prescale = 0;
  }
  return hb_count;
}
