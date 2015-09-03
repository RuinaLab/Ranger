/**

	@file limit_switch.c
  
  Software limit switches. A limit switch will wait for 
  a given limit of consecutive high signals before turning high, and the same
  for turning low again. If a low signal interrupts a stream of high signals (or vice versa),
  the counter is reset.
  
  Example pin setup for Limit Switch (used by ankle controllers):
  @code	
  // *******************************************************************************
  // Limit Switch Setup                                            
  // *******************************************************************************  
  PINSEL0 &= ~(3<<0); //set P0.0 to GPIO
  PINSEL0 &= ~(3<<2); //set P0.1 to GPIO
  FIO0DIR &= ~(1<<0); //set P0.0 to input
  FIO0DIR &= ~(1<<1); //set P0.1 to input
  @endcode
    
	@author Nicolas Williamson
  @date July 2009
  
*/

#include <includes.h>

#define LS_NUM_SWITCHES (10) /**< The maximum number of limit switches. */

LIMIT_SWITCH ls_switches[LS_NUM_SWITCHES]; /**< The array of limit switches. */
int ls_count = 0; /**< The number of limit switches currently being used. */

/**
  Initializes a limit switch with the given limit and signal function.
  Call this function from software_setup for each limit switch.
  @param new_limit The limit of consecutive signals needed before output changes.
  @param funct A signal functino which returns 0 for low and non-zero for high.
  @return The id of the limit switch, or -1 if the switch failed to initialize.
*/
int ls_init_switch(unsigned int new_limit, INT_VOID_F funct)
{
  if (ls_count < LS_NUM_SWITCHES){
    int id = ls_count;
    LIMIT_SWITCH* ls = &(ls_switches[ls_count++]);
  	ls->limit = new_limit;
    ls->function = funct;
  	ls->count = 0;
  	ls->prev = LS_OFF;
  	ls->state = LS_OFF;
    return id;
  } else {
    error_occurred(ERROR_LS_NUM_SWITCH);
    return -1;
  }
}

/**
  Updates the state for every limit switch.
*/
void ls_update(void)
{
  int i;
  for (i = 0; i < ls_count; i++){
    LS_STATE new;
    LIMIT_SWITCH* ls = &(ls_switches[i]);
    INT_VOID_F get_state = ls->function;
    // ***** CHECK SWITCH ***** //
    if (get_state()){ //true, so switch ON
      new = LS_ON;
    } else {
      new = LS_OFF;
    }
    // ***** INCREMENT COUNT ***** //
    if (new == ls->prev){ //consistent values, increment count
      ls->count = ls->count + 1;
    } else { //different values, reset count
      ls->count = 0;
    }
    ls->prev = new;
    // ***** CHECK LIMIT ***** //
    if (ls->count >= ls->limit){ //we have reached the limit, set state
      ls->state = new;
      ls->count = 0;
    }
  }
}

/**
  Returns the state of the switch with the given id.
  The id of the switch was set by the @ref ls_init_switch function.
  The id is also the index of the switch within the array of switches,
  so it begins at 0 for the first switch, and increments with each switch 
  that is initialized.
  @param id The id of the switch.
  @return The output state of the switch.
*/
LS_STATE ls_get_state(int id)
{
  if (id < ls_count && id >= 0){
    LIMIT_SWITCH* ls = &(ls_switches[id]);
    return ls->state;
	} else {
    error_occurred(ERROR_LS_INVALID_ID);
    return LS_OFF;
  }
}


