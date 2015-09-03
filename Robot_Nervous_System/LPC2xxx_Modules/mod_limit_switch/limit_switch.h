/*
	limit_switch.h
	
	Nicolas Williamson - July 2009
	
*/

#ifndef __H_LIMIT_SWITCH__
#define __H_LIMIT_SWITCH__

/**
  The two possible states of the switch.
*/
typedef enum ls_states{
	LS_OFF = 0, /**< The low state. */
	LS_ON /**< The high state. */
} LS_STATE;

/**
  A limit switch object.
*/
typedef struct ls_switch{
	unsigned int limit; /**< The limit to switch states. */
	unsigned int count; /**< The current count of consecutively high or low signals. */
	LS_STATE prev; /**< The previous state of the signal. */
	LS_STATE state; /**< The current state of the signal. */
  INT_VOID_F function; /**< The signal function which returns zero for off/low and non-zero for on/high. */
} LIMIT_SWITCH;

// ***** Functions ***** //
LS_STATE ls_get_state(int ls);
int ls_init_switch(unsigned int new_limit, INT_VOID_F funct);
void ls_update(void);


/* Hardware Setup
	PINSEL0 &= ~(3<<0); //set P0.0 to GPIO
	PINSEL0 &= ~(3<<2); //set P0.1 to GPIO
	FIO0DIR &= ~(1<<0); //set P0.0 to input
	FIO0DIR &= ~(1<<1); //set P0.1 to input
*/

#endif // __H_LIMIT_SWITCH__
