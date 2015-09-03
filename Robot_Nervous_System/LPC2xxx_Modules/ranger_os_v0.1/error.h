/*
	@file error.h 
  Header file for the error handling module
	
	Nicolas Williamson - Summer 2009
	
*/

#ifndef __ROBOT_ERROR_H__
#define __ROBOT_ERROR_H__

/** The size of the ring buffer, used here for static memory allocation */
#define ERR_BUF_SIZE 32

/**
  The priorities of the errors. 
  @deprecated
*/
typedef enum error_priorities {
  PRIORITY_LOW = 0, /**< Ranger should log that this happened, but doesn't need to know */
  PRIORITY_MED = 1, /**< Ranger might need to know about this, but probably fine */
  PRIORITY_HIGH = 2, /**< Ranger needs to deal with this, something is seriously wrong */
  PRIORITY_DEBUG = 3,
  PRIORITY_MAX
//  PRIORITY_FATAL = 3 //Ranger poses immediate danger to surrounding humans
} ERROR_PRIORITY;

/**
  An error. Includes the error's id, the frequency of its occurrence since it was last
  transmitted, and the time when the error occurred.
*/
typedef struct error_info {
	ERROR_ID error_id; /**< The id of the error. */
	unsigned short frequency; /**< The number of times the error has occurred since it was last sent out. */
	unsigned long long int time_occurred; /**< The time the error occurred. */
} ERROR_INFO;

/**
  A ring buffer for errors. 
  Holds errors until they are transmitted.
  If newer errors begin replacing old errors, this information is 
  added to @c overflows. This value is set back to 0 once an error is popped
  and the buffer is no longer completely full.
*/
typedef struct error_buffer{
	ERROR_INFO errors[ERR_BUF_SIZE]; /**< The array for holding the errors. */
	unsigned char first; /**< Points to the first/oldest error added to the buffer. */
	unsigned char next; /**< Points to the front of the buffer, the slot where the next error will be added. */
	unsigned char overflows; /**< The number of lost errors caused by overflows. */
	unsigned char empty; /**< 0: The buffer is not empty, 1: the buffer is empty. */
} ERROR_BUFFER;	


//Initialization
void error_init(VOID_VOID_F f_transmit, INT_VOID_F f_time, BOARD_ID board_id);
//Errors
void error_occurred(ERROR_ID error_code);
void error_occurred_irq(ERROR_ID error_code);
void error_occurred_fiq(ERROR_ID error_code);
void error_update(void);
//Transmit
void error_send_next(void);
int error_get_time(void);
int error_get_info(void);
int error_get_frequency(void);
ERROR_ID error_get_id(void);
BOARD_ID error_get_board(void);
//Ring Buffers - Private
static void error_push(volatile ERROR_BUFFER* buffer, volatile ERROR_INFO new_error);
static volatile ERROR_INFO* error_pop(volatile ERROR_BUFFER* buffer);


#endif //__ROBOT_ERROR_H__

