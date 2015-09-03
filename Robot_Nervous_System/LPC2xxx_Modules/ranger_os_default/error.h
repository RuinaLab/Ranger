/*
	error.h - Header file for the error handling module
	
	Contains all of the error codes and functions for error reporting over CAN.
	
	Nicolas Williamson - Summer 2009
	
*/

#ifndef __ROBOT_ERROR_H__
#define __ROBOT_ERROR_H__

#define ERR_BUF_SIZE 32
#define MED_RESERVE 4
#define LOW_RESERVE 8

//Error types/priority
typedef enum error_priorities {
  /** RUNTIME PRIORITIES **/
  PRIORITY_LOW = 0, //Ranger should log that this happened, but doesn't need to know
  PRIORITY_MED = 1, //Ranger might need to know about this, but probably fine
  PRIORITY_HIGH = 2, //Ranger needs to deal with this, something is seriously wrong
  /** CODE ERRORS **/
  PRIORITY_DEBUG = 3,
  PRIORITY_MAX
//  PRIORITY_FATAL = 3 //Ranger poses immediate danger to surrounding humans
} ERROR_PRIORITY;

typedef struct error_info {
	ERROR_ID error_id; 
	unsigned short frequency;
//	unsigned long long int time_occurred;
  unsigned long timestamp;
} ERROR_INFO;

typedef struct error_buffer{
	ERROR_INFO errors[ERR_BUF_SIZE];
	unsigned char first;
	unsigned char next;
	unsigned char overflows;
	unsigned char empty;
} ERROR_BUFFER;	


//Functions
void error_init(VOID_VOID_F f_transmit, INT_VOID_F f_time, BOARD_ID board_id);
void error_occurred(ERROR_ID error_code);
void error_occurred_irq(ERROR_ID error_code);
void error_occurred_fiq(ERROR_ID error_code);
void error_send_next(void);
//void error_push(ERROR_BUFFER* buffer, ERROR_INFO new_error);
void error_push(ERROR_ID new_error);
unsigned long error_get_time(void);
unsigned long error_get_info(void);
//volatile ERROR_INFO* error_pop(volatile ERROR_BUFFER* buffer);
unsigned short error_pop(ERROR_INFO * error_ptr);
void error_update(void);


#endif //__ROBOT_ERROR_H__

