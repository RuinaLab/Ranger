/*
	mb_error.h - Header file for the error handling module
	
	Contains all of the error codes and functions for Main Brain error reporting.
	
	Nicolas Williamson - Summer 2009
  Modified by Jason Cortell for Main Brain use - March, April 2010
*/

#ifndef __MB_ERROR_H__
#define __MB_ERROR_H__

typedef struct error_info {
	ERROR_ID error_id; 
	unsigned short frequency;
	unsigned long timestamp;
} ERROR_INFO;
	
//Functions
unsigned short mb_error_get_frame(DATA_FRAME * frameptr);
void mb_error_init(unsigned long timestamp_function(void), BOARD_ID board_id);
void mb_error_voidvoid(void);
unsigned long mb_error_ulvoid(void);
void mb_error_occurred(ERROR_ID error_code);
void mb_error_occurred_irq(ERROR_ID error_code);
void mb_error_occurred_fiq(ERROR_ID error_code);
void mb_error_push(ERROR_ID new_error);
unsigned short mb_error_pop(ERROR_INFO * error_ptr);
void mb_error_update(void);
unsigned long error_ulvoid(void); //dummy callback function


#endif //__MB_ERROR_H__

