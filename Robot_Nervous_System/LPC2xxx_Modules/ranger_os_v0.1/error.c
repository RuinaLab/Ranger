/**

	@file error.c
  This module provides error reporting capabilities to other modules.
  
  During operation, errors that occur will set flags. Calls to @ref error_update
  will iterate through the flags and create @ref ERROR_INFO structs for any errors
  that have occurred and add them to a buffer. Finally, calls to @ref error_send_next
  will use the transmit function passed to the module at initialization to transmit or
  display the oldest error.
  
  @note All public functions are now protected if the user does not call @ref error_init first.
  The functions will either do nothing, or will return 0.
	
  @author Nicolas Williamson 
  @date Summer 2009
	
*/
#include <includes.h>

#ifndef __VERSION_0_1__
#warning RangerOS mismatch, expected v0.1. 
#endif


static volatile ERROR_BUFFER er_buffer; /**< The ring buffer for holding errors that have occurred */
static volatile ERROR_INFO* er_active; /**< The oldest error ready to be sent out. Its data can be accessed using @ref error_get_time and @ref error_get_info */
static VOID_VOID_F er_transmit = voidvoid; /**< The transmit function that will access the active error and transmit or display its info */
static INT_VOID_F er_get_timestamp = intvoid; /**< A function that returns the current timestamp */
static BOARD_ID er_board_id; /**< The BOARD_ID for this project's specific board. */
static int er_is_init = 0; /**< Whether or not this module has been initialized yet. 0 = no, 1 = yes. */
static const int er_length = ((int)(ERROR_LAST_ID)/32)+1; /**< The length of the array needed to hold all of the error flags */
static volatile unsigned int er_flags[er_length]; /**< The array holding the error flags. Every error has one flag which takes up a bit within this array of ints */

/**
  Initializes the error modules. Call this from software setup before any other error module functions.
  @param transmit_function A voidvoid function that will be called to report the next error over CAN or otherwise.
  @param timestamp_function A function which returns the current timestamp, usually @ref asched_get_timestamp.
  @param board_id The BOARD_ID for this project's board.
*/
void error_init(VOID_VOID_F transmit_function, INT_VOID_F timestamp_function, BOARD_ID board_id)
{
	er_buffer.first = 0;
	er_buffer.next = 0;
	er_buffer.overflows = 0;
	er_buffer.empty = 1;
  er_transmit = transmit_function;
  er_get_timestamp = timestamp_function;
  er_board_id = board_id;
  er_is_init = 1;
}

MUTEX er_mutex;
/**
  Alerts the error module that an error has occurred. 
  This function will be called by other modules whenever something
  bad or unexpected happens.
  @param error_code The ID of the error that occurred, of type ERROR_ID
*/
void error_occurred(ERROR_ID error_code)
{
  volatile int index, bit;
  if (er_is_init){//make sure error module has been initialized first
    volatile int index, bit;
    if (!mutex_check(&er_mutex)){
      mutex_lock(&er_mutex);
      index = (int)(error_code) / 32;
      bit = (int)(error_code) % 32;
      er_flags[index] |= (1<<bit);
   //   mcu_led_red_blink(100);
      mutex_unlock(&er_mutex);
    } else {
    }
  }
}

MUTEX er_irq_mutex;
/**
  Alerts the error module that an error has occurred.
  This function will be called by other modules whenever something
  bad or unexpected happens during an IRQ (Interrupt Request).
  @param error_code The ID of the error that occurred, of type ERROR_ID
*/
void error_occurred_irq(ERROR_ID error_code)
{
  volatile int index, bit;
  if (er_is_init){//make sure error module has been initialized first
    if (!mutex_check(&er_irq_mutex)){
      mutex_lock(&er_irq_mutex);
      index = (int)(error_code) / 32;
      bit = (int)(error_code) % 32;
      er_flags[index] |= (1<<bit);
   //   mcu_led_red_blink(100);
      mutex_unlock(&er_irq_mutex);
    } else {
    }
  }
}
MUTEX er_fiq_mutex;
/**
  Alerts the error module that an error has occurred.
  This function will be called by other modules whenever something
  bad or unexpected happens during an FIQ (Fast Interrupt Request).
  @param error_code The ID of the error that occurred, of type ERROR_ID
*/
void error_occurred_fiq(ERROR_ID error_code)
{
  volatile int index, bit;
  if (er_is_init){ //make sure error module has been initialized first
    if (!mutex_check(&er_fiq_mutex)){
      mutex_lock(&er_fiq_mutex);
      index = (int)(error_code) / 32;
      bit = (int)(error_code) % 32;
      er_flags[index] |= (1<<bit);
   //   mcu_led_red_blink(100);
      mutex_unlock(&er_fiq_mutex);
    } else {
    }
  }
}

/**
  This function iterates through and updates the list of errors
  that have occurred, putting the error data into @ref ERROR_INFO structs.
  @note @c error_update should be called with every tick of the scheduler.
*/
void error_update(void){
  int chunk, bit;
  int index;
  int found = 0;
  ERROR_ID error_code;
  volatile ERROR_BUFFER* buf;
  volatile ERROR_INFO new_error; 
  
  if (er_is_init){ //make sure error module has been initialized first
    for (chunk = 0; chunk < er_length; chunk++){
      if (er_flags[chunk]){
        for (bit = 0; bit < 32; bit++){
          if (er_flags[chunk] & (1<<bit)){
            error_code = (ERROR_ID)(bit + (32 * chunk));
          	
          	buf = &er_buffer;
          	
          	// ********* CHECK IF ERROR ALREADY OCCURRED ***********
          	if (!(buf->empty)){ //only traverse if the buffer has stuff in it
          		index = buf->first;
          		while (index != buf->next) { //traverse the buf of errors to see if this error has already been generated
          			if (buf->errors[index].error_id == error_code) { //we've had this error before since the last time we sent info
          				buf->errors[index].frequency++; //if it has, increase the frequency
          				found = 1;
          				break;
          			}
                if (index + 1 >= ERR_BUF_SIZE){
                  index = 0;
                } else {
                  index++;
                }
          		}
          	}
          	
          	// ********** ADD NEW ERROR TO BUFFER *********
          	if (!found) { //we haven't seen this error before, add it to the ring buffer
          		new_error.error_id = error_code;
          		new_error.frequency = 1;
          		new_error.time_occurred = er_get_timestamp(); //will hold actual value when sent out; this is just the initial time
              error_push(buf, new_error); //will new_error be out of scope here and be meaningless? TOMMY HELP!!
          	}	
          }
        }
        er_flags[chunk] = 0;
      }  
    } 
  } 
}

/**
  Pushes the error onto a ring buffer.
  @param buffer A pointer to the ring buffer
  @param new_error The new error that is being pushed onto the ring buffer
  @private
*/
static void error_push(volatile ERROR_BUFFER* buffer, volatile ERROR_INFO new_error)
{	
	// ****** CHECK FOR OVERFLOW ******
	if ((buffer->next == buffer->first) && !(buffer->empty)){ //the front has caught up to the tail!! it's eating itself!! nooooo
		buffer->overflows++;
    if (buffer->first + 1 >= ERR_BUF_SIZE){
      buffer->first = 0;
    } else {
      buffer->first++;
    }
	}
	// ****** ADD NEW DATA ******	
	buffer->errors[buffer->next] = new_error;
  if (buffer->next + 1 >= ERR_BUF_SIZE){
    buffer->next = 0;
  } else {
    buffer->next++;
  }
	buffer->empty = 0; //buffer now has stuff in it
}

/**
  Pops the oldest error from the ring buffer.
  @param buffer A pointer to the ring buffer
  @return A pointer to the oldest error information
  @private
*/ 
static volatile ERROR_INFO* error_pop(volatile ERROR_BUFFER* buffer)
{
	volatile ERROR_INFO* err = 0;
	if (!buffer->empty){
		err = &(buffer->errors[buffer->first]);
		if (buffer->first + 1 >= ERR_BUF_SIZE){
      buffer->first = 0;
    } else {
      buffer->first++;
    }
		buffer->overflows = 0;		
		if (buffer->first == buffer->next){
			buffer->empty = 1;
		}
	}	
	return err;
}

/**
  Gets the time from the active error.
  @return The time the error occurred
*/
int error_get_time(void)
{
  if (er_is_init){ //make sure error module has been initialized first
	  return (int)(er_active->time_occurred); 
  } else {
    return 0;
  }
}
/**
  Gets the info of the active error. The format for the int returned
  is:
  - Bits 0:15 (16 bits) = error_id
  - Bits 16:21 (6 bits) = board_id
  - Bits 22:31 (10 bits) = frequency (number of times the error has occurred since it was last transmitted
  @return The info of the current error, including id and frequency
*/
int error_get_info(void)
{
  if (er_is_init){ //make sure error module has been initialized first
    long int info = (er_active->error_id & 0xFFFF) | ((er_board_id & 0x3F) << 16) | ((er_active->frequency & 0x3FF) << 22);
  	return info;
  } else {
    return 0;
  }
}

/**
  Returns the BOARD_ID for this project's board.
  @return The board this project is for.
*/
BOARD_ID error_get_board(void)
{
  return er_board_id;
}

/**
  Returns the ERROR_ID for the current error.
  @return The error id of the active/current error.
*/
ERROR_ID error_get_id(void)
{
  if (er_is_init){ //make sure error module has been initialized first
    return (ERROR_ID)er_active->error_id;
  } else {
    return ERROR_DEFAULT;
  }
}

/**
  Returns the frequency of the active/current error.
  @return The frequency - number of times the error has occurred since it was last transmitted.
*/
int error_get_frequency(void)
{
  if (er_is_init){ //make sure error module has been initialized first
    return er_active->frequency;
  } else {
    return 0;
  }
}
 
/**
  Uses a transmit function to send out the next error.
  This function sets up the next error to be transmitted, then
  uses the @c error_transmit function that was passed at initialization to @ref error_init.
  The transmit function should use the @ref error_get_time and @ref error_get_info to fill up
  whatever communications packets or to display whatever information it needs.
*/
void error_send_next(void)
{
  if (er_is_init){ //make sure error module has been initialized first
    if (!er_buffer.empty){
      er_active = error_pop(&er_buffer);
      if (er_active != 0){
        er_transmit();
      }
    }
  }
}




