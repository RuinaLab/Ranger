/*

	mb_error.c - code to store and periodically send errors from the main brain.
	
  Each individual error type should have a unique number. 
	Errors will also have frequency of occurrence since the last time


	the errors were sent, and timing.
	
	Nicolas Williamson - Summer 2009
  Modifications for main brain use, Jason Cortell - March 2010
	
*/
#include <mb_includes.h>

/*

	error.c - code to store and periodically send errors from
	satellite boards to the main brain.
	
	Each board should have a specific ID number ('BOARD_NAME'), as should each individual error type. 
	Errors will also have priorities, frequency of occurrence since the last time
	the errors were sent, and timing. An error of priority_low is barely a warning
	and usually only for logging purposes, and a priority_fatal means fatal error, probably on
	fire or about to explode (or has become self-aware and deemed humans a threat),
	shutdown immediately!
	
	Nicolas Williamson - Summer 2009
	
*/

//Define three input error ring buffers, one for each preemption level:
//normal, IRQ, and FIQ.
//These are simple ring buffers, safe for use with any one preemption level
//at each end.

static const unsigned short error_in_buf_size = 16;

static volatile ERROR_ID error_in_buf[error_in_buf_size];
static volatile unsigned short error_in_buf_write_idx = 0;
static volatile unsigned short error_in_buf_read_idx = 0;

static volatile ERROR_ID error_in_buf_irq[error_in_buf_size];
static volatile unsigned short error_in_buf_write_idx_irq = 0;
static volatile unsigned short error_in_buf_read_idx_irq = 0;

static volatile ERROR_ID error_in_buf_fiq[error_in_buf_size];
static volatile unsigned short error_in_buf_write_idx_fiq = 0;
static volatile unsigned short error_in_buf_read_idx_fiq = 0;

//Define one common error ring buffer for use with the combined error input data,
//with added time stamps and occurrence frequency.
//This buffer will be searched for duplicate errors and updated accordingly,
//and therefore is safe for only one preemption level (normal) at both ends.

static const unsigned short error_buf_size = 16;

static ERROR_INFO error_buf[error_buf_size];
static unsigned short error_buf_write_idx = 0;
static unsigned short error_buf_read_idx = 0;
static ERROR_INFO er_active;   //ERROR_INFO struct to store most recently popped error

static unsigned long (* er_get_timestamp)(void) = error_ulvoid;  //set to timestamp function in mb_error_init
static unsigned long er_board_id = 0;


//reads in current error and timestamp data, if any, and writes it into the DATA_FRAME pointed to in the input parameter.
//returns 0 if new error was written, 1 if not.
unsigned short mb_error_get_frame(DATA_FRAME * frameptr)
{
  if (!mb_error_pop(&er_active))
  {
    frameptr->payload.ulul.ul1 = (er_active.error_id) | (er_board_id << 16) | (er_active.frequency << 22);
    frameptr->payload.ulul.ul2 = er_active.timestamp;
    return 0;  // New error written into DATA_FRAME at frameptr  
  }
  else
  {
    er_active.error_id = ERROR_DEFAULT;
    er_active.frequency = 0;
    er_active.timestamp = er_get_timestamp();
    //frameptr->payload.ulul.ul1 = (er_active.error_id) | (er_board_id << 16) | (er_active.frequency << 22);
    //frameptr->payload.ulul.ul2 = er_active.timestamp;
    return 1; // Change to 0 if returning ERROR_DEFAULT
  }
}

void mb_error_init(unsigned long timestamp_function(void), BOARD_ID board_id)
{
 // er_transmit = transmit_function;
  er_get_timestamp = timestamp_function;
  er_board_id = board_id;
}

void mb_error_occurred(ERROR_ID error_code)
{
  int next_write_idx;

  next_write_idx = error_in_buf_write_idx;
  if (++next_write_idx == error_in_buf_size) {next_write_idx = 0;}
  if (next_write_idx == error_in_buf_read_idx) {return;} //buffer full

  error_in_buf[next_write_idx] = error_code;
  error_in_buf_write_idx = next_write_idx;
}

void mb_error_occurred_irq(ERROR_ID error_code)
{
  int next_write_idx;

  next_write_idx = error_in_buf_write_idx_irq;
  if (++next_write_idx == error_in_buf_size) {next_write_idx = 0;}
  if (next_write_idx == error_in_buf_read_idx_irq) {return;} //buffer full

  error_in_buf_irq[next_write_idx] = error_code;
  error_in_buf_write_idx_irq = next_write_idx;
}

void mb_error_occurred_fiq(ERROR_ID error_code)
{
  int next_write_idx;

  next_write_idx = error_in_buf_write_idx_fiq;
  if (++next_write_idx == error_in_buf_size) {next_write_idx = 0;}
  if (next_write_idx == error_in_buf_read_idx_fiq) {return;} //buffer full

  error_in_buf_fiq[next_write_idx] = error_code;
  error_in_buf_write_idx_fiq = next_write_idx;
}

void mb_error_update(void)
{
  unsigned short temp_read_idx, temp_write_idx;

  //Read in the normal-preemption level error codes:
  temp_read_idx = error_in_buf_read_idx;
  temp_write_idx = error_in_buf_write_idx;
  while (temp_read_idx != temp_write_idx)
  {
    mb_error_push(error_in_buf[temp_read_idx]);
    if (++temp_read_idx >= error_in_buf_size) {temp_read_idx = 0;}
  }
  error_in_buf_read_idx = temp_read_idx;

  //Read in the IRQ preemption level error codes:
  temp_read_idx = error_in_buf_read_idx_irq;
  temp_write_idx = error_in_buf_write_idx_irq;
  while (temp_read_idx != temp_write_idx)
  {
    mb_error_push(error_in_buf_irq[temp_read_idx]);
    if (++temp_read_idx >= error_in_buf_size) {temp_read_idx = 0;}
  }
  error_in_buf_read_idx_irq = temp_read_idx;

  //Read in the FIQ preemption level error codes:
  temp_read_idx = error_in_buf_read_idx_fiq;
  temp_write_idx = error_in_buf_write_idx_fiq;
  while (temp_read_idx != temp_write_idx)
  {
    mb_error_push(error_in_buf_fiq[temp_read_idx]);
    if (++temp_read_idx >= error_in_buf_size) {temp_read_idx = 0;}
  }
  error_in_buf_read_idx_fiq = temp_read_idx;
}

void mb_error_push(ERROR_ID new_error)
{
  unsigned short temp_index;

  //Search buffer for error matching incoming error ID
  temp_index = error_buf_read_idx;  // Initialize temp_index to point to oldest element of buffer 
  while (temp_index != error_buf_write_idx)
  {
    if (new_error == error_buf[temp_index].error_id)
    {
      //Error match found, increment frequency
      error_buf[temp_index].frequency++;
      return;
    }
    if (++temp_index >= error_buf_size) // Move to next buffer location
    {
      temp_index = 0;
    }
  }

  //Match not found, push new error onto buffer
  temp_index = error_buf_write_idx;  // Initialize temp_index to point to new write spot

  error_buf[temp_index].error_id = new_error;   // Copy error data
  error_buf[temp_index].timestamp = er_get_timestamp();   // Copy timestamp data
  error_buf[temp_index].frequency = 1;        //first error of this type
  if (++temp_index == error_buf_size) // Move to next buffer write location
  {
    temp_index = 0;
  }
  if (temp_index == error_buf_read_idx)
  {
    //error - buffer full; don't change write index; next error may overwrite current error
    return; // buffer full, operation unsuccessful
  }
  else
  {
    error_buf_write_idx = temp_index; // Advance write index
    return; // New error successfully pushed onto buffer
  }
}

unsigned short mb_error_pop(ERROR_INFO * error_ptr)
{
  if (error_buf_read_idx != error_buf_write_idx)
  {
    error_ptr->error_id = error_buf[error_buf_read_idx].error_id;   // Copy error data
    error_ptr->timestamp = error_buf[error_buf_read_idx].timestamp;   // Copy timestamp data
    error_ptr->frequency = error_buf[error_buf_read_idx].frequency;   // Copy frequency data
    
    // Advance to next read buffer location (should be ISR-safe with this construction)
    if (error_buf_read_idx >= (error_buf_size - 1))
    {
      error_buf_read_idx = 0;
    }
    else
    {
      ++error_buf_read_idx;
    }
    return 0; // Error successfully popped from ring buffer
  }
  else
  {
    return 1; // Error buffer is empty
  }	
}

unsigned long error_ulvoid(void)
{
  return 0;
}

