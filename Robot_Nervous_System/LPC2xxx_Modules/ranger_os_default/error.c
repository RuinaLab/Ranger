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
#include <includes.h>

/* 
	The data for this module is stored in two types of structs - the ERROR struct and the ERRORS struct.
	The ERROR struct holds information for a specific error, such as its error_code, the frequency it occurs since
	the last sending of error messages, and the time_elapsed (in milliseconds) since the error first occurred in some code.
	The ERRORS struct holds up to ERR_BUF_SIZE ERRORs of the same priority in a ring buffer. It also holds info necessary to implement
	a ring buffer, such as the first error and the next spot to place a new error. It should only hold errors with
	distinct error codes; if an error occurs that has already occurred then the error's frequency should increase.
*/

//Define three input error ring buffers, one for each preemption level:
//normal, IRQ, and FIQ.
//These are simple ring buffers, safe for use with any one preemption level
//at each end.

const unsigned short error_in_buf_size = 4;

volatile ERROR_ID error_in_buf[error_in_buf_size];
volatile unsigned short error_in_buf_write_idx = 0;
volatile unsigned short error_in_buf_read_idx = 0;

volatile ERROR_ID error_in_buf_irq[error_in_buf_size];
volatile unsigned short error_in_buf_write_idx_irq = 0;
volatile unsigned short error_in_buf_read_idx_irq = 0;

volatile ERROR_ID error_in_buf_fiq[error_in_buf_size];
volatile unsigned short error_in_buf_write_idx_fiq = 0;
volatile unsigned short error_in_buf_read_idx_fiq = 0;

//Define one common error ring buffer for use with the combined error input data,
//with added time stamps and occurrence frequency.
//This buffer will be searched for duplicate errors and updated accordingly,
//and therefore is safe for only one preemption level (normal) at both ends.

const unsigned short error_buf_size = 16;

ERROR_INFO error_buf[error_buf_size];
unsigned short error_buf_write_idx = 0;
unsigned short error_buf_read_idx = 0;
ERROR_INFO er_active;   //ERROR_INFO struct to store most recently popped error

/*
//The three ring-buffer queues for the errors of different priorities
static volatile ERROR_BUFFER er_buffer; //accessed by their priority, ex. er_buffers[PRIORITY_LOW];
static volatile ERROR_INFO* er_active;
*/

VOID_VOID_F er_transmit = voidvoid; //will need to define this function in data_nexus
INT_VOID_F er_get_timestamp = intvoid;
unsigned long er_board_id = 0;

/*
static const int er_length = ((int)(ERROR_LAST_ID)/32)+1;
static volatile unsigned int er_flags[er_length];
*/

/*
void error_init(VOID_VOID_F transmit_function, INT_VOID_F timestamp_function)
{
	er_buffer.first = 0;
	er_buffer.next = 0;
	er_buffer.overflows = 0;
	er_buffer.empty = 1;
  er_transmit = transmit_function;
  er_get_timestamp = timestamp_function;
}
*/

void error_init(VOID_VOID_F transmit_function, INT_VOID_F timestamp_function, BOARD_ID board_id)
{
  er_transmit = transmit_function;
  er_get_timestamp = timestamp_function;
  er_board_id = board_id;
}

/*
MUTEX er_mutex;
void error_occurred(ERROR_ID error_code)
{
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
MUTEX er_irq_mutex;
void error_occurred_irq(ERROR_ID error_code)
{
  volatile int index, bit;

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
MUTEX er_fiq_mutex;
void error_occurred_fiq(ERROR_ID error_code)
{
  volatile int index, bit;
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
*/

void error_occurred(ERROR_ID error_code)
{
  int next_write_idx;

  next_write_idx = error_in_buf_write_idx;
  if (++next_write_idx == error_in_buf_size) {next_write_idx = 0;}
  if (next_write_idx == error_in_buf_read_idx) {return;} //buffer full

  error_in_buf[next_write_idx] = error_code;
  error_in_buf_write_idx = next_write_idx;
}

void error_occurred_irq(ERROR_ID error_code)
{
  int next_write_idx;

  next_write_idx = error_in_buf_write_idx_irq;
  if (++next_write_idx == error_in_buf_size) {next_write_idx = 0;}
  if (next_write_idx == error_in_buf_read_idx_irq) {return;} //buffer full

  error_in_buf_irq[next_write_idx] = error_code;
  error_in_buf_write_idx_irq = next_write_idx;
}

void error_occurred_fiq(ERROR_ID error_code)
{
  int next_write_idx;

  next_write_idx = error_in_buf_write_idx_fiq;
  if (++next_write_idx == error_in_buf_size) {next_write_idx = 0;}
  if (next_write_idx == error_in_buf_read_idx_fiq) {return;} //buffer full

  error_in_buf_fiq[next_write_idx] = error_code;
  error_in_buf_write_idx_fiq = next_write_idx;
}

void error_update(void)
{
  unsigned short temp_read_idx, temp_write_idx;

  //Read in the normal-preemption level error codes:
  temp_read_idx = error_in_buf_read_idx;
  temp_write_idx = error_in_buf_write_idx;
  while (temp_read_idx != temp_write_idx)
  {
    error_push(error_in_buf[temp_read_idx]);
    if (++temp_read_idx >= error_in_buf_size) {temp_read_idx = 0;}
  }
  error_in_buf_read_idx = temp_read_idx;

  //Read in the IRQ preemption level error codes:
  temp_read_idx = error_in_buf_read_idx_irq;
  temp_write_idx = error_in_buf_write_idx_irq;
  while (temp_read_idx != temp_write_idx)
  {
    error_push(error_in_buf_irq[temp_read_idx]);
    if (++temp_read_idx >= error_in_buf_size) {temp_read_idx = 0;}
  }
  error_in_buf_read_idx_irq = temp_read_idx;

  //Read in the FIQ preemption level error codes:
  temp_read_idx = error_in_buf_read_idx_fiq;
  temp_write_idx = error_in_buf_write_idx_fiq;
  while (temp_read_idx != temp_write_idx)
  {
    error_push(error_in_buf_fiq[temp_read_idx]);
    if (++temp_read_idx >= error_in_buf_size) {temp_read_idx = 0;}
  }
  error_in_buf_read_idx_fiq = temp_read_idx;
}

/*
void error_update(void){
  int chunk, bit;
  int index;
  int found = 0;
  ERROR_ID error_code;
  volatile ERROR_BUFFER* buf;
  volatile ERROR_INFO new_error;
 
  for (chunk = 0; chunk < er_length; chunk++){
    if (er_flags[chunk]){
      for (bit = 0; bit < 32; bit++){
        if (er_flags[chunk] & (1<<bit)){
        	buf = &er_buffer;
        	error_code = (ERROR_ID)(bit + (32 * chunk));
        	// ********* CHECK IF ERROR ALREADY OCCURRED ***********
        	if (!(buf->empty)){ //only traverse if the buffer has stuff in it
        		index = buf->first;
        		while (index != buf->next) { //traverse the buf of errors to see if this error has already been generated
              if (buf->errors[index].error_id == error_code) { //we've had this error before since the last time we sent info
                buf->errors[index].frequency++; //if it has, increase the frequency
        				found = 1;
        				break;
        			}
              // ISR-safe buffer increment
        			if (index >= ERR_BUF_SIZE - 1)
                {index = 0;}
              else
              {
                index++;
              }
        		}
        	}
        	
        	// ********** ADD NEW ERROR TO BUFFER *********
        	if (!found) { //we haven't seen this error before, add it to the ring buffer
        		new_error.error_id = error_code;
        		new_error.frequency = 1;
        		new_error.time_occurred = er_get_timestamp(); //will hold actual value when sent out; this is just the initial time
            error_push(buf, new_error); //will new_error be out of scope here and be meaningless? TOMMY HELP!!E
        	}	
        }
      }
      er_flags[chunk] = 0;
    }  
  }  
}

*/

void error_push(ERROR_ID new_error)
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

/*
static void error_push(volatile ERROR_BUFFER* buffer, ERROR_INFO new_error)
{	
	// ****** CHECK FOR OVERFLOW ******
	if ((buffer->next == buffer->first) && !(buffer->empty)){ //the front has caught up to the tail!! it's eating itself!! nooooo
		buffer->overflows++;
    // ISR-safe buffer increment
		if (buffer->first >= ERR_BUF_SIZE - 1) 
    { 
      buffer->first = 0; 
    } 
    else
    {
      buffer->first++;
    }
	}
	// ****** ADD NEW DATA ******	
	buffer->errors[buffer->next] = new_error;
	//increment the index to place the next error	
	//ISR-safe buffer increment
  if (buffer->next >= ERR_BUF_SIZE - 1) 
  {
    buffer->next = 0; 
  }
  else
  {
    buffer->next++;
  }
	buffer->empty = 0; //buffer now has stuff in it
}
*/

/*
static volatile ERROR_INFO* error_pop(volatile ERROR_BUFFER* buffer)
{
	volatile ERROR_INFO* err = 0;

	if (!buffer->empty){
		err = &(buffer->errors[buffer->first]);
		if (buffer->first >= ERR_BUF_SIZE - 1) 
    { 
      buffer->first = 0; 
    }
    else
    {
      buffer->first++;
    }
		buffer->overflows = 0;		
		if (buffer->first == buffer->next){
			buffer->empty = 1;
		}
	}	
	return err;
}
*/

unsigned short error_pop(ERROR_INFO * error_ptr)
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

//CAN getter functions
unsigned long error_get_time(void)
{
	return (int)(er_active.timestamp);	
}
unsigned long error_get_info(void)
{
  long int info = (er_active.error_id) | (er_board_id << 16) | (er_active.frequency << 22);
	return info;
}

/*
//CAN getter functions
int error_get_time(void)
{
	return (int)(er_active->time_occurred);	
}
int error_get_info(void)
{
  long int info = (er_active->error_id) | (er_active->frequency << 15);
	return info;
}
*/

void error_send_next(void)
{
  if (!error_pop(&er_active))
  {
    er_transmit();  
  }
  else
  {
    er_active.error_id = ERROR_DEFAULT;
    er_active.frequency = 0;
    er_active.timestamp = er_get_timestamp();
    //er_transmit();     // Sends ERROR_DEFAULT if no other error found (if desired)
  }
}

/*
//sends the next error 
void error_send_next(void)
{
  if (!er_buffer.empty){
    er_active = error_pop(&er_buffer);
    if (er_active != 0){
      er_transmit();
    }
  }

*/

/*	static unsigned long int low_count = 1, med_count = 1; //counts to keep track of when a low/med priority should be sent
	volatile ERROR_BUFFER* buffer = NULL;
	unsigned int low_flag = (low_count >= LOW_RESERVE);
	unsigned int med_flag = (med_count >= MED_RESERVE);
	int low_empty = er_buffers[PRIORITY_LOW].empty;
	int med_empty = er_buffers[PRIORITY_MED].empty;
	int high_empty = er_buffers[PRIORITY_HIGH].empty;
  
	if (low_flag && !low_empty){
		buffer = &er_buffers[PRIORITY_LOW];
		low_count = 0;
	} else if (med_flag && !med_empty){
		buffer = &er_buffers[PRIORITY_MED];
		med_count = 0;
	} else if (!high_empty){
		buffer = &er_buffers[PRIORITY_HIGH];
	} else if (!med_empty){
		buffer = &er_buffers[PRIORITY_MED];
		med_count = 0;
	} else if (!low_empty){
		buffer = &er_buffers[PRIORITY_LOW];
		low_count = 0;
	} else {
		buffer = NULL;
	}
	
	if (buffer != NULL){
		er_active = error_pop(buffer);
		
		if(er_active != 0){ //make sure there was something in the buffer
      er_transmit();
    }
	
		low_count++;
		med_count++;
	}
}  */




