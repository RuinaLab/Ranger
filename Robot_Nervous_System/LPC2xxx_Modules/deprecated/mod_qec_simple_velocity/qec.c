/**

	@file qec.c
  A Quadrature Encoder Controller.
	
  Each quadrature encoder on a motor is represented by a QEC_DATA struct. The positive direction is
	defined as counter-clockwise when looking at the motor from the front. 
	
	A value of QEC_NULL in initialization will ignore both channels A and B, but the interrupts
  will continue to occur unless they are disabled in Hardware Setup.
  
  A value of QEC_2X will ignore channel B interrupts, but similarly, interrupts for that channel will need to be
  disabled in Hardware Setup for that channel to take full advantage of the time savings.
  
  A value of QEC_4X will use values from both channels, and the interrupts will need to be setup correctly.
  
  This module uses Timer 0, but can be setup to run the scheduler if @ref asched_tick is passed to the overflow function.
  
	@author Nicolas Williamson and Thomas Craig 
  @date June 2009
  @version 0.2

*/

#include <includes.h>

// Global Variables
#define QEC_BUFFER_LENGTH 8
static volatile QEC_DATA qec_data1; /**< The encoder data for the first encoder - J3. */
static volatile QEC_DATA qec_data2; /**< The encoder data for the second encoder - J9. */
static VOID_VOID_F qec_overflow_funct = voidvoid; /**< The function to call when a timer match and reset occurs. */
static unsigned int qec_match_ticks = 0;
static unsigned int qec_prev_ticks = 0;
static float qec_ticks_per_sec = 60000000.0f; //(float)1.0/60000000.0; sec/count

/**
  Initializes the quadrature encoder module. Encoders are enabled with the given resolution,
  and the overflow function will be called whenever there is a match and reset on match 0 of the timer.
  @param enc_res_1 The desired resolution for encoder 1, either NULL, 2X, or 4X.
  @param enc_res_2 The desired resolution for encoder 2, either NULL, 2X, or 4X.
  @param function This function will be called when a timer reset occurs.
*/
void qec_init(QEC_RESOLUTION enc_res_1, QEC_RESOLUTION enc_res_2, VOID_VOID_F function)
{
	qec_init_data(&qec_data1, enc_res_1);
	qec_init_data(&qec_data2, enc_res_2);
  qec_overflow_funct = function;
  qec_match_ticks = T0MR0;
}

/**
  Initializes an encoder's data. 
  @param data A pointer to the encoder data struct.
  @param time_buf The time buffer.
  @param pos_buf The position buffer.
  @param length The length of both buffers.
  @param resolution The resolution desired for the encoder.
*/
static void qec_init_data(volatile QEC_DATA* data, QEC_RESOLUTION resolution)
{
	data->current_pos = 0;
	data->resolution = resolution;
}


/**
  The interrupt service routine for the qec module used by Timer 0.
  Called whenever there is an edge in channel A or B, or timer 0 matched and was reset.
*/
void qec_isr(void) //__irq
{
  unsigned int count = T0TC;
	unsigned int flags = T0IR; //save the interrupt flags
	unsigned int pins = FIO0PIN;  //save the pin state
	int flag_overflow = flags & (1<<0);
	int flag_cap0 = flags & (1<<4); 
	int flag_cap1 = flags & (1<<5);
	int flag_cap2 = flags & (1<<6);
	int flag_cap3 = flags & (1<<7);
	int chA1 = pins & (1<<22); //encoder 1
	int chB1 = pins & (1<<27); //""
	int chA2 = pins & (1<<16); //encoder 2
	int chB2 = pins & (1<<29); //""
	
	//Clear interrupt register flags
	T0IR = flag_overflow | flag_cap0 | flag_cap1 | flag_cap2 | flag_cap3;
	
	//Only update information for an encoder struct that exists
	if (qec_data1.resolution != QEC_NULL){
		qec_update(&qec_data1, flag_cap0, flag_cap1, chA1, chB1);
	}
	if (qec_data2.resolution != QEC_NULL){
		qec_update(&qec_data2, flag_cap2, flag_cap3, chA2, chB2);
	} 	
  if (flag_overflow){
    qec_overflow_funct();
  }
}

/**
  Returns the absolute position of the given encoder.
  @param id The id of the encoder, either QEC_1 (J3) or QEC_2 (J9).
  @return The absolute position in encoder counts.
*/
int qec_get_abs_pos(QEC_ID id)
{
	switch (id) {
		case QEC_1: return qec_data1.current_pos;
		case QEC_2: return qec_data2.current_pos;
		default: error_occurred(ERROR_QEC_BAD_ID); return 0;
	}
}

/**
  MUST call this every schedule tick.
*/
void qec_update_velocity(void){
  unsigned int current_ticks = T0TC;
  int ticks_elapsed = qec_match_ticks + (current_ticks - qec_prev_ticks);
  int delta_pos;
  qec_prev_ticks = current_ticks;
	if (qec_data1.resolution != QEC_NULL){
    delta_pos = qec_data1.current_pos - qec_data1.prev_pos;
    qec_data1.prev_pos = qec_data1.current_pos;
	  qec_data1.velocity = (delta_pos * qec_ticks_per_sec) / (float)ticks_elapsed;
	}	
  if (qec_data2.resolution != QEC_NULL){
    delta_pos = qec_data2.current_pos - qec_data2.prev_pos;
    qec_data2.prev_pos = qec_data2.current_pos;
	  qec_data2.velocity = (delta_pos * qec_ticks_per_sec) / (float)ticks_elapsed;
	}
}

/**
  Returns the velocity, in counts/sec, of the given encoder.
  @param if The id of the encoder to read from, either QEC_1 (J3) or QEC_2 (J9).
  @return The velocity of the encoder in counts/sec.
*/
float qec_get_velocity(QEC_ID id)
{
  switch (id) {
		case QEC_1: return qec_data1.velocity;
		case QEC_2: return qec_data2.velocity;
		default: error_occurred(ERROR_QEC_BAD_ID); return 0.0;
	}
}

/**
  Returns 1 if the motor has stopped (time since last pulse was 2 ms),
  0 otherwise.
  @param id The encoder to check.
  @return 1: motor stopped, 0: motor moving
*/
int qec_is_stopped(QEC_ID id){
  return (qec_get_velocity(id) < 0.2); //0.38 from old version
}

/**
  Updates the given encoder with data from the timer. Called from @ref qec_isr for both encoders, if applicable.
  @param data A pointer to the encoder's data struct.
  @param flag_overflow Whether a match and reset has occurred on the timer.
  @param flag_chA Whether a capture interrupt occurred on channel A.
  @param flag_chB Whether a capture interrupt occurred on channel B.
  @param cap_chA The capture value for channel A.
  @param cap_chB The capture value for channel B.
  @param chA The state (1 = hi, 0 = lo) of channel A.
  @param chB The state of channel B.
*/
static void qec_update(volatile QEC_DATA* data, int flag_chA, int flag_chB, int chA, int chB)
{
	int delta_pos;

	//if resolution only 2x, ignore channel B
	if (data->resolution == QEC_2X){
		flag_chB = 0;
	}
  
  //both channels can't interrupt this fast; that would suck
	if (flag_chA && flag_chB){
		error_occurred(ERROR_QEC_BOTH_CHS);
	}

	// If captures occurred
	if (flag_chA){
    if (data->prev_chA == chA){ //we are back even though we had an interrupt, MISSED COUNT!
      error_occurred(ERROR_QEC_MISSED_A); //missed a count on channel A
    } else {
  		if (chA){ //A high
  			if (chB) { //B high (B & A & fA)
  				delta_pos = -1;	
  			} else { //B low (!B & A & fA)
  				delta_pos = 1;
  			}
  		} else { //A low
  			if (chB) { //B high (B & !A & fA)
  				delta_pos = 1;
  			} else { //B low (!B & !A & fA)
  				delta_pos = -1;
  			} 	
  		} 
  		data->current_pos += delta_pos;
      data->prev_chA = chA;
    }
	} else if (flag_chB){
    if (data->prev_chB == chB){ //missed count
  	  error_occurred(ERROR_QEC_MISSED_B);
    } else {
    	if (chA){ //A high
  			if (chB) { //B high
  				delta_pos = 1;	
  			}else { //B low
  				delta_pos = -1;
  			}
  		} else { //A low
  			if (chB) { //B high
  				delta_pos = -1;
  			} else { //B low
  				delta_pos = 1;
  			} 	
  		} 
  		data->current_pos += delta_pos;
      data->prev_chB = chB;
    }
	}
	
}	











