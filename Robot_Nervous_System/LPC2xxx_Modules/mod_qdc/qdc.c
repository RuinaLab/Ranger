#include <includes.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup of the QDC module is done with the qdc_tmrx_init functions.
//One exception - if you need to change the pulse time buffer size, use a #define statement
//in software_setup.h or calibration.h (for example). Default values are defined below:

#ifndef QDC_TMR0_CAP01_BUFFER_SIZE              //Maximum number of data points captured by
#define QDC_TMR0_CAP01_BUFFER_SIZE 64           //qdc interrupt and available for velocity estimation later.
#endif

#ifndef QDC_TMR0_CAP23_BUFFER_SIZE              //Maximum number of data points captured by
#define QDC_TMR0_CAP23_BUFFER_SIZE 64           //qdc interrupt and available for velocity estimation later.
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Data struct used for encoder pulse counting and timing, along with QDC initialization.
//Order of fields used by the assembly code (the first 15) is important - 
//do not change, any modifications will require changes to the low-level code.
//New fields can be added at the lower levels of the struct as needed.
volatile QDC_DATA qdc_tmr0_data;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Define QDC_TMR0_CAP01 pulse time ring buffer for transmission of time data from
//assembly code ISR to normal preemption level for further processing
volatile signed long qdc_tmr0_cap01_buffer[QDC_TMR0_CAP01_BUFFER_SIZE];   //Ring buffer
//At initialization, qdc_cap01_data.write_index = 1 and qdc_cap01_data.read_index = 0
//In normal operation, data is read from the buffer at the read_index pointer and written at the
//write index pointer, advancing the index afterward in each case. Check for buffer-full condition, true when
//qdc_cap01_data.write_index = qdc_cap01_data.read_index, before writing. Check for buffer-empty condition, true
//when read pointer is one before the write pointer, before reading.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Define QDC_TMR0_CAP23 pulse time ring buffer  for transmission of time data from
//assembly code ISR to normal preemption level for further processing
volatile signed long qdc_tmr0_cap23_buffer[QDC_TMR0_CAP23_BUFFER_SIZE];   //Ring buffer
//At initialization, qdc_cap01_data.write_index = 1 and qdc_cap01_data.read_index = 0
//In normal operation, data is read from the buffer at the read_index pointer and written at the
//write index pointer, advancing the index afterward in each case. Check for buffer-full condition, true when
//qdc_cap01_data.write_index = qdc_cap01_data.read_index, before writing. Check for buffer-empty condition, true
//when read pointer is one before the write pointer, before reading.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void qdc_tmr0_cap01_set_angle_float(float new_angle)
{
  union convert
  {
    unsigned long ul;
    float         fl;
  } converter;

    long long new_count;

  converter.fl = new_angle;   //(float)qdc_tmr0_data.cap01_pos_fixed;

  if (converter.fl != 0)
  {
    converter.ul += (16 << 23); //Multiply floating point value by 2^16 directly from IEEE-754 bit layout,
                                //in preparation for 16.16 fixed-point conversion
  }

  new_count = (signed long)converter.fl;            //Convert float to 16.16 fixed point
  new_count <<= qdc_tmr0_data.cap01_pos_scaling;  //Scale up in preparation for integer divide
  
  //Set to encoder count to new value
  qdc_tmr0_data.encoder_count_01 = new_count/qdc_tmr0_data.cap01_pos_gain;
  //Note: this long-long integer divide is not likely to be very fast. This functions is intended to only
  //be called at rare intervals - e.g., at startup. - when speed is not critical. If speed is critical, add more
  //fields to the data struct and generate another scaling factor and gain to go this direction, but inverted so
  //you need only multiply.   
}

void qdc_tmr0_cap01_set_angle_fixed(signed long new_angle)
{
  long long new_count;

  new_count = new_angle;            //Convert float to 16.16 fixed point
  new_count <<= qdc_tmr0_data.cap01_pos_scaling;  //Scale up in preparation for integer divide
  
  //Set encoder count to new value
  qdc_tmr0_data.encoder_count_01 = new_count/qdc_tmr0_data.cap01_pos_gain;
  //Note: this long-long integer divide is not likely to be very fast. This function is intended to only
  //be called at rare intervals - e.g., at startup. - when speed is not critical. If speed is critical, add more
  //fields to the data struct and generate another scaling factor and gain to go this direction, but inverted so
  //you need only multiply.   
}

void qdc_tmr0_cap23_set_angle_float(float new_angle)
{
  long long new_count;

  union convert
  {
    unsigned long ul;
    float         fl;
  } converter;

  converter.fl = new_angle;

  if (converter.fl != 0)
  {
    converter.ul += (16 << 23); //Multiply floating point value by 2^16 directly from IEEE-754 bit layout,
                                //in preparation for 16.16 fixed-point conversion
  }

  new_count = (signed long)converter.fl;            //Convert float to 16.16 fixed point
  new_count <<= qdc_tmr0_data.cap23_pos_scaling;  //Scale up in preparation for integer divide
  
  //Set to encoder count to new value
  qdc_tmr0_data.encoder_count_23 = new_count/qdc_tmr0_data.cap23_pos_gain;
  //Note: this long-long integer divide is not likely to be very fast. This functions is intended to only
  //be called at rare intervals - e.g., at startup. - when speed is not critical. If speed is critical, add more
  //fields to the data struct and generate another scaling factor and gain to go this direction, but inverted so
  //you need only multiply. 
}

void qdc_tmr0_cap23_set_angle_fixed(signed long new_angle)
{
  long long new_count;

  new_count = new_angle;            //Convert float to 16.16 fixed point
  new_count <<= qdc_tmr0_data.cap23_pos_scaling;  //Scale up in preparation for integer divide
  
  //Set encoder count to new value
  qdc_tmr0_data.encoder_count_23 = new_count/qdc_tmr0_data.cap23_pos_gain;
  //Note: this long-long integer divide is not likely to be very fast. This function is intended to only
  //be called at rare intervals - e.g., at startup. - when speed is not critical. If speed is critical, add more
  //fields to the data struct and generate another scaling factor and gain to go this direction, but inverted so
  //you need only multiply.   
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
signed long qdc_tmr0_cap01_get_angle_fixed(void)
{
  return qdc_tmr0_data.cap01_pos_fixed;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float qdc_tmr0_cap01_get_angle_float(void)
{
  union convert
  {
    unsigned long ul;
    float         fl;
  } converter;

  converter.fl = (float)qdc_tmr0_data.cap01_pos_fixed;

  if (converter.fl != 0)
  {
    converter.ul -= (16 << 23); //Divide floating point value by 2^16 directly from IEEE-754 bit layout
  }
  
  return converter.fl;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
signed long qdc_tmr0_cap01_get_rate_fixed(void)
{
  return qdc_tmr0_data.cap01_vel_fixed;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float qdc_tmr0_cap01_get_rate_float(void)
{
  union convert
  {
    unsigned long ul;
    float         fl;
  } converter;

  converter.fl = (float)qdc_tmr0_data.cap01_vel_fixed;

  if (converter.fl != 0)
  {
    converter.ul -= (16 << 23); //Divide floating point value by 2^16 directly from IEEE-754 bit layout
  }

  return converter.fl;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
signed long qdc_tmr0_cap23_get_angle_fixed(void)
{
  return qdc_tmr0_data.cap23_pos_fixed;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float qdc_tmr0_cap23_get_angle_float(void)
{
  union convert
  {
    unsigned long ul;
    float         fl;
  } converter;

  converter.fl = (float)qdc_tmr0_data.cap23_pos_fixed;

  if (converter.fl != 0)
  {
    converter.ul -= (16 << 23); //Divide floating point value by 2^16 directly from IEEE-754 bit layout
  }
  
  return converter.fl;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
signed long qdc_tmr0_cap23_get_rate_fixed(void)
{
  return qdc_tmr0_data.cap23_vel_fixed;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float qdc_tmr0_cap23_get_rate_float(void)
{
  union convert
  {
    unsigned long ul;
    float         fl;
  } converter;

  converter.fl = (float)qdc_tmr0_data.cap23_vel_fixed;

  if (converter.fl != 0)
  {
    converter.ul -= (16 << 23); //Divide floating point value by 2^16 directly from IEEE-754 bit layout
  }

  return converter.fl;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void qdc_tmr0_cap01_rate_update(void)
{
  static signed long prev_sign = 0, glitch_flag = 0;
  signed long new_sign, new_sign2;
  signed long N = 0, j = 0;
  signed long temp_read_index, temp_read_index2, temp_write_index;
  signed long total_time = 0;
  static signed long time_since_pulse = 0;
  static signed long prev_time = 0x7FFFFFFF;
  static signed long long av_rate = 0;
  signed long long rate;
  signed long filter_rate;

  if (qdc_tmr0_data.write_index_01 == qdc_tmr0_data.read_index_01)
  {
    error_occurred(ERROR_QDC_CH01_BUF_F);  
  }
  
  temp_write_index = qdc_tmr0_data.write_index_01;  
  while (1)
  {
    //qdc_tmr0_data.read_index_01 points to the most recently read data value.
    //Increment the index to point to the oldest unread data value, if any.
    temp_read_index = (qdc_tmr0_data.read_index_01 + 1) & (QDC_TMR0_CAP01_BUFFER_SIZE - 1);
    if (temp_read_index == temp_write_index)    //Buffer is empty
    {
      break;
    }
    
    //Determine sign (direction) of new time value
    if (qdc_tmr0_cap01_buffer[temp_read_index] > 0)    
    {
      new_sign = 1;
    }
    else if (qdc_tmr0_cap01_buffer[temp_read_index] < 0)
    {
      new_sign = -1;
    }
    else
    {
      new_sign = 0;
    }

    //Process a time value from the buffer

    //Check for sign (direction) or glitch (= 0)
    if (new_sign == 0)    //Glitch detected by ISR
    {
      error_occurred(ERROR_QDC_CH01_GLTCH);

      //Move to next data in buffer, ignore this time value
      qdc_tmr0_data.read_index_01 = temp_read_index; 

      //Set glitch flag - throw out the next time value also.
      glitch_flag = 1;
    }
    else if (new_sign == prev_sign)   //Consistent sign
    {
      if (!glitch_flag)               //And no glitch
      {
        //Valid pulse time received, add to total
        time_since_pulse = 0;   //Reset this value, now that a valid pulse time is being counted
        total_time += qdc_tmr0_cap01_buffer[temp_read_index]; //Add up pulse time values from ring buffer
        N++;                    //Increment the number of pulse times included in sum
      }
      //Move to next data in buffer. Glitched values are skipped.
      qdc_tmr0_data.read_index_01 = temp_read_index;

      //Reset glitch flag, later values should be valid
      glitch_flag = 0;
    }
    else    //this data point has a new sign
    {
      //Are there two in a row with this sign? Check sign of next point, if available
      temp_read_index2 = (temp_read_index + 1) & (QDC_TMR0_CAP01_BUFFER_SIZE - 1);
      if (temp_read_index2 != temp_write_index) //An additional time value is available
      {
        //Determine sign (direction) of new time value
        if (qdc_tmr0_cap01_buffer[temp_read_index2] > 0)    
        {
          new_sign2 = 1;
        }
        else if (qdc_tmr0_cap01_buffer[temp_read_index2] < 0)
        {
          new_sign2 = -1;
        }
        else
        {
          new_sign2 = 0;
        }
        
        if (new_sign2 == new_sign)  //Valid new direction
        {
          total_time = 0;             //Throw out previously summed times, start again
          N = 0;
          prev_sign = new_sign;       //Update direction. Execution will continue with the same-sign
                                      //code above, adding up times in the new direction.
        }
        else    //Sign direction was apparently a glitch. Ignore, along with the next point.
        {
          glitch_flag = 1;    //Setting this causes the next point to be ignored.
          error_occurred(ERROR_QDC_CH01_GLTCH);
          //Move to next data point
          qdc_tmr0_data.read_index_01 = temp_read_index;  
        }  
      }
      else    //Leave last time value in buffer
              //Wait for next update function call and another time value to determine sign validity
              //Exit while loop even though buffer is not empty.
      {
        break;
      }
    }
  }

  if (total_time == 0)   //Total time will be zero if elapsed time buffer was empty or the data changed sign;
  {                      //Use the previous value or the time elapsed since the last value, whichever is larger
    
    N = 1;
    if (time_since_pulse < (0x7FFFFFFF - qdc_tmr0_data.cap01_update_time))  //Keep time within range of 32-bit signed integer
    {
      time_since_pulse += qdc_tmr0_data.cap01_update_time;
    }

    if ((time_since_pulse > prev_time) && (prev_time > 0))
    {
      total_time = time_since_pulse;
    }
    else if ((-time_since_pulse < prev_time) && (prev_time < 0))
    {
      total_time = -time_since_pulse;
    }
    else
    {
      total_time = prev_time;
    }
  }

  filter_rate = qdc_tmr0_data.cap01_smoothing_max; //Lower values will give less smoothing for a given value of N (number of data points).
  j = N;
  while (j > 0)   //Basically finding log base 2 of N; this goes into the filter rate calculation
  {
    j >>= 1;
    filter_rate -= 1;
  }

  // Put limit on minimum smoothing value
  if (filter_rate < qdc_tmr0_data.cap01_smoothing_min)
  {
    filter_rate = qdc_tmr0_data.cap01_smoothing_min;
  }

  rate = N * (0x7FFFFFFF/total_time); //rate fits in 32-bit signed integer (total_time != 0)

  //Exponential smoothing filter
  //av_rate is scaled to fit in 48-bit signed integer
  av_rate = av_rate - (av_rate >> filter_rate) + ((rate << 16) >> filter_rate);

  qdc_tmr0_data.cap01_vel_fixed = (qdc_tmr0_data.cap01_vel_gain * av_rate) >> (qdc_tmr0_data.cap01_vel_scaling + 31);

  prev_time = total_time/N;  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void qdc_tmr0_cap01_rate_update(void)
{
  signed long sign_new = 0, sign_old = 0;
  signed long i = 0, j = 0;
  signed long temp_read_index, temp_write_index;
  signed long total_time = 0;
  static signed long time_since_pulse = 0;
  static signed long prev_time = 0x7FFFFFFF;
  static signed long long av_rate = 0;
  signed long long rate;
  signed long filter_rate;

  temp_read_index = (qdc_tmr0_data.read_index_01 + 1) & (QDC_TMR0_CAP01_BUFFER_SIZE - 1);
  temp_write_index = qdc_tmr0_data.write_index_01;
  while (temp_read_index != temp_write_index)     //Buffer is not empty
  {
    time_since_pulse = 0;
    if (qdc_tmr0_cap01_buffer[temp_read_index] > 0)    //Determine sign (direction) of incoming time data
    {
      sign_new = 1;
    }
    else if (qdc_tmr0_cap01_buffer[temp_read_index] < 0)
    {
      sign_new = -1;
    }
    else
    {
      sign_new = 0;
    }

    if (sign_new + sign_old != 0) //Sign (direction) is consistent
    {
      if (qdc_tmr0_cap01_buffer[temp_read_index] != 0)
      {
        i++;
        total_time += qdc_tmr0_cap01_buffer[temp_read_index]; //Add up pulse time values from ring buffer
      }
      else  //Elapsed time was set to zero by ISR, indicating glitch detection.
      {
        //error - glitch
      }
      //Increment ring buffer read index
      //For a ring buffer of size equal to a power of two
      temp_read_index = (temp_read_index + 1) & (QDC_TMR0_CAP01_BUFFER_SIZE - 1);
      sign_old = sign_new;
    }
    else  //Sign has changed
    {
      total_time = 0; //Ignore data packets with inconsistent signs. Could be ok, could be due to a glitch.
      break;          //Ignore the rest of the data if there is a sign change
    }
  }
  qdc_tmr0_data.read_index_01 = ((temp_write_index - 1) & (QDC_TMR0_CAP01_BUFFER_SIZE - 1)); //empty ring buffer

  if (total_time == 0)   //Total time will be zero if elapsed time buffer was empty or the data changed sign;
  {                      //Use the previous value or the time elapsed since the last value, whichever is larger
    
    i = 1;
    if (time_since_pulse < (0x7FFFFFFF - qdc_tmr0_data.cap01_update_time))  //Keep time within range of 32-bit signed integer
    {
      time_since_pulse += qdc_tmr0_data.cap01_update_time;
    }

    if ((time_since_pulse > prev_time) && (prev_time > 0))
    {
      total_time = time_since_pulse;
    }
    else if ((-time_since_pulse < prev_time) && (prev_time < 0))
    {
      total_time = -time_since_pulse;
    }
    else
    {
      total_time = prev_time;
    }
  }

  filter_rate = qdc_tmr0_data.cap01_smoothing_max; //Lower values will give less smoothing for a given value of i (number of data points).
  j = i;
  while (j > 0)   //Basically finding log base 2 of i; this goes into the filter rate calculation
  {
    j >>= 1;
    filter_rate -= 1;
  }

  // Put limit on minimum smoothing value
  if (filter_rate < qdc_tmr0_data.cap01_smoothing_min)
  {
    filter_rate = qdc_tmr0_data.cap01_smoothing_min;
  }

  rate = i * (0x7FFFFFFF/total_time); //rate fits in 32-bit signed integer (total_time != 0)

  //Exponential smoothing filter
  //av_rate is scaled to fit in 48-bit signed integer
  av_rate = av_rate - (av_rate >> filter_rate) + ((rate << 16) >> filter_rate);

  qdc_tmr0_data.cap01_vel_fixed = (qdc_tmr0_data.cap01_vel_gain * av_rate) >> (qdc_tmr0_data.cap01_vel_scaling + 31);

  prev_time = total_time;  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void qdc_tmr0_cap23_rate_update(void)
{
  static signed long prev_sign = 0, glitch_flag = 0;
  signed long new_sign, new_sign2;
  signed long N = 0, j = 0;
  signed long temp_read_index, temp_read_index2, temp_write_index;
  signed long total_time = 0;
  static signed long time_since_pulse = 0;
  static signed long prev_time = 0x7FFFFFFF;
  static signed long long av_rate = 0;
  signed long long rate;
  signed long filter_rate;

  if (qdc_tmr0_data.write_index_23 == qdc_tmr0_data.read_index_23)
  {
    error_occurred(ERROR_QDC_CH23_BUF_F);  
  }
  
  temp_write_index = qdc_tmr0_data.write_index_23;  
  while (1)
  {
    //qdc_tmr0_data.read_index_23 points to the most recently read data value.
    //Increment the index to point to the oldest unread data value, if any.
    temp_read_index = (qdc_tmr0_data.read_index_23 + 1) & (QDC_TMR0_CAP23_BUFFER_SIZE - 1);
    if (temp_read_index == temp_write_index)    //Buffer is empty
    {
      break;
    }
    
    //Determine sign (direction) of new time value
    if (qdc_tmr0_cap23_buffer[temp_read_index] > 0)    
    {
      new_sign = 1;
    }
    else if (qdc_tmr0_cap23_buffer[temp_read_index] < 0)
    {
      new_sign = -1;
    }
    else
    {
      new_sign = 0;
    }

    //Process a time value from the buffer

    //Check for sign (direction) or glitch (= 0)
    if (new_sign == 0)    //Glitch detected by ISR
    {
      error_occurred(ERROR_QDC_CH23_GLTCH);

      //Move to next data in buffer, ignore this time value
      qdc_tmr0_data.read_index_23 = temp_read_index; 

      //Set glitch flag - throw out the next time value also.
      glitch_flag = 1;
    }
    else if (new_sign == prev_sign)   //Consistent sign
    {
      if (!glitch_flag)               //And no glitch
      {
        //Valid pulse time received, add to total
        time_since_pulse = 0;   //Reset this value, now that a valid pulse time is being counted
        total_time += qdc_tmr0_cap23_buffer[temp_read_index]; //Add up pulse time values from ring buffer
        N++;                    //Increment the number of pulse times included in sum
      }
      //Move to next data in buffer. Glitched values are skipped.
      qdc_tmr0_data.read_index_23 = temp_read_index;

      //Reset glitch flag, later values should be valid
      glitch_flag = 0;
    }
    else    //this data point has a new sign
    {
      //Are there two in a row with this sign? Check sign of next point, if available
      temp_read_index2 = (temp_read_index + 1) & (QDC_TMR0_CAP23_BUFFER_SIZE - 1);
      if (temp_read_index2 != temp_write_index) //An additional time value is available
      {
        //Determine sign (direction) of new time value
        if (qdc_tmr0_cap23_buffer[temp_read_index2] > 0)    
        {
          new_sign2 = 1;
        }
        else if (qdc_tmr0_cap23_buffer[temp_read_index2] < 0)
        {
          new_sign2 = -1;
        }
        else
        {
          new_sign2 = 0;
        }
        
        if (new_sign2 == new_sign)  //Valid new direction
        {
          total_time = 0;             //Throw out previously summed times, start again
          N = 0;
          prev_sign = new_sign;       //Update direction. Execution will continue with the same-sign
                                      //code above, adding up times in the new direction.
        }
        else    //Sign direction was apparently a glitch. Ignore, along with the next point.
        {
          glitch_flag = 1;    //Setting this causes the next point to be ignored.
          error_occurred(ERROR_QDC_CH23_GLTCH);
          //Move to next data point
          qdc_tmr0_data.read_index_23 = temp_read_index;  
        }  
      }
      else    //Leave last time value in buffer
              //Wait for next update function call and another time value to determine sign validity
              //Exit while loop even though buffer is not empty.
      {
        break;
      }
    }
  }

  if (total_time == 0)   //Total time will be zero if elapsed time buffer was empty or the data changed sign;
  {                      //Use the previous value or the time elapsed since the last value, whichever is larger
    
    N = 1;
    if (time_since_pulse < (0x7FFFFFFF - qdc_tmr0_data.cap23_update_time))  //Keep time within range of 32-bit signed integer
    {
      time_since_pulse += qdc_tmr0_data.cap23_update_time;
    }

    if ((time_since_pulse > prev_time) && (prev_time > 0))
    {
      total_time = time_since_pulse;
    }
    else if ((-time_since_pulse < prev_time) && (prev_time < 0))
    {
      total_time = -time_since_pulse;
    }
    else
    {
      total_time = prev_time;
    }
  }

  filter_rate = qdc_tmr0_data.cap23_smoothing_max; //Lower values will give less smoothing for a given value of N (number of data points).
  j = N;
  while (j > 0)   //Basically finding log base 2 of N; this goes into the filter rate calculation
  {
    j >>= 1;
    filter_rate -= 1;
  }

  // Put limit on minimum smoothing value
  if (filter_rate < qdc_tmr0_data.cap23_smoothing_min)
  {
    filter_rate = qdc_tmr0_data.cap23_smoothing_min;
  }

  rate = N * (0x7FFFFFFF/total_time); //rate fits in 32-bit signed integer (total_time != 0)

  //Exponential smoothing filter
  //av_rate is scaled to fit in 48-bit signed integer
  av_rate = av_rate - (av_rate >> filter_rate) + ((rate << 16) >> filter_rate);

  qdc_tmr0_data.cap23_vel_fixed = (qdc_tmr0_data.cap23_vel_gain * av_rate) >> (qdc_tmr0_data.cap23_vel_scaling + 31);

  prev_time = total_time/N;  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void qdc_tmr0_cap23_rate_update(void)
{
  signed long sign_new = 0, sign_old = 0;
  signed long i = 0, j = 0;
  signed long temp_read_index, temp_write_index;
  signed long total_time = 0;
  static signed long time_since_pulse = 0;
  static signed long prev_time = 0x7FFFFFFF;
  static signed long long av_rate = 0;
  signed long long rate;
  signed long filter_rate;

  temp_read_index = (qdc_tmr0_data.read_index_23 + 1) & (QDC_TMR0_CAP23_BUFFER_SIZE - 1);
  temp_write_index = qdc_tmr0_data.write_index_23;
  while (temp_read_index != temp_write_index)     //Buffer is not empty
  {
    time_since_pulse = 0;
    if (qdc_tmr0_cap23_buffer[temp_read_index] > 0)    //Determine sign (direction) of incoming time data
    {
      sign_new = 1;
    }
    else if (qdc_tmr0_cap23_buffer[temp_read_index] < 0)
    {
      sign_new = -1;
    }
    else
    {
      sign_new = 0;
    }

    if (sign_new + sign_old != 0) //Sign (direction) is consistent
    {
      if (qdc_tmr0_cap23_buffer[temp_read_index] != 0)
      {
        i++;
        total_time += qdc_tmr0_cap23_buffer[temp_read_index]; //Add up pulse time values from ring buffer
      }
      else  //Elapsed time was set to zero by ISR, indicating glitch detection.
      {
        //error - glitch
      }
      //Increment ring buffer read index
      //For a ring buffer of size equal to a power of two
      temp_read_index = (temp_read_index + 1) & (QDC_TMR0_CAP23_BUFFER_SIZE - 1);
      sign_old = sign_new;
    }
    else  //Sign has changed
    {
      total_time = 0; //Ignore data packets with inconsistent signs. Could be ok, could be due to a glitch.
      break;          //Ignore the rest of the data if there is a sign change
    }
  }
  qdc_tmr0_data.read_index_23 = ((temp_write_index - 1) & (QDC_TMR0_CAP23_BUFFER_SIZE - 1)); //empty ring buffer

  if (total_time == 0)   //Total time will be zero if elapsed time buffer was empty or the data changed sign;
  {                      //Use the previous value or the time elapsed since the last value, whichever is larger
    
    i = 1;
    if (time_since_pulse < (0x7FFFFFFF - qdc_tmr0_data.cap23_update_time))  //Keep time within range of 32-bit signed integer
    {
      time_since_pulse += qdc_tmr0_data.cap23_update_time;
    }

    if ((time_since_pulse > prev_time) && (prev_time > 0))
    {
      total_time = time_since_pulse;
    }
    else if ((-time_since_pulse < prev_time) && (prev_time < 0))
    {
      total_time = -time_since_pulse;
    }
    else
    {
      total_time = prev_time;
    }
  }

  filter_rate = qdc_tmr0_data.cap23_smoothing_max; //Lower values will give less smoothing for a given value of i (number of data points).
  j = i;
  while (j > 0)   //Basically finding log base 2 of i; this goes into the filter rate calculation
  {
    j >>= 1;
    filter_rate -= 1;

    // Put limit on minimum smoothing value
    if (filter_rate < qdc_tmr0_data.cap23_smoothing_min)
    {
      filter_rate = qdc_tmr0_data.cap23_smoothing_min;
    }
  }

  rate = i * (0x7FFFFFFF/total_time); //rate fits in 32-bit signed integer (total_time != 0)

  //Exponential smoothing filter
  //av_rate is scaled to fit in 48-bit signed integer
  av_rate = av_rate - (av_rate >> filter_rate) + ((rate << 16) >> filter_rate);

  qdc_tmr0_data.cap23_vel_fixed = (qdc_tmr0_data.cap23_vel_gain * av_rate) >> (qdc_tmr0_data.cap23_vel_scaling + 31);

  prev_time = total_time;  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void qdc_tmr0_cap01_angle_update(void)
{
  long long fixed_point_position;

  fixed_point_position = qdc_tmr0_data.encoder_count_01;
  fixed_point_position *= qdc_tmr0_data.cap01_pos_gain;
  fixed_point_position >>= qdc_tmr0_data.cap01_pos_scaling; 

  qdc_tmr0_data.cap01_pos_fixed = fixed_point_position; 
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void qdc_tmr0_cap23_angle_update(void)
{
  long long fixed_point_position;

  fixed_point_position = qdc_tmr0_data.encoder_count_23;
  fixed_point_position *= qdc_tmr0_data.cap23_pos_gain;
  fixed_point_position >>= qdc_tmr0_data.cap23_pos_scaling; 

  qdc_tmr0_data.cap23_pos_fixed = fixed_point_position; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void qdc_tmr0_init
  (
    float clock_freq,                     //Count frequency of timer in cycles per second

    float cap01_counts_per_rev,           //Pulses per revolution per channel (1X) of first encoder
    float cap01_gear_ratio,               //Gear ratio to obtain output shaft angle/rate, if desired.
    float cap01_rate_update_time,         //Time in seconds between calls of qdc_tmr0_cap01_rate_update
    signed long cap01_smoothing_max,      //Maximum allowable exponential smoothing coefficient exponent b, in range of 0 to 16
    signed long cap01_smoothing_min,      //Minimum allowable exponential smoothing coefficient exponent b, in range of 0 to 16, <= max
                                          //For example, a value of B = 3 for this parameter would give an
                                          //exponential smoothing coefficient of a = 1/(2^b) = 1/(2^3) = 1/8, and the
                                          //exponential smoothing formula An+1 = (1 - a)An + (a)Yn = (7/8)An + (1/8)Yn,
                                          //where An+1 is the new smoothed value, An is the old value, and Yn
                                          //is the latest velocity measurement. 
                                          //An exponential smoothing filter is equivalent to an RC lowpass filter
                                          //with time constant T = RC = Tu(1 - a)/a, where Tu is the update period. 
                                          //The cutoff frequency fc = 1/(2*pi*T) = a/(2*pi*Tu*(1 - a)) =
                                          //fc = 1/(2*pi*Tu*(2^b - 1))
    float cap23_counts_per_rev,           //Pulses per revolution per channel (1X) of second encoder
    float cap23_gear_ratio,               //Gear ratio to obtain output shaft angle/rate, if desired
    float cap23_rate_update_time,         //Time in seconds between calls of qdc_tmr0_cap23_rate_update
    signed long cap23_smoothing_max,      //Maximum allowable exponential smoothing coefficient exponent b. See above.
    signed long cap23_smoothing_min       //Minimum allowable exponential smoothing coefficient exponent b.
  )
{
  float cap01_pos_gain, cap01_vel_gain, cap23_pos_gain, cap23_vel_gain;
  
  //Initialize qdc data struct
  qdc_tmr0_data.overflow_size = T0MR0 + 1;
  qdc_tmr0_data.tmr_base_addr = 0xE0004000;
  qdc_tmr0_data.FIO0PIN_addr  = 0x3FFFC014;

  qdc_tmr0_data.overflow_01 = 0;
  qdc_tmr0_data.encoder_count_01 = 0;
  qdc_tmr0_data.write_index_01 = 1;
  qdc_tmr0_data.read_index_01 = 0;
  qdc_tmr0_data.buffer_mask_01 = QDC_TMR0_CAP01_BUFFER_SIZE - 1;
  qdc_tmr0_data.time_array_addr_01 = (unsigned long)qdc_tmr0_cap01_buffer;

  qdc_tmr0_data.overflow_23 = 0;
  qdc_tmr0_data.encoder_count_23 = 0;
  qdc_tmr0_data.write_index_23 = 1;
  qdc_tmr0_data.read_index_23 = 0;
  qdc_tmr0_data.buffer_mask_23 = QDC_TMR0_CAP23_BUFFER_SIZE - 1;
  qdc_tmr0_data.time_array_addr_23 = (unsigned long)qdc_tmr0_cap23_buffer;

  qdc_tmr0_data.cap01_update_time = (unsigned long)(cap01_rate_update_time * clock_freq);
  qdc_tmr0_data.cap01_smoothing_max = cap01_smoothing_max;
  qdc_tmr0_data.cap01_smoothing_min = cap01_smoothing_min;
  qdc_tmr0_data.cap01_pos_scaling = 0;  //Initialize scaling factor to 0 (= 2^0 = 1)
  qdc_tmr0_data.cap01_vel_scaling = 0;  //Initialize scaling factor to 0 (= 2^0 = 1)

  //Calculate the position gain, scaled up by 16 bits to give 16.16 fixed point
  cap01_pos_gain = 3.1415927 * 65536 / cap01_counts_per_rev / cap01_gear_ratio;
  
  //Scale the position gain to keep fixed-point math in range
  if (cap01_pos_gain > 0)
  {
    while (cap01_pos_gain <= 16384)
    {
      cap01_pos_gain *= 2;
      ++qdc_tmr0_data.cap01_pos_scaling;
    }
    while (cap01_pos_gain >= 65536)
    {
      cap01_pos_gain /= 2;
      --qdc_tmr0_data.cap01_pos_scaling;
    }
  }
  else if (cap01_pos_gain < 0)
  {
    while (cap01_pos_gain >= -16384)
    {
      cap01_pos_gain *= 2;
      ++qdc_tmr0_data.cap01_pos_scaling;
    }
    while (cap01_pos_gain <= -65536)
    {
      cap01_pos_gain /= 2;
      --qdc_tmr0_data.cap01_pos_scaling;
    }
  }
  qdc_tmr0_data.cap01_pos_gain = (signed long)cap01_pos_gain;

  //Calculate the velocity gain
  cap01_vel_gain = clock_freq * 3.1415927/cap01_counts_per_rev/cap01_gear_ratio;

  //Scale the velocity gain to keep fixed-point math in range
  if (cap01_vel_gain > 0)
  {
    while (cap01_vel_gain <= 4096)
    {
      cap01_vel_gain *= 2;
      ++qdc_tmr0_data.cap01_vel_scaling;
    }
    while (cap01_vel_gain >= 16384)
    {
      cap01_vel_gain /= 2;
      --qdc_tmr0_data.cap01_vel_scaling;
    }
  }
  else if (cap01_vel_gain < 0)
  {
    while (cap01_vel_gain >= -4096)
    {
      cap01_vel_gain *= 2;
      ++qdc_tmr0_data.cap01_vel_scaling;
    }
    while (cap01_vel_gain <= -16384)
    {
      cap01_vel_gain /= 2;
      --qdc_tmr0_data.cap01_vel_scaling;
    }
  }
  qdc_tmr0_data.cap01_vel_gain = (signed long)cap01_vel_gain;

  qdc_tmr0_data.cap23_update_time = (unsigned long)(cap23_rate_update_time * clock_freq);
  qdc_tmr0_data.cap23_smoothing_max = cap23_smoothing_max;
  qdc_tmr0_data.cap23_smoothing_min = cap23_smoothing_min;
  qdc_tmr0_data.cap23_pos_scaling = 0;  //Initialize scaling factor to 0 (= 2^0 = 1)
  qdc_tmr0_data.cap23_vel_scaling = 0;  //Initialize scaling factor to 0 (= 2^0 = 1)

  //Calculate the position gain, scaled up by 16 bits to give 16.16 fixed point
  cap23_pos_gain = 3.1415927 * 65536 / cap23_counts_per_rev / cap23_gear_ratio;

  //Scale the position gain to keep fixed-point math in range
  if (cap23_pos_gain > 0)
  {
    while (cap23_pos_gain <= 16384)
    {
      cap23_pos_gain *= 2;
      ++qdc_tmr0_data.cap23_pos_scaling;
    }
    while (cap23_pos_gain >= 65536)
    {
      cap23_pos_gain /= 2;
      --qdc_tmr0_data.cap23_pos_scaling;
    }
  }
  else if (cap23_pos_gain < 0)
  {
    while (cap23_pos_gain >= -16384)
    {
      cap23_pos_gain *= 2;
      ++qdc_tmr0_data.cap23_pos_scaling;
    }
    while (cap23_pos_gain <= -65536)
    {
      cap23_pos_gain /= 2;
      --qdc_tmr0_data.cap23_pos_scaling;
    }
  }
  qdc_tmr0_data.cap23_pos_gain = (signed long)cap23_pos_gain;

  //Calculate the velocity gain
  cap23_vel_gain = clock_freq * 3.1415927/cap23_counts_per_rev/cap23_gear_ratio;

  //Scale the velocity gain to keep fixed-point math in range
  if (cap23_vel_gain > 0)
  {
    while (cap23_vel_gain <= 4096)
    {
      cap23_vel_gain *= 2;
      ++qdc_tmr0_data.cap23_vel_scaling;
    }
    while (cap23_vel_gain >= 16384)
    {
      cap23_vel_gain /= 2;
      --qdc_tmr0_data.cap23_vel_scaling;
    }
  }
  else if (cap23_vel_gain < 0)
  {
    while (cap23_vel_gain >= -4096)
    {
      cap23_vel_gain *= 2;
      ++qdc_tmr0_data.cap01_vel_scaling;
    }
    while (cap23_vel_gain <= -16384)
    {
      cap23_vel_gain /= 2;
      --qdc_tmr0_data.cap23_vel_scaling;
    }
  }
  qdc_tmr0_data.cap23_vel_gain = (signed long)cap23_vel_gain;

}
////////////////////////////////////////////////////////////////////////////////////////////////////

// DO NOT DELETE THE COMMENTED CODE BELOW - IT IS NEEDED FOR THE FIQ HANDLER
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// QDC Timer 0 2X quadrature decoder assembly code for use as an FIQ interrupt service routine.
// For FIQ use put in a .s file and add it to the project. It expects to be the exclusive FIQ handler.
// Use as an IRQ ISR would require some minor changes in which registers were saved at the start,
// and insertion into a C-callable function framework.
// This implementation uses capture inputs on pins P0[16], P0[22], P0[27], and P0[29]. Use of other
// pins, Timer 1, other similar processors, etc., would require update of these pin references, as shown by "!!!". 
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
        AREA FIQ_HANDLER, CODE, READONLY, ALIGN=3
        ARM
        PRESERVE8

IR_OFFSET         EQU     0x0
TC_OFFSET         EQU     0x8
MR0_OFFSET        EQU     0x18
CCR_OFFSET        EQU     0x28
CR0_OFFSET        EQU     0x2C
CR1_OFFSET        EQU     0x30
CR2_OFFSET        EQU     0x34
CR3_OFFSET        EQU     0x38

;qdc_data struct fields
overflow_size           EQU 0
tmr_base                EQU 4
FIO0PIN                 EQU 8

overflow_01             EQU 12
encoder_count_01        EQU 16
write_index_01          EQU 20
read_index_01           EQU 24
buffer_mask_01          EQU 28
time_array_01           EQU 32

overflow_23             EQU 36
encoder_count_23        EQU 40
write_index_23          EQU 44
read_index_23           EQU 48
buffer_mask_23          EQU 52
time_array_23           EQU 56

        IMPORT  qdc_tmr0_data         [DATA]

        ALIGN 8 
FIQ_Handler
        EXPORT FIQ_Handler

        ;Save variable registers used to stack
        STMFD    SP!,{R0-R7} ;Save non-FIQ registers used to stack
                             ;These registers will be reused by cap23, if
                             ;it so happens that both have interrupts at the same time.

        ;start of 2x code - two counts per full encoder cycle
        LDR R9, =qdc_tmr0_data    ;load data struct base address
        ADD R9, #overflow_01      ;point R9 to overflow value

        LDMDA R9,{R0-R3}          ;load first set of data struct values (backwards from overflow)

        ;R0 = overflow_size
        ;R1 = tmr_base_addr
        ;R2 = FIO0PIN_addr
        ;R3 = overflow

        ;R9 = data struct base address
        ;R13 = stack pointer
        ;R14 = link register
        ;R15 = program counter

        LDRB R12, [R1, #IR_OFFSET]   
        ;R12 = T0IR value

        TST R12, #1         ;Match event on MR0?
        MOVEQ R0, #0        ;No match event? Set overflow size to zero, no added overflow this call

cap01   ;Start first encoder processing (CAP01)
        CMP R3, #0x7F000000 ;Is overflow approaching the 32-bit positive maximum (Note: won't work for too-large overflow_size)
        ADDLT R3, R3, R0    ;Add in overflow if limit has not been reached.

        ;Load FIO0PIN register contents.
        ;Performance of this code is sensitive to when the FIO0PIN values are sampled.
        ;Too soon may lead to glitch detection, if the CAP0 pin is not fully settled in its new value.
        ;Too late, and the CAP1 pin may have moved to a new value, at high count speeds.
        ;Load FIO0PIN contents into R11 for use by both CAP01 and CAP23 as needed.
        LDR R11, [R2]           
        ;R11 = FIO0PIN value

        TST R12, #0x10              ;Test CAP0 flag
        BEQ write_back_overflow_01  ;If no capture event on cap0, skip the count and timing code
                                    ;and write back the updated overflow value
        LDR R10, [R1, #CCR_OFFSET]  ;Read in the CCR register
        MOV R8, R10, LSR #1         ;Falling-edge enable bit moved to bit 0
        ADD R8, R8, R11, LSR #22    ;Add together R10 and R11, with falling-edge and CAP0 pin bits aligned !!!
        AND R8, R8, #1              ;Gives 1 for a valid count transition, 0 for glitch
        BIC R10, R10, #0x3          ;clear edge direction bits
        BIC R10, R10, #0xF000       ;clear reserved bits
        TST R11, #0x00400000        ;check state of CAP0 (P0[22]) !!!
        ORRNE R10, R10, #2          ;Set for falling edge
        ORREQ R10, R10, #1          ;Set for rising edge
        STR R10, [R1, #CCR_OFFSET]  ;Update the CCR register.
        TST R10, #1                 ;Test cap 0 rising edge IE bit in CCR
        RSBNE R8, R8, #0            ;Switch sign if falling edge was active (before rewriting above)
        TST R11, #0x08000000        ;Test P0[27] in FIO0PIN (CAP 1 input) !!!
        RSBNE R8, R8, #0            ;Switch sign again if high (NE zero)

       ;R8 = count increment (-1, 0, or 1). A glitch is indicated by 0 - a rising edge was detected, but the 
       ;CAPO line is now low, or a falling edge was detected, but now the line is high.

        LDR R10, [R1, #CR0_OFFSET]  ;New capture time value
                                    ;Update this even if the count is glitched

        RSB R2, R10, #0    ;R2 will be the next initial overflow value = - capture time

        ;Simultaneous match/CAP0 event handling
        CMP R10, R0, LSR #1 ;Did the capture event occur immediately before, or after, the match event
                            ;If before, captured time will be near max; if after, captured time will be near zero
        ADDHS R2, R2, R0    ;Increase next overflow value if capture occurred before the match
        SUBHS R3, R3, R0    ;Decrease current overflow value if capture before match
                            ;Note that R0 will equal zero if no overflow/match event, so this code
                            ;will have no effect in that case.

        ADD R3, R3, R10     ;R3 now contains elapsed time since previous capture

        ;R10 is now free
 
        ;Load remaining data struct values
        LDMIB R9, {R4 - R7, R10}

        ;R0 = overflow_size
        ;R1 = tmr_base_addr
        ;R2 = overflow (next)
        ;R3 = elapsed time
        ;R4 = encoder_count
        ;R5 = write_index
        ;R6 = read_index
        ;R7 = buffer_mask
        ;R8 = count increment
        ;R9 = data_struct_address (overflow)
        ;R10 = time_array_address
        ;R11 = FIO0PIN
        ;R12 = TOIR
        ;R13 = stack pointer
        ;R14 = link register
        ;R15 = program counter

        CMP R8, #0            ;R8 (count) = 0 means there was a glitch
        MOVEQ R3, #0          ;Set time value to zero in the event of a glitch

        ADD R4, R4, R8        ;Update encoder count

        CMP R5, R6            ;check if ring buffer is full
        BEQ write_back_all_01 ;skip remainder of velocity code if buffer is full

        CMP R8, #1            ;Check sign
        RSBNE R3, R3, #0      ;Make negative if count is minus
  
        STR R3, [R10, R5, LSL #2] ;Save time value to ring buffer.
                                  ;Address is R10 (base) plus (R5 ring buffer write index times 4)
        ADD R5, R5, #1            ;increment write index
        AND R5, R5, R7            ;modulo addition, wraps around ring buffer.
                                  ;(ring buffer size must 2^n; ANDed value must be 2^n-1 for this to work)


write_back_all_01
 
        STMIA R9, {R2, R4, R5}    ;save overflow, encoder count, and write index back to data struct
        B cap23

write_back_overflow_01
       STR R3, [R9]           ;save overflow only, continue on to possible cap23 processing
       
       ;Start of second encoder processing, using TMR0 capture inputs 2 and 3
cap23
        LDR R3, [R9,#overflow_23 - overflow_01]! ;load overflow_23 into R3, and point R9 to overflow_23

        ;R0 = overflow_size
        ;R1 = tmr_base_addr
        ;R2 = FIO0PIN value
        ;R3 = overflow_23

        ;R9 = data struct base address
        ;R11 = FIO0PIN value
        ;R12 = T0IR
        ;R13 = stack pointer
        ;R14 = link register
        ;R15 = program counter

        ;Update overflow_23 value. R0 will be non-zero only in the event of an MR0 flag (checked previously)
        CMP R3, #0x7F000000 ;Is overflow approaching the 32-bit positive maximum (Note: won't work for too-large overflow_size)
        ADDLT R3, R3, R0    ;Add in overflow

        TST R12, #0x40              ;Test CAP2 flag
        BEQ write_back_overflow_23  ;If no capture event on cap2, skip the count and timing code
                                    ;and write back the updated overflow value
        LDR R10, [R1, #CCR_OFFSET]  ;Read in the CCR register
        MOV R8, R10, LSR #7         ;CAP2 falling-edge enable bit moved to bit 0, put in R8
        ADD R8, R8, R11, LSR #16    ;Add together R10 and R11, with CAP2 falling-edge and CAP2 pin bits aligned !!!
        AND R8, R8, #1              ;Gives 1 for a valid count transition, 0 for glitch
        BIC R10, R10, #0xC0         ;Clear CAP2 edge direction bits
        BIC R10, R10, #0xF000       ;clear reserved bits
        TST R11, #0x10000           ;check state of CAP2 pin (P0[16]) !!!
        ORRNE R10, R10, #0x80       ;Set T0CCR for CAP2 falling edge
        ORREQ R10, R10, #0x40       ;Set T0CCR for CAP2 rising edge
        STR R10, [R1, #CCR_OFFSET]  ;Update the CCR register.
        TST R10, #0x40              ;Test CAP2 rising edge IE bit in T0CCR
        RSBNE R8, R8, #0            ;Switch sign if falling edge was active (before rewriting above)
        TST R11, #0x20000000        ;Test P0[29] in FIO0PIN (CAP 3 input) !!!
        RSBNE R8, R8, #0            ;Switch sign again if high (NE zero)

       ;R8 = count increment (-1, 0, or 1). A glitch is indicated by 0 - a rising edge was detected, but the 
       ;CAP2 line is now low, or a falling edge was detected, but now the line is high.

        LDR R10, [R1, #CR2_OFFSET]  ;New capture time value

        RSB R2, R10, #0     ;R2 will be the next initial overflow value = - capture time

        ;Simultaneous overflow handling
        CMP R10, R0, LSR #1 ;Did the capture event occur immediately before, or after, the match event
        ADDHS R2, R2, R0    ;Increase next overflow value if capture occurred before the overflow
        SUBHS R3, R3, R0    ;Decrease current overflow value if capture before overflow
                            ;Note that R0 will equal zero if no overflow/match event, so this code
                            ;will have no effect in that case.

        ADD R3, R3, R10     ;R3 now contains elapsed time since previous capture

        ;R10 is now free
 
        ;Load remaining data struct values
        LDMIB R9, {R4 - R7, R10}

        ;R0 = overflow_size
        ;R1 = tmr_base_addr
        ;R2 = Overflow (next)
        ;R3 = elapsed time
        ;R4 = encoder_count
        ;R5 = write_index
        ;R6 = read_index
        ;R7 = buffer_mask
        ;R8 = count increment
        ;R9 = data_struct_address (overflow)
        ;R10 = time_array_address
        ;R11 = FIO0PIN
        ;R12 = TOIR
        ;R13 = stack pointer
        ;R14 = link register
        ;R15 = program counter

        CMP R8, #0                ;R8 (count) = 0 means there was a glitch
        MOVEQ R3, #0              ;Set time value to zero in the event of a glitch

        ADD R4, R4, R8            ;Update encoder count

        CMP R5, R6                ;check if ring buffer is full
        BEQ write_back_all_23     ;skip remainder of velocity code if buffer is full

        CMP R8, #1                ;Check sign
        RSBNE R3, R3, #0          ;Make negative if count is minus
  
        STR R3, [R10, R5, LSL #2] ;Save time value to ring buffer.
                                  ;Address is R10 (base) plus (R5 ring buffer write index times 4)
        ADD R5, R5, #1            ;increment write index
        AND R5, R5, R7            ;modulo addition, wraps around ring buffer.
                                  ;(ring buffer size must 2^n; ANDed value must be 2^n-1 for this to work)


write_back_all_23
        STMIA R9, {R2, R4, R5}    ;save overflow, encoder count, and write index back to data struct
        B clear_int

write_back_overflow_23
        STR R3, [R9]               ;save overflow only

clear_int        
        ;Clear the Timer0 interrupt flags that were processed (R12)
        STRB R12, [R1,#IR_OFFSET]

        ;reg_pop
        LDMFD SP!,{R0-R7}         ;Pop non-FIQ registers used off of stack

qdc_tmr0_isr_end
 
        ;Return to C main from FIQ handler. Put this at the end of any FIQ handler.
        SUBS PC, R14, #0x04
  
        END
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DO NOT DELETE THE COMMENTED CODE ABOVE - IT IS NEEDED FOR THE FIQ HANDLER
