#ifndef __MOD_QDC_H__
#define __MOD_QDC_H__

/*
typedef struct qdc_lookup_element{
  unsigned long state;
  unsigned long cc_reg;
  signed long   count;
  unsigned long cap_reg;
} QDC_LOOKUP_ELEMENT;
*/

//Structure for setup and initialization of a pair
//of decoders for one timer (two pairs of capture inputs)
//Note that the assembly code is timer-specific, and even board-specific.
//This is because the code cannot know in advance which capture pins you will be using.
//The pin-specific instructions will have to be updated to use a different board timer.
typedef struct qdc_data{
// *** WARNING *** the assembly code assumes a specific order and position of the 15 structure elements below.
// Do not change in any way!!!
  unsigned long overflow_size;    //Timer counts for a complete cycle to Match 0 with reset: Set equal to MR0 + 1
  unsigned long tmr_base_addr; 
  unsigned long FIO0PIN_addr;

  signed long   overflow_01;
  signed long   encoder_count_01;
  unsigned long write_index_01;
  unsigned long read_index_01;
  unsigned long buffer_mask_01;
  unsigned long time_array_addr_01;

  signed long   overflow_23;
  signed long   encoder_count_23;
  unsigned long write_index_23;
  unsigned long read_index_23;
  unsigned long buffer_mask_23;
  unsigned long time_array_addr_23;

// *** WARNING *** the assembly code assumes a specific order and position of the 15 structure elements above.
// Do not change in any way!!!

// Fields below are for C configuration of the module. The order is not critical.
  signed long   cap01_smoothing_max;  //Maximum allowable exponential smoothing coefficient
  signed long   cap01_smoothing_min;  //Minimum allowable exponential smoothing coefficient
  signed long   cap01_pos_gain;       //Calculated based on counts_per_rev and gear_ratio, above, by init function
  signed long   cap01_pos_scaling;    //Fixed-point scaling factor for pos_gain, above, calculated in init function
  signed long   cap01_pos_fixed;      //16.16 fixed point position result output in radians
  signed long   cap01_vel_gain;       //Calculated based on counts_per_rev, clock_freq, gear_ratio, smoothing, etc.
  signed long   cap01_vel_scaling;    //Fixed-point scaling factor for vel_gain, above, calculated in init function
  signed long   cap01_vel_fixed;      //16.16 fixed point velocity/rate result output in radians/sec
  unsigned long cap01_update_time;    //Calculated in init function; in timer clock cycles per update function call

  signed long   cap23_smoothing_max;  //Maximum allowable exponential smoothing coefficient
  signed long   cap23_smoothing_min;  //Minimum allowable exponential smoothing coefficient  
  signed long   cap23_pos_gain;       //Calculated based on counts_per_rev and gear_ratio, above, by init function
  signed long   cap23_pos_scaling;    //Fixed-point scaling factor for pos_gain, above, calculated in init function
  signed long   cap23_pos_fixed;      //16.16 fixed point position result output in radians
  signed long   cap23_vel_gain;       //Calculated based on counts_per_rev, clock_freq, gear_ratio, smoothing, etc.
  signed long   cap23_vel_scaling;    //Fixed-point scaling factor for vel_gain, above, calculated in init function
  signed long   cap23_vel_fixed;      //16.16 fixed point velocity/rate result output in radians/sec
  unsigned long cap23_update_time;    //Calculated in init function; in timer clock cycles per update function call
} QDC_DATA;

//Public functions
float qdc_tmr0_cap01_get_angle_float(void);
float qdc_tmr0_cap23_get_angle_float(void);
void qdc_tmr0_cap01_set_angle_float(float new_angle);
void qdc_tmr0_cap23_set_angle_float(float new_angle);
float qdc_tmr0_cap01_get_rate_float(void);
float qdc_tmr0_cap23_get_rate_float(void);
signed long qdc_tmr0_cap01_get_angle_fixed(void);
signed long qdc_tmr0_cap23_get_angle_fixed(void);
void qdc_tmr0_cap01_set_angle_fixed(signed long new_angle);
void qdc_tmr0_cap23_set_angle_fixed(signed long new_angle);
signed long qdc_tmr0_cap01_get_rate_fixed(void);
signed long qdc_tmr0_cap23_get_rate_fixed(void);
void qdc_tmr0_cap01_angle_update(void);
void qdc_tmr0_cap23_angle_update(void);
void qdc_tmr0_cap01_rate_update(void);
void qdc_tmr0_cap23_rate_update(void);
void qdc_tmr0_init
  (
    float         clock_freq,             //Count frequency of timer in cycles per second

    float         cap01_counts_per_rev,   //Pulses per revolution per channel (1X) of first encoder
    float         cap01_gear_ratio,       //Gear ratio to obtain output shaft angle/rate, if desired.
    float         cap01_update_time,      //Time in seconds per call to cap01_rate_update, = Tu
    signed long   cap01_smoothing_max,    //Maximum allowable exponential smoothing coefficient exponent b, in range 0 to 16
    signed long   cap01_smoothing_min,    //Minimum allowable exponential smoothing coefficient exponent b, in range 0 to 16, <= max
                                          //For example, a value of B = 3 for this parameter would give an
                                          //exponential smoothing coefficient of a = 1/(2^b) = 1/(2^3) = 1/8, and the
                                          //exponential smoothing formula An+1 = (1 - a)An + (a)Yn = (7/8)An + (1/8)Yn,
                                          //where An+1 is the new smoothed value, An is the old value, and Yn
                                          //is the latest velocity measurement. 
                                          //An exponential smoothing filter is equivalent to an RC lowpass filter
                                          //with time constant T = RC = Tu(1 - a)/a, where Tu is the update period. 
                                          //The cutoff frequency fc = 1/(2*pi*T) = a/(2*pi*Tu*(1 - a)) =
                                          //fc = 1/(2*pi*Tu*(2^b - 1))
                                          
    float         cap23_counts_per_rev,   //Pulses per revolution per channel (1X) of second encoder
    float         cap23_gear_ratio,       //Gear ratio to obtain output shaft angle/rate, if desired
    float         cap23_update_time,      //Time in seconds per call to cap23_rate_update
    signed long   cap23_smoothing_max,    //Maximum allowable exponential smoothing coefficient exponent b. See above.
    signed long   cap23_smoothing_min     //Minimum allowable exponential smoothing coefficient exponent b.
  );

//Private functions

/*
void qdc_push_time_tmr0_cap01(unsigned long flags, signed long cap_time, signed short sign);
void qdc_push_time_tmr0_cap23(unsigned long flags, signed long cap_time, signed short sign);
void qdc_tmr0_cap01_decoder(unsigned short flags);
void qdc_tmr0_cap01_decoder_x2(unsigned short tmr_flags);
void qdc_tmr0_cap23_decoder(unsigned short flags);
 */








































#endif //__MOD_QDC_H__
