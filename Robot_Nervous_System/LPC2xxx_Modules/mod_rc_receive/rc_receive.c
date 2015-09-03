/**
  @file rc_receive.c
  
  This file decodes the values coming from the RC module on the UI
  board from 4 channels. 
  To use: 
    - 1. Call @ref rcx_init from software setup.
    - 2. Copy needed code to hardware setup - this module uses Timer0, so make sure
          this is free to use. 
    - 3. Add the interrupt for Timer0 to interrupt setup with
          @ref rcx_isr as the vector address. This module is interrupt driven, so no
          functions need to be added to the schedule.
    - 4. Call any of the getter functions to get out the values from a channel.
    
  Example hardware setup of Timer:
  @code
  // *******************************************************************************
  // RC Receive Setup
  // *******************************************************************************
  PINSEL1 &= ~(3<<12); //p0.22 clear
  PINSEL1 &= ~(3<<22); //p0.27 clear
//  PINSEL1 &= ~(3<<0); //p0.16  clear
//  PINSEL1 &= ~(3<<26); //p0.29 clear
  PINSEL1 |= (2<<12); //p0.22 cap0
  PINSEL1 |= (2<<22); //p0.27 cap1
//  PINSEL1 |= (3<<0); //p0.16 cap2
//  PINSEL1 |= (2<<26); //p0.29 cap3
  T0IR = 0xFF;
  T0CCR |= (1<<0)|(1<<1)|(1<<2); //cap0 interrupt on RE (rising edge) and FE (falling edge)
  T0CCR |= (1<<3)|(1<<4)|(1<<5); //cap1 interrupt on RE (rising edge) and FE (falling edge)
//  T0CCR |= (1<<6)|(1<<7)|(1<<8); //cap2 interrupt on RE (rising edge) and FE (falling edge)
//  T0CCR |= (1<<9)|(1<<10)|(1<<11); //cap3 interrupt on RE (rising edge) and FE (falling edge)
  T0MR0 = (60000000/100)-1; //10ms       
  T0MCR = (1<<0)|(1<<1); // Match Control Reg: Interrupt(b0) and Reset(b1) on MR0;
  T0TCR = 1;   // Timer0 Enable
  @endcode
    
  @author Nicolas Williamson 
  @author Jason Cortell
  @date April 12, 2010
*/

#include <includes.h>

static int rcx_read[RCX_LAST]; /**< Array indicating whether we are using a given channel. 0 = no, 1 = yes. */
static volatile RCX_DATA rcx_chans[RCX_LAST]; /**< An array of the rc channels. */
static int rcx_is_init = 0; /**< Whether or not this module has been initialized with @ref rcx_init yet. */

/**
  Initializes the rc channels.
  Must call this function before any others for
  proper operation.
  @param chan_0 1 to read from channel 0 (RCIN0) or 0.
  @param chan_1 1 to read from channel 1 (RCIN1) or 0.
  @param chan_2 1 to read from channel 2 (RCIN2) or 0.
  @param chan_3 1 to read from channel 3 (RCIN3) or 0.
*/
void rcx_init(int chan_0, int chan_1, int chan_2, int chan_3)
{
  int i = 0;
  volatile RCX_DATA* data;
  for (i = 0; i < RCX_LAST; i++){
    data = &rcx_chans[i];
    data->prev = -1;
    data->start = 0;
    data->value = 0;
    data->overflows = 0;
  }
  rcx_read[0] = chan_0;
  rcx_read[1] = chan_1;
  rcx_read[2] = chan_2;
  rcx_read[3] = chan_3;
  rcx_is_init = 1;
}

/**
  Returns the current pulse width of the given channel.
  @param chan The RCX_CHAN to read from.
  @return Pulse width as a floating point number, or 0.0 if
  the module hasn't been initialized or we aren't using that channel.
*/
float rcx_get_chan(RCX_CHAN chan)
{
  if (rcx_is_init){
    if (rcx_read[chan]){
      return (float)(rcx_chans[chan].value>>14);
    } else {
      error_occurred(ERROR_RCX_BAD_CHAN);
      return 0.0;
    }
  } else {
    error_occurred(ERROR_RCX_NINIT);
    return 0.0;
  }
}

/**
  Gets the pulse width of channel 0.
  @return The pulse with of channel 0 as a floating point number, or 0.0
  if the module hasn't been initialized.
*/
float rcx_get_chan_0(void){return rcx_get_chan(RCX_CHAN_0);} 
/**
  Gets the pulse width of channel 1.
  @return The pulse with of channel 1 as a floating point number, or 0.0
  if the module hasn't been initialized.
*/
float rcx_get_chan_1(void){return rcx_get_chan(RCX_CHAN_1);}
/**
  Gets the pulse width of channel 2.
  @return The pulse with of channel 2 as a floating point number, or 0.0
  if the module hasn't been initialized.
*/
float rcx_get_chan_2(void){return rcx_get_chan(RCX_CHAN_2);}
/**
  Gets the pulse width of channel 3.
  @return The pulse with of channel 3 as a floating point number, or 0.0
  if the module hasn't been initialized.
*/
float rcx_get_chan_3(void){return rcx_get_chan(RCX_CHAN_3);}

/**
  The interrupt service routine for the rc module.
  This function is called when timer0 causes an interrupt,
  either from a capture or a match. If it is a capture event,
  that channel's data is updated.
*/
void rcx_isr(void) __irq
{
  int i;
  int interrupts = T0IR;
  int cap_0 = T0CR0;
  int cap_1 = T0CR1;
  int cap_2 = T0CR2;
  int cap_3 = T0CR3;
  int pins = FIO0PIN;
  int lvl_0 = pins & (1<<22);
  int lvl_1 = pins & (1<<27);
  int lvl_2 = pins & (1<<16);
  int lvl_3 = pins & (1<<29);
  int flag_cap_0 = interrupts & (1<<4);
  int flag_cap_1 = interrupts & (1<<5);
  int flag_cap_2 = interrupts & (1<<6);
  int flag_cap_3 = interrupts & (1<<7);
  int flag_overflow = interrupts & (1<<0);
  T0IR = flag_overflow | flag_cap_0 | flag_cap_1 | flag_cap_2 | flag_cap_3; //clear interrupts
  
  if (flag_overflow){
    for (i = 0; i < RCX_LAST; i++){
      rcx_chans[i].overflows++;
    }
  }
  if (flag_cap_0 && rcx_read[0]){
    rcx_update(RCX_CHAN_0, lvl_0, cap_0, flag_overflow);
  }
  if (flag_cap_1 && rcx_read[1]){
    rcx_update(RCX_CHAN_1, lvl_1, cap_1, flag_overflow);
  }
  if (flag_cap_2 && rcx_read[2]){
    rcx_update(RCX_CHAN_2, lvl_2, cap_2, flag_overflow);
  }
  if (flag_cap_3 && rcx_read[3]){
    rcx_update(RCX_CHAN_3, lvl_3, cap_3, flag_overflow);
  } 
  
  VICVectAddr = 0; //for __irq
  
}

/**
  Updates the given channel from a capture register.
  @param chan The channel to update.
  @param curr The level (0 = low, >0 = high) of the signal
  @param cap_value The value of the timer's capture register.
  @param flag_overflow The timer overflow flag.
*/
void rcx_update(RCX_CHAN chan, int curr, int cap_value, int flag_overflow)
{
  static int coeff = 4;
  int new_value;
  volatile RCX_DATA* data = &rcx_chans[chan];

  if (flag_overflow){
    if (cap_value > T0MR0 / 2){ //overflow happened after capture
      data->overflows--;
    }
  }

  if (!data->prev && curr){ //Capture channel has gone from low to high
    data->start = cap_value; //rcx_initial_time_0 = T0CR0;
  } else if (data->prev && !curr){ //Capture channel has gone from high to low
    //find pulse width
    new_value = (data->overflows * T0MR0) + cap_value - data->start;    
    //add pulse width to average
    if (data->value == 0){ //init average
      data->value = new_value<<14; 
    } else if (new_value > 0) { //average new data
      data->value = data->value - (data->value >> coeff) + (new_value << (14-coeff));
    }

  } else { //undefined operation - either both low or both high <- BAD
    error_occurred(ERROR_RCX_BAD_LVLS);
  }
  data->overflows = 0;
  data->prev = curr;//actual current pin level.
}

