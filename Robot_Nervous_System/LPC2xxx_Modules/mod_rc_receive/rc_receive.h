#ifndef __RC_RECEIVE_H__
#define __RC_RECEIVE_H__

/**
  Holds the info of a channel.
*/
typedef struct rcx_data{
  int prev; /**< The previous level of a capture event (0 = low, >0 = high). */
  int start; /**< The previous captured value of the timer value from a rising edge. */
  int value; /**< Average pulse width. */
  int overflows; /**< Number of overflows occuring since calculation of pulse width. */
} RCX_DATA;

/**
  The rc receive channels.
*/
typedef enum rcx_channels{
  RCX_CHAN_0 = 0,
  RCX_CHAN_1,
  RCX_CHAN_2,
  RCX_CHAN_3,
  RCX_LAST
} RCX_CHAN;

//Functions
void rcx_init(int chan_0, int chan_1, int chan_2, int chan_3);
void rcx_isr(void) __irq;
float rcx_get_chan_0(void);
float rcx_get_chan_1(void);
float rcx_get_chan_2(void);
float rcx_get_chan_3(void);
float rcx_get_chan(RCX_CHAN chan);
static void rcx_update(RCX_CHAN chan, int curr, int cap_value, int flag_overflow);

/* HARDWARE SETUP
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
*/

#endif  //__RC_RECEIVE_H__
