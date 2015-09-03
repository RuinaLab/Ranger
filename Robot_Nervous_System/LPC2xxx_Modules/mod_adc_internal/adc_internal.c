/**
  @file adc_internal.c
	Module for using the internal analog to digital converters on the LPC2194/01 microprocessor.
  
  Example hardware and register setup:
  @code
  // *******************************************************************************
  // Initialize Internal ADC 
  // *******************************************************************************
  //Burst mode setup (old)
  //ADCR = 0x0021C700|(ADCI_CH0_READ)|(ADCI_CH1_READ << 1)|(ADCI_CH2_READ << 2)|(ADCI_CH3_READ << 3);
  //Interrupt driven setup - CLKDIV = 199; bit21 is ADC enabled; 
  //Which channels to read from; if you want to read from a channel, set its value to 1
  #define ADCI_CH0_READ 0
  #define ADCI_CH1_READ 0
  #define ADCI_CH2_READ 0
  #define ADCI_CH3_READ 0
  ADCR = (1<<21)|(199<<8) & ~ADCR_RB;
  //Set Pins to be AINx if reading from that channel
  if (ADCI_CH0_READ) {
    PINSEL1 &= ~(3<<22);
    PINSEL1 |= (1<<22);
  }  //AIN0 on pin P0.27
  if (ADCI_CH1_READ) {
    PINSEL1 &= ~(3<<24);
    PINSEL1 |= (1<<24);
  }  //AIN1 on pin P0.28 
  if (ADCI_CH2_READ) {
    PINSEL1 &= ~(3<<26);
    PINSEL1 |= (1<<26);
  }  //AIN2 on pin P0.29
  if (ADCI_CH3_READ) {
    PINSEL1 &= ~(3<<28);
    PINSEL1 |= (1<<28);
  }  //AIN3 on pin P0.30  
  //adci interrupts for conversion completions
  ADINTEN = (1<<8)//(ADCI_CH0_READ)|(ADCI_CH1_READ << 1)|(ADCI_CH2_READ << 2)|(ADCI_CH3_READ << 3)
  		& ~ADINTEN_RB;
  @endcode
	
  @author Nicolas Williamson 
  @date March 2009; Rewritten November 2009
  @version 0.2

*/
#include <includes.h>

//Variables
static volatile ADCI_FILTER adci_filters[4]; /**< The filters for the incoming data. */ 
static ADCI_CHAN adci_sched[4]; /**< The list of channels to convert. */
static ADCI_STATUS adci_status = ADCI_DONE; /**< The state of the adc conversion. */
static int adci_sched_index = -1; /**< Index of the channel list. */
static int adci_sched_size = 0; /**< Size of the filled portion of the channel array. */

/**
  Initializes the adc_internal module. 
  Coeffs are a value between -1 and 14 (inclusive) and are the coeffecients for the
  exponential filter. Passing -1 (ADCI_NULL) means don't use that channel. 
  Passing 0 (ADCI_NO_FILTER) means no filtering on that channel.
  @param coeff0 The coefficient for channel 0.
  @param coeff1 The coefficient for channel 1.
  @param coeff2 The coefficient for channel 2.
  @param coeff3 The coefficient for channel 3.
*/
void adci_init(int coeff0, int coeff1, int coeff2, int coeff3){
  adci_init_channel(ADCI_CH_0, coeff0);
  adci_init_channel(ADCI_CH_1, coeff1);
  adci_init_channel(ADCI_CH_2, coeff2);
  adci_init_channel(ADCI_CH_3, coeff3);
}

/**
  Begins the conversions of all the channels. The converter will
  interrupt upon completion of a channel and signal the next
  channel to begin converting.
*/
void adci_convert_all(void) //This will be called from the scheduler
{
	if (adci_status == ADCI_NOT_DONE) { //previous conversion did not finish
		error_occurred(ERROR_ADCI_DNF);
	}
  adci_convert_next();
}

/**
  Begins conversion of the next channel in the list.
*/
void adci_convert_next(void)
{
  adci_sched_index++;
//  printf("%i %i\n",adci_sched_index, adci_sched_size);
  if (adci_sched_index < adci_sched_size){
    adci_status = ADCI_NOT_DONE;
    ADCR &= (~0xFF);// & (~ADCR_RB); //clear chan select bits
    ADCR |= (1<<(adci_sched[adci_sched_index])) & (~ADCR_RB);
//    ADCR |= (1<<adci_next) & ~ADCR_RB; 
//    ADCR &= ~(~(1<<adci_next) & 0x0F)& ~ADCR_RB;
	  ADCR |= (1<<24) & (~ADCR_RB); //bit24 is the START conversion bit
  } else {
    adci_sched_index = -1;
    adci_status = ADCI_DONE;
  }
}

/**
  This function is called by the interrupt upon completion of a channel conversion.
  Adds the value to the filter.
*/
void adci_isr(void) __irq
{
  int reg = ADGDR;
  int chan = (reg>>24) & (0x7);
  int result = (reg>>6) & (0x3FF);

  adci_efilter_add(&(adci_filters[chan]), result); 
  

//  printf("chan: %i\n",chan);
  VICVectAddr = 0; //Interrupt Acknowledged
  adci_convert_next(); 

}

/**
  Returns the value of the filtered data for the given channel.
  @param channel The channel to read from.
  @return The filtered adc value.
*/
int adci_get_result(ADCI_CHAN channel)
{
  return adci_filters[channel].average>>14;
}

/**
  Returns the raw value in the filter for the given channel.
  Because of the exponential filter, this value will be (data << 14), or
  data * 2^14.
  @param chan The channel to read from.
  @return The raw adc value from the filter.
*/
int adci_get_raw(ADCI_CHAN chan)
{
  return adci_filters[chan].average;
}

/**
  Initializes the given channel with the filter coefficient.
  @param ch The channel to initialize.
  @param coeff The coefficient for the filter.
*/
static void adci_init_channel(ADCI_CHAN ch, int coeff)
{
  if (coeff >= 0){
	  adci_filters[ch].average = 0;	//no initial value
	  adci_filters[ch].count = 0;		//no values yet
	  adci_filters[ch].coeff = coeff;		//filter coeff
    adci_sched[adci_sched_size] = ch;
    adci_sched_size++;
  }
}

/**
  Returns a pointer to the filter struct for the given channel.
  @param channel The channel to get the filter from.
  @return A pointer to the filter. @see ADCI_FILTER
*/
volatile ADCI_FILTER* adci_get_filter(ADCI_CHAN channel)
{
	return &adci_filters[channel];
}

/**
  Adds a new value to the filter.
  Maximum value accepted is 2^16.
  @note Filter average will hold (your value)*2^14 -> bit shift back if you'd like
  to reduce resolution or to get out values that you expect.
  @param f The filter to add the new value to.
  @param num The new value to add to the filter. 
*/
void adci_efilter_add(volatile ADCI_FILTER* f, short num){
	if (num > (1<<16)){
		error_occurred(ERROR_ADCI_FILT_OOB);
	}
	if (f->average == 0)
		f->average = num << 14;
	else
		f->average = f->average - (f->average >> f->coeff) + (((int)num) << (14 - f->coeff));
	f->count = f->count + 1;
}	


