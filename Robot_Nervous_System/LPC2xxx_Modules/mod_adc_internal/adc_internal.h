/* 

  @file adc_internal.h
  
  @author Nicolas Champagne-Williamson

*/

#ifndef __H_ADC_INT__
#define __H_ADC_INT__


/**
  Struct for an exponential filter
*/
typedef struct adci_exponential_filter{
	int average; /**< Current value of the filtered data. */
	int count; /**< How many data have been added to the filter. */
	char coeff; /**< each value added to the filter is divided by 2^coeff. */
} ADCI_FILTER;

/**
  The possible ADCI channels on the ARM7 chip.
*/
typedef enum adci_channels{
  ADCI_CH_0 = 0,
  ADCI_CH_1,
  ADCI_CH_2,
  ADCI_CH_3
} ADCI_CHAN;

/**
  The status of the internal adc module.
*/
typedef enum adci_stati{
  ADCI_DONE = 0, /**< The conversions are all done. */
  ADCI_NOT_DONE /**< Currently in the middle of converting. */
} ADCI_STATUS;

#define ADCI_NULL (-1) /**< A possible coeff for @ref adci_init. Don't use this channel. */
#define ADCI_NO_FILTER (0) /**< A possible coeff for @ref adci_init. Use this channel without any filtering. */


// ********** Function Declarations **********
//Public Functions
void adci_init(int coeff0, int coeff1, int coeff2, int coeff3);
int adci_get_result(ADCI_CHAN channel); //Returns the int value from the given channel's filter
void adci_convert_all(void); //Converts the given channel and adds the value to the filter
void adci_isr(void) __irq; //interrupt called when an ADC conversion has completed
//Private Functions
void adci_efilter_add(volatile ADCI_FILTER *f, short num); //Adds the given value to the given filter
void adci_init_channel(ADCI_CHAN chan, int coeff); //initializes the adci channel
volatile ADCI_FILTER* adci_get_filter(ADCI_CHAN); //returns the filter of the given channel
void adci_convert_next(void);


/* HARDWARE SOURCE SETUP - Only put channels you wish to read from.
	
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

*/

#endif

