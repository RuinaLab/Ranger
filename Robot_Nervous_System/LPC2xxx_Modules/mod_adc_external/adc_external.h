/*

	adcx.h
	
	Nicolas Williamson and Thomas Craig - Summer 2009
	
*/

#ifndef __H_ADC_EXTERNAL__
#define __H_ADC_EXTERNAL__

// Gains for adcx_convert function
#define ADCX_GAIN_1  0
#define ADCX_GAIN_2  1
#define ADCX_GAIN_4  2
#define ADCX_GAIN_5  3
#define ADCX_GAIN_8  4
#define ADCX_GAIN_10 5
#define ADCX_GAIN_16 6
#define ADCX_GAIN_20 7

// Channels to read in from
#define ADCX_CH_0 8
#define ADCX_CH_1 9
#define ADCX_CH_2 10
#define ADCX_CH_3 11
#define ADCX_CH_4 12
#define ADCX_CH_5 13
#define ADCX_CH_6 14
#define ADCX_CH_7 15
// Differential channels to read in from:
// 'P' is positive channel, and 'N' is negative channel
// Ex. 'ADCX_CH_P1N2' uses positive ch1 and negative ch2 for the differential
#define ADCX_CH_P0N1 0
#define ADCX_CH_P2N3 1
#define ADCX_CH_P4N5 2
#define ADCX_CH_P6N7 3
#define ADCX_CH_P1N0 4
#define ADCX_CH_P3N2 5
#define ADCX_CH_P5N4 6
#define ADCX_CH_P7N6 7

//Flags
#define ADCX_NOT_DONE (1)
#define ADCX_DONE (2)
//#define ADCX_FINISHING 3

/**
  The info needed to convert a channel.
*/
typedef struct adcx_channel_config {
	unsigned char chan;
	unsigned char gain;
} ADCX_CHAN_CFG;

//Functions
void adcx_isr(void) __irq;

void adcx_add_config(unsigned char channel, unsigned char gain);
void adcx_init(void);
signed short adcx_get_result(char channel);

void adcx_conversion_wait(void);
void adcx_convert_all(void);

void adcx_convert_next(void);
void adcx_convert_cfg(ADCX_CHAN_CFG * config);

unsigned char adcx_register_read(unsigned char reg);
void adcx_register_write(unsigned char reg, unsigned char data);
void adcx_write(unsigned short data);
unsigned short adcx_read_buffer(void);

/* HARDWARE SETUP
  // *******************************************************************************
  // Initialize SSP/SPI1 For External ADC
  // *******************************************************************************
  PCONP &= ~(1<<10);  	// power setting: disable SPI1  (bit 10)
  PCONP |= (1<<21);  	// power setting: enable SSP (bit 21!!!!!!!!!!!!!!!!!!!!!!)
  SSPCR1 = 0;  	  	// Disable SSP to allow setting changes
  SSPCR0 = 0x00000000;
  SSPCR0 = (15<<0)  	// data size: 16 bits (bits 0-3 = 15 means 16bit data)
  | (1<<6)			// sclk high when idle
	| (1<<7)		  	// sample on the second edge (rising edge)
	| (2<<8);		  	// bit frequency = PCLK/(CPSDVSR*(SCR+1)) =
//  SSPCR0 &= ~(3<<4);  	// SPI mode (bits 4-5 = 0)
//  SSPCPSR &= ~(3<<0);  	//clear prescale divider
  SSPCPSR = (2<<0);  	// prescale divider  
  PINSEL1 &= ~0x3FC;  	// clear P0.17~P0.20;
//  PINSEL1 |= (2<<2)|(2<<4)|(2<<6);  	// P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: Manual SSEL 
  PINSEL1 |= (2<<2)|(2<<4)|(2<<6)|(2<<8);  // P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: auto SSEL
//  FIO0DIR |= (1<<20); //SSEL is output
//  FIO0CLR = (1<<20); //SSEL low
  SSPCR1 = (1<<1);  	// enable SSP/SPI1   
  PINSEL0 &= ~(3<<24);  //ADC reset line set for GPIO operation (P0.12 bits 24/25)
  PINSEL2 &= ~(1<<3);    // set trace port for GPIO use (bit 3 = 0)
  //Should happen automatically at startup, but just in case...
  //GPIO P1_16 is the ADC convert line
  FIO0DIR |= (1<<12);    // set P0_12 (ADC reset) to be a digital output
  FIO1DIR |= (1<<16);    // set P1_16 (ADC convert) to be a digital output  
  FIO0SET = (1<<12);    //set ADC reset line high
  FIO1CLR = (1<<16);    //set ADC convert line low
  // adc_external interrupts
  PINSEL0 &=~(3<<6);
  PINSEL0 |= (3<<6);  	  	// Set Pin3 to EINT1 (ADCBUSY line)
  EXTMODE = (EXTMODE|(1<<1)) & ~EXTMODE_RB;  	  	  // Set EINT1 (Bit 1) to be edge-sensitive
  EXTPOLAR &= ~(1<<1)&0x0F;  	  	// Set EINT1 to be sensitive on the falling-edge
  EXTINT = 0x0F;//1;
*/

#endif // __H_ADC_EXTERNAL__
