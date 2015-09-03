/*
 *  ADC_External.h
 *  
 *
 *  Created by Nicolas Williamson on 3/20/09.
 *  Copyright 2009 Cornell University. All rights reserved.
 *
 */

#ifndef ADC_EXTERNAL
#define ADC_EXTERNAL

struct adcx_exponential_filter{
	int average;
	int count;
	char coeff;
	float gain;
	float offset;
};

/* HARDWARE HEADER SETUP - Put this into hardware_setup.h

#define ADCX_TRANSMIT_BUFFER_FULL !SSPSR_bit.TNF
#define ADCX_TRANSMIT_BUFFER SSPDR
#define ADCX_BUSY SSPSR_bit.BSY
#define ADCX_RECEIVE_BUFFER_NOT_EMPTY SSPSR_bit.RNE
#define ADCX_RECEIVE_BUFFER SSPDR
#define ADCX_READ_COMMAND (1<<6)
#define ADCX_DUMMY (0x00)
#define ADCX_EXTERNAL_INTERRUPT_CLEAR EXTINT |= (1<<1); // writing 1 to Bit 1 (EINT1) clears
#define ADCX_END_OF_INTERRUPT_SERVICE VICVectAddr = 0; //Dummy write to indicate end of interrupt service

*/

/* HARDWARE SOURCE SETUP - Put this into hardware_setup.c setup_hardware() function

    // *******************************************************************************
	// Initialize SSP/SPI1 For External ADC
	// *******************************************************************************
	
	SSPCR1 = 0;					// Disable SSP to allow setting changes
	PCONP &=	~(1<<10);   // power setting: disable SPI1	(bit 10)
	PCONP |=	(1<<23);    // power setting: enable SSP (bit 23)
	SSPCR0 |=	(7<<0);     // data size: 8 bits (bits 0-3 = 7 means 8bit data)
	SSPCR0 &=	~(1<<4);     // SPI mode (bits 4-5 = 0)
	SSPCR0 |=	(1<<6);    // sclk high when idle
	SSPCR0 |=	(1<<7);    // sample on the second edge (rising edge)
	SSPCPSR |=	(2<<0); // prescale divider
	SSPCR0 |=	(2<<8);    // bit frequency = PCLK/(CPSDVSR*(SCR+1)) = 2Mbps	
	PINSEL1 &=	~0x3FC;  // clear P0.17~P0.20;
	PINSEL1 |=	0x2A8;   // P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: SSEL (auto SSEL1)
	SSPCR1 |=	(1<<1);        // enable SSP/SPI1
	
	PINSEL0 &= ~(3<<24);  //ADC reset line set for GPIO operation (po.12 bits 24/25)
	PINSEL2 &= ~(1<<3);	  // set trace port for GPIO use (bit 3 = 0)
  						  //Should happen automatically at startup, but just in case...
						  //GPIO P1_16 is the ADC convert line
 	FIO0DIR |= (1<<12);    // set P0_12 (ADC reset) to be a digital output
	FIO1DIR |= (1<<16);    // set P1_16 (ADC convert) to be a digital output
	
	FIO0SET = (1<<12); 	 //set ADC reset line high
 	FIO1CLR = (1<<16); 	 //set ADC convert line low
 
*/

/* INTERRUPT SETUP - Put this into the Interrupt Handlers section of hardware_setup.c setup_hardware() function
 
	PINSEL0_bit.P0_3 = 3;				// Set Pin3 to EINT1 (ADCBUSY line)
	EXTMODE |= (1<<1);						 // Set EINT1 (Bit 1) to be edge-sensitive
	EXTPOLAR &= ~(1<<1);						// Set EINT1 to be sensitive on the falling-edge
	EXTINT = 1;							 // Clear all interrupt register flags
 	VICVectAddr0 = (unsigned long)adcx_convert_done;  // Pick a vectored slot for the IRQ (this one uses 0)
 	VICVectCntl0 = (1<<5) | 15;             // (1<<5) is the interrupt enable bit, 15 is EINT1 interrupt
	VICIntEnable |= (1<<15); //Enable EINT1 (bit 15)
	
*/

/* SOFTWARE HEADER SETUP - Put into software_setup.h with custom values
 
 // ***********************************************
 // ADC_External setup
 // ***********************************************
 
 //ERRORS - These should be set to error function calls with the correct values for the board
 #define ADCX_ERROR_TRANSMIT_BUFFER_FULL error(.....);
 #define ADCX_ERROR_RECEIVE_BUFFER_EMPTY error(.....);
 #define ADCX_ERROR_PREVIOUS_CONVERSION_INCOMPLETE error(.....);
 
 */
 
// Gains for adcx_convert function
#define ADCX_GAIN1 0
#define ADCX_GAIN2 1
#define ADCX_GAIN4 2
#define ADCX_GAIN5 3
#define ADCX_GAIN8 4
#define ADCX_GAIN10 5
#define ADCX_GAIN16 6
#define ADCX_GAIN20 7

// Channels to read in from
#define ADCX_CH0 8
#define ADCX_CH1 9
#define ADCX_CH2 10
#define ADCX_CH3 11
#define ADCX_CH4 12
#define ADCX_CH5 13
#define ADCX_CH6 14
#define ADCX_CH7 15
// Differential channels to read in from:
// 'P' is positive channel, and 'N' is negative channel
// Ex. 'ADCX_DIFF_P1N2' uses positive ch1 and negative ch2 for the differential
#define ADCX_DIFF_P0N1 0
#define ADCX_DIFF_P2N3 1
#define ADCX_DIFF_P4N5 2
#define ADCX_DIFF_P6N7 3
#define ADCX_DIFF_P1N0 4
#define ADCX_DIFF_P3N2 5
#define ADCX_DIFF_P5N4 6
#define ADCX_DIFF_P7N6 7

//Flags
#define ADCX_NOT_DONE 1
#define ADCX_DONE 2
#define ADCX_FINISHING 3

// *************** Function Declarations ****************
// Public Functions
void adcx_convert_all(void); //Starts the conversion of the supplied channels
int adcx_get_int(short channel); //Returns the int value from the given channel's filter
float adcx_get_float(short channel); //Returns the scaled float value from the given channel's filter
struct adcx_exponential_filter* adcx_get_filter(short channel); //Returns the filter for the given channel
int adcx_done_converting(void); //Returns true if done/not converting, false otherwise
void adcx_init(void); //initializes the adcx module
void adcx_set_convert_schedule(unsigned char* schedule); //sets the conversion schedule
int adcx_get_timer_counts(void);
// Private Functions
void adcx_write(unsigned char data); //Writes the given data to the external ADC
void adcx_register_write(unsigned char reg, unsigned char w_data); //Writes the given data to the given register
unsigned char adcx_register_read(unsigned char reg); //Reads in the value of the given reg on the ext ADC
unsigned char adcx_read_buffer(void); //Reads in the current value in the receive buffer
void adcx_convert_done(void) __irq; //interrupt service routine for ADC
 
#endif
 
