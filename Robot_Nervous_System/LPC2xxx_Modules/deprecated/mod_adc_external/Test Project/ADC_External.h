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

/* HARDWARE SETUP - Put this into Hardware_Setup.c setup_hardware() function

	// *******************************************************************************
	// Initialize SSP/SPI1 For External ADC
	// *******************************************************************************
	
	SSPCR1 = 0;					// Disable SSP to allow setting changes
	PCONP_bit.PCSPI1=0;   // power setting: disable SPI1
	PCONP_bit.PCSSP=1;    // power setting: enable SSP
	SSPCR0_bit.DSS= 0x7;     // data size: 8 bits
	SSPCR0_bit.FRF=0;     // SPI mode
	SSPCR0_bit.CPOL=1;    // sclk high when idle
	SSPCR0_bit.CPHA=1;    // sample on the second edge (rising edge)
	SSPCPSR_bit.CPSDVSR= 10; // prescale divider
	SSPCR0_bit.SCR= 2;    // bit frequency = PCLK/(CPSDVSR*(SCR+1)) = 2Mbps	
	PINSEL1 &= ~0x3FC;  // clear P0.17~P0.20;
	PINSEL1 |= 0x2A8;   // P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: SSEL (auto SSEL1)
	IO0DIR_bit.P0_20=1;       // set manual SSEL1 as output
	IO0SET_bit.P0_20=1;       // SSEL1 high
	SSPCR1_bit.SSE=1;        // enable SSP/SPI1
	
	PINSEL0_bit.P0_12 = 0;  //ADC reset line set for GPIO operation
	PINSEL2 &= ~(1<<3);	  // set trace port for GPIO use (bit 3 = 0)
  						  //Should happen automatically at startup, but just in case...
						  //GPIO P1_16 is the ADC convert line
 	FIO0DIR_bit.P0_12=1;    // set P0_12 (ADC reset) to be a digital output
	FIO1DIR_bit.P1_16=1;    // set P1_16 (ADC convert) to be a digital output
	
	FIO0SET_bit.P0_12=1; 	 //set ADC reset line high
 	FIO1CLR_bit.P1_16=1; 	 //set ADC convert line low
	
	// Setup ADC registers

	ADC_external_register_write(3,(1<<2));  // two's comp output, autoreadback: MSB first, CCLK div = 1
	//ADC_external_register_write(6,0x0F);  //Temp; GPIO output
	ADC_external_register_write(7,0x00);  // Vref=2.5V, buffer and ref powered down (using external 5V reference)
	ADC_external_register_write(24,0x00); // SPI: MSB first, 3-wire mode, DIN stuff 
	while(SSPSR_bit.BSY){} //Wait for data to be sent
	while(SSPSR_bit.RNE){	 //Read returning dummy byte to clear out buffer.
    	ADC_external_read_buffer();
 	}	
 
*/

/* INTERRUPT SETUP - Put this into the Interrupt Handlers section of Hardware_Setup.c setup_hardware() function
 
	PINSEL0_bit.P0_3 = 3;				// Set Pin3 to EINT1 (ADCBUSY line)
	EXTMODE |= (1<<1);						 // Set EINT1 (Bit 1) to be edge-sensitive
	EXTPOLAR |= 0x0;						// Set EINT1 to be sensitive on the falling-edge
	EXTINT = 1;							 // Clear all interrupt register flags
 	VICVectAddr0 = (unsigned long)AtoD;  // Use slot 0, the highest vectored IRQ priority.
 	VICVectCntl0 = (1<<5) | 15;             // (1<<5) is the interrupt enable bit, 15 is EINT1 interrupt
	VICIntEnable |= (1<<15); //Enable EINT1 (bit 15)
	
*/

/* SOFTWARE SETUP - Put this into Software_Setup.h 
 
 struct Filter{
 int average;
 int count;
 int coeff;
 };
 
*/

/* SOFTWARE SETUP - Put into Software_Setup.h with custom values
 
 // ***********************************************
 // ADC_External setup
 // ***********************************************
 
 // The gains for each channel's filter (floats): (m value in y=mx+b)
 #define ADC_EXT_CH0_GAIN 1
 #define ADC_EXT_CH1_GAIN 1
 #define ADC_EXT_CH2_GAIN 1
 #define ADC_EXT_CH3_GAIN 1
 #define ADC_EXT_CH4_GAIN 1
 #define ADC_EXT_CH5_GAIN 1
 #define ADC_EXT_CH6_GAIN 1
 #define ADC_EXT_CH7_GAIN 1
 #define ADC_EXT_DIFF_P0N1_GAIN 1
 #define ADC_EXT_DIFF_P2N3_GAIN 1
 #define ADC_EXT_DIFF_P4N5_GAIN 1
 #define ADC_EXT_DIFF_P6N7_GAIN 1
 #define ADC_EXT_DIFF_P1N0_GAIN 1
 #define ADC_EXT_DIFF_P3N2_GAIN 1
 #define ADC_EXT_DIFF_P5N4_GAIN 1
 #define ADC_EXT_DIFF_P7N6_GAIN 1
 
 // The offsets for each channel's filter (floats): (b value in y=mx+b)
 #define ADC_EXT_CH0_OFFSET 0
 #define ADC_EXT_CH1_OFFSET 0
 #define ADC_EXT_CH2_OFFSET 0
 #define ADC_EXT_CH3_OFFSET 0
 #define ADC_EXT_CH4_OFFSET 0
 #define ADC_EXT_CH5_OFFSET 0
 #define ADC_EXT_CH6_OFFSET 0
 #define ADC_EXT_CH7_OFFSET 0
 #define ADC_EXT_DIFF_P0N1_OFFSET 0
 #define ADC_EXT_DIFF_P2N3_OFFSET 0
 #define ADC_EXT_DIFF_P4N5_OFFSET 0
 #define ADC_EXT_DIFF_P6N7_OFFSET 0
 #define ADC_EXT_DIFF_P1N0_OFFSET 0
 #define ADC_EXT_DIFF_P3N2_OFFSET 0
 #define ADC_EXT_DIFF_P5N4_OFFSET 0
 #define ADC_EXT_DIFF_P7N6_OFFSET 0
 
 // Filter Coefficients (int between 0 and 14 inclusive): For each channel specify 
 // the exponential averaging filter coefficient
 // 0: No Filter
 // > 1: Each new value added is divided by 2^this number
 #define ADC_EXT_CH0_FILTER 0
 #define ADC_EXT_CH1_FILTER 0
 #define ADC_EXT_CH2_FILTER 0
 #define ADC_EXT_CH3_FILTER 0
 #define ADC_EXT_CH4_FILTER 0
 #define ADC_EXT_CH5_FILTER 0
 #define ADC_EXT_CH6_FILTER 0
 #define ADC_EXT_CH7_FILTER 0
 #define ADC_EXT_DIFF_P0N1_FILTER 0
 #define ADC_EXT_DIFF_P2N3_FILTER 0
 #define ADC_EXT_DIFF_P4N5_FILTER 0
 #define ADC_EXT_DIFF_P6N7_FILTER 0
 #define ADC_EXT_DIFF_P1N0_FILTER 0
 #define ADC_EXT_DIFF_P3N2_FILTER 0
 #define ADC_EXT_DIFF_P5N4_FILTER 0
 #define ADC_EXT_DIFF_P7N6_FILTER 0
 
 */
 
 // Gains for ADC_external_convert function
#define ADC_EXT_GAIN1 0
#define ADC_EXT_GAIN2 1
#define ADC_EXT_GAIN4 2
#define ADC_EXT_GAIN5 3
#define ADC_EXT_GAIN8 4
#define ADC_EXT_GAIN10 5
#define ADC_EXT_GAIN16 6
#define ADC_EXT_GAIN20 7

// Channels to read in from
#define ADC_EXT_CH0 8
#define ADC_EXT_CH1 9
#define ADC_EXT_CH2 10
#define ADC_EXT_CH3 11
#define ADC_EXT_CH4 12
#define ADC_EXT_CH5 13
#define ADC_EXT_CH6 14
#define ADC_EXT_CH7 15
// Differential channels to read in from:
// 'P' is positive channel, and 'N' is negative channel
// Ex. 'ADC_EXT_DIFF_P1N2' uses positive ch1 and negative ch2 for the differential
#define ADC_EXT_DIFF_P0N1 0
#define ADC_EXT_DIFF_P2N3 1
#define ADC_EXT_DIFF_P4N5 2
#define ADC_EXT_DIFF_P6N7 3
#define ADC_EXT_DIFF_P1N0 4
#define ADC_EXT_DIFF_P3N2 5
#define ADC_EXT_DIFF_P5N4 6
#define ADC_EXT_DIFF_P7N6 7

//Flags
#define ADC_EXT_NOT_DONE 1
#define ADC_EXT_DONE 2
#define ADC_EXT_FINISHING 3

//Function Declarations
void ADC_external_register_write(unsigned char reg, unsigned char w_data);
void ADC_external_convert_all(short* schedule);
int ADC_external_get_int(short channel);
float ADC_external_get_float(short channel);
struct Filter* ADC_external_get_filter(short channel);
unsigned char ADC_external_read_buffer(void);
void AtoD(void) __irq;	   //interrupt service routine for ADC
 
#endif
 
