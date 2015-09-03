/*
	abs_enc.h
	
	Nicolas Williamson - July 2009
*/

/* HOW TO USE
  1) Copy the HARDWARE SETUP code into init_hardware()
  2) Call ae_init_encoder(...) from init_software()
  4) Copy the INTERRUPT SETUP code into init_interrupts()
    - You might have to change the priority.
  3) Call ae_update() from the schedule.
    - If you need to read the value right away, call ae_wait afterwards
      to be sure the update has completed before reading a value
  
*/

/* HARDWARE SETUP

  // *******************************************************************************
	// SPI0 Setup for Absolute Encoder 
	// *******************************************************************************
	//Set Pinsel bits for controlling SPI0:
  PINSEL0 &= ~(3<<8); //clear pin P0.4
  PINSEL0 &= ~(3<<10); //clear pin P0.5 
  PINSEL0 &= ~(3<<26); //clear pin P0.5
	PINSEL0 |= (1<<8);		//SCKO for SPI0 (clock for data transfer)
	PINSEL0 |= (1<<10);	    //MISO for SPI0 (master in slave out data)
	PINSEL0 |= (0<<26);		//set P0_13 to GPIO -- set pin to output
	FIO0DIR |= (1<<13);		//this pin controls multiplexers (SN74LVC1G3157) via the SN74LVC1G07 level shitfer
	    	                   		//when P0_13 is low J3 encoder can be read via SPI
									//when   ''  is high J9 encoder can be read 
	//Set Up SPI Register:
	S0SPCR |= (1<<2) | (0<<8) | (1<<9) | (1<<10) | (1<<11); 	//recieve 14 bits of data
	S0SPCR |= (1<<4);				//set clock polarity to active low
	S0SPCR |= (1<<5);				//activate master mode
	S0SPCCR = 60;	//set data transfer rate; PCLK / S0SPCCR = 60MHz / 60 = 1MHz
  
*/

/* INTERRUPT SETUP

  // ************ PRIORITY 10 ******************
	//Absolute Encoder
	VICVectAddr10 = (unsigned long)ae_isr;
	VICVectCntl10 = 0x20 | VIC_SPI0; 
	VICIntEnable = 1 << VIC_SPI0;   

*/

#ifndef __H_ABS_ENC__
#define __H_ABS_ENC__

/**
  The possible states of the absolute encoder module.
*/
typedef enum ae_states{
  AE_NOT_DONE = 0, /**< The absolute encoder has not completed reading all positions. */
  AE_DONE = 1 /**< The absolute encoder has completed reading all positions. */
} AE_STATUS;

/**
  The possible encoders to read from. 
*/
typedef enum ae_encoders{
  AE_1 = 0, /**< J3 on the B2A_MC board. */
  AE_2 = 1, /**< J9 on the B2A_MC board. */
  AE_LAST
} AE_ID;

/**
  All of the necessary information for an encoder.
*/
typedef struct ae_encoder{
  int read; /**< Whether we're reading from this encoder or not. */
  int zero; /**< The zero offset of the encoder. */
  int delta; /**< The maximum change from one value to the next. */
  int value; /**< The latest data read from the encoder. */
} AE_ENCODER;

//Functions
void ae_init_encoder(AE_ID encoder_id, int zero, int rpm, int cpr);
void ae_update(void);
void ae_read(AE_ID encoder);
int ae_get_pos(AE_ID encoder);
void ae_wait(void); //waits until the current update has completed
void ae_isr(void) __irq;

#endif
