/**
  @file abs_enc.c
  Absolute Encoder module.

  Reads from two magnetic encoders. 
  For each call to @ref ae_update, all encoders set by ae_init_encoder_* are read from.
  
  Make sure to call ae_init_encoder_* (* = 1 or 2) for the encoders you wish to use.
  Then call @ref ae_update from every row of the schedule.
  
  B2A_MC: J3 = encoder 1
          J9 = encoder 2
          
  Example Hardware and Register Setup:
  @code
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
  @endcode
  
  The isr @ref ae_isr will also need to be setup:
  @code
  // ************ PRIORITY 10 ******************
	//Absolute Encoder
	VICVectAddr10 = (unsigned long)ae_isr;
	VICVectCntl10 = 0x20 | VIC_SPI0; 
	VICIntEnable = 1 << VIC_SPI0; 
  @endcode 

	@author Nicolas Williamson
	@author Philip Johnson
	@date July 2009
*/

#include <includes.h>

static volatile AE_ID ae_current_encoder; /**< The current encoder data is being read from. */
static AE_ENCODER ae_encoders[2]; /**< Currently only using two encoders. */
static volatile AE_STATUS ae_status = AE_DONE; /**< The status of the absolute encoder module. */

/**
  Initializes an encoder. Call this for each encoder you wish to use.
  @param encoder_id The id of the encoder to initialize.
  @param zero The zero offset for the encoder. Check what this by starting with it at zero,
  then moving the shaft to the desired zero position. The output at that position should be
  the zero.
  @param rpm The max rpm of the shaft the encoder is attached to.
  @param cpr The counts per revolution of the encoder.
*/
void ae_init_encoder(AE_ID encoder_id, int zero, int rpm, int cpr){
  if (encoder_id < AE_LAST){
    AE_ENCODER* encoder = &(ae_encoders[encoder_id]);
    encoder->read = 1;
    encoder->value = 0;
    encoder->zero = zero;
    encoder->delta = (int)((float)rpm * (float)cpr / (60.0 * 1000.0 * SCHED_SPEED)) + 5;
  } else {
    error_occurred(ERROR_AE_INVALID_ID);
  }
}

/**
  Updates the absolute encoder data. CALL EVERY SCHEDULE ROW.
  Reads from the encoders are interrupt driven.
  If you only want to read once, you can call @ref ae_update, and
  then use @ref ae_wait which will block until the data has been read.
*/
void ae_update(void)
{																										   	
	if (ae_encoders[AE_1].read){ //if reading from encoder 1 or both encoder 1 and 2
		ae_read(AE_1); //begin read from encoder 1
    ae_status = AE_NOT_DONE; 
	} else if (ae_encoders[AE_2].read){ //if only reading from encoder 2
		ae_read(AE_2); //begin read from encoder 2
    ae_status = AE_NOT_DONE;
	} //not reading any encoders
}

/**
  Reads in the data for the given encoder.
  @param encoder_id The encoder to read from. Either AE_1 (J3) or AE_2 (J9)
*/
static void ae_read(AE_ID encoder_id)
{
	S0SPCR |= (1<<7); //enable interrupts
	switch (encoder_id){
		case AE_1: FIO0CLR = (1<<13); break; //set P0.13 low to read from J3
		case AE_2: FIO0SET = (1<<13); break; //set P0.13 high to read from J9
		default: error_occurred(ERROR_AE_INVALID_ID); break;
	}
	ae_current_encoder = encoder_id; //we are currently waiting for data for this encoder
	S0SPDR = 0; //send dummy data to begin transfer
}
	
/**
  Returns the position of the given encoder.
  Position = data - zero_offset.
  @param encoder_id The encoder to read from, either AE_1 (J3) or AE_2 (J9).
  @return The latest position of the encoder.
*/
int ae_get_pos(AE_ID encoder_id)
{
  if (encoder_id < AE_LAST){ //valid encoder id
    AE_ENCODER* encoder = &(ae_encoders[encoder_id]);
    return encoder->value - encoder->zero;
  } else {
    error_occurred(ERROR_AE_INVALID_ID);
    return 0;
  }
}	

/**
  Blocks until the current encoder update is complete.
  Only use in init functions, such as init_software or init_values.
*/
void ae_wait(void){
  while (ae_status != AE_DONE) {} 
}

/**
  The interrupt service routine for the absolute encoder module.
  When one encoder has completed its readout, causes an interrupt which will
  read the data in software.
*/
void ae_isr(void) __irq
{
	int int_state = S0SPSR;
  int new_data;
  AE_ENCODER* enc;
  
	S0SPINT = 1;						//clear interrupt flag in SPI register
	
  // ***** READ NEW DATA ***** //
  enc = &(ae_encoders[ae_current_encoder]);
	new_data = S0SPDR&0x1FFF; //read data from SPI
  //already taking data
  //otherwise, need to initialize (it'll think the initial data is a spike and not initialize properly)
  if (enc->value != 0){ 
    //make sure data is valid
    if (new_data > enc->value + enc->delta){ //over bounds, limit
      new_data = enc->value + enc->delta;
      error_occurred(ERROR_AE_SPIKE); //report error
    } else if (new_data < enc->value - enc->delta){ //below bounds, limit
      new_data = enc->value - enc->delta;
      error_occurred(ERROR_AE_SPIKE); //report error
    }
  }
  
  enc->value = new_data;
  
  // ***** START NEXT ENCODER, OR FINISHED ***** //
  if (ae_current_encoder == AE_1){ //currently reading data from encoder 1
    if (ae_encoders[AE_2].read){ //if also using encoder 2
			ae_read(AE_2); //begin read from encoder 2
		} else { //otherwise we were only reading from encoder 1
      ae_status = AE_DONE; //so we're done!
			S0SPCR &= ~(1<<7); //disable interrupts
		}
	} else if (ae_current_encoder == AE_2){ //currently reading data from encoder 2
		ae_status = AE_DONE; //always do encoder 1 then 2, so we're done!
		S0SPCR &= ~(1<<7);	//disable interrupts
	}
  
	VICVectAddr = 0;					//acknowledge interrupt, clear vector to interrupt function 	
}


