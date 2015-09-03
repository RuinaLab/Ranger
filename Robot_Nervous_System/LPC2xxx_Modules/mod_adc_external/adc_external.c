/**

   @file adc_external.c
	
  The new adc_external module.
  Has a schedule of channels to convert using the external ads7871 analog
  to digital converter chip on the B2A_MC.

  Example Hardware setup for the SSP communications:
  @code
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
  @endcode
	
	@author Nicolas Williamson 
  @author Thomas Craig 
  @date Summer 2009
	
*/

#include <includes.h>	

#define ADCX_BUSY	(SSPSR & (1<<4)) /**< The SSP bus is busy. */
#define ADCX_TNF	(SSPSR & (1<<1)) /**< SSP transmit buffer is not full. */
#define ADCX_RNE	(SSPSR & (1<<2)) /**< SSP receive buffer is not empty. */

// Variables
static volatile int adcx_index = 0; /**< The index of the channel being converted. */
static int adcx_num_configs = 0; /**< The total number of channel configurations set up. */
static ADCX_CHAN_CFG adcx_configs[16]; /**< The configurations for the channel conversions. */
static signed short adcx_results[16]; /**< Most recent results of the conversions. */ //re-precated! for direct access to results

static volatile int adcx_status = ADCX_DONE; /**< The status of the external adc module. */
static volatile int adcx_wait_count = 0; /**< Counter for @ref adcx_conversion_wait. */

static int adcx_count = 0;

/**
  The interrupt service routine for the external adc.
  Reads in the result of the last requested operation, 
  and begins the next conversion.
*/
void adcx_isr(void) __irq
{
	unsigned char msb, lsb;
	int chan;
	signed short result;
	ADCX_CHAN_CFG * cfg;
  
	EXTINT = (1<<1); // Clear External Interrupt flag;
	
  if (adcx_num_configs != 0) {
		// ****** READ N-1th RESULT ****** //
		while (ADCX_RNE){ // If the Rx buffer is Not Empty, read data
			adcx_read_buffer(); //Clear out junk from command
		}
		msb = adcx_register_read(1);
		lsb = adcx_register_read(0);
		result = ((signed short)((((unsigned short)msb)<<8)+(unsigned short)lsb))>>2;	//Convert (SIGNED data) to (14 bits, right justified)

		// ****** SAVE RESULT ****** //
		cfg = &(adcx_configs[adcx_index]);
		chan = cfg->chan;
		adcx_results[chan] = result;
	
		// ****** SEND Nth CONVERSION ****** //
		adcx_index = adcx_index + 1;
		adcx_convert_next();
	}
	VICVectAddr = 0;          //indicates end of interrupt service
}

/**
  Begins the conversion of all the channels in the schedule.
*/
void adcx_convert_all(void){
	if (adcx_status != ADCX_DONE){ 
		error_occurred(ERROR_ADCX_DNF);
	} else {
		adcx_index = 0;
		//Start first conversion
	 	adcx_convert_next();
	}

}

/**
  Begins the conversion of the next channel.
*/
void adcx_convert_next(void){
	ADCX_CHAN_CFG * cfg;
	if(adcx_index >= adcx_num_configs){
		adcx_status = ADCX_DONE; //no more configs to convert
		cfg = NULL;
    adcx_count = 0;
	} else {
		adcx_status = ADCX_NOT_DONE;
		cfg = &(adcx_configs[adcx_index]);
	}
	if(cfg != NULL) adcx_convert_cfg(cfg);
}

/**
  Begins the conversion of the given ADCX configuration.
*/
void adcx_convert_cfg(ADCX_CHAN_CFG * config)
{
  adcx_count++;
	adcx_register_write(4, (1<<7)|(config->gain<<4)|(config->chan));
}	

/**
  Write the given 16 bits to the ADCX chip.
  @param data 16 bit data to send to the ADCX chip.
*/
void adcx_write(unsigned short data){
	if (!(ADCX_TNF)){ //Transmit buffer full; TNF (Transmit not full) bit 1
		error_occurred(ERROR_ADCX_TX_FULL);
	} 
	SSPDR = data;
}

/**
  Writes a byte to a register on the ADCX chip.
  @param reg The register to write data to.
  @param data 8 bit data to send to the register.
*/
void adcx_register_write(unsigned char reg, unsigned char data){	
  short command = (reg<<8)|data;
	while(!(ADCX_TNF)){}  // wait if transmit buffer is full
	SSPDR = command;  // register mode, write, 16 bit, big endian
}

/**
  Reads in the value of a register on the ADCX chip.
  @param reg The register of the chip to read data from.
  @return The value of the register.
*/
unsigned char adcx_register_read(unsigned char reg){
	unsigned short read_in;
	while (!(ADCX_TNF)){}
	SSPDR = ((1<<6)|reg)<<8; //16 bits reg read instuction
	while (ADCX_BUSY){}
	read_in = SSPDR;  							
	return (unsigned char)(read_in & 0x00FF);
}

/**
  Reads in a byte from the receive buffer.
  @return 8 bit data from receive buffer.
*/
unsigned short adcx_read_buffer(void){
	unsigned short temp = 0;
	if (ADCX_RNE){ // If the Rx buffer is Not Empty (RNE), read data
		temp = SSPDR;
	} else {
		error_occurred(ERROR_ADCX_RX_EMPTY);
	}
	return temp; 
}

/**
  Gets the result of the latest conversion of a data channel.
  @param channel The adcx channel to get the result of.
  @return The 16 bit conversion.
*/
signed short adcx_get_result(char channel)
{
	return adcx_results[channel];
}

/**
  Initializes the external adc module.
  Call from init_software.
*/
void adcx_init(void)
{	
	// Setup ADC registers
	adcx_register_write(24,0x00); // SPI: MSB first, 3-wire mode, DIN stuff 	
	adcx_register_write(3,0x00);  // 2s-comp output, manual readback, CCLK div = 1
	adcx_register_write(7,0x00);  // Vref=2.5V, buffer and ref powered down (using external 5V reference)
	while(ADCX_BUSY){} //Wait for data to be sent
	while(ADCX_RNE){	 //Read returning dummy byte to clear out buffer.
    	adcx_read_buffer();
 	}	
}

/**
  Adds the given channel and gain to the schedule of conversions.
  @param channel The adcx channel to convert.
  @param gain The gain the ads7871 should use during conversion.
*/
void adcx_add_config(unsigned char channel, unsigned char gain) {
	if(adcx_num_configs<16) {
		adcx_configs[adcx_num_configs].chan = channel;
		adcx_configs[adcx_num_configs].gain = gain;
		adcx_num_configs++;
	} else {
		error_occurred(ERROR_ADCX_NUM_CFGS);
	}
}

/**
  Blocks until the conversions are complete.
*/
void adcx_conversion_wait(void){
	adcx_wait_count = 0;
	while(adcx_status == ADCX_NOT_DONE) {
		adcx_wait_count++;
	}
}
