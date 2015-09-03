
/*
 External ADC Module (adc_external & ADCX)
 
 Nicolas Williamson and Jason Cortell - March/April 2009
 Cornell University
 
*/

#include <includes.h>

// Variables
struct adcx_exponential_filter adcx_filters[16];
unsigned char adcx_index = 0;
unsigned char* adcx_convert_schedule = 0;
volatile int adcx_status = ADCX_DONE;

//Writes the byte to ADC
void adcx_write(unsigned char data){
	if (!(SSPSR & (1<<1))){ //Transmit buffer full; TNF (Transmit not full) bit 1
		//ADCX_ERROR_TRANSMIT_BUFFER_FULL
	} 
	SSPDR = data;
}

//Write byte to selected register
void adcx_register_write(unsigned char reg, unsigned char w_data){	
	while(!(SSPSR & (1<<1))){}  // wait if transmit buffer is full
	SSPDR = reg;               // register mode, write, 8 bit
 	while(!(SSPSR & (1<<1))){}  // wait if transmit buffer is full
	SSPDR = w_data;	
}

//Reads in an ADC register
unsigned char adcx_register_read(unsigned char reg){
	unsigned char read_in;
	while (!(SSPSR & (1<<1))){}
	SSPDR = (1<<6)|reg;
	while (!(SSPSR & (1<<1))){}
	SSPDR = 0x00; 
	while ((SSPSR & (1<<4))){}
	read_in = SSPDR;
	read_in = SSPDR;  							
	return read_in;
}

//Reads in a byte from the buffer
unsigned char adcx_read_buffer(void){
	unsigned char temp = 0;
	if ((SSPSR & (1<<2))){ // If the Rx buffer is Not Empty (RNE), read data
		temp = SSPDR;
	}
	else {
		//ADCX_ERROR_RECEIVE_BUFFER_EMPTY;
	}
	return temp; 
}

//Converts all of the channels in the schedule
void adcx_convert_all(void){
	unsigned char command;

	if (adcx_status != ADCX_DONE){ 
		//ADCX_ERROR_PREVIOUS_CONVERSION_INCOMPLETE;
		//MCU_LED_RED_ON; //turn on red led
	}
	adcx_status = ADCX_NOT_DONE;
	adcx_index = 0;

	command = ((1<<7)|(adcx_convert_schedule[adcx_index+1]<<4)|adcx_convert_schedule[adcx_index]);
	//Start first conversion; send out ADC convert command with desired gain and channel
 	SSPDR = command; 
	adcx_index = 2;
}

//*******************************************************************************
// ADC interrupt function
//*******************************************************************************
void adcx_convert_done(void) __irq{
 
	int MSB = 0, LSB = 0;
	volatile int temp;
	short bit_data;
	struct adcx_exponential_filter* f;
	
	// ****** CLEAR BUFFERS ******
	if ((SSPSR & (1<<2))){ // If the Rx buffer is Not Empty, read data
		temp = SSPDR; //Clear out junk from command
	} 	
	if (adcx_index >= 4){
		if ((SSPSR & (1<<2))){ // If the Rx buffer is Not Empty, read MSB
			MSB = SSPDR;
		}
		if ((SSPSR & (1<<2))){ // If the Rx buffer is Not Empty, read LSB
			LSB = SSPDR;
		}
		bit_data=((MSB<<8)|LSB)>>2;	//Convert data to 14 bits, right justified (Macro here? Abstraction?)
		
		// Add bit_data to the correct filter
		f = &adcx_filters[adcx_convert_schedule[adcx_index-4]];
		if (f->average == 0)
			f->average = bit_data << 14;
		else
			f->average = f->average - (f->average >> f->coeff) + (bit_data << (14 - f->coeff));
		f->count = f->count + 1;
	}
	if (adcx_status == ADCX_NOT_DONE){ //Running normally
		if (adcx_convert_schedule[adcx_index] == NULL){
			adcx_status = ADCX_FINISHING;
		}
		// ****** SEND COMMAND AND DUMMYS ******
//		SSPCR1 &= ~(1<<1); //disable SSP until we finish adding stuff to the buffer
		if (adcx_status != ADCX_FINISHING){ //Normal Commands
			SSPDR = ((1<<7)|(adcx_convert_schedule[adcx_index+1]<<4)|adcx_convert_schedule[adcx_index]); //Convert next channel
		}	
		else if (adcx_status == ADCX_FINISHING){
			SSPDR = ((1<<7)|(1<<3)|0); //Dummy Convert to get next interrupt
		}
		SSPDR = (0x00); //Dummy
		SSPDR = (0x00); //Dummy to clock in conversion	
//		SSPCR1 |= (1<<1); //enable SSP and begin transmission
		
		adcx_index = adcx_index + 2; //Update adcx_index (+2 for channel and gain)
	}
	else{
		adcx_status = ADCX_DONE;
	}		
	
	EXTINT |= (1<<1);;      	// Clear External Interrupt flag;
	VICVectAddr = 0;;          //indicates end of interrupt service
}

//Returns the raw int value from the selected filter. Note: this value is shifted left by 14
int adcx_get_int(short channel)
{
	return adcx_filters[channel].average;
}

//returns a pointer to the filter struct for the selected filter.
struct adcx_exponential_filter* adcx_get_filter(short channel)
{
	return &adcx_filters[channel];
}

//returns the scaled float value from the selected filter using given
//gains and offsets of the filters 
float adcx_get_float(short channel)
{
	return (adcx_filters[channel].average * adcx_filters[channel].gain) + adcx_filters[channel].offset;
}

//For debugging without using a scheduler, send a conversion and then
//use this function to wait until the conversion is complete.
//ex. while(!adc_done_converting()){...}
int adcx_done_converting()
{
	return (adcx_status == ADCX_DONE);
}

//sets the conversions that will be completed when calling adcx_convert_all.
//Should be set at setup
void adcx_set_convert_schedule(unsigned char* schedule)
{
	adcx_convert_schedule = schedule;
}

//initializes adc_external module
void adcx_init()
{
	int i;
		
	for (i = 0; i < 16; i++){
		adcx_filters[i].average = 0;
		adcx_filters[i].count = 0;
		adcx_filters[i].coeff = 0;
		adcx_filters[i].gain = 1.0;
		adcx_filters[i].offset = 0.0;
	}
	
/*	adcx_write(0x00);
	adcx_write(0x00);
	adcx_write(0x00);
	adcx_write(0x00);
	adcx_write(0x00);*/
	
	// Setup ADC registers
	adcx_register_write(3,4);  // two's comp output, autoreadback: MSB first, CCLK div = 1
	//adcx_register_write(6,0x0F);  //Temp; GPIO output
	adcx_register_write(7,0x00);  // Vref=2.5V, buffer and ref powered down (using external 5V reference)
	adcx_register_write(24,0x00); // SPI: MSB first, 3-wire mode, DIN stuff 
	while((SSPSR & (1<<4))){} //Wait for data to be sent
	while((SSPSR & (1<<2))){	 //Read returning dummy byte to clear out buffer.
    	adcx_read_buffer();
 	}
	
	
		
}
