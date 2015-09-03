
/*
 External ADC Module (ADC_External)
 
 Nicolas Williamson and Jason Cortell - March/April 2009
 Cornell University
 
*/

#include "includes.h"

//Global Variables
int ADC_external_status;

struct Filter filters[16] = {{0,0,ADC_EXT_DIFF_P0N1_FILTER},
		{0,0,ADC_EXT_DIFF_P2N3_FILTER},
		{0,0,ADC_EXT_DIFF_P4N5_FILTER},
		{0,0,ADC_EXT_DIFF_P6N7_FILTER},
		{0,0,ADC_EXT_DIFF_P1N0_FILTER},
		{0,0,ADC_EXT_DIFF_P3N2_FILTER},
		{0,0,ADC_EXT_DIFF_P5N4_FILTER},
		{0,0,ADC_EXT_DIFF_P7N6_FILTER},
		{0,0,ADC_EXT_CH0_FILTER},
		{0,0,ADC_EXT_CH1_FILTER},
		{0,0,ADC_EXT_CH2_FILTER},
		{0,0,ADC_EXT_CH3_FILTER},
		{0,0,ADC_EXT_CH4_FILTER},
		{0,0,ADC_EXT_CH5_FILTER},
		{0,0,ADC_EXT_CH6_FILTER},
		{0,0,ADC_EXT_CH7_FILTER}};

float gains[16] = {ADC_EXT_DIFF_P0N1_GAIN, ADC_EXT_DIFF_P2N3_GAIN, ADC_EXT_DIFF_P4N5_GAIN, ADC_EXT_DIFF_P6N7_GAIN,
		ADC_EXT_DIFF_P1N0_GAIN, ADC_EXT_DIFF_P3N2_GAIN, ADC_EXT_DIFF_P5N4_GAIN, ADC_EXT_DIFF_P7N6_GAIN,
		ADC_EXT_CH0_GAIN, ADC_EXT_CH1_GAIN, ADC_EXT_CH2_GAIN, ADC_EXT_CH3_GAIN,
		ADC_EXT_CH4_GAIN, ADC_EXT_CH5_GAIN, ADC_EXT_CH6_GAIN, ADC_EXT_CH7_GAIN};

float offsets[16] = {ADC_EXT_DIFF_P0N1_OFFSET, ADC_EXT_DIFF_P2N3_OFFSET, ADC_EXT_DIFF_P4N5_OFFSET, ADC_EXT_DIFF_P6N7_OFFSET,
		ADC_EXT_DIFF_P1N0_OFFSET, ADC_EXT_DIFF_P3N2_OFFSET, ADC_EXT_DIFF_P5N4_OFFSET, ADC_EXT_DIFF_P7N6_OFFSET,
		ADC_EXT_CH0_OFFSET, ADC_EXT_CH1_OFFSET, ADC_EXT_CH2_OFFSET, ADC_EXT_CH3_OFFSET,
		ADC_EXT_CH4_OFFSET, ADC_EXT_CH5_OFFSET, ADC_EXT_CH6_OFFSET, ADC_EXT_CH7_OFFSET};

//Internal Scheduling Variables
short index = 0;
short* convert_schedule;

//Writes the byte to ADC
void ADC_external_write(unsigned char data){
	if (!SSPSR_bit.TNF){
		//ERROR - Transmit buffer full
	} //(TNF = transmit not full) 
	SSPDR = data;
}

//Write byte to selected register (3-7, 24)
void ADC_external_register_write(unsigned char reg, unsigned char w_data){	
	while(!SSPSR_bit.TNF){}  // wait if transmit buffer is full
  	SSPDR=reg;               // register mode, write, 8 bit
 	while(!SSPSR_bit.TNF){}  // wait if transmit buffer is full
  	SSPDR=w_data;	
	while(SSPSR_bit.BSY){}
}

//Reads in an ADC register
//reg - int between 0 and 7, or 24
unsigned char ADC_external_register_read(int reg){
	unsigned char read_in;
	while (SSPSR_bit.RNE){read_in = SSPDR;} //Empty Rx buffer
	while (SSPSR_bit.BSY){}
	ADC_external_write((1<<6)|(reg&0x1F)); //Read operation command; Assuming they're dumb :)
	ADC_external_write(0x00);
	while (SSPSR_bit.BSY){}
	ADC_external_read_buffer();
	read_in = ADC_external_read_buffer();
	return read_in;
}

//Reads in a byte from the buffer
unsigned char ADC_external_read_buffer(void){
	unsigned char temp = 0;
	if (SSPSR_bit.RNE){ // If the Rx buffer is Not Empty (RNE), read data
		temp = SSPDR;
	}
	else {
		//printf("Buffer Empty\n");
		//Buffer was empty - ERROR
	}
	return temp; 
}

//Converts all of the channels in the schedule
void ADC_external_convert_all(short* schedule){
	unsigned char command;
	if (ADC_external_status == ADC_EXT_NOT_DONE){ // Previous conversion did not complete in time - ERROR
		//printf("PREVIOUS CONVERSION DID NOT FINISH IN TIME");
	}
	ADC_external_status = ADC_EXT_NOT_DONE;
	convert_schedule = schedule;
	index = 0;
	command = ((1<<7)|(convert_schedule[index+1]<<4)|convert_schedule[index]);
	//Start first conversion; send out ADC convert command with desired gain and channel
 	SSPDR = command; 
	index = 2;
}

//*******************************************************************************
// ADC interrupt function
//*******************************************************************************
void AtoD (void) __irq{
 
	int MSB,LSB,temp;
	short bit_data;
	struct Filter* f;
	
	// ****** CLEAR BUFFERS ******
	if (SSPSR_bit.RNE){ // If the Rx buffer is Not Empty (RNE), read data
		temp = SSPDR;
	} //Clear out junk from command
	if (index >= 4){
		if (SSPSR_bit.RNE){ // If the Rx buffer is Not Empty (RNE), read MSB
			MSB = SSPDR;
		}
		if (SSPSR_bit.RNE){ // If the Rx buffer is Not Empty (RNE), read LSB
			LSB = SSPDR;
		}
		bit_data=((MSB<<8)|LSB)>>2;	//Convert data to 14 bits, right justified
		
		// Add bit_data to the correct filter
		f = &filters[convert_schedule[index-4]];
		if (f->average == 0)
			f->average = bit_data << 14;
		else
			f->average = f->average - (f->average >> f->coeff) + (bit_data << (14 - f->coeff));
		f->count = f->count + 1;
	}
	if (ADC_external_status != ADC_EXT_FINISHING){ //Running normally
		if (convert_schedule[index] == NULL){
			ADC_external_status = ADC_EXT_FINISHING;
		}
		// ****** SEND COMMAND AND DUMMYS ******
		if (ADC_external_status != ADC_EXT_FINISHING){ //Normal Commands
			SSPDR = ((1<<7)|(convert_schedule[index+1]<<4)|convert_schedule[index]); //Convert next channel
		}	
		else if (ADC_external_status == ADC_EXT_FINISHING){
			SSPDR = ((1<<7)|(1<<3)|0); //Dummy Convert channel 0 to get next interrupt
		}
		SSPDR = (0x00); //Dummy
		SSPDR = (0x00); //Dummy to clock in conversion	

		index = index + 2; //Update index (+2 for channel and gain)
	}	
	
	EXTINT |= (1<<1);      	// Clear External Interrupt flag; writing 1 to Bit 1 (EINT1) clears
	VICVectAddr = 0;          // Dummy write to indicate end of interrupt service
}

//Returns the raw int value from the selected filter. Note: this value is shifted left by 14
int ADC_external_get_int(short channel)
{
	return filters[channel].average;
}

//returns a pointer to the filter struct for the selected filter.
struct Filter* ADC_external_get_filter(short channel)
{
	return &filters[channel];
}

//returns the scaled float value from the selected filter using given
//offsets and gains in the software_setup.h file.
float ADC_external_get_float(short channel)
{
	return (filters[channel].average * gains[channel]) + offsets[channel];
}


