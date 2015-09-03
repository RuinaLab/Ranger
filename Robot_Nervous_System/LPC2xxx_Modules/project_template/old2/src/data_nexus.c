/*

	data_nexus.c
	
	A board specific conversion module which makes function wrappers
	and contains conversions from data types of one module to those
	required by another module. Ex, take integer ADC values and convert them
	to amps in floating or fixed-point.
	
	Nicolas Williamson - September 2009

*/

#include <includes.h>

void dn_safety(void){

}

/******** CAN RECIEVES AND TRANSMITS ***********************************/
extern CAN_FRAME_DESC error_tx_fd;
// ********************************
// Error Transmit
// ********************************
void dn_error_transmit(void){
  can_transmit(&error_tx_fd);
}

