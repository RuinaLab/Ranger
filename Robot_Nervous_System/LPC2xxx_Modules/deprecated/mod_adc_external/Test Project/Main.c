/******************************************************************************
Main Loop for LPC2194_01 DCMotor_Control_Board
 
Nic Williamson - March 2009
Cornell University
******************************************************************************/

#include "includes.h"

//*******************************************************************************
// Main Function
//*******************************************************************************
int main (void)
{	  
	short schedule[] = {9,0,10,0,11,0,NULL};
	int temp = 0;
	
	//ALWAYS CALL THESE FIRST!
	setup_hardware();
	setup_software();
	
	printf("Starting\n");

	
	while (1){
	ADC_external_convert_all(schedule);
	while (temp < 3){
	printf("Result1: %d\n",ADC_external_get_int(ADC_EXT_CH1)>>14);
	printf("Count1: %d\n",ADC_external_get_filter(ADC_EXT_CH1)->count);
	printf("Result2: %d\n",ADC_external_get_int(ADC_EXT_CH2)>>14);
	printf("Count2: %d\n",ADC_external_get_filter(ADC_EXT_CH2)->count);
	printf("Result3: %d\n",ADC_external_get_int(ADC_EXT_CH3)>>14);
	printf("Count3: %d\n",ADC_external_get_filter(ADC_EXT_CH3)->count);
	temp++;
	}
	temp = 0;
	}
}
//*******************************************************************************
// End of File
//*******************************************************************************

