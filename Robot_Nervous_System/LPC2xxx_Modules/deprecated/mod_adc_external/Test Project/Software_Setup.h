/*
 *  Software_Setup.h
 *  
 *	Test DCMotor_Control_Board Software Setup File
 *
 *  Created by Nicolas Williamson on 3/18/09.
 *  Copyright 2009 Cornell University. All rights reserved.
 *
 */

#ifndef SOFT_SETUP
#define SOFT_SETUP

struct Filter{
	int average;
	int count;
	int coeff;
};

//***********************************************
// ADC_External setup
//***********************************************
 
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


//Function Declaration
int setup_software(void);

#endif
