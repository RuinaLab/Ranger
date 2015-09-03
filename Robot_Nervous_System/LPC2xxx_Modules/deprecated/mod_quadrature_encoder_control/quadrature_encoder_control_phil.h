
/* Header File for Quadrature_Encoder_Control (QEC) */

#ifndef Quadrature_Encoder_Control
#define Quadrature_Encoder_Control

/* HARDWARE SETUP - Put this into Hardware_Setup.c setup_hardware() function
  	// *******************************************************************************
	// Interrupt Handlers - Quadrature Encoder Control (QEC)
	// *******************************************************************************
		QEC_init_encoder_tracking();
*/


/* SOFTWARE SETUP - Put this into Software_Setup.h
	//none -- software setup is below.
*/

//User Options:
  //Select Encoders For Position Tracking
	#define QEC_J3_ENCODER_POSITION_TRACKING 1	  //1 or zero to enable or disable
	#define QEC_J9_ENCODER_POSITION_TRACKING 0
  //Select Resolution of Encoder Tracking
  	#define QEC_J3_RESOLUTION 2					  //1,2, or 4 to select resolution used by encoders
	#define QEC_J9_RESOLUTION 4					  
  //Seclect Encoders For Velocity Tracking
	#define QEC_J3_ENCODER_VELOCITY_TRACKING 1	  //1 or zero to enable or disable
	#define QEC_J9_ENCODER_VELOCITY_TRACKING 0
  //General Function Control
//	#define QEC_PRINT 0	  			 //1 to print output of function to serial header...note: global_print_setting must = 1
//	#define QEC_DEBUG 0				 //will reset encoder count when count = counts per revolution -- used for debugging
//	#define QEC_OVERFLOW_CHECKING 0  //when 1 checks for velocity buffer overflows (and prints...if possible...to command screen)
//	#define QEC_DEBUG_SCOPE_1KHZ 0	 //use when checking velocity calculation -- use 1khz square wave on one input (instead of encoder)
	#define NUM_TO_AV 4
//	#define NUM_TO_SHIFT 2  //2^num_to_shift = num_to_av	 
//Physical Constants:
	#define QEC_COUNTS_PER_REVOLUTION 2688  //gear ratio 14:1, 4x resolution, CPR = 192
//	#define QEC_encoder_counts_per_rad 322.766
//	#define QEC_velocity_conversion_constant 184077.695
	#define QEC_2_PI 6.283185

//Timing Constants:
	#define QEC_ZERO_VELOCITY_TIMEOUTS 10			  	 //number of timer overflows before motor assumed stationary
	#define QEC_TIMER_RESET_VALUE CPUSPEED/TIMESPERSEC0	 //this is the value of T0MR0 (timer 0, match interrupt 0)
	#define QEC_TIMER_COUNTS_PER_SEC CPUSPEED

//Programatic Define Statements:
/*	#define QEC_enc_chan_A	FIO0PIN_bit.P0_22	 //pin locations on motor control board
	#define QEC_enc_chan_B	FIO0PIN_bit.P0_27
	#define QEC_enc_chan_C	FIO0PIN_bit.P0_16
	#define QEC_enc_chan_D	FIO0PIN_bit.P0_29*/
	#define QEC_ENC_CHAN_A_HI	((1<<22))	 //pin locations on motor control board
	#define QEC_ENC_CHAN_B_HI	((1<<27))
	#define QEC_ENC_CHAN_C_HI	((1<<16))
	#define QEC_ENC_CHAN_D_HI	((1<<29))
	#define QEC_BUFFER_LENGTH 300
	#define QEC_NUMBER_OF_ENCODERS (QEC_J3_ENCODER_POSITION_TRACKING + QEC_J9_ENCODER_POSITION_TRACKING)	

//Function Declarations
  //Motion Tracking
	void QEC_init_encoder_tracking(void); 
	void QEC_update_velocity_data(void);
	float QEC_get_floating_point_velocity_1(void);
	int QEC_get_position_1(void);
	int QEC_get_tot_timer_counts_1(void);
	int QEC_get_encoder_counts_moved_1(void);
	float QEC_get_position_radians_1(void);
  //Interrupt Handling
	__forceinline void QEC_update_encoder_position_A(void);
	__forceinline void QEC_update_encoder_position_B(void);
	__forceinline void QEC_update_encoder_position_C(void);
	__forceinline void QEC_update_encoder_position_D(void);
	__forceinline void QEC_time_counter(void);
	//__irq void TIMER0_ISR_Handler(void);
	__irq void FIQ_Handler(void);
	void QEC_zero_count(void);

	int QEC_get_interrupt_count(void);

 #endif
