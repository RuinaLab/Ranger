#include <mb_includes.h>
#include <mb_estimator.h>
#include <RangerMath.h>   // Tan()
#include <input_output.h> // LED functions

/* Local constant parameters */
static const float GYRO_RATE_BIAS = -0.0097;  // Measured in August 2015. Should be checked monthly.
static const float CONTACT_THRESHOLD = 7000.0;  // Threshold for detecting contact on the feet


/* Counters that keep track of # of clock cycles that the foot sensor readings are above the threshold */
static int in_feet_count = 0;
static int out_feet_count = 0;
/* Feet are touching the ground if the sensor reading stays above threshold value for 10 continuous clock cycles (~0.02s) */
static const int ground_duration = 10;

//int count_gyro = 0; //used for estimating the gyro rate bias

/* This will be CALLED by the main brain and it will do all the estimations */
void mb_estimator_update(void){
	clear_UI_LED();
	if(!init){
		filter_init();
		int_init_ang_rate();
		init = 1;
	}
	
	//run butterworth filters 
 	filter_hip_rate();	//run filter on hip_rate; this estimated hip_rate is less noisy than differenciating from hip angle
	//filter_hip_ang();	//run filter on hip_angle then differentiate it to estimate the hip_rate 
	filter_hip_motor_rate();	//run filter on hip_motor_rate	
	filter_foot_sensor();	//run filter on all foot contact sensors
	filter_foot_data();	//run filter on ankle angles & angular rates
	filter_gyro_rate();
	FI_on_ground();
	FO_on_ground();
	
	/*count_gyro = 0;
	if(count_gyro = 1000){
		filter_gyro_rate(); //run filter on gyro_rate-->does not work!! The integrated angle always attenuates to zero
		count_gyro = 0;
	} */
	int_ang_rate();

	mb_io_set_float(ID_EST_GYRO_ANGLE, ID_ang_rate.current_angle);
	return;
}


/* Initializes all butterworth filters used. */
void filter_init(void){
	// Sets the cut_off ratio = cut_off_freq/(sampling_freq/2) where sampling_freq = 1kHz 
	// For angular rates, cut_off_freq = 15Hz; a higher cutoff_freq(50Hz) generated lots of noise when the the hip scissor tracking function is ran
	////TODO - make these top level parameters in labview, with defaults, group into a few speeds.
	float cutoff_freq_hip_rate = 0.03; 
	float cutoff_freq_gyro = 0.03; 
	float cutoff_freq_foot = 0.0005;
	float cutoff_freq_ankle = 0.1;
	float cutoff_freq_hip_ang = 0.1;

	// Initialize filter coefficients for each of the filters   
	setFilterCoeff(&FC_hip, cutoff_freq_hip_rate);
	setFilterCoeff(&FC_gyro, cutoff_freq_gyro);
	setFilterCoeff(&FC_foot, cutoff_freq_foot);
	setFilterCoeff(&FC_ankle, cutoff_freq_ankle);
	setFilterCoeff(&FC_hip_ang, cutoff_freq_hip_ang);

	// Initialize filter data for each of the filters 
	setFilterData(&FD_hip_ang, 0.0);
	setFilterData(&FD_hip_rate, 0.0);
	setFilterData(&FD_hip_motor_rate, 0.0);
	setFilterData(&FD_gyro_rate, 0.0);
	setFilterData(&FD_in_l, 0.0); 
	setFilterData(&FD_in_r, 0.0); 
	setFilterData(&FD_out_l, 0.0); 
	setFilterData(&FD_out_r, 0.0); 
	setFilterData(&FD_FI_angle, 0.0);
	setFilterData(&FD_FI_ang_rate, 0.0);
	setFilterData(&FD_FO_angle, 0.0);
	setFilterData(&FD_FO_ang_rate, 0.0); 

	return;
}


/* Runs a 2nd order butterworth filter on the hip angular rate. 
 * Sets the estimated hip angular rate. 
 */
void filter_hip_rate(void){	
	float read_data;
	unsigned long read_data_t;
	float est_hip_rate;
	
	// Run the filter:
	read_data = mb_io_get_float(ID_MCH_ANG_RATE);
	read_data_t = mb_io_get_time(ID_MCH_ANG_RATE);
	est_hip_rate = runFilter_new(&FC_hip, &FD_hip_rate, read_data, read_data_t);
	
	mb_io_set_float(ID_E_MCH_ANG_RATE, est_hip_rate);	

	return;																																																																   
}

/* Runs a 2nd order butterworth filter on the hip motor velocity. */	
void filter_hip_motor_rate(void){																																																												  																																																		
	float read_data;
	unsigned long read_data_t;
	float est_hip_motor_rate;
	
	// Run the filter:
	read_data = mb_io_get_float(ID_MCH_MOTOR_VELOCITY);
	read_data_t = mb_io_get_time(ID_MCH_MOTOR_VELOCITY);
	est_hip_motor_rate = runFilter_new(&FC_hip, &FD_hip_motor_rate, read_data, read_data_t);
	
	mb_io_set_float(ID_E_MCH_MOTOR_VELOCITY, est_hip_motor_rate);

	return;
}

/* Runs a 2nd order butterworth filter on inner&outer feet angles & angular rates. */	
void filter_foot_data(void){
	float read_data_FI_a, read_data_FI_r, read_data_FO_a, read_data_FO_r;
	unsigned long read_data_t_FI_a, read_data_t_FI_r, read_data_t_FO_a, read_data_t_FO_r;
	float est_FI_a, est_FI_r, est_FO_a, est_FO_r;

	read_data_FI_a = mb_io_get_float(ID_MCFI_MID_ANKLE_ANGLE);
	read_data_t_FI_a = mb_io_get_time(ID_MCFI_MID_ANKLE_ANGLE);
	read_data_FI_r = mb_io_get_float(ID_MCFI_ANKLE_RATE);
	read_data_t_FI_r = mb_io_get_time(ID_MCFI_ANKLE_RATE);
	read_data_FO_a = mb_io_get_float(ID_MCFO_RIGHT_ANKLE_ANGLE);
	read_data_t_FO_a = mb_io_get_time(ID_MCFO_RIGHT_ANKLE_ANGLE);
	read_data_FO_r = mb_io_get_float(ID_MCFO_RIGHT_ANKLE_RATE);
	read_data_t_FO_r = mb_io_get_time(ID_MCFO_RIGHT_ANKLE_RATE);

	est_FI_a = runFilter_new(&FC_ankle, &FD_FI_angle, read_data_FI_a, read_data_t_FI_a);
	est_FI_r = runFilter_new(&FC_ankle, &FD_FI_ang_rate, read_data_FI_r, read_data_t_FI_r);
	est_FO_a = runFilter_new(&FC_ankle, &FD_FO_angle, read_data_FO_a, read_data_t_FO_a);
	est_FO_r = runFilter_new(&FC_ankle, &FD_FO_ang_rate, read_data_FO_r, read_data_t_FO_r);

	mb_io_set_float(ID_E_MCFI_MID_ANKLE_ANGLE, est_FI_a);
	mb_io_set_float(ID_E_MCFI_ANKLE_RATE, est_FI_r);
	mb_io_set_float(ID_E_MCFO_RIGHT_ANKLE_ANGLE, est_FO_a);
	mb_io_set_float(ID_E_MCFO_RIGHT_ANKLE_RATE, est_FO_r);
	return;
}

/* Returns 1 if inner feet are on the ground (higher than threshold values for 10 continuous clock cycles) */
int FI_on_ground(void){
	if(in_feet_count > ground_duration){  
		set_UI_LED(1, 'g'); 
		return 1;
	}
	return 0;
}

/* Returns 1 if outer feet are on the ground (higher than threshold values for 10 continuous clock cycles) */
int FO_on_ground(void){
	if(out_feet_count > ground_duration){
		set_UI_LED(2, 'g');	  ////TODO - make nice table of LED stuff
		return 1;
	}
	return 0;
}

/* Runs a 2nd order butterworth filter on all 4 foot sensors.
 * First filters the two inner foot sensors, then increments the counter  
 * if the combined filtered sensor readings is above the threshold value.
 * Decides that inner foot are on ground if the combined reading exceeds 
 * the threshold values for 10 continuous clock cycles.  
 * Do the same things for the two outer foot sensors. 
 */
void filter_foot_sensor(void){																																																												  																																																	
	float read_data_in_l, read_data_in_r, read_data_out_l, read_data_out_r;
	unsigned long read_data_in_l_t, read_data_in_r_t, read_data_out_l_t, read_data_out_r_t;
	float est_in_l, est_in_r, est_out_l, est_out_r;
	
	// Run the filter:
	read_data_in_l = mb_io_get_float(ID_MCFI_LEFT_HEEL_SENSE);
	read_data_in_l_t = mb_io_get_time(ID_MCFI_LEFT_HEEL_SENSE);
	est_in_l = runFilter_new(&FC_foot, &FD_in_l, read_data_in_l, read_data_in_l_t);	
	mb_io_set_float(ID_E_MCFI_LEFT_HEEL_SENSE, est_in_l);

	read_data_in_r = mb_io_get_float(ID_MCFI_RIGHT_HEEL_SENSE);
	read_data_in_r_t = mb_io_get_time(ID_MCFI_RIGHT_HEEL_SENSE);
	est_in_r = runFilter_new(&FC_foot, &FD_in_r, read_data_in_r, read_data_in_r_t);	
	mb_io_set_float(ID_E_MCFI_RIGHT_HEEL_SENSE, est_in_r);

	// Increment the counter if inner feet's combined reading is greater than threshold value
	if( (read_data_in_l + read_data_in_r) >  CONTACT_THRESHOLD){
		in_feet_count ++; 
	}else{
		in_feet_count = 0;
	}
		
	read_data_out_l = mb_io_get_float(ID_MCFO_LEFT_HEEL_SENSE);
	read_data_out_l_t = mb_io_get_time(ID_MCFO_LEFT_HEEL_SENSE);
	est_out_l = runFilter_new(&FC_foot, &FD_out_l, read_data_out_l, read_data_out_l_t);	
	mb_io_set_float(ID_E_MCFO_LEFT_HEEL_SENSE, est_out_l);

	read_data_out_r = mb_io_get_float(ID_MCFO_RIGHT_HEEL_SENSE);
	read_data_out_r_t = mb_io_get_time(ID_MCFO_RIGHT_HEEL_SENSE);
	est_out_r = runFilter_new(&FC_foot, &FD_out_r, read_data_out_r, read_data_out_r_t);	
	mb_io_set_float(ID_E_MCFO_RIGHT_HEEL_SENSE, est_out_r);	
	
	// Increment the counter if outer feet's combined reading is greater than threshold value
	if( (read_data_out_l + read_data_out_r) >  CONTACT_THRESHOLD){
		out_feet_count ++; 
	}else{
		out_feet_count = 0;
	}

	return;
}

/* Runs a 2nd order butterworth filter on the gyro rate */	
void filter_gyro_rate(void){																																																												  																																																		
	float read_data;
	unsigned long read_data_t;
	float est_gyro_rate;
	
	//Run the filter (used in hip_scissor tracking)
	read_data = mb_io_get_float(ID_UI_ANG_RATE_X);
	read_data_t = mb_io_get_time(ID_UI_ANG_RATE_X);
	est_gyro_rate = runFilter_new(&FC_gyro, &FD_gyro_rate, read_data, read_data_t);
	mb_io_set_float(ID_E_UI_ANG_RATE_X, est_gyro_rate);

	return;
}


/* Computes the coefficients for a second-order low-pass butterworth filter
 * @param r = ratio of cut-off frequncy to half of the sample frequency.
 * valid domain:  0.0001 < r < 0.9999   (coerced if out of bounds)
 */
void setFilterCoeff(struct FilterCoeff * FC, float r) {
	static float c;
	
	if (r < 0.0001) r = 0.0001;  // Prevents a divide by zero
	if (r > 0.9999) r = 0.9999;  // Cannot exceed Nyquist frequency

	c = Tan(0.5 * PI * (1.0 - r));

	FC->b0 = 1.0 / (1.0 + SQRT_TWO * c + c * c);
	FC->b1 = 2.0 * (FC->b0);
	FC->b2 = (FC->b0);

	FC->a1 = -2.0 * (c * c - 1.0) * (FC->b0);
	FC->a2 = (1.0 - SQRT_TWO * c + c * c) * (FC->b0);

}


/* Initializes the filter to a single value. */
void setFilterData(struct FilterData * FD, float z) {
	FD->z0 = z;
	FD->z1 = z;
	FD->z2 = z;
	FD->y0 = z;
	FD->y1 = z;
	FD->y2 = z;
	FD->t0 = mb_io_get_float(ID_TIMESTAMP);
	FD->d0 = z;
}

/* Updates the Butterworth filter based on new sensor data
 * @param z = the measurement at the current time step
 * @param t = current time step in ms
 * @return y = the filtered data at the current time step
 */
float runFilter_new(struct FilterCoeff * FC, struct FilterData * FD, float z, unsigned long t) {	
	//if the time elapsed exceeds 1ms, calculate the estimants for missed periods
	//t = current time stamp
	//FD->to = previous time stamp
	unsigned long t_diff = t - (FD->t0);
	int count;
	for(count = 0; count <= 4; count++){
		if(t_diff > 1){
			// Update sensor history:
			FD->z2 = FD->z1;
			FD->z1 = FD->z0;
			//FD->z0;	 //keep the previous measured value
			// Update estimate history:
			FD->y2 = FD->y1;
			FD->y1 = FD->y0;
			// Update time history;
			FD->t0 = FD->t0 + 1; //increment the time
			// Compute new estimate:
			FD->y0 =
			    (FC->b0) * (FD->z0) +
			    (FC->b1) * (FD->z1) +
			    (FC->b2) * (FD->z2) -
			    (FC->a1) * (FD->y1) -
			    (FC->a2) * (FD->y2);	
			t_diff = t - (FD->t0); //t_diff - 1
		} 
	}
	
	if(t_diff > 1){
	////TODO - do something sensible here (like reset the filter)
		// Error if the time difference b/t current and previous time stamp was greater than 4 ms 
	}

	////TODO - Make into one subfunction called twice?

	// Time difference now is 1ms -> update everything once
	// Update sensor history:
	FD->z2 = FD->z1;
	FD->z1 = FD->z0;
	FD->z0 = z;
	// Update estimate history:
	FD->y2 = FD->y1;
	FD->y1 = FD->y0;
	// Update time history;
	FD->t0 = t;
	// Compute new estimate:
	FD->y0 =
	    (FC->b0) * (FD->z0) +
	    (FC->b1) * (FD->z1) +
	    (FC->b2) * (FD->z2) -
	    (FC->a1) * (FD->y1) -
	    (FC->a2) * (FD->y2);

	// Compute & update the derivative at current time stamp
	// First derivative, second-order one-sided method
	// unit = rad/s
	FD->d0 = (FD->y2 - 4*FD->y1 + 3*FD->y0)/2 * 1000;

	return (FD->y0);
}


/* Initializes the integrator for integrating the gyro rate. */
void int_init_ang_rate(void){
	ID_ang_rate.currently_read_data = 0;
	ID_ang_rate.time_of_curr_read_data = mb_io_get_float(ID_TIMESTAMP);
	ID_ang_rate.prev_read_data = 0;
	ID_ang_rate.time_of_prev_read_data = 0;
	ID_ang_rate.current_angle = 0;
	ID_ang_rate.prev_angle = 0;
}

/* Runs the integrator for integrating the gyro rate. */
void int_ang_rate(void){
    ID_ang_rate.prev_read_data = ID_ang_rate.currently_read_data;
	ID_ang_rate.time_of_prev_read_data = ID_ang_rate.time_of_curr_read_data; 

	////TODO - Make permanent paramter in labview for gyro bias.	
	ID_ang_rate.currently_read_data = -(mb_io_get_float(ID_UI_ANG_RATE_X)- GYRO_RATE_BIAS); // negative sign because of sign convention difference 
	ID_ang_rate.time_of_curr_read_data = mb_io_get_time(ID_UI_ANG_RATE_X);
	
	ID_ang_rate.prev_angle = ID_ang_rate.current_angle; 
	ID_ang_rate.current_angle = ID_ang_rate.current_angle + ( ID_ang_rate.prev_read_data + ID_ang_rate.currently_read_data)*(ID_ang_rate.time_of_curr_read_data - ID_ang_rate.time_of_prev_read_data)/2000; //divide 1000 to convert time from ms to s 
}


/* Calibration: sets the angle integrated over gyro rate to zero */
void calibrate(void){
	int_init_ang_rate();
}

/* Sets the current gyro angle in the integration struct. */
void set_gyro_angle(float a){
	ID_ang_rate.current_angle = a;	
}

/* Returns the gyro angle calucated from the previous ms. */
float get_prev_gyro_angle(void){
	return ID_ang_rate.prev_angle;
}

/* Returns "qr" the angle integrated over gyro rate.
 * This is the absolute angle (of the outer leg) wrt ground; pos when outer leg is in forward
 * (The static variable in mb_estimator.h cannot be accessed by other c-files.) 
 */
float get_out_angle(void){
	return ID_ang_rate.current_angle;
}

/* Returns "dqr" the gyro rate (filtered) for the outer leg. */
float get_out_ang_rate(void){
	return -(mb_io_get_float(ID_E_UI_ANG_RATE_X)- GYRO_RATE_BIAS);
}

/* gets "qh" the hip angle (angle b/t inner&outer legs; pos when inner leg is in front) */
float get_in_angle(void){
 	return mb_io_get_float(ID_MCH_ANGLE);  
}

/* Returns "dqh" angular rate of the hip angle (filtered). */
float get_in_ang_rate(void){
	return mb_io_get_float(ID_E_MCH_ANG_RATE); //the estimated hip angle rate
}
