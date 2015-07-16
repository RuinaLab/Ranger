#include <mb_includes.h>
#include <mb_estimator.h>
#include <RangerMath.h>   // tan()


#define SQRT_TWO 1.414213562373095
#define PI 3.141592653589793
#define DEFAULT_GYRO_BIAS -0.0097 //estimated bias for the rate gyro - based on experiment on 7/15/2015
//default value in the can_table.cvs only works when the parameter is read from labview to the robot (0 to 1)	

/* This will be CALLED by the main brain and it will do all the estimations */
void mb_estimator_update(void){
	if(!init){
		filter_init();
		intergrater_init_ang_rate();
		init = 1;
	}
	
	//run filter on hip_rate 	
 	filter_hip_rate();
	//run filter on hip_motor_rate
	filter_hip_motor_rate();
	//run filter on gyro_rate
	//filter_gyro_rate(); //does not work!! The integrated angle always attenuates to zero

	integrate_ang_rate();
	return;
}

void filter_init(void){
	// Initialize filter coefficients once for the use of all butterworth filters 
	// 0.005 is a super low cutoff_freq for estimating the gyro_rate
	float cutoff_frequency = 0.1;//0.0005;
	  
	setFilterCoeff(&FC, cutoff_frequency);

	// Initialize filter data for each of the filters 
	setFilterData(&FD_hip_rate, 0.0);
	setFilterData(&FD_hip_motor_rate, 0.0);
	setFilterData(&FD_gyro_rate, DEFAULT_GYRO_BIAS);

	return;
}

/* Runs a 2nd order butterworth filter on the hip angle rate */
void filter_hip_rate(void){	
	float read_data;
	unsigned long read_data_t;
	float est_hip_rate;
	
	// Run the filter:
	read_data = mb_io_get_float(ID_MCH_ANG_RATE);
	read_data_t = mb_io_get_time(ID_MCH_ANG_RATE);
	est_hip_rate = runFilter_new(&FC, &FD_hip_rate, read_data, read_data_t);
	
	mb_io_set_float(ID_E_MCH_ANG_RATE, est_hip_rate);	

	return;																																																																   
}

/* Runs a 2nd order butterworth filter on the hip motor velocity */	
void filter_hip_motor_rate(void){																																																												  																																																		
	float read_data;
	unsigned long read_data_t;
	float est_hip_motor_rate;
	
	// Run the filter:
	read_data = mb_io_get_float(ID_MCH_MOTOR_VELOCITY);
	read_data_t = mb_io_get_time(ID_MCH_MOTOR_VELOCITY);
	est_hip_motor_rate = runFilter_new(&FC, &FD_hip_motor_rate, read_data, read_data_t);
	
	mb_io_set_float(ID_E_MCH_MOTOR_VELOCITY, est_hip_motor_rate);

	return;
}

/* Runs a 2nd order butterworth filter on the gyro rate */	
void filter_gyro_rate(void){																																																												  																																																		
	float read_data;
	unsigned long read_data_t;
	float est_gyro_rate;
	
	// Run the filter:
	read_data = mb_io_get_float(ID_UI_ANG_RATE_X/*- mb_io_get_float(ID_EST_GYRO_RATE_BIAS)*/);
	read_data_t = mb_io_get_time(ID_UI_ANG_RATE_X);
	est_gyro_rate = runFilter_new(&FC, &FD_gyro_rate, read_data, read_data_t);
	
	mb_io_set_float(ID_EST_GYRO_RATE_BIAS, est_gyro_rate);
	mb_io_set_float(ID_EST_TEST_W0, est_gyro_rate);
	return;
}


/* Computes the coefficients for a second-order low-pass butterworth filter
 * @param r = ratio of cut-off frequncy to half of the sample frequency.
 * valid domain:  0.01 < r < 0.99   (coerced if out of bounds)
 */
void setFilterCoeff(struct FilterCoeff * FC, float r) {
	static float q = SQRT_TWO;
	static float c;
	
	if (r < 0.0001) r = 0.0001;  // Prevents a divide by zero
	if (r > 0.9999) r = 0.9999;  // Cannot exceed Nyquist frequency

	c = Tan(0.5 * PI * (1.0 - r));

	FC->b0 = 1.0 / (1.0 + q * c + c * c);
	FC->b1 = 2.0 * (FC->b0);
	FC->b2 = (FC->b0);

	FC->a1 = -2.0 * (c * c - 1.0) * (FC->b0);
	FC->a2 = (1.0 - q * c + c * c) * (FC->b0);

}


/* Initializes the filter to a single value
 */
void setFilterData(struct FilterData * FD, float z) {
	FD->z0 = z;
	FD->z1 = z;
	FD->z2 = z;
	FD->y0 = z;
	FD->y1 = z;
	FD->y2 = z;
	FD->t0 = mb_io_get_float(ID_TIMESTAMP);
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
		// Error if the time difference b/t current and previous time stamp was greater than 4 ms 
	}

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

	return (FD->y0);
}

// for imu
//static float gyro_bias = 0;//-0.0092; // -.004 (old value feb 18 2013)
                               // new imu  //earlier imu 0.0165 ; //  // this is the average imu rate read from can id when the imu is supposed to be at rest.
                               // hence this value will be subtracted from the can-id to correct for thsi offset. 
                               // THIS NUMBER SHOUKD BE REGULARLY CHECKED AND UPDATED. it changes with time.
void intergrater_init_ang_rate(void){
	ID_ang_rate.currently_read_data = 0;
	ID_ang_rate.time_of_curr_read_data = 0;
	ID_ang_rate.prev_read_data = 0;
	ID_ang_rate.time_of_prev_read_data = 0;
	ID_ang_rate.current_angle = 0;
}

void integrate_ang_rate(void){
    ID_ang_rate.prev_read_data = ID_ang_rate.currently_read_data;
	ID_ang_rate.time_of_prev_read_data = ID_ang_rate.time_of_curr_read_data; 
	
	ID_ang_rate.currently_read_data = -(mb_io_get_float(ID_UI_ANG_RATE_X)- DEFAULT_GYRO_BIAS/*mb_io_get_float(ID_EST_GYRO_RATE_BIAS)*/); // negative sign because of sign convention difference 
	ID_ang_rate.time_of_curr_read_data = mb_io_get_time(ID_UI_ANG_RATE_X);
	
	ID_ang_rate.current_angle = ID_ang_rate.current_angle + ( ID_ang_rate.prev_read_data + ID_ang_rate.currently_read_data)*(ID_ang_rate.time_of_curr_read_data - ID_ang_rate.time_of_prev_read_data)/2000; //divide 1000 to convert time from ms to s 
	mb_io_set_float(ID_EST_TEST_W1, ID_ang_rate.current_angle);
/*	mb_io_set_float(ID_EST_TEST_W1, ID_ang_rate.prev_read_data);
	mb_io_set_float(ID_EST_TEST_W2, ID_ang_rate.time_of_prev_read_data);
	mb_io_set_float(ID_EST_TEST_W3, ID_ang_rate.currently_read_data);	
	mb_io_set_float(ID_EST_TEST_W4, ID_ang_rate.time_of_curr_read_data);	
	*/
}