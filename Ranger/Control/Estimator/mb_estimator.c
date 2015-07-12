#include <mb_includes.h>
#include <mb_estimator.h>
#include <math.h>   // tan()

#define SQRT_TWO 1.414213562373095
#define PI 3.141592653589793

/* This will be CALLED by the main brain and it will do all the estimations */
void mb_estimator_update(void){
	if(!f_init){
		filter_init();
	}	
 	filter_hip_rate();
	filter_hip_motor_rate();
	return;
}

void filter_init(void){
	// Initialize filter coefficients once for the use of all butterworth filters 
	float cutoff_frequency = 0.01;  // min 0.001 < fc < 0.999 max
	setFilterCoeff(&FC, cutoff_frequency);

	// Initialize filter data for each of the filters 
	setFilterData(&FD_hip_rate, 0.0);
	setFilterData(&FD_hip_motor_rate, 0.0);

	f_init = 1;
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


/* Computes the coefficients for a second-order low-pass butterworth filter
 * @param r = ratio of cut-off frequncy to half of the sample frequency.
 * valid domain:  0.01 < r < 0.99   (coerced if out of bounds)
 */
void setFilterCoeff(struct FilterCoeff * FC, float r) {
	static float q = SQRT_TWO;
	static float c;
	
	if (r < 0.001) r = 0.001;  // Prevents a divide by zero
	if (r > 0.999) r = 0.999;  // Cannot exceed Nyquist frequency

	c = tan(0.5 * PI * (1.0 - r));

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
	FD->t0 = 0;
	FD->t1 = 0;
	FD->t2 = 0;
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
	int counter = 0;
	mb_io_set_float(ID_EST_TEST_W0, (float)t_diff);
	mb_io_set_float(ID_EST_TEST_W1, (float)t);
	mb_io_set_float(ID_EST_TEST_W2, (float)(FD->t0));

	while(t_diff>1){
		mb_io_set_float(ID_EST_TEST_W0, (float)t_diff);
		// Error if this while loop has been iterated more than 4 times		
		if(counter > 4){
		 	// raise error
		}

		// Update sensor history:
		FD->z2 = FD->z1;
		FD->z1 = FD->z0;
		//FD->z0;	 //keep the previous measured value
		// Update estimate history:
		FD->y2 = FD->y1;
		FD->y1 = FD->y0;
		// Update time history;
		FD->t2 = FD->t1;
		FD->t1 = FD->t0;
		FD->t0 = FD->t0 + 1; //increment the time
		// Compute new estimate:
		FD->y0 =
		    (FC->b0) * (FD->z0) +
		    (FC->b1) * (FD->z1) +
		    (FC->b2) * (FD->z2) -
		    (FC->a1) * (FD->y1) -
		    (FC->a2) * (FD->y2);	
		t_diff = t - (FD->t0); //t_diff - 1
		counter ++; 
	}

	//Else: time difference is less than 1ms -> base case for the recursion
	// Update sensor history:
	FD->z2 = FD->z1;
	FD->z1 = FD->z0;
	FD->z0 = z;
	// Update estimate history:
	FD->y2 = FD->y1;
	FD->y1 = FD->y0;
	// Update time history;
	FD->t2 = FD->t1;
	FD->t1 = FD->t0;
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
