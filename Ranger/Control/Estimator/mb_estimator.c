#include <mb_includes.h>
#include <mb_estimator.h>
#include <RangerMath.h>   // Tan(), bool
#include <input_output.h> // LED functions

typedef struct {
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
} FilterCoeff;

typedef struct {
	float z0; // Measurement at time k
	float z1; // Measurement at time k-1
	float z2; // Measurement at time k-2
	float y0; // Estimate at time k
	float y1; // Estimate at time k-1
	float y2; // Estimate at time k-2
	unsigned long t0; // time k
} FilterData;

typedef struct {
	float zLast; // Last measurement
	unsigned long tLast; // Last time stamp
	float sum; // Integral
} IntegralData;

/* Global variables */
extern bool INITIALIZE_ESTIMATOR = true;   // Should the estimator be initialized?

/* Filter cut-off frequencies */
static const float FILTER_CUTOFF_FAST = 0.002 * 50; // (2*period in sec)*(cutoff frequency in Hz)
static const float FILTER_CUTOFF_SLOW = 0.002 * 10; // (2*period in sec)*(cutoff frequency in Hz)

/* Local constant parameters */
static const float GYRO_RATE_BIAS = -0.0097;  // Measured in August 2015. Should be checked monthly.
static const float CONTACT_VALUE_THRESHOLD = 7000.0;  // Threshold for detecting contact on the feet

/* Butterworth filter coefficients */
static FilterCoeff FC_FAST;
static FilterCoeff FC_SLOW;

/* Butterworth filters */
static FilterData FD_UI_ANG_RATE_X;
static FilterData FD_MCFO_LEFT_HEEL_SENSE;
static FilterData FD_MCFO_RIGHT_HEEL_SENSE;
static FilterData FD_MCFI_LEFT_HEEL_SENSE;
static FilterData FD_MCFI_RIGHT_HEEL_SENSE;

/* Estimate of the outer leg angle */
static IntegralData intDat_OUTER_LEG_ANGLE;


/* Initializes the filter to a single value. */
void setFilterData(FilterData * FD, float z) {
	FD->z0 = z;
	FD->z1 = z;
	FD->z2 = z;
	FD->y0 = z;
	FD->y1 = z;
	FD->y2 = z;
	FD->t0 = mb_io_get_float(ID_TIMESTAMP);
}

/* Updates the Butterworth filter based on new sensor data. The filter runs
 * with a period of 1ms, so that it can handle asynchronous data.
 * @param z = the measurement at the current time step
 * @param t = current time step in ms
 * @return y = the filtered data at the current time step
 */
float runFilter(FilterCoeff * FC, FilterData * FD, unsigned long t, float z) {

	unsigned long tDiff = t - (FD->t0);

	// Error checking on timing:
 	if (tDiff > 20) { // Went a long time without calling the filter
		mb_error_occurred(ERROR_EST_FILTER_TIME_VIOLATION);
		setFilterData(FD, z);  // Reset the filter here
		return z;
	} else if (tDiff != 0) { // Recursively call the filter
		return runFilter(FC, FD, t - 1, z);
	} else {	// BASE CASE:  Expect that t == (FD->t0)

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
	}

	return (FD->y0);
}




/* Runs the butterworth filter on the gyro rate */
void runFilter_ui_ang_rate_x(void) {
	unsigned long t = mb_io_get_time(ID_UI_ANG_RATE_X);
	float z = mb_io_get_float(ID_UI_ANG_RATE_X) - GYRO_RATE_BIAS;
	float y = runFilter(&FC_FAST, &FD_UI_ANG_RATE_X, t, z);
	mb_io_set_float(ID_E_UI_ANG_RATE_X, y);
}

/* Runs the butterworth filter on the outer foot contact sensors */
void runFilter_contactOuter(void) {
	unsigned long tr = mb_io_get_time(ID_MCFO_RIGHT_HEEL_SENSE);
	unsigned long tl = mb_io_get_time(ID_MCFO_LEFT_HEEL_SENSE);
	float zr = mb_io_get_float(ID_MCFO_RIGHT_HEEL_SENSE);
	float zl = mb_io_get_float(ID_MCFO_LEFT_HEEL_SENSE);
	float yr = runFilter(&FC_SLOW, &FD_MCFO_RIGHT_HEEL_SENSE, tr, zr);
	float yl = runFilter(&FC_SLOW, &FD_MCFO_LEFT_HEEL_SENSE, tl, zl);
	mb_io_set_float(ID_EST_CONTACT_OUTER, yr + yl);
}

/* Runs the butterworth filter on the inner foot contact sensors */
void runFilter_contactInner(void) {
	unsigned long tr = mb_io_get_time(ID_MCFI_RIGHT_HEEL_SENSE);
	unsigned long tl = mb_io_get_time(ID_MCFI_LEFT_HEEL_SENSE);
	float zr = mb_io_get_float(ID_MCFI_RIGHT_HEEL_SENSE);
	float zl = mb_io_get_float(ID_MCFI_LEFT_HEEL_SENSE);
	float yr = runFilter(&FC_SLOW, &FD_MCFI_RIGHT_HEEL_SENSE, tr, zr);
	float yl = runFilter(&FC_SLOW, &FD_MCFI_LEFT_HEEL_SENSE, tl, zl);
	mb_io_set_float(ID_EST_CONTACT_INNER, yr + yl);
}


/* Computes the coefficients for a second-order low-pass butterworth filter
 * @param r = ratio of cut-off frequncy to half of the sample frequency.
 * valid domain:  0.0001 < r < 0.9999   (coerced if out of bounds)
 */
void setFilterCoeff(FilterCoeff * FC, float r) {
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


/* Returns the derivative of a filtered signal */
float getFilterDerivative(FilterData * FD) {
	float derivative = 0.5 * (FD->y2 - 4 * FD->y1 + 3 * FD->y0); // Second-order backwards difference
	return  1000 * derivative; // Convert to 1/seconds rather than 1/ms
}



/* Returns the integral of the sensor data by trapazoid method */
float computeIntegral(IntegralData * intDat, unsigned long t, float z) {
	float dt = 0.001 * (t - intDat->tLast); // Time in seconds between data points
	float area = 0.5 * dt * (z + intDat->zLast);
	intDat->sum = intDat->sum + area;
	return intDat->sum;
}


/* Computes the integral of the outer leg angular rate */
void runIntegral_outerLegAngle() {
	unsigned long t = mb_io_get_time(ID_UI_ANG_RATE_X);
	float z = mb_io_get_float(ID_UI_ANG_RATE_X);
	float sum = computeIntegral(&intDat_OUTER_LEG_ANGLE, t, z);
	mb_io_set_float(ID_EST_OUTER_LEG_ANGLE, sum);
}

/* Resets the sum in the integral.
 * @param z = current value of sensor
 * @param sum = desired value of the integral */
void resetIntegral(IntegralData * intDat, float z, float sum) {
	intDat->zLast = z;
	intDat->tLast = mb_io_get_float(ID_TIMESTAMP);
	intDat->sum = sum;
}


/********************* Public Functions ***********************************
 *                                                                        *
 **************************************************************************/


/* Returns true if the outer feet are in contact with the ground */
bool getContactOuter(void) {
	float sum = FD_MCFO_RIGHT_HEEL_SENSE.y0 + FD_MCFO_LEFT_HEEL_SENSE.y0;
	return sum > CONTACT_VALUE_THRESHOLD;
}
/* Returns true if the inner feet are in contact with the ground */
bool getContactInner(void) {
	float sum = FD_MCFI_RIGHT_HEEL_SENSE.y0 + FD_MCFI_LEFT_HEEL_SENSE.y0;
	return sum > CONTACT_VALUE_THRESHOLD;
}

/* Sets the outer leg angle estimate to zero */
void resetOuterLegAngle(void){
	resetIntegral(&intDat_OUTER_LEG_ANGLE, 0.0, 0.0);
}




/***************** ENTRY-POINT FUNCTION CALL  *****************************
 *                                                                        *
 **************************************************************************/
void mb_estimator_update(void) {
	clear_UI_LED();  // Clears all LEDs that had been active on the previous cycle

	if (INITIALIZE_ESTIMATOR) {
		// Initialize the filter coefficients:
		setFilterCoeff(&FC_FAST, FILTER_CUTOFF_FAST);
		setFilterCoeff(&FC_SLOW, FILTER_CUTOFF_SLOW);

		// Reset the filters
		setFilterData(&FD_UI_ANG_RATE_X, 0.0);
		setFilterData(&FD_MCFO_LEFT_HEEL_SENSE, 0.0);
		setFilterData(&FD_MCFO_RIGHT_HEEL_SENSE, 0.0);
		setFilterData(&FD_MCFI_LEFT_HEEL_SENSE, 0.0);
		setFilterData(&FD_MCFI_RIGHT_HEEL_SENSE, 0.0);

		// Reset the estimate of the outer leg angle
		resetIntegral(&intDat_OUTER_LEG_ANGLE, 0.0, 0.0);

		// Remember that we've initialized everything properly
		INITIALIZE_ESTIMATOR = false;
	}

	// Run all of the butterworth filters:
	runFilter_ui_ang_rate_x();
	runFilter_contactOuter();
	runFilter_contactInner();

	// Run calculations:
	runIntegral_outerLegAngle();

	return;
}
