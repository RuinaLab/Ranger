#include <mb_includes.h>
#include <mb_estimator.h>
#include <RangerMath.h>   // Tan(), bool
#include <input_output.h> // LED functions
#include <robotParameters.h>

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
bool INITIALIZE_ESTIMATOR = true;   // Should the estimator be initialized?

/* Filter cut-off frequencies */
static const float FILTER_CUTOFF_FAST = 0.002 * 30; // (2*period in sec)*(cutoff frequency in Hz) -- Used by joint sensors
static const float FILTER_CUTOFF_SLOW = 0.002 * 10; // (2*period in sec)*(cutoff frequency in Hz) -- Used by foot contact sensors
static const float FILTER_OUTER_LEG_ANGLE_UPDATE = 0.9; // 1.0->Ignore heel-strike reset, 0.0->update purely based on reset

/* Local constant parameters */
static const float GYRO_RATE_BIAS = -0.009324229372422;  // Measured August 29, 2015. Should be checked monthly.
static const float CONTACT_VALUE_THRESHOLD = 7000.0;  // Threshold for detecting contact on the feet

/* Butterworth filter coefficients */
static FilterCoeff FC_FAST;  // For joint sensors
static FilterCoeff FC_SLOW;  // For contact sensors

/* Butterworth filters on joint sensors */
static FilterData FD_UI_ANG_RATE_X;  // stance leg angular rate
static FilterData FD_MCH_ANG_RATE;  // hip angle rate
static FilterData FD_MCFO_RIGHT_ANKLE_RATE ;  // outer ankle rate
static FilterData FD_MCFI_ANKLE_RATE;  // inner ankle rate

/* Butterworth filters on contact sensors */
static FilterData FD_MCFO_LEFT_HEEL_SENSE;
static FilterData FD_MCFO_RIGHT_HEEL_SENSE;
static FilterData FD_MCFI_LEFT_HEEL_SENSE;
static FilterData FD_MCFI_RIGHT_HEEL_SENSE;

/* Estimate of the outer leg angle */
static IntegralData intDat_OUTER_LEG_ANGLE;

/* Parameters from Labview */
bool LABVIEW_HIP_GRAVITY_COMPENSATION;
bool LABVIEW_HIP_SPRING_COMPENSATION;

/* Robot state variables. Naming conventions in docs. Matches simulator. */
float STATE_qh;  // hip angle
float STATE_q0;  // outer ankle angle
float STATE_q1;  // inner ankle angle
float STATE_dqh;  // hip rate
float STATE_dq0;  // outer ankle rate
float STATE_dq1;  // inner ankle rate
float STATE_th0;  // absolute orientation of outer legs
float STATE_th1;  // absolute orientation of inner legs
float STATE_phi0;  // absolute orientation of outer feet
float STATE_phi1;  // absolute orientation of inner feet
float STATE_dth0;  // absolute orientation rate of outer legs
float STATE_dth1;  // absolute orientation rate of inner legs
float STATE_dphi0;  // absolute orientation rate of outer feet
float STATE_dphi1;  // absolute orientation rate of inner feet

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

	int tDiff = (int) (t - (FD->t0));

	// Error checking on timing:
	if (tDiff > 30 || tDiff < 0) { // Something went wrong
		mb_error_occurred(ERROR_EST_FILTER_TIME_VIOLATION);
		setFilterData(FD, z);  // Reset the filter
		return z;
	} else if (tDiff == 0) { // No new data - return old estimate
		return FD->y0;
	}

	// March forward in time until we reach the present:
	do {
		// Update sensor history:
		FD->z2 = FD->z1;
		FD->z1 = FD->z0;
		FD->z0 = z;
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
	} while (FD->t0 < t);
	return (FD->y0);
}




/* Runs the butterworth filter on the gyro rate */
void runFilter_ui_ang_rate_x(void) {
	unsigned long t = mb_io_get_time(ID_UI_ANG_RATE_X);
	float z = mb_io_get_float(ID_UI_ANG_RATE_X) - GYRO_RATE_BIAS;
	float y = runFilter(&FC_FAST, &FD_UI_ANG_RATE_X, t, z);
	mb_io_set_float(ID_EST_STATE_DTH0, y);
	STATE_dth0 = y; // Send robot orientation rate to the control and estimation code
}

/* Runs the butterworth filter on the three joint sensors */
void runFilter_jointRates(void) {
	unsigned long t; // Store the time here
	float z;  // Store the sensor data
	float y;  // Store the estimate

	// Hip Angular Rate:
	t = mb_io_get_time(ID_MCH_ANG_RATE);
	z = mb_io_get_float(ID_MCH_ANG_RATE);
	y = runFilter(&FC_FAST, &FD_MCH_ANG_RATE, t, z);
	mb_io_set_float(ID_E_MCH_ANG_RATE, y);
	STATE_dqh = y;

	// Outer Ankle Rate:
	t = mb_io_get_time(ID_MCFO_RIGHT_ANKLE_RATE );
	z = mb_io_get_float(ID_MCFO_RIGHT_ANKLE_RATE );
	y = runFilter(&FC_FAST, &FD_MCFO_RIGHT_ANKLE_RATE, t, z);
	mb_io_set_float(ID_E_MCFO_RIGHT_ANKLE_RATE, y);
	STATE_dq0 = y;

	// Inner Ankle Rate:
	t = mb_io_get_time(ID_MCFI_ANKLE_RATE );
	z = mb_io_get_float(ID_MCFI_ANKLE_RATE );
	y = runFilter(&FC_FAST, &FD_MCFI_ANKLE_RATE, t, z);
	mb_io_set_float(ID_E_MCFI_ANKLE_RATE, y);
	STATE_dq1 = y;
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
	intDat->zLast = z;
	intDat->tLast = t;
	return intDat->sum;
}


/* Computes the integral of the outer leg angular rate */
void runIntegral_outerLegAngle() {
	unsigned long t = mb_io_get_time(ID_UI_ANG_RATE_X);
	float z = mb_io_get_float(ID_UI_ANG_RATE_X) - GYRO_RATE_BIAS;
	float sum = computeIntegral(&intDat_OUTER_LEG_ANGLE, t, z);
	mb_io_set_float(ID_EST_STATE_TH0, sum);  // Send over CAN
	STATE_th0 = sum;  // Send to control and estimator code
}

/* Resets the sum in the integral.
 * @param t = current time
 * @param z = current value of sensor
 * @param sum = desired value of the integral */
void resetIntegral(IntegralData * intDat, unsigned long t, float z, float sum) {
	intDat->zLast = z;
	intDat->tLast = t;
	intDat->sum = sum;
}

/* Updates the robot's state.
 * Joint angles are read directly from the sensors
 * Robot angle (th0) is updated by runIntegral_outerLegAngle
 * Joint rates (dth0, dqh, dq0, dq1) are computed by butterworth filters and updated there. */
void updateRobotState(void) {

	// Read the joint angles of the robot
	STATE_qh = mb_io_get_float(ID_MCH_ANGLE); // hip angle - between legs
	STATE_q0 = mb_io_get_float(ID_MCFO_RIGHT_ANKLE_ANGLE);  // outer ankle angle
	STATE_q1 = mb_io_get_float(ID_MCFI_MID_ANKLE_ANGLE);  // inner ankle angle

	// Compute the absolute orientation of the robot's feet and legs
	STATE_th1 = STATE_qh + STATE_th0; // absolute orientation of inner legs
	STATE_phi0 = PARAM_Phi + STATE_th0 - STATE_q0;  // absolute orientation of outer feet
	STATE_phi1 = PARAM_Phi  + STATE_qh + STATE_th0 - STATE_q1; // absolute orientation of inner feet
	STATE_dth1 = STATE_dqh + STATE_dth0;  // absolute orientation rate of inner legs
	STATE_dphi0 = STATE_dth0 - STATE_dq0;  // absolute orientation rate of outer feet
	STATE_dphi1 = STATE_dqh + STATE_dth0 - STATE_dq1; // absolute orientation rate of inner feet

	// Send back across network of data logging and diagnostics:
	mb_io_set_float(ID_EST_STATE_TH1, STATE_th1);
	mb_io_set_float(ID_EST_STATE_PHI0, STATE_phi0);
	mb_io_set_float(ID_EST_STATE_PHI1, STATE_phi1);
	mb_io_set_float(ID_EST_STATE_DTH1, STATE_dth1);
	mb_io_set_float(ID_EST_STATE_DPHI0, STATE_dphi0);
	mb_io_set_float(ID_EST_STATE_DPHI1, STATE_dphi1);
}


/* Updates any controller parameters that are set from LabVIEW */
void updateParameters(void){
	LABVIEW_HIP_GRAVITY_COMPENSATION = mb_io_get_float(ID_CTRL_HIP_GRAVITY_COMPENSATION) > 0.5;
	LABVIEW_HIP_SPRING_COMPENSATION = mb_io_get_float(ID_CTRL_HIP_SPRING_COMPENSATION) > 0.5;
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
void resetOuterLegAngle(float sum) {
	unsigned long t = mb_io_get_time(ID_UI_ANG_RATE_X);
	float z = mb_io_get_float(ID_UI_ANG_RATE_X) - GYRO_RATE_BIAS;
	resetIntegral(&intDat_OUTER_LEG_ANGLE, t, z, sum);
}

/* Updates the outer leg angle based on double stance contact. This should be called from
 * the walking controller each time a heel-strike occurs. */
void updateOuterLegAngle(void) {

	float Slope = 0.0;  // Assume flat ground for now
	float x, y; // scalar distances, in coordinate system aligned with outer legs
	float th0_geometry;  // Store estimate from geometry here
	float th0_update;  // new estimate goes here
	float alpha = 1.0; ////HACK//// FILTER_OUTER_LEG_ANGLE_UPDATE;

	float Phi = PARAM_Phi;
	float l = PARAM_l;
	float d = PARAM_d;
	float qh = STATE_qh;
	float q1 = STATE_q1;
	float q0 = STATE_q0;

	/* Ranger geometry:
	 * [x;y] = vector from outer foot virtual center to the inner foot
	 * virtual center, in a frame that is rotated such that qr = 0
	 * These functions were determined using computer math. The code can
	 * be found in:
	 * templates/Estimator/legAngleEstimator/Derive_Eqns.m
	 */
	x = l * Sin(qh) - d * Sin(Phi - q1 + qh) + d * Sin(Phi - q0);
	y = l + d * Cos(Phi - q1 + qh) - l * Cos(qh) - d * Cos(Phi - q0);

	th0_geometry = Atan(y / x) + Slope;	 //gyro angle calculated from geometry

	/* The new robot angle should be a convex combination of the estimate from
	 * the rate gyro integration and the geometry from double stance. This is basically
	 * a first-order filter, running once per step. The weight should be alpha = 0.9,
	 * but there is a bug in the reset now, so we are neglecting it. */
	th0_update = alpha * STATE_th0 + (1.0 - alpha) * th0_geometry;	//new gyro angle that's a weighted average of the two above

	resetOuterLegAngle(-th0_update);  // outer leg angle = qr = -th0
	mb_io_set_float(ID_EST_LAST_STEP_LENGTH, Sqrt(x * x + y * y)); // Distance between two contact points

	return;

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

		// Reset the joint angle rate filters
		setFilterData(&FD_UI_ANG_RATE_X, 0.0);
		setFilterData(&FD_MCH_ANG_RATE, 0.0);
		setFilterData(&FD_MCFO_RIGHT_ANKLE_RATE, 0.0);
		setFilterData(&FD_MCFI_ANKLE_RATE, 0.0);
		// Reset the contact sensor filters
		setFilterData(&FD_MCFO_LEFT_HEEL_SENSE, 0.0);
		setFilterData(&FD_MCFO_RIGHT_HEEL_SENSE, 0.0);
		setFilterData(&FD_MCFI_LEFT_HEEL_SENSE, 0.0);
		setFilterData(&FD_MCFI_RIGHT_HEEL_SENSE, 0.0);

		// Reset the estimate of the outer leg angle
		resetOuterLegAngle(0.0);

		// Remember that we've initialized everything properly
		INITIALIZE_ESTIMATOR = false;
	}

	// Run all of the butterworth filters:
	runFilter_ui_ang_rate_x();
	runFilter_jointRates();
	runFilter_contactOuter();
	runFilter_contactInner();

	// Run calculations:
	runIntegral_outerLegAngle();

	// Update the state variables:  (absolute orientation and rate)
	updateRobotState();

	// Update controller parameters from LabVIEW
	updateParameters();


	return;
}
