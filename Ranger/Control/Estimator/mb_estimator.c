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

/* Global variables */
bool INITIALIZE_ESTIMATOR = true;   // Should the estimator be initialized?

/* Filter cut-off frequencies */
static const float FILTER_CUTOFF_FAST = 0.002 * 30; // (2*period in sec)*(cutoff frequency in Hz) -- Used by joint sensors
static const float FILTER_CUTOFF_SLOW = 0.002 * 10; // (2*period in sec)*(cutoff frequency in Hz) -- Used by foot contact sensors

/* Local constant parameters */
static const float GYRO_RATE_BIAS = -0.009324229372422;  // Measured August 29, 2015. Should be checked monthly.
static const float CONTACT_VALUE_THRESHOLD = 1500.0;  // Threshold for detecting contact on the feet

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

/* Butterworth filter on the absolute orientation of the robot */
static FilterData FD_OUTER_LEG_ANGLE; // absolute angle of the outer legs

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

/* Contact configuration */
bool STATE_c0;
bool STATE_c1;
ContactMode STATE_contactMode = CONTACT_FL;

/* Static variables for computing integral of rate gyro */
static unsigned long RATE_GYRO_LAST_TIME;
static float RATE_GYRO_LAST_RATE;

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
	STATE_c0 = (yr + yl) > CONTACT_VALUE_THRESHOLD;
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
	STATE_c1 = (yr + yl) > CONTACT_VALUE_THRESHOLD;
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


/* This function must be called when robot starts to initialize the
 * static variables used to compute the integral of the rate gyro */
void initGyroIntegral() {
	RATE_GYRO_LAST_TIME = mb_io_get_time(ID_UI_ANG_RATE_X);
	RATE_GYRO_LAST_RATE = mb_io_get_float(ID_UI_ANG_RATE_X) - GYRO_RATE_BIAS;
}

/* Returns the angle of the outer legs of the robot, by integrating the
 * gyro rate from */
float getOuterLegAngleGyro(float zLast) {
	unsigned long t;  // time now
	float dt;  // Time between data points
	float dz;  // rate now
	float zNew;  // new angle
	t = mb_io_get_time(ID_UI_ANG_RATE_X);
	dz = mb_io_get_float(ID_UI_ANG_RATE_X) - GYRO_RATE_BIAS;
	if (t == RATE_GYRO_LAST_TIME) { // No new data
		return zLast;
	} else { // new data:
		dt = (float) (t - RATE_GYRO_LAST_TIME);
		dt = 0.001 * dt; // convert to seconds;
		zNew = zLast + 0.5*dt*(dz + RATE_GYRO_LAST_RATE);
		RATE_GYRO_LAST_TIME = t;
		RATE_GYRO_LAST_RATE = dz;
		return zNew;
	}
}

/* Returns the angle of the outer legs of the robot, assuming that both feet
 * are on the ground, and that the ground is flat and level. */
float getOuterLegAngleGeometry(void) {

	float Slope = 0.0;  // Assume flat ground for now
	float x, y; // scalar distances, in coordinate system aligned with outer legs

	float Phi = PARAM_Phi;
	float l = PARAM_l;
	float d = PARAM_d;
	float qh, q0, q1; // robot joint angles
	qh = mb_io_get_float(ID_MCH_ANGLE); // hip angle - between legs
	q0 = mb_io_get_float(ID_MCFO_RIGHT_ANKLE_ANGLE);  // outer ankle angle
	q1 = mb_io_get_float(ID_MCFI_MID_ANKLE_ANGLE);  // inner ankle angle

	/* Ranger geometry:
	 * [x;y] = vector from outer foot virtual center to the inner foot
	 * virtual center, in a frame that is rotated such that qr = 0
	 * These functions were determined using computer math. The code can
	 * be found in:
	 * templates/Estimator/legAngleEstimator/Derive_Eqns.m
	 */
	x = l * Sin(qh) - d * Sin(Phi - q1 + qh) + d * Sin(Phi - q0);
	y = l + d * Cos(Phi - q1 + qh) - l * Cos(qh) - d * Cos(Phi - q0);

	mb_io_set_float(ID_EST_LAST_STEP_LENGTH, Sqrt(x * x + y * y)); // Distance between two contact points
	return  -(Atan(y / x) + Slope);	 //outer leg angle calculated from geometry
}


/* Updates the estimate of the outer leg angle on the robot by
 * combining information from the rate gyro and the geometry of
 * the robot (if both feet are on the ground). */
void updateRobotOrientation() {
	float zLast;  // Leg angle at the last time step
	float zGyro;  // Leg angle at this time step, based on rate gyro data
	float zGeom;  // Leg angle at this time step, based on geometry
	float z;  // leg angle at this time step, based on fusion of two sensors
	unsigned long t;  // Time to pass to filter
	float y; // new state estimate;
	float alpha = mb_io_get_float(ID_EST_ROBOT_ANGLE_GYRO_WEIGHT);

	// Figure out the expected new orientation, based on integral of gyro rate:
	zLast = FD_OUTER_LEG_ANGLE.z0; // Last un-filtered leg angle
	zGyro = getOuterLegAngleGyro(zLast);

	// Compute the expected orientation based on geometry, if both feet on ground
	if (STATE_contactMode == CONTACT_DS) {
		zGeom = getOuterLegAngleGeometry();
		z = alpha * zGyro + (1.0 - alpha) * zGeom;
	} else {
		z = zGyro;  // Update purely based on integral of gyro rate
	}

	// Run the filter using the fusion of the two sensors:
	t = mb_io_get_time(ID_UI_ANG_RATE_X);
	y = runFilter(&FC_FAST, &FD_OUTER_LEG_ANGLE, t, z);
	mb_io_set_float(ID_EST_STATE_TH0, y);
	STATE_th0 = y; // Send robot orientation rate to the control and estimation code

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

	// Figure out the contact mode:
	if (STATE_c0 && !STATE_c1) { // Single stance outer
		STATE_contactMode = CONTACT_S0;
	} else if (!STATE_c0 && STATE_c1) { // single stance inner
		STATE_contactMode = CONTACT_S1;
	} else if (STATE_c0 && STATE_c1) { // double stance
		STATE_contactMode = CONTACT_DS;
	} else {                           // Flight
		STATE_contactMode = CONTACT_FL;
	}

}


/* Updates any controller parameters that are set from LabVIEW */
void updateParameters(void) {
	LABVIEW_HIP_GRAVITY_COMPENSATION = mb_io_get_float(ID_CTRL_HIP_GRAVITY_COMPENSATION) > 0.5;
	LABVIEW_HIP_SPRING_COMPENSATION = mb_io_get_float(ID_CTRL_HIP_SPRING_COMPENSATION) > 0.5;
}


/********************* Public Functions ***********************************
 *                                                                        *
 **************************************************************************/

/* Sets the outer leg angle estimate, assuming that vector the bisects
 * the hip angle is pointing straight down. This is true when the robot
 * is hanging, or when it is in double stance with the feet flipped up,
 * or double stance with identical ankle angles. This is called when the
 * robot turns on, and also when the left-most button is pressed. */
void resetRobotOrientation(void) {
	float qh = mb_io_get_float(ID_MCH_ANGLE);
	setFilterData(&FD_OUTER_LEG_ANGLE, -0.5 * qh);
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
		initGyroIntegral();  // sets the static variables used to compute integral
		resetRobotOrientation();  // Also sets FD_OUTER_LEG_ANGLE

		// Remember that we've initialized everything properly
		INITIALIZE_ESTIMATOR = false;
	}

	// Run most of the butterworth filters:
	runFilter_ui_ang_rate_x();
	runFilter_jointRates();
	runFilter_contactOuter();
	runFilter_contactInner();

	// Run calculations for outer leg angle:
	updateRobotOrientation();

	// Update the state variables:  (absolute orientation and rate)
	updateRobotState();

	// Update controller parameters from LabVIEW
	updateParameters();


	return;
}
