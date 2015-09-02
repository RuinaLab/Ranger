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
static const float GYRO_ROLL_BIAS = -0.02;  // Measured September 1, 2015.
static const float CONTACT_VALUE_THRESHOLD = 2500.0;  // Threshold for detecting contact on the feet

/* Butterworth filter coefficients */
static FilterCoeff FC_FAST;  // For joint sensors
static FilterCoeff FC_SLOW;  // For contact sensors

/* Butterworth filters on orientation and rate sensors */
static FilterData FD_OUTER_LEG_ANGLE; // absolute angle of the outer legs
static FilterData FD_UI_ANG_RATE_X;  // stance leg angular rate
static FilterData FD_MCH_ANG_RATE;  // hip angle rate
static FilterData FD_MCFO_RIGHT_ANKLE_RATE ;  // outer ankle rate
static FilterData FD_MCFI_ANKLE_RATE;  // inner ankle rate

/* Butterworth filters on contact sensors */
static FilterData FD_MCFO_LEFT_HEEL_SENSE;
static FilterData FD_MCFO_RIGHT_HEEL_SENSE;
static FilterData FD_MCFI_LEFT_HEEL_SENSE;
static FilterData FD_MCFI_RIGHT_HEEL_SENSE;

/* Parameters from Labview */
bool LABVIEW_HIP_COMPENSATION_TARGET; // Hip compensation at the target (true) or measured state (false)
bool LABVIEW_HIP_GRAVITY_COMPENSATION;  // enable gravity compensation in hip?
bool LABVIEW_HIP_SPRING_COMPENSATION; // enable spring compensation in hip?
float LABVIEW_HIP_KP;  // hip pd controller p gain
float LABVIEW_HIP_KD;  // hip pd controller d gain
float LABVIEW_ANK_PUSH_KP;  // ankle p gain used during push off
float LABVIEW_ANK_PUSH_KD;  // ankle d gain used during push off
float LABVIEW_ANK_STANCE_KP;  // ankle pd controller p gain when foot on ground.
float LABVIEW_ANK_STANCE_KD;  // ankle pd controller d gain when foot on ground.
float LABVIEW_ANK_SWING_KP;  // ankle pd controller p gain when foot in air.
float LABVIEW_ANK_SWING_KD;  // ankle pd controller d gain when foot in air.
float LABVIEW_WALK_CRIT_STANCE_ANGLE; // Angle that the stance leg must rotate through for the FSM to switch
float LABVIEW_WALK_ANK_PUSH;  //magnitude of the push-off during walking, normalized to be on the range [0,1]
float LABVIEW_WALK_HIP_RATE;  //scissor tracking rate, should be near one (~0.5, ~1.5)
float LABVIEW_WALK_HIP_OFFSET;  //How much the swing leg should lead the stance leg during scissor tracking
float LABVIEW_WALK_ANK_PUSH; // magnitude of the push-off during walking  normalized to be on the range 0 to 1
float LABVIEW_WALK_CRIT_STANCE_ANGLE; // the critical stance leg angle when push-off should occur
float LABVIEW_WALK_HIP_STEP_ANGLE; //	Target angle for the hip to reach by the end of the step
float LABVIEW_WALK_SCISSOR_GAIN;  
float LABVIEW_WALK_SCISSOR_OFFSET;

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

/* Initializes the filter to a single value. */
void setFilterData(FilterData * FD, CAN_ID canId) {
	unsigned long t = mb_io_get_time(canId);  // Time stamp on data
	float z = mb_io_get_float(canId);  // Value of the data
	FD->z0 = z;
	FD->z1 = z;
	FD->z2 = z;
	FD->y0 = z;
	FD->y1 = z;
	FD->y2 = z;
	FD->t0 = t;
}

/* Updates the Butterworth filter based on new sensor data. The filter runs
 * with a period of 1ms, so that it can handle asynchronous data.
 * @param canId = the CAN ID for the data channel of interest
 * @return y = the filtered data at the current time step
 */
float runFilter(FilterCoeff * FC, FilterData * FD, CAN_ID canId) {

	unsigned long t;  // Time stamp on data
	float z;  // Value of the data
	int tDiff;  // Diference between current data time and last data time

	t = mb_io_get_time(canId);
	z = mb_io_get_float(canId);
	tDiff = (int) (t - (FD->t0));

	// Error checking on timing:
	if (tDiff > 30 || tDiff < 0) { // Something went wrong
		mb_error_occurred(ERROR_EST_FILTER_TIME_VIOLATION);
		setFilterData(FD, canId);  // Reset the filter
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


/* Runs the butterworth filters for the robot */
void runAllFilters(void) {
	float y;  // Store the estimate

	// outer leg absolute orientation
	y = runFilter(&FC_FAST, &FD_OUTER_LEG_ANGLE, ID_UI_ROLL);
	y = y - GYRO_ROLL_BIAS;  // correct for bias term in the gyro
	mb_io_set_float(ID_EST_STATE_TH0, y);
	STATE_th0 = y; // Send robot orientation to the control and estimation code

	// Outer leg absolute orientation rate
	y = runFilter(&FC_FAST, &FD_UI_ANG_RATE_X, ID_UI_ANG_RATE_X);
	y = y - GYRO_RATE_BIAS;   // correct for gyro bias
	mb_io_set_float(ID_EST_STATE_DTH0, y);
	STATE_dth0 = y; // Send robot orientation rate to the control and estimation code

	// Hip Angular Rate:
	y = runFilter(&FC_FAST, &FD_MCH_ANG_RATE, ID_MCH_ANG_RATE);
	mb_io_set_float(ID_E_MCH_ANG_RATE, y);
	STATE_dqh = y;

	// Outer Ankle Rate:
	y = runFilter(&FC_FAST, &FD_MCFO_RIGHT_ANKLE_RATE, ID_MCFO_RIGHT_ANKLE_RATE);
	mb_io_set_float(ID_E_MCFO_RIGHT_ANKLE_RATE, y);
	STATE_dq0 = y;

	// Inner Ankle Rate:
	y = runFilter(&FC_FAST, &FD_MCFI_ANKLE_RATE, ID_MCFI_ANKLE_RATE);
	mb_io_set_float(ID_E_MCFI_ANKLE_RATE, y);
	STATE_dq1 = y;

	// Outer feet contact filter:
	y = runFilter(&FC_SLOW, &FD_MCFO_RIGHT_HEEL_SENSE, ID_MCFO_RIGHT_HEEL_SENSE);
	y = y + runFilter(&FC_SLOW, &FD_MCFO_LEFT_HEEL_SENSE, ID_MCFO_LEFT_HEEL_SENSE);
	mb_io_set_float(ID_EST_CONTACT_OUTER, y);
	STATE_c0 = y > CONTACT_VALUE_THRESHOLD;

	// Inner feet contact filter
	y = runFilter(&FC_SLOW, &FD_MCFI_RIGHT_HEEL_SENSE, ID_MCFI_RIGHT_HEEL_SENSE);
	y = y + runFilter(&FC_SLOW, &FD_MCFI_LEFT_HEEL_SENSE, ID_MCFI_LEFT_HEEL_SENSE);
	mb_io_set_float(ID_EST_CONTACT_INNER, y);
	STATE_c1 = y > CONTACT_VALUE_THRESHOLD;

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
	LABVIEW_HIP_COMPENSATION_TARGET = mb_io_get_float(ID_CTRL_HIP_COMPENSATION_TARGET) > 0.5;
	LABVIEW_HIP_GRAVITY_COMPENSATION = mb_io_get_float(ID_CTRL_HIP_GRAVITY_COMPENSATION) > 0.5;
	LABVIEW_HIP_SPRING_COMPENSATION = mb_io_get_float(ID_CTRL_HIP_SPRING_COMPENSATION) > 0.5;
	LABVIEW_HIP_KP = mb_io_get_float(ID_CTRL_HIP_KP);  // hip pd controller p gain
	LABVIEW_HIP_KD = mb_io_get_float(ID_CTRL_HIP_KD);  // hip pd controller d gain
	LABVIEW_ANK_PUSH_KP = mb_io_get_float(ID_CTRL_ANK_PUSH_KP);  // push-off ankle p gain
	LABVIEW_ANK_PUSH_KD = mb_io_get_float(ID_CTRL_ANK_PUSH_KD);  // push-off ankle d gain
	LABVIEW_ANK_STANCE_KP = mb_io_get_float(ID_CTRL_ANK_STANCE_KP);  // ankle pd controller p gain when foot on ground.
	LABVIEW_ANK_STANCE_KD = mb_io_get_float(ID_CTRL_ANK_STANCE_KD);  // ankle pd controller d gain when foot on ground.
	LABVIEW_ANK_SWING_KP = mb_io_get_float(ID_CTRL_ANK_SWING_KP);  // ankle pd controller p gain when foot in air.
	LABVIEW_ANK_SWING_KD = mb_io_get_float(ID_CTRL_ANK_SWING_KD);  // ankle pd controller d gain when foot in air.
	LABVIEW_WALK_ANK_PUSH = mb_io_get_float(ID_CTRL_WALK_ANK_PUSH); // magnitude of the push-off during walking  normalized to be on the range 0 to 1
	LABVIEW_WALK_CRIT_STANCE_ANGLE = mb_io_get_float(ID_CTRL_WALK_CRIT_STANCE_ANGLE); // the critical stance leg angle when push-off should occur
	LABVIEW_WALK_HIP_STEP_ANGLE = mb_io_get_float(ID_CTRL_WALK_HIP_STEP_ANGLE); //	Target angle for the hip to reach by the end of the step
	LABVIEW_WALK_SCISSOR_GAIN = mb_io_get_float(ID_CTRL_WALK_HIP_STEP_ANGLE);
	LABVIEW_WALK_SCISSOR_OFFSET = mb_io_get_float(ID_CTRL_WALK_HIP_STEP_ANGLE);
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
		setFilterData(&FD_OUTER_LEG_ANGLE, ID_UI_ROLL);
		setFilterData(&FD_UI_ANG_RATE_X, ID_UI_ANG_RATE_X);
		setFilterData(&FD_MCH_ANG_RATE, ID_MCH_ANG_RATE);
		setFilterData(&FD_MCFO_RIGHT_ANKLE_RATE, ID_MCFO_RIGHT_ANKLE_RATE);
		setFilterData(&FD_MCFI_ANKLE_RATE, ID_MCFI_ANKLE_RATE);

		// Reset the contact sensor filters
		setFilterData(&FD_MCFO_LEFT_HEEL_SENSE, ID_MCFO_LEFT_HEEL_SENSE);
		setFilterData(&FD_MCFO_RIGHT_HEEL_SENSE, ID_MCFI_RIGHT_HEEL_SENSE);
		setFilterData(&FD_MCFI_LEFT_HEEL_SENSE, ID_MCFI_LEFT_HEEL_SENSE);
		setFilterData(&FD_MCFI_RIGHT_HEEL_SENSE, ID_MCFI_RIGHT_HEEL_SENSE);

		// Remember that we've initialized everything properly
		INITIALIZE_ESTIMATOR = false;
	}

	// Run the butterworth filters:
	runAllFilters();

	// Update the state variables:  (absolute orientation and rate)
	updateRobotState();

	// Update controller parameters from LabVIEW
	updateParameters();


	return;
}
