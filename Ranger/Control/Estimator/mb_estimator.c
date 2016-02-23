#include <mb_includes.h>
#include <mb_estimator.h>
#include <RangerMath.h>   // Tan(), bool
#include <input_output.h> // LED functions
#include <robotParameters.h>
#include <PSO.h>
#include <optimizeGait.h>

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
	CAN_ID canId; // CAN ID for the sensor
} FilterData;

/* Global variables */
bool INITIALIZE_ESTIMATOR = true;   // Should the estimator be initialized?

/* Filter cut-off frequencies */
static const float CLOCK_CYCLE_DURATION = 0.002;   // Ranger runs the main brain at 500Hz
static const float FILTER_CUTOFF_FAST = 2 * CLOCK_CYCLE_DURATION * 20.0; // (2*period in sec)*(cutoff frequency in Hz) -- Used by joint sensors
static const float FILTER_CUTOFF_SLOW = 2 * CLOCK_CYCLE_DURATION * 10.0; // (2*period in sec)*(cutoff frequency in Hz) -- Used by foot contact sensors
static const float FILTER_CUTOFF_VERY_SLOW = 2 * CLOCK_CYCLE_DURATION * 2.0; //  (2*period in sec)*(cutoff frequency in Hz) -- Used by steering motor

/* First-Order filter, once per step */
static const float ORIENTATION_GYRO_UPDATE_GAIN = 0.1;  //  0 -> no update, 1 -> full reset at heel-strike

/* Local constant parameters */
static const float GYRO_RATE_BIAS = -0.0086335786; // Measured experimentally on Feb 23, 2016. Should be checked monthly.
static const float GYRO_ROLL_BIAS = -0.021; // Measured experimentally on Feb 23, 2016
static const float CONTACT_VALUE_THRESHOLD = 850.0;  // Threshold for detecting contact on the feet. Roughly 700 in swing, 7000 in stance

/* Butterworth filter coefficients */
static FilterCoeff FC_FAST;  // For joint sensors
static FilterCoeff FC_SLOW;  // For contact sensors
static FilterCoeff FC_VERY_SLOW;  // for steering motor

/* Butterworth filters on orientation and rate sensors */
static FilterData FD_OUTER_LEG_ANGLE; // absolute angle of the outer legs
static FilterData FD_UI_ANG_RATE_X;  // stance leg angular rate
static FilterData FD_MCH_ANG_RATE;  // hip angle rate
static FilterData FD_MCFO_RIGHT_ANKLE_RATE;  // outer ankle rate
static FilterData FD_MCFI_ANKLE_RATE;  // inner ankle rate

/* Butterworth filters on contact sensors */
static FilterData FD_MCFO_LEFT_HEEL_SENSE;
static FilterData FD_MCFO_RIGHT_HEEL_SENSE;
static FilterData FD_MCFI_LEFT_HEEL_SENSE;
static FilterData FD_MCFI_RIGHT_HEEL_SENSE;

/* Butterworth filter on steering angle */
static FilterData FD_MCSI_STEER_ANGLE;

/* Power used by boards */
static float POWER_MCH;
static float POWER_MCFO;
static float POWER_MCFI;
static float POWER_OVERHEAD;

// /* Parameters from Labview */
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
float LABVIEW_WALK_CRIT_STEP_LENGTH; // the critical stance leg angle when push-off should occur
float LABVIEW_WALK_SCISSOR_GAIN;
float LABVIEW_WALK_SCISSOR_OFFSET;
float LABVIEW_WALK_DS_DELAY;
bool LABVIEW_GAIT_USE_MDP_DATA;  // True if walking controller should use MDP generated gait data.

/* Robot state variables. Naming conventions in docs. Matches simulator. */
bool STATE_IS_FALLEN = false;   // Is the robot in a fallen state?
float STATE_t;  // Robot Time IN SECONDS   (NOTE: converted from mb_io_get_float(TIMESTAMP), which is in ms)
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
float STATE_psi;  // Steering angle    (Currently unused due to BROKEN STEERING ANGLE SENSOR)
float STATE_posCom;  // horizontal component of the center of mass position
float STATE_velCom;  // horizontal component of the center of mass velocity
float STATE_lastStepLength;  // length of the last step (meters)
float STATE_lastStepDuration;  // Duration of the last step (seconds)

/* Variables for sensor fusion on the outer leg angle */
static float STATE_th0_gyro;  // outer leg angle, based on integral of the gyro
static float STATE_th0_imu;  // outer leg angle, based on the IMU internal sensor fusion
static float STATE_lastStepTimeSec;  //  cpu clock time at last heel strike.

/* Contact configuration */
bool STATE_c0;
bool STATE_c1;
ContactMode STATE_contactMode = CONTACT_FL;

/* Initializes the filter to a single value. */
void setFilterData(FilterData * FD, CAN_ID canId) {
	float z = mb_io_get_float(canId);  // Value of the data
	FD->z0 = z;
	FD->z1 = z;
	FD->z2 = z;
	FD->y0 = z;
	FD->y1 = z;
	FD->y2 = z;
	FD->canId = canId;
}

/* Updates the Butterworth filter based on new sensor data. The filter runs
 * with a period of 1ms, so that it can handle asynchronous data.
 * @param canId = the CAN ID for the data channel of interest
 * @return y = the filtered data at the current time step
 */
float runFilter(FilterCoeff * FC, FilterData * FD) {

	float z = mb_io_get_float(FD->canId);

	// Update sensor history:
	FD->z2 = FD->z1;
	FD->z1 = FD->z0;
	FD->z0 = z;
	// Update estimate history:
	FD->y2 = FD->y1;
	FD->y1 = FD->y0;
	// Compute new estimate:
	FD->y0 =
	    (FC->b0) * (FD->z0) +
	    (FC->b1) * (FD->z1) +
	    (FC->b2) * (FD->z2) -
	    (FC->a1) * (FD->y1) -
	    (FC->a2) * (FD->y2);

	return (FD->y0);
}


/* Runs the butterworth filters for the robot */
void runAllFilters(void) {
	float y;  // Store the estimate

	// outer leg absolute orientation
	y = runFilter(&FC_FAST, &FD_OUTER_LEG_ANGLE);
	y = y - GYRO_ROLL_BIAS;  // correct for bias term in the gyro
	mb_io_set_float(ID_EST_STATE_TH0_IMU, y);
	STATE_th0_imu = y; // Send robot orientation to the control and estimation code

	// Outer leg absolute orientation rate
	y = runFilter(&FC_FAST, &FD_UI_ANG_RATE_X);
	y = y - GYRO_RATE_BIAS;   // correct for gyro bias
	mb_io_set_float(ID_EST_STATE_DTH0, y);
	STATE_dth0 = y; // Send robot orientation rate to the control and estimation code

	// Hip Angular Rate:
	y = runFilter(&FC_FAST, &FD_MCH_ANG_RATE);
	mb_io_set_float(ID_E_MCH_ANG_RATE, y);
	STATE_dqh = y;

	// Outer Ankle Rate:
	y = runFilter(&FC_FAST, &FD_MCFO_RIGHT_ANKLE_RATE);
	mb_io_set_float(ID_E_MCFO_RIGHT_ANKLE_RATE, y);
	STATE_dq0 = y;

	// Inner Ankle Rate:
	y = runFilter(&FC_FAST, &FD_MCFI_ANKLE_RATE);
	mb_io_set_float(ID_E_MCFI_ANKLE_RATE, y);
	STATE_dq1 = y;

	// Outer feet contact filter:
	y = runFilter(&FC_SLOW, &FD_MCFO_RIGHT_HEEL_SENSE);
	y = y + runFilter(&FC_SLOW, &FD_MCFO_LEFT_HEEL_SENSE);
	mb_io_set_float(ID_EST_CONTACT_OUTER, y);
	STATE_c0 = y > CONTACT_VALUE_THRESHOLD;

	// Inner feet contact filter
	y = runFilter(&FC_SLOW, &FD_MCFI_RIGHT_HEEL_SENSE);
	y = y + runFilter(&FC_SLOW, &FD_MCFI_LEFT_HEEL_SENSE);
	mb_io_set_float(ID_EST_CONTACT_INNER, y);
	STATE_c1 = y > CONTACT_VALUE_THRESHOLD;

	// Steering angle
	y = runFilter(&FC_VERY_SLOW, &FD_MCSI_STEER_ANGLE);
	mb_io_set_float(ID_EST_STATE_PSI, y);
	STATE_psi = y;

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


/* Sub-function, called by updateComKinematics(), used to compute
 * the horizontal position of the robot's center of mass with
 * respect to the stance ankle joint */
float getComPos(float qStance, float qSwing) {
	float sinStance = Sin(qStance);
	float posHip = -PARAM_l * sinStance;
	float posStance = posHip + PARAM_c * sinStance;
	float posSwing = posHip + PARAM_c * Sin(qSwing);
	return 0.5 * (posSwing + posStance);
}

/* Sub-function, called by updateComKinematics(), used to compute
 * the horizontal velocity of the robot's center of mass with
 * respect to the stance ankle joint */
float getComVel(float qStance, float qSwing, float dqStance, float dqSwing) {
	float dSinStance = dqStance * Cos(qStance);
	float velHip = -PARAM_l * dSinStance;
	float velStance = velHip + PARAM_c * dSinStance;
	float velSwing = velHip + PARAM_c * (dqSwing * Cos(qSwing));
	return 0.5 * (velSwing + velStance);
}

/* Computes the horizontal component of the robot's position and velocity,
and then updates the parameters in both labview and the estimator. */
void updateComKinematics(void) {
	float pos, vel;
	switch (STATE_contactMode) {
	case CONTACT_S0:
		pos = getComPos(STATE_th0, STATE_th1);
		vel = getComVel(STATE_th0, STATE_th1, STATE_dth0, STATE_dth1);
		break;
	case CONTACT_S1:
		pos = getComPos(STATE_th1, STATE_th0);
		vel = getComVel(STATE_th1, STATE_th0, STATE_dth1, STATE_dth0);
		break;
	default:
		pos = 0.0;
		vel = 0.0;
		break;
	}
	// Send updates to the estimator and also to LabVIEW
	STATE_posCom = pos;
	STATE_velCom = vel;
	mb_io_set_float(ID_EST_STATE_POSCOM, pos);
	mb_io_set_float(ID_EST_STATE_VELCOM, vel);
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
	STATE_phi0 = PARAM_Phi0 + STATE_th0 - STATE_q0;  // absolute orientation of outer feet
	STATE_phi1 = PARAM_Phi1  + STATE_qh + STATE_th0 - STATE_q1; // absolute orientation of inner feet
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

	mb_io_set_float(ID_EST_STATE_CONTACTMODE, STATE_contactMode); // contact mode (e.g. flying, s0...)
	mb_io_set_float(ID_EST_UI_FSM_MODE, UI_FSM_MODE); // mb_controller FSM (e.g. StandBy, WalkCtrl,...)
	mb_io_set_float(ID_OPTIM_FSM_MODE, OPTIMIZE_FSM_MODE); // optimizeGait FSM (e.g. INIT, TRIAL,...)


	// Figure out the contact mode:
	if (STATE_c0 && !STATE_c1) { // Single stance outer
		STATE_contactMode = CONTACT_S0;
	} else if (!STATE_c0 && STATE_c1) { // single stance inner
		STATE_contactMode = CONTACT_S1;
	} else if (STATE_c0 && STATE_c1) { // double stance
		STATE_contactMode = CONTACT_DS;
	} else {                           // Flight
		STATE_contactMode = CONTACT_FL;
		resetRobotOrientation();  // use imu sensor fusion to reset the robot orientation
	}

	// Do a bit of harder math to figure out the horizontal component of the CoM motion
	updateComKinematics();

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
	LABVIEW_WALK_CRIT_STEP_LENGTH = mb_io_get_float(ID_CTRL_WALK_CRIT_STEP_LENGTH); // the critical ankle joint separation when push-off should occur
	LABVIEW_WALK_SCISSOR_GAIN = mb_io_get_float(ID_CTRL_WALK_SCISSOR_GAIN);
	LABVIEW_WALK_SCISSOR_OFFSET = mb_io_get_float(ID_CTRL_WALK_SCISSOR_OFFSET);
	LABVIEW_GAIT_USE_MDP_DATA = mb_io_get_float(ID_GAIT_USE_MDP_DATA) > 0.5;  // True if walking controller should use off-line generated gait data.
	FSM_LED_FLAG = mb_io_get_float(ID_CTRL_FSM_LED) > 0.5;  // True if FSM LEDs should be turned on
}

/* Computes the change in the angle of the outer legs of the robot
 * that is expected due to the integral of the IMU rate signal over
 * the last time step */
float getIntegralRateGyro(void) {
	static float dth0_last = 0.0;
	float dth0_gyro = mb_io_get_float(ID_UI_ANG_RATE_X);
	float trapz = 0.5 * CLOCK_CYCLE_DURATION * (dth0_last + dth0_gyro);  // Trapazoid integration
	dth0_last = dth0_gyro;
	return trapz;
}


/* Reset the integral of the rate gyro to match the estimate based on the
 * internal sensor fusion of the IMU */
void resetRobotOrientation(void) {
	STATE_th0_gyro = STATE_th0_imu;
}


/* Update the robot orientation by integral of the rate gyro */
void updateRobotOrientation(void) {
	STATE_th0_gyro = STATE_th0_gyro + getIntegralRateGyro();
	mb_io_set_float(ID_EST_STATE_TH0_GYRO, STATE_th0_gyro);

	// For now, just set the robot state to be the imu internal sensor fusion:
	STATE_th0 = 1.0 * STATE_th0_imu + 0.0 * STATE_th0_gyro; ////HACK////  Only use the IMU's internal estimate for now.
	mb_io_set_float(ID_EST_STATE_TH0, STATE_th0);
}


/* Once per set this function is called to partially reset the integral of the rate gyro
 * based on the geometry at heel-strike */
void heelStrikeGyroReset(float th0_heelStrike) {
	float a = ORIENTATION_GYRO_UPDATE_GAIN;
	STATE_th0_gyro = a * th0_heelStrike + (1 - a) * STATE_th0_gyro;
}


/* Sets the estimate for the step length and the stance leg angle, assuming that the robot
 * is in double stance. This is designed to be called once per step, by the walking finite
 * state machine. Returns the step length. Posts other data as STATE variables and to CAN 
 * bus.*/
float computeHeelStrikeGeometry(void) {

	float Slope = 0.0;  // Assume flat ground for now
	float x, y; // scalar distances, in coordinate system aligned with outer legs

	float Phi0 = PARAM_Phi0;
		float Phi1 = PARAM_Phi1;
	float l = PARAM_l;
	float d = PARAM_d;
	float qh, q0, q1; // robot joint angles
	float stepLength, stepAngle;
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
	x = l * Sin(qh) - d * Sin(Phi1 - q1 + qh) + d * Sin(Phi0 - q0);
	y = l + d * Cos(Phi1 - q1 + qh) - l * Cos(qh) - d * Cos(Phi0 - q0);

	stepLength = Sqrt(x * x + y * y);  // Distance between two contact points
	stepAngle = -(Atan(y / x) + Slope);   // angle of the outer legs

	mb_io_set_float(ID_EST_LAST_STEP_LENGTH, stepLength);
	mb_io_set_float(ID_EST_LAST_STEP_ANGLE, stepAngle);

	STATE_lastStepLength = stepLength;

	heelStrikeGyroReset(stepAngle);  // Reset the gyro estiamte for angles at heel-strike
	return stepLength;
}

/* This function is called by gaitControl during walking, whenever there is a heel-strike,
 * which should occur whenever the foot strikes the ground during walking. */
void triggerHeelStrikeUpdate(void){
	float newStepTime;
	float stepDuration;
	float stepLength;

	/// STEP LENGTH:
	stepLength = computeHeelStrikeGeometry();

	/// STEP DURATION:
	newStepTime = STATE_t;  // In seconds
	stepDuration = newStepTime - STATE_lastStepTimeSec; // Duration of the last step (seconds)
	STATE_lastStepDuration = stepDuration;
	STATE_lastStepTimeSec = newStepTime;
	mb_io_set_float(ID_EST_LAST_STEP_DURATION, stepDuration);	

	/// Update the objective function:
	logStepData(stepDuration, stepLength);

}


/* This function sums up the power used by all boards and sends it out as a single
 * data value to LabVIEW */
void sendTotalPower(void) {
	float power;

	// Read CAN bus:
	POWER_MCH = mb_io_get_float(ID_MCH_BATT_POWER);
	POWER_MCFO = mb_io_get_float(ID_MCFO_BATT_POWER);
	POWER_MCFI = mb_io_get_float(ID_MCFI_BATT_POWER);
	POWER_OVERHEAD = mb_io_get_float(ID_CSR_MCU_POWER);

	// Accumulate power use and send out
	power = POWER_MCH + POWER_MCFO + POWER_MCFI + POWER_OVERHEAD;  
	mb_io_set_float(ID_EST_TOTAL_BATT_POWER, power);
}

/* This function accumulates the total energy used by each board */
void updateEnergyUsage(void){
	static float t = 0;  // time at last data point
	static float pHip = 0;  // power used by the hip
	static float p0 = 0;  // power used by the outer feet
	static float p1 = 0;  // power used by the inner feet
	static float pCpu = 0;  // power used by the overheads	
	static float eHip = 0;  // energy used by the hip
	static float e0 = 0;  // energy used by the outer feet
	static float e1 = 0;  // energy used by the inner feet
	static float eCpu = 0;  // energy used by the overheads
	float dt = 0; // time step between current and previous data

	// Approximate integral by trapezoid method
	dt = STATE_t - t;
	eHip = eHip + 0.5*dt*(pHip + POWER_MCH);
	e0 = e0 + 0.5*dt*(p0 + POWER_MCFO);
	e1 = e1 + 0.5*dt*(p1 + POWER_MCFI);
	eCpu = eCpu + 0.5*dt*(pCpu + POWER_OVERHEAD);

	// Send estimates:
	mb_io_set_float(ID_EST_ENERGY_MCH,eHip);
	mb_io_set_float(ID_EST_ENERGY_MCFO,e0);
	mb_io_set_float(ID_EST_ENERGY_MCFI,e1);
	mb_io_set_float(ID_EST_ENERGY_OVERHEAD,eCpu);

	// Update static power variables:
	t = STATE_t;
	pHip = POWER_MCH;
	p0 = POWER_MCFO;
	p1 = POWER_MCFI;
	pCpu = POWER_OVERHEAD;

}


/* Checks to see if the robot fell down, by measuring the absolute angle of both legs
 * If the absolute value of the legs are outside of the bounds, then it sets the
 * IS_FALLEN flag to true  */
void checkIfRobotFellDown(void) {

	STATE_IS_FALLEN = false;

	// Check outer and then inner legs:
	if (Abs(STATE_th0) > PARAM_critical_fall_leg_angle) {
		STATE_IS_FALLEN = true;
	} else if (Abs(STATE_th1) > PARAM_critical_fall_leg_angle) {
		STATE_IS_FALLEN = true;
	}

	// If both feet on the ground and both leg angles have the same sign... no good.
	if (STATE_c1 && STATE_c0){
		if (STATE_th0*STATE_th1 > PARAM_critDoubleFailAngleSqr){
			STATE_IS_FALLEN = true;
		}
	}

	// DO STUFF!
	if (STATE_IS_FALLEN) {
		mb_error_occurred(ERROR_EST_ROBOT_FALL);   	// Throw an error to labview if the robot falls
		enterSafeMode();   // Make the motors do something reasonable.
	}

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
		setFilterCoeff(&FC_VERY_SLOW, FILTER_CUTOFF_VERY_SLOW);

		// Reset the joint angle rate filters
		setFilterData(&FD_OUTER_LEG_ANGLE, ID_UI_ROLL);
		setFilterData(&FD_UI_ANG_RATE_X, ID_UI_ANG_RATE_X);
		setFilterData(&FD_MCH_ANG_RATE, ID_MCH_ANG_RATE);
		setFilterData(&FD_MCFO_RIGHT_ANKLE_RATE, ID_MCFO_RIGHT_ANKLE_RATE);
		setFilterData(&FD_MCFI_ANKLE_RATE, ID_MCFI_ANKLE_RATE);

		// Reset the contact sensor filters
		setFilterData(&FD_MCFO_LEFT_HEEL_SENSE, ID_MCFO_LEFT_HEEL_SENSE);
		setFilterData(&FD_MCFO_RIGHT_HEEL_SENSE, ID_MCFO_RIGHT_HEEL_SENSE);
		setFilterData(&FD_MCFI_LEFT_HEEL_SENSE, ID_MCFI_LEFT_HEEL_SENSE);
		setFilterData(&FD_MCFI_RIGHT_HEEL_SENSE, ID_MCFI_RIGHT_HEEL_SENSE);

		// Steering motor stuff:
		setFilterData(&FD_MCSI_STEER_ANGLE, ID_MCSI_STEER_ANGLE);

		// Robot orientation estimation
		resetRobotOrientation();
		getIntegralRateGyro();   // Run integral to log the current state of the rate gyro

		// Set "once per step" variables:
		STATE_lastStepLength = 0.0;    // Initialize to zero, for lack of a better plan
		STATE_lastStepTimeSec = STATE_t;  //  cpu clock time at last heel strike.
		STATE_lastStepDuration = 0.0;  // Duration of the last step (seconds)


		// Remember that we've initialized everything properly
		INITIALIZE_ESTIMATOR = false;
	}

	STATE_t = 0.001*mb_io_get_float(ID_TIMESTAMP);  // Robot Time (converted to seconds)
	runAllFilters();// Run the butterworth filters:
	updateRobotOrientation();
	sendTotalPower();  
	updateEnergyUsage(); // Must come after sendTotalPower()

	// Update the state variables:  (absolute orientation and rate)
	updateRobotState();

	// Update controller parameters from LabVIEW
	updateParameters();

	// Check if the robot fell down
	checkIfRobotFellDown();

	return;
}
