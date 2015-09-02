/* This file contains functions that are no longer used on the robot
 * but that might be of use later and it is easier to find them here
 * than by searching through git. This file is no linked to the main 
 * project and is not running on the robot */





/********************************************************************
 *                 Double-Stance Estimator Update                   *
 ********************************************************************/



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
		zNew = zLast + 0.5 * dt * (dz + RATE_GYRO_LAST_RATE);
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
void updateRobotOrientatio_OLDVERSION() {
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


/**************************************************************************
 *                        Motor Control Stuff                             *
 **************************************************************************/             



/* Wrapper function.
 * Computes the scissor tracking gains such that the hip angle
 * tracks a linear function of the stance leg angle. The coefficients
 * are set by the boundary conditions:
 * Start: Hip angle measured at start of glide phase
 * FInal: Hip angle is at target angle when push-off begins */
void hipGlide_Saturate(void) {
	float th;  // Stance leg angle
	float q;  // target hip angle
	float dq = LABVIEW_WALK_HIP_TARGET_RATE;  // targe hip angular rate
	float qStar = LABVIEW_WALK_HIP_STEP_ANGLE;   // Should be positive
	float thStar = LABVIEW_WALK_CRIT_STANCE_ANGLE;   // Should be negative 
	switch (STATE_contactMode) {
	case CONTACT_S0:
		th = STATE_th0;
		th = Clamp(th,thStar,0.0);  // thStar is usually something like -0.1.
		q = qStar*th/thStar;
		trackVel_hip(q, dq, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
		break;
	case CONTACT_S1:
		th = STATE_th1;
		th = Clamp(th,thStar,0.0);  // thStar is usually something like -0.1.
		q = -qStar*th/thStar;  // Negate for hip angle convention
		trackVel_hip(q, -dq, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
		break;
	case CONTACT_DS:
		disable_hip();
		break;
	case CONTACT_FL:
		trackRel_hip(0.0, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
		break;
	}
}