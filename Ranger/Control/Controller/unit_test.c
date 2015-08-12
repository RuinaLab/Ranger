#include <mb_includes.h> 
#include <motorControl.h>
#include <fsm.h>
#include <unit_test.h>
#include <RangerMath.h>


// Hold the feet while making the hip angle track zero 
void hold_feet(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;

// Select the correct reference angle
		float time = mb_io_get_float(ID_TIMESTAMP);

	float hold = mb_io_get_float(ID_CTRL_ANK_REF_HOLD);
	float hold_kp = mb_io_get_float(ID_CTRL_ANK_HOLD_KP);
	float hold_kd = mb_io_get_float(ID_CTRL_ANK_HOLD_KD);
	 
	//update all the angle parameters
	angles_update();	 

	// hold outer feet
	out_ank_track_abs(&ctrlAnkOut, hold, 0.0, 0.0, hold_kp, hold_kd);
	// hold inner feet
	inn_ank_track_abs(&ctrlAnkInn, hold, 0.0, 0.0, hold_kp, hold_kd);
	hip_track_rel(&ctrlHip, 0.0, 0.0, mb_io_get_float(ID_CTRL_HIP_KP), mb_io_get_float(ID_CTRL_HIP_KD));

	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
	controller_hip(&ctrlHip);
}


/* Robot Hanging in the air
 * Hip motor off
 * Ankle motors track sine-curve reference
 * Parameters:
 *   - ID_CTRL_TEST_R0 = kp
 *	 - ID_CTRL_TEST_R1 = kd
 *   - ID_CTRL_TEST_R2 = sine curve period
 *   - ID_CTRL_TEST_W0 = reference angle
 */
void ankle_motor_test(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;

	float qUpp = 1.9;  // Maximum ankle angle
	float qLow = 0.1;  // minimum ankle angle					   

	float time = mb_io_get_float(ID_TIMESTAMP);
	
	float PI = 3.14159265359;

	float period;  // period for reference function
	float arg;  // input for trig functions
	float xRef;  // reference ankle angle
	float vRef;  // reference angle rate
	float kp;   // proportional gaint
	float kd;   // derivative gain		

	//period = mb_io_get_float(ID_CTRL_TEST_R2);
	period = 1.0;  // HACK - communications not working?

	arg = 2*PI*time/period;
	xRef = 0.5*(qLow + qUpp) + 0.5*(qUpp-qLow)*Sin(arg);
	vRef = (PI/period)*Cos(arg);

	mb_io_set_float(ID_CTRL_TEST_W0,xRef);

//	kp = mb_io_get_float(ID_CTRL_TEST_R0); 
//	kd = mb_io_get_float(ID_CTRL_TEST_R1);
	kp = 4.0;	    //// HACK - communications not working on robot ?
	kd = 1.;

		// Run a PD-controller on the outer foot angles:
		ctrlAnkOut.kp = kp;
		ctrlAnkOut.kd = kd;
		ctrlAnkOut.xRef = xRef;
		ctrlAnkOut.vRef = vRef;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);	

		// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.kp = kp;
		ctrlAnkInn.kd = kd;
		ctrlAnkInn.xRef = xRef;
		ctrlAnkInn.vRef = vRef;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
	
		// Disable the hip:
		ctrlHip.wn = 0.0;
		ctrlHip.xi = 0.0;
		ctrlHip.xRef = 0.0;
		ctrlHip.vRef = 0.0;
		ctrlHip.uRef = 0.0;
		controller_hip(&ctrlHip);
	
}


		 /* Generates data for fitting a motor model */
 void test_ankle_motor_model(void) {
 		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;

		// Open loop tracking test (random values between 0.2 and 2.0)
		float refAngle[60] = {1.735,  1.320,  0.832,  1.124,  0.923,  0.337,  0.632,  0.422,  0.531,  0.632,  0.951,  0.289,  1.825,  1.901,  1.084,  1.081,  0.808,  1.820,  0.865,  0.400,  1.604,  0.902,  0.635,  0.927,  0.374,  0.438,  1.896,  1.921,  1.235,  0.308,  0.623,  0.836,  1.678,  0.228,  0.277,  0.504,  1.368,  1.517,  1.366,  1.012,  1.185,  0.733,  1.540,  0.540,  1.436,  0.530,  0.863,  1.326,  1.604,  0.346,  1.873,  1.596,  1.076,  0.985,  1.004,  0.751,  1.115,  1.119,  1.672,  1.631};  		

		// Update value with a period of:
		float updatePeriod = 750.0;   // in milliseconds

		// Select the correct reference angle
		float time = mb_io_get_float(ID_TIMESTAMP);
		int nPeriods = (int) (time/updatePeriod);
		int index = nPeriods % 60;

		// Run a PD-controller on the inner foot angles:
		ctrlAnkOut.wn = 8.0;
		ctrlAnkOut.xi = 0.7;
		ctrlAnkOut.xRef = refAngle[index];
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);	

		// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.wn = 8.0;
		ctrlAnkInn.xi = 0.7;
		ctrlAnkInn.xRef = refAngle[index];
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
	
		// Run a PD-controller on the inner foot angles:
		ctrlHip.wn = 0.0;
		ctrlHip.xi = 0.0;
		ctrlHip.xRef = 0.0;
		ctrlHip.vRef = 0.0;
		ctrlHip.uRef = 0.0;
		controller_hip(&ctrlHip);
	
 }