#include <mb_includes.h>
#include <unit_test.h>
#include <motorControl.h>
#include <RangerMath.h>


/*Simple function to return the robot time in seconds */
float getTime(){
return 0.001*mb_io_get_float(ID_TIMESTAMP);
}


/* Check that the wave functions in Ranger math are performing correctly.
 * Creates a sine wave, triangle wave, and a square wave, and then sends 
 * them over the CAN bus to LabVIEW, on channels:
 * 	ID_CTRL_TEST_W0, *_W1, *_W2 */
void test_waveFunctions(){
	float period = 1.0;
	float min = -0.5;
	float max = 2.2;
	float time = getTime();

	mb_io_set_float(ID_CTRL_TEST_W0, SquareWave(time,period,min,max));
	mb_io_set_float(ID_CTRL_TEST_W1, TriangleWave(time,period,min,max));
	mb_io_set_float(ID_CTRL_TEST_W2, SineWave(time,period,min,max));
}


/* Hold both ankles at zero absolute angle */
void test_trackAbs_ankle() {
	float kp = 7.0;
	float kd = 1.0;
	float phi = 0.0;  // Absolute tracking reference angle
	trackAbs_ankOut(phi, kp, kd);
	trackAbs_ankInn(phi, kp, kd);
	disable_hip();
}


/* Have the outer ankles track a square wave and the inner ankles track 
 * a triangle wave. The max and min values are set based on reasonable values
 * for flip-up and push-off target angles */
 void test_trackRel_ankle(){
 	float kp = 7.0;
	float kd = 1.0;
	float max = 2.6; // Push-off
	float min = 0.2; // Flip-up
	float period = 2.0;
	float q0, q1;  // Target angles
	float time = getTime();
	q0 = SquareWave(time,period,min,max);
	q1 = TriangleWave(time,period,min,max);
	trackRel_ankOut(q0,kp,kd);
	trackRel_ankInn(q1,kp,kd);
 }


 /* Hang the robot from the ceiling. The hip motor should 
  * attempt to cancel the effect of the hip spring. */
 void test_hipSpringCompensation(){
 	disable_ankOut();
 	disable_ankInn();
 	trackRel_hip(0.0, 0.0, 0.0); // 
 }


/* Entry-point function for all unit tests */
void runUnitTest(void) {

	/***** Ranger Math ****/
	//test_waveFunctions();

	/***** Motor Control ****/
	//test_trackAbs_ankle();
	//test_trackRel_ankle();
	test_hipSpringCompensation();
}
