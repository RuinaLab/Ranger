#include <mb_includes.h>
#include <unit_test.h>
#include <motorControl.h>
#include <RangerMath.h>

/* Hold both ankles at zero absolute angle */
void test_trackAbs_ankle() {
	static float kp = 7.0;
	static float kd = 1.0;
	static float phi = 0.0;  // Absolute tracking reference angle
	trackAbs_ankOut(phi, kp, kd);
	trackAbs_ankInn(phi, kp, kd);
	disable_hip();
}

/* Check that the wave functions in Ranger math are performing correctly.
 * Creates a sine wave, triangle wave, and a square wave, and then sends 
 * them over the CAN bus to LabVIEW, on channels:
 * 	ID_CTRL_TEST_W0, *_W1, *_W2 */
void test_waveFunctions(){
	float period = 1.0;
	float min = -0.5;
	float max = 2.2;
	float time = 0.001*mb_io_get_float(ID_TIMESTAMP);

	mb_io_set_float(ID_CTRL_TEST_W0, SquareWave(time,period,min,max));
	mb_io_set_float(ID_CTRL_TEST_W1, TriangleWave(time,period,min,max));
	mb_io_set_float(ID_CTRL_TEST_W2, SineWave(time,period,min,max));
}

/* Entry-point function for all unit tests */
void runUnitTest(void) {

	/***** Ranger Math ****/
	test_waveFunctions();

	/***** Motor Control ****/
	// test_trackAbs_ankle();

}
