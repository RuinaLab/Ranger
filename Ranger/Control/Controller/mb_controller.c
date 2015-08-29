#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>
#include <fsm.h>
#include <unit_test.h>

/* Currespond to buttons on UI board */
enum ControlMode {
	M0_Calibrate,
	M3_UnitTest,
	M4_FSM,
	M5_StandBy,
};

/* All test functions are located in unit_test.c 
 * Functions (fsm_param_updated, fsm_init & fsm_run) related to FSM are located in fsm.c 
 * calibrate() can be found in mb_estimator.c   
 * disable_motors() can be found in motorController.c
 */

/*  ENTRY-POINT FUNCTION FOR ALL CONTROL CODE */
void mb_controller_update(void) {

	// Enter stand-by mode when robot first boots
	static enum ControlMode controlMode = M5_StandBy;

	// Check UI buttons to update control mode
	if (detect_UI_button_input(0)) controlMode = M0_Calibrate;	 // 1st button calibrates
	if (detect_UI_button_input(3)) controlMode = M3_UnitTest; 	// 4th button flip feet up
	if (detect_UI_button_input(4)) controlMode = M4_FSM;	 // 5th button moves Ranger
	if (detect_UI_button_input(5)) controlMode = M5_StandBy; // 6th button Stand-by, always goes last (highest priority)

	// Run the desired control mode
	switch (controlMode) {
	case M5_StandBy:	//Button 6 (right-most button)
		set_UI_LED(5, 'g');
		//setPush(); //calls this for the step function in unit_test.c
		fsm_init();	//calls this for the FSM
		fsm_param_update(); //read parameters from LABVIEW for FSM
		disable_motors();
		test_gyro_angle_init(); //calls this for the unit test function of gyro angle
		break;
	case M4_FSM:	//Button 5 (second button from the right)
		set_UI_LED(5, 'r');
		fsm_run();
		break;
	case M3_UnitTest: //Button 4 (third button from the right)
		set_UI_LED(5, 'p');
		//test_trajectory();
		//track_sin();
		//double_stance();
		//foot_flip();
		//hold_feet();
		//motors_off();
		//ankle_motor_test();
		//test_feet();
		test_gravity_compensation();
		//hip_motor_test();
		//test_hip_outer();
		//test_hip_inner();
		//test_gravity_compensation();
		//test_spring_compensation();
		//test_ankle_current_control();
		//test_gyro_angle();
		break;
	case M0_Calibrate: //Button 0 (left-most button)	
		set_UI_LED(5, 'y');
		resetOuterLegAngle(0.0); //resets the angle integrated from gyro rate to zero in the estimator code 
		break;
	}

} // mb_controller_update()
