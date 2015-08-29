#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>

/* Currespond to buttons on UI board */
enum ControlMode {
	M0_Calibrate,
	M3_UnitTest,
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
	if (detect_UI_button_input(5)) controlMode = M5_StandBy; // 6th button Stand-by, always goes last (highest priority)

	// Run the desired control mode
	switch (controlMode) {
	case M5_StandBy:	//Button 6 (right-most button)
		set_UI_LED(5, 'g');
		disable_motors();
		break;
	case M3_UnitTest: //Button 4 (third button from the right)
		set_UI_LED(5, 'p');
		break;
	case M0_Calibrate: //Button 0 (left-most button)	
		set_UI_LED(5, 'y');
		resetOuterLegAngle(0.0); //resets the angle integrated from gyro rate to zero in the estimator code 
		break;
	}

} // mb_controller_update()
