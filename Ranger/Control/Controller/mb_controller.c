#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>
#include <mb_estimator.h>
#include <unit_test.h>

/* Currespond to buttons on UI board */
enum ControlMode {
	M0_Calibrate,
	M3_UnitTest,
	M5_StandBy,
};

static const int LED_CONTACT = 4;
static const int LED_UI_FSM = 5;

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
	if (detect_UI_button_input(3)) controlMode = M3_UnitTest; 	// 4th button flip feet up
	if (detect_UI_button_input(5)) controlMode = M5_StandBy; // 6th button Stand-by, always goes last (highest priority)

	// Run the desired control mode
	switch (controlMode) {
	case M5_StandBy:	//Button 6 (right-most button)
		set_UI_LED(LED_UI_FSM, 'g');
		disable_motors();
		break;
	case M3_UnitTest: //Button 4 (third button from the right)
		set_UI_LED(LED_UI_FSM, 'p');
		runUnitTest();
		break;
	}

	// Check for stand-alone tasks:
	if (detect_UI_button_input(0)){
		set_UI_LED(LED_UI_FSM, 'y');
		resetRobotOrientation();
	}

	// Set LED for contact flags:
	switch (STATE_contactMode) {
	case CONTACT_S0:
		set_UI_LED(LED_CONTACT, 'r');
		break;
	case CONTACT_S1:
		set_UI_LED(LED_CONTACT, 'b');
		break;
	case CONTACT_DS:
		set_UI_LED(LED_CONTACT, 'p');
		break;
	}

} // mb_controller_update()
