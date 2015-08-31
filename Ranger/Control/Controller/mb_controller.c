#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>
#include <mb_estimator.h>
#include <unit_test.h>
#include <walkControl.h>

/* Currespond to buttons on UI board */
typedef enum {
	UnitTest,
	WalkCtrl,
	StandBy
} UiFsmMode;

static UiFsmMode UI_FSM_MODE = StandBy;  // What to run now
static UiFsmMode UI_FSM_MODE_PREV = StandBy;  // What we ran last time


/* Name the LEDs */
const int LED_WALK_FSM = 1;
const int LED_CONTACT = 4;
const int LED_UI_FSM = 5;

/* Name the UI buttons.
 * button 0 is the left-most button,
 * button 5 is the right-most button    */
const int BUTTON_CALIBRATE_GYRO = 0;
const int BUTTON_UNIT_TEST = 3;
const int BUTTON_WALK_CONTROL = 4;
const int BUTTON_STAND_BY = 5;

/* Checks the buttons on the UI board for high-level
 * commands to change the mode of the user-interface
 * finite-state-machine.        */
void update_ui_fsm_state(void) {

	UI_FSM_MODE_PREV = UI_FSM_MODE;

	// Check UI buttons to update control mode
	if (detect_UI_button_input(BUTTON_UNIT_TEST)) {
		UI_FSM_MODE = UnitTest; 	// Run unit test
	}
	if (detect_UI_button_input(BUTTON_WALK_CONTROL)) {
		UI_FSM_MODE = WalkCtrl;   // Start the walking controller
	}
	if (detect_UI_button_input(BUTTON_STAND_BY)) {
		UI_FSM_MODE = StandBy;    // go to stand-by
	}

}


/*  ENTRY-POINT FUNCTION FOR ALL CONTROL CODE */
void mb_controller_update(void) {

	// Checks the buttons to see if state should change
	update_ui_fsm_state();

	// Run the desired control mode  (See top of file for which buttons to use)
	switch (UI_FSM_MODE) {
	case StandBy:
		set_UI_LED(LED_UI_FSM, 'g');
		disable_motors();
		break;
	case UnitTest:
		set_UI_LED(LED_UI_FSM, 'p');
		runUnitTest();
		break;
	case WalkCtrl:
		set_UI_LED(LED_UI_FSM, 'b');
		if (UI_FSM_MODE_PREV != WalkCtrl) {
			walkControl_entry();  // Run the initialization function for the walking FSM
		}
		walkControl_main();  // Run the main walk function
	}

	// Calibrate the rate gyro if the left-most button is pressed:
	if (detect_UI_button_input(BUTTON_CALIBRATE_GYRO)) {
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
