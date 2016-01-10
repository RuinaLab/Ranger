#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>
#include <mb_estimator.h>
#include <unit_test.h>
#include <walkControl.h>
#include <gaitControl.h>
#include <optimizeGait.h>
#include "safeMode.h"

UiFsmMode UI_FSM_MODE = StandBy;  // What to run now
UiFsmMode UI_FSM_MODE_PREV = StandBy;  // What we ran last time

/* Name the LEDs.   
 * 0 --> "do nothing" 
 * 1 --> Top Right
 * 2 --> Middle Right
 * 3 --> Bottom Right
 * 4 --> Middle Left
 * 5 --> Top Left
 * 6 --> Bottom Right  (DEAD) 
 */
const int LED_OPTIMIZE = 1; 
const int LED_WALK_FSM = 0;  
const int LED_CONTACT = 4;  
const int LED_UI_FSM = 5;  
const int LED_GAIT_FSM = 2; 
const int LED_DEBUG = 3; 
bool FSM_LED_FLAG = false;  // Show lights for FSM?

char walkLedColor = 'b';

/* Name the UI buttons.
 * button 0 is the left-most button,
 * button 5 is the right-most button    */
const int BUTTON_ACCEPT_TRIAL = 0;  // Accepts a trial (updates the controller)
const int BUTTON_REJECT_TRIAL = 1;  // Reject a trial (environment caused failure)
const int BUTTON_UNIT_TEST = 3;
const int BUTTON_WALK_CONTROL = 4;
const int BUTTON_STAND_BY = 5;

/* Checks the buttons on the UI board for high-level
 * commands to change the mode of the user-interface
 * finite-state-machine. Note that options are listed
 * by priority (because of return statements)       */
void update_ui_fsm_state(void) {

	UI_FSM_MODE_PREV = UI_FSM_MODE;

	// Check UI buttons to update control mode
	if (STATE_IS_FALLEN || detect_UI_button_input(BUTTON_STAND_BY)) {
		UI_FSM_MODE = StandBy;    // go to stand-by
		return;
	}
	if (detect_UI_button_input(BUTTON_WALK_CONTROL)) {
		UI_FSM_MODE = WalkCtrl;   // Start the walking controller
		return;
	}
	if (detect_UI_button_input(BUTTON_UNIT_TEST)) {
		UI_FSM_MODE = UnitTest; 	// Run unit test
		return;
	}

}


/* Transition to robot to Safe Mode */
void enterSafeMode(void){
	UI_FSM_MODE = SafeMode;
	setSafeModeConfig();  // Set the target angles for safe mode
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
			gaitControl_entry();
			walkControl_entry();  // Run the initialization function for the walking FSM
		}
		gaitControl_main();  // High level - set gains, ect.
		walkControl_main();  // Run the main walk function
		break;
	case SafeMode:
		set_UI_LED(LED_UI_FSM, 'r');
		safeMode_main();
		break;
	}

	if (FSM_LED_FLAG) {
		// Set LED for contact flags
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
	}

} // mb_controller_update()
