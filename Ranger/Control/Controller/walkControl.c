#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <motorControl.h>
#include <walkControl.h>

typedef enum {
	Glide_Out,   // Outer feet on the ground, inner feet swing through with scissor gait
	Push_Out,    // Outer feet on the ground,
	Glide_Inn,  // Inner feet on the ground, inner feet swing through with scissor gait
	Push_Inn,
	Flight
} WalkFsmMode;

static WalkFsmMode WALK_FSM_MODE = Glide_Out;  // What to run now
static WalkFsmMode WALK_FSM_MODE_PREV = Glide_Out;  // What we ran last time


/* This function is called once per loop, and checks the
 * sensors that are used to trigger transitions between
 * modes of the finite state machine */
void updateWalkFsm(void) {
	WALK_FSM_MODE_PREV = WALK_FSM_MODE;

	if (STATE_contactMode == CONTACT_FL) { // Then robot is in the air
		WALK_FSM_MODE = Flight;  // Give up on walking and enter flight mode
	} else {  // Run the normal walking finite state machine
		switch (WALK_FSM_MODE_PREV) {
		case Glide_Out:
			if (STATE_th0 < -LABVIEW_FSM_CRIT_STANCE_ANGLE) {
				WALK_FSM_MODE = Push_Out;
			} break;
		case Push_Out:
			if (STATE_c1) {  // If inner feet hit the ground
				WALK_FSM_MODE = Glide_Inn;
			} break;
		case Glide_Inn:
			if (STATE_th1 < -LABVIEW_FSM_CRIT_STANCE_ANGLE) {
				WALK_FSM_MODE = Push_Inn; 
			} break;
		case Push_Inn:
			if (STATE_c0) {  // If outer feet hit the ground
				WALK_FSM_MODE = Glide_Out;
			} break;
		case Flight:
			if (STATE_c0) {  // If outer feet hit the ground
				WALK_FSM_MODE = Glide_Out;
			} else if (STATE_c1) { // If inner feet hit the ground
				WALK_FSM_MODE = Glide_Inn;
			} break;
		}
	}
}


/* Turns on a specific led for each state of the walking FSM.
 * No LED indicates that the mode is flight. */
void setWalkFsmLed(void) {
	switch (WALK_FSM_MODE_PREV) {
	case Glide_Out:
		set_UI_LED(LED_WALK_FSM , 'r');
		break;
	case Push_Out:
		set_UI_LED(LED_WALK_FSM , 'o');
		break;
	case Glide_Inn:
		set_UI_LED(LED_WALK_FSM , 'b');
		break;
	case Push_Inn:
		set_UI_LED(LED_WALK_FSM , 'p');
		break;
	}
}


/* This function is called once, as soon as the button is pressed
 * for the robot to begin walking. It is used for initialization. */
void walkControl_entry(void) {

	disable_motors(); // Clear old motor commands;

	// Always start with the outer feet in stance, and the inner feet tracking a scissor gait
	WALK_FSM_MODE = Glide_Out;
	WALK_FSM_MODE_PREV = Glide_Out;

}


/* This is the main function for walking. All walking controls
 * are called from here */
void walkControl_main(void) {
	updateWalkFsm();
	setWalkFsmLed();

	// Dummy Test for now:
	holdStance_ankOut();
	holdStance_ankInn();
}
