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


/* Sends commands to the motors based on the current state of the
 * walking finite state machine */
void sendMotorCommands(void) {

	/* Read controller parameters directly from labview for now */
	float push = LABVIEW_WALK_ANK_PUSH;  //magnitude of the push-off during walking, normalized to be on the range [0,1]
	float rate = LABVIEW_WALK_HIP_RATE;  //scissor tracking rate, should be near one (~0.5, ~1.5)
	float offset = LABVIEW_WALK_HIP_OFFSET;  //How much the swing leg should lead the stance leg during scissor tracking
	float angle = LABVIEW_CTRL_WALK_HIP_ANGLE;  //Target angle for the hip to hold during push-off

	switch (WALK_FSM_MODE_PREV) {
	case Glide_Out:
		holdStance_ankOut();
		flipUp_ankInn();
		hipGlide(rate, offset);
		break;
	case Push_Out:
		flipDown_ankInn();
		pushOff_ankOut(push);
		hipHold(angle);
		break;
	case Glide_Inn:
		flipUp_ankOut();
		holdStance_ankInn();
		hipGlide(rate, offset);
		break;
	case Push_Inn:
		flipDown_ankOut();
		pushOff_ankInn(push);
		hipHold(angle);
		break;
	case Flight:  // In the air, get ready for Glide_Out
		holdStance_ankOut();
		flipUp_ankInn();
		disable_hip();
		break;
	}

}


/*******************************************************************************
 *                      TESTING FUNCTIONS                                      *
 *******************************************************************************/


/* Hold both feet in stance and disable the hip. Run the walking finite state
 * machine, and check transitions based on LED sequence.  */
void test_walkFsmTransitions(void) {
	updateWalkFsm();
	setWalkFsmLed();
	disable_hip();
	holdStance_ankOut();
	holdStance_ankInn();
}


/* This function is used for testing various aspects of the walking code,
 * designed to be called by unitTest     */
void walkControl_test(void) {

	/* Simple test for manually stepping through the finite state machine */
	test_walkFsmTransitions();
}



/*******************************************************************************
 *                     MAIN ENTRY-POINT FUNCTIONS                              *
 *******************************************************************************/

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
	sendMotorCommands();
}

