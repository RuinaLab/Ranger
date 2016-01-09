#ifndef __MB_CONTROLLER_H__
#define __MB_CONTROLLER_H__

/* Currespond to buttons on UI board */
typedef enum {
	UnitTest,
	WalkCtrl,
	StandBy
} UiFsmMode;

extern UiFsmMode UI_FSM_MODE;  // What to run now

/* Name the LEDs */
extern const int LED_WALK_FSM; // Top right
extern const int LED_CONTACT;  // middle left
extern const int LED_UI_FSM;  // Top left
extern const int LED_GAIT_FSM; // middle right
extern const int LED_DEBUG;  // bottom right.  Used for debugging code. Should be inactive during normal operation.
extern bool FSM_LED_FLAG;

extern char walkLedColor; // Indicates what sort of walking is happening.

/* Name the UI buttons.
 * button 0 is the left-most button,
 * button 5 is the right-most button    */
extern const int BUTTON_ACCEPT_TRIAL;// = 0;  // Accepts a trial (updates the controller)
extern const int BUTTON_REJECT_TRIAL;  // Reject a trial (environment caused failure)
extern const int BUTTON_UNIT_TEST;// = 3;
extern const int BUTTON_WALK_CONTROL;// = 4;
extern const int BUTTON_STAND_BY;// = 5;

// Entry-point function for all controller stuff
void mb_controller_update(void);

#endif  // __MB_CONTROLLER_H__
