#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>

enum ControlMode {
	M0_StandBy,
	M1_Active
};


/*  ENTRY-POINT FUNCTION FOR ALL CONTROL CODE */
void mb_controller_update(void) {

	// Enter stand-by mode when robot first boots
	static enum ControlMode controlMode = M0_StandBy;

	// Check UI buttons to update control mode
	if (detect_UI_button_input(1)) controlMode = M1_Active;
	if (detect_UI_button_input(0)) controlMode = M0_StandBy; // Stand-by always goes last (highest priority)

	// Run the desired control mode
	switch (controlMode) {
	case M0_StandBy:
		set_UI_LED(5, 'g');
		disable_motors();
		break;
	case M1_Active:
		set_UI_LED(5, 'b');
		test_motor_control() ;
		break;
	}

} // mb_controller_update()
