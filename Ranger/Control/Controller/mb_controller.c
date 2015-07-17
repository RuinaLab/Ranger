#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>
#include <TrajData.h>
#include <Trajectory.h>

enum ControlMode {
	M0_StandBy,
	M1_Active,
	M2_TraceCurve,
	M5_Calibrate,
};


/*  ENTRY-POINT FUNCTION FOR ALL CONTROL CODE */
void mb_controller_update(void) {

	// Enter stand-by mode when robot first boots
	static enum ControlMode controlMode = M0_StandBy;

	// Check UI buttons to update control mode
	if (detect_UI_button_input(5)) controlMode = M5_Calibrate;
	if (detect_UI_button_input(2)) controlMode = M2_TraceCurve;
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
		//test_motor_control() ;
		test_freq_control();
		//test_inner_foot();
		//test_sign();
		break;
	case M2_TraceCurve:
		set_UI_LED(5, 'r');
		test_trajectory();
		//track_sin();
		break;
	case M5_Calibrate:
		set_UI_LED(5, 'y');
		calibrate();
		break;
	}

} // mb_controller_update()
