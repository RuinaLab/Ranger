#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>
#include <TrajData.h>
#include <Trajectory.h>
#include <fsm.h>

enum ControlMode {
	M0_StandBy,
	M1_Active,
	M2_TraceCurve,
	M3_FlipFeet,
	M4_FSM,
	M5_Calibrate,
};


/*  ENTRY-POINT FUNCTION FOR ALL CONTROL CODE */
void mb_controller_update(void) {

	// Enter stand-by mode when robot first boots
	static enum ControlMode controlMode = M0_StandBy;

	// Check UI buttons to update control mode
	if (detect_UI_button_input(3)) controlMode = M3_FlipFeet; 	// 4th button flip feet up
	if (detect_UI_button_input(0)) controlMode = M5_Calibrate;	 // 1st button calibrates
	if (detect_UI_button_input(4)) controlMode = M2_TraceCurve;	 // 5th button moves Ranger
	if (detect_UI_button_input(1)) controlMode = M1_Active;
	if (detect_UI_button_input(2)) controlMode = M4_FSM;
	if (detect_UI_button_input(5)) controlMode = M0_StandBy; // 6th button Stand-by, always goes last (highest priority)

	// Run the desired control mode
	switch (controlMode) {
	case M0_StandBy:
		set_UI_LED(5, 'g');
		//setPush(); //calls this for the step function in motorController
		fsm_init();	//calls this for the FSM
		test_init(); //calls this for the test FSM
		param_update(); //read parameters from LABVIEW for FSM
		disable_motors();
		break;
	case M1_Active:
		set_UI_LED(5, 'b');
		//test_motor_control() ;
		//test_freq_control();
		//test_inner_foot();
		//test_sign();
		//step();
		break;
	case M2_TraceCurve:
		set_UI_LED(5, 'r');
		//test_trajectory();
		//track_sin();
		//double_stance();
		//check_30();
		test_fsm_hip();
		//test_hip();
		break;
	case M3_FlipFeet:
		set_UI_LED(5, 'p');
		//foot_flip();
		test_foot();
		break;
	case M4_FSM:
		set_UI_LED(5, 'c');
		//test_foot();
		//fsm_run();
		//test_fsm();		
		break;
	case M5_Calibrate:
		set_UI_LED(5, 'y');
		calibrate();
		break;
	}

} // mb_controller_update()
