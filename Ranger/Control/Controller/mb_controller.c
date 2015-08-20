#include <mb_includes.h>
#include <input_output.h>
#include <motorControl.h>
#include <TrajData.h>
#include <Trajectory.h>
#include <fsm.h>
#include <unit_test.h>

enum ControlMode {
	M0_Calibrate,
	M3_UnitTest,
	M4_FSM,
	M5_StandBy,
};


/*  ENTRY-POINT FUNCTION FOR ALL CONTROL CODE */
void mb_controller_update(void) {

	// Enter stand-by mode when robot first boots
	static enum ControlMode controlMode = M5_StandBy;

	// Check UI buttons to update control mode
	if (detect_UI_button_input(3)) controlMode = M3_UnitTest; 	// 4th button flip feet up
	if (detect_UI_button_input(0)) controlMode = M0_Calibrate;	 // 1st button calibrates
	if (detect_UI_button_input(4)) controlMode = M4_FSM;	 // 5th button moves Ranger
	if (detect_UI_button_input(5)) controlMode = M5_StandBy; // 6th button Stand-by, always goes last (highest priority)

	// Run the desired control mode
	switch (controlMode) {
	case M5_StandBy:
		set_UI_LED(5, 'g');
		//setPush(); //calls this for the step function in motorController
		fsm_init();	//calls this for the FSM
		test_init(); //calls this for the test FSM
		param_update(); //read parameters from LABVIEW for FSM
		disable_motors();
		angles_update();
		test_gyro_angle_init();
		break;
	case M4_FSM:
		set_UI_LED(5, 'r');
		//test_trajectory();
		//track_sin();
		//double_stance();
		//check_30();
		//test_hip();
		test_fsm();	 
		//test_foot();
		//fsm_run();
		//test_fsm_ank();
		break;
	case M3_UnitTest:
		set_UI_LED(5, 'p');
		//foot_flip();
		//hold_feet();
		//motors_off();
		//ankle_motor_test();
		//test_feet();
		//test_gravity_compensation();
		//hip_motor_test();
		//test_hip_outer();
		//test_hip_inner();
		//test_gravity_compensation();
		//test_spring_compensation();
		//test_ankle_current_control();
		test_gyro_angle();
		break;
	case M0_Calibrate:
		set_UI_LED(5, 'y');
		calibrate();
		break;
	}

} // mb_controller_update()
