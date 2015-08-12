#include <mb_includes.h> 
#include <motorControl.h>
#include <fsm.h>
#include <unit_test.h>


// Hold the feet while making the hip angle track zero 
void hold_feet(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
	float hold = mb_io_get_float(ID_CTRL_ANK_REF_HOLD);
	float hold_kp = mb_io_get_float(ID_CTRL_ANK_HOLD_KP);
	float hold_kd = mb_io_get_float(ID_CTRL_ANK_HOLD_KD);
	 
	//update all the angle parameters
	angles_update();	 

	// hold outer feet
	out_ank_track_abs(&ctrlAnkOut, hold, 0.0, 0.0, hold_kp, hold_kd);
	// hold inner feet
	inn_ank_track_abs(&ctrlAnkInn, hold, 0.0, 0.0, hold_kp, hold_kd);
	hip_track_rel(&ctrlHip, 0.0, 0.0, mb_io_get_float(ID_CTRL_HIP_KP), mb_io_get_float(ID_CTRL_HIP_KD));

	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
	controller_hip(&ctrlHip);
}
