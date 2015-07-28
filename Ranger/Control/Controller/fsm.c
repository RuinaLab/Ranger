#include <mb_includes.h> 
#include <motorControl.h>
#include <fsm.h>

enum States {
	OUT_SWING,
	OUT_PUSH,
	INN_SWING,
	INN_PUSH,
	HOLD_DOUBLE,
};

#define ANK_FAST_KP 50
#define ANK_FAST_KD 3 
#define ANK_SLOW_KP 30
#define ANK_SLOW_KD 15 
#define HIP_KP 40
#define HIP_KD 10
#define HIP_SCISSOR_GAIN 1.5
#define HIP_REF_HOLD 0.3

static float thRef = 0.0;
static enum States current_state = OUT_SWING; 

void fsm_init(void){
	current_state = INN_SWING;
}

void fsm_run(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
	float ankInn_xRef, ankInn_vRef;
	float ankOut_xRef, ankOut_vRef;
	float hip_xRef, hip_vRef, hip_uRef;
	float c0, c1;
//	float th0, th1;	

	switch (current_state){
	case OUT_SWING:	/*swing inner leg*/
		ankInn_xRef = param_joint_ankle_flip;	//flip up inner feet
		ankInn_vRef = FI_flat_rate();
		set_ctrl_data(&ctrlAnkInn, ANK_FAST_KP, ANK_FAST_KD, ankInn_xRef, ankInn_vRef, 0.0); 
		
		ankOut_xRef = FO_flat_angle();	//hold outer feet
		ankOut_vRef = FO_flat_rate();
		set_ctrl_data(&ctrlAnkOut, ANK_SLOW_KP, ANK_SLOW_KD, ankOut_xRef, ankOut_vRef, 0.0); 
		
		c0 = HIP_SCISSOR_GAIN;
		c1 = 1.0;
		hip_scissor_track(&ctrlHip, c0, c1);
		set_ctrl_data(&ctrlHip, HIP_KP, HIP_KD, ctrlHip.xRef, ctrlHip.vRef, ctrlHip.uRef); 
		break;
	case OUT_PUSH:	/*land inner leg*/
		ankInn_xRef = param_joint_ankle_push;	//push down inner feet
		ankInn_vRef = FI_flat_rate();
		set_ctrl_data(&ctrlAnkInn, ANK_FAST_KP, ANK_FAST_KD, ankInn_xRef, ankInn_vRef, 0.0); 
		
		ankOut_xRef = FO_flat_angle();	//hold outer feet
		ankOut_vRef = FO_flat_rate();
		set_ctrl_data(&ctrlAnkOut, ANK_FAST_KP, ANK_FAST_KD, ankOut_xRef, ankOut_vRef, 0.0); 
		
		hip_xRef = HIP_REF_HOLD;	//adjust hip angle
		hip_vRef = 0.0;
		hip_uRef = hip_gravity_compensation();
		set_ctrl_data(&ctrlHip, HIP_KP, HIP_KD, hip_xRef, hip_vRef, hip_uRef); 
		break;
	case INN_SWING:	/*swing outer leg*/
		ankInn_xRef = FI_flat_angle();	//hold inner feet
		ankInn_vRef = FI_flat_rate();
		set_ctrl_data(&ctrlAnkInn, ANK_SLOW_KP, ANK_SLOW_KD, ankInn_xRef, ankInn_vRef, 0.0); 
		
		ankOut_xRef = param_joint_ankle_flip;	//flip up outer feet
		ankOut_vRef = FO_flat_rate();
		set_ctrl_data(&ctrlAnkOut, ANK_FAST_KP, ANK_FAST_KD, ankOut_xRef, ankOut_vRef, 0.0); 
		
		c0 = 1.0;
		c1 = HIP_SCISSOR_GAIN;
		hip_scissor_track(&ctrlHip, c0, c1);
		set_ctrl_data(&ctrlHip, HIP_KP, HIP_KD, ctrlHip.xRef, ctrlHip.vRef, ctrlHip.uRef); 
		break;
	case INN_PUSH:	/*land outer leg*/
		ankInn_xRef = FI_flat_angle();	//hold inner feet
		ankInn_vRef = FI_flat_rate();
		set_ctrl_data(&ctrlAnkInn, ANK_FAST_KP, ANK_FAST_KD, ankInn_xRef, ankInn_vRef, 0.0); 
		
		ankOut_xRef = param_joint_ankle_push;	//push down outer feet
		ankOut_vRef = FO_flat_rate();
		set_ctrl_data(&ctrlAnkOut, ANK_FAST_KP, ANK_FAST_KD, ankOut_xRef, ankOut_vRef, 0.0); 
		
		hip_xRef = -HIP_REF_HOLD;	//adjust hip angle
		hip_vRef = 0.0;
		hip_uRef = hip_gravity_compensation();
		set_ctrl_data(&ctrlHip, HIP_KP, HIP_KD, hip_xRef, hip_vRef, hip_uRef); 
		break;
	case HOLD_DOUBLE:
		ankInn_xRef = FI_flat_angle();	//hold inner feet
		ankInn_vRef = FI_flat_rate();
		set_ctrl_data(&ctrlAnkInn, ANK_FAST_KP, ANK_FAST_KD, ankInn_xRef, ankInn_vRef, 0.0); 
		
		ankOut_xRef = FO_flat_angle();	//hold outer feet
		ankOut_vRef = FO_flat_rate();
		set_ctrl_data(&ctrlAnkOut, ANK_FAST_KP, ANK_FAST_KD, ankOut_xRef, ankOut_vRef, 0.0); 

		if(get_out_angle_abs()>0){
			hip_xRef = -HIP_REF_HOLD;	//outer leg is in front
		}else{
			hip_xRef = HIP_REF_HOLD;	//inner leg is in front
		}
		hip_vRef = 0.0;
		hip_uRef = hip_gravity_compensation();
		set_ctrl_data(&ctrlHip, HIP_KP, HIP_KD, hip_xRef, hip_vRef, hip_uRef); 
		break;
	default: 
		/*state doesn't exist*/
		break;
	}
	
	// run the PD controllers 
	controller_hip(&ctrlHip);
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);

	// update fsm state 
	fsm_update();
}

void fsm_update(void){
	float th0 = get_out_angle_abs();
	float dth0 = get_out_ang_rate();
	
	switch(current_state){
	case OUT_SWING:
		
		break;
	case OUT_PUSH:
		break;
	case INN_SWING:
		break;
	case INN_PUSH:
		break;
	case HOLD_DOUBLE:
		break;
	default: 
		/*state doesn't exist*/
	}
}


/* Make Ranger track a double stance. */
void hip_scissor_track(struct ControllerData * ctrlData, float c0, float c1){
	//use this equation: thRef = c0*(-qr) + c1*(qh-qr)
	float qr = get_out_angle();
	float dqr = get_out_ang_rate(); 
	ctrlData->xRef = (thRef + qr*(c0+c1)) / c1;
	ctrlData->vRef = dqr*(c0+c1)/c1;
	ctrlData->uRef = hip_gravity_compensation();
	return;
}

void set_ctrl_data(struct ControllerData * ctrlData, float KP, float KD, float x, float v, float u){
	ctrlData->kp = KP;
	ctrlData->kd = KD;
	ctrlData->xRef = x;
	ctrlData->vRef = v;
	ctrlData->uRef = u;
	return;
}

 
 /* Returns the torque needed to compensate for gravity pull on the legs. */
float hip_gravity_compensation(void){
 	float in_angle_abs = get_in_angle_abs();
	float out_angle_abs = get_out_angle_abs();
	float u = leg_m * g * leg_r;
	
	if(FI_on_ground() && !FO_on_ground()){
		//track outer, only inner feet on ground
		return u * Sin(in_angle_abs); 
	}else if(!FI_on_ground() && FO_on_ground()){
		//track inner, only outer feet on ground 	
		return -u * Sin(out_angle_abs);	
	}
	return 0.0;
}