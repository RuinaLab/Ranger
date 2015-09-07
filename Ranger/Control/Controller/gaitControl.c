#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <gaitControl.h>
#include <gaitControlData.h>  

typedef enum {
	PreMid_Out,   // Outer feet on the ground, inner feet swing through with scissor gait
	PostMid_Out,    // Outer feet on the ground,
	PreMid_Inn,  // Inner feet on the ground, inner feet swing through with scissor gait
	PostMid_Inn
} MidStanceFsmMode;

static MidStanceFsmMode MIDSTANCE_FSM_MODE = PreMid_Out;

/* Parameters that are updated once per step and read by the estimator,
 * which then passes them to the walking controller. It is initialized 
 * to the raw data for the system to start walking from mid-stance.  */
float GAIT_WALK_ANK_PUSH = GAITDATA_WALK_ANK_PUSH[0];
float GAIT_WALK_CRIT_STANCE_ANGLE = GAITDATA_WALK_CRIT_STANCE_ANGLE[0];
float GAIT_WALK_HIP_STEP_ANGLE = GAITDATA_WALK_HIP_STEP_ANGLE[0];
float GAIT_WALK_SCISSOR_GAIN = GAITDATA_WALK_SCISSOR_GAIN[0];
float GAIT_WALK_SCISSOR_OFFSET = GAITDATA_WALK_SCISSOR_OFFSET[0];


/* This function is called once per walking step at mid-stance
 * It computes the new set of gait data that is used by the 
 * walking controller */
void updateGaitData(float dth){

////TODO////

}


/* This function is called once per loop, and checks the
 * sensors that are used to trigger transitions between
 * modes of the finite state machine */
void updateMidStanceFsm(void) {

	if (STATE_contactMode == CONTACT_FL) { // Then robot is in the air
		MIDSTANCE_FSM_MODE = PreMid_Out;  // Reset the finite state machine to initial state
	} else {  // Run the normal mid-stance finite state machine
		switch (WALK_FSM_MODE_PREV) {
		case PreMid_Out:
			if (STATE_th0 < 0.0) {
				MIDSTANCE_FSM_MODE = PostMid_Out;
			} break;
		case PostMid_Out:
			if (STATE_c1) { // Inner feet hit ground
				MIDSTANCE_FSM_MODE = PreMid_Inn;
			} break;
		case PreMid_Inn:
			if (STATE_th1 < 0.0) {
				MIDSTANCE_FSM_MODE = PostMid_Inn;
			} break;
		case PostMid_Inn:
			if (STATE_c0) { // Inner feet hit ground
				MIDSTANCE_FSM_MODE = PreMid_Out;
			} break;
		}
	}
}



