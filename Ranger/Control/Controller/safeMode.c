#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <motorControl.h>
#include <robotParameters.h>
#include "safeMode.h"

static const float KP_ANKLE = 2.0;
static const float KP_HIP = 3.0;

static const float KD_ANKLE = 1.0;
static const float KD_HIP = 1.0;

static float Q0_TARGET; 
static float Q1_TARGET; 
static float QH_TARGET;

static bool FLAG_INIT = false;

/* Called when entering safe mode. It sets the targets to match the current targets.
 *
 * ALWAYS CALL THIS FIRST WHEN STARTING SAFE MODE 
 * 													*/
void setSafeModeConfig(void) {
	Q0_TARGET = STATE_q0;  // copy state from estimator
	Q1_TARGET = STATE_q1;  // copy state from estimator
	QH_TARGET = STATE_qh;  // copy state from estimator
}

/* Call on each tick to run in safe mode. Make sure that setSafeModeConfig
 * Was called at the transition into safe mode. */
void safeMode_main(void) {
	if (!FLAG_INIT){
		setSafeModeConfig();
		FLAG_INIT = true;
	}
	trackRel_ankOut(Q0_TARGET, KP_ANKLE, KD_ANKLE);  // outer ankles track desired joint angle
	trackRel_ankInn(Q1_TARGET, KP_ANKLE, KD_ANKLE);  // inner ankles track desired joint angle
	trackRel_hip(QH_TARGET, KP_HIP, KD_HIP);  // Hip motor tracks a desired hip joint angle
}
