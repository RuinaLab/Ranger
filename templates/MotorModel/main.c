//============================================================================
// Name        : main.c
// Author      : Matthew Kelly
//
// Demonstrate how to use the motor model for Ranger
//
//============================================================================

#include <stdio.h>

struct MotorParam {
	float R; // (Ohms) terminal resistance
	float Vc; // (Volts) contact voltage drop
	float K; // (Nm/A) torque constant
	float c1; // (Nms/rad) viscous friction
	float c0; // (Nm) const friction
	float mu; // (1) current-dep const friction
	float Jm; // (kg-m^2) motor inertia
	float G_hip; // (1) gearbox ratio (66 = hip, 34 = ankle)
	float G_ank; // (1) gearbox ratio ()
};

/*  MotorModel_Current computes the current that should be required to produce
 * 	the desired torque, given the state of the motor.
 * 	@param omega = joint angular rate
 * 	@param alpha = joint angular acceleration
 * 	@param torque = desired joint torque
 */
float MotorModel_Current(float omega, float alpha, float torque,
                         struct MotorParam mp, float G) {
	// Tf = c1*omega + c0*sign(omega) + mu*G*K*abs(I)*sign(omega);
	// T = G*(K*I - Jm*G*alpha) - Tf; % Torque out of gear box:

	float direction = 0.0;
	if (omega > 0.0) {
		direction = 1.0;
	} else if (omega < 0.0) {
		direction = -1.0;
	}

	float num = torque + G * mp.Jm * G * alpha +
	            mp.c1 * omega + mp.c0 * direction;

	/* Need to invert an abs() function... denPos assumes that the current is
	 * positive, denNeg does the opposite. Later we check the sign of the
	 * current to know which assumption was good.
	 */
	float denPos = (G * mp.K - mp.mu * G * mp.K * direction);
	float denNeg = (G * mp.K + mp.mu * G * mp.K * direction);

	float Ipos = num / denPos; // %Current, assuming that current > 0
	float Ineg = num / denNeg; // %Current, assuming that current <= 0

	if (Ipos > 0 && Ineg > 0) {
		return Ipos; // Then I > 0 assumption was correct!
	} else if (Ipos <= 0 && Ineg <= 0) {
		return Ineg;  // Then I <= 0 assumption was correct!
	} else {
		// Should never reach here
		return num / (G * mp.K); // Just drop the coulomb friction term
	}
}

float MotorModel_HipCurrent(float omega, float alpha, float torque,
                            struct MotorParam mp) {
	float G = mp.G_hip;
	return MotorModel_Current(omega, alpha, torque, mp, G);
}

float MotorModel_AnkCurrent(float omega, float alpha, float torque,
                            struct MotorParam mp) {
	float G = mp.G_ank;
	return MotorModel_Current(omega, alpha, torque, mp, G);
}

float MotorModel_Torque(float omega, float alpha, float current,
                        struct MotorParam mp, float G) {

	// Tf = c1*omega + c0*sign(omega) + mu*G*K*abs(I)*sign(omega);
	// T = G*(K*I - Jm*G*alpha) - Tf; % Torque out of gear box:

	float direction = 0.0;
	if (omega > 0.0) {
		direction = 1.0;
	} else if (omega < 0.0) {
		direction = -1.0;
	}

	float absCurrent = current;
	if (current < 0) {
		absCurrent = -absCurrent;
	}

	// Compute the expected frictional losses
	float friction = mp.c1 * omega + mp.c0 * direction + 
	mp.mu * G * mp.K * absCurrent * direction;

	// Compute expected torque
	return G * (mp.K * current - mp.Jm * G * alpha) - friction;

}


float MotorModel_HipTorque(float omega, float alpha, float current,
                           struct MotorParam mp) {
	float G = mp.G_hip;
	return MotorModel_Torque(omega, alpha, current, mp, G);
}

float MotorModel_AnkTorque(float omega, float alpha, float current,
                           struct MotorParam mp) {
	float G = mp.G_ank;
	return MotorModel_Torque(omega, alpha, current, mp, G);
}


int main( int argc, const char ** argv ) {

	struct MotorParam mp;
	mp.R = 1.3; // (Ohms) terminal resistance
	mp.Vc = 0.7; // (Volts) contact voltage drop
	mp.K = 0.018; // (Nm/A) torque constant
	mp.c1 = 0.0; // (Nms/rad) viscous friction
	mp.c0 = 0.01; // (Nm) const friction
	mp.mu = 0.1; // (1) current-dep const friction
	mp.Jm = 1.6e-6; // (kg-m^2) motor inertia
	mp.G_hip = 66; // (1) gearbox ratio (66 = hip, 34 = ankle)
	mp.G_ank = 34; // (1) gearbox ratio (66 = hip, 34 = ankle)

	float omega = 1.2;
	float alpha = 0.1;
	float torque = 2.5;

	float currentAnk = MotorModel_AnkCurrent(omega, alpha, torque, mp);
	float currentHip = MotorModel_HipCurrent(omega, alpha, torque, mp);

	printf("Hip Motor Current: %f \n", currentHip);
	printf("Ankle Motor Current: %f \n", currentAnk);

	float checkTorqueAnk = MotorModel_AnkTorque(omega, alpha, currentAnk, mp);
	float checkTorqueHip = MotorModel_HipTorque(omega, alpha, currentHip, mp);

	printf("Hip Motor Torque Error: %f \n", checkTorqueAnk - torque);
	printf("Ankle Motor Torque Error: %f \n", checkTorqueHip - torque);

	return 0;

}