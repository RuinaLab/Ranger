//============================================================================
// Name        : main.cpp
// Author      : Matthew Kelly
//
// Demonstrate how to use the motor model for Ranger, and check a few test cases
//
//============================================================================

#include <iostream>
#include <cmath>


using namespace std;


class MotorModel {
public:
	static const float R = 1.3; // (Ohms) terminal resistance
	static const float Vc = 0.7; // (Volts) contact voltage drop
	static const float K = 0.018; // (Nm/A) torque constant
	static const float c1 = 0.0; // (Nms/rad) viscous friction
	static const float c0 = 0.01; // (Nm) const friction
	static const float mu = 0.1; // (1) current-dep const friction
	static const float Jm = 1.6e-6; // (kg-m^2) motor inertia
	static const float G = 66; // (1) gearbox ratio (66 = hip, 34 = ankle)

	static float getCurrent(float, float, float); // expectedCurrent = getCurrent(rate,accel,desiredTorque)
	static float getTorque(float, float, float); // expectedTorque = getTorque(rate,accel,commandCurrent)
};


float MotorModel::getCurrent(float omega, float alpha, float torque) {

	// Tf = c1*omega + c0*sign(omega) + mu*G*K*abs(I)*sign(omega);  // Frictional Torque:
	// T = G*(K*I - Jm*G*alpha) - Tf; % Torque out of gear box:

	float direction = 0.0;
	if (omega > 0.0) {
		direction = 1.0;
	} else if (omega < 0.0) {
		direction = -1.0;
	}

	float num = torque + G * Jm * G * alpha + c1 * omega + c0 * direction;
	float denPos = (G * K - mu * G * K * direction); // Assuming that current > 0
	float denNeg = (G * K + mu * G * K * direction); // Assuming that current <= 0

	float Ipos = num / denPos; // %Current, assuming that current > 0
	float Ineg = num / denNeg; // %Current, assuming that current <= 0

	if (Ipos > 0 && Ineg > 0) {
		return Ipos; // Then I > 0 assumption was correct!
	} else if (Ipos <= 0 && Ineg <= 0) {
		return Ineg;  // Then I <= 0 assumption was correct!
	} else {
		// Should never reach here
		return num / (G * K); // Just drop the coulomb friction term
	}

}


float MotorModel::getTorque(float omega, float alpha, float current) {

	// Tf = c1*omega + c0*sign(omega) + mu*G*K*abs(I)*sign(omega);  // Frictional Torque:
	// T = G*(K*I - Jm*G*alpha) - Tf; % Torque out of gear box:

	float direction = 0.0;
	if (omega > 0.0) {
		direction = 1.0;
	} else if (omega < 0.0) {
		direction = -1.0;
	}

	// Compute the expected frictional losses
	float friction = c1 * omega + c0 * direction + mu * G * K * abs(current) * direction;

	// Compute expected torque
	return G * (K * current - Jm * G * alpha) - friction;

}


int main( int argc, const char ** argv ) {

	MotorModel mm;

	float omega = 1.2;
	float alpha = 0.1;
	float torque = 2.5;

	float current = mm.getCurrent(omega, alpha, torque);

	cout << "Motor Current: " << current << "\n";

	float checkTorque = mm.getTorque(omega, alpha, current);

	cout << "Torque Error: " << (checkTorque - torque) << "\n";

	return 0;

}