//============================================================================
// Name        : main.c
// Author      : Matthew Kelly
//
// Test leg angle calculation
//
//============================================================================

#include <stdio.h>  // file stuff
#include <math.h>  // sin() atan() cos() sqrt()

int main( int argc, const char ** argv ) {

	float l = 0.96;  // robot leg length
	float d = 0.14;  // robot foot joint eccentricity
	float Phi = 1.8;  //  ankle joint orientation constant

	float Slope = 0.0;  // Ground slope (assume linear)

	float qh = 0.3;  // robot hip angle
	float q0 = 1.95; // outer ankle joint angle
	float q1 = 1.65;  // inner ankle joint angle

	/* Ranger geometry:
	 * [x;y] = vector from outer foot virtual center to the inner foot
	 * virtual center, in a frame that is rotated such that qr = 0
	 * These functions were determined using computer math. The code can
	 * be found in:
	 * templates/Estimator/legAngleEstimator/Derive_Eqns.m
	 */
	float x = l*sin(qh) - d*sin(Phi - q1 + qh) + d*sin(Phi - q0);
	float y = l + d*cos(Phi - q1 + qh) - l*cos(qh) - d*cos(Phi - q0);

	float stepLength = sqrt(x*x + y*y);
	float qr = atan(y/x) + Slope;

	printf("Robot angle: %3.3f \n",qr);
	printf("Step Length: %3.3f \n",stepLength);

	return 0;
}


