//============================================================================
// Name        : main.c
// Author      : Matthew Kelly
//
// Test estimator functions
//
//============================================================================


#include "stdafx.h"
#include <stdio.h>  // file stuff
#include <stdlib.h>  // rand()
#include <math.h>  // sin() tan()
#include "estimator.h"

float time[] = {0, 0.01, 0.03, 0.06};
/*float time[] = {0,0.01,0.03,0.04,0.06,0.07,0.09,0.10,0.12,0.15,0.16,0.19,0.21,0.22,0.25,0.26,0.3,0.32,0.33,0.35,0.37,0.38,0.41,0.43,0.45,
				0.46,0.48,0.51,0.53,0.55,0.58,0.6,0.63,0.65,0.67,0.69,0.72,0.75,0.77,0.8,0.81,0.83,0.86,0.88,0.91,0.93,0.94,0.96,0.99,
				1.01,1.03,1.04,1.06,1.07,1.09,1.10,1.12,1.15,1.16,1.19,1.21,1.22,1.25,1.26,1.3,1.32,1.33,1.35,1.37,1.38,1.41,1.43,1.45,
				1.46,1.48,1.51,1.53,1.55,1.58,1.60,1.63,1.65,1.67,1.69,1.72,1.75,1.77,1.80,1.81,1.83,1.86,1.88,1.91,1.93,1.94,1.96,1.99,
				2.01,2.03,2.04,2.06,2.07,2.09,2.10,2.12,2.15,2.16,2.19,2.21,2.22,2.25,2.26,2.30,2.32,2.33,2.35,2.37,2.38,2.41,2.43,2.45,
				2.46,2.48,2.51,2.53,2.55,2.58,2.60,2.63,2.65,2.67,2.69,2.72,2.75,2.77,2.80,2.81,2.83,2.86,2.88,2.91,2.93,2.94,2.96,2.99,
				3.01,3.03,3.04,3.06,3.07,3.09,3.10,3.12,3.15,3.16,3.19,3.21,3.22,3.25,3.26,3.30,3.32,3.33,3.35,3.37,3.38,3.41,3.43,3.45,
				3.46,3.48,3.51,3.53,3.55,3.58,3.60,3.63,3.65,3.67,3.69,3.72,3.75,3.77,3.80,3.81,3.83,3.86,3.88,3.91,3.93,3.94,3.96,3.99
				}; 
*/

/* Generates a random floating point number on the domain (-amp,amp)
 */
float getRand(float amp) {
	return amp * (1.0 - 2.0 * ((float) rand()) / ((float) RAND_MAX));
}

//int main( int argc, const char ** argv ) {
int _tmain(int argc, _TCHAR* argv[]){
	float t; // Sensor time
	float z; // Sensor data
	float y; // Filtered estimate
	float dt = 0.01;  // time step
	float noise = 0.05; // noise amplitude
	int nSample = 400;  // run this many time steps
	int i; // loop iterator

	// Structs for the filter coefficients and data
	float cutoff_frequency; // min 0.001 < fc < 0.999 max
	struct FilterCoeff FC;
	struct FilterData FD;

	// The filter data is written to a matlab script for plotting:
	FILE *out;
	out = fopen("TestScript.m", "w");

	// Run the complicated filter
	// Set up the filter:
	cutoff_frequency = 0.06;
	setFilterData(&FD, 0.0);
	setFilterCoeff(&FC, cutoff_frequency);
	// Run the test:
	fprintf(out, "TestOne = [...\n");
	int size = sizeof(time)/sizeof(time[0]);
	printf("size = %d\n", size);
	for (i = 0; i < size; i++){
		t = time[i];
		z = sin(t) + getRand(noise);
		y = runFilter_new(&FC, &FD, z, t);
		fprintf(out, "%6.6f %6.6f %6.6f ;\n", t, z, y);
	}
	fprintf(out, "];\n");
	fprintf(out, "figure(1); clf; hold on;\n");
	fprintf(out, "plot(TestOne(:,1),TestOne(:,3),'r-')\n");
	fprintf(out, "plot(TestOne(:,1),TestOne(:,2),'k.')\n\n");
	fprintf(out, "title('fc = %6.3f')\n", cutoff_frequency);



	// Run the simple filter
	// Set up the filter:
	cutoff_frequency = 0.06;
	setFilterData(&FD, 0.0);
	setFilterCoeff(&FC, cutoff_frequency);
	// Run the test:
	t = 0.0;
	fprintf(out, "TestTwo = [...\n");
	for (i = 0; i < nSample; i++) {
		z = sin(t) + getRand(noise);
		y = runFilter(&FC, &FD, z);
		fprintf(out, "%6.6f %6.6f %6.6f ;\n", t, z, y);
		t = t + dt;
	}
	fprintf(out, "];\n");
	fprintf(out, "figure(2); clf; hold on;\n");
	fprintf(out, "plot(TestTwo(:,1),TestTwo(:,3),'r-')\n");
	fprintf(out, "plot(TestTwo(:,1),TestTwo(:,2),'k.')\n\n");
	fprintf(out, "title('fc = %6.3f')\n", cutoff_frequency);

	fclose(out);
	while(1);
	return 0;
}


