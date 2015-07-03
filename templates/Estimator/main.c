//============================================================================
// Name        : main.c
// Author      : Matthew Kelly
//
// Test estimator functions
//
//============================================================================

#include <stdio.h>  // file stuff
#include <stdlib.h>  // rand()
#include <math.h>  // sin() tan()

#include "estimator.h"

/* Generates a random floating point number on the domain (-amp,amp)
 */
float getRand(float amp) {
	return amp * (1.0 - 2.0 * ((float) rand()) / ((float) RAND_MAX));
}

int main( int argc, const char ** argv ) {

	float t; // Sensor time
	float z; // Sensor data
	float y; // Filtered estimate
	float dt = 0.01;  // time step
	float noise = 0.05; // noise amplitude
	int nSample = 500;  // run this many time steps
	int i; // loop iterator

	// Structs for the filter coefficients and data
	float cutoff_frequency; // min 0.001 < fc < 0.999 max
	struct FilterCoeff FC;
	struct FilterData FD;

	// The filter data is written to a matlab script for plotting:
	FILE *out;
	out = fopen("TestScript.m", "w");


	// Set up the filter:
	cutoff_frequency = 0.03;
	setFilterData(&FD, 0.0);
	setFilterCoeff(&FC, cutoff_frequency);
	// Run the test:
	t = 0.0;
	fprintf(out, "TestOne = [...\n");
	for (i = 0; i < nSample; i++) {
		z = sin(t) + getRand(noise);
		y = runFilter(&FC, &FD, z);
		fprintf(out, "%6.6f %6.6f %6.6f ;\n", t, z, y);
		t = t + dt;
	}
	fprintf(out, "];\n");
	fprintf(out, "figure(1); clf; hold on;\n");
	fprintf(out, "plot(TestOne(:,1),TestOne(:,3),'r-')\n");
	fprintf(out, "plot(TestOne(:,1),TestOne(:,2),'k.')\n\n");
	fprintf(out, "title('fc = %6.3f')\n", cutoff_frequency);



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



	// Set up the filter:
	cutoff_frequency = 0.09;
	setFilterData(&FD, 0.0);
	setFilterCoeff(&FC, cutoff_frequency);
	// Run the test:
	t = 0.0;
	fprintf(out, "TestThree = [...\n");
	for (i = 0; i < nSample; i++) {
		z = sin(t) + getRand(noise);
		y = runFilter(&FC, &FD, z);
		fprintf(out, "%6.6f %6.6f %6.6f ;\n", t, z, y);
		t = t + dt;
	}
	fprintf(out, "];\n");
	fprintf(out, "figure(3); clf; hold on;\n");
	fprintf(out, "plot(TestThree(:,1),TestThree(:,3),'r-')\n");
	fprintf(out, "plot(TestThree(:,1),TestThree(:,2),'k.')\n\n");
	fprintf(out, "title('fc = %6.3f')\n", cutoff_frequency);



	fclose(out);
	return 0;
}


