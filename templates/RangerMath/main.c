//============================================================================
// Name        : main.c
// Authors     : Matthew Kelly
//
// Demonstrate how to use the simple mathematics functions for Ranger
//
//============================================================================

#include <stdio.h>

#include "RangerMath.h"

int main( int argc, const char ** argv ) {

	float x;  // input
	float xLow, xUpp;  //bounds on the input
	float y;  // output
	float dt; // time step
	int n;    // number of points
	int i; 	  // loop counter

	/* Create a matlab script for plotting the output */
	FILE *out;
	out = fopen("TestScript.m","w");


	/* Test of the Sin() function */
	xLow = -1.0;
	xUpp = 10.0;
	x = xLow;
	n = 200;
	dt = (xUpp-xLow)/(n-1);
	fprintf(out,"TestSin = [...\n");
	for (i=0; i<n; i++){
		y = Sin(x);
		fprintf(out,"%6.6f %6.6f;\n", x, y);
		x = x + dt;
	}
	fprintf(out,"];\n");
	fprintf(out,"figure(1); clf; hold on;\n");
	fprintf(out,"plot(TestSin(:,1),TestSin(:,2),'ko')\n");
	fprintf(out,"plot(TestSin(:,1),sin(TestSin(:,1)),'k.')\n\n");


	/* Test of the Cos() function */
	xLow = -1.0;
	xUpp = 10.0;
	x = xLow;
	n = 200;
	dt = (xUpp-xLow)/(n-1);
	fprintf(out,"TestCos = [...\n");
	for (i=0; i<n; i++){
		y = Cos(x);
		fprintf(out,"%6.6f %6.6f;\n", x, y);
		x = x + dt;
	}
	fprintf(out,"];\n");
	fprintf(out,"figure(2); clf; hold on;\n");
	fprintf(out,"plot(TestCos(:,1),TestCos(:,2),'ko')\n");
	fprintf(out,"plot(TestCos(:,1),cos(TestCos(:,1)),'k.')\n\n");


	/* Test of the Atan() function */
	xLow = -4.0;
	xUpp = 4.0;
	x = xLow;
	n = 200;
	dt = (xUpp-xLow)/(n-1);
	fprintf(out,"TestAtan = [...\n");
	for (i=0; i<n; i++){
		y = Atan(x);
		fprintf(out,"%6.6f %6.6f;\n", x, y);
		x = x + dt;
	}
	fprintf(out,"];\n");
	fprintf(out,"figure(3); clf; hold on;\n");
	fprintf(out,"plot(TestAtan(:,1),TestAtan(:,2),'ko')\n");
	fprintf(out,"plot(TestAtan(:,1),atan(TestAtan(:,1)),'k.')\n\n");



	/* Test of the Tanh() function */
	xLow = -5.0;
	xUpp = 5.0;
	x = xLow;
	n = 800;
	dt = (xUpp-xLow)/(n-1);
	fprintf(out,"TestTanh = [...\n");
	for (i=0; i<n; i++){
		y = Tanh(x);
		fprintf(out,"%6.6f %6.6f;\n", x, y);
		x = x + dt;
	}
	fprintf(out,"];\n");
	fprintf(out,"figure(5); clf; hold on;\n");
	fprintf(out,"plot(TestTanh(:,1),TestTanh(:,2),'ko')\n");
	fprintf(out,"plot(TestTanh(:,1),tanh(TestTanh(:,1)),'k.')\n\n");


	/* Test of the Sqrt() function */
	xLow = -0.1;
	xUpp = 2.5;
	x = xLow;
	n = 200;
	dt = (xUpp-xLow)/(n-1);
	fprintf(out,"TestSqrt = [...\n");
	for (i=0; i<n; i++){
		y = Sqrt(x);
		fprintf(out,"%6.6f %6.6f;\n", x, y);
		x = x + dt;
	}
	fprintf(out,"];\n");
	fprintf(out,"figure(4); clf; hold on;\n");
	fprintf(out,"plot(TestSqrt(:,1),TestSqrt(:,2),'ko')\n");
	fprintf(out,"plot(TestSqrt(:,1),sqrt(TestSqrt(:,1)),'k.')\n\n");



	fclose(out);
	return 0;
}