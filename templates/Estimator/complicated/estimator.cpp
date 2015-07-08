#include "stdafx.h"
#include "estimator.h"
#include <math.h>   // tan()

#define SQRT_TWO 1.414213562373095
#define PI 3.141592653589793

/* Computes the coefficients for a second-order low-pass butterworth filter
 * @param r = ratio of cut-off frequncy to half of the sample frequency.
 * valid domain:  0.01 < r < 0.99   (coerced if out of bounds)
 */
void setFilterCoeff(struct FilterCoeff * FC, float r) {
	if (r < 0.001) r = 0.001;  // Prevents a divide by zero
	if (r > 0.999) r = 0.999;  // Cannot exceed Nyquist frequency

	static float q = SQRT_TWO;
	static float c;
	c = tan(0.5 * PI * (1.0 - r));

	FC->b0 = 1.0 / (1.0 + q * c + c * c);
	FC->b1 = 2.0 * (FC->b0);
	FC->b2 = (FC->b0);

	FC->a1 = -2.0 * (c * c - 1.0) * (FC->b0);
	FC->a2 = (1.0 - q * c + c * c) * (FC->b0);

}


/* Initializes the filter to a single value
 */
void setFilterData(struct FilterData * FD, float z) {
	FD->z0 = z;
	FD->z1 = z;
	FD->z2 = z;
	FD->y0 = z;
	FD->y1 = z;
	FD->y2 = z;
	FD->t0 = 0;
	FD->t1 = 0;
	FD->t2 = 0;
}

#define DIFF 0.01
/* Updates the Butterworth filter based on new sensor data
 * @param z = the measurement at the current time step
 * @return y = the filtered data at the current time step
 */
float runFilter_new(struct FilterCoeff * FC, struct FilterData * FD, float z, float t) {
	printf("enter: t = %f, z = %f, t0 = %f\n", t, z, FD->t0);
	//if the time elapsed exceeds 0.01s, calculate the estimants for time stamps missed 
	if( (t - FD->t0) > DIFF){
		printf("	call runFilter again\n");
		runFilter_new(FC, FD, FD->z0, t-DIFF);	
	}
	// Update sensor history:
	FD->z2 = FD->z1;
	FD->z1 = FD->z0;
	FD->z0 = z;
	// Update estimate history:
	FD->y2 = FD->y1;
	FD->y1 = FD->y0;
	// Update time history;
	FD->t2 = FD->t1;
	FD->t1 = FD->t0;
	FD->t0 = t;
	// Compute new estimate:
	FD->y0 =
	    (FC->b0) * (FD->z0) +
	    (FC->b1) * (FD->z1) +
	    (FC->b2) * (FD->z2) -
	    (FC->a1) * (FD->y1) -
	    (FC->a2) * (FD->y2);
	printf("leave: t = %f, z = %f, t0 = %f\n", t, z, FD->t0);
	printf("       t0 = %f, t1 = %f, t2 = %f\n", FD->t0, FD->t1, FD->t2);
	printf("       z0 = %f, z1 = %f, z2 = %f\n", FD->z0, FD->z1, FD->z2);
	printf("       y0 = %f, y1 = %f, y2 = %f\n", FD->y0, FD->y1, FD->y2);
	return (FD->y0);
}

float runFilter(struct FilterCoeff * FC, struct FilterData * FD, float z) {
	// Update sensor history:
	FD->z2 = FD->z1;
	FD->z1 = FD->z0;
	FD->z0 = z;
	// Update estimate history:
	FD->y2 = FD->y1;
	FD->y1 = FD->y0;
	// Compute new estimate:
	FD->y0 =
	    (FC->b0) * (FD->z0) +
	    (FC->b1) * (FD->z1) +
	    (FC->b2) * (FD->z2) -
	    (FC->a1) * (FD->y1) -
	    (FC->a2) * (FD->y2);
	return (FD->y0);
}
