#ifndef __MB_ESTIMATOR_H__
#define __MB_ESTIMATOR_H__

struct FilterCoeff {
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
};

struct FilterData {
	float z0; // Measurement at time k
	float z1; // Measurement at time k-1
	float z2; // Measurement at time k-2
	float y0; // Estimate at time k
	float y1; // Estimate at time k-1
	float y2; // Estimate at time k-2
	float t0; // time k
	float t1; // time k-1
	float t2; // time k-2
};

void mb_estimator_update(void);
void setFilterCoeff(struct FilterCoeff*, float);
void setFilterData(struct FilterData*, float) ;
float runFilter_new(struct FilterCoeff*, struct FilterData*, float, float);
float runFilter(struct FilterCoeff*, struct FilterData*, float);

#endif  // __MB_ESTIMATOR_H__

