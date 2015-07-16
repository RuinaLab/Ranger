#ifndef __MB_ESTIMATOR_H__
#define __MB_ESTIMATOR_H__

static struct FilterCoeff FC;
static struct FilterData FD_hip_rate, FD_hip_motor_rate, FD_gyro_rate;
//static int f_init = 0; 
static struct IntegratorData ID_ang_rate;
//static int i_init_ang_rate = 0;
static int init = 0;

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
	unsigned long t0; // time k
};

struct IntegratorData {
	float currently_read_data;
	unsigned long time_of_curr_read_data;
	float prev_read_data;
	unsigned long time_of_prev_read_data; 
	float current_angle;
};

void mb_estimator_update(void);
void filter_init(void);
void filter_hip_rate(void);
void filter_hip_motor_rate(void) ;
void setFilterCoeff(struct FilterCoeff*, float);
void setFilterData(struct FilterData*, float) ;
float runFilter_new(struct FilterCoeff*, struct FilterData*, float, unsigned long);
void intergrater_init_ang_rate(void);
void integrate_ang_rate(void);
void filter_gyro_rate(void);
void test_sign(void);

#endif  // __MB_ESTIMATOR_H__

