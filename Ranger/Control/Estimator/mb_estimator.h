#ifndef __MB_ESTIMATOR_H__
#define __MB_ESTIMATOR_H__

static struct FilterCoeff FC_hip, FC_gyro, FC_foot, FC_ankle, FC_hip_ang;
static struct FilterData FD_hip_rate, FD_hip_motor_rate, FD_gyro_rate, FD_hip_ang;
static struct FilterData FD_in_r, FD_in_l, FD_out_r, FD_out_l; // Filter data for foot sensors  
static struct FilterData FD_FI_angle, FD_FI_ang_rate, FD_FO_angle, FD_FO_ang_rate; // Filter data for foot angles & angular rate
static struct IntData ID_ang_rate;	 // integrator data for gyro rate
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
	float d0;	//Derivative at time k 
};

struct IntData {
	float currently_read_data;
	unsigned long time_of_curr_read_data;
	float prev_read_data;
	unsigned long time_of_prev_read_data; 
	float current_angle;
	float prev_angle; //stores the previous angle --> to be used in correcting the gyro angle when Ranger walks 
};

void mb_estimator_update(void);
void filter_init(void);
void filter_hip_ang(void);
void filter_hip_rate(void);
void filter_hip_motor_rate(void);
void filter_foot_sensor(void);
void filter_foot_data(void);
void setFilterCoeff(struct FilterCoeff*, float);
void setFilterData(struct FilterData*, float) ;
float runFilter_new(struct FilterCoeff*, struct FilterData*, float, unsigned long);
void int_init_ang_rate(void);
void int_ang_rate(void);


void filter_gyro_rate(void);
void test_sign(void);
void calibrate(void);
int FI_on_ground(void);
int FO_on_ground(void);
float get_out_angle(void);
float get_out_ang_rate(void);
float get_in_angle(void);
float get_in_ang_rate(void);
void set_gyro_angle(float);
float get_prev_gyro_angle(void);

#endif  // __MB_ESTIMATOR_H__

