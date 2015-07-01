#ifndef __MB_ESTIMATOR_H__
#define __MB_ESTIMATOR_H__

// MAIN function
void mb_estimator_update(void);  // this will be FINALLY CALLED by the main brain and it will do all the estimations

// other functions, many may not be in use any more. 
void mb_abs_leg_and_ankle_angles_and_swingfootheight(void);
void mb_time_since_last_HS_and_step_counter (void); // it calculates times since last HS
void mb_hip_motor_rate_filter (void);
void mb_hip_rate_filter(void);
void mb_imu_rate_filter (void); 
void mb_leg_state_machine_old(void);
void mb_leg_state_machine_new(void);
// void mb_hip_rate_from_hip_angle(void);
void  mb_model_based_estimator(void);
float greg_sqrt(float xin);   // by greg: INPUT should only be BETWEEN 0 and 2. Doesn't work well when less than .01.... considering I use the sqrt for number less than leg+ankle_length so thats always less than 2 in m.
float greg_atan(float xin);    // by greg (it uses Quadratic interpolation)
float greg_cos(float xin);  // by greg (it uses Quadratic interpolation)
float greg_sin(float xin);  // by greg (it uses Quadratic interpolation)
float anoop_asin(float xin); // arc sin by anoop using soem degree polynomial. 
float signum(float x);

void ctrl_hip_(float Kp, float Kd, float uref, float xref, float vref);
char int2ascii_(int num);
void clear_UI_LCD_(int quad_num);
void clear_UI_LED_(void);
void set_UI_LCD_(char* message, int quad_number);
void set_UI_LED_(int led_number, char color);
void control_run(void);

#endif  //__MB_ESTIMATOR_H__
