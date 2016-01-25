/* global_sensors.h
   
   This file will be used to define the global sensor variables which will 
   be updated at each time step. 

*/

#ifndef _GLOBAL_SENSORS_H
#define _GLOBAL_SENSORS_H

//extern double g_right_foot_contact_sensor;
//extern double g_left_foot_contact_sensor;
//extern double g_hip_angle_sensor;
//extern double g_hip_imu_sensor;
//extern double g_right_ankle_rotation_sensor;
//extern double g_left_ankle_rotation_sensor;
extern int g_top_fsm_stop_command;
extern int g_hip_fsm_stop_command;
extern int g_foot_inner_fsm_stop_command;
extern int g_foot_outer_fsm_stop_command;
extern int g_ankle_fsm_stop_command;
extern int g_steering_fsm_stop_command;
extern int g_ui_fsm_stop_command;
#endif
