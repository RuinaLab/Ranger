#ifndef __CONTROL_CODE_H__
#define __CONTROL_CODE_H__

void ctrl_hip_(float Kp, float Kd, float uref, float xref, float vref);

char int2ascii_(int num);

void clear_UI_LCD_(int quad_num);

void clear_UI_LED_();

void set_UI_LCD_(char* message, int quad_number);

void set_UI_LED_(int led_number, char color);

void control_run(void);

#endif 

