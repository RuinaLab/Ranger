#ifndef __UI_FSM_H__
#define __UI_FSM_H__


//#include "fsm.h"

void def_UI_fsm(fsm* h_fsm);

int detect_error(void);
void clear_UI_LCD(int quad_num);
void clear_UI_LED();
void set_UI_LCD(char * message, int quad_number);
void set_UI_LED(int led_number, char color);
char int2ascii(int num);

#endif  //__UI_FSM_H__
