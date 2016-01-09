#ifndef __INPUT_OUTPUT_H__
#define __INPUT_OUTPUT_H__
				 
#include <stdbool.h>

// Utility Functions:
char int2ascii(int);
bool detect_UI_button_input(int);
void set_UI_LCD(char*, int);
void clear_UI_LED(void);
void clear_UI_LCD(int);
void set_UI_LED(int, char); 

// Buzzer stuff
extern float BUZZER_BEEP_TIME_SECONDS;
extern int BUZZER_BEEP_FREQ_HZ;
void buzzer_beep(bool turnOn);  // Turn on a buzzer! 

// Test function:
void test_input_output(void); 

#endif // __INPUT_OUTPUT_H__
