#ifndef __LED_LCD_OUTPUT_H__
#define __LED_LCD_OUTPUT_H__

// Utility Functions:
char int2ascii(int);
void set_UI_LCD(char*, int);
void clear_UI_LED(void);
void clear_UI_LCD(int);
void set_UI_LED(int, char); 

// Test function:
void test_led_lcd(void); 

#endif // __LED_LCD_OUTPUT_H__
