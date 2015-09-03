/*
	lcd.h
	
	Nicolas Williamson - July 2009
*/

#ifndef __H_LCD__
#define __H_LCD__

/**
  States of the controlling display state machine.
*/
typedef enum lcd_state{
	LCD_IDLE = 0, /**< Not displaying a character. */
	LCD_MOVE, /**< Moving the cursor to the position of the next char. */
	LCD_DISPLAY /**< Displaying the character on the screen. */
} LCD_STATE;

/**
  A character to display on the LCD screen.
*/
typedef struct lcd_char{
	volatile unsigned char data; /**< The character to display. */
	volatile unsigned char pos; /**< The position onscreen to display the char. */
  volatile unsigned int quad_num; /**< In which quad of the screen the character resides. */
} LCD_CHAR;

// Functions
void lcd_init(VOID_VOID_F overflowfunc);
void lcd_write_data(unsigned char data);
void lcd_write_instruction(unsigned char instruction);
void lcd_write_bit(unsigned int db, unsigned int bit);

//void lcd_push(LCD_CHAR c);
//LCD_CHAR lcd_pop(void);
int lcd_push(LCD_CHAR* c);
int lcd_pop(LCD_CHAR* c);

void lcd_print(char* string, unsigned int length, unsigned int start_pos);
void lcd_print_char(char ch, unsigned int pos);
void lcd_us_delay(long unsigned int delay_time_us);
//methods for CAN setting
void lcd_set_line1(char c1, char c2, char c3, char c4, char c5, char c6, char c7, char c8);
void lcd_set_line2(char c1, char c2, char c3, char c4, char c5, char c6, char c7, char c8);
void lcd_set_quad1(char c1, char c2, char c3, char c4);
void lcd_set_quad2(char c1, char c2, char c3, char c4);
void lcd_set_quad3(char c1, char c2, char c3, char c4);
void lcd_set_quad4(char c1, char c2, char c3, char c4);

//timer isr
__irq void lcd_isr(void);

/*HARDWARE SETUP

	// *******************************************************************************
	// Initialize LCD
	// *******************************************************************************
	PINSEL0 &= ~(0xFFFF); //Pins P0.0-P0.7 GPIO (D0-D7 on LCD controller)
	PINSEL0 &= ~(((unsigned int)3)<<30); //P0.15 GPIO (RS)
	PINSEL0 &= ~(3<<20); //P0.10 GPIO (R/~W)
	PINSEL0 &= ~(3<<22); //P0.11 GPIO (EN)
	FIO0DIR |= 0x8CFF; //set above pins as output
	FIO0MASK &= ~(0xFF); //zero fiomask
	//Timer stuff
	T0PR = 0;//no prescaling
	T0MR0 = 60000000/20000;//Match Register for scheduler interrupts
	T0TCR = 1;//enabled for counting
	T0CTCR = 0;//simple timer
	T0MCR = (1)|(1<<1);//interrupt (bit0) and reset (bit1) on MR0

*/

#endif
