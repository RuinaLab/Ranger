/**
	@file lcd.c
  
  The LCD module controls the LCD screen on the UI board.
  
  Example hardware setup for pins:
  @code
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
  @endcode
  
  Example interrupt setup:
  @code
  //LCD Timer
	VICVectAddr1 = (unsigned long)lcd_isr;
	VICVectCntl1 = (1<<5) | VIC_TIMER1;
	VICIntEnable = 1<<VIC_TIMER1;
	@endcode
  
	@author Nicolas Williamson 
  @author Emily Lee
  @author Nan (Sarah) Xiao
  @date July 2009
*/

#include <includes.h>

// *******************************************************************************
// LCD Instructions
// *******************************************************************************
#define LCD_CLEAR 0x01   /**< Instruction: clear screen */
#define LCD_HOME 0x02    /**< Instruction: return home */
#define LCD_ENTRY_MODE 0x06   /**< Instruction: don't increase cursor by one, no display shift */
#define LCD_FUNCTION_SET 0x38  /**< Instruction: 8 bit, 2 line display, 5x8 dot character */
#define LCD_DISPLAY_ON 0x0C  /**< Instruction: display on. Cursor and blink off */
#define LCD_DISPLAY_OFF 0x08   /**< Instruction: display, cursor, blink off */ 
#define LCD_CURSOR_SHIFT_R 0x14  /**< Instruction: shift cursor to right */
#define LCD_CURSOR_SHIFT_L 0x10  /**< Instruction: shift cursor to left */
#define LCD_DISPLAY_SHIFT_R 0x1C  /**< Instruction: shift display to right */
#define LCD_DISPLAY_SHIFT_L 0x18  /**< Instruction: shift display to left */
#define LCD_BOTTOM_LINE 0xA0 /**< Instruction: hift to write on the second line */
#define LCD_TOP_LINE 0x80 /**< Instruction: shift to write to the top line */

#define LCD_LENGTH (100) /**< Length of the ring buffer. */
#define QUAD_NUM (4) /**< Number of quads... uh */
LCD_CHAR lcd_ring[QUAD_NUM][LCD_LENGTH]; /**< The ring buffer (or 4 of them?) */
unsigned int lcd_head[QUAD_NUM] = {0,0,0,0}; /**< All of the head variables. */
unsigned int lcd_tail[QUAD_NUM] = {0,0,0,0};
unsigned int lcd_period;
unsigned int lcd_quad = 0; /**< The next buffer to check when popping data. */

VOID_VOID_F lcd_overflowfunc = voidvoid; /**< Function called when the timer overflows. Can be set to asched_tick to save a Timer. */

//Annoying state stuff...
int lcd_wait = 0; //time to execute the next function
LCD_CHAR lcd_target_char;
LCD_STATE lcd_state = LCD_IDLE;

/**
  Write the given byte instruction to the LCD controller.
  @param instruction The instruction to the LCD controller.
*/
void lcd_write_instruction(unsigned char instruction)
{
	int i;
	// ****** WRITE INSTRUCTION ******
	FIO0CLR = (1<<15); // RS = 0 instruction
	FIO0CLR = (1<<10); // R/~W = 0 write
	FIO0SET = (1<<11); // EN = 1   
//	FIO0PIN0 = instruction;
	for (i = 0; i < 8; i++){
		lcd_write_bit(i, instruction & (1<<i));
	}
	FIO0CLR = (1<<11); // EN = 0, falling edge
	
	if (instruction == LCD_HOME || instruction == LCD_CLEAR){
		lcd_wait = 2000;
	} else {
		lcd_wait = 50; //in ms; time to wait before executing the next function
    //lcd_wait = lcd_period;
	}
}

/**
  Writes a bit to data output.
  @param db The data output pin (0 to 8).
  @param bit The bit (0 or 1) to output.
*/
void lcd_write_bit(unsigned int db, unsigned int bit)
{
	if (bit == 0){
		FIO0CLR = (1<<db);
	} else {
		FIO0SET = (1<<db);
	}
}

/**
  Writes a byte of data to the lcd D0-D7.
  @param data Byte of data to send to the LCD.
*/
void lcd_write_data(unsigned char data)
{
	int i;
	// ****** WRITE BYTE ******
	FIO0SET = (1<<15); // RS = 1 data
	FIO0CLR = (1<<10); // R/~W = 0 write
	FIO0SET = (1<<11); // EN = 1
//	FIO0PIN0 = data;
	for (i = 0; i < 8; i++){
		lcd_write_bit(i, data & (1<<i));
	}
	FIO0CLR = (1<<11); // EN = 0, falling edge
	lcd_wait = lcd_period;
}

/**
  Prints a string to the LCD.
  @param string The string of characters to display.
  @param length The length of the string array.
  @param start_pos The start of the string when it is displayed on the LCD, 0 to 15.
*/
void lcd_print(char* string, unsigned int length, unsigned int start_pos)
{
	int i;
	for (i = 0; i < length; i++){
		lcd_print_char(string[i], start_pos + i);
	}
}

/**
  Prints a character to the LCD screen.
  @param ch The character to display.
  @param pos The position on the screen to display the char, 0-15.
*/
void lcd_print_char(char ch, unsigned int pos)
{    
	LCD_CHAR next_char;
	next_char.data = (unsigned char)ch;
	next_char.pos = pos & 0xF; //only acceptable positions are 0-15
  next_char.quad_num = pos>>2;  //quad_num is position divided by 4

	lcd_push(&next_char);
}

/**
  Pushes a char onto the ring buffer.
  If the buffer is full, new data is ignored.
  @param c A pointer to the LCD_CHAR struct to add to the buffer.
  @return 1 if successfully pushed, 0 if buffer is full.
*/
int lcd_push(LCD_CHAR* c)
{
  if ((lcd_head[(*c).quad_num] % LCD_LENGTH) != (lcd_tail[(*c).quad_num] % LCD_LENGTH) //ring not full
    || lcd_head[(*c).quad_num] == lcd_tail[(*c).quad_num]){ //or empty
    lcd_ring[(*c).quad_num][lcd_head[(*c).quad_num] % LCD_LENGTH] = *c;
    
    
    lcd_head[(*c).quad_num]++;
    return 1;
  } else { //buffer full, ignore new data
    error_occurred(ERROR_LCD_STR_OF);
    return 0;
  }
}

/**
  Pops the next character off of the ring buffer.
  The next character is the oldest one.
  @param c Pointer to the LCD_CHAR struct that will be filled with
  the popped info.
  @return 1 if successfully popped, 0 if all quad buffers empty.
*/
int lcd_pop(LCD_CHAR* c)
{
  //int i;
  //for(i = 0; i < 4; i++){ //check all buffers
    if (lcd_tail[0] != lcd_head[0]){ //buffer not empty
      *c = lcd_ring[0][lcd_tail[0] % LCD_LENGTH];
      lcd_tail[0]++;
      return 1;
    } 
  //}
    if (lcd_tail[1] != lcd_head[1]){ //buffer not empty
      *c = lcd_ring[1][lcd_tail[1] % LCD_LENGTH];
      lcd_tail[1]++;
      return 1;
    }
    if (lcd_tail[2] != lcd_head[2]){ //buffer not empty
      *c = lcd_ring[2][lcd_tail[2] % LCD_LENGTH];
      lcd_tail[2]++;
      return 1;
    }
    if (lcd_tail[3] != lcd_head[3]){ //buffer not empty
      *c = lcd_ring[3][lcd_tail[3] % LCD_LENGTH];
      lcd_tail[3]++;
      return 1;
    }
  return 0; //empty
}

/**
  A simple blocking delay. Only used during initialization of the
  LCD.
  @param delay_time_us Wait this long...
*/
void lcd_us_delay(long unsigned int delay_time_us)
{
	float ii = 0;
	while (ii < delay_time_us){
    ii = ii + 1;
    ii = ii * 1.001; 
  }
}

/**
  Prints 4 characters to quad 1.
  @param c1 Character at position 0.
  @param c2 Character at position 1.
  @param c3 Character at position 2.
  @param c4 Character at position 3.
*/
void lcd_set_quad1(char c1, char c2, char c3, char c4)
{
	int i = 0;
	lcd_print_char(c1, i++);
	lcd_print_char(c2, i++);  
	lcd_print_char(c3, i++);
	lcd_print_char(c4, i++);
}

/**
  Prints 4 characters to quad 2.
  @param c1 Character at position 4.
  @param c2 Character at position 5.
  @param c3 Character at position 6.
  @param c4 Character at position 7.
*/
void lcd_set_quad2(char c1, char c2, char c3, char c4)
{
	int i = 4;
	lcd_print_char(c1, i++);
	lcd_print_char(c2, i++);  
	lcd_print_char(c3, i++);
	lcd_print_char(c4, i++);
}

/**
  Prints 4 characters to quad 3.
  @param c1 Character at position 8.
  @param c2 Character at position 9.
  @param c3 Character at position 10.
  @param c4 Character at position 11.
*/
void lcd_set_quad3(char c1, char c2, char c3, char c4)
{
	int i = 8;
	lcd_print_char(c1, i++);
	lcd_print_char(c2, i++);  
	lcd_print_char(c3, i++);
	lcd_print_char(c4, i++);
}

/**
  Prints 4 characters to quad 4.
  @param c1 Character at position 12.
  @param c2 Character at position 13.
  @param c3 Character at position 14.
  @param c4 Character at position 15.
*/
void lcd_set_quad4(char c1, char c2, char c3, char c4)
{
	int i = 12;
	lcd_print_char(c1, i++);
	lcd_print_char(c2, i++);  
	lcd_print_char(c3, i++);
	lcd_print_char(c4, i++);
}

/**
  The interrupt service routine of the LCD module.
  Uses a timer to interrupt every ms.
  Decides the next instruction to give to the LCD screen
  by a state machine.
  @todo This could probably be changed to a non-interrupt lcd_update
  function that is called every ms.
*/
void lcd_isr(void) __irq
{
	//executed every lcd_period ms, so decrement msec wait counter by that much
	lcd_wait -= lcd_period;
  
	// ****** EXECUTE IF ALLOWED ******
	if (lcd_wait <= 0){
		// ****** ACT ACCORDING TO STATE ******
		switch (lcd_state){
			case LCD_MOVE: 
				lcd_write_instruction(0x80 | ((lcd_target_char.pos/8)*0x40)+(lcd_target_char.pos%8));
				lcd_state = LCD_DISPLAY;
				break;
			case LCD_DISPLAY: 
				lcd_write_data(lcd_target_char.data);
        
        
				if (lcd_pop(&lcd_target_char)){
//					lcd_target_char = lcd_pop();
					lcd_state = LCD_MOVE;
				} else {
					lcd_state = LCD_IDLE;
				}
	    		break;
	  		case LCD_IDLE: 
				if (lcd_pop(&lcd_target_char)){
//					lcd_target_char = lcd_pop();
					lcd_state = LCD_MOVE;
				}
				break;
		}
	}

	lcd_overflowfunc();
	
	T1IR = 0xFFFFFFFF;  // Clear ALL Timer1 interrupt flags.
	VICVectAddr = 0;
}

/**
  Initializes the LCD module and the screen.
  @param overflowfunc A void(void) function which is called when the Timer 
  driving the LCD module overflows due to a match.
*/
void lcd_init(VOID_VOID_F overflowfunc)
{
  lcd_period = 60000000 / ((T1MR0 + 1) * 1000); //in ms 
	lcd_overflowfunc = overflowfunc;
	lcd_write_instruction(LCD_FUNCTION_SET);
	lcd_us_delay(200);     
	lcd_write_instruction(LCD_DISPLAY_ON);
	lcd_us_delay(200);
	lcd_write_instruction(LCD_ENTRY_MODE);
	lcd_us_delay(200);
	lcd_write_instruction(LCD_CLEAR);
	lcd_us_delay(3000);	 
	lcd_write_instruction(LCD_HOME);
	lcd_us_delay(3000);
}
