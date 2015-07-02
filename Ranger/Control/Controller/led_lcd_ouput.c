#include <mb_includes.h> 
#include <led_lcd_ouput.h>

/*   Convert Int to Char
 */ 
char int2ascii(int num){
  return (num + '0');  
}


/*  Wrapper function for UI panel LCD screen.  The message should
 * 	contain an array of four characters, and quad_number should be 
 *  from the set {1,2,3,4}.
 */
void set_UI_LCD(char* message, int quad_number){
  //Display message on specified LCD quad
  unsigned long message_data;
  message_data = message[3];
  message_data <<= 8;
  message_data |= message[2] & 0xFF;
  message_data <<= 8;
  message_data |= message[1] & 0xFF;
  message_data <<= 8;
  message_data |= message[0] & 0xFF;
  switch (quad_number){
	  case 1: // Upper left
	  	// set_io_ul(ID_UI_SET_LCD_QUAD_1, message_data);
		set_io_ul((ID_UI_SET_LCD_QUAD_1 + quad_number - 1), message_data);
	  	break;
	  case 2: // 
	  	//set_io_ul(ID_UI_SET_LCD_QUAD_2, message_data);
		set_io_ul((ID_UI_SET_LCD_QUAD_1 + quad_number - 1), message_data);
	  	break;
	  case 3:
	  	//set_io_ul(ID_UI_SET_LCD_QUAD_3, message_data);
		set_io_ul((ID_UI_SET_LCD_QUAD_1 + quad_number - 1), message_data);
	  	break;
	  case 4:
	  	//set_io_ul(ID_UI_SET_LCD_QUAD_4, message_data);
		set_io_ul((ID_UI_SET_LCD_QUAD_1 + quad_number - 1), message_data);
		break;
  }
  
}


/*  Turn off all LEDs. 
 */
 void clear_UI_LED(){
  set_io_ul(ID_UI_SET_LED_1, 0x000000);
  set_io_ul(ID_UI_SET_LED_2, 0x000000);
  set_io_ul(ID_UI_SET_LED_3, 0x000000);
  set_io_ul(ID_UI_SET_LED_4, 0x000000);
  set_io_ul(ID_UI_SET_LED_5, 0x000000);
  set_io_ul(ID_UI_SET_LED_6, 0x000000);
}

	   
/*  Clears the LCD screen
 */
void clear_UI_LCD(int quad_num){  
  //clear the specified LCD quad
  char msg[4];
  int i;
  for (i=0; i<4; i++){
  	msg[i] = ' ';
  }
  set_UI_LCD( msg, quad_num);
}

/*  Wrapper function for UI panel LED control. Note that 
 *  LED 6 appears to be broken.
 */
void set_UI_LED(int led_number, char color){
  //Set specified LED to assigned color
	int val_color;
	switch(color){
		case '-':  //off
      		val_color = 0x000000;  //turn off the led
     		 break;
		case 'r':  //RED
			val_color = 0x000080;  //set color to red
			break;
		case 'g': //GREEN
			val_color = 0x008000;  //set color to green
			break;
		case 'b':  //BLUE
			val_color = 0x800000;  //set color to blue
			break;
    	case 'p':  //PURPLE
			val_color = 0xe22b8a;  //set color to purple
			break;
   		case 'y':  //YELLOW
			val_color = 0x008080;  //set color to yellow
			break;
    	case 'o':  //ORANGE //Looks like cyan (light blue)
      		val_color = 0x008160;  //set color to orange  
			break;
    	case 'm':  //MAGENTA //Looks like cyan (light blue)
      		val_color = 0x800080;  //set color to magenta 
			break;
    	case 'c':  //CYAN //Light blue
      		val_color = 0x808000;  //set color to cyan      
			break; 
	}
	set_io_ul((ID_UI_SET_LED_1 + led_number - 1), val_color);
}



/*  This function runs a simple test of all LEDs and the LCD screen.
 * 	If all goes correctly, each LED should cycle through every color (inlcuding off)
 * 	and the LCD should display '    ', '1111', '2222','3333','4444' in each of the
 * 	four quadrants.
 *
 * 	Results 7/2/15 
 * 		LED 6 does not work
 * 		LCD screen has major problems. Not sure why.
 */
void test_led_lcd(void){

char msg1[4];
char msg2[4];
char msg3[4];
char msg4[4];
int i;
int time;
int color;
int led_id;
int panel;
int lcd_msg;

// Apply values to the messages
for(i=0; i<4; i++){
  	msg1[i] = '1';
	msg2[i] = '2';
	msg3[i] = '3';
 	msg4[i] = '4';
}

clear_UI_LCD(1);
//set_UI_LCD(msg1,1);
//set_UI_LCD(msg2,3);
//set_UI_LCD(msg3,2);
//set_UI_LCD(msg4,1);

//  Read the time (in ms) since the robot was turned on
time = (int) mb_io_get_float(ID_TIMESTAMP);  
time = time / 800;  // integer division!
color = time % 9;
led_id = time % 6;
panel = time % 4;
lcd_msg = time % 5;


// Test I/O for LCD and LED
switch (color) {
	case 0:
	  	set_UI_LED(led_id,'-');
		break;
	case 1:
	  	set_UI_LED(led_id,'r');
		break;
	case 2:
	  	set_UI_LED(led_id,'g');	 
		break;
	case 3:
	  	set_UI_LED(led_id,'b');
		break;
	case 4:
	  	set_UI_LED(led_id,'p');
		break;
	case 5:
	  	set_UI_LED(led_id,'y');	 
		break;
	case 6:
	  	set_UI_LED(led_id,'o');
		break;
	case 7:
	  	set_UI_LED(led_id,'m');
		break;
	case 8:
	  	set_UI_LED(led_id,'c');
		break;
}

switch (lcd_msg) {
 	case 0:
		 clear_UI_LCD(panel);
		 break;
	case 1: 
		set_UI_LCD(msg1,panel);
		break;	
	case 2: 
		set_UI_LCD(msg2,panel);
		break;
	case 3: 
		set_UI_LCD(msg3,panel);
		break;
	case 4: 
		set_UI_LCD(msg4,panel);
		break;
}


} // test_led_lcd()
