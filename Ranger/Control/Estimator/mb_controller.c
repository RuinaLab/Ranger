#include <mb_includes.h> 

/*   Convert Int to Char
 */ 
char int2ascii(int num){
  return (num + '0');  
}


/*  Clears the LCD screen
 */
void clear_UI_LCD(int quad_num){  
  //clear the specified LCD quad
  mb_io_set_ul((ID_UI_SET_LCD_QUAD_1 + quad_num - 1), 0x20202020);
}


/*  Wrapper function for UI panel LCD screen. 
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
  mb_io_set_ul((ID_UI_SET_LCD_QUAD_1 + quad_number - 1), message_data);
}


/*  Turn off all LEDs. 
 */
 void clear_UI_LED(){
  mb_io_set_ul(ID_UI_SET_LED_1, 0x000000);
  mb_io_set_ul(ID_UI_SET_LED_2, 0x000000);
  mb_io_set_ul(ID_UI_SET_LED_3, 0x000000);
  mb_io_set_ul(ID_UI_SET_LED_4, 0x000000);
  mb_io_set_ul(ID_UI_SET_LED_5, 0x000000);
  mb_io_set_ul(ID_UI_SET_LED_6, 0x000000);
}


/*  Wrapper function for UI panel LED control. 
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
	mb_io_set_ul((ID_UI_SET_LED_1 + led_number - 1), val_color);
}



/*  This function is the entry-point for all controller stuff. It is called by the scheduler.
 */
void mb_controller_update(void){

// Read the current time on the robot
int time;  // Store the current robot time;
time = (int) mb_io_get_float(ID_TIMESTAMP);  //  Reads the time (in ms) since the robot was turned on

if (time % 500 < 250){
	set_UI_LED(1,'r');
} else {
	set_UI_LED(1,'b');
}

} // mb_controller_update()
