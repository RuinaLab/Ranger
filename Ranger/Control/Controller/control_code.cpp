#include <mb_includes.h>
//#include "control_code.h"

void ctrl_hip_(float Kp, float Kd, float uref, float xref, float vref){
	//uref = reference current 
	//xref = reference angle 
	//vref = reference rate 
	set_io_float(ID_MCH_STIFFNESS, Kp);
	set_io_float(ID_MCH_DAMPNESS, Kd);
	float command_current = uref + Kp*xref + Kd*vref; 
    set_io_float(ID_MCH_COMMAND_CURRENT, command_current);		
} 

char int2ascii_(int num){
  //Convert Int to Char
  return (num + '0');  
}

void clear_UI_LCD_(int quad_num){  
  //clear the specified LCD quad
  set_io_ul((ID_UI_SET_LCD_QUAD_1 + quad_num - 1), 0x20202020);
}

void clear_UI_LED_(){
  //Turn off all the LEDs
  set_io_ul(ID_UI_SET_LED_1, 0x000000);
  set_io_ul(ID_UI_SET_LED_2, 0x000000);
  set_io_ul(ID_UI_SET_LED_3, 0x000000);
  set_io_ul(ID_UI_SET_LED_4, 0x000000);
  set_io_ul(ID_UI_SET_LED_5, 0x000000);
  set_io_ul(ID_UI_SET_LED_6, 0x000000);
}

void set_UI_LCD_(char* message, int quad_number){
  //Display message on specified LCD quad
	unsigned long message_data;
  message_data = message[3];
  message_data <<= 8;
  message_data |= message[2] & 0xFF;
  message_data <<= 8;
  message_data |= message[1] & 0xFF;
  message_data <<= 8;
  message_data |= message[0] & 0xFF;
  set_io_ul((ID_UI_SET_LCD_QUAD_1 + quad_number - 1), message_data);
}

void set_UI_LED_(int led_number, char color){
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
    case 'm':  //MAGENTA //Looks like cyan (light blue)
      val_color = 0x800080;  //set color to magenta 
    case 'c':  //CYAN //Light blue
      val_color = 0x808000;  //set color to cyan     
      
	}
	set_io_ul((ID_UI_SET_LED_1 + led_number - 1), val_color);
}

void control_run(void){
	//*************yawen***************
	//make the hip angle track a curve in time  
	float Kp = get_io_float(ID_F_TEST4); 
	float Kd = get_io_float(ID_F_TEST5); 
	float uref = get_io_float(ID_F_TEST6);
	float xref = get_io_float(ID_F_TEST7); 
	float vref = get_io_float(ID_F_TEST8);
	ctrl_hip_(Kp, Kd, uref, xref, vref); 

	//make the LEDs blink according to the value of the hip angle  
    char msg[4];
	float hip_angle = get_io_float(ID_MCH_ANGLE);
						 
	if(hip_angle>-0.2){
		set_UI_LED_(2, 'g');	
	}
	if(hip_angle>-0.1){
		set_UI_LED_(3, 'g');
	}
	if(hip_angle>0){
		set_UI_LED_(5, 'g');
	}
	if(hip_angle>0.1){
		set_UI_LED_(4, 'g');
	}
	if(hip_angle>0.2){
	    set_UI_LED_(6, 'g');
	} 

	if(hip_angle<0){
		hip_angle = -hip_angle;
		msg[0]= '-';
	}else{
		msg[0]= '+';
	} 
	msg[1]=	'.';
	msg[2]=	int2ascii_((int)(hip_angle*10)); 
	msg[3]= int2ascii_((int)(hip_angle*100));
	//cout << "hip angle is" << hip_angle << endl;
	clear_UI_LCD_(2);
	set_UI_LCD_(msg, 2);

//*********************************
    
	return;
}

