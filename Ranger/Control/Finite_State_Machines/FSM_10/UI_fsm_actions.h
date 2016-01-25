#include "error_messages.h"
#include <string.h>


///////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int lcd_display_period = 6000;	 //if there's error, error message and default message will display for 1ms alternatively
unsigned int error_flag = 0;  //1 if there's error, 0 if no error
char error_code[4][4]; //[row] = quad, so error_code[0] = quad1
unsigned int lcd_update_period = 400;

extern fsm* hip_fsm;
extern fsm* foot_inner_fsm;
extern fsm* foot_outer_fsm;
extern fsm* steering_fsm;
extern fsm* top_fsm;

//Function declaration 
int detect_error(void);
void clear_UI_LCD(int quad_num);
void clear_UI_LED();
void set_UI_LCD(char * message, int quad_number);
void set_UI_LED(int led_number, char color);
char int2ascii(int num);

// 
//static int counter_for_stop;
//static int get_ready_to_stop;

void motors_off(void)
{
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFI_STIFFNESS, 0.0);
  set_io_float(ID_MCFI_DAMPNESS, 0.0);
  
  set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFO_STIFFNESS, 0.0);
  set_io_float(ID_MCFO_DAMPNESS, 0.0);
  
  set_io_float(ID_MCSI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCSI_STIFFNESS, 0.0);
  set_io_float(ID_MCSI_DAMPNESS, 0.0);
  
//  set_io_ul(ID_MCFO_SHUTDOWN,1);
//  set_io_ul(ID_MCSI_SHUTDOWN,1);
//  set_io_ul(ID_MCH_SHUTDOWN,1);
//  set_io_ul(ID_MCFI_SHUTDOWN,1);
}

void all_fsm_run(void)
{
    hip_fsm->run();
    set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);
    foot_inner_fsm->run();
    set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);
    foot_outer_fsm->run();
    set_io_float(ID_MB_FOOT_OUTER_FSM_STATE, (float)g_foot_outer_fsm_state);
    steering_fsm->run();
    set_io_float(ID_MB_STEERING_FSM_STATE, (float)g_steering_fsm_state);
}


int ACTION_UI_calibrate(void)
{   		 
     
    //********* Put calibrate actions here *******//
    
    //Currently no calibration routines
    
    // ************************************* // 
     
  	//initialize LED	
    clear_UI_LED();
    //set LED
    set_UI_LED(1, 'p');  //Set LED #1 to PURPLE
  		
    //detect error
  	if(detect_error() == 1){	//there's an error
  		error_flag = 1;  //set error flag
  		set_UI_LED(4, 'r');	 //LED #4 turns on to RED along with the mode LED, if there's an error
  	}
	else{
		error_flag = 0;  //set error flag
  		set_UI_LED(4, '-');	 //turns off LED #4
	}
  
    //LCD display
  	if(((mb_get_timestamp() % lcd_display_period) < (lcd_display_period / 2))
  	&& (error_flag == 1)){ //if there's error, then display error code for 3s
  		//display error	code. Don't need to clear lcd, because error code takes all the four quads.
      if((mb_get_timestamp() % lcd_update_period) < 1){
        //error code contains 14 characters. each quad of LCD displays 4 characters.
			set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000001);  //turn on buzzer
    		set_UI_LCD(error_code[0], 1);
    		set_UI_LCD(error_code[1], 2);
    		set_UI_LCD(error_code[2], 3);
    		set_UI_LCD(error_code[3], 4);
      }
  	}
  	else{  //always display mode message if there's no error
      if((mb_get_timestamp() % lcd_update_period) < 1){ 
	  		set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000000);  //turn off buzzer
        //Clear the other LCD quads
    		clear_UI_LCD(2);
    		clear_UI_LCD(3);
    		clear_UI_LCD(4);
    		//Dislplay message for Calibrate Mode on LCD
    		set_UI_LCD("M: C", 1);
      }
  	}  
  	return 1;
}


int ACTION_UI_walk_entry(void)
{ 
   
  	set_io_float(ID_FSM_RESET,0); //Stop the reset of fsm's
    
    return 1;
}


int ACTION_UI_walk(void)
{
  	char batt_power_1, batt_power_2, hip_fsm_state, foot_inner_state, foot_outer_state;  //receive data from io
    char batt_msg[4], hip_msg[4], foot_inner_msg[4], foot_outer_msg[4];  //form LCD display message
	  int batt_power;  //battery power is a 2-digit number
      
    //********* Put walk actions here *******//
    
    //No need to call these if top_fsm starts in normal walk mode which it does.
    //set_io_float(ID_T_TEST1,0.9);
    //set_io_float(ID_T_TEST3,0.0); //don't stop
    //set_io_float(ID_T_TEST2,0.9);
    
    all_fsm_run();
    
    // ************************************* // 
    
  
	  //initialize LED	
    clear_UI_LED();
    //set LED
  	//set_UI_LED(2, 'g'); //Turning off for big walk to save W, Pranav 7/5/2010
		
    //detect error
  	if(detect_error() == 1){
  		error_flag = 1;
  		set_UI_LED(4, 'r');
  	}
	else{
		error_flag = 0;  //set error flag
  		set_UI_LED(4, '-');	 //turns off LED #4
	}	

	  //get Battery Power, state of HIP FSM, state of HIP INNER and state of HIP OUTER from IO
    batt_power = (int)get_io_float(ID_MCFI_BATT_VOLTAGE);
	  batt_power_1 = int2ascii(batt_power/10);
	  batt_power_2 = int2ascii(batt_power - (batt_power/10)*10);
    hip_fsm_state = int2ascii((int)get_io_float(ID_MB_HIP_FSM_STATE));
    foot_inner_state = int2ascii((int)get_io_float(ID_MB_FOOT_INNER_FSM_STATE));
    foot_outer_state = int2ascii((int)get_io_float(ID_MB_FOOT_OUTER_FSM_STATE));
    
    //form messages to display on LCD
    batt_msg[0] = 'B'; batt_msg[1] = '='; batt_msg[2] = batt_power_1; batt_msg[3] = batt_power_2;  //message for Battery Voltage is "B=battery_power"
    hip_msg[0] = ' '; hip_msg[1] = 'H'; hip_msg[2] = '='; hip_msg[3] = hip_fsm_state;  //message for Hip FSM State is " H=hip_fsm_state"
    foot_inner_msg[0] = ' '; foot_inner_msg[1] = 'I'; foot_inner_msg[2] = '='; foot_inner_msg[3] = foot_inner_state;  //message for Inner Foot State is " I=foot_inner_state"
    foot_outer_msg[0] = ' '; foot_outer_msg[1] = 'O'; foot_outer_msg[2] = '='; foot_outer_msg[3] = foot_outer_state;  //message for Outer Foot State is " O=foot_outer_state"

  	//LCD display
  	if(((mb_get_timestamp() % lcd_display_period) < (lcd_display_period / 2))
  	&& (error_flag == 1)){ 
      if((mb_get_timestamp() % lcd_update_period) < 1){
	  		set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000001); //turn on buzzer
    		set_UI_LCD(error_code[0], 1);
    		set_UI_LCD(error_code[1], 2);
    		set_UI_LCD(error_code[2], 3);
    		set_UI_LCD(error_code[3], 4);
      }
  	}
  	else{  //display mode message
  		//message for Walk Mode
      if((mb_get_timestamp() % lcd_update_period) < 2){
	  		set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000000);  //turn off buzzer
    		set_UI_LCD(batt_msg, 1);  //display Battery Voltage on Quad 1
    		set_UI_LCD(hip_msg, 2);  //display Hip FSM State on Quad 2
    		set_UI_LCD(foot_inner_msg, 3);  //display Inner Foot State on Quad 3
    		set_UI_LCD(foot_outer_msg, 4);  //display Outer Foot State on Quad 4
      }
  	}
  
  	return 1;
 }


int ACTION_UI_walk_exit(void)
{ 
   
  	motors_off(); //Turn motors off.
    
    return 1;
}

int ACTION_UI_rc_walk_entry(void)
{ 
   
  	set_io_float(ID_FSM_RESET,0); //Stop the reset of fsm's
   
    set_io_float(ID_A_T_RC1_NORM,0.5); //Set rc1 channel 0.5 on starting
    
    return 1;
}


int ACTION_UI_rc_walk(void)
{
  	char batt_power_1, batt_power_2, hip_fsm_state, foot_inner_state, foot_outer_state;  //receive data from io
    char batt_msg[4], hip_msg[4], foot_inner_msg[4], foot_outer_msg[4];  //form LCD display message
	  int batt_power;  //battery power is a 2-digit number
      
    //********* Put walk actions here *******//
          
    //Extra lines of code in rc_walk      
    top_fsm->run();
    set_io_float(ID_MB_TOP_FSM_STATE, (float)g_top_fsm_state);    
    //Extra lines end
    
    all_fsm_run();
    
    // ************************************* // 
    
  
	  //initialize LED	
    clear_UI_LED();
    //set LED
  	set_UI_LED(2, 'g'); //Turning off for big walk to save W, Pranav 7/5/2010
		
    //detect error
  	if(detect_error() == 1){
  		error_flag = 1;
  		set_UI_LED(4, 'r');
  	}
	else{
		error_flag = 0;  //set error flag
  		set_UI_LED(4, '-');	 //turns off LED #4
	}	

	  //get Battery Power, state of HIP FSM, state of HIP INNER and state of HIP OUTER from IO
    batt_power = (int)get_io_float(ID_MCFI_BATT_VOLTAGE);
	  batt_power_1 = int2ascii(batt_power/10);
	  batt_power_2 = int2ascii(batt_power - (batt_power/10)*10);
    hip_fsm_state = int2ascii((int)get_io_float(ID_MB_HIP_FSM_STATE));
    foot_inner_state = int2ascii((int)get_io_float(ID_MB_FOOT_INNER_FSM_STATE));
    foot_outer_state = int2ascii((int)get_io_float(ID_MB_FOOT_OUTER_FSM_STATE));
    
    //form messages to display on LCD
    batt_msg[0] = 'B'; batt_msg[1] = '='; batt_msg[2] = batt_power_1; batt_msg[3] = batt_power_2;  //message for Battery Voltage is "B=battery_power"
    hip_msg[0] = ' '; hip_msg[1] = 'H'; hip_msg[2] = '='; hip_msg[3] = hip_fsm_state;  //message for Hip FSM State is " H=hip_fsm_state"
    foot_inner_msg[0] = ' '; foot_inner_msg[1] = 'I'; foot_inner_msg[2] = '='; foot_inner_msg[3] = foot_inner_state;  //message for Inner Foot State is " I=foot_inner_state"
    foot_outer_msg[0] = ' '; foot_outer_msg[1] = 'O'; foot_outer_msg[2] = '='; foot_outer_msg[3] = foot_outer_state;  //message for Outer Foot State is " O=foot_outer_state"

  	//LCD display
  	if(((mb_get_timestamp() % lcd_display_period) < (lcd_display_period / 2))
  	&& (error_flag == 1)){ 
      if((mb_get_timestamp() % lcd_update_period) < 1){
	  		set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000001); //turn on buzzer
    		set_UI_LCD(error_code[0], 1);
    		set_UI_LCD(error_code[1], 2);
    		set_UI_LCD(error_code[2], 3);
    		set_UI_LCD(error_code[3], 4);
      }
  	}
  	else{  //display mode message
  		//message for Walk Mode
      if((mb_get_timestamp() % lcd_update_period) < 2){
	  		set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000000);  //turn off buzzer
    		set_UI_LCD(batt_msg, 1);  //display Battery Voltage on Quad 1
    		set_UI_LCD(hip_msg, 2);  //display Hip FSM State on Quad 2
    		set_UI_LCD(foot_inner_msg, 3);  //display Inner Foot State on Quad 3
    		set_UI_LCD(foot_outer_msg, 4);  //display Outer Foot State on Quad 4
      }
  	}
  
  	return 1;
 }


int ACTION_UI_rc_walk_exit(void)
{ 
  	motors_off(); //Turn motors off.   
    return 1;
}


int ACTION_UI_standby_entry(void)
{ 
   
  	set_io_float(ID_FSM_RESET,1); //Turn reset flag on
    all_fsm_run(); //Run fsm once
    
    //Top fsm is started in normal walk mode
    top_fsm->run();
    set_io_float(ID_MB_TOP_FSM_STATE, (float)g_top_fsm_state);    
    
    
    
    return 1;
}



int ACTION_UI_standby(void)
{  
	int i_step_num, i_total_dist;
	char step_num[6], total_distance[6];	
	char step_msg1[4], step_msg2[4], dist_msg1[4], dist_msg2[4];


   //********* Put standby actions here *******//
    
     motors_off(); //Turn motors off
    
  // ************************************* // 
    
    
	  //initialize LED	
    clear_UI_LED();
    //set LED
  	set_UI_LED(3, 'b');
		
  	//detect error
  	if(detect_error() == 1){	//there's an error
  		error_flag = 1;
  		set_UI_LED(4, 'r');	 //turn on the red LED
  	}
	else{
		  error_flag = 0;  //set error flag
  		set_UI_LED(4, '-');	 //turns off LED #4
	}

	//get information of Step Number and Total Distance
	i_step_num = (int)get_io_float(ID_E_STEP_NO);
	i_total_dist = (int)get_io_float(ID_E_TOTAL_DISTANCE);
  step_num[5] = int2ascii(i_step_num / 100000); i_step_num = i_step_num % 100000;
  step_num[4] = int2ascii(i_step_num / 10000); i_step_num = i_step_num % 10000;
  step_num[3] = int2ascii(i_step_num / 1000); i_step_num = i_step_num % 1000;
  step_num[2] = int2ascii(i_step_num / 100); i_step_num = i_step_num % 100;
  step_num[1] = int2ascii(i_step_num / 10); i_step_num = i_step_num % 10;
  step_num[0] = int2ascii(i_step_num);
  total_distance[5] = int2ascii(i_total_dist / 100000); i_total_dist = i_total_dist % 100000;
  total_distance[4] = int2ascii(i_total_dist / 10000); i_total_dist = i_total_dist % 10000;
  total_distance[3] = int2ascii(i_total_dist / 1000); i_total_dist = i_total_dist % 1000;
  total_distance[2] = int2ascii(i_total_dist / 100); i_total_dist = i_total_dist % 100;
  total_distance[1] = int2ascii(i_total_dist / 10); i_total_dist = i_total_dist % 10;
  total_distance[0] = int2ascii(i_total_dist);

	//form messages to display on LCD
	step_msg1[0] = 'S'; step_msg1[1] = ':'; step_msg1[2] = step_num[5]; step_msg1[3] = step_num[4];
	step_msg2[0] = step_num[3]; step_msg2[1] = step_num[2]; step_msg2[2] = step_num[1]; step_msg2[3] = step_num[0];
	dist_msg1[0] = 'D'; dist_msg1[1] = ':'; dist_msg1[2] = total_distance[5]; dist_msg1[3] = total_distance[4];
	dist_msg2[0] = total_distance[3]; dist_msg2[1] = total_distance[2]; dist_msg2[2] = total_distance[1]; dist_msg2[3] = total_distance[0]; 

  	if(((mb_get_timestamp() % (int)(lcd_display_period * 1.5)) < (lcd_display_period / 2))
  	&& (error_flag == 1)){
      if((mb_get_timestamp() % lcd_update_period) < 1){
	  		set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000001);  //turn on buzzer
    		set_UI_LCD(error_code[0], 1);
    		set_UI_LCD(error_code[1], 2);
    		set_UI_LCD(error_code[2], 3);
    		set_UI_LCD(error_code[3], 4);
      }
  	}
  	else{  //display mode message
		set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000000);  //turn off buzzer
		if((mb_get_timestamp() % lcd_display_period) < (lcd_display_period / 2)){
	      if((mb_get_timestamp() % lcd_update_period) < 1){
	    		clear_UI_LCD(2);
	    		//message for Standby Mode
          set_UI_LCD("M: S", 1);
				//Display Step Number on quad3 and quad4
	    		set_UI_LCD(step_msg1, 3);
				set_UI_LCD(step_msg2, 4);
	      }
		}
		else{
			if((mb_get_timestamp() % lcd_update_period) < 1){
	    		clear_UI_LCD(2);
	    		//message for Standby Mode
          set_UI_LCD("M: S", 1);
				//Display Total Distance on quad3 and quad4
	    		set_UI_LCD(dist_msg1, 3);
				set_UI_LCD(dist_msg2, 4);
	    	}
		}
  	}
  
  	return 1;
}


int ACTION_UI_standby_exit(void)
{ 
   
  	set_io_float(ID_FSM_RESET,0); //Stop the reset of fsm's
    
    return 1;
}


int ACTION_UI_stop(void)
{

  return 1;
}

/**
  Detect error reads in the error, if one has occurred, and
  loads the associated error message into the error_code 2D array.
  Returns 1 if there is a new error, 0 otherwise.
*/
int detect_error(void){
  int error_id = -1;
  char* error_message;
  int i;
  
  //get error id, and mark that we've read it
  if (!data_was_read(ID_ERROR_LCD, LCD)){
    error_id = get_io_ul(ID_ERROR_LCD);
    mark_as_read(ID_ERROR_LCD, LCD);
  }
  
  if (error_id >= 0 && error_id < ERROR_LAST_ID){//error occurred
    error_message = mb_error_messages[error_id];
    i = 0;
    while (i < 16){
      error_code[i/4][i%4] = 0; //reset all to zeros
      i++;
    }
    i = 0;
    while (error_message[i]!= '\0' && i < 14){
      error_code[i/4][i%4] = error_message[i];
      i++;
    }
    return 1;
  }
  
	return 0;
}

char int2ascii(int num){
  //Convert Int to Char
  return (num + '0');  
}

void clear_UI_LCD(int quad_num){  
  //clear the specified LCD quad
  set_io_ul((ID_UI_SET_LCD_QUAD_1 + quad_num - 1), 0x20202020);
}

void clear_UI_LED(){
  //Turn off all the LEDs
  set_io_ul(ID_UI_SET_LED_1, 0x000000);
  set_io_ul(ID_UI_SET_LED_2, 0x000000);
  set_io_ul(ID_UI_SET_LED_3, 0x000000);
  set_io_ul(ID_UI_SET_LED_4, 0x000000);
  set_io_ul(ID_UI_SET_LED_5, 0x000000);
  set_io_ul(ID_UI_SET_LED_6, 0x000000);
}

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
  set_io_ul((ID_UI_SET_LCD_QUAD_1 + quad_number - 1), message_data);
}

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
    case 'o':  //ORANGE
      val_color = 0x008160;  //set color to orange
	}
	set_io_ul((ID_UI_SET_LED_1 + led_number - 1), val_color);
}
