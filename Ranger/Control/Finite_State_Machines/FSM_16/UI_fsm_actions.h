#include "error_messages.h"
#include <string.h>

int counter; 

char batt_msg2[4];
int batt_volt, batt_volt1, batt_volt2, batt_volt3; 
char batt_digit1, batt_digit2, batt_digit3;  
  
int i_step_num, i_total_dist;
char step_msg1[4], step_msg2[4], dist_msg1[4], dist_msg2[4];

int step_num1, step_num2, step_num3, step_num4, step_num5, step_num6;
int dist_num1, dist_num2, dist_num3, dist_num4, dist_num5, dist_num6;
char step_digit[6],  dist_digit[6];

const int MB_LOOP_TIME = 2; //MB runs once every 2 ms
const int DISPLAY_TIME = 4; //LCD display time in sec

const int song_step_interval = 3000; //approx 3000 steps is 1050 m, 600 steps is 210 m

enum nav_controls {
    SIGNAL_RC_rightstick = ID_UI_RC_0,    // RC, right joystick; its left-right position determines steering angle
    SIGNAL_RC_leftstick = ID_UI_RC_1,     // RC, left joystick; down position = stop, up position = walk;
    SIGNAL_RC_switch = ID_UI_RC_2,        // RC, switch; position towards user = RC has controls, position away from user = camera board has controls
    SIGNAL_CAM = ID_UI_RC_3       // signal from the camera board
};

int cam_emerg_counter;    // counts the number of calls to UI_fsm while emergency signal from the camera board is on
bool cam_emerg_on = 0;    // says whether emergency signal from the camera is on; used to do initial actions when starting to receive it
const int cam_emerg_lim = 2500;     // number of iterations of UI_fsm before stopping, when emergency signal from the camera is on
                                      // corresponding time interval = cam_emerg_lim*MB_LOOP_TIME = 2*cam_emerg_lim ms
const int cam_emerg_freq = 50;     // frequency of LEDs blinking, when emergency signal from the camera is on
                                      // corresponding time interval = cam_emerg_freq*MB_LOOP_TIME = 2*cam_emerg_freq ms


// temp 
extern int fo_flag_skip_prepush;
extern int fi_flag_skip_prepush;

extern fsm* hip_fsm;
extern fsm* foot_inner_fsm;
extern fsm* foot_outer_fsm;
extern fsm* steering_fsm;
extern fsm* leg_fsm;


//Function declaration 
//int detect_error(void);
void clear_UI_LCD(int quad_num);
void clear_UI_LED();
void set_UI_LCD(char * message, int quad_number);
void set_UI_LED(int led_number, char color);
void update_UI_LED(void);
char int2ascii(int num);

void cam_emergency(void)
// put here everything to be done when the camera sends emergency signal;
// keeps the emergency time counter; blinks all LEDs red
{
    static int blink_counter = 0;
    
    // update time counter for camera emergency
    if (!cam_emerg_on) {     // if camera just started sending emergency signal
        cam_emerg_on = 1;
        cam_emerg_counter = 0;    // start counting for how long the emergency signal has been on
    }else {
        cam_emerg_counter++;
        if (cam_emerg_counter >= cam_emerg_lim) {     // if the camera has been sending emergency signal for long enough, stop the robot
            set_io_float(ID_NAV_WALK, 0.0);
//            set_io_float(ID_NAV_CAM_STEER, 0.0);
        }
    }
    
    // blink all LEDs red with the frequency = cam_emerg_freq
    blink_counter++;
    if (blink_counter < cam_emerg_freq) {
        set_UI_LED(1, 'r');   set_UI_LED(2, 'r');   set_UI_LED(3, 'r');
        set_UI_LED(4, 'r');   set_UI_LED(5, 'r');   set_UI_LED(6, 'r');
    }else if (blink_counter < 2*cam_emerg_freq) {
        clear_UI_LED();
    }else {
        blink_counter = 0;
    }
}

void get_nav_signal(void)
// reads the signals from RC and the Camera Board, normalizes them, and updates the corresponding can_id's
// all signals are listed in nav_controls enum list
{
    // constants a, b are used to project raw RC signals onto interval [-1,1]:
    // norm_signal = a*raw_signal + b
    // the raw signals are expected to be in the range 66000-114000
    //float a = 0.00004167, b = -3.75;     // for the range 66000-114000
    float a = 0.00004348, b = -3.913;    // these coefficients assume the signals are in the range 67000-113000, and are exact for this range
                                         // a smaller range is used to make sure extreme commands (e.g. max steering) are given when they are intended to be
    float nav_command;
    
    ///// RC signals /////
    // the switch between RC and camera: -1 = RC is in control, 1 = camera is in control
    nav_command = a*get_io_float(SIGNAL_RC_switch) + b;    // get normalized RC-is-in-control command from RC
    if (nav_command < -0.5) {    // switch is towards user => RC is in control; update all other signals from RC
        set_io_float(ID_NAV_CAM_USED, 0.0);
    }else if (nav_command > 0.5) {    // switch is away from user => RC is not in control, and camera board is used
        set_io_float(ID_NAV_CAM_USED, 1.0); }
        
    // right joystick of the RC, left-right position
    nav_command = a*get_io_float(SIGNAL_RC_rightstick) + b;    // get normalized steering command from RC
                                                               // nav_command normally should be in [-1,1]
                                                               
/////////////////// 4/21/2014, changed by petr to default steering straight in case of extreme signals from rc
    if ((nav_command<-1.5) || (nav_command>1.5)) {    // if RC gives extreme signals (e.g. when turned off), go straight
        nav_command = 0.0; }
///////////////////

    set_io_float(ID_NAV_RC_STEER, nav_command);   // -1 = max right position, 1 = max left position
    
    // left joystick from the RC, up-down position; read it only if RC is in command
    if (!(int)get_io_float(ID_NAV_CAM_USED)) {
        nav_command = a*get_io_float(SIGNAL_RC_leftstick) + b;    // get normalized walk/stop command from RC
        if (nav_command < -0.9) {    // left joystick is down => stop
            set_io_float(ID_NAV_WALK, 0.0); }
        else if (nav_command > 0.9) {    // left joystick is up => walk
            set_io_float(ID_NAV_WALK, 1.0); }
    }
    
    
    ///// camera board signal /////
    nav_command = a*get_io_float(SIGNAL_CAM) + b;    // get normalized steering command from the camera board
    if (nav_command < -1.5) {     // emergency signal from the camera board; do not update the steering command in this case
        set_io_float(ID_NAV_CAM_EMERG, 1.0); }
    else {    // no emergency signal from the camera board
        set_io_float(ID_NAV_CAM_EMERG, 0.0);
        set_io_float(ID_NAV_CAM_STEER, -nav_command);    // minus sign due to camera signal assuming biggest signal value corresponds to max right
                                                        // (as opposed to max left for the RC command)
    }
}

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
  
  set_io_float(ID_MCSI_COMMAND_ANG, 0.0);
  set_io_float(ID_MCSI_PROP_COEFF, 0.0);
  set_io_float(ID_MCSI_INT_COEFF, 0.0);
  
//  set_io_ul(ID_MCFO_SHUTDOWN,1);
//  set_io_ul(ID_MCSI_SHUTDOWN,1);
//  set_io_ul(ID_MCH_SHUTDOWN,1);
//  set_io_ul(ID_MCFI_SHUTDOWN,1);
}

void all_fsm_run(void)
{
    //Note leg fsm should ideally be called before hip fsm
    leg_fsm->run();
    if ( (int) get_io_float(ID_MB_LEG_FSM_STATE)!= g_leg_fsm_state)
      set_io_float(ID_MB_LEG_FSM_STATE, (float)g_leg_fsm_state);
      
    hip_fsm->run();
    if ( (int) get_io_float(ID_MB_HIP_FSM_STATE)!= g_hip_fsm_state)
      set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);
   
    foot_inner_fsm->run();
    if ( (int) get_io_float(ID_MB_FOOT_INNER_FSM_STATE) != g_foot_inner_fsm_state)  
      set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);
    
    foot_outer_fsm->run();
    if ( (int)get_io_float(ID_MB_FOOT_OUTER_FSM_STATE)!= g_foot_outer_fsm_state )
      set_io_float(ID_MB_FOOT_OUTER_FSM_STATE, (float)g_foot_outer_fsm_state);
    
    steering_fsm->run();
    if ( (int)get_io_float(ID_MB_STEERING_FSM_STATE) != g_steering_fsm_state)
      set_io_float(ID_MB_STEERING_FSM_STATE, (float)g_steering_fsm_state);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int ACTION_UI_calibrate_entry(void)
{   
  set_io_float(ID_FSM_RESET,0); //Turn reset flag on
    clear_UI_LCD(1);
    clear_UI_LCD(2);
    
    clear_UI_LED();
	
  return 1;
}

int ACTION_UI_calibrate(void)
{   		         
    //********* Put calibrate actions here *******//   
    //Currently no calibration routines  
    // ************************************* // 
     
  	//initialize LED	
    clear_UI_LED();
    //set LED
    set_UI_LED(1, 'c');  //Set LED #1 to PURPLE
    
//    set_UI_LED(1, 'g');  //Set LED #1 to PURPLE
//    set_UI_LED(2, 'y');  //Set LED #1 to PURPLE
//    set_UI_LED(3, 'o');  //Set LED #1 to PURPLE
//    set_UI_LED(5, 'm');  //Set LED #1 to PURPLE
//    set_UI_LED(6, 'c');  //Set LED #1 to PURPLE
        
    //Set LCD
    clear_UI_LCD(2);
    set_UI_LCD("M: C", 1);
    
    
    
    // test code to find flip up/flip down max. speed // Petr, 6/28/2013
    get_nav_signal();
    
    static float phase = 3.0;
    if ((int)get_io_float(ID_NAV_WALK) && (phase > 2.5)){ phase = 0.0; }
    else if (!(int)get_io_float(ID_NAV_WALK)){ phase = 3.0; }
    
    float target_angle;
    
    if (phase < 0.5) {
      target_angle = 0.2; 
  
      set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
      set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
      float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
      set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
        
      if (get_io_float(ID_MCFI_MID_ANKLE_ANGLE) < get_io_float(ID_F_TEST3)) { phase = 1.0; }
    }else if ((phase>0.5) && (phase<1.5)){
      target_angle =  2.0; 
        
      set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
      set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
      float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
      set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
        
      if (get_io_float(ID_MCFI_MID_ANKLE_ANGLE) > get_io_float(ID_F_TEST4)) { phase = 2.0; }
    }
    // end of test code
    
    
    
  	return 1;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int ACTION_UI_walk_entry(void)
{ 
   
  	set_io_float(ID_FSM_RESET,2); //Stop the reset of fsm's
    
    clear_UI_LCD(1);
    clear_UI_LCD(2);
    
    clear_UI_LED();
    
    counter = 0;   
    
    set_io_float(ID_E_MIDSTANCE_LEGRATE, 0.5); //put some reasonable value for mid-stance leg-rate
                                               // else walking from falls get screwed up. 
                                               
    set_io_float(ID_E_SWING_LEG,1);
    
    
    return 1;
}


int ACTION_UI_walk(void)
{
      
    //********* Put walk actions here *******//
    get_nav_signal();     // read all navigation signals from RC, camera
    
    all_fsm_run();   
    // ************************************* // 
    
    
    // Play the song and stop it based on steps covered
    int step_no = (int)get_io_float(ID_E_STEP_NO);
    
    if ( (step_no % song_step_interval == 0) && step_no!=0 ) //To prevent playing at step 0
      set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000001);  //start playing the song
    else
      set_io_ul(ID_UI_SET_BUZZER_AMPL, 0x000000);  //stop playing the song
  
    
    // update all LEDs (except for cam emergency signal blinking)
    update_UI_LED();
    

    // if camera is in control and sends emergency signal
    if ((int)get_io_float(ID_NAV_CAM_EMERG) && (int)get_io_float(ID_NAV_CAM_USED)) {
          cam_emergency();
    }else {
          cam_emerg_on = 0;
    }

   
    if(counter==0)
       {
        batt_volt = (int)(get_io_float(ID_MCH_BATT_VOLTAGE)*10); //Get number to three places
        batt_volt1 =(int)(batt_volt*0.01); //Get first digit
        batt_volt2 = (int) ((batt_volt - batt_volt1*100)*0.1); //Get second digit
        batt_volt3 = (int) ( batt_volt - batt_volt1*100 - batt_volt2*10); //Get digit after decimal
      
        batt_digit1 = int2ascii(batt_volt1);
        batt_digit2 = int2ascii(batt_volt2); 
        batt_digit3 = int2ascii(batt_volt3);  
      
        //batt_msg1[0] = 'B'; batt_msg1[1] = 'a'; batt_msg1[2] = 't'; batt_msg1[3] = 't';
        batt_msg2[0] = batt_digit1; batt_msg2[1] = batt_digit2; batt_msg2[2] = '.'; batt_msg2[3] = batt_digit3;  //message for Battery Voltage is "B=battery_power"
  
        clear_UI_LCD(2);
        //set_UI_LCD(batt_msg1,1); 
    		set_UI_LCD(batt_msg2, 1);  //display Battery Voltage on Quad 1
        
        counter = counter + MB_LOOP_TIME;
       }
    else if (counter==DISPLAY_TIME*1000) 
        {     
        i_step_num = (int)get_io_float(ID_E_STEP_NO);
        
        step_num1 = (int)(i_step_num*0.00001);
        step_num2 = (int)(i_step_num*0.0001) - step_num1*10; 
        step_num3 = (int)(i_step_num*0.001) - step_num1*100-step_num2*10; 
        step_num4 = (int)(i_step_num*0.01) - step_num1*1000-step_num2*100-step_num3*10;
        step_num5 = (int)(i_step_num*0.1) - step_num1*10000-step_num2*1000-step_num3*100-step_num4*10;
        step_num6 = i_step_num-step_num1*100000-step_num2*10000-step_num3*1000-step_num4*100-step_num5*10;
         
        step_digit[5] = int2ascii(step_num1); 
        step_digit[4] = int2ascii(step_num2); 
        step_digit[3] = int2ascii(step_num3); 
        step_digit[2] = int2ascii(step_num4); 
        step_digit[1] = int2ascii(step_num5); 
        step_digit[0] = int2ascii(step_num6); 
        
        step_msg1[0] = 'S'; step_msg1[1] = '='; 
        step_msg1[2] = step_digit[5]; step_msg1[3] = step_digit[4]; 
        
        step_msg2[0] = step_digit[3]; step_msg2[1] = step_digit[2]; 
        step_msg2[2] = step_digit[1]; step_msg2[3] = step_digit[0];
  
        set_UI_LCD(step_msg1, 1); //display step number on Quad 1 and 2
			  set_UI_LCD(step_msg2, 2);
        counter = counter + MB_LOOP_TIME;
        }
    else if (counter==2*DISPLAY_TIME*1000) 
	    {     
        i_total_dist = (int)get_io_float(ID_E_TOTAL_DISTANCE);
  
        dist_num1 = (int)(i_total_dist*0.00001);
        dist_num2 = (int)(i_total_dist*0.0001) - dist_num1*10; 
        dist_num3 = (int)(i_total_dist*0.001) - dist_num1*100-dist_num2*10; 
        dist_num4 = (int)(i_total_dist*0.01) - dist_num1*1000-dist_num2*100-dist_num3*10;
        dist_num5 = (int)(i_total_dist*0.1) - dist_num1*10000-dist_num2*1000-dist_num3*100-dist_num4*10;
        dist_num6 = i_total_dist-dist_num1*100000-dist_num2*10000-dist_num3*1000-dist_num4*100-dist_num5*10;
        
        dist_digit[5] = int2ascii(dist_num1); 
        dist_digit[4] = int2ascii(dist_num2); 
        dist_digit[3] = int2ascii(dist_num3); 
        dist_digit[2] = int2ascii(dist_num4); 
        dist_digit[1] = int2ascii(dist_num5); 
        dist_digit[0] = int2ascii(dist_num6); 
        
        dist_msg1[0] = 'D'; dist_msg1[1] = '='; 
        dist_msg1[2] = dist_digit[5]; dist_msg1[3] = dist_digit[4];
    	  dist_msg2[0] = dist_digit[3]; dist_msg2[1] = dist_digit[2]; 
        dist_msg2[2] = dist_digit[1]; dist_msg2[3] = dist_digit[0]; 
    
        set_UI_LCD(dist_msg1, 1); //display step number on Quad 1 and 2
			  set_UI_LCD(dist_msg2, 2);
        counter = counter + MB_LOOP_TIME;
	    }
    else if (counter>=3*DISPLAY_TIME*1000)
      {
	    	counter = 0;
      }
    else
      {
        counter = counter + MB_LOOP_TIME;
		  }
 
  	return 1;
}


int ACTION_UI_walk_exit(void)
{ 
       
  	motors_off(); //Turn motors off.
    set_io_float(ID_FSM_RESET,1); //Stop the reset of fsm's
    all_fsm_run(); //Run fsm once
    
    
    return 1;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int ACTION_UI_standby_entry(void)
{ 
   
  	//set_io_float(ID_FSM_RESET,1); //Turn reset flag on
    //all_fsm_run(); //Run fsm once   
    set_io_float(ID_FSM_RESET,0); //Turn reset flag on
    //clear_UI_LCD(1);
    //clear_UI_LCD(2);
    
    clear_UI_LED();
    
    return 1;
}



int ACTION_UI_standby(void)
{  
   //********* Put standby actions here *******//
    motors_off(); //Turn motors off
    
    //set LED
  	set_UI_LED(3, 'b');
    
    //set LCD
    clear_UI_LCD(2);
    set_UI_LCD("M: S", 1);
    
  	return 1;
}


int ACTION_UI_standby_exit(void)
{ 
   
  	//set_io_float(ID_FSM_RESET,0); //Stop the reset of fsm's   
    return 1;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int ACTION_UI_nogo_entry(void)
{
    clear_UI_LED();
    
    return 1;
}


int ACTION_UI_nogo(void)
{
    get_nav_signal();     // read navigation signals from RC, camera
    
    all_fsm_run();
    
    
    update_UI_LED();     // set navigation LEDs
    
    return 1;
}


int ACTION_UI_nogo_exit(void)
{
    return 1;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int ACTION_UI_stop(void)
{

  return 1;
}

/**
  Detect error reads in the error, if one has occurred, and
  loads the associated error message into the error_code 2D array.
  Returns 1 if there is a new error, 0 otherwise.
*/
/*int detect_error(void){
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
}*/

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
    case 'o':  //ORANGE //Looks like cyan (light blue)
      val_color = 0x008160;  //set color to orange  
    case 'm':  //MAGENTA //Looks like cyan (light blue)
      val_color = 0x800080;  //set color to magenta 
    case 'c':  //CYAN //Light blue
      val_color = 0x808000;  //set color to cyan     
      
	}
	set_io_ul((ID_UI_SET_LED_1 + led_number - 1), val_color);
}

void update_UI_LED(void)
{
    //Errors 
    if ( get_io_float(ID_E_MIDSTANCE_LEGRATE) < get_io_float(ID_P_H_EH_L_TRATE) ) //If robot is going slow
         set_UI_LED(2, 'r'); //set red light
    else                      
          set_UI_LED(2, '-'); //turn off light
    
    int PP_missed = (int) get_io_float(ID_E_F_PP_MISSED);
    if ( PP_missed == 0  ) //outer (feet prepush?) missed: blue + out = blout
           set_UI_LED(3, 'b'); 
    else if (PP_missed == 1) //inner (feet prepush?) missed  green + in = grin                    
           set_UI_LED(3, 'g');
    else
           set_UI_LED(3, '-');  
           
    int PP2HS_TIME = (int) get_io_float(ID_E_F_PP2HS_TIME);
    if ( (PP2HS_TIME > get_io_float(ID_P_UI_PP2HS_TTIME)) && ((int)get_io_float(ID_E_SWING_LEG) == 1)  ) //inner missed  green + in = grin 
           set_UI_LED(6, 'g'); 
    else if ((PP2HS_TIME > get_io_float(ID_P_UI_PP2HS_TTIME)) && ((int)get_io_float(ID_E_SWING_LEG) == 0) )   //outer missed: blue + out = blout                 
           set_UI_LED(6, 'b');
    else
           set_UI_LED(6, '-');  
    
    
    //Turn on led to show left (LED 5) and right (LED 1) turning. 
    //yellow is upto 50% turning of allowed and green is 50 % - 100 % turning of allowed
    float max_steer_angle = get_io_float(ID_P_S_MAX_STEER_ANG); //get_io_float(ID_P_S_ILST_S_MAXANG);
    if (get_io_float(ID_D_S_NULL_S_DANG)>0) //left turn
        {
        set_UI_LED(1, '-'); //turn off led 1  
        if (get_io_float(ID_D_S_NULL_S_DANG)<=0.5*max_steer_angle) //upto 50% steering
            set_UI_LED(5, 'y');
        else                        //more than 50% steering
            set_UI_LED(5, 'b'); 
        } 
    else if (get_io_float(ID_D_S_NULL_S_DANG)<0)//right turn
        {
        set_UI_LED(5, '-'); //turn off led 5
        if (-get_io_float(ID_D_S_NULL_S_DANG)<=0.5*max_steer_angle) //upto 50% steering
          set_UI_LED(1, 'y');
        else                      //more than 50% steering
          set_UI_LED(1, 'b');
        } 
    else    // turn LEDs off, if no steering    
        {
        set_UI_LED(1, '-'); //turn off led 1  
        set_UI_LED(5, '-'); //turn off led 5
        }   
      
    // led 4: blue when RC is used, green otherwise (camera is used)
    if (!(int)get_io_float(ID_NAV_CAM_USED))     // switch is towards user, RC is used
        { set_UI_LED(4, 'b'); }     // turn led 4 blue
    else    // switch is away from user, camera is used
        { set_UI_LED(4, 'g'); }     // turn led 4 green
}
