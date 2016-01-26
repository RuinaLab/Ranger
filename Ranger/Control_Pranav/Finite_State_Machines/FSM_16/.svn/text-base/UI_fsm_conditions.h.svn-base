//#include <mb_includes.h>
//#include "UI_fsm_conditions.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;

// define the states
enum states {STATE_UI_calibrate, 
             STATE_UI_standby, 
             STATE_UI_walk, 
             STATE_UI_stop,
             STATE_UI_nogo
             };
             
// define the inputs
enum conditions { COND_UI_standby_to_calibrate, 
                  COND_UI_standby_to_walk,
                  COND_UI_walk_to_standby, 
                  COND_UI_calibrate_to_walk, 
                  COND_UI_stop,
                  COND_UI_walk_to_nogo,
                  COND_UI_nogo_to_walk,
                  COND_UI_nogo_to_standby
                };
                
//enum rc_controls {
//    SIGNAL_RC_rightstick = ID_UI_RC_0,    // RC, right joystick; its left-right position determines steering angle
//    SIGNAL_RC_leftstick = ID_UI_RC_1,     // RC, left joystick; down position = stop, up position = walk;
//                                              // Jan 9, 2013: left joystick is used as camera simulator
//    SIGNAL_RC_switch = ID_UI_RC_2         // RC, switch; position towards user = RC has controls, position away from user = camera board has controls
//};
//
//// function declarations
//float get_RC_command(int chan)
//// reads the signal from RC, and transforms it to the interval [-1,1]
//{
//    // ID_UI_RC_0 = right joystick; ID_UI_RC_1 = left joystick; ID_UI_RC_2 = switch
//    return(0.0000417*get_io_float(chan)-3.75);    // all rc are in the same interval
//}
                
bool detect_UI_button_input(int button_num){	 
	int buttons;
	buttons = get_io_ul(ID_UI_BUTTONS);
  return (buttons & (1<<button_num));
}

void get_UI_conditions_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;

    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero

 if (0)
{}
  
 else
 {  

  
  if (detect_UI_button_input(0))  //button number of <calibrate> is 0
  {
    sensor_array[COND_UI_standby_to_calibrate] = 1;
  }
  
  
  if (detect_UI_button_input(1))  //button number of <walk> is 1
  {
    sensor_array[COND_UI_standby_to_walk] = 1;
  }
  
  if (detect_UI_button_input(2))  //button number of <standby> is 2
  {
    sensor_array[COND_UI_walk_to_standby] = 1;
  }
  
  
  if (detect_UI_button_input(1))  //button number of <walk> is 1
  {
    sensor_array[COND_UI_calibrate_to_walk] = 1;
  }
  
  if (0)//(!(int)get_io_float(ID_NAV_WALK))           // stop walking when ID_NAV_WALK becomes 0;
  {                                                 // this happens either due to the command from RC, or 
    sensor_array[COND_UI_walk_to_nogo] = 1;         // when emergency signal from camera is on for some fixed time
  }
  
  if ((int)get_io_float(ID_NAV_WALK))           // start walking if ID_NAV_WALK becomes 1
  {                                             // Feb/22/2013: this can happen only when RC is in control, and left joystick is up
    sensor_array[COND_UI_nogo_to_walk] = 1;
  }
  
  if (detect_UI_button_input(2))  //button number of <standby> is 2
  {
    sensor_array[COND_UI_nogo_to_standby] = 1;
  }
  
  if (0)
  {
    sensor_array[COND_UI_stop] = 0;
  }
  
 }

}

