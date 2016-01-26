#include <mb_includes.h>
#include "UI_fsm_conditions.h"
#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;

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

 if (g_ui_fsm_stop_command == 1)
//	{current_sensor_reading = stop_command;}
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
  
  if (0)
  {
    sensor_array[COND_UI_stop] = 0;
  }
  
 }

// cout << "->hip_fsm: getting sensor input " << current_sensor_reading << endl;




 //  sensor_array[current_sensor_reading] = 1;

}

