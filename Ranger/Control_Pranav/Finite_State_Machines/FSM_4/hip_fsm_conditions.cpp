#include <mb_includes.h>
#include "hip_fsm_conditions.h"
#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;
extern int counter_ehold2swing;
extern int counter_swing;
extern int counter_free;
const static int time_ehold2swing = 2000;
const static int time_swing2free = 4000;

//UI buttons
//0 is calibrate, 1 is walk, 2 is standby
bool detect_UI_button_input(int button_num){	 
	int buttons;
	buttons = get_io_ul(ID_UI_BUTTONS);
  return (buttons & (1<<button_num));
}

void get_hip_sensor_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero


 
 if (g_hip_fsm_stop_command == 1)
{}
  
 else
 {


 /* if ((get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0) 
              && 
            (get_io_float(ID_MCH_ANGLE)<-0.2)) //outer heelstrike*/
 // if (detect_UI_button_input(0))  //button number of <calibrate> is 0
 if (0) //never
  {
    sensor_array[COND_H_free_to_HI_ehold] = 1; //Go to inner hold
  }
  
  //if (counter_ehold2swing > time_ehold2swing) //
 //if (detect_UI_button_input(1))  //button number of <walk> is 1
  if (0) //never
  {
     sensor_array[COND_HI_ehold_to_HI_swing] = 1;
    // sensor_array[COND_HI_ehold_to_H_free] = 1;
  }
  
   // if (counter_swing > time_swing2free) //
 //if (detect_UI_button_input(2))  //button number of <standby> is 2
 if( counter_swing > (int) get_io_float(ID_H_TEST4)  )
  {
    sensor_array[COND_HI_swing_to_HI_free] = 1;
  }
    
    if(0)
  //if( counter_free > (int) get_io_float(ID_H_TEST3)  )
  {
    sensor_array[COND_HI_free_to_HI_swing] = 1;
  }
    
  /******** HO not used **************/
  /*if ((get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0) 
        && 
        (get_io_float(ID_MCH_ANGLE)>0.2)) //inner heelstrike*/
  if(0) //never      
  {
    sensor_array[COND_H_free_to_HO_ehold] = 1; //Go to outer hold
  }
  
  // if (counter_ehold2swing > time_ehold2swing) //
  if(0) //never 
  {
    sensor_array[COND_HO_ehold_to_HO_swing] = 1;
  }
  
  //if (counter_swing > time_swing2free) //
  if(0) //never 
  {
    sensor_array[COND_HO_swing_to_HO_free] = 1;
  }
   /******** HO not used **************/
   
   
 // if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
   if (0)
  {
    sensor_array[COND_H_stop] = 1;
  }
  
 }


}

