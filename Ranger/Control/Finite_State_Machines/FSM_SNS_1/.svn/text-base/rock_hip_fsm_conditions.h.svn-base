extern float hip_counter_hold;
extern float hip_counter_set;
extern float hip_counter_free;
extern float hip_hold_angle;

bool detect_UI_button_input(int button_num);

enum states 
{
  STATE_rock_HIP_hold,
  STATE_rock_HIP_set,
  STATE_rock_HIP_free,
};


// define the inputs
//enum conditions
enum sensor 
{
  COND_rock_HIP_hold_to_set,
  COND_rock_HIP_hold_to_free,
  COND_rock_HIP_set_to_free,
  COND_rock_HIP_free_to_hold,
  COND_rock_HIP_free_to_set,
};



void get_rock_hip_sensor_input(int* sensor_array, int sensor_array_length)
{
    //Initialize the condition array to be false
    int num_sensors = sensor_array_length;
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero    
      

  //both feet are off the ground
  bool Floating_Robot=( (get_io_float(ID_MCFO_LEFT_HEEL_SENSE)+get_io_float(ID_MCFO_RIGHT_HEEL_SENSE)+get_io_float(ID_MCFI_LEFT_HEEL_SENSE)+get_io_float(ID_MCFI_RIGHT_HEEL_SENSE)) < 1400 );
  static float hold_time = (get_io_float(ID_P_R_WAIT_TIME)+get_io_float(ID_P_R_FO_PUSH_TIME)+get_io_float(ID_P_R_FI_PUSH_TIME));
  
  //both feet are on the ground and wait time reached; hold the current hip position
  if( !Floating_Robot && (get_io_float(ID_P_R_ROCK_TIMER) >= get_io_float(ID_P_R_WAIT_TIME)) )
      { 
        sensor_array[COND_rock_HIP_set_to_free] = 1;
      }
  if( !Floating_Robot && (get_io_float(ID_P_R_ROCK_TIMER) < hold_time) )
      { 
        sensor_array[COND_rock_HIP_free_to_hold] = 1;
      }

  // after hold_time, allow the hip to swing freely
  if ( !Floating_Robot && (get_io_float(ID_P_R_ROCK_TIMER)> hold_time) )
      { 
          sensor_array[COND_rock_HIP_hold_to_free] = 1;
      }  
  
   
      
  //both feet are off the ground; reset hip position    
  if( Floating_Robot )
      {
        sensor_array[COND_rock_HIP_free_to_set]=1;
      }    
          
  if( Floating_Robot )
      {
        sensor_array[COND_rock_HIP_hold_to_set]=1;
      } 
   
        
        

}
