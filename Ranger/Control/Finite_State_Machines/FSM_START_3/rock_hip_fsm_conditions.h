extern float hip_counter_hold;
extern float hip_counter_set;
extern float hip_counter_free;

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

    //run through every condition to check if it is true
    
    if ( detect_UI_button_input(4) )
      {
        sensor_array[COND_rock_HIP_hold_to_set]=1;
      }  
    
    if ( hip_counter_hold > 2000 )
      {
        sensor_array[COND_rock_HIP_hold_to_free]=1;
      } 
    
    if ( hip_counter_set > 1500 )
      {
        sensor_array[COND_rock_HIP_set_to_free]=1;
      }  
    
    if ( get_io_float(ID_MCH_ANGLE) > 0.15 )
      {
        sensor_array[COND_rock_HIP_free_to_hold]=1;
      }
    
    if ( hip_counter_free > 1200 )
      {
        sensor_array[COND_rock_HIP_free_to_set]=1;
      }      

}
