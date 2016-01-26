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



//  Hold the hip FSM in the initial condition by ignoring all transitions

//  //Define local variables to store the foot contact informationi
//  bool Outer_Foot=(get_io_float(ID_MCFO_LEFT_HEEL_SENSE)+get_io_float(ID_MCFO_RIGHT_HEEL_SENSE) > 2*get_io_float(ID_P_R_CONTACT_THRESHOLD));
//  bool Inner_Foot=(get_io_float(ID_MCFI_LEFT_HEEL_SENSE)+get_io_float(ID_MCFI_RIGHT_HEEL_SENSE) > 2*get_io_float(ID_P_R_CONTACT_THRESHOLD));
//  
//  if( (Outer_Foot && Inner_Foot) || (hip_counter_free > 200) )
//      {
//        //both feet are on the ground or max allowable swing time reached; hold the current hip position 
//        sensor_array[COND_rock_HIP_free_to_hold]=1;
//      }
//    
//  if( ((Outer_Foot && !Inner_Foot) ||  (!Outer_Foot && Inner_Foot)) && (hip_counter_hold > 500) )
//      {
//        // Exactly one foot is in the air; allow the hip to swing freely
//        sensor_array[COND_rock_HIP_hold_to_free]=1;
//      } 
//      
//  if( ((Outer_Foot && !Inner_Foot) ||  (!Outer_Foot && Inner_Foot)) && (hip_counter_set > 5000) )
//      {
//        // Exactly one foot is in the air; allow the hip to swing freely
//        sensor_array[COND_rock_HIP_set_to_free]=1;
//      } 
//    
//  if( !Outer_Foot && !Inner_Foot )
//      {
//        //both feet are in the air; set position to reset
//        sensor_array[COND_rock_HIP_free_to_set]=1;
//      }    
//      
//          
//  if( !Outer_Foot && !Inner_Foot )
//      {
//        //both feet are in the air; set position to reset
//        sensor_array[COND_rock_HIP_hold_to_set]=1;
//      }    
//        
        

}
