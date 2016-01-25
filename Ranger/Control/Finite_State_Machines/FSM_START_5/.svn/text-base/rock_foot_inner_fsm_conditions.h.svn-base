extern float fi_counter_level;
extern float fi_counter_push;
extern float fi_counter_clear;

bool detect_UI_button_input(int button_num);

enum states 
{
  STATE_rock_FI_level, 
  STATE_rock_FI_push,
  STATE_rock_FI_clear,
};

// define the inputs
enum sensor 
{
  COND_rock_FI_level_to_push,
  COND_rock_FI_push_to_level,
  COND_rock_FI_push_to_clear,
  COND_rock_FI_clear_to_level,
};


void get_rock_foot_inner_input(int* sensor_array, int sensor_array_length)
{
    //Initialize all sensor values to false 
    int num_sensors = sensor_array_length;
    
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero

//
//// Ignore the complicated transitions for now
//
//    
//    //If the robot is hanging in the air, then the sum of the contact sensors should be less than 1336
//    bool Floating_Robot=( (get_io_float(ID_MCFO_LEFT_HEEL_SENSE)+get_io_float(ID_MCFO_RIGHT_HEEL_SENSE)+get_io_float(ID_MCFI_LEFT_HEEL_SENSE)+get_io_float(ID_MCFI_RIGHT_HEEL_SENSE)) < 1400 );
//    //Only check transitions if the robot is NOT floating
//   
//   if(!Floating_Robot)
//   {
//  //Check That both feet are on the ground before pushing off
//    if( get_io_float(ID_MCFI_LEFT_HEEL_SENSE)+get_io_float(ID_MCFI_RIGHT_HEEL_SENSE) > 2*get_io_float(ID_P_R_CONTACT_THRESHOLD)
//      &&
//        get_io_float(ID_MCFO_LEFT_HEEL_SENSE)+get_io_float(ID_MCFO_RIGHT_HEEL_SENSE) > 2*get_io_float(ID_P_R_CONTACT_THRESHOLD) )
//      {  
//        if ( fi_counter_level > 0.5*get_io_float(ID_P_R_ROCK_PERIOD) )
//          {
//            sensor_array[COND_rock_FI_level_to_push] = 1;
//          }
//      }
//  
//    //Sensor doesn't register when the foot is extended in the push state, so ignore it.
//    if ( fi_counter_push > 0.5*get_io_float(ID_P_R_ROCK_PERIOD) )
//      {
//       sensor_array[COND_rock_FI_push_to_level] = 1;
//      }
//    }

  if ( fi_counter_level > 0.5*get_io_float(ID_P_R_ROCK_PERIOD) )
          {
            sensor_array[COND_rock_FI_level_to_push] = 1;
          }
          
  if ( fi_counter_push > 0.5*get_io_float(ID_P_R_ROCK_PERIOD) )
      {
       sensor_array[COND_rock_FI_push_to_level] = 1;
      }


////For now, don't need the clear option      
//    if ( fi_counter_push > 600 )
//      {
//       sensor_array[COND_rock_FI_push_to_clear] = 1;
//      }
//
//    if ( fi_counter_clear > 1000 )
//      {
//       sensor_array[COND_rock_FI_clear_to_level] = 1;
//      }

}

