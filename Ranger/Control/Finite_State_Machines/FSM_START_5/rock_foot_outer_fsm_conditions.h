extern float fo_counter_level;
extern float fo_counter_push;

enum states 
{
  STATE_rock_FO_level, 
  STATE_rock_FO_push, 
};

// define the inputs
enum sensor 
{
  COND_rock_FO_level_to_push,
  COND_rock_FO_push_to_level,
};


void get_rock_foot_outer_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero
// 
//// ignore complicated conditions for now
// 
// //If the robot is hanging in the air, then the sum of the contact sensors should be less than 1336
//bool Floating_Robot=( (get_io_float(ID_MCFO_LEFT_HEEL_SENSE)+get_io_float(ID_MCFO_RIGHT_HEEL_SENSE)+get_io_float(ID_MCFI_LEFT_HEEL_SENSE)+get_io_float(ID_MCFI_RIGHT_HEEL_SENSE)) < 1400 );
////Only check transitions if the robot is NOT floating
//
//  if(!Floating_Robot)
//  {
//     //Check That both feet are on the ground before pushing off
//    if( get_io_float(ID_MCFI_LEFT_HEEL_SENSE)+get_io_float(ID_MCFI_RIGHT_HEEL_SENSE) > 2*get_io_float(ID_P_R_CONTACT_THRESHOLD)
//          &&
//        get_io_float(ID_MCFO_LEFT_HEEL_SENSE)+get_io_float(ID_MCFO_RIGHT_HEEL_SENSE) > 2*get_io_float(ID_P_R_CONTACT_THRESHOLD) )
//      {
//          if ( fo_counter_level >= 0.5*get_io_float(ID_P_R_ROCK_PERIOD) )
//            {
//              sensor_array[COND_rock_FO_level_to_push] = 1;
//            }
//      }
//      
//      //Ignore the contact sensors when in the push state because they don't properly register  
//          if ( fo_counter_push >= 0.5*get_io_float(ID_P_R_ROCK_PERIOD) )
//            {
//             sensor_array[COND_rock_FO_push_to_level] = 1;
//            }
//  
//  }


          if ( fo_counter_push >= 0.5*get_io_float(ID_P_R_ROCK_PERIOD) )
            {
             sensor_array[COND_rock_FO_push_to_level] = 1;
            }
          if ( fo_counter_level >= 0.5*get_io_float(ID_P_R_ROCK_PERIOD) )
            {
              sensor_array[COND_rock_FO_level_to_push] = 1;
            }


}
