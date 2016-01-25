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
      
      

    //If the robot is hanging in the air, then the sum of the contact sensors should be less than 1336
    bool Floating_Robot=( (get_io_float(ID_MCFO_LEFT_HEEL_SENSE)+get_io_float(ID_MCFO_RIGHT_HEEL_SENSE)+get_io_float(ID_MCFI_LEFT_HEEL_SENSE)+get_io_float(ID_MCFI_RIGHT_HEEL_SENSE)) < 1400 );
      
    if ( get_io_float(ID_P_R_ROCK_TIMER)<get_io_float(ID_P_R_WAIT_TIME))
        {  
            // wait time has not been reached, set initial angle
            set_io_float(ID_P_R_FO_TANG, get_io_float(ID_P_R_FO_INIT_ANGLE));
            sensor_array[COND_rock_FO_push_to_level] = 1;
        }
//    if ( get_io_float(ID_P_R_ROCK_TIMER)>5000 && get_io_float(ID_P_R_ROCK_TIMER)<5800) 
    if ( get_io_float(ID_P_R_ROCK_TIMER)>get_io_float(ID_P_R_WAIT_TIME) && get_io_float(ID_P_R_ROCK_TIMER)<(get_io_float(ID_P_R_WAIT_TIME)+get_io_float(ID_P_R_FO_PUSH_TIME)))
        { 
            // wait time reached, reset target angle and push
            set_io_float(ID_P_R_FO_TANG, get_io_float(ID_P_R_FO_END_ANGLE));
            sensor_array[COND_rock_FO_level_to_push] = 1;
        }
//    if ( get_io_float(ID_P_R_ROCK_TIMER)>5800) 
     if ( get_io_float(ID_P_R_ROCK_TIMER)>(get_io_float(ID_P_R_WAIT_TIME)+get_io_float(ID_P_R_FO_PUSH_TIME)) )
        { 
            // outer foot remains level
            sensor_array[COND_rock_FO_push_to_level] = 1;
        }
        
        
        
        
//    //both feet are off the ground; set FI to level position    
//    if( Floating_Robot )
//      {
//        sensor_array[COND_rock_FO_push_to_level] = 1;
//      }    
//      
//    //robot is on the ground and maximum wait time has been reached  
//    if( !Floating_Robot && get_io_float(ID_P_R_ROCK_TIMER) >= get_io_float(ID_P_R_WAIT_TIME))
//      {
//        if ( fo_counter_level > fo_level_time )
//          //max wait time has been reached, FO push
//          {
//            sensor_array[COND_rock_FO_level_to_push] = 1;
//          }
//              
//        if ( fo_counter_push > fo_push_time )
//          //after pushing, level FO
//          {
//           sensor_array[COND_rock_FO_push_to_level] = 1;
//          }
//      }   

      

 

}
