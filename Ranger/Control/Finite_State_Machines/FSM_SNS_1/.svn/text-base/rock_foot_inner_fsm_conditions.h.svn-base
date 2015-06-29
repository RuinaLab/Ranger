extern float fi_counter_level;
extern float fi_counter_push;
extern float fi_counter_clear;

void set_UI_LED(int led_number, char color);


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

  
    //If the robot is hanging in the air, then the sum of the contact sensors should be less than 1336
    bool Floating_Robot=( (get_io_float(ID_MCFO_LEFT_HEEL_SENSE)+get_io_float(ID_MCFO_RIGHT_HEEL_SENSE)+get_io_float(ID_MCFI_LEFT_HEEL_SENSE)+get_io_float(ID_MCFI_RIGHT_HEEL_SENSE)) < 1400 );
    static float time_to_swing = (get_io_float(ID_P_R_WAIT_TIME)+get_io_float(ID_P_R_FO_PUSH_TIME)+get_io_float(ID_P_R_FI_PUSH_TIME));  
      
    if ( get_io_float(ID_P_R_ROCK_TIMER) < get_io_float(ID_P_R_WAIT_TIME)) // 5000 
        {
            // before wait time has been reached, inner feet leveled at initial angle
            set_io_float(ID_P_R_FI_TANG, get_io_float(ID_P_R_FI_INIT_ANGLE)); //2.2
            sensor_array [COND_rock_FI_push_to_level] = 1;
            sensor_array [COND_rock_FI_clear_to_level] = 1;
            
        }
        
//    if ( get_io_float(ID_P_R_ROCK_TIMER)>5800&&get_io_float(ID_P_R_ROCK_TIMER)<6200)
    if ( get_io_float(ID_P_R_ROCK_TIMER) > (get_io_float(ID_P_R_WAIT_TIME)+get_io_float(ID_P_R_FO_PUSH_TIME)) && get_io_float(ID_P_R_ROCK_TIMER) < time_to_swing) 
        { 
            // wait time reached and outer feet have pushed, reset target angle, inner feet push
            set_io_float(ID_P_R_FI_TANG, get_io_float(ID_P_R_FI_END_ANGLE)); //2.0
            sensor_array[COND_rock_FI_level_to_push] = 1;
        }
        
//    if ( get_io_float(ID_P_R_ROCK_TIMER)>6500 &&get_io_float(ID_P_R_ROCK_TIMER)<6700) 
      if ( get_io_float(ID_P_R_ROCK_TIMER) > time_to_swing && get_io_float(ID_MCH_ANGLE) < get_io_float(ID_P_R_H_THRESHOLD_ANG)) // 0.16
        { 
            // inner foot push time reached, clear
            sensor_array[COND_rock_FI_push_to_clear] = 1;
        }
        
//        if (get_io_float(ID_P_R_ROCK_TIMER)>6500 && get_io_float(ID_MCH_ANGLE) > 0.16)
     if (get_io_float(ID_P_R_ROCK_TIMER) > time_to_swing && get_io_float(ID_MCH_ANGLE) > get_io_float(ID_P_R_H_THRESHOLD_ANG)) // 0.16
        {
            // threshold angle reached, switch to walk mode
            set_io_float(ID_P_R_ROCK_TO_WALK, 1);
        }
      
}

