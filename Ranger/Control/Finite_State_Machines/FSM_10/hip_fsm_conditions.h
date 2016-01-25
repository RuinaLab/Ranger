extern int stop_walk_on_inner_flag, stop_walk_on_outer_flag;
 
enum states 
{
  STATE_HI_starthold, // Starting hold state
  STATE_HI_preswing,
  STATE_HI_premid, 
  STATE_HI_aftermid, 
  STATE_HO_preswing,
  STATE_HO_premid, 
  STATE_HO_aftermid, 
  STATE_H_stop,
  STATE_HI_ehold,    // Inner swing leg emergency hold state (don't want leg to swing too far back)
  STATE_HO_ehold,     // Outer swing leg emergency hold state (don't want leg to swing too far back)
  STATE_HI_stophold, //Hold during stopping
  STATE_HO_stophold //Hold during stopping
};


// define the inputs
//enum conditions
enum sensor 
{
  COND_HI_starthold_to_HO_preswing,    //Start with leg at some hold angle
  COND_HO_preswing_to_HO_premid, 
  COND_HO_premid_to_HO_aftermid, 
  COND_HO_aftermid_to_HI_preswing,
  COND_HO_aftermid_to_HO_ehold,      // Detect case in which robot is moving too slow
  COND_HO_ehold_to_HI_preswing, 
  COND_HI_preswing_to_HI_premid, 
  COND_HI_premid_to_HI_aftermid,
  COND_HI_aftermid_to_HO_preswing,
  COND_HI_aftermid_to_HI_ehold,      // Detect case in which robot is moving too slow
  COND_HI_ehold_to_HO_preswing, 
  COND_H_stop,
  COND_HI_starthold, //Reset to start hold
  COND_HI_premid_to_HI_ehold,
  COND_HO_premid_to_HO_ehold,
  COND_HI_preswing_to_HI_stophold,
  COND_HO_preswing_to_HO_stophold
};



void get_hip_sensor_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero


if (0) //always false. ie. no stop fsm is coded yet
  {}
  
 else
 {
  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HI_starthold_to_HO_preswing] = 1;
  }

  if ( (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) <= 0.1) && (stop_walk_on_outer_flag == 0) ) 
  {
    sensor_array[COND_HO_preswing_to_HO_premid] = 1;
  }  
 

  if ((get_io_float(ID_E_LI_ABSANG)) > 0.0) 
  {
    sensor_array[COND_HO_premid_to_HO_aftermid] = 1;
  }
  
  if (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HO_aftermid_to_HI_preswing] = 1;
  }

  
  if (
        (get_io_float(ID_MCH_ANGLE) < -get_io_float(ID_P_H_EHOLD_ANG)) //Hold angle when swinging forward
         &&
        (get_io_float(ID_E_MIDSTANCE_LEGRATE) < get_io_float(ID_P_H_EH_L_TRATE)) //check is robot is moving too slow 
      )       
  {
    sensor_array[COND_HO_aftermid_to_HO_ehold] = 1;
    sensor_array[COND_HO_premid_to_HO_ehold] = 1;
  }
  
    if (stop_walk_on_inner_flag == 1 )      
  {
    sensor_array[COND_HI_preswing_to_HI_stophold] = 1;
  }  
   
   if (stop_walk_on_outer_flag == 1 ) 
  {  
    sensor_array[COND_HO_preswing_to_HO_stophold] = 1;
  }

  if (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HO_ehold_to_HI_preswing] = 1;
  }

  if ( (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) <= 0.1) && (stop_walk_on_inner_flag == 0) )
  {
    sensor_array[COND_HI_preswing_to_HI_premid] = 1;
  }
  
  if ((get_io_float(ID_E_LO_ABSANG)) > 0.0) 
  {
    sensor_array[COND_HI_premid_to_HI_aftermid] = 1;
  }

  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HI_aftermid_to_HO_preswing] = 1;
  }

 if (
      (get_io_float(ID_MCH_ANGLE) > get_io_float(ID_P_H_EHOLD_ANG))  //Hold angle when swinging forward
       &&
      (get_io_float(ID_E_MIDSTANCE_LEGRATE) < get_io_float(ID_P_H_EH_L_TRATE))  //check is robot is moving too slow 
       )      
  {
    sensor_array[COND_HI_aftermid_to_HI_ehold] = 1;
    sensor_array[COND_HI_premid_to_HI_ehold] = 1;
  }

  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HI_ehold_to_HO_preswing] = 1;
  }

   if ( (int) get_io_float(ID_FSM_RESET) == 1 )    // Reset to start stance, this is set in ui_fsm
  {
    sensor_array[COND_HI_starthold] = 1;
  }  
  
   if (0) //Disable stopping of fsm
  {
    sensor_array[COND_H_stop] = 1;
  }
  
 }


}

