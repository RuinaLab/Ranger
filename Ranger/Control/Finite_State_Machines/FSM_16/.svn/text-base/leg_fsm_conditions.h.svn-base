extern float ho_preswing_time;
extern float hi_preswing_time;
 
 
enum states 
{
  STATE_LI_preswing,
  STATE_LI_premid, 
  STATE_LI_aftermid, 
  STATE_LO_preswing,
  STATE_LO_premid, 
  STATE_LO_aftermid, 
  STATE_L_stop,
};


// define the inputs
//enum conditions
enum sensor 
{
  COND_LO_preswing_to_LO_premid, 
  COND_LO_premid_to_LO_aftermid, 
  COND_LO_aftermid_to_LI_preswing,
  COND_LI_preswing_to_LI_premid, 
  COND_LI_premid_to_LI_aftermid,
  COND_LI_aftermid_to_LO_preswing,
  COND_LO_startaftermid, //start state for fsm
  COND_L_stop
};



void get_leg_sensor_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero


if (0) //always false. ie. no stop fsm is coded yet
  {}
  
 else
 {

  if  (   
     (get_io_float(ID_MCFO_RIGHT_ANKLE_RATE) < -get_io_float(ID_P_H_PM_F_RATE) )
      )
      //&&
      //(get_io_float(ID_E_FO_ABSANG) < get_io_float(ID_P_H_PM_F_ANG))
  {
    sensor_array[COND_LI_preswing_to_LI_premid] = 1;
  }  
 

  if ((get_io_float(ID_E_LI_ABSANG)) > 0.0) 
  {
    sensor_array[COND_LI_premid_to_LI_aftermid] = 1;
  }
 

   if ( (int) get_io_float(ID_E_SWING_LEG) == 1)
  {
    sensor_array[COND_LI_aftermid_to_LO_preswing] = 1;
  }


  if (
      (get_io_float(ID_MCFI_ANKLE_RATE) < -get_io_float(ID_P_H_PM_F_RATE))
  )
  // (get_io_float(ID_E_FI_ABSANG) < get_io_float(ID_P_H_PM_F_ANG))  
  {
    sensor_array[COND_LO_preswing_to_LO_premid] = 1;
  }
  
  if ((get_io_float(ID_E_LO_ABSANG)) > 0.0) 
  {
    sensor_array[COND_LO_premid_to_LO_aftermid] = 1;
  }


  if ( (int) get_io_float(ID_E_SWING_LEG) == 0)
     {
    sensor_array[COND_LO_aftermid_to_LI_preswing] = 1;
  }


   if ( (int) get_io_float(ID_FSM_RESET) == 1 )    // Reset to start stance, this is set in ui_fsm
  {
    sensor_array[COND_LO_startaftermid] = 1;
  }  
  
   if (0) //Disable stopping of fsm
  {
    sensor_array[COND_L_stop] = 1;
  }
  
 }


}

