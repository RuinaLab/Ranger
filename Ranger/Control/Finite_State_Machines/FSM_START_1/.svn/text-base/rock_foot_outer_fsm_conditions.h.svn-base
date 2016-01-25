
enum states 
{
  STATE_FO_flipup, 
  STATE_FO_flipdown, 
};

// define the inputs
enum sensor 
{
  COND_FO_flipup_to_flipdown,
  COND_FO_flipdown_to_flipup,
};


void get_rock_foot_outer_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;


   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero
 
 int timer=get_io_float(ID_TIMESTAMP);

if ( get_io_float(ID_MCH_ANGLE)>0  )
  {
    sensor_array[COND_FO_flipup_to_flipdown] = 1;
  }
  else
  {
    sensor_array[COND_FO_flipdown_to_flipup] = 1;
  }
 
}

