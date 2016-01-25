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
 


if ( fo_counter_level >= 600 )
  {
    sensor_array[COND_rock_FO_level_to_push] = 1;
  }

if ( fo_counter_push >= 300 )
{
   sensor_array[COND_rock_FO_push_to_level] = 1;
  }

}
