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
 
    //Create an if statement to check if the condition for each transition is true
    if ( fi_counter_level > 800 )
      {
        sensor_array[COND_rock_FI_level_to_push] = 1;
      }
    
    if ( 0 )
      {
       sensor_array[COND_rock_FI_push_to_level] = 1;
      }
      
    if ( fi_counter_push > 600 )
      {
       sensor_array[COND_rock_FI_push_to_clear] = 1;
      }

    if ( fi_counter_clear > 1000 )
      {
       sensor_array[COND_rock_FI_clear_to_level] = 1;
      }
}
