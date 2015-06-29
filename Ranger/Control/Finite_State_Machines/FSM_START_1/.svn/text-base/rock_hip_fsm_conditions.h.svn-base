extern float hi_swing_count;
extern float ho_swing_count;

enum states 
{
  STATE_HI_swing,
  STATE_HO_swing,
};


// define the inputs
//enum conditions
enum sensor 
{
  COND_HI_swing_to_HO_swing,
  COND_HO_swing_to_HI_swing,
};



void get_rock_hip_sensor_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero


int timer=get_io_float(ID_TIMESTAMP);

//if ( ((timer/1000)%2)==0  )
if ( hi_swing_count >= 1000 )
  {
sensor_array[COND_HI_swing_to_HO_swing]=1;
  }  


//if (((timer/1000)%2)==1  )
if ( ho_swing_count >= 1000 )
  {
sensor_array[COND_HO_swing_to_HI_swing]=1;
  }

}
