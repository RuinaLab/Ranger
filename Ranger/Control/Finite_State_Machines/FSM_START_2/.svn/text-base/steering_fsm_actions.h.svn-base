//static float desired_angle;
static float desired_current;

static float target_current;
static float target_dcurrent;
static float target_rate;

static float old_target_current;
static float new_target_current;

//using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Define action functions

int ACT_S_innerlegfront_entry()
{
  //desired_angle = -desired_angle; 
  //desired_current = -get_io_float(ID_P_S_ILST_TWITCH_FRAC)*desired_current;
  
  old_target_current = desired_current; 
  new_target_current = -get_io_float(ID_P_S_ILST_TWITCH_FRAC)*desired_current;


  return 1;
}

int ACT_S_innerlegfront()
{


  return 1;
}

int ACT_S_innerlegstance_entry()
{

  target_current = old_target_current;
  target_dcurrent = new_target_current - old_target_current;
  target_rate = 1.0/get_io_float(ID_P_S_ILST_TWITCH_TIME);
  return 1;
}

int ACT_S_innerlegstance()
{ 
  
  //set_io_float(ID_MCSI_COMMAND_CURRENT, desired_angle * get_io_float(ID_MCSI_STIFFNESS));
  //set_io_float(ID_MCSI_COMMAND_CURRENT, desired_current);
  
  float command_current;
  target_current = target_current + target_rate*target_dcurrent;
  if ( (target_dcurrent > 0 && target_current > new_target_current)  //increasing current
             ||
        (target_dcurrent < 0 && target_current < new_target_current) ) //decreasing current    
        command_current = new_target_current;
  else 
        command_current = target_current;
    
  set_io_float(ID_MCSI_COMMAND_CURRENT, command_current);
    
  return 1; 
}

int ACT_S_innerlegback_entry()
{ 

 
  float rc_command = 0.00004*get_io_float(ID_UI_RC_0)- 3.4; //A number between -1 and 1
 
  if (rc_command<-1.0)
      { rc_command = -1.0;}
  else if (rc_command>1.0)
      { rc_command=1.0; }
      
  //desired_angle = get_io_float(ID_P_S_ILST_S_MAXANG)*rc_command; 
  desired_current = get_io_float(ID_P_S_NULL_S_MAX_CUR)*rc_command;    
  
  if (rc_command<get_io_float(ID_P_S_NULL_RC_DEADBAND) && rc_command >-get_io_float(ID_P_S_NULL_RC_DEADBAND)) //don't steer
     { 
     //shutdown the motor to save some energy
        set_io_ul(ID_MCSI_SHUTDOWN, 1);
        set_io_float(ID_D_S_NULL_S_DANG,0.0);
      } 
  else //steer
    {
      set_io_ul(ID_MCSI_SHUTDOWN, 0); //turn on 
      //set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
      //set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 
      //set_io_float(ID_D_S_NULL_S_DANG,desired_angle);
      set_io_float(ID_D_S_NULL_S_DANG,desired_current);
    }
  return 1; 
}

int ACT_S_innerlegback()
{ 

  return 1; 
}

int ACT_S_innerlegswing_entry()
{ 
  return 1; 
}

int ACT_S_innerlegswing()
{ 

  //set_io_float(ID_MCSI_COMMAND_CURRENT, desired_angle * get_io_float(ID_MCSI_STIFFNESS));
  set_io_float(ID_MCSI_COMMAND_CURRENT, desired_current);


  return 1;
}


int ACT_S_stop()
{  
  set_io_float(ID_MCSI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCSI_STIFFNESS, 0.0);
  set_io_float(ID_MCSI_DAMPNESS, 0.0);
  return 1;
}
