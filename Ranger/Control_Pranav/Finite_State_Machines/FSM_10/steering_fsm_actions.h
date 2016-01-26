static float desired_angle;
//using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Define action functions
//TO DO:
//ID_D_S_NULL_S_DANG to point to desired steer angle commanded by rc

int ACT_S_innerlegfront_entry()
{
  desired_angle = -desired_angle; 
  

  return 1;
}

int ACT_S_innerlegfront()
{
  // Test code to find what ID_UI_RC0 is doing 6/2/2010
  //float rc_command = get_io_float(ID_UI_RC_0)/25000 - 3.4;
  // End Test
  
  return 1;
}

int ACT_S_innerlegstance_entry()
{

  return 1;
}

int ACT_S_innerlegstance()
{ 
  
  set_io_float(ID_MCSI_COMMAND_CURRENT, desired_angle * get_io_float(ID_MCSI_STIFFNESS));
  return 1; 
}

int ACT_S_innerlegback_entry()
{ 

  
  float rc_command = get_io_float(ID_UI_RC_0)/25000 - 3.4; //A number between -1 and 1
  desired_angle = get_io_float(ID_P_S_ILST_S_MAXANG)*rc_command; 
  
  if (rc_command<-1.0)
      { rc_command = -1.0;}
  else if (rc_command>1.0)
      { rc_command=1.0; }
  

  
  if (rc_command<0.15 && rc_command >-0.15) //don't steer
     { 
     //shutdown the motor to save some energy
        set_io_ul(ID_MCSI_SHUTDOWN, 1);
      } 
  else //steer
    {
      set_io_ul(ID_MCSI_SHUTDOWN, 0);
      set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
      set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 
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

  set_io_float(ID_MCSI_COMMAND_CURRENT, desired_angle * get_io_float(ID_MCSI_STIFFNESS));

  return 1;
}


int ACT_S_stop()
{  
  set_io_float(ID_MCSI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCSI_STIFFNESS, 0.0);
  set_io_float(ID_MCSI_DAMPNESS, 0.0);
  return 1;
}
