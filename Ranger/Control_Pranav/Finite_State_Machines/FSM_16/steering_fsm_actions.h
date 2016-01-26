//static float desired_angle;
static float desired_angle_inner;
static float desired_angle_outer;

static float target_angle;
static float target_dangle;
static float target_rate;

static float old_target_angle;
static float new_target_angle;

//using namespace std;

float fmin(float a, float b)
// returns the smaller of two float values;
//////////////// idealy, a way to include a c 'math.h' library here should be found instead
{
    if (a < b) {
        return(a);
    }else {
        return(b); }
}


float auto_steer(float cam_signal)
// auto steering algorithm using the CMUcam4 board
// returns a value in [-1,1], where -1 corresponds to max steering to the right, 1 to max steering to the left
// expected input cam_signal: a value in [-1,1], where -1 = 'the line is at the left edge of the view',
//                                                      1 = 'the line is at the right edge of the view'
{
    static float dist_old = 0;    //Store the previous distance
    static float cmd_old = 0;     //Store the previous command
    
    float dist = cam_signal; // Scale the raw [-1,1] camera data
             
    //These are the parameters in Andy's controller. Later these should be moved to normal parameters
    float P_a = get_io_float(ID_F_TEST6);
    float P_b = get_io_float(ID_F_TEST7);
    float P_c = get_io_float(ID_F_TEST8);
    
    //Compute the desired steering command angle
    float cmd = P_a*dist_old + P_b*dist + P_c*cmd_old;
    
    // Bound the control output. (This is redundant now, but used in the "old cmd" value)
    if(cmd > 1.0){
      cmd = 1.0;
    } else if(cmd < -1.0){
      cmd = -1.0;
    }
    
    //Store the old values for use on the next step
    dist_old = dist;
    cmd_old = cmd;
    return(cmd);     //Return the steering command
    
/*
        ////The following block of code is Dunwen's Steering Algorithm
        static float dist_old = 0;    // distance to the path at the previous step, based on the camera image
        
        float step = 0.67;     // length of two steps of Ranger (steering is done once per two steps)
        float max_ang = get_io_float(ID_P_S_MAX_STEER_ANG);    // maximum steering angle
        float min_r = step/max_ang;     // minimum turning radius, based on the maximum steering radius
        float D = 1.57;     // = pi/2
        
        float dist = -0.458*cam_signal; // -0.236*cam_signal; (older dunwen's number) 
                                        // experimentally distermined number which converts the cam signal to distance of the robot's center from the detected black line on the ground in meters. 
        float theta = (dist - dist_old)/step;    // relative angle
        
        float A = 1/step;
        float B = (D - max_ang) / (min_r - step*max_ang);
        float C = (B*step + 1.0)*max_ang;
        
        float tan_ang, steer_command;
        
        float k = get_io_float(ID_F_TEST2);   // for testing purposes
        
        if (dist < 0.0) {
            tan_ang = fmin(-A*dist*k, fmin(-B*dist+C, D)); // first argument of the min function is typically used for following a straight line,
                                                            // the second one is typically used during 'not small' turns
            
            set_io_float(ID_T_TEST_07, -A*dist*k);
            set_io_float(ID_T_TEST_08, -B*dist + C);
        }else {
            tan_ang = -fmin(A*dist*k, fmin(B*dist+C, D));
            
            set_io_float(ID_T_TEST_07, A*dist*k);
            set_io_float(ID_T_TEST_08, B*dist + C);
        }
        
        steer_command = -(theta - tan_ang)/max_ang;  // out_g = ang-tan_ang;  k = out_g/E
        
        
        // note: saturation of too large commands is done in state_action function(s) of the Steering FSM
        
        set_io_float(ID_T_TEST_04, dist);
        set_io_float(ID_T_TEST_05, dist_old);
    
        dist_old = dist;
        return(steer_command);
*/

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

// associate the states with actions
int ACT_S_innerlegfront_entry()
{
    return 1;
}

int ACT_S_innerlegfront()
{
    return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

int ACT_S_innerlegstance_entry()
{
    old_target_angle = desired_angle_inner; 
    new_target_angle = -desired_angle_outer;
    
    target_angle = old_target_angle;
    target_dangle = new_target_angle - old_target_angle;
    target_rate = 1.0/get_io_float(ID_P_S_ILST_TWITCH_TIME);
    
    return 1;
}

int ACT_S_innerlegstance()
{
    float command_angle;
    
    target_angle = target_angle + target_rate*target_dangle;
    if ( (target_dangle > 0 && target_angle > new_target_angle)  //increasing angle
      || (target_dangle < 0 && target_angle < new_target_angle) ) //decreasing angle
          command_angle = new_target_angle;
    else 
          command_angle = target_angle;
      
    // added to test new steering motor controller;
    // ID_MCSI_PROP_COEFF is kp, the proportional term, and ID_MCSI_INT_COEFF is ki, the integral term in the steering motor controller
    // Petr, Feb/11/2013
    set_io_float(ID_MCSI_PROP_COEFF, get_io_float(ID_P_S_ILSW_S_MAXANG));
    set_io_float(ID_MCSI_INT_COEFF, get_io_float(ID_P_S_ILST_S_MAXANG));
    // end of test code
    
    set_io_float(ID_MCSI_COMMAND_ANG, command_angle);
    
    return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

int ACT_S_innerlegback_entry()
{
    return 1;
}

int ACT_S_innerlegback()
{
    return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

int ACT_S_innerswingpremid_entry()
{
    // turn the inner legs to the middle position to prepare for the next command, which is assigned at the midstance (entry of STATE_S_innerswingaftermid)
    desired_angle_inner = 0.0;
    
    return 1;
}

int ACT_S_innerswingpremid()
{
    // added to test new steering motor controller;
    // ID_MCSI_PROP_COEFF is kp, the proportional term, and ID_MCSI_INT_COEFF is ki, the integral term in the steering motor controller
    // Petr, Feb/11/2013
    set_io_float(ID_MCSI_PROP_COEFF, get_io_float(ID_P_S_ILSW_S_MAXANG));
    set_io_float(ID_MCSI_INT_COEFF, get_io_float(ID_P_S_ILST_S_MAXANG));
    // end of test code
    
    set_io_float(ID_MCSI_COMMAND_ANG, desired_angle_inner);
    
    return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

int ACT_S_innerswingaftermid_entry()
{
    float steer_command;
  
    if (!(int)get_io_float(ID_NAV_CAM_USED)) {   // RC is in control => use right joystick for steering
        steer_command = get_io_float(ID_NAV_RC_STEER);    // a number in [-1,1]; most right position = -1, most left position = 1
        
        auto_steer(get_io_float(ID_NAV_CAM_STEER));   // we call the auto-steer algorithm anyway, so that when it is actually used, 
                                                      // it has correct previous step command (dist_old) to estimate relative angle of the path
    }else {    // switch is away from user, camera signal is used for steering
        steer_command = auto_steer(get_io_float(ID_NAV_CAM_STEER)); // a number in [-1,1]; -1 = max right, 1 = max left
    } 
  
    // saturate too large steering commands
    if (steer_command < -1.0)
        { steer_command = -1.0;}
    else if (steer_command > 1.0)
        { steer_command = 1.0; }
    
    // don't steer, if desired steering angle is too small; used to save some energy
    if (steer_command<get_io_float(ID_P_STEER_DEADBAND) && steer_command >-get_io_float(ID_P_STEER_DEADBAND)) {
        set_io_ul(ID_MCSI_SHUTDOWN, 1);     //shutdown the steering motor to save some energy
        desired_angle_inner = 0.0;
        desired_angle_outer = 0.0; }
    else { // actual steering
        set_io_ul(ID_MCSI_SHUTDOWN, 0); //turn on the steering motor
        desired_angle_outer = get_io_float(ID_P_S_MAX_STEER_ANG)*steer_command;    // desired steering angle for the inner stance phase
    }
    
    set_io_float(ID_D_S_NULL_S_DANG, desired_angle_outer);     // only to read desired angle outside of this file
    
    // desired steering angle for the second half of the inner swing phase;
    // set to a fraction of the desired angle to have a better collision of the inner legs (since they are much closer to each other, compared to outer legs)
    desired_angle_inner = desired_angle_outer * get_io_float(ID_P_S_ILSW_STEER_FRAC);
    
    
    
     // if this is the first run after the start of the walk, make the first step straight,
     // because the transition to InnerSwingAfterMid state of the steering fsm is done during the exit action of the Walk state of the ui fsm,
     // hence steering commands are based on the signals at the end of the previous walk
    if ((int)get_io_float(ID_FSM_RESET) == 1) {
        desired_angle_outer = 0.0;
        desired_angle_inner = 0.0;
        set_io_ul(ID_MCSI_SHUTDOWN, 0); //turn on the steering motor
        set_io_float(ID_D_S_NULL_S_DANG, desired_angle_outer);     // only to read desired angle outside of this file
    }

    
    return 1;
}

int ACT_S_innerswingaftermid()
{
    // added to test new steering motor controller;
    // ID_MCSI_PROP_COEFF is kp, the proportional term, and ID_MCSI_INT_COEFF is ki, the integral term in the steering motor controller
    // Petr, Feb/11/2013
    set_io_float(ID_MCSI_PROP_COEFF, get_io_float(ID_P_S_ILSW_S_MAXANG));
    set_io_float(ID_MCSI_INT_COEFF, get_io_float(ID_P_S_ILST_S_MAXANG));
    // end of test code

    set_io_float(ID_MCSI_COMMAND_ANG, desired_angle_inner);
    
    return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

int ACT_S_stop()
{  
  set_io_float(ID_MCSI_COMMAND_ANG, 0.0);
  set_io_float(ID_MCSI_PROP_COEFF, 0.0);
  set_io_float(ID_MCSI_INT_COEFF, 0.0);
  
  return 1;
}
