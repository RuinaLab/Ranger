
static float abs_leg_angle_at_ss0;
static float abs_hip_angle_at_ss0;
static float time_at_ss0;

float ho_preswing_time;
float hi_preswing_time;
float ho_aftermid_time;
float hi_aftermid_time;

const float e_avg = 1; //0 means estimates are not averaged.
                       //1 means the estimates are averaged
                       
const float e_avg_h = 1; //h is for hip ang and hip rate. Same meaning as above  

const int fd_estimate = 1; //1 than use fd_estimates 
                            //0 than use continuous estimates
                            
//mb_time = 2; //main brain loop time is set in global_communications              

// These estimates can also be done in estimator.c
// Mid-stance estimates start
static float robot_velocity_mid_stance_in;
static float robot_velocity_mid_stance_out;
static float robot_velocity_mid_stance;

static float robot_hip_velocity_mid_stance_in;
static float robot_hip_velocity_mid_stance_out;
static float robot_hip_velocity_mid_stance;

static float robot_hip_angle_mid_stance_in;
static float robot_hip_angle_mid_stance_out;
static float robot_hip_angle_mid_stance;

static float robot_time_mid_stance_in;
static float robot_time_mid_stance_out;
static float robot_time_mid_stance;
// Mid-stance estimates end


// Heel-strike estimates start
//static float robot_absang_hs_in;
//static float robot_absang_hs_out;
//static float robot_absang_hs;
// Heel-strike estimates end

// Step estimates start
/*static float robot_velocity_step_in; //Not estimated
static float robot_velocity_step_out; //Not estimated
static float robot_velocity_step; //Not estimated

static float robot_stance_angle_step_in;
static float robot_stance_angle_step_out;
static float robot_stance_angle_step;

static float robot_hip_angle_step_in;
static float robot_hip_angle_step_out;
static float robot_hip_angle_step;*/

//static float robot_time_step_in;
//static float robot_time_step_out;
//static float robot_time_step;
// Step estimates end

// PS estimates start
static float robot_li_absang_phs;
static float robot_lo_absang_phs;

static float robot_li_angle_ps;
static float robot_lo_angle_ps;
static float robot_stance_angle_ps;

static float robot_li_velocity_ps;
static float robot_lo_velocity_ps;
static float robot_stance_velocity_ps; 
//PS estimates end



int ACT_LO_preswing_entry(void)
{
 
  hi_preswing_time = 0.0;
  
  //Estimation starts   
  //Remember: outer absolute angle here
  robot_lo_absang_phs = -get_io_float(ID_E_LO_ABSANG); //Is made positive
  
  /*robot_absang_hs_in  = get_io_float(ID_E_LI_ABSANG);
  robot_absang_hs = 0.5*((2-e_avg_h)*robot_absang_hs_in + e_avg_h*robot_absang_hs_out); 
  
  robot_hip_angle_step_in = -get_io_float(ID_E_LO_ABSANG); //Is made positive 
  robot_hip_angle_step = 0.5*((2-e_avg_h)*robot_hip_angle_step_in + e_avg_h*robot_hip_angle_step_out);
  set_io_float(ID_E_STEP_HIPANG,robot_hip_angle_step);
  
  robot_stance_angle_step_in = get_io_float(ID_E_LI_ABSANG); //Is positive (tilting fwd)
  robot_stance_angle_step = 0.5*((2-e_avg_h)*robot_stance_angle_step_in + e_avg_h*robot_stance_angle_step_out);
  set_io_float(ID_E_STEP_LEGANG,robot_stance_angle_step);
  
  robot_time_step_out = get_io_float(ID_E_STEP_TIME);
  float robot_time_step = 0.5*((2-e_avg_h)*robot_time_step_in + e_avg_h*robot_time_step_out);
  
  float time_from_mid_to_step = robot_time_step - get_io_float(ID_E_MIDSTANCE_TIME);
  float one_over_time_from_mid_to_step = 1000.0/time_from_mid_to_step;
  robot_velocity_step = get_io_float(ID_E_STEP_LEGANG)*one_over_time_from_mid_to_step;
  set_io_float(ID_E_STEP_LEGRATE,robot_velocity_step);*/

  //Estimation ends 
  
  
  return 1;
}


int ACT_LO_preswing(void)
{
  hi_preswing_time = hi_preswing_time + mb_time;
  return 1;
}

int ACT_LO_preswing_exit(void)
{
  set_io_float(ID_E_H_PS_TIME,hi_preswing_time);
  
  //hi_preswing_time = 0.0; //cannot have re-intialization here because hi_preswing_time is used in the next state
   
  return 1;
}

int ACT_LO_premid_entry(void)
{

//Estimation starts
  abs_leg_angle_at_ss0 = -get_io_float(ID_E_LO_ABSANG);
 // hip_angle_at_hs = get_io_float(ID_MCH_ANGLE); //
 abs_hip_angle_at_ss0 = get_io_float(ID_MCH_ANGLE); //Is negative
 //abs_hip_angle_at_ss0 = -get_io_float(ID_E_LI_ABSANG); //Is made negative with the sign
 time_at_ss0 = get_io_float(ID_E_T_AFTER_HS);
 
 
  //Remember: estimate outer absolute angle and outer absolute rate here
  float one_over_ps_time = 1000.0/hi_preswing_time;
  robot_lo_angle_ps = -get_io_float(ID_E_LO_ABSANG); //Is made positive
  robot_lo_velocity_ps = (robot_lo_absang_phs - robot_lo_angle_ps)*one_over_ps_time; //Is positive
  
  robot_stance_angle_ps = 0.5*((2-e_avg_h)*robot_lo_angle_ps + e_avg_h*robot_li_angle_ps);
  robot_stance_velocity_ps =  0.5*((2-e_avg_h)*robot_lo_velocity_ps + e_avg_h*robot_li_velocity_ps);
  
  set_io_float(ID_E_SS0_LEGANG, robot_stance_angle_ps);
  set_io_float(ID_E_SS0_LEGRATE, robot_stance_velocity_ps);
 //Estimation ends
  

  return 1;
}

int ACT_LO_premid(void)
{
 return 1;
}


int ACT_LO_aftermid_entry(void)
{
  float time_from_ss0_to_mid_stance = get_io_float(ID_E_T_AFTER_HS)-time_at_ss0;
  float one_over_time_after_ss0 = 1000.00/time_from_ss0_to_mid_stance;
  
  
//Estimation starts
  if (fd_estimate)
  {
    robot_velocity_mid_stance_in = abs_leg_angle_at_ss0*one_over_time_after_ss0;
    robot_hip_velocity_mid_stance_in = (robot_hip_angle_mid_stance_in-abs_hip_angle_at_ss0)*one_over_time_after_ss0; //Is positive
  }
  else
  {
    robot_velocity_mid_stance_in = get_io_float(ID_E_OUTER_ANG_RATE);
    robot_hip_velocity_mid_stance_in = get_io_float(ID_E_H_RATE); //Is positive
  }
  
  
  robot_velocity_mid_stance = 0.5*((2-e_avg)*robot_velocity_mid_stance_in + e_avg*robot_velocity_mid_stance_out);
  //safety check 
  if (robot_velocity_mid_stance >= 1.5)
      robot_velocity_mid_stance = 1.5;
  else if (robot_velocity_mid_stance <=0.0)
      robot_velocity_mid_stance = 0.01;
      
  set_io_float(ID_E_MIDSTANCE_LEGRATE, robot_velocity_mid_stance);
     
  robot_hip_angle_mid_stance_in = get_io_float(ID_MCH_ANGLE); //Is positive if swing leg is in front
  //robot_hip_angle_mid_stance_in= -get_io_float(ID_E_LI_ABSANG); //Is positive if swing leg is leaning fwd
  robot_hip_angle_mid_stance = 0.5*((2-e_avg_h)*robot_hip_angle_mid_stance_in + e_avg_h*robot_hip_angle_mid_stance_out);
  set_io_float(ID_E_MIDSTANCE_HIPANG,robot_hip_angle_mid_stance); 

  robot_hip_velocity_mid_stance = 0.5*((2-e_avg_h)*robot_hip_velocity_mid_stance_in+e_avg_h*robot_hip_velocity_mid_stance_out);
  set_io_float(ID_E_MIDSTANCE_HIPRATE,robot_hip_velocity_mid_stance);
  
  robot_time_mid_stance_in = time_from_ss0_to_mid_stance;
  robot_time_mid_stance = 0.5*((2-e_avg_h)*robot_time_mid_stance_in + e_avg_h*robot_time_mid_stance_out);
  set_io_float(ID_E_MIDSTANCE_TIME,robot_time_mid_stance);
  

//Estimation ends  
  

  return 1;
}

int ACT_LO_aftermid(void)
{
  hi_aftermid_time = hi_aftermid_time + mb_time;
  return 1;
}

int ACT_LO_aftermid_exit(void)
{
  set_io_float(ID_E_H_AM_TIME,hi_aftermid_time);
  
  hi_aftermid_time = 0.0;
   
  return 1;
}

int ACT_LI_preswing_entry(void)
{
   
   ho_preswing_time = 0.0; 
  
  //Estimation starts 
   //Remember: inner absolute angle here
   robot_li_absang_phs = -get_io_float(ID_E_LI_ABSANG); //Is made positive
  
  
  /*robot_absang_hs_out  = get_io_float(ID_E_LO_ABSANG);
  robot_absang_hs = 0.5*(e_avg_h*robot_absang_hs_in + (2-e_avg_h)*robot_absang_hs_out); 
  
  robot_hip_angle_step_out = -get_io_float(ID_E_LI_ABSANG); //Is made positive with the sign in front 
  robot_hip_angle_step = 0.5*(e_avg_h*robot_hip_angle_step_in + (2-e_avg_h)*robot_hip_angle_step_out);
  set_io_float(ID_E_STEP_HIPANG,robot_hip_angle_step);
  
  robot_stance_angle_step_out = get_io_float(ID_E_LO_ABSANG); //Is positive (tilting fwd)
  robot_stance_angle_step = 0.5*(e_avg_h*robot_stance_angle_step_in + (2-e_avg_h)*robot_stance_angle_step_out);
  set_io_float(ID_E_STEP_LEGANG,robot_stance_angle_step);
  
   robot_time_step_in = get_io_float(ID_E_STEP_TIME);
  float robot_time_step = 0.5*(e_avg_h*robot_time_step_in + (2-e_avg_h)*robot_time_step_out);
  
  float time_from_mid_to_step = robot_time_step - get_io_float(ID_E_MIDSTANCE_TIME);
  float one_over_time_from_mid_to_step = 1000.0/time_from_mid_to_step;
  robot_velocity_step = get_io_float(ID_E_STEP_LEGANG)*one_over_time_from_mid_to_step;
  set_io_float(ID_E_STEP_LEGRATE,robot_velocity_step);*/
  
  //Estimation ends 
  
  return 1;
}

int ACT_LI_preswing(void)
{
  ho_preswing_time = ho_preswing_time + mb_time;
  return 1;
}

int ACT_LI_preswing_exit(void)
{
  set_io_float(ID_E_H_PS_TIME,ho_preswing_time);
  
  //ho_preswing_time = 0.0; //cannot have re-intialization here because ho_preswing_time is used in the next state
  
  return 1;
}


int ACT_LI_premid_entry(void)
{

//Estimation starts 
  abs_leg_angle_at_ss0 = -get_io_float(ID_E_LI_ABSANG);
  //hip_angle_at_hs = get_io_float(ID_MCH_ANGLE); //Is positive 
  abs_hip_angle_at_ss0 = -get_io_float(ID_MCH_ANGLE); //Is made negative 
  //abs_hip_angle_at_ss0 = -get_io_float(ID_E_LO_ABSANG); //Is made positive with the negative sign
  time_at_ss0 = get_io_float(ID_E_T_AFTER_HS);
  
   //Remember: inner absolute angle and inner absolute rate here
  float one_over_ps_time = 1000.0/ho_preswing_time;
  robot_li_angle_ps = -get_io_float(ID_E_LI_ABSANG); //Is made positive
  robot_li_velocity_ps = (robot_li_absang_phs - robot_li_angle_ps)*one_over_ps_time; //Is positive
  
  robot_stance_angle_ps = 0.5*(e_avg_h*robot_lo_angle_ps + (2-e_avg_h)*robot_li_angle_ps);
  robot_stance_velocity_ps =  0.5*(e_avg_h*robot_lo_velocity_ps + (2-e_avg_h)*robot_li_velocity_ps);
  
  set_io_float(ID_E_SS0_LEGANG, robot_stance_angle_ps);
  set_io_float(ID_E_SS0_LEGRATE, robot_stance_velocity_ps);  
//Estimation ends 
  
  return 1;
}

int ACT_LI_premid(void)
{
  return 1;
}



int ACT_LI_aftermid_entry(void)
{
  float time_from_ss0_to_mid_stance = get_io_float(ID_E_T_AFTER_HS)-time_at_ss0;
  float one_over_time_after_ss0 = 1000.00/time_from_ss0_to_mid_stance;
  
  
//Estimation starts 

  if (fd_estimate)
  {
    robot_velocity_mid_stance_out = abs_leg_angle_at_ss0*one_over_time_after_ss0;
    robot_hip_velocity_mid_stance_out = (robot_hip_angle_mid_stance_out-abs_hip_angle_at_ss0)*one_over_time_after_ss0; //Is positive
  }
  else
  {
    robot_velocity_mid_stance_out = get_io_float(ID_E_INNER_ANG_RATE);
    robot_hip_velocity_mid_stance_out = -get_io_float(ID_E_H_RATE); //Is made positive
  }
  
  robot_velocity_mid_stance = 0.5*(e_avg*robot_velocity_mid_stance_in + (2-e_avg)*robot_velocity_mid_stance_out);
    //safety check 
  if (robot_velocity_mid_stance >= 1.5)
      robot_velocity_mid_stance = 1.5;
  else if (robot_velocity_mid_stance <=0.0)
      robot_velocity_mid_stance = 0.01;
  
  set_io_float(ID_E_MIDSTANCE_LEGRATE,robot_velocity_mid_stance);
  
  robot_hip_angle_mid_stance_out = -get_io_float(ID_MCH_ANGLE); //Is negative if swing leg is in front
 // robot_hip_angle_mid_stance_out  = -get_io_float(ID_E_LO_ABSANG); //Is positive if the hip is leaning in front
  robot_hip_angle_mid_stance = 0.5*(e_avg_h*robot_hip_angle_mid_stance_in + (2-e_avg_h)*robot_hip_angle_mid_stance_out);
  set_io_float(ID_E_MIDSTANCE_HIPANG,robot_hip_angle_mid_stance); 

  robot_hip_velocity_mid_stance = 0.5*(e_avg_h*robot_hip_velocity_mid_stance_in+(2-e_avg_h)*robot_hip_velocity_mid_stance_out);
  set_io_float(ID_E_MIDSTANCE_HIPRATE,robot_hip_velocity_mid_stance);
  
  robot_time_mid_stance_out = time_from_ss0_to_mid_stance;
  robot_time_mid_stance = 0.5*(e_avg_h*robot_time_mid_stance_in + (2-e_avg_h)*robot_time_mid_stance_out);
  set_io_float(ID_E_MIDSTANCE_TIME,robot_time_mid_stance);
//Estimation ends
 
   
  return 1;
}

int ACT_LI_aftermid(void)
{
  ho_aftermid_time = ho_aftermid_time + mb_time;
  return 1;
}

int ACT_LI_aftermid_exit(void)
{
  set_io_float(ID_E_H_AM_TIME,ho_aftermid_time);
  
  ho_aftermid_time = 0.0;
  
  return 1;
}


int ACT_L_stop(void)
{
  return 1;
}

