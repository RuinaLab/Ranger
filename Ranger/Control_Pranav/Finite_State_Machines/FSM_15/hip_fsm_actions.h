static float time_at_ss0;

extern float ho_aftermid_time;
extern float hi_aftermid_time;

//static float discrete_hi_aftermid_dcurrent;
//static float discrete_ho_aftermid_dcurrent;

static float discrete_hi_aftermid_dcurrent_part1;
static float discrete_hi_aftermid_dcurrent_part2;
static float discrete_ho_aftermid_dcurrent_part1;
static float discrete_ho_aftermid_dcurrent_part2;

static float discrete_hi_premid_dcurrent;
static float discrete_hi_premid_dstiffness;
static float discrete_ho_premid_dcurrent;
static float discrete_ho_premid_dstiffness;


int ACT_HI_starthold_entry(void)
{ 
  return 1;
}

int ACT_HI_starthold(void)
{
  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_HI_SH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_HI_SH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));  
  
  
  return 1;
}

int ACT_HI_preswing_entry(void)
{

  set_io_float(ID_MCH_STIFFNESS, 0.0);
  set_io_float(ID_MCH_DAMPNESS, 0.0);
  set_io_float(ID_MCH_COMMAND_CURRENT, -get_io_float(ID_H_TEST1) ); //small negative current to prevent stubbing
  return 1;
}


int ACT_HI_preswing(void)
{
  
  set_io_float(ID_MCH_STIFFNESS, 0.0);
  set_io_float(ID_MCH_DAMPNESS, 0.0);
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0 ); 
  return 1;
}

int ACT_HI_premid_entry(void)
{

 time_at_ss0 = get_io_float(ID_E_T_AFTER_HS);
 
 float robot_angle_before_swing = get_io_float(ID_E_SS0_LEGANG);
 float robot_velocity_before_swing = get_io_float(ID_E_SS0_LEGRATE);
 
 discrete_hi_premid_dcurrent = 0.0;
 discrete_hi_premid_dstiffness = 0.0;
 
 float da_l_rate1 = 0.0, da_l_rate2 = 0.0;
 float da_l_ang1 = 0.0, da_l_ang2 = 0.0;
 
 if (((int) get_io_float(ID_D_H2_ON)) == 1)
 {
  
  da_l_rate1 = -get_io_float(ID_DC_H_PM_L_ABSRATE1)*(robot_velocity_before_swing - get_io_float(ID_DP_H_PM_L_ABSRATE) );
  da_l_rate2 = -get_io_float(ID_DC_H_PM_L_ABSRATE2)*(robot_velocity_before_swing - get_io_float(ID_DP_H_PM_L_ABSRATE) );
  da_l_ang1 = -get_io_float(ID_DC_H_PM_L_ABSANG1)*(robot_angle_before_swing - get_io_float(ID_DP_H_PM_L_ABSANG) );
  da_l_ang2 = -get_io_float(ID_DC_H_PM_L_ABSANG2)*(robot_angle_before_swing - get_io_float(ID_DP_H_PM_L_ABSANG) );
  
  discrete_hi_premid_dstiffness = da_l_ang1 + da_l_rate1;
  discrete_hi_premid_dcurrent = da_l_ang2 + da_l_rate2;
  
  if (discrete_hi_premid_dstiffness<0.0)
    discrete_hi_premid_dstiffness = 0.0;
   
  if (discrete_hi_premid_dcurrent<0.0) 
    discrete_hi_premid_dcurrent = 0.0;
 }
 
 set_io_float(ID_DA_H_PM_L_ABSANG1, da_l_ang1); set_io_float(ID_DA_H_PM_L_ABSANG2, da_l_ang2); 
 set_io_float(ID_DA_H_PM_L_ABSRATE1, da_l_rate1); set_io_float(ID_DA_H_PM_L_ABSRATE2, da_l_rate2); 

 set_io_float(ID_DA_H_PM_A1, discrete_hi_premid_dstiffness); 
 set_io_float(ID_DA_H_PM_A2, discrete_hi_premid_dcurrent); 
 
  return 1;
}

int ACT_HI_premid(void)
{

float command_current = 0.0;

  
   if  ( (get_io_float(ID_E_T_AFTER_HS)- time_at_ss0) < get_io_float(ID_P_H_PM_TIME) )
      {
      command_current = get_io_float(ID_A_H_PM_A0); 
      //command_current = get_io_float(ID_C_H_PM_H_RATE)*(get_io_float(ID_P_H_PM_H_TRATE) - get_io_float(ID_E_H_RATE)); //Velocity control based on hip rate
      }

  /*if (command_current<0)
      command_current = 0.0; */
  
    
   set_io_float(ID_MCH_STIFFNESS, discrete_hi_premid_dstiffness );
   set_io_float(ID_MCH_DAMPNESS, 0.0);      
   set_io_float(ID_MCH_COMMAND_CURRENT, command_current+discrete_hi_premid_dcurrent);
               

return 1;
}

int ACT_HI_premid_exit(void)
{
  return 1;
}


int ACT_HI_aftermid_entry(void)
{

  //discrete_hi_aftermid_dcurrent = 0.0;
   
  float robot_velocity_mid_stance = get_io_float(ID_E_MIDSTANCE_LEGRATE);
  float robot_hip_velocity_mid_stance = get_io_float(ID_E_MIDSTANCE_HIPRATE);
  float robot_hip_angle_mid_stance = get_io_float(ID_E_MIDSTANCE_HIPANG);
    
/*if (((int) get_io_float(ID_D_H_ON)) == 1) //Discrete control
{ 
          float dcurrent1=0.0, dcurrent2 = 0.0, dcurrent3 = 0.0, dcurrent4 = 0.0;
         
          if (robot_velocity_mid_stance > get_io_float(ID_DP_F_PP_L_ABSRATE) ) //robot velocity is faster than target than change discrete control
            {   
              dcurrent3 = get_io_float(ID_DC_H_AM_L_ABSRATE)*(robot_velocity_mid_stance - get_io_float(ID_DP_F_PP_L_ABSRATE) );
              dcurrent4 = -get_io_float(ID_DC_H_AM_H_DRATE)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_DRATE)*robot_velocity_mid_stance);
            } 
           
         if (robot_velocity_mid_stance < get_io_float(ID_DP_F_PP_L_ABSRATE) ) //robot velocity is slower than target than 
            {   
              dcurrent2 =  -get_io_float(ID_DC_H_AM_H_RATE)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) ); 
            } 
        
        if (robot_hip_angle_mid_stance < get_io_float(ID_DP_H_AM_H_ANG) )     
           { 
                  dcurrent1 = -get_io_float(ID_DC_H_AM_H_ANG)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
           }  
      //      if (dcurrent1<0) dcurrent1 = 0.0; 
      //      if (dcurrent2<0) dcurrent2 = 0.0;
      //      if (dcurrent3<0) dcurrent3 = 0.0; 
      //      if (dcurrent4<0) dcurrent4 = 0.0;
                
         discrete_hi_aftermid_dcurrent = dcurrent1 + dcurrent2 + dcurrent3 + dcurrent4;
}*/
  
float discrete_aftermid_dcurrent2_part1 = 0.0;
float discrete_aftermid_dcurrent2_part2 = 0.0;
float discrete_aftermid_dcurrent3_part1 = 0.0;
float discrete_aftermid_dcurrent3_part2 = 0.0;

discrete_hi_aftermid_dcurrent_part1 = 0.0;
discrete_hi_aftermid_dcurrent_part2 = 0.0;
  
float da2_h_ang1 = 0.0, da2_h_ang2 = 0.0;
float da2_h_rate1 = 0.0, da2_h_rate2 = 0.0;
float da3_h_ang1 = 0.0, da3_h_ang2 = 0.0;
float da3_l_rate1 = 0.0, da3_l_rate2 = 0.0;
float da3_h_rate1 = 0.0, da3_h_rate2 = 0.0;
  
if ( ((int)get_io_float(ID_D_H_ON)) == 2 || ((int)get_io_float(ID_D_H_ON)) == 1 )  //Discrete control
{ 
  da2_h_ang1 = -get_io_float(ID_DC2_H_AM_H_ANG1)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
  da2_h_ang2 =  -get_io_float(ID_DC2_H_AM_H_ANG2)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
  da2_h_rate1 = -get_io_float(ID_DC2_H_AM_H_RATE1)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) );
  da2_h_rate2 =  -get_io_float(ID_DC2_H_AM_H_RATE2)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) );
  
  discrete_aftermid_dcurrent2_part1 = da2_h_ang1 + da2_h_rate1;
  discrete_aftermid_dcurrent2_part2 = da2_h_ang2 + da2_h_rate2;
}  
  
if ( ((int) get_io_float(ID_D_H_ON)) == 3 || ((int)get_io_float(ID_D_H_ON)) == 1 ) //Discrete control
{ 
  da3_l_rate1 = -get_io_float(ID_DC3_H_AM_L_ABSRATE1)*(robot_velocity_mid_stance - get_io_float(ID_DP_H_AM_L_ABSRATE) );
  da3_l_rate2 = -get_io_float(ID_DC3_H_AM_L_ABSRATE2)*(robot_velocity_mid_stance - get_io_float(ID_DP_H_AM_L_ABSRATE) );
  da3_h_ang1 = -get_io_float(ID_DC3_H_AM_H_ANG1)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
  da3_h_ang2 =  -get_io_float(ID_DC3_H_AM_H_ANG2)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
  da3_h_rate1 = -get_io_float(ID_DC3_H_AM_H_RATE1)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) );
  da3_h_rate2 =  -get_io_float(ID_DC3_H_AM_H_RATE2)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) );
  
  discrete_aftermid_dcurrent3_part1 = da3_l_rate1 + da3_h_ang1 + da3_h_rate1;
  discrete_aftermid_dcurrent3_part2 = da3_l_rate2 + da3_h_ang2 + da3_h_rate2;
}

if (((int)get_io_float(ID_D_H_ON)) == 2) //push-off controller
{
  discrete_hi_aftermid_dcurrent_part1 = discrete_aftermid_dcurrent2_part1;
  discrete_hi_aftermid_dcurrent_part2 = discrete_aftermid_dcurrent2_part2;
}
else if (((int)get_io_float(ID_D_H_ON)) == 3) //step length controller
{
  discrete_hi_aftermid_dcurrent_part1 = discrete_aftermid_dcurrent3_part1;
  discrete_hi_aftermid_dcurrent_part2 = discrete_aftermid_dcurrent3_part2;
}
else if (((int)get_io_float(ID_D_H_ON)) == 1)
{
  if (robot_velocity_mid_stance >= get_io_float(ID_DP_F_PP_L_ABSRATE) ) //robot moving faster use step length controller, use controller 3
  {
      discrete_hi_aftermid_dcurrent_part1 = discrete_aftermid_dcurrent3_part1;
      discrete_hi_aftermid_dcurrent_part2 = discrete_aftermid_dcurrent3_part2;
      da2_h_ang1 =0.0;   da2_h_ang2 = 0.0;
      da2_h_rate1 =0.0;  da2_h_rate2 = 0.0;
  }
  else //use push-off controller, use controller 2
  {
      discrete_hi_aftermid_dcurrent_part1 = discrete_aftermid_dcurrent2_part1;
      discrete_hi_aftermid_dcurrent_part2 = discrete_aftermid_dcurrent2_part2;
      da3_h_ang1 = 0.0; da3_h_ang2 = 0.0;
      da3_l_rate1 = 0.0; da3_l_rate2 = 0.0;
      da3_h_rate1 = 0.0; da3_h_rate2 = 0.0;
  }
}

  set_io_float(ID_DA2_H_AM_H_ANG1,da2_h_ang1); set_io_float(ID_DA2_H_AM_H_ANG2,da2_h_ang2);
  set_io_float(ID_DA2_H_AM_H_RATE1,da2_h_rate1); set_io_float(ID_DA2_H_AM_H_RATE2,da2_h_rate2);
  
  set_io_float(ID_DA3_H_AM_L_ABSRATE1,da3_l_rate1); set_io_float(ID_DA3_H_AM_L_ABSRATE2,da3_l_rate2);
  set_io_float(ID_DA3_H_AM_H_ANG1,da3_h_ang1); set_io_float(ID_DA3_H_AM_H_ANG2,da3_h_ang2);
  set_io_float(ID_DA3_H_AM_H_RATE1,da3_h_rate1); set_io_float(ID_DA3_H_AM_H_RATE2,da3_h_rate2);
 
 set_io_float(ID_DA_H_AM_A1,discrete_hi_aftermid_dcurrent_part1);
 set_io_float(ID_DA_H_AM_A2,discrete_hi_aftermid_dcurrent_part2);
 
// Discrete control ends   
  
  
  return 1;
}

int ACT_HI_aftermid(void)
{

float command_current= 0.0;
float discrete_hi_aftermid_dcurrent = 0.0; 

if (  (((int) get_io_float(ID_D_H_ON)) == 3 )
             ||
     (((int) get_io_float(ID_D_H_ON)) == 2 )  
              ||
     (((int) get_io_float(ID_D_H_ON)) == 1 )            ) //Discrete control
{ 
        if  (hi_aftermid_time >= 0 && hi_aftermid_time < get_io_float(ID_DP_H_AM_TIME1) )
          discrete_hi_aftermid_dcurrent =  discrete_hi_aftermid_dcurrent_part1;
          
        if  (hi_aftermid_time >= get_io_float(ID_DP_H_AM_TIME1) && hi_aftermid_time < get_io_float(ID_DP_H_AM_TIME2) )
          discrete_hi_aftermid_dcurrent =  discrete_hi_aftermid_dcurrent_part2;
}         
 
   if  ( (get_io_float(ID_E_T_AFTER_HS)- time_at_ss0) < get_io_float(ID_P_H_PM_TIME) )
      {
      command_current = get_io_float(ID_A_H_PM_A0); 
      //command_current = get_io_float(ID_C_H_PM_H_RATE)*(get_io_float(ID_P_H_PM_H_TRATE) - get_io_float(ID_E_H_RATE)); //Velocity control based on hip rate
      }
      
   /*if (command_current<0)
      command_current = 0.0; */
      
 
   set_io_float(ID_MCH_STIFFNESS, 0.0);
   set_io_float(ID_MCH_DAMPNESS, 0.0);      
   set_io_float(ID_MCH_COMMAND_CURRENT, command_current+discrete_hi_aftermid_dcurrent);
   

  
  return 1;
}

int ACT_HO_preswing_entry(void)
{

  set_io_float(ID_MCH_STIFFNESS, 0.0);
  set_io_float(ID_MCH_DAMPNESS, 0.0);  
  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_H_TEST1) ); //small backward current to prevent stubbing
  return 1;
}


int ACT_HO_preswing(void)
{

  set_io_float(ID_MCH_STIFFNESS, 0.0);
  set_io_float(ID_MCH_DAMPNESS, 0.0);  
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0  );
  return 1;
}

int ACT_HO_premid_entry(void)
{

  time_at_ss0 = get_io_float(ID_E_T_AFTER_HS);
  
 float robot_angle_before_swing = get_io_float(ID_E_SS0_LEGANG);
 float robot_velocity_before_swing = get_io_float(ID_E_SS0_LEGRATE);
 
 discrete_ho_premid_dcurrent = 0.0;
 discrete_ho_premid_dstiffness = 0.0;
 
 float da_l_rate1 = 0.0, da_l_rate2 = 0.0;
 float da_l_ang1 = 0.0, da_l_ang2 = 0.0;
 
 if (((int) get_io_float(ID_D_H2_ON)) == 1)
 {
  
  da_l_rate1 = -get_io_float(ID_DC_H_PM_L_ABSRATE1)*(robot_velocity_before_swing - get_io_float(ID_DP_H_PM_L_ABSRATE) );
  da_l_rate2 = -get_io_float(ID_DC_H_PM_L_ABSRATE2)*(robot_velocity_before_swing - get_io_float(ID_DP_H_PM_L_ABSRATE) );
  da_l_ang1 = -get_io_float(ID_DC_H_PM_L_ABSANG1)*(robot_angle_before_swing - get_io_float(ID_DP_H_PM_L_ABSANG) );
  da_l_ang2 = -get_io_float(ID_DC_H_PM_L_ABSANG2)*(robot_angle_before_swing - get_io_float(ID_DP_H_PM_L_ABSANG) );
  
  discrete_ho_premid_dstiffness = da_l_ang1 + da_l_rate1;
  discrete_ho_premid_dcurrent = da_l_ang2 + da_l_rate2;
  
  if (discrete_ho_premid_dstiffness<0.0)
    discrete_ho_premid_dstiffness = 0.0;
   
  if (discrete_ho_premid_dcurrent<0.0) 
    discrete_ho_premid_dcurrent = 0.0;
 }
 
 set_io_float(ID_DA_H_PM_L_ABSANG1, da_l_ang1); set_io_float(ID_DA_H_PM_L_ABSANG2, da_l_ang2); 
 set_io_float(ID_DA_H_PM_L_ABSRATE1, da_l_rate1); set_io_float(ID_DA_H_PM_L_ABSRATE2, da_l_rate2); 

 set_io_float(ID_DA_H_PM_A1, discrete_ho_premid_dstiffness); 
 set_io_float(ID_DA_H_PM_A2, discrete_ho_premid_dcurrent); 
 
  
  return 1;
}

int ACT_HO_premid(void)
{

float command_current=0.0;
float stiffness = 0.0;   
float const_current = 0.0;
float dampness = 0.0;
  
  if  ( (get_io_float(ID_E_T_AFTER_HS)- time_at_ss0) < get_io_float(ID_P_H_PM_TIME) )
  {
    command_current = -get_io_float(ID_A_H_PM_A0); 
    //command_current = get_io_float(ID_C_H_PM_H_RATE)*(-get_io_float(ID_P_H_PM_H_TRATE) - get_io_float(ID_E_H_RATE)); 
  }
    
    /*if (command_current>0)
      command_current = 0.0; */
    
  
   /*if (get_io_float(ID_MCH_ANGLE)>=0.0) //add stiffness in first half
       stiffness = get_io_float(ID_C_H_OUTER_STIFFNESS);
   else //or add constant current in the second half
        {
         const_current = get_io_float(ID_A_H_PM_A0);
         dampness = -get_io_float(ID_C_H_OUTER_DAMPNESS);
         }*/
              
   stiffness = get_io_float(ID_C_H_OUTER_STIFFNESS);                
   set_io_float(ID_MCH_STIFFNESS, stiffness+discrete_ho_premid_dstiffness); 
   set_io_float(ID_MCH_DAMPNESS, dampness);   
   set_io_float(ID_MCH_COMMAND_CURRENT, command_current-const_current-discrete_ho_premid_dcurrent);   

  return 1;
}


int ACT_HO_premid_exit(void)
{
  return 1;
}


int ACT_HO_aftermid_entry(void)
{
  
  //discrete_ho_aftermid_dcurrent = 0.0;   
  float robot_velocity_mid_stance = get_io_float(ID_E_MIDSTANCE_LEGRATE);
  float robot_hip_velocity_mid_stance = get_io_float(ID_E_MIDSTANCE_HIPRATE);
  float robot_hip_angle_mid_stance = get_io_float(ID_E_MIDSTANCE_HIPANG);
  
/*  if (((int) get_io_float(ID_D_H_ON)) == 1) //Discrete control
{ 
  
        float dcurrent1=0.0, dcurrent2 = 0.0, dcurrent3 = 0.0, dcurrent4 = 0.0;
              
        if (robot_velocity_mid_stance > get_io_float(ID_DP_F_PP_L_ABSRATE) ) //robot velocity is faster than target than change discrete control
          {   
            dcurrent3 = get_io_float(ID_DC_H_AM_L_ABSRATE)*(robot_velocity_mid_stance - get_io_float(ID_DP_F_PP_L_ABSRATE) );
            dcurrent4 = -get_io_float(ID_DC_H_AM_H_DRATE)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_DRATE)*robot_velocity_mid_stance);
          } 
          
       if (robot_velocity_mid_stance < get_io_float(ID_DP_F_PP_L_ABSRATE) ) //robot velocity is slower than target than 
          {   
            dcurrent2 =  -get_io_float(ID_DC_H_AM_H_RATE)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) ); 
          } 
         
         if (robot_hip_angle_mid_stance < get_io_float(ID_DP_H_AM_H_ANG) )      
          { 
                dcurrent1 = -get_io_float(ID_DC_H_AM_H_ANG)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
          }
    //      if (dcurrent1<0) dcurrent1 = 0.0; 
    //      if (dcurrent2<0) dcurrent2 = 0.0;
    //      if (dcurrent3<0) dcurrent3 = 0.0; 
    //      if (dcurrent4<0) dcurrent4 = 0.0; 
       discrete_ho_aftermid_dcurrent = dcurrent1 + dcurrent2 + dcurrent3 + dcurrent4;
 }*/
 
float discrete_aftermid_dcurrent2_part1 = 0.0;
float discrete_aftermid_dcurrent2_part2 = 0.0;
float discrete_aftermid_dcurrent3_part1 = 0.0;
float discrete_aftermid_dcurrent3_part2 = 0.0;

discrete_ho_aftermid_dcurrent_part1 = 0.0;
discrete_ho_aftermid_dcurrent_part2 = 0.0;

float da2_h_ang1 = 0.0, da2_h_ang2 = 0.0;
float da2_h_rate1 = 0.0, da2_h_rate2 = 0.0;
float da3_h_ang1 = 0.0, da3_h_ang2 = 0.0;
float da3_l_rate1 = 0.0, da3_l_rate2 = 0.0;
float da3_h_rate1 = 0.0, da3_h_rate2 = 0.0;

if ( ((int)get_io_float(ID_D_H_ON)) == 2 || ((int)get_io_float(ID_D_H_ON)) == 1 )  //Discrete control
{ 
  da2_h_ang1 = -get_io_float(ID_DC2_H_AM_H_ANG1)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
  da2_h_ang2 =  -get_io_float(ID_DC2_H_AM_H_ANG2)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
  da2_h_rate1 = -get_io_float(ID_DC2_H_AM_H_RATE1)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) );
  da2_h_rate2 =  -get_io_float(ID_DC2_H_AM_H_RATE2)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) );
  
  discrete_aftermid_dcurrent2_part1 = da2_h_ang1 + da2_h_rate1;
  discrete_aftermid_dcurrent2_part2 = da2_h_ang2 + da2_h_rate2;
}  
  
if ( ((int) get_io_float(ID_D_H_ON)) == 3 || ((int)get_io_float(ID_D_H_ON)) == 1 ) //Discrete control
{ 
  da3_l_rate1 = -get_io_float(ID_DC3_H_AM_L_ABSRATE1)*(robot_velocity_mid_stance - get_io_float(ID_DP_H_AM_L_ABSRATE) );
  da3_l_rate2 = -get_io_float(ID_DC3_H_AM_L_ABSRATE2)*(robot_velocity_mid_stance - get_io_float(ID_DP_H_AM_L_ABSRATE) );
  da3_h_ang1 = -get_io_float(ID_DC3_H_AM_H_ANG1)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
  da3_h_ang2 =  -get_io_float(ID_DC3_H_AM_H_ANG2)*(robot_hip_angle_mid_stance - get_io_float(ID_DP_H_AM_H_ANG) );
  da3_h_rate1 = -get_io_float(ID_DC3_H_AM_H_RATE1)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) );
  da3_h_rate2 =  -get_io_float(ID_DC3_H_AM_H_RATE2)*(robot_hip_velocity_mid_stance - get_io_float(ID_DP_H_AM_H_RATE) );
 
  discrete_aftermid_dcurrent3_part1 = da3_l_rate1 + da3_h_ang1 + da3_h_rate1;
  discrete_aftermid_dcurrent3_part2 = da3_l_rate2 + da3_h_ang2 + da3_h_rate2;
}

if (((int)get_io_float(ID_D_H_ON)) == 2) //push-off controller
{
  discrete_ho_aftermid_dcurrent_part1 = discrete_aftermid_dcurrent2_part1;
  discrete_ho_aftermid_dcurrent_part2 = discrete_aftermid_dcurrent2_part2;
}
else if (((int)get_io_float(ID_D_H_ON)) == 3) //step length controller
{
  discrete_ho_aftermid_dcurrent_part1 = discrete_aftermid_dcurrent3_part1;
  discrete_ho_aftermid_dcurrent_part2 = discrete_aftermid_dcurrent3_part2;
}
else if (((int)get_io_float(ID_D_H_ON)) == 1)
{
  if (robot_velocity_mid_stance >= get_io_float(ID_DP_F_PP_L_ABSRATE) ) //robot moving faster use step length controller, use controller 3
  {
      discrete_ho_aftermid_dcurrent_part1 = discrete_aftermid_dcurrent3_part1;
      discrete_ho_aftermid_dcurrent_part2 = discrete_aftermid_dcurrent3_part2;
      da2_h_ang1 =0.0;   da2_h_ang2 = 0.0;
      da2_h_rate1 =0.0;  da2_h_rate2 = 0.0;
  }
  else //use push-off controller, use controller 2
  {
      discrete_ho_aftermid_dcurrent_part1 = discrete_aftermid_dcurrent2_part1;
      discrete_ho_aftermid_dcurrent_part2 = discrete_aftermid_dcurrent2_part2;
      da3_h_ang1 = 0.0; da3_h_ang2 = 0.0;
      da3_l_rate1 = 0.0; da3_l_rate2 = 0.0;
      da3_h_rate1 = 0.0; da3_h_rate2 = 0.0;
  }
}

  set_io_float(ID_DA2_H_AM_H_ANG1,da2_h_ang1); set_io_float(ID_DA2_H_AM_H_ANG2,da2_h_ang2);
  set_io_float(ID_DA2_H_AM_H_RATE1,da2_h_rate1); set_io_float(ID_DA2_H_AM_H_RATE2,da2_h_rate2);
  
  set_io_float(ID_DA3_H_AM_L_ABSRATE1,da3_l_rate1); set_io_float(ID_DA3_H_AM_L_ABSRATE2,da3_l_rate2);
  set_io_float(ID_DA3_H_AM_H_ANG1,da3_h_ang1); set_io_float(ID_DA3_H_AM_H_ANG2,da3_h_ang2);
  set_io_float(ID_DA3_H_AM_H_RATE1,da3_h_rate1); set_io_float(ID_DA3_H_AM_H_RATE2,da3_h_rate2);
  
  
  set_io_float(ID_DA_H_AM_A1,discrete_ho_aftermid_dcurrent_part1);
  set_io_float(ID_DA_H_AM_A2,discrete_ho_aftermid_dcurrent_part2);
    
  return 1;
}

int ACT_HO_aftermid(void)
{

float command_current= 0.0;
float stiffness = 0.0;
float const_current = 0.0;
float dampness = 0.0; 

float discrete_ho_aftermid_dcurrent = 0.0; 
if (   ( ((int) get_io_float(ID_D_H_ON)) == 3)
       ||
       (  ((int) get_io_float(ID_D_H_ON)) == 2)
       ||
       (  ((int) get_io_float(ID_D_H_ON)) == 1) ) //Discrete control
{ 
      if  (ho_aftermid_time >= 0 && ho_aftermid_time < get_io_float(ID_DP_H_AM_TIME1) )
        discrete_ho_aftermid_dcurrent =  discrete_ho_aftermid_dcurrent_part1;
        
      if  (ho_aftermid_time >= get_io_float(ID_DP_H_AM_TIME1) && ho_aftermid_time < get_io_float(ID_DP_H_AM_TIME2) )
        discrete_ho_aftermid_dcurrent =  discrete_ho_aftermid_dcurrent_part2;
}      
       

 if  ( (get_io_float(ID_E_T_AFTER_HS)- time_at_ss0) < get_io_float(ID_P_H_PM_TIME) )
 {
    //command_current = get_io_float(ID_C_H_PM_H_RATE)*(-get_io_float(ID_P_H_PM_H_TRATE) - get_io_float(ID_E_H_RATE));  
    command_current = -get_io_float(ID_A_H_PM_A0); 
  }  
  
   /*if (command_current>0)
      command_current = 0.0; */

   /*if (get_io_float(ID_MCH_ANGLE)>=0.0) //add stiffness in first half
       stiffness = get_io_float(ID_C_H_OUTER_STIFFNESS);
    else //or add constant current in the second half
    {
        const_current = get_io_float(ID_A_H_PM_A0);
        dampness = -get_io_float(ID_C_H_OUTER_DAMPNESS);
    }*/
        
  stiffness = get_io_float(ID_C_H_OUTER_STIFFNESS);             
  set_io_float(ID_MCH_STIFFNESS, stiffness);  
  set_io_float(ID_MCH_DAMPNESS, dampness);
  set_io_float(ID_MCH_COMMAND_CURRENT, command_current-discrete_ho_aftermid_dcurrent-const_current);   

  return 1;
}

int ACT_HI_ehold_entry(void)
{
  return 1;
}

int ACT_HI_ehold(void)
{ 
   set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
   set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
   set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG)); 
  
  return 1;
}

int ACT_HO_ehold_entry(void)
{
  return 1;
}

int ACT_HO_ehold(void)
{
    set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
    set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
    set_io_float(ID_MCH_COMMAND_CURRENT, - get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  
   return 1;
}

int ACT_H_stop(void)
{
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

