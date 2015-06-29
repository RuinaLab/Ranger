/*
	Nic Uphill Walking Addon Module Extraordinaire
  
  Algorithm:
  
  run once at heelstrike:
    1) Measure Slope
      a = measure hip angle
      b = read imu roll data
      slope (rads) = (b + 0.024) + a/2
      slope (degs) = slope (rads) * 180/pi
    2) Set target ankle pushoff current
      ankle amps = 1.3 + 3.5 * slope (degs)
      limit ankle amps to 6 amps
    3) Set target hip pushoff current
      hip amps = ( 6.8 + 0.9 * slope(degs) ) / 2
      limit hip amps to 8 amps
      
    
  The slope of the ground beneath the robot is given by the equation
  IMU_ROLL + 0.024 + HIP_ANGLE/2. The robot forms an isosceles triangle
  with the ground slope, so HIP_ANGLE/2 radians away from either leg 
  points down perpendicular to the local ground slope. On level ground,
  the IMU_ROLL angle should match this angle (plus an offset of 0.024 because
  exactly upright the IMU box is slightly avertical) because the G vector is also 
  perpendicular to the ground. In general, to find the slope
  of the ground with respect to the world, we simply find the difference
  between the gravity vector (roll) and where the robot says g is (angle/2). 
  
*/

#include <mb_includes.h>

void walk_uphill(void){
  float hip_angle; //the angle between the legs in radians
  float imu_angle; //the roll of the IMU box with respect to gravity (world frame)
  float abs_angle; //Anoop's estimate of absolute angle of outer leg
  float slope_rads; //
  float slope_degs;
  float ankle_amps;
  float ankle_ang;
  float hip_amps;
  float hip_rate;
  
  // 1) Measure slope
  hip_angle = mb_io_get_float(ID_MCH_ANGLE);
  imu_angle = mb_io_get_float(ID_UI_ROLL);
  abs_angle = mb_io_get_float(ID_E_LO_ABSANG);
  slope_rads = (imu_angle /*+ 0.021*/) + abs_angle;//(hip_angle - 0.01)/2.0;
  slope_degs = (slope_rads * 180.0/3.14159) - 0.5; //change to degrees, minus offset (drift? filtering constants?)
  set_io_float(ID_E_SLOPE, slope_degs);
  if (slope_degs < 0) {slope_degs = 0;}
  
  // 2) Set Target Ankle Params
  ankle_amps = 1.3 + 3.5 * slope_degs;
  ankle_ang = 2.25 + 0.045 * slope_degs;
  if (ankle_amps > 6.0) {ankle_amps = 6;}
  if (ankle_ang > 2.7) {ankle_ang = 2.7;}
  set_io_float(ID_P_F_ST_F_TANG, ankle_ang); // currently 2.0rads
  
  // 3) Set Target Hip Params
  hip_amps = 3.4 + 0.45 * slope_degs;
  hip_rate = 1.8 - 0.054 * slope_degs + 0.032 * slope_degs * slope_degs; //was 1.9
  if (hip_amps > 4.0) {hip_amps = 4.0;}
  //ID_A_H_PM_A0 target hip current, not currently used
  set_io_float(ID_P_H_PM_H_TRATE, hip_rate); // need target hip rate, currently 2.0 rad/s
  
  

}




