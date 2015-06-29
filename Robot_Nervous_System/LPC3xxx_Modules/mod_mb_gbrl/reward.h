/**
	Reward Function Estimator
	
	@author Nicolas Champagne-Williamson
	@date October 2010
*/

#ifndef __H_REWARD__
#define __H_REWARD__

//Functions
float gbrl_calc_reward(void);
float gbrl_calc_power(void);
float gbrl_calc_velocity(void);
float gbrl_calc_stability(void);
void gbrl_average_reward(void);
void gbrl_average_power(void);
void gbrl_average_velocity(void);
void gbrl_average_stability(void);

#endif /* __H_REWARD__ */
