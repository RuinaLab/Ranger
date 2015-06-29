/*
	Gradient Based Reinforcement Learning algorithm
	to optimize control parameters for different environments
	onboard Ranger.
	
	@author Nicolas Champagne-Williamson
	@date September 2010
*/

#ifndef __H_LEARNING__
#define __H_LEARNING__

//Public Functions
void gbrl(void);

//Private Functions
void gbrl_calc_grad(void);
void gbrl_update_policy(void);
void gbrl_perturb(void);
void gbrl_deperturb(void);

#endif

