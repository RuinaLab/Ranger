#ifndef __SAFEMODE_H__
#define __SAFEMODE_H__

/* This motor control mode is used to safely shut down the robot if it enters a 
 * bad state, by  putting a weak PD controller on all of the joints in their current
 * configuration. This prevents the robot from just dropping to the floor, but 
 * also keeps it from sending large currents */

void setSafeModeConfig(void);  // Sets the target angles for safe mode.

void safeMode_main(void);  // Call on each tick while safe mode is active

#endif
