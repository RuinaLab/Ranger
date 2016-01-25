/* read_global_sensors.h

   Author: Chandana Paul
   Date: November 25, 2009

   This file defines the function read_global_sensors() which reads the 
   sensor values from the robot and inserts them into the global sensor
   variables defined in global_sensors.h. This function has to be called 
   at every time step from the main function to reload new sensors values. 

*/

#ifndef _READ_GLOBAL_SENSORS_H
#define _READ_GLOBAL_SENSORS_H

void read_global_sensors();

#endif
