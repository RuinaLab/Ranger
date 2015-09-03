/*

	mutex.h
	
	Nicolas Williamson - January 2010
	
*/

#ifndef __H_MUTEX__
#define __H_MUTEX__

typedef enum mutex_states {
  MUTEX_UNLOCKED = 0,
  MUTEX_LOCKED
} MUTEX_STATE;

typedef volatile unsigned int MUTEX;

void mutex_lock(MUTEX* mutex_ptr); //Blocking; locks the mutex, waiting for the mutex to become unlocked if necessary
void mutex_unlock(MUTEX* mutex_ptr); //unlocks the mutex if it was locked, does nothing otherwise
int mutex_check(MUTEX* mutex_ptr); //returns true (1) if the mutex is locked, false (0) otherwise
 
#endif
