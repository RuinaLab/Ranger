/**

	@file mutex.c
	
	A simple mutex in C. 
	
	@author Nicolas Williamson 
  @date January 2010
	
*/

#include <includes.h>

#ifndef __VERSION_0_1__
#warning RangerOS mismatch, expected v0.1. 
#endif

/**
  Locks a mutex. No other process which requires this mutex
  may run until the mutex is unlocked. This function will block
  and wait until the mutex it is trying to lock is free (no other process 
  has a lock on it).
  @param mutex_ptr A pointer to the mutex.
*/
void mutex_lock(MUTEX* mutex_ptr){
  while (mutex_check(mutex_ptr)){} //block until mutex is unlocked
  *mutex_ptr = 1; //locked
}

/**
  Unlocks a mutex. Once a process has finished executing its
  critical code, it should unlock the mutex to allow other processes
  to lock it.
  @param mutex_ptr A pointer to the mutex.
*/
void mutex_unlock(MUTEX* mutex_ptr){
  *mutex_ptr = 0; //unlocked
}

/**
  Checks whether a given mutex is locked. Non-blocking.
  @param mutex_ptr A pointer to the mutex.
  @return 
    - 1: The mutex is locked.
    - 0: The mutex is unlocked.
*/
int mutex_check(MUTEX* mutex_ptr){
  return *mutex_ptr;
}
