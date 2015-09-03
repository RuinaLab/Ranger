/*

	mutex.c
	
	A simple C mutex. 
	
	Nicolas Williamson - January 2010
	
*/

#include <includes.h>


void mutex_lock(MUTEX* mutex_ptr){
  while (mutex_check(mutex_ptr)){} //block until mutex is unlocked
  *mutex_ptr = 1; //locked
}

void mutex_unlock(MUTEX* mutex_ptr){
  *mutex_ptr = 0; //unlocked
}

int mutex_check(MUTEX* mutex_ptr){
  return *mutex_ptr;
}
