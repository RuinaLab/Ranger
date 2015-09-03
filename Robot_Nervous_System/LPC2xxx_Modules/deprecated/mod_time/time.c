/*
	time.c - very very simple millisecond counter for use on ranger 
			will probably be rewritten to do something more interesting/be more useful
	
	Nicolas Williamson - July 2009
	
*/

#include <includes.h>

unsigned long long int time_in_milliseconds = 0;

void time_tick(void){ //call every millisecond
	time_in_milliseconds++;
}

unsigned long long int time_elapsed(void){
	return time_in_milliseconds;
}
