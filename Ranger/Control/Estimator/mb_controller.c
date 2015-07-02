#include <mb_includes.h> 

//  This function is the entry-point for all controller stuff. It is called by the scheduler.
void mb_controller_update(void){

 mb_io_set_float( ID_E_TEST10, 1.23456);  // Dummy - for testing. ////HACK////

}
