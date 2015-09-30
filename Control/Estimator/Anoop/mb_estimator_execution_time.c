#include <mb_includes.h>

static float estimator_start_time = 0.0; 

void mb_estimator_execution_time_start(void)
{
  estimator_start_time =  mb_clock_get_execution_time();
}

void mb_estimator_execution_time_stop(void)
{
  mb_io_set_float(ID_E_EXEC_TIME, ( mb_clock_get_execution_time() - estimator_start_time ));  
  // this returns the difference between times at start and at end of execution of (TASK_PTR)&mb_estimator_update (basically my whole estimation module)
  // which is the main  file that call all estimator functions, its called in the tast every row of scheduler in mb_software_setup.c . 
}
