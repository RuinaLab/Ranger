#ifndef __MB_INCLUDES_H__
#define __MB_INCLUDES_H__


#define NULL (0)

#include <LPC325x.H>
#include <lpc3250def.h>

#include <rns_global_definitions.h>
#include <error_id.h>
#include <can_id.h>
#include <board_id.h>

#include <mod_mb_gbrl\learning.h> //Online Learning Algorithm
#include <mod_mb_gbrl\reward.h> //Reward Function Estimator
#include <mod_mb_gbrl\stability.h> //Stability Estimator

#include <mod_mb_uphill\mb_uphill.h> //Uphill Walking controls

#include <mb_hardware_setup.h>
#include <mb_software_setup.h>
#include <mb_data_nexus.h>

#include <mb_fsm.h>
//#include <control_code.h>		

#include <mod_mb_heartbeat\mb_heartbeat.h>
#include <mod_mb_io_data\mb_io_data.h>
#include <mod_mb_clock\a9_clock.h>
#include <mod_mb_error\mb_error.h>

#include <mod_mb_scheduler\mb_scheduler.h>

#include <mod_mb_ssp_dma2\mb_ssp_dma.h>

#include <mod_mb_bluetooth\mb_bluetooth.h>

#include <mb_estimator.h>
#include <mb_estimator_execution_time.h>

#include <mb_local_headers.h>


#endif //__MB_INCLUDES_H__

