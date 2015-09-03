#ifndef __INCLUDES_H__
#define __INCLUDES_H__

//NULL Definition
#define NULL (0)

/* STANDARD HEADERS */
#include <stdio.h>
#include <LPC21xx.H>
#include <lpc2194def.h>

/* ROBOT-SPECIFIC HEADERS */
#include <can_id.h>
#include <error_id.h>
#include <board_id.h>

/* BOARD SPECIFIC HEADERS */
#include <local_headers.h>

/* RANGER OS */
#ifndef __RANGER_OS_H__
#define __RANGER_OS_H__
//#warning No operating system defined, using default version.
#include <ranger_os_default\ranger_os.h>
#endif

/* MODULE HEADERS */
#include <mod_can\can.h>
#include <mod_heartbeat\heartbeat.h>
#include <mod_rc_receive\rc_receive.h>
#include <mod_canprobe\canprobe.h>
#include <mod_scheduler\scheduler.h>
#include <mod_uart_int\uart_int.h>
#include <mod_filt\filt.h>
#include <mod_adc_external\adc_external.h>
#include <mod_adc_internal\adc_internal.h>
#include <mod_qec\qec.h>
#include <mod_qdc\qdc.h>
#include <mod_motor_controller\motor_controller.h>
#include <mod_uart\UART.h>
#include <mod_limit_switch\limit_switch.h>
#include <mod_ui_led\ui_led.h>
#include <mod_button\button.h>
#include <mod_lcd\lcd.h>
#include <mod_buzzer\buzzer.h>
#include <mod_abs_enc\abs_enc.h>
#include <mod_microstrain_imu\microstrain_imu.h>
#include <mod_mcu_led\mcu_led.h>
#include <mod_can_ssp_router2\can_ssp_router.h>
#include <mod_ui_sync_led\ui_sync_led.h>
#include <mod_fun\song.h>
#include <mod_i2c_color\i2c_color.h>

/* SETUP HEADERS */
#include <hardware_setup.h>
#include <software_setup.h>
#include <data_nexus.h>
#include <interrupts.h>

#endif //__INCLUDES_H__

