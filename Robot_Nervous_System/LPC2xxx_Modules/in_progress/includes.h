#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#define NULL (0)

#include <LPC21xx.H>
#include <lpc2194def.h>

#include <hardware_setup.h>
#include <software_setup.h>

#include <mod_heartbeat\heartbeat.h>
#include <mod_can\can.h>
#include <mod_error\error.h>
//#include <mod_canprobe\canprobe.h>
#include <mod_scheduler\scheduler.h>
#include <mod_uart_int\uart_int.h>
#include <mod_filt\filt.h>
#include <mod_adcx\adcx.h>
#include <mod_adc_internal\adc_internal.h>
#include <mod_qec\qec.h>
#include <mod_motor_controller\motor_controller.h>
#include <mod_uart\UART.h>
#include <mod_tic_toc\tic_toc.h>
#include <mod_limit_switch\limit_switch.h>
#include <mod_ui_led\ui_led.h>
#include <mod_button\button.h>
#include <mod_lcd\lcd.h>
#include <mod_buzzer\buzzer.h>
#include <mod_abs_enc\abs_enc.h>

#include <data_nexus.h>
#include <local_headers.h>

#endif //__INCLUDES_H__

