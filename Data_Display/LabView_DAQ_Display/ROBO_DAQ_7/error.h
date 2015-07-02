/*
	error.h - Header file for the error handling module
	
	Contains all of the error codes and functions for error reporting over CAN.
	
	Nicolas Williamson - Summer 2009
	
*/

#ifndef __ROBOT_ERROR_H__
#define __ROBOT_ERROR_H__

#define ERR_BUF_SIZE 32
#define MED_RESERVE 4
#define LOW_RESERVE 8

//Error types/priority
typedef enum error_priorities {
  PRIORITY_LOW = 0, //Ranger should log that this happened, but doesn't need to know
  PRIORITY_MED = 1, //Ranger might need to know about this, but probably fine
  PRIORITY_HIGH = 2, //Ranger needs to deal with this, something is seriously wrong
  PRIORITY_MAX
//  PRIORITY_FATAL = 3 //Ranger poses immediate danger to surrounding humans
} ERROR_PRIORITY;

//Specific board IDs - Naming: BOARD_<type>_<specific>
typedef enum board_vals{
	BOARD_DEFAULT = 0,
	BOARD_MAIN_BRAIN,
	BOARD_MC_INVERTATRON,
  BOARD_MCH,
	BOARD_MCSI, //the Maxon Motor steering
	BOARD_MCSO, //the old RC steering
  BOARD_MCFI, //inner feet
  BOARD_MCFO, //outer feet
	BOARD_UI,
	BOARD_CAN_SNIFFER,
	BOARD_CAN_ROUTER
} BOARD_ID;

//Specific error IDs - Naming: ERROR_<module>_<name>
typedef enum error_vals{
	ERROR_DEFAULT = 0, //Default
	ERROR_ADCI_BAD_CHAN,    //ADCI: Attempted to read from invalid channel.
	ERROR_ADCI_DNF,         //ADCI: Did not finish previous conversion.
	ERROR_ADCI_FILT_OOB,    //ADCI: Filter number was out of bounds.
	ERROR_CAN_WARNING,      //CAN:  Warning
	ERROR_CAN_DATA_FULL,    //CAN:  Data buffer full.
	ERROR_CAN_WAKE_UP,      //CAN:  Wake-up error.
	ERROR_CAN_PASSIVE,      //CAN:  
	ERROR_CAN_ARBITRATION_LOST,
	ERROR_CAN_BUS_TX,
	ERROR_CAN_BUS_RX,
	ERROR_CAN_RX_LIST_OVERFLOW,
	ERROR_CAN_RTR_LIST_OVERFLOW,
	ERROR_QEC_BOTH_CHS,
	ERROR_QEC_INVALID_ID,
	ERROR_MC_TARGET_CURRENT_OUT_OF_BOUNDS,
	ERROR_MC_PWM_OVERFLOW,
	ERROR_MC_PWM_UNDERFLOW,
	ERROR_MC_CURRENT_OUT_OF_BOUNDS,
	ERROR_MC_SHUTOFF,
  ERROR_MC_TEMP_LIMIT,
	ERROR_ADCX_CONVERSION_INCOMPLETE,
	ERROR_ADCX_TRANSMIT_BUFFER_FULL,
	ERROR_ADCX_RECEIVE_BUFFER_EMPTY,
	ERROR_ADCX_TOO_MANY_CONFIGS,
	ERROR_BUTTON_OUT_OF_RANGE,
	ERROR_LCD_STRING_OVERFLOW,
	ERROR_LCD_POSITION_OUT_OF_BOUNDS,
	ERROR_AE_INVALID_ENCODER,
	ERROR_LS_INVALID_ID,
  ERROR_LS_TOO_MANY_SWITCHES,
  ERROR_SCHED_ASYNC,
  ERROR_SCHED_RESYNC,
  ERROR_SCHED_OVERRUN,
  ERROR_ASCHED_OVERRUN,
  ERROR_MSIMU_TRANSMIT_FULL,
  ERROR_MSIMU_TOO_MANY_BYTES,
  ERROR_MSIMU_RX_BUFFER_OVERFLOW,
  ERROR_UTIL_DUMMY_CALLED,
  ERROR_STATE_INVALID_STATUS,
  ERROR_LAST_ID
} ERROR_ID;

typedef struct error_info {
	ERROR_ID error_id; 
  ERROR_PRIORITY error_priority;
	unsigned short frequency;
	unsigned long long int time_occurred;
} ERROR_INFO;

typedef struct error_buffer{
	ERROR_INFO errors[ERR_BUF_SIZE];
	unsigned char first;
	unsigned char next;
	unsigned char overflows;
	unsigned char empty;
} ERROR_BUFFER;	


//Functions
void error_init(VOID_VOID_F f_transmit, INT_VOID_F f_time);
void error_occurred(ERROR_ID error_code, ERROR_PRIORITY priority);
void error_send_next(void);
void error_push(volatile ERROR_BUFFER* buffer, ERROR_INFO new_error);
int error_get_time(void);
int error_get_info(void);
volatile ERROR_INFO* error_pop(volatile ERROR_BUFFER* buffer);
void error_update(void);


#endif //__ROBOT_ERROR_H__

