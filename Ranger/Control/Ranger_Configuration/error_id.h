//
// Error Names and Comments for Cornell Ranger Robot
//

#ifndef __H_ERROR_ID__
#define __H_ERROR_ID__ 

typedef enum errors{
	ERROR_DEFAULT,
	ERROR_ADCI_BAD_CHAN,	//	ADCI: Attempted to read from invalid channel
	ERROR_ADCI_DNF,	//	ADCI: Did not finish previous conversion
	ERROR_ADCI_FILT_OOB,	//	ADCI: Filter number was out of bounds
	ERROR_CAN_AFRAM_FULL,	//	CAN: Acceptance filter RAM is full - probably an error in the rx descriptors list
	ERROR_CAN_ACC_FILTER,	//	CAN: The acceptance filter table has an error
	ERROR_CAN_DUP_RX_IDS,	//	CAN: Acceptance filter init function found a pair of rx desciptors with identical IDs
	ERROR_CAN_RXLST_OF,	//	CAN: Receive list overflow
	ERROR_CAN_RTRLST_OF,	//	CAN: RTR list overflow
	ERROR_CAN1_TX_BUSOFF,	//	CAN1: Excessive transmit errors on this controller led to bus off condition
	ERROR_CAN1_ERR_WARN,	//	CAN1: Error warning- shows change in error or bus status bits in either direction
	ERROR_CAN1_DATA_OVRN,	//	CAN1: Receive data overrun - buffer not read before next frame- data lost
	ERROR_CAN1_ERR_PASS,	//	CAN1: Change in active or passive error status in either direction
	ERROR_CAN1_BUS_RX,	//	CAN1: Bus error during receive
	ERROR_CAN1_BUS_TX,	//	CAN1: Bus error during transmit
	ERROR_CAN1_LOCKUP,	//	CAN1: CAN controller lockup
	ERROR_CAN1_RX_BUF_OF,	//	CAN1: Software receive buffer overflow
	ERROR_CAN2_TX_BUSOFF,	//	CAN2: Excessive transmit errors on this controller led to bus off condition
	ERROR_CAN2_ERR_WARN,	//	CAN2: Error warning- shows change in error or bus status bits in either direction
	ERROR_CAN2_DATA_OVRN,	//	CAN2: Receive data overrun - buffer not read before next frame- data lost
	ERROR_CAN2_ERR_PASS,	//	CAN2: Change in active or passive error status in either direction
	ERROR_CAN2_BUS_RX,	//	CAN2: Bus error during receive
	ERROR_CAN2_BUS_TX,	//	CAN2: Bus error during transmit
	ERROR_CAN2_LOCKUP,	//	CAN2: CAN controller lockup
	ERROR_CAN2_RX_BUF_OF,	//	CAN2: Software receive buffer overflow
	ERROR_CAN3_TX_BUSOFF,	//	CAN3: Excessive transmit errors on this controller led to bus off condition
	ERROR_CAN3_ERR_WARN,	//	CAN3: Error warning- shows change in error or bus status bits in either direction
	ERROR_CAN3_DATA_OVRN,	//	CAN3: Receive data overrun - buffer not read before next frame- data lost
	ERROR_CAN3_ERR_PASS,	//	CAN3: Change in active or passive error status in either direction
	ERROR_CAN3_BUS_RX,	//	CAN3: Bus error during receive
	ERROR_CAN3_BUS_TX,	//	CAN3: Bus error during transmit
	ERROR_CAN3_LOCKUP,	//	CAN3: CAN controller lockup
	ERROR_CAN3_RX_BUF_OF,	//	CAN3: Software receive buffer overflow
	ERROR_CAN4_TX_BUSOFF,	//	CAN4: Excessive transmit errors on this controller led to bus off condition
	ERROR_CAN4_ERR_WARN,	//	CAN4: Error warning- shows change in error or bus status bits in either direction
	ERROR_CAN4_DATA_OVRN,	//	CAN4: Receive data overrun - buffer not read before next frame- data lost
	ERROR_CAN4_ERR_PASS,	//	CAN4: Change in active or passive error status in either direction
	ERROR_CAN4_BUS_RX,	//	CAN4: Bus error during receive
	ERROR_CAN4_BUS_TX,	//	CAN4: Bus error during transmit
	ERROR_CAN4_LOCKUP,	//	CAN4: CAN controller lockup
	ERROR_CAN4_RX_BUF_OF,	//	CAN4: Software receive buffer overflow
	ERROR_QDC_CH01_GLTCH,	//	QDC: QDC channel 01 glitch detected
	ERROR_QDC_CH23_GLTCH,	//	QDC: QDC channel 23 glitch detected
	ERROR_MC_TCURR_OOB,	//	MC: Requested target current is out of bounds
	ERROR_MC_PWM_OF,	//	MC: PWM overflow
	ERROR_MC_PWM_UF,	//	MC: PWM underflow
	ERROR_MC_CURR_OOB,	//	MC: Current is out of bounds
	ERROR_MC_SHUTOFF,	//	MC: Motor controller safety shutoff
	ERROR_MC_TEMP_SAFE,	//	MC: Safe temperature limit reached
	ERROR_ADCX_DNF,	//	ADCX: Previous conversion did not finish
	ERROR_ADCX_TX_FULL,	//	ADCX: Transmit buffer full
	ERROR_ADCX_RX_EMPTY,	//	ADCX: Receive buffer empty
	ERROR_ADCX_NUM_CFGS,	//	ADCX: Too many configs have been set up
	ERROR_BUTTON_OOB,	//	BUTTON: Invalid button- out of bounds
	ERROR_LCD_STR_OF,	//	LCD: String length overflow
	ERROR_LCD_POS_OOB,	//	LCD: Position out of bounds
	ERROR_AE_INVALID_ID,	//	AE: Invalid encoder ID
	ERROR_LS_INVALID_ID,	//	LS: Invalid limit switch ID
	ERROR_LS_NUM_SWITCH,	//	LS: Too many switches added
	ERROR_SCHED_ASYNC,	//	SCHED: Scheduler is now asynchronous
	ERROR_SCHED_RESYNC,	//	SCHED: Scheduler has resynced with Main Brain
	ERROR_SCHED_OVERRUN,	//	SCHED: Scheduler overrun- not able to finish in time
	ERROR_ASCHED_DNF,	//	ASCHED: Asynchronous scheduler did not finish in time
	ERROR_MSIMU_TX_FULL,	//	MSIMU: Transmit buffer full
	ERROR_MSIMU_NUM_BYT,	//	MSIMU: Too many bytes
	ERROR_MSIMU_RX_OF,	//	MSIMU: Receive buffer overflow
	ERROR_UTIL_DUMMY,	//	UTIL: Dummy function called
	ERROR_STATE_INVALID,	//	STATE: Invalid state requested
	ERROR_MCFI_ANKL_LS,	//	MCFI: Middle ankles limit switch activated
	ERROR_MCFI_ANKL_HI,	//	MCFI: Middle ankles have reached upper limit (negative direction)
	ERROR_MCFI_ANKL_LO,	//	MCFI: Middle ankles have reached lower limit (positive direction)
	ERROR_MCFO_ANKL_LS,	//	MCFO: Outer ankles limit switch activated
	ERROR_MCFO_ANKL_HI,	//	MCFO: Outer ankles have reached upper limit (negative direction)
	ERROR_MCFO_ANKL_LO,	//	MCFO: Outer ankles have reached lower limit (positive direction)
	ERROR_MCSI_LEFT,	//	MCSI: New steering motor has reached left steer angle limit (positive direction)
	ERROR_MCSI_RIGHT,	//	MCSI: New steering motor has reached right steer angle limit (negative direction)
	ERROR_MB_SSP_TX_FULL,	//	MB_SSP: DMA transmit buffer full (data going to CAN bus lost due to overflow)
	ERROR_MB_SSP_CHKSUM,	//	MB_SSP: Received packet had bad checksum value
	ERROR_MB_SSP_DMA_JAM,	//	MB_SSP: DMA transfer could not be halted
	ERROR_MB_SSP_RX_FULL,	//	MB_SSP: DMA receive buffer full (data coming from CAN bus lost due to overflow)
	ERROR_MB_SER_DMA_JAM,	//	MB_SER: DMA transfer could not be halted
	ERROR_MB_SER_RX_FULL,	//	MB_SER: software receive buffer full (data coming from serial port (Bluetooth- PC) lost due to overflow)
	ERROR_MB_SER_DMA_FLL,	//	MB_SER: DMA hardware receive buffer full (data coming from serial port (Bluetooth- PC) lost due to overflow)
	ERROR_MB_SER_DMA_OFF,	//	MB_SER: Receive DMA transfer stopped unexpectedly or failed to start properly
	ERROR_MB_SER_CHKSUM,	//	MB_SER: Checksum error found in received packet from serial port (Bluetooth- PC)
	ERROR_SSP_RX_BUF_OF,	//	SSP: Receive SSP buffer overflow (data coming in from main brain may be lost)
	ERROR_SSP_LOW_RATE,	//	SSP: SSP bit rate is too slow to finish segment within allowed time. Increase rate- time- or decrease seg size.
	ERROR_SSP_TX_BUF_IDX,	//	SSP: Weird SSP transmit index error
	ERROR_SSP_RX_FIFO,	//	SSP: Receive SSP FIFO overflow
	ERROR_SSP_BAD_CHKSM,	//	SSP: Bad SSP receive packet checksum
	ERROR_SSP_RX_BUF_IDX,	//	SSP: Corrupted receive buffer index
	ERROR_SSP_TX_BUF_OF,	//	SSP: Software transmit buffer full (data coming in from CAN buses may be lost)
	ERROR_RCX_BAD_CHAN,	//	RCX: Attempting to read from a channel that is not active
	ERROR_RCX_NINIT,	//	RCX: The rc receive module has not been initialized properly. Call rcx_init()
	ERROR_RCX_BAD_LVLS,	//	RCX: The capture levels have gone high-high or low-low- indicating a missed pulse
	ERROR_MB_IO_ID_OOR,	//	MB_IO: An attempt was made to access data with a DATA_ID higher than ID_LAST
	ERROR_MB_SSP_ID_OOR,	//	MB_SSP: Received data frame from SSP with a DATA_ID higher than ID_LAST
	ERROR_MC_TEMP_OFF,	//	MC: Motor controller turned off due to thermal limit
	ERROR_MC_MECH_OFF,	//	MC: Motor controller turned off due to mechanical limit
	ERROR_MC_TEMP_ON,	//	MC: Motor controller turned back on after thermal turnoff; integrator below safe limit
	ERROR_MC_MECH_ON,	//	MC: Motor controller turned back on after mechanical turnoff; integrator back to 0
	ERROR_QDC_CH01_BUF_F,	//	QDC: QDC channel 01 time buffer overflow
	ERROR_QDC_CH23_BUF_F,	//	QDC: QDC channel 23 time buffer overflow
	ERROR_MCFI_CBL_STRCH,	//	MCFI: Inner ankle cable stretched during flipup
	ERROR_MCFO_CBL_STRCH,	//	MCFO: Outer ankle cable stretched during flipup
	ERROR_MC_INTEG_SATUR,	//	MC: Integral of PID current control is saturated
	ERROR_MC_FIXED_LIMIT,	//	MC: PID value has reached Fixed point limit
	ERROR_MC_PWM_LIMIT,	//	MC: Requested PWM has reached limit
	ERROR_MC_NOT_RUNNING,	//	MC: MC is not running due to off sleep shutdown thermal or mechanical states
	ERROR_AE_SPIKE,	//	AE: A spike was detected and limited. 
	ERROR_EST_FILTER_TIME_VIOLATION,	//	EST: An invalid time-stamp was passed to the butterworth filter.
	ERROR_EST_ROBOT_FALL,	//	EST: Estimator detected that the robot fell down. Aborting walk  --   force transition to stand-by
	ERROR_EST_INCOMPLETE_TRIAL,	//	EST: You accepted a trial before the minimum required number of steps!!!
	ERROR_OPTIM_STUTTER_STEP,	//	OPTIM: Robot failed to satisfy minimum step time. Stutter Step occurred.
	ERROR_LAST_ID
} ERROR_ID;


#endif //__H_ERROR_ID__

