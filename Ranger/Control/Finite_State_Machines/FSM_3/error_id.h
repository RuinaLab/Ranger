#ifndef __ERROR_ID_H__
#define __ERROR_ID_H__

//Specific error IDs - Naming: ERROR_<module>_<name>
typedef enum error_vals{
	ERROR_DEFAULT = 0,    // <#Priority> <MOD>: <Error description>
	ERROR_ADCI_BAD_CHAN,  // 3 ADCI: Attempted to read from invalid channel
	ERROR_ADCI_DNF,       // 2 ADCI: Did not finish previous conversion
	ERROR_ADCI_FILT_OOB,  // 3 ADCI: Filter number was out of bounds
	ERROR_CAN_AFRAM_FULL, // 0 CAN: Acceptance filter RAM is full - probably an error in the rx descriptors list
  ERROR_CAN_ACC_FILTER, // 0 CAN: The acceptance filter table has an error
  ERROR_CAN_DUP_RX_IDS, // 0 CAN: Acceptance filter init function found a pair of rx desciptors with identical IDs
	ERROR_CAN1_TX_BUSOFF, // 1 CAN1: Excessive transmit errors on this controller led to bus off condition
	ERROR_CAN1_ERR_WARN,  // 0 CAN1: Error warning, shows change in error or bus status bits in either direction
  ERROR_CAN1_DATA_OVRN, // 2 CAN1: Receive data overrun - buffer not read before next frame, data lost
  ERROR_CAN1_ERR_PASS,  // 0 CAN1: Change in active or passive error status in either direction
  ERROR_CAN1_BUS_RX,    // 0 CAN1: Bus error during receive
  ERROR_CAN1_BUS_TX,    // 0 CAN1: Bus error during transmit
  ERROR_CAN1_LOCKUP,    // 0 CAN1: CAN controller lockup
  ERROR_CAN1_RX_BUF_OF, // 2 CAN1: Software receive buffer overflow
  ERROR_CAN2_TX_BUSOFF, // 1 CAN2: Excessive transmit errors on this controller led to bus off condition
	ERROR_CAN2_ERR_WARN,  // 0 CAN2: Error warning, shows change in error or bus status bits in either direction
  ERROR_CAN2_DATA_OVRN, // 2 CAN2: Receive data overrun - buffer not read before next frame, data lost
  ERROR_CAN2_ERR_PASS,  // 0 CAN2: Change in active or passive error status in either direction
  ERROR_CAN2_BUS_RX,    // 0 CAN2: Bus error during receive
  ERROR_CAN2_BUS_TX,    // 0 CAN2: Bus error during transmit
  ERROR_CAN2_LOCKUP,    // 0 CAN2: CAN controller lockup
  ERROR_CAN2_RX_BUF_OF, // 2 CAN2: Software receive buffer overflow
  ERROR_CAN3_TX_BUSOFF, // 1 CAN3: Excessive transmit errors on this controller led to bus off condition
	ERROR_CAN3_ERR_WARN,  // 0 CAN3: Error warning, shows change in error or bus status bits in either direction
  ERROR_CAN3_DATA_OVRN, // 2 CAN3: Receive data overrun - buffer not read before next frame, data lost
  ERROR_CAN3_ERR_PASS,  // 0 CAN3: Change in active or passive error status in either direction
  ERROR_CAN3_BUS_RX,    // 0 CAN3: Bus error during receive
  ERROR_CAN3_BUS_TX,    // 0 CAN3: Bus error during transmit
  ERROR_CAN3_LOCKUP,    // 0 CAN3: CAN controller lockup
  ERROR_CAN3_RX_BUF_OF, // 2 CAN3: Software receive buffer overflow
  ERROR_CAN4_TX_BUSOFF, // 1 CAN4: Excessive transmit errors on this controller led to bus off condition
	ERROR_CAN4_ERR_WARN,  // 0 CAN4: Error warning, shows change in error or bus status bits in either direction
  ERROR_CAN4_DATA_OVRN, // 2 CAN4: Receive data overrun - buffer not read before next frame, data lost
  ERROR_CAN4_ERR_PASS,  // 0 CAN4: Change in active or passive error status in either direction
  ERROR_CAN4_BUS_RX,    // 0 CAN4: Bus error during receive
  ERROR_CAN4_BUS_TX,    // 0 CAN4: Bus error during transmit
  ERROR_CAN4_LOCKUP,    // 0 CAN4: CAN controller lockup
  ERROR_CAN4_RX_BUF_OF, // 2 CAN4: Software receive buffer overflow
	ERROR_CAN_RXLST_OF,   // 3 CAN: Receive list overflow
	ERROR_CAN_RTRLST_OF,  // 3 CAN: RTR list overflow
	ERROR_QEC_BOTH_CHS,   // 1 QEC: Both channels interrupted at once
	ERROR_QEC_BAD_ID,     // 3 QEC: Invalid QEC_ID
	ERROR_MC_TCURR_OOB,   // 2 MC: Requested target current is out of bounds
	ERROR_MC_PWM_OF,      // 0 MC: PWM overflow
	ERROR_MC_PWM_UF,      // 0 MC: PWM underflow
	ERROR_MC_CURR_OOB,    // 1 MC: Current is out of bounds
	ERROR_MC_SHUTOFF,     // 2 MC: Motor controller safety shutoff
  ERROR_MC_TEMP_SAFE,   // 1 MC: Safe temperature limit reached
	ERROR_ADCX_DNF,       // 2 ADCX: Previous conversion did not finish
	ERROR_ADCX_TX_FULL,   // 0 ADCX: Transmit buffer full
	ERROR_ADCX_RX_EMPTY,  // 0 ADCX: Receive buffer empty
	ERROR_ADCX_NUM_CFGS,  // 3 ADCX: Too many configs have been set up
	ERROR_BUTTON_OOB,     // 3 BUTTON: Invalid button, out of bounds
	ERROR_LCD_STR_OF,     // 2 LCD: String length overflow
	ERROR_LCD_POS_OOB,    // 2 LCD: Position out of bounds
	ERROR_AE_INVALID_ID,  // 3 AE: Invalid encoder ID 
	ERROR_LS_INVALID_ID,  // 3 LS: Invalid limit switch ID
  ERROR_LS_NUM_SWITCH,  // 3 LS: Too many switches added
  ERROR_SCHED_ASYNC,    // 1 SCHED: Scheduler is now asynchronous
  ERROR_SCHED_RESYNC,   // 1 SCHED: Scheduler has resynced with Main Brain
  ERROR_SCHED_OVERRUN,  // 2 SCHED: Scheduler overrun, not able to finish in time
  ERROR_ASCHED_DNF,     // 2 ASCHED: Asynchronous scheduler did not finish in time
  ERROR_MSIMU_TX_FULL,  // 1 MSIMU: Transmit buffer full
  ERROR_MSIMU_NUM_BYT,  // 1 MSIMU: Too many bytes  
  ERROR_MSIMU_RX_OF,    // 1 MSIMU: Receive buffer overflow
  ERROR_UTIL_DUMMY,     // 3 UTIL: Dummy function called
  ERROR_STATE_INVALID,  // 2 STATE: Invalid state requested
  ERROR_MCFI_ANKL_LS,   // 2 MCFI: Middle ankles limit switch activated
  ERROR_MCFI_ANKL_HI,   // 1 MCFI: Middle ankles have reached upper limit (negative direction)
  ERROR_MCFI_ANKL_LO,   // 1 MCFI: Middle ankles have reached lower limit (positive direction)
  ERROR_MCFO_ANKL_LS,   // 2 MCFO: Outer ankles limit switch activated
  ERROR_MCFO_ANKL_HI,   // 1 MCFO: Outer ankles have reached upper limit (negative direction)
  ERROR_MCFO_ANKL_LO,   // 1 MCFO: Outer ankles have reached lower limit (positive direction)
  ERROR_MCSI_LEFT,      // 2 MCSI: New steering motor has reached left steer angle limit (positive direction)
  ERROR_MCSI_RIGHT,     // 2 MCSI: New steering motor has reached right steer angle limit (negative direction)
  ERROR_MB_SSP_TX_FULL, // 1 MB_SSP: DMA transmit buffer full (data going to CAN bus lost due to overflow)
  ERROR_MB_SSP_CHKSUM,  // 1 MB_SSP: Received packet had bad checksum value
  ERROR_MB_SSP_DMA_JAM, // 1 MB_SSP: DMA transfer could not be halted
  ERROR_MB_SSP_RX_FULL, // 1 MB_SSP: DMA receive buffer full (data coming from CAN bus lost due to overflow)
  ERROR_MB_SER_DMA_JAM, // 1 MB_SER: DMA transfer could not be halted
  ERROR_MB_SER_RX_FULL, // 1 MB_SER: software receive buffer full (data coming from serial port (Bluetooth, PC) lost due to overflow)
  ERROR_MB_SER_DMA_FLL, // 1 MB_SER: DMA hardware receive buffer full (data coming from serial port (Bluetooth, PC) lost due to overflow)
  ERROR_MB_SER_DMA_OFF, // 1 MB_SER: Receive DMA transfer stopped unexpectedly or failed to start properly
  ERROR_MB_SER_CHKSUM,  // 1 MB_SER: Checksum error found in received packet from serial port (Bluetooth, PC)
  ERROR_SSP_RX_BUF_OF,  // 1 CSR_SSP: Receive SSP buffer overflow (data coming in from main brain may be lost)
  ERROR_SSP_LOW_RATE,   // 3 CSR_SSP: SSP bit rate is too slow to finish segment within allowed time. Increase rate, time, or decrease seg size.
  ERROR_SSP_TX_BUF_IDX, // 2 CSR_SSP: Weird SSP transmit index error
  ERROR_SSP_RX_FIFO,    // 2 CSR_SSP: Receive SSP FIFO overflow
  ERROR_SSP_BAD_CHKSM,  // 1 CSR_SSP: Bad SSP receive packet checksum
  ERROR_SSP_RX_BUF_IDX, // 1 CSR_SSP: Corrupted receive buffer index
  ERROR_FAKE,           // 1 FAKE: Fake error for testing purposes
  ERROR_RCX_BAD_CHAN,   // 3 RCX: Attempting to read from a channel that is not active
  ERROR_RCX_NINIT,      // 3 RCX: The rc receive module has not been initialized properly. Call rcx_init()
  ERROR_RCX_BAD_LVLS,   // 1 RCX: The capture levels have gone high-high or low-low, indicating a missed pulse
  ERROR_LAST_ID 
} ERROR_ID;




#endif    //__ERROR_ID_H__
