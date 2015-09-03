#include <includes.h>

#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot
	
TASK_PTR schedule[]={
	(TASK_PTR)task_every_row, (TASK_PTR)NULL, 
	(TASK_PTR)task_every_row, (TASK_PTR)NULL, 
	(TASK_PTR)NULL
	};
	
char string[100];

// ********************************CAN*********************************
#define RX_FRAME_BUF_LEN 8
CAN_RING rx_ring;
CAN_FRAME rx_frame_buf[RX_FRAME_BUF_LEN];
#define TX_FRAME_BUF_LEN 8
CAN_RING tx_ring;
CAN_FRAME tx_frame_buf[TX_FRAME_BUF_LEN];
#define RX_LIST_SIZE 10
CAN_FRAME_DESC error_fd;
CAN_FRAME_DESC * rxlist[RX_LIST_SIZE];

void fill_descriptors(void){
	int i = 0;
	//NULL Terminator
	rxlist[i++] = (CAN_FRAME_DESC *) 0;
	
	if(i >= RX_LIST_SIZE) {
		error_occurred(ERROR_CAN_RX_LIST_OVERFLOW, PRIORITY_HIGH);
	} 

	can_set_tx_descriptor_iss(&error_fd, 0x301, error_get_time, error_get_id, error_get_freq); 
}

void setup_software(void){

	/*********************************************
	* Scheduler Initialization.  Do not modify. *
	*********************************************/ 
	schedule_init(schedule,SCHEDULE_TICK_DIVIDER,SCHED_SPEED);
	
	/*********************************************
	* Put user initialization code below here.  *
	*********************************************/ 
	set_heartbeat_period_ms(500);
	
	// *******************************************
	// Set up ERROR
	// *******************************************
	error_init(&error_fd);

	// *******************************************
	// Set up CAN
	// *******************************************
	fill_descriptors();
	can_ring_init(&rx_ring,rx_frame_buf,RX_FRAME_BUF_LEN);
	can_rx_init(rxlist, CAN_DISPATCH_AUTO, &rx_ring); 
	can_ring_init(&tx_ring,tx_frame_buf,TX_FRAME_BUF_LEN);
	can_tx_init(&tx_ring);
	
	// *************************************************
	// Interrupt enables
	// *************************************************
	VICSoftIntClr = 0xffffffff;		 //clear all interupts - only do this once

	// ************ PRIORITY 2 ******************	
	//TIMER 1 FOR SCHEDULE
	VICVectAddr2 = (unsigned long)Timer1_ISR;
	VICVectCntl2 = (1<<5) | VIC_TIMER1; // (1<<5) is the interrupt enable bit, 5 is Timer1 interrupt
	VICIntEnable |= (1<<VIC_TIMER1); //Enable Timer1 (bit5)

	// ************ PRIORITY 8 ******************
	//UART (uart_int)
	VICVectAddr8 = (unsigned long)uarti_isr;
	VICVectCntl8 = 0x20 | VIC_UART0; // Timer1 Interrupt 
	VICIntEnable = 1 << VIC_UART0;   // Enable Timer1 Interrupt 

	// ************ PRIORITY 4 ******************
	//CAN RX
	VICVectAddr5 = (unsigned long)can_rx_isr;
	VICVectCntl5 = 0x20 | VIC_CAN1RX;
	VICIntEnable = 1 << VIC_CAN1RX;
  
	// ************ PRIORITY 5 ******************
	//CAN TX
	VICVectAddr4 = (unsigned long)can_tx_isr;
	VICVectCntl4 = 0x20 | VIC_CAN1TX;
	VICIntEnable = 1 << VIC_CAN1TX;
	
	// ************ PRIORITY 6 ******************
	//CAN ERRORS
	VICVectAddr6 = (unsigned long)can_error_isr;
	VICVectCntl6 = (1<<5) | VIC_CAN_AF;
	VICIntEnable = 1<<VIC_CAN_AF;

	C1IER = 1<<0; //Turn on Rx Interrupt
	C1IER |= 1<<1; //Turn on Tx 1 interrupt
	C1IER |= (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); //Turn on CAN error interrupts
//	C1IER |= (1<<6); //Arbitration Lost interrupt
	C1GSR = 0;

	C1CMR = 0x0E;
	C1MOD = 0;
}

#define X 250
#define Y 500
#define Z 5
#define WAIT 1000

extern int global_running;

// *********************************************** TASKS **********************************************

//this task is run every millisecond...  NOT
void task_every_row(void) //runs every 'time' in milliseconds
{

}	


