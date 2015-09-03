#include <includes.h>
#include <can_id.h>

// NOTE: Errors disabled - error_send in sched

extern int global_running; 
// **********************  Variables  ************************** 

// *******************************************
// Scheduler Variables
// *******************************************
#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot
typedef TASK_PTR TP;
TASK_PTR schedule[]={
  (TP)hb_beat,(TP)task_every_row,(TP)NULL,//(TP)adcx_convert_all,(TP)adci_convert_all,(TP)ae_update,(TP)adcx_conversion_wait,/*(TP)mc_run_no_control,*/(TP)dn_safety,(TP)task_every_row,/*(TP)error_send_next,*/(TP)NULL, 
  (TP)hb_beat,(TP)task_every_row,(TP)NULL,//(TP)adcx_convert_all,(TP)adci_convert_all,(TP)ae_update,(TP)adcx_conversion_wait,/*(TP)mc_run_no_control,*/(TP)dn_safety,(TP)task_every_row,/*(TP)error_send_next,*/(TP)NULL, 
  (TASK_PTR)NULL
};
// *******************************************
// CAN Variables
// *******************************************
// ***** Transmit
CAN_FRAME_DESC error_tx_fd;
// ***** Receive
CAN_FRAME_DESC timestamp_rx_fd;



#define RX_FRAME_BUF_LEN 8
CAN_RING rx_ring;
CAN_FRAME rx_frame_buf[RX_FRAME_BUF_LEN];
#define TX_FRAME_BUF_LEN 8
CAN_RING tx_ring1;
CAN_FRAME tx_frame_buf1[TX_FRAME_BUF_LEN];
CAN_RING tx_ring2;
CAN_FRAME tx_frame_buf2[TX_FRAME_BUF_LEN];
#define RX_LIST_SIZE 10
CAN_FRAME_DESC * rxlist[RX_LIST_SIZE];
#define RTR_LIST_SIZE 10
CAN_FRAME_DESC * rtrlist[RTR_LIST_SIZE];
void fill_descriptors(void);


// **********************  Software Initialization  **************************
void init_software(void){

  // *********************************************
  // Standard board initialization.  Do not modify. 
  // *********************************************
  schedule_init(schedule,SCHEDULE_TICK_DIVIDER);
  error_init(dn_error_transmit, BOARD_UI);
  hb_init(500); //heartbeat

  // *********************************************
  // Put user initialization code below here.  
  // *********************************************


  // *******************************************
  // Set up CAN
  // *******************************************
  fill_descriptors();

  // ******** CAN1 ***********
//  can_ring_init(&rx_ring,rx_frame_buf,RX_FRAME_BUF_LEN);
  can_rx_set_descriptors(rxlist, rtrlist);
  can_rx_set_chan_cfg(CHAN_CAN1,(volatile unsigned long *)0xE0044000,NULL,CAN_DISPATCH_AUTO);

  can_ring_init(&tx_ring1,tx_frame_buf1,TX_FRAME_BUF_LEN);
  can_tx_set_chan_cfg(CHAN_CAN1,(volatile unsigned long *)0xE0044000,&tx_ring1);

   // ******** CAN2 ***********
//  can_ring_init(&rx_ring,rx_frame_buf,RX_FRAME_BUF_LEN);
  can_rx_set_chan_cfg(CHAN_CAN2,(volatile unsigned long *)0xE0048000,NULL,CAN_DISPATCH_AUTO);

  can_ring_init(&tx_ring2,tx_frame_buf2,TX_FRAME_BUF_LEN);
  can_tx_set_chan_cfg(CHAN_CAN2,(volatile unsigned long *)0xE0048000,&tx_ring2);
}

void init_values(void){ //startup calibrations and the like

}

void fill_descriptors(void){

  // ************* First, fill regular RX descriptors ***************** //  
  int i = 0;
  
  rxlist[i++] = &timestamp_rx_fd;  //Timestamp
  can_set_rx_descriptor_fi(&timestamp_rx_fd, ID_TIMESTAMP, can_rx_setter_float_dummy, schedule_set_timestamp);
  rxlist[i++] = (CAN_FRAME_DESC *) 0;  //NULL Terminator

  if(i >= RX_LIST_SIZE) {
    error_occurred(ERROR_CAN_RX_LIST_OVERFLOW, PRIORITY_HIGH);
  }

  // ************* NOW, fill RTR descriptors ************************** //
  i = 0;
  //Encoder Info CAN frame - 2 floats
//  rtrlist[i++] = &qec_fd;
//  can_set_tx_descriptor_ff(&qec_fd, 0x101, CAN_CHAN_1,abs_pos_1, velocity_1);
  //NULL Terminator
  rtrlist[i++] = (CAN_FRAME_DESC *) 0;

  if(i >= RTR_LIST_SIZE) {
    error_occurred(ERROR_CAN_RTR_LIST_OVERFLOW, PRIORITY_HIGH);
  }

  // ************** NOW, fill standard non-RTR TX descriptors (not in a list) *** //
//  can_set_tx_descriptor_ff(&qec_subscribe_fd, 0x101, CHAN_CAN1,abs_pos_1, velocity_1);

}

/********************* TASKS ***********************/
void task_every_row(void) {  

  dn_safety();
  task_can_transmit();

}

void task_can_transmit(void){

}

//TicToc
int tt_time = 0;
void tic(void){
  tt_time = T0TC;
}

void toc(void){
  tt_time = (T0TC - tt_time);
  printf("t: %i\n",tt_time);
  tt_time = 0;
}

void print_float(float f){
  printf("%f\n",f);
}

