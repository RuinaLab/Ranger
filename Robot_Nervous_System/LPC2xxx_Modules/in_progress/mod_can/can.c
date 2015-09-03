/*

	can.c
	
	A simpler version of Tommy's CAN module for use on Cornell's Ranger robot.
	
	Copyright Cornell University, 2009
	See copyright notice.
	
	Nicolas Williamson - Sept. 2009
	Using code by Tommy Craig, summer 09

*/

#include <includes.h>

// *******************************************************************************
// USER FUNCTIONS
// *******************************************************************************

void can_transmit(CAN_CH can_ch, CAN_ID data_id, float value){
	
}

void can_receive(CAN_CH can_ch){

}

void can_request(CAN_ID data_id){

}

void can_subscribe(CAN_ID data_id){

}

// *******************************************************************************
// HELPER FUNCTIONS
// *******************************************************************************

// *******************************************************************************
// RING BUFFERS
// *******************************************************************************

void can_push(CAN_BUFFER* buffer, CAN_FRAME new_frame)
{	
	// ****** CHECK FOR OVERFLOW ******
	if ((buffer->next == buffer->first) && !(buffer->empty)){ //the front has caught up to the tail!! it's eating itself!! nooooo
		buffer->overflows++;
		buffer->first++;
		if (buffer->first >= CAN_BUF_SIZE) { buffer->first = 0; } 
	}
	// ****** ADD NEW DATA ******	
	buffer->frames[buffer->next] = new_frames;
  // ****** INCREMENT ******
	buffer->next++; //increment the index to place the next error	
	if (buffer->next >= CAN_BUF_SIZE) { buffer->next = 0; }
	buffer->empty = 0; //buffer now has stuff in it
}
CAN_FRAME* can_pop(CAN_BUFFER* buffer)
{
	CAN_FRAME* can_frame = 0;
	if (!buffer->empty){
		can_frames = &(buffer->frames[buffer->first]);
		buffer->first++;
		if (buffer->first >= CAN_BUF_SIZE) { buffer->first = 0; }
		buffer->overflows = 0;		
		if (buffer->first == buffer->next){
			buffer->empty = 1;
		}
	}	
	return can_frame;
}

// *******************************************************************************
// RECEIVE/TRANSMIT INTERRUPTS
// *******************************************************************************

//Receive
__irq void can_rx1_isr(void){
  volatile int can1icr = C1ICR;
  can_receive(CAN_CH_1); //Get and dispatch received frame
  VICVectAddr = 0; // Clear interrupt in VIC. 
}
__irq void can_rx2_isr(void){
  volatile int can2icr = C2ICR;
  can_receive(CAN_CH_2); //Get and dispatch received frame
  VICVectAddr = 0; // Clear interrupt in VIC. 
}
__irq void can_rx3_isr(void){
  volatile int can3icr = C3ICR;
  can_receive(CAN_CH_3); //Get and dispatch received frame
  VICVectAddr = 0; // Clear interrupt in VIC. 
}
__irq void can_rx4_isr(void){
  volatile int can4icr = C4ICR;
  can_receive(CAN_CH_4); //Get and dispatch received frame
  VICVectAddr = 0; // Clear interrupt in VIC. 
}
//Transmit
__irq void can_tx1_isr(void){
  volatile int can1icr = C1ICR;
  can_tx_send_next_frame(CHAN_CAN1);
  VICVectAddr = 0;    // Clear interrupt in VIC. 
}
__irq void can_tx2_isr(void){
  volatile int can2icr = C2ICR;
  can_tx_send_next_frame(CHAN_CAN2);
  VICVectAddr = 0;    // Clear interrupt in VIC. 
}
__irq void can_tx3_isr(void){
  volatile int can3icr = C3ICR;
  can_tx_send_next_frame(CHAN_CAN3);
  VICVectAddr = 0;    // Clear interrupt in VIC. 
}
__irq void can_tx4_isr(void){
  volatile int can1icr = C4ICR;
  can_tx_send_next_frame(CHAN_CAN4);
  VICVectAddr = 0;    // Clear interrupt in VIC. 
}

// *******************************************************************************
// ERROR HANDLING
// *******************************************************************************
void can_error_isr(void) __irq
{
  volatile int can_status1 = C1ICR; //save state of capture register
  volatile int can_status2 = C2ICR; //save state of capture register
  volatile int can_status3 = C3ICR; //save state of capture register
  volatile int can_status4 = C4ICR; //save state of capture register
//  if (can_status & (1<<2)){error_occurred(ERROR_CAN_WARNING, PRIORITY_LOW);}
//  if (can_status & (1<<3)){error_occurred(ERROR_CAN_DATA_OVERRUN, PRIORITY_LOW);}
//  if (can_status & (1<<4)){error_occurred(ERROR_CAN_WAKE_UP, PRIORITY_LOW);}
//  if (can_status & (1<<5)){error_occurred(ERROR_CAN_PASSIVE, PRIORITY_LOW);}
//  if (can_status & (1<<6)){error_occurred(ERROR_CAN_ARBITRATION_LOST, PRIORITY_LOW);}
//  if (can_status & (1<<7)){
//    if (can_status & (1<<21)){
//      error_occurred(ERROR_CAN_BUS_RX, PRIORITY_LOW);
//    } else {
//      error_occurred(ERROR_CAN_BUS_TX, PRIORITY_LOW);
//    }
//  }

  if (C1GSR & (1<<7)){ //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
    C1MOD = 0; //reset the can bus
    can_tx_send_next_frame(CHAN_CAN1); //start the transmissions again
  }
  if (C2GSR & (1<<7)){ //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
    C2MOD = 0; //reset the can bus
    can_tx_send_next_frame(CHAN_CAN2); //start the transmissions again
  }
  if (C3GSR & (1<<7)){ //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
    C3MOD = 0; //reset the can bus
    can_tx_send_next_frame(CHAN_CAN3); //start the transmissions again
  }
  if (C4GSR & (1<<7)){ //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
    C4MOD = 0; //reset the can bus
    can_tx_send_next_frame(CHAN_CAN4); //start the transmissions again
  }

  VICVectAddr = 0;
}




