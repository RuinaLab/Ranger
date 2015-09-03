#include <includes.h>

CAN_RING * can_probe_ring;

void can_probe_set_ring(CAN_RING * ring){
  can_probe_ring = ring;
}

void can_probe_spit(void){
  CAN_FRAME frame;
  int ret;
  while(1){
    ret = can_ring_pop(can_probe_ring,&frame);
    if(ret){
      return;
    } else {
      //printf("Addr: 0x%03X DLC: %0d RDA: 0x%08X RDB: 0x%08X\n",frame.addr,frame.dlc,frame.payload.w.w1,frame.payload.w.w2);
      printf("Chan: %d Addr: 0x%03X RTR: %d DLC: %0d RDA: %08f RDB: %08f\n",frame.chan,frame.addr,((int)frame.rtr),frame.dlc,frame.payload.f.f1,frame.payload.f.f2);
    }
  }
}

/*
#define CAN_PROBE_BUFFER_LENGTH 16

CAN_FRAME can_probe_buffer[CAN_PROBE_BUFFER_LENGTH];

//Make these globals volatile to allow mixed context usage.
volatile int can_probe_in_idx = CAN_PROBE_BUFFER_LENGTH-1;
volatile int can_probe_out_idx = CAN_PROBE_BUFFER_LENGTH-1;
volatile int never_populated = 1;

int can_probe_push(CAN_FRAME * frame) {
  int  ret;
  int next_in_idx = can_probe_in_idx + 1;
  
  if(next_in_idx >= CAN_PROBE_BUFFER_LENGTH){
    next_in_idx -= CAN_PROBE_BUFFER_LENGTH;
  }
  
  if(next_in_idx == can_probe_out_idx) {
    ret = 1;
  } else {
    can_probe_buffer[next_in_idx] = *frame;
    ret = 0;
    can_probe_in_idx = next_in_idx;
    never_populated = 0;
  }
  
  return ret;
}

int can_probe_pop(CAN_FRAME * frame){
  int out_idx = can_probe_out_idx;
  int  ret;
  int next_out_idx = out_idx + 1;
    
  if(never_populated){
    return 1;
  }
  if(next_out_idx >= CAN_PROBE_BUFFER_LENGTH){
    next_out_idx -= CAN_PROBE_BUFFER_LENGTH;
  }

  if(next_out_idx == can_probe_in_idx){
    ret = 1;
  } else {
    *frame = can_probe_buffer[next_out_idx];
    ret = 0;
    can_probe_out_idx = next_out_idx;
  }
  

  return ret;
}

void can_probe_tx_next_frame(void) {

}

void can_probe_go(void){
  CAN_FRAME frame;
  int i;   
  int ret; 
  
  i = 1;
  while(1){
    frame.addr = i;
    frame.dlc = (C1RFS>>16)&0xF;
    frame.rtr = (C1RFS>>30)&0x1;
    frame.payload.w.w1 = C1RDA;
    frame.payload.w.w2 = C1RDB;
    ret = can_probe_push(&frame);
    if(ret)
      break;
    i=i+1;
 }
}

__irq void can_probe_can1rx_isr (void) {
  CAN_FRAME frame;
  
  volatile int can1icr = C1ICR;
  int ret;
  
  if(can1icr & 1) {//RBS is set
    if(C1GSR & 0x1) {
      frame.addr = C1RID;
      frame.dlc = (C1RFS>>16)&0xF;
      frame.rtr = (C1RFS>>30)&0x1;
      frame.payload.w.w1 = C1RDA;
      frame.payload.w.w2 = C1RDB;
      C1CMR = 1<<2; //Release data
      
      ret = can_probe_push(&frame);
//      printf(".");
      if(ret){
        IO1CLR = 1<<24;
      } else {
        IO1SET = 1<<24;
      }
    }  
  }
  
  VICVectAddr = 0;    // Clear interrupt in VIC. 
}

void can_error_isr(void) __irq
{
	volatile int can_status = C1ICR; //save state of capture register
	if (can_status & (1<<7)){
    C1MOD = 1;
		C1MOD = 0;//Turn off RM
	}	
			
	VICVectAddr = 0;	
}
*/
