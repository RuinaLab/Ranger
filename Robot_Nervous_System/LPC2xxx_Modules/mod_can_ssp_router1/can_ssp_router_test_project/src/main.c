#include <includes.h>

CAN_FRAME test_can_frame;
extern CAN_RING * csr_ssp_tx_ring_ptr;

CAN_FRAME Example_Frame;

void send_CAN1 (void){
  
    if (C1SR & (1<<2)) {   // buffer 1 is available
      C1TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID1 = 15;                     // ID = 15
     
      /* STATIC DATA */
      C1TDA1 = 0x11111111;            // load 0xAAAAAAAAAAAAAAAA;
      C1TDB1 = 0x11111111;		
      
 //     /* DYNAMIC DATA */
 //     C2TDA1 = txCAN2_data1;            // load incrementing numbers;
 //     C2TDB1 = txCAN2_data2;		
      
      C1CMR = 1<<5;               // select buffer 1 for transmit
      
      /* CHOOSE SELF-TEST OR TRANSMISSION */
      //C2CMR_bit.SRR = 1;              // Self Reception Request
      C1CMR = (1<<0);                 // transmission request
    }
}


void send_CAN2 (void){
  
    if (C2SR & (1<<2)) {   // buffer 1 is available
      C2TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID1 = 16;                     // ID = 16
     
      /* STATIC DATA */
      C2TDA1 = 0x22222222;            // load 0xAAAAAAAAAAAAAAAA;
      C2TDB1 = 0x22222222;		
      
 //     /* DYNAMIC DATA */
 //     C2TDA1 = txCAN2_data1;            // load incrementing numbers;
 //     C2TDB1 = txCAN2_data2;		
      
      C2CMR = 1<<5;               // select buffer 1 for transmit
      
      /* CHOOSE SELF-TEST OR TRANSMISSION */
      //C2CMR_bit.SRR = 1;              // Self Reception Request
      C2CMR = (1<<0);                 // transmission request
    }
}


void send_CAN3 (void){
  
    if (C3SR & (1<<2)) {   // buffer 1 is available
      C3TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID1 = 17;                     // ID = 17
     
      /* STATIC DATA */
      C3TDA1 = 0x33333333;            // load 0xAAAAAAAAAAAAAAAA;
      C3TDB1 = 0x33333333;		
      
 //     /* DYNAMIC DATA */
 //     C2TDA1 = txCAN2_data1;            // load incrementing numbers;
 //     C2TDB1 = txCAN2_data2;		
      
      C3CMR = 1<<5;               // select buffer 1 for transmit
      
      /* CHOOSE SELF-TEST OR TRANSMISSION */
      //C2CMR_bit.SRR = 1;              // Self Reception Request
      C3CMR = (1<<0);                 // transmission request
    }
}


void send_CAN4 (void){
  
    if (C4SR & (1<<2)) {   // buffer 1 is available
      C4TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C4TID1 = 20;                     // ID = 20
     
      /* STATIC DATA */
      C4TDA1 = 0x44444444;            // load 0xAAAAAAAAAAAAAAAA;
      C4TDB1 = 0x44444444;		
      
 //     /* DYNAMIC DATA */
 //     C2TDA1 = txCAN2_data1;            // load incrementing numbers;
 //     C2TDB1 = txCAN2_data2;		
      
      C4CMR = 1<<5;               // select buffer 1 for transmit
      
      /* CHOOSE SELF-TEST OR TRANSMISSION */
      //C2CMR_bit.SRR = 1;              // Self Reception Request
      C4CMR = (1<<0);                 // transmission request
    }
}

 
void init_example_frame(void)	//**** TEST CODE ****
 {
   Example_Frame.rtr = 0;
   Example_Frame.dlc = 8;
   Example_Frame.addr = 2;
   Example_Frame.payload.s.s1 = 0x4142;
   Example_Frame.payload.s.s2 = 0x4344;
   Example_Frame.payload.s.s3 = 0x4546;
   Example_Frame.payload.s.s4 = 0x4748;
 }



void FIQ_Handler(void) __irq 
{
  csr_ssp_isr();
}

void init(void)
{
  setup_hardware(); 
  setup_software();
}

void gpio_to_leds(void){
 	if (FIO0PIN & 1<<15)	   //LPC3250 GPIO_0
	{
	  FIO0CLR = 1<<30; 	 	// turn on green CAN1 LED
//	  FIO0CLR = 1<<2; 	 	// turn on green CAN2 LED
//	  FIO0CLR = 1<<10; 	 	// turn on green CAN4 LED
//	  FIO1CLR = 1<<19; 	 	// turn on green CAN3 LED
//	  FIO1CLR = 1<<24; 	 	// turn on red mcu led

	}
	else
	{
//	  FIO0SET = 1<<2; 	 	// turn off green CAN2 LED
//	  FIO1SET = 1<<19; 	 	// turn off green CAN3 LED
//	  FIO0SET = 1<<10; 	 	// turn off green CAN4 LED
	  FIO0SET = 1<<30; 	 	// turn off green CAN1 LED
//	  FIO1SET = 1<<24; 	 	// turn off red mcu	led

//	  if (SSPDR == 0x5555){FIO1CLR	= 1<<23;} // ***test code

//	  Prev = 1;

	}

	if (FIO0PIN & 1<<16)	   //LPC3250 GPIO_1
	{
 //	  FIO0CLR = 1<<3; 	 	// turn on CAN2 LED
	  FIO1CLR = 1<<16; 	 	// turn on red CAN1 LED
//	  FIO0CLR = 1<<11; 	 	// turn on red CAN4 LED
//	  FIO1CLR = 1<<18; 	 	// turn on red CAN3 LED
	}
	else
	{
	  FIO1SET = 1<<16; 	 	// turn off red CAN1 LED
//	  FIO0SET = 1<<3; 	 	// turn off CAN2 LED
//	  FIO0SET = 1<<11; 	 	// turn off red CAN4 LED
//	  FIO1SET = 1<<18; 	 	// turn off red CAN3 LED
	}
}

int main (void)
{
  CAN_FRAME frame;

 init();
 
 init_example_frame();    //**** TEST CODE ****

  while(1)
  {
 	  schedule_run();
    
//    if (!csr_pop_ssp_frame(&frame))
 //   {
 //     frame.addr += 6;
 //     csr_push_ssp_frame(&frame);
//    }
    
  //  if (SSPSR & (1<<2)) //SSP receiver not empty
  //  {
  //    U0THR = SSPDR & 0xFF; //**** TEST CODE ****
  //    U0THR = (SSPDR >> 8) &0xFF; //**** TEST CODE ****
  //  }
 //   csr_ssp_send_test_packet();
//	  ssp_test_junk_receiver();
    gpio_to_leds();
 
//	  csr_pop_ssp_frame(&test_can_frame);
//	  can_ring_push(csr_ssp_tx_ring_ptr, &Example_Frame);

	  csr_route();

  //  send_CAN4();		 
  };

}

