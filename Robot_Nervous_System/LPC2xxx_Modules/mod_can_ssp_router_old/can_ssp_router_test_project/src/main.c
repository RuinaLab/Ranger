#include <includes.h>

CAN_FRAME test_can_frame;
extern CAN_RING * csr_ssp_tx_ring_ptr;

CAN_FRAME Example_Frame;

void init_example_frame(void)	//**** TEST CODE ****
 {
   Example_Frame.rtr = 0;
   Example_Frame.dlc = 8;
   Example_Frame.addr = 99;
   Example_Frame.payload.w.w1 = 0x41424344;
   Example_Frame.payload.w.w2 = 0x45464748;
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
 init();
 
 init_example_frame();    //**** TEST CODE ****

  while(1)
  {
 	  schedule_run();
    
  //  if (SSPSR & (1<<2)) //SSP receiver not empty
  //  {
  //    U0THR = SSPDR & 0xFF; //**** TEST CODE ****
  //    U0THR = (SSPDR >> 8) &0xFF; //**** TEST CODE ****
  //  }
    
	  gpio_to_leds();
 
	  csr_pop_ssp_frame(&test_can_frame);
//	  can_ring_push(csr_ssp_tx_ring_ptr, &Example_Frame);

//	  csr_route();		 
  };

}

