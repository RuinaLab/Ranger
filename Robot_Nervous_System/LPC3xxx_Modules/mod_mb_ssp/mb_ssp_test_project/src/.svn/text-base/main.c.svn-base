//Main for Phytec LPC3250 ARM9 main brain board (mb)

#include <mb_includes.h>
 
// *************************************************************************
// * IRQ HANDLERS **********************************************************
// *************************************************************************
__irq void IRQ_Handler (void)
{

  int mic_sr = MIC_SR;
  
  if(mic_sr & (1<<16))
  {
    //Handle standared Timer/Counter 0 Interrupt
 	   mb_timer0_isr();
  }
}

__irq void FIQ_Handler(void) 
{
 //  T1TCR = (T1TCR | (1<<0)) & 3;		//TEST CODE - start timer 1
   if(MIC_SR & (1<<21))
  {
    //Handle SSP1 Interrupt
    mb_ssp1_isr();
  }
 //   T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
}

void mb_init(void)
{
  mb_setup_hardware(); 
  mb_setup_software();
}


/******************************************************************************
* Function: Wait for a number of cycles                                       *
*******************************************************************************/
 
void wait (unsigned int delay) {
  while (delay--);
}

/******************************************************************************
* Main Program                                                                *
*******************************************************************************/
int main(void)
{
  CAN_FRAME frame;  //**** TEST CODE ****

 mb_init();
 init_example_frame();    //**** TEST CODE ****

  while(1)
  {
 	  mb_schedule_run();
//	  T1TCR = (T1TCR | (1<<0)) & 3;		//TEST CODE - start timer 1
    mb_ssp_test_frame_receiver();
//    mb_ssp_pop_frame(&frame);
//	  T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
//	  if (T1TC > 5400) {P3_OUTP_SET = 1<<26;}
  }
}
