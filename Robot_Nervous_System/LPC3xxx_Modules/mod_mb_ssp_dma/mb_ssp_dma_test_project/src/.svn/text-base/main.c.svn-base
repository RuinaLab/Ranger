//Main for Phytec LPC3250 ARM9 main brain board (mb)

 #include <mb_includes.h>

// *************************************************************************
// * IRQ HANDLERS **********************************************************
// *************************************************************************
__irq void IRQ_Handler (void)
{

  int mic_sr = MIC_SR;
  
  //T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
  
  if(mic_sr & (1<<16))
  {
    //Handle standared Timer/Counter 0 Interrupt
 	   mb_timer0_isr();
  }
  
 //   T1TCR = (T1TCR | (1<<0)) & 3;		//TEST CODE - start timer 1
  
}


__irq void FIQ_Handler(void) 
{
 // if(MIC_SR & (1<<21))
 // {
 //   //Handle SSP1 Interrupt
 //   mb_ssp1_isr();
 // }
 

T1TCR = (T1TCR | (1<<0)) & 3;		//TEST CODE - start timer 1

  if(MIC_SR & (1<<28))
  {
    //Handle GPDMA Interrupt
    mb_gpdma_isr();
  }
  
T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
  
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

//long unsigned int temp,i;
 mb_init();

 mb_ssp_dma_init_example_frame(); //**** TEST CODE ****
 
 

  while(1)
  {
    mb_schedule_run();
    
//	  test_frame_sender();	//**** TEST CODE **** sends out example packets, one per iteration
//	  test_frame_receiver();
//	  T1TCR = (T1TCR | (1<<0)) & 3;		//TEST CODE - start timer 1
//    mb_ssp_pop_frame(&frame);
//	  T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
//	  if (T1TC > 13000) {P3_OUTP_SET = 1<<26;} 
  }
}
