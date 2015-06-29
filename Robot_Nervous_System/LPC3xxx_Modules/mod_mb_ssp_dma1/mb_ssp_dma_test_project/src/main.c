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
  a9_dn_ssel_dma_isr();   //GPIO_4 rising edge interrupt; sets up SSP DMA transfers  
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

  while(1)
  {
    mb_schedule_run();
    
//    a9_dn_ssp_parse();
 //  a9_dn_ssp_send_data();
    
//    if (!(P3_INP_STATE & (1<<14))) {P3_OUTP_SET = 1<<25;}
//    if (P3_INP_STATE & (1<<14)) {P3_OUTP_SET = 1<<26;}

        
//	  T1TCR = (T1TCR | (1<<0)) & 3;		//TEST CODE - start timer 1
//    mb_ssp_pop_frame(&frame);
//	  T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
//	  if (T1TC > 4500) {P3_OUTP_SET = 1<<26;} 
  }
}
