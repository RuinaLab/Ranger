//Main for Phytec LPC3250 ARM9 main brain board (mb)

 #include <mb_includes.h>
 
// *************************************************************************
// * IRQ HANDLERS **********************************************************
// *************************************************************************
__irq void IRQ_Handler (void)
{
  int mic_sr = MIC_SR;
  
  if(mic_sr & (1<<16))  //Timer0 IRQ
  {
    //Handle standared Timer/Counter 0 Interrupt
 	   a9_timer0_isr();
  }
  
  if(mic_sr & (1<<0) && (SIC1_ER & (1<<24))) //Software IRQ
  {
    //Handle software interrupts
 	   a9_software_int_isr();
  }
  
  if(mic_sr & (1<<28))      //GPDMA IRQ
  {
    //Handle GPDMA interrupts
 	   a9_gpdma_isr();
  }
}
//////////////////////////////////////////////////////////////////////////////////
__irq void FIQ_Handler(void) 
{
  a9_dn_ssel_dma_isr();   //GPIO_4 interrupt; sets up SSP DMA transfers  
}



/******************************************************************************
* Main Program                                                                *
*******************************************************************************/
int main(void)
{ 
  #ifdef DEBUG
  while (P3_INP_STATE & 1<<1)
  {
    P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED 
  }
  #endif
 
 mb_setup_hardware();
 mb_setup_software();

  while(1)
  {   
    mb_schedule_run();
  }
}
