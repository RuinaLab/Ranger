#include <mb_includes.h>

CAN_FRAME mb_ssp_dma_test_example_frame;
volatile long unsigned int mb_ssp_dma_test_rx_counter = 0;
volatile long unsigned int mb_ssp_dma_test_tx_counter = 0;

/*
  void mb_ssp_dma_test_frame_receiver(void)	// **** TEST CODE ****
 {
  short unsigned int data[30], i = 0, transfer = 0;
  
 for (i=0; i<30; ++i) {data[i] = 0;}
  
  i = 0;  
  while (!(P3_INP_STATE & (1<<14)))
  {
    if (SSP1SR & (1<<2))
    {
      data[i] = SSP1DR;
      ++i;
      P3_OUTP_SET = 1<<25;  // **** TEST CODE **** turn on green LED
      transfer = 1;
    }
  }

  if (transfer)
  {
    while (SSP1SR & (1<<2))
    {
      data[i] = SSP1DR;
      ++i;
      P3_OUTP_SET = 1<<25;  // **** TEST CODE **** turn on green LED
      transfer = 1;
    }
     i = 0; 
  }
 }
*/

