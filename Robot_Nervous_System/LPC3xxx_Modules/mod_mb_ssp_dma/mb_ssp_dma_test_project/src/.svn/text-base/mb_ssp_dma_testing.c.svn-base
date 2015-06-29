#include <mb_includes.h>

CAN_FRAME mb_ssp_dma_test_example_frame;
volatile long unsigned int mb_ssp_dma_test_rx_counter = 0;
volatile long unsigned int mb_ssp_dma_test_tx_counter = 0;

void mb_ssp_dma_init_example_frame(void)	//**** TEST CODE ****
 {
   mb_ssp_dma_test_example_frame.rtr = 0;
   mb_ssp_dma_test_example_frame.dlc = 8;
   mb_ssp_dma_test_example_frame.addr = 99;
   mb_ssp_dma_test_example_frame.payload.s.s1 = 0x4142;
   mb_ssp_dma_test_example_frame.payload.s.s2 = 0x4344;
   mb_ssp_dma_test_example_frame.payload.s.s3 = 0x4546;
   mb_ssp_dma_test_example_frame.payload.s.s4 = 0x4748;
 }

void mb_ssp_dma_test_frame_sender(void)	//**** TEST CODE ****
 {
//   if (!mb_csr_push_packet(&mb_ssp_dma_test_example_frame))
 //  {++mb_ssp_dma_test_tx_counter;}
 }

  void mb_ssp_dma_test_frame_receiver(void)	//**** TEST CODE ****
 {
 /* CAN_FRAME frame;

  while (!mb_ssp_pop_frame(&frame))
 
  {
    ++mb_ssp_dma_test_rx_counter;
	
  	P3_OUTP_SET = 1<<25;  // **** TEST CODE **** turn on green LED
	  if (frame.addr != mb_ssp_dma_test_example_frame.addr) {P3_OUTP_CLR = 1<<25;}
	  if (frame.dlc != mb_ssp_dma_test_example_frame.dlc) {P3_OUTP_CLR = 1<<25;}
	  if (frame.rtr != mb_ssp_dma_test_example_frame.rtr) {P3_OUTP_CLR = 1<<25;}
	  if (frame.payload.s.s1 != mb_ssp_dma_test_example_frame.payload.s.s1) {P3_OUTP_CLR = 1<<25;}
	  if (frame.payload.s.s2 != mb_ssp_dma_test_example_frame.payload.s.s2) {P3_OUTP_CLR = 1<<25;}
    if (frame.payload.s.s3 != mb_ssp_dma_test_example_frame.payload.s.s3) {P3_OUTP_CLR = 1<<25;}
	  if (frame.payload.s.s4 != mb_ssp_dma_test_example_frame.payload.s.s4) {P3_OUTP_CLR = 1<<25;}
  }
*/
 }
