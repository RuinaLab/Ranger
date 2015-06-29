#ifndef  __MB_SSP_DMA_H__
#define  __MB_SSP_DMA_H__


typedef enum can_channels{
  //Value is the index of the CAN channel used in the can_rx and can_tx code
  CHAN_SSP	   =  0,
  CHAN_CAN1    =  1,
  CHAN_CAN2    =  2,
  CHAN_CAN3    =  3,
  CHAN_CAN4    =  4,
} CAN_CHANNEL;

typedef	__packed union can_payload{
  __packed struct {
  	double d1;
  } d;
  __packed struct {
  	int w1;
  	int w2;
  } w;
  __packed struct {
    int i1;
    int i2;
  } i;
  __packed struct {
  	float f1;
  	float f2;
  } f;
  __packed struct{
  	short s1;
  	short s2;
  	short s3;
  	short s4;
  } s;
  __packed struct{
  	unsigned char b1;
  	unsigned char b2;
  	unsigned char b3;
  	unsigned char b4;
  	unsigned char b5;
  	unsigned char b6;
  	unsigned char b7;
  	unsigned char b8;
  } b;
} CAN_PAYLOAD;

typedef struct can_frame{
  CAN_CHANNEL  chan;
  int          addr : 11;
  int          dlc  :  4;
  int          rtr  :  1;
  CAN_PAYLOAD  payload;
} CAN_FRAME;

void mb_csr_dma_var_pointer_init(void);
void mb_ssp1_isr(void);
void mb_gpdma_isr(void);
void mb_csr_gpdma_rx0_isr(void);
void mb_csr_gpdma_tx1_isr(void);
void mb_ssp_pop_frames(void);
unsigned short int mb_ssp_push_frame(CAN_FRAME * frame);

#endif// __MB_SSP_DMA_H__
