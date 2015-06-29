#ifndef  __MB_SSP_H__
#define  __MB_SSP_H__

typedef struct ssp_ring{
  short unsigned int  idx1 : 5;
  short unsigned int  idx2 : 5;
} SSP_RING;


void mb_ssp1_isr(void);
unsigned short int mb_ssp_pop_frame(CAN_FRAME * frameptr);

#endif// __MB_SSP_H__

