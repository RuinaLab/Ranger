#include <includes.h>

#define INT_RLS   3
#define INT_RDA   2
#define INT_CTI   5
#define INT_THRE  1

static char uarti_str[100];
static int uarti_strlen = 0;

static char * uarti_out_buf;
static volatile int uarti_out_next_idx = 0;
static volatile int uarti_out_len = 0;
static volatile int uarti_out_stall = 1;
static UARTI_CALLBACK_PTR uarti_out_callback;

void uarti_tx_set_empty_callback(UARTI_CALLBACK_PTR callback){
    uarti_out_callback = callback;
}

int uarti_tx_buf(char * buf, int buflen){
  if(uarti_out_next_idx < uarti_out_len){
    //Then the prior string has not been fully transmitted.
    //The current design would have to be reworked a bit to allow aborting the previous transfer
    //and beginning a new one.
    return 1;
  } else {
    uarti_out_buf = buf;
    uarti_out_len = buflen;
    uarti_out_next_idx = 0;
    if(uarti_out_stall) {
      uarti_tx_refill();
    }
    return 0;
  }
}

void uarti_tx_refill(void){
  if(uarti_out_next_idx >= uarti_out_len) {
    //String finished. Do callback for (optional) refill
    if(uarti_out_callback != 0){
      uarti_out_callback();
    }
  }
  if(uarti_out_next_idx < uarti_out_len) {
    U0THR = uarti_out_buf[uarti_out_next_idx];
    uarti_out_next_idx += 1;
    uarti_out_stall = 0;
  } else {
    uarti_out_stall = 1;
  }
    
}

__irq void uarti_isr(void){
  int iir;
  volatile int lsr; //remove volatile later, just to quash warning temporarily
  
  iir = U0IIR;
  lsr = U0LSR;
  
  if(!(iir&1)){//Interrupt pending bit
    switch((iir>>1)&0x7){ //U0IIR 3:1, Interrupt Identification
      case INT_RLS: //Receive Line Status
        //Respond to rx interrupt as given by the Line Status Register
        break;
      case INT_RDA: //Receive Data Available
        break;
      case INT_CTI: //Character Time-Out Indicator
        break;
      case INT_THRE://THRE Interrupt
        uarti_tx_refill();
        break;
      default:
        //should never reach here
        break;
    }
  } else {
    //No interrupt is pending
  }
  //TODO: clear tx interrupt flag(s)
  VICVectAddr = 0;
}

void uarti_print_int2(int i1, int i2){
  uarti_strlen = sprintf(uarti_str,"%i\t%i\n\r",i1, i2);
  uarti_tx_buf(uarti_str, uarti_strlen);
}

void uarti_print_int(int i){
  uarti_strlen = sprintf(uarti_str,"%i\n\r",i);
  uarti_tx_buf(uarti_str, uarti_strlen);
}

void uarti_print_float(float f){
  uarti_strlen = sprintf(uarti_str,"%2.15f\n\r",f);
  uarti_tx_buf(uarti_str, uarti_strlen);
}

void uarti_print_2float(float f1, float f2){
  uarti_strlen = sprintf(uarti_str,"%f\t%f\n\r",f1,f2);
  uarti_tx_buf(uarti_str, uarti_strlen);
}

