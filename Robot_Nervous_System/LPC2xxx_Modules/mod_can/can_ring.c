#include <includes.h>

/*
#define CAN_TX_BUFFER_LENGTH 128
CAN_FRAME can_tx_buffer[CAN_TX_BUFFER_LENGTH];

//Make these globals volatile to allow mixed context usage.
volatile int can_tx_in_idx = CAN_TX_BUFFER_LENGTH-1;
volatile int can_tx_out_idx = CAN_TX_BUFFER_LENGTH-1;
*/

void can_ring_init(CAN_RING * ring, CAN_FRAME * frame_buf, int buf_len){
  ring->buf = frame_buf;
  ring->buf_len = buf_len;
  /*
  ring->in_idx = buf_len - 1;
  ring->out_idx = buf_len - 1;
  */
  ring->in_idx = 0;
  ring->out_idx = 0;
}

int can_ring_push(CAN_RING * ring, CAN_FRAME * frame) {
 // int  ret;
  int next_in_idx;
  /*
  int next_in_idx = ring->in_idx + 1;

  if(next_in_idx >= ring->buf_len){
    next_in_idx -= ring->buf_len;
  }  
  if(next_in_idx == ring->out_idx) {
    ret = 1;
  } else {
    ring->buf[next_in_idx] = *frame;
    ret = 0;
    ring->in_idx = next_in_idx;
  }
  */

  next_in_idx = ring->in_idx;
  if (++next_in_idx == ring->buf_len) {next_in_idx = 0;}
  if (next_in_idx == ring->out_idx) {return 1;} //buffer full

  ring->buf[next_in_idx] = *frame;
  ring->in_idx = next_in_idx;

  return 0;
}

int can_ring_pop(CAN_RING * ring, CAN_FRAME * frame){
 
  if (ring->out_idx != ring->in_idx)
  {
    *frame = ring->buf[ring->out_idx];
    if (ring->out_idx == (ring->buf_len) - 1)
    {
      ring->out_idx = 0;
    }
    else
    {
      ++ring->out_idx;
    }
    return 0;   //frame popped successfully
  }
  else
  {
    return 1;   //buffer empty
  }
 
 /* int out_idx = ring->out_idx;
  int  ret;
  int next_out_idx = out_idx + 1;

  if(next_out_idx >= ring->buf_len){
    next_out_idx -= ring->buf_len;
  }
  if(out_idx == ring->in_idx){
    ret = 1;
  } else {
    *frame = ring->buf[next_out_idx];
    ret = 0;
    ring->out_idx = next_out_idx;
  }
  return ret;

  */
}


