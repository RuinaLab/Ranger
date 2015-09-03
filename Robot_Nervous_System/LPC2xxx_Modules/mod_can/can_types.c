#include <includes.h>

void can_rx_setter_double_dummy(double d){ }
void can_rx_setter_float_dummy(float f){ }
void can_rx_setter_int_dummy(int i){ }
void can_rx_setter_short_dummy(short s){}


double can_tx_getter_double_dummy(void) { return 0; }
float can_tx_getter_float_dummy(void) { return 0; }
int can_tx_getter_int_dummy(void) { return 0; }
short can_tx_getter_short_dummy(void) {return 0;}

//TX DESCRIPTOR SETTING FUNCTIONS
void can_set_tx_descriptor_d(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_TX_GETTER_DOUBLE g_d1
  ){
  frame_desc->addr = addr;
  frame_desc->chan = chan;
  frame_desc->rtr  = 0;
  frame_desc->frame_layout = CAN_LAYOUT_D;
  frame_desc->ptr1 = (CAN_VV_PTR)g_d1;
}

void can_set_tx_descriptor_ff(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_TX_GETTER_FLOAT g_f1,CAN_TX_GETTER_FLOAT g_f2
  ){
  frame_desc->addr = addr;
  frame_desc->chan = chan;
  frame_desc->rtr  = 0;
  frame_desc->frame_layout = CAN_LAYOUT_FF;
  frame_desc->ptr1 = (CAN_VV_PTR)g_f1;
  frame_desc->ptr2 = (CAN_VV_PTR)g_f2;
}
void can_set_tx_descriptor_ii(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_TX_GETTER_INT g_i1,CAN_TX_GETTER_INT g_i2
  ){
  frame_desc->addr = addr;
  frame_desc->chan = chan;
  frame_desc->rtr  = 0;
  frame_desc->frame_layout = CAN_LAYOUT_II;
  frame_desc->ptr1 = (CAN_VV_PTR)g_i1;
  frame_desc->ptr2 = (CAN_VV_PTR)g_i2;
}

void can_set_tx_descriptor_fi(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_TX_GETTER_FLOAT g_f1,CAN_TX_GETTER_INT g_i1
  ){
  frame_desc->addr = addr;
  frame_desc->chan = chan;
  frame_desc->rtr  = 0;
  frame_desc->frame_layout = CAN_LAYOUT_FI;
  frame_desc->ptr1 = (CAN_VV_PTR)g_f1;
  frame_desc->ptr2 = (CAN_VV_PTR)g_i1;
}

void can_set_tx_descriptor_iss(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_TX_GETTER_INT g_i1,CAN_TX_GETTER_SHORT g_s1, CAN_TX_GETTER_SHORT g_s2
  ){
  frame_desc->addr = addr;
  frame_desc->chan = chan;
  frame_desc->rtr  = 0;
  frame_desc->frame_layout = CAN_LAYOUT_ISS;
  frame_desc->ptr1 = (CAN_VV_PTR)g_i1;
  frame_desc->ptr2 = (CAN_VV_PTR)g_s1;
  frame_desc->ptr3 = (CAN_VV_PTR)g_s2;
}

void can_set_tx_descriptor_rtr(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan){
  frame_desc->addr = addr;
  frame_desc->chan = chan;
  frame_desc->rtr  = 1;
}

//RX DESCRIPTOR SETTING FUNCTIONS
void can_set_rx_descriptor_d(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_RX_SETTER_DOUBLE s_d1
  ){
  frame_desc->chan = chan;
  frame_desc->addr = addr;
  frame_desc->frame_layout = CAN_LAYOUT_D;
  frame_desc->ptr1 = (CAN_VV_PTR)s_d1;  
}
void can_set_rx_descriptor_ff(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_RX_SETTER_FLOAT s_f1,CAN_RX_SETTER_FLOAT s_f2
  ){
  frame_desc->chan = chan;
  frame_desc->addr = addr;
  frame_desc->frame_layout = CAN_LAYOUT_FF;
  frame_desc->ptr1 = (CAN_VV_PTR)s_f1;
  frame_desc->ptr2 = (CAN_VV_PTR)s_f2;
}

void can_set_rx_descriptor_ii(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_RX_SETTER_INT s_i1,CAN_RX_SETTER_INT s_i2
  ){
  frame_desc->chan = chan;
  frame_desc->addr = addr;
  frame_desc->frame_layout = CAN_LAYOUT_II;
  frame_desc->ptr1 = (CAN_VV_PTR)s_i1;
  frame_desc->ptr2 = (CAN_VV_PTR)s_i2;
}

void can_set_rx_descriptor_fi(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_RX_SETTER_FLOAT s_f1,CAN_RX_SETTER_INT s_i1
  ){
  frame_desc->chan = chan;
  frame_desc->addr = addr;
  frame_desc->frame_layout = CAN_LAYOUT_FI;
  frame_desc->ptr1 = (CAN_VV_PTR)s_f1;
  frame_desc->ptr2 = (CAN_VV_PTR)s_i1;
}

void can_set_rx_descriptor_iss(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_RX_SETTER_INT s_i1,CAN_RX_SETTER_SHORT s_s1, CAN_RX_SETTER_SHORT s_s2
  ){
  frame_desc->chan = chan;
  frame_desc->addr = addr;
  frame_desc->frame_layout = CAN_LAYOUT_ISS;
  frame_desc->ptr1 = (CAN_VV_PTR)s_i1;
  frame_desc->ptr2 = (CAN_VV_PTR)s_s1;
  frame_desc->ptr3 = (CAN_VV_PTR)s_s2;
} 	 


