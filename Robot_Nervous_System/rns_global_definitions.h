#ifndef __RNS_GLOBAL_DEFINITIONS_H__
#define __RNS_GLOBAL_DEFINITIONS_H__

//Value is the index of the CAN channel used in the can_rx and can_tx code
typedef enum can_channels {
  CHAN_SSP  = 0,
  CHAN_CAN1 = 1,
  CHAN_CAN2 = 2,
  CHAN_CAN3 = 3,
  CHAN_CAN4 = 4
} CAN_CHANNEL;

//Subscriber IDs have an allowable range of 0 to 31 (since they're based on bits in a long int),
//and allow data sending processes to check whether the data is new, or if they've already sent it.
//The intent is to save bandwidth, particularly on the wireless connection to the PC/laptop data display.
//A process using this feature needs a unique subscriber_id, to keep track of whether data is new to it or not. 
typedef enum subscriber_id{
  BLUETOOTH,
  SSP,
  LABVIEW,
  LCD,
  MB_ERR_DIST,
  ESTIMATOR,
  FSM
} SUBSCRIBER_ID;

typedef	__packed union data_payload{
  __packed struct {
    double d1;
  } d;
    __packed struct {
    long long ll;
  } ll;
  __packed struct {
    signed long sl;
    unsigned long ul;
  } slul;
  __packed struct {
    unsigned long ul1;
    unsigned long ul2;
  } ulul; //unsigned long - unsigned long
   __packed struct {
    float f;            
    unsigned long ul;
  } ful;		//float - unsigned long
  __packed struct {
    float f1;
    float f2;
  } ff;
   __packed struct{
    unsigned short us1;
    unsigned short us2;
    unsigned short us3;
    unsigned short us4;
  } us;
  __packed struct{
    signed short ss1;
    signed short ss2;
    signed short ss3;
    signed short ss4;
  } ss;
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
} DATA_PAYLOAD;

typedef struct data_frame{
  CAN_CHANNEL     chan;
  unsigned short  id : 11;
  unsigned char   dlc  :  4;
  unsigned char   rtr  :  1;
  unsigned long   data_read;
  DATA_PAYLOAD    payload;
} DATA_FRAME;

#endif //__RNS_GLOBAL_DEFINITIONS_H__
