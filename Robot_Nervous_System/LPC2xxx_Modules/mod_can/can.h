#ifndef CAN_H
#define CAN_H

//Various type getter and setter methods
typedef void(*CAN_RX_SETTER_DOUBLE)(double);
typedef void(*CAN_RX_SETTER_FLOAT)(float);
typedef void(*CAN_RX_SETTER_INT)(int);
typedef void(*CAN_RX_SETTER_SHORT)(short);

typedef double(*CAN_TX_GETTER_DOUBLE)(void);
typedef float(*CAN_TX_GETTER_FLOAT)(void);
typedef int(*CAN_TX_GETTER_INT)(void);
typedef short(*CAN_TX_GETTER_SHORT)(void);

//void-void function typedef
typedef void(*CAN_VV_PTR)(void);

typedef enum can_dispatch_modes{
  CAN_DISPATCH_AUTO,
  CAN_DISPATCH_MANUAL
} CAN_DISPATCH_MODE;


//Value is the index of the CAN channel used in the can_rx and can_tx code
typedef enum can_channels {
  CHAN_SSP  = 0,
  CHAN_CAN1 = 1,
  CHAN_CAN2 = 2,
  CHAN_CAN3 = 3,
  CHAN_CAN4 = 4,
  CHAN_ROUTER = 5
} CAN_CHANNEL;

//Enumerates the possible CAN Packet layouts
typedef enum can_layout{
  CAN_LAYOUT_D,
  CAN_LAYOUT_FF,
  CAN_LAYOUT_II,
  CAN_LAYOUT_FI,
  CAN_LAYOUT_ISS  
}CAN_LAYOUT;

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

//Generic packet descriptor
typedef struct can_frame_descriptor{
  int          addr : 11;
  CAN_CHANNEL  chan;
  char         rtr  :  1;
  CAN_LAYOUT   frame_layout;
  CAN_VV_PTR   ptr1;
  CAN_VV_PTR   ptr2;
  CAN_VV_PTR   ptr3;
  CAN_VV_PTR   ptr4;
  CAN_VV_PTR   ptr5;
  CAN_VV_PTR   ptr6;
  CAN_VV_PTR   ptr7;
  CAN_VV_PTR   ptr8;
} CAN_FRAME_DESC;

typedef struct can_raw_frame{
  unsigned long canrfs;
  unsigned long canrid;
  unsigned long canrda;
  unsigned long canrdb;
} CAN_RAW_FRAME;

typedef struct can_rx_data{

  unsigned long write_index;
  unsigned long read_index;
  unsigned long buffer_mask;
  unsigned long buffer_addr;
  unsigned long rfs_base_addr;
} CAN_RX_DATA;

typedef struct can_frame{
  CAN_CHANNEL  chan;
  int          addr : 11;
  int          dlc  :  4;
  char         rtr  :  1;
  CAN_PAYLOAD  payload;
} CAN_FRAME;

typedef struct can_ring{
  CAN_FRAME  *  buf;
  int           buf_len;
  volatile int  in_idx;
  volatile int  out_idx;
} CAN_RING;


typedef struct can_rx_chan_cfg{
  volatile unsigned long   * base_addr;
  CAN_RING                 * ring;
  CAN_FRAME_DESC          ** descriptors;
  CAN_DISPATCH_MODE          dispatch_mode;
} CAN_RX_CHAN_CFG;

typedef struct can_tx_chan_cfg{
  volatile unsigned long   * base_addr;
  CAN_RING                 * ring;
  int                        stalled;
} CAN_TX_CHAN_CFG;


//TX DESCRIPTOR SETTING FUNCTIONS
void can_set_tx_descriptor_d(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
     CAN_TX_GETTER_DOUBLE g_d1
     );
void can_set_tx_descriptor_ff(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
     CAN_TX_GETTER_FLOAT g_f1,CAN_TX_GETTER_FLOAT g_f2
     );
void can_set_tx_descriptor_ii(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
     CAN_TX_GETTER_INT g_i1,CAN_TX_GETTER_INT g_i2
     );
void can_set_tx_descriptor_fi(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_TX_GETTER_FLOAT g_f1,CAN_TX_GETTER_INT g_i1
  );
void can_set_tx_descriptor_iss(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
     CAN_TX_GETTER_INT g_i1,CAN_TX_GETTER_SHORT g_s1, CAN_TX_GETTER_SHORT g_s2
     );
     
void can_set_tx_descriptor_rtr(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan);

//RX DESCRIPTOR SETTING FUNCTIONS
void can_set_rx_descriptor_d(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
     CAN_RX_SETTER_DOUBLE s_d1
     );
void can_set_rx_descriptor_ff(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
     CAN_RX_SETTER_FLOAT s_f1,CAN_RX_SETTER_FLOAT s_f2
     );
void can_set_rx_descriptor_ii(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
     CAN_RX_SETTER_INT s_i1,CAN_RX_SETTER_INT s_i2
     );
     
void can_set_rx_descriptor_fi(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
  CAN_RX_SETTER_FLOAT s_f1,CAN_RX_SETTER_INT s_i1
  );
void can_set_rx_descriptor_iss(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
     CAN_RX_SETTER_INT s_i1,CAN_RX_SETTER_SHORT s_s1, CAN_RX_SETTER_SHORT s_s2
     );	 


//public function declarations
void can_init(void);
void can_rx_dispatch_all(void);
unsigned short can_rx_pop_frame(CAN_FRAME * frameptr);

void can_rx1_isr(void) __irq;
void can_rx2_isr(void) __irq;
void can_rx3_isr(void) __irq; 
void can_rx4_isr(void) __irq;
void can_rx1_fiq(void);
void can_rx2_fiq(void);
void can_rx3_fiq(void);
void can_rx4_fiq(void);
void can_tx1(void);
void can_tx2(void);
void can_tx3(void);
void can_tx4(void);
void can_error_isr(void) __irq;
__asm void can_rx_push(CAN_CHANNEL chan);


//Return a CAN_FRAME with the number of CAN receive or transmit
//on that channel since the previous call.
unsigned short can_rx_get_frame_count_1(CAN_FRAME * frame);
unsigned short can_rx_get_frame_count_2(CAN_FRAME * frame);
unsigned short can_rx_get_frame_count_3(CAN_FRAME * frame);
unsigned short can_rx_get_frame_count_4(CAN_FRAME * frame);

unsigned short can_tx_get_frame_count_1(CAN_FRAME * frame);
unsigned short can_tx_get_frame_count_2(CAN_FRAME * frame);
unsigned short can_tx_get_frame_count_3(CAN_FRAME * frame);
unsigned short can_tx_get_frame_count_4(CAN_FRAME * frame);

//Return a CAN_FRAME with the error number on that channel
unsigned short can_get_error_1(CAN_FRAME * frame);
unsigned short can_get_error_2(CAN_FRAME * frame);
unsigned short can_get_error_3(CAN_FRAME * frame);
unsigned short can_get_error_4(CAN_FRAME * frame);

void can_init_status_callback(
  void (* rx_frame_count)(unsigned short),
  void (* tx_frame_count)(unsigned short),
  void (* error_led)(unsigned short));

void can_rx_set_descriptors(CAN_FRAME_DESC ** rx_descriptors,CAN_FRAME_DESC ** rtr_descriptors);
void   can_rx_set_chan_cfg(CAN_CHANNEL chan,volatile unsigned long * base_addr, CAN_RING * rx_ring, CAN_DISPATCH_MODE mode);
void   can_tx_set_chan_cfg(CAN_CHANNEL chan,volatile unsigned long * base_addr, CAN_RING * tx_ring);

int   can_transmit(CAN_FRAME_DESC * fd);
int   can_transmit_alt(CAN_FRAME_DESC * fd,CAN_CHANNEL chan, char rtr);
int   can_transmit_frame(CAN_FRAME * frame);
int   can_tx_now(CAN_CHANNEL chan);

double can_tx_getter_double_dummy(void);
float  can_tx_getter_float_dummy(void);
int    can_tx_getter_int_dummy(void);
short  can_tx_getter_short_dummy(void);

void can_rx_setter_double_dummy(double d);
void can_rx_setter_float_dummy(float f);
void can_rx_setter_int_dummy(int i);
void can_rx_setter_short_dummy(short s);

//........Ring Buffer
void can_ring_init(CAN_RING * ring, CAN_FRAME * frame_buf, int buf_len);
int  can_ring_push(CAN_RING * ring, CAN_FRAME * frame);
int  can_ring_pop(CAN_RING * ring, CAN_FRAME * frame);

//private function declarations
//........Rx
void can_rx_now(CAN_CHANNEL chan);
void can_rx_dispatch_frame(CAN_FRAME * frame);
void can_rx_dispatch_chan(CAN_CHANNEL chan);
void can_rx_dispatch_all(void);
void can_config_acceptance(void);

//........Tx
void can_tx_send_next_frame(CAN_CHANNEL chan);

//........isr
void can_voidint(unsigned short chan);

unsigned short int can_tx1_push_frame(CAN_FRAME * frameptr);
unsigned short int can_tx2_push_frame(CAN_FRAME * frameptr);
unsigned short int can_tx3_push_frame(CAN_FRAME * frameptr);
unsigned short int can_tx4_push_frame(CAN_FRAME * frameptr);


#define CAN_MOD  (0x00)
#define CAN_CMR  (0x04)
#define CAN_GSR  (0x08)
#define CAN_ICR  (0x0C)
#define CAN_IER  (0x10)
#define CAN_BTR  (0x14)
#define CAN_EWL  (0x18)
#define CAN_SR   (0x1C)
#define CAN_RFS  (0x20)
#define CAN_RID  (0x24)
#define CAN_RDA  (0x28)
#define CAN_RDB  (0x2C)
#define CAN_TFI1 (0x30)
#define CAN_TID1 (0x34)
#define CAN_TDA1 (0x38)
#define CAN_TDB1 (0x3C)
#define CAN_TFI2 (0x40)
#define CAN_TID2 (0x44)
#define CAN_TDA2 (0x48)
#define CAN_TDB2 (0x4C)
#define CAN_TFI3 (0x50)
#define CAN_TID3 (0x54)
#define CAN_TDA3 (0x58)
#define CAN_TDB3 (0x5C)

#define CAN_REG(base,offset) \
        (*((volatile unsigned long *) (((volatile unsigned char *)base) + offset)))

#endif /* CAN_H */
