/*

	can.h
	
	Copyright Cornell University, 2009
	
	Nicolas Williamson - Sept. 2009
	Using code by Tommy Craig, summer 09
	
*/


/*#define CAN_BUF_SIZE 32

//CAN_ID Table
typedef enum can_ids{
	CAN_TIME_SYNC = 0
} CAN_ID;

typedef enum can_chs{
  CAN_SSP = 0,
  CAN_CH_1,
  CAN_CH_2,
  CAN_CH_3,
  CAN_CH_4
} CAN_CH;

typedef struct can_frame{
  CAN_CH      chan;
  int         addr : 11;
  int         dlc  :  4;
  char        rtr  :  1;
  float       data;
  long int    timestamp;
} CAN_FRAME;

typedef struct can_buffer{
	CAN_FRAME frames[CAN_BUF_SIZE];
	unsigned char first;
	unsigned char next;
	unsigned char overflows;
	unsigned char empty;
} CAN_BUFFER;	

// *********** CAN REGISTERS ***********
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
*/







