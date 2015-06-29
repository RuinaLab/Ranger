#ifndef __MB_IO_DATA_H__
#define __MB_IO_DATA_H__

//#define MB_IO_DATA_ARRAY_SIZE  2048

/*
Stuff below moved to rns_global_definitions.h

Not sure that's the right place for them ...

TBD ...

typedef struct io_data_point{
  __packed union {
  	float f;
    long unsigned int lu;
    long signed int ls;
  } value;
  long unsigned int time;
  long unsigned int data_read;
  short unsigned int id;
}IO_DATA_POINT;
*/

/*
//Subscriber IDs have an allowable range of 0 to 31 (since they're based on bits in a long int),
//and allow data sending processes to check whether the data is new, or if they've already sent it.
//The intent is to save bandwidth, particularly on the wireless connection to the PC/laptop data display.
//A process using this feature needs a unique subscriber_id, to keep track of whether data is new to it or not. 
typedef enum subscriber_id{
  BLUETOOTH
} SUBSCRIBER_ID;
*/

void mb_io_mark_as_read(unsigned short data_id, unsigned short subscriber_id);
void mb_io_mark_as_unread(unsigned short data_id, unsigned short subscriber_id);
void mb_io_mark_as_unread_by_all(unsigned short data_id);
unsigned short mb_io_data_was_read(unsigned short data_id, unsigned short subscriber_id);
DATA_FRAME * mb_io_get_pointer(unsigned short data_id);
float mb_io_get_float(short unsigned int data_id);  // Gives float value for desired data point
unsigned long mb_io_get_ul(short unsigned int data_id);
signed long mb_io_get_sl(short unsigned int data_id);
unsigned long mb_io_get_time(short unsigned int data_id);  // Returns integer time stamp value in mS for desired data point
void mb_io_set_float(short unsigned int data_id, float value); // Use this function to set a data point value. The time stamp is added automatically.
void mb_io_set_ul(short unsigned int data_id, unsigned long value);
void mb_io_set_sl(short unsigned int data_id, signed long value);
void mb_io_set_time(unsigned short data_id, unsigned long time);
void mb_io_init(void);

#endif  //__MB_IO_DATA_H__
