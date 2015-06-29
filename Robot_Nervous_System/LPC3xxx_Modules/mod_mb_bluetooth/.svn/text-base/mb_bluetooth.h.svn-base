#ifndef  __MB_BT_H__
#define  __MB_BT_H__

void mb_bt_create_data_lists(
  unsigned short first_id,         //First LabView channel data_id to search
  unsigned short last_id,          //Last LabView channel data_id to search
  unsigned short medium_list[], //Array to hold list; size = # channels + 1
  unsigned short fast_list[], //Array to hold list; size = # channels + 1
  float (* get_float_using_id)(short unsigned int data_id) //Pointer to function to get value at data_id
  );
                             
void a9_bt_data_sender(
  unsigned short max_data_id,      //Highest data_id to send at slow rate (0 is lowest)
  unsigned short subscriber_number,//Unique process identifier to determine whether data has been read, range 0 - 31
  unsigned short medium_list[],  //Pointer to array (list) of data_ids to send at medium rate                                //
  unsigned short fast_list[]    //Pointer to array (list) of data_ids to send at fast rate
  );                                                                   

unsigned short a9_bt_push_io_data_point(DATA_FRAME * data_point);
void a9_software_int_isr(void);
void a9_gpdma_isr(void);
void a9_bt_start_ch3_dma_transfer(void);
void a9_bt_dma_receive(void);
void mb_bt_pop_receive_packets(void);

#endif// __MB_BT_H__
