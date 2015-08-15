#ifndef __MB_SW_SETUP_H__
#define __MB_SW_SETUP_H__

//#define DEBUG
#define BT_HSU1_SERIAL_PORT //Data collection via serial port (comment line below)
//#define BT_HSU2_BLUETOOTH //Data collection via blue-tooth (comment earlier line)

typedef enum software_interrupt_designator
{
  A9_SW_INT_DMA_CH3
} SOFTWARE_INTERRUPT_DESIGNATOR;

//Call C functions from C++
#ifdef __cplusplus
extern "C"
{
#endif

  unsigned long mb_get_timestamp(void);
  float get_io_float(unsigned short data_id);
  float get_io_ul(unsigned short data_id);
  void set_io_ul(short unsigned int data_id, unsigned long value);
  void set_io_float(short unsigned int data_id, float value);
  void mark_as_read(unsigned short data_id, unsigned short subscriber_id);
  unsigned short data_was_read(unsigned short data_id, unsigned short subscriber_id);
  
#ifdef __cplusplus  
}
#endif


void mb_read_error(void);
unsigned long mb_get_timestamp(void);
//void mb_update_elapsed_time(void);
void mb_update_execution_time(void);
void mb_create_display_data_lists(void);
void mb_send_data(void);
void mb_setup_software(void);

#endif /* __MB_SW_SETUP_H__ */
