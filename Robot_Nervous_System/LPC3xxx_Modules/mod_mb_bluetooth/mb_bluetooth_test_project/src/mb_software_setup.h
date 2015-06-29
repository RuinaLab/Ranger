#ifndef __MB_SW_SETUP_H__
#define __MB_SW_SETUP_H__

//#define DEBUG
#define BT_HSU1_SERIAL_PORT
//#define BT_HSU2_BLUETOOTH

typedef enum software_interrupt_designator
{ //Max value is 127
  A9_SW_INT_DMA_CH3
} SOFTWARE_INTERRUPT_DESIGNATOR;

void mb_create_display_data_lists(void);
void mb_update_execution_time(void);
void mb_send_data(void);
void mb_setup_software(void);

#endif /* __MB_SW_SETUP_H__ */
