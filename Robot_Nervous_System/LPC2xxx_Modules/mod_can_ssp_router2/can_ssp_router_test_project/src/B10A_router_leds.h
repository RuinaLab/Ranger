#ifndef  __CAN_SSP_ROUTER_H__
#define  __CAN_SSP_ROUTER_H__

//__irq void csr_ssp_isr(void);

#define CSR_TIMER_TICKS_PER_MS  5  //e.g., if timer interrupts occur at 100 uS intervals, this = 10.
void csr_clock_tick(void);
unsigned long int csr_elapsed_ms(void);
void csr_send_can_time_packets(long unsigned int time);

unsigned short int csr_can1_tx_push_frame(CAN_FRAME * frameptr);
unsigned short int csr_can2_tx_push_frame(CAN_FRAME * frameptr);
unsigned short int csr_can3_tx_push_frame(CAN_FRAME * frameptr);
unsigned short int csr_can4_tx_push_frame(CAN_FRAME * frameptr);

void csr_can1_tx_isr(void) __irq;
void csr_can2_tx_isr(void) __irq;
void csr_can3_tx_isr(void) __irq;
void csr_can4_tx_isr(void) __irq;

//csr_leds.c public functions

void csr_can1_packet_count(void);
void csr_can2_packet_count(void);
void csr_can3_packet_count(void);
void csr_can4_packet_count(void);
void csr_can1_red_led_blink(short unsigned int time);
void csr_can1_green_led_blink(short unsigned int time);
void csr_can2_red_led_blink(short unsigned int time);
void csr_can2_green_led_blink(short unsigned int time);
void csr_can3_red_led_blink(short unsigned int time);
void csr_can3_green_led_blink(short unsigned int time);
void csr_can4_red_led_blink(short unsigned int time);
void csr_can4_green_led_blink(short unsigned int time);
void csr_mcu_red_led_blink(short unsigned int time);
void csr_mcu_green_led_blink(short unsigned int time);
void csr_mcu_blue_led_blink(short unsigned int time);

//Private functions from csr_leds.c
void csr_update_mcu_leds(void);
void csr_update_can_leds(void);


void csr_synchronize_arm9(void);
void csr_global_variable_init(void);
void csr_routing_table_init(void);
void csr_init_can_rings(void);
void csr_ssp_isr(void); //Set up for FIQ interrupts
void csr_route(void);
unsigned short int csr_route_frame(CAN_FRAME * frame);
unsigned short int csr_push_ssp_frame(CAN_FRAME * frame);
unsigned short int csr_pop_ssp_frame(CAN_FRAME * frame);
unsigned short int csr_can1_tx_push_frame(CAN_FRAME * frameptr);
void csr_can1_tx_isr(void) __irq;
unsigned short int csr_can2_tx_push_frame(CAN_FRAME * frameptr);
void csr_can2_tx_isr(void) __irq;
unsigned short int csr_can3_tx_push_frame(CAN_FRAME * frameptr);
void csr_can3_tx_isr(void) __irq;
unsigned short int csr_can4_tx_push_frame(CAN_FRAME * frameptr);
void csr_can4_tx_isr(void) __irq;

#endif// __CAN_SSP_ROUTER_H__

