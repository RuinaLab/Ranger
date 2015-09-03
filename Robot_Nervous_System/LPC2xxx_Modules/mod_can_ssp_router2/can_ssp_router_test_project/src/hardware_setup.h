#ifndef __HW_SETUP_H__
#define __HW_SETUP_H__

#define B2A_OVERRIDE 0

void setup_hardware(void);

void timer0_isr(void) __irq;

void b10a_blink_red_can_led(unsigned short chan);
void b10a_blink_green_can_led(unsigned short chan);
void b10a_can_packet_count(unsigned short chan);
void b10a_update_mcu_leds(void);
void b10a_mcu_red_led_blink(short unsigned int time);
void b10a_mcu_green_led_blink(short unsigned int time);
void b10a_mcu_blue_led_blink(short unsigned int time);
void b10a_update_can_leds(void);

// Helpful LED macros
#define MCU_LED_BLUE_ON FIO1CLR = (1<<25);
#define MCU_LED_BLUE_OFF FIO1SET = (1<<25);
#define MCU_LED_GREEN_ON FIO1CLR = (1<<23);
#define MCU_LED_GREEN_OFF FIO1SET = (1<<23);
#define MCU_LED_RED_ON FIO1CLR = (1<<24);
#define MCU_LED_RED_OFF FIO1SET = (1<<24);
#define MCU_LED_ALL_ON FIO1CLR = (1<<25)|(1<<24)|(1<<23);
#define MCU_LED_ALL_OFF FIO1SET = (1<<25)|(1<<24)|(1<<23);

#define CAN1_LED_GREEN_OFF  FIO0SET = 1<<30; 	// turn off green CAN1 LED
#define CAN1_LED_RED_OFF  FIO1SET = 1<<16;	 	// turn off red CAN1 LED

#define CAN2_LED_GREEN_OFF  FIO0SET = 1<<2; 	// turn off green CAN2 LED
#define CAN2_LED_RED_OFF  FIO0SET = 1<<3;	 		// turn off red CAN2 LED

#define CAN3_LED_GREEN_OFF  FIO1SET = 1<<19; 	// turn off green CAN3 LED
#define CAN3_LED_RED_OFF  FIO1SET = 1<<18;	 	// turn off red CAN3 LED

#define CAN4_LED_GREEN_OFF FIO0SET = 1<<10; 	// turn off green CAN4 LED
#define CAN4_LED_RED_OFF  FIO0SET = 1<<11;	 	// turn off red CAN4 LED

#define CAN1_LED_GREEN_ON  FIO0CLR = 1<<30; 	// turn on green CAN1 LED
#define CAN1_LED_RED_ON  FIO1CLR = 1<<16;	 		// turn on red CAN1 LED

#define CAN2_LED_GREEN_ON  FIO0CLR = 1<<2; 	 	// turn on green CAN2 LED
#define CAN2_LED_RED_ON  FIO0CLR = 1<<3;	 		// turn on red CAN2 LED

#define CAN3_LED_GREEN_ON  FIO1CLR = 1<<19; 	// turn on green CAN3 LED
#define CAN3_LED_RED_ON  FIO1CLR = 1<<18;	 		// turn on red CAN3 LED

#define CAN4_LED_GREEN_ON FIO0CLR = 1<<10; 	 	// turn on green CAN4 LED
#define CAN4_LED_RED_ON  FIO0CLR = 1<<11;	 		// turn on red CAN4 LED


#endif /* __HW_SETUP_H__ */
