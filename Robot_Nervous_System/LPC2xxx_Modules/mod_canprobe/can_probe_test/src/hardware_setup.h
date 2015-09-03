#ifndef __HW_SETUP_H__
#define __HW_SETUP_H__

void setup_hardware(void);

void Timer0_ISR(void) __irq;

#define BOARD_NAME 1
#define MCU_LED_RED_ON
#define MCU_LED_GREEN_ON

#endif /* __HW_SETUP_H__ */
