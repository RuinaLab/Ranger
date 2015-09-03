#ifndef __HW_SETUP_H__
#define __HW_SETUP_H__

#define B2A_OVERRIDE 0

void setup_hardware(void);

void timer0_isr(void) __irq;

// Helpful LED macros
#define MCU_LED_BLUE_ON FIO1CLR = (1<<25);
#define MCU_LED_BLUE_OFF FIO1SET = (1<<25);
#define MCU_LED_GREEN_ON FIO1CLR = (1<<23);
#define MCU_LED_GREEN_OFF FIO1SET = (1<<23);
#define MCU_LED_RED_ON FIO1CLR = (1<<24);
#define MCU_LED_RED_OFF FIO1SET = (1<<24);
#define MCU_LED_ALL_ON FIO1CLR = (1<<25)|(1<<24)|(1<<23);
#define MCU_LED_ALL_OFF FIO1SET = (1<<25)|(1<<24)|(1<<23);

#endif /* __HW_SETUP_H__ */
