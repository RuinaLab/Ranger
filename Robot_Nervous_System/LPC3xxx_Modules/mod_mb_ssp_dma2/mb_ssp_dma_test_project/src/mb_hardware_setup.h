#ifndef __MB_HW_SETUP_H__
#define __MB_HW_SETUP_H__

//Note that these are output lines, not LEDs
//The actual LEDs, if any, would be lit on the carrier board
#define A9_DN_GREEN_LED_ON P3_OUTP_SET = 1<<25;
#define A9_DN_GREEN_LED_OFF P3_OUTP_CLR = 1<<25;

#define A9_DN_RED_LED_ON P3_OUTP_SET = 1<<26;
#define A9_DN_RED_LED_OFF P3_OUTP_CLR = 1<<26;


void mb_setup_hardware(void);

void mb_timer0_isr(void);


#endif /* __MB_HW_SETUP_H__ */
