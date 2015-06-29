#ifndef __A9_CLOCK_H__
#define __A9_CLOCK_H__

#define A9_CLOCK_TICKS_PER_MS 5

float mb_clock_get_execution_time(void);
void a9_clock_tick(void);
void a9_timer0_isr(void);
//unsigned long int mb_clock_get_time(void);





#endif  //__A9_CLOCK_H__
