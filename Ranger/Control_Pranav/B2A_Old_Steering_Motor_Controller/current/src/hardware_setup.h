#ifndef __HW_SETUP_H__
#define __HW_SETUP_H__

void init_hardware(void);
void timer1_isr(void) __irq;

//*******************************************************************************
//PLL USER DEFINED VALUES
//*******************************************************************************
#define CRYSTAL 10000000 //in Hertz

/*60MHz*/
//#define CPUSPEED 60000000 //in Hertz
#define MSEL 5
#define PSEL 1
#define CPUSPEED 60000000

// Scheduler
#define SCHED_SPEED 2 //in khz
#define SCHED_MATCH (CPUSPEED/(1000*SCHED_SPEED))

// CAN buses - select bus(es) to use
#define USE_CAN1
//#define USE_CAN2
//#define USE_CAN3
//#define USE_CAN4


// Helpful MCU_LED macros
/*#define MCU_LED_BLUE_ON FIO1CLR = (1<<25);
#define MCU_LED_BLUE_OFF FIO1SET = (1<<25);
#define MCU_LED_BLUE_TOGGLE if(FIO1PIN & (1<<25)){MCU_LED_BLUE_ON;} else{MCU_LED_BLUE_OFF;}
#define MCU_LED_GREEN_ON FIO1CLR = (1<<23);
#define MCU_LED_GREEN_OFF FIO1SET = (1<<23);
#define MCU_LED_GREEN_TOGGLE if(FIO1PIN & (1<<23)){MCU_LED_GREEN_ON;} else{MCU_LED_GREEN_OFF;}
#define MCU_LED_RED_ON FIO1CLR = (1<<24);
#define MCU_LED_RED_OFF FIO1SET = (1<<24);
#define MCU_LED_RED_TOGGLE if(FIO1PIN & (1<<24)){MCU_LED_RED_ON;} else{MCU_LED_RED_OFF;}
#define MCU_LED_ALL_ON FIO1CLR = (1<<25)|(1<<24)|(1<<23);
#define MCU_LED_ALL_OFF FIO1SET = (1<<25)|(1<<24)|(1<<23);
#define MCU_LED_ALL_TOGGLE MCU_LED_RED_TOGGLE; MCU_LED_BLUE_TOGGLE; MCU_LED_GREEN_TOGGLE; */

#endif /* __HW_SETUP_H__ */
