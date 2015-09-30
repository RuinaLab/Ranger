#ifndef __HW_SETUP_H__
#define __HW_SETUP_H__

void init_hardware(void);
void init_interrupts(void);

//*******************************************************************************
//PLL USER DEFINED VALUES
//*******************************************************************************
#define CRYSTAL 10000000 //in Hertz

/*60MHz*/
//#define CPUSPEED 60000000 //in Hertz
#define MSEL 5
#define PSEL 1


// *************************************
// Timers
// ************************************* 
#define SCHED_SPEED 1 //in khz

// CAN buses - select bus(es) to use
//#define USE_CAN1
#define USE_CAN2
//#define USE_CAN3
//#define USE_CAN4
 

#endif /* __HW_SETUP_H__ */
