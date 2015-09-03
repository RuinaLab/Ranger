#ifndef __SW_SETUP_H__
#define __SW_SETUP_H__

//#define DEBUG

#define BOARD_NAME BOARD_CAN_SSP_ROUTER

#define SCHED_SPEED 1   //Schedule tick call rate in kilohertz

void setup_software(void);
void heartbeat_blink_blue(void);

#endif /* __SW_SETUP_H__ */
