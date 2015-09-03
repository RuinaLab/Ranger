/* Hardware setup header file.*/

#ifndef HARD_SETUP
#define HARD_SETUP

//*******************************************************************************
//PLL USER DEFINED VALUES
//*******************************************************************************
#define CRYSTAL 10000000 //in Hertz

/*60MHz*/
#define CPUSPEED 60000000 //in Hertz
#define MSEL 5
#define PSEL 1

/*50MHz*/
//#define CPUSPEED 50000000 //in Hertz
//#define MSEL 4
//#define PSEL 1

/*40MHz*/
//#define CPUSPEED 40000000 //in Hertz
//#define MSEL 3
//#define PSEL 1

/* !!!!PLEASE CHECK !!!!
 ---if it is set wrongly, microcontroller WOULD NOT RUN!---
 a) 10000000 < CRYSTAL < 25000000
 b) 10000000 < CPUSPEED < 60000000
 c) CPUSPEED = M * CRYSTAL
 d) FCCO = CPUSPEED * 2 * P
 156000000 < FCCO < 320000000
 e) M = 1, 2, ..., 32
 MSEL = M - 1;
 f) P = 1 , 2 , 4 , 8
 PSEL = 00 01 10 11 */


 /*************
 Timers
 *************/
#define TIMESPERSEC 2

//Function Declaration
int setup_hardware(void);

#endif	  

