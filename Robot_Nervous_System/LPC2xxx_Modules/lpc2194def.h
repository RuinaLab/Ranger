#ifndef LPC2194DEF_H
#define LPC2194DEF_H

#define IDLE PCON = 1;

/* Processor specific defines for the LPC2194 cpu */


/******************************************/
// LPC2194/01 reserved bit mask macros.
// Use & with these to get rid of reserved bits.
/******************************************/
#define PCONP_RBM 0x21FFBE


/******************************************/
// LPC2194/01 Registers missing from Keil header file
/******************************************/
//Timer 0/1 Count Control Register
#define T0CTCR  (*((volatile unsigned long *) 0xE0004070))
#define T1CTCR  (*((volatile unsigned long *) 0xE0008070))

//UART 0/1 Fractional Divider Register
#define U0FDR   (*((volatile unsigned long *) 0xE000C028))
#define U1FDR   (*((volatile unsigned long *) 0xE0010028))

//UART 0/1 Autobaud Register
#define U0ACR   (*((volatile unsigned long *) 0xE000C020))
#define U1ACR   (*((volatile unsigned long *) 0xE0010020))

//UART 0/1 Transmitter Enable Register
#define U0TER   (*((volatile unsigned long *) 0xE000C030))
#define U1TER   (*((volatile unsigned long *) 0xE0010030))

//UART 0/1 Interrupt Enable Register
//Keil's version of this is a char, but should be a 16-bit word.
#define U0IER_01          (*((volatile unsigned short int *) 0xE000C004))
#define U1IER_01          (*((volatile unsigned short int *) 0xE0010004))

//System control and status flags register (used for enabling fast GPIO)
#define SCS     (*((volatile unsigned long *)  0xE01FC1A0))

//Fast GPIO registers

  //32-bit (word) fast GPIO direction control registers
#define FIO0DIR     (*((volatile unsigned long *)  0x3FFFC000))
#define FIO1DIR     (*((volatile unsigned long *)  0x3FFFC020))

  //16-bit (half-word) fast GPIO direction control registers
#define FIO0DIRL     (*((volatile unsigned short *)  0x3FFFC000))
#define FIO0DIRU     (*((volatile unsigned short *)  0x3FFFC002))

#define FIO1DIRL     (*((volatile unsigned short *)  0x3FFFC020))
#define FIO1DIRU     (*((volatile unsigned short *)  0x3FFFC022))

  //8-bit (byte) fast GPIO direction control registers
#define FIO0DIR0     (*((volatile unsigned char *)  0x3FFFC000))
#define FIO0DIR1    (*((volatile unsigned char *)  0x3FFFC001))
#define FIO0DIR2     (*((volatile unsigned char *)  0x3FFFC002))
#define FIO0DIR3    (*((volatile unsigned char *)  0x3FFFC003))

#define FIO1DIR0     (*((volatile unsigned char *)  0x3FFFC020))
#define FIO1DIR1    (*((volatile unsigned char *)  0x3FFFC021))
#define FIO1DIR2     (*((volatile unsigned char *)  0x3FFFC022))
#define FIO1DIR3    (*((volatile unsigned char *)  0x3FFFC023))

  //32-bit (word) fast GPIO mask registers
#define FIO0MASK     (*((volatile unsigned long *)  0x3FFFC010))
#define FIO1MASK     (*((volatile unsigned long *)  0x3FFFC030))

  //16-bit (half-word) fast GPIO mask registers
#define FIO0MASKL     (*((volatile unsigned short *)  0x3FFFC010))
#define FIO0MASKU     (*((volatile unsigned short *)  0x3FFFC012))

#define FIO1MASKL     (*((volatile unsigned short *)  0x3FFFC030))
#define FIO1MASKU     (*((volatile unsigned short *)  0x3FFFC032))

  //8-bit (byte) fast GPIO mask registers
#define FIO0MASK0     (*((volatile unsigned char *)  0x3FFFC010))
#define FIO0MASK1     (*((volatile unsigned char *)  0x3FFFC011))
#define FIO0MASK2     (*((volatile unsigned char *)  0x3FFFC012))
#define FIO0MASK3     (*((volatile unsigned char *)  0x3FFFC013))

#define FIO1MASK0     (*((volatile unsigned char *)  0x3FFFC030))
#define FIO1MASK1     (*((volatile unsigned char *)  0x3FFFC031))
#define FIO1MASK2     (*((volatile unsigned char *)  0x3FFFC032))
#define FIO1MASK3     (*((volatile unsigned char *)  0x3FFFC033))

  //32-bit (word) fast GPIO pin value registers
#define FIO0PIN     (*((volatile unsigned long *)  0x3FFFC014))
#define FIO1PIN     (*((volatile unsigned long *)  0x3FFFC034))

  //16-bit (half-word) fast GPIO pin value registers
#define FIO0PINL     (*((volatile unsigned short *)  0x3FFFC014))
#define FIO0PINU     (*((volatile unsigned short *)  0x3FFFC016))

#define FIO1PINL     (*((volatile unsigned short *)  0x3FFFC034))
#define FIO1PINU     (*((volatile unsigned short *)  0x3FFFC036))

  //8-bit (byte) fast GPIO pin value registers
#define FIO0PIN0     (*((volatile unsigned char *)  0x3FFFC014))
#define FIO0PIN1     (*((volatile unsigned char *)  0x3FFFC015))
#define FIO0PIN2     (*((volatile unsigned char *)  0x3FFFC016))
#define FIO0PIN3     (*((volatile unsigned char *)  0x3FFFC017))

#define FIO1PIN0     (*((volatile unsigned char *)  0x3FFFC034))
#define FIO1PIN1     (*((volatile unsigned char *)  0x3FFFC035))
#define FIO1PIN2     (*((volatile unsigned char *)  0x3FFFC036))
#define FIO1PIN3     (*((volatile unsigned char *)  0x3FFFC037))

  //32-bit (word) fast GPIO pin set registers
#define FIO0SET     (*((volatile unsigned long *)  0x3FFFC018))
#define FIO1SET     (*((volatile unsigned long *)  0x3FFFC038))

  //16-bit (half-word) fast GPIO pin set registers
#define FIO0SETL     (*((volatile unsigned short *)  0x3FFFC018))
#define FIO0SETU     (*((volatile unsigned short *)  0x3FFFC01A))

#define FIO1SETL     (*((volatile unsigned short *)  0x3FFFC038))
#define FIO1SETU     (*((volatile unsigned short *)  0x3FFFC03A))

  //8-bit (byte) fast GPIO pin set registers
#define FIO0SET0     (*((volatile unsigned char *)  0x3FFFC018))
#define FIO0SET1     (*((volatile unsigned char *)  0x3FFFC019))
#define FIO0SET2     (*((volatile unsigned char *)  0x3FFFC01A))
#define FIO0SET3     (*((volatile unsigned char *)  0x3FFFC01B))

#define FIO1SET0     (*((volatile unsigned char *)  0x3FFFC038))
#define FIO1SET1     (*((volatile unsigned char *)  0x3FFFC039))
#define FIO1SET2     (*((volatile unsigned char *)  0x3FFFC03A))
#define FIO1SET3     (*((volatile unsigned char *)  0x3FFFC03B))

  //32-bit (word) fast GPIO pin clear registers
#define FIO0CLR     (*((volatile unsigned long *)  0x3FFFC01C))
#define FIO1CLR     (*((volatile unsigned long *)  0x3FFFC03C))

  //16-bit (half-word) fast GPIO pin clear registers
#define FIO0CLRL     (*((volatile unsigned short *)  0x3FFFC01C))
#define FIO0CLRU     (*((volatile unsigned short *)  0x3FFFC01E))

#define FIO1CLRL     (*((volatile unsigned short *)  0x3FFFC03C))
#define FIO1CLRU    (*((volatile unsigned short *)  0x3FFFC03E))

  //8-bit (byte) fast GPIO pin clear registers
#define FIO0CLR0     (*((volatile unsigned char *)  0x3FFFC01C))
#define FIO0CLR1     (*((volatile unsigned char *)  0x3FFFC01D))
#define FIO0CLR2     (*((volatile unsigned char *)  0x3FFFC01E))
#define FIO0CLR3     (*((volatile unsigned char *)  0x3FFFC01F))

#define FIO1CLR0     (*((volatile unsigned char *)  0x3FFFC03C))
#define FIO1CLR1     (*((volatile unsigned char *)  0x3FFFC03D))
#define FIO1CLR2     (*((volatile unsigned char *)  0x3FFFC03E))
#define FIO1CLR3     (*((volatile unsigned char *)  0x3FFFC03F))

  //Analog to Digital Converter (ADC)
#define ADINTEN       (*((volatile unsigned long *)  0xE003400C))
#define ADGDR         (*((volatile unsigned long *)  0xE0034004))
#define ADDR0         (*((volatile unsigned long *)  0xE0034010))
#define ADDR1         (*((volatile unsigned long *)  0xE0034014))
#define ADDR2         (*((volatile unsigned long *)  0xE0034018))
#define ADDR3         (*((volatile unsigned long *)  0xE003401C))





/* SSP Controller (SPI1) */

#define SSPCR0         (*((volatile unsigned long *) 0xE005C000))
#define SSPCR1         (*((volatile unsigned long *) 0xE005C004))
#define SSPDR          (*((volatile unsigned long *) 0xE005C008))
#define SSPSR          (*((volatile unsigned long *) 0xE005C00C))
#define SSPCPSR        (*((volatile unsigned long *) 0xE005C010))
#define SSPIMSC        (*((volatile unsigned long *) 0xE005C014))
#define SSPRIS         (*((volatile unsigned long *) 0xE005C018))
#define SSPMIS         (*((volatile unsigned long *) 0xE005C01C))
#define SSPICR         (*((volatile unsigned long *) 0xE005C020))

/***************************************************************************
 **
 **  VIC Interrupt channels ----  Copied from EWARM iolpc2194.h
 **
 ***************************************************************************/
#define VIC_WDT          0  /* Watchdog                           */
#define VIC_SW           1  /* Software interrupts                */
#define VIC_DEBUGRX      2  /* Embedded ICE, DbgCommRx            */
#define VIC_DEBUGTX      3  /* Embedded ICE, DbgCommTx            */
#define VIC_TIMER0       4  /* Timer 0 (Match 0-3 Capture 0-3)    */
#define VIC_TIMER1       5  /* Timer 1 (Match 0-3 Capture 0-3)    */
#define VIC_UART0        6  /* UART 0  (RLS, THRE, RDA, CTI)      */
#define VIC_UART1        7  /* UART 1  (RLS, THRE, RDA, CTI, MSI) */
#define VIC_PWM0         8  /* PWM 0   (Match 0-6 Capture 0-3)    */
#define VIC_I2C          9  /* I2C     (SI)                       */
#define VIC_SPI0        10  /* SPI0    (SPIF, MODF)               */
#define VIC_SPI1        11  /* SPI1    (SPIF, MODF)               */
#define VIC_PLL         12  /* PLL lock (PLOCK)                   */
#define VIC_RTC         13  /* RTC     (RTCCIF, RTCALF)           */
#define VIC_EINT0       14  /* External interrupt 0 (EINT0)       */
#define VIC_EINT1       15  /* External interrupt 1 (EINT1)       */
#define VIC_EINT2       16  /* External interrupt 2 (EINT2)       */
#define VIC_EINT3       17  /* External interrupt 3 (EINT3)       */
#define VIC_AD          18  /* Analog to digital converter        */
#define VIC_CAN_AF      19  /* CAN and Acceptance Filter          */
#define VIC_CAN1TX      20  /* CAN1 Tx                            */
#define VIC_CAN2TX      21  /* CAN2 Tx                            */
#define VIC_CAN3TX      22  /* CAN3 Tx                            */
#define VIC_CAN4TX      23  /* CAN4 Tx                            */
//#define VIC_RES       24  /* Reserved                           */
//#define VIC_RES       25  /* Reserved                           */
#define VIC_CAN1RX      26  /* CAN1 Rx                            */
#define VIC_CAN2RX      27  /* CAN2 Rx                            */
#define VIC_CAN3RX      28  /* CAN3 Rx                            */
#define VIC_CAN4RX      29  /* CAN4 Rx                            */
//#define VIC_RES       30  /* Reserved                           */
//#define VIC_RES       31  /* Reserved                           */

/***************************************************************************
 **
 **  Reserved Bits of registers, used to mask out undefined bits
 **
 ***************************************************************************/
 
#define PCONP_RB (unsigned int)((1<<0)|(1<<6)|(63<<17)|(0xFF000000))
#define EXTMODE_RB (unsigned int)(15<<4)
#define PWMMCR_RB (unsigned int)(0xFFE00000)
#define PWMTCR_RB (unsigned int)((1<<2)|(15<<4))
#define PWMPCR_RB (unsigned int)((3<<0)|(3<<7)|(1<<15))
#define PWMLER_RB (unsigned int)(1<<7)
#define ADCR_RB (unsigned int)((1<<20)|(3<<22)|(0xF0000000))
#define ADINTEN_RB (unsigned int)(0xFFFFFE00)
#define C1MOD_RB (unsigned int)(1<<6)
#define C1GSR_RB (unsigned int)(255<<8)
#define C1IER_RB (unsigned int)(0xFFF80000)
#define C1BTR_RB (unsigned int)((15<<10)|(0xFF000000))

#endif // LPC2194DEF_H

