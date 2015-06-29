#ifndef __LPC3250DEF__
#define __LPC3250DEF__


/***************************************************/
/* LPC3250 Registers missing from Keil header file */
/***************************************************/


// DMA reserved bit masks (partial)
  #define MIC_ITR_RBMASK (~(1<<12 | 1<<2))
  #define MIC_ATR_RBMASK (~(1<<12 | 1<<2))
  #define MIC_APR_RBMASK (~(1<<12 | 1<<2))
  #define MIC_ER_RBMASK  (~(1<<12 | 1<<2))
  #define SIC1_ITR_RBMASK (~(1<<21 | 1<<16 | 1<<15 | 1<<11 | 1<<10 | 1<<9 | 1<<5 | 1<<3 | 1<<0))
  #define SIC1_ATR_RBMASK (~(1<<21 | 1<<16 | 1<<15 | 1<<11 | 1<<10 | 1<<9 | 1<<5 | 1<<3 | 1<<0))
  #define SIC1_APR_RBMASK (~(1<<21 | 1<<16 | 1<<15 | 1<<11 | 1<<10 | 1<<9 | 1<<5 | 1<<3 | 1<<0))
  #define SIC1_ER_RBMASK  (~(1<<21 | 1<<16 | 1<<15 | 1<<11 | 1<<10 | 1<<9 | 1<<5 | 1<<3 | 1<<0))
  #define SIC2_ITR_RBMASK (~(1<<30 | 1<<29 | 1<<21 | 1<<17 | 1<<16 | 1<<14 | 1<<13))
  #define SIC2_ATR_RBMASK (~(1<<30 | 1<<29 | 1<<21 | 1<<17 | 1<<16 | 1<<14 | 1<<13))
  #define SIC2_APR_RBMASK (~(1<<30 | 1<<29 | 1<<21 | 1<<17 | 1<<16 | 1<<14 | 1<<13))
  #define SIC2_ER_RBMASK  (~(1<<30 | 1<<29 | 1<<21 | 1<<17 | 1<<16 | 1<<14 | 1<<13))

 /* LCD Controller */
 //Not using this, but it needs to be off to use other pin functions.
 //So, only the control register is here.
#define LCD_CTRL (*((volatile unsigned long *) 0x31040018))

// Standard Timer/Counter clock control register
#define TIMCLK_CTRL1 (*((volatile unsigned long *) 0x400040C0))

// Standard Timer/Counter 0
#define T0IR 	(*((volatile unsigned long *) 0x40044000))
#define T0TCR 	(*((volatile unsigned long *) 0x40044004))
#define T0TC 	(*((volatile unsigned long *) 0x40044008))
#define T0PR 	(*((volatile unsigned long *) 0x4004400C))
#define T0PC 	(*((volatile unsigned long *) 0x40044010))
#define T0MCR 	(*((volatile unsigned long *) 0x40044014))
#define T0MR0 	(*((volatile unsigned long *) 0x40044018))
#define T0MR1 	(*((volatile unsigned long *) 0x4004401C))
#define T0MR2 	(*((volatile unsigned long *) 0x40044020))
#define T0MR3 	(*((volatile unsigned long *) 0x40044024))
#define T0CCR 	(*((volatile unsigned long *) 0x40044028))
#define T0CR0 	(*((volatile unsigned long *) 0x4004402C))
#define T0CR1 	(*((volatile unsigned long *) 0x40044030))
#define T0CR2 	(*((volatile unsigned long *) 0x40044034))
#define T0CR3 	(*((volatile unsigned long *) 0x40044038))
#define T0EMR 	(*((volatile unsigned long *) 0x4004403C))
#define T0CTCR 	(*((volatile unsigned long *) 0x40044070))

// Standard Timer/Counter 1
#define T1IR 	(*((volatile unsigned long *) 0x4004C000))
#define T1TCR 	(*((volatile unsigned long *) 0x4004C004))
#define T1TC 	(*((volatile unsigned long *) 0x4004C008))
#define T1PR 	(*((volatile unsigned long *) 0x4004C00C))
#define T1PC 	(*((volatile unsigned long *) 0x4004C010))
#define T1MCR 	(*((volatile unsigned long *) 0x4004C014))
#define T1MR0 	(*((volatile unsigned long *) 0x4004C018))
#define T1MR1 	(*((volatile unsigned long *) 0x4004C01C))
#define T1MR2 	(*((volatile unsigned long *) 0x4004C020))
#define T1MR3 	(*((volatile unsigned long *) 0x4004C024))
#define T1CCR 	(*((volatile unsigned long *) 0x4004C028))
#define T1CR0 	(*((volatile unsigned long *) 0x4004C02C))
#define T1CR1 	(*((volatile unsigned long *) 0x4004C030))
#define T1CR2 	(*((volatile unsigned long *) 0x4004C034))
#define T1CR3 	(*((volatile unsigned long *) 0x4004C038))
#define T1EMR 	(*((volatile unsigned long *) 0x4004C03C))
#define T1CTCR 	(*((volatile unsigned long *) 0x4004C070))

/* SSP0 Controller */

#define SSP_CTRL 		(*((volatile unsigned long *) 0x40004078))
#define SSP0CR0         (*((volatile unsigned long *) 0x20084000))
#define SSP0CR1         (*((volatile unsigned long *) 0x20084004))
#define SSP0DR          (*((volatile unsigned long *) 0x20084008))
#define SSP0SR          (*((volatile unsigned long *) 0x2008400C))
#define SSP0CPSR        (*((volatile unsigned long *) 0x20084010))
#define SSP0IMSC        (*((volatile unsigned long *) 0x20084014))
#define SSP0RIS         (*((volatile unsigned long *) 0x20084018))
#define SSP0MIS         (*((volatile unsigned long *) 0x2008401C))
#define SSP0ICR         (*((volatile unsigned long *) 0x20084020))
#define SSP0DMACR       (*((volatile unsigned long *) 0x20084024))

/* SSP1 Controller */

#define SSP1CR0         (*((volatile unsigned long *) 0x2008C000))
#define SSP1CR1         (*((volatile unsigned long *) 0x2008C004))
#define SSP1DR          (*((volatile unsigned long *) 0x2008C008))
#define SSP1SR          (*((volatile unsigned long *) 0x2008C00C))
#define SSP1CPSR        (*((volatile unsigned long *) 0x2008C010))
#define SSP1IMSC        (*((volatile unsigned long *) 0x2008C014))
#define SSP1RIS         (*((volatile unsigned long *) 0x2008C018))
#define SSP1MIS         (*((volatile unsigned long *) 0x2008C01C))
#define SSP1ICR         (*((volatile unsigned long *) 0x2008C020))
#define SSP1DMACR       (*((volatile unsigned long *) 0x2008C024))



#endif //__LPC3250DEF__

