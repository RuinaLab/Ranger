/***************************************************************************                        
 **    This file defines the Special Function Registers for	NXP LPC2194/01                         
 **    Modified (c) Copyright IAR Systems 2003   
 **        Modified by Seong-hee Lee (Emily)                               
 **    $Revision: 1.11.2.1 $   (Modified)
 **                                
 **    Note: Only little endian addressing of 8 bit registers.
 ***************************************************************************/

#ifndef LPC2194_01_H
#define LPC2194_01_H

typedef volatile unsigned char  REG8;
typedef volatile unsigned short REG16;
typedef volatile unsigned long  REG32;

/* External interrupt register */
typedef union{ 
struct {
  REG32 EINT0           :1;
  REG32 EINT1           :1;
  REG32 EINT2           :1;			  
  REG32 EINT3           :1;
  REG32 /**/            :28;
  } bit_extint;
 REG32 reg_extint;
} extint_bits;

/* External interrupt wakeup register */
typedef union{
struct {
  REG32 EXTWAKE0        :1;
  REG32 EXTWAKE1        :1;
  REG32 EXTWAKE2        :1;
  REG32 EXTWAKE3        :1;
  REG32 /**/            :28;
  }	bit_extwake;
  REG32	reg_extwake;
} extwake_bits;

/* External interrupt mode register */
typedef union{
struct {
  REG32 EXTMODE0        :1;
  REG32 EXTMODE1        :1;
  REG32 EXTMODE2        :1;
  REG32 EXTMODE3        :1;
  REG32 /**/            :28; 
  }	bit_extmode;
  REG32	reg_extmode;
} extmode_bits;

/* External interrupt polarity register */
typedef union{
struct {
  REG32 EXTPOLAR0        :1;
  REG32 EXTPOLAR1        :1;
  REG32 EXTPOLAR2        :1;
  REG32 EXTPOLAR3        :1;
  REG32 /**/             :28;
  }	bit_extpolar;
  REG32	reg_extpolar;
} extpolar_bits;

/* Memory mapping control register */
typedef union{
struct {
  REG32 MAP             :2;
  REG32 /**/            :30;
  }	bit_memmap;
  REG32	reg_memmap;
} memmap_bits;

/* PLL control register */
typedef union{
struct {
  REG32 PLLE            :1;
  REG32 PLLC            :1;
  REG32 /**/            :30;
  }	bit_pllcon;
  REG32	reg_pllcon;
} pllcon_bits;

/* PLL config register */
typedef union{
struct {
  REG32 MSEL            :5;
  REG32 PSEL            :2;
  REG32 /**/            :25;
  }	bit_pllcfg;
  REG32	reg_pllcfg;
} pllcfg_bits;

/* PLL status register */
typedef union{
struct {
  REG32 MSEL            :5;
  REG32 PSEL            :2;
  REG32 /**/            :1;
  REG32 PLLE            :1;
  REG32 PLLC            :1;
  REG32 PLOCK           :1;
  REG32 /**/            :21;
  }	bit_pllstat;
  REG32	reg_pllstat;
} pllstat_bits;

/* PLL feed register */
typedef union{
struct {
  REG32 FEED            :8;
  REG32 /**/            :24;
  }	bit_pllfeed;
  REG32	reg_pllfeed;
} pllfeed_bits;

/* Power control register */
typedef union{
struct {
  REG32 IDL             :1;
  REG32 PD              :1;
  REG32 /**/            :30;
  }	bit_pcon;
  REG32	reg_pcon;
} pcon_bits;

/* Power control for peripherals register LPC2194/01 */
typedef union{
struct {
  REG32 /**/            :1;
  REG32 PCTIM0          :1;
  REG32 PCTIM1          :1;
  REG32 PCURT0          :1;
  REG32 PCURT1          :1;
  REG32 PCPWM0          :1;
  REG32 /**/            :1;
  REG32 PCI2C           :1;
  REG32 PCSPI0          :1;
  REG32 PCRTC           :1;
  REG32 PCSPI1          :1;
  REG32 /**/            :1;
  REG32 PCAD            :1;
  REG32 PCCAN1          :1;
  REG32 PCCAN2          :1;
  REG32 PCCAN3          :1;
  REG32 PCCAN4          :1;
  REG32 /**/            :4;
  REG32 PCSSP           :1;
  REG32 /**/            :10;
  }	bit_pconp;
  REG32	reg_pconp;
} pconp_bits;

/* VPB divider register */
typedef union{
struct {
  REG32 VPBDIV          :2;
  REG32 /**/            :30;
  }	bit_vpbdiv;
  REG32	reg_vpbdiv;
} vpbdiv_bits;

/* Memory accelerator module control register */
typedef union{
struct {
  REG32 MODECTRL        :2;
  REG32 /**/            :30;
  }	bit_mamcr;
  REG32	reg_mamcr;
} mamcr_bits;

/* Memory accelerator module timing register */
typedef union{
struct {
  REG32 CYCLES          :3;
  REG32 /**/            :29;
  }	bit_mamtim;
  REG32	reg_mamtim;
} mamtim_bits;


/* VIC Interrupt registers */
typedef union{
struct {
  REG32 INT0            :1;
  REG32 INT1            :1;
  REG32 INT2            :1;
  REG32 INT3            :1;
  REG32 INT4            :1;
  REG32 INT5            :1;
  REG32 INT6            :1;
  REG32 INT7            :1;
  REG32 INT8            :1;
  REG32 INT9            :1;
  REG32 INT10           :1;
  REG32 INT11           :1;
  REG32 INT12           :1;
  REG32 INT13           :1;
  REG32 INT14           :1;
  REG32 INT15           :1;
  REG32 INT16           :1;
  REG32 INT17           :1;
  REG32 INT18           :1;
  REG32 INT19           :1;
  REG32 INT20           :1;
  REG32 INT21           :1;
  REG32 INT22           :1;
  REG32 INT23           :1;
  REG32 INT24           :1;
  REG32 INT25           :1;
  REG32 INT26           :1;
  REG32 INT27           :1;
  REG32 INT28           :1;
  REG32 INT29           :1;
  REG32 INT30           :1;
  REG32 INT31           :1;
  }	bit_vicint;
  REG32	reg_vicint;
} vicint_bits1,vicint_bits2,vicint_bits3,vicint_bits4,vicint_bits5,vicint_bits6,vicint_bits7,vicint_bits8;

/* VIC Vector control registers */
typedef union{
struct {
  REG32 NUMBER          :5;
  REG32 ENABLED         :1;
  REG32 /**/            :26;
  }	bit_vicvectcntl;
  REG32	reg_vicvectcntl;
} vicvectcntl_bits0,vicvectcntl_bits1,vicvectcntl_bits2,vicvectcntl_bits3,vicvectcntl_bits4,vicvectcntl_bits5,vicvectcntl_bits6,vicvectcntl_bits7,vicvectcntl_bits8,vicvectcntl_bits9,vicvectcntl_bits10,vicvectcntl_bits11,vicvectcntl_bits12,vicvectcntl_bits13,vicvectcntl_bits14,vicvectcntl_bits15;

/* VIC protection enable register */
typedef union{
struct {
  REG32 PROTECT         :1;
  REG32 /**/            :31;
  }	bit_vicprotection;
  REG32	reg_vicprotection;
} vicprotection_bits;



/* Pin function select register 0 */
typedef union{
struct {
  REG32 P0_0            :2;
  REG32 P0_1            :2;
  REG32 P0_2            :2;
  REG32 P0_3            :2;
  REG32 P0_4            :2;
  REG32 P0_5            :2;
  REG32 P0_6            :2;
  REG32 P0_7            :2;
  REG32 P0_8            :2;
  REG32 P0_9            :2;
  REG32 P0_10           :2;
  REG32 P0_11           :2;
  REG32 P0_12           :2;
  REG32 P0_13           :2;
  REG32 P0_14           :2;
  REG32 P0_15           :2;	
  }	bit_pinsel0;
  REG32	reg_pinsel0;
} pinsel0_bits;

/* Pin function select register 1 */
typedef union{
struct {
  REG32 P0_16           :2;
  REG32 P0_17           :2;
  REG32 P0_18           :2;
  REG32 P0_19           :2;
  REG32 P0_20           :2;
  REG32 P0_21           :2;
  REG32 P0_22           :2;
  REG32 P0_23           :2;
  REG32 P0_24           :2;
  REG32 P0_25           :2;
  REG32 P0_26           :2;
  REG32 P0_27           :2;
  REG32 P0_28           :2;
  REG32 P0_29           :2;
  REG32 P0_30           :2;
  REG32 P0_31           :2;
  }	bit_pinsel1;
  REG32	reg_pinsel1;
} pinsel1_bits;


/* GPIO register 0 */
typedef union{
struct {
  REG32 P0_0            :1;
  REG32 P0_1            :1;
  REG32 P0_2            :1;
  REG32 P0_3            :1;
  REG32 P0_4            :1;
  REG32 P0_5            :1;
  REG32 P0_6            :1;
  REG32 P0_7            :1;
  REG32 P0_8            :1;
  REG32 P0_9            :1;
  REG32 P0_10           :1;
  REG32 P0_11           :1;
  REG32 P0_12           :1;
  REG32 P0_13           :1;
  REG32 P0_14           :1;
  REG32 P0_15           :1;
  REG32 P0_16           :1;
  REG32 P0_17           :1;
  REG32 P0_18           :1;
  REG32 P0_19           :1;
  REG32 P0_20           :1;
  REG32 P0_21           :1;
  REG32 P0_22           :1;
  REG32 P0_23           :1;
  REG32 P0_24           :1;
  REG32 P0_25           :1;
  REG32 P0_26           :1;
  REG32 P0_27           :1;
  REG32 P0_28           :1;
  REG32 P0_29           :1;
  REG32 P0_30           :1;
  REG32 P0_31           :1;
  }	bit_gpio0;
  REG32	reg_gpio0;
} gpio0_bits1,gpio0_bits2,gpio0_bits3,gpio0_bits4;

/* FGPIO 0 Registers*/
typedef union{
union {
  struct{
    REG32 P0_0   : 1;
    REG32 P0_1   : 1;
    REG32 P0_2   : 1;
    REG32 P0_3   : 1;
    REG32 P0_4   : 1;
    REG32 P0_5   : 1;
    REG32 P0_6   : 1;
    REG32 P0_7   : 1;
    REG32 P0_8   : 1;
    REG32 P0_9   : 1;
    REG32 P0_10  : 1;
    REG32 P0_11  : 1;
    REG32 P0_12  : 1;
    REG32 P0_13  : 1;
    REG32 P0_14  : 1;
    REG32 P0_15  : 1;
    REG32 P0_16  : 1;
    REG32 P0_17  : 1;
    REG32 P0_18  : 1;
    REG32 P0_19  : 1;
    REG32 P0_20  : 1;
    REG32 P0_21  : 1;
    REG32 P0_22  : 1;
    REG32 P0_23  : 1;
    REG32 P0_24  : 1;
    REG32 P0_25  : 1;
    REG32 P0_26  : 1;
    REG32 P0_27  : 1;
    REG32 P0_28  : 1;
    REG32 P0_29  : 1;
    REG32 P0_30  : 1;
    REG32 P0_31  : 1;
  } addr0_w;

  struct
  {
    union
    {
      struct{
        REG8  P0_0   : 1;
        REG8  P0_1   : 1;
        REG8  P0_2   : 1;
        REG8  P0_3   : 1;
        REG8  P0_4   : 1;
        REG8  P0_5   : 1;
        REG8  P0_6   : 1;
        REG8  P0_7   : 1;
      } byte0_bit;
      REG8 byte0;
    } byte_0;
    union
    {
      struct{
        REG8  P0_8    : 1;
        REG8  P0_9    : 1;
        REG8  P0_10   : 1;
        REG8  P0_11   : 1;
        REG8  P0_12   : 1;
        REG8  P0_13   : 1;
        REG8  P0_14   : 1;
        REG8  P0_15   : 1;
      } byte1_bit;
      REG8 byte1;
    } byte_1;
    union
    {
      struct{
        REG8  P0_16   : 1;
        REG8  P0_17   : 1;
        REG8  P0_18   : 1;
        REG8  P0_19   : 1;
        REG8  P0_20   : 1;
        REG8  P0_21   : 1;
        REG8  P0_22   : 1;
        REG8  P0_23   : 1;
      } byte2_bit;
      REG8 byte2;
    } byte_2;
    union
    {
      struct{
        REG8  P0_24   : 1;
        REG8  P0_25   : 1;
        REG8  P0_26   : 1;
        REG8  P0_27   : 1;
        REG8  P0_28   : 1;
        REG8  P0_29   : 1;
        REG8  P0_30   : 1;
        REG8  P0_31   : 1;
      } byte3_bit;
      REG8 byte3;
    } byte_3;
  } addr0_b;

  struct
  {
    union
    {
      struct{
        REG16 P0_0   : 1;
        REG16 P0_1   : 1;
        REG16 P0_2   : 1;
        REG16 P0_3   : 1;
        REG16 P0_4   : 1;
        REG16 P0_5   : 1;
        REG16 P0_6   : 1;
        REG16 P0_7   : 1;
        REG16 P0_8   : 1;
        REG16 P0_9   : 1;
        REG16 P0_10  : 1;
        REG16 P0_11  : 1;
        REG16 P0_12  : 1;
        REG16 P0_13  : 1;
        REG16 P0_14  : 1;
        REG16 P0_15  : 1;
      } shortl_bit;
      REG16 shortl;
    } hw0;
    union
    {
      struct{
        REG16 P0_16   : 1;
        REG16 P0_17   : 1;
        REG16 P0_18   : 1;
        REG16 P0_19   : 1;
        REG16 P0_20   : 1;
        REG16 P0_21   : 1;
        REG16 P0_22   : 1;
        REG16 P0_23   : 1;
        REG16 P0_24   : 1;
        REG16 P0_25   : 1;
        REG16 P0_26   : 1;
        REG16 P0_27   : 1;
        REG16 P0_28   : 1;
        REG16 P0_29   : 1;
        REG16 P0_30   : 1;
        REG16 P0_31   : 1;
      } shortu_bit;
      REG16 shortu;
    } hw1;
  } addr0_hw;
 } bit_fgpio0;
 REG32 reg_fgpio0;
}fgpio0_bits1,fgpio0_bits2,fgpio0_bits3,fgpio0_bits4,fgpio0_bits5;

/* GPIO register 1 */
typedef union{
struct {
  REG32 P1_0            :1;
  REG32 P1_1            :1;
  REG32 P1_2            :1;
  REG32 P1_3            :1;
  REG32 P1_4            :1;
  REG32 P1_5            :1;
  REG32 P1_6            :1;
  REG32 P1_7            :1;
  REG32 P1_8            :1;
  REG32 P1_9            :1;
  REG32 P1_10           :1;
  REG32 P1_11           :1;
  REG32 P1_12           :1;
  REG32 P1_13           :1;
  REG32 P1_14           :1;
  REG32 P1_15           :1;
  REG32 P1_16           :1;
  REG32 P1_17           :1;
  REG32 P1_18           :1;
  REG32 P1_19           :1;
  REG32 P1_20           :1;
  REG32 P1_21           :1;
  REG32 P1_22           :1;
  REG32 P1_23           :1;
  REG32 P1_24           :1;
  REG32 P1_25           :1;
  REG32 P1_26           :1;
  REG32 P1_27           :1;
  REG32 P1_28           :1;
  REG32 P1_29           :1;
  REG32 P1_30           :1;
  REG32 P1_31           :1;
  }	bit_gpio1;
  REG32	reg_gpio1;
} gpio1_bits1,gpio1_bits2,gpio1_bits3,gpio1_bits4;

/* FGPIO 1 Registers*/
typedef union{
union{
  struct {
    REG32 P1_0   : 1;
    REG32 P1_1   : 1;
    REG32 P1_2   : 1;
    REG32 P1_3   : 1;
    REG32 P1_4   : 1;
    REG32 P1_5   : 1;
    REG32 P1_6   : 1;
    REG32 P1_7   : 1;
    REG32 P1_8   : 1;
    REG32 P1_9   : 1;
    REG32 P1_10  : 1;
    REG32 P1_11  : 1;
    REG32 P1_12  : 1;
    REG32 P1_13  : 1;
    REG32 P1_14  : 1;
    REG32 P1_15  : 1;
    REG32 P1_16  : 1;
    REG32 P1_17  : 1;
    REG32 P1_18  : 1;
    REG32 P1_19  : 1;
    REG32 P1_20  : 1;
    REG32 P1_21  : 1;
    REG32 P1_22  : 1;
    REG32 P1_23  : 1;
    REG32 P1_24  : 1;
    REG32 P1_25  : 1;
    REG32 P1_26  : 1;
    REG32 P1_27  : 1;
    REG32 P1_28  : 1;
    REG32 P1_29  : 1;
    REG32 P1_30  : 1;
    REG32 P1_31  : 1;
  } addr1_w;

  struct
  {
    union
    {
      struct{
        REG8  P1_0   : 1;
        REG8  P1_1   : 1;
        REG8  P1_2   : 1;
        REG8  P1_3   : 1;
        REG8  P1_4   : 1;
        REG8  P1_5   : 1;
        REG8  P1_6   : 1;
        REG8  P1_7   : 1;
      } byte0_bit;
      REG8 byte0;
    } byte_0;
    union
    {
      struct{
        REG8  P1_8    : 1;
        REG8  P1_9    : 1;
        REG8  P1_10   : 1;
        REG8  P1_11   : 1;
        REG8  P1_12   : 1;
        REG8  P1_13   : 1;
        REG8  P1_14   : 1;
        REG8  P1_15   : 1;
      } byte1_bit;
      REG8 byte1;
    } byte_1;
    union
    {
      struct{
        REG8  P1_16   : 1;
        REG8  P1_17   : 1;
        REG8  P1_18   : 1;
        REG8  P1_19   : 1;
        REG8  P1_20   : 1;
        REG8  P1_21   : 1;
        REG8  P1_22   : 1;
        REG8  P1_23   : 1;
      } byte2_bit;
      REG8 byte2;
    } byte_2;
    union
    {
      struct{
        REG8  P1_24   : 1;
        REG8  P1_25   : 1;
        REG8  P1_26   : 1;
        REG8  P1_27   : 1;
        REG8  P1_28   : 1;
        REG8  P1_29   : 1;
        REG8  P1_30   : 1;
        REG8  P1_31   : 1;
      } byte3_bit;
      REG8 byte3;
    } byte_3;
  } addr1_b;

  struct
  {
    union
    {
      struct{
        REG16 P1_0   : 1;
        REG16 P1_1   : 1;
        REG16 P1_2   : 1;
        REG16 P1_3   : 1;
        REG16 P1_4   : 1;
        REG16 P1_5   : 1;
        REG16 P1_6   : 1;
        REG16 P1_7   : 1;
        REG16 P1_8   : 1;
        REG16 P1_9   : 1;
        REG16 P1_10  : 1;
        REG16 P1_11  : 1;
        REG16 P1_12  : 1;
        REG16 P1_13  : 1;
        REG16 P1_14  : 1;
        REG16 P1_15  : 1;
      } shortl_bit;
      REG16 shortl;
    } hw0;
    union
    {
      struct{
        REG16 P1_16   : 1;
        REG16 P1_17   : 1;
        REG16 P1_18   : 1;
        REG16 P1_19   : 1;
        REG16 P1_20   : 1;
        REG16 P1_21   : 1;
        REG16 P1_22   : 1;
        REG16 P1_23   : 1;
        REG16 P1_24   : 1;
        REG16 P1_25   : 1;
        REG16 P1_26   : 1;
        REG16 P1_27   : 1;
        REG16 P1_28   : 1;
        REG16 P1_29   : 1;
        REG16 P1_30   : 1;
        REG16 P1_31   : 1;
      } shortu_bit;
      REG16 shortu;
    } hw1;
  } addr1_hw;
 } bit_fgpio1;
 REG32 reg_fgpio1;
} fgpio1_bits1,fgpio1_bits2,fgpio1_bits3,fgpio1_bits4,fgpio1_bits5;

/* UART0 interrupt enable register */
typedef union{
struct {
  REG32 RDAIE        :1;
  REG32 THREIE       :1;
  REG32 RXLSIE       :1;
  REG32 /**/         :5;
  REG32 ABTOIntEn    :1;     
  REG32 ABEOintEn    :1;     
  REG32 /**/         :22;
  }	bit_uartier0;
  REG32	reg_uartier0;
} uartier0_bits;


/* UART1 interrupt enable register */
typedef union{
struct{
REG32 RDAIE      : 1;     
REG32 THREIE     : 1;     
REG32 RXLSIE     : 1;     
REG32 MSIE       : 1;     
REG32 /**/       : 3;     
REG32 CTSIE      : 1;     
REG32 ABTOIntEn  : 1;     
REG32 ABEOintEn  : 1;     
REG32 /**/       :22; 
  }	bit_uartier1;
  REG32	reg_uartier1;   
} uartier1_bits;

/* UART interrupt identification register and fifo control register */
typedef union{
union{
  //UxIIR
  struct {
REG32 IP       : 1;     
REG32 IID      : 3;     
REG32 /**/     : 2;     
REG32 IIRFE    : 2;     
REG32 ABEOInt  : 1;     
REG32 ABTOInt  : 1;     
REG32 /**/     :22;     
  } iir;                        
  //UxFCR                   
  struct {                  
REG32 FCRFE    : 1;     
REG32 RFR      : 1;     
REG32 TFR      : 1;     
REG32 /**/     : 3;     
REG32 RTLS     : 2;     
REG32 /**/     :24;     
  } fcr;		   
 }bit_uartfcriir;
  REG32	reg_uartfcriir;
} uartfcriir_bits0,uartfcriir_bits1;

/* UART line control register */
typedef union{
struct {
  REG8 WLS              :2;
  REG8 SBS              :1;
  REG8 PE               :1;
  REG8 PS               :2;
  REG8 BC               :1;
  REG8 DLAB             :1;	 
  }	bit_uartlcr;
  REG8	reg_uartlcr;
} uartlcr_bits0, uartlcr_bits1;

/* UART modem control register */
typedef union{
struct {
  REG8 DTR              :1;
  REG8 RTS              :1;
  REG8 /**/             :2;
  REG8 LMS              :1;
  REG8 /**/             :1;     
  REG8  RTSen           :1;     
  REG8  CTSen           :1;  
  }	bit_uartmcr;
  REG8	reg_uartmcr;
} uartmcr_bits;

/* UART line status register */
typedef union{
struct {
  REG8 DR               :1;
  REG8 OE               :1;
  REG8 PE               :1;
  REG8 FE               :1;
  REG8 BI               :1;
  REG8 THRE             :1;
  REG8 TEMT             :1;
  REG8 RXFE             :1;	 
  }	bit_uartlsr;
  REG8	reg_uartlsr;
} uartlsr_bits0,uartlsr_bits1;

/* UART modem status register */
typedef union{
//union {
  //U1MSR
  struct {
  REG8 DCTS             :1;
  REG8 DDSR             :1;
  REG8 TERI             :1;
  REG8 DDCD             :1;
  REG8 CTS              :1;
  REG8 DSR              :1;
  REG8 RI               :1;
  REG8 DCD              :1;
  }	bit_uartmsr;
  REG32	reg_uartmsr;
  } uartmsr_bits;
 /* //U1MSR ---- Not Used
  struct {
  REG8 MSR0             :1;
  REG8 MSR1             :1;
  REG8 MSR2             :1;
  REG8 MSR3             :1;
  REG8 MSR4             :1;
  REG8 MSR5             :1;
  REG8 MSR6             :1;
  REG8 MSR7             :1;	
  } bit_uartmsr;
  REG32 reg_uartmsr;
  }; 
} uartmsr_bits;	  */

/* UART Auto-baud Control Register */
typedef union{
struct{
REG32 ENA         : 1;     
REG32 MODE        : 1;     
REG32 RESTART     : 1;     
REG32 /**/        : 5;     
REG32 ABEOIntClr  : 1;     
REG32 ABTOIntClr  : 1;     
REG32 /**/        :22;    
  }	bit_uartacr;
  REG32	reg_uartacr;
} uartacr_bits0,uartacr_bits1;

/* UART Fractional Divider Register */
typedef union{
struct{
REG32 DivAddVal  : 4;     
REG32 MulVal     : 4;     
REG32 /**/       :24;     
  }	bit_uartfdr;
  REG32	reg_uartfdr;
} uartfdr_bits0,uartfdr_bits1;

/* UART Transmit Enable Register */
typedef union{
struct{
REG8  /**/   : 7;     
REG8  TXENA  : 1;      
  }	bit_uartter;
  REG32	reg_uartter;
} uartter_bits0,uartter_bits1;

/* I2C control set register */
typedef union{
struct {
  REG32 /**/            :2;
  REG32 AA              :1;
  REG32 SI              :1;
  REG32 STO             :1;
  REG32 STA             :1;
  REG32 I2EN            :1;
  REG32 /**/            :25; 
  }	bit_i2conset;
  REG32	reg_i2conset;
} i2conset_bits;

/* I2C control clear register */
typedef union{
struct {
  REG32 /**/            :2;
  REG32 AAC             :1;
  REG32 SIC             :1;
  REG32 /**/            :1;
  REG32 STAC            :1;
  REG32 I2ENC           :1;
  REG32 /**/            :25; 
  }	bit_i2conclr;
  REG32	reg_i2conclr;
} i2conclr_bits;

/* I2C status register */
typedef union{
struct {
  REG32 STATUS          :8;
  REG32 /**/            :24; 
  }	bit_i2stat;
  REG32	reg_i2stat;
} i2stat_bits;

/* I2C data register */
typedef union{
struct {
  REG32 DATA            :8;
  REG32 /**/            :24; 
  }	bit_i2dat;
  REG32	reg_i2dat;
} i2dat_bits;

/* I2C slave address register */
typedef union{
struct {
  REG32 GC              :1;
  REG32 ADDR            :7;
  REG32 /**/            :24; 
  }	bit_i2adr;
  REG32	reg_i2adr;
} i2adr_bits;

/* I2C scl duty cycle register */
typedef union{
struct {
  REG32 COUNT           :16;
  REG32 /**/            :16;
  }	bit_i2scl;
  REG32	reg_i2scl;
} i2scl_bits1,i2scl_bits2;


/* SPI control register */
typedef union{
struct {
  REG32 /**/            :3;
  REG32 CPHA            :1;
  REG32 CPOL            :1;
  REG32 MSTR            :1;
  REG32 LSBF            :1;
  REG32 SPIE            :1;
  REG32 /**/            :24; 
  }	bit_spcr;
  REG32	reg_spcr;
} spcr_bits0,spcr_bits1;

/* SPI status register */
typedef union{
struct {
  REG32 /**/            :3;
  REG32 ABRT            :1;
  REG32 MODF            :1;
  REG32 ROVR            :1;
  REG32 WCOL            :1;
  REG32 SPIF            :1;
  REG32 /**/            :24; 
  }	bit_spsr;
  REG32	reg_spsr;
} spsr_bits0,spsr_bits1;

/* SPI data register */
typedef union{
struct {
  REG32 DATA            :8;
  REG32 /**/            :24;
  }	bit_spdr;
  REG32 reg_spdr;
} spdr_bits0,spdr_bits1;

/* SPI clock counter register */
typedef union{
struct {
  REG32 COUNTER         :8;
  REG32 /**/            :24;
  }	bit_spccr;
  REG32	reg_spccr;
} spccr_bits0,spccr_bits1;

/* SPI interrupt register */
typedef union{
struct {
  REG32 SPIINT          :1;
  REG32 /**/            :31; 
  }	bit_spint;
  REG32	reg_spint;
} spint_bits0,spint_bits1;


/* SSP Control Register 0 */
typedef union{
struct{
REG32 DSS   : 4;     
REG32 FRF   : 2;     
REG32 CPOL  : 1;     
REG32 CPHA  : 1;     
REG32 SCR   : 8;     
REG32 /**/  :16;   
  }	bit_sspcr0;
  REG32	reg_sspcr0;
} sspcr0_bits;

/* SSP Control Register 1 */
typedef union{
struct{
REG32 LBM  : 1;     
REG32 SSE  : 1;     
REG32 MS   : 1;     
REG32 SOD  : 1;     
REG32 /**/ :28;  
  }	bit_sspcr1;
  REG32	reg_sspcr1; 
} sspcr1_bits;

/* SSP Data Register */
typedef union{
struct{
REG32 DATA  :16;     
REG32 /**/  :16; 
  }	bit_sspdr;
  REG32	reg_sspdr;    
} sspdr_bits;

/* SSP Status Register */
typedef union{
struct{
REG32 TFE  : 1;     
REG32 TNF  : 1;     
REG32 RNE  : 1;     
REG32 RFF  : 1;     
REG32 BSY  : 1;     
REG32 /**/ :27;  
  }	bit_sspsr;
  REG32	reg_sspsr;  
} sspsr_bits;

/* SSP Clock Prescale Register */
typedef union{
struct{
REG32 CPSDVSR  : 8;     
REG32 /**/     :24; 
  }	bit_sspcpsr;
  REG32	reg_sspcpsr;   
} sspcpsr_bits;

/* SSP Interrupt Mask Set/Clear Register */
typedef union{
struct{
REG32 RORIM  : 1;     
REG32 RTIM   : 1;     
REG32 RXIM   : 1;     
REG32 TXIM   : 1;     
REG32 /**/   :28;  
  }	bit_sspimsc;
  REG32	reg_sspimsc; 
} sspimsc_bits;

/* SSP Raw Interrupt Status Register */
typedef union{
struct{
REG32 RORRIS  : 1;     
REG32 RTRIS   : 1;     
REG32 RXRIS   : 1;     
REG32 TXRIS   : 1;     
REG32 /**/    :28; 
  }	bit_sspris;
  REG32	reg_sspris;    
} sspris_bits;

/* SSP Masked Interrupt Status Register */
typedef union{
struct{
REG32 RORMIS  : 1;     
REG32 RTMIS   : 1;     
REG32 RXMIS   : 1;     
REG32 TXMIS   : 1;     
REG32 /**/    :28;
  }	bit_sspmis;
  REG32	reg_sspmis;    
} sspmis_bits;

/* SSP Interrupt Clear Register */
typedef union{
struct{
REG32 RORIC  : 1;     
REG32 RTIC   : 1;     
REG32 /**/   :30;
  }	bit_sspicr;
  REG32	reg_sspicr;    
} sspicr_bits;

/* CAN acceptance filter mode register */
typedef union{
struct {
  REG32 AccOff          :1;
  REG32 AccBP           :1;
  REG32 eFCAN           :1;
  REG32 /**/            :29; 
  }	bit_afmr;
  REG32	reg_afmr;
} afmr_bits;


/* CAN central transmit status register */
typedef union{
struct {
  REG32 TS              :4;
  REG32 /**/            :4;
  REG32 TBS             :4;
  REG32 /**/            :4;
  REG32 TCS             :4;
  REG32 /**/            :12; 
  }	bit_cantxsr;
  REG32	reg_cantxsr;
} cantxsr_bits;


/* CAN central receive status register */
typedef union{
struct {
  REG32 RS              :4;
  REG32 /**/            :4;
  REG32 RBS             :4;
  REG32 /**/            :4;
  REG32 DOS             :4;
  REG32 /**/            :12;
  }	bit_canrxsr;
  REG32	reg_canrxsr;
} canrxsr_bits;


/* CAN miscellaneous status register */
typedef union{
struct {
  REG32 ES              :4;
  REG32 /**/            :4;
  REG32 BS              :4;
  REG32 /**/            :20;
  }	bit_canmsr;
  REG32	reg_canmsr;
} canmsr_bits;


/* CAN mode register */
typedef union{
struct {
  REG32 RM              :1;
  REG32 LOM             :1;
  REG32 STM             :1;
  REG32 TPM             :1;
  REG32 SM              :1;
  REG32 RPM             :1;
  REG32 /**/            :1;
  REG32 TM              :1;
  REG32 /**/            :24; 
  }	bit_canmod;
  REG32	reg_canmod;
} canmod_bits1,canmod_bits2,canmod_bits3,canmod_bits4;


/* CAN command register */
typedef union{
struct {
  REG32 TR              :1;
  REG32 AT              :1;
  REG32 RRB             :1;
  REG32 CDO             :1;
  REG32 SRR             :1;
  REG32 STB1            :1;
  REG32 STB2            :1;
  REG32 STB3            :1;
  REG32 /**/            :24; 
  }	bit_cancmr;
  REG32	reg_cancmr;
} cancmr_bits1,cancmr_bits2,cancmr_bits3,cancmr_bits4;


/* CAN global status register */
typedef union{
struct {
  REG32 RBS              :1;
  REG32 DOS              :1;
  REG32 TBS              :1;
  REG32 TCS              :1;
  REG32 RS               :1;
  REG32 TS               :1;
  REG32 ES               :1;
  REG32 BS               :1;
  REG32 /**/             :8;
  REG32 RXERR            :8;
  REG32 TXERR            :8; 
  }	bit_cangsr;
  REG32	reg_cangsr;
} cangsr_bits1,cangsr_bits2,cangsr_bits3,cangsr_bits4;


/* CAN interrupt capture register */
typedef union{
struct {
  REG32 RI               :1;
  REG32 TI1              :1;
  REG32 EI               :1;
  REG32 DOI              :1;
  REG32 WUI              :1;
  REG32 EPI              :1;
  REG32 ALI              :1;
  REG32 BEI              :1;
  REG32 IDI              :1;
  REG32 TI2              :1;
  REG32 TI3              :1;
  REG32 /**/             :5;
  REG32 ERRBIT           :5;
  REG32 ERRDIR           :1;
  REG32 ERRC             :2;
  REG32 ALCBIT           :5;
  REG32 /**/             :3; 
  }	bit_canicr;
  REG32	reg_canicr;
} canicr_bits1,canicr_bits2,canicr_bits3,canicr_bits4;


/* CAN interrupt enable register */
typedef union{
struct {
  REG32 RIE               :1;
  REG32 TIE1              :1;
  REG32 EIE               :1;
  REG32 DOIE              :1;
  REG32 WUIE              :1;
  REG32 EPIE              :1;
  REG32 ALIE              :1;
  REG32 BEIE              :1;
  REG32 IDIE              :1;
  REG32 TIE2              :1;
  REG32 TIE3              :1;
  REG32 /**/              :21;
  }	bit_canier;
  REG32	reg_canier;
} canier_bits1,canier_bits2,canier_bits3,canier_bits4;


/* CAN bus timing register */
typedef union{
struct {
  REG32 BRP                :10;
  REG32 /**/               :4;
  REG32 SJW                :2;
  REG32 TSEG1              :4;
  REG32 TSEG2              :3;
  REG32 SAM                :1;
  REG32 /**/               :8;
  }	bit_canbtr;
  REG32	reg_canbtr;
} canbtr_bits1,canbtr_bits2,canbtr_bits3,canbtr_bits4;


/* CAN error warning limit register */
typedef union{
struct {
  REG32 EWL                :8;
  REG32 /**/               :24;
  }	bit_canewl;
  REG32	reg_canewl;
} canewl_bits1,canewl_bits2,canewl_bits3,canewl_bits4;


/* CAN status register */
typedef union{
struct {
  REG32 RBS                :1;
  REG32 DOS                :1;
  REG32 TBS1               :1;
  REG32 TCS1               :1;
  REG32 RS                 :1;
  REG32 TS1                :1;
  REG32 ES                 :1;
  REG32 BS                 :1;
  REG32 /*RBS*/            :1;
  REG32 /*DOS*/            :1;
  REG32 TBS2               :1;
  REG32 TCS2               :1;
  REG32 /*RS*/             :1;
  REG32 TS2                :1;
  REG32 /*ES*/             :1;
  REG32 /*BS*/             :1;
  REG32 /*RBS*/            :1;
  REG32 /*DOS*/            :1;
  REG32 TBS3               :1;
  REG32 TCS3               :1;
  REG32 /*RS*/             :1;
  REG32 TS3                :1;
  REG32 /*ES*/             :1;
  REG32 /*BS*/             :1;
  REG32 /**/               :8; 
  }	bit_cansr;
  REG32	reg_cansr;
} cansr_bits1,cansr_bits2,cansr_bits3,cansr_bits4;


/* CAN rx frame status register */
typedef union{
struct {
  REG32 IDIndex            :10;
  REG32 BP                 :1;
  REG32 /**/               :5;
  REG32 DLC                :4;
  REG32 /**/               :10;
  REG32 RTR                :1;
  REG32 FF                 :1; 
  }	bit_canrfs;
  REG32	reg_canrfs;
} canrfs_bits1,canrfs_bits2,canrfs_bits3,canrfs_bits4;


/* CAN rx identifier register */
typedef union{
union {
  //CxRID
  struct {
   REG32 ID10_0             :11;
   REG32 /**/               :21;
  }ID10;
  //CxRID
  struct {
   REG32 ID29_18            :11;
   REG32 /**/               :21;
  }ID2918;
  //CxRID
  struct {
   REG32 ID29_0             :29;
   REG32 /**/               :3;
  }ID290;						
  }	bit_canrid;
  REG32	reg_canrid;
} canrid_bits1,canrid_bits2,canrid_bits3,canrid_bits4;


/* CAN rx data register A */
typedef union{
struct {
  REG32 Data1               :8;
  REG32 Data2               :8;
  REG32 Data3               :8;
  REG32 Data4               :8;	   
  }	bit_canrda;
  REG32	reg_canrda;
} canrda_bits1,canrda_bits2,canrda_bits3,canrda_bits4;


/* CAN rx data register B */
typedef union{
struct {
  REG32 Data5               :8;
  REG32 Data6               :8;
  REG32 Data7               :8;
  REG32 Data8               :8;	 
  }	bit_canrdb;
  REG32	reg_canrdb;
} canrdb_bits1,canrdb_bits2,canrdb_bits3,canrdb_bits4;


/* CAN tx frame information register */
typedef union{
struct {
  REG32 PRIO              :8;
  REG32 /**/              :8;
  REG32 DLC               :4;
  REG32 /**/              :10;
  REG32 RTR               :1;
  REG32 FF                :1; 
  }	bit_cantfi;
  REG32	reg_cantfi;
} cantfi_bits11,cantfi_bits12,cantfi_bits13,cantfi_bits21,cantfi_bits22,cantfi_bits23,cantfi_bits31,cantfi_bits32,cantfi_bits33,cantfi_bits41,cantfi_bits42,cantfi_bits43;


/* CAN tx identifier register */
typedef union{
union {
  //CxTIDy
  struct {
   REG32 ID10_0             :11;
   REG32 /**/               :21;
  }ID10;
  //CxTIDy
  struct {
   REG32 ID29_18            :11;
   REG32 /**/               :21;
  }ID2918;
  //CxTIDy
  struct {
   REG32 ID29_0             :29;
   REG32 /**/               :3;
  }ID290;					  
  }	bit_cantid;
  REG32	reg_cantid;
} cantid_bits11,cantid_bits12,cantid_bits13,cantid_bits21,cantid_bits22,cantid_bits23,cantid_bits31,cantid_bits32,cantid_bits33,cantid_bits41,cantid_bits42,cantid_bits43;


/* CAN tx data register A */
typedef union{
struct {
  REG32 Data1               :8;
  REG32 Data2               :8;
  REG32 Data3               :8;
  REG32 Data4               :8;	   
  }	bit_cantda;
  REG32	reg_cantda;
} cantda_bits11,cantda_bits12,cantda_bits13,cantda_bits21,cantda_bits22,cantda_bits23,cantda_bits31,cantda_bits32,cantda_bits33,cantda_bits41,cantda_bits42,cantda_bits43;


/* CAN tx data register B */
typedef union{
struct {
  REG32 Data5               :8;
  REG32 Data6               :8;
  REG32 Data7               :8;
  REG32 Data8               :8;	
  }	bit_cantdb;
  REG32	reg_cantdb;
} cantdb_bits11,cantdb_bits12,cantdb_bits13,cantdb_bits21,cantdb_bits22,cantdb_bits23,cantdb_bits31,cantdb_bits32,cantdb_bits33,cantdb_bits41,cantdb_bits42,cantdb_bits43;


/* TIMER interrupt register */
typedef union{
struct {
  REG32 MR0INT          :1;
  REG32 MR1INT          :1;
  REG32 MR2INT          :1;
  REG32 MR3INT          :1;
  REG32 CR0INT          :1;
  REG32 CR1INT          :1;
  REG32 CR2INT          :1;
  REG32 CR3INT          :1;
  REG32 /**/            :24;   
  }	bit_ir;
  REG32	reg_ir;
} ir_bits0,ir_bits1;

/* TIMER control register */
typedef union{
struct {
  REG32 CE              :1;
  REG32 CR              :1;
  REG32 /**/            :30; 
  }	bit_tcr;
  REG32	reg_tcr;
} tcr_bits0,tcr_bits1;

/* Count Control Register */
typedef union{
struct{
REG32 TIMER_MODE  : 2;     
REG32 INPUT_SEL   : 2;     
REG32 /**/        :28;     
  }	bit_ctcr;
  REG32	reg_ctcr;
} ctcr_bits0,ctcr_bits1;

/* TIMER match control register */
typedef union{
struct {
  REG32 MR0INT          :1;
  REG32 MR0RES          :1;
  REG32 MR0STOP         :1;
  REG32 MR1INT          :1;
  REG32 MR1RES          :1;
  REG32 MR1STOP         :1;
  REG32 MR2INT          :1;
  REG32 MR2RES          :1;
  REG32 MR2STOP         :1;
  REG32 MR3INT          :1;
  REG32 MR3RES          :1;
  REG32 MR3STOP         :1;
  REG32 /**/            :20;   
  }	bit_mcr;
  REG32	reg_mcr;
} mcr_bits0,mcr_bits1;

/* TIMER0 capture control register */
typedef union{
struct {
  REG32 CAP0RE          :1;
  REG32 CAP0FE          :1;
  REG32 CAP0INT         :1;
  REG32 CAP1RE          :1;
  REG32 CAP1FE          :1;
  REG32 CAP1INT         :1;
  REG32 CAP2RE          :1;
  REG32 CAP2FE          :1;
  REG32 CAP2INT         :1;
  REG32 /**/            :23;  
  }	bit_ccr0;
  REG32	reg_ccr0;
} ccr0_bits;

/* TIMER1 capture control register */
typedef union{
struct {
  REG32 CAP0RE          :1;
  REG32 CAP0FE          :1;
  REG32 CAP0INT         :1;
  REG32 CAP1RE          :1;
  REG32 CAP1FE          :1;
  REG32 CAP1INT         :1;
  REG32 CAP2RE          :1;
  REG32 CAP2FE          :1;
  REG32 CAP2INT         :1;
  REG32 CAP3RE          :1;
  REG32 CAP3FE          :1;
  REG32 CAP3INT         :1;
  REG32 /**/            :20;   
  }	bit_ccr1;
  REG32	reg_ccr1;
} ccr1_bits;

/* TIMER external match register */
typedef union{
struct {
  REG32 EM0             :1;
  REG32 EM1             :1;
  REG32 EM2             :1;
  REG32 EM3             :1;
  REG32 EMC0            :2;
  REG32 EMC1            :2;
  REG32 EMC2            :2;
  REG32 EMC3            :2;
  REG32 /**/            :20;   
  }	bit_emr;
  REG32	reg_emr;
} emr_bits0,emr_bits1;


/* PWM interrupt register */
typedef union{
struct {
  REG32 MR0INT          :1;
  REG32 MR1INT          :1;
  REG32 MR2INT          :1;
  REG32 MR3INT          :1;
  REG32 /**/            :4;  
  REG32 MR4INT          :1;
  REG32 MR5INT          :1;
  REG32 MR6INT          :1;
  REG32 /**/            :21;  
  }	bit_pwmir;
  REG32	reg_pwmir;
} pwmir_bits;

/* PWM timer control register */
typedef union{
struct {
  REG32 CE              :1;
  REG32 CR              :1;
  REG32 /**/            :1;
  REG32 PWMEN           :1;
  REG32 /**/            :28;  
  }	bit_pwmtcr;
  REG32	reg_pwmtcr;
} pwmtcr_bits;

/* PWM match control register */
typedef union{
struct {
  REG32 MR0INT          :1;
  REG32 MR0RES          :1;
  REG32 MR0STOP         :1;
  REG32 MR1INT          :1;
  REG32 MR1RES          :1;
  REG32 MR1STOP         :1;
  REG32 MR2INT          :1;
  REG32 MR2RES          :1;
  REG32 MR2STOP         :1;
  REG32 MR3INT          :1;
  REG32 MR3RES          :1;
  REG32 MR3STOP         :1;
  REG32 MR4INT          :1;
  REG32 MR4RES          :1;
  REG32 MR4STOP         :1;
  REG32 MR5INT          :1;
  REG32 MR5RES          :1;
  REG32 MR5STOP         :1;
  REG32 MR6INT          :1;
  REG32 MR6RES          :1;
  REG32 MR6STOP         :1;
  REG32 /**/            :11;  
  }	bit_pwmmcr;
  REG32	reg_pwmmcr;
} pwmmcr_bits;


/* PWM  control register */
typedef union{
struct {
  REG32 /**/            :1;
  REG32 SEL1            :1;
  REG32 SEL2            :1;
  REG32 SEL3            :1;
  REG32 SEL4            :1;
  REG32 SEL5            :1;
  REG32 SEL6            :1;
  REG32 /**/            :2;
  REG32 ENA1            :1;
  REG32 ENA2            :1;
  REG32 ENA3            :1;
  REG32 ENA4            :1;
  REG32 ENA5            :1;
  REG32 ENA6            :1;
  REG32 /**/            :17;   
  }	bit_pwmpcr;
  REG32	reg_pwmpcr;
} pwmpcr_bits;

/* PWM latch enable register */
typedef union{
struct {
  REG32 EM0L            :1;
  REG32 EM1L            :1;
  REG32 EM2L            :1;
  REG32 EM3L            :1;
  REG32 EM4L            :1;
  REG32 EM5L            :1;
  REG32 EM6L            :1;
  REG32 /**/            :25;   
  }	bit_pwmler;
  REG32	reg_pwmler;
} pwmler_bits;



/* A/D control register */
typedef union{
struct {
  REG32 SEL             :8;
  REG32 CLKDIV          :8;
  REG32 BURST           :1;
  REG32 CLKS            :3;
  REG32 /**/            :1;
  REG32 PDN             :1;
  REG32 TEST            :2;
  REG32 START           :3;
  REG32 EDGE            :1;
  REG32 /**/            :4;	  
  }	bit_adcr;
  REG32	reg_adcr;
} adcr_bits;

/* A/D data register */
typedef union{
struct {
  REG32 /**/            :6;
  REG32 VVDDA           :10;
  REG32 /**/            :8;
  REG32 CHN             :3;
  REG32 /**/            :3;
  REG32 OVERUN          :1;
  REG32 DONE            :1;	 
  }	bit_addr;
  REG32	reg_addr;
} addr_bits;


/* A/D data register channel 0 */
typedef union{
	struct {
		REG32 /**/            :6;
		REG32 RESULT          :10;
		REG32 /**/            :14;
		REG32 OVERUN          :1;
		REG32 DONE            :1;	 
	}	bit_addr0;
	REG32	reg_addr0;
} addr0_bits;

/* A/D data register channel 1 */
typedef union{
	struct {
		REG32 /**/            :6;
		REG32 RESULT          :10;
		REG32 /**/            :14;
		REG32 OVERUN          :1;
		REG32 DONE            :1;	 
	}	bit_addr1;
	REG32	reg_addr1;
} addr1_bits;

/* A/D data register channel 2 */
typedef union{
	struct {
		REG32 /**/            :6;
		REG32 RESULT          :10;
		REG32 /**/            :14;
		REG32 OVERUN          :1;
		REG32 DONE            :1;	 
	}	bit_addr2;
	REG32	reg_addr2;
} addr2_bits;

/* A/D data register channel 3 */
typedef union{
	struct {
		REG32 /**/            :6;
		REG32 RESULT          :10;
		REG32 /**/            :14;
		REG32 OVERUN          :1;
		REG32 DONE            :1;	 
	}	bit_addr3;
	REG32	reg_addr3;
} addr3_bits;


/* RTC interrupt location register */
typedef union{
struct {
  REG32 RTCCIF          :1;
  REG32 RTCALF          :1;
  REG32 /**/            :30;  
  }	bit_ilr;
  REG32	reg_ilr;
} ilr_bits;

/* RTC clock tick counter register */
typedef union{
struct {
  REG32 /**/            :1;
  REG32 COUNTER         :15;
  REG32 /**/            :16;
  }	bit_ctc;
  REG32	reg_ctc;
} ctc_bits;

/* RTC clock control register */
typedef union{
struct {
  REG32 CLKEN           :1;
  REG32 CTCRST          :1;
  REG32 CTTEST          :2;
  REG32 /**/            :28;
  }	bit_rtcccr;
  REG32	reg_rtcccr;
} rtcccr_bits;

/* RTC counter increment interrupt register */
typedef union{
struct {
  REG32 IMSEC           :1;
  REG32 IMMIN           :1;
  REG32 IMHOUR          :1;
  REG32 IMDOM           :1;
  REG32 IMDOW           :1;
  REG32 IMDOY           :1;
  REG32 IMMON           :1;
  REG32 IMYEAR          :1;
  REG32 /**/            :24; 
  }	bit_ciir;
  REG32	reg_ciir;
} ciir_bits;

/* RTC alarm mask register */
typedef union{
struct {
  REG32 AMRSEC          :1;
  REG32 AMRMIN          :1;
  REG32 AMRHOUR         :1;
  REG32 AMRDOM          :1;
  REG32 AMRDOW          :1;
  REG32 AMRDOY          :1;
  REG32 AMRMON          :1;
  REG32 AMRYEAR         :1;
  REG32 /**/            :24; 
  }	bit_amr;
  REG32	reg_amr;
} amr_bits;

/* RTC consolidated time register 0 */
typedef union{
struct {
  REG32 SEC             :6;
  REG32 /**/            :2;
  REG32 MIN             :6;
  REG32 /**/            :2;
  REG32 HOUR            :5;
  REG32 /**/            :3;
  REG32 DOW             :3;
  REG32 /**/            :5;	
  }	bit_ctime0;
  REG32	reg_ctime0;
} ctime0_bits;

/* RTC consolidated time register 1 */
typedef union{
struct {
  REG32 DOM             :5;
  REG32 /**/            :3;
  REG32 MON             :4;
  REG32 /**/            :4;
  REG32 YEAR            :12;
  REG32 /**/            :4;	
  }	bit_ctime1;
  REG32	reg_ctime1;
} ctime1_bits;

/* RTC consolidated time register 2 */
typedef union{
struct {
  REG32 DOY             :12;
  REG32 /**/            :20; 
  }	bit_ctime2;
  REG32	reg_ctime2;
} ctime2_bits;

/* RTC second register */
typedef union{
struct {
  REG32 SEC             :6;
  REG32 /**/            :26;  
  }	bit_sec;
  REG32	reg_sec;
} sec_bits1,sec_bits2;

/* RTC minute register */
typedef union{
struct {
  REG32 MIN             :6;
  REG32 /**/            :26;  
  }	bit_min;
  REG32	reg_min;
} min_bits1,min_bits2;

/* RTC hour register */
typedef union{
struct {
  REG32 HOUR            :5;
  REG32 /**/            :27; 
  }	bit_hour;
  REG32	reg_hour;
} hour_bits1,hour_bits2;

/* RTC day of month register */
typedef union{
struct {
  REG32 DOM             :5;
  REG32 /**/            :27; 
  }	bit_dom;
  REG32	reg_dom;
} dom_bits1,dom_bits2;

/* RTC day of week register */
typedef union{
struct {
  REG32 DOW             :3;
  REG32 /**/            :29; 
  }	bit_dow;
  REG32	reg_dow;
} dow_bits1,dow_bits2;

/* RTC day of year register */
typedef union{
struct {
  REG32 DOY             :9;
  REG32 /**/            :23; 
  } bit_doy;
  REG32	reg_doy;
} doy_bits1,doy_bits2;

/* RTC month register */
typedef union{
struct {
  REG32 MON             :4;
  REG32 /**/            :28; 
  }	bit_month;
  REG32	reg_month;
} month_bits1,month_bits2;

/* RTC year register */
typedef union{
struct {
  REG32 YEAR            :12;
  REG32 /**/            :20; 
  }	bit_year;
  REG32 reg_year;
} year_bits1,year_bits2;

/* RTC prescaler value, integer portion register */
typedef union{
struct {
  REG32 VALUE           :13;
  REG32 /**/            :19; 
  }	bit_preint;
  REG32	reg_preint;
} preint_bits;

/* RTC prescaler value, fractional portion register */
typedef union{
struct {
  REG32 VALUE           :15;
  REG32 /**/            :17; 
  }	bit_prefrac;
  REG32	reg_prefrac;
} prefrac_bits;

/* Watchdog mode register */
typedef union{
struct {
  REG32 WDEN            :1;
  REG32 WDRESET         :1;
  REG32 WDTOF           :1;
  REG32 WDINT           :1;
  REG32 /**/            :28;  
  }	bit_wdmod;
  REG32	reg_wdmod;
} wdmod_bits;

/* Watchdog feed register */
typedef union{
struct {
  REG32 FEED            :8;
  REG32 /**/            :24;  
  }	bit_wdfeed;
  REG32 reg_wdfeed;
} wdfeed_bits;

/* System Control and Status */
typedef union{
struct {
  REG32 GPIO0M          :1;
  REG32 GPIO1M          :1;  
  REG32 /**/            :30; 
  }	bit_scs;
  REG32	reg_scs;
} scs_bits;

/* Common declarations  ****************************************************/

/***************************************************************************
 **
 ** System control block
 **														   
 ***************************************************************************/
#define EXTINT_str (*((volatile extint_bits*)0xE01FC140))
#define EXTINT	       EXTINT_str.reg_extint
#define EXTINT_bit	   EXTINT_str.bit_extint
#define EXTWAKE_str (*((volatile extwake_bits*)0xE01FC144))
#define EXTWAKE	       EXTWAKE_str.reg_extwake
#define EXTWAKE_bit	   EXTWAKE_str.bit_extwake
#define EXTMODE_str (*((volatile extmode_bits*)0xE01FC148))
#define EXTMODE	       EXTMODE_str.reg_extmode
#define EXTMODE_bit	   EXTMODE_str.bit_extmode
#define EXTPOLAR_str (*((volatile extpolar_bits*)0xE01FC14C))
#define EXTPOLAR	   EXTPOLAR_str.reg_extpolar
#define EXTPOLAR_bit   EXTPOLAR_str.bit_extpolar
#define MEMMAP_str (*((volatile memmap_bits*)0xE01FC040))
#define MEMMAP	       MEMMAP_str.reg_memmap
#define MEMMAP_bit	   MEMMAP_str.bit_memmap
#define PLLCON_str (*((volatile pllcon_bits*)0xE01FC080))
#define PLLCON	       PLLCON_str.reg_pllcon
#define PLLCON_bit	   PLLCON_str.bit_pllcon
#define PLLCFG_str (*((volatile pllcfg_bits*)0xE01FC084))
#define PLLCFG	       PLLCFG_str.reg_pllcfg
#define PLLCFG_bit	   PLLCFG_str.bit_pllcfg
#define PLLSTAT_str (*((volatile pllstat_bits*)0xE01FC088))
#define PLLSTAT	       PLLSTAT_str.reg_pllstat
#define PLLSTAT_bit	   PLLSTAT_str.bit_pllstat
#define PLLFEED_str (*((volatile pllfeed_bits*)0xE01FC08C))
#define PLLFEED	       PLLFEED_str.reg_pllfeed
#define PLLFEED_bit	   PLLFEED_str.bit_pllfeed
#define PCON_str (*((volatile pcon_bits*)0xE01FC0C0))
#define PCON	       PCON_str.reg_pcon
#define PCON_bit	   PCON_str.bit_pcon
#define PCONP_str (*((volatile pconp_bits*)0xE01FC0C4))
#define PCONP	       PCONP_str.reg_pconp
#define PCONP_bit	   PCONP_str.bit_pconp	
#define VPBDIV_str (*((volatile vpbdiv_bits*)0xE01FC100))
#define VPBDIV	       VPBDIV_str.reg_vpbdiv
#define VPBDIV_bit	   VPBDIV_str.bit_vpbdiv
#define SCS_str (*((volatile scs_bits*)0xE01FC1A0))
#define SCS	           SCS_str.reg_scs
#define SCS_bit	       SCS_str.bit_scs

/***************************************************************************
 **
 ** MAM
 **
 ***************************************************************************/
#define MAMCR_str (*((volatile mamcr_bits*)0xE01FC000))
#define MAMCR	           MAMCR_str.reg_mamcr
#define MAMCR_bit	       MAMCR_str.bit_mamcr
#define MAMTIM_str (*((volatile mamtim_bits*)0xE01FC004))
#define MAMTIM	           SCS_str.reg_mamtim
#define MAMTIM_bit	       SCS_str.bit_mamtim

/***************************************************************************
 **
 ** VIC
 **
 ***************************************************************************/
#define VICIRQStatus_str (*((volatile vicint_bits1*)0xFFFFF000))
#define VICIRQStatus	           VICIRQStatus_str.reg_vicint
#define VICIRQStatus_bit	       VICIRQStatus_str.bit_vicint
#define VICFIQStatus_str (*((volatile vicint_bits2*)0xFFFFF004))
#define VICFIQStatus	           VICFIQStatus_str.reg_vicint
#define VICFIQStatus_bit	       VICFIQStatus_str.bit_vicint
#define VICRawIntr_str (*((volatile vicint_bits3*)0xFFFFF008))   
#define VICRawIntr	               VICRawIntr_str.reg_vicint
#define VICRawIntr_bit	           VICRawIntr_str.bit_vicint
#define VICIntSelect_str (*((volatile vicint_bits4*)0xFFFFF00C)) 
#define VICIntSelect	           VICIntSelect_str.reg_vicint
#define VICIntSelect_bit	       VICIntSelect_str.bit_vicint
#define VICIntEnable_str (*((volatile vicint_bits5*)0xFFFFF010)) 
#define VICIntEnable	           VICIntEnable_str.reg_vicint
#define VICIntEnable_bit	       VICIntEnable_str.bit_vicint
#define VICIntEnClear_str (*((volatile vicint_bits6*)0xFFFFF014))	
#define VICIntEnClear	           VICIntEnClear_str.reg_vicint
#define VICIntEnClear_bit	       VICIntEnClear_str.bit_vicint
#define VICSoftInt_str (*((volatile vicint_bits7*)0xFFFFF018))   
#define VICSoftInt	               VICSoftInt_str.reg_vicint
#define VICSoftInt_bit	           VICSoftInt_str.bit_vicint
#define VICSoftIntClear_str (*((volatile vicint_bits8*)0xFFFFF01C)) 
#define VICSoftIntClear	           VICSoftIntClear_str.reg_vicint
#define VICSoftIntClear_bit	           VICSoftIntClear_str.bit_vicint
#define VICProtection_str (*((volatile vicprotection_bits*)0xFFFFF020)) 
#define VICProtection	           VICProtection_str.reg_vicprotection
#define VICProtection_bit	       VICProtection_str.bit_vicprotection
#define VICVectAddr (*((REG32 *)0xFFFFF030))	 
#define VICDefVectAddr (*((REG32 *)0xFFFFF034))
#define VICVectAddr0 (*((REG32 *)0xFFFFF100))	 
#define VICVectAddr1 (*((REG32 *)0xFFFFF104))	 
#define VICVectAddr2 (*((REG32 *)0xFFFFF108))	
#define VICVectAddr3 (*((REG32 *)0xFFFFF10C))	
#define VICVectAddr4 (*((REG32 *)0xFFFFF110))	
#define VICVectAddr5 (*((REG32 *)0xFFFFF114))	
#define VICVectAddr6 (*((REG32 *)0xFFFFF118))	
#define VICVectAddr7 (*((REG32 *)0xFFFFF11C))	
#define VICVectAddr8 (*((REG32 *)0xFFFFF120))	
#define VICVectAddr9 (*((REG32 *)0xFFFFF124))	 
#define VICVectAddr10 (*((REG32 *)0xFFFFF128)) 
#define VICVectAddr11 (*((REG32 *)0xFFFFF12C)) 
#define VICVectAddr12 (*((REG32 *)0xFFFFF130))
#define VICVectAddr13 (*((REG32 *)0xFFFFF134)) 
#define VICVectAddr14 (*((REG32 *)0xFFFFF138)) 
#define VICVectAddr15 (*((REG32 *)0xFFFFF13C))
#define VICVectCntl0_str (*((volatile vicvectcntl_bits0*)0xFFFFF200))
#define VICVectCntl0	           VICVectCntl0_str.reg_vicvectcntl
#define VICVectCntl0_bit	       VICVectCntl0_str.bit_vicvectcntl
#define VICVectCntl1_str (*((volatile vicvectcntl_bits1*)0xFFFFF204))	
#define VICVectCntl1	           VICVectCntl1_str.reg_vicvectcntl
#define VICVectCntl1_bit	       VICVectCntl1_str.bit_vicvectcntl
#define VICVectCntl2_str (*((volatile vicvectcntl_bits2*)0xFFFFF208))	
#define VICVectCntl2	           VICVectCntl2_str.reg_vicvectcntl
#define VICVectCntl2_bit	       VICVectCntl2_str.bit_vicvectcntl
#define VICVectCntl3_str (*((volatile vicvectcntl_bits3*)0xFFFFF20C))
#define VICVectCntl3	           VICVectCntl3_str.reg_vicvectcntl
#define VICVectCntl3_bit	       VICVectCntl3_str.bit_vicvectcntl
#define VICVectCntl4_str (*((volatile vicvectcntl_bits4*)0xFFFFF210))
#define VICVectCntl4	           VICVectCntl4_str.reg_vicvectcntl
#define VICVectCntl4_bit	       VICVectCntl4_str.bit_vicvectcntl
#define VICVectCntl5_str (*((volatile vicvectcntl_bits5*)0xFFFFF214))
#define VICVectCntl5	           VICVectCntl5_str.reg_vicvectcntl
#define VICVectCntl5_bit	       VICVectCntl5_str.bit_vicvectcntl
#define VICVectCntl6_str (*((volatile vicvectcntl_bits6*)0xFFFFF218))
#define VICVectCntl6	           VICVectCntl6_str.reg_vicvectcntl
#define VICVectCntl6_bit	       VICVectCntl6_str.bit_vicvectcntl
#define VICVectCntl7_str (*((volatile vicvectcntl_bits7*)0xFFFFF21C))
#define VICVectCntl7	           VICVectCntl7_str.reg_vicvectcntl
#define VICVectCntl7_bit	       VICVectCntl7_str.bit_vicvectcntl
#define VICVectCntl8_str (*((volatile vicvectcntl_bits8*)0xFFFFF220))	
#define VICVectCntl8	           VICVectCntl8_str.reg_vicvectcntl
#define VICVectCntl8_bit	       VICVectCntl8_str.bit_vicvectcntl
#define VICVectCntl9_str (*((volatile vicvectcntl_bits9*)0xFFFFF224))	
#define VICVectCntl9	           VICVectCntl9_str.reg_vicvectcntl
#define VICVectCntl9_bit	       VICVectCntl9_str.bit_vicvectcntl
#define VICVectCntl10_str (*((volatile vicvectcntl_bits10*)0xFFFFF228))
#define VICVectCntl10	           VICVectCntl10_str.reg_vicvectcntl
#define VICVectCntl10_bit	       VICVectCntl10_str.bit_vicvectcntl
#define VICVectCntl11_str (*((volatile vicvectcntl_bits11*)0xFFFFF22C))
#define VICVectCntl11	           VICVectCntl11_str.reg_vicvectcntl
#define VICVectCntl11_bit	       VICVectCntl11_str.bit_vicvectcntl
#define VICVectCntl12_str (*((volatile vicvectcntl_bits12*)0xFFFFF230))
#define VICVectCntl12	           VICVectCntl12_str.reg_vicvectcntl
#define VICVectCntl12_bit	       VICVectCntl12_str.bit_vicvectcntl
#define VICVectCntl13_str (*((volatile vicvectcntl_bits13*)0xFFFFF234))
#define VICVectCntl13	           VICVectCntl13_str.reg_vicvectcntl
#define VICVectCntl13_bit	       VICVectCntl13_str.bit_vicvectcntl
#define VICVectCntl14_str (*((volatile vicvectcntl_bits14*)0xFFFFF238)) 
#define VICVectCntl14	           VICVectCntl14_str.reg_vicvectcntl
#define VICVectCntl14_bit	       VICVectCntl14_str.bit_vicvectcntl
#define VICVectCntl15_str (*((volatile vicvectcntl_bits15*)0xFFFFF23C))
#define VICVectCntl15	           VICVectCntl15_str.reg_vicvectcntl
#define VICVectCntl15_bit	       VICVectCntl15_str.bit_vicvectcntl

/***************************************************************************
 **
 ** Pin connect block
 **
 ***************************************************************************/
#define PINSEL0_str (*((volatile pinsel0_bits*)0xE002C000))
#define PINSEL0	           PINSEL0_str.reg_pinsel0
#define PINSEL0_bit	       PINSEL0_str.bit_pinsel0
#define PINSEL1_str (*((volatile pinsel1_bits*)0xE002C004))
#define PINSEL1	           PINSEL1_str.reg_pinsel1
#define PINSEL1_bit	       PINSEL1_str.bit_pinsel1
#define PINSEL2 (*((REG32 *)0xE002C014))

/***************************************************************************
 **
 ** GPIO
 **
 ***************************************************************************/
#define IO0PIN_str (*((volatile gpio0_bits1*)0xE0028000))
#define IO0PIN	           IO0PIN_str.reg_gpio0
#define IO0PIN_bit	       IO0PIN_str.bit_gpio0
#define IO0SET_str (*((volatile gpio0_bits2*)0xE0028004))	  
#define IO0SET	           IO0SET_str.reg_gpio0
#define IO0SET_bit	       IO0SET_str.bit_gpio0
#define IO0DIR_str (*((volatile gpio0_bits3*)0xE0028008))
#define IO0DIR	           IO0DIR_str.reg_gpio0
#define IO0DIR_bit	       IO0DIR_str.bit_gpio0
#define IO0CLR_str (*((volatile gpio0_bits4*)0xE002800C))
#define IO0CLR	           IO0CLR_str.reg_gpio0
#define IO0CLR_bit	       IO0CLR_str.bit_gpio0
#define FIO0DIR_str (*((volatile fgpio0_bits1*)0x3FFFC000))
#define FIO0DIR			  FIO0DIR_str.reg_fgpio0
#define FIO0DIR_bit		  FIO0DIR_str.bit_fgpio0.addr0_w
#define FIO0DIR0          FIO0DIR_str.bit_fgpio0.addr0_b.byte_0.byte0
#define FIO0DIR0_bit      FIO0DIR_str.bit_fgpio0.addr0_b.byte_0.byte0_bit
#define FIO0DIR1          FIO0DIR_str.bit_fgpio0.addr0_b.byte_1.byte1
#define FIO0DIR1_bit      FIO0DIR_str.bit_fgpio0.addr0_b.byte_1.byte1_bit
#define FIO0DIR2          FIO0DIR_str.bit_fgpio0.addr0_b.byte_2.byte2
#define FIO0DIR2_bit      FIO0DIR_str.bit_fgpio0.addr0_b.byte_2.byte2_bit
#define FIO0DIR3          FIO0DIR_str.bit_fgpio0.addr0_b.byte_3.byte3
#define FIO0DIR3_bit      FIO0DIR_str.bit_fgpio0.addr0_b.byte_3.byte3_bit
#define FIO0DIRL          FIO0DIR_str.bit_fgpio0.addr0_hw.hw0.shortl
#define FIO0DIRL_bit      FIO0DIR_str.bit_fgpio0.addr0_hw.hw0.shortl_bit
#define FIO0DIRU          FIO0DIR_str.bit_fgpio0.addr0_hw.hw1.shortu
#define FIO0DIRU_bit      FIO0DIR_str.bit_fgpio0.addr0_hw.hw1.shortu_bit
#define FIO0MASK_str (*((volatile fgpio0_bits2*)0x3FFFC010))
#define FIO0MASK		  FIO0MASK_str.reg_fgpio0
#define FIO0MASK_bit	  FIO0MASK_str.bit_fgpio0.addr0_w
#define FIO0MASK0         FIO0MASK_str.bit_fgpio0.addr0_b.byte_0.byte0
#define FIO0MASK0_bit     FIO0MASK_str.bit_fgpio0.addr0_b.byte_0.byte0_bit
#define FIO0MASK1         FIO0MASK_str.bit_fgpio0.addr0_b.byte_1.byte1
#define FIO0MASK1_bit     FIO0MASK_str.bit_fgpio0.addr0_b.byte_1.byte1_bit
#define FIO0MASK2         FIO0MASK_str.bit_fgpio0.addr0_b.byte_2.byte2
#define FIO0MASK2_bit     FIO0MASK_str.bit_fgpio0.addr0_b.byte_2.byte2_bit
#define FIO0MASK3         FIO0MASK_str.bit_fgpio0.addr0_b.byte_3.byte3
#define FIO0MASK3_bit     FIO0MASK_str.bit_fgpio0.addr0_b.byte_3.byte3_bit
#define FIO0MASKL         FIO0MASK_str.bit_fgpio0.addr0_hw.hw0.shortl
#define FIO0MASKL_bit     FIO0MASK_str.bit_fgpio0.addr0_hw.hw0.shortl_bit
#define FIO0MASKU         FIO0MASK_str.bit_fgpio0.addr0_hw.hw1.shortu
#define FIO0MASKU_bit     FIO0MASK_str.bit_fgpio0.addr0_hw.hw1.shortu_bit
#define FIO0PIN_str (*((volatile fgpio0_bits3*)0x3FFFC014))
#define FIO0PIN		      FIO0PIN_str.reg_fgpio0
#define FIO0PIN_bit	      FIO0PIN_str.bit_fgpio0.addr0_w
#define FIO0PIN0          FIO0PIN_str.bit_fgpio0.addr0_b.byte_0.byte0
#define FIO0PIN0_bit      FIO0PIN_str.bit_fgpio0.addr0_b.byte_0.byte0_bit
#define FIO0PIN1          FIO0PIN_str.bit_fgpio0.addr0_b.byte_1.byte1
#define FIO0PIN1_bit      FIO0PIN_str.bit_fgpio0.addr0_b.byte_1.byte1_bit
#define FIO0PIN2          FIO0PIN_str.bit_fgpio0.addr0_b.byte_2.byte2
#define FIO0PIN2_bit      FIO0PIN_str.bit_fgpio0.addr0_b.byte_2.byte2_bit
#define FIO0PIN3          FIO0PIN_str.bit_fgpio0.addr0_b.byte_3.byte3
#define FIO0PIN3_bit      FIO0PIN_str.bit_fgpio0.addr0_b.byte_3.byte3_bit
#define FIO0PINL          FIO0PIN_str.bit_fgpio0.addr0_hw.hw0.shortl
#define FIO0PINL_bit      FIO0PIN_str.bit_fgpio0.addr0_hw.hw0.shortl_bit
#define FIO0PINU          FIO0PIN_str.bit_fgpio0.addr0_hw.hw1.shortu
#define FIO0PINU_bit      FIO0PIN_str.bit_fgpio0.addr0_hw.hw1.shortu_bit
#define FIO0SET_str (*((volatile fgpio0_bits4*)0x3FFFC018))
#define FIO0SET		      FIO0SET_str.reg_fgpio0
#define FIO0SET_bit	      FIO0SET_str.bit_fgpio0.addr0_w
#define FIO0SET0          FIO0SET_str.bit_fgpio0.addr0_b.byte_0.byte0
#define FIO0SET0_bit      FIO0SET_str.bit_fgpio0.addr0_b.byte_0.byte0_bit
#define FIO0SET1          FIO0SET_str.bit_fgpio0.addr0_b.byte_1.byte1
#define FIO0SET1_bit      FIO0SET_str.bit_fgpio0.addr0_b.byte_1.byte1_bit
#define FIO0SET2          FIO0SET_str.bit_fgpio0.addr0_b.byte_2.byte2
#define FIO0SET2_bit      FIO0SET_str.bit_fgpio0.addr0_b.byte_2.byte2_bit
#define FIO0SET3          FIO0SET_str.bit_fgpio0.addr0_b.byte_3.byte3
#define FIO0SET3_bit      FIO0SET_str.bit_fgpio0.addr0_b.byte_3.byte3_bit
#define FIO0SETL          FIO0SET_str.bit_fgpio0.addr0_hw.hw0.shortl
#define FIO0SETL_bit      FIO0SET_str.bit_fgpio0.addr0_hw.hw0.shortl_bit
#define FIO0SETU          FIO0SET_str.bit_fgpio0.addr0_hw.hw1.shortu
#define FIO0SETU_bit      FIO0SET_str.bit_fgpio0.addr0_hw.hw1.shortu_bit
#define FIO0CLR_str (*((volatile fgpio0_bits5*)0x3FFFC01C))
#define FIO0CLR		      FIO0CLR_str.reg_fgpio0
#define FIO0CLR_bit	      FIO0CLR_str.bit_fgpio0.addr0_w
#define FIO0CLR0          FIO0CLR_str.bit_fgpio0.addr0_b.byte_0.byte0
#define FIO0CLR0_bit      FIO0CLR_str.bit_fgpio0.addr0_b.byte_0.byte0_bit
#define FIO0CLR1          FIO0CLR_str.bit_fgpio0.addr0_b.byte_1.byte1
#define FIO0CLR1_bit      FIO0CLR_str.bit_fgpio0.addr0_b.byte_1.byte1_bit
#define FIO0CLR2          FIO0CLR_str.bit_fgpio0.addr0_b.byte_2.byte2
#define FIO0CLR2_bit      FIO0CLR_str.bit_fgpio0.addr0_b.byte_2.byte2_bit
#define FIO0CLR3          FIO0CLR_str.bit_fgpio0.addr0_b.byte_3.byte3
#define FIO0CLR3_bit      FIO0CLR_str.bit_fgpio0.addr0_b.byte_3.byte3_bit
#define FIO0CLRL          FIO0CLR_str.bit_fgpio0.addr0_hw.hw0.shortl
#define FIO0CLRL_bit      FIO0CLR_str.bit_fgpio0.addr0_hw.hw0.shortl_bit
#define FIO0CLRU          FIO0CLR_str.bit_fgpio0.addr0_hw.hw1.shortu
#define FIO0CLRU_bit      FIO0CLR_str.bit_fgpio0.addr0_hw.hw1.shortu_bit
#define IO1PIN_str (*((volatile gpio1_bits1*)0xE0028010))
#define IO1PIN	           IO1PIN_str.reg_gpio1
#define IO1PIN_bit	       IO1PIN_str.bit_gpio1
#define IO1SET_str (*((volatile gpio1_bits2*)0xE0028014))
#define IO1SET	           IO1SET_str.reg_gpio1
#define IO1SET_bit	       IO1SET_str.bit_gpio1
#define IO1DIR_str (*((volatile gpio1_bits3*)0xE0028018))
#define IO1DIR	           IO1DIR_str.reg_gpio1
#define IO1DIR_bit	       IO1DIR_str.bit_gpio1
#define IO1CLR_str (*((volatile gpio1_bits4*)0xE002801C))
#define IO1CLR	           IO1CLR_str.reg_gpio1
#define IO1CLR_bit	       IO1CLR_str.bit_gpio1
#define FIO1DIR_str (*((volatile fgpio1_bits1*)0x3FFFC020))
#define FIO1DIR		      FIO1DIR_str.reg_fgpio1
#define FIO1DIR_bit	      FIO1DIR_str.bit_fgpio1.addr1_w
#define FIO1DIR0          FIO1DIR_str.bit_fgpio1.addr1_b.byte_0.byte0
#define FIO1DIR0_bit      FIO1DIR_str.bit_fgpio1.addr1_b.byte_0.byte0_bit
#define FIO1DIR1          FIO1DIR_str.bit_fgpio1.addr1_b.byte_1.byte1
#define FIO1DIR1_bit      FIO1DIR_str.bit_fgpio1.addr1_b.byte_1.byte1_bit
#define FIO1DIR2          FIO1DIR_str.bit_fgpio1.addr1_b.byte_2.byte2
#define FIO1DIR2_bit      FIO1DIR_str.bit_fgpio1.addr1_b.byte_2.byte2_bit
#define FIO1DIR3          FIO1DIR_str.bit_fgpio1.addr1_b.byte_3.byte3
#define FIO1DIR3_bit      FIO1DIR_str.bit_fgpio1.addr1_b.byte_3.byte3_bit
#define FIO1DIRL          FIO1DIR_str.bit_fgpio1.addr1_hw.hw0.shortl
#define FIO1DIRL_bit      FIO1DIR_str.bit_fgpio1.addr1_hw.hw0.shortl_bit
#define FIO1DIRU          FIO1DIR_str.bit_fgpio1.addr1_hw.hw1.shortu
#define FIO1DIRU_bit      FIO1DIR_str.bit_fgpio1.addr1_hw.hw1.shortu_bit
#define FIO1MASK_str (*((volatile fgpio1_bits2*)0x3FFFC030))
#define FIO1MASK		  FIO1MASK_str.reg_fgpio1
#define FIO1MASK_bit	  FIO1MASK_str.bit_fgpio1.addr1_w
#define FIO1MASK0         FIO1MASK_str.bit_fgpio1.addr1_b.byte_0.byte0
#define FIO1MASK0_bit     FIO1MASK_str.bit_fgpio1.addr1_b.byte_0.byte0_bit
#define FIO1MASK1         FIO1MASK_str.bit_fgpio1.addr1_b.byte_1.byte1
#define FIO1MASK1_bit     FIO1MASK_str.bit_fgpio1.addr1_b.byte_1.byte1_bit
#define FIO1MASK2         FIO1MASK_str.bit_fgpio1.addr1_b.byte_2.byte2
#define FIO1MASK2_bit     FIO1MASK_str.bit_fgpio1.addr1_b.byte_2.byte2_bit
#define FIO1MASK3         FIO1MASK_str.bit_fgpio1.addr1_b.byte_3.byte3
#define FIO1MASK3_bit     FIO1MASK_str.bit_fgpio1.addr1_b.byte_3.byte3_bit
#define FIO1MASKL         FIO1MASK_str.bit_fgpio1.addr1_hw.hw0.shortl
#define FIO1MASKL_bit     FIO1MASK_str.bit_fgpio1.addr1_hw.hw0.shortl_bit
#define FIO1MASKU         FIO1MASK_str.bit_fgpio1.addr1_hw.hw1.shortu
#define FIO1MASKU_bit     FIO1MASK_str.bit_fgpio1.addr1_hw.hw1.shortu_bit
#define FIO1PIN_str (*((volatile fgpio1_bits3*)0x3FFFC034))
#define FIO1PIN		      FIO1PIN_str.reg_fgpio1
#define FIO1PIN_bit	      FIO1PIN_str.bit_fgpio1.addr1_w
#define FIO1PIN0          FIO1PIN_str.bit_fgpio1.addr1_b.byte_0.byte0
#define FIO1PIN0_bit      FIO1PIN_str.bit_fgpio1.addr1_b.byte_0.byte0_bit
#define FIO1PIN1          FIO1PIN_str.bit_fgpio1.addr1_b.byte_1.byte1
#define FIO1PIN1_bit      FIO1PIN_str.bit_fgpio1.addr1_b.byte_1.byte1_bit
#define FIO1PIN2          FIO1PIN_str.bit_fgpio1.addr1_b.byte_2.byte2
#define FIO1PIN2_bit      FIO1PIN_str.bit_fgpio1.addr1_b.byte_2.byte2_bit
#define FIO1PIN3          FIO1PIN_str.bit_fgpio1.addr1_b.byte_3.byte3
#define FIO1PIN3_bit      FIO1PIN_str.bit_fgpio1.addr1_b.byte_3.byte3_bit
#define FIO1PINL          FIO1PIN_str.bit_fgpio1.addr1_hw.hw0.shortl
#define FIO1PINL_bit      FIO1PIN_str.bit_fgpio1.addr1_hw.hw0.shortl_bit
#define FIO1PINU          FIO1PIN_str.bit_fgpio1.addr1_hw.hw1.shortu
#define FIO1PINU_bit      FIO1PIN_str.bit_fgpio1.addr1_hw.hw1.shortu_bit
#define FIO1SET_str (*((volatile fgpio1_bits4*)0x3FFFC038))
#define FIO1SET		      FIO1SET_str.reg_fgpio1
#define FIO1SET_bit	      FIO1SET_str.bit_fgpio1.addr1_w
#define FIO1SET0          FIO1SET_str.bit_fgpio1.addr1_b.byte_0.byte0
#define FIO1SET0_bit      FIO1SET_str.bit_fgpio1.addr1_b.byte_0.byte0_bit
#define FIO1SET1          FIO1SET_str.bit_fgpio1.addr1_b.byte_1.byte1
#define FIO1SET1_bit      FIO1SET_str.bit_fgpio1.addr1_b.byte_1.byte1_bit
#define FIO1SET2          FIO1SET_str.bit_fgpio1.addr1_b.byte_2.byte2
#define FIO1SET2_bit      FIO1SET_str.bit_fgpio1.addr1_b.byte_2.byte2_bit
#define FIO1SET3          FIO1SET_str.bit_fgpio1.addr1_b.byte_3.byte3
#define FIO1SET3_bit      FIO1SET_str.bit_fgpio1.addr1_b.byte_3.byte3_bit
#define FIO1SETL          FIO1SET_str.bit_fgpio1.addr1_hw.hw0.shortl
#define FIO1SETL_bit      FIO1SET_str.bit_fgpio1.addr1_hw.hw0.shortl_bit
#define FIO1SETU          FIO1SET_str.bit_fgpio1.addr1_hw.hw1.shortu
#define FIO1SETU_bit      FIO1SET_str.bit_fgpio1.addr1_hw.hw1.shortu_bit
#define FIO1CLR_str (*((volatile fgpio1_bits5*)0x3FFFC03C))
#define FIO1CLR		      FIO1CLR_str.reg_fgpio1
#define FIO1CLR_bit	      FIO1CLR_str.bit_fgpio1.addr1_w
#define FIO1CLR0          FIO1CLR_str.bit_fgpio1.addr1_b.byte_0.byte0
#define FIO1CLR0_bit      FIO1CLR_str.bit_fgpio1.addr1_b.byte_0.byte0_bit
#define FIO1CLR1          FIO1CLR_str.bit_fgpio1.addr1_b.byte_1.byte1
#define FIO1CLR1_bit      FIO1CLR_str.bit_fgpio1.addr1_b.byte_1.byte1_bit
#define FIO1CLR2          FIO1CLR_str.bit_fgpio1.addr1_b.byte_2.byte2
#define FIO1CLR2_bit      FIO1CLR_str.bit_fgpio1.addr1_b.byte_2.byte2_bit
#define FIO1CLR3          FIO1CLR_str.bit_fgpio1.addr1_b.byte_3.byte3
#define FIO1CLR3_bit      FIO1CLR_str.bit_fgpio1.addr1_b.byte_3.byte3_bit
#define FIO1CLRL          FIO1CLR_str.bit_fgpio1.addr1_hw.hw0.shortl
#define FIO1CLRL_bit      FIO1CLR_str.bit_fgpio1.addr1_hw.hw0.shortl_bit
#define FIO1CLRU          FIO1CLR_str.bit_fgpio1.addr1_hw.hw1.shortu
#define FIO1CLRU_bit      FIO1CLR_str.bit_fgpio1.addr1_hw.hw1.shortu_bit


/***************************************************************************
 **
 **  UART0
 **
 ***************************************************************************/

/* U0DLL, U0RBR and U0THR share the same address */
#define U0RBRTHR (*((REG8 *)0xE000C000))
#define U0DLL U0RBRTHR
#define U0RBR U0RBRTHR
#define U0THR U0RBRTHR

/* U0DLM and U0IER share the same address */
#define U0IER_str (*((volatile uartier0_bits*)0xE000C004))
#define U0IER		U0IER_str.reg_uartier0
#define U0IER_bit	U0IER_str.bit_uartier0
#define U0DLM       U0IER
#define U0DLM_bit	U0IER_bit

/* U0FCR and U0IIR share the same address */
#define U0FCR_IIR_str (*((volatile uartfcriir_bits0*)0xE000C008))
#define U0FCR       U0FCR_IIR_str.reg_uartfcriir
#define U0IIR_bit   U0FCR_IIR_str.bit_uartfcriir.iir
#define U0FCR_bit   U0FCR_IIR_str.bit_uartfcriir.fcr
#define U0IIR       U0FCR

#define U0LCR_str (*((volatile uartlcr_bits0*)0xE000C00C))
#define U0LCR		U0LCR_str.reg_uartlcr
#define U0LCR_bit	U0LCR_str.bit_uartlcr
#define U0LSR_str (*((volatile uartlsr_bits0*)0xE000C014))
#define U0LSR		U0LSR_str.reg_uartlsr
#define U0LSR_bit	U0LSR_str.bit_uartlsr
#define U0SCR (*((REG8 *)0xE000C01C))
#define U0ACR_str (*((volatile uartacr_bits0*)0xE000C020))
#define U0ACR		U0ACR_str.reg_uartacr
#define U0ACR_bit	U0ACR_str.bit_uartacr
#define U0FDR_str (*((volatile uartfdr_bits0*)0xE000C028))
#define U0FDR		U0FDR_str.reg_uartfdr
#define U0FDR_bit	U0FDR_str.bit_uartfdr
#define U0TER_str (*((volatile uartter_bits0*)0xE000C030))
#define U0TER		U0TER_str.reg_uartter
#define U0TER_bit	U0TER_str.bit_uartter


/***************************************************************************
 **
 **  UART1
 **
 ***************************************************************************/

/* U1DLL, U1RBR and U1THR share the same address */
#define U1RBRTHR (*((REG8 *)0xE0010000))
#define U1DLL U1RBRTHR
#define U1RBR U1RBRTHR
#define U1THR U1RBRTHR

/* U1DLM and U1IER share the same address */
#define U1IER_str (*((volatile uartier1_bits*)0xE0010004))
#define U1IER		U1IER_str.reg_uartier1
#define U1IER_bit	U1IER_str.bit_uartier1
#define U1DLM       U1IER
#define U1DLM_bit	U1IER_bit

/* U1FCR and U1IIR share the same address */
#define U1FCR_IIR_str (*((volatile uartfcriir_bits1*)0xE0010008))
#define U1FCR       U1FCR_IIR_str.reg_uartfcriir
#define U1IIR_bit   U1FCR_IIR_str.bit_uartfcriir.iir
#define U1FCR_bit   U1FCR_IIR_str.bit_uartfcriir.fcr
#define U1IIR       U1FCR

#define U1MCR_str (*((volatile uartmcr_bits*)0xE0010010))
#define U1MCR		U1MCR_str.reg_uartmcr
#define U1MCR_bit	U1MCR_str.bit_uartmcr
#define U1LCR_str (*((volatile uartlcr_bits1*)0xE001000C))
#define U1LCR		U1LCR_str.reg_uartlcr
#define U1LCR_bit	U1LCR_str.bit_uartlcr
#define U1LSR_str (*((volatile uartlsr_bits1*)0xE0010014))
#define U1LSR		U1LSR_str.reg_uartlsr
#define U1LSR_bit	U1LSR_str.bit_uartlsr
#define U1MSR_str (*((volatile uartmsr_bits*)0xE0010018))
#define U1SCR (*((REG8 *)0xE001001C))
#define U1ACR_str (*((volatile uartacr_bits1*)0xE0010020))
#define U1ACR		U1ACR_str.reg_uartacr
#define U1ACR_bit	U1ACR_str.bit_uartacr
#define U1FDR_str (*((volatile uartfdr_bits1*)0xE0010028))
#define U1FDR		U1FDR_str.reg_uartfdr
#define U1FDR_bit	U1FDR_str.bit_uartfdr
#define U1TER_str (*((volatile uartter_bits1*)0xE0010030))
#define U1TER		U1TER_str.reg_uartter
#define U1TER_bit	U1TER_str.bit_uartter


/***************************************************************************
 **
 ** I2C
 **
 ***************************************************************************/
#define I2CONSET_str (*((volatile i2conset_bits*)0xE001C000))
#define I2CONSET		I2CONSET_str.reg_i2conset
#define I2CONSET_bit	I2CONSET_str.bit_i2conset
#define I2STAT_str (*((volatile i2stat_bits*)0xE001C004))
#define I2STAT	    	I2STAT_str.reg_i2stat
#define I2STAT_bit    	I2STAT_str.bit_i2stat
#define I2DAT_str (*((volatile i2dat_bits*)0xE001C008))
#define I2DAT	    	I2DAT_str.reg_i2dat
#define I2DAT_bit   	I2DAT_str.bit_i2dat
#define I2ADR_str (*((volatile i2adr_bits*)0xE001C00C))
#define I2ADR	    	I2ADR_str.reg_i2adr
#define I2ADR_bit   	I2ADR_str.bit_i2adr
#define I2SCLH_str (*((volatile i2scl_bits1*)0xE001C010))
#define I2SCLH	    	I2SCLH_str.reg_i2scl
#define I2SCLH_bit   	I2SCLH_str.bit_i2scl
#define I2SCLL_str (*((volatile i2scl_bits2*)0xE001C014))
#define I2SCLL	    	I2SCLL_str.reg_i2scl
#define I2SCLL_bit	    I2SCLL_str.bit_i2scl
#define I2CONCLR_str (*((volatile i2conclr_bits*)0xE001C018))
#define I2CONCLR		I2CONCLR_str.reg_i2conclr
#define I2CONCLR_bit	I2CONCLR_str.bit_i2conclr


/***************************************************************************
 **
 ** SPI
 **
 ***************************************************************************/
#define S0SPCR_str (*((volatile spcr_bits0*)0xE0020000))
#define S0SPCR		S0SPCR_str.reg_spcr
#define S0SPCR_bit	S0SPCR_str.bit_spcr
#define S0SPSR_str (*((volatile spsr_bits0*)0xE0020004))
#define S0SPSR		S0SPSR_str.reg_spsr
#define S0SPSR_bit	S0SPSR_str.bit_spsr
#define S0SPDR_str (*((volatile spdr_bits0*)0xE0020008))
#define S0SPDR		S0SPDR_str.reg_spdr
#define S0SPDR_bit	S0SPDR_str.bit_spdr
#define S0SPCCR_str (*((volatile spccr_bits0*)0xE002000C))
#define S0SPCCR		S0SPCCR_str.reg_spccr
#define S0SPCCR_bit	S0SPCCR_str.bit_spccr
#define S0SPINT_str (*((volatile spint_bits0*)0xE002001C))
#define S0SPINT		S0SPINT_str.reg_spint
#define S0SPINT_bit	S0SPINT_str.bit_spint
#define S1SPCR_str (*((volatile spcr_bits1*)0xE0030000)) 
#define S1SPCR		S1SPCR_str.reg_spcr
#define S1SPCR_bit	S1SPCR_str.bit_spcr
#define S1SPSR_str (*((volatile spsr_bits1*)0xE0030004))	
#define S1SPSR		S1SPSR_str.reg_spsr
#define S1SPSR_bit	S1SPSR_str.bit_spsr
#define S1SPDR_str (*((volatile spdr_bits1*)0xE0030008)) 
#define S1SPDR		S1SPDR_str.reg_spdr
#define S1SPDR_bit	S1SPDR_str.bit_spdr
#define S1SPCCR_str (*((volatile spccr_bits1*)0xE003000C)) 
#define S1SPCCR		S1SPCCR_str.reg_spccr
#define S1SPCCR_bit	S1SPCCR_str.bit_spccr
#define S1SPINT_str (*((volatile spint_bits1*)0xE003001C)) 
#define S1SPINT		S1SPINT_str.reg_spint
#define S1SPINT_bit	S1SPINT_str.bit_spint

/***************************************************************************
 **
 ** SSP
 **
 ***************************************************************************/
#define SSPCR0_str (*((volatile sspcr0_bits*)0xE005C000))
#define SSPCR0		SSPCR0_str.reg_sspcr0
#define SSPCR0_bit	SSPCR0_str.bit_sspcr0
#define SSPCR1_str (*((volatile sspcr1_bits*)0xE005C004))	 
#define SSPCR1		SSPCR1_str.reg_sspcr1
#define SSPCR1_bit	SSPCR1_str.bit_sspcr1
#define SSPDR_str (*((volatile sspdr_bits*)0xE005C008))  
#define SSPDR		SSPDR_str.reg_sspdr
#define SSPDR_bit	SSPDR_str.bit_sspdr
#define SSPSR_str (*((volatile sspsr_bits*)0xE005C00C)) 
#define SSPSR		SSPSR_str.reg_sspsr
#define SSPSR_bit	SSPSR_str.bit_sspsr
#define SSPCPSR_str (*((volatile sspcpsr_bits*)0xE005C010))
#define SSPCPSR		SSPCPSR_str.reg_sspcpsr
#define SSPCPSR_bit	SSPCPSR_str.bit_sspcpsr
#define SSPIMSC_str (*((volatile sspimsc_bits*)0xE005C014))
#define SSPIMSC		SSPIMSC_str.reg_sspimsc
#define SSPIMSC_bit	SSPIMSC_str.bit_sspimsc
#define SSPRIS_str (*((volatile sspris_bits*)0xE005C018))	
#define SSPRIS		SSPRIS_str.reg_sspris
#define SSPRIS_bit	SSPRIS_str.bit_sspris
#define SSPMIS_str (*((volatile sspmis_bits*)0xE005C01C))	
#define SSPMIS		SSPMIS_str.reg_sspmis
#define SSPMIS_bit	SSPMIS_str.bit_sspmis
#define SSPICR_str (*((volatile sspicr_bits*)0xE005C020))
#define SSPICR		SSPICR_str.reg_sspicr
#define SSPICR_bit	SSPICR_str.bit_sspicr

/***************************************************************************
 **
 ** CAN
 **
 ***************************************************************************/

#define AFMR_str (*((volatile afmr_bits*)0xE003C000))
#define AFMR		AFMR_str.reg_afmr
#define AFMR_bit	AFMR_str.bit_afmr
#define SFF_sa (*((REG32 *)0xE003C004))
#define SFF_GRP_sa (*((REG32 *)0xE003C008))
#define EFF_sa (*((REG32 *)0xE003C00C))
#define EFF_GRP_sa (*((REG32 *)0xE003C010))
#define ENDofTable (*((REG32 *)0xE003C014))
#define LUTerrAd (*((REG32 *)0xE003C018))

#define LUTerr (*((REG32 *)0xE003C01C))
#define CANTxSR_str (*((volatile cantxsr_bits*)0xE0040000))
#define CANTxSR	    	CANTxSR_str.reg_cantxsr
#define CANTxSR_bit	    CANTxSR_str.bit_cantxsr
#define CANRxSR_str (*((volatile canrxsr_bits*)0xE0040004))
#define CANRxSR	    	CANRxSR_str.reg_canrxsr
#define CANRxSR_bit	    CANRxSR_str.bit_canrxsr
#define CANMSR_str (*((volatile canmsr_bits*)0xE0040008))
#define CANMSR	    	CANMSR_str.reg_canmsr
#define CANMSR_bit	    CANMSR_str.bit_canmsr

#define C1MOD_str (*((volatile canmod_bits1*)0xE0044000))
#define C1MOD		C1MOD_str.reg_canmod
#define C1MOD_bit	C1MOD_str.bit_canmod
#define C1CMR_str (*((volatile cancmr_bits1*)0xE0044004))
#define C1CMR		C1CMR_str.reg_cancmr
#define C1CMR_bit	C1CMR_str.bit_cancmr
#define C1GSR_str (*((volatile cangsr_bits1*)0xE0044008))
#define C1GSR		C1GSR_str.reg_cangsr
#define C1GSR_bit	C1GSR_str.bit_cangsr
#define C1ICR_str (*((volatile canicr_bits1*)0xE004400C))
#define C1ICR		C1ICR_str.reg_canicr
#define C1ICR_bit	C1ICR_str.bit_canicr
#define C1IER_str (*((volatile canier_bits1*)0xE0044010))
#define C1IER		C1IER_str.reg_canier
#define C1IER_bit	C1IER_str.bit_canier
#define C1BTR_str (*((volatile canbtr_bits1*)0xE0044014))
#define C1BTR		C1BTR_str.reg_canbtr
#define C1BTR_bit	C1BTR_str.bit_canbtr
#define C1EWL_str (*((volatile canewl_bits1*)0xE0044018))
#define C1EWL		C1EWL_str.reg_canewl
#define C1EWL_bit	C1EWL_str.bit_canewl
#define C1SR_str (*((volatile cansr_bits1*)0xE004401C))
#define C1SR		C1SR_str.reg_cansr
#define C1SR_bit	C1SR_str.bit_cansr
#define C1RFS_str (*((volatile canrfs_bits1*)0xE0044020))
#define C1RFS		C1RFS_str.reg_canrfs
#define C1RFS_bit	C1RFS_str.bit_canrfs
#define C1RID_str (*((volatile canrid_bits1*)0xE0044024))
#define C1RID			 C1RID_str.reg_canrid
#define C1RID_ID10_0     C1RID_str.bit_canrid.ID10.ID10_0
#define C1RID_ID29_18    C1RID_str.bit_canrid.ID2918.ID29_18
#define C1RID_ID29_0     C1RID_str.bit_canrid.ID290.ID29_0
#define C1RDA_str (*((volatile canrda_bits1*)0xE0044028))
#define C1RDA		C1RDA_str.reg_canrda
#define C1RDA_bit	C1RDA_str.bit_canrda
#define C1RDB_str (*((volatile canrdb_bits1*)0xE004402C))
#define C1RDB		C1RDB_str.reg_canrdb
#define C1RDB_bit	C1RDB_str.bit_canrdb
#define C1TFI1_str (*((volatile cantfi_bits11*)0xE0044030))
#define C1TFI1		C1TFI1_str.reg_cantfi
#define C1TFI1_bit	C1TFI1_str.bit_cantfi
#define C1TID1_str (*((volatile cantid_bits11*)0xE0044034))
#define C1TID1		 	  C1TID1_str.reg_cantid
#define C1TID1_ID10_0     C1TID1_str.bit_cantid.ID10.ID10_0
#define C1TID1_ID29_18    C1TID1_str.bit_cantid.ID2918.ID29_18
#define C1TID1_ID29_0     C1TID1_str.bit_cantid.ID290.ID29_0
#define C1TDA1_str (*((volatile cantda_bits11*)0xE0044038))
#define C1TDA1		C1TDA1_str.reg_cantda
#define C1TDA1_bit	C1TDA1_str.bit_cantda
#define C1TDB1_str (*((volatile cantdb_bits11*)0xE004403C))
#define C1TDB1		C1TDB1_str.reg_cantdb
#define C1TDB1_bit	C1TDB1_str.bit_cantdb
#define C1TFI2_str (*((volatile cantfi_bits12*)0xE0044040))
#define C1TFI2		C1TFI2_str.reg_cantfi
#define C1TFI2_bit	C1TFI2_str.bit_cantfi
#define C1TID2_str (*((volatile cantid_bits12*)0xE0044044))
#define C1TID2		 	  C1TID2_str.reg_cantid
#define C1TID2_ID10_0     C1TID2_str.bit_cantid.ID10.ID10_0
#define C1TID2_ID29_18    C1TID2_str.bit_cantid.ID2918.ID29_18
#define C1TID2_ID29_0     C1TID2_str.bit_cantid.ID290.ID29_0
#define C1TDA2_str (*((volatile cantda_bits12*)0xE0044048))
#define C1TDA2		C1TDA2_str.reg_cantda
#define C1TDA2_bit	C1TDA2_str.bit_cantda
#define C1TDB2_str (*((volatile cantdb_bits12*)0xE004404C))
#define C1TDB2		C1TDB2_str.reg_cantdb
#define C1TDB2_bit	C1TDB2_str.bit_cantdb
#define C1TFI3_str (*((volatile cantfi_bits13*)0xE0044050))
#define C1TFI3		C1TFI3_str.reg_cantfi
#define C1TFI3_bit	C1TFI3_str.bit_cantfi
#define C1TID3_str (*((volatile cantid_bits13*)0xE0044054))
#define C1TID3		 	  C1TID3_str.reg_cantid
#define C1TID3_ID10_0     C1TID3_str.bit_cantid.ID10.ID10_0
#define C1TID3_ID29_18    C1TID3_str.bit_cantid.ID2918.ID29_18
#define C1TID3_ID29_0     C1TID3_str.bit_cantid.ID290.ID29_0
#define C1TDA3_str (*((volatile cantda_bits13*)0xE0044058))
#define C1TDA3		C1TDA3_str.reg_cantda
#define C1TDA3_bit	C1TDA3_str.bit_cantda
#define C1TDB3_str (*((volatile cantdb_bits13*)0xE004405C))
#define C1TDB3		C1TDB3_str.reg_cantdb
#define C1TDB3_bit	C1TDB3_str.bit_cantdb

#define C2MOD_str (*((volatile canmod_bits2*)0xE0048000))
#define C2MOD		C2MOD_str.reg_canmod
#define C2MOD_bit	C2MOD_str.bit_canmod
#define C2CMR_str (*((volatile cancmr_bits2*)0xE0048004))
#define C2CMR		C2CMR_str.reg_cancmr
#define C2CMR_bit	C2CMR_str.bit_cancmr
#define C2GSR_str (*((volatile cangsr_bits2*)0xE0048008))
#define C2GSR		C2GSR_str.reg_cangsr
#define C2GSR_bit	C2GSR_str.bit_cangsr
#define C2ICR_str (*((volatile canicr_bits2*)0xE004800C))
#define C2ICR		C2ICR_str.reg_canicr
#define C2ICR_bit	C2ICR_str.bit_canicr
#define C2IER_str (*((volatile canier_bits2*)0xE0048010))
#define C2IER		C2IER_str.reg_canier
#define C2IER_bit	C2IER_str.bit_canier
#define C2BTR_str (*((volatile canbtr_bits2*)0xE0048014))
#define C2BTR		C2BTR_str.reg_canbtr
#define C2BTR_bit	C2BTR_str.bit_canbtr
#define C2EWL_str (*((volatile canewl_bits2*)0xE0048018))
#define C2EWL		C2EWL_str.reg_canewl
#define C2EWL_bit	C2EWL_str.bit_canewl
#define C2SR_str (*((volatile cansr_bits2*)0xE004801C))
#define C2SR		C2SR_str.reg_cansr
#define C2SR_bit	C2SR_str.bit_cansr
#define C2RFS_str (*((volatile canrfs_bits2*)0xE0048020))
#define C2RFS		C2RFS_str.reg_canrfs
#define C2RFS_bit	C2RFS_str.bit_canrfs
#define C2RID_str (*((volatile canrid_bits2*)0xE0048024))
#define C2RID		 	 C2RID_str.reg_canrid
#define C2RID_ID10_0     C2RID_str.bit_canrid.ID10.ID10_0
#define C2RID_ID29_18    C2RID_str.bit_canrid.ID2918.ID29_18
#define C2RID_ID29_0     C2RID_str.bit_canrid.ID290.ID29_0
#define C2RDA_str (*((volatile canrda_bits2*)0xE0048028))
#define C2RDA		C2RDA_str.reg_canrda
#define C2RDA_bit	C2RDA_str.bit_canrda
#define C2RDB_str (*((volatile canrdb_bits2*)0xE004802C))
#define C2RDB		C2RDB_str.reg_canrdb
#define C2RDB_bit	C2RDB_str.bit_canrdb
#define C2TFI1_str (*((volatile cantfi_bits21*)0xE0048030))
#define C2TFI1		C2TFI1_str.reg_cantfi
#define C2TFI1_bit	C2TFI1_str.bit_cantfi
#define C2TID1_str (*((volatile cantid_bits21*)0xE0048034))
#define C2TID1		 	  C2TID1_str.reg_cantid
#define C2TID1_ID10_0     C2TID1_str.bit_cantid.ID10.ID10_0
#define C2TID1_ID29_18    C2TID1_str.bit_cantid.ID2918.ID29_18
#define C2TID1_ID29_0     C2TID1_str.bit_cantid.ID290.ID29_0
#define C2TDA1_str (*((volatile cantda_bits21*)0xE0048038))
#define C2TDA1		C2TDA1_str.reg_cantda
#define C2TDA1_bit	C2TDA1_str.bit_cantda
#define C2TDB1_str (*((volatile cantdb_bits21*)0xE004803C))
#define C2TDB1		C2TDB1_str.reg_cantdb
#define C2TDB1_bit	C2TDB1_str.bit_cantdb
#define C2TFI2_str (*((volatile cantfi_bits22*)0xE0048040))
#define C2TFI2		C2TFI2_str.reg_cantfi
#define C2TFI2_bit	C2TFI2_str.bit_cantfi
#define C2TID2_str (*((volatile cantid_bits22*)0xE0048044)) 
#define C2TID2		 	  C2TID2_str.reg_cantid
#define C2TID2_ID10_0     C2TID2_str.bit_cantid.ID10.ID10_0
#define C2TID2_ID29_18    C2TID2_str.bit_cantid.ID2918.ID29_18
#define C2TID2_ID29_0     C2TID2_str.bit_cantid.ID290.ID29_0
#define C2TDA2_str (*((volatile cantda_bits22*)0xE0048048))
#define C2TDA2		C2TDA2_str.reg_cantda
#define C2TDA2_bit	C2TDA2_str.bit_cantda
#define C2TDB2_str (*((volatile cantdb_bits22*)0xE004804C))
#define C2TDB2		C2TDB2_str.reg_cantdb
#define C2TDB2_bit	C2TDB2_str.bit_cantdb
#define C2TFI3_str (*((volatile cantfi_bits23*)0xE0048050))
#define C2TFI3		C2TFI3_str.reg_cantfi
#define C2TFI3_bit	C2TFI3_str.bit_cantfi
#define C2TID3_str (*((volatile cantid_bits23*)0xE0048054))
#define C2TID3		 	  C2TID3_str.reg_cantid
#define C2TID3_ID10_0     C2TID3_str.bit_cantid.ID10.ID10_0
#define C2TID3_ID29_18    C2TID3_str.bit_cantid.ID2918.ID29_18
#define C2TID3_ID29_0     C2TID3_str.bit_cantid.ID290.ID29_0
#define C2TDA3_str (*((volatile cantda_bits23*)0xE0048058))
#define C2TDA3		C2TDA3_str.reg_cantda
#define C2TDA3_bit	C2TDA3_str.bit_cantda
#define C2TDB3_str (*((volatile cantdb_bits23*)0xE004805C))
#define C2TDB3		C2TDB3_str.reg_cantdb
#define C2TDB3_bit	C2TDB3_str.bit_cantdb

#define C3MOD_str (*((volatile canmod_bits3*)0xE004C000))
#define C3MOD		C3MOD_str.reg_canmod
#define C3MOD_bit	C3MOD_str.bit_canmod
#define C3CMR_str (*((volatile cancmr_bits3*)0xE004C004))
#define C3CMR		C3CMR_str.reg_cancmr
#define C3CMR_bit	C3CMR_str.bit_cancmr
#define C3GSR_str (*((volatile cangsr_bits3*)0xE004C008))
#define C3GSR		C3GSR_str.reg_cangsr
#define C3GSR_bit	C3GSR_str.bit_cangsr
#define C3ICR_str (*((volatile canicr_bits3*)0xE004C00C))
#define C3ICR		C3ICR_str.reg_canicr
#define C3ICR_bit	C3ICR_str.bit_canicr
#define C3IER_str (*((volatile canier_bits3*)0xE004C010))
#define C3IER		C3IER_str.reg_canier
#define C3IER_bit	C3IER_str.bit_canier
#define C3BTR_str (*((volatile canbtr_bits3*)0xE004C014))
#define C3BTR		C3BTR_str.reg_canbtr
#define C3BTR_bit	C3BTR_str.bit_canbtr
#define C3EWL_str (*((volatile canewl_bits3*)0xE004C018))
#define C3EWL		C3EWL_str.reg_canewl
#define C3EWL_bit	C3EWL_str.bit_canewl
#define C3SR_str (*((volatile cansr_bits3*)0xE004C01C))
#define C3SR		C3SR_str.reg_cansr
#define C3SR_bit	C3SR_str.bit_cansr
#define C3RFS_str (*((volatile canrfs_bits3*)0xE004C020))
#define C3RFS		C3RFS_str.reg_canrfs
#define C3RFS_bit	C3RFS_str.bit_canrfs
#define C3RID_str (*((volatile canrid_bits3*)0xE004C024))
#define C3RID		 	 C3RID_str.reg_canrid
#define C3RID_ID10_0     C3RID_str.bit_canrid.ID10.ID10_0
#define C3RID_ID29_18    C3RID_str.bit_canrid.ID2918.ID29_18
#define C3RID_ID29_0     C3RID_str.bit_canrid.ID290.ID29_0
#define C3RDA_str (*((volatile canrda_bits3*)0xE004C028))
#define C3RDA		C3RDA_str.reg_canrda
#define C3RDA_bit	C3RDA_str.bit_canrda
#define C3RDB_str (*((volatile canrdb_bits3*)0xE004C02C))
#define C3RDB		C3RDB_str.reg_canrdb
#define C3RDB_bit	C3RDB_str.bit_canrdb
#define C3TFI1_str (*((volatile cantfi_bits31*)0xE004C030))
#define C3TFI1		C3TFI1_str.reg_cantfi
#define C3TFI1_bit	C3TFI1_str.bit_cantfi
#define C3TID1_str (*((volatile cantid_bits31*)0xE004C034))
#define C3TID1		 	  C3TID1_str.reg_cantid
#define C3TID1_ID10_0     C3TID1_str.bit_cantid.ID10.ID10_0
#define C3TID1_ID29_18    C3TID1_str.bit_cantid.ID2918.ID29_18
#define C3TID1_ID29_0     C3TID1_str.bit_cantid.ID290.ID29_0
#define C3TDA1_str (*((volatile cantda_bits31*)0xE004C038))
#define C3TDA1		C3TDA1_str.reg_cantda
#define C3TDA1_bit	C3TDA1_str.bit_cantda
#define C3TDB1_str (*((volatile cantdb_bits31*)0xE004C03C))
#define C3TDB1		C3TDB1_str.reg_cantdb
#define C3TDB1_bit	C3TDB1_str.bit_cantdb
#define C3TFI2_str (*((volatile cantfi_bits32*)0xE004C040))
#define C3TFI2		C3TFI2_str.reg_cantfi
#define C3TFI2_bit	C3TFI2_str.bit_cantfi
#define C3TID2_str (*((volatile cantid_bits32*)0xE004C044))
#define C3TID2		 	  C3TID2_str.reg_cantid
#define C3TID2_ID10_0     C3TID2_str.bit_cantid.ID10.ID10_0
#define C3TID2_ID29_18    C3TID2_str.bit_cantid.ID2918.ID29_18
#define C3TID2_ID29_0     C3TID2_str.bit_cantid.ID290.ID29_0
#define C3TDA2_str (*((volatile cantda_bits32*)0xE004C048))
#define C3TDA2		C3TDA2_str.reg_cantda
#define C3TDA2_bit	C3TDA2_str.bit_cantda
#define C3TDB2_str (*((volatile cantdb_bits32*)0xE004C04C))
#define C3TDB2		C3TDB2_str.reg_cantdb
#define C3TDB2_bit	C3TDB2_str.bit_cantdb
#define C3TFI3_str (*((volatile cantfi_bits33*)0xE004C050))
#define C3TFI3		C3TFI3_str.reg_cantfi
#define C3TFI3_bit	C3TFI3_str.bit_cantfi
#define C3TID3_str (*((volatile cantid_bits33*)0xE004C054))
#define C3TID3		 	  C3TID3_str.reg_cantid
#define C3TID3_ID10_0     C3TID3_str.bit_cantid.ID10.ID10_0
#define C3TID3_ID29_18    C3TID3_str.bit_cantid.ID2918.ID29_18
#define C3TID3_ID29_0     C3TID3_str.bit_cantid.ID290.ID29_0
#define C3TDA3_str (*((volatile cantda_bits33*)0xE004C058))
#define C3TDA3		C3TDA3_str.reg_cantda
#define C3TDA3_bit	C3TDA3_str.bit_cantda
#define C3TDB3_str (*((volatile cantdb_bits33*)0xE004C05C))
#define C3TDB3		C3TDB3_str.reg_cantdb
#define C3TDB3_bit	C3TDB3_str.bit_cantdb

#define C4MOD_str (*((volatile canmod_bits4*)0xE0050000))
#define C4MOD		C4MOD_str.reg_canmod
#define C4MOD_bit	C4MOD_str.bit_canmod
#define C4CMR_str (*((volatile cancmr_bits4*)0xE0050004))
#define C4CMR		C1CMR_str.reg_cancmr
#define C4CMR_bit	C1CMR_str.bit_cancmr
#define C4GSR_str (*((volatile cangsr_bits4*)0xE0050008))
#define C4GSR		C4GSR_str.reg_cangsr
#define C4GSR_bit	C4GSR_str.bit_cangsr
#define C4ICR_str (*((volatile canicr_bits4*)0xE005000C))
#define C4ICR		C4ICR_str.reg_canicr
#define C4ICR_bit	C4ICR_str.bit_canicr
#define C4IER_str (*((volatile canier_bits4*)0xE0050010))
#define C4IER		C4IER_str.reg_canier
#define C4IER_bit	C4IER_str.bit_canier
#define C4BTR_str (*((volatile canbtr_bits4*)0xE0050014))
#define C4BTR		C4BTR_str.reg_canbtr
#define C4BTR_bit	C4BTR_str.bit_canbtr
#define C4EWL_str (*((volatile canewl_bits4*)0xE0050018))
#define C4EWL		C4EWL_str.reg_canewl
#define C4EWL_bit	C4EWL_str.bit_canewl
#define C4SR_str (*((volatile cansr_bits4*)0xE005001C))
#define C4SR		C4SR_str.reg_cansr
#define C4SR_bit	C4SR_str.bit_cansr
#define C4RFS_str (*((volatile canrfs_bits4*)0xE0050020))
#define C4RFS		C4RFS_str.reg_canrfs
#define C4RFS_bit	C4RFS_str.bit_canrfs
#define C4RID_str (*((volatile canrid_bits4*)0xE0050024))
#define C4RID		 	 C4RID_str.reg_canrid
#define C4RID_ID10_0     C4RID_str.bit_canrid.ID10.ID10_0
#define C4RID_ID29_18    C4RID_str.bit_canrid.ID2918.ID29_18
#define C4RID_ID29_0     C4RID_str.bit_canrid.ID290.ID29_0
#define C4RDA_str (*((volatile canrda_bits4*)0xE0050028))
#define C4RDA		C4RDA_str.reg_canrda
#define C4RDA_bit	C4RDA_str.bit_canrda
#define C4RDB_str (*((volatile canrdb_bits4*)0xE005002C))
#define C4RDB		C4RDB_str.reg_canrdb
#define C4RDB_bit	C4RDB_str.bit_canrdb
#define C4TFI1_str (*((volatile cantfi_bits41*)0xE0050030))
#define C4TFI1		C4TFI1_str.reg_cantfi
#define C4TFI1_bit	C4TFI1_str.bit_cantfi
#define C4TID1_str (*((volatile cantid_bits41*)0xE0050034))
#define C4TID1		 	  C4TID1_str.reg_cantid
#define C4TID1_ID10_0     C4TID1_str.bit_cantid.ID10.ID10_0
#define C4TID1_ID29_18    C4TID1_str.bit_cantid.ID2918.ID29_18
#define C4TID1_ID29_0     C4TID1_str.bit_cantid.ID290.ID29_0
#define C4TDA1_str (*((volatile cantda_bits41*)0xE0050038))
#define C4TDA1		C4TDA1_str.reg_cantda
#define C4TDA1_bit	C4TDA1_str.bit_cantda
#define C4TDB1_str (*((volatile cantdb_bits41*)0xE005003C))
#define C4TDB1		C4TDB1_str.reg_cantdb
#define C4TDB1_bit	C4TDB1_str.bit_cantdb
#define C4TFI2_str (*((volatile cantfi_bits42*)0xE0050040))
#define C4TFI2		C4TFI2_str.reg_cantfi
#define C4TFI2_bit	C4TFI2_str.bit_cantfi
#define C4TID2_str (*((volatile cantid_bits42*)0xE0050044))
#define C4TID2		 	  C4TID2_str.reg_cantid
#define C4TID2_ID10_0     C4TID2_str.bit_cantid.ID10.ID10_0
#define C4TID2_ID29_18    C4TID2_str.bit_cantid.ID2918.ID29_18
#define C4TID2_ID29_0     C4TID2_str.bit_cantid.ID290.ID29_0
#define C4TDA2_str (*((volatile cantda_bits42*)0xE0050048))
#define C4TDA2		C4TDA2_str.reg_cantda
#define C4TDA2_bit	C4TDA2_str.bit_cantda
#define C4TDB2_str (*((volatile cantdb_bits42*)0xE005004C))
#define C4TDB2		C4TDB2_str.reg_cantdb
#define C4TDB2_bit	C4TDB2_str.bit_cantdb
#define C4TFI3_str (*((volatile cantfi_bits43*)0xE0050050))
#define C4TFI3		C4TFI3_str.reg_cantfi
#define C4TFI3_bit	C4TFI3_str.bit_cantfi
#define C4TID3_str (*((volatile cantid_bits43*)0xE0050054))
#define C4TID3		 	  C4TID3_str.reg_cantid
#define C4TID3_ID10_0     C4TID3_str.bit_cantid.ID10.ID10_0
#define C4TID3_ID29_18    C4TID3_str.bit_cantid.ID2918.ID29_18
#define C4TID3_ID29_0     C4TID3_str.bit_cantid.ID290.ID29_0
#define C4TDA3_str (*((volatile cantda_bits43*)0xE0050058))
#define C4TDA3		C4TDA3_str.reg_cantda
#define C4TDA3_bit	C4TDA3_str.bit_cantda
#define C4TDB3_str (*((volatile cantdb_bits43*)0xE005005C))
#define C4TDB3		C4TDB3_str.reg_cantdb
#define C4TDB3_bit	C4TDB3_str.bit_cantdb

/***************************************************************************
 **
 ** TIMER0
 **
 ***************************************************************************/
#define T0IR_str (*((volatile ir_bits0*)0xE0004000))
#define T0IR	    	T0IR_str.reg_ir
#define T0IR_bit	    T0IR_str.bit_ir
#define T0TCR_str (*((volatile tcr_bits0*)0xE0004004))
#define T0TCR	    	T0TCR_str.reg_tcr
#define T0TCR_bit	    T0TCR_str.bit_tcr
#define T0TC (*((REG32 *)0xE0004008))
#define T0PR (*((REG32 *)0xE000400c))
#define T0PC (*((REG32 *)0xE0004010))
#define T0MCR_str (*((volatile mcr_bits0*)0xE0004014))
#define T0MCR	    	T0MCR_str.reg_mcr
#define T0MCR_bit	    T0MCR_str.bit_mcr
#define T0MR0 (*((REG32 *)0xE0004018))
#define T0MR1 (*((REG32 *)0xE000401C))
#define T0MR2 (*((REG32 *)0xE0004020))
#define T0MR3 (*((REG32 *)0xE0004024))
#define T0CCR_str (*((volatile ccr0_bits*)0xE0004028))
#define T0CCR	    	T0CCR_str.reg_ccr0
#define T0CCR_bit	    T0CCR_str.bit_ccr0
#define T0CR0 (*((REG32 *)0xE000402C))
#define T0CR1 (*((REG32 *)0xE0004030))
#define T0CR2 (*((REG32 *)0xE0004034))
#define T0CR3 (*((REG32 *)0xE0004038))
#define T0EMR_str (*((volatile emr_bits0*)0xE000403c))
#define T0EMR	    	T0EMR_str.reg_emr
#define T0EMR_bit	    T0EMR_str.bit_emr
#define T0CTCR_str (*((volatile ctcr_bits0*)0xE0004070))
#define T0CTCR	    	T0CTCR_str.reg_ctcr
#define T0CTCR_bit	    T0CTCR_str.bit_ctcr


/***************************************************************************
 **
 ** TIMER1
 **
 ***************************************************************************/
#define T1IR_str (*((volatile ir_bits1*)0xE0008000))
#define T1IR	    	T1IR_str.reg_ir
#define T1IR_bit	    T1IR_str.bit_ir
#define T1TCR_str (*((volatile tcr_bits1*)0xE0008004))
#define T1TCR	    	T1TCR_str.reg_tcr
#define T1TCR_bit	    T1TCR_str.bit_tcr
#define T1TC (*((REG32 *)0xE0008008))
#define T1PR (*((REG32 *)0xE000800c))
#define T1PC (*((REG32 *)0xE0008010))
#define T1MCR_str (*((volatile mcr_bits1*)0xE0008014))
#define T1MCR	    	T1MCR_str.reg_mcr
#define T1MCR_bit	    T1MCR_str.bit_mcr
#define T1MR0 (*((REG32 *)0xE0008018))
#define T1MR1 (*((REG32 *)0xE000801C))
#define T1MR2 (*((REG32 *)0xE0008020))
#define T1MR3 (*((REG32 *)0xE0008024))
#define T1CCR_str (*((volatile ccr1_bits*)0xE0008028))
#define T1CCR	    	T1CCR_str.reg_ccr1
#define T1CCR_bit	    T1CCR_str.bit_ccr1
#define T1CR0 (*((REG32 *)0xE000802C))
#define T1CR1 (*((REG32 *)0xE0008030))
#define T1CR2 (*((REG32 *)0xE0008034))
#define T1CR3 (*((REG32 *)0xE0008038))
#define T1EMR_str (*((volatile emr_bits1*)0xE000803c))
#define T1EMR	    	T1EMR_str.reg_emr
#define T1EMR_bit	    T1EMR_str.bit_emr
#define T1CTCR_str (*((volatile ctcr_bits1*)0xE0008070))
#define T1CTCR	    	T1CTCR_str.reg_ctcr
#define T1CTCR_bit	    T1CTCR_str.bit_ctcr


/***************************************************************************
 **
 ** PWM
 **
 ***************************************************************************/
#define PWMIR_str (*((volatile pwmir_bits*)0xE0014000))
#define PWMIR	    	PWMIR_str.reg_pwmir
#define PWMIR_bit	    PWMIR_str.bit_pwmir
#define PWMTCR_str (*((volatile pwmtcr_bits*)0xE0014004))
#define PWMTCR	    	PWMTCR_str.reg_pwmtcr
#define PWMTCR_bit	    PWMTCR_str.bit_pwmtcr
#define PWMTC (*((REG32 *)0xE0014008))
#define PWMPR (*((REG32 *)0xE001400C))
#define PWMPC (*((REG32 *)0xE0014010))
#define PWMMCR_str (*((volatile pwmmcr_bits*)0xE0014014))
#define PWMMCR	    	PWMMCR_str.reg_pwmmcr
#define PWMMCR_bit	    PWMMCR_str.bit_pwmmcr
#define PWMMR0 (*((REG32 *)0xE0014018))
#define PWMMR1 (*((REG32 *)0xE001401C))
#define PWMMR2 (*((REG32 *)0xE0014020))
#define PWMMR3 (*((REG32 *)0xE0014024))
#define PWMMR4 (*((REG32 *)0xE0014040))
#define PWMMR5 (*((REG32 *)0xE0014044))
#define PWMMR6 (*((REG32 *)0xE0014048))
#define PWMPCR_str (*((volatile pwmpcr_bits*)0xE001404C))
#define PWMPCR	    	PWMPCR_str.reg_pwmpcr
#define PWMPCR_bit	    PWMPCR_str.bit_pwmpcr
#define PWMLER_str (*((volatile pwmler_bits*)0xE0014050))
#define PWMLER	    	PWMLER_str.reg_pwmler
#define PWMLER_bit	    PWMLER_str.bit_pwmler


/***************************************************************************
 **
 ** A/D Converter
 **
 ***************************************************************************/
#define ADCR_str (*((volatile adcr_bits*)0xE0034000))
#define ADCR	    	ADCR_str.reg_adcr
#define ADCR_bit	    ADCR_str.bit_adcr
#define ADDR_str (*((volatile addr_bits*)0xE0034004))
#define ADDR	    	ADDR_str.reg_addr
#define ADDR_bit	    ADDR_str.bit_addr
#define ADDR0_str (*((volatile addr0_bits*) 0xE0034010))
#define ADDR0			ADDR0_str.reg_addr0
#define ADDR0_bit		ADDR0_str.bit_addr0
#define ADDR1_str (*((volatile addr1_bits*) 0xE0034014))
#define ADDR1			ADDR1_str.reg_addr1
#define ADDR1_bit		ADDR1_str.bit_addr1
#define ADDR2_str (*((volatile addr2_bits*) 0xE0034018))
#define ADDR2			ADDR2_str.reg_addr2
#define ADDR2_bit		ADDR2_str.bit_addr2
#define ADDR3_str (*((volatile addr3_bits*) 0xE003401C))
#define ADDR3			ADDR3_str.reg_addr3
#define ADDR3_bit		ADDR3_str.bit_addr3
#define ADINTEN   (*((volatile unsigned long *) 0xE003400C))


/***************************************************************************
 **
 ** RTC
 **
 ***************************************************************************/
#define ILR_str (*((volatile ilr_bits*)0xE0024000))
#define ILR		   ILR_str.reg_ilr
#define ILR_bit	   ILR_str.bit_ilr
#define CTC_str (*((volatile ctc_bits*)0xE0024004))
#define CTC		   CTC_str.reg_ctc
#define CTC_bit	   CTC_str.bit_ctc
#define CCR_str (*((volatile rtcccr_bits*)0xE0024008))
#define CCR		   CCR_str.reg_rtcccr
#define CCR_bit	   CCR_str.bit_rtcccr
#define CIIR_str (*((volatile ciir_bits*)0xE002400C))
#define CIIR		   CIIR_str.reg_ciir
#define CIIR_bit	   CIIR_str.bit_ciir
#define AMR_str (*((volatile amr_bits*)0xE0024010)) 
#define AMR		   AMR_str.reg_amr
#define AMR_bit	   AMR_str.bit_amr
#define CTIME0_str (*((volatile ctime0_bits*)0xE0024014))
#define CTIME0		   CTIME0_str.reg_ctime0
#define CTIME0_bit	   CTIME0_str.bit_ctime0
#define CTIME1_str (*((volatile ctime1_bits*)0xE0024018))	 
#define CTIME1		   CTIME1_str.reg_ctime1
#define CTIME1_bit	   CTIME1_str.bit_ctime1
#define CTIME2_str (*((volatile ctime2_bits*)0xE002401C))	
#define CTIME2		   CTIME2_str.reg_ctime2
#define CTIME2_bit	   CTIME2_str.bit_ctime2
#define SEC_str (*((volatile sec_bits1*)0xE0024020)) 
#define SEC		   SEC_str.reg_sec
#define SEC_bit	   SEC_str.bit_sec
#define MIN_str (*((volatile min_bits1*)0xE0024024))	 
#define MIN		   MIN_str.reg_min
#define MIN_bit	   MIN_str.bit_min
#define HOUR_str (*((volatile hour_bits1*)0xE0024028))  
#define HOUR		   HOUR_str.reg_hour
#define HOUR_bit	   HOUR_str.bit_hour
#define DOM_str (*((volatile dom_bits1*)0xE002402C))
#define DOM		   DOM_str.reg_dom
#define DOM_bit	   DOM_str.bit_dom
#define DOW_str (*((volatile dow_bits1*)0xE0024030)) 
#define DOW		   DOW_str.reg_dow
#define DOW_bit	   DOW_str.bit_dow
#define DOY_str (*((volatile doy_bits1*)0xE0024034)) 
#define DOY		   DOY_str.reg_doy
#define DOY_bit	   DOY_str.bit_doy
#define MONTH_str (*((volatile month_bits1*)0xE0024038))	
#define MONTH		   MONTH_str.reg_month
#define MONTH_bit	   MONTH_str.bit_month
#define YEAR_str (*((volatile year_bits1*)0xE002403C))   
#define YEAR		   YEAR_str.reg_year
#define YEAR_bit	   YEAR_str.bit_year
#define ALSEC_str (*((volatile sec_bits2*)0xE0024060)) 
#define ALSEC		   ALSEC_str.reg_sec
#define ALSEC_bit	       ALSEC_str.bit_sec
#define ALMIN_str (*((volatile min_bits2*)0xE0024064))  
#define ALMIN		   ALMIN_str.reg_min
#define ALMIN_bit	       ALMIN_str.bit_min
#define ALHOUR_str (*((volatile hour_bits2*)0xE0024068))	
#define ALHOUR		   ALHOUR_str.reg_hour
#define ALHOUR_bit	   ALHOUR_str.bit_hour
#define ALDOM_str (*((volatile dom_bits2*)0xE002406C))  
#define ALDOM		   ALDOM_str.reg_dom
#define ALDOM_bit	   ALDOM_str.bit_dom
#define ALDOW_str (*((volatile dow_bits2*)0xE0024070))   
#define ALDOW		   ALDOW_str.reg_dow
#define ALDOW_bit	   ALDOW_str.bit_dow
#define ALDOY_str (*((volatile doy_bits2*)0xE0024074))   
#define ALDOY		   ALDOY_str.reg_doy
#define ALDOY_bit	   ALDOY_str.bit_doy
#define ALMON_str (*((volatile month_bits2*)0xE0024078))	 
#define ALMON		   ALMON_str.reg_month
#define ALMON_bit	   ALMON_str.bit_month
#define ALYEAR_str (*((volatile year_bits2*)0xE002407C))	
#define ALYEAR		   ILR_str.reg_year
#define ALYEAR_bit	   ILR_str.bit_year
#define PREINT_str (*((volatile preint_bits*)0xE0024080))	
#define PREINT		   PREINT_str.reg_preint
#define PREINT_bit	   PREINT_str.bit_preint
#define PREFRAC_str (*((volatile prefrac_bits*)0xE0024084))  
#define PREFRAC		   PREFRAC_str.reg_prefrac
#define PREFRAC_bit	   PREFRAC_str.bit_prefrac

/***************************************************************************
 **
 ** Watchdog
 **
 ***************************************************************************/
#define WDMOD_str (*((volatile wdmod_bits*)0xE0000000))
#define WDMOD	    	WDMOD_str.reg_wdmod
#define WDMOD_bit	    WDMOD_str.bit_wdmod
#define WDTC (*((REG32 *)0xE0000004))
#define WDFEED_str (*((volatile wdfeed_bits*)0xE0000008))
#define WDFEED	    	WDFEED_str.reg_wdfeed
#define WDFEED_bit	    WDFEED_str.bit_wdfeed
#define WDTV (*((REG32 *)0xE000000C))

/***************************************************************************
 **
 **  Interrupt vector table
 **
 ***************************************************************************/
#define RESETV  0x00  /* Reset                           */
#define UNDEFV  0x04  /* Undefined instruction           */
#define SWIV    0x08  /* Software interrupt              */
#define PABORTV 0x0c  /* Prefetch abort                  */
#define DABORTV 0x10  /* Data abort                      */
#define IRQV    0x18  /* Normal interrupt                */
#define FIQV    0x1c  /* Fast interrupt                  */

/***************************************************************************
 **
 **  VIC Interrupt channels
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
#define VIC_AD          18  /* External interrupt 2 (EINT2)       */
#define VIC_CAN_AF      19  /* CAN and Acceptance Filter          */
#define VIC_CAN1TX      20  /* CAN1 Tx                            */
#define VIC_CAN2TX      21  /* CAN2 Tx                            */
//#define VIC_RES       22  /* Reserved                           */
//#define VIC_RES       23  /* Reserved                           */
//#define VIC_RES       24  /* Reserved                           */
//#define VIC_RES       25  /* Reserved                           */
#define VIC_CAN1RX      26  /* CAN1 Rx                            */
#define VIC_CAN2RX      27  /* CAN2 Rx                            */
//#define VIC_RES       28  /* Reserved                           */
//#define VIC_RES       29  /* Reserved                           */
//#define VIC_RES       30  /* Reserved                           */
//#define VIC_RES       31  /* Reserved                           */

#endif    /* LPC2194_01_H */


