;/*****************************************************************************/
;/* LPC32x0.S: Startup file for Philips LPC32x0 device series                 */
;/*****************************************************************************/
;/* <<< Use Configuration Wizard in Context Menu >>>                          */ 
;/*****************************************************************************/
;/* This file is part of the uVision/ARM development tools.                   */
;/* Copyright (c) 2005-2008 Keil Software. All rights reserved.               */
;/* This software may only be used under the terms of a valid, current,       */
;/* end user licence from KEIL for a compatible version of KEIL software      */
;/* development tools. Nothing else gives you the right to use this software. */
;/*****************************************************************************/


;/*
; *  The LPC32x0.S code is executed after CPU Reset. This file may be 
; *  translated with the following SET symbols. In uVision these SET 
; *  symbols are entered under Options - ASM - Define.
; *
; *  EMC_NO_INIT:   when set, the external memory controller is not initialized 
; *                 in startup (if 2-nd level bootloader is used this should 
; *                 set as bootloader already initializes SDRAM).
; *
; *  EMC_DYNAMIC_NO_INIT: when set, the dynamic memory interface of the 
; *                 external memory controller is not initialized in startup 
; *
; *  EMC_STATIC_NO_INIT:  when set, the static memory interface of the 
; *                 external memory controller is not initialized in startup 
; *
; *  NAND_NO_INIT:  when set, the NAND Flash controller is not initialized in 
; *                 startup (if 2-nd level bootloader is used this should 
; *                 be set as bootloader already initializes NAND Flash 
; *                 controller).
; *
; *  RAM_INTVEC:    when set, the startup code copies exception vectors 
; *                 from on-chip Flash to on-chip RAM.
; *
; *  REMAP:         when set, the startup code remaps exception vectors from
; *                 on-chip RAM to address 0.
; *
; */


; Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs

Mode_USR        EQU     0x10
Mode_FIQ        EQU     0x11
Mode_IRQ        EQU     0x12
Mode_SVC        EQU     0x13
Mode_ABT        EQU     0x17
Mode_UND        EQU     0x1B
Mode_SYS        EQU     0x1F

I_Bit           EQU     0x80            ; when I bit is set, IRQ is disabled
F_Bit           EQU     0x40            ; when F bit is set, FIQ is disabled


;----------------------- Memory Definitions ------------------------------------

; Internal Memory Base Addresses
IRAM_BASE       EQU     0x08000000
IROM_BASE       EQU     0x0C000000

; External Memory Base Addresses
DYN_MEM0_BASE   EQU     0x80000000   
DYN_MEM1_BASE   EQU     0xA0000000
STA_MEM0_BASE   EQU     0xE0000000
STA_MEM1_BASE   EQU     0xE1000000
STA_MEM2_BASE   EQU     0xE2000000
STA_MEM3_BASE   EQU     0xE3000000


; System Control User Interface
SYSTEM_BASE     EQU     0x40004000      ; System Control          Base Address
BOOT_MAP_OFS    EQU     0x14            ; Boot Map Control Reg    Address Offset
SDRAMCLK_CTRL_OFS EQU   0x68            ; SDRAM Clock Control Reg Address Offset
DDR_LAP_NOM_OFS   EQU   0x6C            ; DDR DQS Nominal Value   Address Offset
DDR_LAP_COUNT_OFS EQU   0x70            ; DDR Ring Osc Counter    Address Offset
DDR_CAL_DELAY_OFS EQU   0x74            ; DDR DQS Calibrate Value Address Offset

; Constants
REMAP_BIT       EQU     (1<<0)          ; Remap RAM to 0

;----------------------- Stack and Heap Definitions ----------------------------

;// <h> Stack Configuration (Stack Sizes in Bytes)
;//   <o0> Undefined Mode      <0x0-0xFFFFFFFF:8>
;//   <o1> Supervisor Mode     <0x0-0xFFFFFFFF:8>
;//   <o2> Abort Mode          <0x0-0xFFFFFFFF:8>
;//   <o3> Fast Interrupt Mode <0x0-0xFFFFFFFF:8>
;//   <o4> Interrupt Mode      <0x0-0xFFFFFFFF:8>
;//   <o5> User/System Mode    <0x0-0xFFFFFFFF:8>
;// </h>

UND_Stack_Size  EQU     0x00000000
SVC_Stack_Size  EQU     0x00000008
ABT_Stack_Size  EQU     0x00000000
FIQ_Stack_Size  EQU     0x00000080
IRQ_Stack_Size  EQU     0x00000080
USR_Stack_Size  EQU     0x00000400

ISR_Stack_Size  EQU     (UND_Stack_Size + SVC_Stack_Size + ABT_Stack_Size + \
                         FIQ_Stack_Size + IRQ_Stack_Size)

                AREA    STACK, NOINIT, READWRITE, ALIGN=3

Stack_Mem       SPACE   USR_Stack_Size
__initial_sp    SPACE   ISR_Stack_Size
Stack_Top


;// <h> Heap Configuration
;//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF>
;// </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


;----------------------- Clock Definitions -------------------------------------

; Clock User Interface
PWR_CTRL_OFS     EQU    0x44            ; Power Control Register  Address Offset
OSC_CTRL_OFS     EQU    0x4C            ; Main Oscilator Ctrl Reg Address Offset
SYSCLK_CTRL_OFS  EQU    0x50            ; SYSCLK Control Register Address Offset
PLL397_CTRL_OFS  EQU    0x48            ; PLL397 Control Register Address Offset
HCLKPLL_CTRL_OFS EQU    0x58            ; ARM and HCLK Ctrl Reg   Address Offset
HCLKDIV_CTRL_OFS EQU    0x40            ; HCLK Divider Settings   Address Offset

; Constants
NORMAL_RUN_BIT   EQU    (1<<2)          ; Run mode control                   bit
SYSCLK_PLL_BIT   EQU    (1<<1)          ; PLL397 used for system clock       bit
PLL_LOCK_BIT     EQU    (1<<0)          ; PLL locked                         bit
HCLKPLL_PD_BIT   EQU    (1<<16)         ; HCLK PLL power down mode           bit

;// <e> Clock Configuration
CLOCK_SETUP      EQU    1

;//   <h> Main Oscillator Control Register (OSC_CTRL)
;//     <e0.0>                     Main Oscillator Disable
;//     </e>
;//   </h>
OSC_CTRL_Val     EQU    0x00000000

;//   <h> SYSCLK Control Register (SYSCLK_CTRL)
;//     <o0.2..11>                 Bad Phase Length Before Clock Switching Trigger <0x000-0x3FF>
;//     <o0.1>                     Oscillator Switch
;//                   <0=> Switch to main oscillator
;//                   <1=> Switch to 13MHz clock source (PLL397 output)
;//   </h>
SYSCLK_CTRL_Val  EQU    0x00000140

;//   <h> PLL397 Control Register (PLL397_CTRL)
;//     <o0.1>                     PLL397 Operational Control
;//                   <0=> PLL397 is running
;//                   <1=> PLL397 is stopped and is in low power mode
;//   </h>
PLL397_CTRL_Val  EQU    0x00000000

;//   <h> HCLK PLL Control Register (HCLKPLL_CTRL)
;//     <i> Example for 208MHz PLL from 13MHz oscillator
;//     <i> settings:
;//     <i> - PLL Power Down: PLL in operating mode
;//     <i> - Bypass Control: CCO clock is sent to post-divider
;//     <i> - Direct Output Control: CCO clock is the direct output of the PLL
;//     <i> - Feedback Divider Path Control: don't care
;//     <i> - PLL Post-divider Settings (P): don't care
;//     <i> - PLL Pre-divider Settings (N): 1
;//     <i> - PLL Feedback Divider (M): 16
;//     <o0.16>                    PLL Power Down
;//                   <0=> PLL in power down mode
;//                   <1=> PLL in operating mode
;//     <o0.15>                    Bypass Control
;//                   <0=> CCO clock is sent to post-divider
;//                   <1=> PLL input bypasses the CCO
;//     <o0.14>                    Direct Output Control
;//                   <0=> The output of the post-divider is output of the PLL
;//                   <1=> CCO clock is the direct output of the PLL
;//     <o0.13>                    Feedback Divider Path Control
;//                   <0=> Clocked by CCO clock
;//                   <1=> Clocked by PLL_CLKOUT
;//     <o0.11..12>                PLL Post-divider Settings (P)
;//                   <0=> / 2 (P=1)
;//                   <1=> / 4 (P=2)
;//                   <2=> / 8 (P=4)
;//                   <3=> / 16 (P=8)
;//     <o0.9..10>                 PLL Pre-divider Settings (N)
;//                   <0=> 1
;//                   <1=> 2
;//                   <2=> 3
;//                   <3=> 4
;//     <o0.1..8>                  PLL Feedback Divider (M) 
;//                   <1-256><#-1>
;//   </h>
HCLKPLL_CTRL_Val EQU    0x0001401E

;//   <h> HCLK Divider Control Register (HCLKDIV_CTRL)
;//     <o0.7..8>                  DDRAM_CLK Control
;//                   <0=> Stopped
;//                   <1=> Nominal speed
;//                   <2=> Half speed
;//     <o0.2..6>                  PERIPH_CLK Divider Control <1-32><#-1>
;//                   <i>  PERIPH_CLK = ARM PLL clock / value
;//     <o0.0..1>                  HCLK Divider Control
;//                   <0=> HCLK = ARM PLL clock
;//                   <1=> HCLK = ARM PLL clock / 2
;//                   <2=> HCLK = ARM PLL clock / 4
;//   </h>
HCLKDIV_CTRL_Val EQU    0x0000003D

;// </e> Clock Configuration


;----------------------- EMC Definitions ---------------------------------------

; External Memory Controller (EMC) User Interface
EMC_BASE            EQU     0x31080000  ; EMC Controller          Base Address
EMCControl_OFS      EQU     0x00        ; Memory Controller Contr Address Offset
EMCStatus_OFS       EQU     0x04        ; EMC Status              Address Offset
EMCConfig_OFS       EQU     0x08        ; Mem Controller Oper Cfg Address Offset
EMCDynControl_OFS   EQU     0x20        ; Dynamic Mem Control Reg Address Offset
EMCDynRefresh_OFS   EQU     0x24        ; Dynamic Mem Refresh Reg Address Offset
EMCDynReadCfg_OFS   EQU     0x28        ; Dynamic Mem Read Config Address Offset
EMCDynRP_OFS        EQU     0x30        ; Precharge Cmd Period    Address Offset
EMCDynRAS_OFS       EQU     0x34        ; Active to Prchg Period  Address Offset
EMCDynSREX_OFS      EQU     0x38        ; Self-refresh Exit Time  Address Offset
EMCDynWR_OFS        EQU     0x44        ; Write Recovery Time     Address Offset
EMCDynRC_OFS        EQU     0x48        ; Act to Act Cmd Period   Address Offset
EMCDynRFC_OFS       EQU     0x4C        ; Auto-refresh Period     Address Offset
EMCDynXSR_OFS       EQU     0x50        ; Exit Selfref to Act Cmd Address Offset
EMCDynRRD_OFS       EQU     0x54        ; Active bank A to B      Address Offset
EMCDynMRD_OFS       EQU     0x58        ; Load Mode to Act Cmd    Address Offset
EMCDynCDLR_OFS      EQU     0x5C        ; Last data into Read Cmd Address Offset
EMCStaExtWait_OFS   EQU     0x80        ; Static Mem Extend Wait  Address Offset
EMCDynConfig0_OFS   EQU     0x100       ; SDRAM0 Config Info      Address Offset
EMCDynRasCas0_OFS   EQU     0x104       ; RAS0 and CAS0 Latencies Address Offset
EMCDynConfig1_OFS   EQU     0x120       ; SDRAM1 Config Info      Address Offset
EMCDynRasCas1_OFS   EQU     0x124       ; RAS1 and CAS1 Latencies Address Offset
EMCStaConfig0_OFS   EQU     0x200       ; CS0 Memory Config       Address Offset
EMCStaWaitWen0_OFS  EQU     0x204       ; CS0 to Wr Enable Delay  Address Offset
EMCStaWaitOen0_OFS  EQU     0x208       ; CS0 to Out Enable Delay Address Offset
EMCStaWaitRd0_OFS   EQU     0x20C       ; CS0 to Rd Access Delay  Address Offset
EMCStaWaitPage0_OFS EQU     0x210       ; CS0 Seq Access Delay    Address Offset
EMCStaWaitWr0_OFS   EQU     0x214       ; CS0 to Wr Access Delay  Address Offset
EMCStaWaitTurn0_OFS EQU     0x218       ; CS0 Turnaround Cycles   Address Offset
EMCStaConfig1_OFS   EQU     0x220       ; CS1 Memory Config       Address Offset
EMCStaWaitWen1_OFS  EQU     0x224       ; CS1 to Wr Enable Delay  Address Offset
EMCStaWaitOen1_OFS  EQU     0x228       ; CS1 to Out Enable Delay Address Offset
EMCStaWaitRd1_OFS   EQU     0x22C       ; CS1 to Rd Access Delay  Address Offset
EMCStaWaitPage1_OFS EQU     0x230       ; CS1 Seq Access Delay    Address Offset
EMCStaWaitWr1_OFS   EQU     0x234       ; CS1 to Wr Access Delay  Address Offset
EMCStaWaitTurn1_OFS EQU     0x238       ; CS1 Turnaround Cycles   Address Offset
EMCStaConfig2_OFS   EQU     0x240       ; CS2 Memory Config       Address Offset
EMCStaWaitWen2_OFS  EQU     0x244       ; CS2 to Wr Enable Delay  Address Offset
EMCStaWaitOen2_OFS  EQU     0x248       ; CS2 to Out Enable Delay Address Offset
EMCStaWaitRd2_OFS   EQU     0x24C       ; CS2 to Rd Access Delay  Address Offset
EMCStaWaitPage2_OFS EQU     0x250       ; CS2 Seq Access Delay    Address Offset
EMCStaWaitWr2_OFS   EQU     0x254       ; CS2 to Wr Access Delay  Address Offset
EMCStaWaitTurn2_OFS EQU     0x258       ; CS2 Turnaround Cycles   Address Offset
EMCStaConfig3_OFS   EQU     0x260       ; CS3 Memory Config       Address Offset
EMCStaWaitWen3_OFS  EQU     0x264       ; CS3 to Wr Enable Delay  Address Offset
EMCStaWaitOen3_OFS  EQU     0x268       ; CS3 to Out Enable Delay Address Offset
EMCStaWaitRd3_OFS   EQU     0x26C       ; CS3 to Rd Access Delay  Address Offset
EMCStaWaitPage3_OFS EQU     0x270       ; CS3 Seq Access Delay    Address Offset
EMCStaWaitWr3_OFS   EQU     0x274       ; CS3 to Wr Access Delay  Address Offset
EMCStaWaitTurn3_OFS EQU     0x278       ; CS3 Turnaround Cycles   Address Offset
EMCAHBControl0_OFS  EQU     0x400       ; AHB port 0 Control Reg  Address Offset
EMCAHBStatus0_OFS   EQU     0x404       ; AHB port 0 Status  Reg  Address Offset
EMCAHBTimeOut0_OFS  EQU     0x408       ; AHB port 0 Timeout Reg  Address Offset
EMCAHBControl2_OFS  EQU     0x440       ; AHB port 2 Control Reg  Address Offset
EMCAHBStatus2_OFS   EQU     0x444       ; AHB port 2 Status  Reg  Address Offset
EMCAHBTimeOut2_OFS  EQU     0x448       ; AHB port 2 Timeout Reg  Address Offset
EMCAHBControl3_OFS  EQU     0x460       ; AHB port 3 Control Reg  Address Offset
EMCAHBStatus3_OFS   EQU     0x464       ; AHB port 3 Status  Reg  Address Offset
EMCAHBTimeOut3_OFS  EQU     0x468       ; AHB port 3 Timeout Reg  Address Offset
EMCAHBControl4_OFS  EQU     0x480       ; AHB port 4 Control Reg  Address Offset
EMCAHBStatus4_OFS   EQU     0x484       ; AHB port 4 Status  Reg  Address Offset
EMCAHBTimeOut4_OFS  EQU     0x488       ; AHB port 4 Timeout Reg  Address Offset

SDRAM0_MODE_REG     EQU     0x80018000  ; SDRAM0 Mode Register     Address
SDRAM0_EXT_MODE_REG EQU     0x8102C000  ; SDRAM0 Extended Mode Reg Address
SDRAM1_MODE_REG     EQU     0xA0018000  ; SDRAM1 Mode Register     Address
SDRAM1_EXT_MODE_REG EQU     0xA102C000  ; SDRAM1 Extended Mode Reg Address

; Constants
NORMAL_CMD          EQU     (0x0 << 7)  ; NORMAL        Command
MODE_CMD            EQU     (0x1 << 7)  ; MODE          Command
PALL_CMD            EQU     (0x2 << 7)  ; Precharge All Command
NOP_CMD             EQU     (0x3 << 7)  ; NOP           Command
REFSH_MODE          EQU     (0x1 << 2)  ; Self-refresh mode

;//     External Memory Controller Setup (EMC) ---------------------------------
;// <e> External Memory Controller Setup (EMC)
EMC_SETUP           EQU 1

;//   <h> EMC Control Register (EMCControl)
;//     <i> Controls operation of the memory controller
;//     <o0.2> L: SDRAM Low-power mode enable
;//     <o0.0> E: SDRAM Controller enable
;//   </h>
EMCControl_Val      EQU 0x00000001

;//   <h> EMC Configuration Register (EMCConfig)
;//     <i> Configures operation of the memory controller
;//     <o0.0> Endian mode
;//       <0=> Little-endian
;//       <1=> Big-endian
;//   </h>
EMCConfig_Val       EQU 0x00000000

;//       Dynamic Memory Interface Setup ---------------------------------------
;//   <e> Dynamic Memory Interface Setup
EMC_DYNAMIC_SETUP   EQU 1

;//     <h> SDRAM Clock Control Register (SDRAMCLK_CTRL)
;//       <o0.22> SDRAM_PIN_SPEED3: Slew rate on the pin SDRAM pin CLK
;//                   <0=> Fast
;//                   <1=> Slow
;//                   <i>  Default: Fast
;//       <o0.21> SDRAM_PIN_SPEED2: Slew rate on the pins SDRAM pads A[14:0], CKE, CS_N, RAS_N, CAS_N, WR_N
;//                   <0=> Fast
;//                   <1=> Slow
;//                   <i>  Default: Fast
;//       <o0.20> SDRAM_PIN_SPEED1: Slew rate on the pins SDRAM pads D[31:0], DQM[3:0]
;//                   <0=> Fast
;//                   <1=> Slow
;//                   <i>  Default: Fast
;//       <o0.19> SW_DDR_RESET: Reset SDRAM Controller when writing from 0 to 1
;//                   <0=> No Reset
;//                   <1=> Reset Active
;//                   <i>  Default: No Reset
;//       <o0.14..18> HCLKDELAY_DELAY: Delay of the HCLKDELAY input from the HCLK <0-31>
;//                   <i>  Delay = value programmed * 0.25ns
;//                   <i>  Default: 0
;//       <o0.13> Delay circuitry Adder status
;//                   <0=> No overflow or sign bit
;//                   <1=> Last calibration produced overflow or negative number
;//       <o0.10..12> Sensitivity factor for DDR SDRAM calibration <0-7>
;//                   <i>  Number of bits to shift error value
;//                   <i>  Default: 0
;//       <o0.9> CAL_DELAY: Delay settings for DDR SDRAM
;//                   <0=> Un-calibrated
;//                   <1=> Calibrated
;//                   <i>  Default: Un-calibrated
;//       <o0.8> SW_DDR_CAL: Perform DDR calibration
;//                   <0=> No manual DDR delay calibration
;//                   <1=> Perform a DDR delay calibration
;//                   <i>  Default: No manual DDR delay calibration
;//       <o0.7> RTC_TICK_EN: Automatic DDR delay calibration
;//                   <0=> No
;//                   <1=> Yes, on each RTC TICK
;//                   <i>  Default: No
;//       <o0.2..6> DDR_DQSIN_DELAY: Delay of the DQS input from the DDR SDRAM device
;//                   <0-15>
;//                   <i>  Delay = value programmed * 0.25ns
;//                   <i>  Default: 0
;//       <o0.1> DDR_SEL: Pin multiplexing selection
;//                   <0=> SDR SDRAM used
;//                   <1=> DDR SDRAM used
;//                   <i>  Default: SDR SDRAM used
;//       <o0.0> Clock enable
;//                   <0=> SDRAM HCLK and inverted HCLK enabled
;//                   <1=> All clocks to SDRAM block disabled
;//                   <i>  Default: SDRAM HCLK and inverted HCLK enabled
;//     </h>
SDRAMCLK_CTRL_Val  EQU  0x0001C000

;//     <h> Dynamic Memory Refresh Timer Register (EMCDynamicRefresh)
;//       <i> Configures dynamic memory refresh operation
;//       <o0.0..10> REFRESH: Refresh timer <0x000-0x7FF>
;//         <i> 0 = refresh disabled, 0x01-0x7FF: value * 16 CCLKS
;//     </h>
EMCDynRefresh_Val   EQU 0x00000032

;//     <h> Dynamic Memory Read Configuration Register (EMCDynamicReadConfig)
;//       <i> Configures the dynamic memory read strategy
;//       <o0.12> DRP: DDR-SDRAM read data capture polarity
;//         <0=> Data captured on the negative edge of HCLK
;//         <1=> Data captured on the positive edge of HCLK
;//       <o0.8..9> DRD: DDR-SDRAM read data strategy
;//         <0=> Clock out delayed strategy
;//         <1=> Command delayed strategy
;//         <2=> Command delayed strategy plus one clock cycle
;//         <3=> Command delayed strategy plus two clock cycles
;//       <o0.4> SRP: SDR-SDRAM read data capture polarity
;//         <0=> Data captured on the negative edge of HCLK
;//         <1=> Data captured on the positive edge of HCLK
;//       <o0.0..1> SRD: SDR-SDRAM read data strategy
;//         <0=> Clock out delayed strategy
;//         <1=> Command delayed strategy
;//         <2=> Command delayed strategy plus one clock cycle
;//         <3=> Command delayed strategy plus two clock cycles
;//     </h>
EMCDynReadCfg_Val   EQU 0x00000011

;//     <h> Dynamic Memory Timings
;//       <i> All delays are in clock cycles
;//       <h> Dynamic Memory Precharge Command Period Register (EMCDynamictRP)
;//         <o0.0..3> tRP: Precharge command period <1-16> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tRP
;//       </h>
;//       <h> Dynamic Memory Active to Precharge Command Period Register (EMCDynamictRAS)
;//         <o1.0..3> tRAS: Active to precharge command period <1-16> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tRAS
;//       </h>
;//       <h> Dynamic Memory Self-refresh Exit Time Register (EMCDynamictSREX)
;//         <o2.0..3> tSREX: Self-refresh exit time <1-16> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tSREX 
;//           <i> for devices without this parameter you use the same value as tXSR
;//       </h>
;//       <h> Dynamic Memory Write Recovery Time Register (EMCDynamictWR)
;//         <o3.0..3> tWR: Write recovery time <1-16> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tWR, tDPL, tRWL, or tRDL
;//       </h>
;//       <h> Dynamic Memory Active to Active Command Period Register (EMCDynamictRC)
;//         <o4.0..4> tRC: Active to active command period <1-32> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tRC
;//       </h>
;//       <h> Dynamic Memory Auto-refresh Period Register (EMCDynamictRFC)
;//         <o5.0..4> tRFC: Auto-refresh period and auto-refresh to active command period <1-32> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tRFC or tRC
;//       </h>
;//       <h> Dynamic Memory Exit Self-refresh Register (EMCDynamictXSR)
;//         <o6.0..8> tXSR: Exit self-refresh to active command time <1-256> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tXSR
;//       </h>
;//       <h> Dynamic Memory Active Bank A to Active Bank B Time Register (EMCDynamicRRD)
;//         <o7.0..3> tRRD: Active bank A to active bank B latency <1-16> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tRRD
;//       </h>
;//       <h> Dynamic Memory Load Mode Register to Active Command Time (EMCDynamictMRD)
;//         <o8.0..3> tMRD: Load mode register to active command time <1-16> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tMRD or tRSA
;//       </h>
;//       <h> Dynamic Memory Last Data In to Read Command Time (EMCDynamictCDLR)
;//         <o9.0..3> tCDLR: Last data in to read command time <1-16> <#-1>
;//           <i> This value is normally found in SDRAM data sheets as tCDLR
;//       </h>
;//     </h>
EMCDynRP_Val        EQU 0x00000001
EMCDynRAS_Val       EQU 0x00000004
EMCDynSREX_Val      EQU 0x00000008
EMCDynWR_Val        EQU 0x00000001
EMCDynRC_Val        EQU 0x00000007
EMCDynRFC_Val       EQU 0x00000008
EMCDynXSR_Val       EQU 0x00000008
EMCDynRRD_Val       EQU 0x00000002
EMCDynMRD_Val       EQU 0x00000002
EMCDynCDLR_Val      EQU 0x00000001

;//     <e> Configure External Bus Behaviour for Dynamic CS0 Area
EMC_DYNCS0_SETUP    EQU 1

;//       <h> Dynamic Memory Configuration Register (EMCDynamicConfig0)
;//         <i> Defines the configuration information for the dynamic memory CS0
;//         <o0.20> P: Write protect enable
;//         <o0.14> AM 14: External bus data width
;//           <0=> 16 bit
;//           <1=> 32 bit
;//         <o0.12..13> AM 13..12: External bus memory type
;//           <0=> High-performance
;//           <1=> Low-power SDRAM
;//         <o0.7..11> AM 11..7: External bus address mapping (Row, Bank, Column)
;//           <0x00=> 16 Mb = 2MB (2Mx8), 2 banks, row length = 11, column length = 9
;//           <0x01=> 16 Mb = 2MB (1Mx16), 2 banks, row length = 11, column length = 8
;//           <0x04=> 64 Mb = 8MB (8Mx8), 4 banks, row length = 12, column length = 9
;//           <0x05=> 64 Mb = 8MB (4Mx16), 4 banks, row length = 12, column length = 8
;//           <0x08=> 128 Mb = 16MB (16Mx8), 4 banks, row length = 12, column length = 10
;//           <0x09=> 128 Mb = 16MB (8Mx16), 4 banks, row length = 12, column length = 9
;//           <0x0C=> 256 Mb = 32MB (32Mx8), 4 banks, row length = 13, column length = 10
;//           <0x0D=> 256 Mb = 32MB (16Mx16), 4 banks, row length = 13, column length = 9
;//           <0x10=> 512 Mb = 64MB (64Mx8), 4 banks, row length = 13, column length = 11
;//           <0x11=> 512 Mb = 64MB (32Mx16), 4 banks, row length = 13, column length = 10
;//         <o0.3..4> MD: Memory device
;//           <0=> SDR SDRAM
;//           <2=> Low-power SDR SDRAM
;//           <4=> DDR SDRAM
;//           <6=> Low-power DDR SDRAM
;//       </h>
EMCDynConfig0_Val   EQU 0x00005682

;//       <h> Dynamic Memory RAS & CAS Delay register (EMCDynamicRASCAS0)
;//         <i> Controls the RAS and CAS latencies for the dynamic memory CS0
;//         <o0.7..10 > CAS: CAS latency
;//           <1=> 0.5 clock cycle
;//           <2=> 1 clock cycles
;//           <3=> 1.5 clock cycle
;//           <4=> 2 clock cycles
;//           <5=> 2.5 clock cycle
;//           <6=> 3 clock cycles
;//           <7=> 3.5 clock cycle
;//           <8=> 4 clock cycles
;//           <9=> 4.5 clock cycle
;//           <10=> 5 clock cycles
;//           <11=> 5.5 clock cycle
;//           <12=> 6 clock cycles
;//           <13=> 6.5 clock cycle
;//           <14=> 7 clock cycles
;//           <15=> 7.5 clock cycle
;//         <o0.0..3> RAS: RAS latency (active to read/write delay, in clock cycles) <1-15>
;//       </h>
EMCDynRasCas0_Val   EQU 0x00000303

;//       <e> Configure AHB0 Interface for SDRAM Controller
;//         <h> SDRAM Controller AHB Control Register (EMCAHBControl0)
;//           <o1.0..1> E: AHB Port Buffer Enable
;//         </h>
;//         <h> SDRAM Controller AHB Timeout Register (EMCAHBTime0)
;//           <o2.0..9> AHBTIMEOUT: AHB timeout <0-511>
;//             <i> 0 = timeout disabled, 1-511 value: number of AHB cycles
;//         </h>
;//       </e> Configure AHB0 Interface for SDRAM Controller
EMC_AHB0_SETUP      EQU 1
EMCAHBControl0_Val  EQU 0x00000001
EMCAHBTimeOut0_Val  EQU 0x00000064

;//       <e> Configure AHB2 Interface for SDRAM Controller
;//         <h> SDRAM Controller AHB Control Register (EMCAHBControl2)
;//           <o1.0..1> E: AHB Port Buffer Enable
;//         </h>
;//         <h> SDRAM Controller AHB Timeout Register (EMCAHBTime2)
;//           <o2.0..9> AHBTIMEOUT: AHB timeout <0-511>
;//             <i> 0 = timeout disabled, 1-511 value: number of AHB cycles
;//         </h>
;//       </e> Configure AHB2 Interface for SDRAM Controller
EMC_AHB2_SETUP      EQU 1
EMCAHBControl2_Val  EQU 0x00000001
EMCAHBTimeOut2_Val  EQU 0x00000190

;//       <e> Configure AHB3 Interface for SDRAM Controller
;//         <h> SDRAM Controller AHB Control Register (EMCAHBControl3)
;//           <o1.0..1> E: AHB Port Buffer Enable
;//         </h>
;//         <h> SDRAM Controller AHB Timeout Register (EMCAHBTime3)
;//           <o2.0..9> AHBTIMEOUT: AHB timeout <0-511>
;//             <i> 0 = timeout disabled, 1-511 value: number of AHB cycles
;//         </h>
;//       </e> Configure AHB3 Interface for SDRAM Controller
EMC_AHB3_SETUP      EQU 1
EMCAHBControl3_Val  EQU 0x00000001
EMCAHBTimeOut3_Val  EQU 0x00000190

;//       <e> Configure AHB4 Interface for SDRAM Controller
;//         <h> SDRAM Controller AHB Control Register (EMCAHBControl4)
;//           <o1.0..1> E: AHB Port Buffer Enable
;//         </h>
;//         <h> SDRAM Controller AHB Timeout Register (EMCAHBTime4)
;//           <o2.0..9> AHBTIMEOUT: AHB timeout <0-511>
;//             <i> 0 = timeout disabled, 1-511 value: number of AHB cycles
;//         </h>
;//       </e> Configure AHB4 Interface for SDRAM Controller
EMC_AHB4_SETUP      EQU 1
EMCAHBControl4_Val  EQU 0x00000001
EMCAHBTimeOut4_Val  EQU 0x00000190

;//     </e> End of Dynamic Setup for CS0 Area

;//     <e> Configure External Bus Behaviour for Dynamic CS1 Area
EMC_DYNCS1_SETUP    EQU 0

;//       <h> Dynamic Memory Configuration Register (EMCDynamicConfig1)
;//         <i> Defines the configuration information for the dynamic memory CS1
;//         <o0.20> P: Write protect enable
;//         <o0.7..14> AM: Address Mapping
;//           <0=> 16 bit
;//           <1=> 32 bit
;//         <o0.14> AM 14: External bus data width
;//           <0=> 16 bit
;//           <1=> 32 bit
;//         <o0.12..13> AM 13..12: External bus memory type
;//           <0=> High-performance
;//           <1=> Low-power SDRAM
;//         <o0.7..11> AM 11..7: External bus address mapping (Row, Bank, Column)
;//           <0x00=> 16 Mb = 2MB (2Mx8), 2 banks, row length = 11, column length = 9
;//           <0x01=> 16 Mb = 2MB (1Mx16), 2 banks, row length = 11, column length = 8
;//           <0x04=> 64 Mb = 8MB (8Mx8), 4 banks, row length = 12, column length = 9
;//           <0x05=> 64 Mb = 8MB (4Mx16), 4 banks, row length = 12, column length = 8
;//           <0x08=> 128 Mb = 16MB (16Mx8), 4 banks, row length = 12, column length = 10
;//           <0x09=> 128 Mb = 16MB (8Mx16), 4 banks, row length = 12, column length = 9
;//           <0x0C=> 256 Mb = 32MB (32Mx8), 4 banks, row length = 13, column length = 10
;//           <0x0D=> 256 Mb = 32MB (16Mx16), 4 banks, row length = 13, column length = 9
;//           <0x10=> 512 Mb = 64MB (64Mx8), 4 banks, row length = 13, column length = 11
;//           <0x11=> 512 Mb = 64MB (32Mx16), 4 banks, row length = 13, column length = 10
;//         <o0.3..4> MD: Memory device
;//           <0=> SDR SDRAM
;//           <2=> Low-power SDR SDRAM
;//           <4=> DDR SDRAM
;//           <6=> Low-power DDR SDRAM
;//       </h>
EMCDynConfig1_Val   EQU 0x00000000

;//       <h> Dynamic Memory RAS & CAS Delay register (EMCDynamicRASCAS1)
;//         <i> Controls the RAS and CAS latencies for the dynamic memory CS1
;//         <o0.7..10 > CAS: CAS latency
;//           <1=> 0.5 clock cycle
;//           <2=> 1 clock cycles
;//           <3=> 1.5 clock cycle
;//           <4=> 2 clock cycles
;//           <5=> 2.5 clock cycle
;//           <6=> 3 clock cycles
;//           <7=> 3.5 clock cycle
;//           <8=> 4 clock cycles
;//           <9=> 4.5 clock cycle
;//           <10=> 5 clock cycles
;//           <11=> 5.5 clock cycle
;//           <12=> 6 clock cycles
;//           <13=> 6.5 clock cycle
;//           <14=> 7 clock cycles
;//           <15=> 7.5 clock cycle
;//         <o0.0..3> RAS: RAS latency (active to read/write delay, in clock cycles) <1-15>
;//       </h>
EMCDynRasCas1_Val   EQU 0x00000303

;//     </e> End of Dynamic Setup for CS1 Area

;//   </e> End of Dynamic Setup

;//       Static Memory Interface Setup ----------------------------------------
;//   <e> Static Memory Interface Setup
EMC_STATIC_SETUP    EQU 0

;//         Configure External Bus Behaviour for Static CS0 Area ---------------
;//     <e> Configure External Bus Behaviour for Static CS0 Area
EMC_STACS0_SETUP    EQU 1

;//       <h> Static Memory Configuration Register (EMCStaticConfig0)
;//         <i> Defines the configuration information for the static memory CS0
;//         <o0.20> WP: Write protect enable
;//         <o0.8> EW: Extended wait enable
;//         <o0.7> PB: Byte lane state
;//           <0=> For reads BLSn are HIGH, for writes BLSn are LOW
;//           <1=> For reads BLSn are LOW, for writes BLSn are LOW
;//         <o0.6> PC: Chip select polarity
;//           <0=> Active LOW chip select
;//           <1=> Active HIGH chip select
;//         <o0.3> PM: Page mode enable
;//         <o0.0..1> MW: Memory width
;//           <0=> 8 bit
;//           <1=> 16 bit
;//           <2=> 32 bit
;//       </h>
EMCStaConfig0_Val   EQU 0x00000081

;//       <h> Static Memory Write Enable Delay Register (EMCStaticWaitWen0)
;//         <i> Selects the delay from CS0 to write enable
;//         <o.0..3> WAITWEN: Wait write enable <1-16> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitWen0_Val  EQU 0x00000000

;//       <h> Static Memory Output Enable Delay Register (EMCStaticWaitOen0)
;//         <i> Selects the delay from CS0 or address change, whichever is later, to output enable
;//         <o.0..3> WAITOEN: Wait output enable <0-15>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitOen0_Val  EQU 0x00000000
                                      
;//       <h> Static Memory Read Delay Register (EMCStaticWaitRd0)
;//         <i> Selects the delay from CS0 to a read access
;//         <o.0..4> WAITRD: Non-page mode read wait states or asynchronous page mode read first access wait states <1-32> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitRd0_Val   EQU 0x00000007

;//       <h> Static Memory Page Mode Read Delay Register (EMCStaticWaitPage0)
;//         <i> Selects the delay for asynchronous page mode sequential accesses for CS0
;//         <o.0..4> WAITPAGE: Asynchronous page mode read after the first read wait states <1-32> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitPage0_Val EQU 0x00000000

;//       <h> Static Memory Write Delay Register (EMCStaticWaitWr0)
;//         <i> Selects the delay from CS0 to a write access
;//         <o.0..4> WAITWR: Write wait states <2-33> <#-2>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitWr0_Val   EQU 0x00000000

;//       <h> Static Memory Turn Round Delay Register (EMCStaticWaitTurn0)
;//         <i> Selects the number of bus turnaround cycles for CS0
;//         <o.0..4> WAITTURN: Bus turnaround cycles <1-16> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitTurn0_Val EQU 0x00000000

;//     </e> End of Static Setup for Static CS0 Area

;//         Configure External Bus Behaviour for Static CS1 Area ---------------
;//     <e> Configure External Bus Behaviour for Static CS1 Area
EMC_STACS1_SETUP    EQU 0

;//       <h> Static Memory Configuration Register (EMCStaticConfig1)
;//         <i> Defines the configuration information for the static memory CS1
;//         <o0.20> WP: Write protect enable
;//         <o0.8> EW: Extended wait enable
;//         <o0.7> PB: Byte lane state
;//           <0=> For reads BLSn are HIGH, for writes BLSn are LOW
;//           <1=> For reads BLSn are LOW, for writes BLSn are LOW
;//         <o0.6> PC: Chip select polarity
;//           <0=> Active LOW chip select
;//           <1=> Active HIGH chip select
;//         <o0.3> PM: Page mode enable
;//         <o0.0..1> MW: Memory width
;//           <0=> 8 bit
;//           <1=> 16 bit
;//           <2=> 32 bit
;//       </h>
EMCStaConfig1_Val   EQU 0x00000002

;//       <h> Static Memory Write Enable Delay Register (EMCStaticWaitWen1)
;//         <i> Selects the delay from CS1 to write enable
;//         <o.0..3> WAITWEN: Wait write enable <1-16> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitWen1_Val  EQU 0x00000000

;//       <h> Static Memory Output Enable Delay Register (EMCStaticWaitOen1)
;//         <i> Selects the delay from CS1 or address change, whichever is later, to output enable
;//         <o.0..3> WAITOEN: Wait output enable <0-15>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitOen1_Val  EQU 0x00000000
                                      
;//       <h> Static Memory Read Delay Register (EMCStaticWaitRd1)
;//         <i> Selects the delay from CS1 to a read access
;//         <o.0..4> WAITRD: Non-page mode read wait states or asynchronous page mode read first access wait states <1-32> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitRd1_Val   EQU 0x0000001F

;//       <h> Static Memory Page Mode Read Delay Register (EMCStaticWaitPage1)
;//         <i> Selects the delay for asynchronous page mode sequential accesses for CS1
;//         <o.0..4> WAITPAGE: Asynchronous page mode read after the first read wait states <1-32> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitPage1_Val EQU 0x0000001F

;//       <h> Static Memory Write Delay Register (EMCStaticWaitWr1)
;//         <i> Selects the delay from CS1 to a write access
;//         <o.0..4> WAITWR: Write wait states <2-33> <#-2>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitWr1_Val   EQU 0x0000001F

;//       <h> Static Memory Turn Round Delay Register (EMCStaticWaitTurn1)
;//         <i> Selects the number of bus turnaround cycles for CS1
;//         <o.0..4> WAITTURN: Bus turnaround cycles <1-16> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitTurn1_Val EQU 0x0000000F

;//     </e> End of Static Setup for Static CS1 Area

;//         Configure External Bus Behaviour for Static CS2 Area ---------------
;//     <e> Configure External Bus Behaviour for Static CS2 Area
EMC_STACS2_SETUP    EQU 0

;//       <h> Static Memory Configuration Register (EMCStaticConfig2)
;//         <i> Defines the configuration information for the static memory CS2
;//         <o0.20> WP: Write protect enable
;//         <o0.8> EW: Extended wait enable
;//         <o0.7> PB: Byte lane state
;//           <0=> For reads BLSn are HIGH, for writes BLSn are LOW
;//           <1=> For reads BLSn are LOW, for writes BLSn are LOW
;//         <o0.6> PC: Chip select polarity
;//           <0=> Active LOW chip select
;//           <1=> Active HIGH chip select
;//         <o0.3> PM: Page mode enable
;//         <o0.0..1> MW: Memory width
;//           <0=> 8 bit
;//           <1=> 16 bit
;//           <2=> 32 bit
;//       </h>
EMCStaConfig2_Val   EQU 0x00000002

;//       <h> Static Memory Write Enable Delay Register (EMCStaticWaitWen2)
;//         <i> Selects the delay from CS2 to write enable
;//         <o.0..3> WAITWEN: Wait write enable <1-16> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitWen2_Val  EQU 0x00000000

;//       <h> Static Memory Output Enable Delay Register (EMCStaticWaitOen2)
;//         <i> Selects the delay from CS2 or address change, whichever is later, to output enable
;//         <o.0..3> WAITOEN: Wait output enable <0-15>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitOen2_Val  EQU 0x00000000
                                      
;//       <h> Static Memory Read Delay Register (EMCStaticWaitRd2)
;//         <i> Selects the delay from CS2 to a read access
;//         <o.0..4> WAITRD: Non-page mode read wait states or asynchronous page mode read first access wait states <1-32> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitRd2_Val   EQU 0x0000001F

;//       <h> Static Memory Page Mode Read Delay Register (EMCStaticWaitPage2)
;//         <i> Selects the delay for asynchronous page mode sequential accesses for CS2
;//         <o.0..4> WAITPAGE: Asynchronous page mode read after the first read wait states <1-32> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitPage2_Val EQU 0x0000001F

;//       <h> Static Memory Write Delay Register (EMCStaticWaitWr2)
;//         <i> Selects the delay from CS2 to a write access
;//         <o.0..4> WAITWR: Write wait states <2-33> <#-2>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitWr2_Val   EQU 0x0000001F

;//       <h> Static Memory Turn Round Delay Register (EMCStaticWaitTurn2)
;//         <i> Selects the number of bus turnaround cycles for CS2
;//         <o.0..4> WAITTURN: Bus turnaround cycles <1-16> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitTurn2_Val EQU 0x0000000F

;//     </e> End of Static Setup for Static CS2 Area

;//         Configure External Bus Behaviour for Static CS3 Area ---------------
;//     <e> Configure External Bus Behaviour for Static CS3 Area
EMC_STACS3_SETUP    EQU 0

;//       <h> Static Memory Configuration Register (EMCStaticConfig3)
;//         <i> Defines the configuration information for the static memory CS3
;//         <o0.20> WP: Write protect enable
;//         <o0.8> EW: Extended wait enable
;//         <o0.7> PB: Byte lane state
;//           <0=> For reads BLSn are HIGH, for writes BLSn are LOW
;//           <1=> For reads BLSn are LOW, for writes BLSn are LOW
;//         <o0.6> PC: Chip select polarity
;//           <0=> Active LOW chip select
;//           <1=> Active HIGH chip select
;//         <o0.3> PM: Page mode enable
;//         <o0.0..1> MW: Memory width
;//           <0=> 8 bit
;//           <1=> 16 bit
;//           <2=> 32 bit
;//       </h>
EMCStaConfig3_Val   EQU 0x00000002

;//       <h> Static Memory Write Enable Delay Register (EMCStaticWaitWen3)
;//         <i> Selects the delay from CS3 to write enable
;//         <o.0..3> WAITWEN: Wait write enable <1-16> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitWen3_Val  EQU 0x00000000

;//       <h> Static Memory Output Enable Delay Register (EMCStaticWaitOen3)
;//         <i> Selects the delay from CS3 or address change, whichever is later, to output enable
;//         <o.0..3> WAITOEN: Wait output enable <0-15>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitOen3_Val  EQU 0x00000000
                                      
;//       <h> Static Memory Read Delay Register (EMCStaticWaitRd3)
;//         <i> Selects the delay from CS3 to a read access
;//         <o.0..4> WAITRD: Non-page mode read wait states or asynchronous page mode read first access wait states <1-32> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitRd3_Val   EQU 0x0000001F

;//       <h> Static Memory Page Mode Read Delay Register (EMCStaticWaitPage3)
;//         <i> Selects the delay for asynchronous page mode sequential accesses for CS3
;//         <o.0..4> WAITPAGE: Asynchronous page mode read after the first read wait states <1-32> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitPage3_Val EQU 0x0000001F

;//       <h> Static Memory Write Delay Register (EMCStaticWaitWr3)
;//         <i> Selects the delay from CS3 to a write access
;//         <o.0..4> WAITWR: Write wait states <2-33> <#-2>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitWr3_Val   EQU 0x0000001F

;//       <h> Static Memory Turn Round Delay Register (EMCStaticWaitTurn3)
;//         <i> Selects the number of bus turnaround cycles for CS3
;//         <o.0..4> WAITTURN: Bus turnaround cycles <1-16> <#-1>
;//           <i> The delay is in HCLK cycles
;//       </h>
EMCStaWaitTurn3_Val EQU 0x0000000F

;//     </e> End of Static Setup for Static CS3 Area

;//     <h> Static Memory Extended Wait Register (EMCStaticExtendedWait)
;//       <i> Time long static memory read and write transfers
;//       <o.0..9> EXTENDEDWAIT: Extended wait time out <0-1023>
;//         <i> The delay is in (16 * HCLK) cycles
;//     </h>
EMCStaExtWait_Val   EQU 0x00000000

;//   </e> End of Static Setup

;// </e> End of EMC Setup


;----------------------- NAND Flash Definitions --------------------------------

; NAND Flash Controller (NANDC) User Interface
; Single-level NAND flash controller definitions
FLASHCLK_CTRL_OFS   EQU     0xC8        ; NAND Configuration Reg  Address Offset
                                        
SLC_BASE            EQU     0x20020000  ; SLC NAND Controller     Base Address
                                        ; SLC NAND Flash Registers
SLC_DATA_OFS        EQU     0x00        ; Data           Register Address Offset
SLC_ADDR_OFS        EQU     0x04        ; Address        Register Address Offset
SLC_CMD_OFS         EQU     0x08        ; Command        Register Address Offset
SLC_STOP_OFS        EQU     0x0C        ; STOP           Register Address Offset
SLC_CTRL_OFS        EQU     0x10        ; Control        Register Address Offset
SLC_CFG_OFS         EQU     0x14        ; Configuration  Register Address Offset
SLC_STAT_OFS        EQU     0x18        ; Status         Register Address Offset
SLC_INT_STAT_OFS    EQU     0x1C        ; Interrupt Status    Reg Address Offset
SLC_IEN_OFS         EQU     0x20        ; Interrupt Enable    Reg Address Offset
SLC_ISR_OFS         EQU     0x24        ; Interrupt Set       Reg Address Offset
SLC_ICR_OFS         EQU     0x28        ; Interrupt Clear     Reg Address Offset
SLC_TAC_OFS         EQU     0x2C        ; Read Timing Arc Cfg Reg Address Offset
SLC_TC_OFS          EQU     0x30        ; Transfer Count Register Address Offset
SLC_ECC_OFS         EQU     0x34        ; Parity Bits    Register Address Offset
SLC_DMA_DATA_OFS    EQU     0x38        ; DMA DATA       Register Address Offset

; Multi-level NAND flash controller definitions
MLC_DATA_BASE       EQU     0x200A8000  ; MLC Data Buffer         Base Address
MLC_BASE            EQU     0x200B8000  ; MLC NAND Controller     Base Address
                                        ; MLC NAND Flash Registers
MLC_CMD_OFS         EQU     0x00        ; Command        Register Address Offset
MLC_ADDR_OFS        EQU     0x04        ; Address        Register Address Offset
MLC_ECC_ENC_REG_OFS EQU     0x08        ; ECC Encode     Register Address Offset
MLC_ECC_DEC_REG_OFS EQU     0x0C        ; ECC Decode     Register Address Offset
MLC_ECC_AUTO_ENC_REG_OFS EQU 0x10       ; ECC Auto Encode     Reg Address Offset
MLC_ECC_AUTO_DEC_REG_OFS EQU 0x14       ; ECC Auto Decode     Reg Address Offset
MLC_RPR_OFS         EQU     0x18        ; Read Parity    Register Address Offset
MLC_WPR_OFS         EQU     0x1C        ; Write Parity   Register Address Offset
MLC_RUBP_OFS        EQU     0x20        ; Reset User Buf Ptr  Reg Address Offset
MLC_ROBP_OFS        EQU     0x24        ; Reset Overhead Buf Ptr  Address Offset
MLC_SW_WP_ADD_LOW_OFS EQU   0x28        ; Sw Wr Protect Low   Reg Address Offset
MLC_SW_WP_ADD_HIG_OFS EQU   0x2C        ; Sw Wr Protect High  Reg Address Offset
MLC_ICR_OFS         EQU     0x30        ; Configuration  Register Address Offset
MLC_TIME_REG_OFS    EQU     0x34        ; Timing         Register Address Offset
MLC_IRQ_MR_OFS      EQU     0x38        ; Interrupt Mask Register Address Offset
MLC_IRQ_SR_OFS      EQU     0x3C        ; Interrupt Status    Reg Address Offset
MLC_LOCK_PR_OFS     EQU     0x44        ; Lock Protection     Reg Address Offset
MLC_ISR_OFS         EQU     0x48        ; Status         Register Address Offset
MLC_CEH_OFS         EQU     0x4C        ; Chip-Enable Host Ctrl R Address Offset


; NAND Flash Commands
NAND_CMD_READ0      EQU     0x00        ; Read mode (1) command
NAND_CMD_READ1      EQU     0x01        ; Read mode (2) command
NAND_CMD_PAGEPROG   EQU     0x10        ; Auto program command
NAND_CMD_READSTART  EQU     0x30        ; Read start command
NAND_CMD_READ2      EQU     0x50        ; Read mode (3) command
NAND_CMD_ERASE1ST   EQU     0x60        ; Auto block erase 1-st command
NAND_CMD_STATUS     EQU     0x70        ; Status read (1) command
NAND_CMD_STATUS_MULTI EQU   0x71        ; Status read (2) command
NAND_CMD_SDIN       EQU     0x80        ; Serial data input command
NAND_CMD_READID     EQU     0x90        ; ID read (1) command
NAND_CMD_ERASE2ND   EQU     0xD0        ; Auto block erase 2-nd command
NAND_CMD_RESET      EQU     0xFF        ; Reset command

; NAND Constants
MLC_LOCK_Val        EQU     0xA25E      ; Unlocking               Constant

;// <e> NAND Flash Controller Configuration (NANDC)
NANDC_SETUP    EQU     0

;//   <h> NAND Clock Control Register (FLASHCLK_CTRL)
;//     <o0.5>   NAND Flash Controller Interrupt
;//                   <0=> SLC NAND Flash controller interrupt enabled
;//                   <1=> MLC NAND Flash controller interrupt enabled
;//                   <i>  Default: SLC NAND Flash controller interrupt enabled
;//     <o0.4>   NAND_DMA_REQ on NAND_RnB (only for MLC)
;//                   <0=> Disabled
;//                   <1=> Enabled
;//                   <i>  Default: Disabled
;//     <o0.3>   NAND_DMA_REQ on NAND_INT (only for MLC)
;//                   <0=> Disabled
;//                   <1=> Enabled
;//                   <i>  Default: Disabled
;//     <o0.2>   SLC/MLC Select
;//                   <0=> Multi-level (MLC) NAND Flash controller
;//                   <1=> Single-level (LLC) NAND Flash controller
;//                   <i>  Default: Multi-level (MLC) NAND Flash controller
;//     <o0.1>   MLC NAND Flash Clock Enable
;//                   <0=> Disabled
;//                   <1=> Enabled
;//                   <i>  Default: Enabled
;//     <o0.0>   SLC NAND Flash Clock Enable
;//                   <0=> Disabled
;//                   <1=> Enabled
;//                   <i>  Default: Enabled
;//   </h>
FLASHCLK_CTRL_Val  EQU  0x00000002

;//   <h> MLC NAND Flash Chip-Enable Host Control Register (MLC_CEH)
;//     <o0.0>   nCE Assert
;//                   <0=> Force nCE assert
;//                   <1=> Normal nCE operation (nCE controlled by controller)
;//                   <i>  Default: Force nCE assert
;//   </h>
MLC_CEH_Val        EQU  0x00000000

;//   <h> MLC NAND Controller Configuration Register (MLC_ICR)
;//     <o0.3>   Software Write Protection
;//                   <0=> Disabled
;//                   <1=> Enabled
;//                   <i>  Default: Disabled
;//     <o0.2>   Block Size
;//                   <0=> Small block flash device ( 512 + 16 bytes page)
;//                   <1=> Large block flash device (2048 + 64 bytes page)
;//                   <i>  Default: Small block flash device ( 512 + 16 bytes page)
;//     <o0.1>   NAND Flash Address Word Count
;//                   <0=> 3 address cycles
;//                   <1=> 4 address cycles
;//                   <i>  Default: 3 address cycles
;//     <o0.0>   NAND Flash I/O Bus Width
;//                   <0=> 8-bit bus width
;//                   <1=> 16-bit bus width (Not supported)
;//                   <i>  Default: 8-bit bus width
;//   </h>
MLC_ICR_Val        EQU  0x00000000

;// </e> NAND Flash Controller Configuration (NANDC)


;----------------------- Vector Floating-Point Definitions ---------------------

; Constants
VFP_EN_BIT      EQU     (1<<30)         ; VFP Enable Bit


;----------------------- Cache Definitions -------------------------------------

; Constants
ICACHE_EN_BIT   EQU     (1<<12)         ; Instruction Cache Enable Bit

;// <e> Instruction Cache Enable
;// </e>
ICACHE_SETUP    EQU     1


;----------------------- CODE --------------------------------------------------

                PRESERVE8
                

; Area Definition and Entry Point
;  Startup Code must be linked first at Address at which it expects to run.

                AREA    RESET, CODE, READONLY
                ARM

                IF      :LNOT::DEF:__EVAL 
                IF      :DEF:SIZE_INT_INFO
                IMPORT  ||Image$$ER_IROM1$$RO$$Length||
                IMPORT  ||Image$$RW_IRAM1$$RW$$Length||
                ELIF    :DEF:SIZE_EXT_INFO
                IMPORT  ||Image$$ER_ROM1$$RO$$Length||
                IMPORT  ||Image$$RW_RAM1$$RW$$Length||
                ENDIF
                ENDIF

; Exception Vectors
;  Mapped to Address 0.
;  Absolute addressing mode must be used.
;  Dummy Handlers are implemented as infinite loops which can be modified.

Vectors         LDR     PC,Reset_Addr         
                LDR     PC,Undef_Addr
                LDR     PC,SWI_Addr
                LDR     PC,PAbt_Addr
                LDR     PC,DAbt_Addr
                ; Reserved vector is used for image size information
                IF      :DEF:__EVAL
                  DCD   0x4000
                ELSE 
                  IF    :DEF:SIZE_INT_INFO
                    DCD ||Image$$ER_IROM1$$RO$$Length||+\
                        ||Image$$RW_IRAM1$$RW$$Length||
                  ELIF  :DEF:SIZE_EXT_INFO
                    DCD ||Image$$ER_ROM1$$RO$$Length||+\
                        ||Image$$RW_RAM1$$RW$$Length||
                  ELSE
                    NOP
                  ENDIF
                ENDIF
                LDR     PC,IRQ_Addr     
                LDR     PC,FIQ_Addr

Reset_Addr      DCD     Reset_Handler
Undef_Addr      DCD     Undef_Handler
SWI_Addr        DCD     SWI_Handler
PAbt_Addr       DCD     PAbt_Handler
DAbt_Addr       DCD     DAbt_Handler
                DCD     0               ; Reserved Address
				PRESERVE8
				IMPORT  IRQ_Handler
IRQ_Addr        DCD     IRQ_Handler
				IMPORT	FIQ_Handler
FIQ_Addr        DCD     FIQ_Handler

Undef_Handler   B       Undef_Handler
SWI_Handler     B       SWI_Handler
PAbt_Handler    B       PAbt_Handler
DAbt_Handler    B       DAbt_Handler
; IRQ_Handler     B       IRQ_Handler
;FIQ_Handler     B       FIQ_Handler


; Reset Handler

                EXPORT  Reset_Handler
Reset_Handler   


; Clock Setup ------------------------------------------------------------------

                IF      CLOCK_SETUP != 0

                LDR     R0, =SYSTEM_BASE

                ; If PLL397 is used for system clock
                IF      (SYSCLK_CTRL_Val:AND:SYSCLK_PLL_BIT) != 0
PLL_Loop        LDR     R1, [R0, #PLL397_CTRL_OFS]    ; Wait for PLL397 stabil
                ANDS    R1, R1, #PLL_LOCK_BIT
                BEQ     PLL_Loop
                LDR     R1, =SYSCLK_CTRL_Val          ; Switch to PLL397
                STR     R1, [R0, #SYSCLK_CTRL_OFS]
                LDR     R1, =OSC_CTRL_Val             ; Control main oscillator
                STR     R1, [R0, #OSC_CTRL_OFS]

                ELSE    ; If PLL397 is not used for system clock
                LDR     R1, =SYSCLK_CTRL_Val          ; Control PLL397
                STR     R1, [R0, #SYSCLK_CTRL_OFS]
                ENDIF

                LDR     R1, =HCLKPLL_CTRL_Val         ; Setup HCLK PLL
                STR     R1, [R0, #HCLKPLL_CTRL_OFS]

                ; If HCLK PLL is setup for operating mode
                IF      (HCLKPLL_CTRL_Val:AND:HCLKPLL_PD_BIT) != 0
HCLK_Loop       LDR     R1, [R0, #HCLKPLL_CTRL_OFS]   ; Wait for HCLK stabil
                ANDS    R1, R1, #PLL_LOCK_BIT
                BEQ     HCLK_Loop
                LDR     R1, =HCLKDIV_CTRL_Val         ; Setup HCLK dividers
                STR     R1, [R0, #HCLKDIV_CTRL_OFS]

                LDR     R1, [R0, #PWR_CTRL_OFS]       ; Switch to Normal RUN
                ORR     R1, R1, #NORMAL_RUN_BIT
                STR     R1, [R0, #PWR_CTRL_OFS]
                ENDIF

                ENDIF   ;CLOCK_SETUP != 0


; EMC Setup --------------------------------------------------------------------

                IF      (:LNOT::DEF:EMC_NO_INIT):LAND:(EMC_SETUP != 0)

                LDR     R0, =SYSTEM_BASE          ; Address of SYS CON Config
                LDR     R1, =EMC_BASE             ; Address of EMC Controller
                LDR     R2, =DYN_MEM0_BASE        ; External SDRAM0 Start Adr
                LDR     R3, =DYN_MEM1_BASE        ; External SDRAM1 Start Adr

;  Setup Dynamic Memory Interface
                MOV     R5, #0
                MOV     R4, #0x01                 ; Enable SDRAM Controller
                STR     R4, [R1, #EMCControl_OFS]
                LDR     R4, =EMCConfig_Val
                STR     R4, [R1, #EMCConfig_OFS]

;  Setup Dynamic Memory Interface
                IF      (:LNOT::DEF:EMC_DYNAMIC_NO_INIT):LAND:(EMC_DYNAMIC_SETUP != 0)

                LDR     R4, =SDRAMCLK_CTRL_Val
                STR     R4, [R1, #SDRAMCLK_CTRL_OFS]

                LDR     R4, =EMCDynRP_Val
                STR     R4, [R1, #EMCDynRP_OFS]
                LDR     R4, =EMCDynRAS_Val
                STR     R4, [R1, #EMCDynRAS_OFS]
                LDR     R4, =EMCDynSREX_Val
                STR     R4, [R1, #EMCDynSREX_OFS]
                LDR     R4, =EMCDynWR_Val
                STR     R4, [R1, #EMCDynWR_OFS]
                LDR     R4, =EMCDynRC_Val
                STR     R4, [R1, #EMCDynRC_OFS]
                LDR     R4, =EMCDynRFC_Val
                STR     R4, [R1, #EMCDynRFC_OFS]
                LDR     R4, =EMCDynXSR_Val
                STR     R4, [R1, #EMCDynXSR_OFS]
                LDR     R4, =EMCDynRRD_Val
                STR     R4, [R1, #EMCDynRRD_OFS]
                LDR     R4, =EMCDynMRD_Val
                STR     R4, [R1, #EMCDynMRD_OFS]
                LDR     R4, =EMCDynCDLR_Val
                STR     R4, [R1, #EMCDynCDLR_OFS]

                LDR     R4, =EMCDynReadCfg_Val
                STR     R4, [R1, #EMCDynReadCfg_OFS]

                IF      (EMC_DYNCS0_SETUP != 0)
                LDR     R4, =EMCDynRasCas0_Val
                STR     R4, [R1, #EMCDynRasCas0_OFS]
                LDR     R4, =EMCDynConfig0_Val
                STR     R4, [R1, #EMCDynConfig0_OFS]
                ENDIF

                IF      (EMC_DYNCS1_SETUP != 0)
                LDR     R4, =EMCDynRasCas1_Val
                STR     R4, [R1, #EMCDynRasCas1_OFS]
                LDR     R4, =EMCDynConfig1_Val
                STR     R4, [R1, #EMCDynConfig1_OFS]
                ENDIF

                LDR     R4, =(NOP_CMD:OR:0x03)    ; Write NOP Command
                STR     R4, [R1, #EMCDynControl_OFS]

                LDR     R6, =100*52               ; ~100us at 208 MHz
Wait_0          SUBS    R6, R6, #1
                BNE     Wait_0

                LDR     R4, =(PALL_CMD:OR:0x03)   ; Write Precharge All Command
                STR     R4, [R1, #EMCDynControl_OFS]
                LDR     R4, =(NOP_CMD:OR:0x03)    ; Write NOP Command
                STR     R4, [R1, #EMCDynControl_OFS]
                MOV     R4, #2
                STR     R4, [R1, #EMCDynRefresh_OFS]

                MOV     R6, #1*52                 ; ~1us at 208 MHz
Wait_1          SUBS    R6, R6, #1
                BNE     Wait_1

                LDR     R4, =EMCDynRefresh_Val
                STR     R4, [R1, #EMCDynRefresh_OFS]

                MOV     R6, #1*52                 ; ~1us at 208 MHz
Wait_2          SUBS    R6, R6, #1
                BNE     Wait_2

                LDR     R4, =(MODE_CMD:OR:0x03)   ; Write MODE Command
                STR     R4, [R1, #EMCDynControl_OFS]

                ; Set memory mode (dummy read)
                IF      (EMC_DYNCS0_SETUP != 0)
                LDR     R4, =SDRAM0_MODE_REG
                LDR     R4, [R4, #0]
                MOV     R6, #1*52                 ; ~1us at 208 MHz
Wait_3          SUBS    R6, R6, #1
                BNE     Wait_3
                LDR     R4, =SDRAM0_EXT_MODE_REG
                LDR     R4, [R4, #0]
                ENDIF
                IF      (EMC_DYNCS1_SETUP != 0)
                LDR     R4, =SDRAM1_MODE_REG
                LDR     R4, [R4, #0]
                MOV     R6, #1*52                 ; ~1us at 208 MHz
Wait_4          SUBS    R6, R6, #1
                BNE     Wait_4
                LDR     R4, =SDRAM1_EXT_MODE_REG
                LDR     R4, [R4, #0]
                ENDIF

                LDR     R4, =NORMAL_CMD           ; Write NORMAL Command
                STR     R4, [R1, #EMCDynControl_OFS]

                MOV     R6, #1*52                 ; ~1us at 208 MHz
Wait_5          SUBS    R6, R6, #1
                BNE     Wait_5

                IF      (EMC_DYNCS0_SETUP != 0)
                IF      (EMC_AHB0_SETUP != 0)
                LDR     R4, =EMCAHBControl0_Val
                STR     R4, [R1, #EMCAHBControl0_OFS]
                LDR     R4, =EMCAHBTimeOut0_Val  ; AHB 0 Timeout
                STR     R4, [R1, #EMCAHBTimeOut0_OFS]
                ENDIF
                IF      (EMC_AHB2_SETUP != 0)
                LDR     R4, =EMCAHBControl2_Val
                STR     R4, [R1, #EMCAHBControl2_OFS]
                LDR     R4, =EMCAHBTimeOut2_Val  ; AHB 2 Timeout
                STR     R4, [R1, #EMCAHBTimeOut2_OFS]
                ENDIF
                IF      (EMC_AHB3_SETUP != 0)
                LDR     R4, =EMCAHBControl3_Val
                STR     R4, [R1, #EMCAHBControl3_OFS]
                LDR     R4, =EMCAHBTimeOut3_Val  ; AHB 3 Timeout
                STR     R4, [R1, #EMCAHBTimeOut3_OFS]
                ENDIF
                IF      (EMC_AHB4_SETUP != 0)
                LDR     R4, =EMCAHBControl4_Val
                STR     R4, [R1, #EMCAHBControl4_OFS]
                LDR     R4, =EMCAHBTimeOut4_Val  ; AHB 4 Timeout
                STR     R4, [R1, #EMCAHBTimeOut4_OFS]
                ENDIF
                ENDIF   ;(EMC_DYNCS0_SETUP != 0)

                ENDIF   ;(:LNOT::DEF:EMC_DYNAMIC_NO_INIT):LAND:(EMC_DYNAMIC_SETUP != 0)

;  Setup Static Memory Interface
                IF      (:LNOT::DEF:EMC_STATIC_NO_INIT):LAND:(EMC_STATIC_SETUP != 0)

                IF      (EMC_STACS0_SETUP != 0)
                LDR     R4, =EMCStaConfig0_Val
                STR     R4, [R0, #EMCStaConfig0_OFS]
                LDR     R4, =EMCStaWaitWen0_Val
                STR     R4, [R0, #EMCStaWaitWen0_OFS]
                LDR     R4, =EMCStaWaitOen0_Val
                STR     R4, [R0, #EMCStaWaitOen0_OFS]
                LDR     R4, =EMCStaWaitRd0_Val
                STR     R4, [R0, #EMCStaWaitRd0_OFS]
                LDR     R4, =EMCStaWaitPage0_Val
                STR     R4, [R0, #EMCStaWaitPage0_OFS]
                LDR     R4, =EMCStaWaitWr0_Val
                STR     R4, [R0, #EMCStaWaitWr0_OFS]
                LDR     R4, =EMCStaWaitTurn0_Val
                STR     R4, [R0, #EMCStaWaitTurn0_OFS]
                ENDIF

                IF      (EMC_STACS1_SETUP != 0)
                LDR     R4, =EMCStaConfig1_Val
                STR     R4, [R0, #EMCStaConfig1_OFS]
                LDR     R4, =EMCStaWaitWen1_Val
                STR     R4, [R0, #EMCStaWaitWen1_OFS]
                LDR     R4, =EMCStaWaitOen1_Val
                STR     R4, [R0, #EMCStaWaitOen1_OFS]
                LDR     R4, =EMCStaWaitRd1_Val
                STR     R4, [R0, #EMCStaWaitRd1_OFS]
                LDR     R4, =EMCStaWaitPage1_Val
                STR     R4, [R0, #EMCStaWaitPage1_OFS]
                LDR     R4, =EMCStaWaitWr1_Val
                STR     R4, [R0, #EMCStaWaitWr1_OFS]
                LDR     R4, =EMCStaWaitTurn1_Val
                STR     R4, [R0, #EMCStaWaitTurn1_OFS]
                ENDIF

                IF      (EMC_STACS2_SETUP != 0)
                LDR     R4, =EMCStaConfig2_Val
                STR     R4, [R0, #EMCStaConfig2_OFS]
                LDR     R4, =EMCStaWaitWen2_Val
                STR     R4, [R0, #EMCStaWaitWen2_OFS]
                LDR     R4, =EMCStaWaitOen2_Val
                STR     R4, [R0, #EMCStaWaitOen2_OFS]
                LDR     R4, =EMCStaWaitRd2_Val
                STR     R4, [R0, #EMCStaWaitRd2_OFS]
                LDR     R4, =EMCStaWaitPage2_Val
                STR     R4, [R0, #EMCStaWaitPage2_OFS]
                LDR     R4, =EMCStaWaitWr2_Val
                STR     R4, [R0, #EMCStaWaitWr2_OFS]
                LDR     R4, =EMCStaWaitTurn2_Val
                STR     R4, [R0, #EMCStaWaitTurn2_OFS]
                ENDIF

                IF      (EMC_STACS3_SETUP != 0)
                LDR     R4, =EMCStaConfig3_Val
                STR     R4, [R0, #EMCStaConfig3_OFS]
                LDR     R4, =EMCStaWaitWen3_Val
                STR     R4, [R0, #EMCStaWaitWen3_OFS]
                LDR     R4, =EMCStaWaitOen3_Val
                STR     R4, [R0, #EMCStaWaitOen3_OFS]
                LDR     R4, =EMCStaWaitRd3_Val
                STR     R4, [R0, #EMCStaWaitRd3_OFS]
                LDR     R4, =EMCStaWaitPage3_Val
                STR     R4, [R0, #EMCStaWaitPage3_OFS]
                LDR     R4, =EMCStaWaitWr3_Val
                STR     R4, [R0, #EMCStaWaitWr3_OFS]
                LDR     R4, =EMCStaWaitTurn3_Val
                STR     R4, [R0, #EMCStaWaitTurn3_OFS]
                ENDIF

                LDR     R4, =EMCStaExtWait_Val
                STR     R4, [R0, #EMCStaExtWait_OFS]

                ENDIF   ;(:LNOT::DEF:EMC_STATIC_NO_INIT):LAND:(EMC_STATIC_SETUP != 0)

                ENDIF   ;(:LNOT::DEF:EMC_NO_INIT):LAND:(EMC_SETUP != 0)


; NAND Flash Setup -------------------------------------------------------------

                IF      (:LNOT::DEF:NAND_NO_INIT):LAND:(NANDC_SETUP != 0)

                LDR     R0, =SYSTEM_BASE          ; Address of SYSTEM CONTROL Config
                LDR     R1, =MLC_BASE             ; Address of MLC NAND Controller
                LDR     R2, =MLC_DATA_BASE        ; Address of MLC Data Buffer
                MOV     R5, #0                    ; 0 value

                LDR     R4, =FLASHCLK_CTRL_Val    ; Setup NAND Flash Clock Control
                STR     R4, [R0, #FLASHCLK_CTRL_OFS]
                LDR     R4, =MLC_CEH_Val          ; Setup NAND Flash Chip-Enable Control
                STR     R4, [R1, #MLC_CEH_OFS]
                LDR     R4, =NAND_CMD_RESET       ; Reset NAND Flash
                STR     R4, [R1, #MLC_CMD_OFS]
                BL      NAND_Wait                 ; Wait for operation to finish
                LDR     R4, =MLC_LOCK_Val         ; Unlock write to MLC_ICR
                STR     R4, [R1, #MLC_LOCK_PR_OFS]
                LDR     R4, =MLC_ICR_Val          ; Setup NAND Controller Configuration
                STR     R4, [R1, #MLC_ICR_OFS]
                BL      NAND_Wait                 ; Wait for operation to finish
                B       NAND_End                  ; NAND Setup Finished

                MOV     R6, #0x01                 ; Wait function
                MOV     R7, #16
NAND_Wait       LDR     R5, [R0, #MLC_ISR_OFS]
                ANDS    R5, R5, R6
                BNE     NAND_Wait_Dec
NAND_Wait_More  LDR     R5, [R0, #MLC_ISR_OFS]
                ANDS    R5, R5, R6
                BNE     NAND_Wait_More
                BX      R14
NAND_Wait_Dec   SUBS    R7, R7, #1
                BNE     NAND_Wait

NAND_End
                ENDIF   ;(:LNOT::DEF:NAND_NO_INIT):LAND:(NANDC_SETUP != 0)


; Copy Exception Vectors to Internal RAM ---------------------------------------

                IF      :DEF:RAM_INTVEC
                ADR     R8,  Vectors    ; Source
                LDR     R9, =IRAM_BASE  ; Destination
                LDMIA   R8!, {R0-R7}    ; Load Vectors 
                STMIA   R9!, {R0-R7}    ; Store Vectors 
                LDMIA   R8!, {R0-R7}    ; Load Handler Addresses 
                STMIA   R9!, {R0-R7}    ; Store Handler Addresses
                ENDIF


; Remap on-chip RAM to address 0 -----------------------------------------------

                IF      :DEF:REMAP
                LDR     R0, =SYSTEM_BASE
                MOV     R1, #REMAP_BIT  ; Remap for Instruction and Data Master
                STR     R1, [R0, #BOOT_MAP_OFS] ; Execute Remap
                ENDIF


; Vector Floating-Point (VFP) Setup --------------------------------------------

                IF      {FPU} != "SoftVFP"
                MOV     R1, #VFP_EN_BIT ; Enable VFP
                FMXR    FPEXC, R1
                ENDIF


; Cache Setup ------------------------------------------------------------------

                IF      ICACHE_SETUP != 0
                MRC     p15, 0, R0, c1, c0, 0   ; Enable Instruction Cache
                ORR     R0, R0, #ICACHE_EN_BIT
                MCR     p15, 0, R0, c1, c0, 0
                ENDIF


; Setup Stack for each mode ----------------------------------------------------

                LDR     R0, =Stack_Top

;  Enter Undefined Instruction Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_UND:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #UND_Stack_Size

;  Enter Abort Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_ABT:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #ABT_Stack_Size

;  Enter FIQ Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_FIQ:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #FIQ_Stack_Size

;  Enter IRQ Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_IRQ:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #IRQ_Stack_Size

;  Enter Supervisor Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_SVC:OR:I_Bit:OR:F_Bit
                MOV     SP, R0
                SUB     R0, R0, #SVC_Stack_Size

;  Enter User Mode and set its Stack Pointer
                MSR     CPSR_c, #Mode_USR
                IF      :DEF:__MICROLIB

                EXPORT __initial_sp

                ELSE

                MOV     SP, R0
                SUB     SL, SP, #USR_Stack_Size

                ENDIF


; Enter the C code

                IMPORT  __main
                LDR     R0, =__main
                BX      R0

                IF      :DEF:__MICROLIB

                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE
; User Initial Stack & Heap
                AREA    |.text|, CODE, READONLY

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + USR_Stack_Size)
                LDR     R2, = (Heap_Mem +      Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDIF


                END
