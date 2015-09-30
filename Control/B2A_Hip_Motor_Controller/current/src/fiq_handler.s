        AREA FIQ_HANDLER, CODE, READONLY, ALIGN=3
        ARM
        PRESERVE8

T0_BASE           EQU     0xE0004000
T1_BASE           EQU     0xE0008000
IR_OFFSET         EQU     0x0
TC_OFFSET         EQU     0x8
MR0_OFFSET        EQU     0x18
CCR_OFFSET        EQU     0x28
CR0_OFFSET        EQU     0x2C
CR1_OFFSET        EQU     0x30
CR2_OFFSET        EQU     0x34
CR3_OFFSET        EQU     0x38

;qdc_data struct fields
overflow_size           EQU 0
tmr_base                EQU 4
FIO0PIN                 EQU 8

overflow_01             EQU 12
encoder_count_01        EQU 16
write_index_01          EQU 20
read_index_01           EQU 24
buffer_mask_01          EQU 28
time_array_01           EQU 32

overflow_23             EQU 36
encoder_count_23        EQU 40
write_index_23          EQU 44
read_index_23           EQU 48
buffer_mask_23          EQU 52
time_array_23           EQU 56


;T0TC              EQU     0xE0004008
    
                  ;IMPORT  qdc_execution_time          [DATA]
                  ;IMPORT  qdc_execution_timer         [DATA]

                  IMPORT  qdc_tmr0_data         [DATA]

        ALIGN 8 
FIQ_Handler
        EXPORT FIQ_Handler

        ; **** TEST CODE ****
        ;Check initial time, store in qdc_execution_timer
        ;LDR R8, =T0TC
        ;LDR R8, [R8]
        ;LDR R10, =qdc_execution_timer
        ;STR R8, [R10]

        ;Save variable registers used to stack
        STMFD    SP!,{R0-R7} ;Save non-FIQ registers used to stack
                             ;These registers will be reused by cap23, if
                             ;it so happens that both have interrupts at the same time.

        ;start of 2x code - two counts per full encoder cycle
        LDR R9, =qdc_tmr0_data    ;load data struct base address
        ADD R9, #overflow_01      ;point R9 to overflow value

        LDMDA R9,{R0-R3}          ;load first set of data struct values (backwards from overflow)

        ;R0 = overflow_size
        ;R1 = tmr_base_addr
        ;R2 = FIO0PIN_addr
        ;R3 = overflow

        ;R9 = data struct base address
        ;R13 = stack pointer
        ;R14 = link register
        ;R15 = program counter

        LDRB R12, [R1, #IR_OFFSET]   
        ;R12 = T0IR value

        TST R12, #1         ;Match event on MR0?
        MOVEQ R0, #0        ;No match event? Set overflow size to zero, no added overflow this call

cap01   ;Start first encoder processing (CAP01)
        CMP R3, #0x7F000000 ;Is overflow approaching the 32-bit positive maximum (Note: won't work for too-large overflow_size)
        ADDLT R3, R3, R0    ;Add in overflow if limit has not been reached.

        ;Load FIO0PIN register contents.
        ;Performance of this code is sensitive to when the FIO0PIN values are sampled.
        ;Too soon may lead to glitch detection, if the CAP0 pin is not fully settled in its new value.
        ;Too late, and the CAP1 pin may have moved to a new value, at high count speeds.
        ;Load FIO0PIN contents into R11 for use by both CAP01 and CAP23 as needed.
        LDR R11, [R2]           
        ;R11 = FIO0PIN value

        TST R12, #0x10              ;Test CAP0 flag
        BEQ write_back_overflow_01  ;If no capture event on cap0, skip the count and timing code
                                    ;and write back the updated overflow value
        LDR R10, [R1, #CCR_OFFSET]  ;Read in the CCR register
        MOV R8, R10, LSR #1         ;Falling-edge enable bit moved to bit 0
        ADD R8, R8, R11, LSR #22    ;Add together R10 and R11, with falling-edge and CAP0 pin bits aligned
        AND R8, R8, #1              ;Gives 1 for a valid count transition, 0 for glitch
        BIC R10, R10, #0x3          ;clear edge direction bits
        BIC R10, R10, #0xF000       ;clear reserved bits
        TST R11, #0x00400000        ;check state of CAP0 (P0[22])
        ORRNE R10, R10, #2          ;Set for falling edge
        ORREQ R10, R10, #1          ;Set for rising edge
        STR R10, [R1, #CCR_OFFSET]  ;Update the CCR register.
        TST R10, #1                 ;Test cap 0 rising edge IE bit in CCR
        RSBNE R8, R8, #0            ;Switch sign if falling edge was active (before rewriting above)
        TST R11, #0x08000000        ;Test P0[27] in FIO0PIN (CAP 1 input)
        RSBNE R8, R8, #0            ;Switch sign again if high (NE zero)

       ;R8 = count increment (-1, 0, or 1). A glitch is indicated by 0 - a rising edge was detected, but the 
       ;CAPO line is now low, or a falling edge was detected, but now the line is high.

        LDR R10, [R1, #CR0_OFFSET]  ;New capture time value
                                    ;Update this even if the count is glitched

        RSB R2, R10, #0    ;R2 will be the next initial overflow value = - capture time

        ;Simultaneous match/CAP0 event handling
        CMP R10, R0, LSR #1 ;Did the capture event occur immediately before, or after, the match event
                            ;If before, captured time will be near max; if after, captured time will be near zero
        ADDHS R2, R2, R0    ;Increase next overflow value if capture occurred before the match
        SUBHS R3, R3, R0    ;Decrease current overflow value if capture before match
                            ;Note that R0 will equal zero if no overflow/match event, so this code
                            ;will have no effect in that case.

        ADD R3, R3, R10     ;R3 now contains elapsed time since previous capture

        ;R10 is now free
 
        ;Load remaining data struct values
        LDMIB R9, {R4 - R7, R10}

        ;R0 = overflow_size
        ;R1 = tmr_base_addr
        ;R2 = overflow (next)
        ;R3 = elapsed time
        ;R4 = encoder_count
        ;R5 = write_index
        ;R6 = read_index
        ;R7 = buffer_mask
        ;R8 = count increment
        ;R9 = data_struct_address (overflow)
        ;R10 = time_array_address
        ;R11 = FIO0PIN
        ;R12 = TOIR
        ;R13 = stack pointer
        ;R14 = link register
        ;R15 = program counter

        CMP R8, #0            ;R8 (count) = 0 means there was a glitch
        MOVEQ R3, #0          ;Set time value to zero in the event of a glitch

        ADD R4, R4, R8        ;Update encoder count

        CMP R5, R6            ;check if ring buffer is full
        BEQ write_back_all_01 ;skip remainder of velocity code if buffer is full

        CMP R8, #1            ;Check sign
        RSBNE R3, R3, #0      ;Make negative if count is minus
  
        STR R3, [R10, R5, LSL #2] ;Save time value to ring buffer.
                                  ;Address is R10 (base) plus (R5 ring buffer write index times 4)
        ADD R5, R5, #1            ;increment write index
        AND R5, R5, R7            ;modulo addition, wraps around ring buffer.
                                  ;(ring buffer size must 2^n; ANDed value must be 2^n-1 for this to work)


write_back_all_01
 
        STMIA R9, {R2, R4, R5}    ;save overflow, encoder count, and write index back to data struct
        B cap23

write_back_overflow_01
       STR R3, [R9]           ;save overflow only, continue on to possible cap23 processing
       
       ;Start of second encoder processing, using TMR0 capture inputs 2 and 3
cap23
        LDR R3, [R9,#overflow_23 - overflow_01]! ;load overflow_23 into R3, and point R9 to overflow_23

        ;R0 = overflow_size
        ;R1 = tmr_base_addr
        ;R2 = FIO0PIN value
        ;R3 = overflow_23

        ;R9 = data struct base address
        ;R11 = FIO0PIN value
        ;R12 = T0IR
        ;R13 = stack pointer
        ;R14 = link register
        ;R15 = program counter

        ;Update overflow_23 value. R0 will be non-zero only in the event of an MR0 flag (checked previously)
        CMP R3, #0x7F000000 ;Is overflow approaching the 32-bit positive maximum (Note: won't work for too-large overflow_size)
        ADDLT R3, R3, R0    ;Add in overflow

        TST R12, #0x40              ;Test CAP2 flag
        BEQ write_back_overflow_23  ;If no capture event on cap2, skip the count and timing code
                                    ;and write back the updated overflow value
        LDR R10, [R1, #CCR_OFFSET]  ;Read in the CCR register
        MOV R8, R10, LSR #7         ;CAP2 falling-edge enable bit moved to bit 0, put in R8
        ADD R8, R8, R11, LSR #16    ;Add together R10 and R11, with CAP2 falling-edge and CAP2 pin bits aligned
        AND R8, R8, #1              ;Gives 1 for a valid count transition, 0 for glitch
        BIC R10, R10, #0xC0         ;Clear CAP2 edge direction bits
        BIC R10, R10, #0xF000       ;clear reserved bits
        TST R11, #0x10000           ;check state of CAP2 pin (P0[16])
        ORRNE R10, R10, #0x80       ;Set T0CCR for CAP2 falling edge
        ORREQ R10, R10, #0x40       ;Set T0CCR for CAP2 rising edge
        STR R10, [R1, #CCR_OFFSET]  ;Update the CCR register.
        TST R10, #0x40              ;Test CAP2 rising edge IE bit in T0CCR
        RSBNE R8, R8, #0            ;Switch sign if falling edge was active (before rewriting above)
        TST R11, #0x20000000        ;Test P0[29] in FIO0PIN (CAP 3 input)
        RSBNE R8, R8, #0            ;Switch sign again if high (NE zero)

       ;R8 = count increment (-1, 0, or 1). A glitch is indicated by 0 - a rising edge was detected, but the 
       ;CAP2 line is now low, or a falling edge was detected, but now the line is high.

        LDR R10, [R1, #CR2_OFFSET]  ;New capture time value

        RSB R2, R10, #0     ;R2 will be the next initial overflow value = - capture time

        ;Simultaneous overflow handling
        CMP R10, R0, LSR #1 ;Did the capture event occur immediately before, or after, the match event
        ADDHS R2, R2, R0    ;Increase next overflow value if capture occurred before the overflow
        SUBHS R3, R3, R0    ;Decrease current overflow value if capture before overflow
                            ;Note that R0 will equal zero if no overflow/match event, so this code
                            ;will have no effect in that case.

        ADD R3, R3, R10     ;R3 now contains elapsed time since previous capture

        ;R10 is now free
 
        ;Load remaining data struct values
        LDMIB R9, {R4 - R7, R10}

        ;R0 = overflow_size
        ;R1 = tmr_base_addr
        ;R2 = Overflow (next)
        ;R3 = elapsed time
        ;R4 = encoder_count
        ;R5 = write_index
        ;R6 = read_index
        ;R7 = buffer_mask
        ;R8 = count increment
        ;R9 = data_struct_address (overflow)
        ;R10 = time_array_address
        ;R11 = FIO0PIN
        ;R12 = TOIR
        ;R13 = stack pointer
        ;R14 = link register
        ;R15 = program counter

        CMP R8, #0                ;R8 (count) = 0 means there was a glitch
        MOVEQ R3, #0              ;Set time value to zero in the event of a glitch

        ADD R4, R4, R8            ;Update encoder count

        CMP R5, R6                ;check if ring buffer is full
        BEQ write_back_all_23     ;skip remainder of velocity code if buffer is full

        CMP R8, #1                ;Check sign
        RSBNE R3, R3, #0          ;Make negative if count is minus
  
        STR R3, [R10, R5, LSL #2] ;Save time value to ring buffer.
                                  ;Address is R10 (base) plus (R5 ring buffer write index times 4)
        ADD R5, R5, #1            ;increment write index
        AND R5, R5, R7            ;modulo addition, wraps around ring buffer.
                                  ;(ring buffer size must 2^n; ANDed value must be 2^n-1 for this to work)


write_back_all_23
 
        STMIA R9, {R2, R4, R5}    ;save overflow, encoder count, and write index back to data struct
        B clear_int

write_back_overflow_23
       STR R3, [R9]               ;save overflow only

clear_int        
        ;Clear the Timer0 interrupt flags that were processed (R12)
        STRB R12, [R1,#IR_OFFSET]

        ;reg_pop
        LDMFD SP!,{R0-R7}         ;Pop non-FIQ registers used off of stack

qdc_tmr0_isr_end


       ; **** TEST CODE ****
        ;Calculate elapsed time, store in qdc_execution_time
        ;LDR R8, =T0TC
        ;LDR R8, [R8]
        ;LDR R10, =qdc_execution_timer
        ;LDR R10, [R10]
        ;SUB R10, R8, R10
        ;LDR R8, =qdc_execution_time
        ;STR R10, [R8]
 
    ; Return to C main from FIQ handler. Put this at the end of any FIQ handler.
        SUBS PC, R14, #0x04
  
        END

        ;Below is 4X test code with hysteresis. CAP01 largely functional, but could use some more optimization as
        ;in the 2X code above and addition of CAP23 handling. Somewhat slower than the 2X code, even with the
        ;use of a lookup table. However, can increase speed through use of the tricks added to the 2X code above.
        ;The 4X code should be less susceptible to glitches at direction changes, but at the cost of twice or more the 
        ;processing time, since there are twice as many interrupts.
        ;It's not feasible, unfortunately, to combine the 2X and 4X on the same timer; the two channels are too
        ;interconnected in the code.

        ;Double semicolons indicate code that was active and functional before commenting
        ;To restore, remove all double semicolons. Leave the single ones.
        ;start of 4x code
        ;;LDR R9, =qdc_tmr0_data        ;load data struct base address
        ;;LDR R10, =T0_BASE            ;Load timer T0 base address into R10
        ;;LDR R12, [R10, #IR_OFFSET]   ;R12 holds timer IR register interrupt source flags

        ;Check for MR0 flag (overflow/rollover of MR0 value plus 1)
        ;;TST R12, #0x1
        ;;BEQ cap01_check                 ;If not set, no overflow - continue with capture flag handling

        ;;LDR R8, [R9, #overflow_01]    ;load overflow_01 into R8
        ;;CMP R8, #0x7F000000           ;Make sure overflow doesn't overflow! 
        ;;ADDLT R8, R8, #29952          ;Update overflow_01 if not too large; divisible by 2^n
        ;;ADDLT R8, R8, #48             ;Update overflow_01 if not too large; sum to overflow_size
        ;;STRLT R8, [R9, #overflow_01]  ;Store updated overflow_01 value back to data struct
 
        ;;LDR R8, [R9, #overflow_23]    ;load overflow_23 into R8
        ;;CMP R8, #0x7F000000           ;Make sure overflow doesn't overflow!
        ;;ADDLT R8, R8, #29952          ;Update overflow_23 if not too large; divisible by 2^n
        ;;ADDLT R8, R8, #48             ;Update overflow_23 if not too large; sum to overflow_size
        ;;STRLT R8, [R9, #overflow_23]  ;Store updated overflow value back to data struct

        ;Process capture values from CAP0 and CAP1, if flags set
;;cap01_check
        ;Registers initially in use
        ;R15 - PC
        ;R14 - link register. Can be used if pushed to stack, and no BL instructions
        ;   Will be disrupted by function calls, since they use BL
        ;R13 - stack pointer. Keep ALIGN8 (always push/pop even number of registers)
        ;R12 - T0IR interrupt flags. Warning - may be disrupted by function calls
        ;R11 - VICFIQAddr contents, can be pushed to stack
        ;R10 - T0 base address
        ;R9 - QDC data struct base address

        ;R8 is free.
        ;R0 - R7 may be used if pushed to stack first; also R11 and R14 if needed
                
        ;;TST R12, #0x30            ;Check for capture event on input lines 0 or 1
        ;;BEQ clear_int             ;If none, go to capture event 23 handling
                                  ; **** TEST CODE **** skip cap23
                                  ;Should eventually go to cap23_check

        ;Save variable registers used to stack
        ;;STMFD    SP!,{R0-R7} ;Save non-FIQ registers used to stack
                             ;Note that these registers will be reused by cap23, if
                             ;it so happens that both have interrupts at the same time.
                             ;Need to be sure they are popped again only once.
;;cap01_do
        ;;LDMIA R9,{R1-R5}          ;load first 5 data struct values

        ;R1 = prev_time_01
        ;R2 = encoder count 01
        ;R3 = state_01
        ;R4 = lookup table address
        ;R5 = FIO0PIN address
        ;R9 = data struct base address
        ;R10 = T0_BASE
        ;R12 = T0IR
        
        ;;LDR R5, [R5]        ;Put pin states in R5 (starts off containing address)

        ;;AND R0, R5, #0x00400000  ;mask off except P0[22], put in R0 scratch register
        ;;ADD R4, R4, R0, LSR #16      ;Add to lookup table address R4 with P0[22] bit now in bit 6 location
        ;;AND R0, R5, #0x08000000  ;mask off except P0[27], put in R0
        ;;ADD R4, R4, R0, LSR #20  ;add to R4 with P0[27] now in bit 7 location
        ;;ADD R4, R4, R3, LSL #8    ;add in the state value, shifted left by 8 to put in bit 9 and bit 8
        ;;AND R0, R12, #0x30       ;mask T0IR (R12) to include only CAP0 and CAP1 interrupt flags
        ;;ADD R4, R4, R0          ;add in the flag values at bit 5 and 4

        ;Load in lookup table values. Overwrite state (R3) with new state value
        ;;LDMIA R4, {R3,R6,R7,R8}

        ;R4 and R5 are now free.

        ;R3 = new lookup table state
        ;R6 = new capture control register value
        ;R7 = count (+1, -1, or 0)
        ;R8 = active capture register address

        ;;CMP R7, #1
        ;;BEQ test_continue
        ;;CMP R7, #1
;;test_continue

        ;check for valid capture event
        ;;CMP R7, #0
        ;;BEQ cap01_end   ;if zero no valid capture event, no count or timing - finish cap01 handler
                        ;may want to add some kind of error call here. Need to coordinate entry to 
                        ;cap23 so that stack is kept correctly.

        ;;ADD R2, R2, R7  ;update encoder count

        ;;LDR R0, [R8]    ;load capture register time value, freeing R8
        ;;STMIA R9, {R0,R2,R3}  ;Store new time, count, and state back to data struct

        ;;LDR R8, [R10, #CCR_OFFSET]  ;Load old capture control register value
        ;;AND R8, R8, #0xFC0          ;Clear all bits but CAP23 (clear CAP01 and reserve bits)
        ;;ORR R6, R6, R8              ;OR in the new CAP01 CCR bit values
        ;;STR R6, [R10, #CCR_OFFSET]  ;Store updated capture control register value.
        ;R2, R3, R6 and R8 are now free

        ;MOV R6, #0    ;R6 will be the new overflow value
        ;;RSB R6, R0, #0    ;R6 will be the new overflow value = - capture time

        ;Start velocity code

        ;Load in remaining data struct values
        ;;ADD R8, R9, #write_index_01
        ;;LDMIA R8, {R2, R3, R4, R5}

        ;R0 = new time
        ;R1 = prev_time_01
        ;R2 = write_index_01
        ;R3 = overflow_01
        ;R4 = time_array_addr_01
        ;R5 = read_index_01
        ;R6 = new overflow_01
        ;R7 = count direction
        ;R8 = write_index_01 address

        ;;TST R12, #1       ;Simultaneous overflow?
        ;;BEQ cap01_continue
        ;;CMP R0, #16384
        ;;ADDHS R6, #29952
        ;;ADDHS R6, #48
        ;;SUBHS R3, #29952
        ;;SUBHS R3, #48
;;cap01_continue
 
        ;;CMP R2, R5        ;check if ring buffer is full
        ;;BEQ reset_overflow_01     ;skip remainder of velocity code if buffer is full

        ;The next six lines handle the case where the overflow and the capture flags are both set.
        ;(Plus the initialization of R6 to 0, above.)
        ;If the new time value is very low (let's say < 1/2 max timer count), then the overflow happened
        ;before the capture. Then we don't want to change the normal calculation. So, although R6 is
        ;changed to (e.g.) 30,000, it is changed back to zero before being subtracted from overflow R3.
        ;If the new time value is very high (> 1/2 max timer count), then the overflow happened _after_ the
        ;capture interrupt, and therefore properly belongs to the next capture, not this one. So R6 goes to
        ;30,000 and stays there, and is subtracted from R3 overflow. Overflow is reset to 30,000 for the next time.
        ;If no simultaneous overflow occurs, R6 stays at zero, and R3 doesn't change. Overflow reset is to zero. 
  ;      TST R12, #1       ;Simultaneous overflow?
  ;      ADDNE R6, #29952
  ;      ADDNE R6, #48    ;These two should bring R6 to overflow amount (per overflow)
  ;      CMP R0, R6, LSR #1  ;Compare new time R0 to half the overflow amount in R6
  ;      MOVLO R6, #0        ;Don't actually change the overflow value if overflow occurred before capture     
  ;      SUBHS R3, R6        ;Adjust overflow_01 value - if higher, apply to next capture time, not this one

        ;;ADD R0, R0, R3     ;Get elapsed time since previous capture event
        ;;CMP R7, #1          ;Check sign
        ;;RSBNE R0, R0, #0    ;Make negative if count is minus

        ;CMP R7, #1        ;positive count, positive velocity
        ;SUBEQ R0, R0, R1  ;R0 now contains delta time; R1 is now free
        ;ADDEQ R0, R0, R3  ;Add in overflow value from R3.
        ;SUBNE R0, R1, R0  ;'', but negative for negative velocity
        ;SUBNE R0, R0, R3  ;Subtract overflow value from R3         
        ;;STR R0, [R4, R2, LSL #2] ;Save time value to ring buffer.
                                 ;Address is R4 (base) plus (R2 ring buffer write index times 4)
        ;;ADD R2, R2, #1    ;increment write index
        ;;AND R2, R2, #0x7F  ;modulo addition, wraps around ring buffer.
                          ;(ring buffer size must 2^n; ANDed value must be 2^n-1 for this to work)

;;reset_overflow_01
        ;;STMIA R8, {R2,R6} ;save write index and overflow back to data struct

;;cap01_end

;;cap23

;;reg_pop
        ;;LDMFD SP!,{R0-R7}    ;Pop non-FIQ registers used off of stack 
;;clear_int
        ;Clear the Timer0 interrupt flags that were processed (R12)
        ;;STRB R12, [R10,#IR_OFFSET]

;;qdc_tmr0_isr_end

