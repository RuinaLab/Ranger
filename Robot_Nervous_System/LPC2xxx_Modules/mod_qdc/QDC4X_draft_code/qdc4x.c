

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// QDC timer 0 4X assembly code for FIQ handler use. Not fully implemented and tested.
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
        ;Below is 4X test code with hysteresis. CAP01 largely functional, but could use some more optimization as
        ;in the 2X code and addition of CAP23 handling. Somewhat slower than the 2X code, even with the
        ;use of a lookup table. However, can increase speed through use of the tricks added to the 2X code above.
        ;The 4X code should be less susceptible to glitches at direction changes, but at the cost of twice or more the 
        ;processing time, since there are twice as many interrupts.
        ;It's not feasible, unfortunately, to combine the 2X and 4X on the same timer; the two channels are too
        ;interconnected in the code.

        ;start of 4x code
        LDR R9, =qdc_tmr0_data        ;load data struct base address
        LDR R10, =T0_BASE            ;Load timer T0 base address into R10
        LDR R12, [R10, #IR_OFFSET]   ;R12 holds timer IR register interrupt source flags

        ;Check for MR0 flag (overflow/rollover of MR0 value plus 1)
        TST R12, #0x1
        BEQ cap01_check                 ;If not set, no overflow - continue with capture flag handling

        LDR R8, [R9, #overflow_01]    ;load overflow_01 into R8
        CMP R8, #0x7F000000           ;Make sure overflow doesn't overflow! 
        ADDLT R8, R8, #29952          ;Update overflow_01 if not too large; divisible by 2^n
        ADDLT R8, R8, #48             ;Update overflow_01 if not too large; sum to overflow_size
        STRLT R8, [R9, #overflow_01]  ;Store updated overflow_01 value back to data struct
 
        LDR R8, [R9, #overflow_23]    ;load overflow_23 into R8
        CMP R8, #0x7F000000           ;Make sure overflow doesn't overflow!
        ADDLT R8, R8, #29952          ;Update overflow_23 if not too large; divisible by 2^n
        ADDLT R8, R8, #48             ;Update overflow_23 if not too large; sum to overflow_size
        STRLT R8, [R9, #overflow_23]  ;Store updated overflow value back to data struct

        ;Process capture values from CAP0 and CAP1, if flags set
cap01_check
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
                
        TST R12, #0x30            ;Check for capture event on input lines 0 or 1
        BEQ clear_int             ;If none, go to capture event 23 handling
                                  ; **** TEST CODE **** skip cap23
                                  ;Should eventually go to cap23_check

        ;Save variable registers used to stack
        STMFD    SP!,{R0-R7} ;Save non-FIQ registers used to stack
                             ;Note that these registers will be reused by cap23, if
                             ;it so happens that both have interrupts at the same time.
                             ;Need to be sure they are popped again only once.
cap01_do
        LDMIA R9,{R1-R5}     ;load first 5 data struct values

        ;R1 = prev_time_01
        ;R2 = encoder count 01
        ;R3 = state_01
        ;R4 = lookup table address
        ;R5 = FIO0PIN address
        ;R9 = data struct base address
        ;R10 = T0_BASE
        ;R12 = T0IR
        
        LDR R5, [R5]             ;Put pin states in R5 (starts off containing address)

        AND R0, R5, #0x00400000  ;mask off except P0[22], put in R0 scratch register
        ADD R4, R4, R0, LSR #16  ;Add to lookup table address R4 with P0[22] bit now in bit 6 location
        AND R0, R5, #0x08000000  ;mask off except P0[27], put in R0
        ADD R4, R4, R0, LSR #20  ;add to R4 with P0[27] now in bit 7 location
        ADD R4, R4, R3, LSL #8   ;add in the state value, shifted left by 8 to put in bit 9 and bit 8
        AND R0, R12, #0x30       ;mask T0IR (R12) to include only CAP0 and CAP1 interrupt flags
        ADD R4, R4, R0           ;add in the flag values at bit 5 and 4

        ;Load in lookup table values. Overwrite state (R3) with new state value
        LDMIA R4, {R3,R6,R7,R8}

        ;R4 and R5 are now free.

        ;R3 = new lookup table state
        ;R6 = new capture control register value
        ;R7 = count (+1, -1, or 0)
        ;R8 = active capture register address

        CMP R7, #1
        BEQ test_continue
        CMP R7, #1
test_continue

        ;check for valid capture event
        CMP R7, #0
        BEQ cap01_end   ;if zero no valid capture event, no count or timing - finish cap01 handler
                        ;may want to add some kind of error call here. Need to coordinate entry to 
                        ;cap23 so that stack is kept correctly.

        ADD R2, R2, R7  ;update encoder count

        LDR R0, [R8]    ;load capture register time value, freeing R8
        STMIA R9, {R0,R2,R3}  ;Store new time, count, and state back to data struct

        LDR R8, [R10, #CCR_OFFSET]  ;Load old capture control register value
        AND R8, R8, #0xFC0          ;Clear all bits but CAP23 (clear CAP01 and reserve bits)
        ORR R6, R6, R8              ;OR in the new CAP01 CCR bit values
        STR R6, [R10, #CCR_OFFSET]  ;Store updated capture control register value.
        ;R2, R3, R6 and R8 are now free

        ;MOV R6, #0    ;R6 will be the new overflow value
        RSB R6, R0, #0    ;R6 will be the new overflow value = - capture time

        ;Start velocity code

        ;Load in remaining data struct values
        ADD R8, R9, #write_index_01
        LDMIA R8, {R2, R3, R4, R5}

        ;R0 = new time
        ;R1 = prev_time_01
        ;R2 = write_index_01
        ;R3 = overflow_01
        ;R4 = time_array_addr_01
        ;R5 = read_index_01
        ;R6 = new overflow_01
        ;R7 = count direction
        ;R8 = write_index_01 address

        TST R12, #1       ;Simultaneous overflow?
        BEQ cap01_continue
        CMP R0, #16384
        ADDHS R6, #29952
        ADDHS R6, #48
        SUBHS R3, #29952
        SUBHS R3, #48
cap01_continue
 
        CMP R2, R5        ;check if ring buffer is full
        BEQ reset_overflow_01     ;skip remainder of velocity code if buffer is full

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

        ADD R0, R0, R3     ;Get elapsed time since previous capture event
        CMP R7, #1          ;Check sign
        RSBNE R0, R0, #0    ;Make negative if count is minus

        ;CMP R7, #1        ;positive count, positive velocity
        ;SUBEQ R0, R0, R1  ;R0 now contains delta time; R1 is now free
        ;ADDEQ R0, R0, R3  ;Add in overflow value from R3.
        ;SUBNE R0, R1, R0  ;'', but negative for negative velocity
        ;SUBNE R0, R0, R3  ;Subtract overflow value from R3         
        STR R0, [R4, R2, LSL #2] ;Save time value to ring buffer.
                                 ;Address is R4 (base) plus (R2 ring buffer write index times 4)
        ADD R2, R2, #1    ;increment write index
        AND R2, R2, #0x7F  ;modulo addition, wraps around ring buffer.
                          ;(ring buffer size must 2^n; ANDed value must be 2^n-1 for this to work)

reset_overflow_01
        STMIA R8, {R2,R6} ;save write index and overflow back to data struct

cap01_end

cap23

reg_pop
        LDMFD SP!,{R0-R7}    ;Pop non-FIQ registers used off of stack 
clear_int
        ;Clear the Timer0 interrupt flags that were processed (R12)
        STRB R12, [R10,#IR_OFFSET]

qdc_tmr0_isr_end


*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
//QDC 4X initialization function - not fully implemented
void qdc_tmr1_cap01_init(void)
{
  //Initialize qdc data struct
  qdc_tmr0_data.overflow_size = 30000;
  qdc_tmr0_data.tmr_base_addr = 0xE0004000;
  qdc_tmr0_data.FIO0PIN_addr = 0x3FFFC014;

  qdc_tmr0_data.overflow_01 = 0;
  qdc_tmr0_data.encoder_count_01 = 0;
  qdc_tmr0_data.write_index_01 = 1;
  qdc_tmr0_data.read_index_01 = 0;
  qdc_tmr0_data.buffer_mask_01 = QDC_TMR0_CAP01_BUFFER_SIZE - 1;
  qdc_tmr0_data.time_array_addr_01 = (unsigned long)qdc_tmr0_cap01_buffer;

  qdc_tmr0_data.overflow_23 = 0;
  qdc_tmr0_data.encoder_count_23 = 0;
  qdc_tmr0_data.write_index_23 = 1;
  qdc_tmr0_data.read_index_23 = 0;
  qdc_tmr0_data.buffer_mask_23 = QDC_TMR0_CAP23_BUFFER_SIZE - 1;
  qdc_tmr0_data.time_array_addr_23 = (unsigned long)qdc_tmr0_cap23_buffer;

  //Initialize lookup table for 4X QDC (test code, should move to flash if used)  
  //Capture 0 falling edge start state 0
  qdc_lookup[0].cc_reg = 0x6;          //Invalid state, no change to CCR settings
  qdc_lookup[0].count = 0;             //No count
  qdc_lookup[0].state = 0;             //Stay in the same state
  qdc_lookup[0].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[1].cc_reg = 0x28;         //Go to Capture 1 rising edge
  qdc_lookup[1].count = -1;            //Count down
  qdc_lookup[1].state = 3;             //New state
  qdc_lookup[1].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[2].cc_reg = 0x6;          //Invalid state, no change to CCR settings
  qdc_lookup[2].count = 0;             //No count
  qdc_lookup[2].state = 0;             //Stay in the same state
  qdc_lookup[2].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[3].cc_reg = 0x28;         //Go to Capture 1 rising edge
  qdc_lookup[3].count = -1;            //Count down
  qdc_lookup[3].state = 3;             //New state
  qdc_lookup[3].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[4].cc_reg = 0x6;          //Invalid state, no change to CCR settings
  qdc_lookup[4].count = 0;             //No count
  qdc_lookup[4].state = 0;             //Stay in the same state
  qdc_lookup[4].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[5].cc_reg = 0x28;         //Go to Capture 1 rising edge
  qdc_lookup[5].count = -1;            //Count down
  qdc_lookup[5].state = 3;             //New state
  qdc_lookup[5].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[6].cc_reg = 0x6;          //Invalid state, no change to CCR settings
  qdc_lookup[6].count = 0;             //No count
  qdc_lookup[6].state = 0;             //Stay in the same state
  qdc_lookup[6].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[7].cc_reg = 0x28;         //Go to Capture 1 rising edge
  qdc_lookup[7].count = -1;            //Count down
  qdc_lookup[7].state = 3;             //New state
  qdc_lookup[7].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[8].cc_reg = 0x6;          //Invalid state, no change to CCR settings
  qdc_lookup[8].count = 0;             //No count
  qdc_lookup[8].state = 0;             //Stay in the same state
  qdc_lookup[8].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[9].cc_reg = 0x30;         //Go to Capture 1 falling edge
  qdc_lookup[9].count = 1;             //Count up
  qdc_lookup[9].state = 1;             //New state
  qdc_lookup[9].cap_reg = 0xE000402C;        //CR0

  qdc_lookup[10].cc_reg = 0x6;         //Invalid state, no change to CCR settings
  qdc_lookup[10].count = 0;            //No count
  qdc_lookup[10].state = 0;            //Stay in the same state
  qdc_lookup[10].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[11].cc_reg = 0x30;        //Go to Capture 1 falling edge
  qdc_lookup[11].count = 1;            //Count up
  qdc_lookup[11].state = 1;            //New state
  qdc_lookup[11].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[12].cc_reg = 0x6;         //Invalid state, no change to CCR settings
  qdc_lookup[12].count = 0;            //No count
  qdc_lookup[12].state = 0;            //Stay in the same state
  qdc_lookup[12].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[13].cc_reg = 0x30;        //Go to Capture 1 falling edge
  qdc_lookup[13].count = 1;            //Count up
  qdc_lookup[13].state = 1;            //New state
  qdc_lookup[13].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[14].cc_reg = 0x6;         //Invalid state, no change to CCR settings
  qdc_lookup[14].count = 0;            //No count
  qdc_lookup[14].state = 0;            //Stay in the same state
  qdc_lookup[14].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[15].cc_reg = 0x30;        //Go to Capture 1 falling edge
  qdc_lookup[15].count = 1;            //Count up
  qdc_lookup[15].state = 1;            //New state
  qdc_lookup[15].cap_reg = 0xE000402C;       //CR0

  //Capture 1 falling edge start state 1
  qdc_lookup[16].cc_reg = 0x30;        //Invalid state, no change to CCR settings
  qdc_lookup[16].count = 0;            //No count
  qdc_lookup[16].state = 1;            //Stay in the same state
  qdc_lookup[16].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[17].cc_reg = 0x30;        //Invalid state, no change to CCR settings
  qdc_lookup[17].count = 0;            //No count
  qdc_lookup[17].state = 1;            //Stay in the same state
  qdc_lookup[17].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[18].cc_reg = 0x5;         //Go to Capture 0 rising edge
  qdc_lookup[18].count = 1;            //Count up
  qdc_lookup[18].state = 2;            //New state
  qdc_lookup[18].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[19].cc_reg = 0x5;         //Go to Capture 0 rising edge
  qdc_lookup[19].count = 1;            //Count up
  qdc_lookup[19].state = 2;            //New state
  qdc_lookup[19].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[20].cc_reg = 0x30;        //Invalid state, no change to CCR settings
  qdc_lookup[20].count = 0;            //No count
  qdc_lookup[20].state = 1;            //Stay in the same state
  qdc_lookup[20].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[21].cc_reg = 0x30;        //Invalid state, no change to CCR settings
  qdc_lookup[21].count = 0;            //No count
  qdc_lookup[21].state = 1;            //Stay in the same state
  qdc_lookup[21].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[22].cc_reg = 0x6;         //Go to Capture 0 falling edge
  qdc_lookup[22].count = -1;           //Count down
  qdc_lookup[22].state = 0;            //Got to new state
  qdc_lookup[22].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[23].cc_reg = 0x6;         //Go to Capture 0 falling edge
  qdc_lookup[23].count = -1;           //Count down
  qdc_lookup[23].state = 0;            //Got to new state
  qdc_lookup[23].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[24].cc_reg = 0x30;        //Invalid state, no change to CCR settings
  qdc_lookup[24].count = 0;            //No count
  qdc_lookup[24].state = 1;            //Stay in the same state
  qdc_lookup[24].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[25].cc_reg = 0x30;        //Invalid state, no change to CCR settings
  qdc_lookup[25].count = 0;            //No count
  qdc_lookup[25].state = 1;            //Stay in the same state
  qdc_lookup[25].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[26].cc_reg = 0x5;         //Go to Capture 0 rising edge
  qdc_lookup[26].count = 1;            //Count up
  qdc_lookup[26].state = 2;            //Go to new state
  qdc_lookup[26].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[27].cc_reg = 0x5;         //Go to Capture 0 rising edge
  qdc_lookup[27].count = 1;            //Count up
  qdc_lookup[27].state = 2;            //Go to new state
  qdc_lookup[27].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[28].cc_reg = 0x30;        //Invalid state, no change to CCR settings
  qdc_lookup[28].count = 0;            //No count
  qdc_lookup[28].state = 1;            //Stay in the same state
  qdc_lookup[28].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[29].cc_reg = 0x30;        //Invalid state, no change to CCR settings
  qdc_lookup[29].count = 0;            //No count
  qdc_lookup[29].state = 1;            //Stay in the same state
  qdc_lookup[29].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[30].cc_reg = 0x6;         //Go to Capture 0 falling edge
  qdc_lookup[30].count = -1;           //Count down
  qdc_lookup[30].state = 0;            //Go to new state
  qdc_lookup[30].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[31].cc_reg = 0x6;         //Go to Capture 0 falling edge
  qdc_lookup[31].count = -1;           //Count down
  qdc_lookup[31].state = 0;            //Go to new state
  qdc_lookup[31].cap_reg = 0xE0004030;       //CR1

  //Capture 0 rising edge start state 2
  qdc_lookup[32].cc_reg = 0x5;         //Invalid state, no change to CCR settings
  qdc_lookup[32].count = 0;            //No count
  qdc_lookup[32].state = 2;            //Stay in the same state
  qdc_lookup[32].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[33].cc_reg = 0x28;        //Go to Capture 1 rising edge
  qdc_lookup[33].count = 1;            //Count up
  qdc_lookup[33].state = 3;            //Go to new state
  qdc_lookup[33].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[34].cc_reg = 0x5;         //Invalid state, no change to CCR settings
  qdc_lookup[34].count = 0;            //No count
  qdc_lookup[34].state = 2;            //Stay in the same state
  qdc_lookup[34].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[35].cc_reg = 0x28;        //Go to Capture 1 rising edge
  qdc_lookup[35].count = 1;            //Count up
  qdc_lookup[35].state = 3;            //Go to new state
  qdc_lookup[35].cap_reg = 0xE000402C;       //CR0
                                    
  qdc_lookup[36].cc_reg = 0x5;         //Invalid state, no change to CCR settings
  qdc_lookup[36].count = 0;            //No count
  qdc_lookup[36].state = 2;            //Stay in the same state
  qdc_lookup[36].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[37].cc_reg = 0x28;        //Go to Capture 1 rising edge
  qdc_lookup[37].count = 1;            //Count up
  qdc_lookup[37].state = 3;            //Go to new state
  qdc_lookup[37].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[38].cc_reg = 0x5;         //Invalid state, no change to CCR settings
  qdc_lookup[38].count = 0;            //No count
  qdc_lookup[38].state = 2;            //Stay in the same state
  qdc_lookup[38].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[39].cc_reg = 0x28;        //Go to Capture 1 rising edge
  qdc_lookup[39].count = 1;            //Count up
  qdc_lookup[39].state = 3;            //Go to new state
  qdc_lookup[39].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[40].cc_reg = 0x5;        //Invalid state, no change to CCR settings
  qdc_lookup[40].count = 0;            //No count
  qdc_lookup[40].state = 2;            //Stay in the same state
  qdc_lookup[40].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[41].cc_reg = 0x30;        //Go to Capture 1 falling edge
  qdc_lookup[41].count = -1;           //Count down
  qdc_lookup[41].state = 1;            //Go to new state
  qdc_lookup[41].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[42].cc_reg = 0x5;         //Invalid state, no change to CCR settings
  qdc_lookup[42].count = 0;            //No count
  qdc_lookup[42].state = 2;            //Stay in the same state
  qdc_lookup[42].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[43].cc_reg = 0x30;        //Go to Capture 1 falling edge
  qdc_lookup[43].count = -1;           //Count down
  qdc_lookup[43].state = 1;            //Go to new state
  qdc_lookup[43].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[44].cc_reg = 0x5;         //Invalid state, no change to CCR settings
  qdc_lookup[44].count = 0;            //No count
  qdc_lookup[44].state = 2;            //Stay in the same state
  qdc_lookup[44].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[45].cc_reg = 0x30;        //Go to Capture 1 falling edge
  qdc_lookup[45].count = -1;           //Count down
  qdc_lookup[45].state = 1;            //Go to new state
  qdc_lookup[45].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[46].cc_reg = 0x5;         //Invalid state, no change to CCR settings
  qdc_lookup[46].count = 0;            //No count
  qdc_lookup[46].state = 2;            //Stay in the same state
  qdc_lookup[46].cap_reg = 0xE000402C;       //CR0

  qdc_lookup[47].cc_reg = 0x30;        //Go to Capture 1 falling edge
  qdc_lookup[47].count = -1;           //Count down
  qdc_lookup[47].state = 1;            //Go to new state
  qdc_lookup[47].cap_reg = 0xE000402C;  //CR0

  //Capture 1 rising edge start state 3
  qdc_lookup[48].cc_reg = 0x28;        //Invalid state, no change to CCR settings
  qdc_lookup[48].count = 0;            //No count
  qdc_lookup[48].state = 3;            //Stay in the same state
  qdc_lookup[48].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[49].cc_reg = 0x28;        //Invalid state, no change to CCR settings
  qdc_lookup[49].count = 0;            //No count
  qdc_lookup[49].state = 3;            //Stay in the same state
  qdc_lookup[49].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[50].cc_reg = 0x5;         //Go to Capture 0 rising edge
  qdc_lookup[50].count = -1;           //Count down
  qdc_lookup[50].state = 2;            //Go to new state
  qdc_lookup[50].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[51].cc_reg = 0x5;         //Go to Capture 0 rising edge
  qdc_lookup[51].count = -1;           //Count down
  qdc_lookup[51].state = 2;            //Go to new state
  qdc_lookup[51].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[52].cc_reg = 0x28;        //Invalid state, no change to CCR settings
  qdc_lookup[52].count = 0;            //No count
  qdc_lookup[52].state = 3;            //Stay in the same state
  qdc_lookup[52].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[53].cc_reg = 0x28;       //Invalid state, no change to CCR settings
  qdc_lookup[53].count = 0;            //No count
  qdc_lookup[53].state = 3;            //Stay in the same state
  qdc_lookup[53].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[54].cc_reg = 0x6;         //Go to Capture 0 falling edge
  qdc_lookup[54].count = 1;            //Count up
  qdc_lookup[54].state = 0;            //Go to new state
  qdc_lookup[54].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[55].cc_reg = 0x6;         //Go to Capture 0 falling edge
  qdc_lookup[55].count = 1;            //Count up
  qdc_lookup[55].state = 0;            //Go to new state
  qdc_lookup[55].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[56].cc_reg = 0x28;        //Invalid state, no change to CCR settings
  qdc_lookup[56].count = 0;            //No count
  qdc_lookup[56].state = 3;            //Stay in the same state
  qdc_lookup[56].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[57].cc_reg = 0x28;        //Invalid state, no change to CCR settings
  qdc_lookup[57].count = 0;            //No count
  qdc_lookup[57].state = 3;            //Stay in the same state
  qdc_lookup[57].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[58].cc_reg = 0x5;         //Go to Capture 0 rising edge
  qdc_lookup[58].count = -1;           //Count down
  qdc_lookup[58].state = 2;            //Go to new state
  qdc_lookup[58].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[59].cc_reg = 0x5;         //Go to Capture 0 rising edge
  qdc_lookup[59].count = -1;           //Count down
  qdc_lookup[59].state = 2;            //Go to new state
  qdc_lookup[59].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[60].cc_reg = 0x28;        //Invalid state, no change to CCR settings
  qdc_lookup[60].count = 0;            //No count
  qdc_lookup[60].state = 3;            //Stay in the same state
  qdc_lookup[60].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[61].cc_reg = 0x28;        //Invalid state, no change to CCR settings
  qdc_lookup[61].count = 0;            //No count
  qdc_lookup[61].state = 3;            //Stay in the same state
  qdc_lookup[61].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[62].cc_reg = 0x6;         //Go to Capture 0 falling edge
  qdc_lookup[62].count = 1;            //Count up
  qdc_lookup[62].state = 0;            //Go to new state
  qdc_lookup[62].cap_reg = 0xE0004030;       //CR1

  qdc_lookup[63].cc_reg = 0x6;         //Go to Capture 0 falling edge
  qdc_lookup[63].count = 1;            //Count up
  qdc_lookup[63].state = 0;            //Go to new state
  qdc_lookup[63].cap_reg = 0xE0004030;       //CR1
}
*/

//For header file:
/*
// For 4X code, preliminary version
typedef struct qdc_data{
  signed long   prev_time_01;
  signed long   encoder_count_01;
  unsigned long state_01;
  unsigned long lookup_table_addr_01;
  unsigned long FIO0PIN_addr_01;
  unsigned long write_index_01;
  signed long   overflow_01;
  unsigned long time_array_addr_01;
  unsigned long read_index_01;
      
  signed long   prev_time_23;
  signed long   encoder_count_23;
  unsigned long state_23;
  unsigned long lookup_table_addr_23;
  unsigned long FIO0PIN_addr_23;
  unsigned long write_index_23;
  signed long   overflow_23;
  unsigned long time_array_addr_23;
  unsigned long read_index_23;
  
//  unsigned long timer_base_addr;   

} QDC_DATA;
*/

