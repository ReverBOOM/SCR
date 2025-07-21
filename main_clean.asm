; File:   main.asm
; Author: sui
; Created on 15 July 2025, 14:05
; SCR Control System with ZCD and Timer-based delays for PIC18F4523

        LIST P=18F4523
        #include <p18f4523.inc>

        ; Configuration bits
        CONFIG OSC = INTIO67        ; Internal oscillator
        CONFIG WDT = OFF            ; Watchdog timer off
        CONFIG LVP = OFF            ; Low voltage programming off
        CONFIG PBADEN = OFF         ; PORTB<4:0> pins are configured as digital I/O on Reset
        CONFIG FCMEN = OFF          ; Fail-Safe Clock Monitor disabled
        CONFIG IESO = OFF           ; Oscillator Switchover mode disabled
        CONFIG PWRT = OFF           ; Power-up Timer disabled
        CONFIG BOREN = SBORDIS      ; Brown-out Reset disabled in hardware and software
        CONFIG BORV = 3             ; Brown-out Reset Voltage bits
        CONFIG WDTPS = 32768        ; Watchdog Timer Postscale Select bits
        CONFIG MCLRE = ON           ; MCLR pin enabled
        CONFIG LPT1OSC = OFF        ; Timer1 configured for higher power operation
        CONFIG CCP2MX = PORTC       ; CCP2 input/output is multiplexed with RC1
        CONFIG STVREN = ON          ; Stack full/underflow will cause Reset
        CONFIG XINST = OFF          ; Instruction set extension and Indexed Addressing mode disabled
        CONFIG DEBUG = OFF          ; Background debugger disabled
        CONFIG CP0 = OFF            ; Code Protection bit
        CONFIG CP1 = OFF            ; Code Protection bit
        CONFIG CP2 = OFF            ; Code Protection bit
        CONFIG CP3 = OFF            ; Code Protection bit
        CONFIG CPB = OFF            ; Boot Block Code Protection bit
        CONFIG CPD = OFF            ; Data EEPROM Code Protection bit
        CONFIG WRT0 = OFF           ; Write Protection bit
        CONFIG WRT1 = OFF           ; Write Protection bit
        CONFIG WRT2 = OFF           ; Write Protection bit
        CONFIG WRT3 = OFF           ; Write Protection bit
        CONFIG WRTB = OFF           ; Boot Block Write Protection bit
        CONFIG WRTC = OFF           ; Configuration Register Write Protection bit
        CONFIG WRTD = OFF           ; Data EEPROM Write Protection bit
        CONFIG EBTR0 = OFF          ; Table Read Protection bit
        CONFIG EBTR1 = OFF          ; Table Read Protection bit
        CONFIG EBTR2 = OFF          ; Table Read Protection bit
        CONFIG EBTR3 = OFF          ; Table Read Protection bit
        CONFIG EBTRB = OFF          ; Boot Block Table Read Protection bit

        ;==============================================================================
        ; VARIABLE DEFINITIONS
        ;==============================================================================
        CBLOCK 0x20
            zcd_flags           ; Stores which ZCD triggered (bit 0–5)
            scr_output_index    ; Index for RD0–RD5 (which SCR to fire)
            delay_value         ; Delay from ADC (scaled)
            old_portb           ; To detect rising edges
            temp                ; Temporary variable
        ENDC
        
        ; Additional variables for multiplication
        CBLOCK 0x2F
            mtemp1              ; 0x2F - Temporary for multiplication
            mresult_low         ; 0x30 - Low byte of multiplication result
            mresult_high        ; 0x31 - High byte of multiplication result
            mmultiplier         ; 0x32 - Multiplier value
            mcounter            ; 0x33 - Loop counter
            delay_temp1         ; 0x34 - Delay temporary
            delay_temp2         ; 0x35 - Delay temporary
        ENDC

        ;==============================================================================
        ; INTERRUPT VECTORS
        ;==============================================================================
        ORG 0x0000
        goto    INIT

        ORG 0x0008
        goto    ISR

        ;==============================================================================
        ; INTERRUPT SERVICE ROUTINE
        ;==============================================================================
ISR:
        ; Check IOC (PORTB change)
        btfss   INTCON, RBIF
        goto    CHECK_TMR1

        ; Handle PORTB change interrupt
        movf    PORTB, W
        movwf   temp
        xorwf   old_portb, W
        andwf   PORTB, W            ; Rising edges only
        movwf   zcd_flags
        movf    PORTB, W
        movwf   old_portb
        bcf     INTCON, RBIF

        ; Read ADC
        bsf     ADCON0, GO
WAIT_ADC:
        btfsc   ADCON0, GO
        goto    WAIT_ADC
        movf    ADRESL, W
        movwf   delay_value         ; Use ADRESL (8-bit)

        ; Convert delay_value * 31 (approximate scaling for 0–8ms)
        movf    delay_value, W
        movwf   mresult_low         ; multiplicand
        clrf    mresult_high
        movlw   .31
        movwf   mmultiplier
        clrf    mcounter
        call    MUL8                ; result in mresult_high:mresult_low

        ; Calc preload = 65536 - delay
        comf    mresult_low, F
        comf    mresult_high, F
        incf    mresult_low, F
        btfsc   STATUS, Z
        incf    mresult_high, F

        ; Load Timer1
        movf    mresult_high, W
        movwf   TMR1H
        movf    mresult_low, W
        movwf   TMR1L

        ; Copy SCR index
        movf    zcd_flags, W
        movwf   scr_output_index

        ; Start Timer1
        bcf     PIR1, TMR1IF
        bsf     PIE1, TMR1IE
        bsf     T1CON, TMR1ON

        goto    ISR_DONE

CHECK_TMR1:
        btfss   PIR1, TMR1IF
        goto    ISR_DONE

        ; Timer1 done, trigger SCR
        bcf     PIR1, TMR1IF
        bcf     PIE1, TMR1IE
        bcf     T1CON, TMR1ON

        ; Output SCR pulse (on RDx)
        movf    scr_output_index, W
        movwf   LATD

        call    DELAY_2MS

        clrf    LATD

ISR_DONE:
        retfie

        ;==============================================================================
        ; MAIN PROGRAM INITIALIZATION
        ;==============================================================================
INIT:
        ; Configure I/O ports
        clrf    TRISA
        clrf    TRISD
        movlw   0x3F
        movwf   TRISB               ; RB0–RB5 input (ZCD)
        bsf     TRISA, 0            ; RA0 = input

        clrf    LATD

        ; Save initial PORTB state
        movf    PORTB, W
        movwf   old_portb

        ; ADC setup
        movlw   b'00000001'         ; ADCON0: AN0 enabled
        movwf   ADCON0
        movlw   b'00001110'         ; ADCON1: AN0 analog, rest digital
        movwf   ADCON1
        movlw   b'10111111'         ; ADCON2: Right justified, 20 TAD, FOSC/128
        movwf   ADCON2
        bsf     ADCON0, ADON

        ; Timer1 setup (16-bit, Fosc/4, 1:8 prescaler)
        movlw   b'00110000'
        movwf   T1CON

        ; Enable Interrupts
        bsf     INTCON, GIE         ; Enable global interrupts
        bsf     INTCON, PEIE        ; Enable peripheral interrupts
        bsf     INTCON, RBIE        ; Enable IOC for PORTB

        ;==============================================================================
        ; MAIN LOOP
        ;==============================================================================
MAIN_LOOP:
        goto    MAIN_LOOP

        ;==============================================================================
        ; Delay 2ms (Polling loop)
        ;==============================================================================
DELAY_2MS:
        movlw   D'8'                ; Outer loop count for ~2ms
        movwf   delay_temp2
D1:
        movlw   D'250'              ; Inner loop count
        movwf   delay_temp1
D2:
        nop
        decfsz  delay_temp1, f
        goto    D2
        decfsz  delay_temp2, f
        goto    D1
        return

        ;==============================================================================
        ; MUL8 — Multiply 8-bit multiplication using shift and add
        ; Input: mresult_low = multiplicand, mmultiplier = multiplier
        ; Output: mresult_high:mresult_low = 16-bit result
        ;==============================================================================
MUL8:
        clrf    mresult_high        ; Clear high byte of result
        movf    mresult_low, W      ; Get multiplicand
        movwf   mtemp1              ; Store multiplicand in temp
        clrf    mresult_low         ; Clear low byte of result
        movlw   8                   ; Set loop counter
        movwf   mcounter
MUL_LOOP:
        bcf     STATUS, C           ; Clear carry flag
        rrf     mmultiplier, F      ; Shift multiplier right, LSB to carry
        btfss   STATUS, C           ; Test carry flag
        bra     SKIP_ADD            ; Skip if bit was 0
        ; Add multiplicand to result
        movf    mtemp1, W           ; Get multiplicand
        addwf   mresult_low, F      ; Add to low byte
        btfsc   STATUS, C           ; Test carry flag
        incf    mresult_high, F     ; Add carry to high byte
SKIP_ADD:
        bcf     STATUS, C           ; Clear carry flag
        rlf     mtemp1, F           ; Shift multiplicand left
        btfsc   STATUS, C           ; Test carry flag
        incf    mresult_high, F     ; Add carry to high byte
        decfsz  mcounter, F         ; Decrement and test counter
        bra     MUL_LOOP            ; Loop if not zero
        return                      ; Return to caller

        END
