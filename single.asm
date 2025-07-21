; single.asm - Single Phase SCR Firing Controller for PIC18F4523
; - Measures AC frequency from digital input (zero-cross)
; - Analog input (AN0) sets firing angle
; - Output pulse (1ms) on RB0 for each half-cycle
; - Firing delay + 1ms <= half-cycle duration

    LIST    P=18F4523
    #include <p18f4523.inc>

; CONFIGURATION BITS (set as needed)
    CONFIG  OSC = HS          ; High-Speed Oscillator
    CONFIG  PWRT = ON         ; Power-up Timer Enable
    CONFIG  BOREN = OFF       ; Brown-out Reset Disable (correct directive)
    CONFIG  WDT = OFF         ; Watchdog Timer Disable
    CONFIG  MCLRE = ON        ; MCLR Pin Enable

;-------------------------------
; RAM VARIABLES
;-------------------------------
    CBLOCK  0x20
        freq_periodL     ; Low byte of measured half-cycle period
        freq_periodH     ; High byte
        fire_delayL      ; Calculated firing delay (low)
        fire_delayH      ; High
        adc_valueL       ; ADC result low
        adc_valueH       ; High
        state            ; State machine: 0=wait zero, 1=wait delay, 2=fire
    ENDC

;-------------------------------
; CONSTANTS
;-------------------------------
FOSC        EQU 20000000      ; 20MHz
TMR1_PRESC  EQU 8             ; Timer1 prescaler
; TMR1_TICK = 1.6us (Timer1 tick period - comment only)
ON_TIME_MS  EQU 1             ; Output pulse width (ms)
ON_TIME_TICKS EQU 625         ; 1ms in Timer1 ticks (approx 625)

;-------------------------------
; RESET VECTOR
;-------------------------------
    ORG     0x00
    goto    main

;-------------------------------
; INTERRUPT VECTOR
;-------------------------------
    ORG     0x08
    goto    isr

;-------------------------------
; MAIN PROGRAM
;-------------------------------
main:
    clrf    state

    ; Setup ports
    bcf     TRISB, 0       ; RB0 as output (SCR trigger)
    bsf     TRISB, 1       ; RB1 as input (zero-cross)
    bsf     TRISA, 0       ; RA0 as input (analog)
    
    clrf    LATB           ; Initialize LATB (RB0 = 0)
    clrf    LATA           ; Initialize LATA

    ; Setup ADC
    movlw   b'00000001'    ; Channel AN0 selected
    movwf   ADCON0
    movlw   b'00001110'    ; AN0 analog, rest digital
    movwf   ADCON1
    movlw   b'10111110'    ; Right justified, 20 TAD, FOSC/64
    movwf   ADCON2
    bsf     ADCON0, ADON   ; Enable ADC

    ; Setup Timer1
    movlw   b'00110001'    ; 16-bit, prescaler 1:8, internal clock
    movwf   T1CON

    ; Enable interrupts
    bsf     PIE1, TMR1IE   ; Timer1 interrupt enable
    bsf     INTCON, PEIE
    bsf     INTCON, GIE

main_loop:
    ; Wait for zero-cross (RB1 rising edge)
    btfss   PORTB, 1
    goto    main_loop
wait_fall:
    btfsc   PORTB, 1
    goto    wait_fall

    ; Start Timer1 for period measurement
    clrf    TMR1L
    clrf    TMR1H
    bsf     T1CON, TMR1ON

    ; Wait for next zero-cross (RB1 rising edge)
wait_rise:
    btfss   PORTB, 1
    goto    wait_rise

    ; Stop Timer1, read period
    bcf     T1CON, TMR1ON
    movf    TMR1L, W
    movwf   freq_periodL
    movf    TMR1H, W
    movwf   freq_periodH

    ; Start ADC conversion (AN0)
    ; Add small delay for ADC settling
    nop
    nop
    nop
    nop
    bsf     ADCON0, GO
adc_wait:
    btfsc   ADCON0, GO
    goto    adc_wait
    movf    ADRESL, W
    movwf   adc_valueL
    movf    ADRESH, W
    movwf   adc_valueH

    ; Calculate firing delay:
    ; delay = (adc_value / 1023) * (period - ON_TIME_TICKS)
    ; For simplicity, use high byte of ADC as 0-255
    movf    freq_periodL, W
    movwf   fire_delayL
    movf    freq_periodH, W
    movwf   fire_delayH

    ; Subtract ON_TIME_TICKS from period
    movlw   LOW(ON_TIME_TICKS)
    subwf   fire_delayL, F
    movlw   HIGH(ON_TIME_TICKS)
    subwfb  fire_delayH, F

    ; Scale by adc_valueH (approximate)
    ; fire_delayL = (fire_delayL * adc_valueH) / 256
    movf    adc_valueH, W
    mulwf   fire_delayL     ; fire_delayL * adc_valueH
    movf    PRODH, W        ; Use high byte as scaled result
    movwf   fire_delayL     ; Store back to fire_delayL
    clrf    fire_delayH     ; Clear high byte for simple delay

    ; Wait for firing delay using Timer1 countdown
    ; Load Timer1 with (65536 - fire_delayL) for countdown
    comf    fire_delayL, W
    movwf   TMR1L
    movf    fire_delayH, W
    btfsc   STATUS, Z
    decf    fire_delayH, W
    comf    fire_delayH, W
    movwf   TMR1H
    bcf     PIR1, TMR1IF
    bsf     T1CON, TMR1ON
wait_delay:
    btfss   PIR1, TMR1IF
    goto    wait_delay
    bcf     PIR1, TMR1IF
    bcf     T1CON, TMR1ON

    ; Fire SCR: RB0 high for 1ms
    bsf     LATB, 0
    ; Load Timer1 with countdown for 1ms pulse
    movlw   LOW(65536-ON_TIME_TICKS)
    movwf   TMR1L
    movlw   HIGH(65536-ON_TIME_TICKS)
    movwf   TMR1H
    bcf     PIR1, TMR1IF
    bsf     T1CON, TMR1ON
wait_on:
    btfss   PIR1, TMR1IF
    goto    wait_on
    bcf     PIR1, TMR1IF
    bcf     T1CON, TMR1ON
    bcf     LATB, 0

    goto    main_loop

;-------------------------------
; INTERRUPT SERVICE ROUTINE
;-------------------------------
isr:
    ; (No interrupt code needed for polling version)
    retfie

    END