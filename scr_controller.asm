; SCR Controller for PIC18F4523
; Converted from Arduino code
; Pin assignments:
; - RB0/INT0: Zero-cross detection input (equivalent to Arduino pin 2)
; - RA0/AN0: Analog input (equivalent to Arduino A0)
; - RD0: SCR trigger output (equivalent to Arduino pin 9)

    LIST P=18F4523
    #include <p18f4523.inc>

; Configuration bits
    CONFIG OSC = INTIO67        ; Internal oscillator 8MHz
    CONFIG WDT = OFF            ; Watchdog timer off
    CONFIG LVP = OFF            ; Low voltage programming off
    CONFIG PBADEN = OFF         ; PORTB digital I/O
    CONFIG MCLRE = ON           ; MCLR pin enabled
    CONFIG XINST = OFF          ; Extended instruction set off
    CONFIG DEBUG = OFF          ; Debug mode off
    CONFIG PWRT = ON            ; Power-up timer on
    CONFIG BOREN = OFF          ; Brown-out reset off

;==============================================================================
; VARIABLE DEFINITIONS
;==============================================================================
    CBLOCK 0x20
        trigger_flag        ; Equivalent to volatile bool trigger
        analog_value_L      ; Low byte of ADC result (0-1023)
        analog_value_H      ; High byte of ADC result
        delay_time_L        ; Low byte of delay time in Timer1 ticks
        delay_time_H        ; High byte of delay time
        temp_w              ; Temporary W register storage
        temp_status         ; Temporary STATUS register storage
        map_temp            ; Temporary variable for mapping calculation
        pulse_counter       ; Counter for 2ms pulse timing
    ENDC

;==============================================================================
; CONSTANTS
;==============================================================================
; For 8MHz internal oscillator with Timer1 prescaler 1:1: Timer1 tick = 0.5µs
; 1ms = 1000µs = 2000 Timer1 ticks
; 10ms = 10000µs = 20000 Timer1 ticks
MAX_DELAY_TICKS EQU 20000   ; Maximum delay (10ms in Timer1 ticks)
PULSE_TICKS     EQU 2000    ; 1ms pulse width (2000 Timer1 ticks)
MIN_PULSE_TICKS EQU 1000    ; Minimum pulse width (0.5ms = 1000 ticks)
MAX_TOTAL_TICKS EQU 20000   ; Maximum total time for delay + pulse (10ms)

;==============================================================================
; INTERRUPT VECTORS
;==============================================================================
    ORG 0x0000
    goto main

    ORG 0x0008
    goto isr_high

    ORG 0x0018
    goto isr_low

;==============================================================================
; INTERRUPT SERVICE ROUTINE
;==============================================================================
isr_high:
    ; Save context
    movwf   temp_w
    movf    STATUS, W
    movwf   temp_status

    ; Check INT0 interrupt (zero-cross detection)
    btfss   INTCON, INT0IF
    goto    isr_exit

    ; Set trigger flag (equivalent to trigger = true)
    bsf     trigger_flag, 0

    ; Clear INT0 interrupt flag
    bcf     INTCON, INT0IF

isr_exit:
    ; Restore context
    movf    temp_status, W
    movwf   STATUS
    movf    temp_w, W
    retfie

isr_low:
    retfie

;==============================================================================
; MAIN PROGRAM
;==============================================================================
main:
    ; Initialize oscillator to 8MHz
    movlw   b'01110000'     ; 8MHz internal oscillator
    movwf   OSCCON

    ; Initialize variables
    clrf    trigger_flag
    clrf    analog_value_L
    clrf    analog_value_H
    clrf    delay_time_L
    clrf    delay_time_H

    ; Setup I/O ports
    call    setup_ports

    ; Setup ADC
    call    setup_adc

    ; Setup Timer1
    call    setup_timer1

    ; Setup interrupts
    call    setup_interrupts

    ; Main loop (equivalent to Arduino loop())
main_loop:
    ; Check trigger flag (equivalent to if (trigger))
    btfss   trigger_flag, 0
    goto    main_loop

    ; Clear trigger flag (equivalent to trigger = false)
    bcf     trigger_flag, 0

    ; Read analog value (equivalent to analogRead(ANALOG_PIN))
    call    read_analog

    ; Map analog value to delay time
    call    map_delay

    ; Wait for delay time
    call    wait_delay

    ; Generate 1ms pulse (synchronized with zero-cross)
    call    generate_pulse

    goto    main_loop

;==============================================================================
; SETUP FUNCTIONS
;==============================================================================
setup_ports:
    ; Setup PORTB
    bsf     TRISB, 0        ; RB0/INT0 as input (zero-cross)
    
    ; Enable weak pull-ups on PORTB
    bcf     INTCON2, RBPU   ; Enable PORTB weak pull-ups
    
    ; Setup PORTA
    bsf     TRISA, 0        ; RA0/AN0 as input (analog)
    
    ; Setup PORTD
    bcf     TRISD, 0        ; RD0 as output (SCR trigger)
    
    ; Initialize output states
    clrf    LATD
    bcf     LATD, 0         ; RD0 = LOW initially
    
    return

setup_adc:
    ; Configure ADC for RA0/AN0
    movlw   b'00000001'     ; Select AN0 channel
    movwf   ADCON0
    
    movlw   b'00001110'     ; AN0 analog, rest digital
    movwf   ADCON1
    
    movlw   b'10111110'     ; Right justified, 20 TAD, FOSC/64
    movwf   ADCON2
    
    bsf     ADCON0, ADON    ; Enable ADC
    
    return

setup_timer1:
    ; Configure Timer1 for delay timing
    ; Timer1 with 8MHz internal oscillator and prescaler 1:1
    ; Timer1 tick = 4/8MHz = 0.5µs per tick
    ; 1ms = 1000µs = 2000 Timer1 ticks
    movlw   b'00000001'     ; 16-bit mode, prescaler 1:1, internal clock
    movwf   T1CON
    
    return

setup_interrupts:
    ; Setup INT0 for falling edge (proper zero-cross detection)
    bcf     INTCON2, INTEDG0    ; Falling edge trigger
    
    ; Enable INT0 interrupt
    bcf     INTCON, INT0IF      ; Clear interrupt flag
    bsf     INTCON, INT0IE      ; Enable INT0 interrupt
    
    ; Enable global interrupts
    bsf     INTCON, GIE         ; Global interrupt enable
    
    return

;==============================================================================
; ANALOG READ FUNCTION
;==============================================================================
read_analog:
    ; Start ADC conversion
    bsf     ADCON0, GO
    
wait_adc:
    ; Wait for conversion to complete
    btfsc   ADCON0, GO
    goto    wait_adc
    
    ; Read 10-bit result
    movf    ADRESL, W
    movwf   analog_value_L
    movf    ADRESH, W
    movwf   analog_value_H
    
    return

;==============================================================================
; MAP DELAY FUNCTION
;==============================================================================
map_delay:
    ; Map analog value (0-1023) to delay time (0-9ms max)
    ; Use simpler approach: multiply by smaller factor to prevent early saturation
    ; Target: smooth linear mapping across full voltage range
    ; Use only the low 8 bits for smoother control
    
    ; Use analog_value_L (0-255) and multiply by a reasonable factor
    ; 255 * 70 = 17850 (close to 18000 max)
    ; This gives smooth progression without early saturation
    
    movlw   70                  ; Scaling factor
    mulwf   analog_value_L      ; Multiply low byte only
    
    ; Store result
    movf    PRODL, W
    movwf   delay_time_L
    movf    PRODH, W
    movwf   delay_time_H
    
    ; Check if we exceeded 18000 (0x4650)
    movf    delay_time_H, W
    sublw   0x46                ; Compare with high byte of 18000
    btfss   STATUS, C
    goto    limit_delay         ; If carry clear, we exceeded limit
    
    ; If high bytes are equal, check low byte
    movf    delay_time_H, W
    sublw   0x46
    btfss   STATUS, Z
    return                      ; If not equal, we're within limits
    
    ; High bytes equal, check low byte
    movf    delay_time_L, W
    sublw   0x50                ; Compare with low byte of 18000
    btfss   STATUS, C
    goto    limit_delay         ; If carry clear, we exceeded limit
    
    return                      ; Within limits
    
limit_delay:
    ; Limit delay to 18000 ticks (9ms)
    movlw   0x50                ; Low byte of 18000
    movwf   delay_time_L
    movlw   0x46                ; High byte of 18000
    movwf   delay_time_H
    
    return

;==============================================================================
; WAIT DELAY FUNCTION
;==============================================================================
wait_delay:
    ; Check if delay is zero
    movf    delay_time_L, W
    iorwf   delay_time_H, W
    btfsc   STATUS, Z
    return                      ; Return if delay is zero
    
    ; Load Timer1 with countdown value (65536 - delay_time + 1)
    ; Adding 1 to compensate for the reload overhead
    comf    delay_time_L, W
    addlw   1
    movwf   TMR1L
    comf    delay_time_H, W
    addlw   0
    movwf   TMR1H
    
    ; Clear Timer1 interrupt flag and start timer
    bcf     PIR1, TMR1IF
    bsf     T1CON, TMR1ON
    
wait_timer1:
    ; Wait for Timer1 overflow
    btfss   PIR1, TMR1IF
    goto    wait_timer1
    
    ; Stop Timer1 and clear flag
    bcf     T1CON, TMR1ON
    bcf     PIR1, TMR1IF
    
    return

;==============================================================================
; GENERATE PULSE FUNCTION (Fixed 1ms pulse width - ACTIVE HIGH)
;==============================================================================
generate_pulse:
    ; Set RD0 HIGH (active-high SCR trigger pulse)
    bsf     LATD, 0
    
    ; Load Timer1 for fixed 1ms pulse duration (2000 ticks with 1:1 prescaler)
    ; Use direct reload method: 65536 - 2000 = 63536 (0xF830)
    movlw   0x30                ; LOW byte of 63536
    movwf   TMR1L
    movlw   0xF8                ; HIGH byte of 63536
    movwf   TMR1H
    
    ; Clear Timer1 interrupt flag and start timer
    bcf     PIR1, TMR1IF
    bsf     T1CON, TMR1ON
    
wait_pulse:
    ; Wait for fixed 1ms pulse duration
    btfss   PIR1, TMR1IF
    goto    wait_pulse
    
    ; Set RD0 LOW (return to idle state)
    bcf     LATD, 0
    
    ; Stop Timer1 and clear flag
    bcf     T1CON, TMR1ON
    bcf     PIR1, TMR1IF
    
    return

    END
