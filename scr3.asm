; 3-Phase SCR Controller for PIC18F4523 (Converted from C)
; Inputs:  RB0 = Phase A Zero-cross (INT0)
;          RB1 = Phase B Zero-cross (INT1) 
;          RB2 = Phase C Zero-cross (INT2)
;          RA0 = Angle control potentiometer (ADC)
; Outputs: RC0–RC5 = SCR gate outputs (SCR1-SCR6)

        LIST P=18F4523
        #include <p18f4523.inc>

        CONFIG OSC = HS
        CONFIG WDT = OFF
        CONFIG LVP = OFF
        CONFIG PBADEN = OFF
        CONFIG MCLRE = ON
        CONFIG XINST = OFF
        CONFIG DEBUG = OFF
        CONFIG PWRT = ON
        CONFIG BOREN = OFF

; Variables
    CBLOCK 0x20
        current_phase        ; 0–5: SCR1 to SCR6
        half_cycle          ; 0 = positive, 1 = negative
        delay_ticksL, delay_ticksH  ; firing delay in ticks (set by ADC)
        next_output_pin
        adc_resultL, adc_resultH
        temp_w, temp_status
        temp_fsr
    ENDC

; Vectors
    ORG 0x0000
    goto main

    ORG 0x0008
    goto isr_high

    ORG 0x0018
    goto isr_low

; High Priority Interrupt Service Routine
isr_high:
    ; Save context
    movwf temp_w
    movf STATUS, W
    movwf temp_status

    ; Check INT0 - Phase A
    btfsc INTCON, INT0IF
    call handle_int0
    
    ; Check INT1 - Phase B  
    btfsc INTCON3, INT1IF
    call handle_int1
    
    ; Check INT2 - Phase C
    btfsc INTCON3, INT2IF
    call handle_int2
    
    ; Check Timer1 - Fire SCR
    btfsc PIR1, TMR1IF
    call handle_timer1

    ; Restore context
    movf temp_status, W
    movwf STATUS
    movf temp_w, W
    retfie

; Handle INT0 - Phase A zero-cross
handle_int0:
    ; current_phase = half_cycle ? 3 : 0
    btfsc half_cycle, 0
    goto set_phase3
    clrf current_phase      ; Phase 0 (SCR1)
    goto int0_continue
set_phase3:
    movlw 3
    movwf current_phase     ; Phase 3 (SCR4)
int0_continue:
    bcf INTCON, INT0IF
    call reset_timer1
    return

; Handle INT1 - Phase B zero-cross  
handle_int1:
    ; current_phase = half_cycle ? 4 : 1
    btfsc half_cycle, 0
    goto set_phase4
    movlw 1
    movwf current_phase     ; Phase 1 (SCR2)
    goto int1_continue
set_phase4:
    movlw 4
    movwf current_phase     ; Phase 5 (SCR5)
int1_continue:
    bcf INTCON3, INT1IF
    call reset_timer1
    return

; Handle INT2 - Phase C zero-cross
handle_int2:
    ; current_phase = half_cycle ? 5 : 2
    btfsc half_cycle, 0
    goto set_phase5
    movlw 2
    movwf current_phase     ; Phase 2 (SCR3)
    goto int2_continue
set_phase5:
    movlw 5
    movwf current_phase     ; Phase 5 (SCR6)
int2_continue:
    bcf INTCON3, INT2IF
    call reset_timer1
    return

; Reset Timer1 and start counting
reset_timer1:
    clrf TMR1H
    clrf TMR1L
    bsf T1CON, TMR1ON
    return

; Handle Timer1 interrupt - Fire SCR
handle_timer1:
    bcf T1CON, TMR1ON       ; Stop timer
    bcf PIR1, TMR1IF        ; Clear flag
    
    ; Fire the appropriate SCR
    call fire_scr
    
    ; Flip half-cycle (positive ↔ negative)
    btg half_cycle, 0
    return

; Fire SCR pulse
fire_scr:
    ; Set the output bit: LATC |= (1 << current_phase)
    movf current_phase, W
    call create_bit_mask
    iorwf LATC, F
    
    ; Pulse width delay (~300µs)
    call delay_300us
    
    ; Clear the output bit: LATC &= ~(1 << current_phase)  
    movf current_phase, W
    call create_bit_mask
    comf WREG, W
    andwf LATC, F
    return

; Create bit mask for given bit position (0-5)
create_bit_mask:
    addwf PCL, F        ; Jump table
    retlw b'00000001'   ; Bit 0
    retlw b'00000010'   ; Bit 1  
    retlw b'00000100'   ; Bit 2
    retlw b'00001000'   ; Bit 3
    retlw b'00010000'   ; Bit 4
    retlw b'00100000'   ; Bit 5

; 300µs delay for pulse width
delay_300us:
    movlw d'75'         ; ~300µs at 20MHz
delay_loop:
    nop
    nop
    nop  
    nop
    decfsz WREG, f
    goto delay_loop
    return

; Low Priority ISR (unused)
isr_low:
    retfie

; Main program
main:
    call init_io
    call init_extints
    call init_timer1
    call init_adc
    bsf INTCON, GIE     ; Enable global interrupts
    bsf INTCON, PEIE    ; Enable peripheral interrupts

main_loop:
    call read_angle_adc
    call set_timer_delay
    call delay_5ms
    goto main_loop

; Initialize I/O ports
init_io:
    ; TRISB |= 0x07 - RB0–RB2 inputs (ZCD)
    movlw b'00000111'
    iorwf TRISB, F
    
    ; TRISC &= ~0x3F - RC0–RC5 outputs (SCR gates)
    movlw b'11000000'
    andwf TRISC, F
    
    ; LATC &= ~0x3F - Clear outputs
    movlw b'11000000'
    andwf LATC, F
    
    ; TRISA |= 0x01 - RA0 input (potentiometer)
    bsf TRISA, 0
    return

; Initialize external interrupts
init_extints:
    ; Falling edge for all interrupts (typical for zero-cross detectors)
    bcf INTCON2, INTEDG0    ; INT0 falling edge
    bcf INTCON2, INTEDG1    ; INT1 falling edge  
    bcf INTCON2, INTEDG2    ; INT2 falling edge
    
    ; Clear interrupt flags
    bcf INTCON, INT0IF
    bcf INTCON3, INT1IF
    bcf INTCON3, INT2IF
    
    ; Enable interrupts
    bsf INTCON, INT0IE      ; Enable INT0
    bsf INTCON3, INT1IE     ; Enable INT1
    bsf INTCON3, INT2IE     ; Enable INT2
    return

; Initialize Timer1
init_timer1:
    ; T1CON = 0x31 - Prescaler 1:8, Internal clock
    movlw b'00110001'
    movwf T1CON
    clrf TMR1H
    clrf TMR1L
    bsf PIE1, TMR1IE        ; Enable Timer1 interrupt
    bcf PIR1, TMR1IF        ; Clear Timer1 flag
    return

; Initialize ADC
init_adc:
    ; ADCON1 = 0x0E - AN0 analog only
    movlw b'00001110'
    movwf ADCON1
    
    ; ADCON2 = 0xA9 - Right justified, Fosc/8
    movlw b'10101001'
    movwf ADCON2
    
    ; ADCON0 = 0x01 - Enable ADC, select AN0
    movlw b'00000001'
    movwf ADCON0
    return

; Read ADC and convert to delay (0–6250 ticks for 0-10ms)
read_angle_adc:
    bsf ADCON0, GO          ; Start conversion
adc_wait:
    btfsc ADCON0, GO        ; Wait for completion
    goto adc_wait
    
    ; Read result
    movf ADRESL, W
    movwf adc_resultL
    movf ADRESH, W  
    movwf adc_resultH
    
    ; Convert to timer ticks: value * 6250 / 1023
    ; Simplified: value * 6 (approximately)
    movf adc_resultL, W
    mullw 6
    movf PRODL, W
    movwf delay_ticksL
    movf PRODH, W
    movwf delay_ticksH
    return

; Set Timer1 delay value
set_timer_delay:
    ; Load delay for next firing: TMR1H:TMR1L = delay_ticks
    movf delay_ticksH, W
    movwf TMR1H
    movf delay_ticksL, W
    movwf TMR1L
    return

; 5ms delay
delay_5ms:
    movlw d'25'             ; Outer loop counter
delay_5ms_outer:
    movlw d'200'            ; Inner loop counter  
delay_5ms_inner:
    nop
    nop
    nop
    nop
    decfsz WREG, f
    goto delay_5ms_inner
    decfsz delay_ticksL, f  ; Reuse variable as temp counter
    goto delay_5ms_outer
    return

    END
