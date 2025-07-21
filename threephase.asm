    ; SCR Controller for PIC18F4523 (3-Phase, 6 SCRs, Main Loop Timing)
    ; Inputs:  RB0 = Zero-cross for sync (INT0)
    ; Outputs: RD0–RD5 = SCR gate outputs
    ; Analog:  RA0/AN0 = Firing delay control

        LIST P=18F4523
        #include <p18f4523.inc>

        CONFIG OSC = INTIO67
        CONFIG WDT = OFF
        CONFIG LVP = OFF
        CONFIG PBADEN = OFF
        CONFIG MCLRE = ON
        CONFIG XINST = OFF
        CONFIG DEBUG = OFF
        CONFIG PWRT = ON
        CONFIG BOREN = OFF

; Constants
FREQ_TICKS      EQU 40000      ; 20 ms cycle at 0.5us = 40000 ticks
PULSE_TICKS     EQU 2000       ; 1 ms pulse width
MAX_DELAY_TICKS EQU 20000      ; Max delay = 10 ms

; Phase Offsets from RB0 sync (0.5us ticks)
OFFSET_S1   EQU d'13333'  ; 6.67 ms
OFFSET_S2   EQU d'33333'  ; 16.67 ms
OFFSET_T1   EQU d'26666'  ; 13.33 ms
OFFSET_T2   EQU d'6666'   ; 3.33 ms
OFFSET_R1   EQU d'0'
OFFSET_R2   EQU d'20000'  ; 10.0 ms

; SCR output bit positions
SCR_R1 EQU 0
SCR_S1 EQU 1
SCR_T1 EQU 2
SCR_T2 EQU 3
SCR_S2 EQU 4
SCR_R2 EQU 5

; Variable memory
    CBLOCK 0x20
        delayL, delayH
        fired_flags       ; 6-bit flags for SCRs fired this cycle
        pulse_timerL, pulse_timerH
        adc_L, adc_H
        temp_w, temp_status
        tmrL, tmrH
        current_phase
    ENDC

; Vectors
    ORG 0x0000
    goto main

    ORG 0x0008
    goto isr_high

    ORG 0x0018
    goto isr_low

; Interrupt - INT0 syncs Timer1 to zero-cross
isr_high:
    movwf temp_w
    movf STATUS, W
    movwf temp_status

    btfss INTCON, INT0IF
    goto isr_exit

    clrf TMR1H
    clrf TMR1L
    bcf PIR1, TMR1IF

    clrf fired_flags
    clrf LATD

    bcf INTCON, INT0IF

isr_exit:
    movf temp_status, W
    movwf STATUS
    movf temp_w, W
    retfie

isr_low:
    retfie

; Main program
main:
    movlw b'01110000'
    movwf OSCCON

    clrf LATD
    clrf TRISD         ; RD0–RD5 outputs
    bsf TRISB, 0       ; RB0 input (INT0)
    bcf INTCON2, RBPU

    call setup_adc
    call setup_timer1

    bcf INTCON2, INTEDG0
    bcf INTCON, INT0IF
    bsf INTCON, INT0IE
    bsf INTCON, GIE

main_loop:
    call read_adc
    call map_adc_to_delay

    call check_fire_R1
    call check_fire_S1
    call check_fire_T1
    call check_fire_T2
    call check_fire_S2
    call check_fire_R2

    ; Small delay to prevent excessive polling
    movlw d'10'
delay_main:
    nop
    nop
    nop
    nop
    decfsz WREG, f
    goto delay_main

    goto main_loop

setup_adc:
    movlw b'00000001'
    movwf ADCON0
    movlw b'00001110'
    movwf ADCON1
    movlw b'10111110'
    movwf ADCON2
    bsf ADCON0, ADON
    return

read_adc:
    bsf ADCON0, GO
adc_wait:
    btfsc ADCON0, GO
    goto adc_wait
    movf ADRESL, W
    movwf adc_L
    movf ADRESH, W
    movwf adc_H
    return

map_adc_to_delay:
    ; Simple mapping: ADC * 10 for reasonable delay values
    movlw 10
    mulwf adc_L
    movf PRODL, W
    movwf delayL
    movf PRODH, W
    movwf delayH
    return

setup_timer1:
    movlw b'00100001'    ; Timer1 on, prescaler 1:4, internal clock
    movwf T1CON
    bsf T1CON, TMR1ON
    return

; SCR Fire Checkers
check_fire_R1:
    movf delayL, W
    movwf pulse_timerL
    movf delayH, W
    movwf pulse_timerH
    movlw SCR_R1
    call check_and_fire
    return

check_fire_S1:
    ; Add delay + OFFSET_S1 with proper 16-bit arithmetic
    movf delayL, W
    addlw LOW(OFFSET_S1)
    movwf pulse_timerL
    clrf temp_w               ; Clear temp flag
    btfsc STATUS, C           ; If carry occurred
    incf temp_w, F            ; Set temp flag
    movf delayH, W
    addlw HIGH(OFFSET_S1)
    movwf pulse_timerH
    movf temp_w, W            ; Check temp flag
    btfss STATUS, Z           ; If temp flag was set
    incf pulse_timerH, F      ; Add carry to HIGH byte
    movlw SCR_S1
    call check_and_fire
    return

check_fire_T1:
    ; Add delay + OFFSET_T1 with proper 16-bit arithmetic
    movf delayL, W
    addlw LOW(OFFSET_T1)
    movwf pulse_timerL
    clrf temp_w               ; Clear temp flag
    btfsc STATUS, C           ; If carry occurred
    incf temp_w, F            ; Set temp flag
    movf delayH, W
    addlw HIGH(OFFSET_T1)
    movwf pulse_timerH
    movf temp_w, W            ; Check temp flag
    btfss STATUS, Z           ; If temp flag was set
    incf pulse_timerH, F      ; Add carry to HIGH byte
    movlw SCR_T1
    call check_and_fire
    return

check_fire_T2:
    ; Add delay + OFFSET_T2 with proper 16-bit arithmetic
    movf delayL, W
    addlw LOW(OFFSET_T2)
    movwf pulse_timerL
    clrf temp_w               ; Clear temp flag
    btfsc STATUS, C           ; If carry occurred
    incf temp_w, F            ; Set temp flag
    movf delayH, W
    addlw HIGH(OFFSET_T2)
    movwf pulse_timerH
    movf temp_w, W            ; Check temp flag
    btfss STATUS, Z           ; If temp flag was set
    incf pulse_timerH, F      ; Add carry to HIGH byte
    movlw SCR_T2
    call check_and_fire
    return

check_fire_S2:
    ; Add delay + OFFSET_S2 with proper 16-bit arithmetic
    movf delayL, W
    addlw LOW(OFFSET_S2)
    movwf pulse_timerL
    clrf temp_w               ; Clear temp flag
    btfsc STATUS, C           ; If carry occurred
    incf temp_w, F            ; Set temp flag
    movf delayH, W
    addlw HIGH(OFFSET_S2)
    movwf pulse_timerH
    movf temp_w, W            ; Check temp flag
    btfss STATUS, Z           ; If temp flag was set
    incf pulse_timerH, F      ; Add carry to HIGH byte
    movlw SCR_S2
    call check_and_fire
    return

check_fire_R2:
    ; Add delay + OFFSET_R2 with proper 16-bit arithmetic
    movf delayL, W
    addlw LOW(OFFSET_R2)
    movwf pulse_timerL
    clrf temp_w               ; Clear temp flag
    btfsc STATUS, C           ; If carry occurred
    incf temp_w, F            ; Set temp flag
    movf delayH, W
    addlw HIGH(OFFSET_R2)
    movwf pulse_timerH
    movf temp_w, W            ; Check temp flag
    btfss STATUS, Z           ; If temp flag was set
    incf pulse_timerH, F      ; Add carry to HIGH byte
    movlw SCR_R2
    call check_and_fire
    return

; Fire SCR if it's time and not already fired
check_and_fire:
    movwf current_phase       ; store which SCR (0–5)
    
    ; Check if this SCR already fired using proper bit test
    movf current_phase, W
    call shift_out_mask       ; Get bit mask for this SCR
    andwf fired_flags, W      ; Test if this bit is set
    btfss STATUS, Z           ; If result is not zero, bit was set
    return                    ; already fired

    ; Load TMR1H:TMR1L into tmrH:tmrL
    movf TMR1L, W
    movwf tmrL
    movf TMR1H, W
    movwf tmrH

    ; Compare Timer1 >= pulse_timer (check if it's time to fire)
    ; First compare high bytes
    movf pulse_timerH, W
    subwf tmrH, W           ; tmrH - pulse_timerH
    btfsc STATUS, C         ; If carry set, tmrH >= pulse_timerH
    goto check_low_or_fire  ; Either equal (check low) or greater (fire)
    return                  ; tmrH < pulse_timerH, not time yet

check_low_or_fire:
    btfss STATUS, Z         ; If high bytes not equal, tmrH > pulse_timerH
    goto do_fire            ; Fire immediately
    ; High bytes equal, check low bytes
    movf pulse_timerL, W
    subwf tmrL, W           ; tmrL - pulse_timerL
    btfss STATUS, C         ; If no carry, tmrL < pulse_timerL
    return                  ; Not time yet

; Fire pulse for 1ms
do_fire:
    ; Set output high - create bit mask for current_phase
    movf current_phase, W
    call shift_out_mask     ; Returns bit mask in W
    iorwf LATD, F          ; Set the bit

    ; Simple delay approach for pulse width (~1ms)
    movlw d'200'           ; Delay counter for ~1ms
pulse_delay:
    nop
    nop
    nop
    nop
    nop
    decfsz WREG, f
    goto pulse_delay

    ; Set output low - invert the bit mask
    movf current_phase, W
    call shift_out_mask     ; Returns bit mask in W
    comf WREG, W           ; Invert all bits
    andwf LATD, F          ; Clear only the target bit

    ; Mark as fired
    movf current_phase, W
    call shift_out_mask     ; Returns bit mask in W
    iorwf fired_flags, F   ; Set the fired flag

    return

; Shift 1 by temp_w positions (0–5) into W
shift_out_mask:
    movwf temp_w
    movlw 1
    bcf STATUS, C           ; Clear carry before shifting
    movf temp_w, W
    btfsc STATUS, Z
    goto shift_done         ; If shift by 0, return 1
shift_loop:
    bcf STATUS, C           ; Clear carry for each shift
    rlcf WREG, f           ; Shift left
    decfsz temp_w, f
    goto shift_loop
shift_done:
    return

    END
