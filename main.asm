    ; File:   main.asm
    ; Author: sui
    ; Created on 15 July 2025, 14:05
    ; Assembly version of newmain.c for PIC18F4523

    #include <p18f4523.inc>

    ; Configuration bits
        CONFIG OSC = INTIO67        ; Internal oscillator
        CONFIG WDT = OFF            ; Watchdog timer off
        CONFIG LVP = OFF            ; Low voltage programming off

    ; Global variables in RAM
    CBLOCK 0x00
        adc_value:2             ; ADC result (16-bit)
        delay_time:2            ; Calculated delay time
        delay_temp1:1           ; Temporary variable for delay
        delay_temp2:1           ; Temporary variable for delay
        delay_temp3:1           ; Temporary variable for delay
        WREG_TEMP:1             ; Context save for W register
        STATUS_TEMP:1           ; Context save for STATUS register
    ENDC

    ; Reset vector
        ORG 0x0000
        goto main

    ; High priority interrupt vector
        ORG 0x0008
        goto ISR

    ; Low priority interrupt vector  
        ORG 0x0018
        retfie

    ; Main program starts here
        ORG 0x0030

    ;==============================================================================
    ; Initialize ADC for reading analog input on AN0 (RA0)
    ;==============================================================================
    init_adc:
        bsf     TRISA, 0            ; Set RA0/AN0 as input
        movlw   0x00
        movwf   ADCON0              ; Select channel 0 (AN0), ADC off initially
        movlw   0x0E
        movwf   ADCON1              ; AN0 analog, rest digital
        movlw   b'10111110'         ; Right justified, 20 TAD, FOSC/64
        movwf   ADCON2
        bsf     ADCON0, ADON        ; Turn on ADC module
        return

    ;==============================================================================
    ; Read ADC value from AN0 (RA0)
    ; Returns 10-bit result in adc_value (16-bit variable)
    ;==============================================================================
    read_adc:
        bsf     ADCON0, GO_DONE     ; Start conversion
    wait_adc:
        btfsc   ADCON0, GO_DONE     ; Wait for conversion to complete
        bra     wait_adc
        
        ; Combine ADRESH and ADRESL into 16-bit result
        movf    ADRESL, W
        movwf   adc_value           ; Low byte
        movf    ADRESH, W
        movwf   adc_value+1         ; High byte
        return

    ;==============================================================================
    ; Simple delay function in milliseconds (approximate)
    ; Input: delay_time variable contains milliseconds to delay
    ;==============================================================================
    delay_ms:
        movf    delay_time, W
        movwf   delay_temp1         ; Outer loop counter (ms)
        movf    delay_time+1, W
        movwf   delay_temp2
        
    delay_ms_outer:
        movlw   .250                ; Inner loop count for ~1ms at 8MHz
        movwf   delay_temp3
        
    delay_ms_inner:
        nop                         ; 1 cycle delay
        nop
        nop
        nop
        decfsz  delay_temp3, F      ; Decrement inner counter
        bra     delay_ms_inner
        
        ; Decrement 16-bit outer counter
        movf    delay_temp1, W
        iorwf   delay_temp2, W
        btfsc   STATUS, Z           ; Check if both bytes are zero
        return                      ; Return if delay complete
        
        movf    delay_temp1, W
        btfss   STATUS, Z           ; Check if low byte is zero
        bra     dec_low_byte
        decf    delay_temp2, F      ; Decrement high byte
    dec_low_byte:
        decf    delay_temp1, F      ; Decrement low byte
        bra     delay_ms_outer

    ;==============================================================================
    ; 2ms delay subroutine
    ;==============================================================================
    delay_1ms:
        movlw   .2
        movwf   delay_time
        clrf    delay_time+1
        call    delay_ms
        return

    ;==============================================================================
    ; Calculate delay time from ADC value
    ; delay_time = (adc_value * 20) / 1023
    ;==============================================================================
    calc_delay_time:
        ; Multiply adc_value by 20
        ; Simple approach: multiply by 16 + multiply by 4
        
        ; adc_value * 16 (shift left 4 times)
        bcf     STATUS, C
        rlcf    adc_value, F
        rlcf    adc_value+1, F
        rlcf    adc_value, F
        rlcf    adc_value+1, F
        rlcf    adc_value, F
        rlcf    adc_value+1, F
        rlcf    adc_value, F
        rlcf    adc_value+1, F
        
        ; Store result temporarily
        movf    adc_value, W
        movwf   delay_temp1
        movf    adc_value+1, W
        movwf   delay_temp2
        
        ; adc_value * 4 (shift left 2 more times)
        bcf     STATUS, C
        rlcf    adc_value, F
        rlcf    adc_value+1, F
        rlcf    adc_value, F
        rlcf    adc_value+1, F
        
        ; Add results: (adc_value * 16) + (adc_value * 4) = adc_value * 20
        movf    delay_temp1, W
        addwf   adc_value, F
        movf    delay_temp2, W
        addwfc  adc_value+1, F
        
        ; Divide by 1023 (approximately by right shifting 10 times)
        ; This is a rough approximation: divide by 1024 instead of 1023
        bcf     STATUS, C
        rrcf    adc_value+1, F
        rrcf    adc_value, F
        rrcf    adc_value+1, F
        rrcf    adc_value, F
        rrcf    adc_value+1, F
        rrcf    adc_value, F
        rrcf    adc_value+1, F
        rrcf    adc_value, F
        rrcf    adc_value+1, F
        rrcf    adc_value, F
        rrcf    adc_value+1, F
        rrcf    adc_value, F
        
        ; Store result in delay_time
        movf    adc_value, W
        movwf   delay_time
        movf    adc_value+1, W
        movwf   delay_time+1
        
        return

    ;==============================================================================
    ; Interrupt Service Routine
    ;==============================================================================
    ISR:
        ; Save context (simplified)
        movwf   WREG_TEMP
        swapf   STATUS, W
        movwf   STATUS_TEMP
        
        ; Read ADC value
        call    read_adc
        call    calc_delay_time
        
        ; Handle INT0 (RB0) interrupt - Falling edge
        btfss   INTCON, INT0IF      ; Check INT0 interrupt flag
        bra     check_int1
        
        ; INT0 controls RD0 only
        call    delay_ms            ; Variable delay based on analog input
        btg     PORTD, 0            ; Toggle RD0
        call    delay_1ms           ; Debounce delay
        btg     PORTD, 0            ; Toggle RD0 again
        bcf     INTCON, INT0IF      ; Clear INT0 interrupt flag

    check_int1:
        ; Handle INT1 (RB1) interrupt - Falling edge
        btfss   INTCON3, INT1IF     ; Check INT1 interrupt flag
        bra     check_int2
        
        ; INT1 controls RD2 only
        call    delay_ms            ; Variable delay based on analog input
        btg     PORTD, 2            ; Toggle RD2
        call    delay_1ms           ; Debounce delay
        btg     PORTD, 2            ; Toggle RD2 again
        bcf     INTCON3, INT1IF     ; Clear INT1 interrupt flag

    check_int2:
        ; Handle INT2 (RB2) interrupt - Falling edge
        btfss   INTCON3, INT2IF     ; Check INT2 interrupt flag
        bra     isr_exit
        
        ; INT2 controls RD4 only
        call    delay_ms            ; Variable delay based on analog input
        btg     PORTD, 4            ; Toggle RD4
        call    delay_1ms           ; Debounce delay
        btg     PORTD, 4            ; Toggle RD4 again
        bcf     INTCON3, INT2IF     ; Clear INT2 interrupt flag

    isr_exit:
        ; Restore context
        swapf   STATUS_TEMP, W
        movwf   STATUS
        swapf   WREG_TEMP, F
        swapf   WREG_TEMP, W
        retfie

    ;==============================================================================
    ; Main program
    ;==============================================================================
    main:
        ; Initialize variables (no toggle variables needed)
        
        ; Configure internal oscillator to 8MHz
        movlw   b'01110000'         ; 8MHz internal oscillator
        movwf   OSCCON
        
        ; Setup I/O - Only 3 outputs needed now
        movlw   b'11100000'         ; Set RD0, RD2, RD4 as outputs, rest as inputs
        movwf   TRISD
        ; RD0 = R output, RD2 = S output, RD4 = T output
        
        clrf    PORTD               ; Initialize PORTD to 0
        
        ; Initialize ADC
        call    init_adc
        
        ; Configure interrupt pins as inputs with proper TRIS settings
        bsf     TRISB, 0            ; RB0 as input
        bsf     TRISB, 1            ; RB1 as input  
        bsf     TRISB, 2            ; RB2 as input
        
        ; Enable pull-ups on PORTB
        bcf     INTCON2, RBPU       ; Enable pull-ups on PORTB
        
        ; Clear all interrupt flags before enabling
        bcf     INTCON, INT0IF      ; Clear INT0 interrupt flag
        bcf     INTCON3, INT1IF     ; Clear INT1 interrupt flag
        bcf     INTCON3, INT2IF     ; Clear INT2 interrupt flag
        
        ; Configure interrupt edge detection for falling edge
        bcf     INTCON2, INTEDG0    ; INT0 trigger on falling edge
        bcf     INTCON2, INTEDG1    ; INT1 trigger on falling edge
        bcf     INTCON2, INTEDG2    ; INT2 trigger on falling edge
        
        ; Enable individual interrupts
        bsf     INTCON, INT0IE      ; Enable INT0 interrupt
        bsf     INTCON3, INT1IE     ; Enable INT1 interrupt
        bsf     INTCON3, INT2IE     ; Enable INT2 interrupt
        
        ; Enable global and peripheral interrupts (do this last)
        bsf     INTCON, PEIE        ; Enable peripheral interrupts
        bsf     INTCON, GIE         ; Enable global interrupts

    main_loop:
        ; Main program logic goes here
        ; Interrupts will be handled by ISR
        bra     main_loop

        END
