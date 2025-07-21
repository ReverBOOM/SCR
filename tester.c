#include <xc.h>
#define _XTAL_FREQ 20000000UL

#pragma config WDT = OFF, LVP = OFF, FOSC = HS

// Variables
volatile uint8_t current_phase = 0;     // 0–5: SCR1 to SCR6
volatile uint8_t half_cycle = 0;        // 0 = positive, 1 = negative
volatile uint16_t delay_ticks = 5000;   // firing delay in ticks (set by ADC)
volatile uint8_t next_output_pin = 0;

// Initialize I/O
void init_io() {
    TRISB |= 0x07;       // RB0–RB2 inputs (ZCD)
    TRISC &= ~0x3F;      // RC0–RC5 outputs (SCR gates)
    LATC &= ~0x3F;
    TRISA |= 0x01;       // RA0 input (potentiometer)
}

// External interrupt setup
void init_extints() {
    INTCON2bits.INTEDG0 = 1;    // Rising edge
    INTCON2bits.INTEDG1 = 1;
    INTCON2bits.INTEDG2 = 1;

    INTCONbits.INT0IF = 0;
    INTCON3bits.INT1IF = 0;
    INTCON3bits.INT2IF = 0;

    INTCONbits.INT0IE = 1;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT2IE = 1;
}

// Timer1 setup
void init_timer1() {
    T1CON = 0x31;  // Prescaler 1:8, Internal clock
    TMR1 = 0;
    PIE1bits.TMR1IE = 1;
    PIR1bits.TMR1IF = 0;
    INTCONbits.PEIE = 1;
}

// ADC setup (RA0)
void init_adc() {
    ADCON1 = 0x0E;  // AN0 analog only
    ADCON2 = 0xA9;  // Right justified, Fosc/8
    ADCON0 = 0x01;  // Enable ADC
}

// Read ADC and convert to delay (0–180°)
uint16_t read_angle_adc() {
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO);
    uint16_t value = ((uint16_t)ADRESH << 8) | ADRESL;

    // Convert to timer ticks for 0–10ms (half cycle @ 50Hz)
    // Fosc = 20MHz → 5MHz instruction cycle → 0.2µs per tick
    // Prescaler 1:8 → 1.6µs per Timer1 tick → ~6250 ticks = 10ms
    return (uint32_t)value * 6250UL / 1023;
}

// Interrupt Service Routine
void __interrupt() isr() {
    // INT0 – Phase A
    if (INTCONbits.INT0IF) {
        current_phase = half_cycle ? 3 : 0;  // SCR4 or SCR1
        INTCONbits.INT0IF = 0;
        TMR1 = 0;
        T1CONbits.TMR1ON = 1;
    }

    // INT1 – Phase B
    if (INTCON3bits.INT1IF) {
        current_phase = half_cycle ? 4 : 1;  // SCR5 or SCR2
        INTCON3bits.INT1IF = 0;
        TMR1 = 0;
        T1CONbits.TMR1ON = 1;
    }

    // INT2 – Phase C
    if (INTCON3bits.INT2IF) {
        current_phase = half_cycle ? 5 : 2;  // SCR6 or SCR3
        INTCON3bits.INT2IF = 0;
        TMR1 = 0;
        T1CONbits.TMR1ON = 1;
    }

    // Timer1 – Fire SCR gate after delay
    if (PIR1bits.TMR1IF) {
        T1CONbits.TMR1ON = 0;
        PIR1bits.TMR1IF = 0;

        // Fire the appropriate SCR
        LATC |= (1 << current_phase);
        __delay_us(300); // Pulse width (~300–500µs recommended)
        LATC &= ~(1 << current_phase);

        // Flip half-cycle (positive ↔ negative)
        half_cycle ^= 1;
    }
}

// Main Program
void main(void) {
    init_io();
    init_extints();
    init_timer1();
    init_adc();
    ei();  // Enable global interrupts

    while (1) {
        delay_ticks = read_angle_adc();
        // Load delay for next firing
        TMR1H = (delay_ticks >> 8) & 0xFF;
        TMR1L = delay_ticks & 0xFF;
        __delay_ms(5);  // Optional debounce
    }
}
