/*
 * File:   newmain.c
 * Author: sui
 * Created on 15 July 2025, 14:05
 * C version converted from main.asm for PIC18F4523
 */

#include <xc.h>
#include <stdint.h>

// Configuration bits
#pragma config OSC = INTIO67        // Internal oscillator
#pragma config FCMEN = OFF          // Fail-Safe Clock Monitor disabled
#pragma config IESO = OFF           // Oscillator Switchover mode disabled
#pragma config PWRT = OFF           // Power-up Timer disabled
#pragma config BOREN = SBORDIS      // Brown-out Reset disabled in hardware and software
#pragma config BORV = 3             // Brown-out Reset Voltage bits
#pragma config WDT = OFF            // Watchdog timer off
#pragma config WDTPS = 32768        // Watchdog Timer Postscale Select bits
#pragma config MCLRE = ON           // MCLR pin enabled
#pragma config LPT1OSC = OFF        // Timer1 configured for higher power operation
#pragma config PBADEN = OFF         // PORTB<4:0> pins are configured as digital I/O on Reset
#pragma config CCP2MX = PORTC       // CCP2 input/output is multiplexed with RC1
#pragma config STVREN = ON          // Stack full/underflow will cause Reset
#pragma config LVP = OFF            // Low voltage programming off
#pragma config XINST = OFF          // Instruction set extension and Indexed Addressing mode disabled
#pragma config DEBUG = OFF          // Background debugger disabled
#pragma config CP0 = OFF            // Code Protection bit
#pragma config CP1 = OFF            // Code Protection bit
#pragma config CP2 = OFF            // Code Protection bit
#pragma config CP3 = OFF            // Code Protection bit
#pragma config CPB = OFF            // Boot Block Code Protection bit
#pragma config CPD = OFF            // Data EEPROM Code Protection bit
#pragma config WRT0 = OFF           // Write Protection bit
#pragma config WRT1 = OFF           // Write Protection bit
#pragma config WRT2 = OFF           // Write Protection bit
#pragma config WRT3 = OFF           // Write Protection bit
#pragma config WRTB = OFF           // Boot Block Write Protection bit
#pragma config WRTC = OFF           // Configuration Register Write Protection bit
#pragma config WRTD = OFF           // Data EEPROM Write Protection bit
#pragma config EBTR0 = OFF          // Table Read Protection bit
#pragma config EBTR1 = OFF          // Table Read Protection bit
#pragma config EBTR2 = OFF          // Table Read Protection bit
#pragma config EBTR3 = OFF          // Table Read Protection bit
#pragma config EBTRB = OFF          // Boot Block Table Read Protection bit

// Global variables
volatile uint16_t adc_value = 0;    // ADC result (16-bit)
volatile uint16_t delay_time = 0;   // Calculated delay time

// Function prototypes
void init_system(void);
void init_adc(void);
uint16_t read_adc(void);
void calc_delay_time(void);
void delay_ms(uint16_t ms);
void delay_1ms(void);
void interrupt ISR(void);

//==============================================================================
// Initialize ADC for reading analog input on AN0 (RA0)
//==============================================================================
void init_adc(void) {
    TRISAbits.TRISA0 = 1;           // Set RA0/AN0 as input
    ADCON1 = 0x0E;                  // AN0 analog, rest digital, VDD/VSS references
    ADCON2 = 0xBF;                  // Right justified, 20 TAD, FOSC/128
    ADCON0 = 0x01;                  // Channel 0, ADC enabled
}

//==============================================================================
// Read ADC value from AN0 (RA0)
// Returns 10-bit result
//==============================================================================
uint16_t read_adc(void) {
    // Ensure ADC is on and channel is selected
    ADCON0 = 0x01;                  // Select channel 0 (AN0) and enable ADC
    
    // Wait for acquisition time (minimum 20μs for high impedance source)
    __delay_us(20);
    
    ADCON0bits.GO_DONE = 1;         // Start conversion
    while(ADCON0bits.GO_DONE);      // Wait for conversion to complete
    
    // Combine ADRESH and ADRESL into 16-bit result
    return ((uint16_t)ADRESH << 8) | ADRESL;
}

//==============================================================================
// Simple delay function in milliseconds (approximate)
//==============================================================================
void delay_ms(uint16_t ms) {
    uint16_t i;
    uint16_t j;
    
    for(i = 0; i < ms; i++) {
        // For 1ms at 8MHz: need 2000 instruction cycles
        // Each NOP = 1 cycle, loop overhead ≈ 3 cycles per iteration
        // So 500 iterations × 4 cycles = 2000 cycles = 1ms
        for(j = 0; j < 500; j++) {
            __asm("nop");
            // Loop overhead: increment j, compare, branch ≈ 3 cycles
        }
    }
}

//==============================================================================
// 2ms delay subroutine
//==============================================================================
void delay_1ms(void) {
    delay_ms(2);
}

//==============================================================================
// Calculate delay time from ADC value
// Formula: delay_time = (adc_value * 15) / 1024
// This gives: 0V (ADC=0) = 0ms, 5V (ADC=1023) = 15ms
// Simplified approach: divide by 256 for 0-4ms range
//==============================================================================
void calc_delay_time(void) {
    // Check if ADC is 0 - if so, set delay to 0
    if(adc_value == 0) {
        delay_time = 0;
        return;
    }
    
    // For non-zero ADC values, use division by 256 for 0-4ms range
    // This gives us: 0V=0ms, 1V=0.8ms, 2V=1.6ms, 3V=2.4ms, 4V=3.2ms, 5V=4ms
    delay_time = adc_value >> 8;    // Divide by 256 (get high byte)
    
    // If calculated delay is 0 but ADC was non-zero, set minimum 1ms
    if(delay_time == 0) {
        delay_time = 1;
    }
}

//==============================================================================
// Interrupt Service Routine
//==============================================================================
void __interrupt() ISR(void) {
    // Handle INT0 (RB0) interrupt - Falling edge
    if(INTCONbits.INT0IF) {
        // INT0 controls RD0 only
        delay_ms(delay_time);       // Variable delay based on analog input
        PORTDbits.RD0 = !PORTDbits.RD0;  // Toggle RD0
        delay_1ms();                // Debounce delay
        PORTDbits.RD0 = !PORTDbits.RD0;  // Toggle RD0 again
        INTCONbits.INT0IF = 0;      // Clear INT0 interrupt flag
    }
    
    // Handle INT1 (RB1) interrupt - Falling edge
    if(INTCON3bits.INT1IF) {
        // INT1 controls RD2 only
        delay_ms(delay_time);       // Variable delay based on analog input
        PORTDbits.RD2 = !PORTDbits.RD2;  // Toggle RD2
        delay_1ms();                // Debounce delay
        PORTDbits.RD2 = !PORTDbits.RD2;  // Toggle RD2 again
        INTCON3bits.INT1IF = 0;     // Clear INT1 interrupt flag
    }
    
    // Handle INT2 (RB2) interrupt - Falling edge
    if(INTCON3bits.INT2IF) {
        // INT2 controls RD4 only
        delay_ms(delay_time);       // Variable delay based on analog input
        PORTDbits.RD4 = !PORTDbits.RD4;  // Toggle RD4
        delay_1ms();                // Debounce delay
        PORTDbits.RD4 = !PORTDbits.RD4;  // Toggle RD4 again
        INTCON3bits.INT2IF = 0;     // Clear INT2 interrupt flag
    }
}


//==============================================================================
// Initialize system
//==============================================================================
void init_system(void) {
    // Configure internal oscillator to 8MHz
    OSCCON = 0x70;                  // 8MHz internal oscillator
    
    // Setup I/O - Only 3 outputs needed now
    TRISD = 0xEA;                   // Set RD0, RD2, RD4 as outputs (0), rest as inputs (1)
    // RD0 = R output, RD2 = S output, RD4 = T output
    
    TRISC = 0x00;                   // Set all PORTC pins as outputs for ADC display
    TRISA = 0x00;                   // Set PORTA as outputs for delay_time display (except RA0)
    TRISAbits.TRISA0 = 1;           // Keep RA0 as input for ADC
    
    PORTC = 0x00;                   // Initialize PORTC to 0
    PORTD = 0x00;                   // Initialize PORTD to 0
    PORTA = 0x00;                   // Initialize PORTA to 0
    
    // Initialize ADC
    init_adc();
    
    // Configure interrupt pins as inputs with proper TRIS settings
    TRISBbits.TRISB0 = 1;           // RB0 as input
    TRISBbits.TRISB1 = 1;           // RB1 as input
    TRISBbits.TRISB2 = 1;           // RB2 as input
    
    // Enable pull-ups on PORTB
    INTCON2bits.RBPU = 0;           // Enable pull-ups on PORTB
    
    // Clear all interrupt flags before enabling
    INTCONbits.INT0IF = 0;          // Clear INT0 interrupt flag
    INTCON3bits.INT1IF = 0;         // Clear INT1 interrupt flag
    INTCON3bits.INT2IF = 0;         // Clear INT2 interrupt flag
    
    // Configure interrupt edge detection for falling edge
    INTCON2bits.INTEDG0 = 0;        // INT0 trigger on falling edge
    INTCON2bits.INTEDG1 = 0;        // INT1 trigger on falling edge
    INTCON2bits.INTEDG2 = 0;        // INT2 trigger on falling edge
    
    // Enable individual interrupts
    INTCONbits.INT0IE = 1;          // Enable INT0 interrupt
    INTCON3bits.INT1IE = 1;         // Enable INT1 interrupt
    INTCON3bits.INT2IE = 1;         // Enable INT2 interrupt
    
    // Enable global and peripheral interrupts (do this last)
    INTCONbits.PEIE = 1;            // Enable peripheral interrupts
    INTCONbits.GIE = 1;             // Enable global interrupts
}

//==============================================================================
// Main program
//==============================================================================
void main(void) {
    // Initialize system
    init_system();
    
    // Main loop
    while(1) {
        // Continuously update ADC reading and delay calculation in background
        adc_value = read_adc();     // Read current ADC value
        calc_delay_time();          // Calculate delay from ADC value
        
        // Display ADC value on PORTC (8-bit binary display)
        // Show lower 8 bits of 10-bit ADC value
        PORTC = (uint8_t)(adc_value & 0xFF);
        
        // Display delay_time on PORTA (bits 1-7) for debugging
        // Shift left to skip RA0 and mask to fit in 7 bits
        PORTA = (uint8_t)((delay_time & 0x7F) << 1);
        
        // Small delay to prevent excessive ADC reading
        delay_1ms();                // 2ms delay between ADC readings
        
        // Interrupts will be handled by ISR using current delay_time
    }
}



