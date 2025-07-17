/* 
 * File:   newmain.c
 * Author:     // Handle INT2 (RB2) interrupt - Rising edgeui
 *
 * Created on 15 July 2025, 14:05
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

// Configuration bits (example for PIC18F series)
#pragma config OSC = INTIO67    // Internal oscillator
#pragma config WDT = OFF        // Watchdog timer off
#pragma config LVP = OFF        // Low voltage programming off

// Global variables
volatile int toggleR = 0;
volatile int toggleS = 0; 
volatile int toggleT = 0;

// Function declarations
void delay_ms(unsigned int ms);

// Simple delay function in milliseconds (approximate)
void delay_ms(unsigned int ms) {
    unsigned int i, j;
    for(i = 0; i < ms; i++) {
        for(j = 0; j < 250; j++) {
            // Approximate 1ms delay at 8MHz internal oscillator
            // Adjust the inner loop count for your clock
            __asm("NOP");
        }
    }
}

// Interrupt Service Routine
void __interrupt() ISR(void) {
    // Handle INT0 (RB0) interrupt - Rising edge
    if (INTCONbits.INT0IF == 1 && toggleR == 0) {
        // INT0 interrupt handling code
        RD0 = !RD0;  // Example: toggle LED on RD0
        delay_ms(1); // Debounce delay
        RD0 = !RD0;  // Toggle RD0 state
        toggleR = !toggleR; // Toggle R state
        delay_ms(19); // Additional debounce delay
        INTCONbits.INT0IF = 0;  // Clear INT0 interrupt flag
    }

    if (INTCONbits.INT0IF == 1 && toggleR == 1) {
        // INT0 interrupt handling code
        RD0 = !RD1;  // Example: toggle LED on RD0
        delay_ms(1); // Debounce delay
        RD0 = !RD1;  // Toggle RD1 state
        toggleR = !toggleR; // Toggle R state
        delay_ms(19); // Additional debounce delay
        INTCONbits.INT0IF = 0;  // Clear INT0 interrupt flag
    }
    

    // Handle INT1 (RB1) interrupt - Rising edge
    if (INTCON3bits.INT1IF == 1 && toggleS == 0) {
        // INT1 interrupt handling code
        RD2 = !RD2;  // Example: toggle LED on RD2
        delay_ms(1); // Debounce delay
        RD2 = !RD2;  // Toggle RD2 state
        toggleS = !toggleS; // Toggle S state
        delay_ms(19); // Additional debounce delay
        INTCON3bits.INT1IF = 0;  // Clear INT1 interrupt flag
    }
    if (INTCON3bits.INT1IF == 1 && toggleS == 1) {
        // INT1 interrupt handling code
        RD2 = !RD3;  // Example: toggle LED on RD2
        delay_ms(1); // Debounce delay
        RD2 = !RD3;  // Toggle RD3 state
        toggleS = !toggleS; // Toggle S state
        delay_ms(19); // Additional debounce delay
        INTCON3bits.INT1IF = 0;  // Clear INT1 interrupt flag
    }

    
    // Handle INT2 (RB2) interrupt - Falling edge
    if (INTCON3bits.INT2IF == 1 && toggleT == 0) {
        // INT2 interrupt handling code
        RD4 = !RD4;  // Example: toggle LED on RD4
        delay_ms(1); // Debounce delay
        RD4 = !RD4;  // Toggle RD4 state
        toggleT = !toggleT; // Toggle T state
        delay_ms(19); // Additional debounce delay
        INTCON3bits.INT2IF = 0;  // Clear INT2 interrupt flag
    }
    if (INTCON3bits.INT2IF == 1 && toggleT == 1) {
        // INT2 interrupt handling code
        RD4 = !RD5;  // Example: toggle LED on RD4
        delay_ms(1); // Debounce delay
        RD4 = !RD5;  // Toggle RD5 state
        toggleT = !toggleT; // Toggle T state
        delay_ms(19); // Additional debounce delay
        INTCON3bits.INT2IF = 0;  // Clear INT2 interrupt flag
    }
}

int main(int argc, char** argv) {

// setup I/O
    TRISD = 0b11000000; // set RD0-5 as outputs
    /*
     *RD0 = R+
     *RD1 = R-
     *RD2 = S+
     *RD3 = S-
     *RD4 = T+
     *RD5 = T-
     */
    PORTD = 0b00000000; // Initialize PORTD to 0
    
    // Enable global and peripheral interrupts
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;

    // Configure INT0 (RB0) - External interrupt on rising edge
    TRISBbits.TRISB0 = 1;       // RB0 as input
    INTCONbits.INT0IE = 1;      // Enable INT0 interrupt
    INTCON2bits.INTEDG0 = 1;    // Trigger on rising edge
    INTCONbits.INT0IF = 0;      // Clear INT0 interrupt flag
 
    // Configure INT1 (RB1) - External interrupt on rising edge
    TRISBbits.TRISB1 = 1;       // RB1 as input
    INTCON3bits.INT1IE = 1;     // Enable INT1 interrupt
    INTCON2bits.INTEDG1 = 1;    // Trigger on rising edge
    INTCON3bits.INT1IF = 0;     // Clear INT1 interrupt flag
    
    // Configure INT2 (RB2) - External interrupt on rising edge
    TRISBbits.TRISB2 = 1;       // RB2 as input
    INTCON3bits.INT2IE = 1;     // Enable INT2 interrupt
    INTCON2bits.INTEDG2 = 1;    // Trigger on rising edge
    INTCON3bits.INT2IF = 0;     // Clear INT2 interrupt flag
    
    // Disable pull-ups on PORTB
    INTCON2bits.RBPU = 1;       // Disable pull-ups on PORTB

    // Main loop - keep program running
    while(1) {
        // Main program logic goes here
        // Interrupts will be handled by ISR
    }
}



