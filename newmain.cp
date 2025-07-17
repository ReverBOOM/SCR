#line 1 "C:/Users/sui/MPLABXProjects/test.X/newmain.c"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for pic/include/stdio.h"
#line 1 "c:/users/public/documents/mikroelektronika/mikroc pro for pic/include/stdlib.h"







 typedef struct divstruct {
 int quot;
 int rem;
 } div_t;

 typedef struct ldivstruct {
 long quot;
 long rem;
 } ldiv_t;

 typedef struct uldivstruct {
 unsigned long quot;
 unsigned long rem;
 } uldiv_t;

int abs(int a);
float atof(char * s);
int atoi(char * s);
long atol(char * s);
div_t div(int number, int denom);
ldiv_t ldiv(long number, long denom);
uldiv_t uldiv(unsigned long number, unsigned long denom);
long labs(long x);
int max(int a, int b);
int min(int a, int b);
void srand(unsigned x);
int rand();
int xtoi(char * s);
#pragma config OSC = INTIO67
#pragma config WDT = OFF
#pragma config LVP = OFF
#line 18 "C:/Users/sui/MPLABXProjects/test.X/newmain.c"
volatile int toggleR = 0;
volatile int toggleS = 0;
volatile int toggleT = 0;


void delay_ms(unsigned int ms);
unsigned int read_adc(void);
void init_adc(void);


void init_adc(void) {
 TRISAbits.TRISA0 = 1;
 ADCON0bits.CHS = 0;
 ADCON1 = 0x0E;
 ADCON2bits.ADFM = 1;
 ADCON2bits.ACQT = 0b010;
 ADCON2bits.ADCS = 0b010;
 ADCON0bits.ADON = 1;
}


unsigned int read_adc(void) {
 ADCON0bits.GO_DONE = 1;
 while(ADCON0bits.GO_DONE);
 return ((ADRESH << 8) | ADRESL);
}


void delay_ms(unsigned int ms) {
 unsigned int i, j;
 for(i = 0; i < ms; i++) {
 for(j = 0; j < 250; j++) {


 __asm("NOP");
 }
 }
}


void __interrupt() ISR(void) {
 unsigned int adc_value = read_adc();
 unsigned int delay_time = (adc_value * 20) / 1023;


 if (INTCONbits.INT0IF == 1 && toggleR == 0) {

 delay_ms(delay_time);
 RD0 = !RD0;
 delay_ms(1);
 RD0 = !RD0;
 toggleR = !toggleR;
 INTCONbits.INT0IF = 0;
 }

 if (INTCONbits.INT0IF == 1 && toggleR == 1) {

 delay_ms(delay_time);
 RD0 = !RD1;
 delay_ms(1);
 RD0 = !RD1;
 toggleR = !toggleR;
 INTCONbits.INT0IF = 0;
 }



 if (INTCON3bits.INT1IF == 1 && toggleS == 0) {

 delay_ms(delay_time);
 RD2 = !RD2;
 delay_ms(1);
 RD2 = !RD2;
 toggleS = !toggleS;
 INTCON3bits.INT1IF = 0;
 }
 if (INTCON3bits.INT1IF == 1 && toggleS == 1) {

 delay_ms(delay_time);
 RD2 = !RD3;
 delay_ms(1);
 RD2 = !RD3;
 toggleS = !toggleS;
 INTCON3bits.INT1IF = 0;
 }



 if (INTCON3bits.INT2IF == 1 && toggleT == 0) {

 delay_ms(delay_time);
 RD4 = !RD4;
 delay_ms(1);
 RD4 = !RD4;
 toggleT = !toggleT;
 INTCON3bits.INT2IF = 0;
 }
 if (INTCON3bits.INT2IF == 1 && toggleT == 1) {

 delay_ms(delay_time);
 RD4 = !RD5;
 delay_ms(1);
 RD4 = !RD5;
 toggleT = !toggleT;
 INTCON3bits.INT2IF = 0;
 }
}

int main(int argc, char** argv) {


 TRISD = 0b11000000;
#line 138 "C:/Users/sui/MPLABXProjects/test.X/newmain.c"
 PORTD = 0b00000000;


 init_adc();


 INTCONbits.GIE = 1;
 INTCONbits.PEIE = 1;


 TRISBbits.TRISB0 = 1;
 INTCONbits.INT0IE = 1;
 INTCON2bits.INTEDG0 = 0;
 INTCONbits.INT0IF = 0;


 TRISBbits.TRISB1 = 1;
 INTCON3bits.INT1IE = 1;
 INTCON2bits.INTEDG1 = 0;
 INTCON3bits.INT1IF = 0;


 TRISBbits.TRISB2 = 1;
 INTCON3bits.INT2IE = 1;
 INTCON2bits.INTEDG2 = 0;
 INTCON3bits.INT2IF = 0;


 INTCON2bits.RBPU = 0;


 while(1) {


 }
}
