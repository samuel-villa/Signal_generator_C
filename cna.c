/*
 * SIMPLIFIED Conversion in C of the generator project made in assembly language
 * 
 * Microcontroller: PIC18F8722
 * File:            cna.c
 * Author:          Samuel CIULLA
 *
 * Created on January 2021
 */

// PIC18F8722 Configuration Bit Settings
// 'C' source line config statements

// CONFIG1H
#pragma config OSC = RCIO6      // Oscillator Selection bits (External RC oscillator, port function on RA6)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3L
#pragma config MODE = MC        // Processor Data Memory Mode Select bits (Microcontroller mode)
#pragma config ADDRBW = ADDR20BIT// Address Bus Width Select bits (20-bit Address Bus)
#pragma config DATABW = DATA16BIT// Data Bus Width Select bit (16-bit External Bus mode)
#pragma config WAIT = OFF       // External Bus Data Wait Enable bit (Wait selections are unavailable for table reads and table writes)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (ECCP2 input/output is multiplexed with RC1)
#pragma config ECCPMX = PORTE   // ECCP MUX bit (ECCP1/3 (P1B/P1C/P3B/P3C) are multiplexed onto RE6, RE5, RE4 and RE3 respectively)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RG5 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size Select bits (1K word (2 Kbytes) Boot Block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit Block 3 (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF        // Code Protection bit Block 4 (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF        // Code Protection bit Block 5 (Block 5 (014000-017FFFh) not code-protected)
#pragma config CP6 = OFF        // Code Protection bit Block 6 (Block 6 (01BFFF-018000h) not code-protected)
#pragma config CP7 = OFF        // Code Protection bit Block 7 (Block 7 (01C000-01FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection bit Block 4 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection bit Block 5 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection bit Block 6 (Block 6 (01BFFF-018000h) not write-protected)
#pragma config WRT7 = OFF       // Write Protection bit Block 7 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection bit Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection bit Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection bit Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection bit Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>
#include "SPI_Alpha_LCD.h"


/*------------------------------------------------------------------------------
 * Definitions
 *----------------------------------------------------------------------------*/
#define INSTR_CY 1000000            // 4MHz/4 = (1MHz)
#define VAL_MAX    20400            // value sent to MCP4922 in order to get 5V


/*------------------------------------------------------------------------------
 * Global variables
 *----------------------------------------------------------------------------*/
int f = 2, s = 0;
unsigned int dc = 50;


/*------------------------------------------------------------------------------
 * Prototypes
 *----------------------------------------------------------------------------*/
void PIC18_init(unsigned int *frq);
char scan_keypad(void);
void LCD_display(unsigned int frq, char * type, unsigned int duty_cycle);
int config_mode(void);
void signal(unsigned int *frq);
void ramp_saw(void);
void triang(void);

/// MCP4922
void set_reg_bit(unsigned int  *port, char bit1);
unsigned char spi_write(unsigned char data);
void dac_write(unsigned int data, char channel);


/*------------------------------------------------------------------------------
 * Interrupt CCP1 in compare special trigger event mode based on Timer1
 *----------------------------------------------------------------------------*/
void __interrupt() ccp1_interrupt() {
    
    INT_TICK = 1;
    
    PIR2bits.CCP2IF = 0;                // polling on CCP2
    while (!PIR2bits.CCP2IF);
    
    INT_TICK = 0;
    
    PIR1bits.CCP1IF = 0;
}


/*------------------------------------------------------------------------------
 * Main program
 *----------------------------------------------------------------------------*/
int main(void) {
    
    unsigned int freqs[5] = {100, 500, 1000, 10000, 50000};
    char sigs[3][9] = {"SQUARE", "RAMP", "TRIANGLE"};
    
    Init_Alpha_LCD();
    PIC18_init(freqs);
    LCD_display(freqs[f], sigs[s], dc);
    
    while (1) {
        
        if (PORTAbits.RA1){
            if (!config_mode())  LCD_display(freqs[f], sigs[s], dc);
            __delay_ms(200);
        }
        else signal(freqs);
    }
}



/*------------------------------------------------------------------------------
 * Initialise µC PIC18F8722 - this may be changed depending on application needs
 *----------------------------------------------------------------------------*/
void PIC18_init(unsigned int *frq) {
    
    unsigned int delay = 0, dc_pc = 0;
    float pc_mul = 0.0;
    
    delay = INSTR_CY/frq[f];
    pc_mul = dc*0.01;
    dc_pc = (delay*pc_mul);
    
    PORTA = 0x00;
    TRISA = 0x02;                   // only 1 INPUT needed
    ADCON0 = 0x00;                  // internal AD converters not needed
    ADCON1 = 0x0F;                  // all pins digital
    ADCON2 = 0x00;
    
    TRISB = 0xF0;
    TRISC = 0x00;
    TRISG = 0x00;
    
    T3CON = 0x00;                   // Timer1 and Timer2 are the clock sources 
                                    // for ECCP1, ECCP2, ECCP3, CCP4 and CCP5
    
    T1CON = 0x01;                   // enable Timer1
    TMR1H = 0x00;                   // reset Timer1
    TMR1L = 0x00;
    
    CCP1CON = 0x0B;                 // CCP1: Special Trigger event
    CCPR1 = delay;                  // set period
    
    CCP2CON = 0x08;                 // CCP2: Compare mode, CCP2IF bit is set
    CCPR2 = dc_pc;                  // set duty cycle
    
    PIR1bits.CCP1IF = 0;            // clear CCP1 interrupt flag
    PIE1bits.CCP1IE = 1;            // enable CCP1 interrupt
    PIR2bits.CCP2IF = 0;            // clear CCP2 interrupt flag
    
    INTCONbits.PEIE = 1;            // enable peripheral interrupts
    INTCONbits.GIE = 1;             // enable global interrupts
}



/*------------------------------------------------------------------------------
 * polling on keypad - connected to PORTB
 *      return: button pushed, if nothing pushed return '?'
 *----------------------------------------------------------------------------*/
char scan_keypad(void) {
    
    char c = '?';
    
    PORTB = 0x01;                               // scan first column
    if      (PORTBbits.RB4 == 1) c = '7';
    else if (PORTBbits.RB5 == 1) c = '4';
    else if (PORTBbits.RB6 == 1) c = '1';
    else if (PORTBbits.RB7 == 1) c = 'C';       // pushed ON/C
    
    PORTB = 0x02;                               // scan second column
    if      (PORTBbits.RB4 == 1) c = '8';
    else if (PORTBbits.RB5 == 1) c = '5';
    else if (PORTBbits.RB6 == 1) c = '2';
    else if (PORTBbits.RB7 == 1) c = '0';       // pushed 0
    
    PORTB = 0x04;                               // scan third column
    if      (PORTBbits.RB4 == 1) c = '9';
    else if (PORTBbits.RB5 == 1) c = '6';
    else if (PORTBbits.RB6 == 1) c = '3';
    else if (PORTBbits.RB7 == 1) c = '=';       // pushed '='
    
    PORTB = 0x08;                               // scan fourth column
    if      (PORTBbits.RB4 == 1) c = '/';       // pushed '/'
    else if (PORTBbits.RB5 == 1) c = '*';       // pushed '*'
    else if (PORTBbits.RB6 == 1) c = '-';       // pushed '-'
    else if (PORTBbits.RB7 == 1) c = '+';       // pushed '+'
    
    PORTB = 0x00;
    
    return c;
}


/*------------------------------------------------------------------------------
 * Set display data
 *      frq       : frequence
 *      type      : signal type
 *      duty_cycle: duty cycle percentage
 *----------------------------------------------------------------------------*/
void LCD_display(unsigned int frq, char * type, unsigned int duty_cycle) {
    
    char txt[21], pc = '%';
    
    Send_Cmd_LCD(LCD_CLEAR);
    Send_Cmd_LCD(LCD_CUR_OFF);
    Send_Txt_LCD(" WAVEFORM GENERATOR", 1);
    sprintf(txt, "FRQ : %u Hz", frq);
    Send_Txt_LCD(txt, 2);
    sprintf(txt, "WAVE: %s", type);
    Send_Txt_LCD(txt, 3);
    sprintf(txt, "DC  : %d%c", duty_cycle, pc);
    Send_Txt_LCD(txt, 4);
}



/*------------------------------------------------------------------------------
 * CONFIG mode button option
 * allows to set desired option with the keypad
 *----------------------------------------------------------------------------*/
int config_mode(void) {
    
    char c = '?';
    c = scan_keypad();
    
    INTCONbits.GIE = 0;             // disable global interrupts
    PORTAbits.RA0 = 1;              // LED ON
    
    if (c != '?') {
        if (c == 'C') {             // reset to standard values
            while (PORTBbits.RB7);  // wait for button release
            f = 2, s = 0, dc = 50;
        }
        if (c == '=') {             // switch waveform type
            while (PORTBbits.RB7);
            if (s > 2)  s = 0;
            else s++;
        }
        if (c == '/') {             // decrement frequency
            while (PORTBbits.RB4);
            if (f > 0) f--;
        }
        if (c == '*') {             // increment frequency
            while (PORTBbits.RB5);
            if (f < 4) f++;
        }
        if (c == '-') {             // decrement duty cycle
            while (PORTBbits.RB6);
            if (dc > 10) dc-=10;
        }
        if (c == '+') {             // increment duty cycle
            while (PORTBbits.RB7);
            if (dc < 90) dc+=10;
        }
        return 0;
    }
    return 1;
}



/*------------------------------------------------------------------------------
 * RUN mode button option with manually configured variables
 *      frq       : frequence
 *----------------------------------------------------------------------------*/
void signal(unsigned int *frq) {
    
    unsigned int delay = 0, dc_pc = 0;
    float pc_mul = 0.0;
    
    delay = INSTR_CY/frq[f];
    pc_mul = dc*0.01;
    dc_pc = delay*pc_mul;
    
    CCPR1 = delay;                  // set configured values
    CCPR2 = dc_pc;
    
    INTCONbits.GIE = 1;             // enable global interrupts
    PORTAbits.RA0 = 0;              // LED OFF
    SPI_CS1 = 1;
    
    if (s == 1) ramp_saw();
    else if (s == 2) triang();
}



/*------------------------------------------------------------------------------
 * Ramp signal generation
 *----------------------------------------------------------------------------*/
void ramp_saw(void) {
    
    for (unsigned int i=0, j=VAL_MAX; i<VAL_MAX; i+=20, j-=20) {
        dac_write(i,1);             // ramp
        dac_write(j,2);             // sawtooth
    }
}



/*------------------------------------------------------------------------------
 * Triangle signal generation
 *----------------------------------------------------------------------------*/
void triang(void) {
    
    unsigned int half = 0, i = 0, st = 40, cpt = 0;
    half = (VAL_MAX/st)/2;
    
    while (cpt < VAL_MAX/st) {
        
        if (cpt > half) i-=st;
        else i+=st;
        
        dac_write(i,1);
        dac_write(i,2);
        cpt+=5;
    }
}




/*------------------------------------------------------------------------------
 * Send data through SPI protocol
 *      data: data to be sent
 *----------------------------------------------------------------------------*/
unsigned char spi_write(unsigned char data) {
    
    SSPBUF = data;
    while (!SSPIF);
    return SSPBUF;
 }



/*------------------------------------------------------------------------------
 * Set MCP4922 register bit while passing data through SPI protocol (mask)
 *      port: data to be sent
 *      bit1: bit we need to set for sending command to MCP4922
 *----------------------------------------------------------------------------*/
void set_reg_bit (unsigned int *port, char bit1) {
    
    *port |= (1 << bit1);
}



/*------------------------------------------------------------------------------
 * Write data on the MCP4922
 *      data   : data to be sent to the DAC
 *      channel: channel A or B
 *----------------------------------------------------------------------------*/
void dac_write(unsigned int data, char channel) {
    
    unsigned char low_b, hig_b;
    
    set_reg_bit(&data,12);                      // set bit 12 (active mode)
    set_reg_bit(&data,13);                      // set bit 13 (gain 1x)
   
    if (channel == 2)   set_reg_bit(&data,15);  // set bit 15 (channel B)
  
    low_b = data & 0x00FF;                      // mask to get lower byte
    hig_b = data >> 8;                          // get higher byte
    
    SPI_CS2=0;                                  // write data
    spi_write(hig_b);
    spi_write(low_b);   
    SPI_CS2=1;
    
    MCP_LDAC=0;                                 // sync
    MCP_LDAC=1;
}