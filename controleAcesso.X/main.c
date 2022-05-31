/*
 * File:   main.c
 * Author: Mario Nakazato Neto R.A 2142643
 *
 * Created on 31 de Maio de 2022, 15:05
 */

// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (RE3 input pin enabled; MCLR pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "nxlcd.h"

#define _XTAL_FREQ 20000000 // 20MHz frequencia do microcontrolador

void lcd(int tecla){
    char teclado[16] = {
        '1', '2', '3', 'A',
        '4', '5', '6', 'B',
        '7', '8', '9', 'C',
        'F', '0', 'E', 'D'
    };
    
    if(tecla != -1){
        WriteCmdXLCD(0xC7);
        putcXLCD(teclado[tecla]);
        WriteCmdXLCD(0xC7);
    }
}

int tecladoMatricial(){
    int tecla;
    if(!PORTBbits.RB3){
        tecla = !PORTBbits.RB4 ? 0: 
                !PORTBbits.RB5 ? 4: 
                !PORTBbits.RB6 ? 8: 
                !PORTBbits.RB7 ? 12: -1;
        if(tecla == -1){
            PORTBbits.RB3 = 1;
            PORTBbits.RB2 = 0;
        }
    }
    if(!PORTBbits.RB2){
        tecla = !PORTBbits.RB4 ? 1: 
                !PORTBbits.RB5 ? 5: 
                !PORTBbits.RB6 ? 9: 
                !PORTBbits.RB7 ? 13: -1;
        if(tecla == -1){
            PORTBbits.RB2 = 1;
            PORTBbits.RB1 = 0;
        }
    }
    if(!PORTBbits.RB1){
        tecla = !PORTBbits.RB4 ? 2: 
                !PORTBbits.RB5 ? 6: 
                !PORTBbits.RB6 ? 10: 
                !PORTBbits.RB7 ? 14: -1;
        if(tecla == -1){
            PORTBbits.RB1 = 1;
            PORTBbits.RB0 = 0;
        }
    }
    if(!PORTBbits.RB0){
        tecla = !PORTBbits.RB4 ? 3: 
                !PORTBbits.RB5 ? 7: 
                !PORTBbits.RB6 ? 11: 
                !PORTBbits.RB7 ? 15: -1;
        if(tecla == -1){
            PORTBbits.RB0 = 1;
            PORTBbits.RB3 = 0;
        }
    }
    return tecla;
}

void __interrupt() interrupcao(void) {
    if (INTCON3bits.INT1IF) {
        
        INTCON3bits.INT1IF = 0;
    } else if (INTCON3bits.INT2IF) {
        
        INTCON3bits.INT2IF = 0;
    /*} else if (INTCONbits.TMR0IF) {
        modo = 3;
        INTCONbits.TMR0IF = 0;
    */}
}

/*
void __interrupt(low_priority) interrupcao_baixa(void){
    //PORTD = 0x0F;
    INTCON3bits.INT1IF = 0; // Limpa o flag bit da interrupcao extrena INT1
}

void __interrupt(high_priority) interrupcao_alta(void){
    //PORTD = 0xF0;
    INTCONbits.INT0IF = 0; // Limpa o flag bit da interrupcao extrena INT0
}
 */

void config_timer0() {
    T0CONbits.TMR0ON = 1; // Habilitar timer
    T0CONbits.T08BIT = 0; // 8-bits ou 16-bits
    T0CONbits.T0CS = 0; // clock interno do microcontrolador
    //T0CONbits.T0SE = 0;
    T0CONbits.PSA = 1; // Usar prescaler
    T0CONbits.T0PS2 = 1;
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS0 = 1;
    INTCONbits.TMR0IE = 1; // Habilitar Timer
}

void config_interrupcao2() {
    INTCON3bits.INT2IE = 1; // ativa a interrupcao externa INT2 (RB2)
    INTCON2bits.INTEDG2 = 0; // Interrupcao externa INT2 na borda de descida
    INTCON3bits.INT2IF = 0; // Limpa o flag bit da interrupcao extrena INT2
    //INTCON3bits.INT2IP = 0;
}

void config_interrupcao1() {
    INTCON3bits.INT1IE = 1; // ativa a interrupcao externa INT1 (RB1)
    INTCON2bits.INTEDG1 = 0; // Interrupcao externa INT1 na borda de descida
    INTCON3bits.INT1IF = 0; // Limpa o flag bit da interrupcao extrena INT1
    //INTCON3bits.INT1IP = 0;
}

void config_interrupcao0() {
    INTCONbits.INT0IE = 1; // ativa a interrupcao externa INT0 (RB0)
    INTCON2bits.INTEDG0 = 0; // Interrupcao externa INT0 na borda de descida
    INTCONbits.INT0IF = 0; // Limpa o flag bit da interrupcao extrena INT0
}

void config_interrupcao() {
    RCONbits.IPEN = 1; // Com nivel de prioridade
    INTCONbits.GIEH = 1; // Habilita as interrupcoes de alta prioridade
    INTCONbits.GIEL = 1; // Habilita as interrupcoes de baixa prioridade
}

void main(void) {
    
    //Inicializa??o do LCD
    OpenXLCD(FOUR_BIT & LINES_5X7); // Modo 4 bits de dados e caracteres 5x7
    WriteCmdXLCD(0x01);      	    // Limpa o LCD com retorno do cursor
    __delay_ms(8);  	 	        // Atraso de 10ms para aguardar a execu??o do comando
    
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1;

    PORTBbits.RB0 = 1;
    PORTBbits.RB1 = 1;
    PORTBbits.RB2 = 1;
    PORTBbits.RB3 = 0;
    
    RBPU = 0; // Ativa resistores de Pull-Up para o PORT B
    //INTCON2bits.RBPU = 0;
    ADCON1 = 0x0F; // PORTA configurada como I/O digital

    config_interrupcao();
    //config_interrupcao0();
    config_interrupcao1();
    config_interrupcao2();
    //config_timer0();
        //TMR0 = 22000;
    
    WriteCmdXLCD(0x85);
    putsXLCD("*TXT*");
    WriteCmdXLCD(0xC7);
        
    int tecla, teclaAnterior;
    
    while(1){
        tecla = tecladoMatricial();
        if(tecla != teclaAnterior){
            lcd(tecla);
        }
        teclaAnterior = tecla;
        if(!BusyXLCD()){
            __delay_ms(8);
        }
    }

    return;
}