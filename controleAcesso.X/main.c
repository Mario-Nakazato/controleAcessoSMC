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
#include <string.h>
#include "nxlcd.h"

#define _XTAL_FREQ 20000000 // 20MHz frequencia do microcontrolador
#define CAMADA_TECLADO 3
#define LOADTMR0 100
#define LOADTMR1 0
#define CARACTER_MAX 16
#define CARACTER_MIN 1
#define LINHA1 0x80
#define LINHA2 0xC0
#define MMRINIT1 1
#define MMREND1 5
#define MMRINIT2 5
#define MMREND2 9
#define MMRINIT3 9
#define MMREND3 25

int cursor = LINHA2 +1;

void lcdTxt(int linha, char *txt){
    WriteCmdXLCD(linha);
    putrsXLCD(txt);
}

void lcd(int tecla, int camada){
    char teclado[3][16] = {
        {
        '1', '2', '3', 'A',
        '4', '5', '6', 'B',
        '7', '8', '9', 'C',
        '*', '0', '#', 'D'
        },
        {
        'G', 'H', 'I', 'J',
        'K', 'L', 'M', 'N',
        'O', 'P', 'Q', 'R',
        'F', 'S', 'E', 'T'
        },
        {
        'U', 'V', 'W', 'X',
        'Y', 'Z', '?', '!',
        '"', '"', '(', ')',
        'esquerda', '0', 'E', ' '
        }
    };
    
    if(tecla != -1){
        if(teclado[camada][tecla] == 'esquerda' && cursor > LINHA2 +CARACTER_MIN){
            cursor--;
        }else if(teclado[camada][tecla] == ' ' && cursor < LINHA2 +CARACTER_MAX){
            cursor++;
        }else if(teclado[camada][tecla] != 'esquerda' && teclado[camada][tecla] != '#'){
            if(cursor < LINHA2 +CARACTER_MAX){
                WriteCmdXLCD(cursor);
                putcXLCD(teclado[camada][tecla]);
                if(T1CONbits.TMR1ON == 0){
                    cursor++;
                }
            }
        }
        WriteCmdXLCD(cursor);
    }
    if(!BusyXLCD()){
        __delay_ms(8);
    }
}

void varreduraTeclado(){
    if(!PORTBbits.RB3){
        PORTBbits.RB3 = 1;
        PORTBbits.RB2 = 0;
    }else if(!PORTBbits.RB2){
        PORTBbits.RB2 = 1;
        PORTBbits.RB1 = 0;
    }else if(!PORTBbits.RB1){
        PORTBbits.RB1 = 1;
        PORTBbits.RB0 = 0;
    }else if(!PORTBbits.RB0){
        PORTBbits.RB0 = 1;
        PORTBbits.RB3 = 0;
    }
    TMR0 = LOADTMR0;
    //PORTDbits.RD0 = !PORTDbits.RD0;
}

int tecladoMatricial(int tecla){
    if(!PORTBbits.RB3){
        tecla = !PORTBbits.RB4 ? 0: 
                !PORTBbits.RB5 ? 4: 
                !PORTBbits.RB6 ? 8: 
                !PORTBbits.RB7 ? 12: -1;
    }else if(!PORTBbits.RB2){
        tecla = !PORTBbits.RB4 ? 1: 
                !PORTBbits.RB5 ? 5: 
                !PORTBbits.RB6 ? 9: 
                !PORTBbits.RB7 ? 13: -1;
    }else if(!PORTBbits.RB1){
        tecla = !PORTBbits.RB4 ? 2: 
                !PORTBbits.RB5 ? 6: 
                !PORTBbits.RB6 ? 10: 
                !PORTBbits.RB7 ? 14: -1;
    }else if(!PORTBbits.RB0){
        tecla = !PORTBbits.RB4 ? 3: 
                !PORTBbits.RB5 ? 7: 
                !PORTBbits.RB6 ? 11: 
                !PORTBbits.RB7 ? 15: -1;
    }
    if(tecla != -1){
        T0CONbits.TMR0ON = 0;
    }
    if(tecla == -1 && !T0CONbits.TMR0ON){
        TMR0 = LOADTMR0;
        T0CONbits.TMR0ON = 1;
        TMR1 = LOADTMR1;
        T1CONbits.TMR1ON = 1;
    }
    return tecla;
}

void config_teclado(){
    RBPU = 0; // Ativa resistores de Pull-Up para o PORT B
    //INTCON2bits.RBPU = 0;
    ADCON1 = 0x0F; // PORTA configurada como I/O digital
    
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
}

void config_ldc(){
    OpenXLCD(FOUR_BIT & LINES_5X7); // Modo 4 bits de dados e caracteres 5x7
    WriteCmdXLCD(0x01);      	    // Limpa o LCD com retorno do cursor
    __delay_ms(8);
    lcdTxt(LINHA1, "Fechadura");
    lcdTxt(LINHA2, ":");
}

void config_led(){
    TRISDbits.TRISD0 = 0;
    PORTDbits.RD0 = 1;
}

void __interrupt() interrupcao(void) {
    /*if (INTCON3bits.INT1IF) {
        
        INTCON3bits.INT1IF = 0;
    } else if (INTCON3bits.INT2IF) {
        
        INTCON3bits.INT2IF = 0;
    } else */if (INTCONbits.TMR0IF) {
        varreduraTeclado();
        //TMR0 = LOADTMR0;
        INTCONbits.TMR0IF = 0;
    } else if (PIR1bits.TMR1IF) {
        PORTDbits.RD0 = !PORTDbits.RD0;
        T1CONbits.TMR1ON = 0;
        PIR1bits.TMR1IF = 0;
    }
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

void config_timer1() {
    TMR1H = 0x00;
    TMR1L = 0x00;
    T1CONbits.TMR1ON = 1; // Habilitar timer
    T1CONbits.RD16 = 0; // 8-bits ou 16-bits
    T1CONbits.TMR1CS = 0; // clock interno do microcontrolador
    T1CONbits.T1CKPS1 = 1;
    T1CONbits.T1CKPS0 = 1;
    PIE1bits.TMR1IE = 1; // Habilitar Timer
    TMR1 = LOADTMR1;
}

void config_timer0() {
    T0CONbits.TMR0ON = 1; // Habilitar timer
    T0CONbits.T08BIT = 1; // 8-bits ou 16-bits
    T0CONbits.T0CS = 0; // clock interno do microcontrolador
    //T0CONbits.T0SE = 0;
    T0CONbits.PSA = 1; // Usar prescaler
    T0CONbits.T0PS2 = 1;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS0 = 0;
    INTCONbits.TMR0IE = 1; // Habilitar Timer
    TMR0 = LOADTMR0;
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

void EEPROM_Guardar(int dir, char data){
    EEADR = dir;
    EEDATA = data;
    EECON1bits.EEPGD = 0; 
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0x0AA;
    EECON1bits.WR = 1;
    INTCONbits.GIE = 1;
    while(!PIR2bits.EEIF);
    PIR2bits.EEIF = 0;
    EECON1bits.WREN = 0;
}

unsigned char EEPROM_Ler(int dir){
    EEADR = dir;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;
    return EEDATA;
}

int verificaMemoria(){
    if(EEPROM_Ler(0)=='S'){
        return 0;
    }
    else return 1;
}


void main(void) {
    
    config_interrupcao();
    //config_interrupcao0();
    //config_interrupcao1();
    //config_interrupcao2();
    config_timer0();
    config_timer1();
    config_led();
    config_teclado();
    config_ldc();
    
    int tecla = -1, teclaAnterior = -1, camada = 0;
    int senha[4]; 
    char nomeTranca[CARACTER_MAX],opc;
    int ctrl,i,j,n;
    int senhaAtual[4], senhaAdmin[4];
    
    if(verificaMemoria()){
        EEPROM_Guardar(0,'S');
        for (i=MMRINIT1;i<MMREND1;i++){
            EEPROM_Guardar(i,0);
            senhaAtual[i] = 0;
            EEPROM_Guardar(i+4,i+1);
            senhaAdmin[i] = i+1;
        }
        strcpy(nomeTranca,"Tranca");
        n = strlen(nomeTranca);
        
        j=0;
        for(i=MMRINIT3;i<MMREND3;i++){
            EEPROM_Guardar(i,nomeTranca[j]);
            j++;
        }
        EEPROM_Guardar(j,'/');
        
    }else{
        for (i=1;i<5;i++){
            senhaAtual[i] = atoi(EEPROM_Ler(i));
        }
        for (i= 5;i<9;i++){
            senhaAdmin[i] = atoi(EEPROM_Ler(i));
        }
    }
    
    //Display inicial
    i=MMRINIT3;
    j=0;
    while(EEPROM_Ler(i) != '/'){
        nomeTranca[j]=EEPROM_Ler(i);
        j++;
        i++;
    }    
    
    while(1){
        tecla = tecladoMatricial(tecla);
        if(T1CONbits.TMR1ON == 1){
            if(camada < CAMADA_TECLADO){
                camada++;
            }else{
                camada = 0;
            }
        }else{
            camada = 0;
        }
        if(teclaAnterior == -1 && tecla != teclaAnterior){
            lcd(tecla, camada);
        }
        teclaAnterior = tecla;
        
        //Testa se a é a senha da tranca
        ctrl = 1;
        for(int i=0;i<4;i++){
            if(senha[i] != senhaAtual[i]){
                ctrl = 2;
            }
        }
        //Testa se é a senha do admin
        for(int i=0;i<4;i++){
            if(senha[i] != senhaAdmin[i]){
                ctrl = 0;
            }
        }
        
        if(ctrl == 1){
            //Atualiza Display
            //Aciona relé
            PORTCbits.RC6 = 1;
            __delay_ms(100);
            PORTCbits.RC6 = 0;
            //Acende o LED
            PORTDbits.RD0 = 1;                  
            //Verifica se a porta esta aberta
            while(PORTEbits.RE3){
            }
            //Apaga o LED
            PORTDbits.RD0 = 0;
            //Fecha a tranca
            PORTCbits.RC6 = 1;
            __delay_ms(100);
            PORTCbits.RC6 = 0;
        }
        if(ctrl == 2){
            //Atualiza Display
            //Le teclado
            //Opção A
            if(opc == 'A'){
                //Altera memoria
                EEPROM_Guardar(0,'S');
                //Carrega na memória a senha da tranca nova
                for(int i=1;i<5;i++){
                    EEPROM_Guardar(i,senhaAtual[i-1]);
                }
            }
            //Opção B
                //Altera memoria
                EEPROM_Guardar(0,'S');
                //Carrega na memória a senha de admin nova
                for(int i=5;i<9;i++){
                    EEPROM_Guardar(i,senhaAtual[i-5]);
                }
            //Opção C
                //Altera memoria
                EEPROM_Guardar(0,'S');
                //Carrega na memória a senha de admin nova
                for(int i=9;i<(i+CARACTER_MAX);i++){
                    EEPROM_Guardar(i,nomeTranca[i-9]);
                }
            //Opção D
                //Atualiza display
                //Sai da opcao
        }
    }
    return;
}