/*
 * File:   main.c
 * Author: Alvaro Henrique Nunes de Lime R.A 2142520
 * Author: Gustavo Alexandre Dias R.A 2052229
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
#include <stdio.h>
#include <string.h>

#define _XTAL_FREQ 20000000 // 20MHz frequencia do microcontrolador
#define CARACTER_MIN 1
#define CARACTER_MAX 16
#define NCAMADA 3
#define LINHA1 0x80
#define LINHA2 0xC0
#define LOADTMR0 45536 //55536;
#define LOADTMR1H 0x63; // B1E0 0,032s
#define LOADTMR1L 0xC0; // 63C0 0,064s
#define NSENHA_TRANCA 4
#define NSENHA_ADM 4
#define SENHA_ADM "0000"
#define SENHA_TRANCA "1234"
#define NOME "Fechadura"

int tecla = -1, clique = 0, cursor = 1, camada = NCAMADA - 1, i = 0, comando = 0, op = 1, tela = 1, enter = 0, n, k, j;
char entrada[CARACTER_MAX] = "", senha_tranca[CARACTER_MAX], senha_adm[CARACTER_MAX], nome[CARACTER_MAX];

char teclado[NCAMADA][16] = {
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
        'E', 'S', 'F', 'T'
    },
    {
        'U', 'V', 'W', 'X',
        'Y', 'Z', '?', '!',
        '"', '(', ')', ',',
        '.', '-', '|', ' '
    }
};

int teclado_matricial() {
    TMR1H = LOADTMR1H;
    TMR1L = LOADTMR1L;
    T1CONbits.TMR1ON = 1;
    if (!PORTBbits.RB3) {
        tecla = !PORTBbits.RB4 ? 0 :
                !PORTBbits.RB5 ? 4 :
                !PORTBbits.RB6 ? 8 :
                !PORTBbits.RB7 ? 12 : -1;
        if (tecla == -1) {
            PORTBbits.RB3 = 1;
            PORTBbits.RB2 = 0;
        }
    }
    if (!PORTBbits.RB2) {
        tecla = !PORTBbits.RB4 ? 1 :
                !PORTBbits.RB5 ? 5 :
                !PORTBbits.RB6 ? 9 :
                !PORTBbits.RB7 ? 13 : -1;
        if (tecla == -1) {
            PORTBbits.RB2 = 1;
            PORTBbits.RB1 = 0;
        }
    }
    if (!PORTBbits.RB1) {
        tecla = !PORTBbits.RB4 ? 2 :
                !PORTBbits.RB5 ? 6 :
                !PORTBbits.RB6 ? 10 :
                !PORTBbits.RB7 ? 14 : -1;
        if (tecla == -1) {
            PORTBbits.RB1 = 1;
            PORTBbits.RB0 = 0;
        }
    }
    if (!PORTBbits.RB0) {
        tecla = !PORTBbits.RB4 ? 3 :
                !PORTBbits.RB5 ? 7 :
                !PORTBbits.RB6 ? 11 :
                !PORTBbits.RB7 ? 15 : -1;
        if (tecla == -1) {
            PORTBbits.RB0 = 1;
            PORTBbits.RB3 = 0;
        }
    }
    if (tecla == -1 && !T0CONbits.TMR0ON && clique) {
        TMR0 = LOADTMR0;
        T0CONbits.TMR0ON = 1;
    } else if (tecla != -1 && !clique) {
        camada++;
        if (camada == NCAMADA) {
            camada = 0;
        }
        clique = 1;
        T0CONbits.TMR0ON = 0;
    }
    if (tecla == -1) {
        clique = 0;
    }
    return tecla;
}

void lcd_txt(int linha, char *txt) {
    WriteCmdXLCD(linha);
    putsXLCD("                ");
    WriteCmdXLCD(linha);
    putsXLCD(txt);
}

void lcd_char(int pos, char caracter) {
    WriteCmdXLCD(pos);
    putcXLCD(caracter);
}

void lcd_teclado(int tecla, int camada) {
    if (tecla != -1) {
        if (tecla == 16) {
            if (cursor < CARACTER_MAX - 1) {
                cursor++;
                i++;
            }
        } else if (tecla == 17) {
            if (cursor > CARACTER_MIN) {
                cursor--;
                i--;
            }
        } else if (teclado[camada][tecla] == '#') {
            comando = 1;
        } else if (teclado[camada][tecla] == '*') {
            comando = 2;
        } else {
            comando = -1;
            if (cursor < CARACTER_MAX) {
                lcd_char(LINHA2 + cursor, teclado[camada][tecla]);
                entrada[i] = teclado[camada][tecla];
                entrada[i + 1] = '\0';
            }
        }
        WriteCmdXLCD(LINHA2 + cursor);
    }
}

void lcd_espera() {
    if (!BusyXLCD()) {
        __delay_ms(256);
    }
}

void teclado_config() {
    INTCON2bits.RBPU = 0; // Ativa resistores de Pull-Up para o PORT B
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

void lcd_config() {
    OpenXLCD(FOUR_BIT & LINES_5X7); // Modo 4 bits de dados e caracteres 5x7
    WriteCmdXLCD(0x01); // Limpa o LCD com retorno do cursor
    lcd_espera();
}

void rele_config() {
    TRISCbits.TRISC6 = 0;
    PORTCbits.RC6 = 0;
    TRISCbits.TRISC7 = 0;
    PORTCbits.RC7 = 0;
    TRISDbits.TRISD6 = 0;
    PORTDbits.RD6 = 0;
    TRISDbits.TRISD7 = 0;
    PORTDbits.RD7 = 0;
}

void led1_piscar() {
    PORTDbits.RD1 = !PORTDbits.RD1;
}

void led_piscar() {
    PORTDbits.RD0 = !PORTDbits.RD0;
}

void leds_config() {
    TRISDbits.TRISD0 = 0;
    PORTDbits.RD0 = 1;
    TRISDbits.TRISD1 = 0;
    PORTDbits.RD1 = 1;
}

void timer1_config() {
    TMR1H = LOADTMR1H;
    TMR1L = LOADTMR1L;
    T1CONbits.RD16 = 0; // 8-bits ou 16-bits
    T1CONbits.T1RUN = 1;
    T1CONbits.T1CKPS1 = 1;
    T1CONbits.T1CKPS0 = 1;
    T1CONbits.T1OSCEN = 0;
    T1CONbits.T1SYNC = 0;
    T1CONbits.TMR1CS = 0; // clock interno do microcontrolador
    T1CONbits.TMR1ON = 1; // Habilitar timer
    PIE1bits.TMR1IE = 1; // Habilitar Interrupcao
}

void timer0_config() {
    TMR0 = LOADTMR0;
    T0CONbits.TMR0ON = 0; // Habilitar timer
    T0CONbits.T08BIT = 0; // 8-bits ou 16-bits
    T0CONbits.T0CS = 0; // clock interno do microcontrolador
    T0CONbits.T0SE = 0;
    T0CONbits.PSA = 0; // Usar prescaler
    T0CONbits.T0PS2 = 1;
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS0 = 1;
    INTCONbits.TMR0IE = 1; // Habilitar interrupcao
}

void config_interrupcao2() {
    INTCON3bits.INT2IE = 0; // ativa a interrupcao externa INT2 (RB2)
    INTCON2bits.INTEDG2 = 0; // Interrupcao externa INT2 na borda de descida
    INTCON3bits.INT2IF = 0; // Limpa o flag bit da interrupcao extrena INT2
    INTCON3bits.INT2IP = 0;
}

void config_interrupcao1() {
    INTCON3bits.INT1IE = 0; // ativa a interrupcao externa INT1 (RB1)
    INTCON2bits.INTEDG1 = 0; // Interrupcao externa INT1 na borda de descida
    INTCON3bits.INT1IF = 0; // Limpa o flag bit da interrupcao extrena INT1
    INTCON3bits.INT1IP = 0;
}

void config_interrupcao0() {
    INTCONbits.INT0IE = 0; // ativa a interrupcao externa INT0 (RB0)
    INTCON2bits.INTEDG0 = 0; // Interrupcao externa INT0 na borda de descida
    INTCONbits.INT0IF = 0; // Limpa o flag bit da interrupcao extrena INT0
}

void interrupcao_config() {
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 0;
    RCONbits.IPEN = 1; // Nivel de prioridade
    INTCONbits.GIEH = 1; // Habilita as interrupcoes de alta prioridade
    INTCONbits.GIEL = 1; // Habilita as interrupcoes de baixa prioridade
    INTCON2bits.TMR0IP = 1;
    IPR1 = 1;
}

void __interrupt() interrupcao(void) {
    if (INTCONbits.TMR0IF) {
        clique = 0;
        switch (comando) {
            case 1:
                enter = 1;
                lcd_teclado(16, 0);
                break;
            case 2:
                lcd_teclado(17, 0);
                break;
            default:
                lcd_teclado(16, 0);
        }
        comando = 0;
        camada = NCAMADA - 1;
        led_piscar();
        T0CONbits.TMR0ON = 0;
        TMR0 = LOADTMR0;
        INTCONbits.TMR0IF = 0;
    }
    if (PIR1bits.TMR1IF) {
        tecla = teclado_matricial();
        led1_piscar();
        TMR1H = LOADTMR1H;
        TMR1L = LOADTMR1L;
        PIR1bits.TMR1IF = 0;
    }
}

void EEPROM_Guardar(int dir, char data) {
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
    while (!PIR2bits.EEIF);
    PIR2bits.EEIF = 0;
    EECON1bits.WREN = 0;
}

void EEPROM_GuardarVetor(int dir, char *data) {
    int j = 0;
    for (int i = (CARACTER_MAX * dir); i <= (CARACTER_MAX * (dir + 1)); i++) {
        EEPROM_Guardar(i, data[j]);
        j++;
    }
    EEPROM_Guardar(i, 0xFF);
}

char EEPROM_Ler(int dir) {
    EEADR = dir;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;
    return EEDATA;
}

void EEPROM_LerVetor(int dir, char *data) {

    int j = 0;
    for (int i = 0; EEPROM_Ler(CARACTER_MAX * dir + i) != 0xFF ; i++) {
        data[j] = EEPROM_Ler((CARACTER_MAX * dir) + i);
        j++;
    }
}

void EEPROM_Reset() {
    char data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (int i = 0; i < 16; i++) {
        EEPROM_GuardarVetor(i, data);
    }
    EEPROM_Guardar(0,0xFF);
};

int verifica_memoria() {
    return EEPROM_Ler(0x00) == 0xFF ? 0 : 1;
}

void putch(unsigned char data) {
    while (!PIR1bits.TXIF) // wait until the transmitter is ready
        continue;
    TXREG = data; // send one character
}

void init_uart(void) {
    TXSTAbits.TXEN = 1; // enable transmitter
    RCSTAbits.SPEN = 1; // enable serial port
}

void main(void) {
    //EEPROM_Reset();
    if (!verifica_memoria()) {

        strcpy(senha_adm, SENHA_ADM);
        EEPROM_GuardarVetor(0, senha_adm);
        strcpy(senha_tranca, SENHA_TRANCA);
        EEPROM_GuardarVetor(1, senha_tranca);
        strcpy(nome, NOME);
        EEPROM_GuardarVetor(2, nome);

    } else {
        EEPROM_LerVetor(0, senha_adm);
        EEPROM_LerVetor(1, senha_tranca);
        EEPROM_LerVetor(2, nome);
    }
    i = 0;
    interrupcao_config();
    //config_interrupcao0();
    //config_interrupcao1();
    //config_interrupcao2();
    timer0_config();
    timer1_config();
    leds_config();
    rele_config();
    lcd_config();
    teclado_config();
    while (1) {
        switch (tela) {
            case 1:
                lcd_txt(LINHA1, nome);
                lcd_txt(LINHA2, ":");
                WriteCmdXLCD(0x0F);
                break;
            case 2:
                lcd_txt(LINHA1, "Acesso permitido");
                lcd_txt(LINHA2, "Destrancado");
                WriteCmdXLCD(0x0C);
                break;
            case 3:
                lcd_txt(LINHA1, "Acesso ADM");
                lcd_txt(LINHA2, ":");
                WriteCmdXLCD(0x0F);
                break;
            case 4:
                lcd_txt(LINHA1, "Senha invalida");
                lcd_txt(LINHA2, "");
                WriteCmdXLCD(0x0C);
                __delay_ms(2048);
                break;
            case 5:
                lcd_txt(LINHA1, "Porta fechada");
                lcd_txt(LINHA2, "Trancado");
                WriteCmdXLCD(0x0C);
                __delay_ms(2048);
                break;
            case 6:
                lcd_txt(LINHA1, "Nova senha ADM");
                lcd_txt(LINHA2, ":");
                WriteCmdXLCD(0x0F);
                break;
            case 7:
                lcd_txt(LINHA1, "Nova senha");
                lcd_txt(LINHA2, ":");
                WriteCmdXLCD(0x0F);
                break;
            case 8:
                lcd_txt(LINHA1, "Novo nome");
                lcd_txt(LINHA2, ":");
                WriteCmdXLCD(0x0F);
                break;
            case 9:
                lcd_txt(LINHA1, "Escolha invalida");
                lcd_txt(LINHA2, "");
                WriteCmdXLCD(0x0C);
                __delay_ms(2048);
                break;
            case 10:
                lcd_txt(LINHA1, "Nova senha");
                lcd_txt(LINHA2, "cadastrada");
                WriteCmdXLCD(0x0C);
                __delay_ms(2048);
                break;
            case 11:
                lcd_txt(LINHA1, "Novo nome");
                lcd_txt(LINHA2, "cadastrado");
                WriteCmdXLCD(0x0C);
                __delay_ms(2048);
                break;
            default:
                tela = -1;
        }
        tela = -1;
        lcd_teclado(tecla, camada);
        lcd_espera();
        switch (op) {
            case 1:
                if (enter) {
                    if (!strcmp(senha_tranca, entrada)) {
                        op = 3;
                        tela = 2;
                        PORTCbits.RC6 = 1;
                    } else if (!strcmp(senha_adm, entrada)) {
                        op = 4;
                        tela = 3;
                    } else {
                        op = 2;
                        tela = 4;
                    }
                    cursor = 1;
                    i = 0;
                    strcpy(entrada, "");
                    enter = 0;
                }
                break;
            case 2:
                op = 1;
                tela = 1;
                cursor = 1;
                i = 0;
                strcpy(entrada, "");
                break;
            case 3:
                if (enter) {
                    op = 2;
                    tela = 5;
                    cursor = 1;
                    i = 0;
                    strcpy(entrada, "");
                    PORTCbits.RC6 = 0;
                    enter = 0;
                }
                break;
            case 4:
                if (enter) {
                    if (!strncmp("A", entrada, 1)) {
                        op = 5;
                        tela = 6;
                    } else if (!strncmp("B", entrada, 1)) {
                        op = 6;
                        tela = 7;
                    } else if (!strncmp("C", entrada, 1)) {
                        op = 7;
                        tela = 8;
                    } else {
                        op = 2;
                        tela = 9;
                    }
                    cursor = 1;
                    i = 0;
                    strcpy(entrada, "");
                    enter = 0;
                }
                break;
            case 5:
                if (enter) {
                    op = 2;
                    tela = 10;
                    strcpy(senha_adm, entrada);
                    EEPROM_GuardarVetor(0, senha_adm);
                    cursor = 1;
                    i = 0;
                    strcpy(entrada, "");
                    enter = 0;
                }
                break;
            case 6:
                if (enter) {
                    op = 2;
                    tela = 10;
                    strcpy(senha_tranca, entrada);
                    EEPROM_GuardarVetor(1, senha_tranca);
                    cursor = 1;
                    i = 0;
                    strcpy(entrada, "");
                    enter = 0;
                }
                break;
            case 7:
                if (enter) {
                    op = 2;
                    tela = 11;
                    strcpy(nome, entrada);
                    EEPROM_GuardarVetor(2, nome);
                    cursor = 1;
                    i = 0;
                    strcpy(entrada, "");
                    enter = 0;
                }
                break;
            default:
                op = -1;
        }
    }
    return;
}