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
#include <stdio.h>

#define _XTAL_FREQ 20000000 // 20MHz frequencia do microcontrolador
#define CARACTER_MIN 1
#define CARACTER_MAX 16
#define NCAMADA 3
#define LINHA1 0x80
#define LINHA2 0xC0
#define LOADTMR0 0 //55536;
#define LOADTMR1H 0xB1;
#define LOADTMR1L 0xE0;
#define MMRINIT1 1
#define MMREND1 5
#define MMRINIT2 5
#define MMREND2 9
#define MMRINIT3 9
#define MMREND3 25

int tecla = -1, clique = 0, cursor = 1, camada = NCAMADA - 1, i, comando = 0;
char senha[CARACTER_MAX] = "";

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
            }
        } else if (tecla == 17) {
            if (cursor > CARACTER_MIN) {
                cursor--;
            }
        } else if (teclado[camada][tecla] == '#') {
            comando = 1;
        } else if (teclado[camada][tecla] == '*') {
            comando = 2;
        } else {
            comando = -1;
            if (cursor < CARACTER_MAX) {
                lcd_char(LINHA2 + cursor, teclado[camada][tecla]);
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
    lcd_txt(LINHA1, "Fechadura");
    lcd_txt(LINHA2, ":");
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

unsigned char EEPROM_Ler(int dir) {
    EEADR = dir;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;
    return EEDATA;
}

int verificaMemoria() {
    if (EEPROM_Ler(0) == 'S') {
        return 0;
    } else return 1;
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
    interrupcao_config();
    //config_interrupcao0();
    //config_interrupcao1();
    //config_interrupcao2();
    timer0_config();
    timer1_config();
    leds_config();
    lcd_config();
    teclado_config();

    int senha[4];
    char nomeTranca[CARACTER_MAX], opc;
    int ctrl, i, j, n;
    int senhaAtual[4], senhaAdmin[4];

    init_uart();

    if (verificaMemoria()) {
        printf("Escreve na memory");
        EEPROM_Guardar(0, 'S');
        for (i = MMRINIT1; i < MMREND1; i++) {
            EEPROM_Guardar(i, 1);
            senhaAtual[i] = 1;
            EEPROM_Guardar(i + 4, i);
            senhaAdmin[i] = i;
        }
        strcpy(nomeTranca, "Trunca");
        n = strlen(nomeTranca);

        j = 0;
        for (i = MMRINIT3; i < MMREND3 - 1; i++) {
            EEPROM_Guardar(i, nomeTranca[j]);
            j++;
        }
        EEPROM_Guardar(j, '/');

    } else {
        for (i = MMRINIT1; i < MMREND1; i++) {
            senhaAtual[i] = atoi(EEPROM_Ler(i));
            printf("%d", senhaAtual[i]);
        }
        printf("\n");
        for (i = MMRINIT2; i < MMREND2; i++) {
            senhaAdmin[i] = atoi(EEPROM_Ler(i));
            printf("%d", senhaAdmin[i]);
        }
    }

    //Display inicial
    i = MMRINIT3;
    j = 0;
    while (EEPROM_Ler(i) != '/') {
        nomeTranca[j] = EEPROM_Ler(i);
        j++;
        i++;
    }

    lcd_txt(LINHA1, nomeTranca);
    lcd_txt(LINHA2, ":");

    while (1) {
        lcd_teclado(tecla, camada);
        lcd_espera();
        //Testa se a � a senha da tranca
        ctrl = 1;
        for (int i = 0; i < 4; i++) {
            if (senha[i] != senhaAtual[i]) {
                ctrl = 2;
            }
        }
        //Testa se � a senha do admin
        for (int i = 0; i < 4; i++) {
            if (senha[i] != senhaAdmin[i]) {
                ctrl = 0;
            }
        }
        if (ctrl == 1) {
            //Atualiza Display
            //Aciona rel�
            PORTCbits.RC6 = 1;
            __delay_ms(100);
            PORTCbits.RC6 = 0;
            //Acende o LED
            PORTDbits.RD0 = 1;
            //Verifica se a porta esta aberta
            while (PORTEbits.RE3) {
            }
            //Apaga o LED
            PORTDbits.RD0 = 0;
            //Fecha a tranca
            PORTCbits.RC6 = 1;
            __delay_ms(100);
            PORTCbits.RC6 = 0;
        }
        if (ctrl == 2) {
            //Atualiza Display
            //Le teclado
            //Op��o A
            if (opc == 'A') {
                //Altera memoria
                for (i = MMRINIT1; i < MMREND1; i++) {
                    j = 0;
                    EEPROM_Guardar(i, senhaAtual[j]);
                    j++;
                }
            }
            //Op��o B
            if (opc == 'B') {
                //Carrega na mem�ria a senha de admin nova
                for (i = MMRINIT2; i < MMREND2; i++) {
                    j = 0;
                    EEPROM_Guardar(i, senhaAtual[i - 5]);
                    j++;
                }
            }
            //Op��o C
            if (opc == 'C') {
                //Carrega na mem�ria a senha de admin nova
                for (i = MMRINIT3; i < MMREND3; i++) {
                    j = 0;
                    EEPROM_Guardar(i, nomeTranca[i - 9]);
                    j++;
                }
            }
            //Op��o D
            //Atualiza display
            //Sai da opcao
        }
    }
    return;
}