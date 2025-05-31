#ifndef _DEF_PRINCIPAIS_H
#define _DEF_PRINCIPAIS_H

#define F_CPU 16000000UL // define a frequencia do microcontrolador - 16MHz

#include <avr/io.h>       //definições do componente especificado
#include <util/delay.h>   //biblioteca para o uso das rotinas de _delay_ms e _delay_us()
#include <avr/pgmspace.h> //para o uso do PROGMEM, gravação de dados na memória flash

// Definições de macros para o trabalho com bits
#define set_bit(y, bit) (y |= (1 << bit))  // coloca em 1 o bit x da variável Y
#define clr_bit(y, bit) (y &= ~(1 << bit)) // coloca em 0 o bit x da variável Y
#define cpl_bit(y, bit) (y ^= (1 << bit))  // troca o estado lógico do bit x da variável Y
#define tst_bit(y, bit) (y & (1 << bit))   // retorna 0 ou 1 conforme leitura do bit

#endif

#ifndef _LCD_H
#define _LCD_H

// Definições para facilitar a troca dos pinos do hardware e facilitar a re-programação
#define DADOS_LCD PORTC // 4 bits de dados do LCD no PORTC

#define nibble_dados 0 // 0 para via de dados do LCD nos 4 LSBs do PORT empregado (Px0-D4, Px1-D5, Px2-D6, Px3-D7)
                       // 1 para via de dados do LCD nos 4 MSBs do PORT empregado (Px4-D4, Px5-D5, Px6-D6, Px7-D7)

#define CONTR_LCD PORTC // PORT com os pinos de controle do LCD (pino R/W em 0).
#define E PC4           // pino de habilitação do LCD (enable)
#define RS PC5          // pino para informar se o dado é uma instrução ou caractere

// sinal de habilitação para o LCD
#define pulso_enable()     \
    _delay_us(1);          \
    set_bit(CONTR_LCD, E); \
    _delay_us(1);          \
    clr_bit(CONTR_LCD, E); \
    _delay_us(45)

// protótipo das funções
void cmd_LCD(unsigned char c, char cd);
void inic_LCD_4bits();

#endif

// Sub-rotina para enviar caracteres e comandos ao LCD com via de dados de 4 bits

void cmd_LCD(unsigned char c, char cd)
{ // c = caractere ou comando, cd = 0 para comando, 1 para caractere
    if (cd == 0)
        clr_bit(CONTR_LCD, RS);
    else
        set_bit(CONTR_LCD, RS);

// primeiro nibble de dados - 4 MSB
#if (nibble_dados) // compila código para os pinos de dados do LCD nos 4 MSB do PORT
    DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & c);
#else // compila código para os pinos de dados do LCD nos 4 LSB do PORT
    DADOS_LCD = (DADOS_LCD & 0xF0) | (c >> 4);
#endif

    pulso_enable();

// segundo nibble de dados - 4 LSB
#if (nibble_dados) // compila código para os pinos de dados do LCD nos 4 MSB do PORT
    DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (c << 4));
#else // compila código para os pinos de dados do LCD nos 4 LSB do PORT
    DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & c);
#endif

    pulso_enable();

    if ((cd == 0) && (c < 4)) // se for instrução de retorno ou limpeza espera LCD estar pronto
        _delay_ms(2);
}

// Sub-rotina para inicialização do LCD com via de dados de 4 bits

void inic_LCD_4bits()
{ // sequência ditada pelo fabricando do circuito integrado HD44780
  // o LCD será só escrito. Então, R/W é sempre zero.

    clr_bit(CONTR_LCD, RS); // RS em zero indicando que o dado para o LCD será uma instrução
    clr_bit(CONTR_LCD, E);  // pino de habilitação em zero

    _delay_ms(20); // tempo para estabilizar a tensão do LCD, após VCC ultrapassar 4.5 V (na prática pode ser maior).

// interface de 8 bits
#if (nibble_dados)
    DADOS_LCD = (DADOS_LCD & 0x0F) | 0x30;
#else
    DADOS_LCD = (DADOS_LCD & 0xF0) | 0x03;
#endif

    pulso_enable(); // habilitação respeitando os tempos de resposta do LCD
    _delay_ms(5);
    pulso_enable();
    _delay_us(200);
    pulso_enable();
/*até aqui ainda é uma interface de 8 bits.
Muitos programadores desprezam os comandos acima, respeitando apenas o tempo de
estabilização da tensão (geralmente funciona). Se o LCD não for inicializado primeiro no
modo de 8 bits, haverá problemas se o microcontrolador for inicializado e o display já o tiver sido.*/

// interface de 4 bits, deve ser enviado duas vezes (a outra está abaixo)
#if (nibble_dados)
    DADOS_LCD = (DADOS_LCD & 0x0F) | 0x20;
#else
    DADOS_LCD = (DADOS_LCD & 0xF0) | 0x02;
#endif

    pulso_enable();
    cmd_LCD(0x28, 0); // interface de 4 bits 2 linhas (aqui se habilita as 2 linhas)
                      // são enviados os 2 nibbles (0x2 e 0x8)
    cmd_LCD(0x08, 0); // desliga o display
    cmd_LCD(0x01, 0); // limpa todo o display
    cmd_LCD(0x0C, 0); // mensagem aparente cursor inativo não piscando
    cmd_LCD(0x80, 0); // inicializa cursor na primeira posição a esquerda - 1a linha
}

// variaveis auxiliares

// botoes
#define debounceInterval 1  // ms
unsigned int debounce1 = 0; // tempo para debounce dos botoes
unsigned int debounce2 = 0;
int state1 = 1; // ultimo estado dos botoes
int state2 = 1;
int botao1 = 1;
int botao2 = 1;

// contagem dos placares
char placar1E = 0, placar1D = 0;
char placar2E = 0, placar2D = 0;
unsigned char valor1 = 0;
unsigned char valor2 = 0;

// cronometro
int minuto, segundo1, segundo0, ms1, ms0;
long cronometro, tempo = 599000;

const unsigned char msg1[] PROGMEM = "Cronometro";

void setup()
{
    DDRC = 0xFF;       // PORTC como saida
    DDRD = 0b00111111; // PORTD D como saida e PB7 E PB6 como entrada

    PORTD = 0b11000000; // pull-up no PB7 E PB6

    inic_LCD_4bits();

    if (tempo > 599000)
    {
        tempo = 599000;
    }

    cmd_LCD(0x80, 0);
    cmd_LCD('T', 1);
    cmd_LCD('1', 1);

    cmd_LCD(0x83, 0);
    for (int i = 0; i < 10; i++) // enviando caractere por caractere
        cmd_LCD(pgm_read_byte(&msg1[i]), 1);

    cmd_LCD(0x8E, 0);
    cmd_LCD('T', 1);
    cmd_LCD('2', 1);
}

void loop()
{
    cronometro = tempo - millis();

    if (cronometro > 0)
    {
        // cronometro
        minuto = cronometro / 60000;
        segundo1 = (cronometro % 60000) / 10000;
        segundo0 = (cronometro % 10000) / 1000;
        ms1 = (cronometro % 1000) / 100;
        ms0 = (cronometro % 100) / 10;

        // botao 1
        int leitura1 = PIND & (1 << PD6);
        if (leitura1 != state1)
        {
            debounce1 = millis();
        }
        if ((millis() - debounce1) > debounceInterval)
        {
            if (botao1 != leitura1)
            {
                botao1 = leitura1;
                if (botao1 == 0)
                {
                    if (valor1 == 99)
                    {
                        valor1 = 0;
                    }
                    else
                    {
                        valor1++;
                    }
                    placar1E = valor1 % 10;
                    placar1D = valor1 / 10;
                }
            }
        }
        state1 = leitura1;

        // botao 2
        int leitura2 = PIND & (1 << PD7);
        if (leitura2 != state2)
        {
            debounce2 = millis();
        }
        if ((millis() - debounce2) > debounceInterval)
        {
            if (botao2 != leitura2)
            {
                botao2 = leitura2;
                if (botao2 == 0)
                {
                    if (valor2 == 99)
                    {
                        valor2 = 0;
                    }
                    else
                    {
                        valor2++;
                    }
                    placar2E = valor2 % 10;
                    placar2D = valor2 / 10;
                }
            }
        }
        state2 = leitura2;
    }

    // cronometro
    if (millis() % 27 == 0)
    { // atualiza o display apenas quando millis for divisivel por 27 para nao sobrecarregar
        cmd_LCD(0xC4, 0);
        cmd_LCD('0', 1);            // dezena dos minutos == 0
        cmd_LCD(minuto + '0', 1);   // unidade do minuto
        cmd_LCD(':', 1);            //: para separar minutos dos segundos
        cmd_LCD(segundo1 + '0', 1); // dezena dos segundos
        cmd_LCD(segundo0 + '0', 1); // unidade dos segundos
        cmd_LCD(':', 1);            //: para separar segundos dos millisegundos
        cmd_LCD(ms1 + '0', 1);      // dezena dos milisegundos
        cmd_LCD(ms0 + '0', 1);      // unidade dos milisegundos

        cmd_LCD(0xC0, 0);
        cmd_LCD(placar1D + '0', 1); // placar time 1 unidade (direita)
        cmd_LCD(placar1E + '0', 1); // placar time 1 dezena (esquerda)

        // TIME 2
        cmd_LCD(0xCE, 0);
        cmd_LCD(placar2D + '0', 1); // placar time 2 unidade (direita)
        cmd_LCD(placar2E + '0', 1); // placar time 2 dezena (esquerda)
    }
}