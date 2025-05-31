#ifndef _DEF_PRINCIPAIS_H
#define _DEF_PRINCIPAIS_H

#define F_CPU 16000000UL // define a frequencia do microcontrolador - 16MHz

#include <avr/io.h>       //definições do componente especificado
#include <util/delay.h>   //biblioteca para o uso das rotinas de _delay_ms e _delay_us()
#include <avr/pgmspace.h> //para o uso do PROGMEM, gravação de dados na memória flash

// LCD and bit manipulation macros
#define LCD_DATA_PORT PORTC
#define LCD_CONTROL_PORT PORTC
#define LCD_ENABLE_PIN PC4
#define LCD_RS_PIN PC5
#define LCD_USE_MSB 0 // 0: LSB, 1: MSB

#define SET_BIT(y, bit) (y |= (1 << bit))
#define CLR_BIT(y, bit) (y &= ~(1 << bit))
#define CPL_BIT(y, bit) (y ^= (1 << bit))
#define TST_BIT(y, bit) (y & (1 << bit))

#define LCD_PULSE_ENABLE()                     \
    _delay_us(1);                              \
    SET_BIT(LCD_CONTROL_PORT, LCD_ENABLE_PIN); \
    _delay_us(1);                              \
    CLR_BIT(LCD_CONTROL_PORT, LCD_ENABLE_PIN); \
    _delay_us(45)

void lcdSendCommand(unsigned char value, char isData);
void lcdInit4Bit();

#endif

// Sub-rotina para enviar caracteres e comandos ao LCD com via de dados de 4 bits

void lcdSendCommand(unsigned char value, char isData)
{
    if (isData == 0)
        CLR_BIT(LCD_CONTROL_PORT, LCD_RS_PIN);
    else
        SET_BIT(LCD_CONTROL_PORT, LCD_RS_PIN);

#if (LCD_USE_MSB)
    LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (0xF0 & value);
#else
    LCD_DATA_PORT = (LCD_DATA_PORT & 0xF0) | (value >> 4);
#endif
    LCD_PULSE_ENABLE();
#if (LCD_USE_MSB)
    LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (0xF0 & (value << 4));
#else
    LCD_DATA_PORT = (LCD_DATA_PORT & 0xF0) | (0x0F & value);
#endif
    LCD_PULSE_ENABLE();
    if ((isData == 0) && (value < 4))
        _delay_ms(2);
}

// Sub-rotina para inicialização do LCD com via de dados de 4 bits

void lcdInit4Bit()
{
    CLR_BIT(LCD_CONTROL_PORT, LCD_RS_PIN);
    CLR_BIT(LCD_CONTROL_PORT, LCD_ENABLE_PIN);
    _delay_ms(20);
#if (LCD_USE_MSB)
    LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x30;
#else
    LCD_DATA_PORT = (LCD_DATA_PORT & 0xF0) | 0x03;
#endif
    LCD_PULSE_ENABLE();
    _delay_ms(5);
    LCD_PULSE_ENABLE();
    _delay_us(200);
    LCD_PULSE_ENABLE();
#if (LCD_USE_MSB)
    LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x20;
#else
    LCD_DATA_PORT = (LCD_DATA_PORT & 0xF0) | 0x02;
#endif
    LCD_PULSE_ENABLE();
    lcdSendCommand(0x28, 0);
    lcdSendCommand(0x08, 0);
    lcdSendCommand(0x01, 0);
    lcdSendCommand(0x0C, 0);
    lcdSendCommand(0x80, 0);
}

// Button debounce and scoreboard variables
#define DEBOUNCE_INTERVAL 1

typedef struct
{
    unsigned char value;
    char units;
    char tens;
    int lastButtonState;
    int buttonState;
    unsigned int debounceTime;
    int pin;
} ScoreButton;

typedef struct
{
    int minute;
    int secondTens;
    int secondUnits;
    int msTens;
    int msUnits;
    long value;
    long start;
} Timer;

ScoreButton scoreButton1 = {0, 0, 0, 1, 1, 0, PD6};
ScoreButton scoreButton2 = {0, 0, 0, 1, 1, 0, PD7};
Timer timer = {0, 0, 0, 0, 0, 0, 599000};

const unsigned char lcdTitle[] PROGMEM = "Cronometro";

void initializeHardware();
void initializeDisplay();
void updateTimerValues();
void handleButtonPress(ScoreButton *scoreBtn);
void updateDisplay();
void updateScore(ScoreButton *scoreBtn);

// Caracteres customizados para animação (3 frames)
const uint8_t animFrames[3][8] = {
    {0b00100, 0b01110, 0b10101, 0b00100, 0b00100, 0b10101, 0b01110, 0b00100}, // Frame 1
    {0b00100, 0b01110, 0b10101, 0b00100, 0b01110, 0b00100, 0b10101, 0b01110}, // Frame 2
    {0b00100, 0b01110, 0b10101, 0b01110, 0b00100, 0b01110, 0b10101, 0b00100}  // Frame 3
};

void lcdCreateChar(uint8_t location, const uint8_t *charmap);
void displayLayout();

uint8_t animIndex = 0;
unsigned long lastAnimTime = 0;

void setup()
{
    initializeHardware();
    initializeDisplay();
    lcdCreateChar(0, animFrames[0]);
}

void initializeHardware()
{
    DDRC = 0xFF;
    DDRD = 0b00111111;
    PORTD = 0b11000000;
    lcdInit4Bit();
    if (timer.start > 599000)
        timer.start = 599000;
}

void initializeDisplay()
{
    lcdSendCommand(0x80, 0);
    lcdSendCommand('T', 1);
    lcdSendCommand('1', 1);
    lcdSendCommand(0x83, 0);
    for (int i = 0; i < 10; i++)
        lcdSendCommand(pgm_read_byte(&lcdTitle[i]), 1);
    lcdSendCommand(0x8E, 0);
    lcdSendCommand('T', 1);
    lcdSendCommand('2', 1);
}

void updateTimerValues()
{
    timer.value = timer.start - millis();
    timer.minute = timer.value / 60000;
    timer.secondTens = (timer.value % 60000) / 10000;
    timer.secondUnits = (timer.value % 10000) / 1000;
    timer.msTens = (timer.value % 1000) / 100;
    timer.msUnits = (timer.value % 100) / 10;
}

void handleButtonPress(ScoreButton *scoreBtn)
{
    int readButton = PIND & (1 << scoreBtn->pin);
    if (readButton != scoreBtn->lastButtonState)
        scoreBtn->debounceTime = millis();
    if ((millis() - scoreBtn->debounceTime) > DEBOUNCE_INTERVAL)
    {
        if (scoreBtn->buttonState != readButton)
        {
            scoreBtn->buttonState = readButton;
            if (scoreBtn->buttonState == 0)
                updateScore(scoreBtn);
        }
    }
    scoreBtn->lastButtonState = readButton;
}

void updateScore(ScoreButton *scoreBtn)
{
    if (scoreBtn->value == 99)
        scoreBtn->value = 0;
    else
        scoreBtn->value++;
    scoreBtn->units = scoreBtn->value % 10;
    scoreBtn->tens = scoreBtn->value / 10;
}

void lcdCreateChar(uint8_t location, const uint8_t *charmap)
{
    lcdSendCommand(0x40 | ((location & 0x7) << 3), 0); // Endereço CGRAM
    for (uint8_t i = 0; i < 8; i++)
    {
        lcdSendCommand(charmap[i], 1);
    }
}

void displayLayout()
{
    // Linha 1: __A: 00   __   __   __   __   B:00
    lcdSendCommand(0x80, 0);                     // Início da linha 1
    lcdSendCommand(' ', 1);                      // 1º posição
    lcdSendCommand(' ', 1);                      // 2º posição
    lcdSendCommand('A', 1);                      // 3º posição
    lcdSendCommand(':', 1);                      // 4º posição
    lcdSendCommand(' ', 1);                      // 5º posição
    lcdSendCommand(scoreButton1.tens + '0', 1);  // 6º posição (dezena placar A)
    lcdSendCommand(scoreButton1.units + '0', 1); // 7º posição (unidade placar A)
    lcdSendCommand(' ', 1);                      // 8º posição
    lcdSendCommand(' ', 1);                      // 9º posição
    lcdSendCommand(' ', 1);                      // 10º posição
    lcdSendCommand(' ', 1);                      // 11º posição
    lcdSendCommand('B', 1);                      // 12º posição
    lcdSendCommand(':', 1);                      // 13º posição
    lcdSendCommand(scoreButton2.tens + '0', 1);  // 14º posição (dezena placar B)
    lcdSendCommand(scoreButton2.units + '0', 1); // 15º posição (unidade placar B)
    lcdSendCommand(' ', 1);                      // 16º posição
    // Linha 2: MM:SS.CC X(anim)
    lcdSendCommand(0xC0, 0);                    // Início da linha 2
    lcdSendCommand(' ', 1);                     // 1º posição
    lcdSendCommand(' ', 1);                     // 2º posição
    lcdSendCommand(' ', 1);                     // 3º posição
    lcdSendCommand(' ', 1);                     // 4º posição
    lcdSendCommand(' ', 1);                     // 5º posição
    lcdSendCommand(timer.minute + '0', 1);      // 6º posição (minuto)
    lcdSendCommand(':', 1);                     // 7º posição
    lcdSendCommand(timer.secondTens + '0', 1);  // 8º posição (dezena segundo)
    lcdSendCommand(timer.secondUnits + '0', 1); // 9º posição (unidade segundo)
    lcdSendCommand('.', 1);                     // 10º posição
    lcdSendCommand(timer.msTens + '0', 1);      // 11º posição (dezena milissegundo)
    lcdSendCommand(timer.msUnits + '0', 1);     // 12º posição (unidade milissegundo)
    lcdSendCommand(' ', 1);                     // 13º posição
    lcdSendCommand(0, 1);                       // 14º posição Caractere animado (posição X)
    lcdSendCommand(' ', 1);                     // 15º posição
    lcdSendCommand(' ', 1);                     // 16º posição
}

void updateDisplay()
{
    // Atualiza frame da animação a cada 130ms
    if (millis() - lastAnimTime > 130)
    {
        lcdCreateChar(0, animFrames[animIndex]);
        animIndex = (animIndex + 1) % 3;
        lastAnimTime = millis();
    }
    displayLayout();
}

void loop()
{
    updateTimerValues();
    if (timer.value > 0)
    {
        handleButtonPress(&scoreButton1);
        handleButtonPress(&scoreButton2);
    }
    if (millis() % 27 == 0)
    {
        updateDisplay();
    }
}