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
unsigned int debounceTime1 = 0;
unsigned int debounceTime2 = 0;
int lastButtonState1 = 1;
int lastButtonState2 = 1;
int button1State = 1;
int button2State = 1;

char score1Units = 0, score1Tens = 0;
char score2Units = 0, score2Tens = 0;
unsigned char score1 = 0;
unsigned char score2 = 0;

// Timer variables
int minute, secondTens, secondUnits, msTens, msUnits;
long timerValue, startTime = 599000;

const unsigned char lcdTitle[] PROGMEM = "Cronometro";

void initializeHardware();
void initializeDisplay();
void updateTimerValues();
void handleButtonPress(unsigned char *score, char *units, char *tens, int *lastButtonState, int *buttonState, unsigned int *debounceTime, int pin);
void updateDisplay();
void updateScore(unsigned char *score, char *units, char *tens);

void setup()
{
    initializeHardware();
    initializeDisplay();
}

void initializeHardware()
{
    DDRC = 0xFF;
    DDRD = 0b00111111;
    PORTD = 0b11000000;
    lcdInit4Bit();
    if (startTime > 599000)
        startTime = 599000;
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
    timerValue = startTime - millis();
    minute = timerValue / 60000;
    secondTens = (timerValue % 60000) / 10000;
    secondUnits = (timerValue % 10000) / 1000;
    msTens = (timerValue % 1000) / 100;
    msUnits = (timerValue % 100) / 10;
}

void handleButtonPress(unsigned char *score, char *units, char *tens, int *lastButtonState, int *buttonState, unsigned int *debounceTime, int pin)
{
    int readButton = PIND & (1 << pin);
    if (readButton != *lastButtonState)
        *debounceTime = millis();
    if ((millis() - *debounceTime) > DEBOUNCE_INTERVAL)
    {
        if (*buttonState != readButton)
        {
            *buttonState = readButton;
            if (*buttonState == 0)
                updateScore(score, units, tens);
        }
    }
    *lastButtonState = readButton;
}

void updateScore(unsigned char *score, char *units, char *tens)
{
    if (*score == 99)
    {
        *score = 0;
    }
    else
    {
        (*score)++;
    }
    *units = *score % 10;
    *tens = *score / 10;
}

void updateDisplay()
{
    lcdSendCommand(0xC4, 0);
    lcdSendCommand('0', 1);
    lcdSendCommand(minute + '0', 1);
    lcdSendCommand(':', 1);
    lcdSendCommand(secondTens + '0', 1);
    lcdSendCommand(secondUnits + '0', 1);
    lcdSendCommand(':', 1);
    lcdSendCommand(msTens + '0', 1);
    lcdSendCommand(msUnits + '0', 1);
    lcdSendCommand(0xC0, 0);
    lcdSendCommand(score1Tens + '0', 1);
    lcdSendCommand(score1Units + '0', 1);
    lcdSendCommand(0xCE, 0);
    lcdSendCommand(score2Tens + '0', 1);
    lcdSendCommand(score2Units + '0', 1);
}

void loop()
{
    updateTimerValues();
    if (timerValue > 0)
    {
        handleButtonPress(&score1, &score1Units, &score1Tens, &lastButtonState1, &button1State, &debounceTime1, PD6);
        handleButtonPress(&score2, &score2Units, &score2Tens, &lastButtonState2, &button2State, &debounceTime2, PD7);
    }
    if (millis() % 27 == 0)
    {
        updateDisplay();
    }
}