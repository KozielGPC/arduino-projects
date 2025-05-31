// --------------------------------------------------
// Placar Eletrônico com LCD, Animação e Cronômetro
// --------------------------------------------------

// Proteção contra inclusão múltipla (header guard)
#ifndef _DEF_PRINCIPAIS_H
#define _DEF_PRINCIPAIS_H

// Define a frequência do microcontrolador para uso com _delay_ms
#define F_CPU 16000000UL // 16 MHz (Arduino UNO)

// --------------------------------------------------
// Inclusão de bibliotecas padrão AVR
// --------------------------------------------------
#include <avr/io.h>       // Definições de registradores do AVR
#include <util/delay.h>   // Funções de delay (precisam de F_CPU)
#include <avr/pgmspace.h> // Para armazenar strings/constantes na memória flash

// --------------------------------------------------
// Definições de hardware e macros de manipulação de bits
// --------------------------------------------------
#define LCD_DATA_PORT PORTC    // Todos os dados do LCD ligados à PORTC (obrigatório)
#define LCD_CONTROL_PORT PORTC // Pinos de controle (Enable, RS) também na PORTC
#define LCD_ENABLE_PIN PC4     // Pino de Enable do LCD
#define LCD_RS_PIN PC5         // Pino de Register Select do LCD
#define LCD_USE_MSB 0          // 0: usa LSBs da porta, 1: usa MSBs

// Macros para manipulação de bits (facilitam leitura e escrita de pinos)
#define SET_BIT(y, bit) (y |= (1 << bit))
#define CLR_BIT(y, bit) (y &= ~(1 << bit))
#define CPL_BIT(y, bit) (y ^= (1 << bit))
#define TST_BIT(y, bit) (y & (1 << bit))

// Macro para gerar o pulso de Enable do LCD (necessário para cada comando/dado)
#define LCD_PULSE_ENABLE()                                          \
    _delay_us(1);                              /* Pequeno atraso */ \
    SET_BIT(LCD_CONTROL_PORT, LCD_ENABLE_PIN); /* Seta Enable */    \
    _delay_us(1);                              /* Pequeno atraso */ \
    CLR_BIT(LCD_CONTROL_PORT, LCD_ENABLE_PIN); /* Limpa Enable */   \
    _delay_us(45)                              /* Aguarda LCD processar */

// --------------------------------------------------
// Protótipos das funções de LCD (comunicação em 4 bits)
// --------------------------------------------------
void lcdSendCommand(unsigned char value, char isData);
void lcdInit4Bit();

#define DEBOUNCE_INTERVAL 1 // Macro para debounce (ms)

#endif

// --------------------------------------------------
// Estruturas para armazenar estado dos placares e cronômetro
// --------------------------------------------------
typedef struct ScoreButton
{
    unsigned char value;       // Valor do placar (0-99)
    char units;                // Unidade do placar
    char tens;                 // Dezena do placar
    int lastButtonState;       // Último estado lido do botão
    int buttonState;           // Estado atual do botão
    unsigned int debounceTime; // Tempo para debounce
    int pin;                   // Pino do botão
} ScoreButton;

typedef struct Timer
{
    int minute;      // Minuto atual
    int secondTens;  // Dezena dos segundos
    int secondUnits; // Unidade dos segundos
    int msTens;      // Dezena dos centésimos
    int msUnits;     // Unidade dos centésimos
    long value;      // Valor total do cronômetro em ms
    long start;      // Valor inicial do cronômetro em ms
} Timer;

// Protótipos das funções de lógica do placar e cronômetro
void updateScore(ScoreButton *scoreBtn);
void handleButtonPress(ScoreButton *scoreBtn);

// --------------------------------------------------
// Variáveis globais de estado do sistema
// --------------------------------------------------
ScoreButton scoreButton1 = {0, 0, 0, 1, 1, 0, PD6};    // Placar/botão do time A (pino PD6)
ScoreButton scoreButton2 = {0, 0, 0, 1, 1, 0, PD7};    // Placar/botão do time B (pino PD7)
Timer timer = {0, 0, 0, 0, 0, 0, 599000};              // Cronômetro regressivo (9:59.00 max)
const unsigned char lcdTitle[] PROGMEM = "Cronometro"; // Título exibido no LCD

// --------------------------------------------------
// Frames de animação: coração, smiley e estrela
// --------------------------------------------------
// Cada frame é um caractere customizado (8 bytes, 5x8 pontos)
const uint8_t animFrames[3][8] = {
    // Frame 1: Coração
    {0b00000,
     0b01010,
     0b11111,
     0b11111,
     0b11111,
     0b01110,
     0b00100,
     0b00000},
    // Frame 2: Smiley
    {0b00000,
     0b01010,
     0b01010,
     0b00000,
     0b10001,
     0b01110,
     0b00000,
     0b00000},
    // Frame 3: Estrela
    {0b00100,
     0b10101,
     0b01110,
     0b11111,
     0b01110,
     0b10101,
     0b00100,
     0b00000}};

// --------------------------------------------------
// Variáveis para controle da animação
// --------------------------------------------------
uint8_t animIndex = 0;          // Índice do frame atual da animação
unsigned long lastAnimTime = 0; // Último tempo em que o frame foi trocado

// --------------------------------------------------
// Funções utilitárias do LCD
// --------------------------------------------------
// Envia comando ou caractere para o LCD (modo 4 bits)
void lcdSendCommand(unsigned char value, char isData)
{
    // Define se é comando (RS=0) ou caractere (RS=1)
    if (isData == 0)
        CLR_BIT(LCD_CONTROL_PORT, LCD_RS_PIN);
    else
        SET_BIT(LCD_CONTROL_PORT, LCD_RS_PIN);

    // Envia primeiro nibble (4 bits mais significativos)
#if (LCD_USE_MSB)
    LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (0xF0 & value); // Usa MSBs
#else
    LCD_DATA_PORT = (LCD_DATA_PORT & 0xF0) | (value >> 4); // Usa LSBs
#endif
    LCD_PULSE_ENABLE();
    // Envia segundo nibble (4 bits menos significativos)
#if (LCD_USE_MSB)
    LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (0xF0 & (value << 4));
#else
    LCD_DATA_PORT = (LCD_DATA_PORT & 0xF0) | (0x0F & value);
#endif
    LCD_PULSE_ENABLE();
    // Se for comando de retorno/limpeza, aguarda mais tempo
    if ((isData == 0) && (value < 4))
        _delay_ms(2);
}

// Inicializa o LCD em modo 4 bits, conforme datasheet do HD44780
void lcdInit4Bit()
{
    CLR_BIT(LCD_CONTROL_PORT, LCD_RS_PIN);     // RS=0
    CLR_BIT(LCD_CONTROL_PORT, LCD_ENABLE_PIN); // Enable=0
    _delay_ms(20);                             // Aguarda estabilização do LCD
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
    lcdSendCommand(0x28, 0); // 4 bits, 2 linhas
    lcdSendCommand(0x08, 0); // Display off
    lcdSendCommand(0x01, 0); // Limpa display
    lcdSendCommand(0x0C, 0); // Display on, cursor off
    lcdSendCommand(0x80, 0); // Cursor início
}

// Cria um caractere customizado na CGRAM do LCD
void lcdCreateChar(uint8_t location, const uint8_t *charmap)
{
    lcdSendCommand(0x40 | ((location & 0x7) << 3), 0); // Endereço CGRAM
    for (uint8_t i = 0; i < 8; i++)
    {
        lcdSendCommand(charmap[i], 1); // Envia cada linha do caractere
    }
}

// --------------------------------------------------
// Funções de lógica do placar e cronômetro
// --------------------------------------------------
// Atualiza os campos do cronômetro a partir do tempo restante (em ms)
void updateTimerValues()
{
    timer.value = timer.start - millis();             // Calcula tempo restante
    timer.minute = timer.value / 60000;               // Minutos
    timer.secondTens = (timer.value % 60000) / 10000; // Dezena dos segundos
    timer.secondUnits = (timer.value % 10000) / 1000; // Unidade dos segundos
    timer.msTens = (timer.value % 1000) / 100;        // Dezena dos centésimos
    timer.msUnits = (timer.value % 100) / 10;         // Unidade dos centésimos
}

// Atualiza o valor do placar (0-99) e separa em dezena/unidade
void updateScore(ScoreButton *scoreBtn)
{
    if (scoreBtn->value == 99)
        scoreBtn->value = 0; // Volta para 0 se chegar a 99
    else
        scoreBtn->value++;
    scoreBtn->units = scoreBtn->value % 10;
    scoreBtn->tens = scoreBtn->value / 10;
}

// Lida com o pressionamento do botão e faz debounce
void handleButtonPress(ScoreButton *scoreBtn)
{
    int readButton = PIND & (1 << scoreBtn->pin); // Lê estado do botão
    if (readButton != scoreBtn->lastButtonState)
        scoreBtn->debounceTime = millis(); // Atualiza tempo de debounce
    if ((millis() - scoreBtn->debounceTime) > DEBOUNCE_INTERVAL)
    {
        if (scoreBtn->buttonState != readButton)
        {
            scoreBtn->buttonState = readButton;
            if (scoreBtn->buttonState == 0) // Se pressionado
                updateScore(scoreBtn);      // Incrementa placar
        }
    }
    scoreBtn->lastButtonState = readButton;
}

// --------------------------------------------------
// Função para exibir o layout do display LCD
// --------------------------------------------------
void displayLayout()
{
    // Linha 1: __A: 00   __   __   __   __   B:00
    lcdSendCommand(0x80, 0);                     // Início da linha 1
    lcdSendCommand(' ', 1);                      // 1º posição (padding)
    lcdSendCommand(' ', 1);                      // 2º posição
    lcdSendCommand('A', 1);                      // 3º posição: 'A' (Time A)
    lcdSendCommand(':', 1);                      // 4º posição: ':'
    lcdSendCommand(' ', 1);                      // 5º posição
    lcdSendCommand(scoreButton1.tens + '0', 1);  // 6º posição (dezena placar A)
    lcdSendCommand(scoreButton1.units + '0', 1); // 7º posição (unidade placar A)
    lcdSendCommand(' ', 1);                      // 8º posição
    lcdSendCommand(' ', 1);                      // 9º posição
    lcdSendCommand(' ', 1);                      // 10º posição
    lcdSendCommand(' ', 1);                      // 11º posição
    lcdSendCommand('B', 1);                      // 12º posição: 'B' (Time B)
    lcdSendCommand(':', 1);                      // 13º posição: ':'
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
    lcdSendCommand(timer.msTens + '0', 1);      // 11º posição (dezena centésimo)
    lcdSendCommand(timer.msUnits + '0', 1);     // 12º posição (unidade centésimo)
    lcdSendCommand(' ', 1);                     // 13º posição
    lcdSendCommand(0, 1);                       // 14º posição (caractere animado)
    lcdSendCommand(' ', 1);                     // 15º posição
    lcdSendCommand(' ', 1);                     // 16º posição
}

// --------------------------------------------------
// Função que atualiza o display e animação
// --------------------------------------------------
void updateDisplay()
{
    // Troca o frame da animação a cada 130ms
    if (millis() - lastAnimTime > 130)
    {
        lcdCreateChar(0, animFrames[animIndex]); // Atualiza caractere animado
        animIndex = (animIndex + 1) % 3;         // Próximo frame
        lastAnimTime = millis();
    }
    displayLayout(); // Atualiza layout do LCD
}

// --------------------------------------------------
// Setup do sistema: inicializa hardware e LCD
// --------------------------------------------------
void setup()
{
    initializeHardware();            // Configura portas e LCD
    initializeDisplay();             // Exibe título inicial
    lcdCreateChar(0, animFrames[0]); // Inicializa primeiro frame da animação
}

// Inicializa hardware (portas, pull-ups, LCD)
void initializeHardware()
{
    DDRC = 0xFF;        // PORTC como saída (dados/controle LCD)
    DDRD = 0b00111111;  // PORTD: pinos 0-5 saída, 6-7 entrada (botões)
    PORTD = 0b11000000; // Pull-up nos botões (pinos 6 e 7)
    lcdInit4Bit();      // Inicializa LCD em 4 bits
    if (timer.start > 599000)
        timer.start = 599000; // Limita valor inicial do cronômetro
}

// Exibe título e times na inicialização
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

// --------------------------------------------------
// Loop principal do sistema
// --------------------------------------------------
void loop()
{
    updateTimerValues(); // Atualiza cronômetro
    if (timer.value > 0)
    {
        handleButtonPress(&scoreButton1); // Lida com botão/placar A
        handleButtonPress(&scoreButton2); // Lida com botão/placar B
    }
    if (millis() % 27 == 0)
    {
        updateDisplay(); // Atualiza display e animação
    }
}