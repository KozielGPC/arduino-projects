// --------------------------------------------------
// Definições dos pinos e parâmetros do hardware
// --------------------------------------------------
#define F_CPU 16000000UL // Frequência do microcontrolador (16 MHz)

#include <avr/io.h>       // Registradores e funções de I/O do AVR
#include <util/delay.h>   // Funções de delay (_delay_ms, _delay_us)
#include <avr/pgmspace.h> // Manipulação de memória flash (PROGMEM)
#include <stdio.h>        // snprintf para formatação de strings
#include <string.h>       // Funções de manipulação de strings

// --------------------------------------------------
// Parâmetros de comunicação serial e controle de LEDs
// --------------------------------------------------
#define SERIAL_BAUD_RATE 2400                           // Baudrate da serial
#define SERIAL_UBRR (F_CPU / 16 / SERIAL_BAUD_RATE - 1) // Valor do registrador UBRR
#define COMMAND_BUFFER_LENGTH 3                         // Tamanho do buffer de comandos recebidos
#define LED1_PIN PB5                                    // Pino do LED1
#define LED2_PIN PB4                                    // Pino do LED2
#define LED_ON 1                                        // Ação: ligar LED
#define LED_OFF 0                                       // Ação: desligar LED
#define LED_TOGGLE 2                                    // Ação: alternar estado do LED
#define MILLIS_SEND_INTERVAL_50MS 50                    // Intervalo de envio serial: 50ms
#define MILLIS_SEND_INTERVAL_10MS 10                    // Intervalo de envio serial: 10ms
#define MILLIS_BASE 10000                               // Base para cálculo do tempo enviado
#define MILLIS_MODULO 90000                             // Módulo para cálculo do tempo enviado
#define SERIAL_MSG_MAXLEN 20                            // Tamanho máximo da mensagem serial

// --------------------------------------------------
// Variáveis globais e buffers
// --------------------------------------------------
volatile char commandBuffer[COMMAND_BUFFER_LENGTH]; // Buffer para comandos recebidos
volatile short commandBufferPos = 0;                // Posição atual no buffer de comandos
short commandBufferEnd = 0, commandBufferCount = 0; // Variáveis auxiliares (não usadas)
volatile unsigned long systemMillis = 0;            // Contador global de milissegundos

typedef unsigned char uint8_t;

// --------------------------------------------------
// Inicialização do Timer0 para gerar interrupção a cada 1ms
// --------------------------------------------------
void timer0_init()
{
    TCCR0A = (1 << WGM01);              // Modo CTC (Clear Timer on Compare Match)
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler de 64
    OCR0A = 249;                        // Valor para gerar 1ms (com F_CPU=16MHz)
    TIMSK0 |= (1 << OCIE0A);            // Habilita interrupção do timer0
    sei();                              // Habilita interrupções globais
}

// --------------------------------------------------
// Rotina de interrupção do Timer0: incrementa o contador de milissegundos
// --------------------------------------------------
ISR(TIMER0_COMPA_vect)
{
    systemMillis++; // Incrementa o millis a cada interrupção (1ms)
}

// --------------------------------------------------
// Função para obter o valor atual de millis (tempo em ms desde o início)
// --------------------------------------------------
unsigned long millis()
{
    return systemMillis;
}

// --------------------------------------------------
// Inicialização da comunicação serial
// --------------------------------------------------
void serial_init(unsigned int ubrr0)
{
    UBRR0H = (unsigned char)(ubrr0 >> 8);                 // Parte alta do registrador UBRR
    UBRR0L = (unsigned char)ubrr0;                        // Parte baixa do registrador UBRR
    UCSR0A = 0;                                           // Desabilita velocidade dupla
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0); // Habilita RX, TX e interrupção de RX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);               // 8 bits, 1 stop, sem paridade
    sei();                                                // Habilita interrupções globais
}

// --------------------------------------------------
// Transmissão de um caractere pela serial
// --------------------------------------------------
void serial_transmit(char data)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;        // Espera até o buffer estar pronto
    UDR0 = data; // Envia o caractere
}

// --------------------------------------------------
// Transmissão de uma string pela serial
// --------------------------------------------------
void serial_send_string(const char *str)
{
    while (*str)
    {
        serial_transmit(*str++); // Envia caractere por caractere
    }
}

// --------------------------------------------------
// Envia o valor de millis pela serial, formatado conforme especificação
// --------------------------------------------------
void send_millis_over_serial()
{
    char msg[SERIAL_MSG_MAXLEN];
    unsigned long time = MILLIS_BASE + (millis() % MILLIS_MODULO); // Cálculo do tempo a ser enviado
    snprintf(msg, sizeof(msg), "%lu\n", time);                     // Converte para string
    serial_send_string(msg);                                       // Envia pela serial
}

// --------------------------------------------------
// Função para manipular o estado dos LEDs
// --------------------------------------------------
void set_led(uint8_t pin, uint8_t action)
{
    switch (action)
    {
    case LED_ON:
        PORTB |= (1 << pin); // Liga o LED (nível alto)
        break;
    case LED_OFF:
        PORTB &= ~(1 << pin); // Desliga o LED (nível baixo)
        break;
    case LED_TOGGLE:
        PORTB ^= (1 << pin); // Alterna o estado do LED
        break;
    }
}

// --------------------------------------------------
// Interpreta e executa comandos recebidos pela serial
// --------------------------------------------------
void handle_command(const char *cmd)
{
    if (strncmp(cmd, "A13", 3) == 0)
    {
        set_led(LED1_PIN, LED_ON);                                 // Liga LED1
        serial_send_string("====== Ligando PB5 - LED 1 ======\n"); // Confirmação
    }
    else if (strncmp(cmd, "S13", 3) == 0)
    {
        set_led(LED1_PIN, LED_OFF);                                   // Desliga LED1
        serial_send_string("====== Desligando PB5 - LED 1 ======\n"); // Confirmação
    }
    else if (strncmp(cmd, "D12", 3) == 0)
    {
        set_led(LED2_PIN, LED_TOGGLE);                                // Alterna LED2
        serial_send_string("====== Alternando PB4 - LED 2 ======\n"); // Confirmação
    }
}

// --------------------------------------------------
// Rotina de interrupção da serial: armazena caracteres recebidos no buffer
// --------------------------------------------------
ISR(USART_RX_vect)
{
    commandBuffer[commandBufferPos] = UDR0; // Lê caractere recebido
    commandBufferPos++;
    if (commandBufferPos == COMMAND_BUFFER_LENGTH)
    {
        commandBufferPos = 0;                        // Reinicia posição do buffer
        handle_command((const char *)commandBuffer); // Processa comando completo
    }
}

// --------------------------------------------------
// Inicializa os pinos dos LEDs como saída e os desliga
// --------------------------------------------------
void leds_init()
{
    DDRB |= (1 << LED1_PIN) | (1 << LED2_PIN);     // Define PB5 e PB4 como saída
    PORTB &= ~((1 << LED1_PIN) | (1 << LED2_PIN)); // Inicializa LEDs desligados
}

// --------------------------------------------------
// Loop principal: envia periodicamente o valor de millis pela serial
// --------------------------------------------------
void main_loop()
{
    unsigned long lastSend50ms = 0, lastSend10ms = 0; // Último envio para cada intervalo
    while (1)
    {
        // Envia millis a cada 50ms
        if (millis() - lastSend50ms >= MILLIS_SEND_INTERVAL_50MS)
        {
            lastSend50ms = millis();
            send_millis_over_serial();
        }
        // Envia millis a cada 10ms
        if (millis() - lastSend10ms >= MILLIS_SEND_INTERVAL_10MS)
        {
            lastSend10ms = millis();
            send_millis_over_serial();
        }
        // Não é necessário verificar comandos dos LEDs aqui, pois são tratados por interrupção
    }
}

// --------------------------------------------------
// Função principal: inicializa periféricos e entra no loop principal
// --------------------------------------------------
int main()
{
    timer0_init();            // Inicializa timer0 para millis
    serial_init(SERIAL_UBRR); // Inicializa comunicação serial
    leds_init();              // Inicializa LEDs
    main_loop();              // Entra no loop principal
}
