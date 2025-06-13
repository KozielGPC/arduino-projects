#define F_CPU 16000000UL // define a frequencia do microcontrolador - 16MHz

#include <avr/io.h>       //definições do componente especificado
#include <util/delay.h>   //biblioteca para o uso das rotinas de _delay_ms e _delay_us()
#include <avr/pgmspace.h> //para o uso do PROGMEM, gravação de dados na memória flash
#include <stdio.h>        // snprintf pra envia_millis
#include <string.h>

#define SERIAL_BAUD_RATE 2400 // taxa de 2400 bps
#define SERIAL_UBRR F_CPU / 16 / SERIAL_BAUD_RATE - 1

#define COMMAND_BUFFER_LENGTH 3 // buffer para comandos de 3 caracteres
#define LED1_PIN PB5
#define LED2_PIN PB4
#define LED_ON 1
#define LED_OFF 0
#define LED_TOGGLE 2
#define MILLIS_SEND_INTERVAL_50MS 50
#define MILLIS_SEND_INTERVAL_10MS 10
#define MILLIS_BASE 10000
#define MILLIS_MODULO 90000
#define SERIAL_MSG_MAXLEN 20

volatile char commandBuffer[COMMAND_BUFFER_LENGTH];
volatile short commandBufferPos = 0;
short commandBufferEnd = 0, commandBufferCount = 0;
volatile unsigned long systemMillis = 0;

void timer0_init()
{
    TCCR0A = (1 << WGM01);              // configura o timer0 no modo ctc (clear timer on compare match)
    TCCR0B = (1 << CS01) | (1 << CS00); // prescaler de 64
    OCR0A = 249;                        // valor para gerar interrupção a cada 1ms
    TIMSK0 |= (1 << OCIE0A);            // habilita interrupção do timer0
    sei();                              // habilita interrupções globais
}

ISR(TIMER0_COMPA_vect)
{
    systemMillis++; // incrementa o millis
}

unsigned long millis()
{
    return systemMillis; // retorna o millis atual
}

void serial_init(unsigned int ubrr0)
{
    UBRR0H = (unsigned char)(ubrr0 >> 8);
    UBRR0L = (unsigned char)ubrr0;
    UCSR0A = 0;                                           // Desabilitar velocidade dupla
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0); // Habilita TX e RX com interrupção
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);               // 8 bits, 1 stop, sem paridade
    sei();                                                // Habilita interrupções globais
}

void serial_transmit(char data)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;        // se o buffer está pronto pra enviar
    UDR0 = data; // envia o caractere
}

void serial_send_string(const char *str)
{
    while (*str)
    {                            // enquanto percorre a string
        serial_transmit(*str++); // envia caractere por caractere
    }
}

void send_millis_over_serial()
{
    char msg[SERIAL_MSG_MAXLEN];
    unsigned long time = MILLIS_BASE + (millis() % MILLIS_MODULO); // conta especificada na questão 4
    snprintf(msg, sizeof(msg), "%lu\n", time);                     // converte pra string
    serial_send_string(msg);                                       // printa
}

void set_led(uint8_t pin, uint8_t action)
{
    switch (action)
    {
    case LED_ON:
        PORTB |= (1 << pin);
        break;
    case LED_OFF:
        PORTB &= ~(1 << pin);
        break;
    case LED_TOGGLE:
        PORTB ^= (1 << pin);
        break;
    }
}

void handle_command(const char *cmd)
{
    if (strncmp(cmd, "A13", 3) == 0)
    {
        set_led(LED1_PIN, LED_ON);
        serial_send_string("LIGA PB5\n");
    }
    else if (strncmp(cmd, "S13", 3) == 0)
    {
        set_led(LED1_PIN, LED_OFF);
        serial_send_string("APAGA PB5\n");
    }
    else if (strncmp(cmd, "D12", 3) == 0)
    {
        set_led(LED2_PIN, LED_TOGGLE);
        serial_send_string("ALTERNA PB4\n");
    }
}

ISR(USART_RX_vect)
{
    commandBuffer[commandBufferPos] = UDR0; // le o caractere recebido e armazena no buffer
    commandBufferPos++;
    if (commandBufferPos == COMMAND_BUFFER_LENGTH)
    {
        commandBufferPos = 0;
        handle_command((const char *)commandBuffer);
    }
}

void leds_init()
{
    DDRB |= (1 << LED1_PIN) | (1 << LED2_PIN);     // pb5 e pb4 como saida
    PORTB &= ~((1 << LED1_PIN) | (1 << LED2_PIN)); // inicializa os leds desligados em low
}

void main_loop()
{
    unsigned long lastSend50ms = 0, lastSend10ms = 0;
    while (1)
    {
        if (millis() - lastSend50ms >= MILLIS_SEND_INTERVAL_50MS)
        {
            lastSend50ms = millis();
            send_millis_over_serial();
        }
        if (millis() - lastSend10ms >= MILLIS_SEND_INTERVAL_10MS)
        {
            lastSend10ms = millis();
            send_millis_over_serial();
        }
        // não precisa verificar se tem caractere pra parte dos leds porque ele armazena automatico pela interrupçao
    }
}

int main()
{
    timer0_init();
    serial_init(SERIAL_UBRR);
    leds_init();
    main_loop();
}
