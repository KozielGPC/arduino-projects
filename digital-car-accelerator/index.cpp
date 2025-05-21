// --------------------------------------------------
// Definições dos pinos do hardware
// --------------------------------------------------
#define DISPLAY_0_PIN PB0 // Display 0 (centenas)
#define DISPLAY_1_PIN PB1 // Display 1 (dezenas)
#define DISPLAY_2_PIN PB2 // Display 2 (unidades)
#define POT1_PIN PC0      // Potenciômetro 1
#define POT2_PIN PC1      // Potenciômetro 2

// --------------------------------------------------
// Constantes de configuração do sistema
// --------------------------------------------------
#define DISPLAY_DIGIT_COUNT 3
#define DISPLAY_REFRESH_MS 2
#define POT1_MIN 190
#define POT1_MAX 833
#define POT2_MIN 87
#define POT2_MAX 413
#define MAX_DIFF 10

// --------------------------------------------------
// Tabela dos segmentos para display de 7 segmentos (0-9, E, r)
// --------------------------------------------------
const unsigned char SEGMENT_TABLE[] = {0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x80, 0x18, 0x06, 0x2F};

// --------------------------------------------------
// Estruturas de estado
// --------------------------------------------------
typedef struct
{
    unsigned int pot1Value;   // Valor do potenciômetro 1
    unsigned int pot2Value;   // Valor do potenciômetro 2
    unsigned char adcChannel; // Canal atual do ADC
    bool newReading;          // Flag de nova leitura do ADC
} SensorState;

typedef struct
{
    unsigned char digits[DISPLAY_DIGIT_COUNT]; // Buffer dos dígitos do display
    unsigned char currentDigit;                // Índice do dígito atual para multiplexação
    unsigned long lastRefresh;                 // Último tempo de atualização do display
} DisplayState;

SensorState sensors = {0};
DisplayState display = {{10, 11, 11}, 0, 0}; // Padrão para 'Err'

// --------------------------------------------------
// Funções utilitárias
// --------------------------------------------------
int mapAndConstrain(int value, int min, int max)
{
    int mapped = map(value, min, max, 0, 100);
    return constrain(mapped, 0, 100);
}

void updateDisplayDigits(int value1, int value2)
{
    // Se leituras inválidas ou diferença muito grande, mostra erro
    if (sensors.pot1Value == 0 || sensors.pot1Value == 1023 ||
        sensors.pot2Value == 0 || sensors.pot2Value == 1023 ||
        abs(value1 - value2) > MAX_DIFF)
    {
        display.digits[0] = 10; // 'E'
        display.digits[1] = 11; // 'r'
        display.digits[2] = 11; // 'r'
    }
    else
    {
        int avg = (value1 + value2) / 2;
        display.digits[0] = avg / 100;
        display.digits[1] = (avg / 10) % 10;
        display.digits[2] = avg % 10;
    }
}

void displayDigit(unsigned char digit, unsigned char position)
{
    // Define o valor dos segmentos
    PORTD = SEGMENT_TABLE[digit];
    // Ativa apenas o display selecionado
    PORTB = (PORTB & 0b11111000) | (1 << position);
}

void updateDisplayMultiplex()
{
    display.lastRefresh = millis();
    display.currentDigit = (display.currentDigit + 1) % DISPLAY_DIGIT_COUNT;
    displayDigit(display.digits[display.currentDigit], display.currentDigit);
}

// --------------------------------------------------
// Setup do sistema
// --------------------------------------------------
void setup()
{
    // Define pinos do display como saída
    DDRD |= 0b01111111;
    DDRB |= 0b00000111;
    // Define pinos dos potenciômetros como entrada
    DDRC &= ~((1 << POT1_PIN) | (1 << POT2_PIN));
    // Configuração do ADC
    ADMUX &= ~((1 << REFS0) | (1 << REFS1));
    ADMUX |= (1 << REFS0);
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADIE);
    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);
    sei();
}

// --------------------------------------------------
// Loop principal
// --------------------------------------------------
void loop()
{
    if (sensors.newReading)
    {
        sensors.newReading = false;
        int value1 = mapAndConstrain(sensors.pot1Value, POT1_MIN, POT1_MAX);
        int value2 = mapAndConstrain(sensors.pot2Value, POT2_MIN, POT2_MAX);
        updateDisplayDigits(value1, value2);
    }
    if (millis() - display.lastRefresh >= DISPLAY_REFRESH_MS)
    {
        updateDisplayMultiplex();
    }
}

// --------------------------------------------------
// Rotina de Interrupção do ADC
// --------------------------------------------------
ISR(ADC_vect)
{
    if (sensors.adcChannel == 0)
    {
        sensors.pot1Value = ADC;
        sensors.adcChannel = 1;
    }
    else
    {
        sensors.pot2Value = ADC;
        sensors.adcChannel = 0;
        sensors.newReading = true;
    }
    ADMUX = (ADMUX & 0xF0) | sensors.adcChannel;
    ADCSRA |= (1 << ADSC);
}