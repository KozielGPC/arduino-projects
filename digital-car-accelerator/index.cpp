// --------------------------------------------------
// Hardware Pin Definitions
// --------------------------------------------------
#define DISPLAY_0_PIN PB0 // Display 0 (hundreds)
#define DISPLAY_1_PIN PB1 // Display 1 (tens)
#define DISPLAY_2_PIN PB2 // Display 2 (units)
#define POT1_PIN PC0      // Potentiometer 1
#define POT2_PIN PC1      // Potentiometer 2

// --------------------------------------------------
// System Configuration Constants
// --------------------------------------------------
#define DISPLAY_DIGIT_COUNT 3
#define DISPLAY_REFRESH_MS 2
#define POT1_MIN 190
#define POT1_MAX 833
#define POT2_MIN 87
#define POT2_MAX 413
#define MAX_DIFF 10

// --------------------------------------------------
// Segment Table for 7-segment Display (0-9, E, r)
// --------------------------------------------------
const unsigned char SEGMENT_TABLE[] = {0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x80, 0x18, 0x06, 0x2F};

// --------------------------------------------------
// State Structures
// --------------------------------------------------
typedef struct
{
    unsigned int pot1Value;   // Potentiometer 1 value
    unsigned int pot2Value;   // Potentiometer 2 value
    unsigned char adcChannel; // Current ADC channel
    bool newReading;          // New ADC reading flag
} SensorState;

typedef struct
{
    unsigned char digits[DISPLAY_DIGIT_COUNT]; // Display buffer
    unsigned char currentDigit;                // Current digit index for multiplexing
    unsigned long lastRefresh;                 // Last display refresh time
} DisplayState;

SensorState sensors = {0};
DisplayState display = {{10, 11, 11}, 0, 0}; // Default to 'Err'

// --------------------------------------------------
// Utility Functions
// --------------------------------------------------
int mapAndConstrain(int value, int min, int max)
{
    int mapped = map(value, min, max, 0, 100);
    return constrain(mapped, 0, 100);
}

void updateDisplayDigits(int value1, int value2)
{
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
    // Set segment value
    PORTD = SEGMENT_TABLE[digit];
    // Activate only the selected display
    PORTB = (PORTB & 0b11111000) | (1 << position);
}

void updateDisplayMultiplex()
{
    display.lastRefresh = millis();
    display.currentDigit = (display.currentDigit + 1) % DISPLAY_DIGIT_COUNT;
    displayDigit(display.digits[display.currentDigit], display.currentDigit);
}

// --------------------------------------------------
// Arduino Setup
// --------------------------------------------------
void setup()
{
    // Set display pins as output
    DDRD |= 0b01111111;
    DDRB |= 0b00000111;
    // Set potentiometer pins as input
    DDRC &= ~((1 << POT1_PIN) | (1 << POT2_PIN));
    // ADC setup
    ADMUX &= ~((1 << REFS0) | (1 << REFS1));
    ADMUX |= (1 << REFS0);
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADIE);
    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);
    sei();
}

// --------------------------------------------------
// Arduino Main Loop
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
// ADC Interrupt Service Routine
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