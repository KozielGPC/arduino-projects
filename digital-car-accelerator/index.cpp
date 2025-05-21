// Display segment codes for 0-9, E, r
const unsigned char SEGMENT_CODES[] = {0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x80, 0x18, 0x06, 0x2F};

// Display buffer: 10 = 'E', 11 = 'r'
unsigned char displayBuffer[3] = {10, 11, 11};

volatile unsigned int potValue1 = 0;          // Potentiometer 1 value
volatile unsigned int potValue2 = 0;          // Potentiometer 2 value
volatile unsigned char currentAdcChannel = 0; // ADC channel in use (0 or 1)
volatile bool hasNewReading = false;          // Indicates if a new reading is ready

unsigned char displayIndex = 0;
unsigned long lastDisplayRefresh = 0;

void setup()
{
    // Set display pins as output
    DDRD |= 0b01111111;
    DDRB |= 0b00000111;
    // Set potentiometer pins as input
    DDRC &= ~(1 << PC0);
    DDRC &= ~(1 << PC1);
    // ADC setup
    ADMUX &= ~((1 << REFS0) | (1 << REFS1));              // Clear reference bits
    ADMUX |= (1 << REFS0);                                // Set AVCC as reference
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
    ADCSRA |= (1 << ADIE);                                // Enable ADC interrupt
    ADCSRA |= (1 << ADEN);                                // Enable ADC
    ADCSRA |= (1 << ADSC);                                // Start ADC conversion
    sei();                                                // Enable global interrupts
}

// Maps and constrains potentiometer readings to 0-100
int mapAndConstrain(int value, int min, int max)
{
    int mapped = map(value, min, max, 0, 100);
    return constrain(mapped, 0, 100);
}

void updateDisplayBuffer(int value1, int value2)
{
    // If readings are invalid or difference is too large, show error
    if (potValue1 == 0 || potValue1 == 1023 || potValue2 == 0 || potValue2 == 1023 || abs(value1 - value2) > 10)
    {
        displayBuffer[0] = 10; // 'E'
        displayBuffer[1] = 11; // 'r'
        displayBuffer[2] = 11; // 'r'
    }
    else
    {
        int average = (value1 + value2) / 2;
        displayBuffer[0] = average / 100;       // Hundreds
        displayBuffer[1] = (average / 10) % 10; // Tens
        displayBuffer[2] = average % 10;        // Units
    }
}

void refreshDisplay()
{
    displayIndex = (displayIndex + 1) % 3;
    PORTB = (PORTB & 0b11111000) | (1 << displayIndex); // Activate current display
    PORTD = SEGMENT_CODES[displayBuffer[displayIndex]];
}

void loop()
{
    if (hasNewReading)
    {
        hasNewReading = false;
        int value1 = mapAndConstrain(potValue1, 190, 833);
        int value2 = mapAndConstrain(potValue2, 87, 413);
        updateDisplayBuffer(value1, value2);
    }
    if (millis() > (lastDisplayRefresh + 0))
    {
        lastDisplayRefresh = millis();
        refreshDisplay();
    }
}

// ADC interrupt: reads both potentiometers without busy waiting
ISR(ADC_vect)
{
    if (currentAdcChannel == 0)
    {
        potValue1 = ADC;
        currentAdcChannel = 1;
    }
    else
    {
        potValue2 = ADC;
        currentAdcChannel = 0;
        hasNewReading = true;
    }
    ADMUX = (ADMUX & 0xF0) | currentAdcChannel; // Switch ADC channel
    ADCSRA |= (1 << ADSC);                      // Start next conversion
}