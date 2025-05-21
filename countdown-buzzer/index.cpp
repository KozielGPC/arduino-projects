// --------------------------------------------------
// Definições dos pinos do hardware
// --------------------------------------------------
#define ENCODER_CLK_PIN PB0      // Pino do sinal CLK do encoder rotativo
#define ENCODER_DT_PIN PB1       // Pino do sinal DT do encoder rotativo
#define ENCODER_SW_PIN PB2       // Pino do botão do encoder rotativo
#define ENCODER_DOTS_PIN PB3     // Pino para acender os dois pontos do display
#define BUZZER_PIN PB4           // Pino do buzzer
#define DISPLAY_UNIT_SEC_PIN PC2 // Pino unidade de segundos
#define DISPLAY_TEN_SEC_PIN PC3  // Pino dezena de segundos
#define DISPLAY_UNIT_MIN_PIN PC4 // Pino unidade de minutos
#define DISPLAY_TEN_MIN_PIN PC5  // Pino dezena de minutos

// --------------------------------------------------
// Constantes de configuração do sistema
// --------------------------------------------------
#define DISPLAY_DIGIT_COUNT 4          // Quantidade de dígitos do display
#define DISPLAY_REFRESH_MS 5           // Tempo de atualização do display (multiplexação)
#define ENCODER_DEBOUNCE_MS 1          // Debounce do encoder
#define BUZZER_ON_MS 300               // Tempo que o buzzer fica ligado
#define DOTS_ON_MS 500                 // Tempo para alternar os dois pontos
#define DOTS_TOGGLE_MS 1000            // Tempo para alternar o estado dos dois pontos
#define FAST_ROTATION_THRESHOLD_MS 200 // Threshold para considerar giro rápido do encoder
#define FAST_ROTATION_INCREMENT 30     // Incremento para giro rápido do encoder

// Tabela dos segmentos para exibir números de 0 a F no display de 7 segmentos
const unsigned char SEGMENT_TABLE[] PROGMEM = {
    0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02,
    0x78, 0x00, 0x18, 0x08, 0x03, 0x46, 0x21, 0x06, 0x0E};

// --------------------------------------------------
// Estruturas de estado do sistema
// --------------------------------------------------
// Enum para modo de operação: configuração de segundos, minutos ou contagem regressiva
typedef enum
{
    CONFIGURE_SECONDS,
    CONFIGURE_MINUTES,
    RUNNING
} TimerMode;

// Estado de controle do sistema (flags, índices, temporizadores)
typedef struct
{
    unsigned long lastEncoderEventTime; // Último tempo de evento do encoder
    int lastEncoderClkState;            // Último estado do sinal CLK do encoder
    unsigned long lastDisplayUpdate;    // Última atualização do display
    char displayDigitIndex;             // Índice do dígito sendo exibido
    bool countdownStarted;              // Flag se a contagem regressiva está ativa
    bool buzzerActive;                  // Flag se o buzzer está ativo
    bool blinkState;                    // Estado do blink para o dígito configurado
    unsigned long lastBlinkTime;        // Último tempo de troca do blink
    TimerMode mode;                     // Modo atual do sistema
} ControlState;

// Estado dos valores de tempo (minutos, segundos, dígitos)
typedef struct
{
    unsigned long totalSeconds; // Total de segundos configurados
    int minutes;                // Minutos atuais
    int seconds;                // Segundos atuais
    char secondsUnit;           // Unidade de segundos
    char secondsTens;           // Dezena de segundos
    char minutesUnit;           // Unidade de minutos
    char minutesTens;           // Dezena de minutos
} TimeState;

ControlState control = {0};
TimeState timevals = {0};

// --------------------------------------------------
// Funções utilitárias para manipulação dos valores de tempo
// --------------------------------------------------
// Atualiza os campos de minutos, segundos e dígitos a partir do total de segundos
void updateTimeDigits(TimeState *t)
{
    t->minutes = t->totalSeconds / 60;
    t->seconds = t->totalSeconds % 60;
    t->secondsUnit = t->seconds % 10;
    t->secondsTens = (t->seconds / 10) % 10;
    t->minutesUnit = t->minutes % 10;
    t->minutesTens = (t->minutes / 10) % 10;
}

// Exibe um dígito específico no display de 7 segmentos
void displayDigit(char digit, char position)
{
    // Desliga todos os dígitos
    PORTC &= ~((1 << DISPLAY_UNIT_SEC_PIN) | (1 << DISPLAY_TEN_SEC_PIN) | (1 << DISPLAY_UNIT_MIN_PIN) | (1 << DISPLAY_TEN_MIN_PIN));
    // Define os segmentos para o valor desejado
    PORTD = pgm_read_byte(&SEGMENT_TABLE[digit]);
    // Liga o dígito correto
    PORTC |= (1 << position);
}

// Multiplexação do display: alterna entre os dígitos e faz o blink do dígito em configuração
void updateDisplayMultiplex(ControlState *c, TimeState *t)
{
    c->lastDisplayUpdate = millis();
    c->displayDigitIndex = (c->displayDigitIndex + 1) % DISPLAY_DIGIT_COUNT;
    bool showDigit = true;
    // Lógica de blink: pisca o dígito que está sendo configurado
    if (c->mode != RUNNING)
    {
        if (millis() - c->lastBlinkTime > 500)
        {
            c->blinkState = !c->blinkState;
            c->lastBlinkTime = millis();
        }
        // Pisca segundos (dígitos 0 e 3) ou minutos (dígitos 1 e 2) conforme modo
        if ((c->mode == CONFIGURE_SECONDS && (c->displayDigitIndex == 0 || c->displayDigitIndex == 3)) ||
            (c->mode == CONFIGURE_MINUTES && (c->displayDigitIndex == 1 || c->displayDigitIndex == 2)))
        {
            showDigit = c->blinkState;
        }
    }
    if (showDigit)
    {
        switch (c->displayDigitIndex)
        {
        case 0:
            displayDigit(t->secondsUnit, DISPLAY_UNIT_SEC_PIN);
            break;
        case 1:
            displayDigit(t->minutesTens, DISPLAY_TEN_SEC_PIN);
            break;
        case 2:
            displayDigit(t->minutesUnit, DISPLAY_UNIT_MIN_PIN);
            break;
        case 3:
            displayDigit(t->secondsTens, DISPLAY_TEN_MIN_PIN);
            break;
        }
    }
    else
    {
        // Desliga o dígito para efeito de pisca
        PORTC &= ~(1 << (DISPLAY_UNIT_SEC_PIN + c->displayDigitIndex));
    }
}

// --------------------------------------------------
// Funções de manipulação do encoder e botão
// --------------------------------------------------
// Lida com o giro do encoder para alterar segundos ou minutos
void handleEncoderRotation(ControlState *c, TimeState *t, char currentEncoderClkState, int currentEncoderDtState)
{
    unsigned long now = millis();
    static unsigned long lastRotationTime = 0;
    unsigned long delta = now - lastRotationTime;
    lastRotationTime = now;
    int increment = 1;
    if (delta < FAST_ROTATION_THRESHOLD_MS)
    {
        increment = FAST_ROTATION_INCREMENT;
    }
    if (currentEncoderClkState == 0)
    {
        if (currentEncoderDtState != 0)
        {
            t->totalSeconds += increment;
        }
        else
        {
            if (t->totalSeconds > 0)
            {
                if (t->totalSeconds > increment)
                {
                    t->totalSeconds -= increment;
                }
                else
                {
                    t->totalSeconds = 0;
                }
            }
        }
        updateTimeDigits(t);
    }
}

// Lida com o botão do encoder para alternar modos ou iniciar a contagem
void handleEncoderButton(ControlState *c, TimeState *t, char currentEncoderSwState)
{
    if (currentEncoderSwState == 0 && !c->countdownStarted)
    {
        if (t->totalSeconds != 0)
        {
            c->countdownStarted = true;
            c->lastEncoderEventTime = millis();
        }
    }
}

// --------------------------------------------------
// Funções de contagem regressiva e buzzer
// --------------------------------------------------
// Lida com a contagem regressiva, decrementando o tempo e ativando o buzzer ao final
void handleCountdown(ControlState *c, TimeState *t)
{
    if (t->totalSeconds == 0)
    {
        // Quando chega a zero, para a contagem e ativa o buzzer
        if (!c->buzzerActive && c->countdownStarted)
        {
            c->lastEncoderEventTime = millis();
            c->buzzerActive = true;
            c->countdownStarted = false; // Para o timer ao chegar em zero
            PORTB |= (1 << BUZZER_PIN);
        }
    }
    else if (c->countdownStarted)
    {
        // Decrementa o tempo a cada ciclo
        if (millis() > c->lastEncoderEventTime + DOTS_TOGGLE_MS)
        {
            PORTB |= (1 << ENCODER_DOTS_PIN); // Exibe os dois pontos
            c->lastEncoderEventTime = millis();
            if (t->totalSeconds > 0)
            {
                t->totalSeconds--;
                updateTimeDigits(t);
            }
        }
        else if (millis() > c->lastEncoderEventTime + DOTS_ON_MS)
        {
            PORTB ^= (1 << ENCODER_DOTS_PIN); // Alterna os dois pontos
        }
    }
}

// Desliga o buzzer após o tempo definido
void handleBuzzer(ControlState *c)
{
    if (c->buzzerActive && millis() > c->lastEncoderEventTime + BUZZER_ON_MS)
    {
        PORTB &= ~(1 << BUZZER_PIN);
        c->countdownStarted = false;
        c->buzzerActive = false;
    }
}

// Atualiza o display conforme o tempo de multiplexação
void handleDisplayUpdate(ControlState *c, TimeState *t)
{
    if (millis() - c->lastDisplayUpdate >= DISPLAY_REFRESH_MS)
    {
        updateDisplayMultiplex(c, t);
    }
}

// --------------------------------------------------
// Função de inicialização do sistema
// --------------------------------------------------
void setup()
{
    // Configura todos os pinos do display como saída
    DDRD = 0xFF;
    DDRC = 0xFF;
    // Pinos do encoder como entrada, pinos dos dois pontos e buzzer como saída
    DDRB = 0x00;
    DDRB |= (1 << ENCODER_DOTS_PIN) | (1 << BUZZER_PIN);
    PORTB = 0b00000111; // pull-up para o encoder
    control.lastEncoderClkState = (PINB >> ENCODER_CLK_PIN) & 1;
    control.mode = CONFIGURE_SECONDS; // Inicia configurando segundos
    // Inicializa o display com zero
    for (int i = 0; i < DISPLAY_DIGIT_COUNT; i++)
    {
        PORTD = pgm_read_byte(&SEGMENT_TABLE[0]);
        PORTC |= (1 << (DISPLAY_UNIT_SEC_PIN + i));
    }
    updateTimeDigits(&timevals);
}

// --------------------------------------------------
// Função principal de loop do sistema
// --------------------------------------------------
void loop()
{
    char currentEncoderClkState = (PINB >> ENCODER_CLK_PIN) & 1;
    char currentEncoderSwState = (PINB >> ENCODER_SW_PIN) & 1;
    int currentEncoderDtState = (PINB >> ENCODER_DT_PIN) & 1;
    static unsigned long buttonPressTime = 0;
    static bool buttonWasPressed = false;

    // O giro do encoder só altera o valor do modo selecionado (segundos ou minutos)
    if (currentEncoderClkState != control.lastEncoderClkState && millis() > control.lastEncoderEventTime + ENCODER_DEBOUNCE_MS)
    {
        if (control.mode == CONFIGURE_SECONDS)
        {
            handleEncoderRotation(&control, &timevals, currentEncoderClkState, currentEncoderDtState);
        }
        else if (control.mode == CONFIGURE_MINUTES)
        {
            // Gira minutos: adiciona/subtrai 60 segundos por vez
            unsigned long now = millis();
            static unsigned long lastRotationTime = 0;
            unsigned long delta = now - lastRotationTime;
            lastRotationTime = now;
            int increment = 60;
            if (delta < FAST_ROTATION_THRESHOLD_MS)
                increment = 60 * FAST_ROTATION_INCREMENT;
            if (currentEncoderClkState == 0)
            {
                if (currentEncoderDtState != 0)
                {
                    timevals.totalSeconds += increment;
                }
                else
                {
                    if (timevals.totalSeconds >= increment)
                        timevals.totalSeconds -= increment;
                    else
                        timevals.totalSeconds = 0;
                }
                updateTimeDigits(&timevals);
            }
        }
    }
    control.lastEncoderClkState = currentEncoderClkState;

    // Lógica do botão: alterna entre modos ou inicia a contagem
    if (currentEncoderSwState == 0 && !buttonWasPressed)
    {
        buttonPressTime = millis();
        buttonWasPressed = true;
    }
    if (currentEncoderSwState == 1 && buttonWasPressed)
    {
        unsigned long pressDuration = millis() - buttonPressTime;
        buttonWasPressed = false;
        if (pressDuration < 500) // Clique curto: alterna entre segundos/minutos
        {
            if (control.mode == CONFIGURE_SECONDS)
                control.mode = CONFIGURE_MINUTES;
            else if (control.mode == CONFIGURE_MINUTES)
                control.mode = CONFIGURE_SECONDS;
        }
        else // Clique longo: inicia a contagem regressiva
        {
            if (timevals.totalSeconds > 0)
            {
                control.countdownStarted = true;
                control.mode = RUNNING;
                control.lastEncoderEventTime = millis();
            }
        }
    }

    // Se a contagem está ativa, executa a lógica de contagem
    if (control.countdownStarted)
    {
        handleCountdown(&control, &timevals);
    }

    // Lida com o buzzer e atualização do display
    handleBuzzer(&control);
    handleDisplayUpdate(&control, &timevals);
}
