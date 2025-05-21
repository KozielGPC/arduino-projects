unsigned char d = 0;
unsigned long lastDispRefresh = 0; // controle do tempo para refresh do display
unsigned char Tabela[] = {0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x80,
                          0x18, 0x06, 0x2F};
unsigned char todisp[3] = {10, 11, 11}; // E r r
volatile unsigned int leituraPot1 = 0;  // potenciômetro 1
volatile unsigned int leituraPot2 = 0;  // potenciômetro 2
volatile unsigned char canalAtual = 0;  // canal do ADC em uso (0 ou 1)
volatile bool novaLeitura = false;      // indica se tem nova leitura pronta
// char ch = 0;
// int sensorValue = 0;
void setup()
{
    // pinos como saída para os displays
    DDRD |= 0b01111111;
    DDRB |= 0b00000111;
    // pinos de entrada para os potenciômetros
    DDRC &= ~(1 << PC0);
    DDRC &= ~(1 << PC1);
    ADMUX &= ~((1 << REFS0) | (1 << REFS1));              // tensao de referencia
    ADMUX |= (1 << REFS0);                                // AVCC
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADCCLK = CLK/128
    ADCSRA |= (1 << ADIE);                                // habilita interrupção
    ADCSRA |= (1 << ADEN);                                // habilita o ADC
    ADCSRA |= (1 << ADSC);                                // inicia a conversão do ADC
    sei();                                                // habilita interrupções globais
}
void loop()
{
    if (novaLeitura)
    {
        novaLeitura = false; // reseta o indicador de nova leitura
        // mapeaia os valores dos potenciômetros para 0-100
        // VALORES LIDOS NA PARTE SERIAL REMOVIDA DO CÓDIGO VIDE AS RESTRIÇÕES DO PROJETO:
        // POTENCIÔMETRO 1: MIN 190, MAX 833
        // POTÊNCIOMETRO 2: MIN 87, MAX 413
        int valor1 = map(leituraPot1, 190, 833, 0, 100);
        int valor2 = map(leituraPot2, 87, 413, 0, 100);
        // saturação pra não ficar dificil de chegar no limite igual mostrado em aula
        valor1 = constrain(valor1, 0, 100); // 0-100
        valor2 = constrain(valor2, 0, 100); // 0-100
        // ver se está em 0 ou 1023, pois a leitura do pedal nunca da 0 ou 5
        if (leituraPot1 == 0 || leituraPot1 == 1023 || leituraPot2 == 0 || leituraPot2 == 1023 ||
            abs(valor1 - valor2) > 10)
        {
            todisp[0] = 10; // "E"
            todisp[1] = 11; // "r"
            todisp[2] = 11; // "r"
        }
        else
        { // se está dentro do limite, calcula a media e exibe
            int media = (valor1 + valor2) / 2;
            todisp[0] = media / 100;       // centena
            todisp[1] = (media / 10) % 10; // dezena
            todisp[2] = media % 10;        // unidade
        }
    }
    if (millis() > (lastDispRefresh + 0))
    {
        lastDispRefresh = millis();
        d++;
        d %= 3;
        PORTB = (PORTB & 0b11111000) | (1 << d); // ativa o display correspondente ao d
        PORTD = Tabela[todisp[d]];
    }
}
// busy waiting version
// signed int ler_adc(unsigned char canal)
//{
// ADMUX &= 0b11110000;
// ADMUX |= (0b00001111&canal); // seleciona o canal ch no MUX
// DIDR0 = (1<<canal);
// ADCSRA |= (1<<ADSC); //inicia a conversão
// while(ADCSRA & (1<<ADSC)); //espera a conversão ser finalizada
// return ADC;
//}
ISR(ADC_vect)
{ // interrupção do adc, pra nao ter a espera ociosa do ler_adc.
    // chamada automaticamente quando uma conversão do ADC termina
    if (canalAtual == 0)
    {                      // se o canal convertido foi o primeiro, logo, potenciometro 1
        leituraPot1 = ADC; // lê o potenciômetro 1 e passa o valor para leituraPot1
        canalAtual = 1;    // muda para o próximo canal
    }
    else
    {                       // se o canal convertido foi o segundo, logo, potenciometro 2
        leituraPot2 = ADC;  // lê o potenciômetro 2 e passa o valor para leituraPot2
        canalAtual = 0;     // volta para o primeiro canal
        novaLeitura = true; // indica que uma nova leitura foi concluida apos ler ambos
    }
    ADMUX = (ADMUX & 0xF0) | canalAtual; // alterna o canal do ADC pelo canalAtual
    // sem mexer nas referencias de tensão
    ADCSRA |= (1 << ADSC); // inicia a próxima conversão de ADMUX ativando o ADSC
}