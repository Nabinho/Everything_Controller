/*******************************************************************************
 * Everything Controller - Codigo Base para o Controle (v1.0)
 *
 * Codigo base do controle "Everything Controller" para envio de dados para um
 * receptor. Comunicacao baseada em RF24 atraves do modulo NRF24L01+.
 * O controle envia dados de 6 botoes digitais, 2 joysticks (2 eixos X e 2 eixos Y),
 * assim como a leitura analogica de dois sliders.
 * Com esses dados e possivel o controle de diversos robos.
 *
 * Escrito por Giovanni de Castro (07/01/2023).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version (<https://www.gnu.org/licenses/>).
 *******************************************************************************/

// Bibliotecas
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------------------------------------------------------------------------------
// Pinos de controle do modulo de radio
const uint8_t PINO_CE = 9;
const uint8_t PINO_CSN = 10;

// Variaveis de controle do display OLED
#define ENDERECO_I2C 0x3C
#define LARGURA_DISPLAY 128
#define ALTURA_DISPLAY 64
#define RESET_DISPLAY -1

// Pinos de controle dos LEDs
const uint8_t LED_STATUS = 8;
const uint8_t LED_L = 13;

// Pinos de entradas analogicas e digitais

// Endereco do controle
const uint8_t ENDERECO_RADIO[][6] = {"Ctrlr", "Robot"};

// Numero do radio
const uint8_t NUMERO_RADIO = 0;

// Criacao do objeto de controle do modulo
RF24 radio(PINO_CE, PINO_CSN);

// Objeto de controle do display
Adafruit_SSD1306 display(LARGURA_DISPLAY, ALTURA_DISPLAY, &Wire, RESET_DISPLAY);

// Variaveis para a leitura do botao
const uint8_t PINO_BOTAO1 = 2;
bool leitura_botao1;
bool estado_saida1 = LOW;
bool estado_botao1;
bool ultimo_estado_botao1 = HIGH;
unsigned long ultimo_tempo_debounce1 = 0;

// Variaveis para a leitura do botao
const uint8_t PINO_BOTAO2 = 3;
bool leitura_botao2;
bool estado_saida2 = LOW;
bool estado_botao2;
bool ultimo_estado_botao2 = HIGH;
unsigned long ultimo_tempo_debounce2 = 0;

// Variaveis para a leitura do botao
const uint8_t PINO_BOTAO3 = 4;
bool leitura_botao3;
bool estado_saida3 = LOW;
bool estado_botao3;
bool ultimo_estado_botao3 = HIGH;
unsigned long ultimo_tempo_debounce3 = 0;

// Variaveis para a leitura do botao
const uint8_t PINO_BOTAO4 = 5;
bool leitura_botao4;
bool estado_saida4 = LOW;
bool estado_botao4;
bool ultimo_estado_botao4 = HIGH;
unsigned long ultimo_tempo_debounce4 = 0;

// Variaveis para a leitura do botao
const uint8_t PINO_BOTAO5 = 6;
bool leitura_botao5;
bool estado_saida5 = LOW;
bool estado_botao5;
bool ultimo_estado_botao5 = HIGH;
unsigned long ultimo_tempo_debounce5 = 0;

// Variaveis para a leitura do botao
const uint8_t PINO_BOTAO6 = 7;
bool leitura_botao6;
bool estado_saida6 = HIGH;
bool estado_botao6;
bool ultimo_estado_botao6 = HIGH;
unsigned long ultimo_tempo_debounce6 = 0;

// Variavel de debounce dos botoes
const uint8_t DEBOUNCE_BOTOES = 50;

// Variaveis para a leitura analogica
const uint8_t PINO_ANALOGICO1 = A0;
uint16_t leitura_analogica1;

// Variaveis para a leitura analogica
const uint8_t PINO_ANALOGICO2 = A1;
uint16_t leitura_analogica2;

// Variaveis para a leitura analogica
const uint8_t PINO_ANALOGICO3 = A2;
uint16_t leitura_analogica3;

// Variaveis para a leitura analogica
const uint8_t PINO_ANALOGICO4 = A3;
uint16_t leitura_analogica4;

// Variaveis para a leitura analogica
const uint8_t PINO_ANALOGICO5 = A6;
uint16_t leitura_analogica5;

// Variaveis para a leitura analogica
const uint8_t PINO_ANALOGICO6 = A7;
uint16_t leitura_analogica6;

// Variaveis para mapeamento de leituras analogicas
uint16_t mapeamento_joysticks[4] = {0, 0, 0, 0};
uint16_t progresso_sliders[2] = {0, 0};

// Declaracao das variaveis auxiliares para a temporizacao de envio de mensagens
unsigned long tempo_antes = 0;
const uint8_t INTERVALO = 50;

/**********************
 * Copiar estrutura no
 * codigo do robo
 **********************/
// Estrutura que armazena as variaveis que serao enviadas para o robo
typedef struct
{
  uint8_t leitura_botao1;
  uint8_t leitura_botao2;
  uint8_t leitura_botao3;
  uint8_t leitura_botao4;
  uint8_t leitura_botao5;
  uint8_t leitura_botao6;
  uint16_t leitura_eixoX1;
  uint16_t leitura_eixoY1;
  uint16_t leitura_eixoX2;
  uint16_t leitura_eixoY2;
  uint16_t leitura_Slide1;
  uint16_t leitura_Slide2;
} valores_controle;
valores_controle controle;

// Imagem de sinal perdido com o robo
const unsigned char lost_signal[] PROGMEM = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x1f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x1f, 0xfc, 0x0f, 0xfe, 0x0f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0x8f, 0xfc, 0x7f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0x03, 0xf0, 0x1f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0x00, 0xc0, 0x3f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf2, 0x7f, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0xc0, 0x00, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf0, 0x7f, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0xf0, 0x03, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf0, 0x7f, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0xf0, 0x03, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf0, 0x7f, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0xc0, 0x00, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf0, 0x7f, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0x00, 0xc0, 0x3f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf0, 0x7f, 0xf0, 0x3f, 0xf8, 0x1f, 0xff, 0x03, 0xf0, 0x1f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xfc, 0x7f, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// ------------------------------------------------------------------------------
// Funcao para exibir barra de progresso para os sliders
void drawProgressbar(int x, int y, int width, int height, int progress)
{
  float bar = ((float)(height - 4) / 100) * progress;
  display.drawRect(x, y, width, height, WHITE);
  display.fillRect(x + 2, y + 2, width - 4, bar, WHITE);
}

// ------------------------------------------------------------------------------
// Funcao para exibir o estado dos botoes
void drawButtonState(int x, int y, int width, int height, bool state)
{
  display.drawRect(x, y, width, height, WHITE);
  if (state)
  {
    display.fillRect(x + 2, y + 2, width - 4, height - 4, WHITE);
  }
}

// ------------------------------------------------------------------------------
// Funcao para exibir as posicoes dos joysticks
void drawJoyPosition(int x, int y, int radius, int px, int py)
{
  display.drawCircle(x, y, radius, WHITE);
  display.fillCircle(px, py, radius / 10, WHITE);
}

// ------------------------------------------------------------------------------
// Configuracoes de codigo
void setup()
{

  // Inicia comunicacao serial, se o debug estiver declarado
  Serial.begin(9600);

  // Configuracao dos LEDs
  pinMode(LED_L, OUTPUT);
  digitalWrite(LED_L, LOW);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH);

  // Inicializacao do display OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, ENDERECO_I2C))
  { // Se a inicializacao nao for bem sucedida
    // Exibe mensagem de erro no monitor serial, se opcao estiver definida
    Serial.println(F("FALHA INICIALIZACAO DO DISPLAY!!!"));
    while (!display.begin(SSD1306_SWITCHCAPVCC, ENDERECO_I2C))
    { // Enquanto a inicializacao nao for bem sucedida
      // Exibe mensagem de erro no monitor serial, se opcao estiver definida
      Serial.println(F("."));
      // Pisca LED de alerta
      digitalWrite(LED_L, !digitalRead(LED_L));
      delay(100);
    }
  }

  // Limpa o display se ele tiver com imagens do ultimo uso
  display.clearDisplay();
  display.display();

  // Inicializacao do modulo de radio
  if (!radio.begin())
  { // Se a inicializacao nao for bem sucedida
    // Exibe mensagem de erro no monitor serial, se opcao estiver definida
    Serial.println(F("FALHA NA INICIALIZACAO DO RADIO!!!"));
    while (!radio.begin())
    { // Enquanto a inicializacao nao for bem sucedida
      // Exibe mensagem de erro no monitor serial, se opcao estiver definida
      Serial.println(F("."));
      // Pisca LED de alerta
      digitalWrite(LED_L, !digitalRead(LED_L));
      delay(250);
    }
  }

  // Comanda o radio para operar em baixo consumo de energia
  radio.setPALevel(RF24_PA_MAX);

  // Configura tempo maximo de espera para transmissao de dados sendo igual ao tamanho maximo da mensagem que sera enviada
  radio.setPayloadSize(sizeof(controle));

  // Configura os canais de comunicacao
  radio.openWritingPipe(ENDERECO_RADIO[NUMERO_RADIO]);
  radio.openReadingPipe(1, ENDERECO_RADIO[!NUMERO_RADIO]);

  // Inicializa o radio como transmissor
  radio.stopListening();

  // Configuracao dos botoes
  pinMode(PINO_BOTAO1, INPUT_PULLUP);
  pinMode(PINO_BOTAO2, INPUT_PULLUP);
  pinMode(PINO_BOTAO3, INPUT_PULLUP);
  pinMode(PINO_BOTAO4, INPUT_PULLUP);
  pinMode(PINO_BOTAO5, INPUT_PULLUP);
  pinMode(PINO_BOTAO6, INPUT_PULLUP);

  // Configuracao dos pinos analogicos
  pinMode(PINO_ANALOGICO1, INPUT);
  pinMode(PINO_ANALOGICO2, INPUT);
  pinMode(PINO_ANALOGICO3, INPUT);
  pinMode(PINO_ANALOGICO4, INPUT);
  pinMode(PINO_ANALOGICO5, INPUT);
  pinMode(PINO_ANALOGICO6, INPUT);

  // Atualiza o LED para indicar que o setup esta compelto
  delay(1000);
  digitalWrite(LED_STATUS, LOW);
}

// ------------------------------------------------------------------------------
// Repeticao de codigo
void loop()
{

  // Realiza a leitura do botao
  leitura_botao1 = digitalRead(PINO_BOTAO1);
  if (leitura_botao1 != ultimo_estado_botao1)
  {
    ultimo_tempo_debounce1 = millis();
  }
  if ((millis() - ultimo_tempo_debounce1) > DEBOUNCE_BOTOES)
  {
    if (leitura_botao1 != estado_botao1)
    {
      estado_botao1 = leitura_botao1;
      if (estado_botao1 == LOW)
      {
        estado_saida1 = !estado_saida1;
      }
    }
  }
  ultimo_estado_botao1 = leitura_botao1;

  // Realiza a leitura analogica
  leitura_analogica1 = analogRead(PINO_ANALOGICO1);

  // Realiza a leitura do botao
  leitura_botao2 = digitalRead(PINO_BOTAO2);
  if (leitura_botao2 != ultimo_estado_botao2)
  {
    ultimo_tempo_debounce2 = millis();
  }
  if ((millis() - ultimo_tempo_debounce2) > DEBOUNCE_BOTOES)
  {
    if (leitura_botao2 != estado_botao2)
    {
      estado_botao2 = leitura_botao2;
      if (estado_botao2 == LOW)
      {
        estado_saida2 = !estado_saida2;
      }
    }
  }
  ultimo_estado_botao2 = leitura_botao2;

  // Realiza a leitura analogica
  leitura_analogica2 = analogRead(PINO_ANALOGICO2);

  // Realiza a leitura do botao
  leitura_botao3 = digitalRead(PINO_BOTAO3);
  if (leitura_botao3 != ultimo_estado_botao3)
  {
    ultimo_tempo_debounce3 = millis();
  }
  if ((millis() - ultimo_tempo_debounce3) > DEBOUNCE_BOTOES)
  {
    if (leitura_botao3 != estado_botao3)
    {
      estado_botao3 = leitura_botao3;
      if (estado_botao3 == LOW)
      {
        estado_saida3 = !estado_saida3;
      }
    }
  }
  ultimo_estado_botao3 = leitura_botao3;

  // Realiza a leitura analogica
  leitura_analogica3 = analogRead(PINO_ANALOGICO3);

  // Realiza a leitura do botao
  leitura_botao4 = digitalRead(PINO_BOTAO4);
  if (leitura_botao4 != ultimo_estado_botao4)
  {
    ultimo_tempo_debounce4 = millis();
  }
  if ((millis() - ultimo_tempo_debounce4) > DEBOUNCE_BOTOES)
  {
    if (leitura_botao4 != estado_botao4)
    {
      estado_botao4 = leitura_botao4;
      if (estado_botao4 == LOW)
      {
        estado_saida4 = !estado_saida4;
      }
    }
  }
  ultimo_estado_botao4 = leitura_botao4;

  // Realiza a leitura analogica
  leitura_analogica4 = analogRead(PINO_ANALOGICO4);

  // Realiza a leitura do botao
  leitura_botao5 = digitalRead(PINO_BOTAO5);
  if (leitura_botao5 != ultimo_estado_botao5)
  {
    ultimo_tempo_debounce5 = millis();
  }
  if ((millis() - ultimo_tempo_debounce5) > DEBOUNCE_BOTOES)
  {
    if (leitura_botao5 != estado_botao5)
    {
      estado_botao5 = leitura_botao5;
      if (estado_botao5 == LOW)
      {
        estado_saida5 = true;
      }
      else
      {
        estado_saida5 = false;
      }
    }
  }
  ultimo_estado_botao5 = leitura_botao5;

  // Realiza a leitura analogica
  leitura_analogica5 = analogRead(PINO_ANALOGICO5);

  // Realiza a leitura do botao
  leitura_botao6 = digitalRead(PINO_BOTAO6);
  if (leitura_botao6 != ultimo_estado_botao6)
  {
    ultimo_tempo_debounce6 = millis();
  }
  if ((millis() - ultimo_tempo_debounce6) > DEBOUNCE_BOTOES)
  {
    if (leitura_botao6 != estado_botao6)
    {
      estado_botao6 = leitura_botao6;
      if (estado_botao6 == LOW)
      {
        estado_saida6 = true;
      }
      else
      {
        estado_saida6 = false;
      }
    }
  }
  ultimo_estado_botao6 = leitura_botao6;

  // Realiza a leitura analogica
  leitura_analogica6 = analogRead(PINO_ANALOGICO6);

  // Mapeia as barras de progresso com as leituras dos sliders
  progresso_sliders[0] = map(leitura_analogica5, 0, 1015, 100, 0);
  progresso_sliders[1] = map(leitura_analogica6, 0, 1015, 100, 0);

  // Mapeia a leitura dos joysticks para posicoes validas dentro dos circulos de exibicao
  mapeamento_joysticks[0] = map(leitura_analogica4, 0, 1023, 45, 9);   // X1
  mapeamento_joysticks[1] = map(leitura_analogica3, 0, 1023, 49, 13);  // Y1
  mapeamento_joysticks[2] = map(leitura_analogica2, 0, 1023, 119, 83); // X2
  mapeamento_joysticks[3] = map(leitura_analogica1, 0, 1023, 49, 13);  // Y2

  // Atualiza as variaveis para envio
  controle.leitura_botao1 = estado_saida1;
  controle.leitura_botao2 = estado_saida2;
  controle.leitura_botao3 = estado_saida3;
  controle.leitura_botao4 = estado_saida4;
  controle.leitura_botao5 = estado_saida5;
  controle.leitura_botao6 = estado_saida6;
  controle.leitura_eixoX1 = leitura_analogica4;
  controle.leitura_eixoY1 = leitura_analogica3;
  controle.leitura_eixoX2 = leitura_analogica2;
  controle.leitura_eixoY2 = leitura_analogica1;
  controle.leitura_Slide1 = leitura_analogica5;
  controle.leitura_Slide2 = leitura_analogica6;

  if ((millis() - tempo_antes) > INTERVALO)
  {

    // Limpa o display a cada repeticao
    display.clearDisplay();

    // Realiza o envio de dados, e verifica o recebimento
    bool enviado = radio.write(&controle, sizeof(controle));

    if (enviado)
    {
      // Atualiza as barras de progresso
      display.invertDisplay(false);
      drawProgressbar(52, 10, 10, 44, progresso_sliders[0]);
      drawProgressbar(64, 10, 10, 44, progresso_sliders[1]);

      // Exibe as posicoes dos joysticks
      drawJoyPosition(27, 31, 20, mapeamento_joysticks[0], mapeamento_joysticks[1]);
      drawJoyPosition(101, 31, 20, mapeamento_joysticks[2], mapeamento_joysticks[3]);

      // Exibe no display o estado dos botoes
      drawButtonState(2, 0, 60, 10, estado_saida1);
      drawButtonState(64, 0, 60, 10, estado_saida2);
      drawButtonState(2, 54, 29, 10, estado_saida3);
      drawButtonState(33, 54, 29, 10, estado_saida4);
      drawButtonState(64, 54, 29, 10, estado_saida5);
      drawButtonState(95, 54, 29, 10, estado_saida6);

      // Pisca o LED para indicar envio de mensagem
      digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
    }
    else
    {
      // Exibe a imagem de sinal perdido
      display.invertDisplay(true);
      display.drawBitmap(0, 0, lost_signal, 128, 64, 1);

      // Mantem o LED aceso para indicar erro
      digitalWrite(LED_STATUS, HIGH);
    }

    // Exibe as atualizacoes do display
    display.display();
  }
}
// ------------------------------------------------------------------------------