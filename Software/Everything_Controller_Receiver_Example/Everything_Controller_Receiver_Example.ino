/*******************************************************************************
* Everything Controller - Exemplo Bascio de Receptor (v1.0)
* 
* Codigo basico de exemplo de receptor para recepcao de dados do controle
* "Everything Controller" baseado em comunicacao RF24 atraves do modulo NRF24L01+.
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
#include "printf.h"
#include "RF24.h"

// Criacao do objeto de controle do radio receptor
RF24 radio(9, 10); // 9 -> CE | 10 -> CSN

// Enderecos dos radios (controle e receptor (robo))
uint8_t address[][6] = { "Ctrlr", "Robot" };

// Numero do radio no conjunto (controle -> 0 | receptor -> 1)
const bool radioNumber = 1;

// Estrutura de variaveis que sao trnamitidas pelo radio
typedef struct {
  uint8_t leitura_botao1;
  uint8_t leitura_botao2;
  uint8_t leitura_botao3;
  uint8_t leitura_botao4;
  uint8_t leitura_botao5;
  uint8_t leitura_botao6;
  uint8_t leitura_eixoX1;
  uint8_t leitura_eixoY1;
  uint8_t leitura_eixoX2;
  uint8_t leitura_eixoY2;
  uint8_t leitura_Slide1;
  uint8_t leitura_Slide2;
} variaveis_mensagem;
variaveis_mensagem controle;

// Variaveis para o recebimento de mensagens
uint8_t canal;
uint8_t bytes;

// Configuracoes de codigo
void setup() {

  // Inicializacao do monitor serial para verificar dados recebidos
  Serial.begin(9600);

  // Inicializacao da comunicacao SPI e do radio
  if (!radio.begin()) {
    Serial.println(F("Falha na inicializacao do radio"));
    while (1) {}  // hold in infinite loop
  }

  // Configura o radio para operar em baixo consumo de energia
  radio.setPALevel(RF24_PA_LOW);

  // Configura o radio para aguardar o tempo maximo igual ao tamanho da estrutura que sera recebida
  radio.setPayloadSize(sizeof(controle));

  // Configura o canal de escrita do radio, para retornar mensagens ao radio, se precisar
  radio.openWritingPipe(address[radioNumber]);

  // Configura o canal que o radio ira receber as mensagens
  radio.openReadingPipe(1, address[!radioNumber]);

  // Comanda o radio para iniciar o recebimento de mensagens
  radio.startListening();

}

// Repeticao do codigo
void loop() {

  
  if (radio.available(&canal)) 
  {
    bytes = radio.getPayloadSize();
    radio.read(&controle, bytes);
    Serial.print(F("Mensagem de "));
    Serial.print(bytes);
    Serial.print(F(" bytes recebida no canal "));
    Serial.print(canal);
    Serial.println(F(" conteudo : "));
    Serial.print(controle.leitura_botao1);
    Serial.print(F(" | "));
    Serial.print(controle.leitura_botao2);
    Serial.print(F(" | "));
    Serial.print(controle.leitura_botao3);
    Serial.print(F(" | "));
    Serial.print(controle.leitura_botao4);
    Serial.print(F(" | "));
    Serial.print(controle.leitura_botao5);
    Serial.print(F(" | "));
    Serial.println(controle.leitura_botao6);
    Serial.print(controle.leitura_eixoX1);
    Serial.print(F(" | "));
    Serial.print(controle.leitura_eixoY1);
    Serial.print(F(" | "));
    Serial.print(controle.leitura_eixoX2);
    Serial.print(F(" | "));
    Serial.print(controle.leitura_eixoY2);
    Serial.print(F(" | "));
    Serial.print(controle.leitura_Slide1);
    Serial.print(F(" | "));
    Serial.println(controle.leitura_Slide2);
  }
}
