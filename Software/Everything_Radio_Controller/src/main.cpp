/*******************************************************************************
 * Everything Controller (v1.0)
 *
 * Base code for the "Everything Controller" to send data to a receiver.
 * Communication is based on RF24 using the NRF24L01+ module. The controller
 * sends data from 6 digital buttons, 2 joysticks (2 X-axis and 2 Y-axis), as
 * well as the analog readings from two sliders. With this data, it is possible
 * to control various robots.
 *
 * Written by Giovanni de Castro (19/11/2023).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version (<https://www.gnu.org/licenses/>).
 *******************************************************************************/

// Libraries
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------------------------------------------------------------------------------
// Control pins of the radio module
const uint8_t PIN_CE = 9;
const uint8_t PIN_CSN = 10;

// Control variables of the OLED display
const uint8_t DISPLAY_ADDRESS = 0x3C;
const uint8_t DISPLAY_WIDTH = 128;
const uint8_t DISPLAY_HEIGHT = 64;
const int8_t DISPLAY_RESET = -1;

// Control pins of the LEDs
const uint8_t LED_STATUS = 8;
const uint8_t LED_L = 13;

// Controller address
const uint8_t RADIO_ADDRESSES[][6] = {"Ctrlr", "Robot"};

// Radio number
const uint8_t RADIO_NUMBER = 0;

// Creation of the radio module control object
RF24 radio(PIN_CE, PIN_CSN);

// Creation of the display control object
Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, DISPLAY_RESET);

// Variables for reading the button
const uint8_t BUTTON1_PIN = 2;
bool button1_reading;
bool output1_state = LOW;
bool button1_state;
bool last_button1_state = HIGH;
unsigned long last_debounce1 = 0;

// Variables for reading the button
const uint8_t BUTTON2_PIN = 3;
bool button2_reading;
bool output2_state = LOW;
bool button2_state;
bool last_button2_state = HIGH;
unsigned long last_debounce2 = 0;

// Variables for reading the button
const uint8_t BUTTON3_PIN = 4;
bool button3_reading;
bool output3_state = LOW;
bool button3_state;
bool last_button3_state = HIGH;
unsigned long last_debounce3 = 0;

// Variables for reading the button
const uint8_t BUTTON4_PIN = 5;
bool button4_reading;
bool output4_state = LOW;
bool button4_state;
bool last_button4_state = HIGH;
unsigned long last_debounce4 = 0;

// Variables for reading the button
const uint8_t BUTTON5_PIN = 6;
bool button5_reading;
bool output5_state = LOW;
bool button5_state;
bool last_button5_state = HIGH;
unsigned long last_debounce5 = 0;

// Variables for reading the button
const uint8_t BUTTON6_PIN = 7;
bool button6_reading;
bool output6_state = HIGH;
bool button6_state;
bool last_button6_state = HIGH;
unsigned long last_debounce6 = 0;

// Variable for buttons debounce
const uint8_t DEBOUNCE = 50;

// Variables for analog reading
const uint8_t ANALOG1_PIN = A0;
uint16_t analog1_reading;

// Variables for analog reading
const uint8_t ANALOG2_PIN = A1;
uint16_t analog2_reading;

// Variables for analog reading
const uint8_t ANALOG3_PIN = A2;
uint16_t analog3_reading;

// Variables for analog reading
const uint8_t ANALOG4_PIN = A3;
uint16_t analog4_reading;

// Variables for analog reading
const uint8_t ANALOG5_PIN = A6;
uint16_t analog5_reading;

// Variables for analog reading
const uint8_t ANALOG6_PIN = A7;
uint16_t analog6_reading;

// Variables for analog readings mappings
uint16_t joysticks_mapping[4] = {0, 0, 0, 0};
uint16_t sliders_mapping[2] = {0, 0};

// Variables for variables transmission timing
unsigned long last_millis = 0;
const uint8_t TX_INTERVAL = 50;
unsigned long last_message = millis();
const uint16_t TX_TIMEOUT = 500;

// Variables structure
typedef struct
{
  uint8_t button1_reading;
  uint8_t button2_reading;
  uint8_t button3_reading;
  uint8_t button4_reading;
  uint8_t button5_reading;
  uint8_t button6_reading;
  uint16_t X1axis_reading;
  uint16_t Y1axis_reading;
  uint16_t X2axis_reading;
  uint16_t Y2axis_reading;
  uint16_t slider1_reading;
  uint16_t slider2_reading;
} controller_variables;
controller_variables controller;

// Lost signal image
const unsigned char lost_signal[] PROGMEM = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xf0, 0xc6, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x86, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x82, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x02, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x10, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x30, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x38, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x87, 0xf0, 0x78, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x7f, 0x87, 0xf0, 0x7c, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0x87, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0x87, 0xf0, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0x87, 0xe1, 0xc7, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0x87, 0xe1, 0xc7, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0x87, 0xc3, 0xc7, 0x87, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0x87, 0x83, 0xc7, 0x87, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0x87, 0x87, 0xc7, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0x87, 0x0f, 0xc7, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xf3, 0xfc, 0x3f, 0x87, 0x0f, 0xc7, 0xe1, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x86, 0x1f, 0xc7, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x86, 0x1f, 0xcf, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x84, 0x3f, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x84, 0x3f, 0xef, 0xf8, 0x7f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x80, 0x7f, 0xc7, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x80, 0x7f, 0xc7, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x80, 0xff, 0xcf, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x80, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x81, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe1, 0xfc, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xf3, 0xfe, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// ------------------------------------------------------------------------------
// Function to display a progress bar for the sliders
void drawSliderPosition(int x, int y, int width, int height, int progress)
{
  float bar = ((float)(height - 4) / 100) * progress;
  display.drawRoundRect(x, y, width, height, 5, WHITE);
  display.fillRoundRect(x + 2, y + 2, width - 4, height - 4, 2, WHITE);
  display.fillRoundRect(x + 2, y + 2, width - 4, bar, 2, BLACK);
}

// ------------------------------------------------------------------------------
// Function to draw startup progressa bar
void drawProgressBar(int x, int y, int width, int height, int progress)
{
  progress = progress > 100 ? 100 : progress;
  progress = progress < 0 ? 0 : progress;
  float bar = ((float)(width - 4) / 100) * progress;
  display.drawRoundRect(x, y, width, height, 5, WHITE);
  display.fillRoundRect(x + 2, y + 2, bar, height - 4, 2, WHITE);
}

// ------------------------------------------------------------------------------
// Function to display buttons states
void drawButtonState(int x, int y, int width, int height, bool state)
{
  display.drawRoundRect(x, y, width, height, 5, WHITE);
  if (state)
  {
    display.fillRoundRect(x + 2, y + 2, width - 4, height - 4, 2, WHITE);
  }
}

// ------------------------------------------------------------------------------
// Function to display the joysticks positions
void drawJoyPosition(int x, int y, int radius, int px, int py)
{
  display.drawLine(x, (y + radius) - 4, x, (y - radius) + 4, WHITE);
  display.drawLine((x + radius) - 4, y, (x - radius) + 4, y, WHITE);
  display.drawRoundRect(x - radius, y - radius, (radius * 2) + 1, (radius * 2) + 1, 5, WHITE);
  display.drawCircle(x, y, radius, WHITE);
  display.fillCircle(px, py, radius / 10, WHITE);
}

// ------------------------------------------------------------------------------
void setup()
{

  // Serial monitor initialization for debug
  Serial.begin(9600);

  // LEDs configuration and initialization
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  // OLED display initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS))
  {
    Serial.println(F("DISPLAY INITIALIZATION FAILED!!!"));
    while (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS))
    {
      Serial.println(F("."));
      // Blinks alert LED
      digitalWrite(LED_L, !digitalRead(LED_L));
      delay(100);
    }
  }

  // Clears the display from last usage
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(4, 0);
  display.println("Starting");
  drawProgressBar(4, 32, 120, 10, 10);
  display.display();
  delay(250);

  // Radio module initialization
  if (!radio.begin())
  {
    Serial.println(F("RADIO INITIALIZATION FAILED!!!"));
    while (!radio.begin())
    {
      Serial.println(F("."));
      // Blinks alert LED
      digitalWrite(LED_L, !digitalRead(LED_L));
      delay(250);
    }
  }

  // Clears the display from last usage
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(4, 0);
  display.println("Starting.");
  drawProgressBar(4, 32, 120, 10, 35);
  display.display();
  delay(100);

  // Configure the radio for maximum PA level
  radio.setPALevel(RF24_PA_MAX);

  // Set the maximum waiting time for data transmission to be equal to the maximum size of the message to be sent
  radio.setPayloadSize(sizeof(controller));

  // Configuration of the transmission channels
  radio.openWritingPipe(RADIO_ADDRESSES[RADIO_NUMBER]);
  radio.openReadingPipe(1, RADIO_ADDRESSES[!RADIO_NUMBER]);

  // Initializes the radio as transmitter
  radio.stopListening();

  // Clears the display from last usage
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(4, 0);
  display.println("Starting..");
  drawProgressBar(4, 32, 120, 10, 60);
  display.display();
  delay(100);

  // Buttons configuration
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);
  pinMode(BUTTON5_PIN, INPUT_PULLUP);
  pinMode(BUTTON6_PIN, INPUT_PULLUP);

  // Clears the display from last usage
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(4, 0);
  display.println("Starting...");
  drawProgressBar(4, 32, 120, 10, 85);
  display.display();
  delay(100);

  // Analog pins configuration
  pinMode(ANALOG1_PIN, INPUT);
  pinMode(ANALOG2_PIN, INPUT);
  pinMode(ANALOG3_PIN, INPUT);
  pinMode(ANALOG4_PIN, INPUT);
  pinMode(ANALOG5_PIN, INPUT);
  pinMode(ANALOG6_PIN, INPUT);

  // Clears the display from last usage
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(4, 0);
  display.println("Done!");
  drawProgressBar(4, 32, 120, 10, 100);
  display.display();
  delay(250);
}

// ------------------------------------------------------------------------------
void loop()
{

  // Blinks the LED
  digitalWrite(LED_STATUS, LOW);

  // Reads the button
  button1_reading = digitalRead(BUTTON1_PIN);
  if (button1_reading != last_button1_state)
  {
    last_debounce1 = millis();
  }
  if ((millis() - last_debounce1) > DEBOUNCE)
  {
    if (button1_reading != button1_state)
    {
      button1_state = button1_reading;
      if (button1_state == LOW)
      {
        output1_state = !output1_state;
      }
    }
  }
  last_button1_state = button1_reading;

  // Reads analog pin
  analog1_reading = analogRead(ANALOG1_PIN);

  // Reads the button
  button2_reading = digitalRead(BUTTON2_PIN);
  if (button2_reading != last_button2_state)
  {
    last_debounce2 = millis();
  }
  if ((millis() - last_debounce2) > DEBOUNCE)
  {
    if (button2_reading != button2_state)
    {
      button2_state = button2_reading;
      if (button2_state == LOW)
      {
        output2_state = !output2_state;
      }
    }
  }
  last_button2_state = button2_reading;

  // Reads analog pin
  analog2_reading = analogRead(ANALOG2_PIN);

  // Reads the button
  button3_reading = digitalRead(BUTTON3_PIN);
  if (button3_reading != last_button3_state)
  {
    last_debounce3 = millis();
  }
  if ((millis() - last_debounce3) > DEBOUNCE)
  {
    if (button3_reading != button3_state)
    {
      button3_state = button3_reading;
      if (button3_state == LOW)
      {
        output3_state = !output3_state;
      }
    }
  }
  last_button3_state = button3_reading;

  // Reads analog pin
  analog3_reading = analogRead(ANALOG3_PIN);

  // Reads the button
  button4_reading = digitalRead(BUTTON4_PIN);
  if (button4_reading != last_button4_state)
  {
    last_debounce4 = millis();
  }
  if ((millis() - last_debounce4) > DEBOUNCE)
  {
    if (button4_reading != button4_state)
    {
      button4_state = button4_reading;
      if (button4_state == LOW)
      {
        output4_state = !output4_state;
      }
    }
  }
  last_button4_state = button4_reading;

  // Reads analog pin
  analog4_reading = analogRead(ANALOG4_PIN);

  // Reads the button
  button5_reading = digitalRead(BUTTON5_PIN);
  if (button5_reading != last_button5_state)
  {
    last_debounce5 = millis();
  }
  if ((millis() - last_debounce5) > DEBOUNCE)
  {
    if (button5_reading != button5_state)
    {
      button5_state = button5_reading;
      if (button5_state == LOW)
      {
        output5_state = true;
      }
      else
      {
        output5_state = false;
      }
    }
  }
  last_button5_state = button5_reading;

  // Reads analog pin
  analog5_reading = analogRead(ANALOG5_PIN);

  // Reads the button
  button6_reading = digitalRead(BUTTON6_PIN);
  if (button6_reading != last_button6_state)
  {
    last_debounce6 = millis();
  }
  if ((millis() - last_debounce6) > DEBOUNCE)
  {
    if (button6_reading != button6_state)
    {
      button6_state = button6_reading;
      if (button6_state == LOW)
      {
        output6_state = true;
      }
      else
      {
        output6_state = false;
      }
    }
  }
  last_button6_state = button6_reading;

  // Reads analog pin
  analog6_reading = analogRead(ANALOG6_PIN);

  // Maps the progress bars to the readings of the sliders
  sliders_mapping[0] = map(analog5_reading, 0, 1000, 0, 100);
  sliders_mapping[1] = map(analog6_reading, 0, 1023, 0, 100);

  // Maps the readings of the joysticks to valid positions within the display circles
  joysticks_mapping[0] = map(analog4_reading, 0, 1023, 40, 8);   // X1
  joysticks_mapping[1] = map(analog3_reading, 0, 1023, 47, 15);  // Y1
  joysticks_mapping[2] = map(analog2_reading, 0, 1023, 117, 85); // X2
  joysticks_mapping[3] = map(analog1_reading, 0, 1023, 47, 15);  // Y2

  // Updates the variables for transmission
  controller.button1_reading = output1_state;
  controller.button2_reading = output2_state;
  controller.button3_reading = output3_state;
  controller.button4_reading = output4_state;
  controller.button5_reading = output5_state;
  controller.button6_reading = output6_state;
  controller.X1axis_reading = analog4_reading;
  controller.Y1axis_reading = analog3_reading;
  controller.X2axis_reading = analog2_reading;
  controller.Y2axis_reading = analog1_reading;
  controller.slider1_reading = analog5_reading;
  controller.slider2_reading = analog6_reading;

  if ((millis() - last_millis) > TX_INTERVAL)
  {

    // Clears display
    display.clearDisplay();

    // Sends the data and checks for reception
    bool sent = radio.write(&controller, sizeof(controller));

    if (sent)
    {
      // Updates and displays progress bars
      digitalWrite(LED_STATUS, HIGH);
      last_message = millis();
      display.invertDisplay(false);
      drawSliderPosition(52, 10, 10, 44, sliders_mapping[0]);
      drawSliderPosition(64, 10, 10, 44, sliders_mapping[1]);

      // Updates and displays the joysticks positions
      drawJoyPosition(24, 31, 20, joysticks_mapping[0], joysticks_mapping[1]);
      drawJoyPosition(101, 31, 20, joysticks_mapping[2], joysticks_mapping[3]);

      // Updates and displays the buttons states
      drawButtonState(2, 0, 60, 10, output1_state);
      drawButtonState(64, 0, 60, 10, output2_state);
      drawButtonState(2, 54, 29, 10, output3_state);
      drawButtonState(33, 54, 29, 10, output6_state);
      drawButtonState(64, 54, 29, 10, output5_state);
      drawButtonState(95, 54, 29, 10, output4_state);
    }
    else if ((millis() - last_message) > TX_TIMEOUT)
    {
      // Displays lost signal image
      display.invertDisplay(true);
      display.drawBitmap(0, 0, lost_signal, 128, 64, 1);
    }

    // Shows displays update
    display.display();
    last_millis = millis();
  }
}
// ------------------------------------------------------------------------------