#include <Arduino.h>
#include <Wire.h>

// Setup GPIOs for ADC
const int adcPins[] = {32, 33, 34, 35, 36, 39, 25, 26}; // 8 ADC pins
const int numAdcInputs = sizeof(adcPins) / sizeof(adcPins[0]);
int adcValues[numAdcInputs] = {0}; // Store the ADC values
bool adcStates[numAdcInputs] = {0};  // Tracks the last state of each ADC pin
unsigned long lastAdcDebounceTime[numAdcInputs] = {0};
const unsigned long debounceDelay = 50;  // Debounce time in milliseconds

// Setup GPIOs for Digital Inputs
const int digitalPins[] = {4, 5, 13, 14, 18, 19}; // 6 Digital input pins
const int numDigitalInputs = sizeof(digitalPins) / sizeof(digitalPins[0]);
bool lastState[numDigitalInputs] = {0};  // Tracks the last state of each digital pin
unsigned long lastDebounceTime[numDigitalInputs] = {0};

int i2cAddress = 0x08; // I2C address of the ESP32 slave

// Function to handle data received from master
void receiveData(int byteCount) {
  Serial.print("Received data: ");
  while (Wire.available()) {
    char c = Wire.read(); // Receive byte as a character
    Serial.print(c); // Print the received byte
  }
  Serial.println();
}

// Function to handle requests from the master
void requestData() {
  for (int i = 0; i < numAdcInputs; i++) {
    int adcValue = analogRead(adcPins[i]); // Read the ADC value
    Wire.write(adcValue >> 8);  // Send high byte of ADC value
    Wire.write(adcValue & 0xFF);  // Send low byte of ADC value
  }
  for (int i = 0; i < numDigitalInputs; i++) {
    int currentState = digitalRead(digitalPins[i]);
    Wire.write(currentState); // Send digital pin state (HIGH or LOW)
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C as slave
  Wire.begin(i2cAddress); // Set ESP32 as I2C slave with the specified address
  Wire.onReceive(receiveData); // Register a function to handle incoming data from master
  Wire.onRequest(requestData); // Register a function to handle requests from master

  // Initialize ADC pins
  for (int i = 0; i < numAdcInputs; i++) {
    pinMode(adcPins[i], INPUT); // Initialize pins as inputs (ADC pins do not use INPUT_PULLUP)
  }

  // Initialize Digital Input pins
  for (int i = 0; i < numDigitalInputs; i++) {
    pinMode(digitalPins[i], INPUT_PULLUP); // Digital pins initialized with pull-up resistors
  }
}

void loop() {
  // The loop is empty because the I2C slave will respond to requests and data received
  // automatically using the registered functions.
}