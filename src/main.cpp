#include <Arduino.h>
#include <Wire.h>

// Setup GPIOs for ADC
const int adcPins[] = {32, 33, 34, 35, 36, 39, 25, 26}; // 8 ADC pins
const int numAdcInputs = sizeof(adcPins) / sizeof(adcPins[0]);
bool adcStates[numAdcInputs] = {0};  // Tracks the current state of each ADC pin
bool adcTriggered[numAdcInputs] = {0};  // Tracks if the ADC pin has been triggered (sent as HIGH)
unsigned long lastAdcDebounceTime[numAdcInputs] = {0};  // Debounce timers for ADC inputs
const unsigned long debounceDelay = 50;  // Debounce time in milliseconds

// Setup GPIOs for Digital Inputs
const int digitalPins[] = {4, 5, 13, 14, 18, 19}; // 6 Digital input pins
const int numDigitalInputs = sizeof(digitalPins) / sizeof(digitalPins[0]);
bool lastState[numDigitalInputs] = {0};  // Tracks the current state of each digital pin
bool digitalTriggered[numDigitalInputs] = {0};  // Tracks if the digital pin has been triggered (sent as HIGH)
unsigned long lastDebounceTime[numDigitalInputs] = {0};  // Debounce timers for digital inputs

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
    if (adcTriggered[i]) {
      Wire.write(adcPins[i]);  // Send the pin number
      Wire.write(HIGH);  // Send the state as HIGH
      adcTriggered[i] = false;  // Reset the triggered state to LOW after being read
      return; // Only send one state at a time
    }
  }

  for (int i = 0; i < numDigitalInputs; i++) {
    if (digitalTriggered[i]) {
      Wire.write(digitalPins[i]);  // Send the pin number
      Wire.write(HIGH);  // Send the state as HIGH
      digitalTriggered[i] = false;  // Reset the triggered state to LOW after being read
      return; // Only send one state at a time
    }
  }

  // If no pin was triggered, send a neutral signal
  Wire.write(0xFF);  // Send a neutral pin number
  Wire.write(LOW);   // Send a neutral state (LOW)
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
  // Monitor ADC Inputs
  for (int i = 0; i < numAdcInputs; i++) {
    int adcValue = analogRead(adcPins[i]); // Read the ADC value
    float voltage = adcValue * (3.3 / 4095.0); // Convert ADC value to voltage
    bool currentAdcState = (voltage > 0.6 && voltage < 1.5) ? HIGH : LOW; // Determine HIGH or LOW based on 2.5V threshold
    
    if (currentAdcState != adcStates[i]) {
      if (millis() - lastAdcDebounceTime[i] > debounceDelay) {
        adcStates[i] = currentAdcState;  // Update state after debounce
        lastAdcDebounceTime[i] = millis();  // Reset debounce timer
        Serial.print("Pin:"); 
        Serial.print(adcPins[i]);
        Serial.print(" Voltage:"); 
        Serial.print(voltage);  
        Serial.print("V");  
        Serial.println();
        if (adcStates[i] == HIGH) {
          adcTriggered[i] = true;  // Mark the pin as triggered if it's HIGH
        }
      }
    }
  }

  // Monitor Digital Inputs
  for (int i = 0; i < numDigitalInputs; i++) {
    int currentState = digitalRead(digitalPins[i]);

    if (currentState != lastState[i]) {
      if (millis() - lastDebounceTime[i] > debounceDelay) {
        lastState[i] = currentState;  // Update state after debounce
        lastDebounceTime[i] = millis();  // Reset debounce timer

        if (lastState[i] == HIGH) {
          digitalTriggered[i] = true;  // Mark the pin as triggered if it's HIGH
        }
      }
    }
  }
}
