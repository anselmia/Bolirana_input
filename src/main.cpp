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

int i2cAddress = 0x08; // I2C address of the slave (e.g., Raspberry Pi)

void setup() {
  Serial.begin(115200);

  // Initialize I2C as master
  Wire.begin(); // Default pins SDA (GPIO21), SCL (GPIO22)

  // Initialize ADC pins
  for (int i = 0; i < numAdcInputs; i++) {
    
    pinMode(adcPins[i], INPUT); // Initialize pins as inputs (ADC pins do not use INPUT_PULLUP)
  }

  // Initialize Digital Input pins
  for (int i = 0; i < numDigitalInputs; i++) {
    pinMode(digitalPins[i], INPUT_PULLUP); // Digital pins initialized with pull-up resistors
  }
}

void sendState(int pin, String state) {
  Wire.beginTransmission(i2cAddress); // Begin I2C communication with the slave
  Wire.write(pin);  // Send the pin number
  Wire.write(state == "HIGH" ? 1 : 0); // Send digital pin state (HIGH or LOW as 1 or 0)
  Wire.endTransmission(); // End I2C communication
}

void loop() {
  // Check Digital Inputs
  for (int i = 0; i < numDigitalInputs; i++) {
    int currentState = digitalRead(digitalPins[i]);
    if (currentState != lastState[i] && (millis() - lastDebounceTime[i] > debounceDelay)) {
      lastDebounceTime[i] = millis();  // Reset debounce timer
      lastState[i] = currentState;
      sendState(digitalPins[i], currentState ? "HIGH" : "LOW");
      Serial.print("Digital Pin state changed for pin ");
      Serial.println(digitalPins[i]);
      Serial.print(" State :");
      Serial.println(currentState ? "HIGH" : "LOW");
    }
  }

  // Check ADC Inputs
  for (int i = 0; i < numAdcInputs; i++) {
    int adcValue = analogRead(adcPins[i]); // Read the ADC value
    float voltage = adcValue * (3.3 / 4095.0); // Convert ADC value to voltage

    bool currentAdcState = (voltage > 2.5) ? HIGH : LOW;

    if (currentAdcState != adcStates[i] && (millis() - lastAdcDebounceTime[i] > debounceDelay)) {
      lastAdcDebounceTime[i] = millis();  // Reset debounce timer
      adcStates[i] = currentAdcState;
      sendState(adcPins[i], currentAdcState ? "HIGH" : "LOW");
      Serial.print("ADC Pin state changed for pin ");
      Serial.println(adcPins[i]);
      Serial.print(" Voltage :");
      Serial.println(voltage);
      Serial.print(" State :");
      Serial.println(currentAdcState ? "HIGH" : "LOW");
    }
  }

  delay(10); // Delay to prevent rapid looping
}
