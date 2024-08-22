#include <Arduino.h>
#include <Wire.h>

// Setup GPIOs for Digital Inputs
const int digitalPins[] = {4, 5,18, 32, 13, 14, 15, 16, 17, 25, 23, 26, 27, 33, 34, 35, 36, 39, 19};
const int numDigitalInputs = sizeof(digitalPins) / sizeof(digitalPins[0]);
volatile bool digitalTriggered[numDigitalInputs] = {0};  // Tracks if the digital pin has been triggered
volatile unsigned long lastTriggerTime[numDigitalInputs] = {0};  // Store last trigger time to manage debounce
const unsigned long debounceDelay = 10;  // Reduced debounce time in milliseconds
volatile bool lastEdgeWasRising[numDigitalInputs] = {false};  // Track the last detected edge for each pin
volatile bool i2cMasterDetected = false;
int i2cAddress = 0x08; // I2C address of the ESP32 slave

void receiveData(int byteCount) {
  // Minimal code to handle data reception from I2C master
  i2cMasterDetected = true;  // Set the flag indicating the master is present
  while (Wire.available()) {
    char c = Wire.read(); // Read the data to clear the buffer
    Serial.print("Received data: ");
    Serial.println(c);  // Debugging - print received data
  }
  delay(5);
  Wire.write("Hello");
}

// Function to handle requests from the master
void requestData() {
  for (int i = 0; i < numDigitalInputs; i++) {
    if (digitalTriggered[i]) {
      Wire.write(digitalPins[i]);  // Send the pin number
      Wire.write(HIGH);  // Send the state as HIGH
      digitalTriggered[i] = false;  // Reset the triggered state after being read
      return; // Only send one state at a time
    }
  }
  
  // If no pin was triggered, send a neutral signal
  Wire.write(0xFF);  // Send a neutral pin number
  Wire.write(LOW);   // Send a neutral state (LOW)
}

// Interrupt service routine for pin state change
void IRAM_ATTR handleInterrupt(int index) {
  unsigned long currentTime = micros();  // Use micros for better resolution
  if (currentTime - lastTriggerTime[index] > debounceDelay * 1000) {  // Convert debounceDelay to microseconds
    int pinState = digitalRead(digitalPins[index]);

    if (pinState == HIGH) {
      // Detected a rising edge
      lastEdgeWasRising[index] = true;
    } else if (pinState == LOW && lastEdgeWasRising[index]) {
      // Detected a falling edge after a rising edge
      digitalTriggered[index] = true;
      lastEdgeWasRising[index] = false;  // Reset for the next pulse
    }

    lastTriggerTime[index] = currentTime;
  }
}

// Wrapper functions for attaching interrupts
void IRAM_ATTR handleInterrupt0() { handleInterrupt(0); }
void IRAM_ATTR handleInterrupt1() { handleInterrupt(1); }
void IRAM_ATTR handleInterrupt2() { handleInterrupt(2); }
void IRAM_ATTR handleInterrupt3() { handleInterrupt(3); }
void IRAM_ATTR handleInterrupt4() { handleInterrupt(4); }
void IRAM_ATTR handleInterrupt5() { handleInterrupt(5); }
void IRAM_ATTR handleInterrupt6() { handleInterrupt(6); }
void IRAM_ATTR handleInterrupt7() { handleInterrupt(7); }
void IRAM_ATTR handleInterrupt8() { handleInterrupt(8); }
void IRAM_ATTR handleInterrupt9() { handleInterrupt(9); }
void IRAM_ATTR handleInterrupt10() { handleInterrupt(10); }
void IRAM_ATTR handleInterrupt11() { handleInterrupt(11); }
void IRAM_ATTR handleInterrupt12() { handleInterrupt(12); }
void IRAM_ATTR handleInterrupt13() { handleInterrupt(13); }
void IRAM_ATTR handleInterrupt14() { handleInterrupt(14); }
void IRAM_ATTR handleInterrupt15() { handleInterrupt(15); }
void IRAM_ATTR handleInterrupt16() { handleInterrupt(16); }
void IRAM_ATTR handleInterrupt17() { handleInterrupt(17); }
void IRAM_ATTR handleInterrupt18() { handleInterrupt(18); }
void IRAM_ATTR handleInterrupt19() { handleInterrupt(19); }
void IRAM_ATTR handleInterrupt20() { handleInterrupt(20); }
void IRAM_ATTR handleInterrupt21() { handleInterrupt(21); }
void IRAM_ATTR handleInterrupt22() { handleInterrupt(22); }
void IRAM_ATTR handleInterrupt23() { handleInterrupt(23); }
void IRAM_ATTR handleInterrupt24() { handleInterrupt(24); }
void IRAM_ATTR handleInterrupt25() { handleInterrupt(25); }

void waitForI2CMaster() {
  Serial.println("Waiting for I2C master...");

  // Loop until the I2C master sends the first message
  while (!i2cMasterDetected) {
    delay(100);  // Just a small delay to avoid spinning too fast in the loop
    Serial.print(".");
  }

  Serial.println("\nI2C master detected. Proceeding...");
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C as slave
  Wire.begin(i2cAddress); // Set ESP32 as I2C slave with the specified address
  Wire.onReceive(receiveData); // Register a function to handle incoming data from master
  Wire.onRequest(requestData); // Register a function to handle requests from master
  
  // Wait for an I2C master to be present before proceeding
  waitForI2CMaster();

  // Initialize Digital Input pins with pull-up resistors and attach interrupts
  for (int i = 0; i < numDigitalInputs; i++) {
    pinMode(digitalPins[i], INPUT_PULLUP);
    // Attach interrupts based on the index
    switch (i) {
      case 0: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt0, CHANGE); break;
      case 1: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt1, CHANGE); break;
      case 2: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt2, CHANGE); break;
      case 3: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt3, CHANGE); break;
      case 4: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt4, CHANGE); break;
      case 5: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt5, CHANGE); break;
      case 6: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt6, CHANGE); break;
      case 7: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt7, CHANGE); break;
      case 8: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt8, CHANGE); break;
      case 9: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt9, CHANGE); break;
      case 10: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt10, CHANGE); break;
      case 11: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt11, CHANGE); break;
      case 12: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt12, CHANGE); break;
      case 13: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt13, CHANGE); break;
      case 14: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt14, CHANGE); break;
      case 15: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt15, CHANGE); break;
      case 16: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt16, CHANGE); break;
      case 17: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt17, CHANGE); break;
      case 18: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt18, CHANGE); break;
      case 19: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt19, CHANGE); break;
      case 20: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt20, CHANGE); break;
      case 21: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt21, CHANGE); break;
      case 22: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt22, CHANGE); break;
      case 23: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt23, CHANGE); break;
      case 24: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt24, CHANGE); break;
      case 25: attachInterrupt(digitalPinToInterrupt(digitalPins[i]), handleInterrupt25, CHANGE); break;
    }
  }
  Serial.println("Setup Finished");
}

void loop() {
  // Process any triggered events in the main loop
  for (int i = 0; i < numDigitalInputs; i++) {
    if (digitalTriggered[i]) {
      // Log the state change
      Serial.print("Pin:");
      Serial.print(digitalPins[i]);
      Serial.println(" Detected a Pulse (Rising followed by Falling)");
    }
  }
}
