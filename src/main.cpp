#include <Arduino.h>
#include <Wire.h>

// Setup GPIOs for Digital Inputs (photodiode)
const int holePins[] = {4, 5, 18, 32, 13, 14, 15, 16, 17, 25, 23, 26, 27, 33};
const int numPin = sizeof(holePins) / sizeof(holePins[0]);
volatile bool pinTriggered[numPin] = {0};  // Tracks if the digital pin has been triggered
volatile unsigned long lastTriggerTime[numPin] = {0};  // Store last trigger time to manage debounce
const unsigned long debounceDelay = 40;  // Debounce time in microseconds for photodiode
volatile bool lastEdgeWasRising[numPin] = {false};  // Track the last detected edge for each pin

volatile bool i2cMasterDetected = false;
int i2cAddress = 0x08; // I2C address of the ESP32 slave

// GPIOs for buttons
const int buttonPin[] = {19, 35, 34, 39}; // GPIOs for the 4 buttons
const int numButton = sizeof(buttonPin) / sizeof(buttonPin[0]);
volatile bool buttonTriggered[numButton] = {0};  // Tracks if the digital pin has been triggered
volatile unsigned long lastButtonTriggerTime[numButton] = {0};  // Store last trigger time to manage debounce
volatile bool lastButtonEdgeWasRising[numButton] = {false};  // Track the last detected edge for each pin
const unsigned long buttonDebounceDelay = 200000; // 200ms debounce delay for the button (in microseconds)

// Function to handle requests from the master
void requestData() {
  i2cMasterDetected = true;
  bool dataSent = false;

  for (int i = 0; i < numPin; i++) {
      if (pinTriggered[i]) {
          Wire.write(holePins[i]);  // Send the pin number
          Wire.write(HIGH);  // Send the state as HIGH
          Serial.print("Sending holePin: ");
          Serial.println(holePins[i]);
          pinTriggered[i] = false;  // Reset the triggered state after being read
          dataSent = true;
          break; // Only send one state at a time
      }
  }

  if (!dataSent) {
      for (int i = 0; i < numButton; i++) {
          if (buttonTriggered[i]) {
              Wire.write(buttonPin[i]);  // Send the pin number
              Wire.write(HIGH);  // Send the state as HIGH
              Serial.print("Sending buttonPin: ");
              Serial.println(buttonPin[i]);
              buttonTriggered[i] = false;  // Reset the triggered state after being read
              dataSent = true;
              break; // Only send one state at a time
          }
      }
  }

  if (!dataSent) {
      // If no pin was triggered, send a neutral signal
      Wire.write(0xFF);  // Send a neutral pin number
      Wire.write(LOW);   // Send a neutral state (LOW)
  }
}

// Interrupt service routine for pin state change (photodiode detection)
void IRAM_ATTR handlePinInterrupt(int index) {
  unsigned long currentTime = micros();  // Use micros for better resolution

  if (currentTime - lastTriggerTime[index] > debounceDelay) {
    int pinState = digitalRead(holePins[index]);

    if (pinState == LOW && !lastEdgeWasRising[index]) {
      lastEdgeWasRising[index] = true;
    } else if (pinState == HIGH && lastEdgeWasRising[index]) {
      pinTriggered[index] = true;
      lastEdgeWasRising[index] = false;
    }

    lastTriggerTime[index] = currentTime;
  }
}

// Interrupt service routine for button press
void IRAM_ATTR handleButtonInterrupt(int index) {
  unsigned long currentTime = micros();  // Use micros for better resolution

  if (currentTime - lastButtonTriggerTime[index] > buttonDebounceDelay) {
    int pinState = digitalRead(buttonPin[index]);

    if (pinState == LOW && !lastButtonEdgeWasRising[index]) {
      lastButtonEdgeWasRising[index] = true;
    } else if (pinState == HIGH && lastButtonEdgeWasRising[index]) {
      buttonTriggered[index] = true;
      lastButtonEdgeWasRising[index] = false;
    }

    lastButtonTriggerTime[index] = currentTime;  // Corrected variable name
  }
}

// Generic interrupt handler for photodiode pins
void IRAM_ATTR handleGenericPinInterrupt(void* arg) {
    int index = (int)arg;  // Cast the argument to an integer
    handlePinInterrupt(index);
}

// Generic interrupt handler for buttons
void IRAM_ATTR handleGenericButtonInterrupt(void* arg) {
    int index = (int)arg;  // Cast the argument to an integer
    handleButtonInterrupt(index);
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C as slave
  Wire.begin(i2cAddress);
  Wire.onRequest(requestData);

  // Initialize Digital Input pins with pull-up resistors and attach interrupts
  for (int i = 0; i < numPin; i++) {
      pinMode(holePins[i], INPUT_PULLUP);
      attachInterruptArg(digitalPinToInterrupt(holePins[i]), handleGenericPinInterrupt, (void*)i, CHANGE);
  }

  for (int i = 0; i < numButton; i++) {
      pinMode(buttonPin[i], INPUT_PULLUP);
      attachInterruptArg(digitalPinToInterrupt(buttonPin[i]), handleGenericButtonInterrupt, (void*)i, CHANGE);
  }

  Serial.println("Setup Finished");
}

void loop() {
  // The main loop is empty since all the work is done in ISRs and I2C communication
}
