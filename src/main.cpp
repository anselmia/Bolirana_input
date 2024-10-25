#include <Arduino.h>
#include <Wire.h>

// Setup GPIOs for Digital Inputs (photodiode)
const int holePins[] = {4, 5, 18, 32, 13, 14, 15, 16, 17, 25, 23, 26, 27, 33 };
const int numPin = sizeof(holePins) / sizeof(holePins[0]);
volatile bool pinTriggered[numPin] = {0};  // Tracks if the digital pin has been triggered
volatile unsigned long lastTriggerTime[numPin] = {0};  // Store last trigger time to manage debounce
const unsigned long debounceDelay = 50;  // Debounce time in microseconds for photodiode
volatile bool lastEdgeWasRising[numPin] = {false};  // Track the last detected edge for each pin

volatile bool i2cMasterDetected = false;
int i2cAddress = 0x08;  // I2C address of the ESP32 slave

// GPIOs for buttons
const int buttonPin[] = { 19, 0, 12 };  // GPIOs for the buttons
const int numButton = sizeof(buttonPin) / sizeof(buttonPin[0]);
volatile bool buttonTriggered[numButton] = { 0 };  // Tracks if the button has been triggered
unsigned long lastButtonTriggerTime[numButton] = { 0 };  // Store last trigger time to manage debounce
const unsigned long buttonDebounceDelay = 100000;  // 100ms debounce delay
int lastButtonState[numButton] = { HIGH };  // Store the last stable state
int currentButtonState[numButton];  // Store the current read state

const int delayStartTime = 30000;  // Delay time in milliseconds

// Function to handle requests from the master
void requestData() {
    bool dataSent = false;

    for (int i = 0; i < numPin; i++) {
        if (pinTriggered[i]) {
            Wire.write(holePins[i]);  // Send the pin number
            Wire.write(HIGH);  // Send the state as HIGH
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

void IRAM_ATTR handlePinInterrupt2(int index) {
    pinTriggered[index] = true; // Set triggered only when pin goes HIGH}
} 


// Generic interrupt handler for photodiode pins
void IRAM_ATTR handleGenericPinInterrupt(void* arg) {
    int index = (int)arg;  // Cast the argument to an integer
    handlePinInterrupt(index);
}

void IRAM_ATTR handleNewSensorPinInterrupt(void* arg) {
    int index = (int)arg;  // Cast the argument to an integer
    handlePinInterrupt2(index);
}


void setup() {
    Serial.begin(115200);
    delay(delayStartTime);  // Delay for testing (reduce this for real application)

    // Initialize I2C as slave
    Wire.begin(i2cAddress);
    Wire.onRequest(requestData);

    pinMode(holePins[0], INPUT_PULLDOWN);
    attachInterruptArg(digitalPinToInterrupt(holePins[0]), handleNewSensorPinInterrupt, (void*)0, RISING);
    // Initialize Digital Input pins with pull-up resistors and attach interrupts
    for (int i = 0; i < numPin; i++) {
        pinMode(holePins[i], INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(holePins[i]), handleGenericPinInterrupt, (void*)i, CHANGE);
    }

    // Initialize button pins with pull-up resistors
    for (int i = 0; i < numButton; i++) {
        pinMode(buttonPin[i], INPUT_PULLUP);
    }

    Serial.println("Setup Finished");
}

void loop() {
    // Debounce and check buttons
    for (int i = 0; i < numButton; i++) {
        int reading = digitalRead(buttonPin[i]);

        // If the current reading is different from the last stable state, reset the timer
        if (reading != lastButtonState[i]) {
            lastButtonTriggerTime[i] = micros();
        }

        // If the state has been stable for longer than the debounce delay, consider it as the new state
        if ((micros() - lastButtonTriggerTime[i]) > buttonDebounceDelay) {
            if (reading != currentButtonState[i]) {
                currentButtonState[i] = reading;

                // Only trigger on a falling edge (button press)
                if (currentButtonState[i] == LOW) {
                    buttonTriggered[i] = true;
                }
            }
        }

        lastButtonState[i] = reading;  // Update the last stable state
    }
}
