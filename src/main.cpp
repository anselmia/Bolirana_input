#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "yourSSID";        // replace with your WiFi network name
const char* password = "yourPassword"; // replace with your WiFi password

const char* serverName = "http://example.com/api/input"; // API endpoint to send data to

unsigned long lastHighCheck = 0;      // Timer to throttle output
const int checkInterval = 10000;      // Check every 10 seconds

// Setup GPIOs
const int inputPins[] = {4, 5, 13, 14, 17, 18, 19, 21};
const int numInputs = sizeof(inputPins) / sizeof(inputPins[0]);

void setup() {
  Serial.begin(115200);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize GPIOs as input with pull-down
  for (int i = 0; i < numInputs; i++) {
    pinMode(inputPins[i], INPUT_PULLDOWN);
  }
}

void loop() {
  if (millis() - lastHighCheck > checkInterval) {
    lastHighCheck = millis();

    // Check each input pin
    for (int i = 0; i < numInputs; i++) {
      if (digitalRead(inputPins[i]) == HIGH) {
        sendHighState(inputPins[i]); // Send high state to server
      }
    }
  }
}

void sendHighState(int pin) {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Sending high state for pin: ");
    Serial.println(pin);

    HTTPClient http;
    http.begin(serverName);  // Specify the URL
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    String httpRequestData = "pin=" + String(pin) + "&state=HIGH";
    int httpResponseCode = http.POST(httpRequestData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Response: " + response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("Not connected to WiFi");
  }
}
