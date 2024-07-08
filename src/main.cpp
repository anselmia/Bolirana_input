#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_wifi.h>

const char* ssid = "home";  // Replace with your WiFi network name
const char* password = ""; // Replace with your WiFi password
// Static IP address configuration
IPAddress local_IP(192,168,1,50);  // Set your desired static IP address
IPAddress gateway(192,168,1,254);    // Set your network gateway (usually your router's IP)
IPAddress subnet(255,255,255,0);   // Set your network subnet
IPAddress primaryDNS(192,1,168,174);     // Optional: Set a DNS server
IPAddress secondaryDNS(192,168,1,254);   // Optional: Set a secondary DNS server
const char* serverName = "http://192.168.1.21:5000/receive_data";

// Setup GPIOs
const int inputPins[] = {4, 5, 13, 14, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32};
const int numInputs = sizeof(inputPins) / sizeof(inputPins[0]);
bool lastState[numInputs] = {0};  // Tracks the last state of each pin
unsigned long lastDebounceTime[numInputs] = {0};
const unsigned long debounceDelay = 50;  // Debounce time in milliseconds

void WiFiEvent(WiFiEvent_t event) {
  Serial.println("[WiFi-event] event: " + event);
  switch (event) {
    case SYSTEM_EVENT_WIFI_READY:
      Serial.println("WiFi interface ready");
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("WiFi client started");
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.println("WiFi clients stopped");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to access point");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
      Serial.println("Authentication mode of access point has changed");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Obtained IP address: " + WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println("Lost IP address and IP address is reset to 0");
      //      vTaskDelay( 5000 );
      //      ESP.restart();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
      //      ESP.restart();
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
      break;
    case SYSTEM_EVENT_AP_START:
      Serial.println("WiFi access point started");
      break;
    case SYSTEM_EVENT_AP_STOP:
      Serial.println("WiFi access point  stopped");
      //      WiFi.mode( WIFI_OFF);
      //      esp_sleep_enable_timer_wakeup( 1000000 * 2 ); // 1 second times how many seconds wanted
      //      esp_deep_sleep_start();
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("Client connected");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("WiFi client disconnected");
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      Serial.println("Assigned IP address to client");
      break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      Serial.println("Received probe request");
      break;
    case SYSTEM_EVENT_GOT_IP6:
      Serial.println("IPv6 is preferred");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.println("Obtained IP address");
      break;
    default: break;
  }
}

void connectToWiFi() {
  int TryCount = 0;
  Serial.println("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    TryCount++;
    WiFi.disconnect();

    WiFi.mode(WIFI_STA);
    WiFi.enableSTA(true);

    WiFi.begin(ssid, password);
    Serial.print(TryCount);
    Serial.print(" ");

    delay(2000);
    if (TryCount == 100) {
      Serial.println("");
      Serial.println("Restarting");
      ESP.restart();
    }
  }
  WiFi.onEvent(WiFiEvent);
}

void get_network_info() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[*] Network information for ");
    Serial.println(ssid);

    Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
    Serial.print("[+] Gateway IP : ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("[+] Subnet Mask : ");
    Serial.println(WiFi.subnetMask());
    Serial.println((String) "[+] RSSI : " + WiFi.RSSI() + " dB");
    Serial.print("[+] ESP32 IP : ");
    Serial.println(WiFi.localIP());
  }
}

void setup() {
  Serial.begin(115200);
  // Configure static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
   
  //Wifi
  connectToWiFi();

  Serial.println("Connected to the WiFi network");
  get_network_info();

  // Initialize GPIOs as inputs with pull-down
  for (int i = 0; i < numInputs; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
  }
}

void sendState(int pin, String state) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);  // Specify the URL
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    String httpRequestData = "pin=" + String(pin) + "&state=" + state;
    int httpResponseCode = http.POST(httpRequestData);
    Serial.print("HTTP Response code:");
    Serial.println(String(httpResponseCode));
    if (httpResponseCode > 0) {
      String response = http.getString();
    } 
    http.end();
  } else {
    Serial.println("Not connected to WiFi");
  }
}

void loop() {
  for (int i = 0; i < numInputs; i++) {
    int currentState = digitalRead(inputPins[i]);
    if (currentState != lastState[i] && (millis() - lastDebounceTime[i] > debounceDelay)) {
      lastDebounceTime[i] = millis();  // Reset debounce timer
      lastState[i] = currentState;
      sendState(inputPins[i], currentState ? "HIGH" : "LOW");
    }
  }
  delay(10);
}

