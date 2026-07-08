#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ==========================================
// WIFI & FAVORIOT CREDENTIALS 
// ==========================================
const char* ssid     = "ASEM Training";
const char* password = "Class@Asem";

// Favoriot API Settings
const char* favoriot_endpoint = "https://apiv2.favoriot.com/v2/streams";
const char* favoriot_apikey = "y3UHRLYY2a3EPGz2ZI2ijVr3MUGBD8YE";
const char* device_developer_id = "ultrasonicsensor@lambochelsea";

// Hardware Serial 2 Pins (Connected to Arduino Uno)
#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  // Ensure your Arduino Uno code also uses 115200 baud for Serial.begin()
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
}

void loop() {
  // Check if the Arduino has sent any data
  if (Serial2.available()) {
    String incomingData = Serial2.readStringUntil('\n');
    incomingData.trim();
    
    if (incomingData.length() > 0) {
      Serial.println("Received from Arduino: " + incomingData);

      // Send to Favoriot if WiFi is connected
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(favoriot_endpoint);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("apikey", favoriot_apikey);
        
        // Build JSON payload
        // NOTE: Ensure 'distance' matches the Data Stream name in your Favoriot dashboard
// Replace the payload building line in your ESP32 code with this:
String jsonPayload = "{\"device_developer_id\":\"" + String(device_developer_id) + "\",\"data\":{\"distance\":" + incomingData + "}}";        
        Serial.println("Sending to Favoriot...");
        int httpResponseCode = http.POST(jsonPayload);
        
        // Check Response
        if (httpResponseCode > 0) {
          if (httpResponseCode == 200 || httpResponseCode == 201) {
            Serial.println("SUCCESS: Data accepted by Favoriot!");
          } else {
            Serial.print("ERROR: Favoriot returned code: ");
            Serial.println(httpResponseCode);
          }
        } else {
          Serial.print("CONNECTION ERROR: ");
          Serial.println(http.errorToString(httpResponseCode).c_str());
        }
        
        http.end(); // IMPORTANT: Free resources
      } else {
        Serial.println("WiFi not connected. Skipping upload.");
      }
    }
  }
}