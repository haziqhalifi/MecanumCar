// ESP32 WiFi bridge for the mecanum car.
//
// The Uno (../src/MainLatest.cpp) has no WiFi, so it sends one JSON
// telemetry line per state change (command start/finish, block progress,
// e-stop) over a SoftwareSerial link. This sketch relays each line to
// Favoriot over MQTT (mqtt.favoriot.com:1883) rather than plain HTTP POST,
// so the device shows as connected/online on the Favoriot dashboard — a
// stateless HTTP request per event never registers as a live connection.
//
// Wiring (matches lib/main.cpp's convention):
//   Uno pin 11 (RX) <- ESP32 pin 17 (TX2)
//   Uno pin 10 (TX) -> ESP32 pin 16 (RX2)
//   Common GND between Uno and ESP32.
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"

// Favoriot authenticates MQTT clients using the device's access token (not
// the account API key) as both username and password; the stream-publish
// topic is "<access_token>/v2/streams".
const char* MQTT_BROKER = "mqtt.favoriot.com";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = DEVICE_ACCESS_TOKEN "/v2/streams";

#define RXD2 16
#define TXD2 17

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void connectWiFi() {
  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
}

void connectMqtt() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to Favoriot MQTT broker...");
    String clientId = "esp32-mecanumcar-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (mqttClient.connect(clientId.c_str(), DEVICE_ACCESS_TOKEN, DEVICE_ACCESS_TOKEN)) {
      Serial.println("MQTT connected!");
    } else {
      Serial.print("MQTT connect failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 2s...");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  connectWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  // Default 256-byte buffer is too small once the API key (used in both the
  // topic and as username/password) is a ~140-char JWT plus the JSON payload.
  mqttClient.setBufferSize(512);
  connectMqtt();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqttClient.connected()) {
    connectMqtt();
  }
  mqttClient.loop();

  if (!Serial2.available()) return;

  String incomingData = Serial2.readStringUntil('\n');
  incomingData.trim();
  if (incomingData.length() == 0) return;

  Serial.println("Received from Uno: " + incomingData);

  // incomingData is already a JSON object, e.g.
  // {"command":"CMD_1","status":"running","sensor_left":0,"sensor_mid":1,"sensor_right":0,"block_count":2}
  // wrap it as the "data" field of the Favoriot stream payload.
  String jsonPayload = "{\"device_developer_id\":\"" + String(DEVICE_DEVELOPER_ID) +
                        "\",\"data\":" + incomingData + "}";

  Serial.println("Publishing to Favoriot...");
  if (mqttClient.publish(MQTT_TOPIC, jsonPayload.c_str())) {
    Serial.println("SUCCESS: Published to Favoriot!");
  } else {
    Serial.println("ERROR: MQTT publish failed.");
  }
}
