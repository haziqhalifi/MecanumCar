// ESP32 WiFi bridge for the mecanum car.
//
// The Uno (../src/MainLatest.cpp) has no WiFi, so it sends one JSON
// telemetry line per state change (command start/finish, block progress,
// e-stop) over a SoftwareSerial link. This sketch relays each line to
// Favoriot over MQTT (mqtt.favoriot.com:1883) rather than plain HTTP POST,
// so the device shows as connected/online on the Favoriot dashboard — a
// stateless HTTP request per event never registers as a live connection.
//
// It also polls Favoriot's REST API for a "remote_cmd" field (set by a
// Control widget on the dashboard) and forwards it to the Uno, so the
// dashboard can drive the car the same way the IR remote does. Polling
// rather than a push mechanism because Favoriot's MQTT "Send to Device"
// delivery path (RPC) has no documented REST/MQTT contract, and using the
// device's own access token for a second simultaneous MQTT subscriber
// caused the broker to fight over the session — polling avoids both.
//
// Wiring (matches lib/main.cpp's convention):
//   Uno pin 11 (RX) <- ESP32 pin 17 (TX2)
//   Uno pin 10 (TX) -> ESP32 pin 16 (RX2)
//   Common GND between Uno and ESP32.
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

// Favoriot authenticates MQTT clients using the device's access token (not
// the account API key) as both username and password; the stream-publish
// topic is "<access_token>/v2/streams".
const char* MQTT_BROKER = "mqtt.favoriot.com";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = DEVICE_ACCESS_TOKEN "/v2/streams";

// Reading streams back (as opposed to publishing them) uses the account API
// key over REST, not the device access token.
const char* FAVORIOT_STREAMS_ENDPOINT = "https://apiv2.favoriot.com/v2/streams";
const unsigned long REMOTE_POLL_INTERVAL_MS = 3000;

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

// Polls the latest few streams for this device and forwards any newly-seen
// "remote_cmd" field to the Uno. Skips acting on anything already present
// at boot (baseline set on the first poll) so old dashboard button presses
// from a previous session don't replay.
void pollRemoteCommand() {
  static unsigned long lastPollMs = 0;
  static long long lastProcessedTimestamp = -1;

  if (millis() - lastPollMs < REMOTE_POLL_INTERVAL_MS) return;
  lastPollMs = millis();

  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = String(FAVORIOT_STREAMS_ENDPOINT) +
               "?device_developer_id=" + DEVICE_DEVELOPER_ID + "&max=5&order=DESC";
  http.begin(url);
  http.addHeader("apikey", FAVORIOT_APIKEY);
  int code = http.GET();
  if (code != 200) {
    http.end();
    return;
  }
  String body = http.getString();
  http.end();

  JsonDocument doc;
  if (deserializeJson(doc, body)) return;

  JsonArray results = doc["results"].as<JsonArray>();
  if (results.size() == 0) return;

  if (lastProcessedTimestamp < 0) {
    lastProcessedTimestamp = results[0]["timestamp"].as<long long>();
    Serial.println("Remote command polling baseline set.");
    return;
  }

  // Results are newest-first; walk oldest-to-newest within this batch so
  // multiple queued commands dispatch to the Uno in the order they were sent.
  for (int i = results.size() - 1; i >= 0; i--) {
    JsonObject entry = results[i];
    long long ts = entry["timestamp"].as<long long>();
    if (ts <= lastProcessedTimestamp) continue;
    if (ts > lastProcessedTimestamp) lastProcessedTimestamp = ts;

    if (!entry["data"]["remote_cmd"].is<const char*>()) continue;
    const char* remoteCmd = entry["data"]["remote_cmd"];
    Serial.print("Remote command from dashboard: ");
    Serial.println(remoteCmd);
    Serial2.println(remoteCmd);
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
  pollRemoteCommand();

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
