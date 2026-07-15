// ESP32 wireless control bridge for the mecanum car (web dashboard build).
//
// The Uno (../../src/FinalSequence.cpp) has no WiFi. This sketch:
//   1. Joins your WiFi and serves a control dashboard (dashboard-wifi.html,
//      embedded gzipped in dashboard_html.h) on http://<esp32-ip>/ and
//      http://mecanumcar.local/.
//   2. Opens a WebSocket at /ws. Text frames from the browser are forwarded
//      verbatim to the Uno over Serial2 (the same newline-terminated commands
//      the USB dashboard sent: "FWD:2", "LFT", "GRB:0", "STOP", ...).
//   3. Reads the Uno's telemetry/log lines back from Serial2 and broadcasts
//      each line to every connected WebSocket client in real time.
//
// No cloud, no polling — commands and telemetry are ~milliseconds over the LAN.
//
// Wiring (Uno <-> ESP32, plus common GND):
//   Uno RX (pin 0 / USB-serial) is used by the dashboard link, so the Uno
//   talks to the ESP32 over its hardware Serial (same pins the USB dashboard
//   used). Connect:
//     Uno TX (pin 1) ---[divider]--> ESP32 RX2 (GPIO16)
//     Uno RX (pin 0) <-------------- ESP32 TX2 (GPIO17)
//   IMPORTANT: level-shift the Uno's 5V TX down to 3.3V for ESP32 RX2
//   (a 2k/3.3k divider, or a logic-level shifter). ESP32 TX2 -> Uno RX is fine
//   at 3.3V since the Uno reads >2.5V as HIGH.
//   The Uno's Serial runs at 9600 (FinalSequence.cpp Serial.begin(9600)), so
//   Serial2 here matches at 9600.
//
//   NOTE: if you keep the Uno plugged into USB for its own serial monitor, that
//   shares the Uno's single hardware UART with pins 0/1 — don't drive both at
//   once. For the wireless setup, power the Uno without USB serial and wire
//   pins 0/1 to the ESP32 as above.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include "secrets.h"
#include "dashboard_html.h"
#include "showcase_html.h"

// The Uno's Serial (FinalSequence.cpp) is 9600 baud.
#define UNO_BAUD 9600
#define RXD2 16   // ESP32 RX2  <- Uno TX (level-shifted to 3.3V)
#define TXD2 17   // ESP32 TX2  -> Uno RX
#define MDNS_HOST "mecanumcar"   // reachable at http://mecanumcar.local/

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Commands from WebSocket clients are captured in the async callback and
// flushed to the Uno from loop() so we never write Serial2 off the main task.
String pendingCmd = "";
volatile bool haveCmd = false;

// Assembles Serial2 bytes into lines to broadcast to the browser.
String unoLineBuf = "";

// Last [POS] line seen from the Uno, retained and replayed to each client as it
// connects. The Uno only emits [POS] when the pose CHANGES, but the dashboard's
// map needs it as STATE: a browser that wasn't connected at the instant a line
// went out has no way to ask for it, and falls back to its hardcoded home pose —
// so a dashboard opened or reloaded mid-run drew the car at home until it
// happened to cross the next junction. Retaining the pose here closes that
// window on connect. [POS] carries an absolute x,y,h (never a delta), so
// replaying a stale one is always safe: the next real line supersedes it.
String lastPosLine = "";

void broadcastLine(const String &line) {
  if (line.startsWith("[POS]")) lastPosLine = line;
  // WebSocket text frame per line; the browser splits on newlines anyway.
  ws.textAll(line);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[ws] client #%u connected from %s\n",
                    client->id(), client->remoteIP().toString().c_str());
      client->text("[dashboard] websocket connected to ESP32 bridge");
      // Seed the map with the pose the Uno last reported, so a dashboard opened
      // mid-run shows where the car actually is instead of the home default.
      if (lastPosLine.length()) client->text(lastPosLine);
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("[ws] client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo *)arg;
      // Only handle unfragmented, final text frames (commands are tiny).
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        String cmd;
        cmd.reserve(len);
        for (size_t i = 0; i < len; i++) cmd += (char)data[i];
        cmd.trim();
        if (cmd.length()) {
          pendingCmd = cmd;
          haveCmd = true;
        }
      }
      break;
    }
    default:
      break;
  }
}

void connectWiFi() {
  Serial.printf("\n[wifi] connecting to \"%s\"...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // lower latency for control
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\n[wifi] connected. IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(UNO_BAUD, SERIAL_8N1, RXD2, TXD2);

  connectWiFi();

  if (MDNS.begin(MDNS_HOST)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mdns] http://%s.local/\n", MDNS_HOST);
  } else {
    Serial.println("[mdns] failed to start");
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Client-facing showcase is the DEFAULT interface at "/": the story-driven
  // presentation view anyone opening the robot's IP should land on. Answer both
  // GET (browsers) and HEAD (health checkers / `curl -I`) so a HEAD probe 200s.
  auto sendShowcase = [](AsyncWebServerRequest *req) {
    AsyncWebServerResponse *res = req->beginResponse_P(
        200, "text/html; charset=utf-8", SHOWCASE_HTML_GZ, SHOWCASE_HTML_GZ_LEN);
    res->addHeader("Content-Encoding", "gzip");
    req->send(res);
  };
  server.on("/", HTTP_GET | HTTP_HEAD, sendShowcase);
  server.on("/showcase", HTTP_GET | HTTP_HEAD, sendShowcase);   // alias

  // Engineering dashboard — full manual control + debug, moved off the root.
  server.on("/engineering", HTTP_GET | HTTP_HEAD, [](AsyncWebServerRequest *req) {
    AsyncWebServerResponse *res = req->beginResponse_P(
        200, "text/html; charset=utf-8", DASHBOARD_HTML_GZ, DASHBOARD_HTML_GZ_LEN);
    res->addHeader("Content-Encoding", "gzip");
    req->send(res);
  });

  // Lightweight health check / IP echo.
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *req) {
    String j = "{\"ip\":\"" + WiFi.localIP().toString() +
               "\",\"clients\":" + String(ws.count()) +
               ",\"rssi\":" + String(WiFi.RSSI()) + "}";
    req->send(200, "application/json", j);
  });

  server.onNotFound([](AsyncWebServerRequest *req) {
    req->send(404, "text/plain", "not found");
  });

  server.begin();
  Serial.printf("[http] showcase (client view) live at http://%s/\n",
                WiFi.localIP().toString().c_str());
  Serial.printf("[http] engineering dashboard at http://%s/engineering\n",
                WiFi.localIP().toString().c_str());
}

void loop() {
  // Reconnect WiFi if it drops.
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  // 1) Flush any queued command from the browser to the Uno.
  if (haveCmd) {
    noInterrupts();
    String cmd = pendingCmd;
    haveCmd = false;
    interrupts();
    Serial.println("[cmd] -> Uno: " + cmd);
    Serial2.println(cmd);         // newline-terminated, matches Uno parser
  }

  // 2) Read telemetry/log lines from the Uno and broadcast each to clients.
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      unoLineBuf.trim();
      if (unoLineBuf.length()) {
        Serial.println("[uno] " + unoLineBuf);
        broadcastLine(unoLineBuf);
      }
      unoLineBuf = "";
    } else if (c != '\r') {
      unoLineBuf += c;
      if (unoLineBuf.length() > 240) {   // guard against a runaway line
        broadcastLine(unoLineBuf);
        unoLineBuf = "";
      }
    }
  }

  ws.cleanupClients();
}
