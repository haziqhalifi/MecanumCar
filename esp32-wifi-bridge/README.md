# ESP32 Wireless Bridge — MecanumCar

Two firmware builds share this project. Pick one when flashing the ESP32:

| Build | Command | What it does |
|-------|---------|--------------|
| **Web dashboard** (default) | `pio run -e esp32dev -t upload` | ESP32 hosts the control dashboard on your WiFi. Open its IP in a browser — real-time control + telemetry over a WebSocket. **This is the wireless dashboard.** |
| Favoriot relay (legacy) | `pio run -e favoriot -t upload` | Old cloud path: relays telemetry to Favoriot over MQTT and polls for remote commands. |

## Wireless dashboard — how it works

```
browser  <--WebSocket-->  ESP32 web server  <--Serial2 (9600)-->  Uno (FinalSequence.cpp)
```

- The ESP32 joins your WiFi and serves `dashboard/dashboard-wifi.html` (embedded,
  gzipped, in `src/dashboard_html.h`) at `http://<esp32-ip>/`.
- The browser opens a WebSocket to `/ws`. Button presses send the same
  newline-terminated commands the USB dashboard used (`FWD:2`, `LFT`, `GRB:0`,
  `STOP`, `PATH:13`, …). The ESP32 forwards each to the Uno on Serial2.
- Every telemetry/log line the Uno prints is read back on Serial2 and pushed to
  all connected browsers instantly. No cloud, no polling lag.

## Wiring (ESP32 ↔ Uno)

The Uno talks over its hardware `Serial` (pins 0/1), same link the USB dashboard
used. Connect to the ESP32's Serial2, **plus a common GND**:

| Uno | direction | ESP32 |
|-----|-----------|-------|
| TX (pin 1) | → **level-shift 5V→3.3V** → | RX2 (GPIO16) |
| RX (pin 0) | ← (3.3V is fine) ← | TX2 (GPIO17) |
| GND | — | GND |

**Level-shift the Uno TX** (5V) down to the ESP32's 3.3V RX2 — a 2kΩ/3.3kΩ
divider or a logic-level shifter. ESP32 TX2 → Uno RX at 3.3V works directly.

⚠️ The Uno's pins 0/1 share its single UART with the USB serial port. For the
wireless setup, power the Uno **without** the USB serial monitor open, or you'll
have the ESP32 and your computer fighting over the same UART.

## First-time setup

1. Copy credentials: `cp include/secrets.h.example include/secrets.h`, then fill
   in `WIFI_SSID` / `WIFI_PASSWORD`. (For the web dashboard build, the Favoriot
   fields are ignored.)
2. Flash: `pio run -e esp32dev -t upload`
3. Open the serial monitor to see the assigned IP: `pio device monitor -b 115200`
   — look for `[http] dashboard live at http://192.168.x.x/`.
4. On any device on the same WiFi, open that IP (or `http://mecanumcar.local/`).
   The page auto-connects; the dot turns green when the WebSocket is live.

## Editing the dashboard

The served page is generated from `../dashboard/dashboard-wifi.html`. After
editing that file, regenerate the embedded header and re-flash:

```
python3 tools/gen_dashboard_header.py
pio run -e esp32dev -t upload
```

You can also open `dashboard/dashboard-wifi.html` directly as a file for a UI
preview; it will try to reach `ws://mecanumcar.local/ws`.
