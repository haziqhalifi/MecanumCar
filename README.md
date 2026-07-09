# Mecanum Car — Logic Overview

Firmware for a KS0560 mecanum-wheel car that follows a line grid and executes
IR-remote-triggered movement sequences, with live telemetry and remote control
via Favoriot. Main entry point: [src/MainLatest.cpp](src/MainLatest.cpp).

## Hardware

- 3 line sensors: `SENSOR_LEFT` (A0), `SENSOR_MID` (A1), `SENSOR_RIGHT` (A2) — digital, HIGH when over a line.
- IR receiver on `RECV_PIN` (A3), decoded with IRremote.
- 4 mecanum wheels driven independently (upper-left, lower-left, upper-right, lower-right), controlled via the `mecanumCar` library (`car.Advance()`, `car.Turn_Right()`, `car.L_Move()`, etc., bit-banged I2C on pins 2/3) and four global PWM speed variables (`speed_Upper_L/R`, `speed_Lower_L/R`).
- `SoftwareSerial` link to a companion ESP32 board on pins 10 (TX) / 11 (RX), separate from the USB debug `Serial` — see [Favoriot telemetry & remote control](#favoriot-telemetry--remote-control) below.
- TCS3200 color sensor (`TCS_OUT_PIN` 8, `TCS_S2_PIN` 7, `TCS_S3_PIN` 6), HC-SR04 ultrasonic (`ULTRASONIC_TRIG_PIN` 12, `ULTRASONIC_ECHO_PIN` 13), and a gripper servo (`GRIPPER_PIN` 9) — see [Grab before turning](#grab-before-turning) below. **The servo needs its own 5-6V supply with a common ground** — the Arduino 5V pin can't source a servo's stall current, and trying to can brown out the board (freezes Serial/everything).

## Core movement primitives

- **`moveForwardBlocks(targetBlocks, startSpeed, endOffsetMs)`** — line-follows forward, counting "blocks" as crossings where all 3 sensors go HIGH together (a junction), then LOW again. Steers by reacting to which sensor combination is currently active (drifted left/right, off-line, etc.). Slows down slightly (`-5` PWM per block already crossed, floored at `SPEED_MIN`) as it approaches the target block count. Returns the final block count reached. Used by Button 1, Button 2, Button 3, Button 180, and every forward leg of the sequences — Button 2 and Button 3 share this exact same detection logic (the track has full 3-sensor junctions on both sides).
- **`strafeLeftBlocks` / `strafeRightBlocks`** — sideways motion (`car.L_Move()` / `car.R_Move()`), counting junctions via a single edge sensor (left or right respectively) instead of the full 3-sensor pattern. Uses per-wheel calibrated speeds (see [Tuning knobs](#tuning-knobs)) to correct sideways drift. Returns the final count.
- **`rotateRight90` / `rotateLeft90`** — see [Turning](#turning-90) below. Return `false` early if interrupted by E-STOP.
- **`rotate180`** — two chained `rotateRight90()` calls with a short pause between them. Always pivots via two *right* rotations regardless of which button/sequence calls it (not mirrored for left-triggered sequences) — this is a shared implementation detail, not a bug.

Each of these functions polls `IrReceiver` for `CMD_STAR` (E-STOP) on every loop iteration so a command in progress can always be aborted — handled via `checkEstop()` / `reportEstop()`.

## Turning (90°)

Stopping a turn is decided by the sensors, not a fixed timer. Timing is only used to blindly clear the line the car started on. Sequence for `rotateRight90()` (mirrored for left, using `SENSOR_LEFT` instead of `SENSOR_RIGHT`):

1. **Clear start line** (sensor-driven): if already parked on a line, keep turning until that sensor reads LOW.
2. **Blind turn** (timing-driven): turn for a fixed `TURN_BLIND_MS` regardless of sensor state, so the sensor doesn't immediately re-trigger on the line it just left.
3. **Approach next line** (sensor-driven): turn at full `SPEED_ROTATE` until the edge sensor (`SENSOR_RIGHT` for a right turn) first reads HIGH.
4. **Center on line** (`centerOnLine()`, sensor-driven): switch to a slow creep (`TURN_CREEP_SPEED`) and keep turning until `SENSOR_MID` also reads HIGH, debounced for `TURN_CENTER_DEBOUNCE_MS` to reject noise. This stops the car centered on the line rather than the instant a single edge sensor sees it, which is more repeatable turn-to-turn. Falls back to stopping anyway after `TURN_CENTER_TIMEOUT_MS` if the middle sensor never confirms (e.g. dirty/misaligned sensor), so a bad reading can't hang the car indefinitely.

`rotate180()` just runs this twice with a pause in between.

## IR command map

| Command | Code | Action |
|---|---|---|
| Digit 0 (`CMD_STAR`) | 0x52 | Emergency stop |
| Digit 1 (`CMD_1`) | 0x16 | Forward `TARGET_BLOCKS_CMD1` blocks |
| Digit 2 (`CMD_2`) | 0x19 | Rotate right, then forward `TARGET_BLOCKS_CMD2` blocks |
| Digit 3 (`CMD_3`) | 0x0D | Rotate left, then forward `TARGET_BLOCKS_CMD3` blocks |
| Digit 4 (`CMD_SEQ1`) | 0x0C | Sequence 1: fwd 4 → left → fwd 2 → right → fwd 2 → 180° |
| Digit 5 (`CMD_SEQ2`) | 0x18 | Sequence 2: Sequence 1, then mirrored in reverse back to start |
| Digit 6 (`CMD_180`) | 0x5E | 180° turn, then forward `TARGET_BLOCKS_CMD180` blocks |
| Digit 7 (`CMD_7`) | 0x08 | Strafe left, counted via `SENSOR_LEFT` |
| Digit 8 (`CMD_180_ONLY`) | 0x1C | 180° turn only, no forward move |
| Digit 9 (`CMD_9`) | 0x5A | Strafe right, counted via `SENSOR_RIGHT` |
| Arrow UP (`CMD_UP`) | 0x46 | Sequence 3: fwd 5 → 180° → fwd 5 |
| Arrow PREV (`CMD_ROTATE_L`) | 0x44 | Same as Digit 5 (Sequence 2) |
| Arrow PLAY/PAUSE (`CMD_ROTATE_R`) | 0x43 | Sequence 4: Sequence 2 mirrored (every left turn becomes a right turn and vice versa) |

`rotateRight90()`/`rotateLeft90()` are properly mirrored (direction, speed constant, and offset constant all swap correctly leg-by-leg between Sequence 2 and Sequence 4). The one shared, non-mirrored detail is `rotate180()` always pivoting via two right-rotations regardless of which sequence calls it — see [Core movement primitives](#core-movement-primitives).

## Grab before turning

Sequences 2, 3, and 4 (Digit 5 / Arrow PREV, Arrow UP, Arrow PLAY-PAUSE) each
call `grabBlockInFront()` right before their `rotate180()` — **Sequence
1 (Digit 4) deliberately does not**, since the request that added this only
covered the Up/Left/Right arrow buttons. Note `runSequence2()` is shared by
both Digit 5 and Arrow PREV (`CMD_ROTATE_L`) — since they run the exact same
function, Digit 5 also grabs now, as a side effect of that sharing.

`grabBlockInFront()` grabs whatever is in front **regardless of color** —
the color-conditional gate (originally BLUE-only) was removed on request:
1. Reads the TCS3200 color sensor (`classifyColor()`) and logs the result — diagnostic only, doesn't affect whether the grab happens.
2. Always calls `approachAndGrab()`: creeps forward at `SPEED_MIN` while polling the HC-SR04 (`readUltrasonicDistanceCM()`) until within `APPROACH_STOP_DISTANCE_CM`, then closes the gripper to `GRIPPER_GRAB_US` via `gripperMoveTo()` (moves gradually, `GRIPPER_MOVE_STEP` per `GRIPPER_MOVE_DT`, to avoid a current-draw spike). Gives up after `APPROACH_TIMEOUT_MS` with no grab if nothing is ever in range (e.g. no block present).
   - **HC-SR04 blind zone**: most ultrasonic sensors can't get a valid echo below roughly 2-5cm (ping and return overlap), which was causing the car to blindly keep advancing into the block instead of stopping, since a "no echo" reading never satisfied the stop condition. Fixed two ways: `APPROACH_STOP_DISTANCE_CM` is set to `6.0` (above the typical blind zone, so a clean reading usually triggers the stop first), and the loop now **stops advancing** (rather than continuing to drive) on any no-echo reading — after `NO_ECHO_STREAK_TO_GRAB` (`3`) consecutive no-echo reads while approaching, it assumes the block is too close for the sensor to read and grabs anyway, rather than waiting out the full timeout or crashing into it.

This reuses the existing `checkEstop()`/`reportEstop()` E-STOP plumbing (not a
separate IR-polling loop), so a `CMD_STAR` press aborts the approach exactly
like it aborts any other movement.

### Autonomous grab while idle

`checkBackgroundGrab()` is polled from `loop()` on every iteration (throttled
to once per `BACKGROUND_GRAB_CHECK_MS` = `500`ms internally) — since movement
functions block until they finish, this only ever actually runs while the car
is genuinely idle between commands, not mid-sequence. If the ultrasonic
already reads within `APPROACH_STOP_DISTANCE_CM` **and** the gripper isn't
already holding something (`gripperPosUs == GRIPPER_OPEN_US`), it grabs —
**no button press required**.

Deliberately scoped to "already close," not "detected at any distance," so
the car doesn't autonomously drive across the room toward something it
merely sees — it only reacts to a block already right in front of it.

The `#` button (`CMD_HASH`, code `0x4A` — captured directly off this remote,
not assumed) releases the gripper (`gripperMoveTo(GRIPPER_OPEN_US)`), which
also resets `gripperPosUs` back to `GRIPPER_OPEN_US` and re-arms
`checkBackgroundGrab()` for another autonomous grab.

## Command dispatch

`dispatchCommand(cmd)` is the single shared entry point for running any command — it's called identically whether the command came from a real IR remote press (`loop()`, after `IrReceiver.decode()`) or from a Favoriot dashboard button (`checkRemoteCommand()`, see below). It:

1. Sets `currentCommandName` (via `cmdName(cmd)`) and sends a `"running"` telemetry event.
2. Runs the matching movement function/sequence, capturing the final block count where applicable (`moveForwardBlocks`/`strafeLeftBlocks`/`strafeRightBlocks` return it; sequences don't, since they involve multiple legs).
3. Sends a final `"idle"` (or `"estop"` for `CMD_STAR`) telemetry event.

This shared-dispatch design is what let remote control (dashboard → car) reuse all the exact same movement logic as the physical IR remote, instead of duplicating it.

## Tuning knobs

- **Speeds**: `SPEED_START_FAST`, `SPEED_POST_RIGHT_TURN`, `SPEED_POST_LEFT_TURN`, `SPEED_POST_180_TURN` (all `60`), `SPEED_STRAFE_LEFT/RIGHT` (`55`), `SPEED_ROTATE` (`60`), `SPEED_MIN` (`35`, floor for dynamic slowdown), `TURN_CREEP_SPEED` (`40`, centering creep).
- **Strafe calibration**: `STRAFE_L_BIAS_*` / `STRAFE_R_BIAS_*` — per-wheel PWM offsets to correct sideways drift, independently tuned per side (not mirrored values — they compensate for real mechanical differences between the wheels).
- **Timing offsets** (`OFFSET_*_MS`): extra forward "nudge" after a maneuver finishes, to compensate for the line sensors being physically mounted ahead of the wheel axles. Because `SoftwareSerial` blocks for ~80–100ms per telemetry send, **these offsets are only applied once per movement call (start/end), never per-junction mid-loop** — see [Favoriot telemetry](#favoriot-telemetry--remote-control) for why that matters.
  - `OFFSET_CMD1_MS` (`250`) — Button 1, and legs before a 90° turn that start at `SPEED_START_FAST`.
  - `OFFSET_POST_RIGHT_TURN_MS` (`220`) / `OFFSET_POST_LEFT_TURN_MS` (`200`) — nudge after approaching at `SPEED_POST_RIGHT/LEFT_TURN`; also used pre-turn in sequences.
  - `OFFSET_POST_180_MS` (`220`) — nudge after a 180° turn's subsequent forward leg.
  - `OFFSET_PRE_180_MS` (`110`) — **kept deliberately separate** from the 90°-turn offsets above: the leg immediately before any `rotate180()` call needs a *smaller* nudge, since the same value used for 90°-turn approaches was found to overshoot the junction and cause the car to over-rotate on the 180°.
  - `OFFSET_STRAFE_L/R_MS` (`120`) — nudge after strafe junction detection.
  - `TURN_BLIND_MS`, `TURN_CENTER_DEBOUNCE_MS`, `TURN_CENTER_TIMEOUT_MS` — turn timing (see [Turning](#turning-90)).
- **Block counts**: `TARGET_BLOCKS_*`, `TARGET_STRAFE_CMD7/9` — how far each command travels.

All of the above are physical/empirical tuning values — they were calibrated against the actual track and wheel hardware, so changing them requires re-testing on the real car, not just re-flashing.

## Favoriot telemetry & remote control

The Uno has no WiFi, so both telemetry and remote commands go over the dedicated
`SoftwareSerial` link (pins 10/11) to a companion ESP32 board running
[esp32-wifi-bridge/](esp32-wifi-bridge/). See that folder's
`include/secrets.h.example` for WiFi/API credentials setup (real `secrets.h`
is gitignored).

### Outgoing: telemetry (Uno → ESP32 → Favoriot)

`sendTelemetry(status, blockCount)` sends one JSON line per state change —
command start, command finish, and e-stop — **not per junction/block
progress**. Junction-by-junction telemetry was tried and removed: `sendTelemetry()`
blocks for ~80–100ms per call over `SoftwareSerial`, and calling it inside the
tight junction-detection loop caused the car to keep driving unmodulated
during that block, degrading line-following accuracy. `block_count` in the
final telemetry event instead reports the *final* count reached by that
movement call (`-1` when not applicable, e.g. sequences or command start).

```json
{"command":"CMD_1","status":"idle","sensor_left":0,"sensor_mid":1,"sensor_right":0,"block_count":4}
```

`status` is one of `running`, `idle`, `estop`. The ESP32 relays each line to
Favoriot over MQTT (not HTTP POST — MQTT keeps the device showing as
"connected" on the Favoriot dashboard).

### Incoming: remote control (Favoriot dashboard → ESP32 → Uno)

The ESP32 polls Favoriot's REST API for a `remote_cmd` field (set by a
dashboard Control widget) and forwards the value as a plain text line (e.g.
`CMD_1`) over the same serial link. `checkRemoteCommand()` — called every
`loop()` iteration, before checking the IR receiver — reads that line, maps
it back to a command byte via `cmdFromName()` (the inverse of `cmdName()`),
and calls `dispatchCommand()` exactly as if the button had been pressed on
the physical remote. Unrecognized command names are logged and ignored.

Polling (rather than a push mechanism) was used deliberately: Favoriot's
"Send to Device" delivery path has no documented REST/MQTT contract, and
using the device's own access token for a second simultaneous MQTT
subscriber caused the broker to fight over the session.

`lib/WifiToEsp32.cpp` is a leftover sketch from an earlier ultrasonic-sensor
project and is superseded by `esp32-wifi-bridge/` for this project.

## Diagnostics

`SENSOR_DIAGNOSTIC_MODE` (near `loop()`) — when set to `true`, prints `L=`, `M=`, `R=` sensor readings every 200ms to Serial instead of running normally. Useful for comparing how cleanly each sensor triggers when manually sliding the car over a line (e.g. to check if one edge sensor is dirtier/misaligned relative to the others). Set back to `false` for normal operation.
