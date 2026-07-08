# Mecanum Car — Logic Overview

Firmware for a KS0560 mecanum-wheel car that follows a line grid and executes
IR-remote-triggered movement sequences. Main entry point: [src/MainLatest.cpp](src/MainLatest.cpp).

## Hardware

- 3 line sensors: `SENSOR_LEFT` (A0), `SENSOR_MID` (A1), `SENSOR_RIGHT` (A2) — digital, HIGH when over a line.
- IR receiver on `RECV_PIN` (A3), decoded with IRremote.
- 4 mecanum wheels driven independently (upper-left, lower-left, upper-right, lower-right), controlled via the `mecanumCar` library (`car.Advance()`, `car.Turn_Right()`, `car.L_Move()`, etc.) and four global PWM speed variables (`speed_Upper_L/R`, `speed_Lower_L/R`).

## Core movement primitives

- **`moveForwardBlocks(targetBlocks, startSpeed, endOffsetMs)`** — line-follows forward, counting "blocks" as crossings where all 3 sensors go HIGH together (a junction), then LOW again. Steers by reacting to which sensor combination is currently active (drifted left/right, off-line, etc.). Slows down slightly as it approaches the target block count.
- **`moveRightSideMarkers(...)`** — same idea, but counts junctions using `M+R` instead of all three sensors (used after a left turn, where junction markers only exist on the right side of the path).
- **`strafeLeftBlocks` / `strafeRightBlocks`** — sideways motion (`car.L_Move()` / `car.R_Move()`), counting junctions via a single edge sensor (left or right respectively) instead of the full 3-sensor pattern.
- **`rotateRight90` / `rotateLeft90`** — see [Turning](#turning) below.
- **`rotate180`** — two chained `rotateRight90()` calls with a short pause between them.

Each of these functions polls `IrReceiver` for `CMD_STAR` (E-STOP) on every loop iteration so a command in progress can always be aborted.

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
| Digit 3 (`CMD_3`) | 0x0D | Rotate left, then forward `TARGET_MARKERS_CMD3` right-side markers |
| Digit 4 (`CMD_SEQ1`) | 0x0C | Sequence 1: fwd 4 → left → fwd 2 → right → fwd 2 → 180° |
| Digit 5 (`CMD_SEQ2`) | 0x18 | Sequence 2: Sequence 1, then mirrored in reverse back to start |
| Digit 6 (`CMD_180`) | 0x5E | 180° turn, then forward `TARGET_BLOCKS_CMD180` blocks |
| Digit 7 (`CMD_7`) | 0x08 | Strafe left, counted via `SENSOR_LEFT` |
| Digit 8 (`CMD_180_ONLY`) | 0x1C | 180° turn only, no forward move |
| Digit 9 (`CMD_9`) | 0x5A | Strafe right, counted via `SENSOR_RIGHT` |
| Arrow UP (`CMD_UP`) | 0x46 | Sequence 3: fwd 5 → 180° → fwd 5 |
| Arrow PREV (`CMD_ROTATE_L`) | 0x44 | Same as Digit 5 (Sequence 2) |
| Arrow PLAY/PAUSE (`CMD_ROTATE_R`) | 0x43 | Sequence 4: Sequence 2 mirrored (every left turn becomes a right turn and vice versa) |

## Tuning knobs

- **Speeds**: `SPEED_START_FAST`, `SPEED_POST_RIGHT_TURN`, `SPEED_POST_LEFT_TURN`, `SPEED_POST_180_TURN`, `SPEED_STRAFE_LEFT/RIGHT`, `SPEED_ROTATE`, `SPEED_MIN` (floor for dynamic slowdown), `TURN_CREEP_SPEED` (centering creep).
- **Strafe calibration**: `STRAFE_L_BIAS_*` / `STRAFE_R_BIAS_*` — per-wheel PWM offsets to correct sideways drift.
- **Timing offsets**: `OFFSET_*_MS` — extra forward nudge after a maneuver finishes, to compensate for sensor-to-wheel-axle offset. `TURN_BLIND_MS`, `TURN_CENTER_DEBOUNCE_MS`, `TURN_CENTER_TIMEOUT_MS` — turn timing (see [Turning](#turning)).
- **Block/marker counts**: `TARGET_BLOCKS_*`, `TARGET_MARKERS_CMD3`, `TARGET_STRAFE_CMD7/9` — how far each command travels.

## Diagnostics

`SENSOR_DIAGNOSTIC_MODE` (near `loop()`) — when set to `true`, prints `L=`, `M=`, `R=` sensor readings every 200ms to Serial instead of running normally. Useful for comparing how cleanly each sensor triggers when manually sliding the car over a line (e.g. to check if one edge sensor is dirtier/misaligned relative to the others). Set back to `false` for normal operation.
