# Testbasic — Grid-Based Pickup/Drop Firmware

Firmware variant for a mecanum-wheel car that line-follows a grid, picks up a
colored block at a scripted location, and carries it back to a drop zone.
Entry point: [Testbasic.cpp](Testbasic.cpp). Controlled entirely by IR remote
— nothing runs autonomously until a button is pressed.

## Hardware

- 3 line sensors: `LINE_LEFT_PIN` (A0), `LINE_CENTER_PIN` (A1), `LINE_RIGHT_PIN` (A2) — digital, HIGH when over a line.
- HC-SR04 ultrasonic: `ULTRASONIC_TRIG_PIN` (12), `ULTRASONIC_ECHO_PIN` (13).
- TCS3200 color sensor: `COLOR_S0_PIN` (4), `COLOR_S1_PIN` (5), `COLOR_S2_PIN` (7), `COLOR_S3_PIN` (6), `COLOR_OUT_PIN` (8).
- Claw servo on `CLAW_SERVO_PIN` (9), moved between `CLAW_OPEN_ANGLE` (20°) and `CLAW_CLOSED_ANGLE` (100°).
- IR receiver on `IR_RECEIVE_PIN` (A3), decoded with IRremote.
- 4 mecanum wheels via the `mecanumCar` library (`mecCar.Advance()`, `mecCar.Turn_Right()`, etc., bit-banged I2C on pins 2/3) and four global PWM speed variables (`speed_Upper_L/R`, `speed_Lower_L/R`).

## Setup

1. Serial starts at 115200 baud.
2. Line/ultrasonic/color sensor pins are configured.
3. The claw opens once (`openClawWithAttach()`) then detaches, so it doesn't hold torque idle.
4. The drive board initializes and default speed (`SPEED_START_FAST`) is set.
5. IR receiver starts listening. The car then sits idle — `loop()` does nothing until a button is pressed.

## Core movement primitives

- **`gridMoveForwardBlocks(targetBlocks, startSpeed)`** — line-follows forward, counting a "block" each time all 3 sensors go HIGH together (a junction) after being off it. Slows by `-5` PWM per block already crossed, floored at `SPEED_MIN`. After the target is reached, creeps forward an extra `CENTER_OFFSET_MS` to settle centered on the junction, then stops.
- **`gridMoveForwardOneCoord(startSpeed)`** — same idea but for exactly one grid coordinate step (used by the reconstructed grid-navigation helpers below): drives blind for 140ms to clear the current junction, then line-follows until the next full 3-sensor junction, then applies the same `CENTER_OFFSET_MS` settle.
- **`gridRotateLeft90` / `gridRotateRight90`** — sensor-terminated 90° pivot (see [Turning](#turning-90)). Return `false` early if interrupted by E-STOP.
- **`gridSlightReverse()`** — brief reverse bump (`REVERSE_BUMP_MS` at `SPEED_REVERSE_BUMP`) used to back off the line right after a grab, before turning.
- **`gridStop()`** — stops the drive and logs it.

Every blocking loop polls `checkEmergencyStop()` each iteration, so a `CMD_STAR` press can abort mid-maneuver.

## Turning (90°)

`gridRotateLeft90()` (mirrored for right, using `LINE_RIGHT_PIN`):

1. **Blind turn**: spin left for a fixed `TURN_BLIND_MS`, so the sensor doesn't immediately re-trigger on the line it just left.
2. **Approach next line** (sensor-driven): keep spinning left at `SPEED_ROTATE` until `LINE_LEFT_PIN` reads HIGH.
3. **Brake**: counter-spin right at `SPEED_ROTATE + 10` for `BRAKE_MS` to kill momentum, then stop.
4. **Settle**: pause 300ms before returning.

## Color detection

`gridDetectColorValue()` reads red/green/blue channel pulse widths from the
TCS3200 (`gridReadColorChannel()`) and classifies by comparing them:
- Blue channel lowest → `BLUE`
- Red and green both clearly lower than blue and close to each other → `YELLOW`
- Red lowest → `RED`
- Otherwise → `ANY` (unclassified)

Also returns `ANY` immediately if sensors are software-disabled or E-STOP is active.

## Grabbing

`grabColor(color)`:
1. Reads the current color via `gridDetectColorValue()`.
2. If it matches `color` **or** `color == ANY`, sweeps the claw servo gradually from open to closed (2° steps, 15ms apart) instead of snapping shut, holds 800ms for a firm grip, detaches the servo, marks `itemGrabbed = true`, and disables sensors (`disableSensors()`).

`moveToGrab(targetColour)` wraps this with an approach: reads the ultrasonic
distance, calls `executeApproachMovement()` to creep forward until within
`GRAB_APPROACH_DISTANCE_CM`, then calls `grabColor(targetColour)`.

> **Note:** every path below always passes `ANY` as the grab color, so the
> color match is currently a no-op in practice — the claw grabs whatever is
> in front of it regardless of which button (RED/BLUE/YELLOW-labeled) was
> pressed. Wire an actual target color through `runPath()`'s paths if you
> want color-selective grabbing.

## Grid coordinate tracking

`currentX`/`currentY` track position on a logical grid (default home:
`(4, 3)`), `currentHeading` tracks facing (`NORTH`/`EAST`/`SOUTH`/`WEST`,
default `WEST`).

- **`turnToHeading(target)`** — rotates the short way (left or right) until `currentHeading == target`.
- **`moveCoord(x, y)`** — turns to face the needed axis, steps one grid coordinate at a time via `gridMoveForwardOneCoord`, first along X then Y, then finishes facing `WEST`.
- **`moveHome()`** — calls `moveCoord(4, 3)`, then faces `EAST`.
- **`resetCoordinates()`** — resets position/heading to the default home baseline and clears E-STOP.

These are only exercised by `moveToGrab`'s `GRB` steps and by the currently
unused `executeAutoMission()` — see [Dead code](#dead-code-not-reachable-from-loop) below.

## Array-based path scripts

Each mission is a `Step { Action act; uint8_t arg; }` array, run by `runPath()`:

| Action | Meaning |
|---|---|
| `FWD n` | `gridMoveForwardBlocks(n, ...)` — line-follow through `n` junctions |
| `LFT` / `RGT` | `gridRotateLeft90()` / `gridRotateRight90()` |
| `GRB` | `moveToGrab(targetColour)` — approach + grab |
| `REV` | `gridSlightReverse()` |
| `DRP` | `openClawWithAttach()` — open the claw |
| `GDR` | `goToDrop()` — line-follow through 4 junctions to a fixed drop zone (unused by any current path) |
| `DLY n` | `delay(n * 100)` ms |

Three scripted routes:
- **`path1`** / **`path3`** — mirror-image weaves: 2 blocks → turn → 2 blocks → turn → 2 blocks → grab → turn → reverse → turn → 2 blocks → turn → 2 blocks → turn → 5 blocks → drop → reverse 2.
- **`path2`** — simpler: 4 blocks → grab → turn → reverse → turn → 7 blocks → drop → reverse 2.

## IR command map

`loop()` decodes one IR command per call. A held-button repeat is ignored
except that any `CMD_STAR` press always triggers emergency stop first.

| Button | Code | Action |
|---|---|---|
| 1 | 22 | `runPath(path1, ..., RED)` |
| 2 | 25 | `runPath(path2, ..., RED)` |
| 3 | 13 | `runPath(path3, ..., RED)` |
| 4 | 12 | `runPath(path1, ..., BLUE)` |
| 5 | 24 | `runPath(path2, ..., BLUE)` |
| 6 | 94 | `runPath(path3, ..., BLUE)` |
| 7 | 8 | `runPath(path1, ..., YELLOW)` |
| 8 | 28 | `runPath(path2, ..., YELLOW)` |
| 9 | 90 | `runPath(path3, ..., YELLOW)` |
| # | 74 | `openClawWithAttach()` only, no movement |
| * | `CMD_STAR` (0x42) | Emergency stop — halts motors immediately, aborts any in-progress path |

## Emergency stop

`checkEmergencyStop()` is polled inside every blocking movement loop. If a
`CMD_STAR` press is decoded, it sets `emergencyStopActive`, calls
`gridStop()`, and logs it — every subsequent loop iteration of the current
maneuver sees the flag and returns early. A fresh (non-`*`) button press
clears the flag and re-arms the system.

## Dead code (not reachable from `loop()`)

- **`executeAutoMission(targetY, color)`** — an alternate mission flow (drive 2 blocks → `moveCoord` to a target row → `moveToGrab` → `moveHome` → `goToDrop` → open claw → `resetCoordinates`). The six `case` lines that would dispatch it from `loop()` are commented out.
- **`goToDrop()`** — only reachable via `executeAutoMission` or the unused `GDR` step; no active path uses `GDR`.

## Tuning knobs

- **Speeds**: `SPEED_START_FAST` (50), `SPEED_START_SLOW` (40), `SPEED_MIN` (35, floor for dynamic slowdown), `SPEED_ROTATE` (60), `SPEED_REVERSE_BUMP` (60).
- **Timing**: `CENTER_OFFSET_MS` (110, post-block settle nudge), `TURN_BLIND_MS` (150), `BRAKE_MS` (40), `REVERSE_BUMP_MS` (100).
- **Grab distances**: `ITEM_DETECT_DISTANCE_CM` (25, `checkItem()` detection range), `GRAB_APPROACH_DISTANCE_CM` (6, stop-and-grab range).
- **Claw angles**: `CLAW_OPEN_ANGLE` (20°), `CLAW_CLOSED_ANGLE` (100°).

All of the above are physical/empirical values calibrated against the actual
track and hardware — changing them requires re-testing on the real car.

## Known gaps / caveats

- **Color-blind grabbing**: as noted above, every path passes `ANY` into `GRB`, so `grabColor`'s color check never actually filters anything today.
- **`executeApproachMovement`, `turnToHeading`, `moveCoord`, `moveToGrab`** were reconstructed from how they're called elsewhere in the file (the original bodies were missing/corrupted in this file as found) — verify their behavior against real hardware before trusting a full autonomous run; start with a simple button-1 test on a known track.
