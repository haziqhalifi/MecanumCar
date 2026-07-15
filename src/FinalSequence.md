# FinalSequence — Grid-Based Pickup/Drop Firmware

Firmware for a mecanum-wheel car that line-follows a grid, approaches a colored
block with the ultrasonic sensor, grabs it (optionally color-selective), and
carries it to a drop zone. Entry point: [FinalSequence.cpp](FinalSequence.cpp).
Controlled by IR remote, USB serial, or the [wireless WiFi
dashboard](#wireless-bridge-and-dashboard) — nothing runs autonomously until a
button is pressed.

## Hardware

- 3 line sensors: `LINE_LEFT_PIN` (A0), `LINE_CENTER_PIN` (A1), `LINE_RIGHT_PIN` (A2) — digital, HIGH when over a line.
- HC-SR04 ultrasonic: `ULTRASONIC_TRIG_PIN` (12), `ULTRASONIC_ECHO_PIN` (13). Pin modes are re-asserted on every read (`gridGetDistanceCm()`) to survive servo/IR/motor-bus activity that can otherwise leave the pins mis-configured.
- TCS3200 color sensor: `COLOR_S0_PIN` (4), `COLOR_S1_PIN` (5), `COLOR_S2_PIN` (7), `COLOR_S3_PIN` (6), `COLOR_OUT_PIN` (8).
- Claw servo on `CLAW_SERVO_PIN` (9), moved between `CLAW_OPEN_ANGLE` (0°) and `CLAW_CLOSED_ANGLE` (100°).
- IR receiver on `IR_RECEIVE_PIN` (A3), decoded with IRremote.
- 4 mecanum wheels via the `mecanumCar` library (`mecCar.Advance()`, `mecCar.Turn_Right()`, etc., bit-banged I2C on pins 2/3) and four global PWM speed variables (`speed_Upper_L/R`, `speed_Lower_L/R`). Speed values are sent **straight through as raw PWM duty cycle** — there is no minimum-throttle mapping, so any commanded speed below roughly the mid-30s stalls the motors instead of moving the car.

## Setup

1. Serial starts at **9600 baud**.
2. Line/ultrasonic/color sensor pins are configured.
3. The claw opens once (`openClawWithAttach()`) then detaches, so it doesn't hold torque idle.
4. The drive board initializes and default speed (`SPEED_START_FAST`) is set.
5. IR receiver starts listening. The car then sits idle. `loop()` handles serial commands and (when `debugTelemetry` is on) periodic telemetry, but does nothing else until an IR button is pressed.

`sensorsEnabled` starts **false**, so nothing pings the ultrasonic while idle —
it is enabled only for the approach-and-grab phases (see below).

## Core movement primitives

- **`gridMoveForwardBlocks(targetBlocks, startSpeed)`** — line-follows forward, counting a "block" each time all 3 sensors go HIGH together (a junction) after being off it. Slows by `-5` PWM per block already crossed, floored at `SPEED_MIN`. On the **final block** (heading to the last junction) it drops to `FINAL_BLOCK_CRAWL_SPEED` so it eases onto the last grid. After the target is reached, creeps forward an extra `CENTER_OFFSET_MS` to settle centered on the junction, then stops.
- **`gridMoveForwardOneCoord(startSpeed)`** — same idea but for exactly one grid coordinate step: drives blind for 140ms to clear the current junction, then line-follows until the next full 3-sensor junction, then applies the same `CENTER_OFFSET_MS` settle.
- **`gridRotateLeft90` / `gridRotateRight90`** — sensor-terminated 90° pivot (see [Turning](#turning-90)). Return `false` early if interrupted by E-STOP.
- **`gridSteer(baseSpeed, boost, steerLeft)`** — the line-follow **correction**: both sides keep driving forward, but the outer side runs `boost` PWM faster than the inner side, arcing the car back onto the line. Replaces the old pivot-style `Turn_Left/Right` correction (which reversed one side and read as jerky).
- **`gridSlightReverse()`** — brief reverse bump (`REVERSE_BUMP_MS` at `SPEED_REVERSE_BUMP`) used to back off the line right after a grab, before turning.
- **`gridStop()`** — stops the drive and logs it.

Every blocking loop polls `checkEmergencyStop()` each iteration, so a `CMD_STAR` press can abort mid-maneuver.

### Line-follow correction detail

The 7 handled sensor patterns map to: drive straight (center only, or all-off),
or `gridSteer(...)` back toward the line for the four single/double off-center
cases. Straight driving uses the base speed; corrections add
`TURN_CORRECTION_BOOST`. The all-sensors-LOW (line fully lost) case drives
straight — there is no last-direction memory.

## Turning (90°)

`gridRotateLeft90()` (mirrored for right):

1. **Blind turn**: spin for a fixed `TURN_BLIND_LEFT_MS` / `TURN_BLIND_RIGHT_MS`, so the sensor doesn't immediately re-trigger on the line it just left.
2. **Approach next line** (sensor-driven): keep spinning at `SPEED_ROTATE` until the stop sensor reads HIGH. The **left** turn stops on the **CENTER** sensor (on the rotation axis, so it hits ~90° true); the **right** turn stops on `LINE_RIGHT_PIN`.
3. **Brake**: counter-spin at `SPEED_ROTATE + 10` for `BRAKE_MS` to kill momentum, then stop.
4. **Settle**: pause `TURN_SETTLE_MS` before returning.

No telemetry is read inside the spin loops — the blocking ultrasonic/color
`pulseIn` calls would make the turn angle inconsistent.

## Color detection

`gridDetectColorValue()` reads red/green/blue channel pulse widths from the
TCS3200 (`gridReadColorChannel()`) and classifies by comparing them:
- Blue channel lowest → `BLUE`
- Red and green both clearly lower than blue and close to each other → `YELLOW`
- Red lowest → `RED`
- Otherwise → `ANY` (unclassified)

Returns `ANY` immediately if sensors are software-disabled or E-STOP is active.

Robustness measures:
- **`gridReadColorChannel()`** averages several `pulseIn` samples per channel, and maps a timeout (`0`) to `COLOR_PULSE_TIMEOUT_US` so a dropped read doesn't falsely win the "smallest = this color" test.
- **`gridDetectColorMajority()`** reads the full color several times and returns the most-common result — used at grab time so one noisy sample can't flip the decision.
- Setting `logColorRaw` (done automatically around the grab decision) prints the raw R/G/B pulse values for tuning without enabling full telemetry.

## Grabbing

`grabColor(color)`:
1. Reads the current color via `gridDetectColorMajority()` (with raw logging on), printing `[GRAB] target=… detected=…`.
2. If it matches `color` **or** `color == ANY`, calls `closeClawGrip()`. Otherwise logs a color mismatch and skips.

`closeClawGrip()` pauses `GRAB_SETTLE_MS` (let the car fully stop), sweeps the
claw servo gradually open→closed (2° steps, 15ms apart) instead of snapping,
holds 800ms for a firm grip, detaches, marks `itemGrabbed = true`, disables
sensors, then pauses `GRAB_SETTLE_MS` again before the robot moves off.

`moveToGrab(targetColour, approachStartSpeed = SPEED_MIN)` wraps this with an
approach: reads the ultrasonic distance, calls `executeApproachMovement()` to
creep forward until within `GRAB_APPROACH_DISTANCE_CM`, then calls
`grabColor(targetColour)`.

`executeApproachMovement(currentDistance, startSpeed)` line-follows toward the
block while ramping speed down linearly from `startSpeed` (at
`ITEM_DETECT_DISTANCE_CM`) to a floor (at `GRAB_APPROACH_DISTANCE_CM`). The
floor is `SPEED_MIN` for path-default callers, or `APPROACH_SPEED_FLOOR` for
callers starting below `SPEED_MIN`. Line correction during the approach uses
the gentler `APPROACH_CORRECTION_BOOST` (not the full line-follow boost) so it
stays aligned with the block instead of yanking toward the line.

> **Color-selective grabbing is wired up.** IR buttons pass a real target color
> (RED/BLUE/YELLOW) into `runPath()`, so `grabColor` only grips a matching block.
> The `path*` step arrays still list `GRB, ANY`, but `runPath` overrides that with
> the `targetColour` argument passed to `moveToGrab`, so the button's color wins.

## Grid coordinate tracking

`currentX`/`currentY` track position on a logical grid (default home:
`(4, 3)`), `currentHeading` tracks facing (`NORTH`/`EAST`/`SOUTH`/`WEST`,
default `WEST`).

- **`turnToHeading(target)`** — rotates the short way (left or right) until `currentHeading == target`.
- **`moveCoord(x, y)`** — turns to face the needed axis, steps one grid coordinate at a time via `gridMoveForwardOneCoord`, first along X then Y, then finishes facing `WEST`.
- **`moveHome()`** — calls `moveCoord(4, 3)`, then faces `EAST`.
- **`resetCoordinates()`** — resets position/heading to the default home baseline and clears E-STOP.

These are only exercised by the currently unused `executeAutoMission()` — see
[Dead code](#dead-code-not-reachable-from-loop) below.

## Array-based path scripts

Each mission is a `Step { Action act; uint8_t arg; }` array, run by `runPath()`:

| Action | Meaning |
|---|---|
| `FWD n` | `gridMoveForwardBlocks(n, ...)` — line-follow through `n` junctions |
| `LFT` / `RGT` | `gridRotateLeft90()` / `gridRotateRight90()` |
| `GRB` | `moveToGrab(targetColour, PATH_APPROACH_SPEED)` — approach + color-checked grab |
| `REV` | `gridSlightReverse()` |
| `DRP` | `openClawWithAttach()` — open the claw |
| `GDR` | `goToDrop()` — line-follow through 4 junctions to a fixed drop zone (unused by any current path) |
| `DLY n` | `delay(n * 100)` ms |

`runPath()` also:
- Opens the claw once before the first step.
- Keeps `sensorsEnabled` **off** during navigation and turns it on only for the last `FWD` before a `GRB` (via `nextRealStepIsGrab`) and the `GRB` itself, so the ultrasonic only fires during the actual approach.
- Inserts a `STEP_TRANSITION_MS` settle pause between every step.

Three scripted routes, all starting from home `(7,3)` facing WEST and ending on
the drop node `(9,3)`.

### Route invariants

These two hold for **every** outbound route — the three scripted paths, the
colour hunt in `searchAndGrab()`, and `executeAutoMission()` alike. A searched
route and a scripted route to the same block trace the same lane.

- **Row changes happen at column 4** — the clear middle of the collecting grid,
  never on the block column and never inside the START zone. The scripted paths
  spend their first `FWD 3` reaching it; the search targets `SEARCH_TURN_X`
  before its row move (`moveCoord` does X before Y, so aiming at
  `(SEARCH_TURN_X, row)` pulls the car out to column 4 *then* changes row).
- **The approach stops on column 2** (`GRAB_APPROACH_X`), one node east of the
  block at column 1. `moveToGrab()` creeps the remaining cell with the ultrasonic
  and stops at `GRAB_APPROACH_DISTANCE_CM`. This one is load-bearing:
  `gridMoveForwardBlocks()` only counts junctions and never reads the ultrasonic,
  so a `FWD` counting through to column 1 drives the chassis into the block. The
  final 4→2 leg is also a straight, turn-free run, which lets the line-follow
  re-centre the car before it creeps in — a skewed car can't grip.

| | route |
|---|---|
| **`path1`** | `FWD 3` (col 7→4) → `LFT` → `FWD 2` (row 3→1) → `RGT` → `FWD 2` (col 4→2) → grab → reverse → `RGT` → `FWD 2` (row 1→3) → `RGT` → `FWD 7` (col 2→9) → drop → reverse 2 |
| **`path2`** | `FWD 5` (col 7→2) → grab → `RGT` → reverse → `RGT` → `FWD 7` (col 2→9) → drop → reverse 2 |
| **`path3`** | `FWD 3` (col 7→4) → `RGT` → `FWD 2` (row 3→5) → `LFT` → `FWD 2` (col 4→2) → grab → reverse → `LFT` → `FWD 2` (row 5→3) → `LFT` → `FWD 7` (col 2→9) → drop → reverse 2 |

`path1`/`path3` are mirror images. Their return legs change row at column 2 (not
4) on purpose: a block on row 1 or 5 reaches the row-3 corridor with one 90° turn
onto it and another to face east, so the loaded car never has to spin 180°.
`path2`'s block is already on row 3, so it needs no row change outbound — and its
`RGT` … `RGT` return is the one 180° the route allows.

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
| ▲ UP | 70 | `moveToGrab(ANY, 40)` — creep forward, slow on approach, grab whatever's ahead |
| ◀ LEFT | 68 | `clawNudge(true)` — nudge claw toward closed |
| ▶ RIGHT | 67 | `clawNudge(false)` — nudge claw toward open |
| OK | 64 | Full reset: open grip, `resetCoordinates()`, re-enable sensors |
| # | 74 | `openClawWithAttach()` only, no movement |
| * | `CMD_STAR` (0x42) | Emergency stop — halts motors immediately, aborts any in-progress path |

## Serial command control

`handleSerialCommand()` lets a USB dashboard drive the car for step-by-step
debugging, reusing the same functions the IR paths call. Commands are
newline-terminated ASCII (`VERB` or `VERB:arg`); every reply is prefixed
`[ACK]`.

| Command | Effect |
|---|---|
| `STOP` | Latch E-STOP and halt (works even while latched) |
| `RESUME` / `ARM` | Clear E-STOP |
| `FWD:n` | `gridMoveForwardBlocks(n)` |
| `STEP` | `gridMoveForwardOneCoord` — one grid coordinate |
| `LFT` / `RGT` | Rotate 90° |
| `REV` | Slight reverse |
| `GRB:c` | `moveToGrab` with target color `c` (0–3) |
| `DRP` / `OPEN` | Open claw |
| `PING` | Read + print distance and color once |
| `SENSORS` | Print raw line-sensor L/C/R snapshot |
| `PATH:pc` | Run path `p` (1–3) with color `c` — arg encodes `path*10 + color` |
| `RESET` | `resetCoordinates()` |
| `TELEMETRY` / `TEL` (`:0`/`:1` or bare toggle) | Turn distance/color telemetry prints on/off |
| `SEARCH:cr` | `searchGrabAndDrop(color, startRow)` — the full color-hunt mission, arg encodes `color*10 + startRow` (color 0=RED/1=BLUE/2=YELLOW, startRow 1/3/5). Mirrors the 9 IR mission buttons so each one is individually triggerable and debuggable from a dashboard. |

Any other command while E-STOP is latched is refused until `RESUME`.

`STOP` is special: `checkEmergencyStop()` — the same function polled inside
every blocking movement loop — also peeks both serial links (USB `Serial` and
the ESP32 bridge's `espSerial`) for a line starting with `S`. If it reads
`STOP`, it latches E-STOP immediately, mid-maneuver, exactly like the IR
remote's `*`. Without this, a `STOP` sent while `gridMoveForwardBlocks()` or
`runPath()` was blocking would sit unread until `loop()` cycled back to
`handleSerialCommand()` — i.e. until the current move finished. See
[Emergency stop](#emergency-stop).

## Wireless bridge and dashboard

The car can be driven over WiFi instead of USB. Full hardware/flashing setup
lives in [esp32-wifi-bridge/README.md](../esp32-wifi-bridge/README.md); this
section covers how it connects to the firmware above and what the dashboard UI
exposes.

### Connection path

```
browser (dashboard-wifi.html)
   │  WebSocket ws://<esp32-ip>/ws
   ▼
ESP32 (esp32-wifi-bridge/src/main.cpp)
   │  Serial2 @ 9600, forwards text frames verbatim
   ▼
Uno espSerial (SoftwareSerial, pins 11 RX / 10 TX) @ 9600
   │  handleSerialCommand() — same parser as the USB link
   ▼
FinalSequence.cpp
```

- The ESP32 joins WiFi and serves the dashboard page itself (embedded gzipped
  in `esp32-wifi-bridge/src/dashboard_html.h`, generated from
  `dashboard/dashboard-wifi.html` — re-run
  `python3 esp32-wifi-bridge/tools/gen_dashboard_header.py` after editing the
  HTML, then reflash the ESP32).
- Button clicks send the exact same newline-terminated commands documented in
  [Serial command control](#serial-command-control) (`FWD:2`, `SEARCH:11`,
  `STOP`, …) — the Uno can't tell a wireless command from a USB one.
- Every line the Uno prints via `Bridge` (both `Serial` and `espSerial`) is
  read back off Serial2 by the ESP32 and broadcast to all connected browsers,
  so the dashboard's console/telemetry is the same firmware output the USB
  Serial Monitor would show.
- The Uno's `Serial`/`espSerial` are two independent physical links (hardware
  UART pins 0/1 vs. `SoftwareSerial` on 11/10) but feed the **same**
  `handleSerialCommand()` parser and the **same** `emergencyStopActive` state
  — wired debug and wireless dashboard are interchangeable, never both at once
  on pins 0/1 (USB Serial Monitor open + ESP32 bridge wired in will fight over
  the Uno's one hardware UART).

### Dashboard panels (`dashboard/dashboard-wifi.html`)

- **Manual control** — one button per serial command (movement, claw, full
  paths, `PING`/`SENSORS` snapshots, `RESUME`/`RESET`) plus the always-enabled
  **EMERGENCY STOP** button (`STOP`). A **Live readings** toggle sends
  `TELEMETRY:1`/`:0` to stream distance + color every 250ms instead of
  requiring a `PING` click per reading.
- **IR remote debug** — a 3×3 grid (color × start row) mirroring the 9
  `searchGrabAndDrop()` mission buttons from the [IR command map](#ir-command-map),
  each cell sending the matching `SEARCH:cr` command. A physical remote press
  lights up the matching cell (parsed from the `Key Pressed: N` line the
  firmware prints) so the same panel doubles as an IR-receiver check — a code
  arriving with no matching cell shows as "unmapped" rather than silently
  doing nothing. STOP/`#`/OK are wired the same way; the claw-nudge and
  creep-grab remote buttons have no serial equivalent and only light up on a
  physical press (see the `serialStopRequested`/`SEARCH` additions above for
  why STOP and the 9 missions specifically got serial commands).
- **Sensor test bench** — ultrasonic, color, and IR readouts side by side,
  each flashing briefly and stamping "updated at Xs" when a fresh reading
  lands, so a sensor that's stuck or unplugged is visually obvious (no flash =
  no data).
- **Line sensors / current action / session stats / console** — parse the
  firmware's existing log line prefixes (`[TURN]`, `[NAVIGATE]`, `[GRAB]`,
  `[SYSTEM]`, `[ACK] SENSORS L=… C=… R=…`, …) into live UI state; no firmware
  changes were needed for these, they were already being printed.

`checkEmergencyStop()` is polled inside every blocking movement loop. Each
call checks two independent sources, either of which latches the stop:

1. **IR**: a `CMD_STAR` (`*`) press.
2. **Serial**: a `STOP` line peeked off `Serial` or `espSerial` (see
   `serialStopRequested()`) — this is what lets the wireless dashboard's
   EMERGENCY STOP button interrupt a move that's already in progress, not just
   block a new one from starting.

Either source sets `emergencyStopActive`, calls `gridStop()`, and logs it —
every subsequent loop iteration of the current maneuver sees the flag and
returns early. A fresh (non-`*`) IR button press, or `RESUME`/`ARM` over
serial, clears the flag and re-arms the system.

`handleSerialCommand()`'s own top-level `STOP` handling (outside the motion
loops) exists for the case where the car is idle and no blocking loop is
running to poll `checkEmergencyStop()` — both paths converge on the same
`emergencyStopActive` flag.

## Dead code (not reachable from `loop()`)

- **`executeAutoMission(targetY, color)`** — an alternate mission flow (drive 2 blocks → `moveCoord` to a target row → `moveToGrab` → `moveHome` → `goToDrop` → open claw → `resetCoordinates`). The six `case` lines that would dispatch it from `loop()` are commented out.
- **`goToDrop()`** — only reachable via `executeAutoMission` or the unused `GDR` step; no active path uses `GDR`.
- **`checkItem()`** — helper not called from any active path.

## Tuning knobs

- **Speeds**: `SPEED_START_FAST` (48), `SPEED_START_SLOW` (42), `SPEED_MIN` (38, floor for dynamic slowdown), `FINAL_BLOCK_CRAWL_SPEED` (35, last-grid crawl), `SPEED_ROTATE` (60), `SPEED_REVERSE_BUMP` (60), `PATH_APPROACH_SPEED` (42), `APPROACH_SPEED_FLOOR` (38).
- **Correction**: `TURN_CORRECTION_BOOST` (50, line-follow steer boost), `APPROACH_CORRECTION_BOOST` (12, gentler steer during block approach).
- **Timing**: `CENTER_OFFSET_MS` (110, post-block settle nudge), `TURN_BLIND_LEFT_MS` / `TURN_BLIND_RIGHT_MS` (350), `BRAKE_MS` (55), `TURN_SETTLE_MS` (600), `GRAB_SETTLE_MS` (500), `STEP_TRANSITION_MS` (250), `REVERSE_BUMP_MS` (100).
- **Grab distances**: `ITEM_DETECT_DISTANCE_CM` (25, approach ramp start), `GRAB_APPROACH_DISTANCE_CM` (stop-and-grab range).
- **Claw angles**: `CLAW_OPEN_ANGLE` (0°), `CLAW_CLOSED_ANGLE` (100°).
- **Color**: `COLOR_PULSE_TIMEOUT_US` (30000).

All of the above are physical/empirical values calibrated against the actual
track and hardware — changing them requires re-testing on the real car.

## Known gaps / caveats

- **Motor stall floor**: because speed maps straight to raw PWM, values below the mid-30s don't move the car. Several approach/crawl constants sit just above that floor; lower them and the car buzzes in place instead of creeping.
- **Servo brown-out**: gripping a firm object draws stall current; if the servo shares the Arduino 5V rail it can fail to move on the next command (looks like "claw won't open/close"). `openClawWithAttach()` re-sends the open angle several times to mitigate, but a separate 5–6V servo supply with common ground is the real fix.
- **Line-lost handling**: when all three line sensors go LOW the follower drives straight with no memory of the last drift direction, so a hard drift off the line is not actively recovered.
