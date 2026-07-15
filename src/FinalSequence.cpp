#include <Arduino.h>
#include <Servo.h>
#include "MecanumCar_v2.h"
#include <IRremote.hpp>
#include <SoftwareSerial.h>

// ── ESP32 wireless bridge link ───────────────────────────────────────────────
// A SoftwareSerial link to the ESP32 web-dashboard bridge, kept OFF the Uno's
// hardware Serial (pins 0/1) so the USB Serial Monitor stays usable for
// debugging while the ESP32 talks to the Uno independently.
//   SoftwareSerial(RX, TX) = (11, 10)  — matches lib/main.cpp's convention:
//     Uno pin 11 (RX) <- ESP32 TX2 (GPIO17)
//     Uno pin 10 (TX) -> ESP32 RX2 (GPIO16)   [level-shift 5V->3.3V]
//   Common GND. Runs at 9600 to match Serial.
SoftwareSerial espSerial(11, 10);

// Tee Print target: everything written to Bridge goes to BOTH the USB Serial
// (so the wired Serial Monitor still shows it) and espSerial (so the wireless
// dashboard sees the same telemetry/log lines). All the firmware's telemetry
// and [ACK]/[TURN]/[NAVIGATE]/... log lines print through this, so both the USB
// and wireless dashboards receive identical output. Command *input* is read
// from whichever of the two links has bytes (see handleSerialCommand()).
class BridgePrint : public Print {
public:
    size_t write(uint8_t c) override {
        Serial.write(c);
        espSerial.write(c);
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) override {
        Serial.write(buffer, size);
        espSerial.write(buffer, size);
        return size;
    }
};
BridgePrint Bridge;

// ── Line Tracking Sensors Pin Definitions ──────────────────────────────────
#define LINE_LEFT_PIN A0
#define LINE_CENTER_PIN A1
#define LINE_RIGHT_PIN A2

// ── Ultrasonic Sensor Pin Definitions (onboard module for this kit) ────────
#define ULTRASONIC_TRIG_PIN 12 
#define ULTRASONIC_ECHO_PIN 13 

// ── Claw & Sensor Pin Definitions ───────────────────────────────────────────
#define CLAW_SERVO_PIN 9
#define COLOR_S0_PIN 4
#define COLOR_S1_PIN 5
#define COLOR_S2_PIN 7
#define COLOR_S3_PIN 6
#define COLOR_OUT_PIN 8
#define IR_RECEIVE_PIN A3 

// ── Motor Speed References from Library ────────────────────────
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

mecanumCar mecCar(3, 2); 
Servo clawServo;         

const int RED = 0, BLUE = 1, YELLOW = 2, ANY = 3;

bool itemGrabbed = false;
// Sensors (ultrasonic + color) start OFF so nothing pings the ultrasonic while
// idle. runPath()/moveToGrab() enable them only for the approach-and-grab, and
// the UP button enables them for its manual grab.
bool sensorsEnabled = false;
volatile bool emergencyStopActive = false; 
// Enable/disable verbose sensor telemetry prints (distance/color over serial).
// Toggle at runtime with the serial "TELEMETRY"/"TEL" command.
bool debugTelemetry = false;
// Eco-Mode (Group 8 FSM): simulates "a cloud drops solar input mid-transit."
// When engaged the car moves slower and shuts off non-essential sensors
// (ultrasonic/color); line sensors stay on since navigation needs them.
// Toggle at runtime with the "ECO"/"ECO:0"/"ECO:1" command. ecoBypass lets the
// spin helpers keep full, calibrated rotation speed while eco is engaged.
bool ecoMode = false;
bool ecoBypass = false;
unsigned long lastTelemetryMillis = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 1000; // ms between automatic reports

// Pose heartbeat: re-announce [POS] while idle even though nothing has moved.
// [POS] is otherwise only emitted when the pose CHANGES, which makes the
// dashboard map's correctness depend on it having caught every single line —
// one dropped line (or a browser that connected after the last move) leaves the
// map stale with no way to ask for a resync. Since [POS] is an ABSOLUTE pose and
// not a delta, simply repeating it is idempotent and makes the map self-healing.
//
// Deliberately driven from loop() and nowhere else: loop() only runs when the
// car is idle (a mission blocks it until it finishes), so this never adds serial
// traffic mid-move. That matters — espSerial is bit-banged SoftwareSerial, whose
// write() blocks with interrupts DISABLED for ~1ms per byte, which would both
// stall the line-sensor sampling and stop millis() advancing underneath
// JunctionDetector. During a move the per-crossing [POS] lines already cover us.
unsigned long lastPosHeartbeatMillis = 0;
const unsigned long POS_HEARTBEAT_MS = 2000;

// ── Global Position and Heading Tracking ────────────────────────────────────
// Home/start is node (7,3): the car begins in the START zone, which sits east
// of the 6×6 collecting grid. A SEARCH mission drives 6 junctions west from
// here (X7→X1) to reach the blocks at column X=1, rows Y=1/3/5.
int currentX = 7, currentY = 3;
const int NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;
int currentHeading = WEST;

// ── Grid Boundary (keep-out border) ─────────────────────────────────────────
// The playable area is a 6×6 collecting grid (junction nodes 0..6) plus a 3×2
// START zone extending east of its right edge (nodes X 6..9, Y 2..4), home at
// (7,3). The car must NOT drive onto the outermost line on any side — it stays
// one node in from every edge. Tracked node is clamped to X 1..8, Y 1..5:
// X=1 = block column; the car only drives as far as X=2 (one node east) to read
// colour and grab, letting the ultrasonic creep the last bit so it never clashes
// with the block on X=1. X=8 keeps it one in from the START zone's east edge
// (X9); Y 1..5 is one in from the
// collecting top/bottom. moveCoord() clamps every target so the car can never
// be commanded onto or past the outer boundary line.
const int GRID_MIN_X = 1, GRID_MAX_X = 8;
const int GRID_MIN_Y = 1, GRID_MAX_Y = 5;

// Emit the tracked pose over both serial links so the dashboard's live map can
// draw the robot. Format: "[POS] x,y,h" with h = 0..3 (NORTH/EAST/SOUTH/WEST).
// Called every time currentX/currentY/currentHeading changes.
void reportPosition() {
    Bridge.print(F("[POS] "));
    Bridge.print(currentX); Bridge.print(F(","));
    Bridge.print(currentY); Bridge.print(F(","));
    Bridge.println(currentHeading);
}

// ── Tracked-pose updates (single source of truth) ───────────────────────────
// These live INSIDE the motion primitives (gridMoveForwardOneCoord,
// gridMoveForwardBlocks, gridRotate*90) rather than in their callers. They used
// to be done by hand in moveCoord()/turnToHeading() *after* calling a
// primitive, which meant any code path that called a primitive directly — every
// dashboard button (FWD/STEP/LFT/RGT) and every scripted path (runPath) — moved
// the real car while the tracked pose sat still. The map then disagreed with the
// robot until the next resetCoordinates(). Tracking at the primitive means the
// pose follows the wheels no matter who asked them to turn.
//
// Deliberately NOT clamped to the grid bounds: this is a report of where the car
// actually is, so it must be free to say "off the grid" rather than quietly lie.
// (moveCoord() still clamps commanded *targets* — that's the guard that keeps
// the car in bounds. The drop node at X=9 also sits outside GRID_MAX_X=8, so a
// clamp here would both misreport it and hang `while (currentX < 9)`.)
void advanceTrackedNode() {
    switch (currentHeading) {
        case NORTH: currentY += 1; break;
        case SOUTH: currentY -= 1; break;
        case EAST:  currentX += 1; break;
        case WEST:  currentX -= 1; break;
    }
    reportPosition();
}

void rotateTrackedHeading(bool right) {
    currentHeading = right ? (currentHeading + 1) % 4 : (currentHeading + 3) % 4;
    reportPosition();
}


// ── Tuning Constants ───────────────────────────────────────────────────────
const uint8_t SPEED_START_FAST = 48;
const uint8_t SPEED_START_SLOW = 42;
const uint8_t SPEED_MIN = 38;
// Crawl speed for the final block of a forward move (last grid before stopping),
// so the car eases onto the last junction and stops right on it. Kept just
// above the motor stall floor so it still moves but is clearly slower than the
// SPEED_MIN=38 the earlier blocks ramp down to.
const uint8_t FINAL_BLOCK_CRAWL_SPEED = 35;
// Extra speed added to a line-follow CORRECTION turn over the forward speed.
// A bang-bang follower drives straight and yanks left/right to recover; at high
// forward speed a same-speed correction is too weak, so the car weaves and can
// sling off the line. Turning HARDER than it drives catches drift in one move.
const uint8_t TURN_CORRECTION_BOOST = 50;
// Gentler correction used only during the final block approach. The full
// TURN_CORRECTION_BOOST yanks the car sideways hunting the line, which throws
// off its alignment with the block so it can't grab. A small boost just keeps
// it roughly straight toward the block instead of aggressively arcing.
const uint8_t APPROACH_CORRECTION_BOOST = 12;
const unsigned long CENTER_OFFSET_MS = 110;

// ── Junction-clearing (line-width independent) ──────────────────────────────
// Leaving a junction used to be a blind fixed 140ms drive, after which the code
// assumed the junction was behind it and any all-sensors-HIGH read must be the
// NEXT junction. That assumption is only as good as the line width it was tuned
// against: on a WIDER/BOLDER line (the drop node at 9,3 is painted heavier than
// the rest of the grid) the car is still standing on the same line when the
// timer expires, so the hunt loop instantly "finds" the line it never left and
// counts a node it never travelled. Every count downstream is then off by one.
//
// Instead, clear the junction by STATE: drive until the sensors actually stop
// reading the junction (center goes LOW = we're on plain line//mat again), so a
// fat line just takes a few more ms rather than corrupting the count.
// JUNCTION_CLEAR_MIN_MS debounces the sensor as it rolls off the line;
// JUNCTION_CLEAR_TIMEOUT_MS is a backstop so a stuck-HIGH sensor can't hang the
// move forever (it falls through to the old time-based behaviour).
const unsigned long JUNCTION_CLEAR_MIN_MS = 60;
const unsigned long JUNCTION_CLEAR_TIMEOUT_MS = 900;

// ── Junction detection (sensor-offset independent) ──────────────────────────
// Same lesson as the clearing constants above, one step further along: a
// junction is a CROSS, and the obvious test for one — a single sample reading
// all three sensors HIGH at once — assumes the outer sensors reach the cross
// line together. They don't. LEFT is mounted AHEAD of CENTER/RIGHT (the same
// offset that used to break the left 90; see TURN_BLIND_LEFT_MS), so on a
// thinner cross LEFT can go HIGH and fall back LOW before RIGHT ever arrives,
// and no sample ever sees all three. That crossing then never counts.
//
// Which is worse than it sounds, because these moves stop on a COUNT, not on a
// distance: a missed cross doesn't cut the move short, it runs the car ON to
// find its count further down the line, while the tracked pose — and so the
// dashboard map — still reads the intended target. A FWD 3 that drops two
// crossings stops the car on column 2 still believing it is on column 4.
//
// So detect the cross by "both outers saw it within the same short window"
// instead of "both were HIGH in the same sample": latch each outer sensor's
// HIGH for JUNCTION_LATCH_MS and call it a crossing once both latches are live.
// Sensor offset and line width then only change how far APART the two hits
// land, not whether the crossing registers at all.
//
// JUNCTION_LATCH_MS is the tuning knob, and it is squeezed from both sides:
// long enough to span the LEFT-to-RIGHT offset at SPEED_START_FAST, short
// enough that a line-follow wobble — which trips one outer and then the other
// as TURN_CORRECTION_BOOST yanks the car back — can never fake a cross. Raise
// it if crossings are still missed. LOWER it if the car counts phantom nodes on
// the straights: a phantom count on an outbound leg counts through to column 1
// and drives the chassis into the block.
const unsigned long JUNCTION_LATCH_MS = 120;

const uint8_t SPEED_ROTATE = 60;
// Blind-spin duration before we start hunting for the new perpendicular line.
// Must be long enough to rotate PAST the too-early black detection (car stops
// short if this is too small). Tune per surface/battery: raise if it still
// under-rotates, lower if it overshoots.
//
// The LEFT turn used to stop short because it broke on the offset LEFT sensor,
// which reaches the perpendicular line before the body has rotated a full 90.
// It now stops on the CENTER sensor (on the rotation axis), which crosses the
// line at a true ~90, so LEFT no longer needs an inflated blind time. The blind
// window only has to carry the center sensor OFF the starting junction black
// onto white before hunting begins.
const unsigned long TURN_BLIND_LEFT_MS  = 350;
const unsigned long TURN_BLIND_RIGHT_MS = 350;
const unsigned long BRAKE_MS = 55;
// Pause after a 90-degree turn completes, letting the chassis fully settle
// before the next move.
const unsigned long TURN_SETTLE_MS = 600;
// Pause before the claw closes (let the car come to a full stop) and after it
// closes (let the grip firm up before the robot moves the item).
const unsigned long GRAB_SETTLE_MS = 500;
// Settling pause inserted between each step of a scripted path so transitions
// aren't abrupt.
const unsigned long STEP_TRANSITION_MS = 250;
// Safety: if all three line sensors read LOW (no line anywhere under the car)
// continuously for this long, the car has run off the grid onto blank mat —
// latch an emergency stop. Junctions read all-HIGH and normal following keeps
// the centre sensor on the line, so a sustained all-LOW only happens off-track.
// Tune: raise if the car false-stops crossing wide gaps, lower to halt sooner.
const unsigned long LINE_LOST_TIMEOUT_MS = 800;
// Start speed for the scripted-path block approach. Kept below SPEED_MIN so the
// approach creeps in noticeably slower and still tapers toward APPROACH_SPEED_FLOOR.
// Lowered from 42: at the higher speed the car noses onto the block before its
// line-follow correction has straightened it, so it arrives slightly skewed and
// the claw can't grip. A slower approach gives the alignment time to settle.
// Must stay strictly below SPEED_MIN (38) so executeApproachMovement's taper
// engages (it only ramps toward APPROACH_SPEED_FLOOR when startSpeed < SPEED_MIN;
// at exactly SPEED_MIN the floor becomes SPEED_MIN and the approach runs flat).
// Lowered further from 37 to 35 (one notch above APPROACH_SPEED_FLOOR=34) so the
// WHOLE approach — not just the final taper — is a slow crawl, giving the
// line-follow correction the maximum distance to straighten the car onto the line
// before it reaches the block and grabs. Do NOT go below ~34: that is near the
// motors' stall/breakaway floor (see APPROACH_SPEED_FLOOR note) and the car would
// buzz in place and stop short of the block instead of creeping onto it.
const uint8_t PATH_APPROACH_SPEED = 35;
const int ITEM_DETECT_DISTANCE_CM = 25;
// Stop this far from the block to read colour and grab — leave a GAP rather than
// nosing right up to it. The gripper jaws swing FORWARD as they close, so they
// reach the block from this standoff; stopping closer makes the chassis collide
// with the block (and the colour read is taken here too, so it must not ram it).
// Empirical — tune on the real car: too large and the closing claw misses the
// block, too small and the chassis bumps it. Was 6cm (nose-to-block), then 9cm,
// then 8cm. Now 7cm: measured as the distance where the colour sensor reads the
// block reliably. Do NOT go to 6cm — that is the measured collision point where
// the chassis rams the block, leaving no margin for ultrasonic jitter.
const int GRAB_APPROACH_DISTANCE_CM = 7;
// Absolute floor for the approach ramp-down, separate from SPEED_MIN (which is
// still used by the scripted paths' floor). Lets a caller start slower than
// SPEED_MIN (e.g. the UP button's 40) and still have room to taper down
// further as it nears the grab distance.
// NOTE: the speed value is sent straight through as raw PWM duty cycle to the
// I2C motor driver (see mecanumCar::PWM_OUT / Writebyte) — there's no minimum
// throttle mapping. 10/255 (~4%) is below the motors' stall/breakaway torque,
// so the car just buzzed in place instead of creeping — that's why lowering
// this earlier caused it to stop short of the block. Keep this close to
// SPEED_MIN so it still physically moves; it only needs to be slightly below
// SPEED_MIN to give a *visible* taper, not a true crawl.
// Lowered from 38 to 34 so the very last stretch onto the block is a genuine
// crawl — this is where the car needs to be dead-straight for the claw to grip.
// 34 is close to the stall floor (~35 crawl elsewhere) so it still creeps but is
// slow enough that any residual line-follow correction can straighten it before
// it reaches the grab distance and stops.
const uint8_t APPROACH_SPEED_FLOOR = 34;
const uint8_t CLAW_OPEN_ANGLE = 0;         // wider default-open (lower angle = more open)
const uint8_t CLAW_CLOSED_ANGLE = 100;
// pulseIn timeout for one color-sensor channel read (microseconds).
const unsigned long COLOR_PULSE_TIMEOUT_US = 30000UL;
// Tracks the claw's last commanded angle so the LEFT/RIGHT arrow nudge
// buttons can step from wherever it currently sits, rather than jumping to
// an extreme. Kept in sync by openClawWithAttach()/closeClawGrip()/clawNudge().
uint8_t clawCurrentAngle = CLAW_OPEN_ANGLE;
const uint8_t SPEED_REVERSE_BUMP = 60; 
const unsigned long REVERSE_BUMP_MS = 100; 
const int CMD_STAR = 0x42; 

// ── Forward Declarations ────────────────────────────────────────────────────
int gridGetDistanceCm();
int gridDetectColorValue();
const char *gridColorName(int color);
void turnToHeading(int targetHeading);
bool checkEmergencyStop();

void printStatusTelemetry(int distance, int color) {
    if (emergencyStopActive) return;
    Bridge.print(F("distance: "));
    if (distance == -1) Bridge.print(F("---"));
    else Bridge.print(distance);
    Bridge.print(F("cm | color: "));
    Bridge.println(gridColorName(color));
}

// Read sensors and print a single consolidated telemetry line without
// triggering the internal DBG prints from the individual read functions.
void telemetryReport() {
    if (emergencyStopActive) return;
    bool prevDebug = debugTelemetry;
    debugTelemetry = false; // suppress internal DBG prints
    int distance = gridGetDistanceCm();
    int color = gridDetectColorValue();
    debugTelemetry = prevDebug;

    Bridge.print(F("[AUTO] "));
    printStatusTelemetry(distance, color);
}

void motionTelemetryTick(unsigned long &lastTelemetryMillis) {
    if (!debugTelemetry || emergencyStopActive) return;
    if (millis() - lastTelemetryMillis < 250) return;
    telemetryReport();
    lastTelemetryMillis = millis();
}


// Eco-Mode slowdown: ~30% off, floored at APPROACH_SPEED_FLOOR so the motors
// still physically move (below ~34 they stall). Returns the speed unchanged
// when eco is off. Rotations pass through gridSetSpeed with ecoBypass set, so
// timed 90 turns keep their calibrated SPEED_ROTATE.
uint8_t ecoSpeed(uint8_t s) {
    if (!ecoMode) return s;
    int reduced = (int)s * 7 / 10;
    if (reduced < APPROACH_SPEED_FLOOR) reduced = APPROACH_SPEED_FLOOR;
    return (uint8_t)reduced;
}

void gridSetSpeed(uint8_t targetSpeed) {
    if (!ecoBypass) targetSpeed = ecoSpeed(targetSpeed);
    speed_Upper_L = targetSpeed; speed_Lower_L = targetSpeed;
    speed_Upper_R = targetSpeed; speed_Lower_R = targetSpeed;
}

// Gentle line-follow correction: both sides keep driving FORWARD, but the
// outer side runs faster than the inner side, arcing the car back onto the
// line. This replaces using Turn_Left()/Turn_Right() as the correction, which
// pivot the wheels in OPPOSITE directions (one side reverses) — a hard yank
// that reads as jerky when it fires every sensor tick during a line-follow.
// steerLeft=true arcs left (drifting back rightward), false arcs right.
void gridSteer(uint8_t baseSpeed, uint8_t boost, bool steerLeft) {
    baseSpeed = ecoSpeed(baseSpeed);
    uint8_t outer = (uint8_t)min(255, baseSpeed + boost);
    uint8_t inner = baseSpeed;
    if (steerLeft) {
        speed_Upper_L = inner; speed_Lower_L = inner;
        speed_Upper_R = outer; speed_Lower_R = outer;
    } else {
        speed_Upper_L = outer; speed_Lower_L = outer;
        speed_Upper_R = inner; speed_Lower_R = inner;
    }
    mecCar.Advance();
}

void gridStop() {
    mecCar.Stop();
    Bridge.println(F("[SYSTEM] Robot Halted."));
}

// Spins bypass the eco slowdown: the 90 turns are timed against SPEED_ROTATE,
// so slowing them would throw off the turn angle.
void spinRightInPlace(uint8_t speed) { ecoBypass = true; gridSetSpeed(speed); ecoBypass = false; mecCar.Turn_Right(); }
void spinLeftInPlace(uint8_t speed) { ecoBypass = true; gridSetSpeed(speed); ecoBypass = false; mecCar.Turn_Left(); }

void disableSensors() {
    sensorsEnabled = false;
    Bridge.println(F("[SYSTEM] Ultrasonic and Color sensors have been SOFTWARE DISABLED."));
}

// ── Non-blocking serial line reader ─────────────────────────────────────────
// Both the mid-move stop check and the idle command handler pull lines through
// here. Bytes are accumulated into a per-link buffer and a line is only
// returned once its terminating newline actually arrives, so we NEVER block
// waiting on the wire. This matters twice over:
//
//   * readStringUntil() blocks for its 1s timeout when the newline hasn't
//     landed yet. Calling that from inside a motion loop stalls the loop (and
//     the motors keep running) for up to a second per call.
//   * A byte-sniffing peek that bails on a non-match leaves those bytes in the
//     buffer forever. A STOP queued behind any other command would then sit
//     unread and never fire — the exact case where E-STOP must not fail.
//
// Returns true and fills `out` (trimmed, upper-cased) on a complete line.
struct LineReader {
    String buf;
    bool poll(Stream &link, String &out) {
        while (link.available()) {
            char c = (char)link.read();
            if (c == '\n') {
                out = buf;
                buf = "";
                out.trim();
                out.toUpperCase();
                if (out.length()) return true;
            } else if (c != '\r') {
                if (buf.length() < 64) buf += c;   // guard a runaway line
            }
        }
        return false;
    }
};

// One buffer per link. Both the idle path (handleSerialCommand) and the
// mid-move path (checkEmergencyStop) share these, so a partial line read
// during a move is completed by the next poll rather than being lost.
LineReader usbReader, espReader;

// A command that arrived while the car was mid-move but ISN'T a stop. We can't
// run it from inside the motion loop, so it's stashed here and picked up by
// handleSerialCommand() once the move returns — otherwise the mid-move poll
// would silently eat every non-STOP command the dashboard sent.
String deferredCmd = "";

// Poll both links for a STOP while a move is in progress. Non-STOP lines are
// deferred (see above) rather than dropped.
bool serialStopRequested() {
    String line;
    bool stop = false;
    if (usbReader.poll(Serial, line) || espReader.poll(espSerial, line)) {
        if (line == "STOP") stop = true;
        else deferredCmd = line;
    }
    return stop;
}

bool checkEmergencyStop() {
    if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.command == CMD_STAR) {
            emergencyStopActive = true;
            gridStop();
            Bridge.println(F("\n!!! EMERGENCY STOP TRIGGERED !!!"));
            IrReceiver.resume();
            return true;
        }
        IrReceiver.resume();
    }
    // Dashboard / wired STOP mid-move: abort just as hard as the IR remote.
    if (serialStopRequested()) {
        emergencyStopActive = true;
        gridStop();
        Bridge.println(F("\n!!! EMERGENCY STOP TRIGGERED (dashboard) !!!"));
        return true;
    }
    return emergencyStopActive;
}

// A drop-in replacement for delay() that stays responsive to an emergency stop.
// Plain delay() is the enemy of a working E-STOP: every ms spent inside it is a
// ms where a STOP sits unread and the motors keep doing whatever they were
// doing. Use this anywhere a blocking wait is long enough to notice (>~50ms).
// Returns true if a stop fired during the wait, so callers can bail out.
bool interruptibleDelay(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        if (checkEmergencyStop()) return true;
    }
    return false;
}

// Safety: detect the car running off the grid. Past the outermost grid line
// there is only blank mat, so all three line sensors read LOW. If that persists
// for LINE_LOST_TIMEOUT_MS (a real off-track condition, not a momentary wobble)
// this latches an emergency stop, exactly like a STOP press. `lostSince` is the
// caller's per-move timer: pass a local initialised to 0; it's stamped when the
// line is first lost and cleared whenever any sensor sees a line again.
// Call once per iteration of a forward-motion loop and return early if it's true.
bool checkOffGridStop(unsigned long &lostSince) {
    bool allLow = digitalRead(LINE_LEFT_PIN) == LOW &&
                  digitalRead(LINE_CENTER_PIN) == LOW &&
                  digitalRead(LINE_RIGHT_PIN) == LOW;
    if (!allLow) { lostSince = 0; return false; }
    if (lostSince == 0) { lostSince = millis(); return false; }
    if (millis() - lostSince < LINE_LOST_TIMEOUT_MS) return false;
    emergencyStopActive = true;
    gridStop();
    Bridge.println(F("\n!!! SAFETY STOP: line lost — car ran off the grid !!!"));
    return true;
}

// Outer-sensor latches for one forward move — the shared junction detector for
// every counting loop below. See JUNCTION_LATCH_MS for why a crossing is timed
// across two samples rather than read from one.
struct JunctionDetector {
    unsigned long leftSeenAt;
    unsigned long rightSeenAt;
    bool latched;
    // Gap between the two outer hits on the crossing update() last fired on:
    // +ve = LEFT led, -ve = RIGHT led. This is the real, measured LEFT-to-RIGHT
    // sensor offset at speed — the number JUNCTION_LATCH_MS has to span, and
    // which we have so far only guessed at. Recorded by the crossing trace.
    long lastOuterGapMs;

    // startOnJunction: true when the move begins with the car parked ON a node,
    // so the cross already under the wheels isn't counted as a fresh crossing.
    JunctionDetector(bool startOnJunction)
        : leftSeenAt(0), rightSeenAt(0), latched(startOnJunction),
          lastOuterGapMs(0) {}

    // Feed one sensor sample. Returns true exactly once per crossing, on the
    // sample that completes it.
    bool update(uint8_t left, uint8_t right) {
        unsigned long now = millis();
        if (left == HIGH) leftSeenAt = now;
        if (right == HIGH) rightSeenAt = now;

        bool onCross = leftSeenAt != 0 && rightSeenAt != 0 &&
                       (now - leftSeenAt) < JUNCTION_LATCH_MS &&
                       (now - rightSeenAt) < JUNCTION_LATCH_MS;
        // Not on a cross — re-arm for the next one. Note the latches outlive the
        // sensors by JUNCTION_LATCH_MS, so this also debounces the roll-off.
        if (!onCross) { latched = false; return false; }
        if (latched) return false;       // still the crossing we already counted
        latched = true;
        lastOuterGapMs = (long)leftSeenAt - (long)rightSeenAt;
        leftSeenAt = rightSeenAt = 0;    // spent: this cross can't count twice
        return true;
    }
};

// ── Crossing trace (diagnostic) ─────────────────────────────────────────────
// Answers the two questions we can't settle by watching the car: is a forward
// move DROPPING crossings, and how far apart do the outer sensors really land?
//
// Recorded into RAM and dumped only once the car has STOPPED, because printing
// from inside the move would destroy the measurement: espSerial is bit-banged
// SoftwareSerial whose write() blocks with interrupts disabled ~1ms/byte, so a
// log line mid-move stalls the sensor sampling and millis() alike — the very
// signals being measured. (The per-crossing [POS] line already pays that cost;
// see POS_HEARTBEAT_MS. That's a real overshoot source, but it lands after the
// count, so it can't hide a crossing.)
//
// Read the dump like this:
//   * sincePrev — ms between consecutive COUNTED crossings. On an even run
//     these should cluster around one cell-time. A gap that is ~2x the others
//     is a crossing the detector DROPPED, and it's proof rather than inference.
//   * outerGap — ms between the LEFT and RIGHT hits (+ve = LEFT led, as the
//     forward-mounted LEFT sensor should). This is the offset JUNCTION_LATCH_MS
//     must span: set the latch comfortably above the largest gap seen, but below
//     the smallest sincePrev, or a wobble can bridge two crossings into one.
// Set to 0 to compile the trace out (it costs flash, which is at ~84%).
#define CROSSING_TRACE 1
#if CROSSING_TRACE
struct CrossingRec {
    uint16_t sincePrevMs;
    int16_t outerGapMs;
};
const uint8_t CROSSING_TRACE_MAX = 10;
CrossingRec crossingTrace[CROSSING_TRACE_MAX];
uint8_t crossingTraceCount = 0;
unsigned long crossingTracePrevMs = 0;

void crossingTraceReset() {
    crossingTraceCount = 0;
    crossingTracePrevMs = millis();
}

void crossingTraceRecord(long outerGapMs) {
    unsigned long now = millis();
    if (crossingTraceCount < CROSSING_TRACE_MAX) {
        crossingTrace[crossingTraceCount].sincePrevMs = (uint16_t)(now - crossingTracePrevMs);
        crossingTrace[crossingTraceCount].outerGapMs = (int16_t)outerGapMs;
        crossingTraceCount++;
    }
    crossingTracePrevMs = now;
}

// Call only after gridStop() — never mid-move.
void crossingTraceDump() {
    if (!crossingTraceCount) return;
    Bridge.print(F("[TRACE] crossings="));
    Bridge.println(crossingTraceCount);
    for (uint8_t i = 0; i < crossingTraceCount; i++) {
        Bridge.print(F("[TRACE]  #"));
        Bridge.print(i + 1);
        Bridge.print(F(" sincePrev="));
        Bridge.print(crossingTrace[i].sincePrevMs);
        Bridge.print(F("ms outerGap="));
        Bridge.print(crossingTrace[i].outerGapMs);
        Bridge.println(F("ms"));
    }
}
#else
#define crossingTraceReset()
#define crossingTraceRecord(g)
#define crossingTraceDump()
#endif

bool gridRotateLeft90() {
    if (checkEmergencyStop()) return false;
    Bridge.println(F("\n[TURN] Symmetrical Spin 90 Degrees Left..."));
    // NOTE: no telemetry reads inside the spin loops — the ultrasonic/color
    // pulseIn calls block for tens of ms and make the turn angle inconsistent.
    unsigned long startTime = millis();
    spinLeftInPlace(SPEED_ROTATE);
    while (millis() - startTime < TURN_BLIND_LEFT_MS) {
        if (checkEmergencyStop()) return false;
    }
    // Stop on the CENTER sensor, not the offset LEFT sensor: center is on the
    // rotation axis so it reaches the new perpendicular line at a true ~90.
    while (true) {
        if (checkEmergencyStop()) return false;
        if (digitalRead(LINE_CENTER_PIN) == HIGH) break;
    }
    spinRightInPlace(SPEED_ROTATE + 10);

    unsigned long brakeStart = millis();
    while (millis() - brakeStart < BRAKE_MS) { if (checkEmergencyStop()) return false; }
    mecCar.Stop();

    unsigned long settleStart = millis();

    while (millis() - settleStart < TURN_SETTLE_MS) { if (checkEmergencyStop()) return false; }
    rotateTrackedHeading(false);   // completed a left 90 — pose follows the wheels
    return true;
}

bool gridRotateRight90() {
    if (checkEmergencyStop()) return false;
    Bridge.println(F("\n[TURN] Symmetrical Spin 90 Degrees Right..."));
    // NOTE: no telemetry reads inside the spin loops — the ultrasonic/color
    // pulseIn calls block for tens of ms and make the turn angle inconsistent.
    unsigned long startTime = millis();
    spinRightInPlace(SPEED_ROTATE);
    while (millis() - startTime < TURN_BLIND_RIGHT_MS) {
        if (checkEmergencyStop()) return false;
    }
    while (true) {
        if (checkEmergencyStop()) return false;
        if (digitalRead(LINE_RIGHT_PIN) == HIGH) break;
    }
    spinLeftInPlace(SPEED_ROTATE + 10);

    unsigned long brakeStart = millis();
    while (millis() - brakeStart < BRAKE_MS) { if (checkEmergencyStop()) return false; }
    mecCar.Stop();

    unsigned long settleStart = millis();
    while (millis() - settleStart < TURN_SETTLE_MS) { if (checkEmergencyStop()) return false; }
    rotateTrackedHeading(true);    // completed a right 90 — pose follows the wheels
    return true;
}

void gridMoveForwardBlocks(int targetBlocks, uint8_t startSpeed) {
    if (checkEmergencyStop()) return;
    int junctionCount = 0;
    JunctionDetector junctions(true);   // this move starts parked on a node
    unsigned long telemetryTickMillis = millis();
    unsigned long lineLostSince = 0;
    crossingTraceReset();

    while (junctionCount < targetBlocks) {
        if (checkEmergencyStop()) return;
        if (checkOffGridStop(lineLostSince)) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        // Count first, then steer: the detector owns the crossing decision, and
        // the branches below are only about keeping the car on the line.
        if (junctions.update(Left, Right)) {
            junctionCount++;
            crossingTraceRecord(junctions.lastOuterGapMs);
            advanceTrackedNode();   // crossed a junction — pose moves one node
            if (junctionCount == targetBlocks) break;
        }

        int calculatedSpeed = startSpeed - (junctionCount * 5);
        if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
        // On the final block (heading toward the last target junction), crawl
        // slower so the car approaches the last grid gently and stops right on
        // it instead of coasting past.
        if (junctionCount == targetBlocks - 1) calculatedSpeed = FINAL_BLOCK_CRAWL_SPEED;
        uint8_t currentSpeed = (uint8_t)calculatedSpeed;

        if (Left == HIGH && Center == HIGH && Right == HIGH) {
            gridSetSpeed(currentSpeed); mecCar.Advance();
        } else if (Left == LOW && Center == HIGH && Right == LOW) {
            gridSetSpeed(currentSpeed); mecCar.Advance();
        } else if (Left == LOW && Center == LOW && Right == HIGH) {
            gridSteer(currentSpeed, TURN_CORRECTION_BOOST, false);
        } else if (Left == HIGH && Center == LOW && Right == LOW) {
            gridSteer(currentSpeed, TURN_CORRECTION_BOOST, true);
        } else if (Left == HIGH && Center == HIGH && Right == LOW) {
            gridSteer(currentSpeed, TURN_CORRECTION_BOOST, true);
        } else if (Left == LOW && Center == HIGH && Right == HIGH) {
            gridSteer(currentSpeed, TURN_CORRECTION_BOOST, false);
        } else if (Left == LOW && Center == LOW && Right == LOW) {
            gridSetSpeed(currentSpeed); mecCar.Advance();
        }

        motionTelemetryTick(telemetryTickMillis);
    }

    if (junctionCount == targetBlocks && CENTER_OFFSET_MS > 0 && !emergencyStopActive) {
        gridSetSpeed(SPEED_MIN);
        mecCar.Advance();
        unsigned long offsetStart = millis();
        while(millis() - offsetStart < CENTER_OFFSET_MS) { if (checkEmergencyStop()) return; }
    }
    gridStop();
    crossingTraceDump();   // car is stopped — safe to spend serial time now
}

void gridMoveForwardOneCoord(uint8_t startSpeed) {
    if (checkEmergencyStop()) return;
    unsigned long telemetryTickMillis = millis();
    unsigned long lineLostSince = 0;
    // Phase 1 — clear the junction we're standing on. Drive until the sensors
    // confirm it's behind us (a junction reads L+R HIGH together; plain line
    // does not), NOT for a fixed time. This is what makes the move immune to
    // line width: a wide/bold junction simply takes longer to roll off.
    unsigned long leaveTime = millis();
    while (true) {
        if (checkEmergencyStop()) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        unsigned long elapsed = millis() - leaveTime;
        // Both outer sensors off the cross = the junction is genuinely behind
        // us. Debounced by a minimum time so sensor chatter on the way off the
        // line doesn't end this phase a few mm in.
        bool cleared = (Left == LOW && Right == LOW);
        if (elapsed >= JUNCTION_CLEAR_MIN_MS && cleared) break;
        if (elapsed >= JUNCTION_CLEAR_TIMEOUT_MS) {
            Bridge.println(F("[TRACK] Junction clear timed out — sensor stuck HIGH?"));
            break;
        }

        if (Left == LOW && Center == HIGH && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Advance(); }
        else if (Right == HIGH) gridSteer(startSpeed, TURN_CORRECTION_BOOST, false);
        else if (Left == HIGH) gridSteer(startSpeed, TURN_CORRECTION_BOOST, true);
        else { gridSetSpeed(startSpeed); mecCar.Advance(); }

        motionTelemetryTick(telemetryTickMillis);
    }

    // Phase 2 — hunt the next junction. Phase 1 only exits once BOTH outers are
    // LOW, so the detector starts off-cross: the first crossing it completes is
    // a genuinely new node.
    JunctionDetector junctions(false);
    while (true) {
        if (checkEmergencyStop()) return;
        if (checkOffGridStop(lineLostSince)) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        if (junctions.update(Left, Right)) break;
        else if (Left == LOW && Center == HIGH && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Advance(); }

        else if (Left == LOW && Center == LOW && Right == HIGH) gridSteer(startSpeed, TURN_CORRECTION_BOOST, false);
        else if (Left == HIGH && Center == LOW && Right == LOW) gridSteer(startSpeed, TURN_CORRECTION_BOOST, true);
        else if (Left == LOW && Center == LOW && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Advance(); }
        else if (Left == HIGH && Center == HIGH && Right == LOW) gridSteer(startSpeed, TURN_CORRECTION_BOOST, true);
        else if (Left == LOW && Center == HIGH && Right == HIGH) gridSteer(startSpeed, TURN_CORRECTION_BOOST, false);

        motionTelemetryTick(telemetryTickMillis);
    }

    // The junction is reached the moment the loop above breaks, so bank the pose
    // here rather than after the center-offset nudge below — that block can
    // return early on a stop, which would otherwise lose a node the car really
    // did travel.
    advanceTrackedNode();

    if (CENTER_OFFSET_MS > 0 && !emergencyStopActive) {
        gridSetSpeed(SPEED_MIN);
        mecCar.Advance();
        unsigned long offsetStart = millis();
        while(millis() - offsetStart < CENTER_OFFSET_MS) { if (checkEmergencyStop()) return; }
    }
    gridStop();
}

int gridGetDistanceCm() {
    if (!sensorsEnabled || emergencyStopActive) return -1;
    // Re-assert the ultrasonic pin modes on every read. Something between
    // setup() and here (servo attach/detach, IR begin, or the bit-banged
    // I2C motor bus toggling SDA/SCL) can leave TRIG/ECHO in the wrong mode,
    // which makes pulseIn() return 0 forever (distance shows "---"). The
    // working Ultrasonic+Color+Gripper sketch re-asserts these before each
    // ping too.
    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000UL);
    if (duration == 0) {
        if (debugTelemetry) {
            Bridge.print(F("[DBG] Distance: --- (out of range) rawDuration="));
            Bridge.print(duration);
            Bridge.print(F(" echo="));
            Bridge.println(digitalRead(ULTRASONIC_ECHO_PIN));
        }
        return -1;
    }
    int dist = (int)(duration / 58.2);
    if (debugTelemetry) {
        Bridge.print(F("[DBG] Distance: "));
        Bridge.print(dist);
        Bridge.print(F(" cm (duration="));
        Bridge.print(duration);
        Bridge.println(F(")"));
    }
    return dist;
}

bool checkItem() {
    if (emergencyStopActive) return false;
    int distance = gridGetDistanceCm();
    int color = gridDetectColorValue();
    printStatusTelemetry(distance, color);
    return (distance > 0 && distance <= ITEM_DETECT_DISTANCE_CM);
}

// Reads one color channel, averaged over several samples for stability.
// A pulseIn timeout returns 0; we map that to the max timeout value so a
// dropped reading counts as "very little of this color" (largest pulse width)
// instead of falsely winning the "smallest = this color" comparison.
unsigned long gridReadColorChannel(bool s2, bool s3) {
    if (emergencyStopActive) return COLOR_PULSE_TIMEOUT_US;
    digitalWrite(COLOR_S2_PIN, s2 ? HIGH : LOW);
    digitalWrite(COLOR_S3_PIN, s3 ? HIGH : LOW);
    delay(10); // let the sensor's filter settle after switching channels

    const int COLOR_SAMPLES = 3;
    unsigned long total = 0;
    for (int i = 0; i < COLOR_SAMPLES; i++) {
        unsigned long p = pulseIn(COLOR_OUT_PIN, LOW, COLOR_PULSE_TIMEOUT_US);
        if (p == 0) p = COLOR_PULSE_TIMEOUT_US; // timeout -> treat as max, not min
        total += p;
    }
    return total / COLOR_SAMPLES;
}

// When true, gridDetectColorValue() prints its raw R/G/B pulse values on every
// read regardless of debugTelemetry. Turned on around the grab decision so the
// actual numbers are visible for tuning without enabling full telemetry spam.
bool logColorRaw = false;

int gridDetectColorValue() {
    if (!sensorsEnabled || emergencyStopActive) return ANY;
    unsigned long red = gridReadColorChannel(LOW, LOW);
    unsigned long green = gridReadColorChannel(HIGH, HIGH);
    unsigned long blue = gridReadColorChannel(LOW, HIGH);

    int result = ANY;
    if (blue < red && blue < green) result = BLUE;
    else if (red < blue && green < blue && red < green * 1.4 && green < red * 1.4) result = YELLOW;
    else if (red < green && red < blue) result = RED;

    if (debugTelemetry || logColorRaw) {
        Bridge.print(F("[DBG] Color raw R=")); Bridge.print(red);
        Bridge.print(F(" G=")); Bridge.print(green);
        Bridge.print(F(" B=")); Bridge.print(blue);
        Bridge.print(F(" -> "));
        Bridge.println(gridColorName(result));
    }
    return result;
}

// Reads the color several times and returns the most common result — a single
// noisy sample can't flip the decision. Used at grab time where a wrong read
// means the wrong block (or no) grab.
int gridDetectColorMajority() {
    int votes[4] = {0, 0, 0, 0}; // RED, BLUE, YELLOW, ANY
    const int COLOR_VOTES = 5;
    for (int i = 0; i < COLOR_VOTES; i++) {
        int c = gridDetectColorValue();
        if (c >= 0 && c <= ANY) votes[c]++;
    }
    // Pick the most-voted concrete color; ANY only wins if nothing else did.
    int best = ANY, bestCount = votes[ANY];
    for (int c = RED; c <= YELLOW; c++) {
        if (votes[c] > bestCount) { best = c; bestCount = votes[c]; }
    }
    return best;
}

const char *gridColorName(int color) {
    if (color == RED) return "RED";
    if (color == BLUE) return "BLUE";
    if (color == YELLOW) return "YELLOW";
    return "ANY / OTHERS";
}

// NOTE: deliberately has no emergencyStopActive guard. Releasing the gripper is
// always safe and is the natural recovery after a stop that latched mid-grab —
// refusing to run it while stopped would strand a block in a clamped claw.
void openClawWithAttach() {
    clawServo.attach(CLAW_SERVO_PIN);
    // Re-send the open angle a few times with settle pauses in between. If the
    // claw was gripping an object under stall current, a single write can be
    // dropped by a momentary brownout — repeating it gives the servo more
    // chances to actually reach the open position before we detach.
    // Not interruptible on purpose: opening the claw is a SAFE, recovery action
    // (it releases whatever is held). Aborting it on a latched stop would leave
    // the gripper clamped shut with no way to open it. It's ~1.2s of servo
    // settle, and the motors are already halted whenever a stop is latched.
    for (int i = 0; i < 3; i++) {
        clawServo.write(CLAW_OPEN_ANGLE);
        delay(400);
    }
    clawServo.detach();
    clawCurrentAngle = CLAW_OPEN_ANGLE;
    // Releasing an item re-arms the idle auto-grab so the next object can be
    // picked up again.
    itemGrabbed = false;
}

// Manual nudge for the LEFT/RIGHT arrow buttons: steps the claw a few degrees
// toward open or closed from wherever it currently sits, instead of jumping
// straight to the open/closed extreme.
const uint8_t CLAW_NUDGE_STEP = 8;
void clawNudge(bool towardClosed) {
    if (emergencyStopActive) return;
    int target = towardClosed ? clawCurrentAngle + CLAW_NUDGE_STEP
                               : clawCurrentAngle - CLAW_NUDGE_STEP;
    target = constrain(target, CLAW_OPEN_ANGLE, CLAW_CLOSED_ANGLE);
    clawServo.attach(CLAW_SERVO_PIN);
    clawServo.write(target);
    delay(200);
    clawServo.detach();
    clawCurrentAngle = target;
    Bridge.print(F("[GRIPPER] Nudged to "));
    Bridge.println(clawCurrentAngle);
}

void gridSlightReverse() {
    if (checkEmergencyStop()) return;
    Bridge.println(F("[TRACK] Execution of brief post-turn alignment reverse..."));
    gridSetSpeed(SPEED_REVERSE_BUMP);
    mecCar.Back();
    unsigned long startTime = millis();
    while (millis() - startTime < REVERSE_BUMP_MS) { if (checkEmergencyStop()) return; }
    gridStop();
}

// Slow-sweep close of the claw and latch the grabbed state. Shared by the
// path-driven grabColor() and the idle auto-grab in loop().
void closeClawGrip() {
    if (interruptibleDelay(GRAB_SETTLE_MS)) return;  // full stop before gripping

    clawServo.attach(CLAW_SERVO_PIN);

    // SLOW SWEEP: Gradually close the claw instead of snapping it. The sweep is
    // ~2.5s of blocking delay, so it polls for a stop between steps: an E-STOP
    // pressed mid-grab must abort here, not after the claw has already closed.
    // On abort we leave the servo where it stopped and detach (cutting drive)
    // rather than continuing to close on whatever is in the gripper.
    for (int angle = CLAW_OPEN_ANGLE; angle <= CLAW_CLOSED_ANGLE; angle += 2) {
        clawServo.write(angle);
        clawCurrentAngle = angle;
        if (interruptibleDelay(15)) { clawServo.detach(); return; }
    }

    if (interruptibleDelay(800)) { clawServo.detach(); return; }  // firm grip

    clawServo.detach();
    clawCurrentAngle = CLAW_CLOSED_ANGLE;

    itemGrabbed = true;
    disableSensors();

    interruptibleDelay(GRAB_SETTLE_MS); // let the grip firm up before moving
}

// Reads the block color (majority vote) and grips ONLY if it matches `color`
// (or `color == ANY`). Returns true if the claw actually closed on the block,
// false on a color mismatch (claw left open, car untouched). Callers use the
// return value to decide whether to keep searching the other blocks.
bool grabColorIfMatch(int color) {
    if (emergencyStopActive) return false;
    logColorRaw = true;                    // show raw R/G/B for each vote read
    int detected = gridDetectColorMajority();
    logColorRaw = false;

    Bridge.print(F("[GRAB] target="));
    Bridge.print(gridColorName(color));
    Bridge.print(F(" detected="));
    Bridge.println(gridColorName(detected));

    if (detected == color || color == ANY) {
        closeClawGrip();
        return true;
    }
    Bridge.println(F("[GRAB] Color mismatch — skipping grip."));
    return false;
}

// Backward-compatible wrapper: grips on match, ignores the result.
void grabColor(int color) {
    grabColorIfMatch(color);
}

bool isGrabTargetReached(int currentDistance) {
    return (currentDistance > 0 && currentDistance <= GRAB_APPROACH_DISTANCE_CM);
}

// startSpeed is the speed used while the object is still far off; the closer
// currentDistance gets to the grab threshold, the more it's scaled down
// toward the floor so the car visibly slows into the grip instead of
// creeping at one constant speed the whole approach. Paths that pass
// SPEED_MIN (their original default) keep SPEED_MIN as the floor, unchanged.
// A caller starting below SPEED_MIN (e.g. the UP button's 40) instead floors
// at APPROACH_SPEED_FLOOR, so it still has real room to taper down further.
void executeApproachMovement(int currentDistance, uint8_t startSpeed) {
    if (emergencyStopActive) return;
    unsigned long telemetryTickMillis = millis();
    uint8_t floorSpeed = (startSpeed < SPEED_MIN) ? min(APPROACH_SPEED_FLOOR, startSpeed) : SPEED_MIN;

    while (!isGrabTargetReached(currentDistance)) {
        if (checkEmergencyStop()) return;
        int color = gridDetectColorValue();
        int distance = gridGetDistanceCm();
        if (debugTelemetry) {
            Bridge.print(F("[APPROACH] "));
            printStatusTelemetry(distance, color);
        }

        uint8_t approachSpeed = startSpeed;
        if (distance > 0) {
            // Linearly ramp down from startSpeed at ITEM_DETECT_DISTANCE_CM
            // down to floorSpeed at GRAB_APPROACH_DISTANCE_CM.
            int span = ITEM_DETECT_DISTANCE_CM - GRAB_APPROACH_DISTANCE_CM;
            int clamped = constrain(distance, GRAB_APPROACH_DISTANCE_CM, ITEM_DETECT_DISTANCE_CM);
            int scaled = floorSpeed + (long)(startSpeed - floorSpeed) * (clamped - GRAB_APPROACH_DISTANCE_CM) / span;
            approachSpeed = (uint8_t)constrain(scaled, floorSpeed, startSpeed);
        }

        // Stay centered on the line while creeping toward the block — same
        // correction branches as the line-follow functions. Without this the
        // approach drove straight blind and could drift off track over the
        // (up to ITEM_DETECT_DISTANCE_CM) approach distance.
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);
        if (Left == LOW && Center == LOW && Right == HIGH) {
            gridSteer(approachSpeed, APPROACH_CORRECTION_BOOST, false);
        } else if (Left == HIGH && Center == LOW && Right == LOW) {
            gridSteer(approachSpeed, APPROACH_CORRECTION_BOOST, true);
        } else if (Left == HIGH && Center == HIGH && Right == LOW) {
            gridSteer(approachSpeed, APPROACH_CORRECTION_BOOST, true);
        } else if (Left == LOW && Center == HIGH && Right == HIGH) {
            gridSteer(approachSpeed, APPROACH_CORRECTION_BOOST, false);
        } else {
            gridSetSpeed(approachSpeed);
            mecCar.Advance();
        }
        delay(60);
        currentDistance = gridGetDistanceCm();
        motionTelemetryTick(telemetryTickMillis);
    }
    gridStop();
}

// Rotates in place until currentHeading matches targetHeading, always
// turning the short way round the NORTH/EAST/SOUTH/WEST compass.
// The rotate primitives now update currentHeading themselves, so this loop just
// picks a direction and spins until the heading matches — it must NOT also step
// the heading, or every turn would count twice.
void turnToHeading(int targetHeading) {
    while (currentHeading != targetHeading) {
        if (checkEmergencyStop()) return;
        int diff = (targetHeading - currentHeading + 4) % 4;
        if (diff == 1) {
            if (!gridRotateRight90()) return;
        } else {
            if (!gridRotateLeft90()) return;
        }
    }
}

// Drives the car from the current tracked (currentX, currentY) grid node to
// (targetX, targetY), one block at a time, using the line-following
// one-coordinate step and the tracked heading to know which way is which.
void moveCoord(int targetX, int targetY) {
    if (checkEmergencyStop()) return;

    int clampedX = constrain(targetX, GRID_MIN_X, GRID_MAX_X);
    int clampedY = constrain(targetY, GRID_MIN_Y, GRID_MAX_Y);
    if (clampedX != targetX || clampedY != targetY) {
        Bridge.print(F("[NAVIGATE] Target ("));
        Bridge.print(targetX); Bridge.print(F(",")); Bridge.print(targetY);
        Bridge.print(F(") outside grid bounds — clamped to ("));
        Bridge.print(clampedX); Bridge.print(F(",")); Bridge.print(clampedY);
        Bridge.println(F(")."));
    }
    targetX = clampedX;
    targetY = clampedY;

    // gridMoveForwardOneCoord() steps the tracked node itself, so these loops
    // only drive — they must not also bump currentX/currentY.
    int deltaX = targetX - currentX;
    if (deltaX != 0) {
        turnToHeading((deltaX > 0) ? EAST : WEST);
        for (int i = 0; i < abs(deltaX); i++) {
            if (checkEmergencyStop()) return;
            gridMoveForwardOneCoord(SPEED_START_FAST);
            if (emergencyStopActive) return;
        }
    }

    int deltaY = targetY - currentY;
    if (deltaY != 0) {
        turnToHeading((deltaY > 0) ? NORTH : SOUTH);
        for (int i = 0; i < abs(deltaY); i++) {
            if (checkEmergencyStop()) return;
            gridMoveForwardOneCoord(SPEED_START_FAST);
            if (emergencyStopActive) return;
        }
    }

    Bridge.println(F("[NAVIGATE] Target Node Reached. Aligning to WEST baseline..."));
    turnToHeading(WEST);
    gridStop();
}

// Approaches the nearest object with the ultrasonic sensor and grabs it if
// its color matches targetColour (or always, if targetColour == ANY).
// approachStartSpeed lets callers (e.g. the UP button) creep in slower than
// the scripted paths' default.
// Returns true if a block of the target colour was actually grabbed.
bool moveToGrab(int targetColour, uint8_t approachStartSpeed = SPEED_MIN) {
    if (checkEmergencyStop()) return false;
    int distance = gridGetDistanceCm();
    executeApproachMovement(distance, approachStartSpeed);
    if (emergencyStopActive) return false;
    return grabColorIfMatch(targetColour);
}

void moveHome() {
    if (checkEmergencyStop()) return;
    moveCoord(7, 3); // Leveraging existing moveCoord instead of repeating logic
    if (emergencyStopActive) return;
    Bridge.println(F("[NAVIGATE] Home Node Reached. Aligning to EAST baseline..."));
    turnToHeading(EAST);
    gridStop();
}

void goToDrop() {
    if (checkEmergencyStop()) return;
    int junctionsEncountered = 0;
    bool activeJunctionFlag = true;
    unsigned long lineLostSince = 0;
    gridSetSpeed(SPEED_START_FAST);

    while (junctionsEncountered < 4) {
        if (checkEmergencyStop()) return;
        if (checkOffGridStop(lineLostSince)) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        if (Left == HIGH && Center == HIGH && Right == HIGH) {
            if (!activeJunctionFlag) {
                junctionsEncountered++;
                activeJunctionFlag = true;
                if (junctionsEncountered == 4) break;
            }
            mecCar.Advance();
        } else if (Left == LOW && Center == HIGH && Right == LOW) {
            activeJunctionFlag = false; mecCar.Advance();
        } else if (Right == HIGH) {
            activeJunctionFlag = false; mecCar.Turn_Right();
        } else if (Left == HIGH) {
            activeJunctionFlag = false; mecCar.Turn_Left();
        } else { mecCar.Advance(); }
    }
    mecCar.Stop();
    if (!emergencyStopActive) Bridge.println(F("[SYSTEM] Drop zone reached. Stopped instantly."));
}

void resetCoordinates() {
    currentX = 7; currentY = 3; currentHeading = WEST;
    emergencyStopActive = false;
    Bridge.println(F("[SYSTEM] Navigation Tracker Reset to Default Home Baseline (7,3) facing WEST."));
    reportPosition();
}

// ── Color-search across the 3 blocks ─────────────────────────────────────────
// The three blocks sit in a vertical line at grid column X=0, rows Y=1/3/5,
// each approached facing WEST. The robot navigates on the grid node one column
// east of each block (X=1) and lets moveToGrab() creep the final stretch WEST
// onto the block to read its colour and grip.
//
// On a colour mismatch it reverses off the block and hops to the nearest
// unchecked block (rows are 2 apart), re-approaches, and re-reads — repeating
// until the target colour is grabbed or all three blocks are exhausted. If none
// match, it drives home and stops (no grab).
//
// Returns the block row (1/3/5) that was grabbed, or -1 if none matched.
const int BLOCK_ROWS[3] = {1, 3, 5};
const uint8_t SEARCH_APPROACH_SPEED = PATH_APPROACH_SPEED;

// THE row-change lane. Every row change in the firmware happens on this column:
// the search's run out to the first block, every hop between blocks, and the
// carry-home return with a block held. Column 4 is the clear middle of the
// collecting grid and the same column the scripted paths turn on, so every
// route — searched, scripted, outbound or returning — traces the same lane.
// It must stay well clear of the block column (1): changing row any nearer
// would sweep the car past the blocks.
const int ROW_CHANGE_X = 4;
// Column the approach stops on: one node EAST of the block at column 1.
// moveToGrab() creeps the last cell with the ultrasonic from here, so the car
// never drives onto column 1 and clashes with the block.
const int GRAB_APPROACH_X = 2;

// Backs the car off the block until it re-acquires the grid node (all three
// line sensors on the junction cross = all HIGH), so the coordinate-tracked
// navigation to the next block starts from a known node. The grab approach
// crept WEST off the node onto the block, so a fixed slight-reverse isn't
// enough to guarantee we're back on the junction — reverse until we see it.
static void backOffBlock() {
    if (checkEmergencyStop()) return;
    Bridge.println(F("[SEARCH] Reversing off block to re-acquire grid node..."));
    gridSetSpeed(SPEED_REVERSE_BUMP);
    mecCar.Back();
    // Cap the reverse so a missed junction can't run the car off the mat.
    unsigned long startTime = millis();
    const unsigned long BACKOFF_TIMEOUT_MS = 2500;
    // The approach crept off the node, so we start off-cross. Reversing puts the
    // offset LEFT sensor onto the cross LAST rather than first, but it's the same
    // problem either way — the outers land at different times (JUNCTION_LATCH_MS).
    JunctionDetector junctions(false);
    while (millis() - startTime < BACKOFF_TIMEOUT_MS) {
        if (checkEmergencyStop()) return;
        if (junctions.update(digitalRead(LINE_LEFT_PIN), digitalRead(LINE_RIGHT_PIN))) {
            break; // back on the junction
        }
    }
    gridStop();
    if (!emergencyStopActive) delay(STEP_TRANSITION_MS);
}

// Reverses the car exactly one grid node EAST while it keeps facing WEST (so the
// held block keeps leading and NO turn is needed). Assumes it starts on a
// junction — e.g. right after backOffBlock: drives blind briefly to clear the
// current junction cross, then reverses until the next junction (all 3 line
// sensors HIGH). Bumps currentX by +1. Called repeatedly by the post-grab return
// to pull back onto the clear column-4 row-change lane without a 180° turn.
static void reverseOneCoordEast() {
    if (checkEmergencyStop()) return;
    Bridge.println(F("[RETURN] Reversing one node east onto the corridor..."));
    gridSetSpeed(SPEED_REVERSE_BUMP);
    mecCar.Back();
    unsigned long t0 = millis();                 // clear the current junction (blind)
    while (millis() - t0 < 400) { if (checkEmergencyStop()) return; }
    unsigned long startTime = millis();          // then reverse to the next junction
    const unsigned long REVERSE_TIMEOUT_MS = 2500;
    JunctionDetector junctions(false);           // the blind clear above left the cross
    while (millis() - startTime < REVERSE_TIMEOUT_MS) {
        if (checkEmergencyStop()) return;
        if (junctions.update(digitalRead(LINE_LEFT_PIN), digitalRead(LINE_RIGHT_PIN))) {
            break;
        }
    }
    gridStop();
    currentX += 1;                               // moved one node east
    reportPosition();
    if (!emergencyStopActive) delay(STEP_TRANSITION_MS);
}

int searchAndGrab(int targetColour, int startRow) {
    // Visit order: start block first, then remaining blocks nearest-first by
    // row distance from the current block.
    int order[3];
    order[0] = startRow;
    int n = 1;
    // Fill the rest sorted by |row - previousRow| so each hop is to the closest
    // unchecked block.
    bool used[3] = {false, false, false};
    for (int i = 0; i < 3; i++) if (BLOCK_ROWS[i] == startRow) used[i] = true;
    int fromRow = startRow;
    while (n < 3) {
        int bestIdx = -1, bestDist = 999;
        for (int i = 0; i < 3; i++) {
            if (used[i]) continue;
            int d = abs(BLOCK_ROWS[i] - fromRow);
            if (d < bestDist) { bestDist = d; bestIdx = i; }
        }
        used[bestIdx] = true;
        order[n++] = BLOCK_ROWS[bestIdx];
        fromRow = BLOCK_ROWS[bestIdx];
    }

    for (int i = 0; i < 3; i++) {
        if (checkEmergencyStop()) return -1;
        int row = order[i];
        Bridge.print(F("[SEARCH] Checking block at row "));
        Bridge.println(row);

        // Change row at column 4, then run straight west to the approach node.
        // moveCoord does its X move before its Y move, so targeting
        // (ROW_CHANGE_X, row) first pulls the car out to column 4 and only then
        // changes rows — every row change, first block and inter-block hop
        // alike, happens on the clear column-4 lane rather than along the block
        // column. The second moveCoord is then a pure westward run (4 -> 2) with
        // no turn, which also lets the line-follow re-centre the car before the
        // ultrasonic creeps it onto the block. Same lane the scripted paths fly.
        moveCoord(ROW_CHANGE_X, row);
        if (emergencyStopActive) return -1;
        moveCoord(GRAB_APPROACH_X, row);
        if (emergencyStopActive) return -1;
        turnToHeading(WEST);

        sensorsEnabled = true;
        bool grabbed = moveToGrab(targetColour, SEARCH_APPROACH_SPEED);
        if (emergencyStopActive) return -1;

        if (grabbed) {
            Bridge.print(F("[SEARCH] Match found and grabbed at row "));
            Bridge.println(row);
            return row;
        }

        // Wrong colour: back off and (if any remain) hop to the next block.
        Bridge.println(F("[SEARCH] Wrong colour — reversing to try next block."));
        backOffBlock();
        if (emergencyStopActive) return -1;
    }

    Bridge.println(F("[SEARCH] No matching block found — returning home."));
    moveHome();
    return -1;
}

// Carries a just-grabbed block back and out to the drop zone. Entered right
// after the grab with the car at column 2, currentY = the grabbed row, facing
// WEST (block held in front). Route: back off to the block's junction, reverse
// east onto the column-4 row-change lane (still WEST-facing — no turn), funnel
// onto the central row 3 ("path 2"), then run east through home out to the drop
// node (9,3), where the caller releases the grip.
//
// Turning: a block from row 1 or row 5 reaches row 3 with one 90° turn onto the
// corridor and another 90° to face east — never a 180°. A block already on row 3
// (path 2) has no row change to reorient it, so facing east there costs a 180°;
// that is the only case that needs one.
static void carryHomeViaRow3() {
    if (checkEmergencyStop()) return;
    int grabbedRow = currentY;

    backOffBlock();                 // reverse to (2, grabbedRow) junction, facing WEST
    if (emergencyStopActive) return;

    // Reverse east onto the shared row-change lane (column 4) — the same column
    // the outbound search and the scripted paths change row on, so the carried
    // block travels the identical lane it came out on. Reversing (rather than
    // turning) keeps the car facing WEST with the held block leading, so no turn
    // is needed here and the block never sweeps across the grid.
    while (currentX < ROW_CHANGE_X) {
        reverseOneCoordEast();
        if (emergencyStopActive) return;
    }

    // Funnel onto the central row 3 along the column-4 lane (rows 1/5 only).
    if (grabbedRow != 3) {
        int dir = (grabbedRow > 3) ? SOUTH : NORTH;
        turnToHeading(dir);
        if (emergencyStopActive) return;
        int steps = abs(grabbedRow - 3);
        for (int i = 0; i < steps; i++) {
            if (checkEmergencyStop()) return;
            gridMoveForwardOneCoord(SPEED_START_FAST);   // steps currentY itself
            if (emergencyStopActive) return;
        }
    }

    // Face east and run along row 3 all the way out to the drop node (9,3):
    // through home (X=7) and two more nodes east into the START zone, where the
    // grip is released. From row 1/5 the turn onto east is a 90°; from row 3
    // (path 2) it is the one 180° the route allows.
    turnToHeading(EAST);
    if (emergencyStopActive) return;
    // gridMoveForwardOneCoord() advances currentX (heading is EAST), which is
    // what terminates this loop.
    while (currentX < 9) {
        if (checkEmergencyStop()) return;
        gridMoveForwardOneCoord(SPEED_START_FAST);
        if (emergencyStopActive) return;
    }

    Bridge.println(F("[RETURN] Drop node (9,3) reached — releasing grip."));
    gridStop();                     // stop on (9,3); caller opens the claw here
}

// After releasing the block at the drop node (9,3), drive back home to (7,3):
// reverse a little so the in-place 180° spin doesn't sweep the claw into the
// just-dropped block, turn 180° to face WEST, then line-follow the two nodes
// west onto home. Called after the claw has opened and the block is clear for
// the robot arm to pick up. Entered at (9,3) facing EAST.
static void returnHomeFromDrop() {
    if (checkEmergencyStop()) return;

    // Back off the drop point so the 180° spin clears the dropped block.
    Bridge.println(F("[RETURN] Backing off the drop point before turning home..."));
    gridSetSpeed(SPEED_REVERSE_BUMP);
    mecCar.Back();
    unsigned long t0 = millis();
    while (millis() - t0 < REVERSE_BUMP_MS) { if (checkEmergencyStop()) return; }
    gridStop();

    // Spin 180° in place to face home (EAST -> WEST): turnToHeading takes the
    // short way, which for a 180 is two left-90 spins.
    turnToHeading(WEST);
    if (emergencyStopActive) return;

    // Line-follow west back onto home (7,3), one node at a time.
    // gridMoveForwardOneCoord() decrements currentX (heading is WEST), which is
    // what terminates this loop.
    while (currentX > 7) {
        if (checkEmergencyStop()) return;
        gridMoveForwardOneCoord(SPEED_START_FAST);
        if (emergencyStopActive) return;
    }
    gridStop();
    Bridge.println(F("[RETURN] Home (7,3) reached."));
}

// Full mission with colour search: hunt the target colour across the 3 blocks,
// and if one is grabbed, carry it home and drop it. Starts the search at the
// block associated with the pressed button (path1->row1, path2->row3,
// path3->row5).
void searchGrabAndDrop(int targetColour, int startRow) {
    if (checkEmergencyStop()) return;
    openClawWithAttach();          // start with an open gripper
    resetCoordinates();            // assume we begin at home (7,3) facing WEST

    int grabbedRow = searchAndGrab(targetColour, startRow);
    if (emergencyStopActive) return;

    if (grabbedRow < 0) {
        // searchAndGrab already drove home on a total miss.
        gridStop();
        return;
    }

    // Carry the grabbed block back and out to the drop zone: reverse onto the
    // column-4 row-change lane, funnel onto row 3, then run east to home and to
    // the drop — avoiding a 180° turn except for a row-3 (path 2) block.
    carryHomeViaRow3();
    if (emergencyStopActive) return;
    delay(300);
    openClawWithAttach();          // release the block at the drop node (9,3)
    delay(300);
    returnHomeFromDrop();          // reverse, spin 180°, drive back to home (7,3)
    if (emergencyStopActive) return;
    resetCoordinates();
}

// ── Array Based Instruction Execution ───────────────────────────────────────
enum Action { FWD, LFT, RGT, GRB, REV, DRP, GDR, DLY };
struct Step { Action act; uint8_t arg; };

// Every FWD count below is written as a column/row delta against the tracked
// pose, starting from home (7,3) facing WEST. Two invariants hold across all
// three, and they are the same ones searchForColour()/executeAutoMission()
// already obey:
//
//   * The car CHANGES ROW AT COLUMN 4 — never on the block column and never in
//     the START zone. Column 4 is the clear middle of the collecting grid.
//   * The last FWD before a GRB stops on COLUMN 2, one node east of the block
//     at column 1. moveToGrab() creeps the remaining cell with the ultrasonic
//     and halts at GRAB_APPROACH_DISTANCE_CM. gridMoveForwardBlocks() only
//     counts junctions and never consults the ultrasonic, so a FWD that counted
//     as far as column 1 would drive the chassis into the block.

const Step path1[] = {
    // Outbound: FWD 3 = col 7->4, drop to row 1 at col 4 (the row change), then
    // FWD 2 = col 4->2 gives a straight line-follow run so the car re-centers
    // after the RGT turn before the ultrasonic creeps onto the block at column 1.
    {FWD, 3}, {DLY, 3}, {LFT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3},
    {FWD, 2}, {DLY, 3}, {GRB, ANY}, {DLY, 3}, {REV, 0}, {DLY, 3},
    // No 180: back off the block, single RGT to face row 3, cross to row 3,
    // then RGT east and run the row-3 corridor. FWD 7 counts columns 2..9 so
    // the car stops ON the drop node (9,3) and releases the block there.
    {RGT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {FWD, 7}, {DLY, 2}, {DRP, 0},
    {REV, 2}
};

const Step path2[] = {
    // The block sits on row 3, the same row as home — no row change, so no turn
    // at column 4. FWD 5 = col 7->2, stopping one node east of the block.
    {FWD, 5}, {DLY, 3}, {GRB, ANY}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {REV, 0}, {DLY, 3},
    // FWD 7 counts columns 2..9 so the car stops ON the drop node (9,3) and
    // releases the block there.
    {RGT, 0}, {DLY, 3}, {FWD, 7}, {DLY, 2}, {DRP, 0},
    {REV, 2}
};
//GDR was = 0
const Step path3[] = {
    // Outbound (mirror of path1): FWD 3 = col 7->4, rise to row 5 at col 4 (the
    // row change), then FWD 2 = col 4->2 straight run before the grab approach.
    {FWD, 3}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3},
    {FWD, 2}, {DLY, 3}, {GRB, ANY}, {DLY, 3}, {REV, 0}, {DLY, 3},
    // No 180: back off the block, single LFT to face row 3, cross to row 3,
    // then LFT east and run the row-3 corridor. FWD 7 counts columns 2..9 so
    // the car stops ON the drop node (9,3) and releases the block there.
    {LFT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3}, {FWD, 7}, {DLY, 2}, {DRP, 0},
    {REV, 2}
};

// Returns true if the next non-DLY step at or after index `from` is a GRB.
// Used to enable the ultrasonic/color sensors only for the final forward that
// approaches the block, keeping them off during all other navigation.
static bool nextRealStepIsGrab(const Step* path, int len, int from) {
    for (int j = from; j < len; j++) {
        if (path[j].act == DLY) continue;
        return path[j].act == GRB;
    }
    return false;
}

// Add 'int targetColour' to the parameters
void runPath(const Step* path, int len, int targetColour) {
    // Sensors (ultrasonic + color) stay OFF during navigation and only turn on
    // for the last forward that approaches the block (the FWD immediately
    // before a GRB) and the GRB itself.
    sensorsEnabled = false;
    openClawWithAttach();   // ensure the gripper starts open before any sequence runs
    for (int i = 0; i < len; i++) {
        if (emergencyStopActive) break;
        switch(path[i].act) {
            case FWD:
                // Enable sensors if this forward leads straight into a grab.
                sensorsEnabled = nextRealStepIsGrab(path, len, i + 1);
                gridMoveForwardBlocks(path[i].arg, SPEED_START_FAST);
                break;
            case LFT: sensorsEnabled = false; gridRotateLeft90(); break;
            case RGT: sensorsEnabled = false; gridRotateRight90(); break;
            case GRB: sensorsEnabled = true; moveToGrab(targetColour, PATH_APPROACH_SPEED); break;
            case REV: gridSlightReverse(); break;
            case DRP: openClawWithAttach(); break;

            case GDR: goToDrop(); break;
            case DLY: delay(path[i].arg * 100); break;
        }
        // Brief settling pause between steps so transitions aren't abrupt.
        if (!emergencyStopActive) delay(STEP_TRANSITION_MS);
    }
}

void executeAutoMission(int targetY, int color) {
    sensorsEnabled = true;
    gridMoveForwardBlocks(2, SPEED_START_FAST);
    moveCoord(2, targetY);
    if (emergencyStopActive) return;
    delay(300);
    moveToGrab(color);
    if (emergencyStopActive) return;
    delay(300);
    moveHome();
    goToDrop();
    if (emergencyStopActive) return;
    delay(300);
    openClawWithAttach();
    resetCoordinates();
}

// ── Serial Command Control (Dashboard Debugging) ─────────────────────────────
// Lets the web dashboard drive individual movements over USB for step-by-step
// debugging, reusing the exact same functions the IR paths call. Commands are
// newline-terminated ASCII, e.g. "FWD:2", "LFT", "GRB:0", "STOP". Every reply
// is prefixed "[ACK]" so the dashboard can distinguish command echoes from
// normal telemetry.
void handleSerialCommand() {
    // Accept commands from EITHER link: the USB Serial Monitor (wired debug) or
    // the ESP32 wireless bridge on espSerial. Read a line from whichever has
    // data this tick; replies go out through Bridge (both links) so both
    // dashboards see the [ACK]/telemetry response.
    // Read through the same non-blocking readers the mid-move stop check uses,
    // so the two paths can't steal partial lines from each other. A command
    // that arrived mid-move was stashed in deferredCmd — run it first.
    String line;
    if (deferredCmd.length()) {
        line = deferredCmd;
        deferredCmd = "";
    } else if (!usbReader.poll(Serial, line) && !espReader.poll(espSerial, line)) {
        return;
    }
    if (line.length() == 0) return;

    // Split into VERB and optional numeric ARG at the ':'
    String verb = line;
    int arg = 0;
    int colon = line.indexOf(':');
    if (colon >= 0) {
        verb = line.substring(0, colon);
        arg = line.substring(colon + 1).toInt();
    }
    verb.toUpperCase();

    // STOP halts the motors immediately (aborting any in-progress move) and
    // latches the stop just long enough to make the current maneuver return.
    if (verb == "STOP") {
        emergencyStopActive = true;
        gridStop();
        Bridge.println(F("[ACK] STOP — motors halted"));
        return;
    }

    // No separate re-arm step: any command other than STOP automatically clears
    // a prior stop and runs, matching the IR remote where any button press
    // resumes. (RESUME/ARM are still accepted as an explicit clear for any
    // client that sends them, but are no longer required.)
    emergencyStopActive = false;
    if (verb == "RESUME" || verb == "ARM") {
        Bridge.println(F("[ACK] RESUME — ready"));
        return;
    }

    sensorsEnabled = true;

    if (verb == "FWD") {
        Bridge.print(F("[ACK] FWD ")); Bridge.println(arg);
        gridMoveForwardBlocks(arg > 0 ? arg : 1, SPEED_START_FAST);
    } else if (verb == "STEP") {                 // single grid coordinate
        Bridge.println(F("[ACK] STEP one coord"));
        gridMoveForwardOneCoord(SPEED_START_FAST);
    } else if (verb == "LFT") {
        Bridge.println(F("[ACK] LFT rotate 90 left"));
        gridRotateLeft90();
    } else if (verb == "RGT") {
        Bridge.println(F("[ACK] RGT rotate 90 right"));
        gridRotateRight90();
    } else if (verb == "REV") {
        Bridge.println(F("[ACK] REV slight reverse"));
        gridSlightReverse();
    } else if (verb == "GRB") {                  // arg = target color (0-3)
        Bridge.print(F("[ACK] GRB target ")); Bridge.println(gridColorName(arg));
        moveToGrab((arg >= RED && arg <= ANY) ? arg : ANY);
    } else if (verb == "DRP" || verb == "OPEN") {
        Bridge.println(F("[ACK] DRP open claw"));
        openClawWithAttach();
    } else if (verb == "PING") {                 // read sensors on demand
        int distance = gridGetDistanceCm();
        int color = gridDetectColorValue();
        Bridge.println(F("[ACK] PING"));
        printStatusTelemetry(distance, color);
    } else if (verb == "SENSORS") {              // raw line-sensor snapshot
        Bridge.print(F("[ACK] SENSORS L="));
        Bridge.print(digitalRead(LINE_LEFT_PIN));
        Bridge.print(F(" C=")); Bridge.print(digitalRead(LINE_CENTER_PIN));
        Bridge.print(F(" R=")); Bridge.println(digitalRead(LINE_RIGHT_PIN));
    } else if (verb == "PATH") {                 // arg encodes path*10 + color
        int p = arg / 10, c = arg % 10;
        Bridge.print(F("[ACK] PATH ")); Bridge.print(p);
        Bridge.print(F(" color ")); Bridge.println(gridColorName(c));
        if (p == 1) runPath(path1, sizeof(path1)/sizeof(Step), c);
        else if (p == 2) runPath(path2, sizeof(path2)/sizeof(Step), c);
        else if (p == 3) runPath(path3, sizeof(path3)/sizeof(Step), c);
    } else if (verb == "SEARCH") {               // arg encodes color*10 + startRow
        // Mirrors the 9 IR-remote mission buttons: searchGrabAndDrop(color, row)
        // with color 0=RED 1=BLUE 2=YELLOW and startRow 1/3/5 — so the dashboard
        // can debug the exact routine each remote button runs.
        int c = arg / 10, r = arg % 10;
        if (!(r == 1 || r == 3 || r == 5)) r = 1;
        if (c < RED || c > YELLOW) c = RED;
        Bridge.print(F("[ACK] SEARCH ")); Bridge.print(gridColorName(c));
        Bridge.print(F(" from row ")); Bridge.println(r);
        searchGrabAndDrop(c, r);
    } else if (verb == "RESET") {
        resetCoordinates();
        Bridge.println(F("[ACK] RESET coordinates"));
    } else if (verb == "ECO") {
        // Simulate the solar-power FSM transition: a cloud drops solar input, so
        // the harvester drops into Eco-Mode (slower, non-essential sensors off).
        // ECO with no arg toggles; ECO:0 / ECO:1 set off / on explicitly.
        if (line.indexOf(':') >= 0) ecoMode = (arg != 0);
        else ecoMode = !ecoMode;
        if (ecoMode) {
            sensorsEnabled = false;   // ultrasonic + color off; line sensors stay on
            Bridge.println(F("[SYSTEM] Eco-Mode ENGAGED — solar input low: moving slower, non-essential sensors off."));
        } else {
            Bridge.println(F("[SYSTEM] Eco-Mode CLEARED — solar restored: resuming normal speed."));
        }
    } else if (verb == "TELEMETRY" || verb == "TEL") {
        // TELEMETRY with no arg toggles, with arg 0/1 sets off/on
        if (line.indexOf(':') >= 0) {
            debugTelemetry = (arg != 0);
        } else {
            debugTelemetry = !debugTelemetry;
        }
        Bridge.print(F("[ACK] TELEMETRY "));
        Bridge.println(debugTelemetry ? F("ON") : F("OFF"));
    } else {
        Bridge.print(F("[ACK] UNKNOWN command: "));
        Bridge.println(verb);
    }
}

void setup() {
    Serial.begin(9600);
    espSerial.begin(9600);   // ESP32 wireless bridge link (pins 11 RX / 10 TX)
    delay(200);
    Bridge.println(F("[BOOT] Serial @9600 + ESP32 bridge @9600"));

    pinMode(LINE_LEFT_PIN, INPUT);
    pinMode(LINE_CENTER_PIN, INPUT);
    pinMode(LINE_RIGHT_PIN, INPUT);

    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);

    pinMode(COLOR_S0_PIN, OUTPUT);
    pinMode(COLOR_S1_PIN, OUTPUT);
    pinMode(COLOR_S2_PIN, OUTPUT);
    pinMode(COLOR_S3_PIN, OUTPUT);
    pinMode(COLOR_OUT_PIN, INPUT);
    digitalWrite(COLOR_S0_PIN, HIGH);
    digitalWrite(COLOR_S1_PIN, LOW);

    // Open the gripper as soon as power is up.
    openClawWithAttach();
    mecCar.Init();
    gridSetSpeed(SPEED_START_FAST);
    IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

    // Re-open once more now that the motor bus + IR are initialized and the
    // power rail has settled. The very first open (above) can be dropped by the
    // power-on inrush brownout, so this second pass guarantees the claw ends up
    // open every time the robot is turned on.
    openClawWithAttach();

    Bridge.println(F("=== Turn-Locked Autonomous Controller ==="));
    reportPosition();   // sync the dashboard map to the home pose at boot
    // Print an immediate telemetry snapshot at startup
    telemetryReport();
    lastTelemetryMillis = millis();
}

void loop() {
    handleSerialCommand();   // dashboard / USB debug control

    // Periodic automatic telemetry when enabled
    if (debugTelemetry && (millis() - lastTelemetryMillis >= TELEMETRY_INTERVAL_MS)) {
        telemetryReport();
        lastTelemetryMillis = millis();
    }

    // Pose heartbeat — unconditional (not gated on debugTelemetry): the map is a
    // core dashboard readout, not debug output, and this is the only thing that
    // resyncs it after a dropped line. See POS_HEARTBEAT_MS.
    if (millis() - lastPosHeartbeatMillis >= POS_HEARTBEAT_MS) {
        reportPosition();
        lastPosHeartbeatMillis = millis();
    }

    if (IrReceiver.decode()) {
        int key = IrReceiver.decodedIRData.command;
        
        if (key == CMD_STAR) {
            emergencyStopActive = true;
            gridStop();
            Bridge.println(F("\n!!! EMERGENCY STOP TRIGGERED VIA LOOP !!!"));
            IrReceiver.resume();
            return;
        }

        if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
            Bridge.print(F("Key Pressed: "));
            Bridge.println(key);
            emergencyStopActive = false; 

            switch (key) {
                // Each button starts the colour search at its associated block
                // (path1->row1, path2->row3, path3->row5), then hops to the
                // other blocks nearest-first until the target colour is found.
                case 22: searchGrabAndDrop(RED, 1); break; // button 1
                case 25: searchGrabAndDrop(RED, 3); break; // button 2
                case 13: searchGrabAndDrop(RED, 5); break; // button 3

                case 12: searchGrabAndDrop(BLUE, 1); break; // button 1
                case 24: searchGrabAndDrop(BLUE, 3); break; // button 2
                case 94: searchGrabAndDrop(BLUE, 5); break; // button 3

                case 8:  searchGrabAndDrop(YELLOW, 1); break; // button 1
                case 28: searchGrabAndDrop(YELLOW, 3); break; // button 2
                case 90: searchGrabAndDrop(YELLOW, 5); break; // button 3

                // case 12: executeAutoMission(1, BLUE); break;              // button 4
                // case 24: executeAutoMission(3, BLUE); break;              // button 5
                // case 94: executeAutoMission(5, BLUE); break;              // button 6
                // case 8: executeAutoMission(1, YELLOW); break;               // button 4
                // case 28: executeAutoMission(3, YELLOW); break;              // button 5
                // case 90: executeAutoMission(5, YELLOW); break;              // button 6
                case 74: openClawWithAttach(); break; // button #
                case 70: // button UP — creep forward at 40, slowing further on approach, grip on contact
                    sensorsEnabled = true;
                    moveToGrab(ANY, 40);
                    break;
                case 67: clawNudge(false); break; // button RIGHT — nudge claw toward OPEN
                case 68: clawNudge(true);  break; // button LEFT  — nudge claw toward CLOSED
                case 64: // button OK — full reset: open grip, clear nav state, re-enable sensors
                    openClawWithAttach();
                    resetCoordinates();
                    sensorsEnabled = true;
                    Bridge.println(F("[SYSTEM] OK pressed — reset complete."));
                    break;
                default: break;
            }
        }
        IrReceiver.resume();
    }
}