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

// Optional: many TCS3200 modules have 4 white illumination LEDs wired straight
// to VCC, so they burn ~20-60mA continuously whenever the board is powered and
// nothing in software can turn them off. If your module instead breaks the LED
// enable out to a pin (often labelled "LED"), wire it to a spare Uno GPIO and
// define it here — then SLEEP/IDLE will drive it LOW to cut the LEDs, and a
// colour read drives it back HIGH. Leave this commented out if the LEDs are
// hardwired to power (only a physical switch/MOSFET can cut them then).
// #define COLOR_LED_PIN 2   // example: wire the module's LED pin to Uno D2

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
unsigned long lastTelemetryMillis = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 1000; // ms between automatic reports

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
// one node in from every edge. Tracked node is clamped to X 1..9, Y 1..5:
// X=1 = block column; the car only drives as far as X=2 (one node east) to read
// colour and grab, letting the ultrasonic creep the last bit so it never clashes
// with the block on X=1. X=9 is the START zone's east edge, where the block is
// dropped; Y 1..5 is one in from the
// collecting top/bottom. moveCoord() clamps every target so the car can never
// be commanded onto or past the outer boundary line.
const int GRID_MIN_X = 1, GRID_MAX_X = 9;
const int GRID_MIN_Y = 1, GRID_MAX_Y = 5;
// Drop point: the car releases the block at (9,3), the east edge of the
// coordinate-tracked collecting grid (GRID_MAX_X=9), two nodes EAST of home
// (7,3) on row 3. goToDrop() line-follows east from home and counts junctions,
// tracking currentX, until it reaches column 9 — where it stops and releases.
const int DROP_TARGET_X = 9;

// Emit the tracked pose over both serial links so the dashboard's live map can
// draw the robot. Format: "[POS] x,y,h" with h = 0..3 (NORTH/EAST/SOUTH/WEST).
// Called every time currentX/currentY/currentHeading changes.
void reportPosition() {
    Bridge.print(F("[POS] "));
    Bridge.print(currentX); Bridge.print(F(","));
    Bridge.print(currentY); Bridge.print(F(","));
    Bridge.println(currentHeading);
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
const uint8_t SPEED_ROTATE = 60;
// Blind-spin duration before we start hunting for the new perpendicular line.
// Must be long enough to rotate PAST the too-early black detection (car stops
// short if this is too small). Tune per surface/battery: raise if it still
// under-rotates, lower if it overshoots.
//
// Both turns stop on their LEADING-edge sensor (RIGHT for a CW spin, LEFT for a
// CCW spin) so they trigger at ~90 symmetrically. Stopping the LEFT turn on the
// on-axis CENTER sensor instead let the body swing past 90 before the line
// reached it, so LEFT over-rotated. The blind window only has to carry the
// leading sensor OFF the starting junction black onto white before hunting
// begins; raise it if the turn stops short on the junction, lower if it
// overshoots.
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

// ── Time-based coordinate projection (dead-reckoning recovery) ──────────────
// When the line sensors miss the line (a faded/broken grid line, a gap the car
// drifts across), instead of immediately e-stopping we ESTIMATE where the car is
// from how long it has been driving since the last known junction, then actively
// hunt the line back so the mission can continue.
//
// MS_PER_COORD is how long (ms) the car takes to travel ONE grid coordinate at
// the normal follow speed (SPEED_START_FAST). This is a PLACEHOLDER — CALIBRATE
// IT ON THE REAL CAR: send "FWD:1" (or STEP) on a clean line and time with a
// stopwatch how long the car takes to cross one coordinate, then set this to that
// value in ms. It is used only to PROJECT an estimated coordinate for reporting
// and to decide when the car has likely crept a full node past the last junction
// while off the line; the actual junction count still comes from the sensors
// whenever they DO see the line, so a slightly-off value only affects the
// reported estimate, never the real tracked count once the line is re-acquired.
const unsigned long MS_PER_COORD = 1200;
// While the line is lost, run the recovery hunt for at most this long before
// giving up and latching the safety e-stop. Long enough to sweep both ways and
// reverse a little; short enough that a truly off-grid car still halts promptly.
const unsigned long LINE_RECOVER_TIMEOUT_MS = 2500;
// Speed used for the recovery creep/sweep — a gentle crawl so the car doesn't
// fling itself further off the grid while hunting for the line.
const uint8_t LINE_RECOVER_SPEED = 40;
// One recovery sweep leg duration: the car arcs one way for this long looking for
// any line sensor to go HIGH, then arcs back the other way. Kept short so it
// probes a small arc at a time instead of spinning away from the line.
const unsigned long LINE_RECOVER_SWEEP_MS = 350;
// Start speed for the scripted-path block approach. Kept below SPEED_MIN so the
// approach creeps in noticeably slower and still tapers toward APPROACH_SPEED_FLOOR.
// Lowered from 42: at the higher speed the car noses onto the block before its
// line-follow correction has straightened it, so it arrives slightly skewed and
// the claw can't grip. A slower approach gives the alignment time to settle.
// Must stay strictly below SPEED_MIN (38) so executeApproachMovement's taper
// engages (it only ramps toward APPROACH_SPEED_FLOOR when startSpeed < SPEED_MIN;
// at exactly SPEED_MIN the floor becomes SPEED_MIN and the approach runs flat).
// Lowered further from 37 to 35, then to 29 (one notch above APPROACH_SPEED_FLOOR=28)
// so the WHOLE approach — not just the final taper — is a slow crawl, giving the
// line-follow correction the maximum distance to straighten the car onto the line
// before it reaches the block and grabs. MUST stay ABOVE APPROACH_SPEED_FLOOR: the
// taper ramps DOWN from this value to the floor as the block nears, so if this
// drops below the floor the ramp inverts and the car SPEEDS UP on final approach.
// Tried as low as 22/20 on the car — NOT ENOUGH POWER: the motors couldn't
// overcome breakaway torque, so the car buzzed/stalled instead of creeping. Raised
// back to 35 (one notch above APPROACH_SPEED_FLOOR=34), the documented reliable
// crawl. To still get a SLOW, well-aligned final approach without dropping PWM
// below stall, the taper is started EARLIER instead (ITEM_DETECT_DISTANCE_CM=40) so
// the car decelerates over more distance. Do NOT drop this toward the low 20s again
// — that's below the motors' breakaway torque and the car won't move.
const uint8_t PATH_APPROACH_SPEED = 35;
// Distance at which the approach STARTS tapering toward APPROACH_SPEED_FLOOR. Raised
// from 25 to 40 so the slow-down begins farther from the block — a longer, earlier
// glide to straighten onto the line and settle before gripping, WITHOUT needing a
// below-stall PWM. The ultrasonic reads well past 40cm; beyond it the car just holds
// PATH_APPROACH_SPEED until the block comes into range, then the taper takes over.
const int ITEM_DETECT_DISTANCE_CM = 40;
// Stop this far from the block to read colour and grab — leave a GAP rather than
// nosing right up to it. The gripper jaws swing FORWARD as they close, so they
// reach the block from this standoff; stopping closer makes the chassis collide
// with the block (and the colour read is taken here too, so it must not ram it).
// Empirical — tune on the real car: too large and the closing claw misses the
// block, too small and the chassis bumps it. Was 6cm (nose-to-block), then 9cm,
// then 8cm (jaws under-reached), then 7cm — and at 7cm the chassis COLLIDES, so
// it is now back at 8cm. Colour reads reliably at ~7cm and 6cm definitely
// collides, so the usable window is narrow and BOTH failure modes have now been
// observed inside it: <=7 bumps the block, >=8 has previously left the closing
// jaws short of it.
//
// This value has therefore already round-tripped 8 -> 7 -> 8. If the jaws miss
// again at 8cm, do NOT lower it back to 7 — that just reintroduces the collision
// and restarts the loop. The remaining slack is in the gripper, not the standoff:
// adjust CLAW_CLOSED_ANGLE or the jaw linkage so the jaws close fully at 8cm.
const int GRAB_APPROACH_DISTANCE_CM = 8;
// Colour is only TRUSTED for the grab/search DECISION when the block is this close
// (the ~7cm calibrated read distance, plus margin). The raw sensor still reads at
// any distance for the live bench, but grabColorIfMatch ignores far reads.
//
// 10cm is the measured EDGE of usable signal, not a comfortable margin. A 4/7/10cm
// CAL sweep of all three blocks still classified every one correctly at 10cm, so
// this is not a false-positive zone (an earlier note claimed red collapses to BLUE
// past ~9cm — that did not reproduce; red's blue channel stays 12%+ off). But at
// 10cm some channels read BRIGHTER than at 7cm, which is physically impossible for
// reflected light: the block has stopped dominating and the sensor is largely
// reading ambient. Classification survives there on a thin, luck-dependent margin.
// Prefer to read nearer the 8cm standoff; if ambient light changes, 10cm is the
// first thing that breaks, and lowering this to 8 is the correct response.
const int COLOR_DECISION_DISTANCE_CM = 10;
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
// Lowered from 38 to 34, then to 20 so the very last stretch onto the block is a
// genuine crawl — this is where the car needs to be dead-straight for the claw to
// grip, and a slower final creep gives it the most time to square onto the line
// and stop smoothly. Confirmed on the car: values in the low 20s DON'T MOVE — the
// motors can't overcome breakaway torque and just buzz/stall. 34 is the lowest that
// still creeps reliably, so the floor is held there. For a slower FEEL, start the
// taper earlier (ITEM_DETECT_DISTANCE_CM) rather than dropping this below 34.
const uint8_t APPROACH_SPEED_FLOOR = 34;
// Max PWM change per approach tick (~60ms). The approach speed is derived from
// the raw ultrasonic distance, which jitters a few cm between pings, so the
// target speed bounces up and down every tick. Sending those raw jumps straight
// to the motors makes the car stutter as it nears the block. Instead we ease the
// commanded speed toward the target by at most this much per tick, so both the
// distance-based ramp-down and any sensor jitter come out as a smooth glide.
// 2/tick over the ~34..48 approach range settles in well under a second. Raise
// for a snappier (but jerkier) response; lower for an even smoother crawl.
// Lowered from 2 to 1: with the tighter tick below the loop runs often enough
// that 1 PWM/tick still tracks the ramp, and the smaller steps mean the motors
// never feel a speed jump — the crawl comes out glassy instead of pulsing.
const uint8_t APPROACH_SPEED_SLEW = 1;
// Control-loop period for the block approach. Was a bare 60ms, but each tick also
// blocked on a full colour read (9 pulseIn calls, up to ~0.3s of *variable* stall)
// plus two ultrasonic pings, so the real period lurched and the motors stuttered.
// With the colour read gated to debug-only and a single ping per tick (see
// executeApproachMovement), the loop is now light enough to run every 40ms — more
// frequent line-follow corrections and speed slews mean a smoother, straighter
// crawl onto the block. Lower for even tighter control; the ultrasonic ping itself
// takes up to ~30ms, so going much below this leaves little settle margin.
const unsigned long APPROACH_TICK_MS = 40;
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
// goToDrop() drives out to the X=9 drop line at the normal smooth crawl and then
// fires this reverse-brake pulse the instant X=9 is counted, so the car settles
// dead on (9,3) instead of coasting past it. Long enough to clear the H-bridge
// direction dead-time and actually bite, not just idle. Raise it if the car
// still rolls past X=9; lower it if it jerks backward off the line.
const unsigned long DROP_BRAKE_MS = 130;
// After counting the X=9 drop line, creep a little further FORWARD into the drop
// area before stopping, so the block is set down inside the zone rather than right
// on its edge. Runs at the gentle PATH_APPROACH_SPEED crawl. Currently 0: the car
// must stop ON the (9,3) junction, and DROP_BRAKE_MS is what holds it there. Raise
// this only if the claw geometry needs the block set down forward of the sensors.
const unsigned long DROP_FORWARD_NUDGE_MS = 0;
// Final alignment settle: once the drop line is reached, goToDrop() runs a brief
// in-place line-centering phase BEFORE the forward nudge. It only STEERS to square
// the chassis onto the line (corrections fire, but it never advances straight),
// so a car that arrived slightly skewed straightens out and the block is released
// dead-straight instead of at an angle. Runs for this long or until the car reads
// centered (L/R both off the line) for a couple of consecutive ticks, whichever
// comes first. Raise it if the car still releases crooked; lower it — or set to 0
// to skip the settle — if it wastes time wiggling once already straight.
const unsigned long DROP_ALIGN_SETTLE_MS = 600;
// Reverse distance after releasing the block at the drop zone, BEFORE the 180°
// spin back toward home. The plain gridSlightReverse (REVERSE_BUMP_MS=100) left
// the car too close and the spinning chassis clipped ("slashed") the block it had
// just set down. This backs off far enough that the 180° turn swings clear. Note
// it was tuned when DROP_FORWARD_NUDGE_MS still pushed the car deeper into the
// zone; with the nudge now 0 the car stops shorter, so this may be reducible.
// Raise if the spin still catches the block; lower if it reverses too far to
// re-acquire the home junction. moveCoord() on the return counts the junction
// crossing, so a longer reverse here does NOT throw off where it lands.
const unsigned long DROP_CLEAR_REVERSE_MS = 450;
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
// Forces sensors ON for the read so the live stream / test bench always shows
// distance + colour, even when idle (navigation normally leaves sensors OFF).
void telemetryReport() {
    if (emergencyStopActive) return;
    bool prevDebug = debugTelemetry;
    bool prevSensors = sensorsEnabled;
    debugTelemetry = false;   // suppress internal DBG prints
    sensorsEnabled = true;    // always read on the bench, regardless of nav state
    int distance = gridGetDistanceCm();
    int color = gridDetectColorValue();
    debugTelemetry = prevDebug;
    sensorsEnabled = prevSensors;

    Bridge.print(F("[AUTO] "));
    printStatusTelemetry(distance, color);
}

void motionTelemetryTick(unsigned long &lastTelemetryMillis) {
    if (!debugTelemetry || emergencyStopActive) return;
    if (millis() - lastTelemetryMillis < 250) return;
    telemetryReport();
    lastTelemetryMillis = millis();
}


// Last speed value emitted over telemetry, so gridSetSpeed only prints a
// "[SPEED] n" line when the commanded speed actually CHANGES. gridSetSpeed is
// called every motion tick (~every 60ms during an approach); printing every
// call would flood the serial link and console. Emit-on-change keeps the
// dashboard's speed readout live without the spam. 255 = "nothing sent yet".
uint8_t lastSpeedEmitted = 255;

// Emit the current commanded PWM speed to the dashboard, but only when it has
// changed since the last emit. Shared by gridSetSpeed and gridStop (which
// reports 0). Skipped entirely during an e-stop.
void emitSpeedIfChanged(uint8_t speed) {
    if (emergencyStopActive) return;
    if (speed == lastSpeedEmitted) return;
    lastSpeedEmitted = speed;
    Bridge.print(F("[SPEED] "));
    Bridge.println(speed);
}

void gridSetSpeed(uint8_t targetSpeed) {
    speed_Upper_L = targetSpeed; speed_Lower_L = targetSpeed;
    speed_Upper_R = targetSpeed; speed_Lower_R = targetSpeed;
    emitSpeedIfChanged(targetSpeed);
}

// Gentle line-follow correction: both sides keep driving FORWARD, but the
// outer side runs faster than the inner side, arcing the car back onto the
// line. This replaces using Turn_Left()/Turn_Right() as the correction, which
// pivot the wheels in OPPOSITE directions (one side reverses) — a hard yank
// that reads as jerky when it fires every sensor tick during a line-follow.
// steerLeft=true arcs left (drifting back rightward), false arcs right.
void gridSteer(uint8_t baseSpeed, uint8_t boost, bool steerLeft) {
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
    emitSpeedIfChanged(0);   // report the car is stationary to the dashboard
    Bridge.println(F("[SYSTEM] Robot Halted."));
}

void spinRightInPlace(uint8_t speed) { gridSetSpeed(speed); mecCar.Turn_Right(); }
void spinLeftInPlace(uint8_t speed) { gridSetSpeed(speed); mecCar.Turn_Left(); }

void disableSensors() {
    sensorsEnabled = false;
    Bridge.println(F("[SYSTEM] Ultrasonic and Color sensors have been SOFTWARE DISABLED."));
}

// Low-power idle: cut every draw the firmware CAN control while the car sits
// doing nothing. Motors coast (Stop = 0 PWM), the ultrasonic stops pinging
// (sensorsEnabled=false), the 1 s telemetry ping+colour read stops
// (debugTelemetry=false), the claw servo is detached so it isn't held under
// current, and the TCS3200 colour sensor is put into power-down (S0=LOW/S1=LOW)
// — which, if the module's LED enable is wired to COLOR_LED_PIN, also kills its
// illumination LEDs. Everything wakes automatically on the next command: the
// next colour read calls colorSensorWake() to restore the sensor. This does NOT
// touch the ESP32 WiFi radio (the single biggest draw) — that lives on the other
// board and can only be quieted from the ESP32 firmware or a power switch.
void enterIdleSleep() {
    gridStop();                 // coast the motors (already 0 PWM, but be explicit)
    sensorsEnabled = false;     // stop the ultrasonic ever being pinged while idle
    debugTelemetry = false;     // stop the periodic ping + full colour read
    clawServo.detach();         // ensure the servo isn't holding position under load
    // Power down the colour sensor: S0=LOW,S1=LOW = TCS3200 power-down mode.
    digitalWrite(COLOR_S0_PIN, LOW);
    digitalWrite(COLOR_S1_PIN, LOW);
#ifdef COLOR_LED_PIN
    digitalWrite(COLOR_LED_PIN, LOW);   // cut the module LEDs if wired to a GPIO
#endif
    Bridge.println(F("[SYSTEM] IDLE — motors coasted, colour sensor powered down, "
                     "telemetry off. Send any command (or press a remote button) to wake."));
}

// Peek at either serial link for a dashboard STOP without consuming a full
// command line. Called from inside long-running motion loops so a STOP sent
// while the car is mid-move (dashboard or wired) aborts immediately, just like
// the IR remote — instead of waiting for the whole move to finish. We look at
// the first byte only ('S'): the ESP32 forwards "STOP\n" verbatim, and no other
// command the dashboard sends starts with 'S' except SENSORS, which is refused
// while latched anyway. On a match we drain the rest of the line so it doesn't
// linger in the buffer.
bool serialStopRequested(Stream &link) {
    if (!link.available()) return false;
    if (link.peek() != 'S' && link.peek() != 's') return false;
    String line = link.readStringUntil('\n');
    line.trim();
    line.toUpperCase();
    return line == "STOP";
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
    if (serialStopRequested(Serial) || serialStopRequested(espSerial)) {
        emergencyStopActive = true;
        gridStop();
        Bridge.println(F("\n!!! EMERGENCY STOP TRIGGERED (dashboard) !!!"));
        return true;
    }
    return emergencyStopActive;
}

// Safety: detect the car running off the grid. Past the outermost grid line
// there is only blank mat, so all three line sensors read LOW. If that persists
// for LINE_LOST_TIMEOUT_MS (a real off-track condition, not a momentary wobble)
// this latches an emergency stop, exactly like a STOP press. `lostSince` is the
// caller's per-move timer: pass a local initialised to 0; it's stamped when the
// line is first lost and cleared whenever any sensor sees a line again.
// Call once per iteration of a forward-motion loop and return early if it's true.
//
// SUPERSEDED in the forward-motion loops by recoverOrStop(), which first tries to
// dead-reckon the coordinate from travel time and hunt the line back before
// falling back to this same e-stop. Kept as the plain "stop immediately" helper
// for any caller that wants the old behaviour without the recovery attempt.
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

// True if ANY line sensor currently sees the line — used as the recovery
// re-acquire test (re-centering afterwards is the normal follower's job).
static bool anyLineSensorHigh() {
    return digitalRead(LINE_LEFT_PIN) == HIGH ||
           digitalRead(LINE_CENTER_PIN) == HIGH ||
           digitalRead(LINE_RIGHT_PIN) == HIGH;
}

// Time-based coordinate projection while the line is lost. Given how long the
// car has been travelling forward since the last confirmed junction
// (millis() - segmentStartMs), estimate how many WHOLE coordinates it has
// covered (elapsed / MS_PER_COORD) and advance the tracked (currentX,currentY)
// along the current heading by that many nodes — but never past a coordinate the
// sensors have already counted, and clamped to the grid. This keeps the dashboard
// map roughly right even across a stretch of missing line, WITHOUT the sensors:
// pure dead-reckoning. `alreadyCounted` is how many junctions this move's own
// loop has already tallied from the sensors, so we only project BEYOND those.
// Returns the number of extra nodes projected (0 if less than one coord elapsed).
static int projectCoordFromTime(unsigned long segmentStartMs) {
    unsigned long elapsed = millis() - segmentStartMs;
    int projected = (int)(elapsed / MS_PER_COORD);
    if (projected <= 0) return 0;
    for (int i = 0; i < projected; i++) {
        int nx = currentX, ny = currentY;
        switch (currentHeading) {
            case EAST:  nx += 1; break;
            case WEST:  nx -= 1; break;
            case NORTH: ny += 1; break;
            case SOUTH: ny -= 1; break;
        }
        // Stop projecting at the grid boundary — the car can't legitimately be
        // past the outer line, so don't march the estimate off the map.
        if (nx < GRID_MIN_X || nx > GRID_MAX_X || ny < GRID_MIN_Y || ny > GRID_MAX_Y) {
            projected = i;
            break;
        }
        currentX = nx; currentY = ny;
    }
    if (projected > 0) {
        Bridge.print(F("[RECOVER] Line lost — projected "));
        Bridge.print(projected);
        Bridge.print(F(" coord(s) from time onto estimated ("));
        Bridge.print(currentX); Bridge.print(F(",")); Bridge.print(currentY);
        Bridge.println(F(")."));
        reportPosition();
    }
    return projected;
}

// Line-loss handler with dead-reckoning recovery. Drop-in replacement for
// checkOffGridStop in a forward-motion loop: call once per iteration.
//   - While at least one sensor sees the line, does nothing (returns false) and
//     resets the lost-timer.
//   - Once the line has been gone for LINE_LOST_TIMEOUT_MS, instead of e-stopping
//     it (a) projects the estimated coordinate from travel time and (b) actively
//     hunts the line back: creep forward, then sweep left, then sweep right, then
//     reverse a little — repeating until a sensor re-acquires the line or
//     LINE_RECOVER_TIMEOUT_MS elapses. On re-acquire it returns false so the
//     caller's normal follower resumes. Only if recovery fails does it latch the
//     safety e-stop and return true.
// `lostSince` is the caller's per-move lost-timer (local, init 0). `segmentStartMs`
// is when the car left its last confirmed junction (the caller stamps it each time
// it counts a junction), used for the time->coordinate projection. `projectCoord`
// controls whether the projection advances the global (currentX,currentY): pass
// true ONLY from loops that OWN the coordinate for this step (gridMoveForwardOneCoord,
// goToDrop). Loops whose CALLER bumps the coordinate after the whole move (the
// block-counting gridMoveForwardBlocks) pass false, so recovery still hunts the
// line but doesn't double-count against the caller's own coordinate update.
bool recoverOrStop(unsigned long &lostSince, unsigned long segmentStartMs, bool projectCoord) {
    if (anyLineSensorHigh()) { lostSince = 0; return false; }
    if (lostSince == 0) { lostSince = millis(); return false; }
    if (millis() - lostSince < LINE_LOST_TIMEOUT_MS) return false;

    // Line confirmed lost. Project the estimated coordinate from elapsed time so
    // the tracker/dashboard know roughly where the car is, then try to recover.
    if (projectCoord) projectCoordFromTime(segmentStartMs);

    Bridge.println(F("[RECOVER] Hunting for the line to resume..."));
    unsigned long recoverStart = millis();
    // Sweep pattern cycles: 0 = creep straight, 1 = arc left, 2 = arc right,
    // 3 = reverse. Each leg runs up to LINE_RECOVER_SWEEP_MS but bails the instant
    // a sensor sees the line, so re-acquire is immediate.
    int leg = 0;
    while (millis() - recoverStart < LINE_RECOVER_TIMEOUT_MS) {
        if (checkEmergencyStop()) return true;   // honour a real STOP mid-recovery
        switch (leg % 4) {
            case 0: gridSetSpeed(LINE_RECOVER_SPEED); mecCar.Advance(); break;
            case 1: gridSteer(LINE_RECOVER_SPEED, TURN_CORRECTION_BOOST, true);  break;
            case 2: gridSteer(LINE_RECOVER_SPEED, TURN_CORRECTION_BOOST, false); break;
            case 3: gridSetSpeed(LINE_RECOVER_SPEED); mecCar.Back(); break;
        }
        unsigned long legStart = millis();
        while (millis() - legStart < LINE_RECOVER_SWEEP_MS) {
            if (checkEmergencyStop()) return true;
            if (anyLineSensorHigh()) {
                gridStop();
                Bridge.println(F("[RECOVER] Line re-acquired — resuming follow."));
                lostSince = 0;
                return false;   // caller's follower takes over from here
            }
        }
        leg++;
    }

    // Recovery exhausted — the car really is off the grid. Latch the safety stop.
    emergencyStopActive = true;
    gridStop();
    Bridge.println(F("\n!!! SAFETY STOP: line lost and recovery failed — car off the grid !!!"));
    return true;
}

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
    // Stop on the LEFT sensor — the leading edge during a CCW spin — mirroring
    // the RIGHT turn, which stops on its leading RIGHT sensor. Stopping on the
    // on-axis CENTER sensor let the body swing PAST 90 before the line reached
    // it, so the left turn over-rotated; the leading-edge sensor triggers at
    // ~90 like the right turn. The blind window below must carry the LEFT sensor
    // OFF the starting junction black first so it hunts the NEW line, not the old.
    while (true) {
        if (checkEmergencyStop()) return false;
        if (digitalRead(LINE_LEFT_PIN) == HIGH) break;
    }
    spinRightInPlace(SPEED_ROTATE + 10);
    
    unsigned long brakeStart = millis();
    while (millis() - brakeStart < BRAKE_MS) { if (checkEmergencyStop()) return false; }
    mecCar.Stop();
    
    unsigned long settleStart = millis();

    while (millis() - settleStart < TURN_SETTLE_MS) { if (checkEmergencyStop()) return false; }
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
    return true;
}

void gridMoveForwardBlocks(int targetBlocks, uint8_t startSpeed) {
    if (checkEmergencyStop()) return;
    int junctionCount = 0;
    bool onJunction = true;
    unsigned long telemetryTickMillis = millis();
    unsigned long lineLostSince = 0;
    // Stamp when the car last left a confirmed junction, for the time-based
    // coordinate projection in recoverOrStop.
    unsigned long segmentStartMillis = millis();

    while (junctionCount < targetBlocks) {
        if (checkEmergencyStop()) return;
        // Line-loss recovery: hunt the line back instead of e-stopping. This
        // function doesn't own currentX/currentY (moveCoord/runPath bump them
        // after the whole call), so don't project onto the globals here — pass
        // false; recovery still reverses/sweeps the car back onto the line.
        if (recoverOrStop(lineLostSince, segmentStartMillis, false)) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        int calculatedSpeed = startSpeed - (junctionCount * 5);
        if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
        // On the final block (heading toward the last target junction), crawl
        // slower so the car approaches the last grid gently and stops right on
        // it instead of coasting past.
        if (junctionCount == targetBlocks - 1) calculatedSpeed = FINAL_BLOCK_CRAWL_SPEED;
        uint8_t currentSpeed = (uint8_t)calculatedSpeed;

        if (Left == HIGH && Center == HIGH && Right == HIGH) {
            if (!onJunction) {
                junctionCount++;
                onJunction = true;
                segmentStartMillis = millis();   // left a confirmed junction — restart the segment timer
                if (junctionCount == targetBlocks) break;
            }
            gridSetSpeed(currentSpeed);
            mecCar.Advance();
        } else if (Left == LOW && Center == HIGH && Right == LOW) {
            onJunction = false; gridSetSpeed(currentSpeed); mecCar.Advance();
        } else if (Left == LOW && Center == LOW && Right == HIGH) {
            onJunction = false; gridSteer(currentSpeed, TURN_CORRECTION_BOOST, false);
        } else if (Left == HIGH && Center == LOW && Right == LOW) {
            onJunction = false; gridSteer(currentSpeed, TURN_CORRECTION_BOOST, true);
        } else if (Left == HIGH && Center == HIGH && Right == LOW) {
            onJunction = false; gridSteer(currentSpeed, TURN_CORRECTION_BOOST, true);
        } else if (Left == LOW && Center == HIGH && Right == HIGH) {
            onJunction = false; gridSteer(currentSpeed, TURN_CORRECTION_BOOST, false);
        } else if (Left == LOW && Center == LOW && Right == LOW) {
            onJunction = false; gridSetSpeed(currentSpeed); mecCar.Advance();
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
}

void gridMoveForwardOneCoord(uint8_t startSpeed) {
    if (checkEmergencyStop()) return;
    unsigned long telemetryTickMillis = millis();
    unsigned long lineLostSince = 0;
    // Segment timer for recovery: this whole move IS one coordinate, so it starts
    // when the move starts. moveCoord() bumps the global coordinate after this call
    // returns, so recovery here must NOT project onto the globals (would double-count).
    unsigned long segmentStartMillis = millis();
    unsigned long leaveTime = millis();
    while (millis() - leaveTime < 140) {
        if (checkEmergencyStop()) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);
        if (Left == LOW && Center == HIGH && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Advance(); }
        else if (Right == HIGH) gridSteer(startSpeed, TURN_CORRECTION_BOOST, false);
        else if (Left == HIGH) gridSteer(startSpeed, TURN_CORRECTION_BOOST, true);
        else { gridSetSpeed(startSpeed); mecCar.Advance(); }

        motionTelemetryTick(telemetryTickMillis);
    }

    while (true) {
        if (checkEmergencyStop()) return;
        if (recoverOrStop(lineLostSince, segmentStartMillis, false)) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        if (Left == HIGH && Center == HIGH && Right == HIGH) break;
        else if (Left == LOW && Center == HIGH && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Advance(); }

        else if (Left == LOW && Center == LOW && Right == HIGH) gridSteer(startSpeed, TURN_CORRECTION_BOOST, false);
        else if (Left == HIGH && Center == LOW && Right == LOW) gridSteer(startSpeed, TURN_CORRECTION_BOOST, true);
        else if (Left == LOW && Center == LOW && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Advance(); }
        else if (Left == HIGH && Center == HIGH && Right == LOW) gridSteer(startSpeed, TURN_CORRECTION_BOOST, true);
        else if (Left == LOW && Center == HIGH && Right == HIGH) gridSteer(startSpeed, TURN_CORRECTION_BOOST, false);

        motionTelemetryTick(telemetryTickMillis);
    }

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

// Restore the TCS3200 to its normal running configuration: S0=HIGH/S1=LOW =
// 100% frequency scaling (the active mode setup() leaves it in). SLEEP puts the
// sensor into power-down (S0=LOW/S1=LOW), so every colour read calls this first
// to self-heal — the sensor wakes on the next read with no explicit re-arm. Also
// re-lights the module LEDs if their enable is wired to COLOR_LED_PIN.
void colorSensorWake() {
    digitalWrite(COLOR_S0_PIN, HIGH);
    digitalWrite(COLOR_S1_PIN, LOW);
#ifdef COLOR_LED_PIN
    digitalWrite(COLOR_LED_PIN, HIGH);
#endif
}

int gridDetectColorValue() {
    if (emergencyStopActive) return ANY;
    colorSensorWake();   // self-heal from a prior SLEEP power-down before reading
    unsigned long red = gridReadColorChannel(LOW, LOW);
    unsigned long green = gridReadColorChannel(HIGH, HIGH);
    unsigned long blue = gridReadColorChannel(LOW, HIGH);

    // Classify by which channel is MOST reflected (smallest pulse width), with a
    // green-vs-blue tiebreak that separates RED from YELLOW. Both RED and YELLOW
    // reflect red strongest (red channel smallest for both), so "smallest wins"
    // alone can't tell them apart — the LEAST-reflected channel is what does.
    //
    // Calibrated on the car with each block swept 4/7/10cm (see CAL command).
    // Raw pulse widths at the ~7-8cm grab standoff:
    //   RED    R=2023 G=3267 B=2766  -> red most reflected, GREEN least
    //   YELLOW R=1774 G=2161 B=2392  -> red most reflected, BLUE  least
    //   BLUE   R=2852 G=2842 B=1953  -> blue most reflected
    // Signal decays with distance and the channels bunch, so both red-family G/B
    // ratios converge upward as the block gets farther away. Worst case is 10cm,
    // where the two bands are closest — see the threshold note below.
    int result = ANY;
    if (blue < red && blue < green) {
        result = BLUE;                          // blue most reflected
    } else if (red < blue || red < green) {
        // Red strongly reflected -> RED or YELLOW. A raw blue<green compare has
        // too thin a margin and FLIPS on yellow past ~6cm (yellow's green/blue
        // cross over), so use the green/blue RATIO instead, which stays separated.
        // Measured bands (4cm / 7-8cm / 10cm):
        //   RED    1.26  / 1.18  / 1.154  <- floor is 1.154, at 10cm
        //   YELLOW 0.593 / 0.903 / 1.022  <- ceiling is 1.022, at 10cm
        // Both climb toward each other as distance kills the signal, so the bands
        // are tightest at 10cm with a 0.132 gap. 1.09 is that gap's midpoint,
        // giving symmetric +-0.066 worst-case margin (and 0.09/0.19 at the 7-8cm
        // standoff where the mission actually reads).
        result = (green > blue * 1.09f) ? RED : YELLOW;
    }
    // else: green somehow most reflected -> not a target color, leave as ANY.

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

// Calibration helper: hold ONE colored block at the target read distance, then
// send "CAL" (dashboard or serial). It averages many raw R/G/B pulse-width
// samples for a stable reading and prints them alongside the current ultrasonic
// distance (so you can confirm the standoff) and the normalized R/G/B ratios.
//
// Capture RED, YELLOW and BLUE this way, note the numbers, and feed them back so
// the thresholds in gridDetectColorValue() can be set to those measured values.
// Raw pulse width is INVERSELY proportional to how much of that color is present:
// a SMALLER number for a channel means MORE of that color. Normalized ratios make
// the read distance-tolerant — absolute pulse widths shift with distance/lighting,
// but the ratio between channels stays roughly constant for a given color.
void gridCalibrateColorReport() {
    colorSensorWake();
    const int CAL_SAMPLES = 15;
    unsigned long redSum = 0, greenSum = 0, blueSum = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        redSum   += gridReadColorChannel(LOW, LOW);
        greenSum += gridReadColorChannel(HIGH, HIGH);
        blueSum  += gridReadColorChannel(LOW, HIGH);
    }
    unsigned long red = redSum / CAL_SAMPLES;
    unsigned long green = greenSum / CAL_SAMPLES;
    unsigned long blue = blueSum / CAL_SAMPLES;

    // Convert pulse widths to "strength" (inverse), then normalize to percentages
    // so the three add up to ~100. Larger % = more of that color reflected.
    float rInv = red   > 0 ? 1000.0f / red   : 0;
    float gInv = green > 0 ? 1000.0f / green : 0;
    float bInv = blue  > 0 ? 1000.0f / blue  : 0;
    float invTotal = rInv + gInv + bInv;
    int rPct = invTotal > 0 ? (int)(rInv / invTotal * 100.0f + 0.5f) : 0;
    int gPct = invTotal > 0 ? (int)(gInv / invTotal * 100.0f + 0.5f) : 0;
    int bPct = invTotal > 0 ? (int)(bInv / invTotal * 100.0f + 0.5f) : 0;

    int distance = gridGetDistanceCm();

    Bridge.println(F("[CAL] ---- color calibration sample ----"));
    Bridge.print(F("[CAL] distance: ")); Bridge.print(distance);
    Bridge.println(F(" cm  (confirm this matches your target read distance)"));
    Bridge.print(F("[CAL] raw pulse width  R=")); Bridge.print(red);
    Bridge.print(F(" G=")); Bridge.print(green);
    Bridge.print(F(" B=")); Bridge.print(blue);
    Bridge.println(F("  (smaller = more of that color)"));
    Bridge.print(F("[CAL] normalized %     R=")); Bridge.print(rPct);
    Bridge.print(F(" G=")); Bridge.print(gPct);
    Bridge.print(F(" B=")); Bridge.print(bPct);
    Bridge.println(F("  (larger = more of that color)"));
    Bridge.print(F("[CAL] current classifier says: "));
    Bridge.println(gridColorName(gridDetectColorValue()));
    Bridge.println(F("[CAL] -----------------------------------"));
}

void openClawWithAttach() {
    if (emergencyStopActive) return;
    clawServo.attach(CLAW_SERVO_PIN);
    // Re-send the open angle a few times with settle pauses in between. If the
    // claw was gripping an object under stall current, a single write can be
    // dropped by a momentary brownout — repeating it gives the servo more
    // chances to actually reach the open position before we detach.
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
// Nudge exploration limits — DELIBERATELY the servo's full physical travel
// (0..180), NOT the CLAW_OPEN_ANGLE..CLAW_CLOSED_ANGLE grip range. This lets the
// LEFT/RIGHT buttons sweep the whole servo so you can physically find the true
// widest-open angle, then set CLAW_OPEN_ANGLE to whatever the [GRIPPER] Nudged-to
// readout shows at max open. Once that angle is known, this range can be tightened
// back to the grip limits to stop the nudge from over-driving the linkage.
const uint8_t CLAW_NUDGE_MIN = 0;
const uint8_t CLAW_NUDGE_MAX = 180;
void clawNudge(bool towardClosed) {
    if (emergencyStopActive) return;
    int target = towardClosed ? clawCurrentAngle + CLAW_NUDGE_STEP
                               : clawCurrentAngle - CLAW_NUDGE_STEP;
    target = constrain(target, CLAW_NUDGE_MIN, CLAW_NUDGE_MAX);
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
    delay(GRAB_SETTLE_MS); // let the car come to a full stop before gripping

    clawServo.attach(CLAW_SERVO_PIN);

    // SLOW SWEEP: Gradually close the claw instead of snapping it.
    // 1-degree steps with a 25ms dwell each — ~3x slower and smoother than the
    // old 2-degree/15ms sweep so the jaws ease onto the block instead of
    // snapping shut and knocking it out of alignment. Raise the delay further
    // (30/35) to slow it more; shrink it toward 15 to speed the close back up.
    for (int angle = CLAW_OPEN_ANGLE; angle <= CLAW_CLOSED_ANGLE; angle += 1) {
        clawServo.write(angle);
        delay(25);
    }

    delay(800); // Wait almost a full second to ensure a firm grip before moving

    clawServo.detach();
    clawCurrentAngle = CLAW_CLOSED_ANGLE;

    itemGrabbed = true;
    disableSensors();

    delay(GRAB_SETTLE_MS); // let the grip firm up before the robot moves the item
}

// Reads the block color (majority vote) and grips ONLY if it matches `color`
// (or `color == ANY`). Returns true if the claw actually closed on the block,
// false on a color mismatch (claw left open, car untouched). Callers use the
// return value to decide whether to keep searching the other blocks.
bool grabColorIfMatch(int color) {
    if (emergencyStopActive) return false;

    // Only DECIDE on colour when the block is actually close (~7cm calibrated read
    // distance, within COLOR_DECISION_DISTANCE_CM). Farther out the sensor reads
    // the floor and false-positives BLUE, which would grab the wrong block or make
    // the search reject the right one. If we're not close, the block isn't in
    // grabbing range anyway — report no match so the caller keeps approaching/
    // searching rather than gripping on a bogus read.
    int distance = gridGetDistanceCm();
    if (distance <= 0 || distance > COLOR_DECISION_DISTANCE_CM) {
        Bridge.print(F("[GRAB] Too far for a trusted colour read (distance "));
        Bridge.print(distance);
        Bridge.println(F("cm) — skipping grip."));
        return false;
    }

    logColorRaw = true;                    // show raw R/G/B for each vote read
    int detected = gridDetectColorMajority();
    logColorRaw = false;

    Bridge.print(F("[GRAB] target="));
    Bridge.print(gridColorName(color));
    Bridge.print(F(" detected="));
    Bridge.print(gridColorName(detected));
    Bridge.print(F(" @ "));
    Bridge.print(distance);
    Bridge.println(F("cm"));

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
// Returns true if it stopped because a block is in grab range, false if it
// reached the column-1 junction (all line sensors HIGH) WITHOUT finding a block.
// The block is ALWAYS at column 1, so if the car creeps all the way to that
// junction with nothing in ultrasonic range, there is no block here — stop
// instead of driving forward off the grid, and let the caller search elsewhere.
bool executeApproachMovement(int currentDistance, uint8_t startSpeed) {
    if (emergencyStopActive) return false;
    unsigned long telemetryTickMillis = millis();
    uint8_t floorSpeed = (startSpeed < SPEED_MIN) ? min(APPROACH_SPEED_FLOOR, startSpeed) : SPEED_MIN;
    // Running commanded speed, slewed toward the distance-derived target each tick
    // so the car glides in instead of stuttering on ultrasonic jitter. Seeded at
    // startSpeed since that's what the very first (far) tick would command anyway.
    uint8_t smoothedSpeed = startSpeed;

    while (!isGrabTargetReached(currentDistance)) {
        if (checkEmergencyStop()) return false;
        // One ultrasonic ping per tick drives BOTH the speed ramp and the loop
        // exit test. Do NOT read the colour sensor here: a full colour read is
        // 9 blocking pulseIn calls (up to ~0.3s of *variable* stall) and the
        // approach only needs distance — reading it every tick made the loop
        // period lurch and the motors stutter. Colour is decided later in
        // grabColorIfMatch; here we only sample it for the telemetry line when
        // debug output is actually on.
        int distance = gridGetDistanceCm();
        currentDistance = distance;
        if (debugTelemetry) {
            Bridge.print(F("[APPROACH] "));
            printStatusTelemetry(distance, gridDetectColorValue());
        }

        // Boundary guard: the block is always at column 1. If the car has crept
        // to the column-1 junction (all three line sensors HIGH) and the
        // ultrasonic still sees no block in range, there is no block on this row —
        // stop here rather than driving forward past column 1 and off the grid.
        if (digitalRead(LINE_LEFT_PIN) == HIGH &&
            digitalRead(LINE_CENTER_PIN) == HIGH &&
            digitalRead(LINE_RIGHT_PIN) == HIGH) {
            gridStop();
            Bridge.println(F("[APPROACH] Reached column-1 junction with no block — stopping (no block here)."));
            return false;
        }

        uint8_t targetSpeed = startSpeed;
        if (distance > 0) {
            // Linearly ramp down from startSpeed at ITEM_DETECT_DISTANCE_CM
            // down to floorSpeed at GRAB_APPROACH_DISTANCE_CM.
            int span = ITEM_DETECT_DISTANCE_CM - GRAB_APPROACH_DISTANCE_CM;
            int clamped = constrain(distance, GRAB_APPROACH_DISTANCE_CM, ITEM_DETECT_DISTANCE_CM);
            int scaled = floorSpeed + (long)(startSpeed - floorSpeed) * (clamped - GRAB_APPROACH_DISTANCE_CM) / span;
            targetSpeed = (uint8_t)constrain(scaled, floorSpeed, startSpeed);
        }

        // Slew the commanded speed toward the target by at most APPROACH_SPEED_SLEW
        // per tick, so ultrasonic jitter and the ramp-down both come out smooth
        // instead of the motors snapping to a new PWM every 60ms.
        if (targetSpeed > smoothedSpeed) {
            smoothedSpeed = min((int)targetSpeed, smoothedSpeed + APPROACH_SPEED_SLEW);
        } else if (targetSpeed < smoothedSpeed) {
            smoothedSpeed = max((int)targetSpeed, smoothedSpeed - APPROACH_SPEED_SLEW);
        }
        uint8_t approachSpeed = smoothedSpeed;

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
        delay(APPROACH_TICK_MS);
        motionTelemetryTick(telemetryTickMillis);
    }
    gridStop();
    return true;   // exited the loop because a block is within grab range
}

// Rotates in place until currentHeading matches targetHeading, always
// turning the short way round the NORTH/EAST/SOUTH/WEST compass.
void turnToHeading(int targetHeading) {
    while (currentHeading != targetHeading) {
        if (checkEmergencyStop()) return;
        int diff = (targetHeading - currentHeading + 4) % 4;
        if (diff == 1) {
            if (!gridRotateRight90()) return;
            currentHeading = (currentHeading + 1) % 4;
        } else {
            if (!gridRotateLeft90()) return;
            currentHeading = (currentHeading + 3) % 4;
        }
        reportPosition();
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

    int deltaX = targetX - currentX;
    if (deltaX != 0) {
        turnToHeading((deltaX > 0) ? EAST : WEST);
        for (int i = 0; i < abs(deltaX); i++) {
            if (checkEmergencyStop()) return;
            gridMoveForwardOneCoord(SPEED_START_FAST);
            if (emergencyStopActive) return;
            currentX += (currentHeading == EAST) ? 1 : -1;
            reportPosition();
        }
    }

    int deltaY = targetY - currentY;
    if (deltaY != 0) {
        turnToHeading((deltaY > 0) ? NORTH : SOUTH);
        for (int i = 0; i < abs(deltaY); i++) {
            if (checkEmergencyStop()) return;
            gridMoveForwardOneCoord(SPEED_START_FAST);
            if (emergencyStopActive) return;
            currentY += (currentHeading == NORTH) ? 1 : -1;
            reportPosition();
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
    bool blockFound = executeApproachMovement(distance, approachStartSpeed);
    if (emergencyStopActive) return false;
    // No block on this row (car stopped at the column-1 junction) — don't try to
    // grip, just report no-grab so the caller (search) moves to the next block.
    if (!blockFound) {
        Bridge.println(F("[GRAB] No block found on this row — skipping grip."));
        return false;
    }
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

// Line-follow EAST from home out to the drop point (9,3) and stop, ready to
// release. Uses the SLOW approach speed (same crawl as the block grab) with the
// same line-centering correction, so the car straightens onto the line before it
// stops — the block is released dead-straight rather than skewed. Tracks currentX
// as it crosses junctions and stops the instant it reaches DROP_TARGET_X (9).
// Assumes it starts on home (7,3) facing EAST (moveHome / carryHomeViaRow3 leave
// it there).
void goToDrop() {
    if (checkEmergencyStop()) return;
    bool activeJunctionFlag = true;   // debounce: only count a junction on its leading edge
    unsigned long lineLostSince = 0;
    // Segment timer for recovery's time-based projection. NOTE: this run crawls at
    // PATH_APPROACH_SPEED, slower than the SPEED_START_FAST that MS_PER_COORD is
    // calibrated at, so the time->coordinate estimate here is rough — it only kicks
    // in on a lost line as a best-effort fallback, and the projection is clamped at
    // the X=9 drop boundary so it can never march past the drop node.
    unsigned long segmentStartMillis = millis();

    while (currentX < DROP_TARGET_X) {
        if (checkEmergencyStop()) return;
        // goToDrop OWNS currentX (it bumps it on each junction below), so recovery
        // here projects the estimate onto the global coordinate — pass true.
        if (recoverOrStop(lineLostSince, segmentStartMillis, true)) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        // Line-follow with the SAME crawl speed, correction boost and tick as the
        // proven block-grab approach, so the drive out to the drop point tracks
        // the line just as smoothly. A single consistent speed keeps the steer
        // differential gentle (no weaving), and the reverse-brake after the loop —
        // not a slower crawl — is what nails the precise stop on the X=9 line.
        if (Left == HIGH && Center == HIGH && Right == HIGH) {
            // Junction cross — count one coordinate step east on its leading edge.
            if (!activeJunctionFlag) {
                activeJunctionFlag = true;
                currentX += 1;
                segmentStartMillis = millis();   // fresh junction — restart the projection timer
                reportPosition();
                if (currentX >= DROP_TARGET_X) break;   // reached (9,3)
            }
            gridSetSpeed(PATH_APPROACH_SPEED); mecCar.Advance();
        } else if (Left == LOW && Center == HIGH && Right == LOW) {
            activeJunctionFlag = false;
            gridSetSpeed(PATH_APPROACH_SPEED); mecCar.Advance();
        } else if (Left == LOW && Center == LOW && Right == HIGH) {
            activeJunctionFlag = false;
            gridSteer(PATH_APPROACH_SPEED, APPROACH_CORRECTION_BOOST, false);
        } else if (Left == HIGH && Center == LOW && Right == LOW) {
            activeJunctionFlag = false;
            gridSteer(PATH_APPROACH_SPEED, APPROACH_CORRECTION_BOOST, true);
        } else if (Right == HIGH) {
            activeJunctionFlag = false;
            gridSteer(PATH_APPROACH_SPEED, APPROACH_CORRECTION_BOOST, false);
        } else if (Left == HIGH) {
            activeJunctionFlag = false;
            gridSteer(PATH_APPROACH_SPEED, APPROACH_CORRECTION_BOOST, true);
        } else {
            gridSetSpeed(PATH_APPROACH_SPEED); mecCar.Advance();
        }
        delay(40);   // slightly finer than the 60 ms grab tick so corrections fire
                     // more often on the straight run — smoother line tracking.
    }
    // Reverse-brake the instant X=9 is counted. The loop breaks on the LEADING edge
    // of the drop band, with the car still rolling; without this pulse it coasts
    // through the (wide) 9,3 mark before anything else runs. Kill that momentum
    // here so the settle below starts from a car actually parked on the node.
    if (!emergencyStopActive && DROP_BRAKE_MS > 0) {
        gridSetSpeed(SPEED_REVERSE_BUMP);
        mecCar.Back();
        unsigned long brakeStart = millis();
        while (millis() - brakeStart < DROP_BRAKE_MS) { if (checkEmergencyStop()) break; }
        gridStop();
    }
    // Final alignment settle: square the chassis onto the drop mark BEFORE moving
    // in to release. The drive-out crawl can arrive a touch skewed (near the stall
    // floor there isn't much correction authority per tick), which sets the block
    // down at an angle. Here we STEER-only — the outer side nudges the car back
    // toward centre while the base crawl speed keeps it from running forward much.
    //
    // The X=9 drop mark is a WIDE band, so "centred" is NOT "both edges off the
    // line" — on a wide mark the outer L/R sensors sit ON the band and read HIGH.
    // Squared-up is BOTH edges ON the band (L and R both HIGH). Accepting any
    // L==R state instead also accepts both-LOW, which is the car having rolled
    // CLEAR OFF the far side of the band — indistinguishable from square, so the
    // settle would break early and the forward nudge would push it past the drop
    // zone. Requiring both-HIGH means only a car actually sitting on the mark
    // counts as aligned. We steer whenever the edges disagree (one on the mark,
    // one off = skewed), and confirm alignment over two consecutive ticks. Fine
    // 25 ms tick so the squaring is quick and doesn't overshoot into a weave.
    if (!emergencyStopActive && DROP_ALIGN_SETTLE_MS > 0) {
        unsigned long settleStart = millis();
        uint8_t alignedTicks = 0;
        while (millis() - settleStart < DROP_ALIGN_SETTLE_MS) {
            if (checkEmergencyStop()) return;
            uint8_t Left = digitalRead(LINE_LEFT_PIN);
            uint8_t Right = digitalRead(LINE_RIGHT_PIN);
            if (Left == HIGH && Right == HIGH) {
                // Both edges on the wide band → chassis is square ON the mark.
                // Confirm over two ticks so a momentary reading doesn't end early.
                gridStop();
                if (++alignedTicks >= 2) break;
            } else if (Left == LOW && Right == LOW) {
                // Both edges clear of the band. Either the car has not reached the
                // mark yet or it has rolled off the far side; neither is aligned,
                // so don't count it — hold position and let the settle window
                // expire rather than nudging forward from an unknown pose.
                alignedTicks = 0;
                gridStop();
            } else if (Right == HIGH) {
                // Right edge on the mark, left off → skewed; arc right to square up.
                alignedTicks = 0;
                gridSteer(PATH_APPROACH_SPEED, APPROACH_CORRECTION_BOOST, false);
            } else {
                // Left edge on the mark, right off → arc left to square up.
                alignedTicks = 0;
                gridSteer(PATH_APPROACH_SPEED, APPROACH_CORRECTION_BOOST, true);
            }
            delay(25);
        }
        gridStop();
    }
    // Optional nudge further FORWARD into the drop area before stopping. Disabled
    // (DROP_FORWARD_NUDGE_MS = 0) while the car is required to stop on the (9,3)
    // junction itself; the brake above is what sets the final position.
    if (!emergencyStopActive && DROP_FORWARD_NUDGE_MS > 0) {
        gridSetSpeed(PATH_APPROACH_SPEED);
        mecCar.Advance();
        unsigned long nudgeStart = millis();
        while (millis() - nudgeStart < DROP_FORWARD_NUDGE_MS) { if (checkEmergencyStop()) break; }
    }
    gridStop();
    if (!emergencyStopActive) {
        Bridge.print(F("[SYSTEM] Drop point (9,3) reached at column "));
        Bridge.print(currentX);
        Bridge.println(F(" — stopped, ready to release."));
    }
}

void resetCoordinates() {
    currentX = 7; currentY = 3; currentHeading = WEST;
    emergencyStopActive = false;
    Bridge.println(F("[SYSTEM] Navigation Tracker Reset to Default Home Baseline (7,3) facing WEST."));
    reportPosition();
}

// Called once the car is sitting on the drop line at (9,3) facing EAST (right
// after goToDrop()). Releases the carried block, backs off a little so the 180°
// spin clears the block it just set down, turns to face WEST, then line-follows
// one node back onto home (7,3). The return leg uses moveCoord(), which counts
// the junction crossing rather than dead-reckoning distance, so the earlier
// reverse bump doesn't throw off where it lands. Leaves the tracker reset to the
// home baseline (7,3) facing WEST, ready for the next mission.
void releaseAndReturnHome() {
    if (checkEmergencyStop()) return;
    delay(300);
    openClawWithAttach();          // release the carried block at (9,3)
    delay(300);
    // Back off farther than the plain slight-reverse so the 180° spin swings fully
    // clear of the block just released (the short bump let the chassis clip it).
    gridSetSpeed(SPEED_REVERSE_BUMP);
    mecCar.Back();
    unsigned long clearStart = millis();
    while (millis() - clearStart < DROP_CLEAR_REVERSE_MS) { if (checkEmergencyStop()) return; }
    gridStop();
    if (emergencyStopActive) return;
    Bridge.println(F("[RETURN] Released — spinning 180 and heading back to home (7,3)."));
    turnToHeading(WEST);           // 180° spin from EAST to face back toward home
    if (emergencyStopActive) return;
    moveCoord(7, 3);               // line-follow west one node onto home (7,3)
    if (emergencyStopActive) return;
    resetCoordinates();            // re-seed the home baseline (7,3) facing WEST
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

// Backs the car off the block until it re-acquires the grid node (all three
// line sensors on the junction cross = all HIGH), so the coordinate-tracked
// navigation to the next block starts from a known node. The grab approach
// crept WEST off the node onto the block, so a fixed slight-reverse isn't
// enough to guarantee we're back on the junction — reverse until we see it.
static void backOffBlock() {
    if (checkEmergencyStop()) return;
    // If we're already sitting on the junction (the no-block case: the approach
    // stopped the car right on the column-1 junction), we're already at a known
    // node — don't reverse, or we'd drift one node east and desync the tracker.
    if (digitalRead(LINE_LEFT_PIN) == HIGH &&
        digitalRead(LINE_CENTER_PIN) == HIGH &&
        digitalRead(LINE_RIGHT_PIN) == HIGH) {
        Bridge.println(F("[SEARCH] Already on the grid node — no back-off needed."));
        return;
    }
    Bridge.println(F("[SEARCH] Reversing off block to re-acquire grid node..."));
    gridSetSpeed(SPEED_REVERSE_BUMP);
    mecCar.Back();
    // Cap the reverse so a missed junction can't run the car off the mat.
    unsigned long startTime = millis();
    const unsigned long BACKOFF_TIMEOUT_MS = 2500;
    while (millis() - startTime < BACKOFF_TIMEOUT_MS) {
        if (checkEmergencyStop()) return;
        if (digitalRead(LINE_LEFT_PIN) == HIGH &&
            digitalRead(LINE_CENTER_PIN) == HIGH &&
            digitalRead(LINE_RIGHT_PIN) == HIGH) {
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
// sensors HIGH). Bumps currentX by +1. Used by the post-grab return to pull back
// onto the clear X=3 corridor without a 180° turn.
static void reverseOneCoordEast() {
    if (checkEmergencyStop()) return;
    Bridge.println(F("[RETURN] Reversing one node east onto the corridor..."));
    gridSetSpeed(SPEED_REVERSE_BUMP);
    mecCar.Back();
    unsigned long t0 = millis();                 // clear the current junction (blind)
    while (millis() - t0 < 400) { if (checkEmergencyStop()) return; }
    unsigned long startTime = millis();          // then reverse to the next junction
    const unsigned long REVERSE_TIMEOUT_MS = 2500;
    while (millis() - startTime < REVERSE_TIMEOUT_MS) {
        if (checkEmergencyStop()) return;
        if (digitalRead(LINE_LEFT_PIN) == HIGH &&
            digitalRead(LINE_CENTER_PIN) == HIGH &&
            digitalRead(LINE_RIGHT_PIN) == HIGH) {
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

        // Route to the block via the X=4 corridor, then approach only as far as
        // column 3 (TWO nodes EAST of the block at column 1). moveCoord does its
        // X move before its Y move, so going to (4,row) first pulls the car out
        // to column 4 before it changes rows — every reorientation turn happens
        // at column 4, well clear of the block column, and inter-block travel
        // stays on that corridor. The car then stops at column 3 and lets
        // moveToGrab creep the last bit with the ultrasonic — starting the slow,
        // line-corrected approach ONE BLOCK EARLIER (col 3 instead of col 2) so it
        // has a full extra grid cell to straighten and slow before the block. It
        // still never drives onto column 1 and clashes with the block.
        moveCoord(4, row);
        if (emergencyStopActive) return -1;
        moveCoord(3, row);
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
        backOffBlock();                 // reverse to the col-2 junction, still facing WEST
        if (emergencyStopActive) return -1;
        // Resync the tracker to where the car PHYSICALLY is after the back-off.
        // The approach now starts at column 3, but backOffBlock always reverses to
        // the first junction east of the block (column 2, since the block sits at
        // column 1) — so currentX would otherwise read 3 (the last moveCoord node)
        // while the car sits at 2. Pin it to 2 so the two-node re-hop below (2->4)
        // stays correct regardless of where the approach started.
        currentX = 2;
        reportPosition();

        // If another block remains, pull EAST back onto the X=4 corridor WHILE
        // STILL FACING WEST (reverse, no turn). Without this the car sits at
        // column 2 facing WEST, and the next moveCoord(4,row) needs it facing EAST
        // to step up in X — turning WEST->EAST is a 180° spin. Reversing to column
        // 4 first makes that X move zero, so the next hop is only a 90° row change
        // and the reorientation turn stays at column 4. Two nodes: 2 -> 3 -> 4.
        if (i < 2) {
            reverseOneCoordEast();      // (2,row) -> (3,row), still WEST
            if (emergencyStopActive) return -1;
            reverseOneCoordEast();      // (3,row) -> (4,row), currentX now 4, still WEST
            if (emergencyStopActive) return -1;
        }
    }

    Bridge.println(F("[SEARCH] No matching block found — returning home."));
    moveHome();
    return -1;
}

// Carries a just-grabbed block back and out to the drop zone. Entered right
// after the grab with the car at column 2, currentY = the grabbed row, facing
// WEST (block held in front). Route: back off to the block's junction, reverse
// one more node east onto the clear X=3 corridor (still WEST-facing — no turn),
// funnel onto the central row 3 ("path 2"), then run east to home and out to the
// drop.
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
    // The grab approach starts at column 3 now, but backOffBlock reverses to the
    // first junction east of the block — column 2 (block sits at column 1). Pin
    // the tracker to 2 so the reverseOneCoordEast below lands currentX at 3.
    currentX = 2;
    reverseOneCoordEast();          // reverse to (3, grabbedRow), still facing WEST
    if (emergencyStopActive) return;

    // Funnel onto the central row 3 along the X=3 corridor (rows 1/5 only).
    if (grabbedRow != 3) {
        int dir = (grabbedRow > 3) ? SOUTH : NORTH;
        turnToHeading(dir);
        if (emergencyStopActive) return;
        int steps = abs(grabbedRow - 3);
        for (int i = 0; i < steps; i++) {
            if (checkEmergencyStop()) return;
            gridMoveForwardOneCoord(SPEED_START_FAST);
            if (emergencyStopActive) return;
            currentY += (dir == NORTH) ? 1 : -1;
            reportPosition();
        }
    }

    // Face east and run along row 3 back to home (X=7). From row 1/5 this is a
    // 90° turn; from row 3 (path 2) it is the one 180° the route allows.
    turnToHeading(EAST);
    if (emergencyStopActive) return;
    while (currentX < 7) {
        if (checkEmergencyStop()) return;
        gridMoveForwardOneCoord(SPEED_START_FAST);
        if (emergencyStopActive) return;
        currentX += 1;
        reportPosition();
    }

    Bridge.println(F("[RETURN] Home reached via row 3 — heading to drop."));
    goToDrop();                     // line-follow east from home out to the drop zone
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
    // X=3 corridor, funnel onto row 3, then run east to home and to the drop —
    // avoiding a 180° turn except for a row-3 (path 2) block.
    carryHomeViaRow3();
    if (emergencyStopActive) return;
    releaseAndReturnHome();        // release, back off, spin 180, drive home to (7,3)
}

// ── Array Based Instruction Execution ───────────────────────────────────────
enum Action { FWD, LFT, RGT, GRB, REV, DRP, GDR, DLY };
struct Step { Action act; uint8_t arg; };

const Step path1[] = {
    // First leg: on the real mat FWD 2 overshot to column 3; FWD 1 lands the first
    // turn on column 5 (home col 7 -> 5) as intended.
    {FWD, 1}, {DLY, 3}, {LFT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3},
    // Approach from column 5 (turn stays at col 5). FWD 4 = col 5->1 gives a long
    // straight line-follow run so the car re-centers after the RGT turn before the
    // ultrasonic creeps the last stretch onto the block at column 1.
    {FWD, 4}, {DLY, 3}, {GRB, ANY}, {RGT, 0}, {DLY, 3}, {REV, 0}, {DLY, 3}, {RGT, 0},
    // Return leg ends at (3,3) facing EAST; GDR seeds that pose then slow-drops at (9,3).
    // DRP then releases, backs off, spins 180 and drives home to (7,3).
    {DLY, 5}, {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3},{GDR, 3}, {DLY, 2},{DRP, 0}
};

const Step path2[] = {
    {FWD, 4}, {DLY, 3}, {GRB, ANY}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {REV, 0}, {DLY, 3},
    // Return leg ends at (1,3) facing EAST; GDR seeds that pose then slow-drops at (9,3).
    // DRP then releases, backs off, spins 180 and drives home to (7,3).
    {RGT, 0}, {DLY, 3}, {GDR, 1}, {DLY, 2}, {DRP, 0}
};
//GDR was = 0
const Step path3[] = {
    // First leg: on the real mat FWD 2 overshot to column 3; FWD 1 lands the first
    // turn on column 5 (home col 7 -> 5) as intended (mirror of path1).
    {FWD, 1}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3},
    // Approach from column 5 (turn stays at col 5). FWD 4 = col 5->1 gives a long
    // straight line-follow run so the car re-centers after the LFT turn before the
    // ultrasonic creeps the last stretch onto the block at column 1 (mirror of path1).
    {FWD, 4}, {DLY, 3}, {GRB, ANY}, {LFT, 0}, {DLY, 3}, {REV, 0}, {DLY, 3},{LFT, 0},
    // Return leg ends at (3,3) facing EAST; GDR seeds that pose then slow-drops at (9,3).
    // DRP then releases, backs off, spins 180 and drives home to (7,3).
    {DLY, 3}, {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3},{GDR, 3}, {DLY, 2},{DRP, 0}
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
            case DRP: releaseAndReturnHome(); break;   // release, back off, spin 180, drive home to (7,3)

            case GDR:
                // Scripted paths don't track coordinates during FWD/LFT/RGT, so
                // seed the tracker to the known pre-drop pose before the coordinate
                // drop: the car is on row 3 facing EAST, at the column given in arg
                // (path1/path3 finish their return at column 3). goToDrop() then
                // slow-approaches EAST and stops exactly at (9,3).
                currentX = path[i].arg; currentY = 3; currentHeading = EAST;
                reportPosition();
                goToDrop();
                break;
            case DLY: delay(path[i].arg * 100); break;
        }
        // Brief settling pause between steps so transitions aren't abrupt.
        if (!emergencyStopActive) delay(STEP_TRANSITION_MS);
    }
}

void executeAutoMission(int targetY, int color) {
    sensorsEnabled = true;
    gridMoveForwardBlocks(2, SPEED_START_FAST);
    // Stop at column 3 (two nodes east of the block at column 1) so the ultrasonic
    // creep starts ONE BLOCK EARLIER — an extra grid cell to straighten and slow
    // before the block. moveHome() below re-anchors currentX absolutely, so the
    // creep leaving currentX at 3 doesn't desync the tracker.
    moveCoord(3, targetY);
    if (emergencyStopActive) return;
    delay(300);
    moveToGrab(color);
    if (emergencyStopActive) return;
    delay(300);
    moveHome();
    goToDrop();
    if (emergencyStopActive) return;
    releaseAndReturnHome();        // release, back off, spin 180, drive home to (7,3)
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
    String line;
    if (Serial.available()) {
        line = Serial.readStringUntil('\n');
    } else if (espSerial.available()) {
        line = espSerial.readStringUntil('\n');
    } else {
        return;
    }
    line.trim();
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

    // SLEEP/IDLE puts the car into the low-power idle state (motors coasted,
    // colour sensor powered down, telemetry off). Handled before the auto
    // re-arm below so it returns without falling through to sensorsEnabled=true.
    if (verb == "SLEEP" || verb == "IDLE") {
        enterIdleSleep();
        Bridge.println(F("[ACK] SLEEP — idle low-power"));
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
        bool prevSensors = sensorsEnabled;
        sensorsEnabled = true;                   // force a read regardless of nav state
        int distance = gridGetDistanceCm();
        int color = gridDetectColorValue();
        sensorsEnabled = prevSensors;
        Bridge.println(F("[ACK] PING"));
        printStatusTelemetry(distance, color);
    } else if (verb == "CAL") {                  // color calibration sample
        // Hold ONE colored block at the target read distance, then send CAL.
        // Prints averaged raw + normalized R/G/B for tuning the classifier.
        bool prevSensors = sensorsEnabled;
        sensorsEnabled = true;
        Bridge.println(F("[ACK] CAL"));
        gridCalibrateColorReport();
        sensorsEnabled = prevSensors;
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
#ifdef COLOR_LED_PIN
    pinMode(COLOR_LED_PIN, OUTPUT);
    digitalWrite(COLOR_LED_PIN, HIGH);   // module LEDs on for normal running
#endif

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