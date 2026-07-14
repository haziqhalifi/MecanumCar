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
// one node in from every edge. Tracked node is clamped to X 1..8, Y 1..5:
// X=1 = block column (car stops here, claw reaches the west edge), X=8 keeps it
// one in from the START zone's east edge (X9); Y 1..5 is one in from the
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
// Start speed for the scripted-path block approach. Kept below SPEED_MIN so the
// approach creeps in noticeably slower and still tapers toward APPROACH_SPEED_FLOOR.
const uint8_t PATH_APPROACH_SPEED = 42;
const int ITEM_DETECT_DISTANCE_CM = 25;
const int GRAB_APPROACH_DISTANCE_CM = 6;   // close the claw at <= 6cm (measured: where the block sits in the claw's reach)
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
const uint8_t APPROACH_SPEED_FLOOR = 38;
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


void gridSetSpeed(uint8_t targetSpeed) {
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

void spinRightInPlace(uint8_t speed) { gridSetSpeed(speed); mecCar.Turn_Right(); }
void spinLeftInPlace(uint8_t speed) { gridSetSpeed(speed); mecCar.Turn_Left(); }

void disableSensors() {
    sensorsEnabled = false;
    Bridge.println(F("[SYSTEM] Ultrasonic and Color sensors have been SOFTWARE DISABLED."));
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

    while (junctionCount < targetBlocks) {
        if (checkEmergencyStop()) return;
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
    delay(GRAB_SETTLE_MS); // let the car come to a full stop before gripping

    clawServo.attach(CLAW_SERVO_PIN);

    // SLOW SWEEP: Gradually close the claw instead of snapping it
    for (int angle = CLAW_OPEN_ANGLE; angle <= CLAW_CLOSED_ANGLE; angle += 2) {
        clawServo.write(angle);
        delay(15); // Increase to 20 or 25 if it's STILL too fast
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
    gridSetSpeed(SPEED_START_FAST);

    while (junctionsEncountered < 4) {
        if (checkEmergencyStop()) return;
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

        // Drive to the grid node just east of this block, facing WEST.
        moveCoord(1, row);
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

    // Back off the block onto the grid node before navigating home — the grab
    // approach crept WEST past node (1,row) onto the block at (0,row), so the
    // car's physical position is ~1 block west of its tracked coordinate.
    backOffBlock();
    if (emergencyStopActive) return;

    // Carry the grabbed block home and out to the drop zone.
    moveHome();
    if (emergencyStopActive) return;
    goToDrop();
    if (emergencyStopActive) return;
    delay(300);
    openClawWithAttach();          // release at the drop zone
    resetCoordinates();
}

// ── Array Based Instruction Execution ───────────────────────────────────────
enum Action { FWD, LFT, RGT, GRB, REV, DRP, GDR, DLY };
struct Step { Action act; uint8_t arg; };

const Step path1[] = {
    {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3},
    {FWD, 2}, {DLY, 3}, {GRB, ANY}, {RGT, 0}, {DLY, 3}, {REV, 0}, {DLY, 3}, {RGT, 0}, 
    {DLY, 5}, {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3},{FWD, 5}, {DLY, 2},{DRP, 0},
    {REV, 2}
};

const Step path2[] = {
    {FWD, 4}, {DLY, 3}, {GRB, ANY}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {REV, 0}, {DLY, 3}, 
    {RGT, 0}, {DLY, 3}, {FWD, 7}, {DLY, 2}, {DRP, 0},
    {REV, 2}
};
//GDR was = 0
const Step path3[] = {
    {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3}, 
    {FWD, 2}, {DLY, 3}, {GRB, ANY}, {LFT, 0}, {DLY, 3}, {REV, 0}, {DLY, 3},{LFT, 0}, 
    {DLY, 3}, {FWD, 2}, {DLY, 3}, {RGT, 0}, {DLY, 3}, {FWD, 2}, {DLY, 3}, {LFT, 0}, {DLY, 3},{FWD, 5}, {DLY, 2},{DRP, 0},
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

    // STOP must work even while an emergency stop is latched, and it also
    // clears the latch so subsequent debug commands can run.
    if (verb == "STOP") {
        emergencyStopActive = true;
        gridStop();
        Bridge.println(F("[ACK] STOP — motors halted, e-stop latched"));
        return;
    }
    if (verb == "RESUME" || verb == "ARM") {
        emergencyStopActive = false;
        Bridge.println(F("[ACK] RESUME — e-stop cleared, ready"));
        return;
    }

    // Any other command is refused while latched, so the dashboard STOP stays
    // authoritative until explicitly resumed.
    if (emergencyStopActive) {
        Bridge.println(F("[ACK] IGNORED — e-stop active, send RESUME first"));
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