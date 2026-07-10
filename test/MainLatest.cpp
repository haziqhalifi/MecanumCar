/*
  ================================================================
  KS0560 Mecanum Car — FULL CONFIGURATION WITH STRAFE CALIBRATION
  ================================================================
  LOGIC:
    - Button 0: Emergency Stop.
    - Button 1: Move forward by TARGET_BLOCKS_CMD1 (4 blocks).
    - Button 2: Rotate Right, then move forward by TARGET_BLOCKS_CMD2 (2 blocks).
    - Button 3: Rotate Left, then move forward by TARGET_BLOCKS_CMD3 (2 blocks).
    - Button ROTATE_R (arrow): Same as Button 5, mirrored (turns swapped left<->right).
    - Button ROTATE_L (arrow): Same as Button 5.
    - Button 6: Spin 180 degrees, then move forward by TARGET_BLOCKS_CMD180.
    - Button 7: Strafe Left, counts junctions via SENSOR_LEFT only.
    - Button 8: Spin 180 degrees only (no forward move afterward).
    - Button 9: Strafe Right, counts junctions via SENSOR_RIGHT only.
    - Button 4: Sequence - forward 4, turn left, forward 2, turn right, forward 2, turn 180.
    - Button 5: Same as Button 4, then mirrored in reverse to return to the start location/heading.
    - Button UP (arrow): Sequence - forward 5, turn 180, forward 5.
    - Before any 180-degree turn triggered by Up/Left/Right (arrow) buttons:
      approach whatever is in front on the ultrasonic sensor and grab it,
      regardless of color, before turning.
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>
#include <SoftwareSerial.h>
#include <Servo.h>

// ── Hardware pins ─────────────────────────────────────────────
#define RECV_PIN        A3
#define SENSOR_LEFT     A0
#define SENSOR_MID      A1
#define SENSOR_RIGHT    A2

// Color sensor (TCS3200), ultrasonic (HC-SR04), gripper servo — added for the
// pre-180-turn grab. NOTE: the servo needs its own 5-6V supply with a common
// ground; the Arduino 5V pin can't source a servo's stall current and doing
// so can brown out the board (freezes Serial/everything).
#define TCS_OUT_PIN           8
#define TCS_S2_PIN            7
#define TCS_S3_PIN            6
#define GRIPPER_PIN           9
#define ULTRASONIC_TRIG_PIN   12
#define ULTRASONIC_ECHO_PIN   13

// Dedicated link to the ESP32 WiFi bridge (see esp32-wifi-bridge/), kept
// separate from the USB debug Serial so telemetry doesn't get mixed in with
// (or blocked by) debug prints. Same wiring convention as lib/main.cpp:
// Uno pin 11 (RX) <- ESP32 pin 17 (TX2), Uno pin 10 (TX) -> ESP32 pin 16 (RX2).
#define ESP_RX_PIN      11
#define ESP_TX_PIN      10
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

// ── EXTERN LIBRARY VARIABLES ──────────────────────────────────
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

// ── IR Codes ──────────────────────────────────────────────────
#define CMD_STAR        0x52 // Digit 0 -> Emergency Stop
#define CMD_1           0x16 // Digit 1
#define CMD_2           0x19 // Digit 2
#define CMD_3           0x0D // Digit 3
#define CMD_ROTATE_R    0x43 // Arrow button (PLAY/PAUSE) -> Sequence 4 (Sequence 2 mirrored, left<->right swapped)
#define CMD_ROTATE_L    0x44 // Arrow button (PREV) -> Sequence 2 (same as Button 5)
#define CMD_UP          0x46 // Arrow button (UP) -> Sequence 3: forward 5, turn 180, forward 5
#define CMD_180         0x5E // Digit 6 -> 180 degree turn, then move forward
#define CMD_7           0x08 // Digit 7 -> Strafe Left
#define CMD_180_ONLY    0x1C // Digit 8 -> 180 degree turn only (no forward move)
#define CMD_9           0x5A // Digit 9 -> Strafe Right
#define CMD_SEQ1        0x0C // Digit 4 -> Sequence: fwd 4, left, fwd 2, right, fwd 2, 180
#define CMD_SEQ2        0x18 // Digit 5 -> Sequence 1, then mirrored in reverse back to start
#define CMD_HASH        0x4A // '#' button -> release (open) the gripper

// ── MOVEMENT SETTINGS / COUNTS ────────────────────────────────
const int TARGET_BLOCKS_CMD1   = 4;  // How many blocks Button 1 moves forward
const int TARGET_BLOCKS_CMD2   = 2;  // How many blocks Button 2 moves forward after turning
const int TARGET_BLOCKS_CMD3   = 2;  // How many blocks Button 3 moves forward after turning
const int TARGET_STRAFE_CMD7   = 2;  // How many left junctions Button 7 counts while strafing left
const int TARGET_STRAFE_CMD9   = 2;  // How many right junctions Button 9 counts while strafing right
const int TARGET_BLOCKS_CMD180 = 2;  // How many blocks Button 180 moves forward after turning
const int TARGET_BLOCKS_SEQ1_A = 4;  // Sequence leg 1: forward blocks before first turn
const int TARGET_BLOCKS_SEQ1_B = 2;  // Sequence leg 2: forward blocks after left turn
const int TARGET_BLOCKS_SEQ1_C = 2;  // Sequence leg 3: forward blocks after right turn
const int TARGET_BLOCKS_SEQ2_FINAL = 5;  // Button 5: forward blocks on the final return leg
const int TARGET_BLOCKS_SEQ3 = 5;  // Button UP: forward blocks before and after the 180 turn

// ── GRAB (color + ultrasonic + gripper) ────────────────────────
#define GRIPPER_OPEN_US        500   // Servo pulse width: fully open
#define GRIPPER_CLOSE_US       1300  // Servo pulse width: hard max (stay within the servo's free range)
#define GRIPPER_GRAB_US        1100  // Servo pulse width: grab position
#define GRIPPER_MOVE_STEP      10    // us per micro-step when closing/opening gradually
#define GRIPPER_MOVE_DT        15    // ms between micro-steps (gentler current draw than a jump)

#define ULTRASONIC_TIMEOUT_US  30000UL   // pulseIn timeout for the HC-SR04 echo
#define APPROACH_STOP_DISTANCE_CM  4.5   // stop-and-grab distance (accounts for gripper reach); sits inside the HC-SR04's ~2-5cm blind zone, so the no-echo-streak fallback below is what actually triggers most grabs
#define APPROACH_SETTLE_MS     300       // pause after stopping / after grabbing
#define APPROACH_TIMEOUT_MS    8000      // give up approaching if never in range this long
#define NO_ECHO_STREAK_TO_GRAB 3         // consecutive no-echo reads (while approaching) before assuming "too close to read" and grabbing anyway
#define BACKGROUND_GRAB_CHECK_MS 500     // how often to poll the ultrasonic for a block while idle (not mid-command)

#define COLOR_PULSE_TIMEOUT_US 25000UL   // pulseIn timeout per TCS3200 color channel
#define BRIGHTNESS_BLACK_MAX   300.0     // total intensity below this reads as BLACK regardless of ratio
#define ACHROMATIC_TOLERANCE   0.08      // max/min ratio spread below this reads as WHITE (no dominant channel)

// ── SPEED CONFIGURATIONS ──────────────────────────────────────
#define SPEED_START_FAST       60    // Forward start speed for Button 1 (No turning)
#define SPEED_POST_RIGHT_TURN  60    // Speed forward after turning RIGHT (Button 2)
#define SPEED_POST_LEFT_TURN   60    // Speed forward after turning LEFT (Button 3)
#define SPEED_POST_180_TURN    60    // Speed forward after 180-degree turn (Button 180)
#define SPEED_STRAFE_LEFT      55    // Base starting speed for strafing left
#define SPEED_STRAFE_RIGHT     55    // Base starting speed for strafing right

#define SPEED_ROTATE           60    // Maintained strong speed for 90-degree rotations
#define SPEED_MIN              35    // Safety floor so dynamic slowdown speed never hits 0

// ── PWM STRAFE CALIBRATION (ADJUST TO GO STRAIGHT SIDEWAYS) ───
// Adjust these values (positive or negative) if your car drifts while strafing.
// Example: If the rear-left wheel spins too slowly, change BIAS_LL from 0 to 5 or 10.
#define STRAFE_L_BIAS_UL       15    // Takes the value of STRAFE_R_BIAS_UR
#define STRAFE_L_BIAS_LL      -10    // Takes the value of STRAFE_R_BIAS_LR
#define STRAFE_L_BIAS_UR        5    // Takes the value of STRAFE_R_BIAS_UL
#define STRAFE_L_BIAS_LR        10    // Takes the value of STRAFE_R_BIAS_LL

#define STRAFE_R_BIAS_UL       -5     // Strafe Right - Upper Left Wheel PWM Offset
#define STRAFE_R_BIAS_LL       5    // Strafe Right - Lower Left Wheel PWM Offset
#define STRAFE_R_BIAS_UR       10     // Strafe Right - Upper Right Wheel PWM Offset
#define STRAFE_R_BIAS_LR       -10     // Strafe Right - Lower Right Wheel PWM Offset

// ── TIMING & NUDGE OFFSETS ────────────────────────────────────
#define OFFSET_CMD1_MS             250   // Extra ms to drive forward at the end of Button 1 (speed unchanged at 60, so offset unchanged)
#define OFFSET_POST_RIGHT_TURN_MS  220   // Extra ms to nudge forward after RIGHT movement finishes (Button 2; also pre-turn in sequences) — scaled down from 290 since SPEED_POST_RIGHT_TURN went 45->60 (same PWM speed increase covers more distance per ms)
#define OFFSET_POST_LEFT_TURN_MS   200   // Extra ms to nudge forward after LEFT movement finishes (Button 3; also pre-turn in sequences) — scaled down from 300 since SPEED_POST_LEFT_TURN went 40->60
#define OFFSET_POST_180_MS         220   // Extra ms to nudge forward after 180-degree turn (Button 180; also pre-turn in sequences) — scaled down from 290 since SPEED_POST_180_TURN went 45->60
#define OFFSET_PRE_180_MS          110   // Extra ms to nudge forward on the leg immediately before a 180-degree turn in sequences — kept separate from the 90-degree pre-turn offsets above; scaled down since the legs feeding into it now also run at 60
#define OFFSET_STRAFE_L_MS         120   // Extra ms to keep strafing left after final line detection
#define OFFSET_STRAFE_R_MS         120   // Extra ms to keep strafing right after final line detection

#define TURN_BLIND_MS              150   // Blind turn duration to clear the starting line
#define TURN_CREEP_SPEED           40    // Slow speed used while centering on the target line
#define TURN_CENTER_DEBOUNCE_MS    30    // How long MID must stay HIGH before accepting the center
#define TURN_CENTER_TIMEOUT_MS     800   // Fallback: stop creeping after this long even if MID never centers

#define ROTATE180_REVERSE_SPEED    50    // Speed while reversing between the two 90-degree turns of a 180
#define ROTATE180_REVERSE_MS       200   // Duration of that reverse nudge

// ── Turn Configuration ────────────────────────────────────────
#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT

mecanumCar car(3, 2);

Servo gripper;
int gripperPosUs = GRIPPER_OPEN_US;

// ── Color classification table (ratio-based, TCS3200) ──────────
enum ColorID { COLOR_UNKNOWN = 0, COLOR_WHITE, COLOR_BLACK, COLOR_BLUE, COLOR_COUNT };
const ColorID TARGET_COLOR = COLOR_BLUE; // only this color triggers a grab before turning

struct ColorProfile {
  const char* name;
  float rRatioMin, rRatioMax;
  float gRatioMin, gRatioMax;
  float bRatioMin, bRatioMax;
};

ColorProfile colorProfiles[COLOR_COUNT] = {
  /* UNKNOWN */ { "UNKNOWN", 0.0, 1.0, 0.0, 1.0, 0.0, 1.0 },
  /* WHITE   */ { "WHITE",   0.0, 1.0, 0.0, 1.0, 0.0, 1.0 },
  /* BLACK   */ { "BLACK",   0.0, 1.0, 0.0, 1.0, 0.0, 1.0 },
  /* BLUE    */ { "BLUE",    0.00, 0.30, 0.00, 0.40, 0.40, 1.00 },
};

// ================================================================
//  HELPERS
// ================================================================
void setMotorSpeed(uint8_t spd) {
  speed_Upper_L = spd;
  speed_Lower_L = spd;
  speed_Upper_R = spd;
  speed_Lower_R = spd;
}

// Custom speed calculation mapping for strafing left
void setStrafeLeftCalibratedSpeed(uint8_t baseSpeed) {
  speed_Upper_L = constrain(baseSpeed + STRAFE_L_BIAS_UL, SPEED_MIN, 255);
  speed_Lower_L = constrain(baseSpeed + STRAFE_L_BIAS_LL, SPEED_MIN, 255);
  speed_Upper_R = constrain(baseSpeed + STRAFE_L_BIAS_UR, SPEED_MIN, 255);
  speed_Lower_R = constrain(baseSpeed + STRAFE_L_BIAS_LR, SPEED_MIN, 255);
}

// Custom speed calculation mapping for strafing right
void setStrafeRightCalibratedSpeed(uint8_t baseSpeed) {
  speed_Upper_L = constrain(baseSpeed + STRAFE_R_BIAS_UL, SPEED_MIN, 255);
  speed_Lower_L = constrain(baseSpeed + STRAFE_R_BIAS_LL, SPEED_MIN, 255);
  speed_Upper_R = constrain(baseSpeed + STRAFE_R_BIAS_UR, SPEED_MIN, 255);
  speed_Lower_R = constrain(baseSpeed + STRAFE_R_BIAS_LR, SPEED_MIN, 255);
}

void restoreIR() {
  car.Stop();
  IrReceiver.begin(RECV_PIN, false);
}

// Polls the IR receiver and always resumes it after a decode, so a stray
// non-stop command can't leave the receiver stuck and unable to see CMD_STAR later.
bool checkEstop() {
  if (IrReceiver.decode()) {
    bool isStop = (IrReceiver.decodedIRData.command == CMD_STAR);
    IrReceiver.resume();
    return isStop;
  }
  return false;
}

// ================================================================
//  COLOR SENSOR (TCS3200) / ULTRASONIC (HC-SR04) / GRIPPER
// ================================================================
// Moves the gripper gradually to targetUs (gentler current draw than a jump).
void gripperMoveTo(int targetUs) {
  targetUs = constrain(targetUs, GRIPPER_OPEN_US, GRIPPER_CLOSE_US);
  int step = (targetUs >= gripperPosUs) ? GRIPPER_MOVE_STEP : -GRIPPER_MOVE_STEP;
  while (gripperPosUs != targetUs) {
    gripperPosUs += step;
    if ((step > 0 && gripperPosUs > targetUs) || (step < 0 && gripperPosUs < targetUs)) gripperPosUs = targetUs;
    gripper.writeMicroseconds(gripperPosUs);
    delay(GRIPPER_MOVE_DT);
  }
}

void readColorRaw(uint16_t &rPulse, uint16_t &gPulse, uint16_t &bPulse) {
  digitalWrite(TCS_S2_PIN, LOW);  digitalWrite(TCS_S3_PIN, LOW);  delayMicroseconds(50);
  rPulse = pulseIn(TCS_OUT_PIN, HIGH, COLOR_PULSE_TIMEOUT_US);
  digitalWrite(TCS_S2_PIN, LOW);  digitalWrite(TCS_S3_PIN, HIGH); delayMicroseconds(50);
  bPulse = pulseIn(TCS_OUT_PIN, HIGH, COLOR_PULSE_TIMEOUT_US);
  digitalWrite(TCS_S2_PIN, HIGH); digitalWrite(TCS_S3_PIN, HIGH); delayMicroseconds(50);
  gPulse = pulseIn(TCS_OUT_PIN, HIGH, COLOR_PULSE_TIMEOUT_US);

  if (rPulse == 0) rPulse = COLOR_PULSE_TIMEOUT_US;
  if (gPulse == 0) gPulse = COLOR_PULSE_TIMEOUT_US;
  if (bPulse == 0) bPulse = COLOR_PULSE_TIMEOUT_US;
}

ColorID classifyColor() {
  uint16_t rPulse, gPulse, bPulse;
  readColorRaw(rPulse, gPulse, bPulse);

  float rIntensity = 1000000.0 / rPulse;
  float gIntensity = 1000000.0 / gPulse;
  float bIntensity = 1000000.0 / bPulse;
  float total = rIntensity + gIntensity + bIntensity;

  float rRatio = rIntensity / total;
  float gRatio = gIntensity / total;
  float bRatio = bIntensity / total;

  ColorID label = COLOR_UNKNOWN;
  if (total < BRIGHTNESS_BLACK_MAX) {
    label = COLOR_BLACK;
  } else {
    float maxRatio = max(rRatio, max(gRatio, bRatio));
    float minRatio = min(rRatio, min(gRatio, bRatio));
    if ((maxRatio - minRatio) < ACHROMATIC_TOLERANCE) {
      label = COLOR_WHITE;
    } else if (rRatio >= colorProfiles[COLOR_BLUE].rRatioMin && rRatio <= colorProfiles[COLOR_BLUE].rRatioMax &&
               gRatio >= colorProfiles[COLOR_BLUE].gRatioMin && gRatio <= colorProfiles[COLOR_BLUE].gRatioMax &&
               bRatio >= colorProfiles[COLOR_BLUE].bRatioMin && bRatio <= colorProfiles[COLOR_BLUE].bRatioMax) {
      label = COLOR_BLUE;
    }
  }

  Serial.print(F("[COLOR] ratio R/G/B: "));
  Serial.print(rRatio, 2); Serial.print('/'); Serial.print(gRatio, 2); Serial.print('/'); Serial.print(bRatio, 2);
  Serial.print(F(" -> ")); Serial.println(colorProfiles[label].name);
  return label;
}

float readUltrasonicDistanceCM() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) return -1.0;
  return (float)duration / 58.0;
}

// ================================================================
//  FAVORIOT TELEMETRY (via ESP32 WiFi bridge)
// ================================================================
// Name of the IR command currently executing, used to label telemetry
// emitted from inside the movement functions below (which don't otherwise
// know which top-level command triggered them).
const char* currentCommandName = "IDLE";

const char* cmdName(uint8_t cmd) {
  switch (cmd) {
    case CMD_STAR:     return "CMD_STAR";
    case CMD_1:        return "CMD_1";
    case CMD_2:        return "CMD_2";
    case CMD_3:        return "CMD_3";
    case CMD_ROTATE_R: return "CMD_ROTATE_R";
    case CMD_ROTATE_L: return "CMD_ROTATE_L";
    case CMD_UP:       return "CMD_UP";
    case CMD_180:      return "CMD_180";
    case CMD_7:        return "CMD_7";
    case CMD_180_ONLY: return "CMD_180_ONLY";
    case CMD_9:        return "CMD_9";
    case CMD_SEQ1:     return "CMD_SEQ1";
    case CMD_SEQ2:     return "CMD_SEQ2";
    case CMD_HASH:     return "CMD_HASH";
    default:           return "UNKNOWN";
  }
}

// Inverse of cmdName(), used to map a remote command name (received from the
// ESP32 bridge, originating from a Favoriot dashboard Control widget) back
// to the IR command byte so it can be dispatched identically to a real
// remote-control press. Returns 0x00 (unused by any real command) if the
// name isn't recognized.
uint8_t cmdFromName(const String& name) {
  if (name == "CMD_STAR")     return CMD_STAR;
  if (name == "CMD_1")        return CMD_1;
  if (name == "CMD_2")        return CMD_2;
  if (name == "CMD_3")        return CMD_3;
  if (name == "CMD_ROTATE_R") return CMD_ROTATE_R;
  if (name == "CMD_ROTATE_L") return CMD_ROTATE_L;
  if (name == "CMD_UP")       return CMD_UP;
  if (name == "CMD_180")      return CMD_180;
  if (name == "CMD_7")        return CMD_7;
  if (name == "CMD_180_ONLY") return CMD_180_ONLY;
  if (name == "CMD_9")        return CMD_9;
  if (name == "CMD_SEQ1")     return CMD_SEQ1;
  if (name == "CMD_SEQ2")     return CMD_SEQ2;
  if (name == "CMD_HASH")     return CMD_HASH;
  return 0x00;
}

// Sends one JSON telemetry line to the ESP32 bridge, which wraps it as the
// "data" object of a Favoriot stream POST. Called only on state changes
// (command start/finish, block progress, e-stop) rather than continuously,
// since SoftwareSerial briefly disables interrupts while transmitting and
// frequent calls could interfere with IR decode timing.
void sendTelemetry(const char* status, int blockCount) {
  espSerial.print(F("{\"command\":\""));
  espSerial.print(currentCommandName);
  espSerial.print(F("\",\"status\":\""));
  espSerial.print(status);
  espSerial.print(F("\",\"sensor_left\":"));
  espSerial.print(digitalRead(SENSOR_LEFT));
  espSerial.print(F(",\"sensor_mid\":"));
  espSerial.print(digitalRead(SENSOR_MID));
  espSerial.print(F(",\"sensor_right\":"));
  espSerial.print(digitalRead(SENSOR_RIGHT));
  espSerial.print(F(",\"block_count\":"));
  espSerial.print(blockCount);
  espSerial.println(F("}"));
}

// Reports the ultrasonic's live block-detection reading, independent of
// gripper state, so the dashboard always shows whether a block is in front
// of the car right now (not just whether the last grab attempt happened).
void sendBlockDetectionTelemetry(bool blockDetected) {
  espSerial.print(F("{\"command\":\""));
  espSerial.print(currentCommandName);
  espSerial.print(F("\",\"block_detected\":"));
  espSerial.print(blockDetected ? F("true") : F("false"));
  espSerial.println(F("}"));
}

void reportEstop() {
  Serial.println(F("E-STOP!"));
  restoreIR();
  sendTelemetry("estop", -1);
}

// Creeps forward on the ultrasonic reading until within grab range (or times
// out), then closes the gripper. Grabs whatever is there regardless of
// color. Called only from grabBlockInFront() below.
void approachAndGrab() {
  Serial.println(F(">>> Approaching to grab <<<"));
  car.Stop();
  delay(150);

  unsigned long approachStart = millis();
  bool reachedTarget = false;
  int noEchoStreak = 0;

  while (millis() - approachStart < APPROACH_TIMEOUT_MS) {
    if (checkEstop()) {
      reportEstop();
      return;
    }

    float distanceCm = readUltrasonicDistanceCM();
    Serial.print(F("[APPROACH] dist: "));
    if (distanceCm < 0) Serial.println(F("no echo")); else { Serial.print(distanceCm); Serial.println(F("cm")); }

    if (distanceCm > 0 && distanceCm <= APPROACH_STOP_DISTANCE_CM) {
      Serial.print(F("[APPROACH] Reached ")); Serial.print(distanceCm); Serial.println(F("cm - grabbing."));
      reachedTarget = true;
      break;
    }

    if (distanceCm < 0) {
      // No echo: either nothing is there yet, or (much more likely once
      // we've been advancing) the block is now inside the HC-SR04's blind
      // zone (~2-5cm) where it physically can't get a valid echo back.
      // Don't blindly keep driving on missing data — that's what was
      // causing the car to crash into the block instead of stopping.
      // A few consecutive no-echo reads after we've started closing in
      // is treated as "too close to read" == close enough to grab.
      noEchoStreak++;
      if (noEchoStreak >= NO_ECHO_STREAK_TO_GRAB) {
        Serial.println(F("[APPROACH] Repeated no-echo (likely inside blind zone) - grabbing."));
        reachedTarget = true;
        break;
      }
      delay(60); // hold position and retry, instead of advancing on bad data
      continue;
    }
    noEchoStreak = 0;

    setMotorSpeed(SPEED_MIN); car.Advance(); delay(60);
  }

  car.Stop();
  delay(APPROACH_SETTLE_MS);

  if (reachedTarget) {
    gripperMoveTo(GRIPPER_GRAB_US);
    Serial.println(F("[GRIPPER] Grabbed."));
    delay(APPROACH_SETTLE_MS);
  } else {
    Serial.println(F("[APPROACH] Timeout - never reached range, no grab."));
  }

  restoreIR();
}

// Approaches and grabs whatever is in front, regardless of color, before the
// caller proceeds to a 180-degree turn. The color reading is logged for
// diagnostics only — it no longer gates whether the grab happens.
void grabBlockInFront() {
  Serial.println(F("[GRAB] Reading color (diagnostic only, no longer gates the grab)..."));
  classifyColor();
  approachAndGrab();
}

// Polled from loop() only while idle (movement functions are blocking, so
// this naturally never runs mid-command). Every BACKGROUND_GRAB_CHECK_MS,
// checks the ultrasonic; if something is already within grab range and the
// gripper isn't already holding something, grabs it — no button press
// needed. Scoped to "already close" rather than "detected at any range" so
// the car doesn't autonomously drive across the room toward something it
// merely sees; it only reacts to a block already right in front of it.
// Fires once per grab: after closing on a block, gripperPosUs != GRIPPER_OPEN_US
// suppresses further detection until the '#' button (CMD_HASH) reopens the
// gripper via gripperMoveTo(GRIPPER_OPEN_US).
void checkBackgroundGrab() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < BACKGROUND_GRAB_CHECK_MS) return;
  lastCheck = millis();

  float distanceCm = readUltrasonicDistanceCM();
  bool blockSeen = (distanceCm > 0 && distanceCm <= APPROACH_STOP_DISTANCE_CM);

  // Always logged (even when the gripper is already holding something and
  // no grab will be attempted) so the ultrasonic's live reading is visible
  // on Serial for debugging sensor behavior/placement.
  Serial.print(F("[BACKGROUND] dist: "));
  if (distanceCm < 0) Serial.print(F("no echo")); else { Serial.print(distanceCm); Serial.print(F("cm")); }
  Serial.print(F(" -> block "));
  Serial.println(blockSeen ? F("DETECTED") : F("not detected"));

  sendBlockDetectionTelemetry(blockSeen);

  if (gripperPosUs != GRIPPER_OPEN_US) return; // already holding something

  if (blockSeen) {
    Serial.println(F("[BACKGROUND] Block detected while idle -> grabbing."));
    grabBlockInFront();
  }
}

// ================================================================
//  ROTATION FUNCTIONS
// ================================================================
// After the edge sensor first sees the target line, creep slowly and wait
// for SENSOR_MID to also read HIGH (debounced) so the car stops centered on
// the line rather than the instant the edge sensor alone triggers. Falls
// back to a timeout so a bad/dirty sensor can't hang the turn forever.
// turnRight: true = keep turning right while centering, false = keep turning left.
bool centerOnLine(bool turnRight) {
  unsigned long centerStart = millis();
  unsigned long midHighSince = 0;

  while (true) {
    if (checkEstop()) {
      reportEstop(); return false;
    }

    setMotorSpeed(TURN_CREEP_SPEED);
    if (turnRight) car.Turn_Right(); else car.Turn_Left();

    if (digitalRead(SENSOR_MID) == HIGH) {
      if (midHighSince == 0) midHighSince = millis();
      if (millis() - midHighSince >= TURN_CENTER_DEBOUNCE_MS) break;
    } else {
      midHighSince = 0;
    }

    if (millis() - centerStart >= TURN_CENTER_TIMEOUT_MS) {
      Serial.println(F("Center timeout - stopping at edge-sensor position"));
      break;
    }
  }
  return true;
}

bool rotateRight90() {
  Serial.println(F("\n--- Rotating 90 Degrees Right (Speed 60) ---"));
  // If already parked on a line (e.g. chained from a previous rotation), spin off it first
  // so the blind period + detection below finds the NEXT line, not the current one.
  while (digitalRead(STOP_SENSOR_RIGHT_TURN) == HIGH) {
    if (checkEstop()) {
       reportEstop(); return false;
    }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();
  }
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEstop()) {
       reportEstop(); return false;
    }
    setMotorSpeed(SPEED_ROTATE); 
    car.Turn_Right();          
  }
  while (true) {
    if (checkEstop()) {
       reportEstop(); return false;
    }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();
    if (digitalRead(STOP_SENSOR_RIGHT_TURN) == HIGH) {
      break;
    }
  }
  if (!centerOnLine(true)) return false;
  car.Stop();
  delay(200);
  return true;
}

bool rotateLeft90() {
  Serial.println(F("\n--- Rotating 90 Degrees Left (Speed 60) ---"));
  // If already parked on a line (e.g. chained from a previous rotation), spin off it first
  // so the blind period + detection below finds the NEXT line, not the current one.
  while (digitalRead(STOP_SENSOR_LEFT_TURN) == HIGH) {
    if (checkEstop()) {
       reportEstop(); return false;
    }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();
  }
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEstop()) {
       reportEstop(); return false;
    }
    setMotorSpeed(SPEED_ROTATE); 
    car.Turn_Left();
  }
  while (true) {
    if (checkEstop()) {
       reportEstop(); return false;
    }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();
    if (digitalRead(STOP_SENSOR_LEFT_TURN) == HIGH) {
      break;
    }
  }
  if (!centerOnLine(false)) return false;
  car.Stop();
  delay(200);
  return true;
}

bool rotate180() {
  if (!rotateRight90()) return false;

  unsigned long t_start = millis();
  while (millis() - t_start < ROTATE180_REVERSE_MS) {
    if (checkEstop()) {
      reportEstop(); return false;
    }
    setMotorSpeed(ROTATE180_REVERSE_SPEED);
    car.Back();
  }
  car.Stop();
  delay(100);

  return rotateRight90();
}


// ================================================================
//  LINE TRACKING FORWARD MOVEMENTS
// ================================================================
int moveForwardBlocks(int targetBlocks, uint8_t startSpeed, int endOffsetMs) {
  int junctionCount = 0;
  bool onJunction = true;

  while (junctionCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    int calculatedSpeed = startSpeed - (junctionCount * 5);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == HIGH && M == HIGH && R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        onJunction = true;
        if (junctionCount == targetBlocks) break;
      }
      setMotorSpeed(currentSpeed); car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW) {
      onJunction = false; setMotorSpeed(currentSpeed); car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH) {
      onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW) {
      onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW) {
      onJunction = false; setMotorSpeed(currentSpeed); car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW) {
      onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH) {
      onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right();
    }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) break;
      IrReceiver.resume();
    }
  }

  if (junctionCount == targetBlocks && endOffsetMs > 0) {
    setMotorSpeed(SPEED_MIN); car.Advance(); delay(endOffsetMs);
  }
  restoreIR();
  return junctionCount;
}

// ================================================================
//  STRAFE LEFT MOVEMENT (Counts via SENSOR_LEFT Only)
// ================================================================
int strafeLeftBlocks(int targetBlocks, uint8_t startSpeed, int endOffsetMs) {
  int junctionCount = 0;
  bool onJunction = true;

  Serial.print(F("\n--- Strafing Left (Left Sensor Count) ---"));

  while (junctionCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);

    int calculatedSpeed = startSpeed - (junctionCount * 5);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == HIGH) {
      if (!onJunction) {
        junctionCount++;
        Serial.print(F("Left Sensor Hit! Total: ")); Serial.println(junctionCount);
        onJunction = true;
        if (junctionCount == targetBlocks) break;
      }
    } else {
      onJunction = false;
    }

    setStrafeLeftCalibratedSpeed(currentSpeed);
    car.L_Move(); // Execute sideways left motion

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) break;
      IrReceiver.resume();
    }
  }

  if (junctionCount == targetBlocks && endOffsetMs > 0) {
    setStrafeLeftCalibratedSpeed(SPEED_MIN);
    car.L_Move();
    delay(endOffsetMs);
  }
  restoreIR();
  return junctionCount;
}

// ================================================================
//  STRAFE RIGHT MOVEMENT (Counts via SENSOR_RIGHT Only)
// ================================================================
int strafeRightBlocks(int targetBlocks, uint8_t startSpeed, int endOffsetMs) {
  int junctionCount = 0;
  bool onJunction = true;

  Serial.print(F("\n--- Strafing Right (Right Sensor Count) ---"));

  while (junctionCount < targetBlocks) {
    uint8_t R = digitalRead(SENSOR_RIGHT);

    int calculatedSpeed = startSpeed - (junctionCount * 5);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        Serial.print(F("Right Sensor Hit! Total: ")); Serial.println(junctionCount);
        onJunction = true;
        if (junctionCount == targetBlocks) break;
      }
    } else {
      onJunction = false;
    }

    setStrafeRightCalibratedSpeed(currentSpeed);
    car.R_Move(); // Execute sideways right motion

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) break;
      IrReceiver.resume();
    }
  }

  if (junctionCount == targetBlocks && endOffsetMs > 0) {
    setStrafeRightCalibratedSpeed(SPEED_MIN);
    car.R_Move();
    delay(endOffsetMs);
  }
  restoreIR();
  return junctionCount;
}

// ================================================================
//  SEQUENCES
// ================================================================
// Button 4: forward 4, turn left, forward 2, turn right, forward 2, turn 180.
// Aborts early (leaving the car stopped) if a turn is interrupted by E-STOP.
void runSequence1() {
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_A, SPEED_START_FAST, OFFSET_CMD1_MS);
  delay(1000);
  if (!rotateLeft90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_B, SPEED_POST_LEFT_TURN, OFFSET_POST_LEFT_TURN_MS);
  delay(1000);
  if (!rotateRight90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_C, SPEED_POST_RIGHT_TURN, OFFSET_PRE_180_MS);
  delay(1000);
  rotate180();
}

// Button 5: runs Sequence 1, then retraces the same path mirrored/reversed
// so the car ends back at its original location and heading.
// Aborts early (leaving the car stopped) if a turn is interrupted by E-STOP.
void runSequence2() {
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_A, SPEED_START_FAST, OFFSET_CMD1_MS);
  delay(1000);
  if (!rotateLeft90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_B, SPEED_POST_LEFT_TURN, OFFSET_POST_LEFT_TURN_MS);
  delay(1000);
  if (!rotateRight90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_C, SPEED_POST_RIGHT_TURN, OFFSET_PRE_180_MS);
  delay(1000);
  grabBlockInFront();
  if (!rotate180()) return;
  delay(1000);
  // Reverse leg: retrace forward 2 -> left -> forward 2 -> right -> forward 5, back to start.
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_C, SPEED_POST_180_TURN, OFFSET_POST_180_MS);
  delay(1000);
  if (!rotateLeft90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_B, SPEED_POST_LEFT_TURN, OFFSET_POST_LEFT_TURN_MS);
  delay(1000);
  if (!rotateRight90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ2_FINAL, SPEED_POST_RIGHT_TURN, OFFSET_POST_RIGHT_TURN_MS);
}

// Button UP: forward 5, turn 180, forward 5.
// Aborts early (leaving the car stopped) if the turn is interrupted by E-STOP.
void runSequence3() {
  moveForwardBlocks(TARGET_BLOCKS_SEQ3, SPEED_START_FAST, OFFSET_PRE_180_MS);
  delay(1000);
  grabBlockInFront();
  if (!rotate180()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ3, SPEED_POST_180_TURN, OFFSET_POST_180_MS);
}

// Button ROTATE_R (arrow): same path as Sequence 2 (Button 5), but mirrored --
// every left turn becomes a right turn and vice versa.
// Aborts early (leaving the car stopped) if a turn is interrupted by E-STOP.
void runSequence4() {
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_A, SPEED_START_FAST, OFFSET_CMD1_MS);
  delay(1000);
  if (!rotateRight90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_B, SPEED_POST_RIGHT_TURN, OFFSET_POST_RIGHT_TURN_MS);
  delay(1000);
  if (!rotateLeft90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_C, SPEED_POST_LEFT_TURN, OFFSET_PRE_180_MS);
  delay(1000);
  grabBlockInFront();
  if (!rotate180()) return;
  delay(1000);
  // Reverse leg: retrace forward 2 -> right -> forward 2 -> left -> forward 5, back to start.
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_C, SPEED_POST_180_TURN, OFFSET_POST_180_MS);
  delay(1000);
  if (!rotateRight90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_B, SPEED_POST_RIGHT_TURN, OFFSET_POST_RIGHT_TURN_MS);
  delay(1000);
  if (!rotateLeft90()) return;
  delay(1000);
  moveForwardBlocks(TARGET_BLOCKS_SEQ2_FINAL, SPEED_POST_LEFT_TURN, OFFSET_POST_LEFT_TURN_MS);
}

// ================================================================
//  COMMAND DISPATCH (shared by IR remote and Favoriot dashboard control)
// ================================================================
void dispatchCommand(uint8_t cmd) {
  currentCommandName = cmdName(cmd);
  sendTelemetry("running", 0);

  int finalBlockCount = -1;

  if (cmd == CMD_1) {
    finalBlockCount = moveForwardBlocks(TARGET_BLOCKS_CMD1, SPEED_START_FAST, OFFSET_CMD1_MS);
  }
  else if (cmd == CMD_2) {
    if (rotateRight90()) {
      finalBlockCount = moveForwardBlocks(TARGET_BLOCKS_CMD2, SPEED_POST_RIGHT_TURN, OFFSET_POST_RIGHT_TURN_MS);
    }
  }
  else if (cmd == CMD_3) {
    if (rotateLeft90()) {
      finalBlockCount = moveForwardBlocks(TARGET_BLOCKS_CMD3, SPEED_POST_LEFT_TURN, OFFSET_POST_LEFT_TURN_MS);
    }
  }
  else if (cmd == CMD_ROTATE_R) {
    runSequence4();
  }
  else if (cmd == CMD_ROTATE_L) {
    runSequence2();
  }
  else if (cmd == CMD_180) {
    if (rotate180()) {
      finalBlockCount = moveForwardBlocks(TARGET_BLOCKS_CMD180, SPEED_POST_180_TURN, OFFSET_POST_180_MS);
    }
  }
  else if (cmd == CMD_180_ONLY) {
    rotate180();
  }
  else if (cmd == CMD_7) {
    finalBlockCount = strafeLeftBlocks(TARGET_STRAFE_CMD7, SPEED_STRAFE_LEFT, OFFSET_STRAFE_L_MS);
  }
  else if (cmd == CMD_9) {
    finalBlockCount = strafeRightBlocks(TARGET_STRAFE_CMD9, SPEED_STRAFE_RIGHT, OFFSET_STRAFE_R_MS);
  }
  else if (cmd == CMD_STAR) {
    car.Stop();
    Serial.println(F("E-STOP (idle)!"));
    sendTelemetry("estop", -1);
  }
  else if (cmd == CMD_SEQ1) {
    runSequence1();
  }
  else if (cmd == CMD_SEQ2) {
    runSequence2();
  }
  else if (cmd == CMD_UP) {
    runSequence3();
  }
  else if (cmd == CMD_HASH) {
    gripperMoveTo(GRIPPER_OPEN_US);
    Serial.println(F("[GRIPPER] Released (opened)."));
  }

  if (cmd != CMD_STAR) sendTelemetry("idle", finalBlockCount);
  currentCommandName = "IDLE";
}

// Polls the ESP32 link for a remote command name (e.g. "CMD_1"), forwarded
// from a Favoriot dashboard Control widget the ESP32 picked up by polling
// Favoriot's REST API (see esp32-wifi-bridge/). Dispatches it exactly like
// a real IR remote press.
void checkRemoteCommand() {
  if (!espSerial.available()) return;
  String line = espSerial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  uint8_t cmd = cmdFromName(line);
  if (cmd == 0x00) {
    Serial.print(F("Unknown remote command: "));
    Serial.println(line);
    return;
  }

  Serial.print(F("Remote cmd received: "));
  Serial.println(line);
  dispatchCommand(cmd);
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);
  pinMode(SENSOR_LEFT,  INPUT);
  pinMode(SENSOR_MID,   INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  pinMode(TCS_S2_PIN, OUTPUT);
  pinMode(TCS_S3_PIN, OUTPUT);
  pinMode(TCS_OUT_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  gripper.attach(GRIPPER_PIN, GRIPPER_OPEN_US, GRIPPER_CLOSE_US);
  gripper.writeMicroseconds(GRIPPER_OPEN_US);
  gripperPosUs = GRIPPER_OPEN_US;

  Serial.println(F("Ready."));
}

// Set to true to continuously print SENSOR_LEFT vs SENSOR_RIGHT readings
// (once every 200ms) so you can compare how reliably each triggers HIGH
// while sliding the car by hand over a line. Leave false for normal operation.
#define SENSOR_DIAGNOSTIC_MODE  false

void loop() {
#if SENSOR_DIAGNOSTIC_MODE
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print(F("L="));
    Serial.print(digitalRead(SENSOR_LEFT));
    Serial.print(F("  M="));
    Serial.print(digitalRead(SENSOR_MID));
    Serial.print(F("  R="));
    Serial.println(digitalRead(SENSOR_RIGHT));
  }
#endif

  checkRemoteCommand();
  checkBackgroundGrab();

  if (!IrReceiver.decode()) return;

  uint8_t cmd = IrReceiver.decodedIRData.command;

  if (cmd == 0x00 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    IrReceiver.resume();
    return;
  }

  Serial.print(F("IR cmd received: 0x"));
  Serial.println(cmd, HEX);

  dispatchCommand(cmd);

  IrReceiver.resume();
}