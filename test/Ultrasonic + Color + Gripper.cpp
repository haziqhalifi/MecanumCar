/*
  ================================================================
  Vroom — Sequence 1 (fwd5 -> read BLUE -> approach -> grab) + Button 8
  ================================================================
  Sequence 1 (Button 1):
    1. Line-follow forward 5 blocks, then STOP.
    2. Read the color sensor. If it's the TARGET (BLUE)...
    3. ...slowly approach on the ultrasonic until <= APPROACH_STOP_DISTANCE_CM,
    4. ...then close the gripper to GRIPPER_GRAB_US (1100).
  Otherwise (not the target color): stop, no grab.

  NOTE: if the servo makes the board freeze / serial stop, that is a POWER
  brown-out — give the servo its own 5-6V supply with a common ground; the
  Arduino 5V pin can't source a servo's stall current.
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>

#define IR_RECEIVE_PIN IR_PIN
#define DECODE_NEC
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>
#include <Servo.h>

// ── IR codes ──────────────────────────────────────────────────
#define BTN_1     0xFF6897   // Sequence 1: fwd5 -> read BLUE -> approach -> grab
#define BTN_8     0xFF38C7   // Wait for YELLOW @ <= 4.5cm, then slow grab
#define BTN_0     0xFF4AB5   // e-stop
#define BTN_OK    0xFF02FD   // toggle continuous color print
#define BTN_STAR  0xFF42BD   // '*' close gripper by one step
#define BTN_HASH  0xFF52AD   // '#' open  gripper by one step

// ── Hardware pins ─────────────────────────────────────────────
#define IR_PIN        A3
#define SensorLeft    A0
#define SensorMiddle  A1
#define SensorRight   A2
#define GRIPPER_PIN   9

// HC-SR04 ultrasonic
#define ULTRASONIC_TRIG_PIN   12
#define ULTRASONIC_ECHO_PIN   13
#define ULTRASONIC_TIMEOUT_US 30000UL

// TCS3200 color sensor
#define TCS_OUT_PIN   8
#define TCS_S2_PIN    7
#define TCS_S3_PIN    6

// ── Gripper servo pulse widths (us) ──────────────────────────
#define GRIPPER_OPEN_US   500
#define GRIPPER_CLOSE_US  1300   // hard max (stay within the servo's free range)
#define GRIPPER_GRAB_US   1100   // grab position for Sequence 1
#define GRIPPER_STEP_US   100    // Button 8 stepped-grab increment
#define GRIPPER_MANUAL_STEP_US 50 // * / # manual nudge per press
#define GRIPPER_MOVE_STEP 10     // us per micro-step when moving gradually
#define GRIPPER_MOVE_DT   15     // ms between micro-steps
#define GRIPPER_GRAB_STEP_DT_MS 250 // ms pause between Button 8 grab steps

// ── Sequence 1 target color ──────────────────────────────────
#define TARGET_COLOR   COLOR_BLUE

// ── Button 8 trigger (explicit, so the target is easy to confirm) ──
#define BTN8_TARGET_COLOR      COLOR_YELLOW
#define BTN8_GRAB_DISTANCE_CM  4.5

// ── Line polarity ─────────────────────────────────────────────
#define BLACK_LINE HIGH
#define WHITE_LINE LOW    // FIX: replaces the fragile "== !BLACK_LINE" (which was LOW anyway)

// ── Speeds & line-follow timing ──────────────────────────────
#define SPEED_START_FAST          40
#define SPEED_MIN                 30
#define TURN_CORRECTION_BOOST     10
#define SPEED_DECAY_PER_JUNCTION   5
#define CENTER_OFFSET_MS          210
#define LINE_DEBOUNCE_MS          180

// ── Approach & grab ──────────────────────────────────────────
#define APPROACH_SPEED             SPEED_MIN
#define APPROACH_STOP_DISTANCE_CM  4.0     // stop-and-grab distance (accounts for gripper reach)
#define APPROACH_SETTLE_MS         300
#define APPROACH_TIMEOUT_MS        8000

// ── Color classification tuning (ratio-based) ────────────────
#define COLOR_PULSE_TIMEOUT_US  25000UL
#define BRIGHTNESS_BLACK_MAX    300.0
#define ACHROMATIC_TOLERANCE    0.08

// ── EXTERN LIBRARY VARIABLES ──────────────────────────────────
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

mecanumCar car(3, 2);
Servo gripper;

int gripperPosUs = GRIPPER_OPEN_US;

// ================================================================
//  COLOR TABLE
// ================================================================
enum ColorID { COLOR_UNKNOWN = 0, COLOR_WHITE, COLOR_BLACK, COLOR_YELLOW, COLOR_BLUE, COLOR_COUNT };

struct ColorProfile {
  const char* name;
  float rRatioMin, rRatioMax;
  float gRatioMin, gRatioMax;
  float bRatioMin, bRatioMax;
  bool  isGrabTarget;
};

ColorProfile colorProfiles[COLOR_COUNT] = {
  /* UNKNOWN */ { "UNKNOWN", 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, false },
  /* WHITE   */ { "WHITE",   0.0, 1.0, 0.0, 1.0, 0.0, 1.0, false },
  /* BLACK   */ { "BLACK",   0.0, 1.0, 0.0, 1.0, 0.0, 1.0, false },
  /* YELLOW  */ { "YELLOW",  0.35, 0.60, 0.35, 0.60, 0.00, 0.25, false },  // TUNE: verify against [COLOR] print — prior hardware readings put yellow's gRatio ~0.34-0.35, right at the 0.35 min, so it can read UNKNOWN until this window is widened
  /* BLUE    */ { "BLUE",    0.00, 0.30, 0.00, 0.40, 0.40, 1.00, true  }  // <-- Seq1 grab target (reachable: blue reads r~0.24 g~0.29 b~0.47)
};

bool g_colorMonitor = false;   // OK toggles continuous color printing

// ================================================================
//  HELPERS
// ================================================================
void setMotorSpeed(uint8_t spd) {
  speed_Upper_L = spd; speed_Lower_L = spd;
  speed_Upper_R = spd; speed_Lower_R = spd;
}

void restoreIR() {
  car.Stop();
  while (IrReceiver.decode()) IrReceiver.resume();
}

// Move the gripper GRADUALLY to targetUs (gentler current draw than a jump).
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

// Close the gripper in GRIPPER_STEP_US (100us) steps, GRIPPER_GRAB_STEP_DT_MS
// (250ms) apart, logging each step. Used ONLY by the Button 8 grab — deliberately
// slower than the shared gripperMoveTo() so the close is easy to watch/abort and
// spreads the current draw over time.
void gripperGrabStepped(int targetUs) {
  targetUs = constrain(targetUs, GRIPPER_OPEN_US, GRIPPER_CLOSE_US);
  while (gripperPosUs < targetUs) {
    gripperPosUs += GRIPPER_STEP_US;
    if (gripperPosUs > targetUs) gripperPosUs = targetUs;
    gripper.writeMicroseconds(gripperPosUs);
    Serial.print(F("[BTN8 GRAB] step -> ")); Serial.print(gripperPosUs); Serial.println(F("us"));
    delay(GRIPPER_GRAB_STEP_DT_MS);
  }
}

void printDistanceAndColor(const __FlashStringHelper* tag, float distanceCm, ColorID detectedColor) {
  Serial.print(tag);
  Serial.print(F(" dist: "));
  if (distanceCm < 0) Serial.print(F("no echo"));
  else { Serial.print(distanceCm); Serial.print(F("cm")); }
  Serial.print(F(" color: "));
  Serial.println(colorProfiles[detectedColor].name);
}

void motionTelemetryTick(unsigned long &lastTelemetryMillis) {
  if (millis() - lastTelemetryMillis < 250) return;
  float distanceCm = readUltrasonicDistanceCM();
  ColorID detectedColor = classifyColor();
  printDistanceAndColor(F("[SEQ]"), distanceCm, detectedColor);
  lastTelemetryMillis = millis();
}

// ================================================================
//  TCS3200 COLOR SENSOR
// ================================================================
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

  if (g_colorMonitor) {
    Serial.print(F("[COLOR] raw us R/G/B: ")); Serial.print(rPulse); Serial.print('/'); Serial.print(gPulse); Serial.print('/'); Serial.print(bPulse);
    Serial.print(F("  ratio: ")); Serial.print(rRatio, 2); Serial.print('/'); Serial.print(gRatio, 2); Serial.print('/'); Serial.print(bRatio, 2);
  }

  ColorID label = COLOR_UNKNOWN;
  if (total < BRIGHTNESS_BLACK_MAX) {
    label = COLOR_BLACK;
  } else {
    float maxRatio = max(rRatio, max(gRatio, bRatio));
    float minRatio = min(rRatio, min(gRatio, bRatio));
    if ((maxRatio - minRatio) < ACHROMATIC_TOLERANCE) {
      label = COLOR_WHITE;
    } else {
      for (int i = COLOR_YELLOW; i < COLOR_COUNT; i++) {
        ColorProfile &p = colorProfiles[i];
        if (rRatio >= p.rRatioMin && rRatio <= p.rRatioMax &&
            gRatio >= p.gRatioMin && gRatio <= p.gRatioMax &&
            bRatio >= p.bRatioMin && bRatio <= p.bRatioMax) { label = (ColorID)i; break; }
      }
    }
  }

  if (g_colorMonitor) { Serial.print(F(" -> ")); Serial.println(colorProfiles[label].name); }
  return label;
}

// ================================================================
//  HC-SR04 ULTRASONIC SENSOR
// ================================================================
// NOTE: sleepUltrasonic() and wakeUltrasonic() are IDENTICAL — both only hold
// TRIG LOW. The HC-SR04 pings solely when readUltrasonicDistanceCM() toggles
// TRIG, so "sleep"/"wake" are effectively no-ops kept for readability, not real
// power states. (Left as-is deliberately; not worth an interrupt rewrite.)
void sleepUltrasonic() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); digitalWrite(ULTRASONIC_TRIG_PIN, LOW); pinMode(ULTRASONIC_ECHO_PIN, INPUT);
}
void wakeUltrasonic() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); digitalWrite(ULTRASONIC_TRIG_PIN, LOW); pinMode(ULTRASONIC_ECHO_PIN, INPUT);
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
//  APPROACH (ultrasonic creep to distance) — no color check here;
//  the caller has already confirmed the target color.
// ================================================================
void approachAndGrab() {
  Serial.println(F(">>> Approaching to grab <<<"));
  car.Stop(); delay(150); wakeUltrasonic();

  unsigned long approachStart = millis();
  bool reachedTarget = false;
  unsigned long telemetryTickMillis = millis();

  while (millis() - approachStart < APPROACH_TIMEOUT_MS) {
    if (IrReceiver.decode()) {
      uint32_t code = IrReceiver.decodedIRData.decodedRawData;
      if (code == BTN_0) { Serial.println(F("E-STOP!")); car.Stop(); sleepUltrasonic(); restoreIR(); return; }
      IrReceiver.resume();
    }

    float distanceCm = readUltrasonicDistanceCM();
    ColorID detectedColor = classifyColor();
    printDistanceAndColor(F("[APPROACH]"), distanceCm, detectedColor);

    if (distanceCm > 0 && distanceCm <= APPROACH_STOP_DISTANCE_CM) {
      Serial.print(F("[APPROACH] Reached ")); Serial.print(distanceCm); Serial.println(F("cm - grabbing."));
      reachedTarget = true;
      break;
    }

    setMotorSpeed(APPROACH_SPEED); car.Advance(); delay(60);
    motionTelemetryTick(telemetryTickMillis);
  }

  car.Stop(); delay(APPROACH_SETTLE_MS);

  if (reachedTarget) {
    gripperMoveTo(GRIPPER_GRAB_US);   // gradual close to 1100
    Serial.println(F("[GRIPPER] Grabbed (1100us)."));
    delay(APPROACH_SETTLE_MS);
  } else {
    Serial.println(F("[APPROACH] Timeout - never reached range, no grab."));
  }

  sleepUltrasonic(); restoreIR();
}

// ================================================================
//  FORWARD LINE-FOLLOW (plain — no color polling during the drive)
// ================================================================
void moveForwardBlocks(int targetBlocks, uint8_t startSpeed) {
  int junctionCount = 0; bool onJunction = true; unsigned long lastJunctionTime = 0;
  Serial.print(F("\n--- Forward ")); Serial.print(targetBlocks); Serial.println(F(" blocks ---"));
  unsigned long telemetryTickMillis = millis();

  while (junctionCount < targetBlocks) {
    uint8_t L = digitalRead(SensorLeft); uint8_t M = digitalRead(SensorMiddle); uint8_t R = digitalRead(SensorRight);
    int calculatedSpeed = max((int)SPEED_MIN, startSpeed - (junctionCount * SPEED_DECAY_PER_JUNCTION));
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == BLACK_LINE && M == BLACK_LINE && R == BLACK_LINE) {
    motionTelemetryTick(telemetryTickMillis);
      if (!onJunction) {
        if (millis() - lastJunctionTime >= LINE_DEBOUNCE_MS) {
          junctionCount++; lastJunctionTime = millis();
          Serial.print(F("Junction: ")); Serial.println(junctionCount);
          if (junctionCount == targetBlocks) break;
        }
        onJunction = true;
      }
      setMotorSpeed(currentSpeed); car.Advance();
    }
    else if (L == WHITE_LINE && M == BLACK_LINE && R == WHITE_LINE) { onJunction = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == WHITE_LINE && M == WHITE_LINE && R == BLACK_LINE) { onJunction = false; setMotorSpeed(currentSpeed + TURN_CORRECTION_BOOST); car.Turn_Right(); }
    else if (L == BLACK_LINE && M == WHITE_LINE && R == WHITE_LINE) { onJunction = false; setMotorSpeed(currentSpeed + TURN_CORRECTION_BOOST); car.Turn_Left(); }
    else if (L == WHITE_LINE && M == WHITE_LINE && R == WHITE_LINE) { onJunction = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == BLACK_LINE && M == BLACK_LINE && R == WHITE_LINE) { onJunction = false; setMotorSpeed(currentSpeed + TURN_CORRECTION_BOOST); car.Turn_Left(); }
    else if (L == WHITE_LINE && M == BLACK_LINE && R == BLACK_LINE) { onJunction = false; setMotorSpeed(currentSpeed + TURN_CORRECTION_BOOST); car.Turn_Right(); }

    if (IrReceiver.decode()) { if (IrReceiver.decodedIRData.decodedRawData == BTN_0) { Serial.println(F("E-STOP!")); break; } IrReceiver.resume(); }
  }

  if (junctionCount == targetBlocks && CENTER_OFFSET_MS > 0) { setMotorSpeed(SPEED_MIN); car.Advance(); delay(CENTER_OFFSET_MS); }
  car.Stop(); restoreIR();
}

// ================================================================
//  SEQUENCE 1 (Button 1) — fwd5 -> read color -> if BLUE -> approach+grab
// ================================================================
void runSequence1() {
  Serial.println(F("\n======== SEQUENCE 1 ========"));
  moveForwardBlocks(5, SPEED_START_FAST);   // 1) drive 5 blocks, then stop

  // 2) read the color of what's in front
  Serial.println(F("[SEQ1] Reading color..."));
  bool prevMon = g_colorMonitor; g_colorMonitor = true;  // force the [COLOR] line to print
  ColorID c = classifyColor();
  g_colorMonitor = prevMon;
  Serial.print(F("[SEQ1] Color = ")); Serial.println(colorProfiles[c].name);

  // 3) grab only if it's the target color
  if (c == TARGET_COLOR) {
    Serial.println(F("[SEQ1] Target BLUE confirmed -> approach + grab."));
    approachAndGrab();
  } else {
    Serial.println(F("[SEQ1] Not the target color - no grab."));
    car.Stop();
  }
  Serial.println(F("======== SEQUENCE 1 DONE ========\n"));
}

// ================================================================
//  BUTTON 8 — wait for YELLOW @ <= 4.5cm, then slow grab
// ================================================================
void conditionalGrabBtn8() {
  Serial.println(F("\n[BTN 8] Waiting for YELLOW at <= 4.5cm..."));
  wakeUltrasonic();
  bool conditionsMet = false;
  unsigned long telemetryTickMillis = millis();

  while (!conditionsMet) {
    if (IrReceiver.decode()) {
      uint32_t code = IrReceiver.decodedIRData.decodedRawData;
      if (code == BTN_0) { Serial.println(F("E-STOP! Aborting Btn8.")); car.Stop(); sleepUltrasonic(); restoreIR(); return; }
      IrReceiver.resume();
    }
    float distanceCm = readUltrasonicDistanceCM();
    ColorID detectedColor = classifyColor();
    printDistanceAndColor(F("[BTN 8]"), distanceCm, detectedColor);
    if (distanceCm > 0.0 && distanceCm <= BTN8_GRAB_DISTANCE_CM && detectedColor == BTN8_TARGET_COLOR) {
      Serial.print(F("[BTN 8] Met: ")); Serial.print(distanceCm); Serial.println(F("cm YELLOW -> grab."));
      conditionsMet = true;
    } else {
      delay(50);
    }

    motionTelemetryTick(telemetryTickMillis);
  }

  sleepUltrasonic();
  gripperGrabStepped(GRIPPER_CLOSE_US);   // slow, stepped, logged close (100us / 250ms)
  Serial.println(F("[BTN 8] Grab complete.\n"));
  restoreIR();
}

// ================================================================
//  GRIPPER STEP BUTTONS
// ================================================================
void gripperClose() {
  gripperMoveTo(gripperPosUs + GRIPPER_MANUAL_STEP_US);   // 50us nudge
  Serial.print(F("[GRIPPER] * close -> ")); Serial.print(gripperPosUs); Serial.println(F("us"));
}
void gripperOpen() {
  gripperMoveTo(gripperPosUs - GRIPPER_MANUAL_STEP_US);   // 50us nudge
  Serial.print(F("[GRIPPER] # open  -> ")); Serial.print(gripperPosUs); Serial.println(F("us"));
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);
  car.Init();

  pinMode(SensorLeft, INPUT); pinMode(SensorMiddle, INPUT); pinMode(SensorRight, INPUT);
  pinMode(TCS_S2_PIN, OUTPUT); pinMode(TCS_S3_PIN, OUTPUT); pinMode(TCS_OUT_PIN, INPUT);

  sleepUltrasonic();

  gripper.attach(GRIPPER_PIN, GRIPPER_OPEN_US, GRIPPER_CLOSE_US);
  gripper.writeMicroseconds(GRIPPER_OPEN_US);
  gripperPosUs = GRIPPER_OPEN_US;

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.println(F("Ready. Btn1=Sequence1(grab BLUE). Btn8=wait YELLOW. OK=color monitor. *=close #=open. Btn0=STOP."));
}

void loop() {
  static unsigned long lastColorPrint = 0;

  if (g_colorMonitor && millis() - lastColorPrint >= 500) {
    lastColorPrint = millis();
    classifyColor();
  }

  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    // Ignore NEC repeat frames (0xFFFFFFFF) so one press = one action.
    if (code != 0xFFFFFFFF) {
      if      (code == BTN_1)    runSequence1();
      else if (code == BTN_8)    conditionalGrabBtn8();
      else if (code == BTN_OK)  { g_colorMonitor = !g_colorMonitor; Serial.print(F("[OK] Color monitor ")); Serial.println(g_colorMonitor ? F("ON") : F("OFF")); }
      else if (code == BTN_STAR) gripperClose();
      else if (code == BTN_HASH) gripperOpen();
      else if (code == BTN_0)  { car.Stop(); Serial.println(F("E-STOP Active")); }
    }
    IrReceiver.resume();
  }
}