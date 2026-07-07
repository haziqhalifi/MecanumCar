/*
 * ============================================================================
 *  VROOM — base firmware: line tracking + grid navigation
 *  Target: Keyestudio UNO PLUS (UNO-compatible) on KS0560 mecanum base
 * ============================================================================
 *
 *  ARCHITECTURE (top to bottom — each layer only talks to the one below it):
 *
 *    1. Mission table  — maps a trigger (IR remote on A3, or serial) to a path.
 *    2. Path library   — arrays of Step {type, param}, e.g. {STEP_FOLLOW, 3}.
 *    3. Interpreter    — runs one path, one step at a time. Advances the
 *                        active primitive every loop(); on DONE moves to the
 *                        next step, on FAULT stops safely.
 *    4. Primitives     — non-blocking modules with init() + update() that
 *                        return PRIM_RUNNING / PRIM_DONE / PRIM_FAULT.
 *                        EXACTLY ONE primitive is active at a time; it alone
 *                        owns the sensors. There is no global line-correction
 *                        rule — correction lives inside followLine.
 *    5. Motion layer   — the ONLY place that calls mecanumCar.*. Exposes
 *                        intent-named functions (driveStraight, nudgeLeft,
 *                        spinRight, brakePulse, stopMotors, ...).
 *
 *  DESIGN PRINCIPLES (non-negotiable):
 *    - Closed-loop on landmarks, never on time. Every primitive terminates
 *      on a physical event (line catch, all-black junction, counted
 *      junction). The single deliberate exception is ADVANCE_BEFORE_PIVOT_MS
 *      (see turn primitive), which is a tunable constant by design.
 *    - No delay() anywhere in the control path — all timing via millis().
 *    - Every primitive has a timeout failsafe (PRIMITIVE_TIMEOUT_MS): a
 *      missed line catch must never hang or run away.
 *    - Localization: per-leg junction counting + a heading state (N/E/S/W).
 *      No global metric coordinates.
 *
 *  FUTURE UPGRADE PATH (motion layer only — nothing above it changes):
 *    The STC8 motor slave also exposes 8 per-wheel PWM registers (0x1..0x8)
 *    over I2C. Swapping the library calls below for direct per-wheel PWM
 *    enables true speed control, gentle correction arcs, and pivoting about
 *    the FRONT SENSOR instead of the chassis center — which eliminates
 *    ADVANCE_BEFORE_PIVOT_MS entirely.
 *
 *  RESERVED PINS (do not use in this build):
 *    D12 (TRIG) / D13 (ECHO)  — ultrasonic, for the APPROACH primitive
 *    D9                       — gripper servo, for GRAB / RELEASE
 *    A4 / A5                  — hardware I2C bus, keep free for TCS3200 etc.
 *  IN USE: A0/A1/A2 line sensors, A3 IR receiver, D3/D2 motor I2C.
 *
 *  FUTURE PICKUP SUB-SEQUENCE (primitives are stubbed below, paths can
 *  already reference them):
 *    APPROACH (ultrasonic-terminated creep up to the block)
 *      -> READ_COLOR (confirm the block is the expected color)
 *      -> GRAB (close gripper servo)
 *  The start zone will later serve as a re-homing fixture: REHOME squares
 *  Vroom up against it and re-zeros the heading estimate each cycle.
 * ============================================================================
 */

#include <Arduino.h>
#include "MecanumCar_v2.h"   // Keyestudio MecanumCar_v2 library (STC8 I2C slave)
#include <IRremote.hpp>      // IR remote decoder (current IRremote API)

/* ============================================================================
 *  CONSTANTS — all tunables live here.
 *  Values marked TUNE ON HARDWARE must be dialed in on the physical robot;
 *  preserve them on future edits.
 * ==========================================================================*/

// ---- Line sensor array (3-channel digital IR, comparator + trimpot) -------
const uint8_t PIN_SENSOR_LEFT   = A0;
const uint8_t PIN_SENSOR_MIDDLE = A1;
const uint8_t PIN_SENSOR_RIGHT  = A2;

// Logic level the sensor outputs over BLACK tape. Flip if your board's
// comparator is wired the other way.  // TUNE ON HARDWARE
const int LINE_BLACK_LEVEL = HIGH;

// ---- followLine timing -----------------------------------------------------
// All three sensors must read black continuously this long before a junction
// is registered (rejects tape seams / electrical noise).
const unsigned long JUNCTION_DEBOUNCE_MS   = 30;    // TUNE ON HARDWARE

// After a junction is registered, junction detection is suppressed for this
// window so the same crossing is never counted twice while driving over it.
const unsigned long POST_JUNCTION_BLANK_MS = 400;   // TUNE ON HARDWARE

// All-white recovery: keep driving straight this long to re-acquire the line
// (e.g. sensor bounce over a gap); past it, followLine returns FAULT.
const unsigned long LOST_LINE_TIMEOUT_MS   = 600;   // TUNE ON HARDWARE

// ---- turn timing -------------------------------------------------------------
// The chassis is longer than one grid cell and the sensor array is at the
// FRONT, offset from the rotation center. Before pivoting we drive straight
// this long so the rotation CENTER (not the sensor) sits over the crossing.
// This is the only near-open-loop value in the design: it only needs to be
// approximately right, because the followLine that runs after the turn
// immediately re-centers on the new line. Keep it tunable — never replace it
// with a measured-and-hardcoded motion.
// (Per-wheel PWM upgrade note: pivoting about the front sensor via registers
// 0x1..0x8 eliminates this constant entirely — a motion-layer-only change.)
const unsigned long ADVANCE_BEFORE_PIVOT_MS = 350;  // TUNE ON HARDWARE

// While pivoting, ignore all line contacts for this long (or until all three
// sensors read white, whichever comes first) so the turn doesn't instantly
// re-detect the line it is leaving.
const unsigned long TURN_BLANK_MS           = 300;  // TUNE ON HARDWARE

// Brief counter-rotation after the middle sensor catches the new line, to
// kill rotational coast before stopping.
const unsigned long TURN_BRAKE_MS           = 60;   // TUNE ON HARDWARE

// ---- Failsafe ---------------------------------------------------------------
// If any primitive hasn't reached its terminating event by now: FAULT + stop.
const unsigned long PRIMITIVE_TIMEOUT_MS = 8000;    // TUNE ON HARDWARE

// ---- Speed (software duty-cycle throttle) -----------------------------------
// The MecanumCar_v2 library methods run at a fixed speed, so the motion layer
// throttles by gating the drive command on/off inside a short millis() window.
// 100 = full library speed, lower = slower. This is FOLLOW_SPEED for this
// build; the per-wheel PWM upgrade replaces it with real speed registers.
const uint8_t FOLLOW_SPEED_DUTY_PCT = 20;           // TUNE ON HARDWARE
const uint8_t TURN_SPEED_DUTY_PCT   = 20;           // TUNE ON HARDWARE
const unsigned long MOTION_DUTY_PERIOD_MS = 40;     // TUNE ON HARDWARE

// ---- IR remote (NEC 32-bit codes, KS0560 kit remote, receiver on A3) --------
// The five directional/OK codes below are the known-good Keyestudio reference
// values. Number/symbol keys VARY BY REMOTE UNIT: any press that doesn't match
// a known code is printed in HEX on serial — press the key, read the value off
// the monitor, and paste it into the matching const below.
const uint8_t PIN_IR_RECEIVER = A3;

const unsigned long IR_CODE_OK    = 0xFF02FD;  // abort / stop
const unsigned long IR_CODE_UP    = 0xFF629D;  // unmapped (no manual drive mode)
const unsigned long IR_CODE_DOWN  = 0xFFA857;  // unmapped (no manual drive mode)
const unsigned long IR_CODE_LEFT  = 0xFF22DD;  // unmapped (no manual drive mode)
const unsigned long IR_CODE_RIGHT = 0xFFC23D;  // unmapped (no manual drive mode)

// Number key "1" — run TEST_PATH. Common Keyestudio value, but number keys
// differ between remote units: confirm via the HEX printout described above.
const unsigned long IR_CODE_KEY_1 = 0xFF6897;  // VERIFY ON HARDWARE

// NEC repeat frame, sent while a button is HELD. Ignored so a slightly-long
// press doesn't queue multiple triggers.
const unsigned long IR_CODE_REPEAT = 0xFFFFFFFF;

// Non-blocking software debounce: re-triggers of the SAME button within this
// window are rejected (replaces the old blocking delay(300) pattern).
const unsigned long IR_REPEAT_LOCKOUT_MS = 250;  // TUNE ON HARDWARE

/* ============================================================================
 *  HARDWARE
 * ==========================================================================*/

// Four DC motors via STC8 I2C slave (bit-banged bus: SDA -> D3, SCL -> D2).
mecanumCar mecanumCar(3, 2);

// IR remote receiver on A3 (mission triggers only — never read by primitives).
// Current IRremote API uses the global IrReceiver singleton.

/* ============================================================================
 *  SHARED TYPES
 * ==========================================================================*/

enum PrimStatus { PRIM_RUNNING, PRIM_DONE, PRIM_FAULT };

enum StepType {
  STEP_FOLLOW,      // param = junction count to stop at (also the expected
                    //         count for the leg — see localization note)
  STEP_TURN,        // param = TURN_LEFT / TURN_RIGHT
  // ---- future hooks (stubbed below, paths may already reference them) ----
  STEP_APPROACH,    // ultrasonic-terminated creep (D12/D13)     // TODO
  STEP_READ_COLOR,  // TCS3200 block confirmation                // TODO
  STEP_GRAB,        // gripper servo close (D9)                  // TODO
  STEP_RELEASE,     // gripper servo open (D9)                   // TODO
  STEP_REHOME       // square up on start-zone fixture, re-zero heading // TODO
};

const int TURN_LEFT  = 0;
const int TURN_RIGHT = 1;

struct Step {
  StepType type;
  int      param;
};

// Heading state — the robot's only global orientation estimate. Updated by
// each completed turn; re-zeroed by REHOME (future).
enum Heading { NORTH, EAST, SOUTH, WEST };
Heading g_heading = NORTH;

/* ============================================================================
 *  MOTION LAYER — the only code that touches mecanumCar.*
 *
 *  Callers express INTENT (driveStraight / nudgeLeft / spinRight / ...);
 *  this layer maps intent to library calls and applies the duty-cycle
 *  throttle. motionService() must run every loop().
 *
 *  Upgrade path: replace the switch in motionApply() with direct writes to
 *  the STC8 per-wheel PWM registers 0x1..0x8 — nudges become gentle arcs,
 *  spins become front-sensor pivots, and nothing above this layer changes.
 * ==========================================================================*/

enum MotionCmd {
  M_STOP,
  M_FORWARD,   // straight line following        (follow duty)
  M_NUDGE_L,   // gentle correction toward left  (follow duty)
  M_NUDGE_R,   // gentle correction toward right (follow duty)
  M_SPIN_L,    // pivot left                     (turn duty)
  M_SPIN_R,    // pivot right                    (turn duty)
  M_BRAKE_L,   // full-power counter-rotation left  (kills rightward coast)
  M_BRAKE_R    // full-power counter-rotation right (kills leftward coast)
};

static MotionCmd g_motionCmd     = M_STOP;  // what the active primitive wants
static MotionCmd g_motionApplied = M_STOP;  // what was last sent over I2C
static MotionCmd g_lastSpin      = M_STOP;  // remembered for brakePulse()

// Send one command to the STC8. Called only on change (motionService) to
// avoid hammering the bit-banged I2C bus every loop.
static void motionApply(MotionCmd cmd) {
  switch (cmd) {
    case M_FORWARD:                mecanumCar.Advance();    break;
    case M_NUDGE_L: case M_SPIN_L:
    case M_BRAKE_L:                mecanumCar.Turn_Left();  break;
    case M_NUDGE_R: case M_SPIN_R:
    case M_BRAKE_R:                mecanumCar.Turn_Right(); break;
    case M_STOP: default:          mecanumCar.Stop();       break;
  }
}

static uint8_t motionDutyFor(MotionCmd cmd) {
  switch (cmd) {
    case M_FORWARD: case M_NUDGE_L: case M_NUDGE_R: return FOLLOW_SPEED_DUTY_PCT;
    case M_SPIN_L:  case M_SPIN_R:                  return TURN_SPEED_DUTY_PCT;
    default:                                        return 100; // brake at full
  }
}

// Runs every loop(): applies the commanded motion, gated by the duty window.
void motionService() {
  MotionCmd want = g_motionCmd;
  uint8_t duty = motionDutyFor(want);
  if (want != M_STOP && duty < 100) {
    unsigned long phase = millis() % MOTION_DUTY_PERIOD_MS;
    if (phase >= (MOTION_DUTY_PERIOD_MS * duty) / 100) {
      want = M_STOP;  // coast portion of the duty window
    }
  }
  if (want != g_motionApplied) {
    motionApply(want);
    g_motionApplied = want;
  }
}

// ---- Intent-named API used by the primitives -------------------------------
void driveStraight() { g_motionCmd = M_FORWARD; }
void nudgeLeft()     { g_motionCmd = M_NUDGE_L; }
void nudgeRight()    { g_motionCmd = M_NUDGE_R; }
void spinLeft()      { g_motionCmd = M_SPIN_L; g_lastSpin = M_SPIN_L; }
void spinRight()     { g_motionCmd = M_SPIN_R; g_lastSpin = M_SPIN_R; }
void stopMotors()    { g_motionCmd = M_STOP; }

// Brief counter-rotation against the most recent spin. The caller times the
// pulse (TURN_BRAKE_MS) and then calls stopMotors().
void brakePulse() {
  g_motionCmd = (g_lastSpin == M_SPIN_L) ? M_BRAKE_R
              : (g_lastSpin == M_SPIN_R) ? M_BRAKE_L
              : M_STOP;
}

/* ============================================================================
 *  SENSOR READ
 * ==========================================================================*/

struct LineReading {
  bool left;    // true = black
  bool middle;
  bool right;
};

LineReading readLineSensors() {
  LineReading r;
  r.left   = (digitalRead(PIN_SENSOR_LEFT)   == LINE_BLACK_LEVEL);
  r.middle = (digitalRead(PIN_SENSOR_MIDDLE) == LINE_BLACK_LEVEL);
  r.right  = (digitalRead(PIN_SENSOR_RIGHT)  == LINE_BLACK_LEVEL);
  return r;
}

/* ============================================================================
 *  PRIMITIVE: followLine
 *
 *  Follow the line at low speed; stop at the Nth junction (param).
 *
 *  Every update() reads all three sensors and evaluates these checks in
 *  STRICT PRIORITY ORDER — first match wins:
 *    1. Junction (all black), debounced. The junction test runs BEFORE the
 *       steering test so a crossing is never misread as a steering error.
 *    2. Centered (white-black-white) -> driveStraight().
 *    3. Drifted (black on one side)  -> nudge toward the black side.
 *    4. Lost (all white), debounced  -> drive straight briefly to re-acquire;
 *       FAULT past LOST_LINE_TIMEOUT_MS.
 *
 *  Localization note: param doubles as the leg's EXPECTED junction count.
 *  Because the primitive stops exactly when the count is reached, the only
 *  way the count can deviate is by never getting there — which surfaces as
 *  the PRIMITIVE_TIMEOUT_MS or lost-line FAULT rather than the robot
 *  proceeding on a bad position estimate.
 * ==========================================================================*/

struct FollowState {
  int           targetJunctions;   // param: stop at this junction count
  int           junctionCount;     // junctions registered this leg
  unsigned long startMs;           // for PRIMITIVE_TIMEOUT_MS
  bool          inAllBlack;        // currently inside an all-black debounce?
  unsigned long allBlackSince;
  unsigned long blankUntil;        // junction detection suppressed until here
  bool          inAllWhite;        // currently inside a lost-line window?
  unsigned long allWhiteSince;
};
static FollowState g_follow;

void followLineInit(int targetJunctions) {
  g_follow.targetJunctions = targetJunctions;
  g_follow.junctionCount   = 0;
  g_follow.startMs         = millis();
  g_follow.inAllBlack      = false;
  g_follow.allBlackSince   = 0;
  g_follow.blankUntil      = 0;
  g_follow.inAllWhite      = false;
  g_follow.allWhiteSince   = 0;
  driveStraight();
}

PrimStatus followLineUpdate() {
  unsigned long now = millis();

  // Failsafe: never hang or run away on a missed landmark.
  if (now - g_follow.startMs > PRIMITIVE_TIMEOUT_MS) {
    stopMotors();
    return PRIM_FAULT;
  }

  LineReading s = readLineSensors();
  bool anyBlack = s.left || s.middle || s.right;
  bool allBlack = s.left && s.middle && s.right;

  // Any black contact resets the lost-line debounce.
  if (anyBlack) g_follow.inAllWhite = false;

  // -- Priority 1: junction (all black), debounced, suppressed during the
  //    post-junction blank window. ------------------------------------------
  if (allBlack && now >= g_follow.blankUntil) {
    if (!g_follow.inAllBlack) {
      g_follow.inAllBlack    = true;
      g_follow.allBlackSince = now;
    } else if (now - g_follow.allBlackSince >= JUNCTION_DEBOUNCE_MS) {
      // Junction confirmed.
      g_follow.inAllBlack = false;
      g_follow.junctionCount++;
      if (g_follow.junctionCount >= g_follow.targetJunctions) {
        stopMotors();
        return PRIM_DONE;
      }
      // Pass-through junction: keep driving, suppress re-detection while the
      // (long) chassis clears the crossing.
      g_follow.blankUntil = now + POST_JUNCTION_BLANK_MS;
    }
    driveStraight();      // hold course while debouncing / passing through
    return PRIM_RUNNING;
  }
  g_follow.inAllBlack = false;

  // -- Priority 2: centered (white-black-white) ------------------------------
  if (s.middle && !s.left && !s.right) {
    driveStraight();
    return PRIM_RUNNING;
  }

  // -- Priority 3: drifted — nudge toward the black side ---------------------
  if (s.left && !s.right) {          // line under left sensor -> steer left
    nudgeLeft();
    return PRIM_RUNNING;
  }
  if (s.right && !s.left) {          // line under right sensor -> steer right
    nudgeRight();
    return PRIM_RUNNING;
  }
  if (anyBlack) {                    // odd signature (e.g. L+R, no M, or
    driveStraight();                 // all-black during blank): hold course
    return PRIM_RUNNING;
  }

  // -- Priority 4: lost line (all white), debounced recovery -----------------
  if (!g_follow.inAllWhite) {
    g_follow.inAllWhite    = true;
    g_follow.allWhiteSince = now;
  }
  if (now - g_follow.allWhiteSince > LOST_LINE_TIMEOUT_MS) {
    stopMotors();
    return PRIM_FAULT;
  }
  driveStraight();                   // continue straight briefly to re-acquire
  return PRIM_RUNNING;
}

/* ============================================================================
 *  PRIMITIVE: turn (sensor-terminated 90°, built for the long chassis)
 *
 *  A naive spin-in-place overshoots: the front sensor array swings a wide
 *  arc around the rear-offset rotation center and lands in white space.
 *  Sequence:
 *    1. ADVANCE — drive straight ADVANCE_BEFORE_PIVOT_MS so the rotation
 *       CENTER (not the sensor) sits over the crossing.
 *    2. BLANK   — pivot; ignore line contacts until TURN_BLANK_MS elapses or
 *       all sensors read white (origin line confirmed cleared).
 *    3. SEEK    — keep pivoting; terminate the instant the MIDDLE sensor
 *       catches the new perpendicular line.
 *    4. BRAKE   — brakePulse() (counter-rotation) for TURN_BRAKE_MS to kill
 *       coast, then stopMotors(), update heading, DONE.
 *  PRIMITIVE_TIMEOUT_MS is enforced across all phases.
 *
 *  The followLine that runs next immediately re-centers on the new line, so
 *  ADVANCE_BEFORE_PIVOT_MS only needs to drop the sensor near the target
 *  line, not on its exact center.
 * ==========================================================================*/

enum TurnPhase { TP_ADVANCE, TP_BLANK, TP_SEEK, TP_BRAKE };

struct TurnState {
  int           dir;          // TURN_LEFT / TURN_RIGHT
  TurnPhase     phase;
  unsigned long startMs;      // for PRIMITIVE_TIMEOUT_MS
  unsigned long phaseStartMs;
};
static TurnState g_turn;

// Heading bookkeeping: NORTH->EAST->SOUTH->WEST is clockwise (right turns).
static Heading headingAfterTurn(Heading h, int dir) {
  int i = (int)h;
  i = (dir == TURN_RIGHT) ? (i + 1) % 4 : (i + 3) % 4;
  return (Heading)i;
}

void turnInit(int dir) {
  g_turn.dir          = dir;
  g_turn.phase        = TP_ADVANCE;
  g_turn.startMs      = millis();
  g_turn.phaseStartMs = g_turn.startMs;
  driveStraight();   // phase 1: put the rotation center over the crossing
}

PrimStatus turnUpdate() {
  unsigned long now = millis();

  if (now - g_turn.startMs > PRIMITIVE_TIMEOUT_MS) {
    stopMotors();
    return PRIM_FAULT;
  }

  switch (g_turn.phase) {

    case TP_ADVANCE:
      if (now - g_turn.phaseStartMs >= ADVANCE_BEFORE_PIVOT_MS) {
        if (g_turn.dir == TURN_LEFT) spinLeft(); else spinRight();
        g_turn.phase        = TP_BLANK;
        g_turn.phaseStartMs = now;
      }
      break;

    case TP_BLANK: {
      LineReading s = readLineSensors();
      bool allWhite = !s.left && !s.middle && !s.right;
      // Blanking ends by time, or early once the origin line is cleared.
      if (now - g_turn.phaseStartMs >= TURN_BLANK_MS || allWhite) {
        g_turn.phase        = TP_SEEK;
        g_turn.phaseStartMs = now;
      }
      break;
    }

    case TP_SEEK: {
      LineReading s = readLineSensors();
      if (s.middle) {  // middle sensor caught the new perpendicular line
        brakePulse();
        g_turn.phase        = TP_BRAKE;
        g_turn.phaseStartMs = now;
      }
      break;
    }

    case TP_BRAKE:
      if (now - g_turn.phaseStartMs >= TURN_BRAKE_MS) {
        stopMotors();
        g_heading = headingAfterTurn(g_heading, g_turn.dir);
        return PRIM_DONE;
      }
      break;
  }
  return PRIM_RUNNING;
}

/* ============================================================================
 *  FUTURE PRIMITIVE STUBS
 *
 *  Each slots into the interpreter as a new StepType without touching
 *  followLine, turn, or the interpreter itself. They currently return
 *  PRIM_DONE immediately so paths that reference them run end-to-end.
 * ==========================================================================*/

// APPROACH: creep forward until the ultrasonic (TRIG=D12, ECHO=D13) reports
// param mm to the block. Terminates on distance — a landmark, not time.
void approachBlockInit(int /*targetDistanceMm*/) { /* TODO */ }
PrimStatus approachBlockUpdate() { return PRIM_DONE; }   // TODO

// READ_COLOR: sample the TCS3200 (pins TBD — A4/A5 stay free for I2C) and
// confirm the block color; FAULT/branch handling designed when implemented.
void readColorInit(int /*expectedColor*/) { /* TODO */ }
PrimStatus readColorUpdate() { return PRIM_DONE; }       // TODO

// GRAB / RELEASE: gripper servo on D9, non-blocking sweep via millis().
void grabInit(int /*param*/) { /* TODO */ }
PrimStatus grabUpdate() { return PRIM_DONE; }            // TODO
void releaseInit(int /*param*/) { /* TODO */ }
PrimStatus releaseUpdate() { return PRIM_DONE; }         // TODO

// REHOME: square up against the start-zone fixture, re-zero g_heading and
// the position estimate. Runs at the start of each cycle to stop drift
// from accumulating across missions.
void reHomeInit(int /*param*/) { /* TODO */ }
PrimStatus reHomeUpdate() { return PRIM_DONE; }          // TODO

/* ============================================================================
 *  PATH LIBRARY
 * ==========================================================================*/

// Hard-coded test path for validating line-tracking consistency:
// follow to the 2nd junction, turn left, follow 1 junction, turn right,
// follow 1 junction, stop.
const Step TEST_PATH[] = {
  { STEP_FOLLOW, 2 },
  { STEP_TURN,   TURN_LEFT },
  { STEP_FOLLOW, 1 },
  { STEP_TURN,   TURN_RIGHT },
  { STEP_FOLLOW, 1 },
};
const int TEST_PATH_LEN = sizeof(TEST_PATH) / sizeof(TEST_PATH[0]);

/* ============================================================================
 *  INTERPRETER
 *
 *  Holds the current path, the step index, and drives the active primitive.
 *  Exactly one primitive is active at a time; the interpreter is the only
 *  code that starts or switches primitives.
 * ==========================================================================*/

enum RobotState { ROBOT_IDLE, ROBOT_RUNNING, ROBOT_FAULTED };

static RobotState  g_state     = ROBOT_IDLE;
static const Step* g_path      = nullptr;
static int         g_pathLen   = 0;
static int         g_stepIndex = 0;

static void interpreterStartStep() {
  const Step& st = g_path[g_stepIndex];
  Serial.print(F("[step "));
  Serial.print(g_stepIndex);
  Serial.print(F("] type="));
  Serial.print((int)st.type);
  Serial.print(F(" param="));
  Serial.println(st.param);

  switch (st.type) {
    case STEP_FOLLOW:     followLineInit(st.param);   break;
    case STEP_TURN:       turnInit(st.param);         break;
    case STEP_APPROACH:   approachBlockInit(st.param); break;
    case STEP_READ_COLOR: readColorInit(st.param);    break;
    case STEP_GRAB:       grabInit(st.param);         break;
    case STEP_RELEASE:    releaseInit(st.param);      break;
    case STEP_REHOME:     reHomeInit(st.param);       break;
  }
}

static PrimStatus interpreterUpdateActive() {
  switch (g_path[g_stepIndex].type) {
    case STEP_FOLLOW:     return followLineUpdate();
    case STEP_TURN:       return turnUpdate();
    case STEP_APPROACH:   return approachBlockUpdate();
    case STEP_READ_COLOR: return readColorUpdate();
    case STEP_GRAB:       return grabUpdate();
    case STEP_RELEASE:    return releaseUpdate();
    case STEP_REHOME:     return reHomeUpdate();
  }
  return PRIM_FAULT;  // unknown step type: stop safely
}

void interpreterStartPath(const Step* path, int len) {
  g_path      = path;
  g_pathLen   = len;
  g_stepIndex = 0;
  g_state     = ROBOT_RUNNING;
  interpreterStartStep();
}

void interpreterService() {
  if (g_state != ROBOT_RUNNING) return;

  PrimStatus st = interpreterUpdateActive();

  if (st == PRIM_DONE) {
    g_stepIndex++;
    if (g_stepIndex >= g_pathLen) {
      stopMotors();
      g_state = ROBOT_IDLE;
      Serial.println(F("[path] complete"));
    } else {
      interpreterStartStep();
    }
  } else if (st == PRIM_FAULT) {
    stopMotors();
    g_state = ROBOT_FAULTED;
    Serial.print(F("[path] FAULT at step "));
    Serial.println(g_stepIndex);
  }
}

/* ============================================================================
 *  MISSION LAYER
 *
 *  Maps triggers to mission actions. Two trigger sources feed one dispatch:
 *    - IR remote (receiver on A3, NEC codes):
 *        OK      — abort / clear fault, stop motors
 *        key "1" — run TEST_PATH (only when not already running)
 *        Up/Down/Left/Right — reserved; no manual-drive mode exists in the
 *        base build, so they are deliberately unmapped (see missionOnIrCode).
 *    - Serial (kept in parallel for bench testing):
 *        '1' — run TEST_PATH        'x' — abort
 *
 *  This layer only starts/aborts missions. It never touches the primitives,
 *  the interpreter internals, or the motion layer beyond stopMotors().
 * ==========================================================================*/

// ---- Shared mission actions (both serial and IR land here) -----------------
static void missionRunTestPath() {
  if (g_state == ROBOT_RUNNING) return;  // ignore trigger while a path runs
  Serial.println(F("[mission] starting TEST_PATH"));
  interpreterStartPath(TEST_PATH, TEST_PATH_LEN);
}

static void missionAbort() {
  stopMotors();
  g_state = ROBOT_IDLE;
  Serial.println(F("[mission] abort/reset"));
}

// ---- Serial trigger source -------------------------------------------------
static void missionServiceSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if      (c == '1') missionRunTestPath();
  else if (c == 'x') missionAbort();
}

// ---- IR trigger source -----------------------------------------------------
// Non-blocking debounce state: last accepted code + when it was accepted.
static unsigned long g_irLastCode     = 0;
static unsigned long g_irLastAcceptMs = 0;

static void missionOnIrCode(unsigned long code) {
  if      (code == IR_CODE_OK)    missionAbort();
  else if (code == IR_CODE_KEY_1) missionRunTestPath();
  // Directional keys: recognized but unmapped — the base build has no manual
  // drive mode and one is deliberately NOT invented here (a manual mode would
  // need its own primitive so it obeys the one-owner-of-the-motors rule).
  else if (code == IR_CODE_UP)    { /* TODO: manual forward (future mode) */ }
  else if (code == IR_CODE_DOWN)  { /* TODO: manual backward (future mode) */ }
  else if (code == IR_CODE_LEFT)  { /* TODO: manual left (future mode) */ }
  else if (code == IR_CODE_RIGHT) { /* TODO: manual right (future mode) */ }
  else {
    // CALIBRATION AID: unrecognized key. Press any button on your remote and
    // read its NEC code off the serial monitor, then paste it into the
    // matching IR_CODE_* const in the constants block.
    Serial.print(F("[ir] unknown code: 0x"));
    Serial.println(code, HEX);
  }
}

static void missionServiceIr() {
  if (!IrReceiver.decode()) return;

  unsigned long code = IrReceiver.decodedIRData.decodedRawData;
  unsigned long now  = millis();

  // NEC repeat frames (button held) are ignored outright, and re-triggers of
  // the same button inside the lockout window are rejected — no delay() ever.
  bool isRepeatFrame = (code == IR_CODE_REPEAT) ||
                       ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) != 0);
  bool inLockout     = (code == g_irLastCode) &&
                       (now - g_irLastAcceptMs < IR_REPEAT_LOCKOUT_MS);
  if (!isRepeatFrame && !inLockout) { 
    g_irLastCode     = code;
    g_irLastAcceptMs = now;
    missionOnIrCode(code);
  }

  IrReceiver.resume();  // re-arm the receiver for the next frame
}

void missionService() {
  missionServiceSerial();
  missionServiceIr();
}

/* ============================================================================
 *  SETUP / LOOP
 * ==========================================================================*/

void setup() {
  Serial.begin(9600);

  pinMode(PIN_SENSOR_LEFT,   INPUT);
  pinMode(PIN_SENSOR_MIDDLE, INPUT);
  pinMode(PIN_SENSOR_RIGHT,  INPUT);
  // Reserved, deliberately untouched: D12/D13 (ultrasonic), D9 (servo),
  // A4/A5 (hardware I2C for future color sensor).

  IrReceiver.begin(PIN_IR_RECEIVER, DISABLE_LED_FEEDBACK);  // start the IR receiver on A3 (mission triggers)

  mecanumCar.Init();
  stopMotors();

  g_heading = NORTH;  // starting orientation; REHOME will re-zero this later

  Serial.println(F("Vroom base firmware ready."));
  Serial.println(F("  IR: key 1 = run test path, OK = abort. Serial: '1' / 'x'."));
  Serial.println(F("  Unknown IR keys print their NEC code for calibration."));
}

void loop() {
  missionService();      // trigger handling (serial stub)
  interpreterService();  // advance the active primitive / step sequencing
  motionService();       // apply the commanded motion (duty-cycle throttle)
}
