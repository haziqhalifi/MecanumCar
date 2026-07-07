/*
  ================================================================
  KS0560 Mecanum Car — FULL CONFIGURATION WITH STRAFE CALIBRATION
  ================================================================
  LOGIC:
    - Button 0: Emergency Stop.
    - Button 1: Move forward by TARGET_BLOCKS_CMD1 (4 blocks).
    - Button 2: Rotate Right, then move forward by TARGET_BLOCKS_CMD2 (2 blocks).
    - Button 3: Rotate Left, then move by TARGET_MARKERS_CMD3 (2 markers).
    - Button ROTATE_R (arrow): Spin right until any black line is detected.
    - Button ROTATE_L (arrow): Spin left until any black line is detected.
    - Button 6: Spin 180 degrees, then move forward by TARGET_BLOCKS_CMD180.
    - Button 7: Strafe Left, counts junctions via SENSOR_LEFT only.
    - Button 8: Spin 180 degrees only (no forward move afterward).
    - Button 9: Strafe Right, counts junctions via SENSOR_RIGHT only.
    - Button 4: Sequence - forward 4, turn left, forward 2, turn right, forward 2, turn 180.
    - Button 5: Same as Button 4, then mirrored in reverse to return to the start location/heading.
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>

// ── Hardware pins ─────────────────────────────────────────────
#define RECV_PIN        A3
#define SENSOR_LEFT     A0
#define SENSOR_MID      A1
#define SENSOR_RIGHT    A2

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
#define CMD_ROTATE_R    0x43 // Arrow button (PLAY/PAUSE) -> Spin right until line
#define CMD_ROTATE_L    0x44 // Arrow button (PREV) -> Spin left until line
#define CMD_180         0x5E // Digit 6 -> 180 degree turn, then move forward
#define CMD_7           0x08 // Digit 7 -> Strafe Left
#define CMD_180_ONLY    0x1C // Digit 8 -> 180 degree turn only (no forward move)
#define CMD_9           0x5A // Digit 9 -> Strafe Right
#define CMD_SEQ1        0x0C // Digit 4 -> Sequence: fwd 4, left, fwd 2, right, fwd 2, 180
#define CMD_SEQ2        0x18 // Digit 5 -> Sequence 1, then mirrored in reverse back to start

// ── MOVEMENT SETTINGS / COUNTS ────────────────────────────────
const int TARGET_BLOCKS_CMD1   = 4;  // How many blocks Button 1 moves forward
const int TARGET_BLOCKS_CMD2   = 2;  // How many blocks Button 2 moves forward after turning
const int TARGET_MARKERS_CMD3  = 2;  // How many right markers Button 3 counts after turning
const int TARGET_STRAFE_CMD7   = 2;  // How many left junctions Button 7 counts while strafing left
const int TARGET_STRAFE_CMD9   = 2;  // How many right junctions Button 9 counts while strafing right
const int TARGET_BLOCKS_CMD180 = 2;  // How many blocks Button 180 moves forward after turning
const int TARGET_BLOCKS_SEQ1_A = 4;  // Sequence leg 1: forward blocks before first turn
const int TARGET_BLOCKS_SEQ1_B = 2;  // Sequence leg 2: forward blocks after left turn
const int TARGET_BLOCKS_SEQ1_C = 2;  // Sequence leg 3: forward blocks after right turn
const int TARGET_BLOCKS_SEQ2_FINAL = 5;  // Button 5: forward blocks on the final return leg

// ── SPEED CONFIGURATIONS ──────────────────────────────────────
#define SPEED_START_FAST       60    // Forward start speed for Button 1 (No turning)
#define SPEED_POST_RIGHT_TURN  45    // Speed forward after turning RIGHT (Button 2)
#define SPEED_POST_LEFT_TURN   40    // Speed forward after turning LEFT (Button 3)
#define SPEED_POST_180_TURN    45    // Speed forward after 180-degree turn (Button 180)
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
#define OFFSET_CMD1_MS             110   // Extra ms to drive forward at the end of Button 1
#define OFFSET_POST_RIGHT_TURN_MS  150   // Extra ms to nudge forward after RIGHT movement finishes (Button 2)
#define OFFSET_POST_LEFT_TURN_MS   130   // Extra ms to nudge forward after LEFT movement finishes (Button 3)
#define OFFSET_POST_180_MS         150   // Extra ms to nudge forward after 180-degree turn (Button 180)
#define OFFSET_STRAFE_L_MS         120   // Extra ms to keep strafing left after final line detection
#define OFFSET_STRAFE_R_MS         120   // Extra ms to keep strafing right after final line detection

#define TURN_BLIND_MS              150   // Blind turn duration to clear the starting line

// ── Turn Configuration ────────────────────────────────────────
#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT

mecanumCar car(3, 2);

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
//  ROTATION FUNCTIONS
// ================================================================
bool rotateRight90() {
  Serial.println(F("\n--- Rotating 90 Degrees Right (Speed 60) ---"));
  // If already parked on a line (e.g. chained from a previous rotation), spin off it first
  // so the blind period + detection below finds the NEXT line, not the current one.
  while (digitalRead(STOP_SENSOR_RIGHT_TURN) == HIGH) {
    if (checkEstop()) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();
  }
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEstop()) {
       Serial.println(F("E-STOP!")); restoreIR(); return false; 
    }
    setMotorSpeed(SPEED_ROTATE); 
    car.Turn_Right();          
  }
  while (true) {
    if (checkEstop()) {
       Serial.println(F("E-STOP!")); restoreIR(); return false; 
    }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();
    if (digitalRead(STOP_SENSOR_RIGHT_TURN) == HIGH) {
      break;
    }
  }
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
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();
  }
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEstop()) {
       Serial.println(F("E-STOP!")); restoreIR(); return false; 
    }
    setMotorSpeed(SPEED_ROTATE); 
    car.Turn_Left();
  }
  while (true) {
    if (checkEstop()) {
       Serial.println(F("E-STOP!")); restoreIR(); return false; 
    }
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();
    if (digitalRead(STOP_SENSOR_LEFT_TURN) == HIGH) {
      break;
    }
  }
  car.Stop();
  delay(200); 
  return true;
}

bool rotate180() {
  if (!rotateRight90()) return false;
  delay(300);
  return rotateRight90();
}

void rotateRightUntilLine() {
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEstop()) { restoreIR(); return; }
    setMotorSpeed(SPEED_ROTATE); car.Turn_Right();          
  }
  while (true) {
    if (checkEstop()) { restoreIR(); return; }
    setMotorSpeed(SPEED_ROTATE); car.Turn_Right();
    if (digitalRead(SENSOR_LEFT) == HIGH || digitalRead(SENSOR_MID) == HIGH || digitalRead(SENSOR_RIGHT) == HIGH) break;
  }
  restoreIR(); 
}

void rotateLeftUntilLine() {
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEstop()) { restoreIR(); return; }
    setMotorSpeed(SPEED_ROTATE); car.Turn_Left();          
  }
  while (true) {
    if (checkEstop()) { restoreIR(); return; }
    setMotorSpeed(SPEED_ROTATE); car.Turn_Left();
    if (digitalRead(SENSOR_LEFT) == HIGH || digitalRead(SENSOR_MID) == HIGH || digitalRead(SENSOR_RIGHT) == HIGH) break;
  }
  restoreIR(); 
}

// ================================================================
//  LINE TRACKING FORWARD MOVEMENTS
// ================================================================
void moveForwardBlocks(int targetBlocks, uint8_t startSpeed, int endOffsetMs) {
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
}

void moveRightSideMarkers(int targetBlocks, uint8_t startSpeed, int endOffsetMs) {
  int markerCount = 0;
  bool onMarker = true; 

  while (markerCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    int calculatedSpeed = startSpeed - (markerCount * 5);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (M == HIGH && R == HIGH) {
      if (!onMarker) {
        markerCount++;
        onMarker = true; 
        if (markerCount == targetBlocks) break; 
      }
      setMotorSpeed(currentSpeed); car.Advance();                  
    } 
    else if (L == LOW && M == HIGH && R == LOW) {
      onMarker = false; setMotorSpeed(currentSpeed); car.Advance();                  
    }
    else if (L == LOW && M == LOW && R == HIGH) {
      onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right();               
    }
    else if (L == HIGH && M == LOW && R == LOW) {
      onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left();                
    }
    else if (L == LOW && M == LOW && R == LOW) {
      onMarker = false; setMotorSpeed(currentSpeed); car.Advance();                  
    }
    else if (L == HIGH && M == HIGH && R == LOW) {
      onMarker = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left();
    }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) break; 
      IrReceiver.resume();
    }
  }

  if (markerCount == targetBlocks && endOffsetMs > 0) {
    setMotorSpeed(SPEED_MIN); car.Advance(); delay(endOffsetMs); 
  }
  restoreIR(); 
}

// ================================================================
//  STRAFE LEFT MOVEMENT (Counts via SENSOR_LEFT Only)
// ================================================================
void strafeLeftBlocks(int targetBlocks, uint8_t startSpeed, int endOffsetMs) {
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
}

// ================================================================
//  STRAFE RIGHT MOVEMENT (Counts via SENSOR_RIGHT Only)
// ================================================================
void strafeRightBlocks(int targetBlocks, uint8_t startSpeed, int endOffsetMs) {
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
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_C, SPEED_POST_RIGHT_TURN, OFFSET_POST_RIGHT_TURN_MS);
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
  moveForwardBlocks(TARGET_BLOCKS_SEQ1_C, SPEED_POST_RIGHT_TURN, OFFSET_POST_RIGHT_TURN_MS);
  delay(1000);
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

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_LEFT,  INPUT);
  pinMode(SENSOR_MID,   INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  car.Init();
  IrReceiver.begin(RECV_PIN, false);
  Serial.println(F("Ready."));
}

void loop() {
  if (!IrReceiver.decode()) return;

  uint8_t cmd = IrReceiver.decodedIRData.command;

  if (cmd == 0x00 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    IrReceiver.resume();
    return;
  }

  Serial.print(F("IR cmd received: 0x"));
  Serial.println(cmd, HEX);

  if (cmd == CMD_1) {
    moveForwardBlocks(TARGET_BLOCKS_CMD1, SPEED_START_FAST, OFFSET_CMD1_MS); 
  } 
  else if (cmd == CMD_2) {
    if (rotateRight90()) { 
      moveForwardBlocks(TARGET_BLOCKS_CMD2, SPEED_POST_RIGHT_TURN, OFFSET_POST_RIGHT_TURN_MS); 
    }
  } 
  else if (cmd == CMD_3) {
    if (rotateLeft90()) { 
      moveRightSideMarkers(TARGET_MARKERS_CMD3, SPEED_POST_LEFT_TURN, OFFSET_POST_LEFT_TURN_MS); 
    } 
  } 
  else if (cmd == CMD_ROTATE_R) {
    rotateRightUntilLine();
  } 
  else if (cmd == CMD_ROTATE_L) {
    rotateLeftUntilLine();
  }
  else if (cmd == CMD_180) {
    if (rotate180()) {
      moveForwardBlocks(TARGET_BLOCKS_CMD180, SPEED_POST_180_TURN, OFFSET_POST_180_MS);
    }
  }
  else if (cmd == CMD_180_ONLY) {
    rotate180();
  }
  else if (cmd == CMD_7) {
    strafeLeftBlocks(TARGET_STRAFE_CMD7, SPEED_STRAFE_LEFT, OFFSET_STRAFE_L_MS);
  }
  else if (cmd == CMD_9) {
    strafeRightBlocks(TARGET_STRAFE_CMD9, SPEED_STRAFE_RIGHT, OFFSET_STRAFE_R_MS);
  }
  else if (cmd == CMD_STAR) {
    car.Stop();
    Serial.println(F("E-STOP (idle)!"));
  }
  else if (cmd == CMD_SEQ1) {
    runSequence1();
  }
  else if (cmd == CMD_SEQ2) {
    runSequence2();
  }

  IrReceiver.resume();
}