#include <Arduino.h>
#include <Servo.h>
#include "MecanumCar_v2.h"
#include <IRremote.hpp>

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
bool sensorsEnabled = true;
volatile bool emergencyStopActive = false; 

// ── Global Position and Heading Tracking ────────────────────────────────────
int currentX = 4, currentY = 3;
const int NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;
int currentHeading = WEST;


// ── Tuning Constants ───────────────────────────────────────────────────────
const uint8_t SPEED_START_FAST = 50;
const uint8_t SPEED_START_SLOW = 40;
const uint8_t SPEED_MIN = 35;
const unsigned long CENTER_OFFSET_MS = 110;
const uint8_t SPEED_ROTATE = 60;
const unsigned long TURN_BLIND_MS = 150;
const unsigned long BRAKE_MS = 40;
const int ITEM_DETECT_DISTANCE_CM = 25;
const int GRAB_APPROACH_DISTANCE_CM = 6;
const uint8_t CLAW_OPEN_ANGLE = 20;
const uint8_t CLAW_CLOSED_ANGLE = 100;
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
    Serial.print(F("distance: "));
    if (distance == -1) Serial.print(F("---"));
    else Serial.print(distance);
    Serial.print(F("cm | color: "));
    Serial.println(gridColorName(color));
}


void gridSetSpeed(uint8_t targetSpeed) { 
    speed_Upper_L = targetSpeed; speed_Lower_L = targetSpeed;
    speed_Upper_R = targetSpeed; speed_Lower_R = targetSpeed;
}

void gridStop() {
    mecCar.Stop();
    Serial.println(F("[SYSTEM] Robot Halted."));
}

void spinRightInPlace(uint8_t speed) { gridSetSpeed(speed); mecCar.Turn_Right(); }
void spinLeftInPlace(uint8_t speed) { gridSetSpeed(speed); mecCar.Turn_Left(); }

void disableSensors() {
    sensorsEnabled = false;
    Serial.println(F("[SYSTEM] Ultrasonic and Color sensors have been SOFTWARE DISABLED."));
}

bool checkEmergencyStop() {
    if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.command == CMD_STAR) {
            emergencyStopActive = true;
            gridStop();
            Serial.println(F("\n!!! EMERGENCY STOP TRIGGERED !!!"));
            IrReceiver.resume();
            return true;
        }
        IrReceiver.resume();
    }
    return emergencyStopActive;
}

bool gridRotateLeft90() {
    if (checkEmergencyStop()) return false;
    Serial.println(F("\n[TURN] Symmetrical Spin 90 Degrees Left..."));
    unsigned long startTime = millis();
    while (millis() - startTime < TURN_BLIND_MS) {
        if (checkEmergencyStop()) return false;
        spinLeftInPlace(SPEED_ROTATE);
    }
    while (true) {
        if (checkEmergencyStop()) return false;
        spinLeftInPlace(SPEED_ROTATE);
        if (digitalRead(LINE_LEFT_PIN) == HIGH) break;
    }
    spinRightInPlace(SPEED_ROTATE + 10);
    
    unsigned long brakeStart = millis();
    while (millis() - brakeStart < BRAKE_MS) { if (checkEmergencyStop()) return false; }
    mecCar.Stop();
    
    unsigned long settleStart = millis();

    while (millis() - settleStart < 300) { if (checkEmergencyStop()) return false; }
    return true;
}

bool gridRotateRight90() {
    if (checkEmergencyStop()) return false;
    Serial.println(F("\n[TURN] Symmetrical Spin 90 Degrees Right..."));
    unsigned long startTime = millis();
    while (millis() - startTime < TURN_BLIND_MS) {
        if (checkEmergencyStop()) return false;
        spinRightInPlace(SPEED_ROTATE);
    }
    while (true) {
        if (checkEmergencyStop()) return false;
        spinRightInPlace(SPEED_ROTATE);
        if (digitalRead(LINE_RIGHT_PIN) == HIGH) break;
    }
    spinLeftInPlace(SPEED_ROTATE + 10);
    
    unsigned long brakeStart = millis();
    while (millis() - brakeStart < BRAKE_MS) { if (checkEmergencyStop()) return false; }
    mecCar.Stop();
    
    unsigned long settleStart = millis();
    while (millis() - settleStart < 300) { if (checkEmergencyStop()) return false; }
    return true;
}

void gridMoveForwardBlocks(int targetBlocks, uint8_t startSpeed) {
    if (checkEmergencyStop()) return;
    int junctionCount = 0;
    bool onJunction = true;

    while (junctionCount < targetBlocks) {
        if (checkEmergencyStop()) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        int calculatedSpeed = startSpeed - (junctionCount * 5);
        if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
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
            onJunction = false; gridSetSpeed(currentSpeed); mecCar.Turn_Right();
        } else if (Left == HIGH && Center == LOW && Right == LOW) {
            onJunction = false; gridSetSpeed(currentSpeed); mecCar.Turn_Left();
        } else if (Left == LOW && Center == LOW && Right == LOW) {
            onJunction = false; gridSetSpeed(currentSpeed); mecCar.Advance();
        } else if (Left == HIGH && Center == HIGH && Right == LOW) {
            onJunction = false; gridSetSpeed(currentSpeed); mecCar.Turn_Left();
        } else if (Left == LOW && Center == HIGH && Right == HIGH) {
            onJunction = false; gridSetSpeed(currentSpeed); mecCar.Turn_Right();
        }
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
    unsigned long leaveTime = millis();
    while (millis() - leaveTime < 140) {
        if (checkEmergencyStop()) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);
        gridSetSpeed(startSpeed);
        if (Left == LOW && Center == HIGH && Right == LOW) mecCar.Advance();
        else if (Right == HIGH) mecCar.Turn_Right();
        else if (Left == HIGH) mecCar.Turn_Left();
        else mecCar.Advance();
    }

    while (true) {
        if (checkEmergencyStop()) return;
        uint8_t Left = digitalRead(LINE_LEFT_PIN);
        uint8_t Center = digitalRead(LINE_CENTER_PIN);
        uint8_t Right = digitalRead(LINE_RIGHT_PIN);

        if (Left == HIGH && Center == HIGH && Right == HIGH) break;
        else if (Left == LOW && Center == HIGH && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Advance(); }

        else if (Left == LOW && Center == LOW && Right == HIGH) { gridSetSpeed(startSpeed); mecCar.Turn_Right(); }
        else if (Left == HIGH && Center == LOW && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Turn_Left(); }
        else if (Left == LOW && Center == LOW && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Advance(); }
        else if (Left == HIGH && Center == HIGH && Right == LOW) { gridSetSpeed(startSpeed); mecCar.Turn_Left(); }
        else if (Left == LOW && Center == HIGH && Right == HIGH) { gridSetSpeed(startSpeed); mecCar.Turn_Right(); }
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
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000UL);
    if (duration == 0) return -1;
    return (int)(duration / 58.2);
}

bool checkItem() {
    if (emergencyStopActive) return false;
    int distance = gridGetDistanceCm();
    int color = gridDetectColorValue();
    printStatusTelemetry(distance, color);
    return (distance > 0 && distance <= ITEM_DETECT_DISTANCE_CM);
}

unsigned long gridReadColorChannel(bool s2, bool s3) {
    if (emergencyStopActive) return 0;
    digitalWrite(COLOR_S2_PIN, s2 ? HIGH : LOW);
    digitalWrite(COLOR_S3_PIN, s3 ? HIGH : LOW);
    delay(10);
    return pulseIn(COLOR_OUT_PIN, LOW, 30000UL);
}

int gridDetectColorValue() {
    if (!sensorsEnabled || emergencyStopActive) return ANY;
    unsigned long red = gridReadColorChannel(LOW, LOW);
    unsigned long green = gridReadColorChannel(HIGH, HIGH);
    unsigned long blue = gridReadColorChannel(LOW, HIGH);

    if (blue < red && blue < green) return BLUE;
    if (red < blue && green < blue && red < green * 1.4 && green < red * 1.4) return YELLOW;
    if (red < green && red < blue) return RED;
    return ANY;
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
    clawServo.write(CLAW_OPEN_ANGLE);
    delay(500);
    clawServo.detach();
}

void gridSlightReverse() {
    if (checkEmergencyStop()) return;
    Serial.println(F("[TRACK] Execution of brief post-turn alignment reverse..."));
    gridSetSpeed(SPEED_REVERSE_BUMP);
    mecCar.Back();
    unsigned long startTime = millis();
    while (millis() - startTime < REVERSE_BUMP_MS) { if (checkEmergencyStop()) return; }
    gridStop();
}

void grabColor(int color) {
    if (emergencyStopActive) return;
    int detected = gridDetectColorValue();
    
    if (detected == color || color == ANY) {
        clawServo.attach(CLAW_SERVO_PIN);
        
        // SLOW SWEEP: Gradually close the claw instead of snapping it
        for (int angle = CLAW_OPEN_ANGLE; angle <= CLAW_CLOSED_ANGLE; angle += 2) {
            clawServo.write(angle);
            delay(15); // Increase to 20 or 25 if it's STILL too fast
        }
        
        delay(800); // Wait almost a full second to ensure a firm grip before moving

        clawServo.detach();

        itemGrabbed = true;
        disableSensors();
    }
}

bool isGrabTargetReached(int currentDistance) {
    return (currentDistance > 0 && currentDistance <= GRAB_APPROACH_DISTANCE_CM);
}

void executeApproachMovement(int currentDistance) {
    if (emergencyStopActive) return;
    uint8_t approachSpeed = (currentDistance <= 0) ? 25 : SPEED_MIN;

    while (!isGrabTargetReached(currentDistance)) {
        if (checkEmergencyStop()) return;
        gridSetSpeed(approachSpeed);
        mecCar.Advance();
        delay(60);
        currentDistance = gridGetDistanceCm();
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
    }
}

// Drives the car from the current tracked (currentX, currentY) grid node to
// (targetX, targetY), one block at a time, using the line-following
// one-coordinate step and the tracked heading to know which way is which.
void moveCoord(int targetX, int targetY) {
    if (checkEmergencyStop()) return;

    int deltaX = targetX - currentX;
    if (deltaX != 0) {
        turnToHeading((deltaX > 0) ? EAST : WEST);
        for (int i = 0; i < abs(deltaX); i++) {
            if (checkEmergencyStop()) return;
            gridMoveForwardOneCoord(SPEED_START_FAST);
            if (emergencyStopActive) return;
            currentX += (currentHeading == EAST) ? 1 : -1;
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
        }
    }

    Serial.println(F("[NAVIGATE] Target Node Reached. Aligning to WEST baseline..."));
    turnToHeading(WEST);
    gridStop();
}

// Approaches the nearest object with the ultrasonic sensor and grabs it if
// its color matches targetColour (or always, if targetColour == ANY).
void moveToGrab(int targetColour) {
    if (checkEmergencyStop()) return;
    int distance = gridGetDistanceCm();
    executeApproachMovement(distance);
    if (emergencyStopActive) return;
    grabColor(targetColour);
}

void moveHome() {
    if (checkEmergencyStop()) return;
    moveCoord(4, 3); // Leveraging existing moveCoord instead of repeating logic
    if (emergencyStopActive) return;
    Serial.println(F("[NAVIGATE] Home Node Reached. Aligning to EAST baseline..."));
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
    if (!emergencyStopActive) Serial.println(F("[SYSTEM] Drop zone reached. Stopped instantly."));
}

void resetCoordinates() {
    currentX = 4; currentY = 3; currentHeading = WEST;
    emergencyStopActive = false; 
    Serial.println(F("[SYSTEM] Navigation Tracker Reset to Default Home Baseline (4,3) facing WEST."));
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

// Add 'int targetColour' to the parameters
void runPath(const Step* path, int len, int targetColour) {
    sensorsEnabled = true;
    for (int i = 0; i < len; i++) {
        if (emergencyStopActive) break;
        switch(path[i].act) {
            case FWD: gridMoveForwardBlocks(path[i].arg, SPEED_START_FAST); break;
            case LFT: gridRotateLeft90(); break;
            case RGT: gridRotateRight90(); break;
            case GRB: moveToGrab(targetColour); break; // Use targetColour here
            case REV: gridSlightReverse(); break;
            case DRP: openClawWithAttach(); break;

            case GDR: goToDrop(); break;
            case DLY: delay(path[i].arg * 100); break;
        }
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

void setup() {
    Serial.begin(115200);

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

    openClawWithAttach();
    mecCar.Init();
    gridSetSpeed(SPEED_START_FAST);
    IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

    Serial.println(F("=== Turn-Locked Autonomous Controller ==="));
}

void loop() {
    if (IrReceiver.decode()) {
        int key = IrReceiver.decodedIRData.command;
        
        if (key == CMD_STAR) {
            emergencyStopActive = true;
            gridStop();
            Serial.println(F("\n!!! EMERGENCY STOP TRIGGERED VIA LOOP !!!"));
            IrReceiver.resume();
            return;
        }

        if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
            Serial.print(F("Key Pressed: "));
            Serial.println(key);
            emergencyStopActive = false; 

            switch (key) {
                case 22: runPath(path1, sizeof(path1)/sizeof(Step), RED); break; // button 1
                case 25: runPath(path2, sizeof(path2)/sizeof(Step), RED); break; // button 2
                case 13: runPath(path3, sizeof(path3)/sizeof(Step), RED); break; // button 3
 
                case 12: runPath(path1, sizeof(path1)/sizeof(Step), BLUE); break; // button 1
                case 24: runPath(path2, sizeof(path2)/sizeof(Step), BLUE); break; // button 2
                case 94: runPath(path3, sizeof(path3)/sizeof(Step), BLUE); break; // button 3
 
                case 8: runPath(path1, sizeof(path1)/sizeof(Step), YELLOW); break; // button 1
                case 28: runPath(path2, sizeof(path2)/sizeof(Step), YELLOW); break; // button 2
                case 90: runPath(path3, sizeof(path3)/sizeof(Step), YELLOW); break; // button 3

                // case 12: executeAutoMission(1, BLUE); break;              // button 4
                // case 24: executeAutoMission(3, BLUE); break;              // button 5
                // case 94: executeAutoMission(5, BLUE); break;              // button 6
                // case 8: executeAutoMission(1, YELLOW); break;               // button 4
                // case 28: executeAutoMission(3, YELLOW); break;              // button 5
                // case 90: executeAutoMission(5, YELLOW); break;              // button 6
                case 74: openClawWithAttach(); break; // button #
                default: break;
            }
        }
        IrReceiver.resume();
    }
}