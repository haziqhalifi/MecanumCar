/*
  ================================================================
  Servo Isolation Test
  ================================================================
  Minimal sketch to check whether the claw servo physically moves,
  independent of the rest of Testbasic.cpp (motors, IR, ultrasonic,
  color sensor all untouched — nothing else is running).

  On boot: sweeps the servo open -> closed -> open once, then every
  2 seconds toggles between CLAW_OPEN_ANGLE and CLAW_CLOSED_ANGLE,
  printing each write over serial.

  If the horn visibly moves here but not in the full sketch, the
  problem is contention/power draw from the rest of the robot
  (motors + servo sharing a supply) rather than the servo itself.
  If it does NOT move here either, the servo/wiring/power is at fault
  regardless of the rest of the code.

  To build this instead of Testbasic.cpp, temporarily change
  platformio.ini's src_filter to:
    src_filter = +<ServoIsolationTest.cpp> from test dir, or copy
    this file's contents into src/ under a temp name and adjust
    src_filter to +<TempFile.cpp> -<Testbasic.cpp>
  ================================================================
*/

#include <Arduino.h>
#include <Servo.h>

#define CLAW_SERVO_PIN 9
const uint8_t CLAW_OPEN_ANGLE = 0;
const uint8_t CLAW_CLOSED_ANGLE = 100;

Servo clawServo;
bool isOpen = true;
unsigned long lastToggle = 0;

void setup() {
  Serial.begin(9600);
  delay(200);
  Serial.println(F("[TEST] Servo isolation test starting."));

  clawServo.attach(CLAW_SERVO_PIN);
  Serial.println(F("[TEST] Sweep: OPEN"));
  clawServo.write(CLAW_OPEN_ANGLE);
  delay(1000);
  Serial.println(F("[TEST] Sweep: CLOSED"));
  clawServo.write(CLAW_CLOSED_ANGLE);
  delay(1000);
  Serial.println(F("[TEST] Sweep: OPEN"));
  clawServo.write(CLAW_OPEN_ANGLE);
  delay(1000);
  clawServo.detach();

  Serial.println(F("[TEST] Initial sweep done. Toggling every 2s now."));
  lastToggle = millis();
}

void loop() {
  if (millis() - lastToggle >= 2000) {
    clawServo.attach(CLAW_SERVO_PIN);
    isOpen = !isOpen;
    uint8_t angle = isOpen ? CLAW_OPEN_ANGLE : CLAW_CLOSED_ANGLE;
    Serial.print(F("[TEST] Writing angle: "));
    Serial.println(angle);
    clawServo.write(angle);
    delay(400);
    clawServo.detach();
    lastToggle = millis();
  }
}
