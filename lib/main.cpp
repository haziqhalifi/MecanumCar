#include <Arduino.h>         // <-- This fixes the 'delay' and scope errors!
#include <SoftwareSerial.h>

// ==========================================
// 1. SENSOR PINS
// ==========================================
#define TRIG_PIN 12  
#define ECHO_PIN 13  

// ==========================================
// 2. SERIAL COMMUNICATION PINS
// ==========================================
// SoftwareSerial(RX, TX)
// Arduino Pin 11 (RX) connects to ESP32 Pin 17 (TX2)
// Arduino Pin 10 (TX) connects to ESP32 Pin 16 (RX2)
SoftwareSerial espSerial(11, 10); 

void setup() {
  // Start the hardware serial for USB debugging 
  Serial.begin(9600);
  
  // Start the software serial to talk to the ESP32
  espSerial.begin(9600);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("Robot started. Reading distance and sending to ESP32...");
}

void loop() {
  long duration;
  float distance;
  
  // --- STEP 1: Trigger the ultrasonic sensor ---
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // --- STEP 2: Read and calculate distance ---
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.034) / 2;
  
  // Print to the computer for debugging
  Serial.print("Local Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // --- STEP 3: Send data to the ESP32 ---
  espSerial.println(distance);
  
  // Wait 5 seconds before sending the next reading
  delay(5000); 
}