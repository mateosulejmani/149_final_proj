// Imports
#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h" 
#include "Balance.h"

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

//Pins left wheel
const int LEFT_PWM  = 25;
const int LEFT_IN1  = 26;
const int LEFT_IN2  = 27;

//Pin for right wheel
const int RIGHT_PWM = 14;
const int RIGHT_IN1 = 12;
const int RIGHT_IN2 = 13;

// NEW ENCODER PIN MAPPING
const int RIGHT_ENC_B = 17;   
const int LEFT_ENC_B  = 18;   
const int RIGHT_ENC_A = 5;   
const int LEFT_ENC_A  = 19;   

// IMU Setup
MPU9250 myIMU(MPU9250_ADDRESS_AD0, Wire, 400000);

// Wheel encoder ticks
volatile long leftCount = 0;
volatile long rightCount = 0;

void IRAM_ATTR leftEncoderISR() {
  // If the B pin state equals the A pin state, we are going one way, else the other.
  // Standard quadrature logic check.
  int b_val = digitalRead(LEFT_ENC_B);
  if (b_val == LOW) leftCount++; 
  else leftCount--;
}

void IRAM_ATTR rightEncoderISR() {
  int b_val = digitalRead(RIGHT_ENC_B);
  if (b_val == LOW) rightCount++; 
  else rightCount--;
}

// Get the amount of ticks for the left encoder
long getLeftEncoderCounts() {
  noInterrupts();
  long c = leftCount;
  interrupts();
  return c;
}

// Get the amount of ticks for the right encoder
long getRightEncoderCounts() {
  noInterrupts();
  long c = rightCount;
  interrupts();
  return c;
}

// Set the speed of the motors
void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
  // LEFT MOTOR
  if (leftSpeed > 0) {
    digitalWrite(LEFT_IN1, LOW);  // If wheel spins wrong way, swap LOW/HIGH here
    digitalWrite(LEFT_IN2, HIGH);
  } else if (leftSpeed < 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
  }
  analogWrite(LEFT_PWM, abs(leftSpeed));

  // RIGHT MOTOR
  if (rightSpeed > 0) {
    digitalWrite(RIGHT_IN1, HIGH); // If wheel spins wrong way, swap LOW/HIGH here
    digitalWrite(RIGHT_IN2, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
  analogWrite(RIGHT_PWM, abs(rightSpeed));
}

// Initializing all the pins, ISR, and IMU
void setup() {
  Serial.begin(115200);
  while(!Serial) {};

  // Motor Pins
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Encoder Pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP); 
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Attach Interrupts to A channels only (standard quadrature)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // IMU Setup
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  
  if (myIMU.readByte(MPU9250_ADDRESS_AD0, WHO_AM_I_MPU9250) == 0x71) {
    // ...
    //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
    
    // Enable DLPF (Important for Balboa logic!)
    myIMU.writeByte(MPU9250_ADDRESS_AD0, 26, 0x03); 
  }

  // --- CHANGED INSTRUCTION ---
  Serial.println("========================================");
  Serial.println("CALIBRATING GYRO BIAS");
  Serial.println("KEEP ROBOT LYING FLAT AND STILL!");
  Serial.println("========================================");
  
  balanceSetup(); // Measures the 'Still' value of the gyro
  
  Serial.println("Calibration Done.");
  Serial.println("Lift Robot to Upright Position to Start.");
}

void loop() {
  balanceUpdate();
  
  // Debug to verify angle is 0 when upright
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    if(isBalancing()) {
       Serial.print("BALANCING! Angle: "); Serial.println(angle / 1000.0);
    } else {
       // While lying down, this should read ~90.0 or ~-90.0
       // When you lift it up, watch it go to 0.0
       Serial.print("Lying Down. Angle: "); Serial.println(angle / 1000.0);
    }
  }
}





