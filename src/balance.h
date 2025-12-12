#pragma once

#include <Arduino.h>
#include <stdint.h>

const float GEAR_RATIO = 43.8; 
const int16_t MOTOR_SPEED_LIMIT = 250; 
const int16_t ANGLE_RATE_RATIO = 80;

// *** MECHANICAL OFFSET ***
const float VERTICAL_ANGLE_OFFSET = 8.7;

// PID constants
// determines the response to a combination of angle and angle_rate
// the combination measures how far the robot is from a stable trajectory
const int16_t ANGLE_RESPONSE = 13;
// determines how much the robot resists being moved away from its starting point.
const int16_t DISTANCE_RESPONSE =20; //55
// determines the response to differences between the left and right motors
const int16_t DISTANCE_DIFF_RESPONSE = -30; //-60
// supresses the large back-and-forth oscillations caused by DISTANCE_RESPONSE
const int16_t SPEED_RESPONSE = 6000; //6800

// Update rate
const uint8_t UPDATE_TIME_MS = 5;

// Calibration
const uint8_t CALIBRATION_ITERATIONS = 200;

// Start/Stop angles (millidegrees)
const int32_t START_BALANCING_ANGLE = 8000; // 8 degrees
const int32_t STOP_BALANCING_ANGLE = 45000;  // 45 degrees

// Global Variables
extern int32_t angle; 
extern int32_t angleRate; 
extern int16_t motorSpeed; 

void balanceSetup();
void balanceUpdate();
bool isBalancing();
void setMotorSpeeds(int16_t left, int16_t right);
long getLeftEncoderCounts();
long getRightEncoderCounts();