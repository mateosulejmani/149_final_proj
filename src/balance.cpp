#include "Balance.h"
#include <Wire.h>
#include "MPU9250.h" 


// EXTERNAL HARDWARE GLOBALS (Defined in Main.ino)
extern MPU9250 myIMU; 

// Internal state variables
int32_t gYZero;
int32_t angle; // millidegrees
int32_t angleRate; // degrees/s
int32_t distanceLeft;
int32_t speedLeft;
int32_t driveLeft;
int32_t distanceRight;
int32_t speedRight;
int32_t driveRight;
int16_t motorSpeed;
bool isBalancingStatus = false;
bool balanceUpdateDelayedStatus;

bool isBalancing() { return isBalancingStatus; }
bool balanceUpdateDelayed() { return balanceUpdateDelayedStatus; }

float getAccelAngle() {
    // Read data
    myIMU.readAccelData(myIMU.accelCount);
    
    // Get y and z reads from accelerometer
    float ay = (float)myIMU.accelCount[1];
    float az = (float)myIMU.accelCount[2];
    
    // Convert accelerometer data to tilt
    float rawAngle = atan2(ay, az) * 57.296; 
    
    // Apply offset to angle so upright is at 0 degrees
    // Apply negative sign to make forward be positive angle
    return -(rawAngle - VERTICAL_ANGLE_OFFSET); 
}

void balanceSetup()
{
  int32_t total = 0;
  int samples = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    if(myIMU.readByte(MPU9250_ADDRESS_AD0, INT_STATUS) & 0x01) {
        myIMU.readGyroData(myIMU.gyroCount);
        total += myIMU.gyroCount[0]; 
        samples++; // Count valid samples
    }
    delay(UPDATE_TIME_MS);
  }
  if (samples > 0) gYZero = total / samples;
}

void lyingDown()
{
  // Reset PID variables
  motorSpeed = 0;
  distanceLeft = 0;
  distanceRight = 0;
  setMotorSpeeds(0, 0);

  // If robot is calm (gyro showing little rotation)
  // We snap the angle to the Accelerometer's gravity vector
  if (abs(angleRate) < 20) //Was2
  {
    // Convert float angle to Millidegrees for the PID
    angle = (int32_t)(getAccelAngle() * 1000); 
  }
}

void integrateGyro()
{
  // Read gyro data
  myIMU.readGyroData(myIMU.gyroCount);
  
  // Apply offset to x-axis gyro data and divide by 32768/1000
  // 32768 is the max ADC count and MPU set to reach this when moving 1000 degrees per second
  angleRate = (myIMU.gyroCount[0] - gYZero) / 32.8; 

  // Gyro angle is our last angle + our current changed angle calculated from gyro
  int32_t gyroAngle = angle + angleRate * UPDATE_TIME_MS;

  // Get accelerometer angle
  int32_t accelAngle = (int32_t)(getAccelAngle() * 1000);

  // Combine accelerometer angle and gyro angle to account for gyro drift
  angle = (gyroAngle * 99 + accelAngle * 1) / 100;
}

void integrateEncoders()
{
  // Find how much the encoders have changed from last update
  // update the distance we are from center, and the last encoder count
  static long lastCountsLeft;
  long countsLeft = getLeftEncoderCounts();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += speedLeft;
  lastCountsLeft = countsLeft;

  static long lastCountsRight;
  long countsRight = getRightEncoderCounts();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += speedRight;
  lastCountsRight = countsRight;

  // Serial.println(countsRight);
  // Serial.println(countsLeft);
}

const int16_t MIN_POWER = 30;
void balance()
{
  // Standard Balboa PID Logic
  int32_t pdOutput = (angle * ANGLE_RESPONSE) + (angleRate * ANGLE_RATE_RATIO);
  
  // Scale down (remove the /GEAR_RATIO if it makes it too weak)
  motorSpeed = pdOutput / 100 / GEAR_RATIO;

  // 2. Deadband Compensation (The Fix for "Falling Over")
  if (motorSpeed > 0) {
    motorSpeed += MIN_POWER;
  } else if (motorSpeed < 0) {
    motorSpeed -= MIN_POWER;
  }

  // 3. Constrain to limits
  if (motorSpeed > MOTOR_SPEED_LIMIT) motorSpeed = MOTOR_SPEED_LIMIT;
  if (motorSpeed < -MOTOR_SPEED_LIMIT) motorSpeed = -MOTOR_SPEED_LIMIT;

  // 4. Differential Drive (Steering)
  int16_t distanceDiff = distanceLeft - distanceRight;
  
  setMotorSpeeds(
    motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100,
    motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100
  );
  Serial.print(motorSpeed);
}

void balanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();
  static uint8_t count = 0;

  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  lastMillis = ms;

  integrateGyro();
  integrateEncoders();

  // STATE MACHINE
  if (isBalancingStatus)
  {
    balance();
    // If we fall over (Angle > 70 degrees), Stop Balancing
    if (abs(angle) > STOP_BALANCING_ANGLE)
    {
      if (++count > 5) {
        isBalancingStatus = false;
        count = 0;
      }
    }
    else count = 0;
  }
  else
  {
    lyingDown(); // Keeps resetting Angle to Gravity while lying down
    
    // If we are lifted up (Angle < 20 degrees), Start Balancing
    if (abs(angle) < START_BALANCING_ANGLE)
    {
      if (++count > 5) {
        isBalancingStatus = true;
        count = 0;
      }
    }
    else count = 0;
  }
}