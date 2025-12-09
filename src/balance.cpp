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
    // Read fresh data
    myIMU.readAccelData(myIMU.accelCount);
    
    // Convert to G-Force (floats)
    float ay = (float)myIMU.accelCount[1];
    float az = (float)myIMU.accelCount[2];
    
    // YOUR FORMULA: atan2(ay, az)
    // We convert to Degrees ( * 180/PI )
    float rawAngle = atan2(ay, az) * 57.296; 
    //Serial.println(rawAngle);

    // If rawAngle is -7.0 when upright, and OFFSET is -7.0:
    // -7.0 - (-7.0) = 0.0 (Perfect Vertical)
    return (rawAngle - VERTICAL_ANGLE_OFFSET); 
}

void balanceSetup()
{
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    if(myIMU.readByte(MPU9250_ADDRESS_AD0, INT_STATUS) & 0x01) {
        myIMU.readGyroData(myIMU.gyroCount);
        // We use X-Axis Gyro because your Angle is atan2(y, z)
        // (Rotation around X changes Y and Z)
        total += myIMU.gyroCount[0]; 
    }
    delay(UPDATE_TIME_MS);
  }
  gYZero = total / CALIBRATION_ITERATIONS;
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
  if (abs(angleRate) < 2) 
  {
    // Convert float angle to Millidegrees for the PID
    angle = (int32_t)(getAccelAngle() * 1000); 
  }
}

void integrateGyro()
{
  // 1. Read Gyro
  // Ensure we read the axis perpendicular to your Y/Z plane (Usually X)
  myIMU.readGyroData(myIMU.gyroCount);
  
  // Convert to deg/s (using 2000dps scale typical of these libs, ~16.4 LSB/deg/s)
  // We use raw counts relative to the calibrated zero
  // 29 is roughly the divider for 1000deg/s scale to match Pololu units
  angleRate = (myIMU.gyroCount[0] - gYZero) / 29; 

  // 2. Integrate Gyro (Angle = Angle + Rate * Time)
  int32_t gyroAngle = angle + angleRate * UPDATE_TIME_MS;

  // 3. Read Accel for Drift Correction (Complementary Filter)
  int32_t accelAngle = (int32_t)(getAccelAngle() * 1000);

  // 4. Fuse them (95% Gyro, 5% Accel)
  angle = (gyroAngle * 95 + accelAngle * 5) / 100;
}

void integrateEncoders()
{
  static long lastCountsLeft;
  long countsLeft = getLeftEncoderCounts();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;


  static long lastCountsRight;
  long countsRight = getRightEncoderCounts();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;
}

void balance()
{
  // Standard Balboa PID Logic
  int32_t risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;

  motorSpeed += (
    + ANGLE_RESPONSE * risingAngleOffset
    + DISTANCE_RESPONSE * (distanceLeft + distanceRight)
    + SPEED_RESPONSE * (speedLeft + speedRight)
    ) / 100 / GEAR_RATIO;

  if (motorSpeed > MOTOR_SPEED_LIMIT) motorSpeed = MOTOR_SPEED_LIMIT;
  if (motorSpeed < -MOTOR_SPEED_LIMIT) motorSpeed = -MOTOR_SPEED_LIMIT;

  int16_t distanceDiff = distanceLeft - distanceRight;

  setMotorSpeeds(
    motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100,
    motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100
  );
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