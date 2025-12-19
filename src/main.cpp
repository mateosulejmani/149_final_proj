#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "MPU9250.h"
#include <esp_wifi.h>

// WIFI Config
const char* ssid = "DroneRobot";
const char* password = "12345678";
WebServer server(80);

// WiFi optimization settings
#define WIFI_CHANNEL 1
#define WIFI_MAX_CONNECTIONS 1
#define WIFI_POWER WIFI_POWER_19_5dBm

// Tunable
#define THROTTLE_STEP 50
#define UPRIGHT_THROTTLE 1300
#define HOVER_THROTTLE 1300

// Trim adjustments
int rollTrim = 5;
#define PITCH_TRIM 0

// Jump tunable
#define JUMP_THROTTLE 1635
#define JUMP_PITCH 20
#define JUMP_HOLD_TIME 1000
#define JUMP_RAMP_RATE 50

// Encoder-based movement
#define WHEEL_DIAMETER_CM 7.0
#define ENCODER_CPR 2800.0
#define TARGET_DISTANCE_CM 50.0
#define TRAVEL_SPEED 15.0
#define STEERING_KP 0.1

// Turn Encoder based
#define TURN_SPEED 15 // PWM speed for turning
#define TURN_ENCODER_TICKS 750 // Ticks for ~45 degree turn
#define TURN_STEERING_KP 0.1 // Speed matching correction

// Turn state
bool isTurning = false;
int16_t turnDirection = 0;
int turnTargetTicks = TURN_ENCODER_TICKS;
unsigned long turnStartTime = 0;
#define TURN_TIMEOUT 5000 // Safety timeout

// Safety Checks
static const unsigned long FAILSAFE_TIMEOUT_MS = 5000;
static const float ANGLE_LIMIT = 60.0f;

// PPM comms with Drone
#define PPM_PIN 17
#define CHANNELS 8
#define PPM_FRAME_LENGTH 22500
#define PPM_PULSE_LENGTH 300

// IMU
#define SDA_PIN 21
#define SCL_PIN 22

// Motors
const int LEFT_PWM = 25;
const int LEFT_IN1 = 26;
const int LEFT_IN2 = 27;
const int RIGHT_PWM = 14;
const int RIGHT_IN1 = 12;
const int RIGHT_IN2 = 13;

// Encoders
const int LEFT_ENC_A = 19;
const int LEFT_ENC_B = 18;
const int RIGHT_ENC_A = 5;
const int RIGHT_ENC_B = 23;

// Channels for Drone
enum { CH_ROLL=0, CH_PITCH=1, CH_THROTTLE=2, CH_YAW=3, CH_AUX1=4, CH_AUX2=5, CH_AUX3=6, CH_AUX4=7 };

volatile uint16_t channels[CHANNELS] = {
  1520,
  1500 + PITCH_TRIM,
  1000,
  1500,
  1000,
  2000,
  1000,
  1000
};

bool armed = false;
int16_t currentThrottle = 1000;
int16_t wheelSpeed = 0;

// Encoder state
volatile int leftDuration = 0;
volatile int rightDuration = 0;
volatile boolean leftDirection = true;
volatile boolean rightDirection = true;
volatile byte leftPinALast = LOW;
volatile byte rightPinALast = LOW;

bool isTraveling = false;
int travelStartLeft = 0;
int travelStartRight = 0;
float targetDistance = TARGET_DISTANCE_CM;
int16_t travelDirection = 0;

enum UprightState { U_IDLE, U_ARMING, U_THROTTLE_UP, U_DONE };
UprightState uprightState = U_IDLE;
unsigned long uprightStateTime = 0;

// Jump state
enum JumpState { J_IDLE, J_JUMPING, J_HOLDING, J_RAMPING };
JumpState jumpState = J_IDLE;
unsigned long jumpStateTime = 0;
unsigned long lastRampTime = 0;

hw_timer_t *timer = NULL;
volatile int currentChannel = 0;
volatile bool isPulse = true;
volatile uint32_t frameUsed = 0;

MPU9250 mpu(MPU9250_ADDRESS_AD0, Wire, 400000);
bool imuInitialized = false;
float pitch = 0.0f;
float roll = 0.0f;
float pitchOffset = 0.0f;
float rollOffset = 0.0f;

// Gyro data
float gyroZ = 0.0f;
float gyroZOffset = 0.0f;

unsigned long lastImuUpdate = 0;
const unsigned long IMU_UPDATE_INTERVAL = 20;

// Failsafe state
unsigned long lastHeartbeat = 0;
bool failsafeTriggered = false;

// Safety
bool safetyKill = false;

// Encoder Interrupts
void IRAM_ATTR leftEncoderISR() {
int Lstate = digitalRead(LEFT_ENC_A);
if((leftPinALast == LOW) && Lstate == HIGH) {
int val = digitalRead(LEFT_ENC_B);
if(val == LOW && leftDirection) {
leftDirection = false;
} else if(val == HIGH && !leftDirection) {
  leftDirection = true;
}
}
leftPinALast = Lstate;

if(!leftDirection) leftDuration++;
else leftDuration--;
}

void IRAM_ATTR rightEncoderISR() {
int Lstate = digitalRead(RIGHT_ENC_A);
if((rightPinALast == LOW) && Lstate == HIGH) {
int val = digitalRead(RIGHT_ENC_B);
if(val == LOW && rightDirection) {
rightDirection = false;
} else if(val == HIGH && !rightDirection) {
rightDirection = true;
}
}
rightPinALast = Lstate;

if(rightDirection) rightDuration++;
else rightDuration--;
}

int getLeftEncoderCounts() {
noInterrupts();
int c = leftDuration;
interrupts();
return c;
}

int getRightEncoderCounts() {
noInterrupts();
int c = rightDuration;
interrupts();
return c;
}

void resetEncoders() {
noInterrupts();
leftDuration = 0;
rightDuration = 0;
interrupts();
}

// Convert to Distance
float countsToDistance(int counts) {
return (float)((abs(counts) * 3.14159 * WHEEL_DIAMETER_CM) / ENCODER_CPR);
}

float getLeftDistanceCm() {
return countsToDistance(getLeftEncoderCounts());
}

float getRightDistanceCm() {
return countsToDistance(getRightEncoderCounts());
}

// PPM sending based on timer
void IRAM_ATTR onTimer() {
if (isPulse) {
digitalWrite(PPM_PIN, HIGH);
timerAlarmWrite(timer, PPM_PULSE_LENGTH, true);
frameUsed += PPM_PULSE_LENGTH;
} else {
digitalWrite(PPM_PIN, LOW);

if (currentChannel < CHANNELS) {
uint32_t gap = channels[currentChannel] - PPM_PULSE_LENGTH;
timerAlarmWrite(timer, gap, true);
frameUsed += gap;
currentChannel++;
} else {
uint32_t syncGap = PPM_FRAME_LENGTH - frameUsed;
timerAlarmWrite(timer, syncGap, true);
currentChannel = 0;
frameUsed = 0;
}
}
isPulse = !isPulse;
}

// Motor Control
void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
if (leftSpeed > 0) {
digitalWrite(LEFT_IN1, LOW);
digitalWrite(LEFT_IN2, HIGH);
} else if (leftSpeed < 0) {
digitalWrite(LEFT_IN1, HIGH);
digitalWrite(LEFT_IN2, LOW);
} else {
digitalWrite(LEFT_IN1, LOW);
digitalWrite(LEFT_IN2, LOW);
}
analogWrite(LEFT_PWM, abs(leftSpeed));

if (rightSpeed > 0) {
digitalWrite(RIGHT_IN1, LOW);
digitalWrite(RIGHT_IN2, HIGH);
} else if (rightSpeed < 0) {
digitalWrite(RIGHT_IN1, HIGH);
digitalWrite(RIGHT_IN2, LOW);
} else {
digitalWrite(RIGHT_IN1, LOW);
digitalWrite(RIGHT_IN2, LOW);
}
analogWrite(RIGHT_PWM, abs(rightSpeed));
}

// IMU Functions
void readRawAngles(float &p, float &r) {
mpu.readAccelData(mpu.accelCount);
mpu.getAres();

float ax = (float)mpu.accelCount[0] * mpu.aRes;
float ay = (float)mpu.accelCount[1] * mpu.aRes;
float az = (float)mpu.accelCount[2] * mpu.aRes;

p = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
r = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;
}

// Read gyro Z axis (yaw rate in degrees/sec)
float readGyroZ() {
mpu.readGyroData(mpu.gyroCount);
mpu.getGres();
return (float)mpu.gyroCount[2] * mpu.gRes - gyroZOffset;
}

void calibrateIMU() {
Serial.println("Calibrating.....");

float pitchSum = 0;
float rollSum = 0;
float gyroZSum = 0;

for (int i = 0; i < 100; i++) {
float p, r;
readRawAngles(p, r);
pitchSum += p;
rollSum += r;

// Read raw gyro for calibration
mpu.readGyroData(mpu.gyroCount);
mpu.getGres();
gyroZSum += (float)mpu.gyroCount[2] * mpu.gRes;

delay(10);
}

pitchOffset = pitchSum / 100.0f;
rollOffset = rollSum / 100.0f;
gyroZOffset = gyroZSum / 100.0f;

Serial.print("Gyro Z offset: ");
Serial.println(gyroZOffset);
Serial.println("Calibration done");
}

bool imuInit() {
Serial.println("IMU init...");

Wire.begin(SDA_PIN, SCL_PIN);
delay(100);

uint8_t whoami = mpu.readByte(MPU9250_ADDRESS_AD0, WHO_AM_I_MPU9250);
if (whoami != 0x71 && whoami != 0x73 && whoami != 0x68) {
Serial.println("IMU FAILED");
return false;
}

mpu.initMPU9250();
mpu.writeByte(MPU9250_ADDRESS_AD0, CONFIG, 0x03);

Serial.println("IMU OK");
imuInitialized = true;

delay(500);
calibrateIMU();

return true;
}

void imuUpdate() {
if (!imuInitialized) return;

float rawPitch, rawRoll;
readRawAngles(rawPitch, rawRoll);

pitch = rawPitch - pitchOffset;
roll = rawRoll - rollOffset;

// Read gyro Z for turn tracking
gyroZ = readGyroZ();
}

// Update roll channel with current trim
void updateRollTrim() {
channels[CH_ROLL] = 1500 + rollTrim;
Serial.print("Roll trim: ");
Serial.println(rollTrim);
}

// Adjust roll trim
void adjustRollTrim(int delta) {
rollTrim += delta;
rollTrim = constrain(rollTrim, -100, 100);
updateRollTrim();
}

// Wifi disconnect failsafe
void failsafeCheck() {
if (channels[CH_AUX1] < 1500) {
failsafeTriggered = false;
return;
}

if (millis() - lastHeartbeat > FAILSAFE_TIMEOUT_MS) {
failsafeTriggered = true;
Serial.println("Failsafe, WIFI went out :(");

channels[CH_AUX1] = 1000;
channels[CH_THROTTLE] = 1000;
channels[CH_ROLL] = 1500 + rollTrim;
channels[CH_PITCH] = 1500 + PITCH_TRIM;
armed = false;
currentThrottle = 1000;
setMotorSpeeds(0, 0);
isTraveling = false;
isTurning = false;
uprightState = U_IDLE;
jumpState = J_IDLE;
}
}

// Upright
void startUprightSequence() {
if (uprightState != U_IDLE) return;
Serial.println("Upright begun");

uprightState = U_ARMING;
uprightStateTime = millis();
}

void updateUprightSequence() {
unsigned long elapsed = millis() - uprightStateTime;

switch (uprightState) {
case U_IDLE:
break;

case U_ARMING:
Serial.println("Step 1: Arming flight controller");
armed = true;
channels[CH_AUX1] = 2000;

uprightState = U_THROTTLE_UP;
uprightStateTime = millis();
break;

case U_THROTTLE_UP:
if (elapsed >= 500) {
Serial.println("Step 2: Throttle to 1300");
currentThrottle = UPRIGHT_THROTTLE;
channels[CH_THROTTLE] = currentThrottle;

resetEncoders();
Serial.println("Step 3: Encoders reset");

uprightState = U_DONE;
uprightStateTime = millis();
}
break;

case U_DONE:
if (elapsed >= 500) {
Serial.println("Upright Complete");
uprightState = U_IDLE;
}
break;
}
}

// Jump function
void startJump() {
if (!armed || jumpState != J_IDLE || uprightState != U_IDLE || isTurning) return;

Serial.println("JUMP started!");
jumpState = J_JUMPING;
jumpStateTime = millis();
}

void updateJump() {
if (jumpState == J_IDLE) return;

unsigned long now = millis();
unsigned long elapsed = now - jumpStateTime;

switch (jumpState) {
case J_IDLE:
break;

case J_JUMPING:
currentThrottle = JUMP_THROTTLE;
channels[CH_THROTTLE] = currentThrottle;
channels[CH_PITCH] = 1500 + PITCH_TRIM - JUMP_PITCH;
channels[CH_ROLL] = 1500 + rollTrim;
Serial.print("Jump throttle: ");
Serial.print(currentThrottle);
Serial.print(" Pitch: ");
Serial.print(channels[CH_PITCH]);
Serial.print(" Roll: ");
Serial.println(channels[CH_ROLL]);

jumpState = J_HOLDING;
jumpStateTime = now;
break;

case J_HOLDING:
channels[CH_ROLL] = 1500 + rollTrim;

if (elapsed >= JUMP_HOLD_TIME) {
Serial.println("Jump hold complete, ramping down");
channels[CH_PITCH] = 1500 + PITCH_TRIM;
channels[CH_ROLL] = 1500 + rollTrim;
jumpState = J_RAMPING;
jumpStateTime = now;
lastRampTime = now;
}
break;

case J_RAMPING:
channels[CH_ROLL] = 1500 + rollTrim;

if (now - lastRampTime >= 1000) {
lastRampTime = now;
currentThrottle -= JUMP_RAMP_RATE;

if (currentThrottle <= HOVER_THROTTLE) {
currentThrottle = HOVER_THROTTLE;
channels[CH_THROTTLE] = currentThrottle;
Serial.println("Jump complete, at hover throttle");
jumpState = J_IDLE;
} else {
channels[CH_THROTTLE] = currentThrottle;
Serial.print("Ramping down: ");
Serial.println(currentThrottle);
}
}
break;
}
}

// Turn functions
void startTurn(int16_t direction) {
if (!armed || uprightState != U_IDLE || isTraveling || jumpState != J_IDLE || isTurning) return;

Serial.print("Turn ~30° ");
Serial.println(direction > 0 ? "RIGHT" : "LEFT");

// Reset encoders
resetEncoders();

turnDirection = direction;
turnTargetTicks = TURN_ENCODER_TICKS;
turnStartTime = millis();
isTurning = true;

Serial.print("Target ticks: ");
Serial.println(turnTargetTicks);
}

void updateTurn() {
if (!isTurning) return;

unsigned long now = millis();

// Safety timeout
if (now - turnStartTime > TURN_TIMEOUT) {
setMotorSpeeds(0, 0);
isTurning = false;
Serial.println("Turn TIMEOUT!");
return;
}

// Get encoder counts (absolute values)
int leftCount = abs(getLeftEncoderCounts());
int rightCount = abs(getRightEncoderCounts());
int avgCount = (leftCount + rightCount) / 2;

// Debug every 200ms
static unsigned long lastDebug = 0;
if (now - lastDebug > 200) {
lastDebug = now;
Serial.print("L:");
Serial.print(leftCount);
Serial.print(" R:");
Serial.print(rightCount);
Serial.print(" Avg:");
Serial.print(avgCount);
Serial.print("/");
Serial.println(turnTargetTicks);
}

// Check if turn complete
if (avgCount >= turnTargetTicks) {
setMotorSpeeds(0, 0);
isTurning = false;

Serial.print("Turn done! L:");
Serial.print(leftCount);
Serial.print(" R:");
Serial.println(rightCount);
return;
}

// Speed matching - keep wheels at same speed
int countDiff = leftCount - rightCount;
float correction = countDiff * TURN_STEERING_KP;

// Opposite directions for in-place turn
// Right turn (dir=1): left forward, right backward
// Left turn (dir=-1): left backward, right forward
int16_t leftSpeed = turnDirection * TURN_SPEED - (int16_t)correction;
int16_t rightSpeed = -turnDirection * TURN_SPEED + (int16_t)correction;

leftSpeed = constrain(leftSpeed, -255, 255);
rightSpeed = constrain(rightSpeed, -255, 255);

setMotorSpeeds(leftSpeed, rightSpeed);
}

// Safety checker
void safetyCheck() {
if (channels[CH_AUX1] < 1500 || channels[CH_THROTTLE] < 1100) {
safetyKill = false;
return;
}

if (abs(pitch) > ANGLE_LIMIT || abs(roll) > ANGLE_LIMIT) {
safetyKill = true;
Serial.println("Safety Kill, Angle Exceeded");

channels[CH_AUX1] = 1000;
channels[CH_THROTTLE] = 1000;
channels[CH_ROLL] = 1500 + rollTrim;
channels[CH_PITCH] = 1500 + PITCH_TRIM;
armed = false;
currentThrottle = 1000;
setMotorSpeeds(0, 0);
isTraveling = false;
isTurning = false;
uprightState = U_IDLE;
jumpState = J_IDLE;
}
}

// Arming and Disarming
void armSystem() {
if (!armed && !failsafeTriggered && !safetyKill) {
Serial.println("ARMING");
armed = true;
channels[CH_AUX1] = 2000;
channels[CH_THROTTLE] = 1000;
currentThrottle = 1000;
delay(100);
}
}

void disarmSystem() {
Serial.println("DISARM");
armed = false;
currentThrottle = 1000;
wheelSpeed = 0;
channels[CH_AUX1] = 1000;
channels[CH_THROTTLE] = 1000;
channels[CH_PITCH] = 1500 + PITCH_TRIM;
channels[CH_ROLL] = 1500 + rollTrim;
setMotorSpeeds(0, 0);
uprightState = U_IDLE;
isTraveling = false;
isTurning = false;
jumpState = J_IDLE;
}

// Throttle Control
void adjustThrottle(int16_t delta) {
if (armed && uprightState == U_IDLE && !isTraveling && jumpState == J_IDLE && !isTurning) {
currentThrottle += delta;
currentThrottle = constrain(currentThrottle, 1000, 1750);
channels[CH_THROTTLE] = currentThrottle;
Serial.print("Throttle: ");
Serial.println(currentThrottle);
}
}

// Traveling Control
void startTravel(int16_t direction) {
if (!armed || uprightState != U_IDLE || isTraveling || jumpState != J_IDLE || isTurning) return;

Serial.print("Travel 50cm ");
Serial.println(direction > 0 ? "FWD" : "BWD");

resetEncoders();

travelStartLeft = 0;
travelStartRight = 0;
targetDistance = TARGET_DISTANCE_CM;
travelDirection = direction;
isTraveling = true;

Serial.print("Target: ");
Serial.print(targetDistance);
Serial.println(" cm");
}

void updateTravel() {
if (!isTraveling) return;

float distL = getLeftDistanceCm();
float distR = getRightDistanceCm();
float distTraveled = (distL + distR) / 2.0;

static unsigned long lastDebug = 0;
if (millis() - lastDebug > 200) {
lastDebug = millis();
Serial.print("L:");
Serial.print(distL, 2);
Serial.print("cm R:");
Serial.print(distR, 2);
Serial.print("cm Avg:");
Serial.print(distTraveled, 2);
Serial.println("cm");
}

if (distTraveled >= targetDistance) {
setMotorSpeeds(0, 0);
wheelSpeed = 0;
isTraveling = false;

Serial.print("Complete! L:");
Serial.print(distL, 2);
Serial.print("cm R:");
Serial.print(distR, 2);
Serial.println("cm");
return;
}

float distError = distL - distR;
float correction = distError * STEERING_KP;

int16_t baseSpeed = travelDirection * TRAVEL_SPEED;
int16_t leftSpeed = baseSpeed - (int16_t)correction;
int16_t rightSpeed = baseSpeed + (int16_t)correction;

leftSpeed = constrain(leftSpeed, -255, 255);
rightSpeed = constrain(rightSpeed, -255, 255);

setMotorSpeeds(leftSpeed, rightSpeed);
}

// Webpage
String getHTML() {
bool a = channels[CH_AUX1] > 1500;

String h = "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width\">";
h += "<style>body{background:#000;color:#0f0;font:32px monospace;text-align:center}</style></head><body>";

if (failsafeTriggered) h += "<p style=\"color:#ff0\">FAIL</p>";
if (safetyKill) h += "<p style=\"color:#f00\">KILL</p>";
if (jumpState != J_IDLE) h += "<p style=\"color:#0ff\">JUMP</p>";
if (isTurning) h += "<p style=\"color:#f0f\">TURN</p>";

h += "<p id=\"arm\" style=\"color:" + String(a?"#f00":"#0f0") + "\">" + String(a?"ARM":"DIS") + "</p>";
h += "<p id=\"thr\">" + String(currentThrottle) + "</p>";
h += "<p id=\"trim\">T:" + String(rollTrim) + "</p>";
h += "<p id=\"h\">-</p>";

h += "<script>";
h += "setInterval(()=>fetch('/h').then(r=>document.getElementById('h').innerText='OK').catch(()=>document.getElementById('h').innerText='X'),500);";

h += "setInterval(()=>fetch('/s').then(r=>r.text()).then(t=>{";
h += "let p=t.split(',');";
h += "document.getElementById('arm').innerText=p[0];";
h += "document.getElementById('arm').style.color=p[0]=='ARM'?'#f00':'#0f0';";
h += "document.getElementById('thr').innerText=p[1];";
h += "if(p[2])document.getElementById('trim').innerText='T:'+p[2];";
h += "}),500);";

h += "document.onkeydown=e=>{let c='';let k=e.key.toLowerCase();";
h += "if(k=='u')c='u';if(k=='a')c='a';if(k=='d')c='d';if(k=='r')c='r';if(k=='w')c='w';if(k=='s')c='s';";
h += "if(k==' ')c='j';";
h += "if(k=='q')c='q';if(k=='e')c='e';"; // Q = turn left, E = turn right
h += "if(e.key=='ArrowUp')c='f';if(e.key=='ArrowDown')c='b';";
h += "if(e.key=='ArrowLeft')c='l';if(e.key=='ArrowRight')c='t';";
h += "if(c)fetch('/c?c='+c);e.preventDefault()};";
h += "</script></body></html>";

return h;
}

void handleRoot() {
lastHeartbeat = millis();
server.sendHeader("Cache-Control", "no-cache");
server.send(200, "text/html", getHTML());
}

void handleCmd() {
lastHeartbeat = millis();
String c = server.arg("c");

if (c == "u") startUprightSequence();
else if (c == "a") armSystem();
else if (c == "d") disarmSystem();
else if (c == "r") {
failsafeTriggered = false;
safetyKill = false;
}
else if (c == "w") adjustThrottle(THROTTLE_STEP);
else if (c == "s") adjustThrottle(-THROTTLE_STEP);
else if (c == "f") startTravel(1);
else if (c == "b") startTravel(-1);
else if (c == "j") startJump();
else if (c == "l") adjustRollTrim(1);
else if (c == "t") adjustRollTrim(-1);
else if (c == "q") startTurn(-1); // Q = turn left
else if (c == "e") startTurn(1); // E = turn right

server.send(200, "text/plain", "K");
}

void handleHeartbeat() {
lastHeartbeat = millis();
server.send(200, "text/plain", "K");
}

void handleStatus() {
lastHeartbeat = millis();
bool a = channels[CH_AUX1] > 1500;
String status = String(a ? "ARM" : "DIS") + "," + String(currentThrottle) + "," + String(rollTrim);
server.send(200, "text/plain", status);
}

// Setup
void setup() {
Serial.begin(115200);
delay(500);

Serial.println("\nDrone robot");

// Motors
pinMode(LEFT_PWM, OUTPUT);
pinMode(LEFT_IN1, OUTPUT);
pinMode(LEFT_IN2, OUTPUT);
pinMode(RIGHT_PWM, OUTPUT);
pinMode(RIGHT_IN1, OUTPUT);
pinMode(RIGHT_IN2, OUTPUT);
setMotorSpeeds(0, 0);

// Encoders
pinMode(LEFT_ENC_A, INPUT);
pinMode(LEFT_ENC_B, INPUT);
pinMode(RIGHT_ENC_A, INPUT);
pinMode(RIGHT_ENC_B, INPUT);

attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

// PPM
pinMode(PPM_PIN, OUTPUT);
digitalWrite(PPM_PIN, LOW);
timer = timerBegin(0, 80, true);
timerAttachInterrupt(timer, &onTimer, true);
timerAlarmWrite(timer, PPM_PULSE_LENGTH, true);
timerAlarmEnable(timer);

// IMU
imuInit();

// Initialize roll trim
updateRollTrim();

// WIFI
WiFi.mode(WIFI_AP);
WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, WIFI_MAX_CONNECTIONS);
WiFi.setTxPower(WIFI_POWER);
esp_wifi_set_ps(WIFI_PS_NONE);

Serial.print("WiFi: ");
Serial.println(ssid);
Serial.print("IP: ");
Serial.println(WiFi.softAPIP());

server.on("/", handleRoot);
server.on("/c", handleCmd);
server.on("/h", handleHeartbeat);
server.on("/s", handleStatus);
server.begin();

lastHeartbeat = millis();

Serial.println("Ready at 192.168.4.1");
}

void loop() {
unsigned long now = millis();

// IMU 50Hz
if (now - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
lastImuUpdate = now;
imuUpdate();
safetyCheck();
}

// Always apply roll trim when armed
if (armed) {
channels[CH_ROLL] = 1500 + rollTrim;
}

// Failsafe check
failsafeCheck();

// Update sequences
updateUprightSequence();
updateTravel();
updateTurn();
updateJump();

// Web server
server.handleClient();
}
