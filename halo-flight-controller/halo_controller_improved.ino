// Halo Drone Flight Controller - Improved Version
// Based on halo_contoroller_241201.ino with critical fixes and optimizations

#include <Wire.h>
#include <MadgwickAHRS.h>
#include <MPU6050.h>
#include <Arduino.h>

// ============================================
// Configuration Constants
// ============================================

// PPM Receiver
#define CHANNELS 6
#define SYNC_GAP 3000
#define PPM_PIN 4

// Motor Pins
#define m1Pin 16  // Front Left
#define m2Pin 17  // Front Right
#define m3Pin 18  // Back Right
#define m4Pin 19  // Back Left

// PWM Settings
const int pwmFrequency = 50;
const int pwmResolution = 16;
const int throttle_max = 1923;
const int throttle_min = 900;

// Motor Offsets (Calibration values for each motor)
const int MOTOR1_OFFSET = 1040;
const int MOTOR2_OFFSET = 980;
const int MOTOR3_OFFSET = 980;
const int MOTOR4_OFFSET = 910;

// Timing
#define LOOP_TIMING 4000  // 4ms = 250Hz loop rate (was 100Hz)
#define PRINT_INTERVAL 100  // Print debug info every 100ms

// Safety
#define THROTTLE_CUTOFF 1050  // Below this throttle, motors stop
#define SIGNAL_TIMEOUT 1000   // 1 second timeout for receiver signal

// ============================================
// Cascade PID Parameters
// ============================================
// Outer loop: angle error (deg) → desired rate (deg/s)
// Inner loop: rate error (deg/s) → motor command (normalized)

// Integrator Limits
float rate_i_limit = 0.15;

// Angle Mode Limits
float maxRoll = 18.0;   // degrees
float maxPitch = 18.0;  // degrees
float maxYaw = 160.0;   // deg/sec

// Motor Throttle Limits
float maxMotor = 0.8;
float throttleScale = 0.6;

// Outer Loop - Angle P (Roll/Pitch)
float Kp_roll_angle = 4.0;   // deg → deg/s
float Kp_pitch_angle = 4.0;

// Inner Loop - Rate PID (Roll)
float Kp_roll_rate = 0.0012;
float Ki_roll_rate = 0.0001;
float Kd_roll_rate = 0.000015;

// Inner Loop - Rate PID (Pitch)
float Kp_pitch_rate = 0.0012;
float Ki_pitch_rate = 0.0001;
float Kd_pitch_rate = 0.000015;

// Yaw Rate PID
float Kp_yaw = 0.003;
float Ki_yaw = 0.0005;
float Kd_yaw = 0.0;

// Stick Input Filtering
float stick_dampener = 0.3;

// ============================================
// IMU Calibration Values
// ============================================

float AccErrorX = 0.03;
float AccErrorY = 0.02;
float AccErrorZ = 0.01;
float GyroErrorX = 0.9;
float GyroErrorY = -0.40;
float GyroErrorZ = 0.0;

float RollError = -8.0;
float PitchError = 4.0;


// ============================================
// Global Objects
// ============================================

MPU6050 mpu;
Madgwick MadgwickFilter;

// ============================================
// PPM Receiver Variables
// ============================================

volatile unsigned long lastPulseTime = 0;
volatile int channelValues[CHANNELS] = {0};
volatile int currentChannel = 0;
volatile unsigned long lastValidSignal = 0;

// ============================================
// IMU Variables
// ============================================

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float roll_IMU, pitch_IMU;

// ============================================
// Controller Variables
// ============================================

float thro_des, roll_des, pitch_des, yaw_des;

// Outer loop (angle → desired rate)
float desired_rate_roll, desired_rate_pitch;

// Inner loop (rate PID)
float error_rate_roll, integral_rate_roll = 0, prev_error_rate_roll = 0, roll_PID = 0;
float error_rate_pitch, integral_rate_pitch = 0, prev_error_rate_pitch = 0, pitch_PID = 0;
float error_yaw, integral_yaw = 0, prev_error_yaw = 0, yaw_PID = 0;

// ============================================
// Motor Command Variables
// ============================================

float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

// ============================================
// Radio Receiver Variables
// ============================================

unsigned long PWM_throttle, PWM_roll, PWM_Elevation, PWM_Rudd;
unsigned long PWM_throttle_prev, PWM_roll_prev, PWM_Elevation_prev, PWM_Rudd_prev;

// ============================================
// Timing Variables
// ============================================

unsigned long currentMicros, previousMicros;
unsigned long lastPrintTime = 0;
float deltaTime = 0.004;  // 4ms = 250Hz

bool motorsArmed = false;

// ============================================
// PPM Interrupt Handler
// ============================================

void IRAM_ATTR ppmInterrupt() {
  unsigned long pulseTime = micros();
  unsigned long pulseWidth = pulseTime - lastPulseTime;
  lastPulseTime = pulseTime;

  if (pulseWidth > SYNC_GAP) {
    currentChannel = 0;
    lastValidSignal = millis();  // Update last valid signal time
  } else {
    if (currentChannel < CHANNELS) {
      channelValues[currentChannel] = pulseWidth;
      currentChannel++;
    }
  }
}

// ============================================
// Setup Function
// ============================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== Halo Drone Flight Controller ===");
  Serial.println("Version: Improved v1.0");

  // Initialize PPM Receiver
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(PPM_PIN, ppmInterrupt, RISING);
  Serial.println("[OK] PPM Receiver Initialized");

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] MPU6050 connection failed!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("[OK] MPU6050 Connected");

  // Initialize Madgwick Filter (250Hz update rate)
  MadgwickFilter.begin(250);
  Serial.println("[OK] Madgwick Filter Initialized (250Hz)");

  // Initialize PWM for Motors
  if (!ledcAttach(m1Pin, pwmFrequency, pwmResolution)) {
    Serial.println("[ERROR] Failed to attach PWM to motor 1");
  }
  if (!ledcAttach(m2Pin, pwmFrequency, pwmResolution)) {
    Serial.println("[ERROR] Failed to attach PWM to motor 2");
  }
  if (!ledcAttach(m3Pin, pwmFrequency, pwmResolution)) {
    Serial.println("[ERROR] Failed to attach PWM to motor 3");
  }
  if (!ledcAttach(m4Pin, pwmFrequency, pwmResolution)) {
    Serial.println("[ERROR] Failed to attach PWM to motor 4");
  }
  Serial.println("[OK] PWM Attached to All Motors");

  // Set all motors to minimum throttle
  setMotorPWM(throttle_min, throttle_min, throttle_min, throttle_min);
  Serial.println("[OK] Motors Initialized to Minimum Throttle");

  Serial.println("\n[READY] Flight Controller Initialized");
  Serial.println("Waiting for valid receiver signal...\n");

  delay(1000);
  previousMicros = micros();
}

// ============================================
// Main Loop
// ============================================

void loop() {
  currentMicros = micros();

  // Calculate delta time
  deltaTime = (currentMicros - previousMicros) / 1000000.0;

  // Only run control loop at specified rate
  if (currentMicros - previousMicros >= LOOP_TIMING) {
    previousMicros = currentMicros;

    loopDrone();

    // Print debug info at reduced rate
    if (millis() - lastPrintTime >= PRINT_INTERVAL) {
      printDebugInfo();
      lastPrintTime = millis();
    }
  }
}

// ============================================
// Main Drone Control Loop
// ============================================

void loopDrone() {
  // 1. Get radio receiver inputs FIRST
  getRadioSticks();

  // 2. Check for signal loss
  if (millis() - lastValidSignal > SIGNAL_TIMEOUT) {
    // Signal lost - cut throttle and reset PID
    motorsArmed = false;
    resetPID();
    setMotorPWM(throttle_min, throttle_min, throttle_min, throttle_min);
    return;
  }

  // 3. Get IMU data
  getIMUdata();

  // 4. Update attitude estimation
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);

  // 5. Convert receiver inputs to desired angles/rates
  getDesiredAnglesAndThrottle();

  // 6. Calculate PID outputs
  PIDControlCalcs();

  // 7. Mix PID outputs to motor commands
  controlMixer();

  // 8. Scale commands to PWM values
  scaleCommands();

  // 9. Send commands to motors
  commandMotors();
}

// ============================================
// Helper Functions
// ============================================

int getChannelValue(int channelIndex) {
  if (channelIndex >= 0 && channelIndex < CHANNELS) {
    return channelValues[channelIndex];
  }
  return 1500;  // Return neutral value if invalid
}

void getRadioSticks() {
  // Get raw PWM values from receiver
  PWM_throttle = getChannelValue(2);
  PWM_roll = getChannelValue(0);
  PWM_Elevation = getChannelValue(1);
  PWM_Rudd = getChannelValue(3);

  // Apply low-pass filter to smooth inputs
  // Throttle: slower descent, faster ascent
  if (PWM_throttle < PWM_throttle_prev) {
    PWM_throttle = 0.95 * PWM_throttle_prev + 0.05 * PWM_throttle;
  } else {
    PWM_throttle = stick_dampener * PWM_throttle_prev + (1.0 - stick_dampener) * PWM_throttle;
  }

  // Other channels: standard filtering
  PWM_roll = (1.0 - stick_dampener) * PWM_roll_prev + stick_dampener * PWM_roll;
  PWM_Elevation = (1.0 - stick_dampener) * PWM_Elevation_prev + stick_dampener * PWM_Elevation;
  PWM_Rudd = (1.0 - stick_dampener) * PWM_Rudd_prev + stick_dampener * PWM_Rudd;

  // Update previous values
  PWM_throttle_prev = PWM_throttle;
  PWM_roll_prev = PWM_roll;
  PWM_Elevation_prev = PWM_Elevation;
  PWM_Rudd_prev = PWM_Rudd;
}

void getIMUdata() {
  int16_t ax, ay, az, gx, gy, gz;

  // Get raw data from MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to g and apply calibration offsets
  AccX = (ax / 16384.0) - AccErrorX;
  AccY = (ay / 16384.0) - AccErrorY;
  AccZ = (az / 16384.0) - AccErrorZ;

  // Convert to deg/s and apply calibration offsets
  GyroX = (gx / 131.0) - GyroErrorX;
  GyroY = (gy / 131.0) - GyroErrorY;
  GyroZ = (gz / 131.0) - GyroErrorZ;
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az) {
  MadgwickFilter.updateIMU(gx, gy, gz, ax, ay, az);
  roll_IMU = MadgwickFilter.getRoll() - RollError;
  pitch_IMU = -MadgwickFilter.getPitch() - PitchError;
}

void getDesiredAnglesAndThrottle() {
  // Convert PWM to normalized values
  thro_des = (PWM_throttle - 1000.0) / 1000.0;
  roll_des = (PWM_roll - 1500.0) / 500.0;
  pitch_des = (PWM_Elevation - 1500.0) / 500.0;
  yaw_des = (PWM_Rudd - 1500.0) / 500.0;

  // Apply constraints
  thro_des = constrain(thro_des, 0.0, 1.0) * throttleScale;
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch;
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;
}

void PIDControlCalcs() {
  // Safety: Reset PID when throttle is low
  if (PWM_throttle < THROTTLE_CUTOFF) {
    resetPID();
    motorsArmed = false;
    return;
  }

  motorsArmed = true;

  // === ROLL CASCADE ===
  // Outer loop: angle error → desired rate (uses Madgwick, slow but OK)
  float error_angle_roll = roll_des - roll_IMU;
  desired_rate_roll = Kp_roll_angle * error_angle_roll;

  // Inner loop: rate error → motor command (uses gyro direct, fast)
  error_rate_roll = desired_rate_roll - GyroX;
  integral_rate_roll += error_rate_roll * deltaTime;
  integral_rate_roll = constrain(integral_rate_roll, -rate_i_limit, rate_i_limit);
  float derivative_rate_roll = (error_rate_roll - prev_error_rate_roll) / deltaTime;
  roll_PID = Kp_roll_rate * error_rate_roll
           + Ki_roll_rate * integral_rate_roll
           + Kd_roll_rate * derivative_rate_roll;
  prev_error_rate_roll = error_rate_roll;

  // === PITCH CASCADE ===
  float error_angle_pitch = pitch_des - pitch_IMU;
  desired_rate_pitch = Kp_pitch_angle * error_angle_pitch;

  error_rate_pitch = desired_rate_pitch - GyroY;
  integral_rate_pitch += error_rate_pitch * deltaTime;
  integral_rate_pitch = constrain(integral_rate_pitch, -rate_i_limit, rate_i_limit);
  float derivative_rate_pitch = (error_rate_pitch - prev_error_rate_pitch) / deltaTime;
  pitch_PID = Kp_pitch_rate * error_rate_pitch
            + Ki_pitch_rate * integral_rate_pitch
            + Kd_pitch_rate * derivative_rate_pitch;
  prev_error_rate_pitch = error_rate_pitch;

  // === YAW (rate mode only, no cascade) ===
  error_yaw = yaw_des - GyroZ;
  integral_yaw += error_yaw * deltaTime;
  integral_yaw = constrain(integral_yaw, -rate_i_limit, rate_i_limit);
  float derivative_yaw = (error_yaw - prev_error_yaw) / deltaTime;
  yaw_PID = Kp_yaw * error_yaw
          + Ki_yaw * integral_yaw
          + Kd_yaw * derivative_yaw;
  prev_error_yaw = error_yaw;
}

void resetPID() {
  integral_rate_roll = 0;
  integral_rate_pitch = 0;
  integral_yaw = 0;
  prev_error_rate_roll = 0;
  prev_error_rate_pitch = 0;
  prev_error_yaw = 0;
  roll_PID = 0;
  pitch_PID = 0;
  yaw_PID = 0;
}

void controlMixer() {
  // Quad X configuration mixing (roll sign corrected)
  m1_command_scaled = maxMotor * thro_des - pitch_PID - roll_PID + yaw_PID;  // Front Left
  m2_command_scaled = maxMotor * thro_des - pitch_PID + roll_PID - yaw_PID;  // Front Right
  m3_command_scaled = maxMotor * thro_des + pitch_PID + roll_PID + yaw_PID;  // Back Right
  m4_command_scaled = maxMotor * thro_des + pitch_PID - roll_PID - yaw_PID;  // Back Left

  // Constrain to valid range
  m1_command_scaled = constrain(m1_command_scaled, 0.0, 1.0);
  m2_command_scaled = constrain(m2_command_scaled, 0.0, 1.0);
  m3_command_scaled = constrain(m3_command_scaled, 0.0, 1.0);
  m4_command_scaled = constrain(m4_command_scaled, 0.0, 1.0);
}

void scaleCommands() {
  // Scale normalized commands to PWM range
  m1_command_PWM = m1_command_scaled * throttle_max;
  m2_command_PWM = m2_command_scaled * throttle_max;
  m3_command_PWM = m3_command_scaled * throttle_max;
  m4_command_PWM = m4_command_scaled * throttle_max;
}

void commandMotors() {
  // Safety check
  if (!motorsArmed || PWM_throttle < THROTTLE_CUTOFF) {
    setMotorPWM(throttle_min, throttle_min, throttle_min, throttle_min);
    return;
  }

  // Apply motor-specific offsets
  m1_command_PWM += MOTOR1_OFFSET;
  m2_command_PWM += MOTOR2_OFFSET;
  m3_command_PWM += MOTOR3_OFFSET;
  m4_command_PWM += MOTOR4_OFFSET;

  // Send commands to motors
  setMotorPWM(m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM);
}

void setMotorPWM(int m1, int m2, int m3, int m4) {
  // Constrain to valid PWM range
  m1 = constrain(m1, throttle_min, throttle_max);
  m2 = constrain(m2, throttle_min, throttle_max);
  m3 = constrain(m3, throttle_min, throttle_max);
  m4 = constrain(m4, throttle_min, throttle_max);

  // Convert microseconds to duty cycle for 50Hz/16-bit PWM
  // 50Hz = 20000us period, 16-bit = 65536 steps
  // duty = microseconds * 65536 / 20000
  int duty1 = (int)((m1 * 65536L) / 20000);
  int duty2 = (int)((m2 * 65536L) / 20000);
  int duty3 = (int)((m3 * 65536L) / 20000);
  int duty4 = (int)((m4 * 65536L) / 20000);

  // Write to motors
  ledcWrite(m1Pin, duty1);
  ledcWrite(m2Pin, duty2);
  ledcWrite(m3Pin, duty3);
  ledcWrite(m4Pin, duty4);
}

// ============================================
// Debug Print Functions
// ============================================

void printDebugInfo() {
  float loopRate = 1.0 / deltaTime;
  Serial.print("Loop: ");
  Serial.print(loopRate);
  Serial.print("Hz | ");

  if (millis() - lastValidSignal > SIGNAL_TIMEOUT) {
    Serial.print("RX: LOST | ");
  } else {
    Serial.print("RX: OK | ");
  }

  Serial.print("Armed: ");
  Serial.print(motorsArmed ? "YES" : "NO");
  Serial.print(" | Thr: ");
  Serial.print((int)PWM_throttle);
  Serial.print(" | R: ");
  Serial.print(roll_IMU, 1);
  Serial.print(" P: ");
  Serial.print(pitch_IMU, 1);
  Serial.print(" | PID R:");
  Serial.print(roll_PID, 3);
  Serial.print(" P:");
  Serial.print(pitch_PID, 3);
  Serial.println();
}

