/*
  実験C/D: PIDチューニングテスト
  ===============================
  シリアルコマンドでPID値をリアルタイムに変更しながらステップ応答を測定する。
  1軸固定（roll軸）で使う。

  手順:
    1. roll軸のみ回転できるように固定
    2. プロペラ外した状態で書き込み
    3. シリアルモニター(115200)を開く
    4. コマンドでPID値を設定
    5. スロットルを上げて手で傾けて離す
    6. CSVデータで収束特性を確認

  コマンド:
    Kp3.0    -> Kp=3.0 に変更
    Kd1.5    -> Kd=1.5 に変更
    Ki0.01   -> Ki=0.01 に変更
    pid      -> 現在のPID値を表示
    arm      -> モーターアーム（スロットル50%固定）
    disarm   -> モーターディスアーム
    help     -> コマンド一覧

  CSV format: timestamp_ms,GyroX,roll_IMU,roll_PID,M1,M2,M3,M4
*/

#include <Wire.h>
#include <MadgwickAHRS.h>
#include <MPU6050.h>
#include <Arduino.h>

// Motor Pins (same as flight controller)
#define m1Pin 16  // Front Left
#define m2Pin 17  // Front Right
#define m3Pin 18  // Back Right
#define m4Pin 19  // Back Left

// PWM Settings
const int pwmFrequency = 50;
const int pwmResolution = 16;
const int throttle_max = 1923;
const int throttle_min = 900;

// Motor Offsets
const int MOTOR1_OFFSET = 1040;
const int MOTOR2_OFFSET = 980;
const int MOTOR3_OFFSET = 980;
const int MOTOR4_OFFSET = 910;

#define LOOP_TIMING 4000  // 250Hz
#define SAMPLE_INTERVAL_MS 10  // CSV at 100Hz

MPU6050 mpu;
Madgwick MadgwickFilter;

// PID gains (start conservative)
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.5;
float i_limit = 20.0;

// IMU calibration
float AccErrorX = 0.03;
float AccErrorY = 0.02;
float AccErrorZ = 0.01;
float GyroErrorX = 0.9;
float GyroErrorY = -0.40;
float GyroErrorZ = 0.0;
float RollError = -8.0;
float PitchError = 4.0;

// State
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float roll_IMU, pitch_IMU;
float error_roll, integral_roll = 0, roll_PID = 0;
float thro_des = 0.3;  // Fixed throttle for testing

int m1_PWM, m2_PWM, m3_PWM, m4_PWM;
bool armed = false;

unsigned long previousMicros;
unsigned long lastSampleTime = 0;
float deltaTime = 0.004;

int usToduty(int us) {
  return (int)((us * 65536L) / 20000);
}

void setMotorPWM(int m1, int m2, int m3, int m4) {
  m1 = constrain(m1, throttle_min, throttle_max);
  m2 = constrain(m2, throttle_min, throttle_max);
  m3 = constrain(m3, throttle_min, throttle_max);
  m4 = constrain(m4, throttle_min, throttle_max);
  ledcWrite(m1Pin, usToduty(m1));
  ledcWrite(m2Pin, usToduty(m2));
  ledcWrite(m3Pin, usToduty(m3));
  ledcWrite(m4Pin, usToduty(m4));
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== PID Tuning Test (Experiment C/D) ===");

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 not found");
    while (1) delay(1000);
  }
  Serial.println("[OK] MPU6050");

  MadgwickFilter.begin(250);
  Serial.println("[OK] Madgwick Filter (250Hz)");

  ledcAttach(m1Pin, pwmFrequency, pwmResolution);
  ledcAttach(m2Pin, pwmFrequency, pwmResolution);
  ledcAttach(m3Pin, pwmFrequency, pwmResolution);
  ledcAttach(m4Pin, pwmFrequency, pwmResolution);
  setMotorPWM(throttle_min, throttle_min, throttle_min, throttle_min);
  Serial.println("[OK] Motors (OFF)");

  Serial.println();
  Serial.print("Initial PID: Kp="); Serial.print(Kp);
  Serial.print(" Ki="); Serial.print(Ki);
  Serial.print(" Kd="); Serial.println(Kd);
  Serial.println("Type 'help' for commands, 'arm' to start motors");
  Serial.println();

  delay(1000);
  previousMicros = micros();
}

void loop() {
  // Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("Kp")) {
      Kp = cmd.substring(2).toFloat();
      Serial.print("# Kp="); Serial.println(Kp);
    } else if (cmd.startsWith("Kd")) {
      Kd = cmd.substring(2).toFloat();
      Serial.print("# Kd="); Serial.println(Kd);
    } else if (cmd.startsWith("Ki")) {
      Ki = cmd.substring(2).toFloat();
      integral_roll = 0;  // Reset integral on Ki change
      Serial.print("# Ki="); Serial.println(Ki);
    } else if (cmd == "pid") {
      Serial.print("# PID: Kp="); Serial.print(Kp);
      Serial.print(" Ki="); Serial.print(Ki);
      Serial.print(" Kd="); Serial.println(Kd);
    } else if (cmd == "arm") {
      armed = true;
      integral_roll = 0;
      roll_PID = 0;
      Serial.println("# ARMED");
      Serial.println("# CSV_START");
      Serial.println("timestamp_ms,GyroX,roll_IMU,roll_PID,M1,M2,M3,M4");
    } else if (cmd == "disarm") {
      armed = false;
      setMotorPWM(throttle_min, throttle_min, throttle_min, throttle_min);
      integral_roll = 0;
      roll_PID = 0;
      Serial.println("# DISARMED");
    } else if (cmd == "help") {
      Serial.println("# Commands:");
      Serial.println("#   Kp<val>  Ki<val>  Kd<val>  - set PID gains");
      Serial.println("#   pid     - show current PID values");
      Serial.println("#   arm     - start motors (50% throttle + PID)");
      Serial.println("#   disarm  - stop motors");
    }
  }

  // Control loop at 250Hz
  unsigned long currentMicros = micros();
  deltaTime = (currentMicros - previousMicros) / 1000000.0;

  if (currentMicros - previousMicros >= LOOP_TIMING) {
    previousMicros = currentMicros;

    // Read IMU
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    AccX = (ax / 16384.0) - AccErrorX;
    AccY = (ay / 16384.0) - AccErrorY;
    AccZ = (az / 16384.0) - AccErrorZ;
    GyroX = (gx / 131.0) - GyroErrorX;
    GyroY = (gy / 131.0) - GyroErrorY;
    GyroZ = (gz / 131.0) - GyroErrorZ;

    // Madgwick
    MadgwickFilter.updateIMU(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);
    roll_IMU = MadgwickFilter.getRoll() - RollError;
    pitch_IMU = -MadgwickFilter.getPitch() - PitchError;

    if (armed) {
      // Roll PID (angle mode, same as flight controller)
      error_roll = 0.0 - roll_IMU;  // Target = level
      integral_roll += error_roll * deltaTime;
      integral_roll = constrain(integral_roll, -i_limit, i_limit);
      float derivative_roll = GyroX;
      roll_PID = 0.0001 * (Kp * error_roll + Ki * integral_roll - Kd * derivative_roll);

      // Mixer (roll only, same signs as flight controller)
      float maxMotor = 0.8;
      float m1_scaled = constrain(maxMotor * thro_des - roll_PID, 0.0, 1.0);
      float m2_scaled = constrain(maxMotor * thro_des + roll_PID, 0.0, 1.0);
      float m3_scaled = constrain(maxMotor * thro_des + roll_PID, 0.0, 1.0);
      float m4_scaled = constrain(maxMotor * thro_des - roll_PID, 0.0, 1.0);

      m1_PWM = (int)(m1_scaled * throttle_max) + MOTOR1_OFFSET;
      m2_PWM = (int)(m2_scaled * throttle_max) + MOTOR2_OFFSET;
      m3_PWM = (int)(m3_scaled * throttle_max) + MOTOR3_OFFSET;
      m4_PWM = (int)(m4_scaled * throttle_max) + MOTOR4_OFFSET;

      setMotorPWM(m1_PWM, m2_PWM, m3_PWM, m4_PWM);
    } else {
      m1_PWM = throttle_min;
      m2_PWM = throttle_min;
      m3_PWM = throttle_min;
      m4_PWM = throttle_min;
    }

    // CSV output
    if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
      lastSampleTime = millis();

      Serial.print(millis());
      Serial.print(",");
      Serial.print(GyroX, 2);
      Serial.print(",");
      Serial.print(roll_IMU, 2);
      Serial.print(",");
      Serial.print(roll_PID, 4);
      Serial.print(",");
      Serial.print(m1_PWM);
      Serial.print(",");
      Serial.print(m2_PWM);
      Serial.print(",");
      Serial.print(m3_PWM);
      Serial.print(",");
      Serial.println(m4_PWM);
    }
  }
}
