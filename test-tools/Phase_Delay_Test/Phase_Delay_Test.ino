/*
  実験B: Madgwickフィルタ位相遅れ測定
  =====================================
  ジャイロ生値とMadgwick出力の時間差を測定する。
  モーター不要。手で傾けてデータを取る。

  手順:
    1. 書き込み
    2. シリアルモニター(115200)を開く
    3. 機体を水平に5秒間置いてフィルタを安定させる
    4. roll方向に素早く傾けて戻す（5回繰り返す）
    5. CSVデータを ~/halo-experiments/csv/ に保存
    6. analyze_experiment.py で位相差を分析

  CSV format: timestamp_ms,GyroX,GyroY,roll_IMU,pitch_IMU,roll_acc,pitch_acc
  roll_acc/pitch_acc = 加速度計から直接計算した角度（フィルタなし、比較用）
*/

#include <Wire.h>
#include <MadgwickAHRS.h>
#include <MPU6050.h>
#include <Arduino.h>

#define LOOP_TIMING 4000  // 4ms = 250Hz (same as flight controller)
#define SAMPLE_INTERVAL_MS 5  // CSV output at 200Hz

MPU6050 mpu;
Madgwick MadgwickFilter;

// IMU calibration (same as flight controller)
float AccErrorX = 0.03;
float AccErrorY = 0.02;
float AccErrorZ = 0.01;
float GyroErrorX = 0.9;
float GyroErrorY = -0.40;
float GyroErrorZ = 0.0;
float RollError = -8.0;
float PitchError = 4.0;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float roll_IMU, pitch_IMU;
float roll_acc, pitch_acc;

unsigned long previousMicros;
unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Phase Delay Test (Experiment B) ===");

  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 not found");
    while (1) delay(1000);
  }
  Serial.println("[OK] MPU6050");

  MadgwickFilter.begin(250);
  Serial.println("[OK] Madgwick Filter (250Hz)");

  Serial.println();
  Serial.println("Hold drone level for 5 seconds, then tilt quickly on roll axis.");
  Serial.println("Repeat 5 times. Save CSV output for analysis.");
  Serial.println();

  delay(1000);
  Serial.println("# CSV_START");
  Serial.println("timestamp_ms,GyroX,GyroY,roll_IMU,pitch_IMU,roll_acc,pitch_acc");

  previousMicros = micros();
}

void loop() {
  unsigned long currentMicros = micros();

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

    // Accelerometer-only angle (no filter, instant but noisy)
    roll_acc = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI;
    pitch_acc = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

    // Madgwick filter (same axis mapping as flight controller)
    MadgwickFilter.updateIMU(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ);
    roll_IMU = MadgwickFilter.getRoll() - RollError;
    pitch_IMU = -MadgwickFilter.getPitch() - PitchError;

    // Output CSV at sampling rate
    if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
      lastSampleTime = millis();

      Serial.print(millis());
      Serial.print(",");
      Serial.print(GyroX, 2);
      Serial.print(",");
      Serial.print(GyroY, 2);
      Serial.print(",");
      Serial.print(roll_IMU, 2);
      Serial.print(",");
      Serial.print(pitch_IMU, 2);
      Serial.print(",");
      Serial.print(roll_acc, 2);
      Serial.print(",");
      Serial.println(pitch_acc, 2);
    }
  }
}
