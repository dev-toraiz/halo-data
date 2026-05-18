/*
  実験A: IMU振動ノイズ測定
  ========================
  モーター停止/回転時のMPU6050ノイズを測定する。
  CSV出力をシリアルで受信し、FFTで振動の周波数と大きさを分析する。

  手順:
    1. プロペラ外した状態で書き込み
    2. シリアルモニター(115200)を開く
    3. 10秒間データ取得（モーター停止）
    4. 's' を送信してモーター起動（50%スロットル）
    5. 10秒間データ取得
    6. 's' を送信してモーター停止
    7. CSVデータを ~/halo-experiments/csv/ に保存

  CSV format: timestamp_ms,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
*/

#include <Wire.h>
#include <MPU6050.h>

// Motor Pins (same as flight controller)
#define m1Pin 16
#define m2Pin 17
#define m3Pin 18
#define m4Pin 19

// PWM Settings
const int pwmFrequency = 50;
const int pwmResolution = 16;

// Sampling
#define SAMPLE_INTERVAL_MS 5  // 200Hz sampling

MPU6050 mpu;

bool motorsRunning = false;
unsigned long lastSampleTime = 0;

// IMU calibration (same as flight controller)
float AccErrorX = 0.03;
float AccErrorY = 0.02;
float AccErrorZ = 0.01;
float GyroErrorX = 0.9;
float GyroErrorY = -0.40;
float GyroErrorZ = 0.0;

int usToduty(int us) {
  return (int)((us * 65536L) / 20000);
}

void setAllMotors(int throttle_us) {
  int duty = usToduty(throttle_us);
  ledcWrite(m1Pin, duty);
  ledcWrite(m2Pin, duty);
  ledcWrite(m3Pin, duty);
  ledcWrite(m4Pin, duty);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== IMU Vibration Test (Experiment A) ===");

  // Init MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 not found");
    while (1) delay(1000);
  }
  Serial.println("[OK] MPU6050");

  // Init motor PWM
  ledcAttach(m1Pin, pwmFrequency, pwmResolution);
  ledcAttach(m2Pin, pwmFrequency, pwmResolution);
  ledcAttach(m3Pin, pwmFrequency, pwmResolution);
  ledcAttach(m4Pin, pwmFrequency, pwmResolution);
  setAllMotors(900);  // Motors off
  Serial.println("[OK] Motors initialized (OFF)");

  Serial.println();
  Serial.println("Commands:");
  Serial.println("  's' = toggle motors (50% throttle)");
  Serial.println("  'q' = quit (motors off)");
  Serial.println();
  Serial.println("# CSV_START");
  Serial.println("timestamp_ms,AccX,AccY,AccZ,GyroX,GyroY,GyroZ");
}

void loop() {
  // Process commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 's') {
      motorsRunning = !motorsRunning;
      if (motorsRunning) {
        setAllMotors(1500);  // 50% throttle
        Serial.println("# MOTORS ON (1500us)");
      } else {
        setAllMotors(900);
        Serial.println("# MOTORS OFF");
      }
    } else if (cmd == 'q') {
      setAllMotors(900);
      motorsRunning = false;
      Serial.println("# QUIT - Motors OFF");
    }
  }

  // Sample IMU at fixed rate
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = millis();

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float AccX = (ax / 16384.0) - AccErrorX;
    float AccY = (ay / 16384.0) - AccErrorY;
    float AccZ = (az / 16384.0) - AccErrorZ;
    float GyroX = (gx / 131.0) - GyroErrorX;
    float GyroY = (gy / 131.0) - GyroErrorY;
    float GyroZ = (gz / 131.0) - GyroErrorZ;

    Serial.print(millis());
    Serial.print(",");
    Serial.print(AccX, 4);
    Serial.print(",");
    Serial.print(AccY, 4);
    Serial.print(",");
    Serial.print(AccZ, 4);
    Serial.print(",");
    Serial.print(GyroX, 2);
    Serial.print(",");
    Serial.print(GyroY, 2);
    Serial.print(",");
    Serial.println(GyroZ, 2);
  }
}
