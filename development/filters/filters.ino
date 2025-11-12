#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

Adafruit_MPU6050 mpu;
Madgwick filter;

float rollMadgwick = 0, pitchMadgwick = 0;
float rollComplementary = 0, pitchComplementary = 0;
float rollNoFilter = 0, pitchNoFilter = 0;

unsigned long lastUpdate = 0; // フィルター更新タイミング
const float alpha = 0.70;     // 相補性フィルターの係数

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }

  Serial.println("MPU6050 initialized!");

  // センサ設定
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Madgwickフィルター初期化
  filter.begin(100); // 更新レートを100Hzに設定
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 加速度データ (m/s^2) を取得
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // ジャイロデータ (rad/s) を取得
  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  // フィルターなし
  rollNoFilter = atan2(ay, az) * RAD_TO_DEG;
  pitchNoFilter = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Madgwickフィルター
  unsigned long now = millis();
  float deltaTime = (now - lastUpdate) / 1000.0;
  lastUpdate = now;
  filter.updateIMU(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG, ax, ay, az);
  rollMadgwick = filter.getRoll();
  pitchMadgwick = filter.getPitch();

  // 相補性フィルター
  rollComplementary = alpha * (rollComplementary + gx * deltaTime * RAD_TO_DEG) + 
                      (1 - alpha) * rollNoFilter;
  pitchComplementary = alpha * (pitchComplementary + gy * deltaTime * RAD_TO_DEG) + 
                       (1 - alpha) * pitchNoFilter;

  // 結果をシリアル出力
  Serial.print("No Filter: ");
  Serial.print(rollNoFilter); Serial.print(", ");
  Serial.print(pitchNoFilter); Serial.print(" | ");

  Serial.print("Madgwick: ");
  Serial.print(rollMadgwick); Serial.print(", ");
  Serial.print(pitchMadgwick); Serial.print(" | ");

  Serial.print("Complementary: ");
  Serial.print(rollComplementary); Serial.print(", ");
  Serial.println(pitchComplementary);

  delay(10); // 100Hz更新
}
