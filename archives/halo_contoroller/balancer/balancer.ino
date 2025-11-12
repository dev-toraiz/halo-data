#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

// MPU6050オブジェクトを作成
Adafruit_MPU6050 mpu;

// Madgwickフィルターオブジェクトを作成
Madgwick filter;
unsigned long lastUpdate = 0;
float deltat = 0.0f; // サンプリング周期

// 初期オフセット値
float initialRoll = 0.0f;
float initialPitch = 0.0f;
float initialYaw = 0.0f;

void setup() {
  // シリアル通信を開始
  Serial.begin(115200);

  // I2C通信を開始
  Wire.begin();

  // MPU6050を初期化
  if (!mpu.begin()) {
    Serial.println("MPU6050の初期化に失敗しました。センサーが接続されていることを確認してください。");
    while (1) {
      delay(10);
    }
  }

  // センサーの範囲を設定
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // フィルターを初期化
  filter.begin(50); // サンプリングレートを50Hzに設定

  // センサーの初期化完了メッセージ
  Serial.println("MPU6050の初期化が完了しました。");

  // 最初のタイムスタンプを取得
  lastUpdate = micros();

  // 初期オフセットを計算
  delay(100); // センサーが安定するまで待機
  calculateInitialOffsets();
}

void loop() {
  // 現在の時間を取得
  unsigned long currentTime = micros();
  deltat = (currentTime - lastUpdate) / 1000000.0f;
  lastUpdate = currentTime;

  // センサーの値を取得
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Gyroデータをrad/sに変換（MPU6050のデフォルト出力はdeg/s）
  float gx = g.gyro.x * DEG_TO_RAD;
  float gy = g.gyro.y * DEG_TO_RAD;
  float gz = g.gyro.z * DEG_TO_RAD;

  // Madgwickフィルターを更新
  filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);

  // 姿勢角度を取得
  float roll = filter.getRoll() - initialRoll;
  float pitch = filter.getPitch() - initialPitch;
  float yaw = filter.getYaw() - initialYaw;

  // 姿勢角度をシリアルモニタに出力
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.println(yaw);

}

void calculateInitialOffsets() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Gyroデータをrad/sに変換（MPU6050のデフォルト出力はdeg/s）
  float gx = g.gyro.x * DEG_TO_RAD;
  float gy = g.gyro.y * DEG_TO_RAD;
  float gz = g.gyro.z * DEG_TO_RAD;

  // Madgwickフィルターを更新
  filter.updateIMU(gx, gy, gz, a.acceleration.x, a.acceleration.y, a.acceleration.z);

  // 初期角度を取得
  initialRoll = filter.getRoll();
  initialPitch = filter.getPitch();
  initialYaw = filter.getYaw();
}
