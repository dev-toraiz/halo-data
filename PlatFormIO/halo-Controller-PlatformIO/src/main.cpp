#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h>
#include <SPI.h>
#include <ESP32Servo.h>

// 定数定義
#define CHANNELS 8
#define SYNC_GAP 3000
#define PPM_PIN 4
#define LOOP_TIMING 100

// モーターピン定義
#define MOTOR1_PIN 25 // RL
#define MOTOR2_PIN 26 // RR
#define MOTOR3_PIN 27 // FR
#define MOTOR4_PIN 32 // FL

// PWM設定
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 16
#define THROTTLE_MIN 1000
#define THROTTLE_MAX 2000
#define THROTTLE_LIMIT 1800

// MPU6050レジスタアドレス
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_PWR_MGMT_1 0x6b

// キャリブレーション設定
#define CALIBRATION_SAMPLES 1200
#define CALIBRATION_DISCARD 200

// 物理定数
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232

// グローバル変数として定義
volatile unsigned long lastPulseTime = 0;
volatile int channelValues[CHANNELS] = {0};
volatile int currentChannel = 0;
const unsigned long SYNC_GAP_VALUE = SYNC_GAP; // 定数をグローバルに定義

// 通常の関数として割り込みハンドラを実装
void IRAM_ATTR ppmInterruptHandler()
{
  unsigned long pulseTime = micros();
  unsigned long pulseWidth = pulseTime - lastPulseTime;
  lastPulseTime = pulseTime;

  if (pulseWidth > SYNC_GAP_VALUE)
  {
    currentChannel = 0;
  }
  else if (currentChannel < CHANNELS)
  {
    channelValues[currentChannel] = pulseWidth;
    currentChannel++;
  }
}

// カルマンフィルタークラス
class KalmanFilter
{
private:
  float Q_angle = 0.001;  // プロセスノイズの分散
  float Q_bias = 0.003;   // プロセスノイズの分散
  float R_measure = 0.03; // 測定ノイズの分散

  float angle = 0;                  // 角度
  float bias = 0;                   // ジャイロバイアス
  float P[2][2] = {{0, 0}, {0, 0}}; // 誤差共分散行列

public:
  KalmanFilter()
  {
    P[0][0] = 0.001;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0.001;
  }

  float update(float newAngle, float newRate, float dt)
  {
    // 時間更新
    angle += dt * (newRate - bias);

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // 測定更新
    float y = newAngle - angle;
    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

  void reset()
  {
    angle = 0;
    bias = 0;
    P[0][0] = 0.001;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0.001;
  }
};

// クラス定義
class DroneController
{
private:
  // MPU6050関連
  MPU6050 mpu;
  int mpuAddr = 0x68;
  int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float rollIMU, pitchIMU, yawIMU;
  float accAngleX, accAngleY;
  double gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
  float interval, preInterval;
  double offsetX = 0, offsetY = 0, offsetZ = 0;
  float angleX, angleY, angleZ;
  float dpsX, dpsY, dpsZ;
  double initAngleX = 0, initAngleY = 0, initAngleZ = 0;
  double initAccX = 0, initAccY = 0, initAccZ = 0;
  double initDpsX = 0, initDpsY = 0, initDpsZ = 0;

  // カルマンフィルター
  KalmanFilter kalmanRoll;
  KalmanFilter kalmanPitch;
  KalmanFilter kalmanYaw;
  float rollKalman = 0, pitchKalman = 0, yawKalman = 0;

  // モーター制御
  Servo ESC1, ESC2, ESC3, ESC4;
  int m1_command_PWM = 0, m2_command_PWM = 0, m3_command_PWM = 0, m4_command_PWM = 0;
  float m1_command_scaled = 0, m2_command_scaled = 0, m3_command_scaled = 0, m4_command_scaled = 0;
  int morter1_buffer = 0, morter2_buffer = 0, morter3_buffer = 0, morter4_buffer = 0;

  // 受信機データ
  unsigned long PWM_throttle, PWM_roll, PWM_Elevation, PWM_Rudd, PWM_ThrottleCutSwitch;
  unsigned long PWM_throttle_prev, PWM_roll_prev, PWM_Elevation_prev, PWM_Rudd_prev;
  unsigned long PWM_throttle_output, PWM_roll_output, PWM_Elevation_output, PWM_Rudd_output;

  // PID制御パラメータ
  float i_limit = 25.0;
  float maxRoll = 15.0;
  float maxPitch = 15.0;
  float maxYaw = 140.0;
  float throttle_Limit = 1800;

  float hoverRoll = 0;
  float hoverPitch = 0;
  float hoverYaw = 0;

  float parameter_rate = 1.0;
  float PID_Adjuster = 1.0;
  float PID_Limit = 0.20;

  // PIDゲイン
  float Kp_roll_angle = 1.2 * parameter_rate;
  float Ki_roll_angle = 0.1 * parameter_rate;
  float Kd_roll_angle = 0.8 * parameter_rate;

  float Kp_pitch_angle = 1.2 * parameter_rate;
  float Ki_pitch_angle = 0.1 * parameter_rate;
  float Kd_pitch_angle = 0.8 * parameter_rate;

  float Kp_yaw = 15.0;
  float Ki_yaw = 5.0;
  float Kd_yaw = 0.1;

  float Roll_ProportionalBand = 15.0;
  float Pitch_ProportionalBand = 15.0;
  float Yaw_ProportionalBand = 30.0;

  float Out_ProportionalBand_Roll, Out_ProportionalBand_Pitch;

  // PID変数
  float error_roll = 0, error_roll_prev = 0;
  float integral_roll = 0, integral_roll_prev = 0;
  float derivative_roll = 0;

  float error_pitch = 0, error_pitch_prev = 0;
  float integral_pitch = 0, integral_pitch_prev = 0;
  float derivative_pitch = 0;

  float error_yaw = 0, error_yaw_prev = 0;
  float integral_yaw = 0, integral_yaw_prev = 0;
  float derivative_yaw = 0;

  double roll_PID = 0, pitch_PID = 0, yaw_PID = 0;

  // フィルター係数
  float alpha = 0.70;
  float alphaDes = 0.15;
  float alphaDerivative = 0.1;

  // 目標値
  float thro_des = 0, roll_des = 0, pitch_des = 0, yaw_des = 0;
  float thro_pre = 0, roll_pre = 0, pitch_pre = 0, yaw_pre = 0;
  float derivative_yaw_pre = 0;

  // その他
  float rollPIDError = -0.00;
  float pitchPIDError = 0.00;
  float rollWeight = 0.001;
  float pitchWeight = 0.001;
  float yawWeight = 0.0001;
  float minRotation = 200;
  bool keepRotating = false;
  bool zeroThrottleSafety = true;
  bool emergency = false;
  float stickDampener = 0.95;
  float deltaTime = 0.01;
  unsigned long previousMillis = 0;
  unsigned long currentMillis;
  float frameRate;

public:
  // コンストラクタ
  DroneController()
  {
    emergency = false;
  }

  // 初期化
  void begin()
  {
    // PPM受信機初期化
    pinMode(PPM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ::ppmInterruptHandler, RISING);
    Serial.println("PPM Receiver Initialized");

    // MPU6050初期化
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection())
    {
      Serial.println("MPU6050接続失敗！");
      while (1)
        ;
    }
    Serial.println("MPU6050接続成功！");
    initializeMPU6050();

    // モーター初期化
    ESC1.attach(MOTOR1_PIN);
    ESC2.attach(MOTOR2_PIN);
    ESC3.attach(MOTOR3_PIN);
    ESC4.attach(MOTOR4_PIN);
    Serial.println("PWM successfully attached to all motors");

    calibrateESCs();
  }

  // メインループ
  void update()
  {
    currentMillis = millis();

    if (currentMillis - previousMillis > 0)
    {
      deltaTime = (currentMillis - previousMillis) / 1000.0;
      frameRate = 1000.0 / (currentMillis - previousMillis);
      previousMillis = currentMillis;
    }

    loopDrone();
  }

private:
  // ドローン制御ループ
  void loopDrone()
  {
    getIMUData();
    applyFilters();
    getDesiredAnglesAndThrottle();
    calculatePIDControl();
    controlMixer();
    scaleCommands();
    commandMotors();
    getRadioSticks();

    // デバッグ出力
    printAcc();
    printGyro();
    printRollPitchYaw();
    printPIDoutput();
    printYawPID();
    printRollPID();
    printDes();
    printMotorCommands();
    ShowGyro();
  }

  // MPU6050初期化
  void initializeMPU6050()
  {
    findMPU6050Address();
    configureAccelerometer();
    calibrateGyro();
    calculateInitialAngles();
    calculateInitialAcceleration();
  }

  void findMPU6050Address()
  {
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++)
    {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0)
      {
        if (address < 16)
          mpuAddr = address;
        nDevices++;
      }
    }
  }

  void configureAccelerometer()
  {
    writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
    writeMPU6050(MPU6050_CONFIG, 0x00);
    writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
    writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
    writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  }

  void calibrateGyro()
  {
    Serial.print("ジャイロキャリブレーション開始");
    offsetX = offsetY = offsetZ = 0;

    // 最初のサンプルを捨てる
    for (int i = 0; i < CALIBRATION_DISCARD; i++)
    {
      readMPU6050Data();
      delay(1);
    }

    // 平均値計算用の変数
    float sumX = 0, sumY = 0, sumZ = 0;
    float maxX = -32768, maxY = -32768, maxZ = -32768;
    float minX = 32767, minY = 32767, minZ = 32767;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
      readMPU6050Data();
      dpsX = ((float)rawGyroX) / 65.5;
      dpsY = ((float)rawGyroY) / 65.5;
      dpsZ = ((float)rawGyroZ) / 65.5;

      sumX += dpsX;
      sumY += dpsY;
      sumZ += dpsZ;

      // 最大・最小値の更新
      maxX = max(maxX, dpsX);
      maxY = max(maxY, dpsY);
      maxZ = max(maxZ, dpsZ);
      minX = min(minX, dpsX);
      minY = min(minY, dpsY);
      minZ = min(minZ, dpsZ);

      if (i % 400 == 0)
      {
        Serial.print(".");
      }
      delay(1);
    }

    offsetX = sumX / CALIBRATION_SAMPLES;
    offsetY = sumY / CALIBRATION_SAMPLES;
    offsetZ = sumZ / CALIBRATION_SAMPLES;

    // 変動範囲の計算
    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    float rangeZ = maxZ - minZ;

    Serial.println("\nキャリブレーション完了:");
    Serial.printf("オフセット X: %.2f | Y: %.2f | Z: %.2f\n", offsetX, offsetY, offsetZ);
    Serial.printf("変動範囲 X: %.2f | Y: %.2f | Z: %.2f\n", rangeX, rangeY, rangeZ);

    // 変動が大きすぎる場合は警告
    if (rangeX > 1.0 || rangeY > 1.0 || rangeZ > 1.0)
    {
      Serial.println("警告: キャリブレーション中にセンサーの変動が大きすぎます");
      Serial.println("ドローンを安定した場所に置いて再キャリブレーションしてください");
    }
  }

  void calculateInitialAngles()
  {
    Serial.print("初期角度計算中");
    float sumAngleX = 0, sumAngleY = 0, sumAngleZ = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
      calculateRotation();
      sumAngleX += angleX;
      sumAngleY += angleY;
      sumAngleZ += angleZ;

      if (i % 400 == 0)
      {
        Serial.print(".");
      }
    }

    initAngleX = sumAngleX / CALIBRATION_SAMPLES;
    initAngleY = sumAngleY / CALIBRATION_SAMPLES;
    initAngleZ = sumAngleZ / CALIBRATION_SAMPLES;

    Serial.println("\n初期角度:");
    Serial.printf("X: %.2f | Y: %.2f | Z: %.2f\n", initAngleX, initAngleY, initAngleZ);
  }

  void calculateInitialAcceleration()
  {
    Serial.print("初期加速度計算中");
    float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
    float sumDpsX = 0, sumDpsY = 0, sumDpsZ = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
      calculateRotation();

      sumAccX += accX;
      sumAccY += accY;
      sumAccZ += accZ;
      sumDpsX += dpsX;
      sumDpsY += dpsY;
      sumDpsZ += dpsZ;

      if (i % 400 == 0)
      {
        Serial.print(".");
      }
    }

    initAccX = sumAccX / CALIBRATION_SAMPLES;
    initAccY = sumAccY / CALIBRATION_SAMPLES;
    initAccZ = sumAccZ / CALIBRATION_SAMPLES;
    initDpsX = sumDpsX / CALIBRATION_SAMPLES;
    initDpsY = sumDpsY / CALIBRATION_SAMPLES;
    initDpsZ = sumDpsZ / CALIBRATION_SAMPLES;

    Serial.println("\n初期加速度計算完了");
    Serial.printf("AccX: %.4f | AccY: %.4f | AccZ: %.4f\n", initAccX, initAccY, initAccZ);
    Serial.printf("DpsX: %.4f | DpsY: %.4f | DpsZ: %.4f\n", initDpsX, initDpsY, initDpsZ);
  }

  // MPU6050データ処理
  void readMPU6050Data()
  {
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(mpuAddr, 14, true);

    rawAccX = Wire.read() << 8 | Wire.read();
    rawAccY = Wire.read() << 8 | Wire.read();
    rawAccZ = Wire.read() << 8 | Wire.read();
    rawTemp = Wire.read() << 8 | Wire.read();
    rawGyroX = Wire.read() << 8 | Wire.read();
    rawGyroY = Wire.read() << 8 | Wire.read();
    rawGyroZ = Wire.read() << 8 | Wire.read();
  }

  void writeMPU6050(byte reg, byte data)
  {
    Wire.beginTransmission(mpuAddr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
  }

  void calculateRotation()
  {
    accX = (((float)rawAccX) / 16384.0);
    accY = (((float)rawAccY) / 16384.0);
    accZ = (((float)rawAccZ) / 16384.0);

    accAngleY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;
    accAngleX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;

    dpsX = (((float)rawGyroX) / 65.5);
    dpsY = (((float)rawGyroY) / 65.5);
    dpsZ = (((float)rawGyroZ) / 65.5);

    interval = millis() - preInterval;
    preInterval = millis();

    gyroAngleX += (dpsX - offsetX) * (interval * 0.001);
    gyroAngleY += (dpsY - offsetY) * (interval * 0.001);
    gyroAngleZ += (dpsZ - offsetZ) * (interval * 0.001);

    angleX = (0.996 * gyroAngleX) + (0.004 * accAngleX);
    angleY = (0.996 * gyroAngleY) + (0.004 * accAngleY);
    angleZ = gyroAngleZ;

    gyroAngleX = angleX;
    gyroAngleY = angleY;
    gyroAngleZ = angleZ;

    gyroX = initAngleX - angleX;
    gyroY = -(initAngleY - angleY);
    gyroZ = initAngleZ - angleZ;
  }

  void getIMUData()
  {
    readMPU6050Data();
    calculateRotation();

    // 加速度データの正規化
    float norm = sqrt(accX * accX + accY * accY + accZ * accZ);
    if (norm != 0)
    {
      accX /= norm;
      accY /= norm;
      accZ /= norm;
    }
  }

  void applyFilters()
  {
    // 相補フィルター
    float complementaryRoll = alpha * (rollIMU + (dpsX - initDpsX) * deltaTime) + (1 - alpha) * gyroX;
    float complementaryPitch = alpha * (pitchIMU + (dpsY - initDpsY) * deltaTime) + (1 - alpha) * gyroY;
    float complementaryYaw = alpha * (yawIMU + (dpsZ - initDpsZ) * deltaTime) + (1 - alpha) * gyroZ;

    // カルマンフィルター
    rollKalman = kalmanRoll.update(gyroX, dpsX - initDpsX, deltaTime);
    pitchKalman = kalmanPitch.update(gyroY, dpsY - initDpsY, deltaTime);
    yawKalman = kalmanYaw.update(gyroZ, dpsZ - initDpsZ, deltaTime);

    // 適応型フィルター - 動きの大きさに応じてカルマンと相補フィルターを混合
    float gyroMagnitude = sqrt(pow(dpsX - initDpsX, 2) + pow(dpsY - initDpsY, 2) + pow(dpsZ - initDpsZ, 2));
    float adaptiveWeight = constrain(gyroMagnitude / 100.0, 0.0, 1.0);

    // 最終的なフィルタリング結果
    rollIMU = adaptiveWeight * rollKalman + (1 - adaptiveWeight) * complementaryRoll;
    pitchIMU = adaptiveWeight * pitchKalman + (1 - adaptiveWeight) * complementaryPitch;
    yawIMU = adaptiveWeight * yawKalman + (1 - adaptiveWeight) * complementaryYaw;
  }

  // 制御関連
  void getDesiredAnglesAndThrottle()
  {
    // 入力PWM値から計算された目標値
    thro_des = (PWM_throttle - 1000.0) / 1000.0;
    roll_des = (PWM_roll - 1500.0) / 500.0;
    pitch_des = -((PWM_Elevation - 1500.0) / 500.0);
    yaw_des = -(PWM_Rudd - 1500.0) / 500.0;

    // ローパスフィルター適用
    thro_des = alphaDes * thro_des + (1.0 - alphaDes) * thro_pre;
    roll_des = alphaDes * roll_des + (1.0 - alphaDes) * roll_pre;
    pitch_des = alphaDes * pitch_des + (1.0 - alphaDes) * pitch_pre;
    yaw_des = alphaDes * yaw_des + (1.0 - alphaDes) * yaw_pre;

    thro_pre = thro_des;
    roll_pre = roll_des;
    pitch_pre = pitch_des;
    yaw_pre = yaw_des;

    // 範囲制限
    thro_des = constrain(thro_des, 0.0, 1.0) * throttle_Limit;
    roll_des = constrain(roll_des + hoverRoll, -1.0, 1.0) * maxRoll;
    pitch_des = constrain(pitch_des + hoverPitch, -1.0, 1.0) * maxPitch;
    yaw_des = constrain(yaw_des + hoverYaw, -1.0, 1.0) * maxYaw;
  }

  void calculatePIDControl()
  {
    // ロール制御
    error_roll = roll_des - rollIMU;
    integral_roll = integral_roll_prev + error_roll * deltaTime;
    integral_roll = constrain(integral_roll, -i_limit, i_limit);
    derivative_roll = dpsX - initDpsX;
    roll_PID = (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll + Kd_roll_angle * derivative_roll);
    roll_PID -= rollPIDError;
    roll_PID = constrain(roll_PID, -PID_Limit / rollWeight, PID_Limit / rollWeight);

    Out_ProportionalBand_Roll = (Kp_roll_angle * Roll_ProportionalBand);
    Out_ProportionalBand_Roll = constrain(Out_ProportionalBand_Roll, -PID_Limit / rollWeight, PID_Limit / rollWeight);

    if (error_roll > Roll_ProportionalBand)
    {
      roll_PID = Out_ProportionalBand_Roll;
    }
    else if (error_roll < -Roll_ProportionalBand)
    {
      roll_PID = -Out_ProportionalBand_Roll;
    }

    // ピッチ制御
    error_pitch = pitch_des - pitchIMU;
    integral_pitch = integral_pitch_prev + error_pitch * deltaTime;
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
    derivative_pitch = dpsY - initDpsY;
    pitch_PID = (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch + Kd_pitch_angle * derivative_pitch);
    pitch_PID -= pitchPIDError;
    pitch_PID = constrain(pitch_PID, -PID_Limit / pitchWeight, PID_Limit / pitchWeight);

    Out_ProportionalBand_Pitch = (Kp_pitch_angle * Pitch_ProportionalBand);
    Out_ProportionalBand_Pitch = constrain(Out_ProportionalBand_Pitch, -PID_Limit / pitchWeight, PID_Limit / pitchWeight);

    if (error_pitch > Pitch_ProportionalBand)
    {
      pitch_PID = Out_ProportionalBand_Pitch;
    }
    else if (error_pitch < -Pitch_ProportionalBand)
    {
      pitch_PID = -Out_ProportionalBand_Pitch;
    }

    // ヨー制御
    error_yaw = yaw_des - (dpsZ - initDpsZ);
    integral_yaw = integral_yaw_prev + error_yaw * deltaTime;
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
    derivative_yaw = -(error_yaw - error_yaw_prev) / deltaTime;
    derivative_yaw = alphaDerivative * derivative_yaw + (1.0 - alphaDes) * derivative_yaw_pre;
    derivative_yaw_pre = derivative_yaw;
    yaw_PID = yawWeight * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);
    yaw_PID = constrain(yaw_PID, -PID_Limit, PID_Limit);

    // 変数更新
    integral_roll_prev = integral_roll;
    integral_pitch_prev = integral_pitch;
    error_yaw_prev = error_yaw;
    error_pitch_prev = error_pitch;
    error_roll_prev = error_roll;
    integral_yaw_prev = integral_yaw;
  }

  void controlMixer()
  {
    m1_command_scaled = (thro_des) + PID_Adjuster * (-pitchWeight * pitch_PID + rollWeight * roll_PID + yaw_PID);
    m2_command_scaled = (thro_des) + PID_Adjuster * (-pitchWeight * pitch_PID - rollWeight * roll_PID - yaw_PID);
    m3_command_scaled = (thro_des) + PID_Adjuster * (pitchWeight * pitch_PID - rollWeight * roll_PID + yaw_PID);
    m4_command_scaled = (thro_des) + PID_Adjuster * (pitchWeight * pitch_PID + rollWeight * roll_PID - yaw_PID);

    m1_command_scaled = constrain(m1_command_scaled, 0, 1.0);
    m2_command_scaled = constrain(m2_command_scaled, 0, 1.0);
    m3_command_scaled = constrain(m3_command_scaled, 0, 1.0);
    m4_command_scaled = constrain(m4_command_scaled, 0, 1.0);
  }

  void scaleCommands()
  {
    m1_command_PWM = m1_command_scaled * (THROTTLE_MAX - THROTTLE_MIN);
    m2_command_PWM = m2_command_scaled * (THROTTLE_MAX - THROTTLE_MIN);
    m3_command_PWM = m3_command_scaled * (THROTTLE_MAX - THROTTLE_MIN);
    m4_command_PWM = m4_command_scaled * (THROTTLE_MAX - THROTTLE_MIN);
  }

  void getRadioSticks()
  {
    // チャンネル値を安全にコピー
    noInterrupts();
    PWM_throttle = channelValues[2];
    PWM_roll = channelValues[0];
    PWM_Elevation = channelValues[1];
    PWM_Rudd = channelValues[3];
    interrupts();

    if (channelValues[6] <= 1500)
    {
      keepRotating = true;
    }
    else
    {
      keepRotating = false;
    }

    PWM_throttle_output = PWM_throttle;
    PWM_roll_output = PWM_roll;
    PWM_Elevation_output = PWM_Elevation;
    PWM_Rudd_output = PWM_Rudd;

    PWM_throttle = (stickDampener)*PWM_throttle_prev + (1 - stickDampener) * PWM_throttle;
    PWM_roll = (1.0 - stickDampener) * PWM_roll_prev + stickDampener * PWM_roll;
    PWM_Elevation = (1.0 - stickDampener) * PWM_Elevation_prev + stickDampener * PWM_Elevation;
    PWM_Rudd = (1.0 - stickDampener) * PWM_Rudd_prev + stickDampener * PWM_Rudd;

    PWM_throttle_prev = PWM_throttle;
    PWM_roll_prev = PWM_roll;
    PWM_Elevation_prev = PWM_Elevation;
    PWM_Rudd_prev = PWM_Rudd;
  }

  int getChannelValue(int channelIndex)
  {
    if (channelIndex >= 0 && channelIndex < CHANNELS)
    {
      noInterrupts();
      int value = channelValues[channelIndex];
      interrupts();
      return value;
    }
    else
    {
      Serial.print("Error: Invalid channel index ");
      Serial.println(channelIndex);
      return 0;
    }
  }

  // モーター制御
  void commandMotors()
  {
    // ベーススロットル値とバッファ値を追加
    m1_command_PWM += THROTTLE_MIN + morter1_buffer;
    m2_command_PWM += THROTTLE_MIN + morter2_buffer;
    m3_command_PWM += THROTTLE_MIN + morter3_buffer;
    m4_command_PWM += THROTTLE_MIN + morter4_buffer;

    // PWM値を範囲内に制限
    m1_command_PWM = constrain(m1_command_PWM, THROTTLE_MIN, throttle_Limit);
    m2_command_PWM = constrain(m2_command_PWM, THROTTLE_MIN, throttle_Limit);
    m3_command_PWM = constrain(m3_command_PWM, THROTTLE_MIN, throttle_Limit);
    m4_command_PWM = constrain(m4_command_PWM, THROTTLE_MIN, throttle_Limit);

    // スロットルが0の場合モーターを回さないようにする
    if (thro_des <= 0.02 && zeroThrottleSafety && !keepRotating)
    {
      m1_command_PWM = THROTTLE_MIN;
      m2_command_PWM = THROTTLE_MIN;
      m3_command_PWM = THROTTLE_MIN;
      m4_command_PWM = THROTTLE_MIN;
    }

    // もしプロポとの通信が切れた場合モーターを停止する
    bool allZero = true;
    noInterrupts();
    for (int i = 0; i < CHANNELS; i++)
    {
      if (channelValues[i] != 0)
      {
        allZero = false;
        break;
      }
    }
    interrupts();

    if (allZero)
    {
      m1_command_PWM = THROTTLE_MIN;
      m2_command_PWM = THROTTLE_MIN;
      m3_command_PWM = THROTTLE_MIN;
      m4_command_PWM = THROTTLE_MIN;
    }

    // モーターにPWMを設定
    setMotorPWM(m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, false);
  }

  void calibrateESCs()
  {
    Serial.println("Starting calibration");
    setMotorPWM(THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, true);
    Serial.println("Setting maximum throttle");
    delay(2000);
    Serial.println("Setting minimum throttle");
    delay(2000);
  }

  void setMotorPWM(int m1, int m2, int m3, int m4, bool cal)
  {
    int duty1, duty2, duty3, duty4;

    if (!cal && keepRotating)
    {
      duty1 = constrain(m1, THROTTLE_MIN + minRotation, THROTTLE_MAX);
      duty2 = constrain(m2, THROTTLE_MIN + minRotation, THROTTLE_MAX);
      duty3 = constrain(m3, THROTTLE_MIN + minRotation, THROTTLE_MAX);
      duty4 = constrain(m4, THROTTLE_MIN + minRotation, THROTTLE_MAX);
    }
    else
    {
      duty1 = constrain(m1, THROTTLE_MIN, THROTTLE_MAX);
      duty2 = constrain(m2, THROTTLE_MIN, THROTTLE_MAX);
      duty3 = constrain(m3, THROTTLE_MIN, THROTTLE_MAX);
      duty4 = constrain(m4, THROTTLE_MIN, THROTTLE_MAX);
    }

    if (emergency)
    {
      ESC1.writeMicroseconds(THROTTLE_MIN);
      ESC2.writeMicroseconds(THROTTLE_MIN);
      ESC3.writeMicroseconds(THROTTLE_MIN);
      ESC4.writeMicroseconds(THROTTLE_MIN);
    }
    else
    {
      ESC1.writeMicroseconds(duty1);
      ESC2.writeMicroseconds(duty2);
      ESC3.writeMicroseconds(duty3);
      ESC4.writeMicroseconds(duty4);
    }
  }

  // デバッグ出力関数
  void showRecievedData()
  {
    Serial.print("Channel values: ");
    for (int i = 0; i < CHANNELS; i++)
    {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(channelValues[i]);
      Serial.print("us\t");
    }
    Serial.println();
  }

  void printRollPitchYaw()
  {
    Serial.print(F(" roll_imu: "));
    Serial.print(rollIMU);
    Serial.print(F(" pitch_imu: "));
    Serial.print(pitchIMU);
    Serial.print(F(" yaw_imu: "));
    Serial.println(yawIMU);
  }

  void printDes()
  {
    Serial.print(F("  roll_des: "));
    Serial.print(roll_des);
    Serial.print(F("| pitch_des: "));
    Serial.print(pitch_des);
    Serial.print(F("| yaw_des: "));
    Serial.println(yaw_des);
  }

  void printAcc()
  {
    Serial.print(F(" AccX: "));
    if (accX >= 0)
      Serial.print("+");
    Serial.print(accX);

    Serial.print(F(" AccY: "));
    if (accY >= 0)
      Serial.print("+");
    Serial.print(accY);

    Serial.print(F(" AccZ: "));
    if (accZ >= 0)
      Serial.print("+");
    Serial.println(accZ);
  }

  void printGyro()
  {
    Serial.print(F(" GyroX: "));
    Serial.print(gyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(gyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(gyroZ);
  }

  void ShowGyro()
  {
    float sg_gx = dpsX - initDpsX;
    float sg_gy = dpsY - initDpsY;
    float sg_gz = dpsZ - initDpsZ;
    Serial.print("gx: ");
    Serial.print(sg_gx);
    Serial.print(" gy: ");
    Serial.print(sg_gy);
    Serial.print(" gz: ");
    Serial.println(sg_gz);
  }

  void printMotorCommands()
  {
    Serial.print(F("["));
    Serial.print(F("m1_command: "));
    Serial.print(m1_command_PWM);
    Serial.print(F(","));

    Serial.print(F("m2_command: "));
    Serial.print(m2_command_PWM);
    Serial.print(F(","));

    Serial.print(F("m3_command: "));
    Serial.print(m3_command_PWM);
    Serial.print(F(","));

    Serial.print(F("m4_command: "));
    Serial.print(m4_command_PWM);
    Serial.println(F("]"));
  }

  void printPIDoutput()
  {
    Serial.print(F("roll_PID: "));
    if (roll_PID >= 0)
      Serial.print("+");
    Serial.print(roll_PID);

    Serial.print(F(" pitch_PID: "));
    if (pitch_PID >= 0)
      Serial.print("+");
    Serial.print(pitch_PID);

    Serial.print(F(" yaw_PID: "));
    if (yaw_PID >= 0)
      Serial.print("+");
    Serial.println(yaw_PID);
  }

  void printReceive()
  {
    Serial.print(F(", \"PWM_throttle\": "));
    Serial.print(PWM_throttle_output);
    Serial.print(F(", \"PWM_roll\": "));
    Serial.print(PWM_roll_output);
    Serial.print(F(", \"PWM_Elevation\": "));
    Serial.print(PWM_Elevation_output);
    Serial.print(F(", \"PWM_Rudd\": "));
    Serial.print(PWM_Rudd_output);
  }

  void printYawPID()
  {
    Serial.print("yaw_des: ");
    if (yaw_des >= 0)
      Serial.print("+");
    Serial.print(yaw_des);

    Serial.print(", dpsZ: ");
    if (dpsZ >= 0)
      Serial.print("+");
    Serial.print(dpsZ);

    Serial.print(", error_yaw: ");
    if (error_yaw >= 0)
      Serial.print("+");
    Serial.print(error_yaw);

    Serial.print(", integral_yaw: ");
    if (integral_yaw >= 0)
      Serial.print("+");
    Serial.print(integral_yaw);

    Serial.print(", derivative_yaw: ");
    if (derivative_yaw >= 0)
      Serial.print("+");
    Serial.print(derivative_yaw);

    Serial.print(", yaw_PID: ");
    if (yaw_PID >= 0)
      Serial.print("+");
    Serial.println(yaw_PID);
  }

  void printRollPID()
  {
    Serial.print("roll_des: ");
    if (roll_des >= 0)
      Serial.print("+");
    Serial.print(roll_des);

    Serial.print(", roll_IMU: ");
    if (rollIMU >= 0)
      Serial.print("+");
    Serial.print(rollIMU);

    Serial.print(", error_roll: ");
    if (error_roll >= 0)
      Serial.print("+");
    Serial.print(error_roll);

    Serial.print(", integral_roll: ");
    if (integral_roll >= 0)
      Serial.print("+");
    Serial.print(integral_roll);

    Serial.print(", derivative_roll: ");
    if (derivative_roll >= 0)
      Serial.print("+");
    Serial.print(derivative_roll);

    Serial.print(", roll_PID: ");
    if (roll_PID >= 0)
      Serial.print("+");
    Serial.println(roll_PID);
  }
};

// グローバルインスタンス
DroneController drone;

void setup()
{
  Serial.begin(115200);
  drone.begin();
}

void loop()
{
  drone.update();
}