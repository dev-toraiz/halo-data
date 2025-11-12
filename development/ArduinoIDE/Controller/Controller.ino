#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h>
#include <SPI.h>
#include <ESP32Servo.h>

// ==================== 定数定義 ====================

// ピン定義
#define PPM_PIN 4      // PPM信号の入力ピン
#define LED_PIN1 18    // LED1のピン
#define LED_PIN2 19    // LED2のピン
#define MOTOR_PIN1 25  // モーター1のピン (RL)
#define MOTOR_PIN2 26  // モーター2のピン (RR)
#define MOTOR_PIN3 27  // モーター3のピン (FR)
#define MOTOR_PIN4 32  // モーター4のピン (FL)

// PPM設定
#define CHANNELS 8     // 使用するチャネル数
#define SYNC_GAP 3000  // 同期信号判定のしきい値 (マイクロ秒)

// PWM設定
#define PWM_FREQUENCY 50   // 50Hz (20ms周期、一般的なESCに対応)
#define PWM_RESOLUTION 16  // 16ビット解像度
#define THROTTLE_MIN 1000  // 最小PWM
#define THROTTLE_MAX 2000  // 最大PWM

// ループタイミング設定
#define LOOP_TIMING 100  // IMUの更新レートに基づく

// MPU6050レジスタアドレス
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_PWR_MGMT_1 0x6b

// ==================== グローバル変数 ====================

// PPM信号処理用変数
volatile unsigned long lastPulseTime = 0;      // 前回のパルス時間
volatile int channelValues[CHANNELS] = { 0 };  // 各チャネルの値を格納
volatile int currentChannel = 0;               // 現在のチャネルインデックス

// MPU6050センサー
MPU6050 mpu;
int mpuAddress = 0x68;

// サーボ（ESC制御用）
Servo ESC1, ESC2, ESC3, ESC4;

// モーター調整値
int motorTrim[4] = { 0, 0, 0, 0 };  // RL, RR, FR, FL

// 時間管理変数
float deltaTime = 1;
unsigned long currentMillis, previousMillis = 0;
float frameRate;

// IMUデータ
float accX, accY, accZ;
float accX_prev, accY_prev, accZ_prev;
float gyroX, gyroY, gyroZ;
float gyroX_prev, gyroY_prev, gyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

// 生のセンサーデータ
int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;

// 角度計算用変数
float accAngleX, accAngleY;
double gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
float interval, preInterval;
double offsetX = 0, offsetY = 0, offsetZ = 0;
float angleX, angleY, angleZ;
float dpsX, dpsY, dpsZ;
double initAngleX = 0, initAngleY = 0, initAngleZ = 0;
double initAccX = 0, initAccY = 0, initAccZ = 0, initDpsX = 0, initDpsY = 0, initDpsZ = 0;

// 制御パラメータ
float iLimit = 25;      // 積分制限
float maxRoll = 15.0;   // 最大ロール角度
float maxPitch = 15.0;  // 最大ピッチ角度
float maxYaw = 140.0;   // 最大ヨーレート
float throttleLimit = 1800;

// ホバリング調整値
float hoverRoll = 0;
float hoverPitch = 0;
float hoverYaw = 0;

// PIDゲイン調整
float parameterRate = 1.0;
float pidAdjuster = 1.0;
float pidLimit = 0.20;

// ロールPIDゲイン
float kpRollAngle = 1.2 * parameterRate;
float kiRollAngle = 0.1 * parameterRate;
float kdRollAngle = 0.8 * parameterRate;

// ピッチPIDゲイン
float kpPitchAngle, kiPitchAngle, kdPitchAngle;

// ヨーPIDゲイン
float kpYaw = 15;
float kiYaw = 5;
float kdYaw = 0.1;

// 比例帯設定
float rollProportionalBand = 15;
float pitchProportionalBand = 15;
float yawProportionalBand = 30;
float outProportionalBandRoll, outProportionalBandPitch;

// スティック入力の平滑化係数
float stickDampener = 0.95;

// PIDエラー補正値
float rollPIDError = -0.00;
float pitchPIDError = 0.00;

// 重み係数
float rollWeight = 0.001;
float pitchWeight = 0.001;
float yawWeight = 0.0001;

// 安全機能設定
float minRotation = 200;
bool keepRotating = false;
bool zeroThrottleSafety = true;
bool emergency = false;

// フィルター係数
const float alpha = 0.70;     // 相補性フィルターの係数
float alphaDes = 0.15;        // 目標値フィルター係数
float alphaDerivative = 0.1;  // 微分項フィルター係数

// 前回値保存用変数
float throPre = 0.0;
float rollPre = 0.0;
float pitchPre = 0.0;
float yawPre = 0.0;
float derivativeRollPre = 0.0;
float derivativePitchPre = 0.0;
float derivativeYawPre = 0.0;

// 目標値
float throDes, rollDes, pitchDes, yawDes;

// PID計算用変数
float errorRoll, errorRollPrev, rollDesPrev, integralRoll, integralRollPrev;
float errorPitch, errorPitchPrev, pitchDesPrev, integralPitch, integralPitchPrev;
float errorYaw, errorYawPrev, integralYaw, integralYawPrev, derivativeYaw;
double rollPID, pitchPID, yawPID;

// モーター出力値
float m1CommandScaled, m2CommandScaled, m3CommandScaled, m4CommandScaled;
int m1CommandPWM, m2CommandPWM, m3CommandPWM, m4CommandPWM;

// 受信機からの入力値
unsigned long pwmThrottle, pwmRoll, pwmElevation, pwmRudd;
unsigned long pwmThrottlePrev, pwmRollPrev, pwmElevationPrev, pwmRuddPrev;
unsigned long pwmThrottleOutput, pwmRollOutput, pwmElevationOutput, pwmRuddOutput;

// ==================== 関数プロトタイプ宣言 ====================

// 初期化関連
void setupLEDs();
void setupPPMReceiver();
void setupMPU6050();
void setupMotors();
void calibrateESCs();

// MPU6050関連
void findAccelerometerAddress();
void writeMPU6050Register(byte reg, byte data);
void readAccelerometerData();
void calculateRotation();
void calibrateAccelerometer();
void calculateInitialAngles();
void calculateInitialAcceleration();

// ドローン制御関連
void loopDrone();
void getIMUData();
void applyComplementaryFilter();
void getDesiredAnglesAndThrottle();
void calculatePIDControl();
void calculateRollPID();
void calculatePitchPID();
void calculateYawPID();
void updatePIDPreviousValues();
void mixControls();
void scaleCommands();
void commandMotors();
bool isConnectionLost();
void setMotorPWM(int m1, int m2, int m3, int m4, bool cal);
void getRadioSticks();
int getChannelValue(int channelIndex);

// デバッグ出力関連
void printDebugInfo();
void printAcceleration();
void printGyroscope();
void printRollPitchYaw();
void printPIDOutput();
void printYawPID();
void printRollPID();
void printDesiredValues();
void printMotorCommands();
void printGyroRaw();

// ==================== 割り込み処理 ====================

// PPM信号の割り込み処理
void IRAM_ATTR ppmInterrupt() {
  unsigned long pulseTime = micros();                    // 現在の時間を取得
  unsigned long pulseWidth = pulseTime - lastPulseTime;  // パルス幅を計算
  lastPulseTime = pulseTime;

  if (pulseWidth > SYNC_GAP) {
    // 同期信号を検出
    currentChannel = 0;  // チャネルをリセット
  } else {
    if (currentChannel < CHANNELS) {
      // 有効なチャネル範囲内であれば
      channelValues[currentChannel] = pulseWidth;  // チャネル値を格納
      currentChannel++;                            // 次のチャネルへ
    }
  }
}

// ==================== セットアップ関数 ====================

void setup() {
  Serial.begin(115200);

  emergency = false;

  // 各コンポーネントの初期化
  setupLEDs();
  setupPPMReceiver();
  setupMPU6050();
  setupMotors();

  // PIDゲインの初期化
  kpPitchAngle = kpRollAngle;
  kiPitchAngle = kiRollAngle;
  kdPitchAngle = kdRollAngle;

  // ESCキャリブレーション
  calibrateESCs();
}

// LEDの初期化
void setupLEDs() {
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  digitalWrite(LED_PIN1, HIGH);
  digitalWrite(LED_PIN2, HIGH);
}

// PPM受信機の初期化
void setupPPMReceiver() {
  pinMode(PPM_PIN, INPUT_PULLUP);                  // ピンを入力モードに設定
  attachInterrupt(PPM_PIN, ppmInterrupt, RISING);  // 割り込みを設定
  Serial.println("PPM Receiver Initialized");
}

// MPU6050の初期化
void setupMPU6050() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050接続失敗！");
    while (1)
      ;
  }
  Serial.println("MPU6050接続成功！");

  // 加速度センサーの設定
  findAccelerometerAddress();
  readAccelerometerData();

  // MPU6050の設定
  writeMPU6050Register(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050Register(MPU6050_CONFIG, 0x00);
  writeMPU6050Register(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050Register(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050Register(MPU6050_PWR_MGMT_1, 0x01);

  // キャリブレーション
  calibrateAccelerometer();
}

// モーターの初期化
void setupMotors() {
  // 各モーターをピンにアタッチ
  ESC1.attach(MOTOR_PIN1);
  ESC2.attach(MOTOR_PIN2);
  ESC3.attach(MOTOR_PIN3);
  ESC4.attach(MOTOR_PIN4);

  Serial.println("PWM successfully attached to all motors");
}

// ==================== メインループ ====================

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis > 0) {
    deltaTime = (currentMillis - previousMillis) / 1000.0;
    frameRate = 1000.0 / (currentMillis - previousMillis);
    previousMillis = currentMillis;
  }

  loopDrone();
}

// ドローン制御ループ
void loopDrone() {
  getIMUData();                   // IMUからデータ取得
  applyComplementaryFilter();     // 相補性フィルター適用
  getDesiredAnglesAndThrottle();  // 目標値計算
  calculatePIDControl();          // PID制御計算
  mixControls();                  // 制御信号ミキシング
  scaleCommands();                // コマンドのスケーリング
  commandMotors();                // モーター制御
  getRadioSticks();               // 受信機からの入力取得

  // デバッグ情報出力
  printDebugInfo();
}

// ==================== MPU6050関連関数 ====================

// 加速度センサーアドレス検索
void findAccelerometerAddress() {
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      if (address < 16)
        mpuAddress = address;
      nDevices++;
    }
  }
}

// MPU6050レジスタ書き込み
void writeMPU6050Register(byte reg, byte data) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// 加速度センサーからデータ読み取り
void readAccelerometerData() {
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddress, 14, true);

  rawAccX = Wire.read() << 8 | Wire.read();
  rawAccY = Wire.read() << 8 | Wire.read();
  rawAccZ = Wire.read() << 8 | Wire.read();
  rawTemp = Wire.read() << 8 | Wire.read();
  rawGyroX = Wire.read() << 8 | Wire.read();
  rawGyroY = Wire.read() << 8 | Wire.read();
  rawGyroZ = Wire.read() << 8 | Wire.read();
}

// 回転計算
void calculateRotation() {
  // 加速度データの正規化
  accX = (((float)rawAccX) / 16384.0);
  accY = (((float)rawAccY) / 16384.0);
  accZ = (((float)rawAccZ) / 16384.0);

  // 加速度から角度を計算
  accAngleY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;
  accAngleX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;

  // ジャイロデータの変換
  dpsX = (((float)rawGyroX) / 65.5);
  dpsY = (((float)rawGyroY) / 65.5);
  dpsZ = (((float)rawGyroZ) / 65.5);

  // 時間間隔の計算
  interval = millis() - preInterval;
  preInterval = millis();

  // ジャイロ角度の積分
  gyroAngleX += (dpsX - offsetX) * (interval * 0.001);
  gyroAngleY += (dpsY - offsetY) * (interval * 0.001);
  gyroAngleZ += (dpsZ - offsetZ) * (interval * 0.001);

  // 相補フィルターで角度を計算
  angleX = (0.996 * gyroAngleX) + (0.004 * accAngleX);
  angleY = (0.996 * gyroAngleY) + (0.004 * accAngleY);
  angleZ = gyroAngleZ;

  // 角度の更新
  gyroAngleX = angleX;
  gyroAngleY = angleY;
  gyroAngleZ = angleZ;

  // 初期値からの差分を計算
  gyroX = initAngleX - angleX;
  gyroY = -(initAngleY - angleY);
  gyroZ = initAngleZ - angleZ;
}

// 加速度センサーキャリブレーション
void calibrateAccelerometer() {
  Serial.print("Calculating Calibration");

  // ジャイロオフセットの計算
  for (int i = 0; i < 1200; i++) {
    readAccelerometerData();
    dpsX = ((float)rawGyroX) / 65.5;
    dpsY = ((float)rawGyroY) / 65.5;
    dpsZ = ((float)rawGyroZ) / 65.5;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;

    if (i % 400 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();

  // 平均値の計算
  offsetX /= 1200;
  offsetY /= 1200;
  offsetZ /= 1200;

  Serial.println("Calibration complete:");
  Serial.print("Offset X: ");
  Serial.print(offsetX);
  Serial.print("| Offset Y: ");
  Serial.print(offsetY);
  Serial.print("| Offset Z: ");
  Serial.println(offsetZ);
  Serial.println();

  // 初期角度の計算
  calculateInitialAngles();

  // 初期加速度の計算
  calculateInitialAcceleration();
}

// 初期角度の計算
void calculateInitialAngles() {
  Serial.print("Calculating Rotation");

  float sumAngleX = 0;
  float sumAngleY = 0;
  float sumAngleZ = 0;

  for (int i = 0; i < 1200; i++) {
    calculateRotation();

    // 角度の累積
    sumAngleX += angleX;
    sumAngleY += angleY;
    sumAngleZ += angleZ;

    // プログレスを表示
    if (i % 400 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();

  // 平均値の計算
  initAngleX = sumAngleX / 1200;
  initAngleY = sumAngleY / 1200;
  initAngleZ = sumAngleZ / 1200;

  Serial.println("Initial angles:");
  Serial.print("Initial Angle X: ");
  Serial.print(initAngleX);
  Serial.print("| Initial Angle Y: ");
  Serial.print(initAngleY);
  Serial.print("| Initial Angle Z: ");
  Serial.println(initAngleZ);
  Serial.println();
}

// 初期加速度の計算
void calculateInitialAcceleration() {
  float sumAccX = 0;
  float sumAccY = 0;
  float sumAccZ = 0;
  float sumDpsX = 0;
  float sumDpsY = 0;
  float sumDpsZ = 0;

  Serial.print("Calculating Acceleration");
  for (int i = 0; i < 1200; i++) {
    calculateRotation();

    sumAccX += accX;
    sumAccY += accY;
    sumAccZ += accZ;
    sumDpsX += dpsX;
    sumDpsY += dpsY;
    sumDpsZ += dpsZ;

    if (i % 400 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();

  // 平均値の計算
  initAccX = sumAccX / 1200;
  initAccY = sumAccY / 1200;
  initAccZ = sumAccZ / 1200;

  initDpsX = sumDpsX / 1200;
  initDpsY = sumDpsY / 1200;
  initDpsZ = sumDpsZ / 1200;

  Serial.println("Initial acc");
  Serial.print("Initial AccX: ");
  Serial.print(initAccX);
  Serial.print("| Initial AccY : ");
  Serial.print(initAccY);
  Serial.print("| Initial AccZ: ");
  Serial.print(initAccZ);
  Serial.print("| Initial dpsX: ");
  Serial.print(initDpsX);
  Serial.print("| Initial dpsY: ");
  Serial.print(initDpsY);
  Serial.print("| Initial dosZ: ");
  Serial.print(initDpsZ);
  Serial.println();
}

// ==================== モーター制御関連関数 ====================

// ESCキャリブレーション
void calibrateESCs() {
  Serial.println("Starting calibration");

  setMotorPWM(THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MIN, true);
  Serial.println("Setting maximum throttle");
  delay(2000);
  Serial.println("Setting minimum throttle");
  delay(2000);
}

// モーターPWM設定
void setMotorPWM(int m1, int m2, int m3, int m4, bool cal) {
  int duty1 = 0, duty2 = 0, duty3 = 0, duty4 = 0;

  if (!cal && keepRotating) {
    duty1 = constrain(m1, THROTTLE_MIN + minRotation, THROTTLE_MAX);
    duty2 = constrain(m2, THROTTLE_MIN + minRotation, THROTTLE_MAX);
    duty3 = constrain(m3, THROTTLE_MIN + minRotation, THROTTLE_MAX);
    duty4 = constrain(m4, THROTTLE_MIN + minRotation, THROTTLE_MAX);
  } else {
    duty1 = constrain(m1, THROTTLE_MIN, THROTTLE_MAX);
    duty2 = constrain(m2, THROTTLE_MIN, THROTTLE_MAX);
    duty3 = constrain(m3, THROTTLE_MIN, THROTTLE_MAX);
    duty4 = constrain(m4, THROTTLE_MIN, THROTTLE_MAX);
  }

  if (emergency) {
    ESC1.writeMicroseconds(THROTTLE_MIN);
    ESC2.writeMicroseconds(THROTTLE_MIN);
    ESC3.writeMicroseconds(THROTTLE_MIN);
    ESC4.writeMicroseconds(THROTTLE_MIN);
  } else {
    ESC1.writeMicroseconds(duty1);
    ESC2.writeMicroseconds(duty2);
    ESC3.writeMicroseconds(duty3);
    ESC4.writeMicroseconds(duty4);
  }
}

// ==================== ドローン制御関連関数 ====================

// IMUデータ取得
void getIMUData() {
  readAccelerometerData();
  calculateRotation();

  // 加速度データの正規化
  float norm = sqrt(accX * accX + accY * accY + accZ * accZ);
  if (norm != 0) {
    accX /= norm;
    accY /= norm;
    accZ /= norm;
  }
}

// 相補性フィルター適用
void applyComplementaryFilter() {
  roll_IMU = alpha * (roll_IMU + dpsX * deltaTime) + (1 - alpha) * gyroX;
  pitch_IMU = alpha * (pitch_IMU + dpsY * deltaTime) + (1 - alpha) * gyroY;
  yaw_IMU = alpha * (yaw_IMU + dpsZ * deltaTime) + (1 - alpha) * gyroZ;
}

// 目標角度とスロットル取得
void getDesiredAnglesAndThrottle() {
  // 入力PWM値から計算された目標値
  throDes = (pwmThrottle - 1000.0) / 1000.0;
  rollDes = (pwmRoll - 1500.0) / 500.0;
  pitchDes = -((pwmElevation - 1500.0) / 500.0);
  yawDes = -(pwmRudd - 1500.0) / 500.0;

  // ローパスフィルター適用
  throDes = alphaDes * throDes + (1.0 - alphaDes) * throPre;
  rollDes = alphaDes * rollDes + (1.0 - alphaDes) * rollPre;
  pitchDes = alphaDes * pitchDes + (1.0 - alphaDes) * pitchPre;
  yawDes = alphaDes * yawDes + (1.0 - alphaDes) * yawPre;

  // 前回値の更新
  throPre = throDes;
  rollPre = rollDes;
  pitchPre = pitchDes;
  yawPre = yawDes;

  // 範囲制限
  float throttleLimit = 0.9;  // スロットル制限
  throDes = constrain(throDes, 0.0, 1.0) * throttleLimit;
  rollDes = constrain(rollDes + hoverRoll, -1.0, 1.0) * maxRoll;
  pitchDes = constrain(pitchDes + hoverPitch, -1.0, 1.0) * maxPitch;
  yawDes = constrain(yawDes + hoverYaw, -1.0, 1.0) * maxYaw;
}

// PID制御計算
void calculatePIDControl() {
  // ロールPID計算
  calculateRollPID();

  // ピッチPID計算
  calculatePitchPID();

  // ヨーPID計算
  calculateYawPID();

  // 前回値の更新
  updatePIDPreviousValues();
}

// ロールPID計算
void calculateRollPID() {
  errorRoll = rollDes - roll_IMU;
  integralRoll = integralRollPrev + errorRoll * deltaTime;
  integralRoll = constrain(integralRoll, -iLimit, iLimit);
  float derivativeRoll = dpsX - initDpsX;

  rollPID = (kpRollAngle * errorRoll + kiRollAngle * integralRoll + kdRollAngle * derivativeRoll);
  rollPID -= rollPIDError;
  rollPID = constrain(rollPID, -pidLimit / rollWeight, pidLimit / rollWeight);

  // 比例帯処理
  outProportionalBandRoll = (kpRollAngle * rollProportionalBand);
  outProportionalBandRoll = constrain(outProportionalBandRoll, -pidLimit / rollWeight, pidLimit / rollWeight);

  if (errorRoll > rollProportionalBand) {
    rollPID = outProportionalBandRoll;
  } else if (errorRoll < -rollProportionalBand) {
    rollPID = -outProportionalBandRoll;
  }
}

// ピッチPID計算
void calculatePitchPID() {
  errorPitch = pitchDes - pitch_IMU;
  integralPitch = integralPitchPrev + errorPitch * deltaTime;
  integralPitch = constrain(integralPitch, -iLimit, iLimit);
  float derivativePitch = dpsY - initDpsY;

  pitchPID = (kpPitchAngle * errorPitch + kiPitchAngle * integralPitch + kdPitchAngle * derivativePitch);
  pitchPID -= pitchPIDError;
  pitchPID = constrain(pitchPID, -pidLimit / pitchWeight, pidLimit / pitchWeight);

  // 比例帯処理
  outProportionalBandPitch = (kpPitchAngle * pitchProportionalBand);
  outProportionalBandPitch = constrain(outProportionalBandPitch, -pidLimit / pitchWeight, pidLimit / pitchWeight);

  if (errorPitch > pitchProportionalBand) {
    pitchPID = outProportionalBandPitch;
  } else if (errorPitch < -pitchProportionalBand) {
    pitchPID = -outProportionalBandPitch;
  }
}

// ヨーPID計算
void calculateYawPID() {
  errorYaw = yawDes - (dpsZ - initDpsZ);
  integralYaw = integralYawPrev + errorYaw * deltaTime;
  integralYaw = constrain(integralYaw, -iLimit, iLimit);

  derivativeYaw = -(errorYaw - errorYawPrev) / deltaTime;
  derivativeYaw = alphaDerivative * derivativeYaw + (1.0 - alphaDes) * derivativeYawPre;
  derivativeYawPre = derivativeYaw;

  yawPID = yawWeight * (kpYaw * errorYaw + kiYaw * integralYaw + kdYaw * derivativeYaw);
  yawPID = constrain(yawPID, -pidLimit, pidLimit);
}

// PID前回値の更新
void updatePIDPreviousValues() {
  integralRollPrev = integralRoll;
  integralPitchPrev = integralPitch;
  errorYawPrev = errorYaw;
  errorPitchPrev = errorPitch;
  errorRollPrev = errorRoll;
  integralYawPrev = integralYaw;
}

// 制御信号ミキシング
void mixControls() {
  m1CommandScaled = (throDes) + pidAdjuster * (-pitchWeight * pitchPID + rollWeight * rollPID + yawPID);
  m2CommandScaled = (throDes) + pidAdjuster * (-pitchWeight * pitchPID - rollWeight * rollPID - yawPID);
  m3CommandScaled = (throDes) + pidAdjuster * (pitchWeight * pitchPID - rollWeight * rollPID + yawPID);
  m4CommandScaled = (throDes) + pidAdjuster * (pitchWeight * pitchPID + rollWeight * rollPID - yawPID);

  m1CommandScaled = constrain(m1CommandScaled, 0, 1.0);
  m2CommandScaled = constrain(m2CommandScaled, 0, 1.0);
  m3CommandScaled = constrain(m3CommandScaled, 0, 1.0);
  m4CommandScaled = constrain(m4CommandScaled, 0, 1.0);
}

// コマンドのスケーリング
void scaleCommands() {
  // スケーリングされた値をPWM値に変換
  m1CommandPWM = m1CommandScaled * THROTTLE_MAX;
  m2CommandPWM = m2CommandScaled * THROTTLE_MAX;
  m3CommandPWM = m3CommandScaled * THROTTLE_MAX;
  m4CommandPWM = m4CommandScaled * THROTTLE_MAX;
}

// モーター制御
void commandMotors() {
  // ベーススロットル値を追加
  m1CommandPWM += THROTTLE_MIN;
  m2CommandPWM += THROTTLE_MIN;
  m3CommandPWM += THROTTLE_MIN;
  m4CommandPWM += THROTTLE_MIN;

  // 各モーターのトリム値を追加
  m1CommandPWM += motorTrim[0];
  m2CommandPWM += motorTrim[1];
  m3CommandPWM += motorTrim[2];
  m4CommandPWM += motorTrim[3];

  // PWM値を範囲内に制限
  m1CommandPWM = constrain(m1CommandPWM, THROTTLE_MIN, throttleLimit);
  m2CommandPWM = constrain(m2CommandPWM, THROTTLE_MIN, throttleLimit);
  m3CommandPWM = constrain(m3CommandPWM, THROTTLE_MIN, throttleLimit);
  m4CommandPWM = constrain(m4CommandPWM, THROTTLE_MIN, throttleLimit);

  // スロットルが0の場合モーターを回さないようにする
  if (throDes <= 0.02 && zeroThrottleSafety && !keepRotating) {
    m1CommandPWM = THROTTLE_MIN;
    m2CommandPWM = THROTTLE_MIN;
    m3CommandPWM = THROTTLE_MIN;
    m4CommandPWM = THROTTLE_MIN;
  }

  // 通信切断時の安全機能
  if (isConnectionLost()) {
    m1CommandPWM = THROTTLE_MIN;
    m2CommandPWM = THROTTLE_MIN;
    m3CommandPWM = THROTTLE_MIN;
    m4CommandPWM = THROTTLE_MIN;
  }

  // モーターにPWMを設定
  setMotorPWM(m1CommandPWM, m2CommandPWM, m3CommandPWM, m4CommandPWM, false);
}

// 通信切断チェック
bool isConnectionLost() {
  bool allZero = true;
  // チャンネル値をチェック
  for (int i = 0; i < CHANNELS; i++) {
    if (channelValues[i] != 0) {
      allZero = false;
      break;
    }
  }
  return allZero;
}

// 受信機からの入力取得
void getRadioSticks() {
  // 各PWM入力値を channelValues 配列から割り当て
  pwmThrottle = getChannelValue(2);
  pwmRoll = getChannelValue(0);
  pwmElevation = getChannelValue(1);
  pwmRudd = getChannelValue(3);

  // 特殊モード設定
  if (getChannelValue(6) <= 1500) {
    keepRotating = true;
  } else {
    keepRotating = false;
  }

  // 出力用に値を保存
  pwmThrottleOutput = pwmThrottle;
  pwmRollOutput = pwmRoll;
  pwmElevationOutput = pwmElevation;
  pwmRuddOutput = pwmRudd;

  // スティック入力の平滑化
  pwmThrottle = (stickDampener)*pwmThrottlePrev + (1 - stickDampener) * pwmThrottle;
  pwmRoll = (1.0 - stickDampener) * pwmRollPrev + stickDampener * pwmRoll;
  pwmElevation = (1.0 - stickDampener) * pwmElevationPrev + stickDampener * pwmElevation;
  pwmRudd = (1.0 - stickDampener) * pwmRuddPrev + stickDampener * pwmRudd;

  // 前回値の更新
  pwmThrottlePrev = pwmThrottle;
  pwmRollPrev = pwmRoll;
  pwmElevationPrev = pwmElevation;
  pwmRuddPrev = pwmRudd;
}

// チャンネル値取得
int getChannelValue(int channelIndex) {
  if (channelIndex >= 0 && channelIndex < CHANNELS) {
    return channelValues[channelIndex];
  } else {
    Serial.print("Error: Invalid channel index ");
    Serial.println(channelIndex);
    return 0;  // 無効なインデックスの場合はデフォルト値を返す
  }
}

// ==================== デバッグ出力関連関数 ====================

// デバッグ情報出力
void printDebugInfo() {
 // printAcceleration();
 // printGyroscope();
 // printRollPitchYaw();
 // printPIDOutput();
 // printYawPID();
 // printRollPID();
 // printDesiredValues();
  printMotorCommands();
 // printGyroRaw();
}

// 加速度データ出力
void printAcceleration() {
  Serial.print(F(" AccX: "));
  if (accX >= 0) Serial.print("+");
  Serial.print(accX);

  Serial.print(F(" AccY: "));
  if (accY >= 0) Serial.print("+");
  Serial.print(accY);

  Serial.print(F(" AccZ: "));
  if (accZ >= 0) Serial.print("+");
  Serial.println(accZ);
}

// ジャイロデータ出力
void printGyroscope() {
  Serial.print(F(" GyroX: "));
  Serial.print(gyroX);
  Serial.print(F(" GyroY: "));
  Serial.print(gyroY);
  Serial.print(F(" GyroZ: "));
  Serial.println(gyroZ);
}

// ロール・ピッチ・ヨー出力
void printRollPitchYaw() {
  Serial.print(F(" roll_imu: "));
  Serial.print(roll_IMU);
  Serial.print(F(" pitch_imu: "));
  Serial.print(pitch_IMU);
  Serial.print(F(" yaw_imu: "));
  Serial.println(yaw_IMU);
}

// PID出力値表示
void printPIDOutput() {
  Serial.print(F("roll_PID: "));
  if (rollPID >= 0) Serial.print("+");
  Serial.print(rollPID);

  Serial.print(F(" pitch_PID: "));
  if (pitchPID >= 0) Serial.print("+");
  Serial.print(pitchPID);

  Serial.print(F(" yaw_PID: "));
  if (yawPID >= 0) Serial.print("+");
  Serial.println(yawPID);
}

// ヨーPID詳細出力
void printYawPID() {
  Serial.print("yaw_des: ");
  if (yawDes >= 0) Serial.print("+");
  Serial.print(yawDes);

  Serial.print(", dpsZ: ");
  if (dpsZ >= 0) Serial.print("+");
  Serial.print(dpsZ);

  Serial.print(", error_yaw: ");
  if (errorYaw >= 0) Serial.print("+");
  Serial.print(errorYaw);

  Serial.print(", integral_yaw: ");
  if (integralYaw >= 0) Serial.print("+");
  Serial.print(integralYaw);

  Serial.print(", derivative_yaw: ");
  if (derivativeYaw >= 0) Serial.print("+");
  Serial.print(derivativeYaw);

  Serial.print(", yaw_PID: ");
  if (yawPID >= 0) Serial.print("+");
  Serial.println(yawPID);
}

// ロールPID詳細出力
void printRollPID() {
  Serial.print("roll_des: ");
  if (rollDes >= 0) Serial.print("+");
  Serial.print(rollDes);

  Serial.print(", roll_IMU: ");
  if (roll_IMU >= 0) Serial.print("+");
  Serial.print(roll_IMU);

  Serial.print(", error_roll: ");
  if (errorRoll >= 0) Serial.print("+");
  Serial.print(errorRoll);

  Serial.print(", integral_roll: ");
  if (integralRoll >= 0) Serial.print("+");
  Serial.print(integralRoll);

  Serial.print(", derivative_roll: ");
  float derivativeRoll = dpsX - initDpsX;
  if (derivativeRoll >= 0) Serial.print("+");
  Serial.print(derivativeRoll);

  Serial.print(", roll_PID: ");
  if (rollPID >= 0) Serial.print("+");
  Serial.println(rollPID);
}

// 目標値出力
void printDesiredValues() {
  Serial.print(F("  roll_des: "));
  Serial.print(rollDes);
  Serial.print(F("| pitch_des: "));
  Serial.print(pitchDes);
  Serial.print(F("| yaw_des: "));
  Serial.println(yawDes);
}

// モーターコマンド出力
void printMotorCommands() {
  Serial.print(F("["));
  Serial.print(F("m1_command: "));
  Serial.print(m1CommandPWM);
  Serial.print(F(","));

  Serial.print(F("m2_command: "));
  Serial.print(m2CommandPWM);
  Serial.print(F(","));

  Serial.print(F("m3_command: "));
  Serial.print(m3CommandPWM);
  Serial.print(F(","));

  Serial.print(F("m4_command: "));
  Serial.print(m4CommandPWM);
  Serial.println(F("]"));
}

// 生のジャイロデータ出力
void printGyroRaw() {
  float sgGx = dpsX - initDpsX;
  float sgGy = dpsY - initDpsY;
  float sgGz = dpsZ - initDpsZ;
  Serial.print("gx: ");
  Serial.print(sgGx);

  Serial.print(" gy: ");
  Serial.print(sgGy);

  Serial.print(" gz: ");
  Serial.println(sgGz);
}