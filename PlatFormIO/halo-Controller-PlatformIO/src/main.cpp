#include <Wire.h>
#include <MadgwickAHRS.h>
#include <MPU6050.h>
#include <Arduino.h>
#include <SPI.h>
#include <ESP32Servo.h>

// SDA（データライン）: GPIO21
// SCL（クロックライン）: GPIO22

#define CHANNELS 8    // 使用するチャネル数
#define SYNC_GAP 3000 // 同期信号判定のしきい値 (マイクロ秒)
#define PPM_PIN 4     // PPM信号の入力ピン

const int ledPin1 = 18; // LED1が接続されているピン番号
const int ledPin2 = 19; // LED2が接続されているピン番号

MPU6050 mpu;
Madgwick MadgwickFilter;
volatile unsigned long lastPulseTime = 0;   // 前回のパルス時間
volatile int channelValues[CHANNELS] = {0}; // 各チャネルの値を格納
volatile int currentChannel = 0;            // 現在のチャネルインデックス

// The LOOP_TIMING is based on the IMU.  For the Arduino_LSM6DSOX, it is 104Hz.  So, the loop time is set a little longer so the IMU has time to update from the control change.
#define LOOP_TIMING 100

// モーターピン定義
#define m1Pin 25
#define m2Pin 26
#define m3Pin 27
#define m4Pin 32
Servo ESC1, ESC2, ESC3, ESC4;

// PWM設定
const int pwmFrequency = 50;  // 50Hz (20ms周期、一般的なESCに対応)
const int pwmResolution = 16; // 16ビット解像度

const int throttle_max = 2000; // 最大PWM
const int throttle_min = 1000; // 最小PWM

const float throttle_limit = 0.9;

int morter1_buffer = 0; // RL
int morter2_buffer = 0; // RR
int morter3_buffer = 0; // FR
int morter4_buffer = 0; // FL

// Controller parameters (this is where you "tune it".  It's best to use the WiFi interface to do it live and then update once its tuned.):
float i_limit = 25;    // Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 20.0;  // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxPitch = 20.0; // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxYaw = 140.0;  // Max yaw rate in deg/sec (default 160.0)
float throttle_Limit = 1800;

float hoverRoll = 0; //-1 to 1
float hoverPitch = 0;
float hoverYaw = 0;

float parameter_rate = 1.0;

float PID_Adjuster = 0.7;
float PID_Limit = 0.16;

float Kp_roll_angle = 1.1 * parameter_rate; // Roll P-gain
float Ki_roll_angle = 0.9 * parameter_rate; // Roll I-gain
float Kd_roll_angle =0.8* parameter_rate;   // Roll D-gain2

float Kp_pitch_angle = Kp_roll_angle; // Pitch P-gain
float Ki_pitch_angle = Ki_roll_angle; // Pitch I-gain
float Kd_pitch_angle = Kd_pitch_angle; // Pitch D-gain

float Kp_yaw = 20; // Yaw P-gain default 30
float Ki_yaw = 0;  // Yaw I-gain default 5
float Kd_yaw = 1;  // Yaw D-gain default .015 (be careful when increasing too high, motors will begin to overheat!)

float Roll_ProportionalBand = 30;  // deg
float Pitch_ProportionalBand = 30; // deg
float Yaw_ProportionalBand = 30;   // deg

float Out_ProportionalBand_Roll, Out_ProportionalBand_Pitch;

float stick_dampener = 0.95;

// General stuff for controlling timing of things
float deltaTime = 1;
float invFreq = (1.0 / LOOP_TIMING) * 1000000.0;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;

unsigned long previousMillis = 0;
unsigned long currentMillis; // Declare currentMillis as a global variable
float frameRate;

// IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;

float rollPIDError = -0.00;
float pitchPIDError = 0.00;

float roll_Weight = 0.001, pitch_Weight = 0.001, yaw_Weight = 0.0001;

float min_rotation = 200;
float keep_rotating = false;
float zero_throttle_safty = true;
bool emergency;

const float alpha = 0.70; // 相補性フィルターの係数

float alpha_des = 0.15;
float thro_pre = 0.0;
float roll_pre = 0.0;
float pitch_pre = 0.0;
float yaw_pre = 0.0;

float alpha_derivative = 0.1;
float derivative_roll_pre, derivative_pitch_pre, derivative_yaw_pre;

int MPU6050_ADDR = 0x68;
int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z;
float acc_angle_x, acc_angle_y;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float interval, preInterval;
double offsetX = 0, offsetY = 0, offsetZ = 0;
float angleX, angleY, angleZ;
float dpsX, dpsY, dpsZ;
double init_angleX = 0, init_angleY = 0, init_angleZ = 0;
double init_AccX = 0, init_AccY = 0, init_AccZ = 0, init_dpsX = 0, init_dpsY = 0, init_dpsZ = 0;
float gx_for_Madgwick, gy_for_Madgwick, gz_for_Madgwick;

#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_PWR_MGMT_1 0x6b

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;

float volA_norm, volB_norm;

// Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw;
double roll_PID, pitch_PID, yaw_PID;

// Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;

// Radio communication:
unsigned long PWM_throttle, PWM_roll, PWM_Elevation, PWM_Rudd, PWM_ThrottleCutSwitch;
unsigned long PWM_throttle_prev, PWM_roll_prev, PWM_Elevation_prev, PWM_Rudd_prev;
unsigned long PWM_throttle_output, PWM_roll_output, PWM_Elevation_output, PWM_Rudd_output;
unsigned long volA, volB;

int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

// 関数を宣言
//  プロトタイプ宣言（関数宣言）
void calibrateESCs();
void setMotorPWM(int m1, int m2, int m3, int m4, bool cal);
void loopDrone();
void showRecievedData();
void getIMUdata();
void ComplementaryFilter();
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az);
void getDesiredAnglesAndThrottle();
void PIDControlCalcs();
void controlMixer();
void scaleCommands();
void commandMotors();
void getRadioSticks();
void printAcc();
void printGyro();
void printRollPitchYaw();
void printPIDoutput();
void printMotorCommands();
void printDes();
void printYawPID();
void printRollPID();
void showRecievedData();

// 割り込み関数
void IRAM_ATTR ppmInterrupt()
{
  unsigned long pulseTime = micros();                   // 現在の時間を取得
  unsigned long pulseWidth = pulseTime - lastPulseTime; // パルス幅を計算
  lastPulseTime = pulseTime;

  if (pulseWidth > SYNC_GAP)
  {                     // 同期信号を検出
    currentChannel = 0; // チャネルをリセット
  }
  else
  {
    if (currentChannel < CHANNELS)
    {                                             // 有効なチャネル範囲内であれば
      channelValues[currentChannel] = pulseWidth; // チャネル値を格納
      currentChannel++;                           // 次のチャネルへ
    }
  }
}

void AcceleroMeterAddressSetup()
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
        MPU6050_ADDR = address;
      nDevices++;
    }
  }
}

void AcceleroMeterWireRead()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  raw_t = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();
}

void calcRotation()
{
  AccX = (((float)raw_acc_x) / 16384.0);
  AccY = (((float)raw_acc_y) / 16384.0);
  AccZ = (((float)raw_acc_z) / 16384.0);
  acc_angle_y = atan2(AccX, AccZ + abs(AccY)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(AccY, AccZ + abs(AccX)) * 360 / 2.0 / PI;
  dpsX = (((float)raw_gyro_x) / 65.5);
  dpsY = (((float)raw_gyro_y) / 65.5);
  dpsZ = (((float)raw_gyro_z) / 65.5);
  interval = millis() - preInterval;
  preInterval = millis();
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.001);
  angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
  angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
  angleZ = gyro_angle_z;
  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;
  GyroX = init_angleX - angleX;
  GyroY = -(init_angleY - angleY);
  GyroZ = init_angleZ - angleZ;
}

void ShowGyro()
{
  float sg_gx = dpsX - init_dpsX;
  float sg_gy = dpsY - init_dpsY;
  float sg_gz = dpsZ - init_dpsZ;
  Serial.print("gx: ");
  Serial.print(sg_gx);

  Serial.print("gy: ");
  Serial.print(sg_gy);

  Serial.print("gz: ");
  Serial.println(sg_gz);
}

void writeMPU6050(byte reg, byte data)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void AcceleroMeterAngleSetup()
{
  AcceleroMeterAddressSetup();
  AcceleroMeterWireRead();
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);

  Serial.print("Calculating Calibration");
  for (int i = 0; i < 1200; i++)
  {
    AcceleroMeterWireRead();
    dpsX = ((float)raw_gyro_x) / 65.5;
    dpsY = ((float)raw_gyro_y) / 65.5;
    dpsZ = ((float)raw_gyro_z) / 65.5;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;

    if (i % 400 == 0)
    {
      Serial.print(".");
    }
  }
  Serial.println();
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

  Serial.print("Calculating Rotation");

  float sum_angleX = 0;
  float sum_angleY = 0;
  float sum_angleZ = 0;

  for (int i = 0; i < 1200; i++)
  {
    calcRotation();

    // 角度の累積
    sum_angleX += angleX;
    sum_angleY += angleY;
    sum_angleZ += angleZ;

    // プログレスを表示
    if (i % 400 == 0)
    {
      Serial.print(".");
    }
  }
  Serial.println();

  init_angleX = sum_angleX / 1200;
  init_angleY = sum_angleY / 1200;
  init_angleZ = sum_angleZ / 1200;

  Serial.println("Initial angles:");
  Serial.print("Initial Angle X: ");
  Serial.print(init_angleX);
  Serial.print("| Initial Angle Y: ");
  Serial.print(init_angleY);
  Serial.print("| Initial Angle Z: ");
  Serial.println(init_angleZ);
  Serial.println();

  float sum_AccX = 0;
  float sum_AccY = 0;
  float sum_AccZ = 0;
  float sum_dpsX = 0;
  float sum_dpsY = 0;
  float sum_dpsZ = 0;

  Serial.print("Calculating Acceleration");
  for (int i = 0; i < 1200; i++)
  {
    calcRotation();

    sum_AccX += AccX;
    sum_AccY += AccY;
    sum_AccZ += AccZ;
    sum_dpsX += dpsX;
    sum_dpsY += dpsY;
    sum_dpsZ += dpsZ;

    if (i % 400 == 0)
    {
      Serial.print(".");
    }
  }
  Serial.println();

  init_AccX = sum_AccX / 1200;
  init_AccY = sum_AccY / 1200;
  init_AccZ = sum_AccZ / 1200;

  init_dpsX = sum_dpsX / 1200;
  init_dpsY = sum_dpsY / 1200;
  init_dpsZ = sum_dpsZ / 1200;

  Serial.println("Initial acc");
  Serial.print("Initial AccX: ");
  Serial.print(init_AccX);
  Serial.print("| Initial AccY : ");
  Serial.print(init_AccY);
  Serial.print("| Initial AccZ: ");
  Serial.print(init_AccZ);
  Serial.print("| Initial dpsX: ");
  Serial.print(init_dpsX);
  Serial.print("| Initial dpsY: ");
  Serial.print(init_dpsY);
  Serial.print("| Initial dosZ: ");
  Serial.print(init_dpsZ);
  Serial.println();
}

void setup()
{
  Serial.begin(115200);

  emergency = false;

  // ピンモードを設定
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);

  // レシーバー
  pinMode(PPM_PIN, INPUT_PULLUP);                 // ピンを入力モードに設定
  attachInterrupt(PPM_PIN, ppmInterrupt, RISING); // 割り込みを設定
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
  AcceleroMeterAngleSetup();

  // Madgwickフィルタの初期化
  MadgwickFilter.begin(100);

  // 各モーターをピンにアタッチ
  ESC1.attach(m1Pin);
  ESC2.attach(m2Pin);
  ESC3.attach(m3Pin);
  ESC4.attach(m4Pin);

  Serial.println("PWM successfully attached to all motors");

  calibrateESCs();
}

void loop()
{

  currentMillis = millis();

  if (currentMillis - previousMillis > 0)
  {
    deltaTime = (currentMillis - previousMillis) / 1000.0;
    // Calculate the frame rate in frames per second (FPS)
    frameRate = 1000.0 / (currentMillis - previousMillis);
    previousMillis = currentMillis;
  }

  loopDrone();
}

void loopDrone()
{
  // showRecievedData();
  getIMUdata();
  ComplementaryFilter(); // Pulls raw gyro andaccelerometer data from IMU and applies LP filters to remove noise
  // Madgwick6DOF(gx_for_Madgwick, gy_for_Madgwick, gz_for_Madgwick, AccX, AccY, AccZ); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  getDesiredAnglesAndThrottle(); // Convert raw commands to normalized values based on saturated control limits
  PIDControlCalcs();             // The PID functions. Stabilize on angle setpoint from getDesiredAnglesAndThrottle
  controlMixer();                // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands();               // Scales motor commands to 0-1
  commandMotors();               // Sends command pulses to each ESC pin to drive the motors
  getRadioSticks();              // Gets the PWM from the radio receiver

  // printAcc();
  // printGyro();
  // printRollPitchYaw();
  //  printPIDoutput();
  // printYawPID();
  printRollPID();
  // printDes();
  // printMotorCommands();
  // ShowGyro();
}

int getChannelValue(int channelIndex)
{
  if (channelIndex >= 0 && channelIndex < CHANNELS)
  {
    return channelValues[channelIndex];
  }
  else
  {
    Serial.print("Error: Invalid channel index ");
    Serial.println(channelIndex);
    return 0; // 無効なインデックスの場合はデフォルト値を返す
  }
}

void showRecievedData()
{
  // 各チャネルの値をシリアルモニターに表示
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

void getIMUdata()
{
  AcceleroMeterWireRead();
  calcRotation();

  // ジャイロデータをラジアン毎秒に変換
  gx_for_Madgwick = dpsX * DEG_TO_RAD;
  gy_for_Madgwick = dpsY * DEG_TO_RAD;
  gz_for_Madgwick = dpsZ * DEG_TO_RAD;

  // 加速度データの正規化
  float norm = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
  if (norm != 0)
  {
    AccX /= norm;
    AccY /= norm;
    AccZ /= norm;
  }
}

void ComplementaryFilter()
{
  // 相補性フィルター
  roll_IMU = alpha * (roll_IMU + dpsX * deltaTime) + (1 - alpha) * GyroX;
  pitch_IMU = alpha * (pitch_IMU + dpsY * deltaTime) + (1 - alpha) * GyroY;
  yaw_IMU = alpha * (yaw_IMU + dpsZ * deltaTime) + (1 - alpha) * GyroZ;
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az)
{
  MadgwickFilter.updateIMU(gx, gy, gz, ax, ay, az);

  // roll_IMU = -(MadgwickFilter.getRoll() - init_angleX);
  // pitch_IMU = MadgwickFilter.getPitch() - init_angleY;
  roll_IMU = GyroX;
  pitch_IMU = GyroY;
  yaw_IMU = GyroZ;

  if (abs(roll_IMU) >= 40 || abs(pitch_IMU) >= 40)
  {
    emergency = true;
  }
}

void getDesiredAnglesAndThrottle()
{

  // 入力PWM値から計算された目標値
  thro_des = (PWM_throttle - 1000.0) / 1000.0;
  roll_des = (PWM_roll - 1500.0) / 500.0;
  pitch_des = -((PWM_Elevation - 1500.0) / 500.0);
  yaw_des = -(PWM_Rudd - 1500.0) / 500.0;

  // ローパスフィルター適用
  thro_des = alpha_des * thro_des + (1.0 - alpha_des) * thro_pre;
  roll_des = alpha_des * roll_des + (1.0 - alpha_des) * roll_pre;
  pitch_des = alpha_des * pitch_des + (1.0 - alpha_des) * pitch_pre;
  yaw_des = alpha_des * yaw_des + (1.0 - alpha_des) * yaw_pre;

  thro_pre = thro_des;
  roll_pre = roll_des;
  pitch_pre = pitch_des;
  yaw_pre = yaw_des;

  // Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0) * throttle_limit;           // Between 0 and 1
  roll_des = constrain(roll_des + hoverRoll, -1.0, 1.0) * maxRoll;     // Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des + hoverPitch, -1.0, 1.0) * maxPitch; // Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des + hoverYaw, -1.0, 1.0) * maxYaw;         // Between -maxYaw and +maxYaw
}

void PIDControlCalcs()
{

  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * deltaTime;
  integral_roll = constrain(integral_roll, -i_limit, i_limit);  // Limit integrator to prevent saturating
  derivative_roll = (error_roll - error_roll_prev) / deltaTime; // deg/sec
  derivative_roll = alpha_derivative * derivative_roll + (1.0 - alpha_des) * derivative_roll_pre;
  derivative_roll_pre = derivative_roll;

  roll_PID = (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll + Kd_roll_angle * derivative_roll);
  roll_PID -= rollPIDError;
  roll_PID = constrain(roll_PID, -PID_Limit / roll_Weight, PID_Limit / roll_Weight);

  Out_ProportionalBand_Roll = (Kp_roll_angle * Roll_ProportionalBand);
  Out_ProportionalBand_Roll = constrain(Out_ProportionalBand_Roll, -PID_Limit / roll_Weight, PID_Limit / roll_Weight);

  if (error_roll > Roll_ProportionalBand)
  {
    roll_PID = Out_ProportionalBand_Roll;
  }
  else if (error_roll < -Roll_ProportionalBand)
  {
    roll_PID = -Out_ProportionalBand_Roll;
  }

  // Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * deltaTime;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
  derivative_pitch = (error_pitch - error_pitch_prev) / deltaTime;
  derivative_pitch = alpha_derivative * derivative_pitch + (1.0 - alpha_des) * derivative_pitch_pre;
  derivative_pitch_pre = derivative_pitch;
  pitch_PID = (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch + Kd_pitch_angle * derivative_pitch);
  pitch_PID -= pitchPIDError;
  pitch_PID = constrain(pitch_PID, -PID_Limit / pitch_Weight, PID_Limit / pitch_Weight);

  Out_ProportionalBand_Pitch = (Kp_pitch_angle * Pitch_ProportionalBand);
  Out_ProportionalBand_Pitch = constrain(Out_ProportionalBand_Pitch, -PID_Limit / pitch_Weight, PID_Limit / pitch_Weight);

  if (error_pitch > Pitch_ProportionalBand)
  {
    pitch_PID = Out_ProportionalBand_Pitch;
  }
  else if (error_pitch < -Pitch_ProportionalBand)
  {
    pitch_PID = -Out_ProportionalBand_Pitch;
  }

  // Yaw
  error_yaw = yaw_des - (dpsZ - init_dpsZ);
  integral_yaw = integral_yaw_prev + error_yaw * deltaTime;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
  derivative_yaw = -(error_yaw - error_yaw_prev) / deltaTime;
  derivative_yaw = alpha_derivative * derivative_yaw + (1.0 - alpha_des) * derivative_yaw_pre;
  derivative_yaw_pre = derivative_yaw;
  yaw_PID = yaw_Weight * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);
  yaw_PID = constrain(yaw_PID, -PID_Limit, PID_Limit);

  // Update roll variables
  integral_roll_prev = integral_roll;
  integral_pitch_prev = integral_pitch;
  error_yaw_prev = error_yaw;
  error_pitch_prev = error_pitch;
  error_roll_prev = error_roll;
  integral_yaw_prev = integral_yaw;
}

void controlMixer()
{
  m1_command_scaled = (thro_des) + PID_Adjuster * (-pitch_Weight * pitch_PID + roll_Weight * roll_PID + yaw_PID);
  m2_command_scaled = (thro_des) + PID_Adjuster * (-pitch_Weight * pitch_PID - roll_Weight * roll_PID - yaw_PID);
  m3_command_scaled = (thro_des) + PID_Adjuster * (pitch_Weight * pitch_PID - roll_Weight * roll_PID + yaw_PID);
  m4_command_scaled = (thro_des) + PID_Adjuster * (pitch_Weight * pitch_PID + roll_Weight * roll_PID - yaw_PID);

  m1_command_scaled = constrain(m1_command_scaled, 0, 1.0);
  m2_command_scaled = constrain(m2_command_scaled, 0, 1.0);
  m3_command_scaled = constrain(m3_command_scaled, 0, 1.0);
  m4_command_scaled = constrain(m4_command_scaled, 0, 1.0);
}

void scaleCommands()
{
  // DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  // Scale to Servo PWM 0-180 degrees for stop to full speed.  No need to constrain since mx_command_scaled already is.
  m1_command_PWM = m1_command_scaled * throttle_max;
  m2_command_PWM = m2_command_scaled * throttle_max;
  m3_command_PWM = m3_command_scaled * throttle_max;
  m4_command_PWM = m4_command_scaled * throttle_max;
}

void getRadioSticks()
{
  // 各PWM入力値を channelValues 配列から割り当て
  PWM_throttle = getChannelValue(2);
  PWM_roll = getChannelValue(0);
  PWM_Elevation = getChannelValue(1);
  PWM_Rudd = getChannelValue(3);

  if (getChannelValue(6) <= 1500)
  {
    keep_rotating = true;
  }
  else
  {
    keep_rotating = false;
  }

  PWM_throttle_output = PWM_throttle;
  PWM_roll_output = PWM_roll;
  PWM_Elevation_output = PWM_Elevation;
  PWM_Rudd_output = PWM_Rudd;

  PWM_throttle = (stick_dampener)*PWM_throttle_prev + (1 - stick_dampener) * PWM_throttle;
  PWM_roll = (1.0 - stick_dampener) * PWM_roll_prev + stick_dampener * PWM_roll;
  PWM_Elevation = (1.0 - stick_dampener) * PWM_Elevation_prev + stick_dampener * PWM_Elevation;
  PWM_Rudd = (1.0 - stick_dampener) * PWM_Rudd_prev + stick_dampener * PWM_Rudd;

  PWM_throttle_prev = PWM_throttle;
  PWM_roll_prev = PWM_roll;
  PWM_Elevation_prev = PWM_Elevation;
  PWM_Rudd_prev = PWM_Rudd;
}

void commandMotors()
{
  // ベーススロットル値を追加
  m1_command_PWM += throttle_min;
  m2_command_PWM += throttle_min;
  m3_command_PWM += throttle_min;
  m4_command_PWM += throttle_min;

  // 各モーターのバッファ値を追加
  m1_command_PWM += morter1_buffer;
  m2_command_PWM += morter2_buffer;
  m3_command_PWM += morter3_buffer;
  m4_command_PWM += morter4_buffer;

  // PWM値を範囲内に制限
  m1_command_PWM = constrain(m1_command_PWM, throttle_min, throttle_Limit);
  m2_command_PWM = constrain(m2_command_PWM, throttle_min, throttle_Limit);
  m3_command_PWM = constrain(m3_command_PWM, throttle_min, throttle_Limit);
  m4_command_PWM = constrain(m4_command_PWM, throttle_min, throttle_Limit);

  // スロットルが0の場合モーターを回さないようにする
  if (thro_des <= 0.02 && zero_throttle_safty && !keep_rotating)
  {
    m1_command_PWM = throttle_min;
    m2_command_PWM = throttle_min;
    m3_command_PWM = throttle_min;
    m4_command_PWM = throttle_min;
  }

  // もしプロポとの通信が切れた場合モーターを停止する
  bool allZero = true;
  // チャンネル値をチェック
  for (int i = 0; i < CHANNELS; i++)
  {
    Serial.print(channelValues[i]);
    if (channelValues[i] != 0)
    {
      allZero = false;
    }
  }
  if (allZero)
  {
    m1_command_PWM = throttle_min;
    m2_command_PWM = throttle_min;
    m3_command_PWM = throttle_min;
    m4_command_PWM = throttle_min;

    // Serial.println("All channels are zero. Setting PWM to throttle_min.");
  }

  // モーターにPWMを設定
  setMotorPWM(m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, false);
}

void calibrateESCs()
{
  Serial.println("Starting calibration");

  setMotorPWM(throttle_min, throttle_min, throttle_min, throttle_min, true);
  Serial.println("Setting maximum throttle");
  delay(2000);
  Serial.println("Setting minimum throttle");
  delay(2000);
}

void setMotorPWM(int m1, int m2, int m3, int m4, bool cal)
{
  int duty1 = 0, duty2 = 0, duty3 = 0, duty4 = 0;

  if (!cal && keep_rotating)
  {
    duty1 = constrain(m1, throttle_min + min_rotation, throttle_max);
    duty2 = constrain(m2, throttle_min + min_rotation, throttle_max);
    duty3 = constrain(m3, throttle_min + min_rotation, throttle_max);
    duty4 = constrain(m4, throttle_min + min_rotation, throttle_max);
  }
  else
  {
    duty1 = constrain(m1, throttle_min, throttle_max);
    duty2 = constrain(m2, throttle_min, throttle_max);
    duty3 = constrain(m3, throttle_min, throttle_max);
    duty4 = constrain(m4, throttle_min, throttle_max);
  }

  if (emergency)
  {
    ESC1.writeMicroseconds(throttle_min);
    ESC2.writeMicroseconds(throttle_min);
    ESC3.writeMicroseconds(throttle_min);
    ESC4.writeMicroseconds(throttle_min);
  }
  else
  {
    ESC1.writeMicroseconds(duty1);
    ESC2.writeMicroseconds(duty2);
    ESC3.writeMicroseconds(duty3);
    ESC4.writeMicroseconds(duty4);
  }
}

void printRollPitchYaw()
{
  Serial.print(F(" roll_imu: "));
  Serial.print(roll_IMU);
  Serial.print(F(" pitch_imu: "));
  Serial.print(pitch_IMU);
  Serial.print(F(" yaw_imu: "));
  Serial.println(yaw_IMU);
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
  if (AccX >= 0)
    Serial.print("+"); // 正なら "+" を付ける
  Serial.print(AccX);

  Serial.print(F(" AccY: "));
  if (AccY >= 0)
    Serial.print("+");
  Serial.print(AccY);

  Serial.print(F(" AccZ: "));
  if (AccZ >= 0)
    Serial.print("+");
  Serial.println(AccZ);
}

void printGyro()
{
  Serial.print(F(" GyroX: "));
  Serial.print(GyroX);
  Serial.print(F(" GyroY: "));
  Serial.print(GyroY);
  Serial.print(F(" GyroZ: "));
  Serial.println(GyroZ);
}

void printMotorCommands()
{
  Serial.print(F("m1_command: "));
  Serial.print(m1_command_PWM);
  Serial.print(F("  : "));
  Serial.print(m1_command_scaled);

  Serial.print(F("   m2_command: "));
  Serial.print(m2_command_PWM);
  Serial.print(F("  : "));
  Serial.print(m2_command_scaled);

  Serial.print(F("   m3_command: "));
  Serial.print(m3_command_PWM);
  Serial.print(F("  :  "));
  Serial.print(m3_command_scaled);
  Serial.print(F("   m4_command: "));

  Serial.println(m4_command_PWM);
  Serial.print(F("  : "));
  Serial.print(m4_command_scaled);
}

void printPIDoutput()
{
  Serial.print(F("roll_PID: "));
  if (roll_PID >= 0)
    Serial.print("+"); // 正の値に "+" を付ける
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
  if (roll_IMU >= 0)
    Serial.print("+");
  Serial.print(roll_IMU);

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

float invSqrt(float x)
{
  return 1.0 / sqrtf(x); // Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}