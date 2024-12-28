#include <Wire.h>
#include <MadgwickAHRS.h>
#include <MPU6050.h>
#include <Arduino.h>
#include <SPI.h>

// SDA（データライン）: GPIO21
// SCL（クロックライン）: GPIO22

#define CHANNELS 6    // 使用するチャネル数
#define SYNC_GAP 3000 // 同期信号判定のしきい値 (マイクロ秒)
#define PPM_PIN 4     // PPM信号の入力ピン

const int ledPin1 = 18; // LED1が接続されているピン番号
const int ledPin2 = 19; // LED2が接続されているピン番号
// 状態を定義
enum Status
{
  All_On,
  All_Off,
  Slow_Blink,
  Alternating_1,
  Alternating_2,
  Alternating_3,
  Flash
};
Status currentStatus = All_On; // 初期ステータス

MPU6050 mpu;
Madgwick MadgwickFilter;
volatile unsigned long lastPulseTime = 0;   // 前回のパルス時間
volatile int channelValues[CHANNELS] = {0}; // 各チャネルの値を格納
volatile int currentChannel = 0;            // 現在のチャネルインデックス

// The LOOP_TIMING is based on the IMU.  For the Arduino_LSM6DSOX, it is 104Hz.  So, the loop time is set a little longer so the IMU has time to update from the control change.
#define LOOP_TIMING 100

// モーターピン定義
#define m1Pin 32
#define m2Pin 25
#define m3Pin 26
#define m4Pin 27

// PWM設定
const int pwmFrequency = 50;  // 50Hz (20ms周期、一般的なESCに対応)
const int pwmResolution = 16; // 16ビット解像度

const int throttle_max = 1923; // 最大PWM
const int throttle_min = 900;  // 最小PWM

// madgwick
float B_madgwick = 0.04; //(default 0.04)
float q0 = 1.0f;         // Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

// Controller parameters (this is where you "tune it".  It's best to use the WiFi interface to do it live and then update once its tuned.):
float i_limit = 20;    // Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 18.0;  // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxPitch = 18.0; // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxYaw = 10.0;   // Max yaw rate in deg/sec (default 160.0)
float maxMotor = 0.8;
float Kp_range = 20;
float Kd_range = 5;

float parameter_rate = 1.0;

float Kp_roll_angle = 6.06 * parameter_rate; // Roll P-gain
float Ki_roll_angle = 0.00 * parameter_rate; // Roll I-gain
float Kd_roll_angle = 0.83 * parameter_rate; // Roll D-gain

float Kp_pitch_angle = 6.06 * parameter_rate; // Pitch P-gain
float Ki_pitch_angle = 0.00 * parameter_rate; // Pitch I-gain
float Kd_pitch_angle = 0.83 * parameter_rate; // Pitch D-gain

float Kp_yaw = 0; // Yaw P-gain default 30
float Ki_yaw = 0; // Yaw I-gain default 5
float Kd_yaw = 0; // Yaw D-gain default .015 (be careful when increasing too high, motors will begin to overheat!)

float stick_dampener = 0.3;

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

float AccErrorX = 0.03;
float AccErrorY = 0.02;
float AccErrorZ = 0.01;
float GyroErrorX = 0.9;
float GyroErrorY = -0.40;
float GyroErrorZ = 0.0;

float RollError = -8.0;
float PitchError = 4.0;

float rollPIDError = -0.02;
float pitchPIDError = 0.00;

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;

float volA_norm, volB_norm;

// Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

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
void setMotorPWM(int m1, int m2, int m3, int m4);
void loopDrone();
void showRecievedData();
void getIMUdata();
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

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue)
{
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100)
    return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Red the channel and return a boolean value
bool redSwitch(byte channelInput, bool defaultValue)
{
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

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


// 点滅パターンを設定する関数
void setLedPattern(Status status)
{
  switch (status)
  {
  case All_On:
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    break;

  case All_Off:
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    break;

  case Slow_Blink:
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    delay(1000);
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    delay(1000);
    break;

  case Alternating_1:
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
    delay(100);
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH);
    delay(100);
    break;

  case Alternating_2:
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
    delay(100);
    digitalWrite(ledPin1, LOW);
    delay(100);
    digitalWrite(ledPin1, HIGH);
    delay(100);

    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH);
    delay(100);
    digitalWrite(ledPin2, LOW);
    delay(100);
    digitalWrite(ledPin2, HIGH);
    delay(100);
    break;

  case Alternating_3:
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
    delay(100);
    digitalWrite(ledPin1, LOW);
    delay(100);
    digitalWrite(ledPin1, HIGH);
    delay(100);
    digitalWrite(ledPin1, LOW);
    delay(100);
    digitalWrite(ledPin1, HIGH);
    delay(100);

    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH);
    delay(100);
    digitalWrite(ledPin2, LOW);
    delay(100);
    digitalWrite(ledPin2, HIGH);
    delay(100);
    digitalWrite(ledPin2, LOW);
    delay(100);
    digitalWrite(ledPin2, HIGH);
    delay(100);
    break;
    ;

  case Flash:
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
    delay(50);
    digitalWrite(ledPin1, LOW);
    delay(1000);

    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin1, LOW);
    delay(50);
    digitalWrite(ledPin2, LOW);
    delay(1000);
    break;
  }
}


void setup()
{
  Serial.begin(115200);

  // ピンモードを設定
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  // 初期ステータスを設定
  currentStatus = All_On;
  setLedPattern(currentStatus);

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

  // Madgwickフィルタの初期化
  MadgwickFilter.begin(50);
  // PWM出力の初期化（ledcAttachPinを使用）
  ledcSetup(0, pwmFrequency, pwmResolution);
  ledcAttachPin(m1Pin, 0);

  ledcSetup(1, pwmFrequency, pwmResolution);
  ledcAttachPin(m2Pin, 1);

  ledcSetup(2, pwmFrequency, pwmResolution);
  ledcAttachPin(m3Pin, 2);

  ledcSetup(3, pwmFrequency, pwmResolution);
  ledcAttachPin(m4Pin, 3);

  Serial.println("PWM successfully attached to all motors");

  calibrateESCs();

  // 全てのモーターを最小値で初期化
  setMotorPWM(throttle_min, throttle_min, throttle_min, throttle_min);
  delay(2000); // 安定のための遅延
  currentStatus = Flash;
}

void loop()
{

  // 現在のステータスに応じてLEDパターンを変更
  setLedPattern(currentStatus);
  currentMillis = millis();

  if (currentMillis - previousMillis > 0)
  {
    // Calculate the frame rate in frames per second (FPS)
    frameRate = 1000.0 / (currentMillis - previousMillis);

    // Print the frame rate to the serial monitor
    //Serial.println(frameRate);

    // Update previousMillis for the next loop
    previousMillis = currentMillis;
  }

  loopDrone();
}

void loopDrone()
{
  showRecievedData();
  getIMUdata();                                           // Pulls raw gyro andaccelerometer data from IMU and applies LP filters to remove noise
  Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  getDesiredAnglesAndThrottle();                          // Convert raw commands to normalized values based on saturated control limits
  PIDControlCalcs();                                      // The PID functions. Stabilize on angle setpoint from getDesiredAnglesAndThrottle
  controlMixer();                                         // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands();                                        // Scales motor commands to 0-1
  commandMotors();                                        // Sends command pulses to each ESC pin to drive the motors
  getRadioSticks();                                       // Gets the PWM from the radio receiver

  //printAcc();
  //printGyro();
  //printRollPitchYaw();
  //printPIDoutput();
  //printMotorCommands();
}

// チャネル値を取得するヘルパー関数
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
  // DESCRIPTION: Request full dataset from MPU6050

  int16_t ax, ay, az, gx, gy, gz;

  // MPU6050からデータを取得
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 加速度データをgに変換し、誤差を補正
  AccX = (ax / 16384.0) - AccErrorX;
  AccY = (ay / 16384.0) - AccErrorY;
  AccZ = (az / 16384.0) - AccErrorZ;

  // ジャイロデータを°/sに変換し、誤差を補正
  GyroX = (gx / 131.0) - GyroErrorX;
  GyroY = (gy / 131.0) - GyroErrorY;
  GyroZ = (gz / 131.0) - GyroErrorZ;
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az)
{
  MadgwickFilter.updateIMU(gx, gy, gz, ax, ay, az);
  roll_IMU = MadgwickFilter.getRoll() - RollError;
  pitch_IMU = -MadgwickFilter.getPitch() - PitchError;
  yaw_IMU = MadgwickFilter.getYaw();
}

void getDesiredAnglesAndThrottle()
{
  thro_des = (PWM_throttle - 1000.0) / 1000.0;  // Between 0 and 1
  roll_des = (PWM_roll - 1482.0) / 500.0;       // Between -1 and 1
  pitch_des = (PWM_Elevation - 1487.0) / 500.0; // Between -1 and 1
  yaw_des = (PWM_Rudd - 1485.0) / 500.0;        // Between -1 and 1

  // Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0) * 0.6;         // Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;    // Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch; // Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;       // Between -maxYaw and +maxYaw
}

void PIDControlCalcs()
{
  // if (PWM_throttle < 1300)
  // { //This will keep the motors from spinning with the throttle at zero should the drone be sitting unlevel.
  //   integral_roll_prev = 0;
  //   integral_pitch_prev = 0;
  //   error_yaw_prev = 0;
  //   integral_yaw_prev = 0;
  //   roll_PID=0;
  //   pitch_PID=0;
  //   yaw_PID=0;
  //   return;
  // }

  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * deltaTime;
  integral_roll = constrain(integral_roll, -i_limit, i_limit);                                                        // Limit integrator to prevent saturating
  derivative_roll = GyroX;                                                                                            //(roll_des-roll_IMU-roll_des-previous_IMU)/dt=current angular velocity since last IMU read and therefore GyroX in deg/s
  roll_PID = 0.0001 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll); // Scaled by .0001 to bring within -1 to 1 range
  roll_PID -= rollPIDError;

  // Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * deltaTime;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
  derivative_pitch = GyroY;
  pitch_PID = .0001 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch); // Scaled by .0001 to bring within -1 to 1 range
  pitch_PID -= pitchPIDError;

  // Yaw, stablize on rate from GyroZ versus angle.  In other words, your stick is setting y axis rotation speed - not the angle to get to.
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * deltaTime;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);
  derivative_yaw = (error_yaw - error_yaw_prev) / deltaTime;
  yaw_PID = .0001 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); // Scaled by .0001 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev = integral_roll;
  integral_pitch_prev = integral_pitch;
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlMixer()
{
  // DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration

  // Quad mixing. maxMotor is used to keep the motors from being too violent if you have a big battery and concers about that.
  m1_command_scaled = maxMotor * (thro_des)-pitch_PID + roll_PID + yaw_PID;   // Front left
  m2_command_scaled = maxMotor * (thro_des)-pitch_PID - roll_PID - yaw_PID;   // Front right
  m3_command_scaled = maxMotor * (thro_des) + pitch_PID - roll_PID + yaw_PID; // Back Right
  m4_command_scaled = maxMotor * (thro_des) + pitch_PID + roll_PID - yaw_PID; // Back Left

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

  PWM_throttle_output = PWM_throttle;
  PWM_roll_output = PWM_roll;
  PWM_Elevation_output = PWM_Elevation;
  PWM_Rudd_output = PWM_Rudd;

  // Low-pass the critical commands and update previous values
  if (PWM_throttle - PWM_throttle_prev < 0)
  {
    // Going down  - slow
    PWM_throttle = (.95) * PWM_throttle_prev + 0.05 * PWM_throttle;
  }
  else
  { // Going up - fast
    PWM_throttle = (stick_dampener)*PWM_throttle_prev + (1 - stick_dampener) * PWM_throttle;
  }
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
  m1_command_PWM += 1040;
  m2_command_PWM += 980;
  m3_command_PWM += 980;
  m4_command_PWM += 910;

  setMotorPWM(m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM);
}
void calibrateESCs()
{
  // ESCキャリブレーション用に全てのモーターを最大スロットルに設定
  setMotorPWM(throttle_max, throttle_max, throttle_max, throttle_max);
  delay(2000);

  // ESCキャリブレーション用に全てのモーターを最小スロットルに設定
  setMotorPWM(throttle_min, throttle_min, throttle_min, throttle_min);
  delay(2000);
}

// モーターPWM信号を設定する関数
void setMotorPWM(int m1, int m2, int m3, int m4)
{
  int duty1 = map(m1, 1000, 2000, 0, 65535);
  int duty2 = map(m2, 1000, 2000, 0, 65535);
  int duty3 = map(m3, 1000, 2000, 0, 65535);
  int duty4 = map(m4, 1000, 2000, 0, 65535);

  // 修正: PWMチャネル (0～3) を指定
  ledcWrite(0, duty1);
  ledcWrite(1, duty2);
  ledcWrite(2, duty3);
  ledcWrite(3, duty4);
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

void printAcc()
{
  Serial.print(F(" AccX: "));
  Serial.print(AccX);
  Serial.print(F(" AccY: "));
  Serial.print(AccY);
  Serial.print(F(" AccZ: "));
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
  // Serial.print(m1_command_scaled);
  Serial.print(F(" m2_command: "));
  Serial.print(m2_command_PWM);
  // Serial.print(m2_command_scaled);
  Serial.print(F(" m3_command: "));
  Serial.print(m3_command_PWM);
  // Serial.print(m3_command_scaled);
  Serial.print(F(" m4_command: "));
  Serial.println(m4_command_PWM);
  // Serial.print(m4_command_scaled);
}

void printPIDoutput()
{
  Serial.print(F("roll_PID: "));
  Serial.print(roll_PID);
  Serial.print(F(" pitch_PID: "));
  Serial.print(pitch_PID);
  Serial.print(F(" yaw_PID: "));
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

float invSqrt(float x)
{
  return 1.0 / sqrtf(x); // Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}
