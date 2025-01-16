//THERE IS NO WARRANTY FOR THE SOFTWARE, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR
//OTHER PARTIES PROVIDE THE SOFTWARE “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
//OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH THE CUSTOMER. SHOULD THE
//SOFTWARE PROVE DEFECTIVE, THE CUSTOMER ASSUMES THE COST OF ALL NECESSARY SERVICING, REPAIR, OR CORRECTION EXCEPT TO THE EXTENT SET OUT UNDER THE HARDWARE WARRANTY IN THESE TERMS.

#include <Wire.h>
#include <ESP32Servo.h>  // Change to the standard Servo library for ESP32

volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
const int mot1_pin = 25;
const int mot2_pin = 26;
const int mot3_pin = 27;
const int mot4_pin = 32;

const int ledPin1 = 18;

#define CHANNELS 8                             // 使用するチャネル数
#define SYNC_GAP 3000                          // 同期信号判定のしきい値 (マイクロ秒)
#define PPM_PIN 4                              // PPM信号の入力ピン
volatile unsigned long lastPulseTime = 0;      // 前回のパルス時間
volatile int channelValues[CHANNELS] = { 0 };  // 各チャネルの値を格納
volatile int currentChannel = 0;               // 現在のチャネルインデックス

// The LOOP_TIMING is based on the IMU.  For the Arduino_LSM6DSOX, it is 104Hz.  So, the loop time is set a little longer so the IMU has time to update from the control change.
#define LOOP_TIMING 100

// General stuff for controlling timing of things
float deltaTime = 1;
float invFreq = (1.0 / LOOP_TIMING) * 1000000.0;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;

unsigned long previousMillis = 0;
unsigned long currentMillis;  // Declare currentMillis as a global variable
float frameRate;

// float voltage;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = { 0, 0, 0 };

// float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch;
// float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
// float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
// float Kalman1DOutput[]={0,0};

float PRateRoll = 0.75;  //For outdoor flights, keep this gain to 0.75 and for indoor flights keep the gain to be 0.6
float IRateRoll = 0.012;
float DRateRoll = 0.0085;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4.2;
float IRateYaw = 2.8;
float DRateYaw = 0;

// Controller parameters (this is where you "tune it".  It's best to use the WiFi interface to do it live and then update once its tuned.):
float i_limit = 400;     // Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;   // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxPitch = 30.0;  // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode (default 30.0)
float maxYaw = 160.0;   // Max yaw rate in deg/sec (default 160.0)
const int throttle_limit = 1800;
const int throttle_max = 2000;  // 最大PWM
const int throttle_min = 1000;  // 最小PWM
int ThrottleIdle = 1170;

uint32_t LoopTimer;
float t = 0.006;  //time cycle

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = { 0, 0 };
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2;
float PAnglePitch = PAngleRoll;
float IAngleRoll = 0;
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0;
float DAnglePitch = DAngleRoll;
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4);                     //here 4 is the vairnece of IMU i.e 4 deg/s
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);  //std deviation of error is 3 deg
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// 割り込み関数
void IRAM_ATTR ppmInterrupt() {
  unsigned long pulseTime = micros();                    // 現在の時間を取得
  unsigned long pulseWidth = pulseTime - lastPulseTime;  // パルス幅を計算
  lastPulseTime = pulseTime;

  if (pulseWidth > SYNC_GAP) {  // 同期信号を検出
    currentChannel = 0;         // チャネルをリセット
  } else {
    if (currentChannel < CHANNELS) {               // 有効なチャネル範囲内であれば
      channelValues[currentChannel] = pulseWidth;  // チャネル値を格納
      currentChannel++;                            // 次のチャネルへ
    }
  }
}
int getChannelValue(int channelIndex) {
  if (channelIndex >= 0 && channelIndex < CHANNELS) {

    // channelValues[channelIndex] = constrain(channelValues[channelIndex], 1000, 2000);
    return channelValues[channelIndex];
  } else {
    Serial.print("Error: Invalid channel index ");
    Serial.println(channelIndex);
    return 0;
  }
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = -(float)GyroX / 65.5;
  RatePitch = -(float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = -(float)AccXLSB / 4096;
  AccY = -(float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  AccZ = AccZ - 0;  // calibration offset
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + (I * (Error + PrevError) * (t / 2));
  if (Iterm > i_limit) {
    Iterm = i_limit;
  } else if (Iterm < -i_limit) {
    Iterm = -i_limit;
  }
  float Dterm = D * ((Error - PrevError) / t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > i_limit) {
    PIDOutput = i_limit;
  } else if (PIDOutput < -i_limit) {
    PIDOutput = -i_limit;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;
  PrevErrorAngleRoll = 0;
  PrevErrorAnglePitch = 0;
  PrevItermAngleRoll = 0;
  PrevItermAnglePitch = 0;
}


void calibrateESCs()
{
  Serial.println("Starting calibration");
  mot1.write(map(throttle_max, throttle_min, throttle_max, 0, 180));
  mot2.write(map(throttle_max, throttle_min, throttle_max, 0, 180));
  mot3.write(map(throttle_max, throttle_min, throttle_max, 0, 180));
  mot4.write(map(throttle_max, throttle_min, throttle_max, 0, 180));

  Serial.println("Setting maximum throttle");
  delay(2000);
  
  mot1.write(map(throttle_min, throttle_min, throttle_max, 0, 180));
  mot2.write(map(throttle_min, throttle_min, throttle_max, 0, 180));
  mot3.write(map(throttle_min, throttle_min, throttle_max, 0, 180));
  mot4.write(map(throttle_min, throttle_min, throttle_max, 0, 180));
  Serial.println("Setting minimum throttle");
  delay(2000);
}

void setup(void) {

  Serial.begin(115200);

  Serial.println("");
  pinMode(PPM_PIN, INPUT_PULLUP);                  // ピンを入力モードに設定
  attachInterrupt(PPM_PIN, ppmInterrupt, RISING);  // 割り込みを設定
  Serial.println("PPM Receiver Initialized");


  int led_time = 100;
  pinMode(ledPin1, OUTPUT);
  digitalWrite(ledPin1, LOW);
  delay(led_time);
  digitalWrite(ledPin1, HIGH);
  delay(led_time);
  digitalWrite(ledPin1, LOW);
  delay(led_time);
  digitalWrite(ledPin1, HIGH);
  delay(led_time);
  digitalWrite(ledPin1, LOW);
  delay(led_time);
  digitalWrite(ledPin1, HIGH);
  delay(led_time);
  digitalWrite(ledPin1, LOW);
  delay(led_time);
  digitalWrite(ledPin1, HIGH);
  delay(led_time);
  digitalWrite(ledPin1, LOW);
  delay(led_time);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();


  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);
  Serial.println("PWM successfully attached to all motors");

  calibrateESCs();


  int total = 1000;
  for (RateCalibrationNumber = 0; RateCalibrationNumber < total; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= total;
  RateCalibrationPitch /= total;
  RateCalibrationYaw /= total;
  //Gyro Calibrated Values
  // Serial.print("Gyro Calib: ");
  // Serial.print(RateCalibrationRoll);
  // Serial.print("  ");
  // Serial.print(RateCalibrationPitch);
  // Serial.print("  ");
  // Serial.print(RateCalibrationYaw);
  // Serial.print(" -- ");


  digitalWrite(ledPin1, HIGH);
  delay(1000);
  digitalWrite(ledPin1, LOW);
  delay(1000);
  digitalWrite(ledPin1, HIGH);
  delay(1000);
  digitalWrite(ledPin1, LOW);
  delay(1000);

  LoopTimer = micros();
}

void loop(void) {
  //enter your loop code here
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;


  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  DesiredAngleRoll = (getChannelValue(0) - 1500.0) / 500.0;   // Between -1 and 1
  DesiredAnglePitch = (getChannelValue(1) - 1500.0) / 500.0;  // Between -1 and 1
  DesiredRateYaw = (-getChannelValue(3) - 1500.0) / 500.0;     // Between -1 and 1


  // Constrain within normalized bounds
  InputThrottle = constrain(getChannelValue(2), throttle_min, throttle_limit);  // Between 1000 and 1800
  DesiredAngleRoll = constrain(DesiredAngleRoll, -1.0, 1.0) * maxRoll;          // Between -maxRoll and +maxRoll
  DesiredAnglePitch = constrain(DesiredAnglePitch, -1.0, 1.0) * maxPitch;       // Between -maxPitch and +maxPitch
  DesiredRateYaw = constrain(DesiredRateYaw, -1.0, 1.0) * maxYaw;               // Between -maxYaw and +maxYaw

  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;

  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];

  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];


  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw);  // front right - counter clockwise
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw);  // rear right - clockwise
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw);  // rear left  - counter clockwise
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw);  //front left - clockwise


  if (MotorInput1 > throttle_max) {
    MotorInput1 = throttle_max - 1;
  }

  if (MotorInput2 > throttle_max) {
    MotorInput1 = throttle_max - 1;
  }

  if (MotorInput3 > throttle_max) {
    MotorInput1 = throttle_max - 1;
  }

  if (MotorInput4 > throttle_max) {
    MotorInput1 = throttle_max - 1;
  }



  if (MotorInput1 < ThrottleIdle) {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle) {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle) {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle) {
    MotorInput4 = ThrottleIdle;
  }

  int ThrottleCutOff = 1000;
  if (getChannelValue(2) < 1050) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }

  mot1.write(map(MotorInput1, 1000, 2000, 0, 180));
  mot2.write(map(MotorInput2, 1000, 2000, 0, 180));
  mot3.write(map(MotorInput3, 1000, 2000, 0, 180));
  mot4.write(map(MotorInput4, 1000, 2000, 0, 180));

  // voltage= (analogRead(36)/4096)*12.46*(35.9/36);
  // if(voltage<11.1)
  // {

  // }

  //Reciever signals
  /*Serial.print(getChannelValue(0));
  Serial.print(" - ");
  Serial.print(getChannelValue(1));
  Serial.print(" - ");
  Serial.print(getChannelValue(2));
  Serial.print(" - ");
  Serial.print(getChannelValue(3));
  Serial.print(" - ");
  Serial.print(getChannelValue(4));
  Serial.print(" - ");
  Serial.println(getChannelValue(5));*/

  // Receiver translated rates
  /*Serial.print("InputThrottle: ");
  Serial.print(InputThrottle);
  Serial.print("Desired Rate Roll: ");
  Serial.print(DesiredAngleRoll);
  Serial.print(" - ");
  Serial.print("Desired Rate Pitch: ");
  Serial.print(DesiredAnglePitch);
  Serial.print(" - ");
  Serial.print("Desired Rate Yaw: ");
  Serial.println(DesiredRateYaw);*/


  //Motor PWMs in us
  Serial.print("MotVals-");
  Serial.print(MotorInput1);
  Serial.print("  ");
  Serial.print(MotorInput2);
  Serial.print("  ");
  Serial.print(MotorInput3);
  Serial.print("  ");
  Serial.println(MotorInput4);


  // //Gyro Rates
  // Serial.print(" Gyro rates:");
  // Serial.print(RateRoll);
  // Serial.print("  ");
  // Serial.print(RatePitch);
  // Serial.print("  ");
  // Serial.print(RateYaw);
  // Serial.print(" -- ");



  //PID outputs
  // Serial.print("PID O/P ");
  // Serial.print(InputPitch);
  //   Serial.print("  ");
  // Serial.print(InputRoll);
  //   Serial.print("  ");
  // Serial.print(InputYaw);
  //   Serial.print(" -- ");

  //Angles from MPU
  // Serial.print("AngleRoll:");
  // Serial.print(AngleRoll);
  // //serial.print("  ");
  //   Serial.print("AnglePitch:");
  // Serial.println(AnglePitch);


  //  Serial.println(" ");



  while (micros() - LoopTimer < (t * 1000000))
    ;
  {
    LoopTimer = micros();
  }
}
