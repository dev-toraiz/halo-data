#include <Wire.h>

int MPU6050_ADDR = 0x68;    //or 0x69
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_PWR_MGMT_1 0x6b
int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z;
float absAccelerometer;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float interval, preInterval;
double offsetX = 0, offsetY = 0, offsetZ = 0;
float angleX, angleY, angleZ;
float dpsX, dpsY, dpsZ;
double init_angleX = 0, init_angleY = 0, init_angleZ = 0;
volatile float rel_angleX, rel_angleY, rel_angleZ;

void AcceleroMeterWireRead() {
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

void AcceleroMeterAddressSetup() {
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      if (address < 16) MPU6050_ADDR = address;
      nDevices++;
    }
  }
}

void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void calcAcceleration() {
  acc_x = ((float)raw_acc_x) / 16384.0;
  acc_y = ((float)raw_acc_y) / 16384.0;
  acc_z = ((float)raw_acc_z) / 16384.0;
  absAccelerometer = pow(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2), 0.5);
}

void calcRotation() {
  acc_x = ((float)raw_acc_x) / 16384.0;
  acc_y = ((float)raw_acc_y) / 16384.0;
  acc_z = ((float)raw_acc_z) / 16384.0;
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;
  dpsX = ((float)raw_gyro_x) / 65.5;
  dpsY = ((float)raw_gyro_y) / 65.5;
  dpsZ = ((float)raw_gyro_z) / 65.5;
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
  rel_angleX = init_angleX - angleX;
  rel_angleY =-( init_angleY - angleY);
  rel_angleZ = init_angleZ - angleZ;
}

void SubProcess(void* pvParameters) {
  AcceleroMeterAddressSetup();
  while (1) {
    AcceleroMeterWireRead();
    calcAcceleration();
    calcRotation();
  }
}

void AcceleroMeterAngleSetup() {
  AcceleroMeterAddressSetup();
  AcceleroMeterWireRead();
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  Serial.print("Calculate Calibration");
  for (int i = 0; i < 3000; i++) {
    AcceleroMeterWireRead();
    dpsX = ((float)raw_gyro_x) / 65.5;
    dpsY = ((float)raw_gyro_y) / 65.5;
    dpsZ = ((float)raw_gyro_z) / 65.5;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;
    if (i % 1000 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();
  offsetX /= 3000;
  offsetY /= 3000;
  offsetZ /= 3000;
  Serial.print("Calculate Rotation");
  for (int i = 0; i < 1000; i++) {
    calcRotation();
    if (i % 1000 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();
  init_angleX = angleX;
  init_angleY = angleY;
  init_angleZ = angleZ;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  AcceleroMeterAngleSetup();
  xTaskCreatePinnedToCore(SubProcess, "SubProcess", 4096, NULL, 1, NULL, 0);
}

void loop() {
  Serial.print("abs:");
  Serial.print(absAccelerometer);
  Serial.print(" accx:");
  Serial.print(acc_x);
  Serial.print(" accy:");
  Serial.print(acc_y);
  Serial.print(" accz:");
  Serial.print(acc_z);
  Serial.print(" rel_x:");
  Serial.print(rel_angleX);
  Serial.print(" rel_y:");
  Serial.print(rel_angleY);
  Serial.print(" rel_z:");
  Serial.print(rel_angleZ);
  Serial.println();
}