#define MOTOR1 13
#define MOTOR2 14
#define MOTOR3 15
#define MOTOR4 16

int maxSpeed = 255;  // 最大速度 (PWM値)
int minSpeed = 0;    // 最小速度 (PWM値)
int step = 5;        // 加速・減速のステップ
int delayTime = 50;  // ステップ間の遅延 (ミリ秒)

void setup() {
  Serial.begin(115200);

  // PWMの設定
  ledcAttach(MOTOR1, 5000, 8); // MOTOR1: 周波数5000Hz, 解像度8ビット
  ledcAttach(MOTOR2, 5000, 8); // MOTOR2
  ledcAttach(MOTOR3, 5000, 8); // MOTOR3
  ledcAttach(MOTOR4, 5000, 8); // MOTOR4

  // 初期化
  ledcWrite(MOTOR1, 0);
  ledcWrite(MOTOR2, 0);
  ledcWrite(MOTOR3, 0);
  ledcWrite(MOTOR4, 0);
}

void loop() {
  // すべてのモーターを加速
  Serial.println("すべてのモーターを加速");
  accelerateAll();

  // 最大速度で動作
  delay(2000);

  // すべてのモーターを減速
  Serial.println("すべてのモーターを減速");
  decelerateAll();

  // 停止
  delay(2000);
}

void accelerateAll() {
  for (int speed = minSpeed; speed <= maxSpeed; speed += step) {
    setMotorSpeed(speed);
    delay(delayTime);
  }
}

void decelerateAll() {
  for (int speed = maxSpeed; speed >= minSpeed; speed -= step) {
    setMotorSpeed(speed);
    delay(delayTime);
  }
}

void setMotorSpeed(int speed) {
  ledcWrite(MOTOR1, speed); // MOTOR1の速度を設定
  ledcWrite(MOTOR2, speed); // MOTOR2
  ledcWrite(MOTOR3, speed); // MOTOR3
  ledcWrite(MOTOR4, speed); // MOTOR4
  Serial.print("現在の速度: ");
  Serial.println(speed);
}
