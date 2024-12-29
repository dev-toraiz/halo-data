#include <ESP32Servo.h>

Servo motor; // サーボオブジェクトを作成

const int motorPin = 25; // モーター制御用のピン
int pwmValue = 0; // 初期PWM値

void setup() {
  Serial.begin(115200); // シリアル通信を初期化
  motor.attach(motorPin); // サーボピンを設定
  motor.writeMicroseconds(1000); // モーター初期化用の信号を送信（1000usは一般的な最小スロットル値）
  delay(2000); // モーターを初期化するための待機

  Serial.println("ブラシレスモーター制御プログラム");
  Serial.println("PWM値を入力してください (1000 - 2000):");
}

void loop() {
  if (Serial.available() > 0) { // シリアル入力がある場合
    String input = Serial.readStringUntil('\n'); // 入力を読み取る
    pwmValue = input.toInt(); // 入力を整数に変換

    if (pwmValue >= 1000 && pwmValue <= 2000) { // 有効な範囲か確認
      motor.writeMicroseconds(pwmValue); // モーターにPWM値を送信
      Serial.print("PWM値を設定: ");
      Serial.println(pwmValue);
    } else {
      Serial.println("無効な値です。1000から2000の範囲で入力してください。");
    }
  }
}
