#include <Arduino.h>

const int escPin = 23; // ESCを接続したGPIOピン
int throttle = 2000;      // 現在のスロットル値
int targetThrottle = 1000; // 目標のスロットル値

void setup() {
  // シリアル通信の開始
  Serial.begin(9600);

  // ESCのセットアップ
  pinMode(escPin, OUTPUT);
  ledcSetup(0, 50, 10);
  ledcAttachPin(escPin, 0);
}

void loop() {
  if (Serial.available() > 0) {
    // シリアルからの入力を読み取る
    int newThrottle = Serial.parseInt();
    if (newThrottle > 0) {
      targetThrottle = newThrottle;
      Serial.print("設定されたスロットル値: ");
      Serial.println(targetThrottle);

      // 目標スロットル値まで滑らかに変更
      while (throttle != targetThrottle) {
        if (throttle < targetThrottle) throttle+=1; 
        else if (throttle > targetThrottle) throttle-=10;
        ledcWrite(0, throttle);
        // スロットル値の表示
        Serial.print("変更中: ");
        Serial.println(throttle);
      }
    } else {
      Serial.println("無効なスロットル値が入力されました。");
    }
  }
  // スロットル値の表示
  Serial.print("現在のスロットル値: ");
  Serial.println(throttle);
  delay(0); // 無限ループを避けるための小さな遅延
}

