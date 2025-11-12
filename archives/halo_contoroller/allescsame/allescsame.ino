#include <Arduino.h>

const int escPins[] = {32,25,26,27}; // ESCを接続したGPIOピン
const int numMotors = 4; // モーターの数
int throttle[numMotors] = {1023, 1023, 1023, 1023}; // 現在のスロットル値
int targetThrottle[numMotors] = {512, 512, 512, 512}; // 目標のスロットル値

bool motorChanging[numMotors] = {false, false, false, false}; // モーターが変更中かどうかのフラグ

const int minThrottle = 512; // 最小のスロットル値

void setup() {
  // シリアル通信の開始
  Serial.begin(115200);
  Serial.println("\n\n\n\n\n\n\nキャリブレーション開始");

  // ESCのセットアップ
  for (int i = 0; i < numMotors; i++) {
    pinMode(escPins[i], OUTPUT);
    ledcSetup(i, 50, 10);
    ledcAttachPin(escPins[i], i);
  }

  // キャリブレーション開始
  for (int i = 0; i < numMotors; i++) {
    throttle[i] = 512; // 最大スロットル値に設定
    ledcWrite(i, throttle[i]);
  }
  Serial.println("最大値入力中: 512");
  delay(2000); // 2秒間待機

  for (int i = 0; i < numMotors; i++) {
    throttle[i] = minThrottle; // 最小スロットル値に設定
    ledcWrite(i, throttle[i]);
  }
  Serial.println("最小値入力中");

  // ビープ音の確認（ユーザーが手動でビープ音を確認する）
  Serial.println("モーターのビープ音を確認してください。");
}

void loop() {
  // シリアルからの入力処理
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // 入力文字列の前後の空白を取り除く

    if (input.startsWith("all")) {
      // "all" コマンドの場合、すべてのモーターのスロットルを設定
      int newThrottle = input.substring(3).toInt();
      if (newThrottle >= 0 && newThrottle <= 1023) {
        for (int i = 0; i < numMotors; i++) {
          targetThrottle[i] = newThrottle;
          motorChanging[i] = true;
        }
        Serial.print("すべてのモーターの目標スロットル値を設定しました: ");
        Serial.println(newThrottle);
      } else {
        Serial.println("無効なスロットル値が入力されました。");
      }
    } else {
      // 個別のモーターのスロットルを設定
      int motorIndex = input.toInt();
      int newThrottle = Serial.parseInt();
      if (motorIndex >= 0 && motorIndex < numMotors && newThrottle >= 0 && newThrottle <= 1023) {
        targetThrottle[motorIndex] = newThrottle;
        motorChanging[motorIndex] = true;
        Serial.print("モーター ");
        Serial.print(motorIndex);
        Serial.print(" の目標スロットル値を設定しました: ");
        Serial.println(targetThrottle[motorIndex]);
      } else {
        Serial.println("無効なモーター番号かスロットル値が入力されました。");
      }
    }
  }

  // スロットル値を更新
  for (int i = 0; i < numMotors; i++) {
    if (motorChanging[i] && throttle[i] != targetThrottle[i]) {
      if (throttle[i] < targetThrottle[i]) throttle[i] += 1;
      else if (throttle[i] > targetThrottle[i]) throttle[i] -= 1;
      ledcWrite(i, throttle[i]);
    } else {
      motorChanging[i] = false; // モーターの変更が完了したらフラグをリセット
    }
  }

  // 各モーターのスロットル値をログで表示
  Serial.print("現在のスロットル値: ");
  for (int i = 0; i < numMotors; i++) {
    Serial.print("モーター ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(throttle[i]);
    if (i < numMotors - 1) {
      Serial.print(" | ");
    }
  }
  Serial.println();

  delay(0); // 1秒ごとにログを表示
}
