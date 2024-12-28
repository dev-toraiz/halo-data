#include <Arduino.h>

#define PPM_PIN 4  // PPM信号を受信するピン
#define CHANNEL_COUNT 6  // チャンネル数

volatile uint16_t ppmValues[CHANNEL_COUNT];
volatile uint8_t currentChannel = 0;
volatile uint32_t lastInterruptTime = 0;

void IRAM_ATTR handlePPMInterrupt() {
  uint32_t currentTime = micros();
  uint32_t pulseWidth = currentTime - lastInterruptTime;
  lastInterruptTime = currentTime;

  if (pulseWidth > 3000) {
    // 同期パルス (フレームのリセット)
    currentChannel = 0;
  } else if (currentChannel < CHANNEL_COUNT) {
    // 各チャンネルのパルス幅を記録
    ppmValues[currentChannel] = pulseWidth;
    currentChannel++;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), handlePPMInterrupt, RISING);
}

void loop() {
  static uint16_t lastValues[CHANNEL_COUNT] = {0};

  noInterrupts();
  memcpy((void*)lastValues, (const void*)ppmValues, sizeof(ppmValues));
  interrupts();

  Serial.print("PPM Values: ");
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    Serial.print(lastValues[i]);
    if (i < CHANNEL_COUNT - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();

  delay(100);  // 更新間隔
}
