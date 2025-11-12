#define CHANNELS 6       // 使用するチャネル数
#define SYNC_GAP 3000    // 同期信号判定のしきい値 (マイクロ秒)
#define PPM_PIN 4        // PPM信号の入力ピン

volatile unsigned long lastPulseTime = 0; // 前回のパルス時間
volatile int channelValues[CHANNELS] = {0}; // 各チャネルの値を格納
volatile int currentChannel = 0; // 現在のチャネルインデックス

// 割り込み関数
void IRAM_ATTR ppmInterrupt() {
  unsigned long pulseTime = micros(); // 現在の時間を取得
  unsigned long pulseWidth = pulseTime - lastPulseTime; // パルス幅を計算
  lastPulseTime = pulseTime;

  if (pulseWidth > SYNC_GAP) { // 同期信号を検出
    currentChannel = 0; // チャネルをリセット
  } else {
    if (currentChannel < CHANNELS) { // 有効なチャネル範囲内であれば
      channelValues[currentChannel] = pulseWidth; // チャネル値を格納
      currentChannel++; // 次のチャネルへ
    }
  }
}

void setup() {
  Serial.begin(115200); // シリアル通信を開始
  pinMode(PPM_PIN, INPUT_PULLUP); // ピンを入力モードに設定
  attachInterrupt(PPM_PIN, ppmInterrupt, RISING); // 割り込みを設定
  Serial.println("PPM Receiver Initialized");
}

void loop() {
  // 各チャネルの値をシリアルモニターに表示
  Serial.print("Channel values: ");
  for (int i = 0; i < CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(channelValues[i]);
    Serial.print("us\t");
  }
  Serial.println(); // 改行
  delay(100); // データ出力の間隔を調整
}
