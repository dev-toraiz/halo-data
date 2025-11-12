import processing.serial.*;

Serial myPort; // シリアルポート
String receivedData = ""; // 受信データ
float[] m1_values = new float[100];
float[] m2_values = new float[100];
float[] m3_values = new float[100];
float[] m4_values = new float[100];
int index = 0; // データのインデックス

void setup() {
  size(1200, 800); // ウィンドウサイズ
  println(Serial.list()); // 利用可能なシリアルポートのリストを表示
  myPort = new Serial(this, "COM4", 115200); // シリアルポートの初期化
  myPort.bufferUntil('\n'); // 改行まで読み取る
}

void draw() {
  background(255); // 背景色
  stroke(0);
  fill(0);
  textSize(12);
  text("Command Graphs", width / 2 - 40, 20); // タイトル

  // 軸の描画
  drawGrid();

  // 凡例の描画
  drawLegend();

  // データのグラフ化
  strokeWeight(2);
  stroke(255, 0, 0); // m1_command の線
  plotGraph(m1_values, 950, 2050); // 範囲を 950～2050 に調整
  stroke(0, 255, 0); // m2_command の線
  plotGraph(m2_values, 950, 2050); // 同上
  stroke(0, 0, 255); // m3_command の線
  plotGraph(m3_values, 950, 2050); // 同上
  stroke(255, 165, 0); // m4_command の線
  plotGraph(m4_values, 950, 2050); // 同上
}

void drawGrid() {
  stroke(200);
  fill(0);
  textAlign(RIGHT, CENTER);
  
  for (int i = 0; i <= 10; i++) {
    float y = map(i, 0, 10, height - 50, 50);
    line(50, y, width, y); // 横線
    text(nf(950 + i * 100, 0, 0), 40, y); // Y軸ラベル（950～2050）
  }
  textAlign(CENTER, CENTER);
  for (int i = 1; i <= 10; i++) {
    line(50 + i * (width - 100) / 10, 50, 50 + i * (width - 100) / 10, height - 50); // 縦線
  }
}

void drawLegend() {
  fill(0);
  textAlign(LEFT, CENTER);
  textSize(12);
  stroke(255, 0, 0); // m1_command
  line(50, 30, 70, 30);
  text("m1_command", 80, 30);

  stroke(0, 255, 0); // m2_command
  line(50, 50, 70, 50);
  text("m2_command", 80, 50);

  stroke(0, 0, 255); // m3_command
  line(50, 70, 70, 70);
  text("m3_command", 80, 70);

  stroke(255, 165, 0); // m4_command
  line(50, 90, 70, 90);
  text("m4_command", 80, 90);
}

void plotGraph(float[] data, float minVal, float maxVal) {
  for (int i = 1; i < data.length; i++) {
    line(
      map(i - 1, 0, data.length - 1, 50, width - 50),
      map(data[i - 1], minVal, maxVal, height - 50, 50),
      map(i, 0, data.length - 1, 50, width - 50),
      map(data[i], minVal, maxVal, height - 50, 50)
    );
  }
}

void serialEvent(Serial myPort) {
  receivedData = myPort.readStringUntil('\n'); // データの読み取り
  if (receivedData != null) {
    receivedData = trim(receivedData); // 不要なスペースを削除
    parseData(receivedData); // データの解析
  }
}

void parseData(String data) {
  try {
    // `[ ]`で囲まれた部分を抽出
    int startIdx = data.indexOf("[");
    int endIdx = data.indexOf("]");
    if (startIdx != -1 && endIdx != -1) {
      String extractedData = data.substring(startIdx + 1, endIdx); // 中括弧を除去
      String[] parts = split(extractedData, ","); // カンマで分割
      
      // 各コマンド値を解析
      m1_values[index] = parseCommand(parts[0], "m1_command:");
      m2_values[index] = parseCommand(parts[1], "m2_command:");
      m3_values[index] = parseCommand(parts[2], "m3_command:");
      m4_values[index] = parseCommand(parts[3], "m4_command:");

      // インデックスを更新
      index = (index + 1) % m1_values.length;
    }
  } catch (Exception e) {
    println("Error parsing data: " + e.getMessage());
  }
}

float parseCommand(String command, String prefix) {
  if (command.startsWith(prefix)) {
    return float(trim(command.substring(prefix.length())));
  }
  return Float.NaN;
}
