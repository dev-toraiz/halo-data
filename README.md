# Halo Drone - ESP32 Flight Controller Project

ESP32ベースのクアッドコプター用フライトコントローラープロジェクト

## プロジェクト構成

```
halo-data/
├── halo-flight-controller/    # メインのフライトコントローラー
├── archives/                   # 過去のバージョン・参考実装
├── test-tools/                 # ハードウェアテストツール
├── development/                # 開発環境・実験コード
└── docs/                       # ドキュメント・回路図
```

## メインフライトコントローラー

### [halo-flight-controller/](halo-flight-controller/)

現在の最新実装。以下の2つのバージョンがあります：

#### h.ino
- **IMU**: MPU6050（ライブラリ使用）
- **姿勢推定**: Madgwickフィルター（6DOF）
- **受信機**: PPM（6チャネル）
- **モーターピン**: 16, 17, 18, 19
- **PWM方式**: ledcAttach（ESP32 Arduino Core 3.x対応）
- **PID制御**: 角度モード（2段階PID）
  - Angle PID: Roll/Pitch (P=6.06, I=0, D=0.83)
  - Yaw: レートモード (P=0, I=0, D=0)
- **特徴**: モーター個別オフセット、フレームレート計測

#### halo_contoroller_241201.ino
- ESCキャリブレーション機能付き
- その他の仕様は h.ino と同じ

### 主な機能
- MPU6050によるIMU読み取り
- Madgwickフィルタによる姿勢推定
- PPM受信機からの制御信号受信
- PID制御によるクアッドコプター安定化
- 4モーター独立制御

## アーカイブ

### [archives/](archives/)

#### ESP32-Flight-controller-main/
オリジナルの実装。Kalmanフィルター使用、8チャネルPPM対応。

主なファイル:
- `Anglemode_flightcontroller_ver3/` - メインのフライトコントローラー
- `motor_calibration_esp32/` - モーターキャリブレーション
- `reciever_pwm_esp32/` - 受信機テスト
- `measure_angles_from_mpu/` - IMU角度測定
- `Voltage_measurement_esp32/` - バッテリー電圧測定
- `anglemode_flightcontroller_ver3_PID_values_tuning_webserver/` - Webサーバー経由PIDチューニング

#### halo_contoroller/
旧バージョンの実装

#### esp-drone-master/
Espressif公式のドローン参考実装

## テストツール

### [test-tools/](test-tools/)

ハードウェアの動作確認用ツール:
- **ESC_Checker/** - ESC（モータードライバー）チェック
- **L293D_Checker/** - L293Dモータードライバーチェック
- **MPU6050_Checker/** - MPU6050 IMUセンサーチェック
- **PPM_Checker/** - PPM受信機チェック
- **ppm_receiver/** - PPM受信機テスト

## 開発環境

### [development/](development/)

開発関連ファイル:
- **PlatFormIO/** - PlatformIO プロジェクト
- **ArduinoIDE/** - Arduino IDE プロジェクト
- **Python/** - Pythonスクリプト
- **MINI-python/** - 小規模Pythonツール
- **filters/** - フィルター実験コード
- **controller/** - コントローラー関連
- **h/** - h.inoのオリジナルディレクトリ
- **halo_contoroller_241201/** - 最新版のオリジナルディレクトリ

## ドキュメント

### [docs/](docs/)

- **Circuit/** - 回路図・基板設計
  - `micon_board/` - マイコンボード設計
- **その他関連データ/** - その他のドキュメント

## ハードウェア仕様

### 主要コンポーネント
- **マイコン**: ESP32
- **IMU**: MPU6050（加速度計 + ジャイロ）
- **モーター**: 4基（ブラシレスモーター）
- **ESC**: 各モーター用に1基ずつ
- **受信機**: PPM方式（6チャネル）

### ピン配置（halo-flight-controller）
- **PPM入力**: GPIO 4
- **モーター1**: GPIO 16 (Front Left)
- **モーター2**: GPIO 17 (Front Right)
- **モーター3**: GPIO 18 (Back Right)
- **モーター4**: GPIO 19 (Back Left)
- **I2C (MPU6050)**: SDA/SCL（デフォルト）

### モーター配置（X型クアッド）
```
    Front
  M1     M2
    \ X /
    / X \
  M4     M3
    Rear
```

## セットアップ

### 必要なライブラリ
- Wire (標準)
- MadgwickAHRS
- MPU6050
- Arduino (ESP32 Core)

### 初回セットアップ
1. ESP32ボードを水平な場所に設置
2. コードをアップロード
3. キャリブレーション待機（約10秒）
4. LED点滅後、飛行準備完了

### ESCキャリブレーション（必要に応じて）
`halo_contoroller_241201.ino`の`calibrateESCs()`関数を有効化してアップロード

## PIDチューニング

### 現在の設定値
- **Angle PID (Roll/Pitch)**
  - P: 6.06
  - I: 0.00
  - D: 0.83
- **Angle Limits**
  - Max Roll: 18度
  - Max Pitch: 18度
  - Max Yaw Rate: 10度/秒

### チューニング方法
1. コード内のPIDパラメータを調整
2. 再アップロード
3. または、WebサーバーPIDチューニング版を使用（archives/ESP32-Flight-controller-main/参照）

## 安全上の注意

1. **初回テストは必ず屋外で実施**
2. **プロペラの回転方向を回路図通りに確認**
3. **モーター回転方向をメインコードアップロード前に確認**
4. **バッテリー電圧を定期的にチェック**
5. **テスト時はプロペラガードの使用を推奨**

## トラブルシューティング

### MPU6050接続失敗
- I2C配線を確認
- MPU6050の電源供給を確認
- I2Cプルアップ抵抗（4.7kΩ）の確認

### モーターが回転しない
- ESCキャリブレーションを実行
- モーターピン配線を確認
- スロットル最小値以上であることを確認

### 機体が不安定
- PIDパラメータを調整
- センサーキャリブレーションをやり直す
- モーターバランスを確認

## ライセンス

各サブプロジェクトのライセンスに従います。

## 参考リンク

- ESP32 Flight Controller YouTube: https://youtu.be/BcPTe-hTJGc
- Tutorial Video: https://www.youtube.com/watch?v=TmAVuMhu9no
