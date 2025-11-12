import pygame
import socket
import time

# ゲームパッドの初期化
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("ゲームパッドが接続されていません。接続して再実行してください。")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"ゲームパッドが接続されました: {joystick.get_name()}")

# ESP32のIPとポートを設定
esp_ip = "192.168.0.27"  # ESP32のIPアドレスをここに設定
esp_port = 12345          # ESP32で設定したポート番号

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    while True:
        pygame.event.pump()  # イベントキューを更新

        # ジョイスティック1の値を取得
        x1_axis = joystick.get_axis(0)  # ジョイスティック1のX軸（-1.0～1.0）
        y1_axis = joystick.get_axis(1)  # ジョイスティック1のY軸（-1.0～1.0）

        # ジョイスティック2の値を取得
        x2_axis = joystick.get_axis(2)  # ジョイスティック2のX軸（-1.0～1.0）
        y2_axis = joystick.get_axis(3)  # ジョイスティック2のY軸（-1.0～1.0）

        # ボタンAの値（例: 動作確認用）
        button_a = joystick.get_button(0)

        # データを整形して送信
        data = f"x1:{x1_axis:.2f},y1:{y1_axis:.2f},x2:{x2_axis:.2f},y2:{y2_axis:.2f},button:{button_a}"
        sock.sendto(data.encode(), (esp_ip, esp_port))
        print(f"送信: {data}")

        # 短い待ち時間を入れる
        time.sleep(0.1)
except KeyboardInterrupt:
    print("プログラムを終了します。")
finally:
    pygame.quit()
    sock.close()
