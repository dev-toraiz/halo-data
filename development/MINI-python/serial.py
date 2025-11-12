import requests
import time

# ESP32のIPアドレスを設定（ESP32が接続されているWi-Fiネットワーク内のIPアドレス）
esp32_ip = 'http://192.168.0.27'

while True:
    try:
        # ESP32からシリアルデータを取得
        response = requests.get(esp32_ip)
        
        if response.status_code == 200:
            print(f"Received data: {response.text}")
        else:
            print(f"Error: {response.status_code}")
        
        time.sleep(2)  # 2秒間隔でデータを取得

    except Exception as e:
        print(f"Error: {e}")
        time.sleep(5)
