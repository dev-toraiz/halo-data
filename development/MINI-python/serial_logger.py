"""
Halo Drone Serial Logger
=========================
ESP32からの診断CSVデータをファイルに保存する。

使い方:
  python serial_logger.py                    # デフォルト: /dev/cu.usbserial*, 実験名を聞く
  python serial_logger.py -p /dev/ttyUSB0    # ポート指定
  python serial_logger.py -n A_vibration_off # ファイル名指定

ESP32側で "diag" コマンドを送ると診断モードが有効になる。
このスクリプトは起動時に自動で "diag" を送信する。
Ctrl+C で停止、ファイルを保存。
"""

import serial
import serial.tools.list_ports
import sys
import os
from datetime import datetime

CSV_DIR = os.path.expanduser("~/halo-experiments/csv")
CSV_HEADER = "timestamp_ms,AccX,AccY,GyroX,GyroY,roll_IMU,pitch_IMU,roll_PID,pitch_PID,M1,M2,M3,M4"


def find_esp32_port():
    """ESP32のシリアルポートを自動検出"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description + port.device).lower()
        if any(k in desc for k in ["cp210", "ch340", "usb", "serial", "uart"]):
            return port.device
    if ports:
        return ports[0].device
    return None


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Halo Drone Serial Logger")
    parser.add_argument("-p", "--port", help="Serial port")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-n", "--name", help="Experiment name for filename")
    args = parser.parse_args()

    port = args.port or find_esp32_port()
    if not port:
        print("Error: No serial port found. Use -p to specify.")
        sys.exit(1)

    name = args.name
    if not name:
        name = input("Experiment name (e.g. A_vibration_off): ").strip()
        if not name:
            name = "unnamed"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{name}_{timestamp}.csv"
    filepath = os.path.join(CSV_DIR, filename)

    os.makedirs(CSV_DIR, exist_ok=True)

    print(f"Port: {port} @ {args.baud} baud")
    print(f"Output: {filepath}")
    print("Connecting...")

    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Enable diagnostic mode
    ser.write(b"diag\n")
    print("Sent 'diag' command to enable diagnostic mode")
    print("Recording... Press Ctrl+C to stop.\n")

    line_count = 0
    with open(filepath, "w") as f:
        f.write(CSV_HEADER + "\n")

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()

                # Skip comments and non-CSV lines
                if not line or line.startswith("#") or "," not in line:
                    print(f"  [{line}]")
                    continue

                # Validate: should have 13 comma-separated values
                parts = line.split(",")
                if len(parts) != 13:
                    continue

                f.write(line + "\n")
                line_count += 1

                if line_count % 100 == 0:
                    print(f"  {line_count} lines recorded...")
                    f.flush()

        except KeyboardInterrupt:
            pass

    # Disable diagnostic mode
    ser.write(b"diag\n")
    ser.close()

    print(f"\nDone. {line_count} lines saved to {filepath}")


if __name__ == "__main__":
    main()
