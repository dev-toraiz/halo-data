"""
Halo Drone Experiment Analyzer
===============================
各実験のCSVデータを分析する。

使い方:
  python analyze_experiment.py vibration <csv_file>     # 実験A: 振動ノイズ分析
  python analyze_experiment.py phase <csv_file>         # 実験B: 位相遅れ分析
  python analyze_experiment.py pid <csv_file>           # 実験C/D: PID応答分析
  python analyze_experiment.py compare <file1> <file2>  # 2つのCSVを比較
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

PLOT_DIR = os.path.expanduser("~/halo-experiments/plots")
os.makedirs(PLOT_DIR, exist_ok=True)


def load_csv(filepath):
    """CSVファイルを読み込み、コメント行をスキップ"""
    lines = []
    header = None
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if header is None:
                header = line.split(",")
                continue
            parts = line.split(",")
            if len(parts) == len(header):
                try:
                    lines.append([float(x) for x in parts])
                except ValueError:
                    continue
    data = np.array(lines)
    return header, data


def analyze_vibration(filepath):
    """実験A: 振動ノイズ分析"""
    header, data = load_csv(filepath)
    name = os.path.splitext(os.path.basename(filepath))[0]

    # columns: timestamp_ms, AccX, AccY, AccZ, GyroX, GyroY, GyroZ
    t = (data[:, 0] - data[0, 0]) / 1000.0  # seconds
    dt = np.median(np.diff(t))
    fs = 1.0 / dt  # sampling frequency

    channels = {
        "AccX": data[:, 1], "AccY": data[:, 2], "AccZ": data[:, 3],
        "GyroX": data[:, 4], "GyroY": data[:, 5], "GyroZ": data[:, 6],
    }

    print(f"\n=== Vibration Analysis: {name} ===")
    print(f"Duration: {t[-1]:.1f}s, Samples: {len(t)}, Fs: {fs:.0f}Hz\n")

    print(f"{'Channel':>8}  {'Mean':>8}  {'Std':>8}  {'Peak':>8}  {'Peak-Peak':>10}")
    print("-" * 50)
    for ch_name, ch_data in channels.items():
        mean = np.mean(ch_data)
        std = np.std(ch_data)
        peak = np.max(np.abs(ch_data - mean))
        pp = np.max(ch_data) - np.min(ch_data)
        print(f"{ch_name:>8}  {mean:>8.4f}  {std:>8.4f}  {peak:>8.4f}  {pp:>10.4f}")

    # Time series plot
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Vibration Test: {name}", fontsize=14)

    for ch_name in ["AccX", "AccY", "AccZ"]:
        axes[0].plot(t, channels[ch_name], label=ch_name, alpha=0.7, linewidth=0.5)
    axes[0].set_ylabel("Acceleration [g]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    for ch_name in ["GyroX", "GyroY", "GyroZ"]:
        axes[1].plot(t, channels[ch_name], label=ch_name, alpha=0.7, linewidth=0.5)
    axes[1].set_ylabel("Angular Rate [deg/s]")
    axes[1].set_xlabel("Time [s]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(PLOT_DIR, f"{name}_timeseries.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")

    # FFT plot
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle(f"FFT: {name}", fontsize=14)

    for ch_name in ["AccX", "AccY"]:
        signal = channels[ch_name] - np.mean(channels[ch_name])
        freqs = np.fft.rfftfreq(len(signal), d=dt)
        fft_mag = np.abs(np.fft.rfft(signal)) / len(signal)
        axes[0].plot(freqs, fft_mag, label=ch_name, alpha=0.7)
    axes[0].set_ylabel("Magnitude [g]")
    axes[0].set_xlim(0, fs / 2)
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    for ch_name in ["GyroX", "GyroY"]:
        signal = channels[ch_name] - np.mean(channels[ch_name])
        freqs = np.fft.rfftfreq(len(signal), d=dt)
        fft_mag = np.abs(np.fft.rfft(signal)) / len(signal)
        axes[1].plot(freqs, fft_mag, label=ch_name, alpha=0.7)
    axes[1].set_ylabel("Magnitude [deg/s]")
    axes[1].set_xlabel("Frequency [Hz]")
    axes[1].set_xlim(0, fs / 2)
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(PLOT_DIR, f"{name}_fft.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Saved: {plot_path}")
    plt.show()


def analyze_phase(filepath):
    """実験B: 位相遅れ分析"""
    header, data = load_csv(filepath)
    name = os.path.splitext(os.path.basename(filepath))[0]

    # columns: timestamp_ms, GyroX, GyroY, roll_IMU, pitch_IMU, roll_acc, pitch_acc
    t = (data[:, 0] - data[0, 0]) / 1000.0
    dt = np.median(np.diff(t))
    gyroX = data[:, 1]
    roll_imu = data[:, 3]
    roll_acc = data[:, 5]

    print(f"\n=== Phase Delay Analysis: {name} ===")
    print(f"Duration: {t[-1]:.1f}s, Samples: {len(t)}, dt: {dt*1000:.1f}ms\n")

    # Method 1: Cross-correlation
    gyro_norm = gyroX - np.mean(gyroX)
    roll_norm = roll_imu - np.mean(roll_imu)

    correlation = np.correlate(gyro_norm, roll_norm, mode='full')
    lags = np.arange(-len(gyro_norm) + 1, len(gyro_norm)) * dt * 1000  # ms
    peak_lag_idx = np.argmax(correlation)
    phase_delay_ms = lags[peak_lag_idx]

    print(f"Cross-correlation peak lag: {phase_delay_ms:.1f} ms")

    # Method 2: Peak detection
    # Find peaks in GyroX (angular velocity peaks during tilts)
    gyro_abs = np.abs(gyroX)
    threshold = np.max(gyro_abs) * 0.5
    peaks_gyro = []
    peaks_roll = []

    in_peak = False
    for i in range(1, len(gyro_abs) - 1):
        if gyro_abs[i] > threshold and not in_peak:
            in_peak = True
            peak_start = i
        elif gyro_abs[i] < threshold * 0.3 and in_peak:
            in_peak = False
            peak_idx = peak_start + np.argmax(gyro_abs[peak_start:i])
            peaks_gyro.append(peak_idx)

            # Find corresponding roll_IMU peak (search forward)
            search_end = min(peak_idx + int(0.5 / dt), len(roll_imu))
            roll_window = np.abs(roll_imu[peak_idx:search_end] - roll_imu[peak_idx - 5])
            roll_peak_idx = peak_idx + np.argmax(roll_window)
            peaks_roll.append(roll_peak_idx)

    if peaks_gyro:
        delays = [(t[r] - t[g]) * 1000 for g, r in zip(peaks_gyro, peaks_roll)]
        print(f"\nPeak-to-peak delays: {[f'{d:.1f}ms' for d in delays]}")
        print(f"Mean peak delay: {np.mean(delays):.1f} ms")
        print(f"{'-> Madgwick delay is significant (>10ms)!' if np.mean(delays) > 10 else '-> Madgwick delay is acceptable (<10ms)'}")

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"Phase Delay: {name}", fontsize=14)

    axes[0].plot(t, gyroX, 'b-', label="GyroX (raw)", linewidth=0.8)
    for p in peaks_gyro:
        axes[0].axvline(t[p], color='b', alpha=0.3, linestyle='--')
    axes[0].set_ylabel("Angular Rate [deg/s]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, roll_imu, 'r-', label="roll_IMU (Madgwick)", linewidth=0.8)
    axes[1].plot(t, roll_acc, 'g-', label="roll_acc (accel only)", linewidth=0.5, alpha=0.5)
    for p in peaks_roll:
        axes[1].axvline(t[p], color='r', alpha=0.3, linestyle='--')
    axes[1].set_ylabel("Roll Angle [deg]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # Overlay normalized signals
    if np.std(gyro_norm) > 0 and np.std(roll_norm) > 0:
        axes[2].plot(t, gyro_norm / np.max(np.abs(gyro_norm)), 'b-',
                     label="GyroX (normalized)", linewidth=0.8)
        axes[2].plot(t, roll_norm / np.max(np.abs(roll_norm)), 'r-',
                     label="roll_IMU (normalized)", linewidth=0.8)
    axes[2].set_ylabel("Normalized")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(PLOT_DIR, f"{name}_phase.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    plt.show()


def analyze_pid(filepath):
    """実験C/D: PID応答分析"""
    header, data = load_csv(filepath)
    name = os.path.splitext(os.path.basename(filepath))[0]

    # columns: timestamp_ms, GyroX, roll_IMU, roll_PID, M1, M2, M3, M4
    t = (data[:, 0] - data[0, 0]) / 1000.0
    gyroX = data[:, 1]
    roll_imu = data[:, 2]
    roll_pid = data[:, 3]
    m1 = data[:, 4]
    m2 = data[:, 5]

    print(f"\n=== PID Response Analysis: {name} ===")
    print(f"Duration: {t[-1]:.1f}s, Samples: {len(t)}\n")

    # Detect oscillation
    zero_crossings = np.where(np.diff(np.sign(roll_imu - np.mean(roll_imu))))[0]
    if len(zero_crossings) > 2:
        periods = np.diff(t[zero_crossings])
        osc_freq = 1.0 / (2 * np.median(periods))  # half-period to full period
        print(f"Oscillation frequency: {osc_freq:.1f} Hz")
        print(f"Oscillation period: {1/osc_freq*1000:.0f} ms")
    else:
        print("No significant oscillation detected")

    # Overshoot
    roll_range = np.max(np.abs(roll_imu))
    print(f"Max roll deviation: {roll_range:.1f} deg")
    print(f"Final roll (last 0.5s): {np.mean(roll_imu[-50:]):.2f} deg")

    # Settling check
    final_std = np.std(roll_imu[-50:])
    print(f"Final stability (std of last 0.5s): {final_std:.2f} deg")
    print(f"{'-> Settled' if final_std < 2.0 else '-> NOT settled (still oscillating)'}")

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"PID Response: {name}", fontsize=14)

    axes[0].plot(t, roll_imu, 'b-', linewidth=0.8, label="roll_IMU")
    axes[0].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[0].fill_between(t, -2, 2, alpha=0.1, color='green', label="±2° target")
    axes[0].set_ylabel("Roll [deg]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, roll_pid, 'r-', linewidth=0.8, label="roll_PID")
    axes[1].set_ylabel("PID Output")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, m1, label="M1 (FL)", linewidth=0.8)
    axes[2].plot(t, m2, label="M2 (FR)", linewidth=0.8)
    axes[2].set_ylabel("Motor PWM [us]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(PLOT_DIR, f"{name}_pid.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    plt.show()


def compare_files(file1, file2):
    """2つの振動テストCSVを比較"""
    header1, data1 = load_csv(file1)
    header2, data2 = load_csv(file2)
    name1 = os.path.splitext(os.path.basename(file1))[0]
    name2 = os.path.splitext(os.path.basename(file2))[0]

    print(f"\n=== Comparison: {name1} vs {name2} ===\n")

    # Assume vibration test format
    channels = ["AccX", "AccY", "GyroX", "GyroY"]
    indices = [1, 2, 4, 5]  # column indices

    print(f"{'Channel':>8}  {'Std(' + name1 + ')':>16}  {'Std(' + name2 + ')':>16}  {'Ratio':>8}")
    print("-" * 60)
    for ch, idx in zip(channels, indices):
        std1 = np.std(data1[:, idx])
        std2 = np.std(data2[:, idx])
        ratio = std2 / std1 if std1 > 0 else float('inf')
        print(f"{ch:>8}  {std1:>16.4f}  {std2:>16.4f}  {ratio:>8.1f}x")

    print()
    avg_ratio = np.mean([np.std(data2[:, i]) / max(np.std(data1[:, i]), 1e-6)
                         for i in indices])
    if avg_ratio > 5:
        print(f"-> Noise ratio {avg_ratio:.1f}x: VIBRATION IS SIGNIFICANT. Mount IMU with damping.")
    elif avg_ratio > 2:
        print(f"-> Noise ratio {avg_ratio:.1f}x: Moderate vibration. Damping recommended.")
    else:
        print(f"-> Noise ratio {avg_ratio:.1f}x: Vibration is acceptable.")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    mode = sys.argv[1]
    if mode == "vibration":
        analyze_vibration(sys.argv[2])
    elif mode == "phase":
        analyze_phase(sys.argv[2])
    elif mode == "pid":
        analyze_pid(sys.argv[2])
    elif mode == "compare" and len(sys.argv) >= 4:
        compare_files(sys.argv[2], sys.argv[3])
    else:
        print(__doc__)
        sys.exit(1)
