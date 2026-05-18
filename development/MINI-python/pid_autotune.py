"""
Halo Drone PID Auto-Tuner
==========================
Twiddle (coordinate ascent) + シミュレーションで最適なPID値を自動探索する。

使い方:
  python pid_autotune.py

ドローンの物理モデルをシミュレーションし、以下を最小化するPID値を探す:
  - 定常偏差（目標角度とのズレ）
  - オーバーシュート（行き過ぎ）
  - 収束時間（安定するまでの時間）
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass


# ============================================
# ドローン物理モデル
# ============================================

@dataclass
class DroneParams:
    """実機に近いドローンの物理パラメータ"""
    mass: float = 0.5           # kg（機体重量）
    arm_length: float = 0.15    # m（アーム長さ）
    inertia: float = 0.005      # kg*m^2（慣性モーメント、推定値）
    motor_tau: float = 0.02     # s（モーター応答時定数）
    drag_coeff: float = 0.01    # 空気抵抗係数
    dt: float = 0.004           # s（制御ループ周期 = 250Hz）
    sim_time: float = 3.0       # s（シミュレーション時間）
    noise_std: float = 0.5      # deg（センサーノイズの標準偏差）


def simulate_axis(kp, ki, kd, drone: DroneParams, disturbance_deg=15.0):
    """
    1軸（Roll or Pitch）のPID制御をシミュレーション。

    Args:
        kp, ki, kd: PIDゲイン（実機のスケーリング 0.0001 を含む）
        drone: ドローンの物理パラメータ
        disturbance_deg: 初期外乱角度 [deg]

    Returns:
        time_array, angle_array, cost（評価値、小さいほど良い）
    """
    steps = int(drone.sim_time / drone.dt)
    dt = drone.dt

    # 状態変数
    angle = disturbance_deg  # 現在の角度 [deg]
    angular_vel = 0.0        # 角速度 [deg/s]
    motor_output = 0.0       # モーター出力（遅れあり）

    # PID内部状態
    integral = 0.0
    prev_error = 0.0

    # 記録用
    angles = np.zeros(steps)
    times = np.zeros(steps)

    # 評価指標
    total_error_sq = 0.0     # 二乗誤差の積分
    max_overshoot = 0.0
    settled = False
    settle_time = drone.sim_time

    for i in range(steps):
        t = i * dt
        times[i] = t

        # センサーノイズ
        measured_angle = angle + np.random.normal(0, drone.noise_std)

        # PID計算（実機と同じスケーリング）
        error = 0.0 - measured_angle
        integral += error * dt
        integral = np.clip(integral, -20.0, 20.0)  # i_limit
        derivative = angular_vel  # D-on-measurement（実機と同じ）

        pid_output = 0.0001 * (kp * error + ki * integral - kd * derivative)

        # モーター応答遅れ（1次遅れ系）
        alpha = dt / (drone.motor_tau + dt)
        motor_output = motor_output + alpha * (pid_output - motor_output)

        # 物理シミュレーション
        # トルク = モーター出力 * アーム長 / 慣性モーメント
        torque_scale = 50.0  # モーターの力をdeg/s^2に変換するスケール
        angular_accel = (motor_output * torque_scale
                         - drone.drag_coeff * angular_vel)

        angular_vel += angular_accel * dt
        angle += angular_vel * dt

        angles[i] = angle

        # 評価指標の更新
        total_error_sq += angle ** 2 * dt
        if abs(angle) > abs(max_overshoot) and np.sign(angle) != np.sign(disturbance_deg):
            max_overshoot = abs(angle)

        # 収束判定（±1度以内に入ったら）
        if not settled and abs(angle) < 1.0 and abs(angular_vel) < 5.0:
            settled = True
            settle_time = t

    # コスト関数: 重み付きの総合評価
    cost = (
        1.0 * total_error_sq          # 偏差の積分（追従性）
        + 5.0 * max_overshoot         # オーバーシュートへのペナルティ
        + 2.0 * settle_time           # 収束時間
        + 10.0 * abs(angles[-1])      # 最終偏差
    )

    # 発散チェック
    if np.any(np.abs(angles) > 90):
        cost = 1e6  # 発散したら大きなペナルティ

    return times, angles, cost


# ============================================
# Twiddleアルゴリズム（自動最適化）
# ============================================

def twiddle(drone: DroneParams, initial_params=None, tol=0.001, max_iter=500):
    """
    Twiddle (coordinate ascent) でPIDパラメータを最適化する。

    Args:
        drone: ドローン物理パラメータ
        initial_params: [Kp, Ki, Kd] の初期値
        tol: 収束閾値
        max_iter: 最大反復回数

    Returns:
        best_params, cost_history
    """
    if initial_params is None:
        params = [6.0, 0.0, 0.8]  # 現在のhaloの値
    else:
        params = list(initial_params)

    # 探索幅の初期値
    dp = [1.0, 0.1, 0.2]

    def evaluate(p):
        """複数回シミュレーションの平均コスト（ノイズの影響を減らす）"""
        costs = []
        for disturbance in [15.0, -10.0, 5.0, -20.0]:
            _, _, c = simulate_axis(
                max(p[0], 0.01), max(p[1], 0.0), max(p[2], 0.0),
                drone, disturbance
            )
            costs.append(c)
        return np.mean(costs)

    best_cost = evaluate(params)
    cost_history = [best_cost]
    best_params = list(params)

    print(f"初期値: Kp={params[0]:.3f}, Ki={params[1]:.3f}, Kd={params[2]:.3f}")
    print(f"初期コスト: {best_cost:.2f}")
    print(f"最適化開始...\n")

    iteration = 0
    while sum(dp) > tol and iteration < max_iter:
        for i in range(len(params)):
            # パラメータを増やしてみる
            params[i] += dp[i]
            cost = evaluate(params)

            if cost < best_cost:
                best_cost = cost
                best_params = list(params)
                dp[i] *= 1.1  # 探索幅を広げる
            else:
                # パラメータを減らしてみる
                params[i] -= 2 * dp[i]
                if params[i] < 0 and i > 0:  # Ki, Kd は非負
                    params[i] = 0
                    dp[i] *= 0.5
                    continue

                cost = evaluate(params)
                if cost < best_cost:
                    best_cost = cost
                    best_params = list(params)
                    dp[i] *= 1.1
                else:
                    # どちらも改善しない → 元に戻して探索幅を縮小
                    params[i] += dp[i]
                    dp[i] *= 0.5

        cost_history.append(best_cost)
        iteration += 1

        if iteration % 20 == 0:
            print(f"  Iter {iteration}: Kp={best_params[0]:.3f}, "
                  f"Ki={best_params[1]:.3f}, Kd={best_params[2]:.3f}, "
                  f"Cost={best_cost:.2f}")

    params = best_params
    print(f"\n最適化完了 ({iteration} iterations)")
    print(f"最適PID値: Kp={params[0]:.4f}, Ki={params[1]:.4f}, Kd={params[2]:.4f}")
    print(f"最終コスト: {best_cost:.2f}")

    return best_params, cost_history


# ============================================
# 結果の可視化
# ============================================

def plot_comparison(drone, old_params, new_params):
    """最適化前後のPID応答を比較プロット"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("PID Auto-Tune: Before vs After", fontsize=14)

    disturbances = [15.0, -10.0, 5.0, -20.0]
    titles = ["15° disturbance", "-10° disturbance",
              "5° disturbance", "-20° disturbance"]

    np.random.seed(42)  # 再現性のため

    for idx, (dist, title) in enumerate(zip(disturbances, titles)):
        ax = axes[idx // 2][idx % 2]

        # 旧パラメータ
        np.random.seed(42 + idx)
        t_old, a_old, c_old = simulate_axis(*old_params, drone, dist)

        # 新パラメータ
        np.random.seed(42 + idx)
        t_new, a_new, c_new = simulate_axis(*new_params, drone, dist)

        ax.plot(t_old, a_old, 'r-', alpha=0.7, linewidth=1,
                label=f"Before (cost={c_old:.1f})")
        ax.plot(t_new, a_new, 'b-', alpha=0.7, linewidth=1,
                label=f"After (cost={c_new:.1f})")
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.fill_between(t_old, -1, 1, alpha=0.1, color='green',
                        label="±1° target zone")
        ax.set_title(title)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Angle [deg]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-30, 30)

    plt.tight_layout()
    plt.savefig("pid_autotune_comparison.png", dpi=150)
    print("\nグラフを pid_autotune_comparison.png に保存しました")
    plt.show()


def plot_cost_history(cost_history):
    """最適化のコスト推移をプロット"""
    plt.figure(figsize=(8, 4))
    plt.plot(cost_history, 'b-o', markersize=2)
    plt.xlabel("Iteration")
    plt.ylabel("Cost")
    plt.title("Twiddle Optimization Progress")
    plt.grid(True, alpha=0.3)
    plt.yscale('log')
    plt.tight_layout()
    plt.savefig("pid_autotune_cost.png", dpi=150)
    plt.show()


# ============================================
# メイン
# ============================================

if __name__ == "__main__":
    print("=" * 50)
    print("  Halo Drone PID Auto-Tuner (Twiddle)")
    print("=" * 50)
    print()

    drone = DroneParams()

    # 現在のPIDゲイン
    old_params = [6.06, 0.00, 0.83]

    print("--- 現在のPID値でシミュレーション ---")
    np.random.seed(42)
    _, _, old_cost = simulate_axis(*old_params, drone, 15.0)
    print(f"  Kp={old_params[0]}, Ki={old_params[1]}, Kd={old_params[2]}")
    print(f"  Cost: {old_cost:.2f}\n")

    print("--- Twiddle最適化開始 ---")
    np.random.seed(0)
    best_params, cost_history = twiddle(drone, initial_params=old_params)

    print(f"\n{'=' * 50}")
    print(f"  結果")
    print(f"{'=' * 50}")
    print(f"  Before: Kp={old_params[0]:.4f}, Ki={old_params[1]:.4f}, Kd={old_params[2]:.4f}")
    print(f"  After:  Kp={best_params[0]:.4f}, Ki={best_params[1]:.4f}, Kd={best_params[2]:.4f}")
    print()
    print("  halo_controller_improved.ino に書くべき値:")
    print(f"    float Kp_roll_angle = {best_params[0]:.2f};")
    print(f"    float Ki_roll_angle = {best_params[1]:.2f};")
    print(f"    float Kd_roll_angle = {best_params[2]:.2f};")
    print()
    print(f"    float Kp_pitch_angle = {best_params[0]:.2f};")
    print(f"    float Ki_pitch_angle = {best_params[1]:.2f};")
    print(f"    float Kd_pitch_angle = {best_params[2]:.2f};")
    print()

    # 可視化
    plot_comparison(drone, old_params, best_params)
    plot_cost_history(cost_history)
