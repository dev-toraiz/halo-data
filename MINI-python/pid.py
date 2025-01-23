import matplotlib.pyplot as plt
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint  # Desired setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error

        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

# Simulation parameters
time = np.linspace(0, 10, 1000)  # 10 seconds, 1000 steps
dt = time[1] - time[0]  # Time step

# PID controller setup for roll control
setpoint = 0.0  # Target roll angle (level position)
pid = PIDController(kp=15, ki=0.01, kd=5.0, setpoint=setpoint)

# Drone roll simulation variables
roll_angle = 30.0  # Initial roll angle (in degrees)
angular_velocity = 0.0  # Initial angular velocity (in degrees/sec)
roll_angles = []  # List to store roll angles
control_values = []  # List to store control values

# Simulate the roll dynamics
for t in time:
    control_signal = pid.compute(roll_angle, dt)
    control_values.append(control_signal)

    # Simulate the drone's roll dynamics (simplified model)
    angular_acceleration = control_signal  # Control signal directly affects angular acceleration
    angular_velocity += angular_acceleration * dt
    roll_angle += angular_velocity * dt

    roll_angles.append(roll_angle)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(time, roll_angles, label="Roll Angle", color="blue")
plt.plot(time, [setpoint] * len(time), label="Setpoint", linestyle="--", color="red")
plt.xlabel("Time (s)")
plt.ylabel("Roll Angle (degrees)")
plt.title("Drone Roll Angle PID Control Simulation")
plt.legend()
plt.grid()
plt.show()

