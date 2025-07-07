import numpy as np
import matplotlib.pyplot as plt

class KalmanFilterPVA:
    """
    Kalman Filter for estimating Position, Velocity, and Acceleration
    using acceleration (frequent) + position (occasional) measurements
    """
    def __init__(self, dt, process_var, meas_var_acc, meas_var_pos):
        self.dt = dt

        # State: [position, velocity, acceleration]
        self.A = np.array([
            [1, dt, 0.5 * dt ** 2],
            [0,  1,        dt],
            [0,  0,         1]
        ])

        self.R = np.eye(3) * process_var       # Process noise covariance
        self.Q_acc = meas_var_acc              # Acceleration measurement noise
        self.Q_pos = meas_var_pos              # Position measurement noise

        self.x = np.zeros((3, 1))              # Initial state
        self.P = np.eye(3)                     # Initial covariance
        self.I = np.eye(3)

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.R

    def update_accel(self, z):
        C = np.array([[0, 0, 1]])              # Measure acceleration
        S = C @ self.P @ C.T + self.Q_acc
        K = self.P @ C.T @ np.linalg.inv(S)
        y = z - C @ self.x
        self.x = self.x + K @ y
        self.P = (self.I - K @ C) @ self.P

    def update_pos(self, z):
        C = np.array([[1, 0, 0]])              # Measure position
        S = C @ self.P @ C.T + self.Q_pos
        K = self.P @ C.T @ np.linalg.inv(S)
        y = z - C @ self.x
        self.x = self.x + K @ y
        self.P = (self.I - K @ C) @ self.P

    def get_state(self):
        return self.x.flatten()


def simulate_motion(t, fall_time=12, fall_magnitude=-15, noise_acc=3.0, noise_pos=2.0):
    # True motion
    acc_true = 0.5 * np.sin(2 * np.pi * 0.5 * t)
    fall_index = int(fall_time / (t[1] - t[0]))
    acc_true[fall_index:fall_index+3] += fall_magnitude

    vel_true = np.cumsum(acc_true) * (t[1] - t[0])
    pos_true = np.cumsum(vel_true) * (t[1] - t[0])

    acc_meas = acc_true + np.random.normal(0, noise_acc, size=len(t))

    # Simulated position measurements every 10 steps (like GPS at 1 Hz)
    pos_meas = np.full_like(t, np.nan)
    pos_noise = np.random.normal(0, noise_pos, size=len(t))
    for i in range(0, len(t), 10):
        pos_meas[i] = pos_true[i] + pos_noise[i]

    return pos_true, vel_true, acc_true, acc_meas, pos_meas


def main():
    dt = 0.1
    t = np.arange(0, 40, dt)

    # Simulate data
    pos_true, vel_true, acc_true, acc_meas, pos_meas = simulate_motion(
        t, noise_acc=3.0, noise_pos=2.0
    )

    # Kalman filter config
    process_var = 0.3
    meas_var_acc = 2.0     # std dev^2 = 3^2
    meas_var_pos = 0.5     # std dev^2 = 2^2

    kf = KalmanFilterPVA(dt, process_var, meas_var_acc, meas_var_pos)

    pos_kf, vel_kf, acc_kf = [], [], []

    for i in range(len(t)):
        kf.predict()
        kf.update_accel(np.array([[acc_meas[i]]]))
        if not np.isnan(pos_meas[i]):
            kf.update_pos(np.array([[pos_meas[i]]]))

        x = kf.get_state()
        pos_kf.append(x[0])
        vel_kf.append(x[1])
        acc_kf.append(x[2])
    
    # Detect fall event based on estimated acceleration from Kalman Filter
    fall_threshold = -5
    acc_kf_arr = np.array(acc_kf)
    fall_indices = np.where(acc_kf_arr < fall_threshold)[0]

    fall_time = t[fall_indices[0]] if len(fall_indices) > 0 else None

    # Plotting
    fig, axs = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

    axs[0].plot(t, pos_true, label="True Position", linestyle="--")
    axs[0].plot(t, pos_meas, label="Position Measured (GPS-like)", color="gray", alpha=0.4)
    axs[0].plot(t, pos_kf, label="Kalman Estimated Position", color="r")
    axs[0].set_ylabel("Position (m)")
    axs[0].legend()
    axs[0].grid()

    #axs[1].plot(t, vel_true, label="True Velocity", linestyle="--")
    #axs[1].plot(t, vel_kf, label="Kalman Estimated Velocity", color="g")
    #axs[1].set_ylabel("Velocity (m/s)")
    #axs[1].legend()
    #axs[1].grid()

    axs[1].plot(t, acc_true, label="True Acceleration", linestyle="--")
    axs[1].plot(t, acc_meas, label="Measured Acceleration", alpha=0.3, color="gray")
    axs[1].plot(t, acc_kf, label="Kalman Estimated Acceleration", color="b")
    axs[1].set_ylabel("Acceleration (m/s²)")
    axs[1].set_xlabel("Time (s)")
    axs[1].legend()
    axs[1].grid()
    
    if fall_time is not None:
        for ax in axs:
            ax.axvline(fall_time, color='r', linestyle='--', label='Fall Detected')
        axs[0].text(fall_time, axs[0].get_ylim()[1]*0.9, 'Fall Detected', color='r', fontsize=12)

    plt.suptitle("Kalman Filter Fusion of Acceleration + Position (GPS) Measurements")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
