# FallDetection_Kalman
This project simulates a fall detection system inspired by smartwatches like Garmin or Apple Watch. It uses a Kalman Filter to estimate position, velocity, and acceleration from noisy sensor data, and detects falls based on a sudden drop in the estimated acceleration.

# Project Summary
Goal: Estimate a person’s motion in real-time and detect a fall event using simulated sensor data.

Sensors Simulated:

📡 GPS-like position data (sparse and noisy)

⚙️ Accelerometer data (frequent and noisy)

Core Technique: Kalman Filter with a Position-Velocity-Acceleration (PVA) state model.

Fall Detection: Triggers when the estimated acceleration drops below a threshold, simulating a free-fall event.

# Features
Kalman Filter implementation with continuous prediction + measurement updates

Sensor fusion from multiple noisy sources

Visualization of:

- True vs. Estimated motion

- Noisy vs. Filtered acceleration

- Detected fall event

- Fall detection logic using physically meaningful acceleration thresholds

# Tools & Technologies
- Python

- NumPy

- Matplotlib

# How It Works
1) Simulated Motion: A synthetic motion profile is generated with sinusoidal acceleration and a sudden "fall" (drop) at a specified time.

2) Sensor Noise:

- Acceleration readings are noisy.

- Position updates occur every 1 second and include noise.

3)Kalman Filter:

- Uses a constant acceleration (PVA) model.

- Estimates the full state vector: [position, velocity, acceleration].

4) Fall Detection:

- If estimated acceleration < -5 m/s² → fall is flagged.

- Detected fall is shown on the plot with a vertical red line.


# Concepts Covered
- Kalman Filtering for continuous systems

- Sensor Fusion: combining high-rate inertial data and sparse GPS-like data

- Real-time State Estimation (Position, Velocity, Acceleration)

- Fall Detection via proper acceleration analysis

- Signal Denoising and Dynamic Modeling

# Let's Connect
This project combines control theory and real-world health-tech applications.
Feel free to reach out or collaborate if you're working in:

- Controls & Estimation

- Wearable Tech

- Embedded Health Monitoring

- Robotics or Signal Processing
