
```markdown
# 🤖 Self-Balancing Robot (PID & Kalman Filter)

![C++](https://img.shields.io/badge/Language-C++-blue.svg)
![Hardware](https://img.shields.io/badge/Hardware-Arduino%20%7C%20MPU6050-orange.svg)
![Status](https://img.shields.io/badge/Status-Completed-success.svg)

> **First Place Winner** - University-wide Robotics Competition, June 2025.

## 📖 Overview
This project is a two-wheeled self-balancing robot based on the inverted pendulum concept. It utilizes a closed-loop control system to maintain stability in real-time. 

The system reads orientation data (pitch angle) from an IMU sensor, filters the noise using a **Kalman Filter**, and computes the necessary motor adjustments using a **PID (Proportional-Integral-Derivative)** controller.

![Robot Demo](https://via.placeholder.com/600x400?text=Upload+a+GIF+of+Robot+Balancing+Here)
*(Note: Replace this image with a GIF/Photo of your actual robot)*

## 🚀 Key Features
* **Real-Time Stability:** Maintains vertical balance within ±2 degrees of error.
* **Sensor Fusion:** Implements **Kalman Filters** to merge Accelerometer data (noisy) and Gyroscope data (prone to drift) for precise angle estimation.
* **PID Control Loop:** Custom-tuned P, I, and D parameters for rapid response to external disturbances.
* **Wireless Tuning:** Integration of a Bluetooth module for real-time parameter tuning without re-uploading code.

## 🛠️ Hardware Stack
* **Microcontroller:** Arduino Uno / Nano
* **Sensor:** MPU6050 (6-Axis Accelerometer & Gyroscope)
* **Motor Driver:** L298N H-Bridge Module
* **Actuators:** 2x High-torque DC Gear Motors
* **Power:** 12V Li-ion Battery Pack
* **Communication:** HC-05 Bluetooth Module

## 💻 Software & Algorithms

### 1. The Kalman Filter
Raw sensor data is inherently imperfect. The accelerometer is sensitive to vibrations, while the gyroscope drifts over time. I implemented a Kalman Filter to estimate the "True Angle."

```cpp
// Pseudocode Logic used in the project
float kalmanCalculate(float newAngle, float newRate, float dt) {
    // Predict phase
    rate = newRate - bias;
    angle += dt * rate;

    // Update phase (Correction based on error covariance)
    // ... matrix math for gain calculation ...
    return angle;
}

```

### 2. PID Control

The robot corrects its tilt by driving the wheels in the direction of the fall. The PID controller calculates the motor speed (`Output`) based on the error (`SetPoint - CurrentAngle`).

* **Proportional (Kp):** Reacts to the current error (leaning angle).
* **Integral (Ki):** Reacts to the accumulation of past errors (eliminates steady-state drift).
* **Derivative (Kd):** Predicts future error based on the rate of change (dampens oscillations).
```
## 🔌 Pinout / Wiring

| Component | Pin (Arduino) |
| --- | --- |
| **MPU6050 SDA** | A4 |
| **MPU6050 SCL** | A5 |
| **Motor Enable A** | 9 (PWM) |
| **Motor Enable B** | 10 (PWM) |
| **Bluetooth TX** | RX |
| **Bluetooth RX** | TX |
```
## ⚙️ Installation & Usage

1. **Clone the repository:**
```bash
git clone [https://github.com/Kahaan83/Self-Balancing-robot.git](https://github.com/Kahaan83/Self-Balancing-robot.git)

```


2. **Open the Project:**
Open the `.ino` file in the Arduino IDE.
3. **Install Dependencies:**
Ensure you have the `Wire.h` and `MPU6050` libraries installed via Library Manager.
4. **Upload:**
Connect your board via USB and upload the code.
5. **Calibration:**
Place the robot on a flat surface and hold it upright before powering on to calibrate the gyro offsets.

## 📈 Future Improvements

* Implement a remote control feature via a mobile app.
* Upgrade to an ESP32 for faster processing and WiFi telemetry.
* Add obstacle avoidance using ultrasonic sensors.

## 👤 Author

**Kahaan Shah**

* [GitHub Profile](https://www.google.com/search?q=https://github.com/Kahaan83)
* [LinkedIn](www.linkedin.com/in/kahaan-shah-842095334)

---

*This project was built as part of the University Robotics Competition 2025.*

```

```
