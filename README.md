# 🤖 Self-Balancing Monowheel Robot

This repository contains the complete hardware design, embedded code, documentation, and control algorithm implementations for a **Self-Balancing Monowheel Robot** using a **ball-balancing mechanism**.

Developed as part of a Project-Based Learning (PBL) initiative at **Symbiosis Institute of Technology**, this robot integrates real-time sensor fusion, motor control, and PID-based feedback systems using affordable and accessible components.

---

## 📌 Project Overview

The Self-Balancing Monowheel Robot is a compact, real-time control system that balances on a spherical wheel using sensor feedback and motor actuation. It combines:

- MPU6050 IMU (Accelerometer + Gyroscope)
- ESP32 Microcontroller
- MD220A Motor Driver
- N20 V Motors with Encoders

> The robot is capable of restoring balance from disturbances and navigating smoothly across flat terrain.

---

## ⚙️ Hardware Used

| Component              | Description                                 |
|------------------------|---------------------------------------------|
| ESP32                  | Dual-core microcontroller (control unit)    |
| MPU6050                | 6-axis IMU (sensing unit)                   |
| MD220A Motor Driver    | Dual-channel motor driver                   |
| N20 V Motors + Encoders| High torque motors with feedback            |
| Spherical Ball         | Balancing mechanism                         |
| Omni Wheels            | Supports the ball from multiple directions  |
| 3D Printed Chassis     | Holds all components compactly              |

---

## 🧠 Control Algorithm

The control system is based on a **PID (Proportional-Integral-Derivative)** controller implemented in C++ on the ESP32. The MPU6050 provides real-time orientation data which is fused using a complementary filter to estimate the tilt angle. The controller then adjusts motor PWM outputs accordingly to maintain balance.

> Future versions will integrate LQR and adaptive control methods for comparison and improved performance.

---

## 🖥️ Software Setup

### 1. Requirements
- Arduino IDE
- ESP32 Board Package
- Libraries:
  - `Wire.h`
  - `Adafruit MPU6050`
  - `PID_v1.h`
  - `Encoder.h`

### 2. ESP32 Setup
- Install board support from:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

### 3. Uploading Code
- Open `main.ino` file in Arduino IDE.
- Select **Board**: `ESP32 Dev Module`.
- Select correct **COM Port**.
- Click **Upload**.

---

## 🛠️ Hardware Assembly

1. 3D-print or fabricate the chassis.
2. Mount MPU6050 at the center.
3. Connect N20 motors with encoders to the ball supports.
4. Connect MD220A motor driver between ESP32 and motors.
5. Wire power supply (11.1V LiPo recommended).
6. Upload code and test balance recovery.

---

## 🧪 Performance Metrics

| Test Type             | Result                   |
|------------------------|---------------------------|
| Balance Recovery Time | ~1.2 seconds (avg.)       |
| Max Stable Tilt       | ±2.5°                     |
| Control Frequency     | ~100 Hz                   |
| Surface Adaptability  | Flat / slight incline     |

---

## 📌 Future Scope

- LQR and fuzzy-PID control integration
- Terrain-adaptive stabilization
- Bluetooth remote control via mobile app
- Autonomous path planning (via ROS)

---

## 👨‍💻 Contributors

- **Yash Pund**
- **Rahul Kumar**
- **Varad Desai**
- **Aditya Naik**

Department of Robotics and Automation  
Symbiosis Institute of Technology

---

## 📄 License

This project is for academic and educational use.  
If you use this work, please cite the team and institution.
