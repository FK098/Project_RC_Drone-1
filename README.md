# 🛸 Arduino Nano Flight Controller — A From-Scratch Drone Firmware

> A deterministic quadcopter Flight Controller built from scratch on an 8-bit AVR architecture (**Arduino Nano / ATmega328P**), featuring custom low-level drivers for sensors, sensor fusion algorithms, and 2.4 GHz radio telemetry.

![Status](https://img.shields.io/badge/status-in_development-yellow)
![License](https://img.shields.io/badge/license-MIT-blue)
![Platform](https://img.shields.io/badge/platform-AVR%20%7C%20Arduino-red)

---

## 👨‍💻 About Me

I'm a **Computer Engineering undergraduate at Politecnico di Milano**, passionate about embedded systems, firmware development, robotics, and low-level hardware/software architectures.

This repository documents the **complete design and implementation of a drone's onboard firmware**: from reading raw sensor registers via I2C/SPI, to writing mathematical filters and sensor fusion, up to real-time stabilization using cascaded PID control loops — all without relying on pre-built flight controller libraries.

- **GitHub:** [@FK098](https://github.com/FK098)

---

## 📌 Project Overview

Most commercial flight controllers rely on high-level libraries or pre-compiled firmware (Betaflight, ArduPilot, etc.). This project takes the opposite approach: **every driver, filter, and control algorithm is written from the component datasheets**, with a strict focus on the severe constraints of the ATmega328P:

- ⚡ **2 KB SRAM** — every byte counts
- ⏱️ **16 MHz clock** — deterministic timing is mandatory
- 📉 **8-bit architecture** — no floating-point luxury (but we use it where unavoidable)

### ✨ Key Features

- **Deterministic Flight Loop @ 250 Hz:** Fixed-cycle 4 ms scheduling ensures stable convergence of cascaded PID loops (Roll, Pitch, Yaw rate).
- **Custom Sensor Libraries (`Mio*`):** Bare-metal I2C drivers written from datasheets for the MPU-6050, BMP280, and QMC5883L, with register-level control, DLPF filtering, and factory calibration parsing.
- **Sensor Fusion:** Complementary filter for attitude estimation (gyro integration + accelerometer gravity reference) and tilt-compensated magnetometer heading.
- **Cascaded PID Architecture:** Outer angle loop → inner rate loop → motor mixer, with integral windup protection and derivative-on-measurement to avoid setpoint kicks.
- **2.4 GHz Radio Link (`nRF24L01+`):** Bidirectional SPI communication using a custom binary `struct`-based packet protocol with CRC and failsafe timeout.
- **ESC & Motor Management:** Standard PWM (1000–2000 µs), armed/disarmed state machine, software deadband, and automatic throttle-range calibration routines.
- **Hard/Soft Iron Magnetometer Calibration:** In-flight 3D rotation routine to compute bias offsets for reliable heading hold.

------

## 📐 Pinout & Wiring Diagram

### I2C Bus (Sensors)
All I2C devices share the same bus. The MPU-6050 and BMP280 are standard, while the QMC5883L is mounted as far away from the high-current ESC wires as possible to avoid magnetic interference.

| Sensor Pin | Arduino Nano Pin | Notes |
| :--- | :--- | :--- |
| **SDA** | A4 | I2C Data Line |
| **SCL** | A5 | I2C Clock Line (400 kHz Fast Mode) |

### SPI Bus (nRF24L01 Radio)
> ⚠️ **Note:** Powered strictly via the AMS1117 3.3V step-down regulator. A 100µF bypass capacitor is soldered directly to the radio's VCC/GND pins to prevent voltage drops during transmission spikes.

| nRF24L01 Pin | Arduino Nano Pin | Description |
| :--- | :--- | :--- |
| **CE** | D7 | RX/TX activation |
| **CSN** | D8 | SPI Chip Select |
| **SCK** | D13 | Serial Clock |
| **MOSI** | D11 | Master Out Slave In |
| **MISO** | D12 | Master In Slave Out |

### Motor Outputs (PWM)
| ESC Channel | Arduino Nano Pin | Quadcopter Position (X Config) |
| :--- | :--- | :--- |
| **Motor 1** | D3 | Front Right (CCW) |
| **Motor 2** | D5 | Rear Right (CW) |
| **Motor 3** | D6 | Rear Left (CCW) |
| **Motor 4** | D9 | Front Left (CW) |

---

## 🏗️ Hardware Assembly & 3D Printed Parts

To ensure the sensors operate correctly, vibration isolation is critical. If you are fabricating custom FDM mounts or structural chassis components for the drone, they will be uploaded to the `hardware/stl/` directory. Currently, the flight controller stack requires:
- Anti-vibration rubber standoffs for the Arduino/IMU mount.
- A non-magnetic mast/spacer for the QMC5883L compass.

---

## 🚀 Getting Started

### Prerequisites
- Arduino IDE (v2.x) or VS Code + PlatformIO.
- AVR Board Package installed.

### Installation & Calibration
1. **Clone the repo:**
   ```bash
   git clone [https://github.com/FK098/arduino-flight-controller.git](https://github.com/FK098/arduino-flight-controller.git)
