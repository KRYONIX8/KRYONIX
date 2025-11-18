```markdown
# Autonomous Steering-Wheel Robot Car – Control System & Software Architecture

This repository contains the complete documentation and software for an autonomous robot car using a steering-wheel (Ackermann-style) mechanism.  
The system integrates a **Raspberry Pi 4**, an **MG996R servo**, a **DC motor controlled with DRV8870**, and a sensor suite of **two HC-SR04 ultrasonic sensors** and **two VL53L0X ToF sensors**. A front webcam is optionally used for visual navigation.

This README explains:
- The software architecture and modules developed  
- How each module interacts with the electromechanical components  
- The full procedure to build, compile, and load the software  
- How the robot operates in Raspberry Pi OS  

---

# 📑 Table of Contents
- [1. System Overview](#1-system-overview)
- [2. Code Architecture](#2-code-architecture)
  - [2.1 motor_control.py](#21-motor_controlpy)
  - [2.2 servo_steering.py](#22-servo_steeringpy)
  - [2.3 ultrasonic_module.py](#23-ultrasonic_modulepy)
  - [2.4 tof_module.py](#24-tof_modulepy)
  - [2.5 vision_module.py](#25-vision_modulepy)
  - [2.6 gui_interface.py](#26-gui_interfacepy)
  - [2.7 main.py](#27-mainpy)
- [3. Software ↔ Electromechanical Relationship](#3-software--electromechanical-relationship)
- [4. Process to Build, Compile, and Load the Code](#4-process-to-build-compile-and-load-the-code)
  - [4.1 Editing Code in Geany](#41-editing-code-in-geany)
  - [4.2 Running the Code from Terminal](#42-running-the-code-from-terminal)
- [5. Future Improvements](#5-future-improvements)
- [6. License](#6-license)

---

# 1. System Overview

The robot is an autonomous Ackermann-steering vehicle capable of obstacle detection, distance measurement, and directional navigation using multiple sensors and optional computer vision modules.

### 🔹 Power Distribution
- **LiPo 3S (11.1V)** main power source  
- **General ON/OFF switch**  
- **XL4016 Buck Converter** → regulated 5V  
- **UBEC (optional)** for sensors  
- Common GND shared across all hardware

### 🔹 Actuation
- **DC Motor + DRV8870 H-Bridge** for propulsion  
- **MG996R Servo** for steering motion  

### 🔹 Sensors
- **HC-SR04 Ultrasonic Sensors (Front and Rear)**  
- **VL53L0X ToF Sensors (Left and Right)**  
- **Webcam (Front)** for optional video feed  

---

# 2. Code Architecture

The control software is modular. Each module has one responsibility and directly corresponds to hardware in the robot.

---

## 2.1 `motor_control.py`
Handles:
- PWM output to the DRV8870  
- Direction control (N1/N2)  
- Speed regulation  

---

## 2.2 `servo_steering.py`
Controls:
- Steering angle via PWM  
- Calibration and boundary limits  
- Smooth turning maneuvers  

---

## 2.3 `ultrasonic_module.py`
Manages:
- TRIG/ECHO pulse generation  
- Distance calculation  
- Detection of frontal and rear obstacles  

---

## 2.4 `tof_module.py`
Responsible for:
- I2C communication  
- Reading millimeter-accurate distances  
- Handling dual VL53L0X sensors  

---

## 2.5 `vision_module.py` (optional)
Provides:
- Real-time camera feed  
- Frame capture  
- Optional computer vision processing  

---

## 2.6 `gui_interface.py`
Displays:
- Camera stream  
- Motor speed  
- Steering angle  
- Sensor distance table  
- Current robot routine execution  

---

## 2.7 `main.py`
The brain of the system:
- Initializes all modules  
- Polls sensors  
- Sends steering + speed commands  
- Updates GUI  
- Executes navigation routines  

---

# 3. Software ↔ Electromechanical Relationship

| Module               | Hardware Component         | Description |
|---------------------|---------------------------|-------------|
| `motor_control.py`  | DRV8870 + DC Motor        | Speed and direction control |
| `servo_steering.py` | MG996R Servo              | Steering angle control |
| `ultrasonic_module.py` | HC-SR04 Sensors        | Front/rear obstacle detection |
| `tof_module.py`     | VL53L0X Sensors           | Left/right distance measurement |
| `vision_module.py`  | USB Webcam                | Video feed |
| `gui_interface.py`  | Raspberry Pi Display      | Live telemetry GUI |
| `main.py`           | Complete System           | Logic integration |

You may insert your wiring diagram here:

```

<!-- Insert wiring diagram image here -->

<!-- ![General Diagram](docs/images/general-diagram.png) -->

```

---

# 4. Process to Build, Compile, and Load the Code

The robot code is created and executed using **Geany**, included in Raspberry Pi OS.

> **NOTE**  
> Geany is a lightweight IDE commonly used for Python development on Raspberry Pi.

---

## 4.1 Editing Code in Geany

### Steps:

1. **Open Geany** in Raspberry Pi OS  
```

   <!-- Insert image: opening Geany -->

```

2. Create a **new file** or open an existing `.py` file  
```

   <!-- Insert image: new file -->

```

3. Write your Python code normally  
- Syntax highlighting  
- Auto indentation  

4. Save the file with the **`.py` extension**  
```

   <!-- Insert image: saved .py file -->

````

> **TIP**  
> Once saved as `.py`, Geany automatically recognizes it as a Python script.

---

## 4.2 Running the Code from Terminal

1. Open the **Terminal**  
2. Navigate to the folder:

```bash
cd /home/pi/my_robot_code/
````

3. Run:

```bash
python3 my_script.py
```

Replace *my_script.py* with your real filename.

> **NOTE**
> When executed, the robot starts:
>
> * Sensor polling
> * Motor actuation
> * Steering control
> * GUI display
> * (Optional) Vision window

```
<!-- Insert image: terminal example -->
```

---

# 5. Future Improvements

Planned upgrades include:

* IMU integration
* PID control loops
* Lane detection vision algorithms
* Enhanced GUI with diagnostics
* Wireless tuning dashboard

---

# 6. License

This project is licensed under the **MIT License**.

---
