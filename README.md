
## 𝄜 Table of Contents

* [Autonomous Steering-Wheel Robot Car – Control System & Software Architecture](#autonomous-steering-wheel-robot-car--control-system--software-architecture)

  * [➔ System Overview](#-system-overview)

    * [1. Power Distribution](#power-distribution)
    * [2. Actuation](#actuation)
    * [3. Sensors](#sensors)
  * [➔ Code Architecture](#-code-architecture)
  * [➔ Relationship Between Software & Electromechanical Components](#-relationship-between-software--electromechanical-components)
  * [➔ Process to Build, Compile, and Load the Code](#-process-to-build-compile-and-load-the-code)

    * [1. Opening and Editing Code in Geany](#1-opening-and-editing-code-in-geany)
    * [2. Running the Code from the Terminal](#2-running-the-code-from-the-terminal)
  * [➔ Future Improvements](#-future-improvements)
  * [➔ License](#-license)

---

# Autonomous Steering-Wheel Robot Car – Control System & Software Architecture

This repository contains the complete documentation and software for an autonomous robot car using a steering-wheel (Ackermann-style) mechanism.
The system integrates a **Raspberry Pi 4**, an **MG996R servo**, a **DC motor controlled with DRV8870**, and a full sensor suite consisting of **two HC-SR04 ultrasonic sensors** and **two VL53L0X laser time-of-flight sensors**.
The purpose of this documentation is to describe:

* The architecture of the software developed.
* The modules that compose the code and their direct relationship with the electromechanical components of the vehicle.
* The process for building, compiling, and loading the code into the robot controllers.
* The workflow used to execute the robot’s main routines in Raspberry Pi OS.

---

## ➔ System Overview

The robot is a semi-autonomous vehicle designed for navigation, obstacle avoidance, and directional control using vision and distance sensing.
The Raspberry Pi 4 acts as the **primary controller** while the DRV8870 and MG996R interface provide the mechanical actuation of propulsion and steering.

The key subsystems are:

### ◆ 1. Power Distribution  <a id="power-distribution"></a>

* **LiPo 3S (11.1 V)** → main power source
* **General switch**
* **XL4016 buck converter** → provides stable 5V for Raspberry Pi and servo
* **UBEC (optional)** → provides isolated 5V for sensors
* Ground reference shared across all modules

### ◆ 2. Actuation  <a id="actuation"></a>

* **DC Motor** driven by the DRV8870 H-bridge
* **MG996R Servo** controlling the steering angle
* PWM and digital outputs from Raspberry Pi for speed and direction

### ◆ 3. Sensors  <a id="sensors"></a>

* **HC-SR04 (Front / Rear)**: detects obstacles at medium distance
* **VL53L0X (Left / Right)**: detects walls or side distances with millimeter precision
* **Webcam (Front)**: vision input for advanced routines (optional module)

---

## ➔ Code Architecture

The codebase is divided into several logical modules. Each one is responsible for controlling a specific hardware component or algorithm.
Below is a description of each module and how it relates to the physical hardware of the robot.

### ### 1. `motor_control.py`

**Purpose:** Control the rotational speed and direction of the DC motor.
**Electromechanical relation:**

* Sends PWM signals to the **DRV8870 EN pin**
* Uses digital outputs for **N1/N2** to determine direction
* Directly affects vehicle propulsion

### ### 2. `servo_steering.py`

**Purpose:** Turn the steering wheel using the MG996R servo.
**Electromechanical relation:**

* Requires a **stable 5V line** from XL4016
* Reads/writes precise PWM signals to position the servo horn
* Steering angles define the motion path of the robot

### ### 3. `ultrasonic_module.py`

**Purpose:** Manage distance readings from the HC-SR04 sensors.
**Electromechanical relation:**

* Uses **5V from UBEC or Pi** and needs common ground
* TRIG and ECHO connected to RPi GPIO pins
* Provides centimeter-level detection of frontal and rear obstacles

### ### 4. `tof_module.py`

**Purpose:** Measure side distances using VL53L0X ToF sensors.
**Electromechanical relation:**

* Communicates via **I2C (SDA/SCL)**
* Requires 3.3V stable power
* Provides millimeter-accuracy readings essential for precise navigation

### ### 5. `vision_module.py` (optional)

**Purpose:** Capture and display the webcam feed.
**Electromechanical relation:**

* Uses USB camera
* Requires Raspberry Pi 4 performance for real-time processing

### ### 6. `gui_interface.py`

**Purpose:** Provide a graphical interface showing:

* Camera feed
* Motor speed
* Servo angle
* Sensor measurement table
* Text describing the current robot routine

Works using Python + available GUI frameworks (e.g., Tkinter, PyQt, or OpenCV imshow).

### ### 7. `main.py`

**Purpose:** Integrate every subsystem, execute routines, and run the robot.
**Electromechanical relation:**

* Acts as the brain controlling all modules synchronously
* Makes decisions based on sensor inputs
* Executes movement commands through the motors and steering

> [!NOTE]
> The following figure shows the **general flowchart** of `main.py` for the robot car: it loads the configuration, initializes the motor, steering, distance sensors, optional vision module and graphical interface; then it enters a loop where it reads all sensors, processes the camera if it is enabled, computes speed and steering, updates the interface, and when an exit is requested, it stops the car and releases resources.

![Control flow diagram](<other/Flowchart.png>)

---

## ➔ Relationship Between Software & Electromechanical Components

The following list summarizes the mapping between code modules and physical hardware:

| Code Module            | Hardware Component | Purpose                         |
| ---------------------- | ------------------ | ------------------------------- |
| `motor_control.py`     | BTS7960 + DC Motor | Speed / direction control       |
| `servo_steering.py`    | MG996R Servo       | Steering angle                  |
| `ultrasonic_module.py` | HC-SR04 Sensors    | Front/rear distance detection   |
| `tof_module.py`        | VL53L0X Sensors    | Left/right distance measurement |
| `vision_module.py`     | Webcam             | Vision processing               |
| `gui_interface.py`     | Raspberry Pi       | Visualization of robot state    |
| `main.py`              | Entire Robot       | Integrates all behavior         |

> [!NOTE]
> The following figure shows the **General Wiring Diagram** of the robot car: a Raspberry Pi 4 is connected to the steering servo, the DC drive motor and the distance sensors (two VL53L0X ToF modules and two HC-SR04 ultrasonic modules), while all components are powered from a Li-Po battery through a main switch and a step-down regulator, sharing a common ground.

![General circuit diagram](<schemes/3. General Circuit Diagram.png>)

---

## ➔ Process to Build, Compile, and Load the Code

(*Improved version of what you already had*)

## 1. Opening and Editing Code in Geany

The entire development process is performed directly on the Raspberry Pi OS using **Geany**, a lightweight IDE.

> [!NOTE]
> **Geany** is a simple but powerful IDE available in Raspberry Pi OS, capable of handling Python scripts with syntax highlighting and run support.

### **Steps:**

1. Open Geany in Raspberry Pi OS.

   <!-- Add image: Geany icon or desktop -->

   <!-- ![Open Geany](docs/images/open-geany.png) -->

2. Create a new file or open an existing `.py` file.

   <!-- Add image showing unsaved code -->

3. Write your Python code normally.
   Geany automatically provides indentation, coloring, and Python formatting.

4. Save the file with the `.py` extension.
   Example: `main.py`

> [!TIP]
> Once saved with `.py`, Geany automatically treats the file as a Python script.

---

## 2. Running the Code from the Terminal

To run your script:

1. Open the Raspberry Pi OS terminal.

   <!-- Add image: Terminal window -->

2. Navigate to the folder where your file is located:

```bash
cd /home/pi/my_robot_code/
```

3. Execute the script with:

```bash
python3 my_script.py
```

Replace `my_script.py` with your actual filename.

The program will start running immediately, activating:

* Sensor readings
* Motor control
* Steering
* GUI
* Optional vision modules

---

## ➔ Future Improvements

* Add IMU (accelerometer + gyroscope)
* Expand GUI with debugging tools
* Implement advanced vision-based navigation
* Add PID controllers for both motor and steering

---

## ➔ License

This project is open-source and licensed under the MIT License.
