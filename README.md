# Embedded Spatial Scanner (LiDAR) 🛰️

A low-cost **3D mapping system** that rotates a VL53L1X **Time-of-Flight** sensor on an **MSP432 microcontroller** and streams distance readings over UART to render **real-time 3D point clouds** with Python.  

## 🔍 Explore  
📄 Project Overview [COMING SOON]      
📁 Code Files: `2dx_project_final.c`, `scanner_visualizer.py`        
🔀 [Data Flow Diagram](data_flow_diagram.png)   🗂️ [Flowchart](flowchart_diagram.png)   🔌[Circuit Schematic](circuit_schematic.png)  
🏠 [Hallway Scanned](hallway_scanned.png)      
📊 [Scanner Output #1](scanner_output_1.png)      
📊 [Scanner Output #2](scanner_output_2.png)      
🖥️ [Device Setup](device_setup.png)      

> **Note:** Since this is an academic project, only the core implementation files and example images are included, in accordance with university policies.
---

## 📚 Background
Commercial LiDAR scanners are **expensive and bulky**, limiting their use in educational and budget-conscious prototyping environments. Inspired by a need for **accessible spatial mapping** and **exploring embedded systems**, this project demonstrates how an MSP432E401Y paired with a VL53L1X sensor and a simple stepper can capture 360° distance data for under CA$110.

---

## 🔧 How It Works

1. **Stepper & Sensor Control**  
   - MSP432E401Y steps a 28BYJ-48 motor via ULN2003 at 11.25° increments.  
   - VL53L1X measures distance (up to 4 m) via I2C at each angular position.  
2. **UART Streaming**  
   - The firmware packages millimeter readings into a byte stream at 115,200 bps.  
   - A pushbutton toggles scanning on demand, and LEDs indicate status.  
3. **PC Visualization**  
   - `scanner_visualizer.py` uses `pyserial` to read UART data, converts polar coordinates (angle + distance) to Cartesian points, and renders live 3D slices in Open3D.
   - Automatically simulates forward movement between each 2D scan slice by incrementally translating the point cloud along the scanning axis, generating a continuous 3D representation despite the sensor’s 2D limitations.

---

## 📂 Code Files

- **`2dx_project_final.c`** — MSP432 firmware: I2C setup, stepper control, distance sampling, UART transmission.  
- **`scanner_visualizer.py`** — Host-side Python: serial communication, data parsing, point cloud rendering.  

---

## ⚙️ Tech Stack

- **Microcontroller:** MSP432E401Y (ARM Cortex-M4) with Keil uVision5  
- **Sensor:** ST VL53L1X Time-of-Flight (I2C, address 0x29)  
- **Actuator:** 28BYJ-48 stepper motor + ULN2003 driver  
- **Comm:** UART at 115,200 bps  
- **Languages:** C (firmware), Python 3.10+ (visualizer)  
- **Libraries:** `pyserial`, `numpy`, `open3d`

---

## 📈 Key Results
- Built a **synchronized state‑machine control** loop in C, polling I2C and stepping the motor at 11.25° increments to achieve consistent 1 mm radial resolution scans in real time.
- Achieved ~2 full 360° point-cloud scans per second over a 115,200 bps **UART pipeline**, rendered live in Python with Open3D and NumPy.
- **Improved scan accuracy by nearly 20%** via sensor signal tuning, stepper motor calibration, and **latency reduction** optimizations.
- Demonstrated reliable map outlines of indoor environments, including room and hallway scans, with clear object silhouettes.
- Kept total hardware cost under CA$110, offering **significant cost reduction** compared to entry‑level commercial LiDAR systems.
---

## 🤝 Connect

Have feedback or want to collaborate? Reach out!  

📧 Email: yash.panchal1932@gmail.com        
🌐 [GitHub](https://github.com/yashp1932)      
💼 [LinkedIn](https://www.linkedin.com/in/yash-panchal-aba8b12a6/)      

---


