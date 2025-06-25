# Embedded Spatial Scanner (LiDAR) 🛰️

A low-cost **3D mapping system** that rotates a VL53L1X **Time-of-Flight** sensor on an **MSP432 microcontroller** and streams distance readings over UART to render **real-time 3D point clouds** with Python.  

## 🔍 Explore  
📄 Project Overview [COMING SOON]      
📁 Code Files: `2dx_project_final.c`, `scanner_visualizer.py`        
🔀 [Data Flow Diagram](data_flow_diagram.png) 🗂️ [Flowchart](flowchart_diagram.png) 🔌 [Circuit Schematic](circuit_schematic.png)  
🖥️ [Device Setup](device_setup.png)     
🏠 [Hallway Scanned](hallway_scanned.png)      
📊 [Scanner Output #1](scanner_output_1.png)      
📊 [Scanner Output #2](scanner_output_2.png)      
🖥️ [Device Setup](device_setup.png)      

---

## 🌱 Background
Commercial LiDAR scanners are expensive and bulky, limiting their use in educational and budget-conscious prototyping environments. Inspired by a need for accessible spatial mapping, this project demonstrates how an MSP432E401Y paired with a VL53L1X sensor and a simple stepper can capture 360° distance data for under CA$110.

---

## 🔧 How It Works

1. **Stepper & Sensor Control**  
   - MSP432E401Y steps a 28BYJ-48 motor via ULN2003 at 11.25° increments.  
   - VL53L1X measures distance (up to 4 m) via I2C at each angular position.  
2. **UART Streaming**  
   - Firmware packages millimeter readings into a byte stream at 115,200 bps.  
   - A pushbutton toggles scanning on demand, and LEDs indicate status.  
3. **PC Visualization**  
   - `scanner_visualizer.py` uses `pyserial` to read UART data, converts polar coordinates (angle + distance) to Cartesian points, and renders live 3D slices in Open3D.

---

## 📂 Code Files

- **`2dx_project_final.c`** — MSP432 firmware: I2C setup, stepper control, distance sampling, UART transmission.  
- **`scanner_visualizer.py`** — Host-side Python: serial communication, data parsing, point cloud rendering.  

> **Note:** Only core implementation files are included here for review. Full source repositories, hardware schematics, and build scripts will accompany sample outputs and diagrams.

---

## ⚙️ Tech Stack

- **Microcontroller:** MSP432E401Y (ARM Cortex-M4) with Keil uVision5  
- **Sensor:** ST VL53L1X Time-of-Flight (I2C, address 0x29)  
- **Actuator:** 28BYJ-48 stepper motor + ULN2003 driver  
- **Comm:** UART at 115,200 bps  
- **Languages:** C (firmware), Python 3.10+ (visualizer)  
- **Libraries:** `pyserial`, `numpy`, `open3d`

---

## 📈 Outcomes & Schematics

- **Key Results:**  
  - Reliable 360° scans capturing clear object silhouettes.  
  - Achieved ~2 full scans per second (32 points/rev).  
  - Total system cost: CA$106.16.  
- **Sample Outputs:**  
  - Point cloud snapshots (`output_frame_001.png`, `scan_data.xyz`).  
- **Schematics:**  
  - Wiring diagram for MSP432 ↔ VL53L1X ↔ stepper ↔ UART [COMING SOON].

---

## 🤝 Connect

Have feedback or want to collaborate? Reach out!  
📧 Yash: yash.panchal1932@gmail.com  
📧 Partner: kalpkansara123@gmail.com  
🌐 GitHub: [yashp1932](https://github.com/yashp1932)

---

## 📑 One-Page Project Summary

**Project:** Affordable 3D Spatial Scanner using MSP432 + VL53L1X  
**Cost:** < CA$110  
**Date:** April 9, 2025

**Objective:**  
Enable 360° spatial mapping with a rotating ToF sensor and real-time PC visualization on minimal hardware.

**System Flow:**  
1. **Acquire:** MSP432 controls stepper and reads VL53L1X distances at fixed angles.  
2. **Transmit:** Millimeter readings streamed via UART at 115,200 bps.  
3. **Render:** Python script converts polar data to 3D points and displays with Open3D.

**Key Results:**  
- Demonstrated clear object outlines at ~2 Hz scan rate.  
- Verified cost-effectiveness versus commercial LiDAR.

**Next Steps:**  
- Integrate vertical axis motion or motorized lift for volumetric scans.  
- Add onboard SD logging for offline analysis.

**Files Provided:**  
- `2dx_project_final.c`  
- `scanner_visualizer.py`

**Extras to Follow:**  
- Full hardware schematics  
- Sample point cloud datasets  
- Detailed build and deployment guide


