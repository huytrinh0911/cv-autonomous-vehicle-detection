# Autonomous Driving Vision System
Team of 3  
Real-time traffic sign detection and lane tracking on Jetson TX2.  
**Technologies:** Python, OpenCV, YOLOv4-Tiny, ONNX, TensorRT, Jetson TX2, GoPro  

- Trained YOLOv4-Tiny on a custom traffic sign dataset using Google Colab  
- Converted model to ONNX and deployed it with TensorRT for real-time inference  
- Implemented lane detection using OpenCV and sent lane position to STM32F407 for steering control
## Overview
This project implements an **autonomous driving assistance system** capable of:
- **Lane keeping** using real-time camera input.
- **Traffic sign detection and recognition** for 8 predefined classes using **YOLOv4-tiny** with **20 FPS**.
- **Vehicle control and monitoring** via a graphical user interface (GUI).
- **Automated driving decisions** such as slowing down, turning, or stopping, based on recognized traffic signs or user commands.

---
![demo.png](https://raw.githubusercontent.com/huytrinh0911/cv-autonomous-vehicle-detection/refs/heads/main/demo.png)
## Features

### 1. Lane Keeping
- Utilizes camera feed to detect and follow lane lines.
- Maintains vehicle position within the lane in real time.

### 2. Traffic Sign Recognition
- Detects and classifies 8 traffic signs:  
1. **Bus_Stop**  
2. **Children_Crossing**  
3. **Green_Light**  
4. **Left_Turn_Only**  
5. **No_Stopping**  
6. **Red_Light**  
7. **Speed_Limit_40**  
8. **Stop**  
- Powered by **YOLOv4-tiny** for fast and efficient inference.

▶️ [Demo Video](https://drive.google.com/drive/folders/1osMUJ9KnHjuEjdHjcsuOOYBj8tc4ppCK?usp=sharing)
