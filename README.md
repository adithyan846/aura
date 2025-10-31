# **Aura ROS**
**An intelligent aerial robotics framework for disaster management and field operations.**

Aura ROS integrates real-time perception, flight telemetry, and onboard AI vision into a cohesive system built for the Raspberry Pi 5 and Pixhawk-based drones. It’s designed as a modular ROS 2 (Jazzy Jalisco) ecosystem capable of autonomous detection, communication, and coordination across complex mission profiles.

---

## **System Overview**

Aura ROS is developed as part of the *Aura Drone Project*, a next-generation disaster-response UAV platform.  
The ROS layer acts as the central nervous system — handling sensor integration, perception, decision logic, and telemetry relay between the flight controller (Pixhawk) and onboard AI modules.

---

## **Core Features**

| Category | Description |
|-----------|--------------|
| **Hardware Platform** | Raspberry Pi 5 running Ubuntu 24.04 |
| **Flight Controller** | Pixhawk 2.4.8 communicating via MAVLink |
| **ROS Distribution** | ROS 2 Jazzy Jalisco |
| **AI Module** | YOLOv11n (custom-trained) for human and object detection |
| **Telemetry Node** | Interfaces with Pixhawk to stream flight data, GPS, and system status |
| **AI Node** | Processes real-time camera feed and publishes detections |
| **Camera Node** | Handles image acquisition using the RPi camera (libcamera or V4L2 pipeline) |
| **Bringup System** | Unified launch mechanism to start all Aura subsystems |
| **Modular Design** | Each functional block is an independent ROS 2 package for clarity and reusability |

---

## **Repository Structure**

├── src/
│ ├── aura_ai/ # AI node (YOLOv11n detection and inference)
│ ├── aura_camera/ # Camera node for RPi video feed
│ ├── aura_telemetry/ # MAVLink-based telemetry and GPS data node
│ ├── aura_bringup/ # Unified launch files and configuration
│ └── aura_interfaces/ # Custom ROS message definitions (Detection, Telemetry)



---

## **Installation**

### 1. System Requirements
- Ubuntu 24.04 (64-bit)
- Raspberry Pi 5
- ROS 2 Jazzy Jalisco
- Python 3.10+
- OpenCV, cv_bridge, and ultralytics dependencies installed

### 2. Clone the Workspace
```bash
mkdir -p ~/aura_ws/src
cd ~/aura_ws/src
git clone https://github.com/adithyan846/aura.git
cd ~/aura_ws
colcon build
source install/setup.bash


Development Notes

The AI node is optimized for edge inference using YOLOv11n (custom trained weights).

Telemetry communication uses MAVLink over serial (typically /dev/ttyAMA0 or /dev/ttyUSB0).

ROS launch files are designed to bring up all systems in a controlled order.

Modular packages ensure plug-and-play debugging and isolated testing on the Raspberry Pi.

Roadmap

 Multi-drone coordination layer

 Fail-safe and recovery node

 Integration with ground base station via radio/4G link

 Mission planner interface

 Onboard decision and navigation AI


 License

This project is released under the MIT License unless otherwise specified.
© 2025 Aura Research Group. All rights reserved.

Citations

If you reference Aura ROS in academic or technical work, please cite it as:

Aura ROS: A Modular AI-Integrated Drone Framework for Disaster Response.
