# Aura ROS
### An intelligent aerial robotics framework for disaster management and field operations.

| | |
| :--- | :--- |
| **ROS 2 Distribution** | **Jazzy Jalisco** |
| **Platform** | **Raspberry Pi 5** on Ubuntu 24.04 |
| **Primary Use** | Disaster Response & Field Operations |

[![ROS Build](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://ros.org/)
[![Platform](https://img.shields.io/badge/Platform-RPi%205%20%7C%20Ubuntu%2024.04-orange)](https://www.raspberrypi.com/products/raspberry-pi-5/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Aura ROS integrates **real-time perception**, **flight telemetry**, and **onboard AI vision** into a cohesive system built for the Raspberry Pi 5 and Pixhawk-based drones. It is designed as a **modular ROS 2 ecosystem** capable of autonomous detection, communication, and coordination across complex mission profiles.

---

## System Overview

Aura ROS is the core software layer for the **Aura Drone Project**, a next-generation UAV platform for disaster response.

The **ROS 2 layer** acts as the central nervous system, managing the flow of data between the drone's hardware and software components:

* **Sensor Integration:** Handles image feeds and GPS data.
* **Perception:** Processes raw data using the AI module.
* **Decision Logic:** Relays processed information to the flight controller.
* **Telemetry:** Streams critical flight data back to a ground station.

---

## Core Features

| Feature | Component | Description |
| :--- | :--- | :--- |
| **Embedded Platform** | Raspberry Pi 5 | Runs **Ubuntu 24.04 (64-bit)** for powerful edge processing. |
| **Flight Control** | Pixhawk 2.4.8 | Communicates via **MAVLink** for reliable data exchange. |
| **AI Vision** | **YOLOv11n** | Custom-trained model for efficient human and object detection. |
| **Communication** | `aura_telemetry` | Interfaces with Pixhawk to stream GPS and system status. |
| **Modularity** | Independent Packages | Each functional block is an independent ROS 2 package for isolated testing. |

---

## System Prerequisites

Ensure your Raspberry Pi 5 is set up with the following:

* **Operating System:** Ubuntu 24.04 (64-bit)
* **ROS 2:** Jazzy Jalisco
* **Python:** Python 3.10+
* **Dependencies:** **OpenCV**, **cv\_bridge**, **ultralytics**, and **pymavlink** (for MAVLink communication).

---

## Installation Guide

### 1. Clone the Workspace
Create and navigate to your ROS 2 workspace directory.
```bash
mkdir -p ~/aura_ws/src
cd ~/aura_ws/src
git clone [https://github.com/adithyan846/aura.git](https://github.com/adithyan846/aura.git)
