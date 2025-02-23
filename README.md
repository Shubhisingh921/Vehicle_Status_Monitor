---

# Vehicle Status Monitor

## Overview

The **Vehicle Status Monitor** is a ROS 2 node designed to monitor the status of **LaserScan** and **IMU** sensors in real-time. It checks whether these sensors are operational and publishes their statuses accordingly.

## Features

- **Subscribes to LaserScan (`/scan`) and IMU (`/imu/data_raw`) topics.**
- **Monitors sensor data reception and detects malfunctions.**
- **Publishes a combined status message to the `multi_sensor_status` topic.**
- **Logs operational and malfunction states of both sensors.**
- **Provides a GUI for visualizing sensor status.**
- **Implements automatic reconnection for malfunctioning sensors.**
- **Enables data logging to a file for debugging.**

## Dependencies

Ensure you have **ROS 2** installed. This node is built using **rclpy**, and it requires the following packages:

```bash
sudo apt install ros-humble-rclpy ros-humble-sensor-msgs ros-humble-std-msgs
```

> Replace `humble` with your ROS 2 distribution if using a different version.

## Installation

Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

Run the Vehicle Status Monitor node:

```bash
ros2 run <your_package_name> monitor
```

## Node Details

### **Published Topics**
- **`multi_sensor_status` (`std_msgs/String`)**  
  - Publishes a status message with the operational state of both sensors.

### **Subscribed Topics**
- **`/scan` (`sensor_msgs/LaserScan`)**  
  - Receives LaserScan data to determine if the sensor is operational.
- **`/imu/data_raw` (`sensor_msgs/Imu`)**  
  - Receives IMU data to determine if the sensor is operational.

## How It Works

1. The node subscribes to **LaserScan** and **IMU** topics.
2. If data is received within **2 seconds**, the sensor is marked **operational**.
3. If no data is received for over **2 seconds**, the sensor is marked **malfunction**.
4. The status is published in the `multi_sensor_status` topic every **1 second**.
5. The GUI provides a visual representation of sensor status.
6. Automatic reconnection ensures malfunctioning sensors resume operation when available.
7. Data is logged to a file for debugging and analysis.

### Example Published Status Message:
```
LaserScan: Operational
IMU: Malfunction
```

## Stopping the Node

Use **Ctrl + C** to stop execution.

---

