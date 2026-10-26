# Moppy 2.0 - Autonomous Floor-Cleaning Robot

Moppy 2.0 is an autonomous floor-cleaning robot utilizing a dual-architecture control system. High-level processing, telemetry, and ROS 2 mapping are managed by a Raspberry Pi 4B, while an STM32F407 microcontroller runs FreeRTOS to handle real-time sensor polling and motor actuation.

## System Architecture

The repository is divided into two primary subsystems:

* **Moppy 2.0 - R Pi 4B**: Contains the Python-based high-level control scripts, MQTT bridges, and database logging nodes.
* **Moppy 2.0 - STM32**: Contains the C/C++ firmware, hardware abstraction layers, and FreeRTOS configurations for the STM32 microcontroller.

## Hardware & Component Drivers

The STM32 firmware is modularized into distinct component drivers to manage the robot's physical operations:

* **IMU Integration**: Dedicated drivers for the HMC5883L magnetometer and MPU6050 accelerometer/gyroscope, managed via an I2C recovery middleware.
* **Motor Control**: Actuation modules for the vacuum blower, main chassis wheels (TB6612FNG), roller brush, and side brushes.
* **LiDAR Actuation**: UART communication and PID control specifically for the LiDAR motor.
* **User Interface**: I2C OLED display drivers (`oled.c`, `oledGUI.c`) and custom bitmap fonts for on-device status monitoring.
* **System & Telemetry**: Push-button inputs, general system parameters, and telemetry data handling.

## Software Stack

* **Raspberry Pi**: Executes `moppy_lidar_node.py` for spatial data and relies on `mqtt_bridge.py` and `mqtt_to_mysql.py` for message brokering and database logging. Startup operations are automated via `start_moppy.sh`.
* **STM32 Firmware**: Built on FreeRTOS for task scheduling, utilizing the STM32 HAL (Hardware Abstraction Layer) and CMSIS core drivers. Hardware pinouts and clock configurations are mapped in `Moppy2.0.ioc` and the `STM32F407 PIN CONFIG MAP.xlsx` reference sheet.

## Getting Started

1. **Raspberry Pi Setup**: Review the `R Pi 4B GPIO.png` and the provided datasheet in the `Documents` folder for hardware wiring references. Execute `start_moppy.sh` to initialize the MQTT bridge and LiDAR nodes.
2. **STM32 Build**: Open the `.project` or `.cproject` files in your STM32 IDE (e.g., STM32CubeIDE). Ensure FreeRTOS and all middleware components are properly linked before flashing the firmware to the `STM32F407VETX` target using the provided linker scripts.