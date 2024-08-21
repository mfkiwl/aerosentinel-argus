<p align="center">
  <img src="https://github.com/user-attachments/assets/a31a767f-9484-4454-ac91-99be2e87a4a0" alt="aerosentinel logo">
</p>

#

Welcome to Aerosentinel Argus Navigation Module firmware repository. Configured & developed using STM32CubeIDE and written in C.

## Table of Contents
1. [Introduction](#introduction)
2. [Getting Started](#getting-started)
    - [Prerequisites](#prerequisites)
    - [Installation](#installation)
3. [Features](#features)
4. [Software Architecture](#software-architecture)
5. [Current Project Status](#current-project-status)
6. [Rev 2 Module update](#rev-2-module-update)
7. [Contributing](#contributing)
8. [License](#license)

## Introduction
This firmware is designed to provide precise navigation and guidance capabilities for rocketry applications using the Aerosentinel Argus Navigation Module. It implements advanced algorithms and features to ensure optimal performance, safety, and reliability during launch, ascent, and recovery phases.

## Getting Started

### Prerequisites
Before getting started, make sure you have the following installed:
- STM32CubeIDE (version 1.15.1 or higher)

### Installation
1. Clone this repository to your local machine.
2. Open STM32CubeIDE.
3. Import the cloned repository to your workspace
4. You're all set!

## Features
- **Comprehensive Telemetry**: Provides real-time data on altitude, velocity, acceleration, positionning and environmental conditions.
- **Autonomous Operation**: Automatically gathers sensors & GPS data, execute extended kalman filter fusion algorithms in cascade, and send the results via UART to the Flight Computer. 
- **Reliability**: Built to withstand harsh temperatures, vibrations, and G-forces for robust performance.

## Software Architecture

The Aerosentinel Argus Navigation Module is designed with a modular and hierarchical software architecture, ensuring clarity and maintainability. Below is the visual representation of the software architecture:

<p align="center">
  <img src="https://github.com/user-attachments/assets/2bdded6f-abc9-4551-b408-89c27daf86a9" alt="Aerosentinel Argus Software Architecture">
</p>

This architecture comprises several key components:

- **Peripherals Drivers**: Interfaces for various sensors such as IMUs, barometers, and magnetometers.
- **Peripherals Data Management**: Manages data from peripherals, including sensor fusion using Cascade Extended Kalman Filtering (EKF).
- **Module Control Center**: Handles states, commands, error management, data management, and communication interfaces.
- [**Aerosentinel Relay Protocol (A.R.P.)**](https://github.com/yaxsomo/aerosentinel-relay-protocol): Ensures secure and reliable communication using data encryption, authentication mechanisms, and a robust protocol stack.

## Current Project Status
<div align="center">
    <img src="current_status.svg" width="100%" heigth="auto" alt="css-in-readme">
</div>

**Initial Testing** (Started : 16-07-2024) :
- **(16-07-2024)** Argus boards received, starting testing phase. I'm ensuring that all components power up correctly and communicate as expected.
- **(18-07-2024)** Eveything seems to **power up correctly**, without power issues.
- **(21-07-2024)** After testing the communication with pretty much all the peripherals (except for the GPS and SDMMC), I found some **issues on the electronics conception** :
  - **ASM330LHH not communicating** -> Root cause : The device is supposed to communicate with I2C, but the CS pin (aka Protocol Selection pin) has been unfortunately setted to SPI mode (GND -> SPI Mode | VDD -> I2C Mode).
  - **LIS2MDLTR not communicating** -> Root cause : The device is supposed to communicate with I2C, but the CS pin (aka Protocol Selection pin) has been unfortunately setted to SPI mode (GND -> SPI Mode | VDD -> I2C Mode).
The issue with those sensors is already **being corrected for the Rev 2 module** (See the [Rev 2 Module update](#rev-2-module-update) section). 
Besides that, everything else seems to work perfectly fine, and the work of the two faulty sensors has been replaced by the functionning ones.

TODO-> Add advancement notes for the GPS and SDMMC

**Basic Functionnality Verification** (Started : 22-07-2024) :
- **(22-07-2024)** Started to write drivers for the functionning devices.
- **(23-07-2024)** BMI323, BNO055, BME680 and MS5607 are working and communicating properly, with pretty good readings (very little noise to solve).
- **(25-07-2024)** GPS Module responding correctly to UART commands. For now i don't have the antenna, so I cannot gather readings from satellites.

**Firmware Development**
- **(29-07-2024)** Added scaling factor division for quaternion normalization, and tested graphically with [Aerosentinel Sensor Fusion Transformer](https://github.com/yaxsomo/aerosentinel-sensor-fusion-transformer). Impressive results!! Preparing the module for the first flight test, in a few days.

## Rev 2 Module update
As some hardware issues have been found on the Rev.1 module, I'm actively improving this second revision.

**Improvements:**

- **ASM330LHH & LIS2MDLTR** : Solved protocol selection on CS pins. They should be operational for the next hardware iteration.
- **GPS Antenna Connector** : Changing the GPS antenna connector to a female one
- **Light Emitting Diodes** : Adding a health status LED and a second LED for additional informations/alerts.
- **Multiprocessor Communication Connector** : Adding a Board Reset pin and Module Heartbeat pin
- **Black Box SD Card connector** : Rotated the SD Card connector to 180°

## Contributing
Contributions are welcome! Please follow the [contribution guidelines](CONTRIBUTING.md) when making contributions to this project.

## License
This project is licensed under the [BSD 3-Clause License](LICENSE).
