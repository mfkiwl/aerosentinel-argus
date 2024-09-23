<p align="center">
  <img src="https://github.com/user-attachments/assets/dd768899-18c6-43a8-8959-26c94cd3e7ab" alt="aerosentinel logo">
</p>

#

Welcome to Aerosentinel Argus Navigation Module firmware repository. Configured & developed using STM32CubeIDE and written in C.

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [Software Architecture](#software-architecture)
4. [Current Project Status](#current-project-status)
5. [Contributing](#contributing)
6. [License](#license)

## Introduction
This firmware is designed to provide precise navigation and guidance capabilities for rocketry applications using the Aerosentinel Argus Navigation Module. It implements advanced algorithms and features to ensure optimal performance, safety, and reliability during launch, ascent, and recovery phases.

## Features
- **Comprehensive Telemetry**: Provides real-time data on altitude, velocity, acceleration, positionning and environmental conditions.
- **Autonomous Operation**: Automatically gathers sensors & GPS data, execute extended kalman filter fusion algorithms in cascade, and send the results via UART to the Flight Computer. 
- **Reliability**: Built to withstand harsh temperatures, vibrations, and G-forces for robust performance.

## Software Architecture

Argus is designed with a modular and hierarchical software architecture, ensuring clarity and maintainability. Below is the visual representation of the software architecture:

<p align="center">
  <img src="https://github.com/user-attachments/assets/f0f63ed4-793e-4ccc-898c-4125032018be" alt="Aerosentinel Argus Software Architecture">
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

**Initial Testing** : Waiting for Argus V2 module to be delivered.

## Contributing
Contributions are welcome! Please follow the [contribution guidelines](CONTRIBUTING.md) when making contributions to this project.

## License
This project is licensed under the [BSD 3-Clause License](LICENSE).
