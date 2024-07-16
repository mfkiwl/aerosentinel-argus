<p align="center">
  <img src="https://github.com/yaxsomo/aerosentinel-argus/assets/71334330/143f75be-1a29-449a-a178-e01ef7d16065" alt="aerosentinel logo">
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
5. [Contributing](#contributing)
6. [License](#license)

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

## Current Status

Test Progress Bar

<div align="center">
    <img src="current_status.svg" width="100%" heigth="auto" alt="css-in-readme">
</div>


## Contributing
Contributions are welcome! Please follow the [contribution guidelines](CONTRIBUTING.md) when making contributions to this project.

## License
This project is licensed under the [BSD 3-Clause License](LICENSE).
