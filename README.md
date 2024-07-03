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
3. [Usage](#usage)
    - [Building the Firmware](#building-the-firmware)
5. [Features](#features)
6. [Contributing](#contributing)
7. [License](#license)

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

## Usage

### Building the Firmware
After modifying the code, you'll have to build it! To do so, safe the modified files and click on this Build icon :

<img width="542" alt="Builing_STM32CubeIDE" src="https://github.com/yaxsomo/Aerosentinel-Mach-10/assets/71334330/49d36bb4-e72e-46e0-b066-6b889667e3e0">

To make sure the build process has been successful, you can check the console for errors or warnings :

<img width="542" alt="Build_Result_STM32CubeIDE" src="https://github.com/yaxsomo/Aerosentinel-Mach-10/assets/71334330/89327704-94d9-4f79-b6a8-58f2b967f4f4">



## Features
- **Comprehensive Telemetry**: Provides real-time data on altitude, velocity, acceleration, positionning and environmental conditions.
- **Autonomous Operation**: Automatically gathers sensors & GPS data, execute extended kalman filter fusion algorithms in cascade, and send the results via UART to the Flight Computer. 
- **Reliability**: Built to withstand harsh temperatures, vibrations, and G-forces for robust performance.

## Contributing
Contributions are welcome! Please follow the [contribution guidelines](CONTRIBUTING.md) when making contributions to this project.

## License
This project is licensed under the [BSD 3-Clause License](LICENSE).
