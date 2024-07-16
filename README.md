<p align="center">
  <img src="https://github.com/yaxsomo/aerosentinel-argus/assets/71334330/143f75be-1a29-449a-a178-e01ef7d16065" alt="aerosentinel logo">
</p>

<style>
  @import url("https://fonts.googleapis.com/css?family=Montserrat:400,700");

* {
	box-sizing: border-box;
}

body {
	--h: 212deg;
	--l: 43%;
	--brandColor: hsl(var(--h), 71%, var(--l));
	font-family: Montserrat, sans-serif;
	margin: 0;
	background-color: whitesmoke;
}

p {
	margin: 0;
	line-height: 1.6;
}

ol {
	list-style: none;
	counter-reset: list;
	padding: 0 1rem;
}

li {
	--stop: calc(100% / var(--length) * var(--i));
	--l: 62%;
	--l2: 88%;
	--h: calc((var(--i) - 1) * (180 / var(--length)));
	--c1: hsl(var(--h), 71%, var(--l));
	--c2: hsl(var(--h), 71%, var(--l2));
	
	position: relative;
	counter-increment: list;
	max-width: 45rem;
	margin: 2rem auto;
	padding: 2rem 1rem 1rem;
	box-shadow: 0.1rem 0.1rem 1.5rem rgba(0, 0, 0, 0.3);
	border-radius: 0.25rem;
	overflow: hidden;
	background-color: white;
}

li::before {
	content: '';
	display: block;
	width: 100%;
	height: 1rem;
	position: absolute;
	top: 0;
	left: 0;
	background: linear-gradient(to right, var(--c1) var(--stop), var(--c2) var(--stop));
}

h3 {
	display: flex;
	align-items: baseline;
	margin: 0 0 1rem;
	color: rgb(70 70 70);
}

h3::before {
	display: flex;
	justify-content: center;
	align-items: center;
	flex: 0 0 auto;
	margin-right: 1rem;
	width: 3rem;
	height: 3rem;
	content: counter(list);
	padding: 1rem;
	border-radius: 50%;
	background-color: var(--c1);
	color: white;
}

@media (min-width: 40em) {
	li {
		margin: 3rem auto;
		padding: 3rem 2rem 2rem;
	}
	
	h3 {
		font-size: 2.25rem;
		margin: 0 0 2rem;
	}
	
	h3::before {
		margin-right: 1.5rem;
	}
}
</style>

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




## Contributing
Contributions are welcome! Please follow the [contribution guidelines](CONTRIBUTING.md) when making contributions to this project.

## License
This project is licensed under the [BSD 3-Clause License](LICENSE).
