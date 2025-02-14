# Inverse Kinematics Solver with SFML

This project demonstrates a simple implementation of an Inverse Kinematics (IK) solver using the SFML (Simple and Fast Multimedia Library) for rendering. The IK solver is used to control a chain of linked segments (often referred to as a "robot arm" or "kinematic chain") to reach a target position in a 2D space.

![Inverse Kinematics Preview](https://github.com/lazycodebaker/Forward-Kinematics/blob/main/inverse_kinematics.gif)

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [How It Works](#how-it-works)
- [License](#license)

## Introduction

Inverse Kinematics is a technique used in robotics, animation, and game development to determine the joint parameters (e.g., angles) that will cause a chain of linked segments to reach a desired position. This project implements a simple IK solver using the **Cyclic Coordinate Descent (CCD)** algorithm, which iteratively adjusts the angles of each joint to minimize the distance between the end effector (the end of the chain) and the target position.

The project uses **SFML** for rendering the kinematic chain and the target position. The user can click anywhere in the window to set a new target, and the IK solver will adjust the chain to reach the target.

## Features

- **Interactive Target Setting**: Click anywhere in the window to set a new target position.
- **Real-Time IK Solver**: The IK solver updates the chain's angles in real-time to reach the target.
- **Visualization**: The kinematic chain and target are visualized using SFML.
- **Configurable Parameters**: The number of joints, link length, and solver speed can be adjusted in the code.

## Dependencies

- **SFML**: The Simple and Fast Multimedia Library is used for rendering and window management.
  - [SFML GitHub](https://github.com/SFML/SFML)
  - [SFML Website](https://www.sfml-dev.org/)
- **Eigen**: A C++ template library for linear algebra, used for vector and matrix operations.
  - [Eigen Website](https://eigen.tuxfamily.org/)

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/lazycodebaker/Forward-Kinematics
   cd Forward-Kinematics
   ```

2. **Install Dependencies**:
   - Install SFML:
     - On Ubuntu:
       ```bash
       sudo apt-get install libsfml-dev
       ```
     - On macOS (using Homebrew):
       ```bash
       brew install sfml
       ```
     - On Windows: Download SFML from the [official website](https://www.sfml-dev.org/download.php) and link it to your project.
   - Install Eigen:
     - On Ubuntu:
       ```bash
       sudo apt-get install libeigen3-dev
       ```
     - On macOS (using Homebrew):
       ```bash
       brew install eigen
       ```
     - On Windows: Download Eigen from the [official website](https://eigen.tuxfamily.org/dox/GettingStarted.html) and include it in your project.

3. **Build the Project**:
   - Use CMake or your preferred build system to compile the project.
   - Example using CMake:
     ```bash
     mkdir build
     cd build
     cmake ..
     make
     ```

4. **Run the Executable**:
   - After building, run the executable:
     ```bash
     ./inverse_kinematics_solver
     ```

## Usage

- **Set Target**: Click anywhere in the window to set a new target position. The kinematic chain will adjust to reach the target.
- **Close Window**: Press the close button or `Alt+F4` to exit the application.

## How It Works

### Inverse Kinematics Solver
The IK solver uses the **Cyclic Coordinate Descent (CCD)** algorithm:
1. **Forward Kinematics**: The positions of each joint are calculated based on the current angles.
2. **Error Calculation**: The solver calculates the angle needed to rotate each joint to reduce the distance between the end effector and the target.
3. **Angle Adjustment**: The angles of each joint are adjusted iteratively until the end effector is close enough to the target (within a specified tolerance).

### Rendering
- The kinematic chain is drawn using SFML's `sf::Vertex` and `sf::PrimitiveType::Lines`.
- The target is represented as a red circle.

### Code Structure
- **InverseKinematicsSolver Class**: Handles the IK calculations and stores the state of the kinematic chain.
- **Main Loop**: Handles user input, updates the IK solver, and renders the scene.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

Feel free to contribute to this project by opening issues or submitting pull requests. Enjoy experimenting with inverse kinematics!
