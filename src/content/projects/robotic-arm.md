---
title: "6-DOF Robotic Arm with Visual Servoing"
description: "Designed and programmed a 6-degree-of-freedom robotic arm for pick-and-place tasks. Features real-time visual servoing, force control, and inverse kinematics solver."
tags:
  - "robotics"
  - "manipulation"
  - "computer-vision"
  - "C++"
  - "python"
github: "https://github.com/Sohum-Prime/robotic-arm"
status: "completed"
featured: true
order: 90
---

## Overview

A custom-built 6-DOF robotic arm designed for precision manipulation tasks in research and educational settings.

## Technical Details

- **Kinematics**: Custom inverse kinematics solver with singularity handling
- **Control**: PID control with gravity compensation
- **Vision**: Eye-in-hand camera configuration with ArUco marker tracking
- **Force sensing**: 6-axis force-torque sensor integration

## Hardware

- Dynamixel servo motors
- Intel RealSense D435 camera
- ATI Nano17 force-torque sensor
- Custom 3D-printed gripper

## Results

Achieved sub-millimeter positioning accuracy and 0.5 second cycle time for pick-and-place operations.
