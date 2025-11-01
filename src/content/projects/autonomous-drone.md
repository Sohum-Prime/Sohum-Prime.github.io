---
title: "Autonomous Drone Navigation System"
description: "Built a fully autonomous drone capable of navigating complex indoor environments using SLAM and deep learning-based obstacle avoidance. Integrates ROS2, PX4 autopilot, and custom vision algorithms."
tags:
  - "robotics"
  - "computer-vision"
  - "autonomous-systems"
  - "ROS2"
  - "python"
github: "https://github.com/Sohum-Prime/autonomous-drone"
demo: "https://www.youtube.com/watch?v=example"
status: "completed"
featured: true
order: 100
---

## Project Overview

This project implements a complete autonomous navigation system for quadrotor drones in GPS-denied environments. The system uses a combination of visual-inertial odometry, 3D mapping, and learned control policies.

## Key Features

- **Real-time SLAM**: Visual-inertial SLAM running at 30Hz
- **Obstacle avoidance**: Deep learning model for dynamic obstacle detection
- **Path planning**: A* planner with smooth trajectory generation
- **Hardware integration**: Custom-built quadrotor with Jetson Xavier NX

## Technologies Used

- ROS2 (Humble)
- PX4 Autopilot
- OpenCV
- PyTorch
- Gazebo simulation

## Achievements

Successfully demonstrated autonomous flight in cluttered indoor environments with 95% success rate over 100+ test flights.
