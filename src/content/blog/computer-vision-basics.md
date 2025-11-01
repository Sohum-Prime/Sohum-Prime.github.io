---
title: "Computer Vision Basics for Robotics"
description: "Essential computer vision techniques every robotics engineer should know, with practical examples using OpenCV and Python."
publishDate: 2024-07-18
tags:
  - "computer-vision"
  - "robotics"
  - "opencv"
  - "python"
draft: false
featured: false
---

## Introduction

Computer vision is fundamental to modern robotics. This post covers essential CV techniques with practical code examples.

## Core Concepts

### Image Processing Pipeline

1. **Acquisition**: Capture image from camera
2. **Preprocessing**: Denoise, normalize
3. **Feature Extraction**: Identify regions of interest
4. **Analysis**: Make decisions based on features

### Essential Techniques

#### 1. Color Filtering

```python
import cv2
import numpy as np

# Convert to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define color range (e.g., red)
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

# Create mask
mask = cv2.inRange(hsv, lower_red, upper_red)
```

#### 2. Edge Detection

```python
# Canny edge detection
edges = cv2.Canny(gray_image, threshold1=50, threshold2=150)
```

#### 3. Object Detection

Using contour detection:

```python
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    area = cv2.contourArea(contour)
    if area > 1000:  # Filter small contours
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
```

## Practical Applications

### Visual Servoing

Track object and move robot accordingly:

```python
def calculate_control(object_center, image_center):
    error_x = object_center[0] - image_center[0]
    error_y = object_center[1] - image_center[1]
    
    # Simple proportional control
    cmd_vel_x = -Kp * error_y
    cmd_vel_angular = -Kp * error_x
    
    return cmd_vel_x, cmd_vel_angular
```

## Common Pitfalls

1. **Lighting variations**: Use adaptive thresholding
2. **Motion blur**: Increase shutter speed or use deblurring
3. **Perspective distortion**: Camera calibration is essential

## Next Steps

- Learn about deep learning for vision (YOLO, SegFormer)
- Explore 3D vision (stereo, depth cameras)
- Study SLAM algorithms

## Resources

- OpenCV documentation
- ROS vision tutorials
- My computer vision examples on GitHub
