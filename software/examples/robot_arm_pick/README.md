# Robot Arm Pick Sample

## Overview

This sample demonstrates how to use the Orbbec SDK to implement a robot arm pick and place application. The application displays the color image from the camera, allows the user to click on objects in the image, calculates the 3D coordinates of the clicked point, and sends these coordinates to a robot arm via serial communication to perform the pick action.

## Features

- Displays live color image from the Orbbec camera
- Enables mouse clicking on the color image to select points
- Calculates the 3D world coordinates of the clicked pixel using depth information
- Controls the robot arm via serial communication to perform pick actions

## Implementation Details

1. The application creates a pipeline to capture both color and depth streams from the camera.
2. It enables software alignment from depth to color to ensure coordinate consistency.
3. The color image is displayed in an OpenCV window.
4. When the user clicks on the image, the application:
   - Gets the pixel coordinates of the click
   - Retrieves the corresponding depth value at that pixel
   - Converts the 2D pixel coordinates plus depth into 3D world coordinates
   - Transforms the 3D coordinates to the robot coordinate system
   - Sends control commands to the robot arm via serial communication to perform the pick action

## Usage

1. Ensure the robot arm is connected to the computer via a USB-to-serial adapter (defaults to /dev/ttyUSB0)
2. Build and run the application
3. When the color image window appears, click on any object visible in the image
4. The console will display the 3D coordinates of the clicked point
5. The robot arm will perform a predefined pick action
6. Press ESC key to exit the application

## Customization

To integrate with an actual robot arm:

1. Configure `zyarm_sdk::ZyArm` for your robot arm serial port and firmware protocol
2. Adjust the coordinate transformation parameters according to the actual installation position:
   - ry_camera: Camera rotation angle around Y-axis (radians)
   - h_camera_base: Vertical height from camera to base (mm)
   - l_base_robot: Horizontal distance from base to robot arm base (mm)
3. Modify the robot action sequence in the robotArmPick function as needed

## Coordinate System

### Camera Coordinate System
The 3D coordinates are provided in the camera's coordinate system:
- X: Horizontal axis (positive to the right)
- Y: Vertical axis (positive downward)
- Z: Depth axis (positive forward from the camera)

### Robot Coordinate System Transformation
The code implements coordinate system transformation through the transformPointCloudToRobotBase function:
1. First convert point cloud coordinates to camera coordinates
2. Apply rotation transformation (around Y-axis)
3. Finally apply translation transformation

Units are in millimeters.
