# ROS Project

## Overview

A Robotics project that involves functionalities utilizing ROS framework. The proposed package first converts vehicle and obstacle GPS coordinates to ENU format, publishing them as TF and Odometry messages. Next, a service filters these messages and computes the distance between objects. Finally, a safety assessment is performed by publishing a custom status message based on the calculated distance.

## Features

- Conversion of GPS coordinates to ENU coordinates.

- Publication of TF and Odometry messages.

- Custom service for distance calculation.

- Custom status messages for safety checks.

- Configurable via ROS parameters.
