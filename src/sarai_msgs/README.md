# SARAI ROS2 Headers

## Overview

This package contains the custom message and service headers of the SARAI Lab . They mediate the communication between SARAI's ROS packages. 

### License

## Installation

### Building from source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws/src
    colcon build --packages-select sarai_msgs
