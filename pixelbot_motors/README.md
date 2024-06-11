# PixelBot Motors

## Overview

This package allows to control the motors of PixelBot. 

**Keywords:** motors, gestures

### License

[![License: GPL-3.0](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)

The whole package is under GPL-3.0 License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/main/LICENSE).

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure21@gmail.com**

The [pixelbot_motors](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_motors) package has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [pixelbot_msgs](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_msgs) for the custom ROS2 headers.
- [adafruit_servokit](https://pypi.org/project/adafruit-circuitpython-servokit/) to control multiple servo motors through a PCA9865.
    ```
	sudo pip3 install adafruit-circuitpython-servokit
    ```

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_motors
    ```

## Usage

You can run the main node with:
```
ros2 run pixelbot_motors pixelbot_motors_node
```

## Nodes

### pixelbot_motors_node

Allows to control the motors of PixelBot.

#### Services

* **`motors_movement`** ([pixelbot_msgs/MotorsMovement](https://github.com/RomainMaure/PixelBot/blob/main/src/pixelbot_msgs/srv/MotorsMovement.srv))

	Move the motors of your choice to a given angle. For example:
    ```
    # Move the right arm motor to an angle position of 180 degrees
	ros2 service call /motors_movement pixelbot_msgs/MotorsMovement "body_parts: ['right_arm']
    angles: [180]"
    ```
    ```
    # Move quickly all the motors to a given angle position
	ros2 service call /motors_movement pixelbot_msgs/MotorsMovement "body_parts: ['right_arm', 'left_arm', 'right_antenna', 'left_antenna']
    angles: [180, 0, 90, 135]
    ```

* **`walking_movement`** ([std_srvs/Empty](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html))

	Perform a walking gesture with the arms. For example:
    ```
	ros2 service call /walking_movement std_srvs/srv/Empty 
    ```

* **`hand_waving`** ([std_srvs/Empty](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html))

	Perform a hand waving gesture with the right arm. For example:
    ```
	ros2 service call /hand_waving std_srvs/srv/Empty 
    ```

* **`emotion_antennae_movement`** ([pixelbot_msgs/DisplayEmotion](https://github.com/RomainMaure/PixelBot/blob/main/src/pixelbot_msgs/srv/DisplayEmotion.srv))

	Perform antennae movements to show emotions. For example:
    ```
	ros2 service call /emotion_antennae_movement pixelbot_msgs/DisplayEmotion "desired_emotion: 'happy'"
    ```

    Available emotions: "happy", "angry", "sad", "surprise".

## Troubleshooting

- **i2c permission error**:
    ```
    PermissionError: [Errno 13] Permission denied: '/dev/i2c-1'
    ```

    A quick fix is to run in a terminal:
    ```
    sudo chmod a+rw /dev/i2c-1
    ```

    However this is temporary and will be lost at next boot. To fix it permanently, you can run this command in a terminal and then reboot:
    ```
    sudo adduser $USER dialout
    ```


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).
