# How is it to be human - Interaction

## Overview

This package allows to launch the robot-based discussion activity that uses ChatGPT.

**Keywords:**  human-robot interaction, AI, ChatGPT.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [pixelbot_msgs](https://gitlab.kit.edu/kit/iar/sarai/software/ros2/pixelbot/pixelbot_msgs) for the custom ROS2 headers.
- [sarai_msgs](https://gitlab.kit.edu/kit/iar/sarai/software/ros2/sarai-standalone/sarai_msgs) for the custom ROS2 headers.

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select how_is_it_to_be_human_interaction
    ```

## Usage

You can run the main node alongside the other thesis related nodes with:
```
ros2 launch how_is_it_to_be_human_interaction how_is_it_to_be_human_interaction.launch.py
```

## Nodes

### how_is_it_to_be_human_interaction_node

This node uses all the other dependent nodes and implements the main conversation flow activity aiming at providing a discussion between a robot and a human about some philosophical topics.
