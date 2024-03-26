# SARAI Speech Recognition

## Overview

This package allows to recognize speech.

**Keywords:** speech recognition, audio

### License

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [sarai_msgs](https://gitlab.kit.edu/kit/iar/sarai/software/ros2/sarai-standalone/sarai_msgs) for the custom ROS2 headers.
- [SpeechRecognition](https://pypi.org/project/SpeechRecognition/) Python library for the speech recognition.
    ```
    sudo pip3 install SpeechRecognition
    ```

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select sarai_speech_recognition
    ```

## Usage

You can run the main node with:
```
ros2 run sarai_speech_recognition sarai_speech_recognition_node
```

## Nodes

### sarai_speech_recognition_node

Allows to run the speech recognition.

#### Services

* **`recognize_speech`** ([sarai_msgs/GPTRequest](https://gitlab.kit.edu/kit/iar/sarai/software/ros2/sarai-standalone/sarai_msgs/-/blob/main/srv/GPTRequest.srv))
        
    Starts trying to recognize speech. If successful, returns the spoken text as a String. For example:
    ```
    ros2 service call /recognize_speech sarai_msgs/RecognizeSpeech
    ```

