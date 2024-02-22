# Sarai Text to Speech - Playsound

## Overview

This package allows any robot to speak. 

**Keywords:**  human-robot interaction, audio communication, voice synthesis

### License

[![License: GPL-3.0](https://img.shields.io/badge/license-GPLv3-blue)](https://www.gnu.org/licenses/gpl-3.0.en.html)

The whole package is under GPL-3.0 License, see [LICENSE](https://github.com/RomainMaure/PixelBot/blob/main/LICENSE).

**Author: Romain Maure<br />
Affiliation: [CHILI Lab, EPFL](https://www.epfl.ch/labs/chili/)<br />
Maintainer: Romain Maure, romain.maure21@gmail.com**

The [pixelbot_audio](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_audio) package has been tested under [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [sarai_msgs](https://github.com/RomainMaure/PixelBot/tree/main/src/pixelbot_msgs) for the custom ROS2 headers.
- [Pyttsx3](https://pypi.org/project/pyttsx3/) and [gTTS](https://pypi.org/project/gTTS/): two text to speech libraries for Python.
    ```
	sudo pip3 install pyttsx3
    sudo pip3 install gTTS
    ```    
- [Playsound](https://pypi.org/project/playsound/), [Pydub](https://pypi.org/project/pydub/), [PyAudio](https://pypi.org/project/PyAudio/), [FFmpeg](https://ffmpeg.org/), [eSpeak](https://doc.ubuntu-fr.org/espeak) and [Flac](https://doc.ubuntu-fr.org/flac): a set of audio related libraries.
    ```
    sudo pip3 install playsound
    sudo pip3 install pydub
    sudo apt install python3-pyaudio
    sudo apt install ffmpeg
    sudo apt install espeak
    sudo apt-get install flac
    ```

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select pixelbot_audio
    ```

## Usage

You can run the main node with:
```
ros2 run pixelbot_audio pixelbot_audio_node
```

## Nodes

### pixelbot_audio_node

Allows PixelBot to speak.

#### Services

* **`speak`** ([sarai_msgs/SetSpeech](https://github.com/RomainMaure/PixelBot/blob/main/src/pixelbot_msgs/srv/SetSpeech.srv))

	Make PixelBot speak:
    ```
	ros2 service call /speak sarai_msgs/SetSpeech "message: 'Bonjour, je suis PixelBot.'"
    ```

* **`change_voice_alteration`** ([sarai_msgs/SetVoiceAlteration](https://github.com/RomainMaure/PixelBot/blob/main/src/pixelbot_msgs/srv/SetVoiceAlteration.srv))

	Allows to enable or disable voice alteration (robotic like voice):
    ```
    # Enable voice alteration
	ros2 service call /change_voice_alteration sarai_msgs/SetVoiceAlteration "is_voice_altered: true"
    ```
    ```
    # Disable voice alteration
	ros2 service call /change_voice_alteration sarai_msgs/SetVoiceAlteration "is_voice_altered: false"
    ```

* **`change_language`** ([sarai_msgs/SetLanguage](https://github.com/RomainMaure/PixelBot/blob/main/src/pixelbot_msgs/srv/SetLanguage.srv))

	Allows to specify the language to be spoken (french or english):
    ```
    # Change to english
	ros2 service call /change_language sarai_msgs/SetLanguage "language: 'english'"
    ```
    ```
    # Change to french
	ros2 service call /change_language sarai_msgs/SetLanguage "language: 'french'"
    ```

* **`play_happy_sound`** ([std_srvs/Empty](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html))

	Allows PixelBot to play a happy sound:
    ```
	ros2 service call /play_happy_sound std_srvs/srv/Empty
    ```

## Troubleshooting

- [gTTS](https://pypi.org/project/gTTS/) is a Python library and CLI tool to interface with Google Translate's text-to-speech API. Internet connectivity is thus required to make it work, as opposed to [Pyttsx3](https://pypi.org/project/pyttsx3/) which can work offline.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/RomainMaure/PixelBot/issues).
