# Source code

## Raspberry Pi setup

To setup the Raspberry Pi, you will need a microSD card (32 GB recommended) and an SD card reader. To flash the operating system on the microSD card, the use of [Raspberry Pi Imager](https://www.raspberrypi.com/software/) is advised.

### Installation

On Raspberry Pi imager, select the *Other general-purpose OS (other)* option. Then choose *Ubuntu* and select *Ubuntu Desktop 22.04.2 LTS (64-bit)* and write it on your microSD card. After the writing operation, you can plug the microSD card on your Raspberry Pi and turn it on. 

After the setup process of Ubuntu, you will have to download the dependencies required to run the activity. To do so, you will first have to download the [PixelBot repository](https://github.com/RomainMaure/PixelBot) eitheir using git or by downloading the zip version of the repository. It is important that the PixelBot folder is located in your *home* directory. You can then install the required dependencies by using the script [install.sh](https://github.com/RomainMaure/PixelBot/blob/main/src/install.sh). To do so, open a terminal in the PixelBot/src/ folder and run:

```
chmod +x install.sh
./install.sh
sudo reboot
```

### Possible additional steps

- You might have to rotate the screen on the LCD display, to do so, you can open a terminal and run one of the following commands:
    ```
    # To rotate 90 degrees on the right
    DISPLAY=:0 xrandr --output HDMI-1 --rotate right

    # To rotate 90 degrees on the left
    DISPLAY=:0 xrandr --output HDMI-1 --rotate left

    # To rotate 180 degrees
    DISPLAY=:0 xrandr --output HDMI-1 --rotate inverted

    # To reset the rotation back to normal
    DISPLAY=:0 xrandr --output HDMI-1 --rotate normal
    ```
    For more documentation, check the following [link](https://linuxhint.com/rotate-screen-in-raspberry-pi/).

## ROS2 packages

- **[pixelbot_display](https://github.com/Revilo1409/HowIsItToBeHuman-Interaction/tree/main/src/pixelbot_display)**: Allows the robot to perform facial expressions using the LCD.
- **[pixelbot_motors](https://github.com/Revilo1409/HowIsItToBeHuman-Interaction/tree/main/src/pixelbot_motors)**: Allows to control the motors of the robot.
- **[pixelbot_msgs](https://github.com/Revilo1409/HowIsItToBeHuman-Interaction/tree/main/src/pixelbot_msgs)**: Custom message and service headers used by the packages of PixelBot.
- **[sarai_chatgpt](https://github.com/Revilo1409/HowIsItToBeHuman-Interaction/tree/main/src/sarai_chatgpt)**: Allows to communicate with OpenAI's GPT-3.5.
- **[sarai_msgs](https://github.com/Revilo1409/HowIsItToBeHuman-Interaction/tree/main/src/sarai_msgs)**: Custom message and service headers used by the packages of SARAI.

<img src="/imgs/Nodes.png">

## Build

In case you used the `install.sh` script, all the packages should already be built and you can skip this step. Otherwise, follow the instructions described below:

1) Copy all the packages in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the packages with colcon:
```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Important: Before running
Before being able to successfully communicate with the OpenAI API, you need to setup your API Key in the environment variables like this:

For Ubuntu:
```
$echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
```

## Run

To run the robot-based learning activity aiming at raising awareness of gender inequality among children, run:
```
ros2 launch how_is_it_to_be_human_interaction how_is_it_to_be_human_interaction.launch.py
```

You can also use the packages available to create your own activity with the robot.