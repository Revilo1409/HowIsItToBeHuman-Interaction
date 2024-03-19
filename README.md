# SARAI ChatGPT

## Overview

This package allows to send messages to ChatGPT via the Python API.

**Keywords:** ChatGPT, AI

### License

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS2)](https://docs.ros.org/en/humble/index.html) (middleware for robotics).
- [sarai_msgs](https://gitlab.kit.edu/kit/iar/sarai/software/ros2/sarai-standalone/sarai_msgs) for the custom ROS2 headers.

#### Building

1) Copy this package in your ROS2 workspace (e.g. `~/ros2_ws/src`).

2) Build the package with colcon:
    ```
    cd ~/ros2_ws
    colcon build --packages-select sarai_chatgpt
    ```

## IMPORTANT: Before Usage

Before being able to successfully communicate with the OpenAI API, you need to setup your API Key in the environment variables like this:

For Ubuntu:
```
$echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
```

## Usage

You can run the main node with:
```
ros2 run sarai_chatgpt gpt_requester_node
```

## Nodes

### gpt_requester_node

Allows to send messages to ChatGPT.

#### Services

* **`gpt_request`** ([sarai_msgs/GPTRequest](https://gitlab.kit.edu/kit/iar/sarai/software/ros2/sarai-standalone/sarai_msgs/-/blob/main/srv/GPTRequest.srv))
        
    Sends a request to ChatGPT and returns the response. 
    For example:
    ```
    ros2 service call /gpt_request sarai_msgs/GPTRequest "user_input: 'How are you GPT?'"
    ```

* **`get_gpt_request_params`** ([sarai_msgs/GetGPTRequestParams](TODO))

    Returns the currently used parameters in this Node (does not return the parameter api_key).
    For example:
    ```
    ros2 service call /get_gpt_request_params sarai_msgs/GetGPTRequestParams
    ```

#### Parameters

* **`api_key`** (string)

    The API key used for communicating with the OpenAI API.

* **`chatgpt_persona`** (string, default: '')

    The role given to ChatGPT by the user

    Example:
    ```
    You are a social robot with an actual robot body, who will have open conversations with humans on fundamental topics. You are leading the conversation and thus also ask questions. This also means if the conversation is going nowhere, you have to provide something new to the topic.You have some fundamental philosophical knowledge. Your responses have a maximum length of ca. 40 words.
    ```

* **`max_window_messages`** (int, default: 100)

    The maximum number of messages from the message history, that should be sent in the request to ChatGPT. It can be necessary to limit this, because the message history is sent to ChatGPT in every request to construct the next response. Limiting this can reduce the response time of the request (especially in longer conversations) and also limit the costs of using the API (you pay for every word you send in the request).

* **`temperature`** (float, default: 0.7, range: 0.0 - 1.0)

    Controls the degree of randomness or creativity in the generated response of ChatGPT by modifying the probabilities of activation functions. The higher the value, the more creative it gets.