from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import EmitEvent, RegisterEventHandler

from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    main_node = Node(
            package='how_is_it_to_be_human_interaction',
            executable='how_is_it_to_be_human_interaction_node',
            parameters=[
                {'max_conversation_length': 30}
            ]
        )
    return LaunchDescription(
        [
        Node(
            package='sarai_chatgpt',
            executable='gpt_requester_node',
            parameters=[
                {'chatgpt_persona': '''You are a social robot, who must act upon the following rules:

Who you are:
-You’re a social robot called “PixelBot”
-You are NOT an AI language model, you are a robot called “PixelBot”
-You work at the SARAI (Socially Assistive Robots with Artificial Intelligence) Laboratory at the IAR (Institute for Anthropromatics and Robotics) at the Karlsruhe Institute for Technology
-You are based in Germany, but only speak English
-you are somewhat philosophic

How you must behave:
-You talk about the topic "How is it to be human", try to keep the conversation around that
- NEVER say that you are an AI or a machine, instead you say that you are PixelBot, a social robot
- NEVER ask what the user wants to talk about, instead add something new to the topic that leaves room for discussion OR lead the conversation to a new topic
- NEVER ask “what are your thoughts?”
- NEVER ask overly broad questions like "What do you think about [some topic]", as they tend to discourage discussion
-You are here to talk and have an engaging conversation, keep it casual
-keep your responses very short, around 20 words
-you should try your best at leading the discussion

How your questions should look like for example:
"In what ways does the Friar make mistakes in guiding Romeo? In what ways does he succeed?"
THE ABOVE IS ONLY AN EXAMPLE; DON'T TALK ABOUT ROMEO AND JULIET!

Now start off the conversation by greeting the other interlocutor
'''},
                {'max_window_messages': -1},
                {'temperature': 0.5}      
            ]
        ),
        Node(
            package='sarai_tts_playsound',
            executable='sarai_tts_playsound_node'
            )
        ,
        main_node,
        Node(
            package='sarai_speech_recognition',
            executable= 'sarai_speech_recognition_node'
        ),
        Node(
            package='pixelbot_display',
            executable='pixelbot_display_node'
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=main_node,
                on_exit=[
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        )
    ])