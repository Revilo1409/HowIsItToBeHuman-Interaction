from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
import sys

def generate_launch_description():
    return LaunchDescription(
        [
        Node(
            package='sarai_chatgpt',
            executable='gpt_requester_node',
            parameters=[
                {'chatgpt_persona': '''You are now a social robot with an actual robot 
                 body, who will have open conversations with humans on 
                 fundamental topics. You are leading the conversation and thus 
                 also ask questions. This also means if the conversation is 
                 going nowhere, you have to provide something new to the topic.
                 You have some fundamental philosophical knowledge. Your 
                 responses have a maximum length of ca. 40 words.'''},
                {'max_window_messages': 4},         
            ]
        ),
        Node(
            package='sarai_tts_playsound',
            executable='sarai_tts_playsound_node'
            )
        ,
        Node(
            package='how_is_it_to_be_human_interaction',
            executable='how_is_it_to_be_human_interaction_node',
        ),
        Node(
            package='sarai_speech_recognition',
            executable= 'sarai_speech_recognition_node'
        ),
        Node(
            package='pixelbot_display',
            executable='pixelbot_display_node'
        )
    ])