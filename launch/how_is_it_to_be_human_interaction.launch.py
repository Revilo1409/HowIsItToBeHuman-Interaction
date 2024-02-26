from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

def generate_launch_description():
    api_key_launch_arg= DeclareLaunchArgument('api_key', default_value=TextSubstitution(text='None'))
    return LaunchDescription([
        api_key_launch_arg,
        Node(
            package='sarai_chatgpt',
            executable='gpt_requester_node',
            parameters=[
                {'role': '''You are now a social robot with an actual robot 
                 body, who will have open conversations with humans on 
                 fundamental topics. You are leading the conversation and thus 
                 also ask questions. This also means if the conversation is 
                 going nowhere, you have to provide something new to the topic.
                 You have some fundamental philosophical knowledge. Your 
                 responses have a maximum length of ca. 40 words.'''},
                {'maxWindow_messages': 4},         
            ],
            arguments= [LaunchConfiguration('api_key')]
        ),
        Node(
            package='sarai_tts_playsound',
            executable='sarai_tts_playsound_node'
            )
        ,
        Node(
            package='how_is_it_to_be_human_interaction',
            executable='how_is_it_to_be_human_interaction_node',
        )
    ])