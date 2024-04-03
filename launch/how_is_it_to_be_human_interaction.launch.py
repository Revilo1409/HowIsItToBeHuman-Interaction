from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

def generate_launch_description():
    return LaunchDescription(
        [
        Node(
            package='sarai_chatgpt',
            executable='gpt_requester_node',
            parameters=[
                {'chatgpt_persona': '''You are a social robot with an actual robot body. You will have an open discussion with an interlocutor on “How is it to be a human?”. You are the leader of the discussion. Your responses should be as short as 20 words and should create an engaging discussion with the interlocutor. You will start off the conversation by greeting the interlocutor. '''},
                {'max_window_messages': -1},
                {'temperature': 0.7}      
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
            parameters=[
                {'max_conversation_length': 30}
            ]
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