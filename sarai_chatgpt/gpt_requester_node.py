import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from std_srvs.srv import Empty

from sarai_msgs.srv import GPTRequest, GetGPTRequestParams, UnsuccessfulSpeechRecognition

from openai import OpenAI
import os


class GPTRequester(Node):
    MODEL = "gpt-3.5-turbo-1106"
    INFINITE_MESSAGE_HISTORY = -1

    def __init__(self):
        super().__init__("gptrequester")

        # Service for sending a text request to the GPT API
        self.gpt_request_srv = self.create_service(GPTRequest, "gpt_request", self.gpt_request_callback)

        # Service for getting the necessary independant variables from the parameters
        self.get_gpt_request_params_srv = self.create_service(GetGPTRequestParams, "get_gpt_request_params", self.get_gpt_request_params_callback)

        # Service for appending the unsuccessful speech recognition message to the message history
        self.unsuccessful_speech_recognition_srv = self.create_service(UnsuccessfulSpeechRecognition, "unsuccessful_speech_recognition", self.unsuccessful_speech_recognition_callback)

        # Descriptor for api_key parameter
        api_key_descriptor = ParameterDescriptor(description="API Key for ChatGPT API")

        # Descriptor for role parameter
        chatgpt_persona_descriptor = ParameterDescriptor(description="This is a parameter for setting the role of ChatGPT")

        # Descriptor for maxWindow_messages parameter
        max_window_messages_descriptor = ParameterDescriptor(description="Max number of messages stored in the message history")

        # Descriptor for the temperature parameter
        temperature_descriptor = ParameterDescriptor(description="The temperature used in the ChatGPT API request")

        # Get the API key from the environment variable.
        self.declare_parameter("api_key", os.getenv("OPENAI_API_KEY"), api_key_descriptor)

        # Parameter for setting the role of ChatGPT
        default_chatgpt_persona = """You are now a social robot with an actual robot 
                 body, who will have open conversations with humans on 
                 fundamental topics. You are leading the conversation and thus 
                 also ask questions. This also means if the conversation is 
                 going nowhere, you have to provide something new to the topic.
                 You have some fundamental philosophical knowledge. Your 
                 responses have a maximum length of ca. 20 words."""
        self.declare_parameter("chatgpt_persona", default_chatgpt_persona, chatgpt_persona_descriptor)

        # Parameter for setting the maximum number of messages in the history that should be sent to GPT
        self.declare_parameter("max_window_messages", self.INFINITE_MESSAGE_HISTORY, max_window_messages_descriptor)
        
        # Parameter for setting the temperature in the ChatGPT API request
        self.declare_parameter("temperature", 0.7, temperature_descriptor)

        # Creating client for communicating with GPT API
        self.gpt_client = OpenAI(api_key=self.get_parameter("api_key").get_parameter_value().string_value)

        # Parameter for storing the message history with GPT
        self.message_history = []

        # Boolean that states if the conversation already has been started before
        self.conversation_started = False   

    def gpt_request_callback(self, request, response):
        """
        Service handler performing a request to the OpenAI API

        :param request: See GPTRequest service definition.
        :param response: See GPTRequest service definition
        """

         # Value between 0 and 1. Used to set the creativity of ChatGPTs answers
        temperature = self.get_parameter("temperature").get_parameter_value()._double_value
        
        messages = []

        # If the conversation wasn't started yet, only put the persona message 
        # into the messages.
        if not self.conversation_started:
            self.conversation_started = True
            messages = [self.get_chatgpt_persona_message()]
        else:
            user_input_message = {"role": "user", "content": request.user_input}
            self.message_history.append(user_input_message)
            messages = self.get_max_window_messages(user_input_message)

        chat = self.gpt_client.chat.completions.create(
            model=self.MODEL,
            messages=messages,
            temperature=temperature,
        )

        # TODO: Catch error from GPT response

        # By default, the API request creates one answer, but multiple could also
        # be given. We only create one.
        chatgpt_response = chat.choices[0].message.content
        response.chatgpt_response = chatgpt_response
        response.success = True

        self.message_history.append({"role": "assistant", "content": chatgpt_response})

        return response

    def get_max_window_messages(self, user_input_message):
        """
        Help method that appends role_message, last maxWindow_messages of
        message history and the user_input

        :param user_input: Message the user wants to send to GPT API
        :return: role_message, last max_window_messages of
                 message history and the user_input appended together
        """

        chatgpt_persona_message = self.get_chatgpt_persona_message()
        max_window_messages = self.get_parameter("max_window_messages").get_parameter_value().integer_value
        
        # If max_window_messages = -1, use whole message history
        if max_window_messages == self.INFINITE_MESSAGE_HISTORY:
            last_max_window_messages = self.message_history
        else:
            last_max_window_messages = self.message_history[-max_window_messages:]

        # Prepends the chatgpt_persona_message
        last_max_window_messages.insert(0, chatgpt_persona_message)

        # Appends the current user input message to the return value
        last_max_window_messages.append(user_input_message)

        # Returns the role message, and max. the last max_window_messages of the message history + the current user input message
        return last_max_window_messages

    def get_chatgpt_persona_message(self):
        """
        Returns the formatted role message by using the role parameter.

        :return: Dict containing the role message.
        """
        
        chatgpt_persona_message = {
            "role": "system",
            "content": self.get_parameter("chatgpt_persona").get_parameter_value().string_value,
        }

        return chatgpt_persona_message
    
    def get_gpt_request_params_callback(self, request, response):
        """
        Service handler returning all the node's ROS2 parameters (except api_key).

        :param request: See GetGPTRequestParams service definition.
        :param response: See GetGPTRequestParams service definition
        """

        response.chatgpt_persona = self.get_parameter("chatgpt_persona").get_parameter_value().string_value
        response.temperature = self.get_parameter("temperature").get_parameter_value().double_value
        response.max_window_messages = self.get_parameter("max_window_messages").get_parameter_value().integer_value
        return response
    
    def unsuccessful_speech_recognition_callback(self, request, response):
        """
        Appends the error message to the message history, if the speech 
        recognition wasn't successful.

        :param request: See UnsuccessfulSpeechRecognition service definition.
        :param response: See UnsuccessfulSpeechRecognition service definition
        """

        error_message =  {
            "role": "assistant",
            "content": request.error_message
        }
        self.message_history.append(error_message)
        return response


def main(args=None):
    rclpy.init(args=args)

    gpt_requester_node = GPTRequester()

    rclpy.spin(gpt_requester_node)

    gpt_requester_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
