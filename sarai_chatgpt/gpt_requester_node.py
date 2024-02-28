import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from std_srvs.srv import Empty

from sarai_msgs.srv import GPTRequest

from openai import OpenAI
import os


class GPTRequester(Node):
    MODEL = "gpt-3.5-turbo-1106"

    # Parameter for storing the message history with GPT
    message_history = []

    # Boolean that states if the conversation already has been started before
    conversation_started = False

    def __init__(self):
        super().__init__("gptrequester")

        # Service for sending a text request to the GPT API
        self.gpt_request_srv = self.create_service(GPTRequest, "gpt_request", self.gpt_request_callback)

        # Descriptor for role parameter
        chatgpt_persona_descriptor = ParameterDescriptor(description="This is a parameter for setting the role of ChatGPT")
        # Descriptor for maxWindow_messages parameter
        max_window_messages_descriptor = ParameterDescriptor(description="Max number of messages stored in the message history")
        # Descriptor for api_key parameter
        api_key_descriptor = ParameterDescriptor(description="API Key for ChatGPT API")

        # Parameter for setting the role of ChatGPT
        default_chatgpt_persona = """You are now a social robot with an actual robot 
                 body, who will have open conversations with humans on 
                 fundamental topics. You are leading the conversation and thus 
                 also ask questions. This also means if the conversation is 
                 going nowhere, you have to provide something new to the topic.
                 You have some fundamental philosophical knowledge. Your 
                 responses have a maximum length of ca. 40 words."""
        
        self.declare_parameter("chatgpt_persona", default_chatgpt_persona, chatgpt_persona_descriptor,)

        # Parameter for setting the maximum number of messages in the history that should be sent to GPT
        self.declare_parameter("max_window_messages", 100, max_window_messages_descriptor)
        # TODO: ADD THIS IN README --> API Key for ChatGPT API
        # You must add OPENAI_API_KEY as an environment variable
        # In Ubuntu: echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
        # Get the API key from the environment variable.
        self.declare_parameter("api_key", os.getenv("OPENAI_API_KEY"), api_key_descriptor)

        # Creating client for communicating with GPT API
        self.gpt_client = OpenAI(api_key=self.get_parameter("api_key").get_parameter_value().string_value)

    def gpt_request_callback(self, request, response):
        """
        Service handler performing a request to the GPT API

        :param request: See GPTRequest service definition.
        :param response: See GPTRequest service definition
        """

        # Value between 0 and 1. Used to set the creativity of ChatGPTs answers
        temperature = 0.7
        
        messages= self.get_max_window_messages(request.user_input)

        if not(self.conversation_started):
            messages = [self.get_chatgpt_persona_message()]

        chat = self.gpt_client.chat.completions.create(
            model=self.MODEL,
            messages=messages,
            temperature=temperature,
        )

        # TODO: Catch error from GPT response
        response.chatgpt_response = chat.choices[0].message.content
        response.success = True

        return response

    def get_max_window_messages(self, user_input):
        """
        Help method that appends role_message, last maxWindow_messages of
        message history and the user_input

        :param user_input: Message the user wants to send to GPT API
        :return: role_message, last maxWindow_messages of
                message history and the user_input appended together
        """

        role_message = self.get_chatgpt_persona_message()
        max_window_messages = self.get_parameter("max_window_messages").get_parameter_value().integer_value

        # Appends the current user input to the message history
        self.message_history.append({"role": "user", "content": user_input})

        # Returns the role message, and max. the last 4 messages of the message history + the current user input
        return [role_message] + (self.message_history[-(max_window_messages + 1) :])

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
    

def main(args=None):
    rclpy.init(args=args)

    gpt_requester_node = GPTRequester()

    rclpy.spin(gpt_requester_node)

    gpt_requester_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
