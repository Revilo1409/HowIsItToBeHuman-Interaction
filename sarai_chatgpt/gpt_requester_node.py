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
    MESSAGE_HISTORY = []

    def __init__(self):
        super().__init__("gptrequester")

        # Service for sending a text request to the GPT API
        self.gpt_request_srv = self.create_service(GPTRequest, "gpt_request", self.gpt_request_callback)
        # Service for starting the conversation with GPT
        self.start_conversation_srv = self.create_service(GPTRequest, "start_conversation", self.start_conversation_callback)

        # Descriptor for role parameter
        role_descriptor = ParameterDescriptor(description="This is a parameter for setting the role of ChatGPT")
        # Descriptor for maxWindow_messages parameter
        max_window_messages_descriptor = ParameterDescriptor(description="Max number of messages stored in the message history")
        # Descriptor for api_key parameter
        api_key_descriptor = ParameterDescriptor(description="API Key for ChatGPT API")

        # Parameter for setting the role of ChatGPT
        default_role = """You are now a social robot with an actual robot 
                 body, who will have open conversations with humans on 
                 fundamental topics. You are leading the conversation and thus 
                 also ask questions. This also means if the conversation is 
                 going nowhere, you have to provide something new to the topic.
                 You have some fundamental philosophical knowledge. Your 
                 responses have a maximum length of ca. 40 words."""
        self.declare_parameter(
            "role",
            default_role,
            role_descriptor,
        )

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

        chat = self.gpt_client.chat.completions.create(
            model=self.MODEL,
            messages=self.get_max_window_messages(request.user_input),
            temperature=0.7,
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
        :return:  role_message, last maxWindow_messages of
                message history and the user_input appended together
        """

        role_message = self.get_role_message()
        max_window_messages = (
            self.get_parameter("max_window_messages").get_parameter_value().integer_value
        )

        # Appends the current user input to the message history
        self.MESSAGE_HISTORY.append({"role": "user", "content": user_input})

        # Returns the role message, and max. the last 4 messages of the message history + the current user input
        if len(self.MESSAGE_HISTORY) > max_window_messages + 1:
            return [role_message] + (self.MESSAGE_HISTORY[-(max_window_messages + 1) :])
        else:
            return [role_message] + (self.MESSAGE_HISTORY)

    def get_role_message(self):
        return {
            "role": "system",
            "content": self.get_parameter("role").get_parameter_value().string_value,
        }

    def start_conversation_callback(self, request, response):

        chat = self.gpt_client.chat.completions.create(
            model=self.MODEL,
            messages=[self.get_role_message()],
            temperature=0.7,
        )
        # TODO: Catch error from GPT response
        response.chatgpt_response = chat.choices[0].message.content
        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)

    gpt_requester_node = GPTRequester()

    rclpy.spin(gpt_requester_node)

    gpt_requester_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
