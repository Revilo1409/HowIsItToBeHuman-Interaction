import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor

from std_srvs.srv import Empty

from sarai_msgs.srv import GPTRequest

from openai import OpenAI
import openai
import sys
import time


class GPTRequester(Node):
    MODEL = "gpt-3.5-turbo-1106"
    MESSAGE_HISTORY = []

    def __init__(self):
        super().__init__('gptrequester')
        self.gpt_request_srv = self.create_service(GPTRequest, 'gpt_request', self.gpt_request_callback) 
        role_descriptor = ParameterDescriptor(description = """This is a 
                            parameter for setting the role of ChatGPT""") 
        maxWindow_messages_descriptor = ParameterDescriptor(description = "Max number of messages stored in the message history") 
        api_key_descriptor = ParameterDescriptor(description= "API Key for ChatGPT API")                                      
        self.declare_parameter('role', '', role_descriptor)
        self.declare_parameter('maxWindow_messages', 100, maxWindow_messages_descriptor)
        self.api_key = sys.argv[1]                               
        self.client = OpenAI(api_key = self.api_key)

    def gpt_request_callback(self, request, response):
        chat = self.client.chat.completions.create(model=self.MODEL, messages= self.get_maxWindow_messages(request.user_input),temperature=0.7)
        response.chatgpt_response = chat.choices[0].message.content
        response.success = True
        return response  
        
    def get_maxWindow_messages(self, user_input):
        role_message = {"role": "system", "content": self.get_parameter('role').get_parameter_value().string_value}
        maxWindow_messages = self.get_parameter('maxWindow_messages').get_parameter_value().integer_value
        self.MESSAGE_HISTORY.append({"role": "user", "content" : user_input})
        if len(self.MESSAGE_HISTORY) > maxWindow_messages + 1: 
            return [role_message]+(self.MESSAGE_HISTORY[-(maxWindow_messages + 1):])
        else:
            return [role_message]+(self.MESSAGE_HISTORY)

def main(args=None):
    rclpy.init(args=args)
    gpt_requester_node = GPTRequester()
    rclpy.spin(gpt_requester_node)
    gpt_requester_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

    