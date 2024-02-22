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
    API_KEY=""
    MESSAGE_HISTORY = []

    def __init__(self):
        self.gpt_request_srv = self.create_service(GPTRequest, 'gpt_request',
                                                   self.gpt_request_callback) 
        role_descriptor = ParameterDescriptor(description = """This is a 
                            parameter for setting the role of ChatGPT""") 
        maxWindow_messages_descriptor = ParameterDescriptor(description = "Max number of messages stored in the message history")                                       
        self.declare_parameters([('role', '', role_descriptor),
                                 ('maxWindow_messages', 100, 
                                  maxWindow_messages_descriptor)])
        self.client = OpenAI(api_key = self.API_KEY)

    def gpt_request_callback(self, request, response):
        
        chat = self.client.chat.completions.create(
            model=self.MODEL, messages= self.get_maxWindow_messages(), 
            temperature=0.7
        )
        response.chatgpt_response = ""
        response.success = True
        return response  
        
    def get_maxWindow_messages(self):
        role_message = {"role": "system", "content": self.get_parameter('role').get_parameter_value().string_value}
        maxWindow_messages = self.get_parameter('maxWindow_messages').get_parameter_value().integer_value
        if len(self.MESSAGE_HISTORY) > maxWindow_messages:
            return role_message.append(self.MESSAGE_HISTORY[-maxWindow_messages:])
        else:
            return role_message.append(self.MESSAGE_HISTORY)

def main(args=None):
    rclpy.init(args=args)
    gpt_requester_node = GPTRequester()
    rclpy.spin(gpt_requester_node)
    gpt_requester_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

    