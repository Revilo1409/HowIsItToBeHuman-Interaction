import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

from sarai_msgs.srv import GPTRequest

from openai import OpenAI
import openai
import sys
import time


class GPTRequester(Node):
    MODEL = "gpt-3.5-turbo-1106"
    API_KEY=""

    def __init__(self):
        self.gpt_request_srv = self.create_service(GPTRequest, 'gpt_request', 
                                                   self.gpt_request_callback)
        self.client = OpenAI(api_key = self.API_KEY)