import rclpy
from rclpy.node import Node

from sarai_msgs.srv import GPTRequest

class Interaction(Node):
    def __init__(self):
        super().__init__('interaction_node')
        self.gpt_request_cli = self.create_client(GPTRequest, 'gpt_request')
        while not self.gpt_request_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.gpt_request_cli.srv_name} service not available, waiting again...')
    
    def gpt_request(self, user_input):
        request = GPTRequest.Request()
        request.user_input = user_input
        self.future = self.gpt_request_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()

    interaction = Interaction()
    while True:
        try:
            message = input("User: ")
        except KeyboardInterrupt:
            break
        if message:
            response = interaction.gpt_request(message)
            print(f"GPT: {response.chatgpt_response}")
    print("\nClosing GPTClient...")

if __name__ == '__main__':
    main()