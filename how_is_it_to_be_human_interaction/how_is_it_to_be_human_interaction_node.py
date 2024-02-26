import rclpy
from rclpy.node import Node

from sarai_msgs.srv import SetSpeech, GPTRequest, RecognizeSpeech

class Interaction(Node):
    def __init__(self):
        super().__init__('interaction_node')

        # Create client to send a request to ChatGPT
        self.gpt_request_cli = self.create_client(GPTRequest, 'gpt_request')

        # Create client to make PixelBot speak
        self.speak_cli = self.create_client(SetSpeech, 'speak')

        # Create client to recognize speech
        self.recognize_speech_cli = self.create_client(RecognizeSpeech, 'recognize_speech')

        while not self.gpt_request_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.gpt_request_cli.srv_name} service not available, waiting again...')
    
    def send_gpt_request(self, user_input):
        """
        Send a request to the gpt_request service server.

        :param user_input: String to specifiy the message sent to GPT.
        """
        request = GPTRequest.Request()
        request.user_input = user_input
        self.future = self.gpt_request_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def send_speak_request(self, message):
        """
        Send a request to the speak service server.
        :param message: String to specify the text to be spoken.
        """
        request = SetSpeech.Request()

        print(f"typ: {type(message)}")
        request.message = message

        self.future = self.speak_cli.call_async(request)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
    
    def send_recognize_speech_request(self):
        """
        Send a request to the recognize_speech service server.
        """
        request = RecognizeSpeech.Request()
        self.future = self.recognize_speech_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def interaction(self):
        """
        Main interaction.
        """
        
        while True:
            try:
                message = input("User: ")
            except KeyboardInterrupt:
                break
            if message:
                response = self.send_gpt_request(message)
                self.send_speak_request(response.chatgpt_response)
                print(f"GPT: {response.chatgpt_response}")

        print("\nClosing GPTClient...")

def main():
    rclpy.init()

    interaction_node = Interaction()

    interaction_node.interaction()

    interaction_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()