import rclpy
from rclpy.node import Node

from sarai_msgs.srv import SetSpeech, GPTRequest, RecognizeSpeech

from pixelbot_msgs.srv import DisplayEmotion

class Interaction(Node):
    def __init__(self):
        super().__init__("how_is_it_to_be_human_interaction_node")

        self.gpt_request_cli = self.create_client(GPTRequest, "gpt_request")

        self.speak_cli = self.create_client(SetSpeech, "speak")

        self.recognize_speech_cli = self.create_client(
            RecognizeSpeech, "recognize_speech"
        )

        # Create client to perform emotion
        self.display_emotion_cli = self.create_client(DisplayEmotion, 'display_emotion')

        while not self.gpt_request_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"{self.gpt_request_cli.srv_name} service not available, waiting again..."
            )

    def send_display_emotion_request(self, desired_emotion):
        """
        Send a request to the display_emotion service server.

        :param desired_emotion: String to specify which emotion
                                should be displayed.
        """

        self.request = DisplayEmotion.Request()
        self.request.desired_emotion = desired_emotion

        self.future = self.display_emotion_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_gpt_request(self, user_input):
        request = GPTRequest.Request()
        request.user_input = user_input
        self.future = self.gpt_request_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_speak_request(self, gpt_response):
        request = SetSpeech.Request()
        request.message = gpt_response

        self.future = self.speak_cli.call_async(request)

        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_recognize_speech_request(self):
        request = RecognizeSpeech.Request()
        self.future = self.recognize_speech_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def interaction(self):
        
        # Empty input for starting the conversation with ChatGPT
        gpt_response = self.send_gpt_request("")
        self.send_speak_request(gpt_response.chatgpt_response)

        while True:
            try:
                print("Trying to recognize speech")
                response = self.send_recognize_speech_request()
                self.send_display_emotion_request("surprise")
                message = response.recognized_speech
                success = response.success
            except KeyboardInterrupt:
                break
            if success:
                print(f"The PC understood this:{message}")
                gpt_response = self.send_gpt_request(message)
                print(f"GPT: {gpt_response.chatgpt_response}")
                self.send_speak_request(gpt_response.chatgpt_response)
            else:
                self.send_speak_request(
                    "Sorry, I did not understand you. Can you please repeadt what you said?"
                )
                print("Message finished.")
        print("\nClosing GPTClient...")


def main():
    rclpy.init()

    interaction_node = Interaction()

    interaction_node.interaction()

    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
