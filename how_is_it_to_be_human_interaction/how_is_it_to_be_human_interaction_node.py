import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sarai_msgs.srv import SetSpeech, GPTRequest, RecognizeSpeech, SetVoiceAlteration, GetGPTRequestParams

from pixelbot_msgs.srv import DisplayEmotion

import logging, logging.handlers
import time, statistics


class Interaction(Node):

    def __init__(self):    
        super().__init__("how_is_it_to_be_human_interaction_node")

        # Create client to send a request to ChatGPT
        self.gpt_request_cli = self.create_client(GPTRequest, "gpt_request")

        # Create client to make PixelBot speak
        self.speak_cli = self.create_client(SetSpeech, "speak")

        # Create client to recognize speech
        self.recognize_speech_cli = self.create_client(RecognizeSpeech, "recognize_speech")

        # Create client to perform emotion
        self.display_emotion_cli = self.create_client(DisplayEmotion, 'display_emotion')

        # Create a client for setting the voice alteration
        self.change_voice_alteration_cli = self.create_client(SetVoiceAlteration, 'change_voice_alteration')

        # Create client to get parameters of gpt_requester
        self.get_gpt_request_params_cli = self.create_client(GetGPTRequestParams, 'get_gpt_request_params')

        # Wait for clients to be ready
        for client in [self.gpt_request_cli, self.speak_cli, self.recognize_speech_cli, 
                       self.display_emotion_cli, self.change_voice_alteration_cli, 
                       self.get_gpt_request_params_cli]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{client.srv_name} service not available, waiting again...')

        # List of response times, needed for test purposes.
        self.response_times = []

        self.set_up_logger()

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

    def send_gpt_request(self, user_input=None):
        """
        Send a request to the gpt_request service server.

        :param user_input: String to specifiy the message sent to GPT.
        """
        
        request = GPTRequest.Request()
        if user_input:
            request.user_input = user_input

        self.future = self.gpt_request_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_speak_request(self, gpt_response):
        """
        Send a request to the speak service server.

        :param gpt_response: String to specify the text to be spoken.
        """
        
        request = SetSpeech.Request()
        request.message = gpt_response

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
    
    def send_change_voice_alteration_request(self, voice_alteration):
        """
        Send a request to the change_voice_alteration service server.

        :param voice_alteration: Boolean to set the voice alteration.
        """
        
        request = SetVoiceAlteration.Request()
        request.is_voice_altered = voice_alteration

        self.future = self.change_voice_alteration_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
    
    def send_get_gpt_request_params_request(self):
        """
        Sends a request to the get_gpt_request_params service server.
        """

        request = GetGPTRequestParams.Request()

        self.future = self.get_gpt_request_params_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

    def set_up_logger(self):
        """
        Setting up the logger so its ready to use for logging into a file.
        """
        filename = "chat.log"

        self.logger = logging.getLogger()

        # Sets the logging level to INFO
        self.logger.setLevel(logging.INFO)

        # Formatter that puts just the given message into the log
        formatter = logging.Formatter("%(message)s")

        # Creates a RotatingFileHandler, that handles log file creation for 
        # every new conversation
        handler = logging.handlers.RotatingFileHandler(filename, mode = 'w', backupCount=10)
        handler.setLevel(logging.INFO)
        handler.setFormatter(formatter)

        # Does the rollover for the log file, so for every new conversation, 
        # a new log file is created.
        logging.handlers.RotatingFileHandler.doRollover(handler)
        
        self.logger.addHandler(handler)

    def interaction(self):
        """
        Main interaction.
        """
        
        # Sends a request to get the parameter values of the GPTRequest node.
        # And then put all the parameters at the beginning of the log file.
        gpt_params = self.send_get_gpt_request_params_request()
        gpt_params_string = f"ChatGPT persona: {gpt_params.chatgpt_persona} \n" 
        gpt_params_string += f"Temperature: {gpt_params.temperature}\n"
        gpt_params_string += f"Max Window of last messages: {gpt_params.max_window_messages}\n ---"
        self.logger.info(gpt_params_string)

        # Sets the voice alteration to False.
        self.send_change_voice_alteration_request(False)

        # Empty input for starting the conversation with ChatGPT
        gpt_response = self.send_gpt_request()
        self.logger.info(f"Robot: {gpt_response.chatgpt_response}")
        self.send_speak_request(gpt_response.chatgpt_response)

        # While Loop for conversation flow
        while True:

            # Trying to recognize user speech input
            try:
                print("Trying to recognize speech")
                response = self.send_recognize_speech_request()
                self.send_display_emotion_request("surprise")
                message = response.recognized_speech
                success = response.success
            except KeyboardInterrupt:
                # If at least one response time is given
                if self.response_times:
                    average_response_time = statistics.mean(self.response_times)
                    standard_deviation = statistics.stdev(self.response_times)
                    self.logger.info("---")
                    self.logger.info(f"Average reponse time: {average_response_time}")
                    self.logger.info(f"Standard Deviaton of response time: {standard_deviation}")
                break

            # If successfully recognized speech input --> Send a request to ChatGPT
            # and use TTS for ChatGPTs response
            if success:
                self.logger.info(f"User: {message}")
                print(f"The PC understood this:{message}")
                
                # Measuring the response time of the request
                start = time.time()
                gpt_response = self.send_gpt_request(message)
                end = time.time()
                response_time = end - start
                self.response_times.append(response_time)

                # Logging the response time and message
                self.logger.info(f"Response time: {response_time}")
                self.logger.info(f"Robot: {gpt_response.chatgpt_response}")

                print(f"GPT: {gpt_response.chatgpt_response}")
                self.send_speak_request(gpt_response.chatgpt_response)
            else:
                self.send_speak_request("Sorry, I did not understand you. Can you please repeat what you said?")
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
