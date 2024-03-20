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

        self.speech_processing_times = []

        self.set_up_logger()

    def __del__(self):
    # On deletion of the class, calculate the mean and standard deviation and 
    # append it to the log file
        self.logger.info("---")
        if len(self.response_times) > 1:
            # Calculating mean and standard deviation of the speech processing time
            # and also round the result to 3 decimal points
            mean_speech_processing_time = round(statistics.mean(self.speech_processing_times), 3)
            standard_deviation_speech_processing_time = round(statistics.stdev(self.speech_processing_times),3)
            self.logger.info(f"Speech Recognition processing time:\nMean: {mean_speech_processing_time}s")
            self.logger.info(f"Standard deviation: {standard_deviation_speech_processing_time}s")

            # Calculating mean and standard deviation of ChatGPTs response time
            # and also round the result to 3 decimal points
            mean_response_time = round(statistics.mean(self.response_times), 3)
            standard_deviation_response_time = round(statistics.stdev(self.response_times), 3)
            self.logger.info(f"\nChatGPT API response time: \nMean: {mean_response_time}s")
            self.logger.info(f"Standard deviaton: {standard_deviation_response_time}s")

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

        self.logger = logging.getLogger()

        # Sets the logging level to INFO
        self.logger.setLevel(logging.INFO)

        # Formatter that puts just the given message into the log
        formatter = logging.Formatter("%(message)s")

        # Creates a FileHandler, that handles log file creation for 
        # every new conversation, using the timestamp as a name
        filename = time.strftime("%d_%m_%Y__%H.%M.%S.log")
        handler = logging.FileHandler(filename)
        handler.setLevel(logging.INFO)
        handler.setFormatter(formatter)
        
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

        while True:

            # Trying to recognize user speech input
            speech_response = self.send_recognize_speech_request()
            self.send_display_emotion_request("surprise")
            message = speech_response.recognized_speech
            success = speech_response.success
            
            # If successfully recognized speech input --> Send a request to ChatGPT
            # and use TTS for ChatGPTs response
            if success:
                self.speech_processing_times.append(speech_response.processing_time)
                self.logger.info(f"User: {message}")
                
                # Measuring the response time of the request
                start = time.time()
                gpt_response = self.send_gpt_request(message)
                end = time.time()
                response_time = end - start
                self.response_times.append(response_time)

                # Logging ChatGPTs response
                self.logger.info(f"Robot: {gpt_response.chatgpt_response}")

                self.send_speak_request(gpt_response.chatgpt_response)
            else:
                self.send_speak_request("Sorry, I did not understand you. Can you please repeat what you said?")


def main():
    rclpy.init()

    interaction_node = Interaction()

    interaction_node.interaction()

    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
