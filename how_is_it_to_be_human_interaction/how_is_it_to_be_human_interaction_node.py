import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from std_srvs.srv import Empty

from sarai_msgs.srv import SetSpeech, GPTRequest, RecognizeSpeech, SetVoiceAlteration, GetGPTRequestParams, UnsuccessfulSpeechRecognition

from pixelbot_msgs.srv import DisplayEmotion, MotorsMovement

import logging, logging.handlers
import time, numpy


class Interaction(Node):
    
    # Defines
    INFINITE_CONVERSATION = -1
    RIGHT_ANTENNA, LEFT_ANTENNA = "right_antenna", "left_antenna"

    def __init__(self):    
        super().__init__('how_is_it_to_be_human_interaction_node')

        # Create client to send a request to ChatGPT
        self.gpt_request_cli = self.create_client(GPTRequest, 'gpt_request')

        # Create client to make PixelBot speak
        self.speak_cli = self.create_client(SetSpeech, 'speak')

        # Create client to listen
        self.listen_cli = self.create_client(Empty, 'listen')

        # Create client to recognize speech
        self.recognize_speech_cli = self.create_client(RecognizeSpeech, 'recognize_speech')

        # Create client to perform emotion
        self.display_emotion_cli = self.create_client(DisplayEmotion, 'display_emotion')

        # Create client to perform antennae movements for emotions
        self.motors_movement_cli = self.create_client(MotorsMovement, 'motors_movement')

        # Create a client for setting the voice alteration
        self.change_voice_alteration_cli = self.create_client(SetVoiceAlteration, 'change_voice_alteration')

        # Create client to get parameters of gpt_requester
        self.get_gpt_request_params_cli = self.create_client(GetGPTRequestParams, 'get_gpt_request_params')

        # Create a client for unsuccessful speech recognition
        self.unsuccessful_speech_recognition_cli = self.create_client(UnsuccessfulSpeechRecognition, 'unsuccessful_speech_recognition')

        # Wait for clients to be ready
        for client in [self.gpt_request_cli, self.speak_cli, self.recognize_speech_cli, 
                       self.display_emotion_cli, self.change_voice_alteration_cli, 
                       self.get_gpt_request_params_cli, self.unsuccessful_speech_recognition_cli,
                       self.motors_movement_cli]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{client.srv_name} service not available, waiting again...')

        # Lists of response and processing times, needed for logging purposes.
        self.gpt_response_times = []
        self.speech_processing_times = []
        self.tts_processing_times = []

        # Parameter for limiting the conversation length by number of back and forth messages
        max_conversation_length_descriptor = ParameterDescriptor(description="Number of back and forth messages")
        self.declare_parameter('max_conversation_length', -1, max_conversation_length_descriptor)

        self.set_up_logger()

    def __del__(self):
        """
        Called upon deletion of the class: calculate the means and standard deviations and 
        append it to the log file; also position antennae into neutral again
        """

        self.conversation_logger.info("---")

        # Calculating mean and standard deviation of the speech processing time
        # and also round the result to 3 decimal points
        mean_speech_processing_time = numpy.mean(numpy.array(self.speech_processing_times))
        standard_deviation_speech_processing_time = numpy.std(numpy.array(self.speech_processing_times))
        self.conversation_logger.info(f"Speech Recognition processing time:\nMean: {mean_speech_processing_time}s")
        self.conversation_logger.info(f"Standard deviation: {standard_deviation_speech_processing_time}s")
        self.conversation_logger.info(f"All times: {self.speech_processing_times}")

        # Calculating mean and standard deviation of ChatGPTs response time
        # and also round the result to 3 decimal points
        mean_gpt_response_time = round(numpy.mean(numpy.array(self.gpt_response_times)), 3)
        standard_deviation_gpt_response_time = round(numpy.std(numpy.array(self.gpt_response_times)), 3)
        self.conversation_logger.info(f"\nChatGPT API response time: \nMean: {mean_gpt_response_time}s")
        self.conversation_logger.info(f"Standard deviaton: {standard_deviation_gpt_response_time}s")
        self.conversation_logger.info(f"All times: {self.gpt_response_times}")

        # Calculating mean and standard deviation of TTS processing time
        mean_tts_processing_time = numpy.mean(numpy.array(self.tts_processing_times))
        standard_deviation_tts_processing_time = numpy.std(numpy.array(self.tts_processing_times))
        self.conversation_logger.info(f"\nTTS processing time: \nMean: {mean_tts_processing_time}s")
        self.conversation_logger.info(f"Standard deviaton: {standard_deviation_tts_processing_time}s")
        self.conversation_logger.info(f"All times: {self.tts_processing_times}")

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
    
    def send_motors_movement_request(self, body_parts, angles):
        """
        Send a request to the emotion_antennae_movement service server.

        :param desired_emotion: String to specify which emotion
                                should be performed.
        """

        self.request = MotorsMovement.Request()
        self.request.body_parts = body_parts
        self.request.angles = angles

        self.future = self.motors_movement_cli.call_async(self.request)
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

    def send_listen_request(self):
        """
        Send a request to the listen service server.
        """

        request = Empty.Request()

        self.future = self.listen_cli.call_async(request)
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

        self.conversation_logger = logging.getLogger()

        # Sets the logging level to INFO
        self.conversation_logger.setLevel(logging.INFO)

        # Formatter that puts just the given message into the log
        formatter = logging.Formatter("%(message)s")

        # Creates a FileHandler, that handles log file creation for 
        # every new conversation, using the timestamp as a name
        filename = time.strftime("%m_%d_%Y__%H.%M.log")
        handler = logging.FileHandler(f"./thesis_logs/{filename}")
        handler.setLevel(logging.INFO)
        handler.setFormatter(formatter)
        
        self.conversation_logger.addHandler(handler)

    def send_unsuccessful_speech_recognition_request(self, error_message):
        """
        Sends a request to the unsuccessful_speech_recognition service server.

        :param error_message: String to set the error_message.
        """

        request = UnsuccessfulSpeechRecognition.Request()
        request.error_message = error_message

        self.future = self.unsuccessful_speech_recognition_cli.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def interaction(self):
        """
        Main interaction.
        """

        self.conversation_logger.info("Participant information:\n")
        self.conversation_logger.info("Age: ")
        self.conversation_logger.info("Gender: \nMale(M)/Female(F)/Divers(D)/Non-specified(N)\n---")

        # Sends a request to get the parameter values of the GPTRequest node.
        # And then put all the parameters at the beginning of the log file.
        gpt_params = self.send_get_gpt_request_params_request()
        gpt_params_string = f"ChatGPT persona: {gpt_params.chatgpt_persona} \n" 
        gpt_params_string += f"Temperature: {gpt_params.temperature}\n"
        gpt_params_string += f"Maximum Window of last messages: {gpt_params.max_window_messages}\n---"
        self.conversation_logger.info(gpt_params_string)

        max_conversation_length = self.get_parameter("max_conversation_length").get_parameter_value().integer_value
        self.conversation_logger.info(f"Maximum conversation length: {max_conversation_length}\n---")

        # Sets the voice alteration to False.
        self.send_change_voice_alteration_request(False)

        # Empty input for starting the conversation with ChatGPT
        start = time.time()
        gpt_response = self.send_gpt_request()
        end = time.time()
        self.gpt_response_times.append(end - start)
        
        if not gpt_response.success:
            self.conversation_logger.info("ERROR: %s",gpt_response.chatgpt_response, exc_info=1)
            return

        # Logging the response
        self.conversation_logger.info(f"Robot: {gpt_response.chatgpt_response}")

        # Positioning the antennae both pointed towards the middle when the robot is speaking
        self.send_motors_movement_request(['right_antenna', 'left_antenna'], [100, 80])

        tts_response = self.send_speak_request(gpt_response.chatgpt_response)
        self.tts_processing_times.append(tts_response.processing_time)

        # Number of sent messages to GPT
        conversation_length = 0

        while max_conversation_length == self.INFINITE_CONVERSATION or conversation_length < max_conversation_length:

            # Positioning the antennae both upwards when the robot is listening
            self.send_motors_movement_request(['right_antenna', 'left_antenna'], [70, 110])

            time.sleep(0.5)

            # Perform an emotion to let the user know that the robot is listening
            self.send_display_emotion_request("happy")

            # Trying to listen to user speech input
            self.send_listen_request()
            
            # Positioning the antennae bot to the left while the robot is thinking
            self.send_motors_movement_request(['right_antenna', 'left_antenna'], [60, 70])

            # Perform an emotion to let the user know that the robot stopped listening
            self.send_display_emotion_request("surprise")

            # Trying to recognize user speech input
            speech_response = self.send_recognize_speech_request()

            # If successfully recognized speech input --> Send a request to ChatGPT
            # and use TTS for ChatGPTs response
            if speech_response.success:
                self.speech_processing_times.append(speech_response.processing_time)
                self.conversation_logger.info(f"User: {speech_response.recognized_speech}")
                
                # Measuring the response time of the request
                start = time.time()
                gpt_response = self.send_gpt_request(speech_response.recognized_speech)
                end = time.time()

                if not gpt_response.success:
                    self.conversation_logger.info("ERROR: %s",gpt_response.chatgpt_response, exc_info=1)
                    return

                self.gpt_response_times.append(end - start)
                self.conversation_logger.info(f"Robot response time: {end - start}s")

                # Logging ChatGPTs response
                self.conversation_logger.info(f"Robot: {gpt_response.chatgpt_response}")

                # Positioning the antennae both pointed towards the middle when the robot is speaking
                self.send_motors_movement_request(['right_antenna', 'left_antenna'], [100, 80])

                tts_response = self.send_speak_request(gpt_response.chatgpt_response)
                self.tts_processing_times.append(tts_response.processing_time)

                # Increment the back and forth messages counter
                conversation_length += 1
            else:
                self.send_motors_movement_request(['right'])
                self.send_display_emotion_request("sad")
                error_message = "Sorry, I did not understand you. Can you please repeat what you said?"
                
                # Adds the error_message to the message history
                self.send_unsuccessful_speech_recognition_request(error_message)
                self.conversation_logger.info(f"Robot: {error_message}")

                tts_response = self.send_speak_request(error_message)
                self.tts_processing_times.append(tts_response.processing_time)

        self.send_speak_request("Thank you for participating in this test. Have a wonderful day!")


def main():
    rclpy.init()

    interaction_node = Interaction()
    interaction_node.interaction()

    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
