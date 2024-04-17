import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from sarai_msgs.srv import RecognizeSpeech

import speech_recognition as sr
import time

# Removes all unnecessary error prints
import sounddevice

class Sarai_Speech_Recognition(Node):

    def __init__(self):
        super().__init__("sarai_speech_recognition_node")

        # Service to listen to the microphone
        self.listen_srv = self.create_service(Empty, "listen", self.listen_callback)

        # Service to perform speech recognition
        self.recognize_speech_srv = self.create_service(RecognizeSpeech, "recognize_speech", self.recognize_speech_callback)

        self.speech_recognizer = sr.Recognizer()

        # Initialize microphone by listening for 1 second to calibrate the energy 
        # threshold for ambient noise levels
        with sr.Microphone() as source:
            self.speech_recognizer.adjust_for_ambient_noise(source)

            # Need to disable dynamic energy threshold, otherwise 
            # SpeechRecognition won't work after a few times
            self.speech_recognizer.dynamic_energy_threshold = False

            # Seconds of non speaking audio before the speaking audio is considered a phrase
            # Default is 0.8
            self.speech_recognizer.pause_threshold = 1.5

    def listen_callback(self, request, response):
        """
        Service handler listening until the speech is finished.

        :param request: See RecognizeSpeech service definition.
        :param response: See RecognizeSpeech service definition.
        """

        # Setting up microphone for Speech Recognition
        with sr.Microphone() as source:
            self.audio = self.speech_recognizer.listen(source, None)
        
    def recognize_speech_callback(self, request, response):
        """
        Service handler trying to recognize speech from the recorded audio

        :param request: See RecognizeSpeech service definition.
        :param response: See RecognizeSpeech service definition.
        """
        try:
            start = time.time()
            response.recognized_speech = self.speech_recognizer.recognize_google(self.audio)
            end = time.time()

            response.processing_time = end - start
            response.success = True
            return response
        except sr.UnknownValueError:
            response.recognized_speech = "Google Speech Recognition could not understand audio. Please try again!"
            response.success = False
            return response
        except sr.RequestError as error:
            response.recognized_speech = "Could not request results from Google Speech Recognition service; {0}. Please try again!".format(error)
            response.success = False
            return response
        

def main(args=None):
    rclpy.init(args=args)

    speech_recognition_node = Sarai_Speech_Recognition()

    rclpy.spin(speech_recognition_node)

    speech_recognition_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
