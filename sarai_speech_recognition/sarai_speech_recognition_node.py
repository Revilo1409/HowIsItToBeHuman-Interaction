import rclpy
from rclpy.node import Node

from sarai_msgs.srv import RecognizeSpeech

import speech_recognition as sr


class Sarai_Speech_Recognition(Node):

    def __init__(self):
        super().__init__("sarai_speech_recognition_node")

        # Service to perform speech recognition
        self.recognize_speech_srv = self.create_service(RecognizeSpeech, "recognize_speech", self.recognize_speech_callback)

        self.speech_recognizer = sr.Recognizer()

        # Initialize microphone by listening for 1 second to calibrate the energy 
        # threshold for ambient noise levels
        with sr.Microphone() as source:
            self.speech_recognizer.adjust_for_ambient_noise(source)  

    def recognize_speech_callback(self, request, response):
        """
        Service handler performing the speech recognition

        :param request: See RecognizeSpeech service definition.
        :param response: See RecognizeSpeech service definition.
        """

        # Setting up microphone for Speech Recognition
        with sr.Microphone() as source:
            print("Say something!")
            self.audio = self.speech_recognizer.listen(source, None)
            print("Said something")
            
        try:
            response.recognized_speech = self.speech_recognizer.recognize_google(self.audio)
            response.success = True
            return response
        except sr.UnknownValueError:
            response.recognized_speech = "Google Speech Recognition could not understand audio. Please try again!"
            response.success = False
            return response
        except sr.RequestError as e:
            response.recognized_speech = "Could not request results from Google Speech Recognition service; {0}. Please try again!".format(e)
            response.success = False
            return response


def main(args=None):
    rclpy.init(args=args)

    speech_recognition_node = Sarai_Speech_Recognition()

    rclpy.spin(speech_recognition_node)

    speech_recognition_node.destroy_node()
    rclpy.shutdown()


"""
    nodee = Sarai_Speech_Recognition()
    request = RecognizeSpeech.Request
    response = RecognizeSpeech.Response
    while True:
        nodee.recognize_speech_callback(request, response)"""


if __name__ == "__main__":
    main()
