import rclpy
from rclpy.node import Node

from sarai_msgs.srv import RecognizeSpeech

import speech_recognition as sr


class Sarai_Speech_Recognition(Node):

    def __init__(self):
        super().__init__("sarai_speech_recognition_node")

        # Service to perform speech recognition
        self.recognize_speech_srv = self.create_service(
            RecognizeSpeech, "recognize_speech", self.recognize_speech_callback
        )

        self.r = sr.Recognizer()

    def recognize_speech_callback(self, request, response):
        """
        Service handler performing the speech recognition
        :param request: See RecognizeSpeech service definition.
        :param response: See RecognizeSpeech service definition.
        """
        # Setting up microphone for Speech Recognition

        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(
                source
            )  # listen for 1 second to calibrate the energy threshold for ambient noise levels
            print("Say something!")
            self.audio = self.r.listen(source, None)

        try:
            response.recognized_speech = self.r.recognize_google(self.audio)
            response.success = True
            return response
        except sr.UnknownValueError:
            response.recognized_speech = "Google Speech Recognition could not understand audio. Please try again!"
            response.success = False
            return response
        except sr.RequestError as e:
            response.recognized_speech = "Could not request results from Google Speech Recognition service; {0}. Please try again!".format(
                e
            )
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
