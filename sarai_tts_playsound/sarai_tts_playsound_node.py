import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

from pixelbot_msgs.srv import SetVoiceAlteration
from pixelbot_msgs.srv import SetSpeech
from pixelbot_msgs.srv import SetLanguage

import pyttsx3
import numpy as np
import scipy.io.wavfile as sc
from gtts import gTTS
from pydub import AudioSegment
from playsound import playsound

from ament_index_python import get_package_share_directory


class TTS_Playsound(Node):

    def __init__(self):
        super().__init__('sarai_tts_playsound_node')

        # Service allowing the robot to speak
        self.speak_srv = self.create_service(SetSpeech, 'speak', self.speak_callback)

        # Service to specify if the voice should be altered or not
        self.voice_alteration_srv = self.create_service(SetVoiceAlteration, 'change_voice_alteration', self.change_voice_alteration_callback)

        # Service allowing to change the language to be spoken
        self.language_srv = self.create_service(SetLanguage, 'change_language', self.change_language_callback)

        # Service allowing the robot to play a happy sound
        self.play_happy_sound_srv = self.create_service(Empty, 'play_happy_sound', self.play_happy_sound_callback)

        # Create the pyttsx3 object and perform initial configuration
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('volume', 1.0)
        self.tts_engine.setProperty('rate', 125)
        self.tts_engine.setProperty('voice', "english")

        # If the voice should sound robotic like
        self.altered_voice = True

        # Language to use
        self.language = 'en'

    def speak_callback(self, request, response):
        """
        Service handler allowing the robot to speak.

        :param request: See SetSpeech service definition.
        """
        
        if self.altered_voice:
            # Create an audio file containing the speech to alter
            tts = gTTS(request.message, lang=self.language)
            tts.save('voiceToAlter.mp3')
            sound = AudioSegment.from_mp3('voiceToAlter.mp3')
            sound.export('voiceToAlter.wav', format='wav')

            # Alter the previously created audio file
            self.diodeRingModulator('voiceToAlter.wav')

            # Play the altered audio file
            sound = AudioSegment.from_wav("alteredVoice.wav")
            sound.export("alteredVoice.mp3", format='mp3')
            playsound("alteredVoice.mp3")

        else:
            self.tts_engine.say(request.message)
            self.tts_engine.runAndWait()

        return response

    def diode(self, signalArray):
        """
        Apply an approximation of the diode non linearity model.
        The approximation of the diode is described by:

               { 0           , if x <= 0
        f(x) = |
               { 0.1*x^(1.7) , if x > 0

        :param signalArray: The signal to be altered.
        :return: The signal altered by the diode non linearity.
        """

        diodeArray = [0.1*(x**1.7) if x > 0 else 0.0 for x in signalArray]

        return np.array(diodeArray)

    def diodeRingModulator(self, intputFileName):
        """
        Simulate a diode ring modulator electrical circuit.
        Alter the audio file containing the text to be said to make it sounds
        more robotic like.

        :param intputFileName: Name of the audio file containing the voice to
                               be altered.
        """

        # Read the audio file
        [_, data] = sc.read(intputFileName)

        # Get maximum absolute value of input signal
        maxVal = max(abs(data))

        # Scale down the input signal
        scaledData = data/maxVal

        # Create carrier signal
        fCarrier = 500
        t = np.linspace(0, len(scaledData), len(scaledData))
        carrier = np.sin(2*np.pi*fCarrier*t)

        # Compute output of the ring modulator circuit
        topFirst = carrier + 0.5*scaledData
        top = self.diode(topFirst) + self.diode(-topFirst)

        bottomFirst = carrier - 0.5*scaledData
        bottom = self.diode(bottomFirst) + self.diode(-bottomFirst)

        output = top - bottom

        # Scale back
        output = 5*maxVal*output

        # Save the signal
        sc.write('alteredVoice.wav', 22050, np.int16(output))

    def change_voice_alteration_callback(self, request, response):
        """
        Service handler to specify if the voice should be altered or not

        :param request: See SetVoiceAlteration service definition.
        :param response: See SetVoiceAlteration service definition.
        """

        self.altered_voice = request.is_voice_altered

        return response

    def change_language_callback(self, request, response):
        """
        Service handler allowing to change the language to be spoken.

        :param request: See SetLanguage service definition.
        :param response: See SetLanguage service definition.
        """

        if request.language not in ["french", "english"]:
            response.success = False
        else:
            response.success = True
            if request.language == "french":
                self.language = "fr"
                self.tts_engine.setProperty('voice', "french")
            if request.language == "english":
                self.language = "en"
                self.tts_engine.setProperty('voice', "english")

        return response

    def play_happy_sound_callback(self, request, response):
        """
        Service handler allowing to play a happy sound.

        :param request: See Empty service definition.
        :param response: See Empty service definition.
        """

        sound_path = get_package_share_directory('sarai_tts_playsound') + "/sound/happy.mp3"
        playsound(sound_path)

        return response


def main(args=None):
    rclpy.init(args=args)

    tts_playsound_node = TTS_Playsound()

    rclpy.spin(tts_playsound_node)

    tts_playsound_node.tts_engine.stop()
    tts_playsound_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
