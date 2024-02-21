import rclpy
from rclpy.node import Node

from pixelbot_msgs.srv import DisplayEmotion
from pixelbot_msgs.srv import DisplayLocation

from ament_index_python import get_package_share_directory

import time
import pygame
from pygame.locals import *


class Display(Node):

    # Defines
    BLINK_NUMBER_IMAGES = 5
    HAPPY_NUMBER_IMAGES = 6
    ANGRY_NUMBER_IMAGES = 5
    SAD_NUMBER_IMAGES = 5
    SURPRISE_NUMBER_IMAGES = 6
    LOCATIONS_NUMBER_IMAGES = 4
    BLINKING, FORWARD, BACKWARD = 0, 1, 2
    AVAILABLE_EMOTIONS = ["happy", "angry", "sad", "surprise"]
    STRING_LOCATIONS_TO_IDX = {"flat": 0, "ski": 1, "nasa": 2, "judge": 3}
    COUNT_CHANGE_IMAGE_OF_SEQUENCE = 2
    COUNT_TRIGGER_BLINKING_SEQUENCE = 180

    def __init__(self):
        super().__init__('pixelbot_display_node')

        # Timer callback for the display
        timer_period = 1.0 / 30.0 # 30 fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Service to perform desired emotion
        self.emotion_srv = self.create_service(DisplayEmotion, 'display_emotion', self.emotion_callback)

        # Service to display desired location
        self.location_srv = self.create_service(DisplayLocation, 'display_location', self.location_callback)

        # Initialize Pygame
        pygame.init()

        # Create window
        self.window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        # Do not show computer mouse
        pygame.mouse.set_visible(False)

        # Get screen size
        infoObject = pygame.display.Info()
        dimensions = (infoObject.current_w, infoObject.current_h)

        # Load blinking image sequence
        self.blink_sequence = self.load_sequence("blink", self.BLINK_NUMBER_IMAGES, dimensions)
        self.blink_sequence += reversed(self.blink_sequence)
        self.blink_sequence.remove(self.blink_sequence[len(self.blink_sequence)//2])

        # Load happy image sequences
        self.happy_forward_sequence = self.load_sequence("happy", self.HAPPY_NUMBER_IMAGES, dimensions)
        self.happy_backward_sequence = list(reversed(self.happy_forward_sequence))

        # Load angry image sequences
        self.angry_forward_sequence = self.load_sequence("angry", self.ANGRY_NUMBER_IMAGES, dimensions)
        self.angry_backward_sequence = list(reversed(self.angry_forward_sequence))

        # Load sad image sequences
        self.sad_forward_sequence = self.load_sequence("sad", self.SAD_NUMBER_IMAGES, dimensions)
        self.sad_backward_sequence = list(reversed(self.sad_forward_sequence))

        # Load surprise image sequences
        self.surprise_forward_sequence = self.load_sequence("surprise", self.SURPRISE_NUMBER_IMAGES, dimensions)
        self.surprise_backward_sequence = list(reversed(self.surprise_forward_sequence))

        # Load location images
        self.location_sequence = self.load_sequence("locations", self.LOCATIONS_NUMBER_IMAGES, dimensions)

        # Initial image on window
        self.window.blit(self.blink_sequence[0], (0, 0))

        # General purpose variables
        self.should_display = True
        self.current_emotion = "blinking"
        self.previous_emotion = "blinking"
        self.next_emotion = None
        self.emotion_type = self.BLINKING
        self.previous_emotion_type = self.BLINKING
        self.current_sequence = self.blink_sequence
        self.counter_change_image_of_sequence = 0
        self.emotion_sequence_idx = 0
        self.is_performing_emotion = False
        self.counter_trigger_blinking_sequence = 0
        self.start_time = time.time()
        self.desired_location = None

        self.current_emotion_to_emotion_sequence = {("happy", self.FORWARD): self.happy_forward_sequence,
                                                    ("happy", self.BACKWARD): self.happy_backward_sequence,
                                                    ("angry", self.FORWARD): self.angry_forward_sequence,
                                                    ("angry", self.BACKWARD): self.angry_backward_sequence,
                                                    ("sad", self.FORWARD): self.sad_forward_sequence,
                                                    ("sad", self.BACKWARD): self.sad_backward_sequence,
                                                    ("surprise", self.FORWARD): self.surprise_forward_sequence,
                                                    ("surprise", self.BACKWARD): self.surprise_backward_sequence,}

    def load_sequence(self, sequence_name, number_images, dim):
        """
        Allow to load the sequence of images composing a facial animation
        (blinking animation, smilling animation etc). The images are scaled
        to the screen size.

        :param sequence_name: String indicating the facial animation to load.
        :param number_images: Integer representing the number of images in 
                              the desired sequence.
        :param dim: Integer tuple containing the screen height and width.
        :return: List of scaled images composing a facial animation.
        """

        sequence = []
        images_path = get_package_share_directory('pixelbot_display') + "/imgs/"

        for i in range(number_images):
            sequence.append(pygame.image.load(images_path + sequence_name + str(i) + ".png").convert_alpha())

        sequence = [pygame.transform.scale(img, dim) for img in sequence]

        return sequence

    def play_sequence(self, emotion_sequence):
        """
        Do the transition between two images of a sequence.

        :param emotion_sequence: List of images composing an emotion sequence.
        """

        self.counter_change_image_of_sequence += 1

        if self.counter_change_image_of_sequence == self.COUNT_CHANGE_IMAGE_OF_SEQUENCE and \
        self.emotion_sequence_idx < len(emotion_sequence) - 1:

            self.emotion_sequence_idx += 1
            self.counter_change_image_of_sequence = 0

            self.window.blit(emotion_sequence[self.emotion_sequence_idx], (0, 0))

    def stop_pygame(self):
        """
        Stop Pygame.
        """

        pygame.quit()

    def timer_callback(self):
        """
        Main loop of the display.
        """

        # Event catching
        for event in pygame.event.get():
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                self.should_display = False

        # Reset variables for emotion display
        if self.previous_emotion != self.current_emotion or self.previous_emotion_type != self.emotion_type:
            self.counter_change_image_of_sequence = 0
            self.emotion_sequence_idx = 0
            self.previous_emotion = self.current_emotion
            self.previous_emotion_type = self.emotion_type

        # Location display
        if self.desired_location is not None:
            if time.time() - self.start_time > 8:
                self.desired_location = None
                self.counter_trigger_blinking_sequence = 0
                self.window.blit(self.blink_sequence[0], (0, 0))

        # Emotions display 
        elif self.emotion_type == self.BLINKING:
            self.counter_trigger_blinking_sequence += 1

            if self.counter_trigger_blinking_sequence == self.COUNT_TRIGGER_BLINKING_SEQUENCE:
                self.is_performing_emotion = True

            if self.is_performing_emotion:
                self.play_sequence(self.blink_sequence)

                if self.emotion_sequence_idx == len(self.blink_sequence) - 1:
                    self.counter_trigger_blinking_sequence = 0
                    self.emotion_sequence_idx = 0
                    self.is_performing_emotion = False

        elif self.emotion_type == self.FORWARD:
            self.current_sequence = self.current_emotion_to_emotion_sequence[(self.current_emotion, self.FORWARD)]
            if self.counter_change_image_of_sequence == 0 and self.emotion_sequence_idx == 0:
                self.start_time = time.time()

            self.play_sequence(self.current_sequence)

            if time.time() - self.start_time > 3:
                self.emotion_type = self.BACKWARD

        elif self.emotion_type == self.BACKWARD:
            self.current_sequence = self.current_emotion_to_emotion_sequence[(self.current_emotion, self.BACKWARD)]
            self.play_sequence(self.current_sequence)

            if self.emotion_sequence_idx == len(self.current_sequence) - 1:
                self.emotion_type = self.BLINKING
                self.is_performing_emotion = False

        # Emotion requests handling
        if self.next_emotion is not None and not self.is_performing_emotion:
            self.is_performing_emotion = True
            self.emotion_type = self.FORWARD
            self.current_emotion = self.next_emotion
            self.next_emotion = None

        pygame.display.flip()

    def emotion_callback(self, request, response):
        """
        Service handler allowing to perform a desired emotion.

        :param request: See DisplayEmotion service definition.
        :param response: See DisplayEmotion service definition.
        """

        if request.desired_emotion in self.AVAILABLE_EMOTIONS:
            self.next_emotion = request.desired_emotion
            response.success = True
        else:
            self.get_logger().info("Error: requested emotion does not exist!")
            response.success = False

        return response

    def location_callback(self, request, response):
        """
        Service handler allowing to display the desired location.

        :param request: See DisplayLocation service definition.
        :param response: See DisplayLocation service definition.
        """

        if request.desired_location in self.STRING_LOCATIONS_TO_IDX.keys():
            self.desired_location = request.desired_location
            self.window.blit(self.location_sequence[self.STRING_LOCATIONS_TO_IDX[self.desired_location]], (0, 0))
            self.start_time = time.time()
            response.success = True
        else:
            self.get_logger().info("Error: requested location does not exist!")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)

    display_node = Display()

    while rclpy.ok() and display_node.should_display:
        rclpy.spin_once(display_node)

    display_node.stop_pygame()

    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
