import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

from pixelbot_msgs.srv import MotorsMovement
from pixelbot_msgs.srv import DisplayEmotion

import time
from adafruit_servokit import ServoKit


class Motors(Node):

    # Defines
    RIGHT_ARM, LEFT_ARM, RIGHT_ANTENNA, LEFT_ANTENNA = 0, 1, 2, 3
    MOTOR_STRING_TO_DEFINE = {"right_arm": RIGHT_ARM,
                              "left_arm": LEFT_ARM,
                              "right_antenna": RIGHT_ANTENNA,
                              "left_antenna": LEFT_ANTENNA}

    def __init__(self):
        super().__init__('pixelbot_motors_node')

        # Service to perform the desired movement of given motors
        self.motors_movement_srv = self.create_service(MotorsMovement, 'motors_movement', self.motors_movement_callback)

        # Service to perform walking movement
        self.walking_movement_srv = self.create_service(Empty, 'walking_movement', self.walking_movement_callback)

        # Service to perform hand waving movement
        self.hand_waving_srv = self.create_service(Empty, 'hand_waving', self.hand_waving_callback)

        # Service to perform emotion with antennae movements
        self.emotion_antennae_movement_srv = self.create_service(DisplayEmotion, 'emotion_antennae_movement', self.emotion_antennae_movement_callback)

        # Servokit object
        self.kit = ServoKit(channels=16)

    
    def motors_movement_callback(self, request, response):
        """
        Service handler performing the desired movemement
        of given motors.

        :param request: See MotorsMovement service definition.
        :param response: See MotorsMovement service definition.
        """

        # Check request arguments
        if len(request.body_parts) != len(request.angles):
            self.get_logger().info("Error: The input lists should have the same length!")
            response.success = False
            return response

        for body_part, angle in zip(request.body_parts, request.angles):
            if body_part not in self.MOTOR_STRING_TO_DEFINE.keys():
                self.get_logger().info("Error: requested body part does not exist!")
                response.success = False
                return response

            if not 0 <= angle <= 180:
                self.get_logger().info("Error: requested angle should be in [0; 180]!")
                response.success = False
                return response

        # Perform the requested movements if the arguments are valid
        for body_part, angle in zip(request.body_parts, request.angles):
            self.kit.servo[self.MOTOR_STRING_TO_DEFINE[body_part]].angle = angle

        response.success = True
        return response

    def walking_movement_callback(self, request, response):
        """
        Service handler performing a walking gesture.
        Assume the initial motor position is a 90 degrees i.e.
        the arms are in a vertical position.
        """

        start_time = time.time()

        while time.time() - start_time < 4:
            self.kit.servo[self.RIGHT_ARM].angle = 135
            self.kit.servo[self.LEFT_ARM].angle = 135
            time.sleep(0.5)
            self.kit.servo[self.RIGHT_ARM].angle = 45
            self.kit.servo[self.LEFT_ARM].angle = 45
            time.sleep(0.5)

        self.kit.servo[self.RIGHT_ARM].angle = 90
        self.kit.servo[self.LEFT_ARM].angle = 90

        return response

    def hand_waving_callback(self, request, response):
        """
        Service handler performing a hand waving gesture with the right arm.
        Assume the initial motor position is a 90 degrees i.e.
        the arm is in a vertical position.
        """
        
        self.kit.servo[self.RIGHT_ARM].angle = 165
        time.sleep(0.5)

        start_time = time.time()

        while time.time() - start_time < 4:
            self.kit.servo[self.RIGHT_ARM].angle = 175
            time.sleep(0.5)
            self.kit.servo[self.RIGHT_ARM].angle = 155
            time.sleep(0.5)

        self.kit.servo[self.RIGHT_ARM].angle = 90

        return response

    def emotion_antennae_movement_callback(self, request, response):
        """
        Service handler performing antennae movements for emotions.

        :param request: See DisplayEmotion service definition.
        :param response: See DisplayEmotion service definition.
        """

        # Check request argument
        if request.desired_emotion not in ["happy", "angry", "sad", "surprise"]:
            self.get_logger().info("Error: requested emotion does not exist!")
            response.success = False
        
        # Perform the requested movements if the arguments are valid
        else:
            response.success = True

            if request.desired_emotion == "happy":
                start_time = time.time()
                while time.time() - start_time < 3:
                    self.kit.servo[self.RIGHT_ANTENNA].angle = 100
                    self.kit.servo[self.LEFT_ANTENNA].angle = 100
                    time.sleep(0.5)
                    self.kit.servo[self.RIGHT_ANTENNA].angle = 80
                    self.kit.servo[self.LEFT_ANTENNA].angle = 80
                    time.sleep(0.5)

            elif request.desired_emotion == "angry":
                start_time = time.time()
                while time.time() - start_time < 3:
                    self.kit.servo[self.RIGHT_ANTENNA].angle = 80
                    self.kit.servo[self.LEFT_ANTENNA].angle = 100
                    time.sleep(0.5)
                    self.kit.servo[self.RIGHT_ANTENNA].angle = 100
                    self.kit.servo[self.LEFT_ANTENNA].angle = 80
                    time.sleep(0.5)

            elif request.desired_emotion == "sad":
                self.kit.servo[self.RIGHT_ANTENNA].angle = 10
                self.kit.servo[self.LEFT_ANTENNA].angle = 170
                time.sleep(3)

            elif request.desired_emotion == "surprise":
                self.kit.servo[self.RIGHT_ANTENNA].angle = 70
                self.kit.servo[self.LEFT_ANTENNA].angle = 110
                time.sleep(3)

            self.kit.servo[self.RIGHT_ANTENNA].angle = 90
            self.kit.servo[self.LEFT_ANTENNA].angle = 90

        return response


def main(args=None):
    rclpy.init(args=args)

    motors_node = Motors()

    rclpy.spin(motors_node)

    motors_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
