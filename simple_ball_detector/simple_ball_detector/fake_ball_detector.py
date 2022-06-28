#!/usr/bin/env python3

# Based on simple_ball_detector

# Import the necessary libraries
import numpy as np
import cv2 # OpenCV library
import rclpy # Python Client Library for ROS 2
import yaml

from copy import deepcopy

from rclpy.node import Node # Handles the creation of nodes
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from ball_detector_msgs.msg import Ball, BallList
from simple_ball_detector.simple_ball_detector import SimpleBallDectector

from pynput.keyboard import KeyCode
from pynput import keyboard

class FakeBallDectector(SimpleBallDectector):
    """
    Create an FakeBallDectector class, which is a subclass of the BallDetector class.
    """
    def __init__(self, name="fake_ball_detector"):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        self._augmented_image = None

        super().__init__(name)
        self.declare_parameter('fake_ball_key_map_file', '')  # color name to HSV limits
        self._fake_ball_key_map = None
        key_map_file = self.get_parameter('fake_ball_key_map_file').get_parameter_value().string_value
        if key_map_file != '':
            try:
                with open(key_map_file, 'rt') as fin:
                    fake_ball_key_map = yaml.safe_load(fin)['fake_ball_key_map']

                self._fake_ball_key_map = {}
                for key, key_data in fake_ball_key_map.items():
                    try:
                        ball = Ball()
                        ball.id = key_data['id']
                        ball.label = key_data['label']
                        ball.rgb_color = key_data['rgb_color']
                        ball.center.x = float(key_data['center']['col'])
                        ball.center.y = float(key_data['center']['row'])
                        # use ideal projection to calculate radius in image pixels
                        ball.radius = self.average_focal_length*self.true_ball_radius/float(key_data['range'])
                        ball.position = self.project_ball((ball.center.x, ball.center.y), ball.radius, 480, 640)

                        self._fake_ball_key_map[key] = ball
                    except Exception as exc:
                        self.get_logger().error(f" Error processing filter map for {key} : {key_data}!\n{exc}")

            except Exception as exc:
                self.get_logger().error(f" Error processing filter map {key_map_file}!\n{exc}")

        if self._fake_ball_key_map is None:
            self.get_logger().error(' No fake ball key map file!')

        self._ball_list = None
        self._ball_key_char = None
        self._run = True

    def _image_callback(self, data):
        """
        Callback function just used to set the augmented image data
        """
        try:
            image_timestamp = Time(seconds=data.header.stamp.sec, nanoseconds=data.header.stamp.nanosec, clock_type=rclpy.clock.ClockType.ROS_TIME)
            #self.get_logger().info(f"timestamp check p={image_timestamp} {self._prior_processing_timestamp} {self._prior_data_timestamp}")
            elapsed_nanoseconds = (image_timestamp - self._prior_processing_timestamp).nanoseconds
            # if elapsed_nanoseconds < 0:
            #     self.get_logger().info(f'Skipping image - wait for newer data from after we started prior processing : {elapsed_nanoseconds/1e9:.6f} seconds')
            #     self._skipped_images += 1
            #     return

            elapsed_seconds = (image_timestamp - self._prior_data_timestamp).nanoseconds/1e9
            if elapsed_seconds < self.desired_update_period:
                self.get_logger().info(f'Skipping image - too close to prior data {elapsed_seconds:.6f} < {self.desired_update_period}')
                self._skipped_images += 1
                return

            # Log when we start so we can avoid processing too many old messages that are already in the queue
            self._prior_processing_timestamp = self.get_clock().now()
            elapsed_seconds = (image_timestamp - self._prior_processing_timestamp).nanoseconds/1e9
            if self._skipped_images > 0:
                self.get_logger().info(f"Skipped {self._skipped_images} prior images - this image is {elapsed_seconds:.6f} seconds old!")

            self._skipped_images = 0

        except Exception as exc:
            self.get_logger().error(f' time stamp check error:\n{exc}')
            self.get_logger().info(f"         timestamp check p={image_timestamp} {self._prior_processing_timestamp} {self._prior_data_timestamp}")

        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self._br.imgmsg_to_cv2(data, "bgr8")

            if self._image_publisher:
                #self.get_logger().info(f' update augmented image ...')
                self._augmented_image = current_frame.copy()
        except Exception as exc:
            self.get_logger().error(f' image conversion error creating augmented image :\n{exc}')


    def on_press(self, key):
        #self.get_logger().error(f' key press = {key} is type {type(key)}')
        char = getattr(key, 'char', None)
        try:

            if isinstance(char, str):
                if char in self._fake_ball_key_map:
                    self._ball = deepcopy(self._fake_ball_key_map[char])
                    self._ball_key_char = char
                    self.get_logger().error(f' key press ={self._ball_key_char} found\n {self._ball.id} {self._ball.label}')
                elif char == 'q':
                    self._run = False
                    self.get_logger().info(f' Shutdown requested ...')
                elif char == 'c':
                    self._ball_key_char = char
                    self._ball = None

                #else:
                #    self.get_logger().error(f' ignoring key press = {key}:{char} not in ball map')

            # else:
            #     self.get_logger().error(f' key press = {key}:{char} is type {type(char)}')

        except Exception as exc:
            self.get_logger().error(f' error on key press = {key}')
            self.get_logger().error(str(exc))


    def on_release(self, key):
        char = getattr(key, 'char', None)
        try:
            if isinstance(char, str):
                if char == self._ball_key_char:
                    if self._ball is not None:
                        if self._ball_list:
                            self.get_logger().error(f' ignoring key release ={char} with active ball in list!')
                            self.get_logger().error(f'  {self._ball_list}')
                        else:
                            ball_list = BallList()
                            ball_list.header.stamp = self.get_clock().now().to_msg()
                            ball_list.header.frame_id = self.camera_frame  # default frame
                            ball_list.balls.append(self._ball)
                            self.get_logger().error(f' process new ball list on key release ={char}!')
                            self._ball_list = ball_list
                    elif char == 'c':
                            ball_list = BallList()
                            ball_list.header.stamp = self.get_clock().now().to_msg()
                            ball_list.header.frame_id = self.camera_frame  # default frame
                            self.get_logger().error(f' clear existing ball list on key release ={char}!')
                            self._ball_list = ball_list
                    else:
                        self.get_logger().error(f' ignoring key release = {char} - no ball defined!')
                else:
                    if self._ball_key_char is not None:
                        self.get_logger().error(f' ignoring key release = {char} - mismatched {char} and {self._ball_key_char} - clear all!')
                    self._ball_list = None
                    self._ball = None
                    self._ball_key_char = None

            #else:
            #    self.get_logger().error(f' ignoring key release = {char} {type(char)}')
        except Exception as exc:
            self.get_logger().error(f' error processing ball on key release = {key}')
            self.get_logger().error(str(exc))
            self._ball_list = None
            self._ball = None
            self._ball_key_char = None


    def spin(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while rclpy.ok() and listener.running and self._run:
                rclpy.spin_once(self, timeout_sec=0.005)

                if self._ball_list is not None:

                    if self._image_publisher is not None and self._augmented_image is not None:
                        augmented_image = self._augmented_image.copy()
                    else:
                        augmented_image = None

                    try:
                        self._process_balls_list(self._ball_list, augmented_image )

                        # Publish the ball list a few times
                        start_time = self.get_clock().now()
                        pub_time = start_time
                        for i in range(3): # 4 times total including _process_balls_list
                            rclpy.spin_once(self, timeout_sec=0.005)
                            if self.get_clock().now().nanoseconds - start_time.nanoseconds > self.desired_update_period:
                                self._balls_publisher.publish(self._ball_list)
                                pub_time = self.get_clock().now()

                        # # Reset visible balls
                        # self._ball_list.balls = []
                        # rclpy.spin_once(self, timeout_sec=0.005)
                        #
                        # if self._image_publisher is not None and self._augmented_image is not None:
                        #     augmented_image = self._augmented_image.copy()
                        # rclpy.spin_once(self, timeout_sec=0.005)
                        #
                        # self._process_balls_list(self._ball_list, augmented_image )
                        # rclpy.spin_once(self, timeout_sec=0.005)
                        #
                        # start_time = self.get_clock().now()
                        # pub_time = start_time
                        # for i in range(1): # publish empty twice
                        #     rclpy.spin_once(self, timeout_sec=0.005)
                        #     if self.get_clock().now().nanoseconds - start_time.nanoseconds > self.desired_update_period:
                        #         self._balls_publisher.publish(self._ball_list)
                        #         pub_time = self.get_clock().now()


                    except Exception as exc:
                        self.get_logger().error(f' error processing ball list!\n{exc}')
                    finally:
                        # Clear ball list to begin accepting new key presses
                        self._ball_list = None
                        self._ball = None
                        self._ball_key_char = None
                        self._dump_ball_key_map()
                        rclpy.spin_once(self, timeout_sec=0.005)


        self.get_logger().error(f' Shutting down fake ball detector node')
        self.destroy_node()

    def _dump_parameters(self):
        # Print parameter data
        super()._dump_parameters()
        self._dump_ball_key_map()

    def _dump_ball_key_map(self):
        if self._fake_ball_key_map:
            self.get_logger().info(80*"=")
            self.get_logger().info(f"Fake ball key mapping:")
            for item, ball in self._fake_ball_key_map.items():
                self.get_logger().info(f"       '{item}' : {str(ball.id):>5s} {ball.label:^10s} "
                                       f"rgb({ball.rgb_color[0]}, {ball.rgb_color[1]}, {ball.rgb_color[2]})"
                                       f" px,py=({ball.center.x:.1f}, {ball.center.y:.1f}) rp={ball.radius:.3f}")
            self.get_logger().info(80*"_")
            self.get_logger().info("    Press 'q' to quit; 'c' to clear and publish empty list")
            self.get_logger().warning("   WARNING: This captures all key presses regardless of window focus!")
            self.get_logger().info(80*"=")

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    ball_detector = FakeBallDectector()
    ball_detector._dump_parameters()

    # Custom spinner for keyboard input
    ball_detector.spin()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
