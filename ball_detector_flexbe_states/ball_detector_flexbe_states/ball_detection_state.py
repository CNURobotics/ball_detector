#!/usr/bin/env python3

###############################################################################
#  Copyright (c) 2022
#  Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
#  Christopher Newport University
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
#       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#       "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#       LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#       FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#       COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#       INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#       BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#       LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#       LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#       WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#       POSSIBILITY OF SUCH DAMAGE.
###############################################################################

import traceback

import rclpy
from rclpy.duration import Duration
import numpy as np

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped, Quaternion
from ball_detector_msgs.msg import Ball, BallList

class BallDetectionState(EventState):

    '''
    Changes output based on balls detected and published on topic
    --  balls_topic         string      topic name of the list of balls
    --  min_radius_pixels   double      minimum radius to trigger output

    #> ball_detected        Ball        Detected ball data
    #> goal                 PoseStamped target pose
    #> ball_label           string      color of the ball


    <= done        A ball has been detected with valid color
    <= invalid     Invalid ball detection message
    <= unavailable Ball detection topic is unavailable
    '''

    def __init__(self, balls_topic ='/ball_detector/balls', min_radius_pixels=-1.0 ):
        super(BallDetectionState, self).__init__(outcomes = ['unavailable', 'invalid', "done"],
                                                 output_keys=['ball_detected', 'goal', 'ball_label'])

        self._balls_topic = balls_topic
        self._msg_type = BallList
        self._min_radius_pixels = min_radius_pixels*0.999999
        self._return  = None
        self._best_ball = None

        self._target_time = Duration(seconds=1)

        self._outcome = None

        self.label_to_return_mapping  = {"blue":         "blue_ball",
                                         "bluish":       "blue_ball",
                                         "bluish-green": "green_ball",
                                         "green":        "green_ball",
                                         "redish-green": "green_ball",
                                         "redish":       "red_ball",
                                         "red":          "red_ball"
                                        }

        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._balls_topic, self.name))

    def _connect(self):
        try:
            self._sub = ProxySubscriberCached({self._balls_topic: self._msg_type})
            self._connected = True
            return True
        except Exception as e:
            return False

    def execute(self, userdata):
        if self._outcome:
            # If blocked
            return self._outcome

        if not self._connected:
            Logger.logwarn("Not connected to ball detector")
            userdata.ball_detected = None
            userdata.goal = None
            userdata.ball_label = ""
            self._outcome = 'unavailable'
            return self._outcome

        if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._target_time.nanoseconds and self._best_ball is None:
            Logger.logwarn("Received no ball detector messages")
            self._outcome = 'invalid'
            return self._outcome

        if self._sub.has_msg(self._balls_topic) and self._best_ball is None:
            best_ball = None
            max_radius = self._min_radius_pixels
            balls_list = self._sub.get_last_msg(self._balls_topic)
            self._sub.remove_last_msg(self._balls_topic)
            for ball in balls_list.balls:
                if ball.radius > max_radius:
                    best_ball = ball
                    max_radius = ball.radius

            if best_ball is not None:
                Logger.loginfo('Detected ball %s' % str(best_ball))
                userdata.ball_detected = best_ball

                # Pass along the ball location as a possible goal location for future
                userdata.goal = PoseStamped()
                userdata.goal.header = balls_list.header

                # Assumes default orientation 
                userdata.goal.pose.position = best_ball.position

                try:
                    userdata.ball_label = self.label_to_return_mapping[best_ball.label]
                    self._best_ball = best_ball
                    Logger.loginfo('Found %s' % userdata.ball_label)
                    self._outcome = 'done'
                except:
                    self._outcome = 'invalid'
                    Logger.logwarn(f'Invalid ball label <{best_ball.label}>')

                # We have detected some ball, but might be invalid color
                return self._outcome

        if self._best_ball is not None:
            self._outcome = 'done'
            return self._outcome

        return None

    def on_enter(self, userdata):
        self._start_time = self._node.get_clock().now()
        self._best_ball = None
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._balls_topic)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._balls_topic)

        if self._connected and self._sub.has_msg(self._balls_topic):
            # Make sure we only get fresh messages
            self._sub.remove_last_msg(self._balls_topic)


    # def quaternion_from_euler(self, roll, pitch, yaw):
    #     """
    #     Converts euler roll, pitch, yaw to quaternion
    #     """
    #     cy = np.cos(yaw * 0.5)
    #     sy = np.sin(yaw * 0.5)
    #     cp = np.cos(pitch * 0.5)
    #     sp = np.sin(pitch * 0.5)
    #     cr = np.cos(roll * 0.5)
    #     sr = np.sin(roll * 0.5)
    #
    #     x = cy * cp * sr - sy * sp * cr
    #     y = sy * cp * sr + cy * sp * cr
    #     z = sy * cp * cr - cy * sp * sr
    #     w = cy * cp * cr + sy * sp * sr
    #
    #     quaternion = Quaternion(x=x, y=y, z=z, w=w)
    #
    #     return quaternion
