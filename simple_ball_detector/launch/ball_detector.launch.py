# Copyright 2022, CHRISLab, Christopher Newport University
# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import argparse
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription()
    ld.add_action( SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action( DeclareLaunchArgument(
                    'use_sim_time', default_value='false',
                    description='Use simulation (Gazebo) clock if true'))

    parser = argparse.ArgumentParser(description='simple_ball_detector')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='ball_detector')

    args, unknown = parser.parse_known_args(sys.argv[4:])
    node_name = args.node_name
    print(f" Using name = ({node_name})")

    config = os.path.join(
          get_package_share_directory('simple_ball_detector'),
          'config',
          'ball_detector_params.yaml'
          )
    try:
        with open(config, 'rt') as fin:
            params = yaml.safe_load(fin)['simple_ball_detector']['ros__parameters']

        params['filter_map_file'] = os.path.join(
              get_package_share_directory('simple_ball_detector'),
              'config',
              params['filter_map_file']
              )
        params['use_sim_time'] = use_sim_time

    except Exception as exc:
        print("Failed to load ball detector parameters!")
        print(exc)
        params = {}

    ld.add_action(Node(
        package='simple_ball_detector', executable='simple_ball_detector', output='screen',
        name=node_name,
        # namespace=ns,
        parameters=[params],
        remappings=[('image_in', '/camera/image_raw'),
                    ('info_in',   '/camera/camera_info'),
                    ('image_out', '/ball_detector/image'),
                    ('info_out',  '/ball_detector/camera_info'),
                    ('detected_balls', '/ball_detector/balls'),
                    ('markers_out', '/ball_detector/ball_markers'),
                   ]
        ))

    return ld
