#! /usr/bin/env python3

#-----------------------------------------------------------------------------------
# MIT License

# Copyright (c) 2025 ItsKalvik

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#-----------------------------------------------------------------------------------

from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    ping360_data = Node(package='bluerobotics_sonar',
                        executable='ping360',
                        parameters=[{
    # Device connection
    'device': '/dev/ttyUSB0',    # serial path or IPv4 for UDP
    'baudrate': 115200,          # serial baud; acts as UDP port if device is IPv4

    # Sonar scanning / firmware-tuned settings
    'mode': 1,
    'gain_setting': 0,
    'transmit_frequency': 750,   # kHz
    'start_angle': 0,            # grads (0-399)
    'stop_angle': 399,           # grads (0-399)
    'num_steps': 1,              # grads per step
    'delay': 0,                  # ms between pings
    'motor_off': False,
    'speed_of_sound': 1500,      # m/s
    'scan_threshold': 150,
    'range': 1.0,                # meters
    'offset': 20,                # bins ignored near transducer
    'window_size': 15,           # peak/edge window for range finder

    # ROS topics
    'topic': '/sonar/ping360/data',
    'frame_id': 'ping360',
}],
                        output='screen')

    return LaunchDescription([ping360_data])