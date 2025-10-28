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
    ping1d_data = Node(package='bluerobotics_sonar',
                       executable='ping1d',
                       parameters=[{
    # Device connection
    'device': '/dev/ttyUSB0',    # serial path or IPv4 for UDP
    'baudrate': 115200,          # serial baud; acts as UDP port if device is IPv4

    # Sonar configuration
    'gain_setting': 0,
    'mode_auto': 0,              # 0=manual, 1=auto
    'ping_enable': True,
    'pings_per_second': 30,      # preferred way to set rate (1-50)
    'ping_interval': -1,         # -1 -> derive from pings_per_second
    'scan_start': 0.0,           # meters
    'scan_length': 1.0,          # meters
    'speed_of_sound': 1500,      # m/s

    # ROS topics
    'topic': '/sonar/ping1d/data',
    'frame_id': 'ping1d',
}],
                       output='screen')

    return LaunchDescription([ping1d_data])
