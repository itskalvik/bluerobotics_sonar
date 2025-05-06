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

from brping import Ping1D
from brping.definitions import PING1D_PROFILE
from bluerobotics_sonar_msgs.msg import SonarPing1D

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class Ping1DNode(Node):

    def __init__(self, node_name='ping1d'):
        super().__init__(node_name)

        # Declare Parameters
        params = {
            'gain_setting': [0, int],
            'mode_auto': [0, int],
            'ping_enable': [True, bool],
            'ping_interval': [100, int],
            'scan_start': [0.0, float],
            'scan_length': [1.0, float], 
            'speed_of_sound': [1500, int],
            'device': ['/dev/ttyUSB0', int],
            'baudrate': [115200, int],
            'topic': ['sonar/ping1d/data', str],
            'frame_id': ['ping1d', str],
        }

        for param, [value, dtype] in params.items():
            self.declare_parameter(param, value)
            exec(f"self.{param}:dtype = self.get_parameter(param).value")
            self.get_logger().info(f'{param}: {value}')

        # Handle parameter updates
        self.param_handler_ptr_ = self.add_on_set_parameters_callback(
            self.set_param_callback)

        # Init and configure Ping1D sonar
        self.sonar = Ping1D()
        if len(self.device.split('.')) == 4:
            self.sonar.connect_udp(self.device, self.baudrate)
        else:
            self.sonar.connect_serial(self.device, self.baudrate)
        if not self.sonar.initialize():
            self.get_logger().info("Failed to initialize Ping!")
            exit(1)

        # Verify sonar firmware version is compatible
        device_data = self.sonar.get_device_information()
        if device_data['device_type'] == 1:
            self.get_logger().info(
                f"Ping1D firmware version: {device_data['firmware_version_major']}.{device_data['firmware_version_minor']}"
            )
            if device_data['device_revision'] == 1:
                self.get_logger().info("Ping1D device detected!")
                if device_data['firmware_version_major'] < 3 and \
                     device_data['firmware_version_minor'] < 29:
                    self.get_logger().info(
                        "Ping1D firmware version is not compatible! Update to 3.29 or higher."
                    )

        self.sonar.set_gain_setting(self.gain_setting)
        self.sonar.set_mode_auto(self.mode_auto)
        self.sonar.set_ping_enable(self.ping_enable)
        self.sonar.set_ping_interval(self.ping_interval)
        self.sonar.set_range(int(self.scan_start * 1000),
                             int(self.scan_length * 1000))
        self.sonar.set_speed_of_sound(int(self.speed_of_sound * 1000))
        self.sonar.control_continuous_start(PING1D_PROFILE)

        # Setup the publisher
        self.publisher = self.create_publisher(SonarPing1D, self.topic, 10)
        self.get_logger().info("Node initialized! Publishing sonar data...")

        # Continuously publish sonar data when available
        self.msg = SonarPing1D()
        self.msg.header.frame_id = self.frame_id

        while rclpy.ok():
            if not self.ping_enable:
                rclpy.spin_once(self, timeout_sec=1.0)
                continue
            data = self.sonar.wait_message([PING1D_PROFILE])
            if data:
                self.msg.header.stamp = self.get_clock().now().to_msg()
                self.msg.distance = data.distance * 0.001
                self.msg.confidence = data.confidence
                self.msg.transmit_duration = data.transmit_duration
                self.msg.ping_number = data.ping_number
                self.msg.scan_start = data.scan_start * 0.001
                self.msg.scan_length = data.scan_length * 0.001
                self.msg.gain_setting = data.gain_setting
                self.msg.profile_data = data.profile_data

                self.publisher.publish(self.msg)

                # Allow for params callback to be processed
                rclpy.spin_once(self, timeout_sec=0.01)

    def set_param_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            if exec(f"self.{param.name} != param.value"):
                exec(f"self.{param.name} = param.value")
                self.get_logger().info(f'Updated {param.name}: {param.value}')

            if param.name == 'gain_setting':
                self.sonar.set_gain_setting(self.gain_setting)
            if param.name == 'mode_auto':
                self.sonar.set_mode_auto(self.mode_auto)
            if param.name == 'ping_enable':
                self.sonar.set_ping_enable(self.ping_enable)
            if param.name == 'ping_interval':
                self.sonar.set_ping_interval(self.ping_interval)
            if param.name == 'scan_start':
                self.sonar.set_range(int(self.scan_start * 1000),
                                     int(self.scan_length * 1000))
            if param.name == 'scan_length':
                self.sonar.set_range(int(self.scan_start * 1000),
                                     int(self.scan_length * 1000))
            if param.name == 'speed_of_sound':
                self.sonar.set_speed_of_sound(int(self.speed_of_sound * 1000))

        return result


def main(args=None):
    rclpy.init(args=args)
    node = Ping1DNode()


if __name__ == '__main__':
    main()
