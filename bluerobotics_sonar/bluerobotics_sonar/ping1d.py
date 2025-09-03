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

"""
ROS 2 node for the Blue Robotics Ping1D scanning sonar.

This node connects to a Ping1D device, configures it based on ROS
parameters, and publishes the sonar data to a topic. It also allows for
dynamic reconfiguration of the sonar settings.
"""

from brping import Ping1D
from brping.definitions import PING1D_PROFILE

import numpy as np
from bluerobotics_sonar_msgs.msg import SonarPing1D
from .utils import SonarRangeFinder, SonarStabilityFilter

import rclpy
from rclpy import qos
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos_overriding_options import QoSOverridingOptions


class Ping1DNode(Node):
    """
    The main class for the sonar node.
    """

    def __init__(self, node_name='ping1d'):
        """
        Initializes the node, parameters, subscriber, and other necessary objects.
        """
        super().__init__(node_name)

        # --- Parameters ---
        # Declare and get parameters for the node.
        params = {
            'gain_setting': [0, int],
            'mode_auto': [0, int],
            'ping_enable': [True, bool],
            'pings_per_second': [30, int],
            'ping_interval': [-1, int],
            'scan_start': [0.0, float],
            'scan_length': [1.0, float], 
            'speed_of_sound': [1500, int],
            'device': ['/dev/ttyUSB0', int],
            'baudrate': [115200, int],
            'scan_threshold': [100, int],
            'filter_threshold': [0.2, float],
            'filter_window_size': [15, int],
            'offset': [20, int],
            'window_size': [5, int],
            'topic': ['/sonar/ping1d/data', str],
            'frame_id': ['ping1d', str],
        }

        for param, [value, dtype] in params.items():
            self.declare_parameter(param, value)
            exec(f"self.{param}:dtype = self.get_parameter(param).value")
        params = self.get_parameters(params.keys())
        for param in params:
            self.get_logger().info(f'{param.name}: {param.value}')

        qos_override_opts = QoSOverridingOptions(
            policy_kinds=(
                qos.QoSPolicyKind.HISTORY,
                qos.QoSPolicyKind.DEPTH,
                qos.QoSPolicyKind.RELIABILITY,
                )
        )
        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data
        self.range_finder = SonarRangeFinder(max_range=self.scan_length,
                                             offset=self.offset,
                                             scan_threshold=self.scan_threshold,
                                             window_size=self.window_size)
        self.stability_filter = SonarStabilityFilter(window_size=self.filter_window_size,
                                                     threshold=self.filter_threshold)
        
        if self.ping_interval == -1:
            if self.pings_per_second > 0 and self.pings_per_second <= 50:
                self.ping_interval = int(1000.0/self.pings_per_second)
            else:
                self.ping_interval = 33
    
        # --- Parameter Handler ---
        # Handle parameter updates
        self.param_handler_ptr_ = self.add_on_set_parameters_callback(
            self.set_param_callback)

        # --- Sonar Device ---
        # Init and configure Ping1D sonar
        self.sonar = Ping1D()
        if len(self.device.split('.')) == 4:
            self.sonar.connect_udp(self.device, self.baudrate)
        else:
            self.sonar.connect_serial(self.device, self.baudrate)
        if not self.sonar.initialize():
            self.get_logger().info("Failed to initialize Ping!")
            exit(1)

        # --- Verify Firmware Version ---
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

        # --- Configure Sonar ---
        self.sonar.set_gain_setting(self.gain_setting)
        self.sonar.set_mode_auto(self.mode_auto)
        self.sonar.set_ping_enable(self.ping_enable)
        self.sonar.set_ping_interval(self.ping_interval)
        self.sonar.set_range(int(self.scan_start * 1000),
                             int(self.scan_length * 1000))
        self.sonar.set_speed_of_sound(int(self.speed_of_sound * 1000))
        self.sonar.control_continuous_start(PING1D_PROFILE)

        # --- Publisher ---
        # Setup the publisher
        self.publisher = self.create_publisher(SonarPing1D, self.topic,
                                               SENSOR_QOS,
                                               qos_overriding_options=qos_override_opts) 
        self.get_logger().info("Node initialized! Publishing sonar data...")

        # --- Main Loop ---
        # Continuously publish sonar data when available
        self.msg = SonarPing1D()
        self.msg.header.frame_id = self.frame_id

        try:
            while True:
                if not self.ping_enable:
                    rclpy.spin_once(self, timeout_sec=1.0)
                    continue
                data = self.sonar.wait_message([PING1D_PROFILE])
                if data:
                    self.msg.header.stamp = self.get_clock().now().to_msg()
                    self.msg.confidence = data.confidence
                    self.msg.transmit_duration = data.transmit_duration
                    self.msg.ping_number = data.ping_number
                    self.msg.scan_start = data.scan_start * 0.001
                    self.msg.scan_length = data.scan_length * 0.001
                    self.msg.gain_setting = data.gain_setting
                    self.msg.profile_data = data.profile_data
                    distance = self.range_finder(
                        np.frombuffer(data.profile_data, dtype=np.uint8))
                    self.msg.distance = self.stability_filter(distance)
                    self.publisher.publish(self.msg)

                    # Allow for params callback to be processed
                    rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            # Stop the transponder before destroying the node
            self.sonar.set_ping_enable(False)
            self.destroy_node()
            rclpy.shutdown()

    def set_param_callback(self, params):
        """
        This function is the callback for when parameters are changed.
        It updates the node's attributes and sends the new settings to the sonar.
        """
        result = SetParametersResult(successful=True)
        for param in params:
            if "qos" in param.name:
                continue
            exec(f"self.flag = self.{param.name} != param.value")
            if self.flag:
                exec(f"self.{param.name} = param.value")
                self.get_logger().info(f'Updated {param.name}: {param.value}')

            # Apply the new settings to the sonar device
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
            if param.name == 'pings_per_second':
                if self.pings_per_second > 0 and self.pings_per_second <= 50:
                    self.ping_interval = int(1000.0/self.pings_per_second)
                else:
                    self.get_logger().warn(f"pings_per_second: {self.pings_per_second}; out of range [1-50]! Defaulting to 30 pings/s!")
                    self.ping_interval = 33
        return result


def main(args=None):
    """
    The main function to run the node.
    """
    rclpy.init(args=args)
    node = Ping1DNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
