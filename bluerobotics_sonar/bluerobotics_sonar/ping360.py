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
ROS 2 node for the Blue Robotics Ping360 scanning sonar.

This node connects to a Ping360 device, configures it based on ROS
parameters, and publishes the sonar data to a topic. It also allows for
dynamic reconfiguration of the sonar settings.
"""

from brping import Ping360
from brping.definitions import PING360_AUTO_DEVICE_DATA

import math
import numpy as np
from bluerobotics_sonar_msgs.msg import SonarPing360
from .utils import SonarRangeFinder, KF

import rclpy
from rclpy import qos
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos_overriding_options import QoSOverridingOptions


class Ping360Node(Node):
    """
    The main class for the sonar node.
    """

    # Firmware defaults from ping-viewer/src/sensor/ping360.cpp
    _firmwareMaxNumberOfPoints = 1200
    _viewerDefaultNumberOfSamples = _firmwareMaxNumberOfPoints
    _firmwareMaxTransmitDuration = 500
    _firmwareMinTransmitDuration = 5
    _firmwareMinSamplePeriod = 80
    _firmwareDefaultTransmitDuration = 32
    _viewerDefaultSamplePeriod = 88
    _epsilon = 1e-6
    _samplePeriodTickDuration = 25e-9
    _speed_of_sound = 1500
    _autoTransmitDuration = True

    transmit_duration = _firmwareDefaultTransmitDuration
    num_points = _viewerDefaultNumberOfSamples
    sample_period = _viewerDefaultSamplePeriod

    def __init__(self, node_name='ping360'):
        """
        Initializes the node, parameters, subscriber, and other necessary objects.
        """
        super().__init__(node_name)

        # --- Parameters ---
        # Declare and get parameters for the node.
        params = {
            'mode': [1, int],
            'gain_setting': [0, int],
            'transmit_frequency': [750, int],
            'start_angle': [0, int],
            'stop_angle': [399, int],
            'num_steps': [1, int],
            'delay': [0, int],
            'device': ['/dev/ttyUSB0', int],
            'baudrate': [115200, int],
            'motor_off': [False, bool],
            'speed_of_sound': [1500, int],
            'scan_threshold': [50, int],
            'range': [1.0, float],
            'offset': [20, int],
            'window_size': [15, int],
            'topic': ['/sonar/ping360/data', str],
            'frame_id': ['ping360', str],
            'ref_dist': [0.45, float]
        }

        for param, [value, dtype] in params.items():
            self.declare_parameter(param, value)
            exec(f"self.{param}:dtype = self.get_parameter(param).value")
        for param in self.get_parameters(params.keys()):
            self.get_logger().info(f'{param.name}: {param.value}')

        qos_override_opts = QoSOverridingOptions(
            policy_kinds=(
                qos.QoSPolicyKind.HISTORY,
                qos.QoSPolicyKind.DEPTH,
                qos.QoSPolicyKind.RELIABILITY,
                )
        )
        SENSOR_QOS = rclpy.qos.qos_profile_sensor_data
        self.range_finder = SonarRangeFinder(offset=self.offset, 
                                             window_size=self.window_size, 
                                             threshold=self.scan_threshold)
        self.filter = KF(noise=5.0)

        # --- Parameter Handler ---
        # Handle parameter updates
        _ = self.add_on_set_parameters_callback(self.set_param_callback)

        # --- Sonar Device ---
        # Init and configure Ping360 sonar
        self.sonar = Ping360()
        if len(self.device.split('.')) == 4:
            self.sonar.connect_udp(self.device, self.baudrate)
        else:
            self.sonar.connect_serial(self.device, self.baudrate)
        if not self.sonar.initialize():
            self.get_logger().info("Failed to initialize Ping360!")
            exit(1)

        # --- Verify Firmware Version ---
        # Verify sonar firmware version is compatible
        device_data = self.sonar.get_device_information()
        if device_data['device_type'] == 2:
            self.get_logger().info(
                f"Ping360 firmware version: {device_data['firmware_version_major']}.{device_data['firmware_version_minor']}"
            )
            if device_data['firmware_version_major'] < 3 and \
                 device_data['firmware_version_minor'] < 3:
                self.get_logger().info(
                    "Ping360 firmware version is not compatible! Update to 3.3 or higher."
                )
                exit(1)

        # --- Configure Sonar ---
        self.set_speed_of_sound(self.speed_of_sound)
        self.set_range(self.range)
        self.sonar.control_auto_transmit(
            mode=self.mode,
            gain_setting=self.gain_setting,
            transmit_duration=self.transmit_duration,
            sample_period=self.sample_period,
            transmit_frequency=self.transmit_frequency,
            number_of_samples=self.num_points,
            start_angle=self.start_angle,
            stop_angle=self.stop_angle,
            num_steps=self.num_steps,
            delay=self.delay)

        # --- Publisher ---
        # Setup the publisher
        self.publisher = self.create_publisher(SonarPing360, self.topic,
                                               SENSOR_QOS,
                                               qos_overriding_options=qos_override_opts) 
        self.get_logger().info("Node initialized! Publishing sonar data...")

        # --- Main Loop ---
        # Continuously publish sonar data when available
        self.msg = SonarPing360()
        self.msg.header.frame_id = self.frame_id

        dist_buf = []
        dist_ref = 0.45

        try:
            while True:
                if self.motor_off:
                    rclpy.spin_once(self, timeout_sec=1.0)
                    continue
                data = self.sonar.wait_message([PING360_AUTO_DEVICE_DATA])
                if data:
                    self.msg.header.stamp = self.get_clock().now().to_msg()
                    self.msg.mode = data.mode
                    self.msg.gain_setting = data.gain_setting
                    self.msg.angle = data.angle
                    self.msg.transmit_duration = data.transmit_duration
                    self.msg.sample_period = data.sample_period
                    self.msg.transmit_frequency = data.transmit_frequency
                    self.msg.range = self.range
                    self.msg.profile_data = data.data
                    distance = self.range_finder(
                        np.frombuffer(data.data, dtype=np.uint8), dist_ref)
                    self.msg.distance = self.filter(distance)
                    self.publisher.publish(self.msg)
                    dist_ref = self.msg.distance

                    dist_buf.append(self.msg.distance)
                    if len(dist_buf) > 15:
                        self.get_logger().info(f'')
                        for param in self.get_parameters(params.keys()):
                            self.get_logger().info(f'{param.name}: {param.value}')                    
                        self.get_logger().info(f'Error: {np.linalg.norm(np.array(dist_buf)-self.ref_dist):.4f}')
                        dist_buf = []

                    # Allow for params callback to be processed
                    rclpy.spin_once(self, timeout_sec=0.01)
        finally:
            # Stop the transponder before destroying the node
            if not len(self.device.split('.')) == 4:
                self.sonar.connect_serial(self.device, self.baudrate)
            self.sonar.control_motor_off()
            self.destroy_node()
            rclpy.shutdown()

    def set_param_callback(self, params):
        """
        This function is the callback for when parameters are changed.
        It updates the node's attributes and sends the new settings to the sonar.
        """
        update = False
        for param in params:
            if "qos" in param.name:
                continue
            exec(f"self.flag = self.{param.name} != param.value")
            if self.flag:
                exec(f"self.{param.name} = param.value")
                self.get_logger().info(f'Updated {param.name}: {param.value}')

            if param.name == "motor_off" and param.value:
                # if serial device, reconnect to send a line break
                # and stop auto-transmitting
                if not len(self.device.split('.')) == 4:
                    self.sonar.connect_serial(self.device, self.baudrate)
                self.sonar.control_motor_off()
            else:
                update = True

            if param.name == "offset" or \
               param.name == "window_size" or \
               param.name == "scan_threshold":
                self.range_finder = SonarRangeFinder(offset=self.offset, 
                                                     window_size=self.window_size, 
                                                     threshold=self.scan_threshold)

        # Apply the new settings to the sonar device
        if update and not self.motor_off:
            # if serial device, reconnect to send a line break
            # and stop auto-transmitting
            if not len(self.device.split('.')) == 4:
                self.sonar.connect_serial(self.device, self.baudrate)
            self.set_speed_of_sound(self.speed_of_sound)
            self.set_range(self.range)
            self.sonar.control_auto_transmit(
                mode=self.mode,
                gain_setting=self.gain_setting,
                transmit_duration=self.transmit_duration,
                sample_period=self.sample_period,
                transmit_frequency=self.transmit_frequency,
                number_of_samples=self.num_points,
                start_angle=self.start_angle,
                stop_angle=self.stop_angle,
                num_steps=self.num_steps,
                delay=self.delay)
        return SetParametersResult(successful=True)

    """
    --- Helper Functions ---

    Helper functions to compute the transmit_duration, sample period, number of points based on the range, and speed of sound
    The calculations are based on the Ping Viewer source code: https://github.com/bluerobotics/ping-viewer 
    Refer to src/sensor/ping360.cpp and src/sensor/ping360.h
    """

    def set_speed_of_sound(self, speed_of_sound: int):
        """
        Set the speed of sound (m/s) used for calculating distance from time-of-flight
        The default speed of sound is 1500 m/s in water
        """
        if speed_of_sound != self._speed_of_sound:
            desired_range = round(self.get_range())
            self._speed_of_sound = speed_of_sound
            self.sample_period = self.calculate_sample_period(desired_range)

            while self.sample_period < self._firmwareMinSamplePeriod:
                self.num_points -= 1
                self.sample_period = self.calculate_sample_period(
                    desired_range)

            self.adjust_transmit_duration()

    def sample_period_in_seconds(self):
        """
        Converts the sample period from ticks to seconds.
        """
        return self.sample_period * self._samplePeriodTickDuration

    def get_range(self):
        """
        Calculates the current maximum range of the sonar in meters.
        """
        range = self.sample_period_in_seconds(
        ) * self.num_points * self._speed_of_sound / 2
        self.range = range
        return range

    def set_range(self, new_range: float):
        """
        Compute the transmit_duration, sample period, and number of points based on the range
        """
        if math.isclose(new_range, self.get_range(), rel_tol=self._epsilon):
            return

        self.num_points = self._firmwareMaxNumberOfPoints
        self.sample_period = self.calculate_sample_period(new_range)

        while self.sample_period < self._firmwareMinSamplePeriod:
            self.num_points -= 1
            self.sample_period = self.calculate_sample_period(new_range)

        self.adjust_transmit_duration()

    def adjust_transmit_duration(self):
        """
        Automatically adjusts the transmit duration for optimal performance at the current range.
        """
        if self._autoTransmitDuration:
            auto_duration = round(8000 * self.get_range() /
                                  self._speed_of_sound)
            auto_duration = max(
                int(2.5 * self.sample_period_in_seconds() / 1e-6),
                auto_duration)
            self.transmit_duration = max(
                int(self._firmwareMinTransmitDuration),
                min(self.transmit_duration_max(), auto_duration))
        elif self.transmit_duration > self.transmit_duration_max():
            self.transmit_duration = self.transmit_duration_max()

    def transmit_duration_max(self) -> int:
        """
        Calculates the maximum possible transmit duration for the current sample period.
        """
        return min(self._firmwareMaxTransmitDuration,
                   int(self.sample_period_in_seconds() * 64e6))

    def calculate_sample_period(self, distance: float) -> int:
        """
        Calculates the required sample period for a given distance.
        """
        try:
            calculated_sample_period = 2.0 * distance / (
                self.num_points * self._speed_of_sound *
                self._samplePeriodTickDuration)
        except ZeroDivisionError:
            self.get_logger().warn(
                "Division by zero encountered in sample period calculation. Using default."
            )
            return self._viewerDefaultSamplePeriod

        if (abs(calculated_sample_period) < self._epsilon
                or calculated_sample_period < 0
                or calculated_sample_period > (2**16 - 1)):
            self.get_logger().warn(
                "Invalid calculation of sample period. Using viewer default values."
            )
            self.get_logger().debug(
                f"calculatedSamplePeriod: {calculated_sample_period}, distance: {distance}, "
                f"num_points: {self.num_points}, speed_of_sound: {self._speed_of_sound}, "
                f"samplePeriodTickDuration: {self._samplePeriodTickDuration}")
            return self._viewerDefaultSamplePeriod

        return int(calculated_sample_period)


def main(args=None):
    """
    The main function to run the node.
    """
    rclpy.init(args=args)
    node = Ping360Node()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
