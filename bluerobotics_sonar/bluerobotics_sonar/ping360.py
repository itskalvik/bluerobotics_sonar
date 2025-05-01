#! /usr/bin/env python3

#-----------------------------------------------------------------------------------
# MIT License

# Copyright (c) 2025 Kalvik Jakkala

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

from brping import Ping360
from brping.definitions import PING360_AUTO_DEVICE_DATA

import math
import numpy as np
from bluerobotics_sonar_msgs.msg import SonarPing360

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d


class Ping360Node(Node):
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
    super().__init__(node_name)

    # Declare Parameters
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
       'speed_of_sound': [1500, int], # m/s in water
       'scan_threshold': [100, int],  # Threshold for peak detection
       'range': [1.0, float],         # [meters] range(0.75 to 50)
       'offset': [120, int],          # [samples] offset to ignore the first few samples before peak detection
       'topic': ['sonar/ping360/data', str],
       'frame_id': ['ping360', str],
    }

    for param, [value, dtype] in params.items():
      self.declare_parameter(param, value)
      exec(f"self.{param}:dtype = self.get_parameter(param).value")
      self.get_logger().info(f'{param}: {value}')

    # Handle parameter updates
    _ = self.add_on_set_parameters_callback(self.set_param_callback)

    # Init and configure Ping360 sonar
    self.sonar = Ping360()
    if len(self.device.split('.')) == 4:
      self.sonar.connect_udp(self.device, self.baudrate)
    else:
      self.sonar.connect_serial(self.device, self.baudrate)
    if not self.sonar.initialize():
      self.get_logger().info("Failed to initialize Ping!")
      exit(1)
    self.set_speed_of_sound(self.speed_of_sound)
    self.set_range(self.range)
    self.sonar.control_auto_transmit(
        mode = self.mode,
        gain_setting = self.gain_setting,
        transmit_duration = self.transmit_duration,
        sample_period = self.sample_period,
        transmit_frequency = self.transmit_frequency,
        number_of_samples = self.num_points,
        start_angle = self.start_angle,
        stop_angle = self.stop_angle,
        num_steps = self.num_steps,
        delay = self.delay)

    # Setup the publisher
    self.publisher = self.create_publisher(SonarPing360, 
                                           self.topic, 
                                           10)
    self.get_logger().info("Node initialized! Publishing sonar data...")

    # Continuously publish sonar data when available
    self.msg = SonarPing360()
    self.msg.header.frame_id = self.frame_id

    while rclpy.ok():
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
        self.msg.distance = self.get_distance(np.frombuffer(data.data, dtype=np.uint8))

        self.publisher.publish(self.msg)

        # Allow for params callback to be processed
        rclpy.spin_once(self, timeout_sec=0.01)

  def set_param_callback(self, params):
    update = False
    for param in params:
      if exec(f"self.{param.name}") != param.value:
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

    if update and not self.motor_off:
        # if serial device, reconnect to send a line break
        # and stop auto-transmitting
        if not len(self.device.split('.')) == 4:
          self.sonar.connect_serial(self.device, self.baudrate)
        self.set_speed_of_sound(self.speed_of_sound)
        self.set_range(self.range)
        self.sonar.control_auto_transmit(
            mode = self.mode,
            gain_setting = self.gain_setting,
            transmit_duration = self.transmit_duration,
            sample_period = self.sample_period,
            transmit_frequency = self.transmit_frequency,
            number_of_samples = self.num_points,
            start_angle = self.start_angle,
            stop_angle = self.stop_angle,
            num_steps = self.num_steps,
            delay = self.delay)
    return SetParametersResult(successful=True)
  
  def get_distance(self, data):
      data = gaussian_filter1d(data[self.offset:], 5)
      peaks, _ = find_peaks(data, height=self.scan_threshold)
      if len(peaks)>0:
          dist = peaks[0]+self.offset+1
          dist *= self.range/self.num_points
      else:
          dist = self.range
      return dist

  '''
  Helper functions to compute the transmit_duration, sample period, number of points based on the range, and speed of sound
  The calculations are based on the Ping Viewer source code: https://github.com/bluerobotics/ping-viewer 
  Refer to src/sensor/ping360.cpp
  Refer to src/sensor/ping360.h
  '''

  def set_speed_of_sound(self, speed_of_sound: int):
      ''' Set the speed of sound (m/s) used for calculating distance from time-of-flight
      The default speed of sound is 1500 m/s in water
      '''
      if speed_of_sound != self._speed_of_sound:
          desired_range = round(self.get_range())
          self._speed_of_sound = speed_of_sound
          self.sample_period = self.calculate_sample_period(desired_range)
          
          while self.sample_period < self._firmwareMinSamplePeriod:
              self.num_points -= 1
              self.sample_period = self.calculate_sample_period(desired_range)
          
          self.adjust_transmit_duration()
  
  def sample_period_in_seconds(self):
    return self.sample_period * self._samplePeriodTickDuration
  
  def get_range(self):
      range = self.sample_period_in_seconds() * self.num_points * self._speed_of_sound / 2
      self.range = range
      return range

  def set_range(self, new_range: float):
      '''Compute the transmit_duration, sample period, and number of points based on the range
      '''
      if math.isclose(new_range, self.get_range(), rel_tol=self._epsilon):
          return
      
      self.num_points = self._firmwareMaxNumberOfPoints
      self.sample_period = self.calculate_sample_period(new_range)

      while self.sample_period < self._firmwareMinSamplePeriod:
          self.num_points -= 1
          self.sample_period = self.calculate_sample_period(new_range)

      self.adjust_transmit_duration()

  def adjust_transmit_duration(self):
      if self._autoTransmitDuration:
          auto_duration = round(8000 * self.get_range() / self._speed_of_sound)
          auto_duration = max(int(2.5 * self.sample_period_in_seconds() / 1e-6), auto_duration)
          self.transmit_duration = max(
              int(self._firmwareMinTransmitDuration),
              min(self.transmit_duration_max(), auto_duration)
          )
      elif self.transmit_duration > self.transmit_duration_max():
          self.transmit_duration = self.transmit_duration_max()

  def transmit_duration_max(self) -> int:
      return min(self._firmwareMaxTransmitDuration, int(self.sample_period_in_seconds() * 64e6))

  def calculate_sample_period(self, distance: float) -> int:
      try:
          calculated_sample_period = 2.0 * distance / (self.num_points * self._speed_of_sound * self._samplePeriodTickDuration)
      except ZeroDivisionError:
          self.get_logger().warn("Division by zero encountered in sample period calculation. Using default.")
          return self._viewerDefaultSamplePeriod

      if (abs(calculated_sample_period) < self._epsilon or
          calculated_sample_period < 0 or
          calculated_sample_period > (2**16 - 1)):
          self.get_logger().warn("Invalid calculation of sample period. Using viewer default values.")
          self.get_logger().debug(
              f"calculatedSamplePeriod: {calculated_sample_period}, distance: {distance}, "
              f"num_points: {self.num_points}, speed_of_sound: {self._speed_of_sound}, "
              f"samplePeriodTickDuration: {self._samplePeriodTickDuration}")
          return self._viewerDefaultSamplePeriod

      return int(calculated_sample_period)


def main(args=None):
    rclpy.init(args=args)
    node = Ping360Node()

if __name__ == '__main__':
    main()