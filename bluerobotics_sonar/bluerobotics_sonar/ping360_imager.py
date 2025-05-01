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

import numpy as np
from bluerobotics_sonar_msgs.msg import SonarPing360

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class Ping360ImagerNode(Node):
  def __init__(self, node_name='ping360_imager'):
    super().__init__(node_name)

    # Declare Parameters
    params = {
      'data_topic': ['sonar/ping360/data', str],
      'image_topic': ['sonar/ping360/image', str],
      'rotation': [0.0, float],
    }

    for param, [value, dtype] in params.items():
      self.declare_parameter(param, value)
      exec(f"self.{param}:dtype = self.get_parameter(param).value")
      self.get_logger().info(f'{param}: {value}')

    # Handle parameter updates
    _ = self.add_on_set_parameters_callback(self.set_param_callback)

    # Setup the publisher and subscrber                                        
    self.image_publisher = self.create_publisher(Image, 
                                                 self.image_topic, 
                                                 10)
    self.subscrber = self.create_subscription(SonarPing360,
                                              self.data_topic, 
                                              self.data_callback,
                                              10)

    # Init other variables
    self.bridge = CvBridge()
    self.scan_image = None
    
    self.get_logger().info("Node initialized!")

  def data_callback(self, msg):
      if self.scan_image is None:
        self.scan_image = np.zeros((400, len(msg.profile_data)), 
                                    dtype=np.uint8)

      self.scan_image[msg.angle] = msg.profile_data         
      scan_image = cv2.warpPolar(self.scan_image, 
                                 dsize=(1500, 1500), 
                                 center=(750, 750), 
                                 maxRadius=750, 
                                 flags=cv2.WARP_INVERSE_MAP|cv2.WARP_FILL_OUTLIERS)
      scan_image = cv2.applyColorMap(scan_image, cv2.COLORMAP_VIRIDIS)
      scan_image = cv2.line(scan_image, 
                            (750, 750), 
                            (750 + int(750 * np.cos(np.deg2rad(msg.angle*0.9))), 
                             750 + int(750 * np.sin(np.deg2rad(msg.angle*0.9)))), 
                            (255, 255, 255), 2)
      scan_image = cv2.circle(scan_image, 
                              (750, 750), 
                              750, 
                              (255, 255, 255), 2)
      if self.rotation != 0.0:
        scan_image = cv2.rotate(scan_image, 
                                cv2.ROTATE_90_CLOCKWISE if self.rotation > 0 else cv2.ROTATE_90_COUNTERCLOCKWISE)
      scan_image = cv2.putText(scan_image, 
                               f'Distance: {msg.distance:.2f} m',
                               (20, 50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 
                               1, (255, 255, 255), 2, 
                               cv2.LINE_AA)
      scan_image = cv2.putText(scan_image, 
                               f'Angle: {(msg.angle*0.9)%360:.0f} deg',
                               (20, 100),
                               cv2.FONT_HERSHEY_SIMPLEX,
                               1, (255, 255, 255), 2,
                               cv2.LINE_AA)
      self.image_publisher.publish(self.bridge.cv2_to_imgmsg(scan_image))

  def set_param_callback(self, params):
    for param in params:
      exec(f"self.flag = self.{param.name} != param.value")
      if self.flag:
        exec(f"self.{param.name} = param.value")
        self.get_logger().info(f'Updated {param.name}: {param.value}')
    return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = Ping360ImagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()