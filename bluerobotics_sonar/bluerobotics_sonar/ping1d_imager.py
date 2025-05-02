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


import cv2
import numpy as  np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from bluerobotics_sonar_msgs.msg import SonarPing1D

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class Ping1DImagerNode(Node):
    def __init__(self, node_name='ping1d_imager'):
        super().__init__(node_name)

        # Declare Parameters
        params = {
          'image_length': [200, int],
          'data_topic': ['sonar/ping1d/data', str],
          'image_topic': ['sonar/ping1d/image', str],
        }

        for param, [value, dtype] in params.items():
            self.declare_parameter(param, value)
            exec(f"self.{param}:dtype = self.get_parameter(param).value")
            self.get_logger().info(f'{param}: {value}')

        # Handle parameter updates
        self.param_handler_ptr_ = self.add_on_set_parameters_callback(self.set_param_callback)

        # Setup the publisher and subscrber                                        
        self.image_publisher = self.create_publisher(Image, 
                                                     self.image_topic, 
                                                     10)
        self.subscrber = self.create_subscription(SonarPing1D,
                                                  self.data_topic, 
                                                  self.data_callback,
                                                  10)

        # Init other variables
        self.bridge = CvBridge()
        self.scan_image = None

        self.get_logger().info("Node initialized!")

    def data_callback(self, msg):
        if self.scan_image is None:
            self.scan_image = np.zeros((len(msg.profile_data), 
                                        self.image_length), dtype=np.uint8)
        self.scan_image = np.roll(self.scan_image, -1, axis=1)
        self.scan_image[:, -1] = msg.profile_data
        scan_image = cv2.applyColorMap(self.scan_image, cv2.COLORMAP_VIRIDIS)
        scan_image = cv2.rectangle(scan_image,
                                   (0, 0), 
                                   (50, 12), 
                                   (0, 0, 0), -1)
        scan_image = cv2.putText(scan_image, 
                                 f'{msg.distance:.2f} m',
                                 (1, 10), 
                                 cv2.FONT_HERSHEY_SIMPLEX, 
                                 0.4, (255, 255, 255), 1, 
                                 cv2.LINE_AA)
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(scan_image))

    def set_param_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            exec(f"self.flag = self.{param.name} != param.value")
            if self.flag:
                exec(f"self.{param.name} = param.value")
                self.get_logger().info(f'Updated {param.name}: {param.value}')

            if param.name == 'image_length':
                self.scan_image = np.zeros((len(self.scan_image), 
                                            self.image_length))

        return result

def main(args=None):
    rclpy.init(args=args)
    node = Ping1DImagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()