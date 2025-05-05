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
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from bluerobotics_sonar_msgs.msg import SonarPing1D

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class Ping1DImagerNode(Node):

    def __init__(self, node_name='ping1d_imager'):
        super().__init__(node_name)

        # Declare Parameters
        params = {
            'image_length': [200, int],
            'data_topic': ['sonar/ping1d/data', str],
            'image_topic': ['sonar/ping1d/image', str],
            'bag_file': ['', str],
            'video_file': ['', str],
        }

        for param, [value, dtype] in params.items():
            self.declare_parameter(param, value)
            exec(f"self.{param}:dtype = self.get_parameter(param).value")
            self.get_logger().info(f'{param}: {value}')

        # Handle parameter updates
        self.param_handler_ptr_ = self.add_on_set_parameters_callback(
            self.set_param_callback)

        # Init other variables
        self.bridge = CvBridge()
        self.scan_image = None

        # Determine if output is a topic or a video
        if len(self.video_file) == 0:
            self.to_video = False
            self.publisher = self.create_publisher(Image, self.image_topic, 10)
            self.get_logger().info("Publishing data to ros2 topic")
        else:
            self.video_writer = None
            self.to_video = True
            self.get_logger().info("Saving data to video file")

        # Determine if input is a topic or a bag
        if len(self.bag_file) == 0:
            self.from_bag = False
            self.subscrber = self.create_subscription(SonarPing1D,
                                                      self.data_topic,
                                                      self.data_callback, 10)
            self.get_logger().info("Reading data from ros2 topic")
        else:
            self.from_bag = True
            self.get_logger().info("Reading data from bag file")
            self.process_bag()

    def data_callback(self, msg):
        if self.scan_image is None:
            self.scan_image = np.zeros(
                (len(msg.profile_data), self.image_length), dtype=np.uint8)
        self.scan_image = np.roll(self.scan_image, -1, axis=1)
        self.scan_image[:, -1] = msg.profile_data
        scan_image = cv2.applyColorMap(self.scan_image, cv2.COLORMAP_VIRIDIS)
        scan_image = cv2.rectangle(scan_image, (0, 0), (50, 12), (0, 0, 0), -1)
        scan_image = cv2.putText(scan_image, f'{msg.distance:.2f} m', (1, 10),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                 (255, 255, 255), 1, cv2.LINE_AA)

        if self.to_video:
            if self.video_writer is None:
                height, width = scan_image.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(self.video_file, fourcc,
                                                    self.video_fps,
                                                    (width, height))
                if not self.video_writer.isOpened():
                    self.get_logger().error("Failed to open video writer.")
                    return
            self.video_writer.write(scan_image)
        else:
            self.publisher.publish(self.br.cv2_to_imgmsg(scan_image))

    def process_bag(self):
        storage_options = StorageOptions(uri=self.bag_file,
                                         storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr',
                                             output_serialization_format='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topic_types}
        msg_type_str = type_map.get(self.data_topic, None)
        if not msg_type_str:
            self.get_logger().error(
                f"Topic '{self.data_topic}' not found in bag.")
            exit()
        msg_type = get_message(msg_type_str)

        # Estimate FPS
        times = []
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == self.data_topic:
                msg = deserialize_message(data, msg_type)
                times.append(t)
        time_del = np.diff(times)
        self.video_fps = np.round(1.0 / (np.mean(time_del) * 1e-9)).astype(int)
        self.get_logger().info(f'Estimated FPS: {self.video_fps}')

        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == self.data_topic:
                msg = deserialize_message(data, msg_type)
                self.data_callback(msg)

        if self.to_video:
            self.video_writer.release()
            self.get_logger().info('Finished writing to video file')
            exit()

    def set_param_callback(self, params):
        result = SetParametersResult(successful=True)
        for param in params:
            exec(f"self.flag = self.{param.name} != param.value")
            if self.flag:
                exec(f"self.{param.name} = param.value")
                self.get_logger().info(f'Updated {param.name}: {param.value}')

            if param.name == 'image_length':
                self.scan_image = np.zeros(
                    (len(self.scan_image), self.image_length))

        return result


def main(args=None):
    rclpy.init(args=args)
    node = Ping1DImagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
