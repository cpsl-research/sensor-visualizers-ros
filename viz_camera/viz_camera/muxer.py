# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This file takes inspiration from the detection_visualizer class
# which can be found at https://github.com/ros2/detection_visualizer/


import rclpy
from rclpy.node import Node

import cv2
import cv_bridge
import message_filters
import sensor_msgs.msg
import vision_msgs.msg


class DetectionMuxer(Node):
    def __init__(self):
        super().__init__("multicam_visualizer", allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self._bridge = cv_bridge.CvBridge()
        self.receiving_detections = False

        qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        # Subscribe to incoming images
        self._image_sub = message_filters.Subscriber(
            self, sensor_msgs.msg.Image, 'images', qos_profile=qos,
        )
        self._caminfo_sub = message_filters.Subscriber(
            self, sensor_msgs.msg.CameraInfo, "camera_info", qos_profile=qos,
        )
        self._synchronizer_images = message_filters.TimeSynchronizer(
            (self._image_sub, self._caminfo_sub), 10
        )
        self._synchronizer_images.registerCallback(self.image_receive)

        # Subscribe to detections
        self._detection_sub = message_filters.Subscriber(
            self, vision_msgs.msg.Detection2DArray, "detections", qos_profile=qos,
        )
        self._detection_sub.registerCallback(self.detection_receive)

        # Synchronize images and detections
        self._synchronizer_detections = message_filters.TimeSynchronizer(
            (self._image_sub, self._caminfo_sub, self._detection_sub), 10)
        self._synchronizer_detections.registerCallback(self.synch_receive)

        # Publish muxed images and camera info
        self._image_pub = self.create_publisher(
            sensor_msgs.msg.Image, "muxed_image", qos_profile=qos,
        )
        self._caminfo_pub = self.create_publisher(
            sensor_msgs.msg.CameraInfo, "camera_info", qos_profile=qos,
        )

    def image_receive(self, image_msg, camera_info):
        """Publish a passthrough if not receiving detections"""
        if not self.receiving_detections:
            self._image_pub.publish(image_msg)
            self._caminfo_pub.publish(camera_info)

    def detection_receive(self, detection_msg):
        self.receiving_detections = True
    
    def synch_receive(self, image_msg, camera_info, detection_msg):
        cv_image = self._bridge.imgmsg_to_cv2(image_msg)

        # Draw boxes on image
        for detection in detection_msg.detections:
            max_class = None
            max_score = 0.0
            for result in detection.results:
                hypothesis = result.hypothesis
                if hypothesis.score > max_score:
                    max_score = hypothesis.score
                    max_class = hypothesis.class_id
            if max_class is None:
                raise RuntimeError("Failed to find class with highest score")

            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            sx = detection.bbox.size_x
            sy = detection.bbox.size_y

            min_pt = (round(cx - sx / 2.0), round(cy - sy / 2.0))
            max_pt = (round(cx + sx / 2.0), round(cy + sy / 2.0))
            color = (0, 255, 0)
            thickness = 1
            cv2.rectangle(cv_image, min_pt, max_pt, color, thickness)

            label = '{} {:.3f}'.format(max_class, max_score)
            pos = (min_pt[0], max_pt[1])
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, label, pos, font, 0.75, color, 1, cv2.LINE_AA)
            
        detection_image_msg = self._bridge.cv2_to_imgmsg(cv_image, encoding=image_msg.encoding)
        detection_image_msg.header = image_msg.header

        self._image_pub.publish(detection_image_msg)
        self._caminfo_pub.publish(camera_info)


def main(args=None):
    rclpy.init(args=args)

    muxer = DetectionMuxer()

    rclpy.spin(muxer)
    muxer.destroy_node()
    rclpy.shutdown()