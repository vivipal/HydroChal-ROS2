#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image, CameraInfo
from interfaces.msg import WideObstacle, ObstacleList

from tools import VideoProcessor, EPSILON

RETURN_STRATEGY = True


class ObstacleDetector(Node):
    def __init__(self, update_rate=1.):  # update_rate in Hz
        super().__init__('obstacle_detector')

        self.image_subscriber = self.create_subscription(Image, 'stereo_left', self.image_callback, 10)
        self.disparity_subscriber = self.create_subscription(DisparityImage, 'disparity_map', self.disparity_callback, 10)
        self.camera_info_subscriber = self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)

        # Create a timer with a callback function and a period of 1 / update_rate
        self.processing_timer = self.create_timer(1. / update_rate, self.processing_callback)
        self.strategy_publisher = self.create_publisher(Image, 'strategy_image', 10)
        self.obstacles_publisher = self.create_publisher(ObstacleList, 'detected_obstacles', 10)

        self.image_left = None
        self.depth_map = None
        
        self.focal_length = None
        self.cx = self.cy = 0.
        self.d_stereo = 0.

        self.cv_bridge = CvBridge()
        self.video_processor = VideoProcessor()
    
    def processing_callback(self):
        obstacles = []

        if (self.image_left is not None) and (self.depth_map is not None):
            boxes, strategy = self.video_processor.process(self.image_left, self.depth_map,
                                                           return_strategy=RETURN_STRATEGY)

            if strategy is not None:  # publish strategy image
                ros_image_msg = self.cv_bridge.cv2_to_imgmsg(strategy, encoding="bgr8")
                self.strategy_publisher.publish(ros_image_msg)

            for (x0, y0, x1, y1, z) in boxes:
                tx = -self.d_stereo / 2.  # left camera
                obstacle = WideObstacle()

                # Expressed coordinates in the centered frame (in between the 2 cameras)
                obstacle.x_min = ((x0 - self.cx) * z - tx) / self.focal_length
                obstacle.x_max = ((x1 - self.cx) * z - tx) / self.focal_length

                obstacle.y_min = (y0 - self.cy) * (z / self.focal_length)
                obstacle.y_max = (y1 - self.cy) * (z / self.focal_length)

                obstacle.z = float(z)
                obstacles.append(obstacle)

        obstacles_msg = ObstacleList()
        obstacles_msg.obstacles = obstacles
        self.obstacles_publisher.publish(obstacles_msg)
    
    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        self.image_left = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def disparity_callback(self, msg):
        # Convert ROS image message to OpenCV image
        frame = self.cv_bridge.imgmsg_to_cv2(msg.image, desired_encoding='32FC1')

        if self.focal_length is not None:  # did we received camera information?
            bounded_disparity = np.maximum(msg.min_disparity, np.minimum(frame, msg.max_disparity))
            self.depth_map = self.d_stereo / np.maximum(EPSILON, bounded_disparity)
    
    def camera_info_callback(self, msg):
        projection_matrix = msg.p

        self.focal_length = projection_matrix[0]

        self.cx = projection_matrix[2]
        self.cy = projection_matrix[6]

        self.d_stereo = projection_matrix[3]


def main():
    rclpy.init()
    obstacle_detector = ObstacleDetector(update_rate=8)  # in Hz

    rclpy.spin(obstacle_detector)

    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
