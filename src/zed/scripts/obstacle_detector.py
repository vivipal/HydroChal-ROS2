#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage

from tools import VideoProccessor, EPSILON

DEBUG = False


class ObstacleDetector(Node):
    def __init__(self, update_rate=1.):  # update_rate in Hz
        super().__init__('obstacle_detector')

        self.stereo_subscriber = self.create_subscription(Image, 'stereo_stream', self.stereo_callback, 10)
        self.disparity_subscriber = self.create_subscription(DisparityImage, 'disparity_stream', self.disparity_callback, 10)

        # Create a timer with a callback function and a period of 1 / update_rate
        self.detection_timer = self.create_timer(1. / update_rate, self.detection_callback)

        self.stereo_left = None
        self.stereo_right = None
        self.depth_map = None

        self.bridge = CvBridge()
        self.video_proccessor = VideoProccessor()
    
    def detection_callback(self):
        if (self.stereo_left is not None) and (self.depth_map is not None):
            w0, w1, h0, h1 = self.video_proccessor.proccess(self.stereo_left)

            window = self.depth_map[h0:h1, w0:w1]
            # only consider distances smaller than 30m
            valid_distances = window[window < 30e3]

            if valid_distances.size > 0:
                distance = np.median(valid_distances)
                self.get_logger().info("Median distance in tracking window: %icm" % (distance / 10.))
    
    def stereo_callback(self, msg):
        # Convert ROS image message to OpenCV image
        w = msg.width
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.stereo_left = frame[:, :w//2]
        self.stereo_right = frame[:, w//2:]

    def disparity_callback(self, msg):
        # Convert ROS image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='32FC1')
        h, w, *_ = frame.shape

        focal_length = msg.f
        baseline = msg.t

        min_disparity = msg.min_disparity
        max_disparity = msg.max_disparity
        
        bounded_disparity = np.maximum(min_disparity, np.minimum(frame, max_disparity))
        self.depth_map = (focal_length * baseline) / np.maximum(EPSILON, bounded_disparity)

        if DEBUG and (self.stereo_left is not None):
            left_gray = cv2.cvtColor(self.stereo_left, cv2.COLOR_BGR2GRAY).reshape(h, w, 1)
            normalized_disparity = ((bounded_disparity - min_disparity) * (255. / max_disparity))

            layout = np.concatenate((left_gray, left_gray, normalized_disparity.astype(np.uint8).reshape(h, w, 1)), axis=2)
            cv2.imshow('Image/Disparity', layout)


def main():
    rclpy.init()
    obstacle_detector = ObstacleDetector(update_rate=8.)

    rclpy.spin(obstacle_detector)

    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
