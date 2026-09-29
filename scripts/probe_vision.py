"""Receive simulated RGB and depth frames; run the supplied YOLO model once."""
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO


class Probe(Node):
    def __init__(self):
        super().__init__('vision_probe')
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        color_topic = os.getenv('SIM_COLOR_TOPIC', '/d435/image_raw')
        depth_topic = os.getenv('SIM_DEPTH_TOPIC', '/d435/depth/image_raw')
        self.create_subscription(Image, color_topic, self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data)
        print(f'waiting for {color_topic} and {depth_topic}', flush=True)

    def on_color(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def main():
    rclpy.init()
    node = Probe()
    deadline = time.time() + 45
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.rgb is not None and node.depth is not None:
            break
    if node.rgb is None or node.depth is None:
        raise TimeoutError('Camera topics unavailable; inspect: ros2 topic list | grep d435')
    rgb, depth = node.rgb, node.depth
    print('color:', rgb.shape, rgb.dtype)
    valid = depth[np.isfinite(depth) & (depth > 0)]
    print('depth:', depth.shape, depth.dtype, 'valid pixels:', valid.size,
          'median metres:', float(np.median(valid)) if valid.size else None)
    cv2.imwrite('/opt/sim/camera_sample.png', rgb)
    model = YOLO('/opt/sim/legacy/best.pt')
    result = model.predict(rgb, verbose=False)[0]
    print('YOLO classes:', model.names)
    print('YOLO detections:', len(result.boxes))
    for box in result.boxes:
        print('class:', int(box.cls[0]), 'confidence:', float(box.conf[0]))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
