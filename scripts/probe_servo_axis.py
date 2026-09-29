"""Measure image motion caused by small Cartesian arm moves in the simulator."""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pymycobot import MyPalletizerSocket


class Camera(Node):
    def __init__(self):
        super().__init__('servo_axis_probe')
        self.bridge = CvBridge()
        self.image = None
        self.create_subscription(Image, '/d435/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def red_center(self):
        if self.image is None:
            return None
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 80, 50]), np.array([10, 255, 255]))
        mask |= cv2.inRange(hsv, np.array([170, 80, 50]), np.array([180, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        m = cv2.moments(contour)
        return (m['m10'] / m['m00'], m['m01'] / m['m00']) if m['m00'] else None


def observe(camera):
    deadline = time.monotonic() + 2
    camera.image = None
    while time.monotonic() < deadline:
        rclpy.spin_once(camera, timeout_sec=0.2)
    if camera.image is None:
        raise TimeoutError('No D435 color frame')
    return camera.red_center()


def main():
    rclpy.init()
    camera = Camera()
    robot = MyPalletizerSocket('127.0.0.1', 9000)
    try:
        robot.sync_send_angles([0, 0, 0, 0], 30, 20)
        time.sleep(2)
        robot.sync_send_coords([150, 0, 200, 0], 40, 15)
        time.sleep(2)
        base = robot.get_coords()
        print('base', base, 'pixel', observe(camera), flush=True)
        for axis in (0, 1):
            goal = list(base)
            goal[axis] += 8
            robot.sync_send_coords(goal, 20, 15)
            time.sleep(2)
            actual = robot.get_coords()
            print(f'+8 mm axis {axis}: requested {goal}, actual {actual}, pixel {observe(camera)}', flush=True)
            robot.sync_send_coords(base, 20, 15)
    finally:
        robot.sync_send_angles([0, 0, 0, 0], 30, 20)
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
