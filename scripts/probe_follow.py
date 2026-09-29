"""Check that the D435 frame travels with the gripper during an arm command."""
import math
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from pymycobot import MyPalletizerSocket


def pose(node, buffer):
    deadline = time.time() + 10
    latest = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            t = buffer.lookup_transform('world', 'd435_link', rclpy.time.Time())
            v = t.transform.translation
            latest = (v.x, v.y, v.z)
        except Exception:
            pass
        if latest is not None and time.time() > deadline - 2:
            return latest
    raise TimeoutError('No world -> d435_link transform')


def main():
    rclpy.init()
    node = Node('follow_probe')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    robot = MyPalletizerSocket('127.0.0.1', 9000)
    robot.sync_send_angles([0, 0, 0, 0], 40, 20)
    before = pose(node, buffer)
    robot.sync_send_angles([30, 20, -20, 10], 40, 20)
    print('arm angles after movement:', robot.get_angles())
    after = pose(node, buffer)
    displacement = math.dist(before, after)
    print('D435 before:', before, 'after:', after, 'travel metres:', round(displacement, 4))
    robot.sync_send_angles([0, 0, 0, 0], 40, 20)
    if displacement < 0.02:
        raise AssertionError('D435 did not follow the arm')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
