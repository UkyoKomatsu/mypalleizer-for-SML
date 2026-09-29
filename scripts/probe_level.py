"""Verify that the gripper grasp axis stays vertical while the arm moves."""
import math
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from pymycobot import MyPalletizerSocket


def tool_axis(node, buffer):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            transform = buffer.lookup_transform(
                'world', 'gripper_base', rclpy.time.Time()
            )
            q = transform.transform.rotation
            # World direction of the gripper frame's local +Y tool axis.
            return (
                2 * (q.x * q.y - q.w * q.z),
                1 - 2 * (q.x * q.x + q.z * q.z),
                2 * (q.y * q.z + q.w * q.x),
            )
        except Exception:
            pass
    raise TimeoutError('No world -> gripper_base transform')


def main():
    rclpy.init()
    node = Node('gripper_level_probe')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    robot = MyPalletizerSocket('127.0.0.1', 9000)
    try:
        for angles in ([0, 0, 0, 0], [0, 30, -10, 0], [30, 40, 0, 45]):
            robot.sync_send_angles(angles, 30, 20)
            actual = robot.get_angles()
            axis = tool_axis(node, buffer)
            tilt = math.degrees(math.acos(max(-1, min(1, -axis[2]))))
            print(f'angles={actual} gripper axis={axis} tilt from down={tilt:.2f} degrees')
            if tilt > 5:
                raise AssertionError('Gripper did not remain downward')
    finally:
        robot.sync_send_angles([0, 0, 0, 0], 30, 20)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
