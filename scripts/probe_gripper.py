"""Verify the adaptive gripper controller responds to pymycobot commands."""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymycobot import MyPalletizerSocket


class JointReader(Node):
    def __init__(self):
        super().__init__('gripper_probe')
        self.position = None
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)

    def on_state(self, msg):
        if 'gripper_controller' in msg.name:
            self.position = msg.position[msg.name.index('gripper_controller')]


def sample(node, seconds=2):
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.position is None:
        raise TimeoutError('No gripper joint state')
    return node.position


def main():
    rclpy.init()
    reader = JointReader()
    robot = MyPalletizerSocket('127.0.0.1', 9000)
    robot.set_gripper_state(0, 50)
    opened = sample(reader)
    robot.set_gripper_state(1, 50)
    closed = sample(reader)
    robot.set_gripper_state(0, 50)
    print('gripper open:', round(opened, 3), 'closed:', round(closed, 3))
    if abs(opened - closed) < 0.2:
        raise AssertionError('Gripper did not move')
    reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
