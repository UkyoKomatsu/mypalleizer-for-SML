"""Subset of the pymycobot MyPalletizerSocket wire protocol for this simulator.

The same 9000/TCP endpoint accepts send_angles, send_coords, gripper commands,
get_angles, get_coords and is_moving. Cartesian IK is provisional and should be
calibrated against the real robot before relying on existing pick coordinates.
"""
import math
import socket
import struct
import threading
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


ARM = ["joint1_to_base", "joint2_to_joint1", "joint3_to_joint2", "joint5_to_joint4"]
ROBOT = ET.parse('/opt/sim/urdf/robot.urdf').getroot()
JOINTS = {j.get('name'): j for j in ROBOT.findall('joint')}
CHAIN = ['joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2',
         'joint4_to_joint3', 'joint5_to_joint4']
LIMITS = [(-2.824, 2.824), (-0.0349, 1.5708), (-1.6057, 1.0471), (-3.14, 3.14)]


def transform(xyz, rpy):
    x, y, z = [float(v) for v in xyz.split()]
    roll, pitch, yaw = [float(v) for v in rpy.split()]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = (x, y, z)
    return result


def fk(angles):
    matrix = np.eye(4)
    index = 0
    for name in CHAIN:
        joint = JOINTS[name]
        origin = joint.find('origin')
        matrix = matrix @ transform(origin.get('xyz'), origin.get('rpy'))
        if joint.get('type') == 'revolute':
            axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            theta = angles[index]
            index += 1
            if np.allclose(axis, [0, 0, 1]):
                matrix = matrix @ transform('0 0 0', f'0 0 {theta}')
    return matrix


def frame(code, payload=b''):
    return bytes([0xfe, 0xfe, len(payload) + 2, code]) + payload + b'\xfa'


def pack16(values):
    return b''.join(struct.pack('>h', max(-32768, min(32767, int(v)))) for v in values)


class Bridge(Node):
    def __init__(self):
        super().__init__('mypalletizer_socket_bridge')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.angles = np.zeros(4)
        self.target = np.zeros(4)
        self.lock = threading.Lock()
        threading.Thread(target=self.serve, daemon=True).start()

    def on_state(self, msg):
        states = dict(zip(msg.name, msg.position))
        with self.lock:
            self.angles = np.array([states.get(name, self.angles[i]) for i, name in enumerate(ARM)])

    def move(self, publisher, names, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=max(1, int(math.ceil(seconds))))
        msg.points = [point]
        publisher.publish(msg)

    def serve(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('0.0.0.0', 9000))
            server.listen(8)
            self.get_logger().info('MyPalletizerSocket simulator listening on port 9000')
            while rclpy.ok():
                client, _ = server.accept()
                threading.Thread(target=self.client, args=(client,), daemon=True).start()

    def client(self, conn):
        with conn:
            buffer = b''
            while True:
                chunk = conn.recv(1024)
                if not chunk:
                    return
                buffer += chunk
                while True:
                    start = buffer.find(b'\xfe\xfe')
                    if start < 0:
                        buffer = buffer[-1:]
                        break
                    buffer = buffer[start:]
                    if len(buffer) < 4:
                        break
                    size = buffer[2] + 3
                    if len(buffer) < size:
                        break
                    packet, buffer = buffer[:size], buffer[size:]
                    if packet[-1] != 0xfa:
                        continue
                    reply = self.dispatch(packet[3], packet[4:-1])
                    if reply is not None:
                        conn.sendall(reply)

    def dispatch(self, code, data):
        with self.lock:
            angles = self.angles.copy()
            target = self.target.copy()
        if code == 0x20:  # GET_ANGLES
            return frame(code, pack16(np.degrees(angles) * 100))
        if code == 0x23:  # GET_COORDS
            pose = fk(angles)
            yaw = math.degrees(math.atan2(pose[1, 0], pose[0, 0]))
            return frame(code, pack16([*(pose[:3, 3] * 10000), yaw * 100]))
        if code == 0x2b:  # IS_MOVING
            moving = int(np.max(np.abs(angles - target)) > 0.025)
            return frame(code, bytes([moving]))
        if code == 0x22 and len(data) >= 9:  # SEND_ANGLES
            goal = np.radians(np.array(struct.unpack('>hhhh', data[:8])) / 100)
            speed = max(1, data[8])
            self.command_arm(goal, speed)
            return None
        if code == 0x25 and len(data) >= 9:  # SEND_COORDS
            raw = struct.unpack('>hhhh', data[:8])
            goal_xyz = np.array(raw[:3]) / 10000.0
            goal_yaw = math.radians(raw[3] / 100)
            lower, upper = np.array(LIMITS).T

            def error(candidate):
                pose = fk(candidate)
                yaw = math.atan2(pose[1, 0], pose[0, 0])
                return [*((pose[:3, 3] - goal_xyz) * 10),
                        math.atan2(math.sin(yaw - goal_yaw), math.cos(yaw - goal_yaw))]

            result = least_squares(error, np.clip(angles, lower, upper), bounds=(lower, upper), max_nfev=100)
            if np.linalg.norm(fk(result.x)[:3, 3] - goal_xyz) > 0.02:
                self.get_logger().warn('Cartesian goal outside 20 mm tolerance; refusing motion')
            else:
                self.command_arm(result.x, max(1, data[8]))
            return None
        if code in (0x66, 0x67):
            # 0=open, 1=close for state; value 0=closed, 100=open.
            value = (0 if data[0] == 1 else 100) if code == 0x66 else data[0]
            position = -0.45 + (value / 100) * 0.60
            self.move(self.grip_pub, ['gripper_controller'], [position], 1.0)
            return frame(code, b'\x01') if code == 0x67 else None
        self.get_logger().warn(f'Unsupported pymycobot command 0x{code:02x}')
        return None

    def command_arm(self, goal, speed):
        with self.lock:
            present = self.angles.copy()
            self.target = np.asarray(goal)
        seconds = max(1.0, float(np.max(np.abs(goal - present))) / (speed / 100 * 1.5))
        self.move(self.arm_pub, ARM, goal, seconds)


if __name__ == '__main__':
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
