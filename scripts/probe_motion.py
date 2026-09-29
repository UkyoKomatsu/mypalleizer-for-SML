"""First milestone: arm joint motion and attached gripper motion."""
import time
from pymycobot import MyPalletizerSocket

robot = MyPalletizerSocket('127.0.0.1', 9000)
print('start angles:', robot.get_angles())
robot.sync_send_angles([20, 20, -20, 15], 25, 15)
print('moved angles:', robot.get_angles())
robot.set_gripper_state(0, 50)
time.sleep(1)
robot.set_gripper_state(1, 50)
time.sleep(1)
robot.set_gripper_state(0, 50)
robot.sync_send_angles([0, 0, 0, 0], 25, 15)
print('home angles:', robot.get_angles())
