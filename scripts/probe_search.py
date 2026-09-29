"""Check that a Cartesian search command results in actual Gazebo movement."""
import time
from pymycobot import MyPalletizerSocket

robot = MyPalletizerSocket('127.0.0.1', 9000)
print('before', robot.get_angles(), robot.get_coords(), flush=True)
robot.sync_send_coords([150, 0, 200, 0], 40, 15)
for second in range(5):
    time.sleep(1)
    print(f'after {second + 1}s', robot.get_angles(), robot.get_coords(), flush=True)
