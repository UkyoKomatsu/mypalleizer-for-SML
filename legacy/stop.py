from pymycobot import MyPalletizerSocket
import time
mc = MyPalletizerSocket("10.42.10.104",9000)

print(mc.get_coords())
mc.sync_send_angles([0,0,0,0],20,15)
print(mc.get_angles())
time.sleep(1)
mc.release_all_servos()
time.sleep(3)
mc.set_gripper_state(1,100)

