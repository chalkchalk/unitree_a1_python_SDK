import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/build")
print(os.path.dirname(os.path.abspath(__file__)) + "/build")
from robot_interface import RobotInterface
import time
import numpy as np
# q kp dq kd tau



cmd = []
for i in range(12):
        cmd.extend([0, 0, 0, 0, 0])

robot_interface = RobotInterface()
leg_index = 0
motor_index = leg_index * 3 + 2


for i in range(1000):
    angle = 0.5 * np.sin(0.02 * i) - 1.8
    cmd[motor_index * 5 + 0] = angle
    cmd[motor_index * 5 + 1] = 20
    cmd[motor_index * 5 + 2] = 0.0
    cmd[motor_index * 5 + 3] = 0.1 # kd
    cmd[motor_index * 5 + 4] = 0
    # print(angle)
    robot_interface.update_command(cmd)
    time.sleep(0.01)
    
        








