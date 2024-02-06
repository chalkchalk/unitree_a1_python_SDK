import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/build")
print(os.path.dirname(os.path.abspath(__file__)) + "/build")
from robot_interface import RobotInterface
import time
import numpy as np
# q kp dq kd tau

UPPER_BOUND = [0.802851455917, 4.18879020479, -0.916297857297]
LOWER_BOUND = [-0.802851455917, -1.0471975512, -2.69653369433]

cmd = []
for i in range(12):
        cmd.extend([0, 0, 0, 0, 0])

robot_interface = RobotInterface()
leg_index = 3
joint_index = 2
motor_index = leg_index * 3 + joint_index


for i in range(1000):
    angle = 0.4 * (UPPER_BOUND[joint_index] - LOWER_BOUND[joint_index]) * np.sin(0.02 * i) + 0.5 * (UPPER_BOUND[joint_index] + LOWER_BOUND[joint_index])
    cmd[motor_index * 5 + 0] = angle
    cmd[motor_index * 5 + 1] = 60
    cmd[motor_index * 5 + 2] = 0.0
    cmd[motor_index * 5 + 3] = 1.0 # kd
    cmd[motor_index * 5 + 4] = 0
    # print(angle)
    robot_interface.update_command(cmd)
    # print(robot_interface.receive_observation())
    time.sleep(0.01)
    
        








