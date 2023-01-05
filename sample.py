"""
This script just represent the functions of the Remote tool for UR Robots E-Series
"""

import time

# Importing the robot_control.py (need to be in same folder as the main script)
import robot_control as rc

#  Robot IP must be String to parse it to the Socket connections
robot_ip = '192.168.8.121'

# There about 3 connection ways to the UR.

# You can use the RTDE to request data from Robot. So you will be able to get feedback from robot with 125hz
# Refresh rates up to 500Hz are also possible, but require Gigabit network and Unix Based Computer like Linux or MacOS
info = rc.RTDE(robot_ip)  # init Info Level connection to Robot

# This represend the instant control level. The commands send to the robot will be executed directly.
# It is ignoring if robot is still working on something.
control = rc.URControl(robot_ip, True)  # init instand Control level

# This part can be used if you want to have a full script that does not require live control.
# Scripts must be finished and send to the robot. The robot can handle up to 500 lines.
# After 500 Lines reached, you need to clear the intrpreter buffer and start counting from 1.
icontrol = rc.URControl(robot_ip, False)  # init intepreter Control level


def digital_out_test():
    """
    This Scrip should show how to control Digital Outs.

    Important note!
    To interact with Tool Output just use digital out 8 for tool out 0 and digital out 9 for tool out 1
    But to ask the state of the Tool output you will have different id's (TDO0=16, TDO1=17).
    This is commented in Robot control
    """
    while 1:  # Infinite Loop
        # Check if the tool digital output 0 is on
        if info.get_digital_output_on(16):
            control.set_digital_out(8, False)  # Turn the TDO0 off
        else:
            control.set_digital_out(8, True)  # Turn the TDO0 on
        time.sleep(1)  # sleep is not needed but it is better to visualize


def move_robot():
    """
    How to move the robot
    targets or joints always need to be type(List) and the coordinates for targets need to be in meter. 
    Joints are in Radians, but you can use the rc.degree_to_radians() func to give degree and return Radians. 
    The lib provide also a function to wait until the movement is done. But be Careful with that. 
    It will block the whole script. A Sample how to use will be below. 
    """
    control.send_home()  # this will tell the robot to move to his home position in joints not as target.
    while not info.target_reached(rc.home_joints, level=5, joints=True):
        # this line will tell the script to wait until the joints are reached. You can modify the level.
        # level 1 is less precise than level 5.
        # the home joints can be edited in the lib
        pass

    # you can always check where the robot is as Target Position.
    actual_pose = info.get_TCP_pose()
    # To ask for joints you can use info.get_joints()
    actual_pose[2] -= 0.1  # just modify the z axis to show the movement.
    # this is the most simple type you can send a linear movement.
    control.send_movel(actual_pose)
    # you can also give the movement t=1, but this will ignore a and v.
    control.send_movel(actual_pose, a=1, v=1, r=0)
    while not info.target_reached(actual_pose, level=2):
        pass


if __name__ == '__main__':
    move_robot()
    digital_out_test()
