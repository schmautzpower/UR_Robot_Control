"""
This script just represent the functions of the Remote tool for UR Robots E-Series
"""

# Importing the robot_control.py (need to be in same folder as the main script)
from robot_control import *

#  Robot IP must be String to parse it to the Socket connections
robot_ip = '192.168.8.229'

# initialise the robot by just creating the Robot object
# the refresh_rate specify the cycle hz, Defaut is 125hz
robot = Robot(robot_ip, refresh_rate=250)


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
        if robot.get_digital_output_on(16):
            robot.set_digital_out(8, False)  # Turn the TDO0 off
        else:
            robot.set_digital_out(8, True)  # Turn the TDO0 on
        time.sleep(1)  # sleep is not needed but it is better to visualize


def move_robot():
    # to move the robot, just call the robot.move function.
    # Now it is made simple to give robot a list with the target or joints.
    # the list represent in movel, movej and speedl/j [X, Y, Z, rX, rY, rZ], for joints it is the number of motor.
    # with the functions movel and so on you can specify all you need to control the robot or jsut leave blank.
    # Check the Default values, which are pretty slow for safety reasons.
    robot.move(
        movel(
            [0.086, -0.52813, 0.017, 0, 3.14, 0]
        )
    )

    # usually the script will not wait until the target is reached.
    # This will make it pretty hard to expect precise movement.
    # Therefore i extend the move() function with a wait parameter.
    robot.move(  # tell the robot to move
        movel(  # this will tell the robot how to move
            # this is the target where robot will move. (100e-3 == 100mm or 0.1m)
            [100e-3, -1000e-3, 10e-3, 2.221, 2.221, 0],
            # this represent the time the robot should take to move from actual pose to target (t parameter always override a(acceleration) and v(speed))
            t=5,
            # r represent the radius which is in direct control not important, because you cannot make smooth movements with several targets.
            r=0.001
        ),
        wait=True,  # this will block the script until the target is reached in 0.001m or 1mm radius around target by default
        # this can be overwritten with this value to make is more precise or less precise for speedy movement.
        area=0.01
    )

    # you can always check where the robot is as Target Position.
    actual_pose = robot.get_TCP_pose()
    # To ask for joints you can use info.get_joints()
    actual_pose[2] -= 0.1  # just modify the z axis to show the movement.
    #  this is the simplest way to move the robot.
    robot.move(movel(actual_pose))


if __name__ == '__main__':
    move_robot()
    digital_out_test()
