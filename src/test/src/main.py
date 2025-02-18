#!/usr/bin/env python

from six.moves import input
# from test.move_group_python_interface_nursing import MoveGroupPythonInterfaceNursing
# import test.patrol_test
# import test.arm_control_test
# import test.plat_move
import rotation_client
import patrol_outside
import plat_move2
from std_msgs.msg import Bool
import numpy as np
import roslib
import sys
from camera import Camera
import rospy

import time
import select
import termios
import tty

def fix_angle():
    straight = rotation_client.rotation_cl("1")
    print(straight)
    angle = float(str(straight))
    while(angle != 0):
        straight = rotation_client.rotation_cl("1")
        print(straight)
        angle = float(str(straight))
        
        # if(angle == "clockwise"):
        #     print("clockwise")
        plat_move2.rotate(angle)
        
        # else:
        #     print("counterclockwise")
            # plat_move2.rotate(2)
            # fix_up = patrol_plat_vision.Patrol_plat("2")
            # fix_up.rotate()
        if(abs(angle) > 3):
            time.sleep(4)
        else:
            time.sleep(3)
if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    # roslib.load_manifest("vision")
    # print(sys.path)
    # realsense = Camera()
    # print(realsense.get_point())

    # patrol_outside.move()

    try:
        fix_angle()
        # plat_move2.move()
        # fix_angle()
        # plat_move2.move()
        # fix_angle()
        # plat_move2.rotate(1)


        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Nursing")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        # # plat_move.rotate()
        # # arm_control_test.arm_key()
        # # print(vision.realsense_tf())
        # # print(vision.image_depth())
        # input("============ Press `Enter` to begin the process ...")
        # tutorial = MoveGroupPythonInterfaceNursing()
        # tutorial.gripper_hold()
        # time_duration = 3.5
        # time.sleep(time_duration)
        # tutorial.gripper_release()

        # input("============ Go to the door knob ...")
        # node = Detection(0.9, 1, 10)
        # #patrol_test.move(4.61152219772, -5.15782785416, -0.37051665948, 0.928825820619)

        # input("============ Standby position ...")
        # tutorial.go_to_joint_state()

        # input("============ Stretch arm ...")

        # input("gripper_hold")
        # tutorial.gripper_hold()

        # input("gripper_release")
        # tutorial.gripper_release()

        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # tutorial.go_to_pose_goal()

        # input("============ Press `Enter` to plan and execute a path with an attached collision object ...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

        print("============ Caring system complete!")
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass


def test():
    print()
