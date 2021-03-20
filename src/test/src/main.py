#!/usr/bin/env python

from six.moves import input
from move_group_python_interface_nursing import MoveGroupPythonInterfaceNursing
import variable
import patrol_test
from std_msgs.msg import Bool
#import arm

import sys
import rospy
if __name__ == '__main__':
  try:
    print(variable.group_name)
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    input("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    tutorial = MoveGroupPythonInterfaceNursing()
    input("patrol_test")
    patrol_test.move(4.61152219772, -5.15782785416, -0.37051665948, 0.928825820619)

    input("gripper_hold")
    tutorial.gripper_hold()
    input("gripper_release")
    tutorial.gripper_release()

    input("============ Press `Enter` to execute a movement using a joint state goal ...")
    tutorial.go_to_joint_state()

    input("============ Press `Enter` to execute a movement using a pose goal ...")
    tutorial.go_to_pose_goal()

    input("============ Press `Enter` to plan and display a Cartesian path ...")
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    input("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    tutorial.display_trajectory(cartesian_plan)

    input("============ Press `Enter` to execute a saved path ...")
    tutorial.execute_plan(cartesian_plan)

    input("============ Press `Enter` to add a box to the planning scene ...")
    tutorial.add_box()

    input("============ Press `Enter` to attach a Box to the Panda robot ...")
    tutorial.attach_box()

    input("============ Press `Enter` to plan and execute a path with an attached collision object ...")
    cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    tutorial.execute_plan(cartesian_plan)

    input("============ Press `Enter` to detach the box from the Panda robot ...")
    tutorial.detach_box()

    input("============ Press `Enter` to remove the box from the planning scene ...")
    tutorial.remove_box()

    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    pass
  except KeyboardInterrupt:
    pass


