#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import tf


class MoveGroupInterface(object):
  """MoveGroupInterface"""
  def __init__(self):
    super(MoveGroupInterface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "tm_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ocm = moveit_msgs.msg.OrientationConstraint()
    endeff_constraint = moveit_msgs.msg.Constraints()
    ocm.link_name = "tm_tip_link"
    ocm.header.frame_id = "tm_base_link"
    ocm.absolute_x_axis_tolerance = 0.1
    ocm.absolute_y_axis_tolerance = 2*pi
    ocm.absolute_z_axis_tolerance = 0.1
    ocm.weight = 1.0
    endeff_constraint.orientation_constraints.append(ocm)

    pub = rospy.Publisher("gripper/cmd_gripper", Bool, queue_size=10)

    ## Getting Basic Information

    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.endeff_constraint = endeff_constraint
    self.pub = pub
    pub.publish(True)

  def go_to_home_state(self):
    move_group = self.move_group
    pub = self.pub
    pub.publish(True)
    joint_goal = [-0.5336353182792664, -0.07512450218200684, 1.9735796451568604, -1.7973496913909912, 0.5216982960700989, 0.09126132726669312]
    move_group.go(joint_goal,wait=True)
    move_group.stop()
  
  def go_to_p_state(self):
    move_group = self.move_group
    pub = self.pub
    pub.publish(True)
    pi = 3.14
    joint_goal = [0.0004727941413875669,-0.6478399634361267,2.3247411251068115,-1.6761173009872437,1.5687698125839233,0.0015320111997425556]
    move_group.go(joint_goal,wait=True)
    move_group.stop()

  def move_item_cb(self, data):
    #pub = self.pub
    #pub.publish(True)
    raw_input("Please enter to press button")
    item_pose = geometry_msgs.msg.Pose()
    item_pose = self.tf_item_listener()
    item_pose.orientation.x = 0.741375997577
    item_pose.orientation.y = -0.00206070522393
    item_pose.orientation.z = -0.000807733047346
    item_pose.orientation.w = 0.671086232371

    item_pose.position.x -= 0.015
    item_pose.position.y += 0.187
    item_pose.position.z += 0.08
    print item_pose
    self.go_to_pose_goal(item_pose)
    print("After go_to_pose_goal")
    item_movement = [0,-0.05,0]
    self.go_to_cartesian_goal(item_movement)

    item_movement = [0,0.05,0]
    self.go_to_cartesian_goal(item_movement)
    #pub.publish(True)
    rospy.sleep(1)

    self.go_to_home_state()

  def move_item_cb_p(self, data):
    raw_input("Please enter to press button")
    item_pose = geometry_msgs.msg.Pose()
    item_pose = self.tf_item_listener()
    item_pose.orientation.x = 0.5
    item_pose.orientation.y = 0.5
    item_pose.orientation.z = 0.5
    item_pose.orientation.w = 0.5

    item_pose.position.x -= 0.19
    item_pose.position.y -= 0.015
    item_pose.position.z += 0.055
    print item_pose
    self.go_to_pose_goal(item_pose)
    print("After go_to_pose_goal")
    item_movement = [0.05,0,0]
    self.go_to_cartesian_goal(item_movement)

    item_movement = [-0.05,0,0]
    self.go_to_cartesian_goal(item_movement)

    rospy.sleep(1)

    self.go_to_p_state()


  def go_to_pose_goal(self,pose_goal):
    move_group = self.move_group
    #move_group.set_path_constraints(self.endeff_constraint)
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

  def go_to_cartesian_goal(self,item_movement):
    move_group= self.move_group
    #move_group.set_path_constraints(self.endeff_constraint)
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.x += item_movement[0]
    wpose.position.y += item_movement[1]
    wpose.position.z += item_movement[2]

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)

    move_group.execute(plan,wait = True)

  def go_to_cartesian_goal_error(self,item_movement):
    move_group= self.move_group
    #move_group.set_path_constraints(self.endeff_constraint)
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.x += item_movement[0]
    wpose.position.y += item_movement[1]
    wpose.position.z += item_movement[2]

    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)
    move_group.execute(plan,wait = True)
    rospy.sleep(1)
    (plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)
    move_group.execute(plan,wait = True)

  def go_to_pose_joint_angle(self,angle_goal):
    move_group = self.move_group
    joint_goal = angle_goal 
    move_group.go(joint_goal, wait=True)
    move_group.stop
    move_group.clear_pose_targets()

  def tf_item_listener(self):
    listener = tf.TransformListener()
    listener.waitForTransform("base_link","item_tf_link",rospy.Time(),rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("base_link","item_tf_link",rospy.Time(0))   
    item_pose = geometry_msgs.msg.Pose()
    item_pose.position.x = trans[0]
    item_pose.position.y = trans[1]
    item_pose.position.z = trans[2]
    item_pose.orientation.x = rot[0]
    item_pose.orientation.y = rot[1]
    item_pose.orientation.z = rot[2]
    item_pose.orientation.w = rot[3]
    return item_pose

  def go_to_end_position_1(self):
    pub = self.pub
    angle_goal_up = [3.00321316719,-0.145751237869,1.98954260349,-1.7794213295,1.70037734509,-0.00975445192307]
    angle_goal = [3.00342464447,0.208993673325,2.29876232147,-2.44641828537,1.70335412025,-0.0034518733155]
    self.go_to_pose_joint_angle(angle_goal_up)
    self.go_to_pose_joint_angle(angle_goal)
    pub.publish(False)
    rospy.sleep(1)
    item_movement = [0,0,0.20]
    self.go_to_cartesian_goal_error(item_movement)
    self.go_to_home_state()
    raw_input()

  def move_item(self):
    rospy.Subscriber("object_detection/item_position", geometry_msgs.msg.Pose, self.move_item_cb_p, queue_size=1)
    rospy.spin()
    


def main():
  try:
    print "============ Press Enter to set up the moveit_commander ..."
    #raw_input()
    _move_group_interface = MoveGroupInterface()

    print "============ Press Enter to execute a movement using home state ..."
    #raw_input()
    _move_group_interface.go_to_p_state()

    print "============ Press Enter to execute a movement using pose goals ..."
    _move_group_interface.move_item() 

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
