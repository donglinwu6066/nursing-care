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
from move_base_msgs.msg import MoveBaseActionGoal
import os
import time
import threading


class MoveGroupInterface(object):
  """MoveGroupInterface"""
  def __init__(self,button):
    super(MoveGroupInterface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)

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
    #print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    #print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    print "*********arm************"
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

    #pub Coke
    pub2 = rospy.Publisher('/object_detection/object', String, queue_size=10)
    rate = rospy.Rate(10)
    rospy.loginfo(button)
    rospy.sleep(1)
    rospy.loginfo(button)
    pub2.publish(button)



  def right_pose(self):
    move_group = self.move_group
    pub = self.pub
    pub.publish(True)
    joint_goal = [-0.5336353182792664, -0.07512450218200684, 1.9735796451568604, -1.7973496913909912, 0.5216982960700989, 0.09126132726669312]
    move_group.go(joint_goal,wait=True)
    move_group.stop()
  
  def left_pose(self):
    move_group = self.move_group
    pub = self.pub
    pub.publish(True)
    joint_goal = [1.6443593502044678, 1.69889235496521, -1.8340119123458862, 0.1071632131934166, 1.4464706182479858, -0.0030737188644707203]
    move_group.go(joint_goal,wait=True)
    move_group.stop()

  def middle_pose(self):
    move_group = self.move_group
    pub = self.pub
    pub.publish(True)
    joint_goal = [0.0003732585464604199, -0.7324762940406799, 0.9247729182243347, -0.17437778413295746, 1.5701175928115845, -0.00033936958061531186]
    move_group.go(joint_goal,wait=True)
    move_group.stop()
    
  def home_pose(self):
    move_group = self.move_group
    pub = self.pub
    pub.publish(True)
    joint_goal = [-1.5707651376724243, -0.7329863905906677, 1.9722110033035278, -0.17461049556732178, 1.5708060264587402, 0.0]
    move_group.go(joint_goal,wait=True)
    move_group.stop()

  def __playsound(self):
    for i in range(4):
        os.system("play ~/mars_lite_ws/src/josh/move_group_control/src/mp3/Disinfecting_zh.mp3")
        #os.system("play ~/mars_lite_ws/src/josh/move_group_control/src/mp3/Disinfecting.mp3")
  def press_on_right_hand_side(self):
    #pub = self.pub
    #pub.publish(True)
    t = threading.Thread(target = self.__playsound)
    t.start()
    item_pose = geometry_msgs.msg.Pose()
    item_pose = self.tf_item_listener()
    item_pose.orientation.x = 0.741375997577
    item_pose.orientation.y = -0.00206070522393
    item_pose.orientation.z = -0.000807733047346
    item_pose.orientation.w = 0.671086232371

    item_pose.position.x -= 0.0
    item_pose.position.y += 0.185
    item_pose.position.z += 0.075
    print item_pose
    self.go_to_pose_goal(item_pose)
    rospy.sleep(1)
    #####disinfect around button########
    item_movement = [0.05,0,0]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [0,0,-0.14]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [-0.1,0,0]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [0,0,0.22]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [0.1,0,0]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [0,0,-0.08]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [-0.05,0,0]
    self.go_to_cartesian_goal(item_movement)
    #press button
    item_movement = [0,-0.05,0]
    self.go_to_cartesian_goal(item_movement)

    #self.go_to_home_state()

  def press_on_left_hand_side(self):

    t = threading.Thread(target = self.__playsound)
    t.start()
    item_pose = geometry_msgs.msg.Pose()
    item_pose = self.tf_item_listener()
    item_pose.orientation.x = 0.0169632364197
    item_pose.orientation.y = 0.715924512541
    item_pose.orientation.z = 0.697872881007
    item_pose.orientation.w = 0.0117380964518


    item_pose.position.x -= 0.01
    item_pose.position.y -= 0.194
    item_pose.position.z += 0.075
    print item_pose
    self.go_to_pose_goal(item_pose)
    rospy.sleep(1)

    #####disinfect around button########
    item_movement = [0.05,0,0]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [0,0,-0.14]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [-0.1,0,0]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [0,0,0.22]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [0.1,0,0]
    self.go_to_cartesian_goal(item_movement)
    item_movement = [0,0,-0.08]
    self.go_to_cartesian_goal(item_movement)

    self.go_to_pose_goal(item_pose)
    #####press button#########
    item_movement = [0,0.05,0]
    self.go_to_cartesian_goal(item_movement)



    
    #self.go_to_p_state()


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


    



if __name__ == '__main__':
    rospy.init_node('press_button', anonymous=True)
    
    print("============ Press button ...")
    _move_group_interface = MoveGroupInterface("down")
    #_move_group_interface.middle_pose()
    _move_group_interface.left_pose()
    #rospy.sleep(5)
    _move_group_interface.press_on_left_hand_side() 
    _move_group_interface.left_pose()


