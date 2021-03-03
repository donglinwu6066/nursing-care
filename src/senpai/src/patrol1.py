#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
import time
import threading


class Patrol(object):
  def __init__(self,loc_goal):
    self.loc_goal = loc_goal

    # Starts a new node
    self.velocity_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    

    self.cmd = MoveBaseActionGoal()
    #rate = rospy.Rate(0.0636) # 0.1hz

    #Receiveing the user's input
    print("Let's move your marslite lol")
    t = threading.Thread(target = self.__motions)
    t.start()

  def __motions(self):
    if(self.loc_goal == "outside_button"):
        rospy.sleep(1.)
        #raw_input("Press Enter to continue...")
        print("outside button")
        self.cmd.goal.target_pose.header.frame_id = 'map'
        self.cmd.goal.target_pose.pose.position.x = 1.140
        self.cmd.goal.target_pose.pose.position.y = 0.095
        self.cmd.goal.target_pose.pose.orientation.z = -0.004
        self.cmd.goal.target_pose.pose.orientation.w = 1.000
        self.velocity_publisher.publish(self.cmd)

    elif(self.loc_goal == "outside_elevator"):
        rospy.sleep(1.)
        #raw_input("Press Enter to continue...")
        print("outside door")
        self.cmd.goal.target_pose.header.frame_id = 'map'
        self.cmd.goal.target_pose.pose.position.x = 0.997
        self.cmd.goal.target_pose.pose.position.y = 0.205
        self.cmd.goal.target_pose.pose.orientation.z = -0.379
        self.cmd.goal.target_pose.pose.orientation.w = 0.925
        self.velocity_publisher.publish(self.cmd)

    elif(self.loc_goal == "middle_point"):
        rospy.sleep(1.)
        #raw_input("Press Enter to continue...")
        print("middle point")
        self.cmd.goal.target_pose.header.frame_id = 'map'
        self.cmd.goal.target_pose.pose.position.x = 1.012
        self.cmd.goal.target_pose.pose.position.y = 0.120
        self.cmd.goal.target_pose.pose.orientation.z = -0.795
        self.cmd.goal.target_pose.pose.orientation.w = 0.607
        self.velocity_publisher.publish(self.cmd)


    

