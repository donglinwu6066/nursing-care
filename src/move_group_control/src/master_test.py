#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time
import threading
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
###### motions #########
from press_button import MoveGroupInterface
from opendoor import Detection
from opendoor import Turn_right
from opendoor import Turn_left
from opendoor import Go_back_few
from opendoor import Go_forward_few
from patrol1 import Patrol

linear_x = 0
angular_z = 0
def sub_vel():
    scaner = rospy.Subscriber("/mob_plat/cmd_vel",Twist,process_detection)
    rospy.spin()
def process_detection(twist):
    global linear_x, angular_z    
    linear_x = twist.linear.x
    angular_z = twist.angular.z
def wait():
    global linear_x, angular_z
    start_time = time.time()
    while(True):
        rospy.sleep(0.1)
        if(abs(linear_x)<0.001 and abs(angular_z)<0.001 and (time.time()-start_time>5)):
            counter+=1
        else:
            counter = 0
        if(counter>=5):
            break
    print(linear_x, angular_z) 
    print("****wait done*********")

if __name__ =='__main__':
    rospy.init_node('move_group_interface', anonymous=True)
    
    ## detect for zero velocity	
    t = threading.Thread(target = sub_vel)
    t.start()
	

    print "============ arm to left pose ..."
    node = MoveGroupInterface("None")
    node.middle_pose()
    node = Detection()
    #print "============ go to front of button ..."
    #node = Patrol("outside_button")
    #wait()




    



