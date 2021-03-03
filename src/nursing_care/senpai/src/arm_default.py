#!/usr/bin/env python2.7
from math import pi, sqrt
import sys
import copy
import random
import copy
import rospy
import tf
import moveit_commander

import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from moveit_msgs.msg import DisplayTrajectory, JointConstraint, Constraints
from std_msgs.msg import String, Float64, Bool, Header
from std_srvs.srv import Trigger, TriggerResponse, Empty
from std_msgs.msg import Bool

import time
import sys, select, termios, tty
#from pose_estimate_and_pick.srv import *
#from object_detection.srv import *
#from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection




armControl={
        'q':(0.01,0,0),
        'a':(-0.01,0,0),
        'w':(0,0.01,0),
        's':(0,-0.01,0),
        'e':(0,0,0.01),
        'd':(0,0,-0.01),
	'x':(0,0,0),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('final_round_node', anonymous=False)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("tm_arm")
    #group = moveit_commander.MoveGroupCommander("manipulator")
    pub = rospy.Publisher('gripper/cmd_gripper', Bool, queue_size=10)

    try:
        print "============ your key..."
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0.020243054255843163
        joint_goal[1] = -0.7387097477912903
        joint_goal[2] = 2.326383590698242
        joint_goal[3] = -1.5347648859024048
        joint_goal[4] = 1.5300719738006592
        joint_goal[5] = 0.0065352884121239185
        group.go(joint_goal, wait=True)
        while(1):
            key = getKey()
            if key in armControl.keys():
                print "============ position"
                pose_target = group.get_current_pose().pose
                pose_target.position.x +=  armControl[key][0]
                pose_target.position.y +=  armControl[key][1]
                pose_target.position.z +=  armControl[key][2]
                group.set_pose_target(pose_target)
                group.go(wait=True)
            if(key=='h'):
                joint_goal = group.get_current_joint_values()
                joint_goal[0] = 0
                joint_goal[1] = 0
                joint_goal[2] = pi/2
                joint_goal[3] = -pi/2
                joint_goal[4] = pi/2
                joint_goal[5] = 0
                group.go(joint_goal, wait=True) 
            if(key=='g'):
                joint_goal = group.get_current_joint_values()
                joint_goal[0] = -1.5638910532
                joint_goal[1] = 0.0138541124761
                joint_goal[2] = 2.26129961014
                joint_goal[3] = -2.22698187828
                joint_goal[4] = 1.54945480824
                joint_goal[5] = 0.00317068165168
                group.go(joint_goal, wait=True)  
            if(key=='o'):
                joint_goal = group.get_current_joint_values()
                joint_goal[0] = -1.5707963705062866
                joint_goal[1] = -0.7330362200737
                joint_goal[2] = 1.9722295999526978
                joint_goal[3] = -0.1745329201221466
                joint_goal[4] = 1.5707963705062866
                joint_goal[5] = 0.0
                group.go(joint_goal, wait=True)    
            if(key=='r'):
                pub.publish(True)
            if(key=='f'):
                pub.publish(False)
            if(key == 'z'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[0] += pi/24
                group.go(joint_goal, wait=True) 
            if(key == 'x'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[0] -= pi/24
                group.go(joint_goal, wait=True) 
            if(key == 'c'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[1] += pi/24
                group.go(joint_goal, wait=True) 
            if(key == 'v'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[1] -= pi/24
                group.go(joint_goal, wait=True) 
            if(key == 'b'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[2] += pi/24
                group.go(joint_goal, wait=True) 
            if(key == 'n'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[2] -= pi/24
                group.go(joint_goal, wait=True) 
            if(key == 'm'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[3] += pi/24
                group.go(joint_goal, wait=True) 
            if(key == ','):
                joint_goal = group.get_current_joint_values()	
                joint_goal[3] -= pi/24
                group.go(joint_goal, wait=True) 
            if(key == '.'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[4] += pi/24
                group.go(joint_goal, wait=True) 
            if(key == '/'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[4] -= pi/24
                group.go(joint_goal, wait=True)
            if(key == 'j'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[5] += pi/24
                group.go(joint_goal, wait=True) 
            if(key == 'k'):
                joint_goal = group.get_current_joint_values()	
                joint_goal[5] -= pi/24
                group.go(joint_goal, wait=True)  
            if (key == '\x03'):
                break


    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print "============ Bye"
    #rospy.spin()
