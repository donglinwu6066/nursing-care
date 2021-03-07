#!/usr/bin/env python2.7

import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('gripper/cmd_gripper', Bool, queue_size=10)
    rospy.init_node('gripper_action', anonymous=True)
    rate = rospy.Rate(0.25)
    while not rospy.is_shutdown():
        pub.publish(False)
	rate.sleep()
	pub.publish(True)
	rate.sleep()
	print("one turn")


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('gripper/cmd_gripper', Bool, queue_size=10)
        rospy.init_node('gripper_action', anonymous=True)
        rate = rospy.Rate(0.25)
        #while not rospy.is_shutdown():
        #     pub.publish(False) #release
	#     rate.sleep() 
        for i in range(1):
	    pub.publish(True)
	    rate.sleep()
	#    pub.publish(True) #hold
	#    rate.sleep()
	print("open")
    except rospy.ROSInterruptException:
        pass
