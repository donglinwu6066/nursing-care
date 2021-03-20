#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def Coke():
    pub = rospy.Publisher('/object_detection/object', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = raw_input("next: ")
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
if __name__ == '__main__':
    try:
        Coke()
    except rospy.ROSInterruptException:
        pass
