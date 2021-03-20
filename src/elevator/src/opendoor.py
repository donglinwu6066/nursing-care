#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class Detection(object):

    def __init__(self):
        self.ranges = None
        self.detecrange = None
        self.modifyrange = []
        self.rangeAvg = 0  
        self.twist = None    
        print("fdsaf")
        rospy.on_shutdown(self.__shutdown_cb)
        rospy.Subscriber("/scan",LaserScan,self.__process_detection)
        self.pub = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size = 1)
        rospy.spin()

    def __shutdown_cb(self):
        rospy.loginfo("Node shutdown")	

    def __process_detection(self,scandata):
        self.modifyrange = []
        self.ranges = scandata.ranges
        #print(self.ranges)
        self.detecrange = self.ranges[:10]+self.ranges[-10:] 
        for i in self.detecrange:
            if i>0.1:
                self.modifyrange.append(i)  
        #print(self.modifyrange)    
        self.rangeAvg = sum(self.modifyrange)/len(self.modifyrange)
        print(self.rangeAvg)
        if self.rangeAvg>0.5:
            twist = Twist()
            twist.linear.x = 0.2
            self.pub.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = 0.0
            self.pub.publish(twist)
            
if __name__ =='__main__':
    rospy.init_node('opendoor',anonymous=True)
    node = Detection()
