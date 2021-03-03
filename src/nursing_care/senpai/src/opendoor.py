#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time
import threading

class Detection(object):

    def __init__(self,distance,max_distance,stop_distance):

        #distance: if detacted remainding () distance than stop walking 
        #max_distance: if distance more than max_distance meter, terminate this motion
        #stop_distance: if distance more than stop_distance, stop going forward
        self.ranges = []
        self.detecrange = []
        self.modifyrange = []
        self.rangeAvg = 0  
        self.twist = None  
        self.scaner = None  
        print("fdsaf")
        rospy.on_shutdown(self.__shutdown_cb)
        self.pub = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size = 1)
        t = threading.Thread(target = self.__sub_laser_scan)
        t.start()

        total_time = 0
        speed = 0.2
        start_time = time.time()

        while(True):
            try:
                self.detecrange = self.ranges[:10]+self.ranges[-10:] 
                for i in self.detecrange:
                    if i>0.1:
                        self.modifyrange.append(i)  
                self.rangeAvg = sum(self.modifyrange)/len(self.modifyrange)
                print(self.rangeAvg)
                if self.rangeAvg>distance:  
                    twist = Twist()
                    twist.linear.x = speed
                    self.pub.publish(twist)
                    total_time=time.time()-start_time
                    if(total_time>stop_distance/speed):
                        break;
                else:
                    twist = Twist()
                    twist.linear.x = 0.0
                    self.pub.publish(twist)
                    #self.scaner.unregister()
                    if(total_time>(max_distance/speed)): 
                        break 
                    start_time = time.time()
            except ZeroDivisionError:
                pass

    def __shutdown_cb(self):
        rospy.loginfo("Node shutdown")	
    def __process_detection(self,scandata):
        self.modifyrange = []
        self.ranges = scandata.ranges
    def __sub_laser_scan(self):
        self.scaner = rospy.Subscriber("/scan",LaserScan,self.__process_detection)
        rospy.spin()

class Turn_right(object):
    def __init__(self,angle):
        pub = rospy.Publisher("/mob_plat/cmd_vel", Twist, queue_size=1)
        twist= Twist()
        speed = 0.3
         #turn right
        total_time = (3.14159*(angle/180.0))/speed
        print(angle)
        start_time = time.time()
        twist.linear.x=0.0
        twist.angular.z=-speed
        while(time.time()-start_time < total_time):
            pub.publish(twist) 
        #stop
        total_time = 0.1 #seconds
        start_time = time.time()
        twist.linear.x=0.0
        twist.angular.z= 0.0
        while(time.time()-start_time < total_time):
            pub.publish(twist)
class Turn_left(object):
    def __init__(self,angle):
        pub = rospy.Publisher("/mob_plat/cmd_vel", Twist, queue_size=1)
        twist= Twist()
        speed = 0.3
         #turn right
        total_time = (3.14159*(angle/180.0))/speed
        print(angle)
        start_time = time.time()
        twist.linear.x=0.0
        twist.angular.z=speed
        while(time.time()-start_time < total_time):
            pub.publish(twist) 
        #stop
        total_time = 0.1 #seconds
        start_time = time.time()
        twist.linear.x=0.0
        twist.angular.z= 0.0
        while(time.time()-start_time < total_time):
            pub.publish(twist)
class Go_back_few(object):
    def __init__(self,distance):
        pub = rospy.Publisher("/mob_plat/cmd_vel", Twist, queue_size=1)
        twist= Twist()
   
         #turn right
        total_time = distance*10 #seconds
        start_time = time.time()
        twist.linear.x-=0.1
        twist.angular.z=0.0
        while(time.time()-start_time < total_time):
            pub.publish(twist) 
        #stop
        total_time = 0.1 #seconds
        start_time = time.time()
        twist.linear.x=0.0
        twist.angular.z= 0.0
        while(time.time()-start_time < total_time):
            pub.publish(twist)
class Go_forward_few(object):
    def __init__(self,distance):
        pub = rospy.Publisher("/mob_plat/cmd_vel", Twist, queue_size=1)
        twist= Twist()
   
         #turn right
        total_time = distance*10 #seconds
        start_time = time.time()
        twist.linear.x=0.1
        twist.angular.z=0.0
        while(time.time()-start_time < total_time):
            pub.publish(twist) 
        #stop
        total_time = 0.1 #seconds
        start_time = time.time()
        twist.linear.x=0.0
        twist.angular.z= 0.0
        while(time.time()-start_time < total_time):
            pub.publish(twist)


