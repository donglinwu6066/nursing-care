#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Patrol_plat(object):

    def __init__(self):
        
        #Starts a new node
        rospy.init_node('plat_move', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        
        self.PI = 3.1415926535897
        self.choice()

    def choice(self):
        while not rospy.is_shutdown():
            rot = input("Rotate or not?")
            if(rot):
                self.rotate()
                self.x = raw_input("Press Enter to continue...")
            else: 
                self.move()
                self.x = raw_input("Press Enter to continue...")    

    def rotate(self):
        # Receiveing the user's input
        print("Let's rotate your robot")
        speed = 15 #input("Input your speed (degrees/sec):")
        angle = input("Type your distance (degrees):")
        clockwise = input("Clockwise?: ") #True or false

        #Converting from angles to radians
        angular_speed = speed*2*self.PI/360
        relative_angle = angle*2*self.PI/360

        #We wont use linear components
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            self.vel_msg.angular.z = -abs(angular_speed)
        else:
            self.vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        #Forcing our robot to stop
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        # rospy.spin()

    def move(self):
        # Starts a new node
        # rospy.sleep(2.)
        # rospy.init_node('plat_move', anonymous=True)
        # velocity_publisher = rospy.Publisher('/mob_plat/cmd_vel', Twist, queue_size=10)
        # vel_msg = Twist()

        #Receiveing the user's input
        print("Let's move your marslite lol")
        speed = 0.1 # input("Input your speed:")
        distance = input("Type your distance:")
        isForward = input("Foward?: ")#True or False

        #Checking if the movement is forward or backwards
        if(isForward):
            self.vel_msg.linear.x = abs(speed)
        else:
            self.vel_msg.linear.x = -abs(speed)
        #Since we are moving just in x-axis
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        self.vel_msg.linear.x = 0
        #Force the robot to stop
        self.velocity_publisher.publish(self.vel_msg)
        # rospy.spin()

if __name__ == '__main__':
    # try:
        # #Testing our function
        # while not rospy.is_shutdown():
        #     rot = input("Rotate or not?")
        #     if(rot):
        #         rotate()
        #         raw_input("Press Enter to continue...")
        #     else: 
        #         move()
        #         raw_input("Press Enter to continue...")
    # except rospy.ROSInterruptException: pass    
    try:
        move_plat = Patrol_plat()
    except rospy.ROSException as e:
        rospy.logerr("Time out for camera info!")
        exit(-1)    